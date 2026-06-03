"""
Model Predictive Controller for the pusher-slider system.

Based on Hogan & Rodriguez 2016 (WAFR): "Feedback Control of the Pusher-Slider
System: A Story of Hybrid and Underactuated Contact Dynamics."

Uses the Family of Modes (FOM) approach: solve 3 convex QPs (one per representative
mode schedule) and pick the one with minimum cost.

State: x = [slider_x, slider_y, slider_theta, p_tangential]^T
  (slider_x, slider_y, slider_theta) is the slider pose in world frame.
  p_tangential is the tangential position of the pusher contact in body frame.

The equations use a "contact-aligned" coordinate system at the contact point:
  - n-axis: inward face normal (the direction pushing *into* the slider)
  - t-axis: tangent along the face (90 deg CCW from n)
This is obtained by rotating the original body-frame contact point (px, py)
so that the face normal aligns with the paper's primary push axis (+x).

Control: u = [vn, vt]^T
  vn: pusher velocity along the face inward normal (>= 0 for contact)
  vt: pusher velocity along the face tangent
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import minimize as scipy_minimize


def _face_rotation(face: str) -> np.ndarray:
    """Return 2x2 rotation R that maps body frame to contact-aligned frame.

    In the contact-aligned frame, +x is the inward normal of the face.
    Applying R to a body-frame vector gives the contact-aligned vector.
    Applying R to the body-frame contact point (px, py) gives the
    rotated contact point (px', py') used in the paper's equations.

    Args:
        face: which slider face the pusher contacts: '-y', '+y', '-x', '+x'.
    """
    if face == "-y":
        # inward normal = +y_body. Rotate -90 deg so +y_body -> +x_aligned.
        # R(-90) = [[0, 1], [-1, 0]]
        return np.array([[0.0, 1.0], [-1.0, 0.0]])
    elif face == "+y":
        # inward normal = -y_body. Rotate +90 deg so -y_body -> +x_aligned.
        return np.array([[0.0, -1.0], [1.0, 0.0]])
    elif face == "-x":
        # inward normal = +x_body. Already aligned.
        return np.eye(2)
    elif face == "+x":
        # inward normal = -x_body. Rotate 180 deg.
        return np.array([[-1.0, 0.0], [0.0, -1.0]])
    else:
        raise ValueError(f"Unknown face '{face}', expected '-y', '+y', '-x', '+x'")


class PusherSliderMPC:
    """MPC controller for quasi-static pusher-slider manipulation.

    Args:
        slider_dims: (a, b) slider dimensions in meters.
        mass: slider mass in kg (unused in quasi-static, kept for interface).
        mu_pusher: pusher-slider friction coefficient.
        mu_ground: slider-ground friction coefficient (unused directly).
        dt: discretization timestep.
        horizon_N: prediction horizon length.
        Q_weights: (4,) diagonal weights for state cost.
        R_weights: (2,) diagonal weights for control cost.
        Q_terminal_scale: multiplier for terminal cost relative to Q_weights.
        v_max: maximum pusher velocity magnitude per component.
        contact_face: which slider face the pusher contacts ('-y', '+y', '-x', '+x').
    """

    def __init__(
        self,
        slider_dims: tuple[float, float] = (0.08, 0.06),
        mass: float = 1.05,
        mu_pusher: float = 0.3,
        mu_ground: float = 0.35,
        dt: float = 0.05,
        horizon_N: int = 15,
        Q_weights: np.ndarray | None = None,
        R_weights: np.ndarray | None = None,
        Q_terminal_scale: float = 10.0,
        v_max: float = 0.1,
        contact_face: str = "-y",
    ) -> None:
        self.a, self.b = slider_dims
        self.mass = mass
        self.mu = mu_pusher
        self.mu_ground = mu_ground
        self.dt = dt
        self.N = horizon_N
        self.v_max = v_max
        self.contact_face = contact_face

        self.c = self._limit_surface_c()

        # R_face rotates body-frame coords to contact-aligned coords.
        # R_face^T (= R_face_inv) rotates back.
        self.R_face = _face_rotation(contact_face)
        self.R_face_inv = self.R_face.T  # orthogonal, so inv = transpose

        if Q_weights is None:
            Q_weights = np.array([10.0, 10.0, 5.0, 0.1])
        if R_weights is None:
            R_weights = np.array([0.01, 0.01])

        self.Q = np.diag(Q_weights)
        self.R = np.diag(R_weights)
        self.Q_N = Q_terminal_scale * self.Q

    def _limit_surface_c(self) -> float:
        """Compute limit surface parameter c (radius of gyration for uniform pressure)."""
        return np.sqrt((self.a**2 + self.b**2) / 12.0)

    def _motion_cone(self, px_a: float, py_a: float) -> tuple[float, float]:
        """Compute motion cone boundaries in the contact-aligned frame.

        Args:
            px_a: contact x in aligned frame (along face inward normal).
            py_a: contact y in aligned frame (along face tangent).

        Returns:
            (gamma_t, gamma_b): motion cone boundary slopes for vt/vn.
        """
        mu = self.mu
        c2 = self.c**2
        px, py = px_a, py_a
        gamma_t = (mu * c2 - px * py + mu * py**2) / (c2 + px**2 + mu * px * py)
        gamma_b = (-mu * c2 - px * py - mu * py**2) / (c2 + px**2 - mu * px * py)
        return gamma_t, gamma_b

    def _rotation_matrix(self, theta: float) -> np.ndarray:
        """2x2 rotation from body to world frame."""
        ct, st = np.cos(theta), np.sin(theta)
        return np.array([[ct, -st], [st, ct]])

    def _aligned_contact(self, px_body: float, py_body: float) -> tuple[float, float]:
        """Transform body-frame contact point to contact-aligned frame."""
        p_a = self.R_face @ np.array([px_body, py_body])
        return float(p_a[0]), float(p_a[1])

    def _compute_B(
        self, theta: float, px_body: float, py_body: float, mode: int
    ) -> np.ndarray:
        """Compute B matrix (4x2) for x_dot = B @ [vn, vt] (contact-aligned frame).

        Internally:
          1. Rotate contact point to aligned frame: (px_a, py_a) = R_face @ (px, py)
          2. Apply the paper's Q_ls, P in the aligned frame to get slider velocity
             in the aligned frame: v_slider_aligned = Q_ls @ P @ [vn, vt]
          3. Rotate slider velocity back to body frame, then to world frame.
          4. Compute angular velocity and tangential contact shift.

        Args:
            theta: current slider angle.
            px_body: pusher contact x in slider body frame.
            py_body: pusher contact y in slider body frame.
            mode: 1=stick, 2=slide_up, 3=slide_down.

        Returns:
            B (4, 2) mapping [vn, vt] to [sx_dot, sy_dot, theta_dot, pt_dot].
        """
        px_a, py_a = self._aligned_contact(px_body, py_body)
        c2 = self.c**2
        denom = c2 + px_a**2 + py_a**2

        # Q_ls in aligned frame
        Q_ls = (
            np.array(
                [
                    [c2 + px_a**2, px_a * py_a],
                    [px_a * py_a, c2 + py_a**2],
                ]
            )
            / denom
        )

        gamma_t, gamma_b = self._motion_cone(px_a, py_a)

        if mode == 1:  # sticking
            P = np.eye(2)
            b_vec = np.array([(-py_a) / denom, px_a / denom])
            c_vec = np.array([0.0, 0.0])
        elif mode == 2:  # sliding up
            P = np.array([[1.0, 0.0], [gamma_t, 0.0]])
            b_vec = np.array([(-py_a + gamma_t * px_a) / denom, 0.0])
            c_vec = np.array([-gamma_t, 0.0])
        elif mode == 3:  # sliding down
            P = np.array([[1.0, 0.0], [gamma_b, 0.0]])
            b_vec = np.array([(-py_a + gamma_b * px_a) / denom, 0.0])
            c_vec = np.array([-gamma_b, 0.0])
        else:
            raise ValueError(f"Unknown mode {mode}")

        # Slider velocity in aligned frame
        v_slider_aligned = Q_ls @ P  # (2x2), operates on [vn, vt]

        # Rotate to body frame, then to world frame
        # body = R_face^T @ aligned, world = C(theta) @ body
        C = self._rotation_matrix(theta)
        C_total = C @ self.R_face_inv  # aligned -> body -> world

        B = np.zeros((4, 2))
        B[0:2, :] = C_total @ v_slider_aligned
        B[2, :] = b_vec  # angular velocity (scalar, frame-independent)
        B[3, :] = c_vec  # tangential shift (in aligned frame)
        return B

    def _continuous_dynamics(
        self, state: np.ndarray, u: np.ndarray, px_a: float, mode: int
    ) -> np.ndarray:
        """Evaluate x_dot = f(x, u) where u = [vn, vt] in contact-aligned frame.

        Args:
            state: [sx, sy, stheta, py_a] where py_a is tangential contact in aligned frame.
            u: [vn, vt] in contact-aligned frame.
            px_a: normal-direction contact position in aligned frame (fixed constant).
            mode: 1=stick, 2=slide_up, 3=slide_down.
        """
        theta = state[2]
        py_a = state[3]
        # Reconstruct body-frame contact from aligned-frame contact
        p_body = self.R_face_inv @ np.array([px_a, py_a])

        B = self._compute_B(theta, p_body[0], p_body[1], mode)
        return B @ u

    def _select_mode(
        self, vn: float, vt: float, px_a: float, py_a: float, tol: float = 1e-6
    ) -> int:
        """Determine contact mode from aligned-frame velocity and motion cone.

        The motion cone constrains vt/vn in the aligned frame. Points at the
        cone boundary (within tol) are classified as sticking.

        Returns:
            mode: 1=stick, 2=slide_up, 3=slide_down.
        """
        gamma_t, gamma_b = self._motion_cone(px_a, py_a)
        if abs(vn) < 1e-12:
            return 1
        ratio = vt / vn
        if ratio > gamma_t + tol:
            return 2
        elif ratio < gamma_b - tol:
            return 3
        else:
            return 1

    def _build_prediction_matrices(
        self, x0: np.ndarray, px_a: float, schedule: list[int]
    ) -> tuple[np.ndarray, np.ndarray]:
        """Build stacked prediction matrices for the linear dynamics.

        Since A=0, x_{k+1} = x_k + dt*B_k*u_k.
        Unrolling: x_{k+1} = x_0 + dt * sum_{j=0}^{k} B_j * u_j.

        B is frozen at the current state for all prediction steps.

        Args:
            x0: state [sx, sy, stheta, py_a].
            px_a: normal-direction contact position in aligned frame (fixed).
            schedule: mode sequence.

        Returns:
            (S, x_free) where:
              S (4*N, 2*N): maps stacked u to stacked x_{1:N}
              x_free (4*N,): free response (u=0) stacked states
        """
        N = self.N
        dt = self.dt
        theta = x0[2]
        py_a = x0[3]

        # Recover body-frame contact from aligned frame
        p_body = self.R_face_inv @ np.array([px_a, py_a])

        B_list = []
        for n in range(N):
            B_n = self._compute_B(theta, p_body[0], p_body[1], schedule[n])
            B_list.append(B_n)

        S = np.zeros((4 * N, 2 * N))
        x_free = np.zeros(4 * N)

        for k in range(N):
            x_free[4 * k : 4 * (k + 1)] = x0
            for j in range(k + 1):
                S[4 * k : 4 * (k + 1), 2 * j : 2 * (j + 1)] = dt * B_list[j]

        return S, x_free

    def _solve_fom_qp(
        self, x0: np.ndarray, target: np.ndarray, px_a: float
    ) -> tuple[np.ndarray, float]:
        """Solve the Family of Modes (FOM) QPs and return the best first control.

        Tries 3 mode schedules:
          M1: slide_up at n=0, stick for n>0
          M2: slide_down at n=0, stick for n>0
          M3: stick for all n

        Args:
            x0: state [sx, sy, stheta, py_a].
            target: target state [sx, sy, stheta, py_a].
            px_a: normal-direction contact position in aligned frame (fixed).
        """
        mode_schedules = {
            "M1": [2] + [1] * (self.N - 1),
            "M2": [3] + [1] * (self.N - 1),
            "M3": [1] * self.N,
        }

        best_u = np.array([0.01, 0.0])
        best_cost = np.inf

        for _name, schedule in mode_schedules.items():
            u_opt, cost = self._solve_single_schedule(x0, target, px_a, schedule)
            if cost < best_cost:
                best_cost = cost
                best_u = u_opt

        return best_u, best_cost

    def _solve_single_schedule(
        self,
        x0: np.ndarray,
        target: np.ndarray,
        px_a: float,
        schedule: list[int],
    ) -> tuple[np.ndarray, float]:
        """Solve a single QP for a fixed mode schedule.

        Decision variables: z = [vn_0, vt_0, ..., vn_{N-1}, vt_{N-1}] (2N,).

        Args:
            x0: state [sx, sy, stheta, py_a].
            target: target state.
            px_a: normal-direction contact position in aligned frame (fixed).
            schedule: mode sequence.
        """
        N = self.N
        n_vars = 2 * N

        S, x_free = self._build_prediction_matrices(x0, px_a, schedule)
        target_stacked = np.tile(target, N)

        # Block-diagonal cost
        Q_blk = np.zeros((4 * N, 4 * N))
        for k in range(N - 1):
            Q_blk[4 * k : 4 * (k + 1), 4 * k : 4 * (k + 1)] = self.Q
        Q_blk[4 * (N - 1) : 4 * N, 4 * (N - 1) : 4 * N] = self.Q_N

        R_blk = np.zeros((n_vars, n_vars))
        for k in range(N):
            R_blk[2 * k : 2 * (k + 1), 2 * k : 2 * (k + 1)] = self.R

        d = x_free - target_stacked
        H = S.T @ Q_blk @ S + R_blk
        f_vec = S.T @ Q_blk @ d

        H = 0.5 * (H + H.T) + 1e-8 * np.eye(n_vars)

        # Motion cone constraints in aligned frame
        py_a = x0[3]
        gamma_t, gamma_b = self._motion_cone(px_a, py_a)

        A_ineq_rows = []
        b_ineq_rows = []

        for n in range(N):
            mode = schedule[n]
            if mode == 1:
                # Sticking: gamma_b * vn <= vt <= gamma_t * vn
                # Upper: vt - gamma_t * vn <= 0
                r_up = np.zeros(n_vars)
                r_up[2 * n] = -gamma_t
                r_up[2 * n + 1] = 1.0
                A_ineq_rows.append(r_up)
                b_ineq_rows.append(0.0)

                # Lower: gamma_b * vn - vt <= 0
                r_lo = np.zeros(n_vars)
                r_lo[2 * n] = gamma_b
                r_lo[2 * n + 1] = -1.0
                A_ineq_rows.append(r_lo)
                b_ineq_rows.append(0.0)

            elif mode == 2:
                # Sliding up: vt >= gamma_t * vn  =>  gamma_t * vn - vt <= 0
                r = np.zeros(n_vars)
                r[2 * n] = gamma_t
                r[2 * n + 1] = -1.0
                A_ineq_rows.append(r)
                b_ineq_rows.append(0.0)

            elif mode == 3:
                # Sliding down: vt <= gamma_b * vn  =>  vt - gamma_b * vn <= 0
                r = np.zeros(n_vars)
                r[2 * n] = -gamma_b
                r[2 * n + 1] = 1.0
                A_ineq_rows.append(r)
                b_ineq_rows.append(0.0)

        if A_ineq_rows:
            A_ineq = np.array(A_ineq_rows)
            b_ineq = np.array(b_ineq_rows)
        else:
            A_ineq = np.zeros((0, n_vars))
            b_ineq = np.zeros(0)

        # Bounds: vn >= 0 (push into slider), |vt| <= v_max
        bounds = []
        for n in range(N):
            bounds.append((0.0, self.v_max))
            bounds.append((-self.v_max, self.v_max))

        def objective(z: np.ndarray) -> float:
            return 0.5 * z @ H @ z + f_vec @ z

        def grad(z: np.ndarray) -> np.ndarray:
            return H @ z + f_vec

        constraints = []
        if len(b_ineq) > 0:
            constraints.append(
                {
                    "type": "ineq",
                    "fun": lambda u: b_ineq - A_ineq @ u,
                    "jac": lambda u: -A_ineq,
                }
            )

        # Initial guess: push forward at moderate speed
        u0 = np.zeros(n_vars)
        for n in range(N):
            u0[2 * n] = 0.05

        result = scipy_minimize(
            objective,
            u0,
            jac=grad,
            method="SLSQP",
            bounds=bounds,
            constraints=constraints,
            options={"maxiter": 100, "ftol": 1e-9, "disp": False},
        )

        u_opt_first = result.x[:2].copy()
        cost = objective(result.x)
        return u_opt_first, cost

    def compute_control(
        self,
        slider_pose: np.ndarray,
        pusher_pos_body: np.ndarray,
        target_pose: np.ndarray,
    ) -> tuple[float, float]:
        """Compute optimal pusher velocity in the contact-aligned frame.

        Args:
            slider_pose: (3,) array [x, y, theta] of slider in world frame.
            pusher_pos_body: (2,) array [px, py] of pusher in slider body frame.
            target_pose: (3,) array [x_target, y_target, theta_target].

        Returns:
            (vn, vt) pusher velocity in contact-aligned frame.
                vn: along face inward normal (>= 0 for contact maintenance).
                vt: along face tangent.
        """
        px_body, py_body = pusher_pos_body
        px_a, py_a = self._aligned_contact(px_body, py_body)

        x0 = np.array([slider_pose[0], slider_pose[1], slider_pose[2], py_a])
        target = np.array([target_pose[0], target_pose[1], target_pose[2], py_a])

        u_opt, _ = self._solve_fom_qp(x0, target, px_a)
        return float(u_opt[0]), float(u_opt[1])

    def contact_to_body(self, vn: float, vt: float) -> np.ndarray:
        """Convert contact-aligned velocity to body-frame velocity."""
        return self.R_face_inv @ np.array([vn, vt])

    def contact_to_world(self, vn: float, vt: float, theta: float) -> np.ndarray:
        """Convert contact-aligned velocity to world-frame velocity."""
        v_body = self.contact_to_body(vn, vt)
        C = self._rotation_matrix(theta)
        return C @ v_body

    def body_to_contact(self, vp_x: float, vp_y: float) -> np.ndarray:
        """Convert body-frame velocity to contact-aligned velocity."""
        return self.R_face @ np.array([vp_x, vp_y])


def generate_straight_trajectory(
    y_start: float = 0.2,
    y_end: float = 0.8,
    push_speed: float = 0.05,
    dt: float = 0.05,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a nominal straight-line push trajectory along y-axis.

    Returns:
        (times, poses) where poses is (T, 3) array of [x, y, theta].
    """
    distance = y_end - y_start
    total_time = distance / push_speed
    n_steps = int(total_time / dt)
    times = np.arange(n_steps + 1) * dt
    poses = np.zeros((n_steps + 1, 3))
    poses[:, 1] = y_start + push_speed * times
    return times, poses


def simulate_analytical(
    mpc: PusherSliderMPC,
    x0_body: np.ndarray,
    target: np.ndarray,
    px_body: float,
    n_steps: int,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Simulate the pusher-slider system using analytical dynamics + MPC control.

    Args:
        mpc: MPC controller.
        x0_body: initial state [sx, sy, stheta, py_body] (py in body frame).
        target: target pose [sx_target, sy_target, stheta_target].
        px_body: pusher contact x in body frame (fixed).
        n_steps: number of simulation steps.
        dt: simulation timestep.

    Returns:
        (states, controls) arrays of shape (n_steps+1, 4) and (n_steps, 2).
        states columns: [sx, sy, stheta, py_aligned].
        controls columns: [vn, vt] in contact-aligned frame.
    """
    # Convert initial body-frame contact to aligned frame
    px_a, py_a_init = mpc._aligned_contact(px_body, x0_body[3])

    states = np.zeros((n_steps + 1, 4))
    controls = np.zeros((n_steps, 2))
    states[0] = np.array([x0_body[0], x0_body[1], x0_body[2], py_a_init])

    for k in range(n_steps):
        x = states[k]
        slider_pose = x[:3]
        py_a = x[3]

        # Reconstruct body-frame pusher position from aligned frame
        p_body = mpc.R_face_inv @ np.array([px_a, py_a])
        pusher_body = p_body

        vn, vt = mpc.compute_control(slider_pose, pusher_body, target)
        controls[k] = [vn, vt]

        # Determine mode in aligned frame
        mode = mpc._select_mode(vn, vt, px_a, py_a)

        # Forward integrate (pass px_a for aligned-frame dynamics)
        x_dot = mpc._continuous_dynamics(x, np.array([vn, vt]), px_a, mode)
        states[k + 1] = x + dt * x_dot

    return states, controls


if __name__ == "__main__":
    print("=" * 70)
    print("Pusher-Slider MPC Verification (Analytical Dynamics)")
    print("=" * 70)

    # --- Sanity check ---
    print("\n--- Sanity Check: Dynamics at nominal state ---")
    mpc_check = PusherSliderMPC(
        slider_dims=(0.08, 0.06), mu_pusher=0.3, contact_face="-y"
    )
    # For -y face: body contact (0, -0.03) -> aligned contact (-0.03, 0)
    px_a, py_a = mpc_check._aligned_contact(0.0, -0.03)
    print(f"Aligned contact: px_a={px_a:.4f}, py_a={py_a:.4f}")

    gt, gb = mpc_check._motion_cone(px_a, py_a)
    print(f"Motion cone: gamma_t={gt:.4f}, gamma_b={gb:.4f}")

    B = mpc_check._compute_B(theta=0.0, px_body=0.0, py_body=-0.03, mode=1)
    u_test = np.array([0.05, 0.0])  # vn=0.05 along face normal (+y for -y face)
    x_dot_test = B @ u_test
    print(f"B @ [vn=0.05, vt=0] = {x_dot_test}")
    print(f"  sy_dot = {x_dot_test[1]:.4f} (expect ~0.05)")
    assert abs(x_dot_test[1] - 0.05) < 0.01, "sy_dot should be ~0.05"
    print("  OK: normal push moves slider in +y direction")

    # vn/vt = inf for pure normal push -> inside cone? Let's check: vt/vn = 0
    print(f"  vt/vn = 0, in cone [{gb:.4f}, {gt:.4f}]? {gb <= 0 <= gt}")

    # --- MPC simulation ---
    slider_a, slider_b = 0.08, 0.06
    dt = 0.05
    horizon_N = 10
    v_max = 0.1

    Q_w = np.array([20.0, 20.0, 10.0, 0.1])
    R_w = np.array([0.1, 0.1])

    mpc = PusherSliderMPC(
        slider_dims=(slider_a, slider_b),
        mass=1.05,
        mu_pusher=0.3,
        mu_ground=0.35,
        dt=dt,
        horizon_N=horizon_N,
        Q_weights=Q_w,
        R_weights=R_w,
        Q_terminal_scale=10.0,
        v_max=v_max,
        contact_face="-y",
    )

    print(f"\nLimit surface c = {mpc.c:.4f} m")
    print(f"Horizon: N={horizon_N}, dt={dt} s")

    px_body = 0.0
    py_body_init = -slider_b / 2.0

    y_start, y_target = 0.2, 0.8
    target_pose = np.array([0.0, y_target, 0.0])

    x0_body = np.array([0.0, y_start, 0.0, py_body_init])

    push_distance = y_target - y_start
    sim_time = push_distance / 0.05 * 1.5
    n_steps = int(sim_time / dt)

    print(f"\nSimulating {n_steps} steps ({sim_time:.1f} s)")
    print(f"Initial: y={y_start}, target: y={y_target}")

    # --- Test 1: Nominal ---
    print("\n--- Test 1: Nominal initial condition ---")
    states, controls = simulate_analytical(
        mpc, x0_body, target_pose, px_body, n_steps, dt
    )

    final = states[-1]
    y_err = abs(final[1] - y_target)
    theta_err = abs(np.rad2deg(final[2]))
    x_drift = abs(final[0])

    print(
        f"\nFinal: x={final[0]:.4f}, y={final[1]:.4f}, "
        f"theta={np.rad2deg(final[2]):.2f} deg"
    )
    print(f"Errors: y={y_err:.4f}, theta={theta_err:.2f} deg, x_drift={x_drift:.4f}")
    print(f"First 3 controls: {controls[:3].round(4)}")

    y_ok = y_err < 0.05
    theta_ok = theta_err < 5.0
    vn_max = np.max(np.abs(controls[:, 0]))
    vt_max = np.max(np.abs(controls[:, 1]))
    ctrl_ok = vn_max <= v_max + 1e-6 and vt_max <= v_max + 1e-6

    print(
        f"\ny-convergence (<0.05 m):     {'PASS' if y_ok else 'FAIL'} (err={y_err:.4f})"
    )
    print(
        f"theta-convergence (<5 deg):  {'PASS' if theta_ok else 'FAIL'} "
        f"(err={theta_err:.2f})"
    )
    print(
        f"Control bounds:              {'PASS' if ctrl_ok else 'FAIL'} "
        f"(vn_max={vn_max:.4f}, vt_max={vt_max:.4f})"
    )
    x_drift_ok = x_drift < 0.05
    print(
        f"x-drift (<0.05 m):           {'PASS' if x_drift_ok else 'FAIL'} "
        f"(drift={x_drift:.4f})"
    )

    # --- Test 2: Perturbed ---
    print("\n--- Test 2: Perturbed (theta0=5 deg, x0=0.02) ---")
    x0_perturbed = np.array([0.02, y_start, np.deg2rad(5.0), py_body_init])
    states2, controls2 = simulate_analytical(
        mpc, x0_perturbed, target_pose, px_body, n_steps, dt
    )

    final2 = states2[-1]
    y_err2 = abs(final2[1] - y_target)
    theta_err2 = abs(np.rad2deg(final2[2]))
    x_drift2 = abs(final2[0])

    print(
        f"Final: x={final2[0]:.4f}, y={final2[1]:.4f}, "
        f"theta={np.rad2deg(final2[2]):.2f} deg"
    )
    print(f"Errors: y={y_err2:.4f}, theta={theta_err2:.2f} deg, x_drift={x_drift2:.4f}")

    y_ok2 = y_err2 < 0.10
    theta_ok2 = theta_err2 < 10.0
    print(
        f"y-convergence (<0.10 m):     {'PASS' if y_ok2 else 'FAIL'} (err={y_err2:.4f})"
    )
    print(
        f"theta-convergence (<10 deg): {'PASS' if theta_ok2 else 'FAIL'} "
        f"(err={theta_err2:.2f})"
    )

    # --- Summary ---
    print("\n" + "=" * 70)
    all_pass = y_ok and theta_ok and ctrl_ok and x_drift_ok and y_ok2 and theta_ok2
    print(f"Overall: {'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")
    print("=" * 70)

    times, nom_poses = generate_straight_trajectory(
        y_start, y_target, push_speed=0.05, dt=dt
    )
    print(f"\nNominal trajectory: {len(times)} waypoints over {times[-1]:.1f} s")
