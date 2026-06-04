"""
Stage-0 analytical pusher-slider simulator for validating a CoM-estimation
experiment via quasi-static 2D pushing.

Physics: single-point STICKING contact + ellipsoidal limit surface.
Object frame is centered at the GEOMETRIC center; the true center of mass (CoM)
is offset by `com` = (cx, cy). All moment arms that drive rotation are measured
RELATIVE TO THE TRUE CoM. The control "belief" about the CoM only changes the
push direction chosen to attempt a pure translation -- which is exactly the A/B/C
contrast.

Key relations (body frame, contact at r=(px,py) relative to CoM):
  omega = (px*vy - py*vx) / c**2
  [vp_x; vp_y] = M @ [vx; vy],  M = [[1+py^2/c^2, -px*py/c^2],
                                     [-px*py/c^2, 1+px^2/c^2]]
where (vp_x,vp_y) is the pusher (contact-point) velocity and (vx,vy,omega) the
body twist. To translate without rotating you must push along the line through
the CoM -- so a wrong CoM belief yields a systematic, d-proportional rotation.
"""

from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .. import paths

C = 0.05  # limit-surface characteristic length [m] (~ radius of gyration of pressure)
HALF = 0.05  # object half-size [m]
CONTACT_GEOM = np.array([0.0, -HALF])  # physical contact point in OBJECT (geom-centered) frame
PUSH_SPEED = 0.02  # m/s (quasi-static, slow)
DT = 0.01
T_PUSH = 1.0  # s


def rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def body_twist(contact_rel_com, vp_body, c=C):
    """Given contact point relative to CoM and pusher velocity (body frame),
    return body twist (vx, vy, omega) under sticking + ellipsoidal LS."""
    px, py = contact_rel_com
    M = np.array([[1.0 + py * py / c**2, -px * py / c**2], [-px * py / c**2, 1.0 + px * px / c**2]])
    vx, vy = np.linalg.solve(M, vp_body)
    omega = (px * vy - py * vx) / c**2
    return vx, vy, omega


def simulate_push(
    com, push_dir_world, contact_geom=CONTACT_GEOM, speed=PUSH_SPEED, dt=DT, t_push=T_PUSH, c=C
):
    """Forward-integrate object pose for a straight push in a fixed WORLD
    direction. `com` is the TRUE CoM (object frame). Returns final (x,y,theta)
    and the body-twist history (for the estimator)."""
    pose = np.array([0.0, 0.0, 0.0])  # world x, y, theta
    push_dir_world = push_dir_world / np.linalg.norm(push_dir_world)
    contact_rel_com = contact_geom - np.asarray(com)
    history = []
    steps = int(t_push / dt)
    for _ in range(steps):
        theta = pose[2]
        vp_world = speed * push_dir_world
        vp_body = rot(theta).T @ vp_world
        vx, vy, omega = body_twist(contact_rel_com, vp_body, c=c)
        history.append((contact_rel_com.copy(), vp_body.copy(), (vx, vy, omega)))
        v_world = rot(theta) @ np.array([vx, vy])
        pose[:2] += v_world * dt
        pose[2] += omega * dt
    return pose, history


def estimate_com(true_com, contacts, c=C, noise=0.0, seed=0):
    """Identify CoM from a few exploratory pushes with varied contact/direction.
    Linear LS on:  CoMx*vy - CoMy*vx = (contactx*vy - contacty*vx) - omega*c^2 .
    Observations (vx,vy,omega) are taken in the BODY frame at the first step."""
    rng = np.random.default_rng(seed)
    A, b = [], []
    for cg in contacts:
        for ang in (np.deg2rad(80), np.deg2rad(100), np.deg2rad(60)):
            push_dir = np.array([np.cos(ang), np.sin(ang)])
            _, hist = simulate_push(
                true_com, push_dir, contact_geom=cg, t_push=DT, c=c
            )  # single-step probe
            _, _, (vx, vy, omega) = hist[0]
            vx += rng.normal(0, noise)
            vy += rng.normal(0, noise)
            omega += rng.normal(0, noise / C)
            A.append([vy, -vx])
            b.append(cg[0] * vy - cg[1] * vx - omega * c**2)
    com_hat, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
    return com_hat


def induced_rotation(true_com, believed_com, contact_geom=CONTACT_GEOM):
    """Pick the push direction that the controller *believes* gives pure
    translation (line from contact through believed CoM), then run TRUE physics."""
    push_dir = np.asarray(believed_com) - contact_geom  # push toward believed CoM
    pose, _ = simulate_push(true_com, push_dir, contact_geom=contact_geom)
    return abs(pose[2])  # |Delta theta| over the push


def main(out_dir: Path | None = None):
    out_dir = Path(out_dir) if out_dir is not None else paths.results_dir()
    out_dir.mkdir(parents=True, exist_ok=True)
    offsets = np.linspace(0.0, 0.03, 7)  # true CoM offset d along x [m]
    contacts = [np.array([0.0, -HALF]), np.array([HALF, -HALF]), np.array([-HALF, -HALF])]

    rows = []
    for d in offsets:
        true_com = np.array([d, 0.0])
        com_hat = estimate_com(true_com, contacts, noise=1e-4)
        dth_A = np.rad2deg(induced_rotation(true_com, np.array([0.0, 0.0])))  # geom center
        dth_B = np.rad2deg(induced_rotation(true_com, com_hat))  # estimated CoM
        dth_C = np.rad2deg(induced_rotation(true_com, true_com))  # true CoM
        rows.append((d, np.linalg.norm(com_hat - true_com), dth_A, dth_B, dth_C))
        print(
            f"d={d * 1000:5.1f} mm | CoM err={1000 * rows[-1][1]:5.2f} mm | "
            f"dtheta  A(geom)={dth_A:6.2f}  B(est)={dth_B:6.2f}  C(true)={dth_C:6.2f} deg"
        )

    rows = np.array(rows)
    plt.figure(figsize=(6, 4))
    plt.plot(rows[:, 0] * 1000, rows[:, 2], "o-", label="A: geometric center")
    plt.plot(rows[:, 0] * 1000, rows[:, 3], "s-", label="B: estimated CoM")
    plt.plot(rows[:, 0] * 1000, rows[:, 4], "^-", label="C: true CoM")
    plt.xlabel("true CoM offset d [mm]")
    plt.ylabel(r"induced rotation $|\Delta\theta|$ [deg]")
    plt.title("CoM-belief vs induced rotation in a 'pure-translation' push")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    out_path = out_dir / "induced_rotation.png"
    plt.savefig(out_path, dpi=140)
    print(f"\nSaved plot -> {out_path}")


if __name__ == "__main__":
    main()
