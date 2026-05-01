from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Optional


@dataclass
class BowGeometry:
    frog_pos:  np.ndarray   # [3] frog position in sensor frame [m]
    direction: np.ndarray   # [3] unit vector frog->tip in sensor frame
    length:    float        # [m]
    mass:      float        # [kg]
    com_pos:   np.ndarray   # [3] bow CoM in sensor frame [m]


@dataclass
class StringConfig:
    name:      str
    point:     np.ndarray   # [3] any point on the string in world [m]
    direction: np.ndarray   # [3] string direction vector in world

    def unit_dir(self) -> np.ndarray:
        return self.direction / np.linalg.norm(self.direction)


def bow_string_contact_geometric(
    frog:       np.ndarray,
    bow_dir:    np.ndarray,
    bow_length: float,
    string_pt:  np.ndarray,
    string_dir: np.ndarray,
) -> Optional[float]:
    # Returns d [m] from frog to closest point on bow to the string.
    # Returns None if bow and string are parallel.
    u = bow_dir    / np.linalg.norm(bow_dir)
    v = string_dir / np.linalg.norm(string_dir)
    w = frog - string_pt
    uv    = float(u @ v)
    denom = 1.0 - uv ** 2
    if abs(denom) < 1e-8:
        return None
    d = float((-(w @ u) + (w @ v) * uv) / denom)
    return float(np.clip(d, 0.0, bow_length))


def detect_active_string(
    frog_world:    np.ndarray,
    bow_dir_world: np.ndarray,
    bow_length:    float,
    strings:       list[StringConfig],
) -> tuple[int, float]:
    # Returns (string_index, d_geo) for the string closest to the bow midpoint.
    bow_mid = frog_world + (bow_length / 2.0) * bow_dir_world / np.linalg.norm(bow_dir_world)

    best_idx  = 0
    best_dist = np.inf
    for i, s in enumerate(strings):
        v    = s.unit_dir()
        diff = bow_mid - s.point
        dist = np.linalg.norm(diff - (diff @ v) * v)
        if dist < best_dist:
            best_dist = dist
            best_idx  = i

    d_geo = bow_string_contact_geometric(
        frog_world, bow_dir_world, bow_length,
        strings[best_idx].point, strings[best_idx].direction,
    )
    return best_idx, (d_geo if d_geo is not None else bow_length / 2.0)


class BowContactKF:
    """
    Kalman filter for bow-string contact position d [m] along bow from frog.
    State: x = [d, d_dot]
    """

    def __init__(
        self,
        bow:      BowGeometry,
        dt:       float,
        q_pos:    float = 1e-5,
        q_vel:    float = 5e-3,
        r_torque: float = 4e-6,
        r_geo:    float = 1e-4,
    ):
        self.bow = bow
        self.bow.direction = bow.direction / np.linalg.norm(bow.direction)
        self.dt = dt

        self.x = np.array([bow.length / 2.0, 0.0]) # state: [d, d_dot]
        self.P = np.diag([bow.length ** 2, 1.0]) # # initial covariance (very uncertain)

        self.F = np.array([[1.0, dt], [0.0, 1.0]]) # constant-velocity prediction
        self.Q = np.diag([q_pos, q_vel]) # process noise
        self.R_torque = np.eye(3) * r_torque # measurement noise (3D torque vector)
        self.R_geo    = np.array([[r_geo]])

    def _gravity_wrench(self, R_world_to_sensor: np.ndarray): # used if the sensor measures gravity on the real robot
        g_s   = R_world_to_sensor @ np.array([0.0, 0.0, -9.81])
        f_g   = self.bow.mass * g_s
        tau_g = np.cross(self.bow.com_pos, f_g)
        return f_g, tau_g

    def _kf_update(self, H: np.ndarray, z: np.ndarray, R: np.ndarray):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))
        self.x += K @ (z - H @ self.x)
        self.P  = (np.eye(2) - K @ H) @ self.P
        self.x[0] = np.clip(self.x[0], 0.0, self.bow.length)

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.x[0] = np.clip(self.x[0], 0.0, self.bow.length)

    def update_torque( # gravity subtracted torque update if needed on the real robot
        self,
        f_meas:            np.ndarray,
        tau_meas:          np.ndarray,
        R_world_to_sensor: np.ndarray,
    ):
        # tau_contact = frog_pos x f_c + d * (bow_dir x f_c)
        # -> linear measurement: z = d * c, where c = bow_dir x f_c
        f_g, tau_g = self._gravity_wrench(R_world_to_sensor)
        f_c   = f_meas   - f_g
        tau_c = tau_meas - tau_g

        c = np.cross(self.bow.direction, f_c)
        if np.linalg.norm(c) < 1e-6:   # contact force along bow, no torque info
            return

        H = np.column_stack([c, np.zeros(3)])
        z = tau_c - np.cross(self.bow.frog_pos, f_c)
        self._kf_update(H, z, self.R_torque)

    def update_wrench( # adds string perpendicularity constraint on top (not used right now since the sim sensor already reads contact force only, but could be useful on the real robot if the sensor sees gravity)
        self,
        f_meas:            np.ndarray,
        tau_meas:          np.ndarray,
        R_world_to_sensor: np.ndarray,
        string_dir_world:  np.ndarray,
    ):
        """
        Joint force+torque update using string perpendicularity constraint.
        Solves for f_contact and d simultaneously, reducing sensitivity to gravity error.
        f_contact must be perpendicular to the string (string can't pull the bow).
        """
        f_g, tau_g = self._gravity_wrench(R_world_to_sensor)
        f_c   = f_meas   - f_g
        tau_c = tau_meas - tau_g

        # perpendicularity: f_contact . string_dir = 0 in sensor frame
        # use this to project out the gravity error along string direction
        s = R_world_to_sensor @ (string_dir_world / np.linalg.norm(string_dir_world))
        f_c = f_c - np.dot(f_c, s) * s  # remove component along string

        c = np.cross(self.bow.direction, f_c)
        if np.linalg.norm(c) < 1e-6:
            return

        H = np.column_stack([c, np.zeros(3)])
        z = tau_c - np.cross(self.bow.frog_pos, f_c)
        self._kf_update(H, z, self.R_torque)

    def update_geometric(self, d_geo: float): # fuses d_geo as an additional measurement (not used for now since the sim doesn't have geometric noise, but could be useful on the real robot to complement the torque update)
        H = np.array([[1.0, 0.0]])
        z = np.array([float(d_geo)])
        self._kf_update(H, z, self.R_geo)

    @property
    def d(self) -> float:
        return float(self.x[0])

    @property
    def d_dot(self) -> float:
        return float(self.x[1])

    @property
    def d_std(self) -> float:
        return float(np.sqrt(max(self.P[0, 0], 0.0)))


def run_simulation(): # used when running this file standalone to test the KF against a simulated bow-string contact scenario with noise
    rng = np.random.default_rng(0)

    dt  = 0.01
    T   = 4.0
    t   = np.arange(0.0, T, dt)
    N   = len(t)

    BOW_LENGTH = 0.776

    bow = BowGeometry(
        frog_pos  = np.array([0.0, 0.0, -0.40]),
        direction = np.array([1.0, 0.0, 0.0]),
        length    = BOW_LENGTH,
        mass      = 0.060,
        com_pos   = np.array([0.30, 0.0, 0.0]),
    )

    R_world_to_sensor = np.eye(3)

    # triangular wave: frog->tip->frog over 2 s
    d_true = BOW_LENGTH * (1.0 - np.abs(1.0 - (t % 2.0)))

    f_contact = np.array([0.0, -1.5, 0.0])
    tau_true  = np.stack([np.cross(d * bow.direction, f_contact) for d in d_true])

    g_s   = R_world_to_sensor @ np.array([0.0, 0.0, -9.81])
    f_g   = bow.mass * g_s
    tau_g = np.cross(bow.com_pos, f_g)

    sigma_f   = 0.05
    sigma_tau = 0.002
    sigma_geo = 0.010

    kf = BowContactKF(bow, dt)

    d_est      = np.zeros(N)
    d_std_hist = np.zeros(N)

    for k in range(N):
        f_meas   = f_contact + f_g + rng.normal(0.0, sigma_f,   3)
        tau_meas = tau_true[k] + tau_g + rng.normal(0.0, sigma_tau, 3)
        d_geo    = float(np.clip(d_true[k] + rng.normal(0.0, sigma_geo), 0.0, BOW_LENGTH))

        kf.predict()
        kf.update_torque(f_meas, tau_meas, R_world_to_sensor)
        kf.update_geometric(d_geo)

        d_est[k]      = kf.d
        d_std_hist[k] = kf.d_std

    err_cm = (d_est - d_true) * 100.0

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    ax1.plot(t, d_true * 100, 'k--', lw=2, label='Ground truth (OpenSAI)')
    ax1.fill_between(
        t,
        (d_est - 2 * d_std_hist) * 100,
        (d_est + 2 * d_std_hist) * 100,
        alpha=0.25, color='C1', label='KF +-2sigma',
    )
    ax1.plot(t, d_est * 100, color='C1', lw=2, label='KF estimate')
    ax1.set_ylabel('Contact position [cm from frog]')
    ax1.set_ylim(-5, BOW_LENGTH * 100 + 5)
    ax1.legend(loc='upper right')
    ax1.grid(alpha=0.3)
    ax1.set_title('Bow-string contact point - KF vs. ground truth')

    ax2.plot(t, err_cm, color='C2', lw=1.5)
    ax2.axhline(0, color='k', lw=0.5)
    ax2.fill_between(t, -2 * d_std_hist * 100, 2 * d_std_hist * 100,
                     alpha=0.2, color='C1', label='KF +-2sigma')
    ax2.set_ylabel('Error [cm]')
    ax2.set_xlabel('Time [s]')
    ax2.legend(loc='upper right')
    ax2.grid(alpha=0.3)
    ax2.set_title('Estimation error')

    plt.tight_layout()
    plt.savefig('bow_contact_estimation.png', dpi=150, bbox_inches='tight')
    plt.show()

    rmse = np.sqrt(np.mean(err_cm ** 2))
    print(f"RMSE: {rmse:.3f} cm  |  Max |error|: {np.max(np.abs(err_cm)):.3f} cm")


def opensai_loop(dt: float = 0.01):
    """
    Connects to Redis, runs the KF in real time, and publishes results.
    Run after simviz and controller are already running.
    """
    import time
    import redis

    r = redis.Redis(host="localhost", port=6379, decode_responses=True)

    def parse_floats(raw: str) -> list:
        return [float(x) for x in raw.replace('[', '').replace(']', '').replace('\n', ',').split(',') if x.strip()]

    def get_vec(key: str) -> np.ndarray:
        return np.array(parse_floats(r.get(key)))

    def set_scalar(key: str, val: float):
        r.set(key, str(val))

    # --- bow geometry in bow-link frame ---
    # From mesh analysis (bow.obj Z range x 0.01 scale):
    #   frog end: Z = -0.400 m  (where robot holds the bow)
    #   tip end:  Z = +0.376 m
    #   total length: about 0.776 m
    # Controller publishes actual frog world position: bow_origin + R*[0,0,-0.40]
    BOW_LENGTH = 0.776  # meters (frog to tip)
    bow = BowGeometry(
        frog_pos  = np.array([0.0, 0.0, -0.40]),              # frog in bow frame
        direction = np.array([0.0, 0.0,  1.0]),               # bow extends along +Z in bow frame
        length    = BOW_LENGTH,
        mass      = 0.060,
        com_pos   = np.array([0.0, 0.0, -0.40 + BOW_LENGTH / 2.0]),  # midpoint in bow frame
    )

    # --- string positions in world frame --- not really needed since the sim publishes the contact point directly, but useful for testing geometric estimation and string detection logic
    # violin_origin calibrated from contact point at G string: [0.894, -0.075, 0.253]
    # G string offset is [-0.02, 0, 0.12] => violin_origin = [0.914, 0, 0.133]
    violin_origin = np.array([0.914, 0.0, 0.133])
    strings = [
        StringConfig("G", point=violin_origin + np.array([-0.02, 0.0, 0.12]), direction=np.array([0.0, 1.0, 0.0])),
        StringConfig("D", point=violin_origin + np.array([-0.06, 0.0, 0.10]), direction=np.array([0.0, 1.0, 0.0])),
        StringConfig("A", point=violin_origin + np.array([ 0.02, 0.0, 0.12]), direction=np.array([0.0, 1.0, 0.0])),
        StringConfig("E", point=violin_origin + np.array([ 0.06, 0.0, 0.10]), direction=np.array([0.0, 1.0, 0.0])),
    ]

    kf = BowContactKF(bow, dt, q_pos=1e-4, q_vel=1e-3, r_torque=1e-4)


    # --- real-time plot setup ---
    history = 500  # about 5s at dt=0.01
    t_hist       = np.full(history, np.nan)
    d_est_hist   = np.full(history, np.nan)
    d_geo_hist   = np.full(history, np.nan)
    d_gt_hist    = np.full(history, np.nan)
    std_hist     = np.full(history, np.nan)

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    fig.suptitle('Bow-string contact point estimation', fontsize=13)

    # top: contact position
    line_gt,  = ax1.plot([], [], 'k--', lw=1.5, label='ground truth')
    line_geo, = ax1.plot([], [], 'b--', lw=1.5, label='geometric')
    line_est, = ax1.plot([], [], 'C1',  lw=2,   label='KF estimate')
    ax1.set_ylabel('d from frog [cm]')
    ax1.axhline(0,              color='gray', lw=0.8)
    ax1.axhline(BOW_LENGTH*100, color='gray', lw=0.8)
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(alpha=0.3)

    # bottom: error (KF vs gt)
    line_err, = ax2.plot([], [], 'C2', lw=1.5, label='|err| KF vs GT')
    ax2.axhline(0, color='gray', lw=0.8)
    ax2.set_ylabel('|error| [cm]')
    ax2.set_xlabel('time step')
    ax2.set_ylim(0, 6)
    ax2.legend(loc='upper right', fontsize=9)
    ax2.grid(alpha=0.3)

    fig.tight_layout()

    # --- no gravity subtraction needed ---
    # The SAI2 F/T sensor reads contact forces only; the physics engine handles bow
    # gravity separately (the robot controller's gravity compensation ensures the arm
    # is in equilibrium, but the sensor sees only the string's reaction force on the bow).
    # In free air: f_meas is about 0, tau_meas is about 0.  During bowing: f_meas = f_contact,
    # tau_meas = tau_contact.  Using f_c = f_meas and tau_c = tau_meas directly gives
    # the exact linear measurement model:
    #   z = tau_meas - cross(frog_pos, f_meas) = d * cross(bow_dir, f_meas) = d * c
    # --> d_raw = d_true exactly (no gravity error at any force level).
    print("KF running.")

    step = 0
    while True:
        loop_start = time.time()

        # read from Redis
        f_meas            = get_vec("sai::sim::bow::sensor::force::local")
        tau_meas          = get_vec("sai::sim::bow::sensor::moment::local")
        frog_world        = get_vec("sai::sim::flexiv::bow::frog_position")
        bow_dir_world     = get_vec("sai::sim::flexiv::bow::direction")
        contact_pos_world = get_vec("sai::sim::bow::contact::position")

        # bow geometry is constant in sensor frame
        # frog (held end) at bow Z=+0.376, sensor at bow Z=+0.35
        # => frog in sensor frame = [0, -0.05, 0.376-0.35] = [0, -0.05, 0.026]
        # tip at bow Z=-0.40 => tip in sensor frame = [0, -0.05, -0.75]
        # bow_dir = frog->tip = [0, 0, -1] in sensor frame
        kf.bow.direction = np.array([0.0,  0.0, -1.0])
        kf.bow.frog_pos  = np.array([0.0, -0.05,  0.026])

        # read target string from controller
        target_str_raw = r.get("sai::sim::flexiv::target_string")
        if target_str_raw is not None:
            target_str_val = float(target_str_raw)
        else:
            target_str_val = -1.0

        # sensor reads contact force only — use directly, no gravity subtraction
        f_c   = f_meas
        tau_c = tau_meas

        # pick active string (clamp to 0-3 once bowing, else default 0)
        if 0 <= target_str_val <= 3:
            string_idx = int(target_str_val)
        else:
            string_idx, _ = detect_active_string(frog_world, bow_dir_world, bow.length, strings)
            string_idx = max(0, min(3, string_idx))

        d_geo = bow_string_contact_geometric(
            frog_world, bow_dir_world, bow.length,
            strings[string_idx].point, strings[string_idx].direction,
        ) or bow.length / 2.0

        # ground truth d
        d_gt = float(np.dot(contact_pos_world - frog_world, bow_dir_world))

        # KF update using full f_c (no perpendicularity projection).
        # The string direction in this sim is [0,1,0]=Y, and the contact force is also
        # primarily in Y (from ee_force_desired=[0,-5,0]), so projecting out Y from f_c
        # would remove the entire contact signal. Use f_c directly instead.
        c = np.cross(kf.bow.direction, f_c)
        c_norm = np.linalg.norm(c)

        # debug: raw d estimate from torque (instantaneous, no filtering)
        if c_norm > 1e-6:
            lhs   = tau_c - np.cross(kf.bow.frog_pos, f_c)
            d_raw = float(np.dot(lhs, c) / np.dot(c, c))
            if step % 20 == 0:
                print(f"  d_raw={d_raw*100:.1f}cm  d_gt={d_gt*100:.1f}cm  diff={abs(d_raw-d_gt)*100:.1f}cm"
                      f"  |c|={c_norm:.3f}  f=[{f_c[0]:.2f},{f_c[1]:.2f},{f_c[2]:.2f}]"
                      f"  tau=[{tau_c[0]:.3f},{tau_c[1]:.3f},{tau_c[2]:.3f}]")

        kf.predict()
        # Gate KF torque update: avoid numerical instability near-zero contact force.
        # Without gravity subtraction, d_raw = d_true exactly, so threshold only
        # prevents division-by-near-zero noise amplification.
        MIN_CONTACT_FORCE = 0.1  # N
        if c_norm > MIN_CONTACT_FORCE:
            H = np.column_stack([c, np.zeros(3)])
            z = tau_c - np.cross(kf.bow.frog_pos, f_c)
            kf._kf_update(H, z, kf.R_torque)
        else:
            # no contact (air phase / string switch) --> so we want to grow position uncertainty
            # so KF snaps quickly to the correct d when contact resumes
            kf.P[0, 0] += 1e-3

        # publish KF output
        set_scalar("sai::sim::bow::kf::d", kf.d)
        set_scalar("sai::sim::bow::kf::d_dot", kf.d_dot)
        set_scalar("sai::sim::bow::kf::string_index", string_idx)

        # update rolling history
        t_hist     = np.roll(t_hist,     -1); t_hist[-1]     = step
        d_est_hist = np.roll(d_est_hist, -1); d_est_hist[-1] = kf.d * 100
        d_geo_hist = np.roll(d_geo_hist, -1); d_geo_hist[-1] = d_geo * 100
        d_gt_hist  = np.roll(d_gt_hist,  -1); d_gt_hist[-1]  = d_gt  * 100
        std_hist   = np.roll(std_hist,   -1); std_hist[-1]   = kf.d_std * 100

        # update plot every 5 steps
        if step % 5 == 0:
            valid = ~np.isnan(t_hist)
            xs = t_hist[valid]
            est = d_est_hist[valid]
            geo = d_geo_hist[valid]
            gt  = d_gt_hist[valid]
            std = std_hist[valid]
            err = np.abs(est - gt)

            line_est.set_data(xs, est)
            line_geo.set_data(xs, geo)
            line_gt.set_data(xs,  gt)
            line_err.set_data(xs, err)

            # redraw confidence band
            for col in ax1.collections:
                col.remove()
            ax1.fill_between(xs, est - 2*std, est + 2*std, alpha=0.2, color='tomato')

            # auto-scale y to fit gt (which can go negative)
            all_vals = np.concatenate([est, gt])
            finite = all_vals[np.isfinite(all_vals)]
            if len(finite):
                margin = 2.0
                ax1.set_ylim(min(finite) - margin, max(finite) + margin)
            ax2.set_ylim(0, max(6.0, float(np.nanmax(err)) + 1))

            ax1.set_xlim(xs[0], xs[-1] + 1)
            fig.suptitle(f'string = {strings[string_idx].name}  |  '
                         f'd_est = {kf.d*100:.1f} cm  |  '
                         f'd_gt = {d_gt*100:.1f} cm  |  '
                         f'err = {abs(kf.d - d_gt)*100:.1f} cm', fontsize=12)
            fig.canvas.draw()
            fig.canvas.flush_events()

        if step % 20 == 0:
            print(f"string={strings[string_idx].name}  "
                  f"d_est={kf.d*100:.1f}cm  d_geo={d_geo*100:.1f}cm  "
                  f"d_gt={d_gt*100:.1f}cm  err={abs(kf.d - d_gt)*100:.1f}cm  "
                  f"std={kf.d_std*100:.1f}cm")

        step += 1
        elapsed = time.time() - loop_start
        time.sleep(max(0.0, dt - elapsed))


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == '--sim':
        run_simulation()
    else:
        opensai_loop()
