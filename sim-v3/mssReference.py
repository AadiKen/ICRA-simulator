# mss_benchmark.py
# Runs the Otter USV model under the same open-loop commands as your JS sim
# and produces a comparison CSV + plots.
#
# Install: pip install python-vehicle-simulator numpy matplotlib pandas
# Run:     python mss_benchmark.py --js_log js_sim_log.csv

import argparse, json, math, os, sys, tempfile

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "bcod-matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", os.path.join(tempfile.gettempdir(), "bcod-cache"))

import numpy as np

JS_TIME = "time"
JS_NORTH = "north"
JS_EAST = "east"
JS_HEADING = "heading"
JS_TOTAL_ACCEL_N = "total_accel_n"
JS_YAW_ACCEL = "yaw_accel"
JS_SURGE = "surge"
JS_SWAY = "sway"
JS_SPEED = "speed"

MSS_PRESET = {
    "simHz": 12,
    "durationSec": 90,
    "dt": 1 / 12,
    "boat": {
        "maxSpeed": 2.5,
        "maxAcceleration": 0.9,
        "maxDeceleration": 0.9,
        "maxTurn": 0.7,
        "mass": 1000.0,
        "beam": 2.4,
        "height": 1.0,
        "length": 4.8,
        "startNorth": 12.0,
        "startEast": 12.0,
        "draft": 0.15397881251539788,
        "waterDensity": 1025.0,
        "surgeCd": 0.22,
        "swayCd": 1.15,
        "Iz": 2399.9999999999995,
        "XuDot": -50.0,
        "YvDot": -750.0,
        "NrDot": -191.99999999999997,
        "Xu": 25.6,
        "Yv": 144.00000000000003,
        "Nr": 2414.9999999999995,
        "Xuu": 41.66666666666667,
        "Yvv": 435.6060606060606,
        "Nrr": 327.2727272727273
    },
    # MSS/NED order: (north, east). scenarioPresets.js stores vec3(east, up, north).
    "waypoints": [(22.0, 22.0), (50.0, 25.0), (65.0, 72.0)],
    "tolerance": 1.0,
    "current": {"north": 0.02, "east": 0.08},
    "waves": [
        {"heading": 35, "peakHeight": 0.16, "wavelength": 11, "speed": 1.34, "steepness": 0.45},
        {"heading": 120, "peakHeight": 0.16, "wavelength": 6, "speed": 1.24, "steepness": 0.25}
    ]
}


def load_js_log(path):
    """Load either the current browser export schema or the older benchmark schema."""
    import pandas as pd
    df = pd.read_csv(path, comment="#")
    if df.empty:
        sys.exit(f"JS log is empty: {path}")

    if JS_TIME in df.columns:
        df[JS_TIME] = df[JS_TIME] - df[JS_TIME].iloc[0]

    # Backward-compatible aliases for older benchmark CSVs.
    if JS_NORTH not in df.columns and "x" in df.columns:
        df[JS_NORTH] = df["x"]
    if JS_EAST not in df.columns and "z" in df.columns:
        df[JS_EAST] = df["z"]
    if JS_TOTAL_ACCEL_N not in df.columns:
        if "az" in df.columns:
            df[JS_TOTAL_ACCEL_N] = df["az"]
        elif "total_accel_n" not in df.columns:
            df[JS_TOTAL_ACCEL_N] = 0.0
    if JS_YAW_ACCEL not in df.columns:
        if "yaw_rate_dot" in df.columns:
            df[JS_YAW_ACCEL] = df["yaw_rate_dot"]
        else:
            df[JS_YAW_ACCEL] = 0.0

    required = [JS_TIME, JS_NORTH, JS_EAST, JS_HEADING]
    missing = [col for col in required if col not in df.columns]
    if missing:
        sys.exit(f"JS CSV missing columns: {missing}\nFound: {list(df.columns)}")

    return df.sort_values(JS_TIME).reset_index(drop=True)


def js_speed_series(df):
    import pandas as pd
    if JS_SPEED in df.columns:
        return df[JS_SPEED]
    if JS_SURGE in df.columns and JS_SWAY in df.columns:
        return np.sqrt(df[JS_SURGE]**2 + df[JS_SWAY]**2)
    if "vx" in df.columns and "vz" in df.columns:
        return np.sqrt(df["vx"]**2 + df["vz"]**2)
    return pd.Series(np.zeros(len(df)), index=df.index)

# ── 3-DOF reference parameters matched to scenarioPresets.js ─────────────────

class OtterUSV:
    """
    3-DOF Fossen maneuvering model (surge/sway/yaw) matching MSS otter.py
    structure but parameterised to your schema.js boatConfig values.

    State vector:  eta = [x, y, psi]   (NED position + heading)
                   nu  = [u, v, r]     (surge vel, sway vel, yaw rate)

    EOM:  M * nu_dot + C(nu)*nu + D(nu)*nu = tau
    """

    def __init__(self):
        preset = MSS_PRESET["boat"]
        self.mass = preset["mass"]
        self.beam = preset["beam"]
        self.length = preset["length"]
        self.height = preset["height"]
        Iz_rb = preset["Iz"]
        Xu_dot = preset["XuDot"]
        Yv_dot = preset["YvDot"]
        Nr_dot = preset["NrDot"]

        self.M = np.diag([
            self.mass - Xu_dot,
            self.mass - Yv_dot,
            Iz_rb     - Nr_dot
        ])
        self.M_inv = np.linalg.inv(self.M)

        self.Xu_lin = preset["Xu"]
        self.Yv_lin = preset["Yv"]
        self.Nr_lin = preset["Nr"]
        self.Xu_q = preset["Xuu"]
        self.Yv_q = preset["Yvv"]
        self.Nr_q = preset["Nrr"]
        self.max_speed = preset["maxSpeed"]
        self.max_accel = preset["maxAcceleration"]
        self.max_turn = preset["maxTurn"]

    def coriolis(self, nu):
        """Rigid-body Coriolis matrix C(nu)"""
        u, v, r = nu
        m = self.mass
        return np.array([
            [0,     0,    -m * v],
            [0,     0,     m * u],
            [m * v, -m * u, 0  ]
        ])

    def damping(self, nu):
        """Linear + quadratic damping forces"""
        u, v, r = nu
        Du = self.Xu_lin + self.Xu_q * abs(u)
        Dv = self.Yv_lin + self.Yv_q * abs(v)
        Dr = self.Nr_lin + self.Nr_q * abs(r)
        return np.array([Du * u, Dv * v, Dr * r])

    def step(self, eta, nu, tau, dt):
        """
        Semi-implicit Euler step — matches your schema.js integrator.
        eta = [x, y, psi],  nu = [u, v, r],  tau = [X, Y, N] (forces/moments)
        """
        psi = eta[2]
        R = np.array([
            [ math.cos(psi), -math.sin(psi), 0],
            [ math.sin(psi),  math.cos(psi), 0],
            [0,               0,             1]
        ])

        # Equations of motion
        nu_dot = self.M_inv @ (tau - self.coriolis(nu) @ nu - self.damping(nu))

        # Integrate velocity → update state (Euler, same as your sim)
        nu_new  = nu  + nu_dot * dt
        eta_new = eta + R @ nu_new * dt   # use updated velocity (semi-implicit)

        # Wrap heading
        eta_new[2] = math.atan2(math.sin(eta_new[2]), math.cos(eta_new[2]))

        return eta_new, nu_new, nu_dot


def replay_js_commands(js_df, dt=1/12):
    """
    Open-loop replay: read thrust/rudder from JS log, apply to MSS model.
    Since JS logs don't record raw commands, we reconstruct tau from
    the logged acceleration (F = m * a).
    """
    usv = OtterUSV()
    results = []

    # Initial conditions from your scenarioPresets.js
    # startPos = vec3(12, 0, 12), initial heading toward first waypoint (22,22)
    start_n = MSS_PRESET["boat"]["startNorth"]
    start_e = MSS_PRESET["boat"]["startEast"]
    first_n, first_e = MSS_PRESET["waypoints"][0]
    psi0 = math.atan2(first_e - start_e, first_n - start_n)

    eta = np.array([start_n, start_e, psi0])
    nu  = np.array([0.0, 0.0, 0.0])

    for i, row in js_df.iterrows():
        # Reconstruct surge/yaw forces from JS logged acceleration
        # tau_surge = mass * guidance_accel (projected forward)
        # tau_yaw   = Iz * angular_accel_y
        a_total = row.get(JS_TOTAL_ACCEL_N, row.get("az", 0.0))
        alpha_y = row.get(JS_YAW_ACCEL, row.get("yaw_rate_dot", 0.0))

        tau_surge = usv.mass * a_total
        tau_yaw   = usv.M[2, 2] * alpha_y
        tau = np.array([tau_surge, 0.0, tau_yaw])

        eta, nu, nu_dot = usv.step(eta, nu, tau, dt)
        results.append({
            "time":    row["time"],
            "mss_x":   eta[0],
            "mss_z":   eta[1],
            "mss_u":   nu[0],    # surge speed
            "mss_v":   nu[1],    # sway speed
            "mss_psi": eta[2],   # heading
        })

    return pd.DataFrame(results)


def run_independent_mss(duration=90, dt=1/12):
    """
    Run the MSS Otter model independently under a simple
    waypoint-following guidance law — no JS dependency.
    Use this when you want a clean MSS baseline, not a replay.
    """
    usv = OtterUSV()
    waypoints = MSS_PRESET["waypoints"]
    wp_idx = 0
    tolerance = MSS_PRESET["tolerance"]

    start_n = MSS_PRESET["boat"]["startNorth"]
    start_e = MSS_PRESET["boat"]["startEast"]
    psi0 = math.atan2(waypoints[0][1] - start_e, waypoints[0][0] - start_n)
    eta = np.array([start_n, start_e, psi0])
    nu  = np.array([0.0, 0.0, 0.0])

    results = []
    max_steps = math.ceil(duration / dt - 1e-12)

    for step_index in range(max_steps):
        if wp_idx >= len(waypoints):
            break
        t = step_index * dt
        waypoint_n, waypoint_e = waypoints[wp_idx]
        dn, de = waypoint_n - eta[0], waypoint_e - eta[1]
        dist = math.sqrt(dn**2 + de**2)

        if dist < tolerance:
            wp_idx += 1
            if wp_idx >= len(waypoints):
                break
            waypoint_n, waypoint_e = waypoints[wp_idx]
            dn, de = waypoint_n - eta[0], waypoint_e - eta[1]
            dist = math.sqrt(dn**2 + de**2)

        # Simple heading error → rudder → yaw torque
        desired_psi = math.atan2(de, dn)
        psi_err = math.atan2(math.sin(desired_psi - eta[2]),
                             math.cos(desired_psi - eta[2]))

        # P-controller for heading, speed proportional to alignment
        speed_factor = max(0.15, 1 - abs(psi_err) / math.pi)
        desired_speed = usv.max_speed * speed_factor
        speed_err = desired_speed - nu[0]

        tau_surge = usv.mass * np.clip(speed_err, -usv.max_accel, usv.max_accel)
        tau_yaw   = usv.M[2, 2] * np.clip(psi_err * 1.5 - nu[2] * 2.0,
                                           -usv.max_turn, usv.max_turn)
        tau = np.array([tau_surge, 0.0, tau_yaw])

        eta, nu, _ = usv.step(eta, nu, tau, dt)

        results.append({
            "time":    t,
            "mss_x":   eta[0],
            "mss_z":   eta[1],
            "mss_u":   nu[0],
            "mss_psi": eta[2],
            "wp_idx":  wp_idx
        })
    return pd.DataFrame(results)


def compare_and_plot(js_df, mss_df):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    """Align on time, compute error metrics, produce comparison plots."""

    # Interpolate MSS onto JS timestamps
    from scipy.interpolate import interp1d
    t_js = js_df["time"].values
    t_mss = mss_df["time"].values

    interp_x   = interp1d(t_mss, mss_df["mss_x"],   bounds_error=False, fill_value="extrapolate")
    interp_z   = interp1d(t_mss, mss_df["mss_z"],   bounds_error=False, fill_value="extrapolate")
    interp_psi = interp1d(t_mss, mss_df["mss_psi"], bounds_error=False, fill_value="extrapolate")

    mss_x_aligned   = interp_x(t_js)
    mss_z_aligned   = interp_z(t_js)
    mss_psi_aligned = interp_psi(t_js)

    pos_error   = np.sqrt((js_df[JS_NORTH] - mss_x_aligned)**2 +
                          (js_df[JS_EAST] - mss_z_aligned)**2)
    heading_err = np.abs(js_df[JS_HEADING] - mss_psi_aligned)

    rmse_pos     = np.sqrt(np.mean(pos_error**2))
    max_pos_err  = pos_error.max()
    rmse_heading = np.sqrt(np.mean(heading_err**2))

    print("\n── Benchmark Results ─────────────────────────────")
    print(f"  Position RMSE:        {rmse_pos:.3f} m")
    print(f"  Max position error:   {max_pos_err:.3f} m")
    print(f"  Heading RMSE:         {np.degrees(rmse_heading):.2f} deg")
    print("──────────────────────────────────────────────────\n")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("JS Sim vs MSS Otter USV — Benchmark Comparison", fontsize=14)

    # Trajectory
    ax = axes[0, 0]
    ax.plot(js_df[JS_NORTH], js_df[JS_EAST], "b-", lw=2, label="JS Sim")
    ax.plot(mss_df["mss_x"], mss_df["mss_z"], "r--", lw=2, label="MSS Otter")
    for waypoint_n, waypoint_e in MSS_PRESET["waypoints"]:
        ax.plot(waypoint_n, waypoint_e, "g^", ms=10)
    ax.set_xlabel("North (m)"); ax.set_ylabel("East (m)")
    ax.set_title("Trajectory"); ax.legend(); ax.set_aspect("equal"); ax.grid(True)

    # Position error over time
    ax = axes[0, 1]
    ax.plot(t_js, pos_error, "k-", lw=1.5)
    ax.axhline(rmse_pos, color="r", ls="--", label=f"RMSE = {rmse_pos:.2f} m")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Position error (m)")
    ax.set_title("Position Divergence Over Time"); ax.legend(); ax.grid(True)

    # Surge speed
    ax = axes[1, 0]
    ax.plot(js_df["time"], js_speed_series(js_df), "b-", lw=2, label="JS Sim")
    if "mss_u" in mss_df:
        ax.plot(mss_df["time"], mss_df["mss_u"], "r--", lw=2, label="MSS")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Speed (m/s)")
    ax.set_title("Surge Speed"); ax.legend(); ax.grid(True)

    # Heading
    ax = axes[1, 1]
    ax.plot(js_df["time"], np.degrees(js_df["heading"]), "b-",  lw=2, label="JS Sim")
    ax.plot(mss_df["time"], np.degrees(mss_df["mss_psi"]), "r--", lw=2, label="MSS")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Heading (deg)")
    ax.set_title("Heading"); ax.legend(); ax.grid(True)

    plt.tight_layout()
    plt.savefig("benchmark_comparison.png", dpi=150)
    print("Plot saved to benchmark_comparison.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--js_log", default=None,
                        help="Path to CSV exported from JS sim. "
                             "If omitted, runs MSS standalone only.")
    parser.add_argument("--dt",     default=1/12, type=float)
    parser.add_argument("--duration", default=90, type=float)
    parser.add_argument("--print_preset_json", action="store_true",
                        help="Print the MSS preset as JSON and exit.")
    parser.add_argument("--out_csv", default="mss_baseline.csv",
                        help="Output path for standalone MSS CSV when --js_log is omitted.")
    args = parser.parse_args()

    if args.print_preset_json:
        print(json.dumps(MSS_PRESET, indent=2, sort_keys=True))
        sys.exit(0)

    mss_df = run_independent_mss(duration=args.duration, dt=args.dt)
    print(f"MSS run complete: {len(mss_df)} steps, "
          f"final pos ({mss_df.mss_x.iloc[-1]:.1f}, {mss_df.mss_z.iloc[-1]:.1f})")

    if args.js_log:
        js_df = load_js_log(args.js_log)
        compare_and_plot(js_df, mss_df)
    else:
        print("\nNo --js_log provided. MSS standalone results:")
        print(mss_df[["time","mss_x","mss_z","mss_u","mss_psi"]].tail(10).to_string())
        mss_df.to_csv(args.out_csv, index=False)
        print(f"Saved to {args.out_csv}")
