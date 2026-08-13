"""Compare bcod-sim and Unity planar trajectory CSV files.

Both inputs must use the exportBcodLog.js schema and SI units:
    t,N,E,yaw,u,v,r

N and E are NED world-frame positions in metres. yaw is heading in radians,
positive from North toward East. u and v are body-frame surge and sway in
metres per second, and r is body yaw rate in radians per second.
"""

import argparse
import csv
import math
import sys


COLUMNS = ["t", "N", "E", "yaw", "u", "v", "r"]
COMPARE_COLUMNS = COLUMNS[1:]


def load_csv(path):
    with open(path, newline="") as stream:
        reader = csv.DictReader(stream)
        missing = [column for column in COLUMNS if column not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path} is missing required columns: {', '.join(missing)}")
        rows = [{column: float(row[column]) for column in COLUMNS} for row in reader]

    if not rows:
        raise ValueError(f"{path} contains no trajectory rows")
    if any(current["t"] <= previous["t"] for previous, current in zip(rows, rows[1:])):
        raise ValueError(f"{path} timestamps must be strictly increasing")
    return rows


def resample_to_common_time(reference_rows, other_rows):
    """Linearly interpolate the other trajectory at the reference timestamps."""
    times = [row["t"] for row in other_rows]
    resampled = []
    for reference in reference_rows:
        t = reference["t"]
        if t <= times[0]:
            resampled.append(other_rows[0])
            continue
        if t >= times[-1]:
            resampled.append(other_rows[-1])
            continue

        lo, hi = 0, len(times) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if times[mid] <= t:
                lo = mid
            else:
                hi = mid

        t0, t1 = times[lo], times[hi]
        fraction = (t - t0) / (t1 - t0)
        interpolated = {"t": t}
        for column in COMPARE_COLUMNS:
            start = other_rows[lo][column]
            if column == "yaw":
                delta = wrap_radians(other_rows[hi][column] - start)
                interpolated[column] = wrap_radians(start + fraction * delta)
            else:
                interpolated[column] = start + fraction * (other_rows[hi][column] - start)
        resampled.append(interpolated)
    return resampled


def wrap_radians(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def normalize_initial_pose(rows):
    """Express world position and heading relative to the trajectory's start."""
    initial_n = rows[0]["N"]
    initial_e = rows[0]["E"]
    initial_yaw = rows[0]["yaw"]
    cosine = math.cos(initial_yaw)
    sine = math.sin(initial_yaw)
    normalized = []
    for row in rows:
        delta_n = row["N"] - initial_n
        delta_e = row["E"] - initial_e
        item = dict(row)
        # Standard 2D rotation of (N, E) through -initial_yaw.
        item["N"] = cosine * delta_n + sine * delta_e
        item["E"] = -sine * delta_n + cosine * delta_e
        item["yaw"] = wrap_radians(row["yaw"] - initial_yaw)
        normalized.append(item)
    return normalized


def rmse(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def compare(reference_rows, other_rows):
    errors = {column: [] for column in COMPARE_COLUMNS}
    for reference, other in zip(reference_rows, other_rows):
        for column in COMPARE_COLUMNS:
            delta = reference[column] - other[column]
            errors[column].append(wrap_radians(delta) if column == "yaw" else delta)
    return {column: rmse(values) for column, values in errors.items()}, errors


def vector_rmse(errors, first, second):
    return rmse([math.hypot(a, b) for a, b in zip(errors[first], errors[second])])


def write_report(path, channel_rmse, position_rmse, velocity_rmse, count, duration):
    lines = [
        "# Trajectory comparison report\n",
        f"- Compared points: {count}",
        f"- Duration: {duration:.2f} s\n",
        "## Per-channel RMSE\n",
        "| Channel | RMSE |",
        "|---|---|",
    ]
    for column, value in channel_rmse.items():
        lines.append(f"| {column} | {value:.6f} |")
    lines.extend([
        f"\n## Horizontal position RMSE: {position_rmse:.6f} m",
        f"\n## Horizontal body-velocity RMSE: {velocity_rmse:.6f} m/s\n",
        "Yaw and r are reported in radians and radians per second, respectively.",
        "Interpret full-coupled6 residuals separately from matched-terms residuals, as "
        "described in VALIDATION_HARNESS_SPEC.md.",
    ])
    with open(path, "w") as stream:
        stream.write("\n".join(lines))


def maybe_plot(times, errors, output_path):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available -- skipping plot", file=sys.stderr)
        return

    figure, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    axes[0].plot(times, errors["N"], label="N error")
    axes[0].plot(times, errors["E"], label="E error")
    axes[0].set_ylabel("Position error (m)")
    axes[0].legend()
    axes[1].plot(times, errors["yaw"], label="yaw error")
    axes[1].set_ylabel("Heading error (rad)")
    axes[1].legend()
    axes[2].plot(times, errors["u"], label="u error")
    axes[2].plot(times, errors["v"], label="v error")
    axes[2].plot(times, errors["r"], label="r error")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Velocity error")
    axes[2].legend()
    figure.tight_layout()
    figure.savefig(output_path)
    print(f"Wrote plot to {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("reference_log", help="bcod-sim t,N,E,yaw,u,v,r CSV")
    parser.add_argument("other_log", help="Unity t,N,E,yaw,u,v,r CSV")
    parser.add_argument("--out", default="report.md")
    parser.add_argument("--plot")
    parser.add_argument(
        "--no-normalize", action="store_true",
        help="compare absolute N/E/yaw instead of normalizing each initial pose",
    )
    args = parser.parse_args()

    try:
        reference_rows = load_csv(args.reference_log)
        other_rows = load_csv(args.other_log)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    if not args.no_normalize:
        reference_rows = normalize_initial_pose(reference_rows)
        other_rows = normalize_initial_pose(other_rows)

    resampled = resample_to_common_time(reference_rows, other_rows)
    channel_rmse, errors = compare(reference_rows, resampled)
    position_rmse = vector_rmse(errors, "N", "E")
    velocity_rmse = vector_rmse(errors, "u", "v")
    duration = reference_rows[-1]["t"] - reference_rows[0]["t"]
    write_report(
        args.out, channel_rmse, position_rmse, velocity_rmse,
        len(reference_rows), duration,
    )
    print(f"Wrote report to {args.out}")

    if args.plot:
        maybe_plot([row["t"] for row in reference_rows], errors, args.plot)


if __name__ == "__main__":
    main()
