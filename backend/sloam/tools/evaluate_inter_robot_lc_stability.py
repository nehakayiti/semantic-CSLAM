#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path


def load_stability_eval_csv(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def load_trajectory_csv(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
      return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_to_rot(q):
    x, y, z, w = quat_normalize(q)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rot_transpose(R):
    return [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]],
    ]


def matmul3(A, B):
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(A[i][k] * B[k][j] for k in range(3))
    return out


def rotation_difference_deg(q1, q2):
    R1 = quat_to_rot(q1)
    R2 = quat_to_rot(q2)
    R_rel = matmul3(rot_transpose(R1), R2)
    trace_val = R_rel[0][0] + R_rel[1][1] + R_rel[2][2]
    cos_angle = (trace_val - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle_rad = math.acos(cos_angle)
    return angle_rad * 180.0 / 3.14159265358979323846


def translation_difference(p1, p2):
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 +
        (p1[1] - p2[1]) ** 2 +
        (p1[2] - p2[2]) ** 2
    )


def compute_final_trajectory_smoothness(trajectory_rows,
                                        trajectory_jump_translation_threshold_m,
                                        trajectory_jump_rotation_threshold_deg):
    if len(trajectory_rows) < 2:
        return {
            "trajectory_segment_count": 0,
            "trajectory_total_translation_m": 0.0,
            "trajectory_mean_translation_step_m": 0.0,
            "trajectory_max_translation_step_m": 0.0,
            "trajectory_mean_rotation_step_deg": 0.0,
            "trajectory_max_rotation_step_deg": 0.0,
            "trajectory_large_jump_count": 0,
        }

    translation_steps = []
    rotation_steps = []
    large_jump_count = 0

    for i in range(1, len(trajectory_rows)):
        prev = trajectory_rows[i - 1]
        curr = trajectory_rows[i]

        prev_p = (float(prev["x"]), float(prev["y"]), float(prev["z"]))
        curr_p = (float(curr["x"]), float(curr["y"]), float(curr["z"]))

        prev_q = (
            float(prev["qx"]),
            float(prev["qy"]),
            float(prev["qz"]),
            float(prev["qw"]),
        )
        curr_q = (
            float(curr["qx"]),
            float(curr["qy"]),
            float(curr["qz"]),
            float(curr["qw"]),
        )

        translation_step = translation_difference(prev_p, curr_p)
        rotation_step = rotation_difference_deg(prev_q, curr_q)

        translation_steps.append(translation_step)
        rotation_steps.append(rotation_step)

        if (translation_step > trajectory_jump_translation_threshold_m or
                rotation_step > trajectory_jump_rotation_threshold_deg):
            large_jump_count += 1

    return {
        "trajectory_segment_count": len(translation_steps),
        "trajectory_total_translation_m": sum(translation_steps),
        "trajectory_mean_translation_step_m": (
            sum(translation_steps) / len(translation_steps)
        ),
        "trajectory_max_translation_step_m": max(translation_steps),
        "trajectory_mean_rotation_step_deg": (
            sum(rotation_steps) / len(rotation_steps)
        ),
        "trajectory_max_rotation_step_deg": max(rotation_steps),
        "trajectory_large_jump_count": large_jump_count,
    }


def summarize_stability_run(stability_rows,
                            accepted_jump_translation_threshold_m,
                            accepted_jump_rotation_threshold_deg):
    accepted_closures_count = len(stability_rows)

    if accepted_closures_count == 0:
        return {
            "accepted_closures_count": 0,
            "repeat_count_mean": 0.0,
            "repeat_count_max": 0,
            "accepted_max_translation_jump_m": 0.0,
            "accepted_max_rotation_jump_deg": 0.0,
            "accepted_large_pose_jump_count": 0,
        }

    repeat_counts = [int(row["repeat_count"]) for row in stability_rows]
    combined_translation_jumps = [
        float(row["combined_max_translation_jump_m"]) for row in stability_rows
    ]
    combined_rotation_jumps = [
        float(row["combined_max_rotation_jump_deg"]) for row in stability_rows
    ]

    accepted_large_pose_jump_count = 0
    for t_jump, r_jump in zip(combined_translation_jumps,
                              combined_rotation_jumps):
        if (t_jump > accepted_jump_translation_threshold_m or
                r_jump > accepted_jump_rotation_threshold_deg):
            accepted_large_pose_jump_count += 1

    return {
        "accepted_closures_count": accepted_closures_count,
        "repeat_count_mean": sum(repeat_counts) / len(repeat_counts),
        "repeat_count_max": max(repeat_counts),
        "accepted_max_translation_jump_m": max(combined_translation_jumps),
        "accepted_max_rotation_jump_deg": max(combined_rotation_jumps),
        "accepted_large_pose_jump_count": accepted_large_pose_jump_count,
    }


def evaluate_one_run(stability_eval_csv,
                     trajectory_csv,
                     accepted_jump_translation_threshold_m,
                     accepted_jump_rotation_threshold_deg,
                     trajectory_jump_translation_threshold_m,
                     trajectory_jump_rotation_threshold_deg):
    stability_rows = load_stability_eval_csv(stability_eval_csv)
    trajectory_rows = load_trajectory_csv(trajectory_csv)

    stability_summary = summarize_stability_run(
        stability_rows,
        accepted_jump_translation_threshold_m,
        accepted_jump_rotation_threshold_deg,
    )

    trajectory_summary = compute_final_trajectory_smoothness(
        trajectory_rows,
        trajectory_jump_translation_threshold_m,
        trajectory_jump_rotation_threshold_deg,
    )

    merged = {}
    merged.update(stability_summary)
    merged.update(trajectory_summary)
    return merged


def print_summary(name, summary):
    print(f"\n=== {name} ===")
    print(f"Accepted closures count           : {summary['accepted_closures_count']}")
    print(f"Repeat count mean                : {summary['repeat_count_mean']:.6f}")
    print(f"Repeat count max                 : {summary['repeat_count_max']}")
    print(f"Accepted max translation jump m  : {summary['accepted_max_translation_jump_m']:.6f}")
    print(f"Accepted max rotation jump deg   : {summary['accepted_max_rotation_jump_deg']:.6f}")
    print(f"Accepted large pose jump count   : {summary['accepted_large_pose_jump_count']}")
    print(f"Trajectory segment count         : {summary['trajectory_segment_count']}")
    print(f"Trajectory total translation m   : {summary['trajectory_total_translation_m']:.6f}")
    print(f"Trajectory mean translation step : {summary['trajectory_mean_translation_step_m']:.6f}")
    print(f"Trajectory max translation step  : {summary['trajectory_max_translation_step_m']:.6f}")
    print(f"Trajectory mean rotation step    : {summary['trajectory_mean_rotation_step_deg']:.6f}")
    print(f"Trajectory max rotation step     : {summary['trajectory_max_rotation_step_deg']:.6f}")
    print(f"Trajectory large jump count      : {summary['trajectory_large_jump_count']}")


def write_summary_csv(output_csv_path, raw_summary, buffer_summary):
    fieldnames = [
        "run_name",
        "accepted_closures_count",
        "repeat_count_mean",
        "repeat_count_max",
        "accepted_max_translation_jump_m",
        "accepted_max_rotation_jump_deg",
        "accepted_large_pose_jump_count",
        "trajectory_segment_count",
        "trajectory_total_translation_m",
        "trajectory_mean_translation_step_m",
        "trajectory_max_translation_step_m",
        "trajectory_mean_rotation_step_deg",
        "trajectory_max_rotation_step_deg",
        "trajectory_large_jump_count",
    ]

    with open(output_csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        raw_row = {"run_name": "raw"}
        raw_row.update(raw_summary)
        writer.writerow(raw_row)

        buffer_row = {"run_name": "buffer"}
        buffer_row.update(buffer_summary)
        writer.writerow(buffer_row)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw_stability_eval_csv", required=True)
    parser.add_argument("--buffer_stability_eval_csv", required=True)
    parser.add_argument("--raw_trajectory_csv", required=True)
    parser.add_argument("--buffer_trajectory_csv", required=True)
    parser.add_argument("--accepted_jump_translation_threshold_m",
                        type=float, default=1.5)
    parser.add_argument("--accepted_jump_rotation_threshold_deg",
                        type=float, default=8.0)
    parser.add_argument("--trajectory_jump_translation_threshold_m",
                        type=float, default=2.0)
    parser.add_argument("--trajectory_jump_rotation_threshold_deg",
                        type=float, default=10.0)
    parser.add_argument("--output_summary_csv",
                        default="inter_robot_lc_stability_summary.csv")
    args = parser.parse_args()

    raw_summary = evaluate_one_run(
        args.raw_stability_eval_csv,
        args.raw_trajectory_csv,
        args.accepted_jump_translation_threshold_m,
        args.accepted_jump_rotation_threshold_deg,
        args.trajectory_jump_translation_threshold_m,
        args.trajectory_jump_rotation_threshold_deg,
    )

    buffer_summary = evaluate_one_run(
        args.buffer_stability_eval_csv,
        args.buffer_trajectory_csv,
        args.accepted_jump_translation_threshold_m,
        args.accepted_jump_rotation_threshold_deg,
        args.trajectory_jump_translation_threshold_m,
        args.trajectory_jump_rotation_threshold_deg,
    )

    print_summary("RAW / PASS-THROUGH", raw_summary)
    print_summary("BUFFERED", buffer_summary)

    print("\n=== DELTA (buffer - raw) ===")
    print(f"Accepted closures delta          : {buffer_summary['accepted_closures_count'] - raw_summary['accepted_closures_count']}")
    print(f"Repeat count mean delta         : {buffer_summary['repeat_count_mean'] - raw_summary['repeat_count_mean']:.6f}")
    print(f"Accepted large pose jump delta  : {buffer_summary['accepted_large_pose_jump_count'] - raw_summary['accepted_large_pose_jump_count']}")
    print(f"Trajectory large jump delta     : {buffer_summary['trajectory_large_jump_count'] - raw_summary['trajectory_large_jump_count']}")
    print(f"Trajectory max translation delta: {buffer_summary['trajectory_max_translation_step_m'] - raw_summary['trajectory_max_translation_step_m']:.6f}")
    print(f"Trajectory max rotation delta   : {buffer_summary['trajectory_max_rotation_step_deg'] - raw_summary['trajectory_max_rotation_step_deg']:.6f}")

    write_summary_csv(args.output_summary_csv, raw_summary, buffer_summary)
    print(f"\nWrote summary CSV to: {args.output_summary_csv}")


if __name__ == "__main__":
    main()