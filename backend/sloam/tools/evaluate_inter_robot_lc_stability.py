#!/usr/bin/env python3

import argparse
import csv


def load_stability_eval_csv(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


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


def print_summary(name, summary):
    print(f"\n=== {name} ===")
    print(f"Accepted closures count          : {summary['accepted_closures_count']}")
    print(f"Repeat count mean               : {summary['repeat_count_mean']:.6f}")
    print(f"Repeat count max                : {summary['repeat_count_max']}")
    print(f"Accepted max translation jump m : {summary['accepted_max_translation_jump_m']:.6f}")
    print(f"Accepted max rotation jump deg  : {summary['accepted_max_rotation_jump_deg']:.6f}")
    print(f"Accepted large pose jump count  : {summary['accepted_large_pose_jump_count']}")


def write_summary_csv(output_csv_path, raw_summary, buffer_summary):
    fieldnames = [
        "run_name",
        "accepted_closures_count",
        "repeat_count_mean",
        "repeat_count_max",
        "accepted_max_translation_jump_m",
        "accepted_max_rotation_jump_deg",
        "accepted_large_pose_jump_count",
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
    parser.add_argument("--accepted_jump_translation_threshold_m",
                        type=float, default=1.5)
    parser.add_argument("--accepted_jump_rotation_threshold_deg",
                        type=float, default=8.0)
    parser.add_argument("--output_summary_csv",
                        default="inter_robot_lc_stability_only_summary.csv")
    args = parser.parse_args()

    raw_rows = load_stability_eval_csv(args.raw_stability_eval_csv)
    buffer_rows = load_stability_eval_csv(args.buffer_stability_eval_csv)

    raw_summary = summarize_stability_run(
        raw_rows,
        args.accepted_jump_translation_threshold_m,
        args.accepted_jump_rotation_threshold_deg,
    )

    buffer_summary = summarize_stability_run(
        buffer_rows,
        args.accepted_jump_translation_threshold_m,
        args.accepted_jump_rotation_threshold_deg,
    )

    print_summary("RAW / PASS-THROUGH", raw_summary)
    print_summary("BUFFERED", buffer_summary)

    print("\n=== DELTA (buffer - raw) ===")
    print(f"Accepted closures delta         : {buffer_summary['accepted_closures_count'] - raw_summary['accepted_closures_count']}")
    print(f"Repeat count mean delta        : {buffer_summary['repeat_count_mean'] - raw_summary['repeat_count_mean']:.6f}")
    print(f"Accepted large pose jump delta : {buffer_summary['accepted_large_pose_jump_count'] - raw_summary['accepted_large_pose_jump_count']}")
    print(f"Accepted max translation delta : {buffer_summary['accepted_max_translation_jump_m'] - raw_summary['accepted_max_translation_jump_m']:.6f}")
    print(f"Accepted max rotation delta    : {buffer_summary['accepted_max_rotation_jump_deg'] - raw_summary['accepted_max_rotation_jump_deg']:.6f}")

    write_summary_csv(args.output_summary_csv, raw_summary, buffer_summary)
    print(f"\nWrote summary CSV to: {args.output_summary_csv}")


if __name__ == "__main__":
    main()
