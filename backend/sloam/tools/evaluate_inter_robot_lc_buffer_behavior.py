#!/usr/bin/env python3

import argparse
import csv


def load_csv(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def summarize_candidate_run(candidate_rows):
    candidate_count = len(candidate_rows)
    accepted_rows = [r for r in candidate_rows if r["status"] == "accepted"]
    rejected_rows = [r for r in candidate_rows if r["status"] == "rejected"]

    accepted_count = len(accepted_rows)
    rejected_count = len(rejected_rows)
    acceptance_rate = (accepted_count / candidate_count) if candidate_count > 0 else 0.0

    repeat_counts_all = [int(r["repeat_count"]) for r in candidate_rows] if candidate_rows else []
    repeat_counts_accepted = [int(r["repeat_count"]) for r in accepted_rows] if accepted_rows else []

    summary = {
        "candidate_count": candidate_count,
        "accepted_count": accepted_count,
        "rejected_count": rejected_count,
        "acceptance_rate": acceptance_rate,
        "repeat_count_mean_all": (
            sum(repeat_counts_all) / len(repeat_counts_all)
            if repeat_counts_all else 0.0
        ),
        "repeat_count_max_all": max(repeat_counts_all) if repeat_counts_all else 0,
        "repeat_count_mean_accepted": (
            sum(repeat_counts_accepted) / len(repeat_counts_accepted)
            if repeat_counts_accepted else 0.0
        ),
        "repeat_count_max_accepted": (
            max(repeat_counts_accepted) if repeat_counts_accepted else 0
        ),
    }
    return summary


def summarize_stability_run(stability_rows,
                            accepted_jump_translation_threshold_m,
                            accepted_jump_rotation_threshold_deg):
    accepted_solve_count = len(stability_rows)

    if accepted_solve_count == 0:
        return {
            "accepted_solve_count": 0,
            "accepted_max_translation_jump_m": 0.0,
            "accepted_max_rotation_jump_deg": 0.0,
            "accepted_large_pose_jump_count": 0,
        }

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
        "accepted_solve_count": accepted_solve_count,
        "accepted_max_translation_jump_m": max(combined_translation_jumps),
        "accepted_max_rotation_jump_deg": max(combined_rotation_jumps),
        "accepted_large_pose_jump_count": accepted_large_pose_jump_count,
    }


def merge_summaries(candidate_summary, stability_summary):
    merged = {}
    merged.update(candidate_summary)
    merged.update(stability_summary)
    return merged


def print_summary(name, summary):
    print(f"\n=== {name} ===")
    print(f"Candidate count                 : {summary['candidate_count']}")
    print(f"Accepted count                  : {summary['accepted_count']}")
    print(f"Rejected count                  : {summary['rejected_count']}")
    print(f"Acceptance rate                 : {summary['acceptance_rate']:.6f}")
    print(f"Repeat count mean all           : {summary['repeat_count_mean_all']:.6f}")
    print(f"Repeat count max all            : {summary['repeat_count_max_all']}")
    print(f"Repeat count mean accepted      : {summary['repeat_count_mean_accepted']:.6f}")
    print(f"Repeat count max accepted       : {summary['repeat_count_max_accepted']}")
    print(f"Accepted solve count            : {summary['accepted_solve_count']}")
    print(f"Accepted max translation jump m : {summary['accepted_max_translation_jump_m']:.6f}")
    print(f"Accepted max rotation jump deg  : {summary['accepted_max_rotation_jump_deg']:.6f}")
    print(f"Accepted large pose jump count  : {summary['accepted_large_pose_jump_count']}")


def write_summary_csv(output_csv_path, raw_summary, buffer_summary):
    fieldnames = [
        "run_name",
        "candidate_count",
        "accepted_count",
        "rejected_count",
        "acceptance_rate",
        "repeat_count_mean_all",
        "repeat_count_max_all",
        "repeat_count_mean_accepted",
        "repeat_count_max_accepted",
        "accepted_solve_count",
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


def evaluate_one_run(candidate_csv,
                     stability_csv,
                     accepted_jump_translation_threshold_m,
                     accepted_jump_rotation_threshold_deg):
    candidate_rows = load_csv(candidate_csv)
    stability_rows = load_csv(stability_csv)

    candidate_summary = summarize_candidate_run(candidate_rows)
    stability_summary = summarize_stability_run(
        stability_rows,
        accepted_jump_translation_threshold_m,
        accepted_jump_rotation_threshold_deg,
    )
    return merge_summaries(candidate_summary, stability_summary)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw_candidate_eval_csv", required=True)
    parser.add_argument("--buffer_candidate_eval_csv", required=True)
    parser.add_argument("--raw_stability_eval_csv", required=True)
    parser.add_argument("--buffer_stability_eval_csv", required=True)
    parser.add_argument("--accepted_jump_translation_threshold_m",
                        type=float, default=1.5)
    parser.add_argument("--accepted_jump_rotation_threshold_deg",
                        type=float, default=8.0)
    parser.add_argument("--output_summary_csv",
                        default="inter_robot_lc_buffer_behavior_summary.csv")
    args = parser.parse_args()

    raw_summary = evaluate_one_run(
        args.raw_candidate_eval_csv,
        args.raw_stability_eval_csv,
        args.accepted_jump_translation_threshold_m,
        args.accepted_jump_rotation_threshold_deg,
    )

    buffer_summary = evaluate_one_run(
        args.buffer_candidate_eval_csv,
        args.buffer_stability_eval_csv,
        args.accepted_jump_translation_threshold_m,
        args.accepted_jump_rotation_threshold_deg,
    )

    print_summary("RAW / PASS-THROUGH", raw_summary)
    print_summary("BUFFERED", buffer_summary)

    print("\n=== DELTA (buffer - raw) ===")
    print(f"Candidate count delta           : {buffer_summary['candidate_count'] - raw_summary['candidate_count']}")
    print(f"Accepted count delta            : {buffer_summary['accepted_count'] - raw_summary['accepted_count']}")
    print(f"Rejected count delta            : {buffer_summary['rejected_count'] - raw_summary['rejected_count']}")
    print(f"Acceptance rate delta           : {buffer_summary['acceptance_rate'] - raw_summary['acceptance_rate']:.6f}")
    print(f"Repeat count mean all delta     : {buffer_summary['repeat_count_mean_all'] - raw_summary['repeat_count_mean_all']:.6f}")
    print(f"Repeat count mean accepted delta: {buffer_summary['repeat_count_mean_accepted'] - raw_summary['repeat_count_mean_accepted']:.6f}")
    print(f"Accepted solve count delta      : {buffer_summary['accepted_solve_count'] - raw_summary['accepted_solve_count']}")
    print(f"Accepted large pose jump delta  : {buffer_summary['accepted_large_pose_jump_count'] - raw_summary['accepted_large_pose_jump_count']}")

    write_summary_csv(args.output_summary_csv, raw_summary, buffer_summary)
    print(f"\nWrote summary CSV to: {args.output_summary_csv}")


if __name__ == "__main__":
    main()