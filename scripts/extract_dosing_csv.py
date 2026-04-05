#!/usr/bin/env python3
import argparse
import csv
import re
from pathlib import Path

CSV_HEADER = [
    "tick_ms",
    "state",
    "dispensed_ug",
    "remaining_ug",
    "tick_delta_ug",
    "ema_flow_ug",
    "desired_ug",
    "nudge_dir",
    "retries",
    "flap1_adc",
    "flap2_adc",
]

CSV_PATTERN = re.compile(r"(?:^|[^0-9])((?:\d+,){10}\d+)(?:[^0-9]|$)")


def extract_csv_line(line: str) -> str | None:
    stripped = line.strip()
    if not stripped:
        return None
    if stripped.startswith("tick_ms,state,"):
        return stripped
    if stripped[0].isdigit():
        parts = stripped.split(",")
        if len(parts) == len(CSV_HEADER) and all(part.strip().lstrip("-+").isdigit() for part in parts):
            return stripped
    match = CSV_PATTERN.search(stripped)
    if match:
        candidate = match.group(1)
        parts = candidate.split(",")
        if len(parts) == len(CSV_HEADER) and all(part.strip().lstrip("-+").isdigit() for part in parts):
            return candidate
    return None


def parse_file(src_path: Path) -> tuple[list[str], dict[str, int]]:
    rows: list[str] = []
    stats = {
        "lines_total": 0,
        "rows_saved": 0,
        "header_found": 0,
        "agitator_events": 0,
        "spawn_sm_events": 0,
        "dose_main_no_flow_events": 0,
    }

    with src_path.open("r", encoding="utf-8", errors="replace") as fh:
        for raw_line in fh:
            stats["lines_total"] += 1
            line = raw_line.rstrip("\n")
            if "Agitator:" in line:
                stats["agitator_events"] += 1
            if "Spawn SM:" in line:
                stats["spawn_sm_events"] += 1
            if "Spawn DOSE_MAIN: no-flow" in line:
                stats["dose_main_no_flow_events"] += 1

            extracted = extract_csv_line(line)
            if extracted:
                if extracted.startswith("tick_ms,state,"):
                    stats["header_found"] += 1
                    if not rows:
                        rows.append(extracted)
                else:
                    rows.append(extracted)
                    stats["rows_saved"] += 1

    return rows, stats


def write_csv(rows: list[str], dst_path: Path) -> None:
    dst_path.parent.mkdir(parents=True, exist_ok=True)
    with dst_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        if rows:
            header = rows[0].split(",")
            if header != CSV_HEADER:
                raise ValueError(f"Unexpected CSV header: {header}")
            writer.writerow(header)
            for row in rows[1:]:
                writer.writerow([c.strip() for c in row.split(",")])


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract dosing CSV rows from a noisy log file.")
    parser.add_argument("input", type=Path, help="Input log file path")
    parser.add_argument("output", type=Path, help="Output CSV file path")
    args = parser.parse_args()

    rows, stats = parse_file(args.input)
    if not rows:
        raise SystemExit("No valid CSV data found in input file.")
    write_csv(rows, args.output)

    print(f"Input: {args.input}")
    print(f"Output: {args.output}")
    print(f"Lines scanned: {stats['lines_total']}")
    print(f"CSV rows saved: {stats['rows_saved']}")
    print(f"Header found: {stats['header_found']}")
    print(f"Agitator events: {stats['agitator_events']}")
    print(f"Spawn SM events: {stats['spawn_sm_events']}")
    print(f"Spawn DOSE_MAIN no-flow events: {stats['dose_main_no_flow_events']}")


if __name__ == "__main__":
    main()
