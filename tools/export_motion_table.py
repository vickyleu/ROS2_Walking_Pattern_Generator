#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser(description="Export motion_table.json from joint_states_record dat file.")
    parser.add_argument("--input", required=True, help="Path to joint_states.dat")
    parser.add_argument("--output", required=True, help="Path to motion_table.json")
    parser.add_argument("--action", required=True, help="Action name, e.g. stand")
    parser.add_argument("--dt-ms", type=int, default=10, help="Sample period in ms (default: 10)")
    parser.add_argument("--version", type=int, default=1, help="Motion table version (default: 1)")
    parser.add_argument(
        "--joint-source",
        choices=["legL", "legR", "both", "custom"],
        default="legL",
        help="Select joint positions source (default: legL)",
    )
    parser.add_argument(
        "--indices",
        default="",
        help="Comma-separated 0-based indices into position list when joint-source=custom",
    )
    return parser.parse_args()

def iter_points(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            yield line

def select_positions(tokens, joint_source, indices):
    # Format: step_count | legL_pos(6) | legR_pos(6) | legL_vel(6) | legR_vel(6)
    # Indices below are 0-based into the position block (legL+legR => 12 items).
    positions = tokens[1:13]
    if joint_source == "legL":
        return positions[0:6]
    if joint_source == "legR":
        return positions[6:12]
    if joint_source == "both":
        return positions
    if joint_source == "custom":
        return [positions[idx] for idx in indices]
    raise ValueError(f"unknown joint_source: {joint_source}")

def main():
    args = parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        raise SystemExit(f"input not found: {input_path}")

    indices = []
    if args.joint_source == "custom":
        if not args.indices:
            raise SystemExit("--indices is required when --joint-source=custom")
        indices = [int(value) for value in args.indices.split(",") if value.strip()]
        if not indices:
            raise SystemExit("--indices is empty after parsing")

    points = []
    for line in iter_points(input_path):
        parts = line.split()
        if len(parts) < 13:
            continue
        try:
            tokens = [float(item) for item in parts]
        except ValueError:
            continue
        positions = select_positions(tokens, args.joint_source, indices)
        points.append(positions)

    if not points:
        raise SystemExit("no points parsed from input")

    motion = {
        "joints": len(points[0]),
        "dt_ms": args.dt_ms,
        "points": points,
    }
    payload = {
        "version": args.version,
        "motions": {
            args.action: motion,
        },
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")

if __name__ == "__main__":
    main()
