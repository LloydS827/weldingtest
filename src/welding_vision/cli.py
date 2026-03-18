from __future__ import annotations

import argparse
import json
import sys

from .config import PipelineConfig
from .io import load_points, save_json
from .pipeline import PointCloudPipeline


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Point-cloud pipeline for weld seam detection.")
    parser.add_argument("--input", required=True, help="Path to the raw point cloud JSON/CSV.")
    parser.add_argument("--background", help="Optional empty-workcell background point cloud JSON/CSV.")
    parser.add_argument("--config", required=True, help="Path to the pipeline JSON config.")
    parser.add_argument("--output", help="Optional path to write the JSON result.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed for RANSAC.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    config = PipelineConfig.from_json_file(args.config)
    raw_points = load_points(args.input)
    background_points = load_points(args.background) if args.background else None

    pipeline = PointCloudPipeline(config=config, seed=args.seed)
    result = pipeline.run(raw_points=raw_points, background_points=background_points)
    payload = result.to_dict()

    if args.output:
        save_json(args.output, payload)
    else:
        json.dump(payload, sys.stdout, indent=2)
        sys.stdout.write("\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

