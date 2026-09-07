from __future__ import annotations

import argparse
import importlib

BUILDERS = {
    "0": "fig0_architecture", "1": "fig1_vessels", "2": "fig2_geography",
    "3": "fig3_sensing", "4": "fig4_determinism", "5": "fig5_validation",
    "6": "table6_comparison",
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Render reproducible paper figures from committed artifacts")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--figure", choices=BUILDERS)
    group.add_argument("--all", action="store_true")
    args = parser.parse_args()
    selected = list(BUILDERS) if args.all else [args.figure]
    for number in selected:
        module = importlib.import_module(f"validation.figures.{BUILDERS[number]}")
        print(module.build())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
