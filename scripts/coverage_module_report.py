#!/usr/bin/env python3

"""
Compute per-module line/branch coverage from a gcovr --json-pretty report
(see the "Code Coverage Status" section of agents.md for the full
build+test+gcovr recipe this consumes).

Deduplicates files that appear twice under modules/<pkg>/... and
install/<pkg>/... (colcon symlink-install artifact) by merging line hit
counts (max) before aggregating, as gcovr treats them as unrelated files
otherwise and inflates/deflates totals.

Usage: coverage_module_report.py coverage.json <module_name> [<module_name> ...]
Example: coverage_module_report.py coverage.json mrpt_math mrpt_poses
"""
import json
import sys
from collections import defaultdict


def normalize_path(path: str) -> str:
    for prefix in ("modules/", "install/"):
        if path.startswith(prefix):
            return path[len(prefix):]
    return path


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    json_path = sys.argv[1]
    modules = sys.argv[2:]

    with open(json_path) as f:
        data = json.load(f)

    # norm_path -> {line_no: max_hit_count}
    merged_lines = defaultdict(dict)
    # norm_path -> {(line_no, branch_index): covered_bool}, OR-merged
    merged_branches = defaultdict(dict)

    for file_entry in data["files"]:
        norm = normalize_path(file_entry["file"])
        for line in file_entry["lines"]:
            ln = line["line_number"]
            count = line["count"]
            merged_lines[norm][ln] = max(merged_lines[norm].get(ln, 0), count)
            for i, br in enumerate(line.get("branches", [])):
                key = (ln, i)
                covered = br["count"] > 0
                merged_branches[norm][key] = merged_branches[norm].get(key, False) or covered

    def totals_for(prefix):
        lines_total = 0
        lines_covered = 0
        branches_total = 0
        branches_covered = 0
        per_file = {}
        for norm, lines in merged_lines.items():
            if not norm.startswith(prefix):
                continue
            ft = len(lines)
            fc = sum(1 for v in lines.values() if v > 0)
            lines_total += ft
            lines_covered += fc
            bt = len(merged_branches[norm])
            bc = sum(1 for v in merged_branches[norm].values() if v)
            branches_total += bt
            branches_covered += bc
            per_file[norm] = (fc, ft, bc, bt)
        return lines_covered, lines_total, branches_covered, branches_total, per_file

    for module in modules:
        lc, lt, bc, bt, per_file = totals_for(f"{module}/")
        print(f"=== {module} ===")
        if lt:
            print(f"lines:    {lc}/{lt} = {100.0 * lc / lt:.1f}%")
        else:
            print("no lines found (check the module name / that it was built+tested)")
        if bt:
            print(f"branches: {bc}/{bt} = {100.0 * bc / bt:.1f}%")
        print()
        print(f"{'file':70s} {'line%':>8s} {'branch%':>8s}")
        for norm, (fc, ft, bc2, bt2) in sorted(
            per_file.items(), key=lambda kv: -(kv[1][1] - kv[1][0])
        ):
            lp = 100.0 * fc / ft if ft else 0.0
            bp = 100.0 * bc2 / bt2 if bt2 else 0.0
            print(f"{norm:70s} {lp:7.1f}% {bp:7.1f}%")
        print()


if __name__ == "__main__":
    main()
