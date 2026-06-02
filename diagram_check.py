#!/usr/bin/env python3
"""Validate ASCII box-drawing alignment in markdown code blocks.

Usage:
    python diagram_check.py [file ...]
    python diagram_check.py *.md

Checks:
  - Box top/bottom borders align vertically with content side-bars
  - Vertical-flow characters (│, ┼, etc.) are consistent across lines
"""

import re
import sys
from pathlib import Path

BOX_CHARS = set("┌┐└┘├┤│┼┬┴")
VERT_CHARS = set("│┼├┤┬┴")


def check_file(filepath: Path) -> list[str]:
    issues = []
    text = filepath.read_text(encoding="utf-8")
    blocks = re.findall(r"```\n(.*?)```", text, re.DOTALL)

    for bi, block in enumerate(blocks):
        if not BOX_CHARS & set(block):
            continue

        lines = block.split("\n")
        # Record positions of all box-drawing chars per line
        verticals = []
        for line in lines:
            v = {}
            for ci, ch in enumerate(line):
                if ch in BOX_CHARS:
                    v[ci] = ch
            verticals.append(v)

        for li in range(len(lines)):
            v_cur = verticals[li]

            for corner, adj in [("┌", 1), ("┐", 1), ("└", -1), ("┘", -1)]:
                adj_li = li + adj
                if not (0 <= adj_li < len(verticals)):
                    continue
                v_adj = verticals[adj_li]
                if not v_adj:
                    continue

                for col, ch in v_cur.items():
                    if ch != corner:
                        continue
                    adj_cols = sorted(v_adj.keys())
                    if not adj_cols:
                        continue
                    closest = min(adj_cols, key=lambda c: abs(c - col))
                    if abs(closest - col) > 2:
                        issues.append(
                            f"{filepath.name}: block {bi}, line {li}: "
                            f"'{corner}' at col {col} has no nearby vertical "
                            f"on line {adj_li} (closest at col {closest}, gap {abs(closest-col)})"
                        )

        # Warn about Unicode chars with ambiguous monospace width
        AMBIGUOUS = set("▶►→←↑↓↔↕➔➤")
        for li, line in enumerate(lines):
            for ci, ch in enumerate(line):
                if ch in AMBIGUOUS:
                    issues.append(
                        f"{filepath.name}: block {bi}, line {li}, col {ci}: "
                        f"ambiguous-width char U+{ord(ch):04X} '{ch}' — "
                        f"use ASCII instead (> -> <-) for portable monospace alignment"
                    )


def main():
    if len(sys.argv) < 2:
        files = sorted(Path(".").glob("*.md"))
    else:
        files = [Path(f) for f in sys.argv[1:]]

    total_issues = 0
    for fp in files:
        if not fp.exists():
            print(f"SKIP: {fp} not found")
            continue
        issues = check_file(fp)
        if issues:
            for i in issues:
                print(f"  ISSUE: {i}")
            total_issues += len(issues)

    if total_issues == 0:
        print("All box-drawing diagrams appear aligned.")
    else:
        print(f"\n{total_issues} issue(s) found.")
        sys.exit(1)


if __name__ == "__main__":
    main()
