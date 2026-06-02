#!/usr/bin/env python3
"""CJK-aware ASCII diagram aligner.

Usage:
    python cjk_diagram.py

Reads a diagram from stdin (or a file), replaces box content with CJK-aware
padding so all │ borders align at the correct visual column.
"""

import sys
import unicodedata


def vw(s: str) -> int:
    """Visual width: CJK = 2, everything else = 1."""
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1 for ch in s)


def align_diagram(lines: list[str]) -> list[str]:
    """Given a diagram with CJK text, fix padding so all │ align."""
    # Find the top border line to determine box boundaries
    top_line = None
    for line in lines:
        if "┌" in line and "┐" in line:
            top_line = line
            break
    if not top_line:
        return lines

    # Extract box [left, right] visual column pairs from top border
    boxes = []
    for ci, ch in enumerate(top_line):
        if ch == "┌":
            vpos = vw(top_line[:ci])
            boxes.append([vpos, None])
        elif ch == "┐":
            vpos = vw(top_line[: ci + 1]) - 1
            for b in reversed(boxes):
                if b[1] is None:
                    b[1] = vpos
                    break

    # Inner visual widths
    inner_w = [r - l - 1 for l, r in boxes]

    result = []
    for line in lines:
        if "│" not in line:
            result.append(line)
            continue
        parts = line.split("│")
        if len(parts) < len(boxes) * 2 + 1:
            result.append(line)
            continue

        new_parts = [parts[0]]
        for bi in range(len(boxes)):
            box_idx = 1 + bi * 2
            content = parts[box_idx]
            # Split leading spaces, text, trailing spaces
            lead = 0
            while lead < len(content) and content[lead] == " ":
                lead += 1
            trail = len(content)
            while trail > lead and content[trail - 1] == " ":
                trail -= 1
            leading = content[:lead]
            text = content[lead:trail]
            needed = inner_w[bi] - vw(leading) - vw(text)
            if needed < 0:
                # Text too wide — truncation needed (unlikely with CJK)
                new_parts.append(content)
            else:
                new_parts.append(leading + text + " " * needed)
            if bi < len(boxes) - 1:
                gap_idx = box_idx + 1
                new_parts.append(parts[gap_idx])
        # Remaining parts after last box
        new_parts.extend(parts[1 + len(boxes) * 2 :])
        result.append("│".join(new_parts))

    return result


def main():
    if len(sys.argv) > 1:
        with open(sys.argv[1], encoding="utf-8") as f:
            text = f.read()
    else:
        text = sys.stdin.read()

    lines = text.split("\n")
    # Find diagram blocks (fenced or indented)
    fixed = align_diagram(lines)
    print("\n".join(fixed))


if __name__ == "__main__":
    main()
