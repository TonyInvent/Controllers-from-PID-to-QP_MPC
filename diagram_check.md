# Diagram Check Skill

When drawing or editing ASCII box-drawing diagrams (using `┌┐└┘├┤│─┬┴┼`) in markdown, run the validator before committing:

```bash
python diagram_check.py youla_parameterization.md
```

Or check all markdown files:

```bash
python diagram_check.py *.md
```

## Rules

- Box corners (`┌┐└┘`) must align vertically with the corresponding `│` on the adjacent line.
- A corner at column N should have a vertical bar within ±2 columns on the adjacent line.
- Keep box-drawing characters in monospace fenced code blocks (triple backticks).

## Validation script

`diagram_check.py` inspects all fenced code blocks in the given files, locates box-drawing characters, and checks that every corner has a nearby vertical bar on its adjacent line. Exit code 1 if misalignments found.

## Common pitfalls

- Top border `┌─────────┐` one space to the right of the `│ Q(s) │` content line.
- A `└` corner aligning with the wrong box's vertical bar (happens when two boxes share a line).
- Using non-monospace rendering (HTML, variable-width fonts) — the checker validates source, not rendered output.
