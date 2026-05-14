#!/usr/bin/env python3
"""Generate self-contained HTML pages from markdown podcast scripts."""

import html
import sys
from pathlib import Path

TEMPLATE = '''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title}</title>
<script src="https://cdn.jsdelivr.net/npm/marked/marked.min.js"></script>
<style>
:root {{
  --bg: #0d1117;
  --surface: #161b22;
  --border: #30363d;
  --text: #e6edf3;
  --text-secondary: #8b949e;
  --accent: #58a6ff;
  --font: 'Segoe UI', system-ui, -apple-system, sans-serif;
  --mono: 'Cascadia Code', 'JetBrains Mono', 'Fira Code', 'Consolas', monospace;
  --orange: #f0883e;
}}

* {{ box-sizing: border-box; margin: 0; padding: 0; }}

body {{
  background: var(--bg);
  color: var(--text);
  font-family: var(--font);
  line-height: 1.7;
  min-height: 100vh;
}}

.back-bar {{
  padding: 16px 24px;
  border-bottom: 1px solid var(--border);
  background: var(--surface);
  position: sticky;
  top: 0;
  z-index: 10;
}}
.back-bar a {{
  color: var(--accent);
  text-decoration: none;
  font-size: 0.85rem;
  font-weight: 600;
}}
.back-bar a:hover {{ text-decoration: underline; }}

#content {{
  max-width: 820px;
  margin: 0 auto;
  padding: 32px 24px 80px;
}}

#content h1 {{
  font-size: 1.8rem;
  font-weight: 700;
  margin: 24px 0 12px;
  padding-bottom: 8px;
  border-bottom: 1px solid var(--border);
  letter-spacing: -0.5px;
}}
#content h2 {{
  font-size: 1.35rem;
  font-weight: 600;
  margin: 28px 0 10px;
  color: #f0f6fc;
}}
#content h3 {{
  font-size: 1.1rem;
  font-weight: 600;
  margin: 22px 0 8px;
  color: #c9d1d9;
}}
#content p {{
  margin: 8px 0 14px;
  color: var(--text);
  font-size: 0.92rem;
}}
#content ul, #content ol {{
  margin: 8px 0 14px;
  padding-left: 24px;
  color: var(--text);
  font-size: 0.92rem;
}}
#content li {{ margin: 4px 0; }}
#content strong {{ color: #f0f6fc; }}
#content em {{ color: #d2d9e0; }}
#content code {{
  font-family: var(--mono);
  font-size: 0.82rem;
  background: var(--surface);
  padding: 2px 6px;
  border-radius: 4px;
  border: 1px solid var(--border);
  color: var(--orange);
}}
#content pre {{
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 16px;
  overflow-x: auto;
  margin: 12px 0 18px;
}}
#content pre code {{
  background: none;
  border: none;
  padding: 0;
  color: var(--text);
  font-size: 0.78rem;
}}
#content blockquote {{
  border-left: 3px solid var(--accent);
  padding: 8px 16px;
  margin: 12px 0 18px;
  background: rgba(88,166,255,0.06);
  border-radius: 0 6px 6px 0;
  color: var(--text-secondary);
}}
#content blockquote p {{ color: inherit; }}
#content a {{ color: var(--accent); text-decoration: none; }}
#content a:hover {{ text-decoration: underline; }}
#content hr {{
  border: none;
  border-top: 1px solid var(--border);
  margin: 24px 0;
}}
#content table {{
  border-collapse: collapse;
  width: 100%;
  margin: 12px 0 18px;
  font-size: 0.85rem;
}}
#content th, #content td {{
  border: 1px solid var(--border);
  padding: 8px 12px;
  text-align: left;
}}
#content th {{ background: var(--surface); font-weight: 600; }}
#content img {{ max-width: 100%; border-radius: 6px; }}

.loading {{
  text-align: center;
  padding: 60px 0;
  color: var(--text-secondary);
}}
</style>
</head>
<body>

<div class="back-bar">
  <a href="welcome.html">&larr; Back to simulators</a>
</div>

<div id="content">
  <div class="loading">Loading&hellip;</div>
</div>

<script type="text/markdown" id="md-source">
{content}
</script>

<script>
const md = document.getElementById('md-source').textContent;
document.getElementById('content').innerHTML = marked.parse(md);
</script>

</body>
</html>'''


def main():
    script_dir = Path(__file__).resolve().parent
    pairs = [
        (
            script_dir / "Servo Motor controllers from PID, LQR to QP-MPC.md",
            script_dir / "Servo Motor controllers from PID, LQR to QP-MPC.html",
        ),
        (
            script_dir / "Servo Motor controllers from PID, LQR to QP-MPC-zh.md",
            script_dir / "Servo Motor controllers from PID, LQR to QP-MPC-zh.html",
        ),
    ]

    for md_path, html_path in pairs:
        if not md_path.exists():
            print(f"SKIP: {md_path} not found")
            continue

        md_text = md_path.read_text(encoding="utf-8")

        # Escape </script> and <!-- in the markdown to keep the embedding safe
        safe_content = md_text.replace("</script>", "<\\/script>").replace("<!--", "<\\!--")

        # Derive title from the first h1 line
        title = md_path.stem

        html_text = TEMPLATE.format(title=html.escape(title), content=safe_content)

        html_path.write_text(html_text, encoding="utf-8")
        print(f"Wrote: {html_path.name}")


if __name__ == "__main__":
    main()
