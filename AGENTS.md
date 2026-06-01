# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project overview

Interactive, self-contained HTML simulators tracing the evolution of feedback control — from classical PID through LQR to constrained QP-MPC — all applied to the same brushed DC motor servo. Each simulator is a single HTML file with inline CSS and JS; no build step, no server, no framework. Open any `.html` file directly in a browser.

## Running things

```bash
# Interactive simulators — open any in a browser:
open pid_explorer.html          # PID on 2nd-order plant
open servo_motor_pid.html       # PID on DC motor physics
open lqr_explorer.html          # LQR/LQI optimal control
open servo_qp_mpc.html          # QP-MPC constrained control
open zero_effect_explorer.html  # Zero effects on step response

# Rendered podcast pages:
open "Servo Motor controllers from PID, LQR to QP-MPC.html"

# Run LQR integration tests (requires numeric.js from npm):
npm install
node test_lqr.js

# Generate zero-effect figures (requires Python control + matplotlib):
pip install control matplotlib
python3 zero_effect_demo.py

# Regenerate podcast HTML pages from markdown sources:
python3 generate_podcast_html.py

# Validate ASCII box-drawing diagrams in markdown:
python diagram_check.py *.md
```

## Architecture

Each HTML simulator is a self-contained file with three layers: inline CSS (dark theme, GitHub-inspired), a Plotly-based chart, and a <script> block containing the full simulation engine. There are no shared libraries, no imports between files — each `.html` is a standalone application.

**Shared pattern across simulators:**
- Canvas/Plotly charts for real-time step response and pole-zero maps
- Slider inputs mapped to physical parameters or control gains
- RK4 integration for time-domain simulation (with stiff-ODE detection for very small L)
- Preset configurations illustrating key scenarios
- Dynamic insight panel explaining behavior at the current operating point

**Key algorithmic components:**
- `lqr_explorer.html` — 3rd-order motor model + integral augmentation → 4th-order LQI; CARE solved via Hamiltonian method with `numeric.eig()` on an 8×8 matrix; reference feedforward via `k_θ·r` term (analogous to PID's `Kp·r`)
- `servo_qp_mpc.html` — discretized motor model + coordinate-descent box-constrained QP solver; parallel unconstrained LQR simulation for direct constraint comparison; receding-horizon optimization
- `servo_motor_pid.html` — 3rd-order motor physics; automatically switches between full 4th-order quartic (Ferrari's method) and 3rd-order cubic for PD-only mode based on Ki; anti-windup on voltage saturation

**Podcast rendering:** `generate_podcast_html.py` embeds markdown content into a styled HTML template using `marked.js` (CDN) for client-side rendering. Input: `*.md` → output: `*.html`.

**Testing:** `test_lqr.js` extracts the inline JS from `lqr_explorer.html`, evaluates it in Node.js with mocked DOM (no browser needed), and runs 16 integration tests against the CARE solver, simulation engine, and fallback gain logic using real `numeric.js` from npm.
