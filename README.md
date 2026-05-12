# Zero Effect Explorer

Interactive tools to understand how **zeros** change the step response of a second-order system — a fundamental concept in control theory that's notoriously hard to build intuition for.

## What's here

| File | What it is | Open with |
|------|-----------|-----------|
| `zero_effect_explorer.html` | Self-contained interactive web page | Any browser — no server, no install |
| `zero_effect_demo.py` | Python script that generates the 3 figures | `python3 zero_effect_demo.py` (needs `control`, `matplotlib`) |
| `zero_effect_video_script.md` | English video script (7 scenes, ~12-15 min) | Any text editor |
| `zero_effect_video_script_cn.md` | Chinese video script for Bilibili (7 scenes) | Any text editor |

## Try it now

```bash
# Interactive web page — just open it:
open zero_effect_explorer.html

# Or run the Python demo:
pip install control matplotlib
python3 zero_effect_demo.py
```

## What you'll learn

**Left-half-plane (LHP) zeros** — a speed-versus-overshoot trade-off:
- A zero far from the origin is practically invisible
- As the zero moves closer to the origin, the derivative "kick" grows, amplifying overshoot while reducing rise time
- This is what PID derivative action does — you're placing a zero, and *where* you place it matters

**Right-half-plane (RHP) zeros** — a hard physical limit:
- The system initially moves the **wrong** way before recovering
- Closer to the origin = deeper and longer undershoot
- You cannot remove an RHP zero with any controller; it imposes a bandwidth limit of roughly |z|/2
- Real-world example: boiler drum level — adding cold feedwater temporarily drops the level before it rises

**Multiple zeros** — effects compound:
- Each additional zero adds another derivative term
- Two LHP zeros amplify overshoot more than one

## The interactive explorer

`zero_effect_explorer.html` is a single 1180-line HTML file with zero dependencies:

- **Real-time step response** — computed via state-space (controllable canonical form) + RK4 simulation
- **Live pole-zero map** — shows how poles and zeros move as you adjust parameters
- **Sliders** for damping ratio ζ, natural frequency ωₙ, and up to 3 zeros (each spanning LHP ↔ RHP)
- **7 presets** covering the key scenarios: far/near LHP zeros, far/near RHP zeros, two zeros, mixed
- **Dynamic insights** — the bottom panel explains what you're seeing based on the current configuration
- **Combined trace** — when 2+ zeros are active, a dashed white line shows their compound effect

## Screenshots

Run `python3 zero_effect_demo.py` to generate these three figures:

- `zero_effect_one_zero.png` — LHP zeros (left) and RHP zeros (right) compared to baseline
- `zero_effect_two_zeros.png` — one vs. two LHP zeros with a fast 3rd pole
- `zero_effect_pzmap.png` — pole-zero map for an example configuration

## Requirements

- **Web explorer**: any modern browser (Chrome, Firefox, Safari, Edge)
- **Python demo**: Python 3.9+, `control >= 0.10`, `matplotlib >= 3.6`, `numpy`

## License

MIT
