# Zero Effect Explorer

Interactive tools to understand how **zeros** change the step response of a second-order system — a fundamental concept in control theory that's notoriously hard to build intuition for.

## What's here

| File | What it is | Open with |
|------|-----------|-----------|
| `zero_effect_explorer.html` | Self-contained interactive web page | Any browser — no server, no install |
| `pid_explorer.html` | PID controller explorer — see how Kp, Kd, Ki change ζ_eff, ωₙ_eff, and the step response | Any browser — no server, no install |
| `servo_motor_pid.html` | Servo motor PID demo — concrete DC motor position control with real physics (R, L, Kt, J, B) | Any browser — no server, no install |
| `zero_effect_demo.py` | Python script that generates the 3 figures | `python3 zero_effect_demo.py` (needs `control`, `matplotlib`) |
| `zero_effect_video_script.md` | English video script (7 scenes, ~12-15 min) | Any text editor |
| `zero_effect_video_script_cn.md` | Chinese video script for Bilibili (7 scenes) | Any text editor |

## Try it now

```bash
# Interactive web pages — just open them:
open zero_effect_explorer.html   # Zero-effect explorer
open pid_explorer.html           # PID controller explorer
open servo_motor_pid.html        # Servo motor PID demo

# Or run the Python demo:
pip install control matplotlib
python3 zero_effect_demo.py
```

## What you'll learn

**PID Explorer** — how controller gains reshape closed-loop dynamics:
- Kp raises effective ωₙ (faster response) but lowers effective ζ (more overshoot)
- Kd adds damping — it pulls the dominant poles leftward, raising ζ
- Ki eliminates steady-state error by adding an integrator pole at s=0
- PD lets you tune ζ_eff and ωₙ_eff independently; PI kills offset at the cost of a longer tail
- The effective ζ and ωₙ come from the dominant closed-loop pole pair — Cardano's cubic formula under the hood

**Zero-Effect Explorer** — how zeros change the step response of a second-order system:

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

## The PID explorer

`pid_explorer.html` applies the same interactive approach to PID control of a second-order plant:

- **Real-time step response** — closed-loop RK4 simulation with PID state-space model and open-loop plant overlaid for comparison
- **Live pole-zero map** — closed-loop poles, controller zeros, and ghosted open-loop plant poles; stability boundary highlighted
- **Sliders** for plant ζ, ωₙ plus color-coded PID gains (Kp pink, Kd cyan, Ki green)
- **Live readout** of effective ζ,eff, ωₙ,eff, overshoot %, and steady-state error
- **7 presets** covering P-only, PD, PI, tuned PID, aggressive, heavy D, and no-control baseline
- **Dynamic insights** — explains what each gain is doing at the current operating point
- **Unstable detection** — RHP poles trigger a red overlay and warning badge

### 🔑 A fundamental insight: the integrator pole moves

Here's something you'll see immediately when you slide Ki — and it's not a bug:

The PID controller has a pole at **s = 0** from the integrator term (Ki/s). In the **open-loop**, that pole sits right at the origin. But when the feedback loop closes, it **moves**. For the "PID Tuned" preset (ζ=0.5, ωₙ=2, Kp=3, Kd=0.8):

| Ki | Real pole (was at s=0) | Complex pair | What happens |
|----|-------------------------|-------------|--------------|
| 0 (PD) | cancelled | −2.6 ± j3.0 | No integrator — steady-state error remains |
| 0.4 | **−0.103** | −2.55 ± j3.0 | Slow real pole near origin → the long settling tail |
| 2 | −0.48 | −2.36 ± j3.2 | Integrator speeds up, complex pair drifts right |
| 10 | −3.77 | −0.71 ± j3.2 | Complex pair near instability — too much Ki! |

**This is fundamental PID behavior**: integral action pulls the steady-state error to zero, but as Ki increases, the real pole moves left (faster integrator) while the complex pair moves right toward the imaginary axis. Crank Ki too high, and the complex poles cross into the RHP — the system goes unstable.

The root-locus interpretation: the integrator pole at s=0 departs along the negative real axis, while the two plant poles loop toward each other and then break away as Ki grows. The PID explorer shows all of this in real time — watch the pole-zero map as you slide Ki.

## The servo motor PID demo

`servo_motor_pid.html` grounds PID in real physics — a **brushed DC motor** driving a servo position loop:

- **3rd-order motor model** — physical parameters: armature resistance R (Ω), inductance L (H), torque constant Kt (N·m/A), rotor inertia J (kg·m²), viscous friction B (N·m·s/rad)
- **Real constraints** — voltage saturation (±Vmax) with anti-windup, disturbance torque injection
- **Dual-axis step response** — angle θ [rad] on the left, applied voltage V [V] on the right, with saturation fill regions
- **Live pole-zero map** — open-loop motor poles (s=0, mechanical, electrical), closed-loop poles (4th-order via Ferrari's quartic), controller zeros
- **Stiff ODE handling** — automatically switches to reduced-order model when the electrical time constant is too small for explicit RK4 integration
- **7 presets** — No Control, P Only, PD, PID Factory Tune, High Inertia, Voltage Saturated, Disturbance Load
- **SVG motor schematic** in the sidebar — rotor, coil, shaft, terminals
- **Performance readout** — bandwidth, ζ_eff, overshoot %, steady-state error, peak voltage, settling time

This demo answers: what happens to a real motor when you crank up Kp? Why does derivative gain (Kd) prevent overshoot? How does the integrator (Ki) reject a constant torque disturbance?

### Why the CL pole count changes with Ki

A subtle but important detail: switch to the PD preset (Ki=0) and you'll see **3** closed-loop poles. Switch to PID (Ki>0) and you'll see **4**. This is not a bug — it's a direct consequence of the algebra.

The motor's open-loop TF is 3rd order:

$$G(s) = \frac{K_t}{LJ s^3 + (LB+RJ) s^2 + (K_t^2+RB)s}$$

The $s$ factor in the denominator is the mechanical integrator (angle = ∫ angular velocity · dt). The PID controller $C(s) = K_p + K_d s + K_i/s$ adds another integration via $K_i/s$.

- **Ki > 0 (full PID):** clearing the $K_i/s$ term forces a multiply-through by $s$, yielding a **4th-order quartic**: $LJ s^4 + (LB+RJ)s^3 + (K_t^2+RB+K_t K_d)s^2 + K_t K_p s + K_t K_i = 0$. Four poles.
- **Ki = 0 (PD):** there is no $K_i/s$ to clear. The characteristic equation is directly **3rd-order cubic**: $LJ s^3 + (LB+RJ)s^2 + (K_t^2+RB+K_t K_d)s + K_t K_p = 0$. The motor's open-loop pole at $s=0$ **moves** under PD feedback — $K_p$ and $K_d$ shift it left along the real axis. Three poles.

Multiplying by $s$ when $K_i=0$ would introduce a spurious root at the origin — a mathematical artifact, not a physical pole. The code correctly uses the cubic solver for PD and the quartic solver for full PID.

## The interactive zero-effect explorer

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
