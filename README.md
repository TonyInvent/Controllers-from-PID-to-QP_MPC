# Controllers from PID to QP-MPC

Interactive simulators tracing the evolution of feedback control — from classical PID through LQR to constrained QP-MPC — all applied to the same DC motor servo.

**The big idea:** three generations of engineers looked at the same motor and came up with three radically different answers. Not because the motor changed, but because *what they had available* changed — the math they knew, the computers they could touch, the problems they were being paid to solve.

## What's here

### Interactive simulators (open in any browser — no install, no server)

| File | Controller | What it does |
|------|-----------|-------------|
| `pid_explorer.html` | PID | Classical 3-term control of a 2nd-order plant — sliders for Kp, Kd, Ki with real-time pole-zero map |
| `servo_motor_pid.html` | PID | DC motor position servo with real physics (R, L, Kt, J, B), voltage saturation, anti-windup, disturbance torque |
| `lqr_explorer.html` | LQR / LQI | Optimal state feedback via CARE solution — tune cost weights Q & R instead of gains, with PID comparison overlay |
| `servo_qp_mpc.html` | QP-MPC | Constrained Model Predictive Control — online quadratic programming with voltage & current limits, parallel LQR comparison |
| `zero_effect_explorer.html` | — | Transfer-function intuition: how LHP/RHP zeros change step response of a 2nd-order system |

### Companion documents

| File | What it is |
|------|-----------|
| `Servo Motor controllers from PID, LQR to QP-MPC.md` | English podcast script — the century-long history of control theory in 7 slides |
| `Servo Motor controllers from PID, LQR to QP-MPC-zh.md` | Chinese podcast script — Bilibili edition of the same story |
| `zero_effect_video_script.md` | English video script on zero effects (7 scenes) |
| `zero_effect_video_script_cn.md` | Chinese video script on zero effects for Bilibili |
| `zero_effect_demo.py` | Python script generating 3 pole-zero/step-response figures (`control` + `matplotlib`) |
| `servo_qp_mpc.py` | **New** — Python QP-MPC demo: LQR vs naive saturation vs constrained QP on a DC servo (`numpy scipy cvxpy matplotlib`) |
| `The Century of Feedback - A History of Control Theory.md` | **New** — 50-minute expanded podcast: the full 250-year arc from Watt to SpaceX, with inventor stories and historical context |
| `The Century of Feedback - A History of Control Theory-zh.md` | **新** —— 中文版，50分钟播客，反馈控制的250年演化史 |

### Python demos

The two Python scripts need different dependencies.  Use a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

# zero_effect_demo.py — pole-zero / step response figures
pip install control matplotlib
python3 zero_effect_demo.py

# servo_qp_mpc.py — QP-MPC constrained servo demo
pip install numpy scipy cvxpy matplotlib
python3 servo_qp_mpc.py
```

Or install everything at once:  `pip install control matplotlib numpy scipy cvxpy`

## Try it now

```bash
# Interactive web pages — just open any of them in a browser:
open pid_explorer.html              # PID controller explorer
open servo_motor_pid.html           # Servo motor PID demo
open lqr_explorer.html              # LQR explorer
open servo_qp_mpc.html              # QP-MPC explorer
open zero_effect_explorer.html      # Zero-effect explorer
```

## Suggested learning path

Start here and work forward — the historical progression is also the pedagogical one:

1. **`pid_explorer.html`** — grasp the three-term intuition on a simple 2nd-order plant
2. **`servo_motor_pid.html`** — see PID applied to real motor physics (saturation, disturbance)
3. **`lqr_explorer.html`** — discover optimal state feedback: declare what you care about, math returns the gains
4. **`servo_qp_mpc.html`** — add hard constraints: when the 12V supply can't deliver what LQR demands, QP-MPC finds the constrained optimum
5. **`zero_effect_explorer.html`** — deep-dive on zeros: the transfer-function view that explains *why* derivative action speeds things up

## What you'll learn

**PID Explorer** — how controller gains reshape closed-loop dynamics:
- Kp raises effective ωₙ (faster response) but lowers effective ζ (more overshoot)
- Kd adds damping — it pulls the dominant poles leftward, raising ζ
- Ki eliminates steady-state error by adding an integrator pole at s=0
- PD lets you tune ζ_eff and ωₙ_eff independently; PI kills offset at the cost of a longer tail

**Servo Motor PID** — what happens when you apply PID to real hardware:
- Voltage saturation clips your command — and anti-windup keeps the integrator sane
- Torque disturbances cause steady-state error that only Ki can reject
- Stiff electrical dynamics (small L) demand reduced-order modeling for stable simulation

**LQR Explorer** — how optimal control differs from classical tuning:
- You don't tune gains — you declare what you care about (Q, R weights) and math returns the optimal K
- LQR guarantees stability margins (≥60° PM, ∞ gain margin) that PID can't promise
- LQI adds integral action through state augmentation — zero steady-state error, guaranteed
- Reference feedforward is essential: without it, LQI takes ~7 seconds to reach setpoint (vs. 0.5s with FF)

**QP-MPC Explorer** — what happens when constraints are non-negotiable:
- LQR commands whatever voltage optimality demands — QP-MPC finds the best voltage the 12V supply can actually deliver
- Receding-horizon optimization: solve a constrained QP at every time step, apply the first move, repeat
- When constraints are inactive, QP-MPC *is* LQR — smooth, automatic transition between regimes
- Prediction horizon visualization shows where the controller plans to go

**Zero-Effect Explorer** — how zeros change the step response of a second-order system:
- LHP zeros: speed-vs-overshoot trade-off — this is what derivative action does
- RHP zeros: hard physical limits — the system moves the wrong way first, and no controller can remove this
- Multiple zeros compound — each derivative term adds another kick

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

## The LQR explorer

`lqr_explorer.html` demonstrates **Linear Quadratic Regulator (LQR)** design — the modern, state-space counterpart to PID — on the same brushed DC motor:

- **LQI (LQR + Integral Action)** — augments the 3rd-order motor model with an integral-of-error state for zero steady-state tracking error
- **Tune weights, not gains** — instead of heuristically adjusting Kp/Kd/Ki, you specify **cost weights** Q (state penalty) and R (control effort penalty); the optimal state-feedback gains K are computed automatically
- **CARE Solver** — solves the Continuous-time Algebraic Riccati Equation via the Hamiltonian method, using `numeric.eig()` for eigenvalue decomposition of the 8×8 Hamiltonian matrix
- **Real-time step response** — dual-Y-axis plot with θ [rad] on the left and voltage V [V] on the right; saturation fill regions when V hits ±Vmax
- **Live pole-zero map** — 4 LQR closed-loop poles (teal ×) vs. 3 open-loop motor poles (grey ×); stability boundary highlighted
- **Log-mapped cost weights** — sliders for q_θ (position error), q_ω (velocity), q_i (current), q_∫ (integral error), and R (control penalty)
- **PID comparison toggle** — overlay a PID step response (gold dashed) to contrast classical tuning against optimal LQR
- **Computed gains readout** — K = [k_θ, k_ω, k_i, k_∫] with color coding mapping to PID concepts: k_θ≈Kp, k_ω≈Kd, k_i (current feedback), k_∫≈Ki
- **6 presets** — Balanced LQR, Cheap Control, Expensive Control, Position Focus, Heavy Damping, Fast Integrator
- **Anti-windup** — integrator state freezes when voltage saturates, preventing integral windup
- **Adaptive time window** — simulation duration auto-scales based on closed-loop settling time (4/(ζ·ωₙ))

### LQR vs. PID: what changes

| Aspect | PID | LQR |
|--------|-----|-----|
| Tuning | Heuristic — Kp, Kd, Ki | Systematic — Q, R weights |
| Gains | 3 scalar gains | 4 gains: k_θ, k_ω, k_i, k_∫ |
| Current feedback | None (implicit) | Explicit k_i feedback |
| Design | Try-and-see | One-shot optimal |
| Guarantees | None (can destabilize) | Guaranteed stability margins (≥60° PM, ≥6 dB GM) |
| Effort | Shape response directly | Penalize states + effort → optimal trade-off |

### The LQI cost function

$$J = \int_0^\infty \left( q_\theta \theta^2 + q_\omega \omega^2 + q_i i^2 + q_{\int} (\smallint e)^2 + R u^2 \right) dt$$

- **q_θ ↑** — faster position response, more overshoot
- **q_ω ↑** — more damping, less overshoot
- **q_i ↑** — less aggressive current draw
- **q_∫ ↑** — faster steady-state error recovery
- **R ↑** — smaller, more conservative control (penalizes voltage)

The CARE solver produces the optimal gain matrix **K** that minimizes J. The closed-loop eigenvalues of (A − BK) are the LQR-optimal pole locations — no manual pole placement needed.

### 🔑 How LQR handles the reference command

A question every LQR newcomer hits: **where does the reference `r` go?** The control law `u = −Kx` only mentions the state — it drives everything to zero. So how does LQR track a desired position?

**LQR is a regulator, not a tracker.** It was designed to return a perturbed system to equilibrium (`x → 0`), not to follow a command. To make it track, you must transform the tracking problem into a regulation problem. Two standard approaches exist.

**Approach 1: LQI (LQR + Integral Action).** Augment the state with the integral of tracking error:

$$x_{aug} = [\theta, \omega, i, \smallint e]^T, \quad e = r - \theta$$

The augmented dynamics inject the reference through the error integral:

$$\dot{x}_{aug} = A_{aug} x_{aug} + B_{aug} u + \underbrace{[0, 0, 0, 1]^T}_{B_{ref}} \cdot r$$

The reference `r` enters by driving the rate of $\smallint e$: $d(\smallint e)/dt = e = r - \theta$. The optimal control `u = −K·x_aug` then includes integral action automatically because $x_{aug}$ contains $\smallint e$. This guarantees **zero steady-state error** for constant references — the integrator adjusts until $\theta = r$.

**Approach 2: Reference feedforward.** Add a feedforward term directly to the control law:

$$u = N r - K x$$

`N` is chosen to give unity DC gain from `r` to the output. Without N, the closed-loop system has a DC gain that is generally not 1, so $\theta_{ss} \neq r$ even without disturbances.

**Why pure LQI feels slow — and how `lqr_explorer.html` fixes it.** In Approach 1, the reference only reaches the plant *indirectly* through the error integrator. At $t=0$, all states are zero, so $u = 0$ — no immediate control action. The integrator must accumulate error over time before meaningful voltage appears. For the default motor, this takes ~7 seconds just to reach the setpoint.

The fix: combine both approaches. Use $k_\theta r$ as feedforward (analogous to PID's $K_p r$) while keeping the integral state for steady-state correction:

$$u = k_\theta r - K x_{aug} = k_\theta(r-\theta) - k_\omega \omega - k_i i - k_{\smallint} \smallint e$$

This is what `lqr_explorer.html` does by default. Here's how each method injects the reference:

| Method | Control law | $u$ at $t$=0 | $\theta_{ss}$ | Speed |
|--------|------------|--------------|---------------|-------|
| PID (PD, Ki=0) | $u = K_p(r-\theta) - K_d\omega$ | $K_p r$ (5.7 V) | $r$ − offset | Fast |
| Pure LQI (no FF) | $u = -K x_{aug}$ | 0 V | $r$ (via ∫) | Very slow |
| LQI + $k_\theta$ FF | $u = k_\theta r - K x_{aug}$ | $k_\theta r$ | $r$ (via ∫) | Fast |
| LQI + exact DC FF | $u = N r - K x_{aug}$ | $N r$ | $r$ (via ∫) | Fast, exact |

The $k_\theta$ feedforward is a pragmatic choice — it's simple, it's exactly what PID does, and any residual DC offset is corrected by the integral action. The exact DC-gain feedforward $N = -(C(A-BK)^{-1}B)^{-1}$ would eliminate the offset entirely but requires a matrix inverse and depends on the current K.

**In short:** LQR needs help to track a reference. LQI provides it through the error integral (guaranteed zero steady-state error, but slow). Adding $k_\theta r$ feedforward gives the immediate "kick" that makes it responsive. This is not a hack — it's standard practice in LQI implementation, directly analogous to PID's $K_p r$ term.

### 🔑 The q_ω speed paradox: faster → slower → faster

Try this in the LQR explorer: set q_θ = 5, then slide q_ω from 0.01 to 100. You'll see the step response go **fast → slow → fast again**. This is correct LQR behavior, caused by **dominant pole switching** as the velocity penalty increases.

At q_θ = 5 with the default motor:

| q_ω | k_θ | k_ω | Dominant poles | ζ_eff | Settling | What happens |
|-----|-----|-----|---------------|-------|----------|--------------|
| 0.01 | 9.1 | 1.58 | Complex −5.2 ± j5.0 | 0.72 | 0.78 s | Underdamped, **fast** rise with ringing |
| 0.1 | 9.4 | 1.86 | Complex −6.2 ± j3.5 | 0.87 | 0.65 s | More damped, clean — quickest settle |
| 0.3 | 9.9 | 2.36 | **Real** −1.5 (dominant) | ~1.0 | 2.63 s | Pole switch! Overdamped, **sluggish** |
| 1.0 | 11.1 | 3.59 | Complex −1.7 ± j0.6 | 0.95 | 2.37 s | Underdamped again but slow ωₙ |
| 10 | 16.0 | 10.2 | Complex −0.8 ± j0.6 | 0.79 | 5.06 s | High k_θ gives strong kick — **fast initial rise**, long tail |

**Why it happens:**

- **Low q_ω** — velocity is cheap to penalize. The LQR solution doesn't damp much, so the dominant poles are a fast, lightly-damped complex pair (ωₙ ≈ 7 rad/s). Response is quick but rings.

- **Medium q_ω (~0.3)** — damping pushes the complex poles so far left that they split into two real poles. Meanwhile the integral-action pole near s = 0 becomes the **slowest** pole. The response becomes overdamped and sluggish — the integrator pole dominates.

- **High q_ω** — the real poles merge back into a slow complex pair, while the proportional gain k_θ grows significantly (from 9 → 26). The large k_θ gives a strong initial "kick" that feels fast, but the dominant poles are slow (ωₙ ≈ 0.6 rad/s), creating a long settling tail.

Watch the pole-zero map as you slide q_ω — you'll see the four closed-loop poles undergo this switching, and the controller zeros (orange ○) tell the same story: one zero stays near the origin (the slow tail) while the other shoots far left (the derivative kick).

### 🔑 LQR vs PID: the reference feedforward

A common first impression: no matter how you adjust the Q/R weights, LQR seems slower than a well-tuned PD controller (e.g. Kp=5.7, Kd=1.72, Ki=0). This is **not** a fundamental limitation of LQR — it's a missing feedforward term.

**Why PID seems faster.** The PID control law is:

$$u = K_p e - K_d \omega = K_p (r - \theta) - K_d \omega = \underbrace{K_p r}_{\text{feedforward}} - K_p \theta - K_d \omega$$

At $t=0$, the $K_p r$ term immediately applies $K_p \cdot 1 = 5.7\mathrm{V}$ to the motor — an instant "kick" that drives the response.

**Why LQI seems slower.** The pure LQI control law is:

$$u = -(k_\theta \theta + k_\omega \omega + k_i i + k_{\smallint} \smallint e)$$

At $t=0$, all states are zero, so $u=0$. The controller must wait for $\smallint e = \int (r-\theta)dt$ to slowly accumulate before applying meaningful voltage. In the default configuration, this takes **~7 seconds** just to reach the setpoint — compared to PID's 0.54s rise time.

**The fix: reference feedforward.** Adding $k_\theta r$ to the control law (exactly analogous to PID's $K_p r$) gives:

$$u = k_\theta r - (k_\theta \theta + k_\omega \omega + k_i i + k_{\smallint} \smallint e) = k_\theta (r-\theta) - k_\omega \omega - k_i i - k_{\smallint} \smallint e$$

This is the LQR explorer's default behavior. The integral action then only needs to correct for residual steady-state error — not supply the entire DC bias.

**Performance comparison (default motor, R=4, L=0.02, Kt=0.06, J=0.002, B=2e-4):**

| Controller | Gains | $t_r$ [s] | $t_s$ [s] | OS % | SS err % | Notes |
|------------|-------|-----------|-----------|------|----------|-------|
| PD (Ki=0) | Kp=5.7, Kd=1.72 | 0.54 | 0.94 | 0 | 0.01 | No integral — can't reject disturbance |
| LQI (balanced, q_θ=100) | k=[33, 4.4, 0.7, −10] | 0.22 | 2.0 | 3.0 | 0.02 | Integral action — zero SS error guaranteed |
| LQI (fast, high q_θ=500) | k=[71, 5.7, 0.9, −10] | 0.14 | 0.24 | 1.2 | 0.005 | Saturates at 12V — aggressive |
| LQI (tiny q_∫, low R) | k=[22, 4.0, 0.6, −1.0] | 0.32 | 0.55 | 0.7 | 0.007 | Relaxed integral ≈ PD-like dynamics |

**How to tune LQI to exceed PID performance:**
- **q_θ ↑** — increases $k_\theta$ (proportional gain), giving a stronger feedforward kick and faster rise. Watch for saturation at high values.
- **q_ω ↑** — increases $k_\omega$ (derivative gain), adding damping. Unlike PID, LQR does this optimally — it won't amplify sensor noise.
- **R ↓** — reduces the control penalty, allowing larger gains (Cheap Control). This is the LQR equivalent of "more aggressive tuning."
- **q_∫ ↓** — shrinks the integral gain $k_{\smallint}$, making the response more PD-like. Use when disturbance rejection isn't critical.

## The QP-MPC explorer

`servo_qp_mpc.html` demonstrates **Quadratic Programming Model Predictive Control** — the third generation of servo control, where hard constraints are front and center:

- **3-state motor model** — position θ, velocity ω, current i — discretized via forward Euler at a configurable sample time Ts
- **Online QP solver** — coordinate-descent box-constrained quadratic programming: `min ½UᵀHU + fᵀU` subject to `−Vmax ≤ uₖ ≤ +Vmax`
- **Parallel LQR simulation** — a separate motor instance runs unconstrained LQR, so you can directly compare: QP-MPC clips at ±12V while LQR blithely commands 30V
- **Prediction horizon visualization** — dashed cyan dots show the MPC's planned trajectory over the next N steps
- **Configurable horizon** — N = 3 to 30 steps; longer horizons see further ahead but cost more computation
- **Real-time QP diagnostics** — active constraint count, QP iterations, solve time (ms), and cost function value J
- **Three live charts** — position tracking θ(t), control voltage V(t) with ±Vmax limit lines, motor current i(t) with ±Imax limit lines
- **Motor shaft animation** — visual indicator of the rotor angle
- **Adaptive speed control** — slow down simulation to watch constraint boundary behavior frame by frame

### QP-MPC vs. LQR: the constraint story

| Scenario | LQR | QP-MPC |
|----------|-----|--------|
| **Constraint inactive** | Applies optimal voltage freely | Same as LQR — unconstrained optimum is feasible |
| **Voltage hits +Vmax** | Saturates — optimality lost, may wind up | Finds constrained optimum: best voltage ≤ Vmax |
| **Starting from rest** | Commands large spike to accelerate quickly | Same spike — but clipped to Vmax if needed |
| **Near steady-state** | Small corrections, well within limits | Same as LQR — constraints are slack |

This is the key insight: **when constraints are inactive, QP-MPC *is* LQR.** There's no switched controller, no gain scheduling. The QP solver naturally recovers the unconstrained optimum when all bounds are slack, and smoothly transitions to a boundary-riding solution when a constraint becomes active.

### 🔑 The most common misconception: "just saturate LQR"

Here's the trap. You have LQR, it's optimal, and your amplifier can only output ±12V. The obvious fix: compute u = −Kx as usual, then clip it to ±12V with a one-liner:

```
u = clamp(−Kx, −Vmax, +Vmax)
```

This is called **naive saturation**, and it feels right. After all, you're applying the optimal control whenever it's "available," and when it's not, you're doing the best the hardware can do — right?

**No. And this is the single most important idea in constrained control.**

Saturating LQR's output is not the constrained optimum. It's not even close. Here's why, broken into two layers:

**Layer 1 — the controller doesn't know it's saturated.** LQR computes its gains assuming unlimited authority. When you build feedback gains on that assumption, the *entire gain matrix K* is wrong for the constrained regime. The controller plans aggressive moves — large voltage spikes, sharp reversals — that only make sense if the amplifier can deliver them. Saturating after the fact means the controller is flying blind: it keeps computing commands as if the motor got 30V, while the motor actually got 12V. The internal state trajectory diverges from what the controller believes.

A concrete example: from rest, LQR commands a 30V spike to accelerate the motor. You clamp to 12V. The motor accelerates slower than LQR expects. LQR's state estimate now says "we're behind schedule — hit it harder." Next step: another 30V command, clamped again. The controller is permanently confused, permanently commanding voltages it will never get. This is how naive saturation produces oscillation, overshoot, and sometimes instability — not because the motor can't handle 12V, but because the *controller was designed for a world where 30V exists.*

**Layer 2 — the constrained optimum is a different shape.** When you solve the QP with the constraint baked in, the optimizer knows it only has 12V. It doesn't just clip the unconstrained solution — it finds a *completely different control sequence* that stays within 12V for the entire horizon while minimizing the tracking cost. This often means:

- Starting to decelerate **earlier** than the unconstrained solution would, because the limited voltage can't brake as hard
- Applying **less voltage early** to avoid overshooting a target that can't be corrected quickly
- **Not saturating at all** for some steps where the naive clamp would hit the rail — the optimizer knows holding 11.9V now is better than hitting 12V, overshooting, and needing −12V later

The key distinction: naive saturation says "do your best, I'll clean up the mess." Constrained optimization says "given the walls, find the best path that never touches them unnecessarily." The two produce identical results only in the trivial case where constraints are never binding.

**And the simulator proves it.** In `servo_qp_mpc.html`, the LQR trace (orange) and QP-MPC trace (cyan) run on the same motor with the same cost weights. When constraints are active, the QP-MPC voltage profile is not just a clipped version of LQR's — it's structurally different. Watch the voltage chart during a large step reference: LQR demands a spike that gets clipped, overshoots, then oscillates. QP-MPC pre-emptively backs off, delivering a cleaner, often faster settling response with less total energy.

**The deeper principle:** every physical system has constraints — voltage, current, torque, temperature, pressure. An optimal controller that doesn't know about them is answering the wrong question. QP-MPC answers the right one: *what is the best I can do, given what I actually have?*

### 🔑 How the control problem becomes a QP — condensing explained

The QP at each time step doesn't appear out of thin air. It comes from **eliminating the dynamics constraints** from the MPC optimisation, a process called *condensing*. Here's the full derivation, traced through `servo_qp_mpc.py`.

#### 1. The original MPC problem

At each timestep, given the current state error `x₀_err = x − x_ref`, we want to solve:

```
min    Σᵢ₌₀ᴺ⁻¹ (xᵢᵀ Q xᵢ + uᵢᵀ R uᵢ)   +   x_Nᵀ P x_N
u₀…uₙ₋₁

s.t.   xᵢ₊₁ = A_d xᵢ + B_d uᵢ        (discrete motor dynamics)
      −V_max ≤ uᵢ ≤ V_max            (voltage limits)
```

The terminal cost `x_Nᵀ P x_N` (where `P` comes from the discrete algebraic Riccati equation) guarantees stability — it approximates the infinite-horizon cost beyond the prediction window.

This problem has both dynamics equality constraints and box inequality constraints. A general-purpose QP solver can't handle arbitrary equality constraints efficiently — we need to eliminate the dynamics.

#### 2. Condensing — write the state trajectory purely as a function of `x₀` and `U`

Unroll the linear dynamics over the horizon of length `N`. For `N = 3`:

```
x₁ = A_d x₀ + B_d u₀
x₂ = A_d² x₀ + A_d B_d u₀ + B_d u₁
x₃ = A_d³ x₀ + A_d² B_d u₀ + A_d B_d u₁ + B_d u₂
```

Stack all N future states into one big vector `X = [x₁; x₂; …; x_N]`. In matrix form:

```
X = A_aug · x₀_err  +  B_aug · U
```

where:
- `A_aug` is a vertical stack of `A_d, A_d², …, A_dᴺ` — the *free response* of each step to the initial state
- `B_aug` is a **block-lower-triangular Toeplitz matrix** — element `(row, col)` is `A_d^(row−col) B_d`, capturing how the input at step `col` propagates to the state at step `row` through the dynamics

This is the code in `servo_qp_mpc.py`:

```python
A_aug = np.vstack([np.linalg.matrix_power(Ad, k+1) for k in range(N)])

B_aug = np.zeros((N*n, N*m))
for row in range(N):
    for col in range(row + 1):
        B_aug[row*n:(row+1)*n, col] = (
            np.linalg.matrix_power(Ad, row-col) @ Bd).flatten()
```

#### 3. Substitute into the cost — all that's left is a standard QP

The original cost is `J = Xᵀ Q̄ X + Uᵀ R̄ U`, where `Q̄ = block-diag(Q,…,Q,P)` puts the Riccati matrix `P` on the final block, and `R̄ = block-diag(R,…,R)`.

**Substitute** `X = A_aug x₀_err + B_aug U` into `J` and expand:

```
J = x₀ᵀ A_augᵀ Q̄ A_aug x₀          ← constant (doesn't depend on U, can be dropped)
  + 2 x₀ᵀ A_augᵀ Q̄ B_aug U        ← linear term in U
  + Uᵀ (B_augᵀ Q̄ B_aug + R̄) U     ← quadratic term in U
```

Define:

```
H = B_augᵀ Q̄ B_aug + R̄              (quadratic cost matrix — penalises control energy)
F = A_augᵀ Q̄ B_aug                   (maps initial state into linear cost term)
```

The code:

```python
Qbar = np.kron(np.eye(N), Q_lqr)
Qbar[-n:, -n:] = P_lqr               # terminal cost

H = B_aug.T @ Qbar @ B_aug + Rbar    # Uᵀ H U / 2
F = A_aug.T @ Qbar @ B_aug           # (Fᵀ x₀_err)ᵀ U
```

The original MPC problem is now equivalent to the **standard box-constrained QP**:

```
min    ½ Uᵀ H U  +  (Fᵀ x₀_err)ᵀ U
 U

s.t.   −V_max ≤ uₖ ≤ V_max      (box constraints only — one per control step)
```

All the dynamics constraints have been absorbed into `H` and `F`. The QP solver sees only `N` scalar variables with simple upper/lower bounds — that's what makes it fast enough for a 1 ms control loop.

#### 4. Building the QP once, solving it repeatedly

The QP **structure** (`H`, `F`, the box constraints) is built once before the simulation loop — `H` and `F` only depend on the motor model and cost weights, which don't change. What changes at each timestep is the current state error `x₀_err`, which enters through the linear term `(Fᵀ x₀_err)ᵀ U`.

This is handled with a **parameter** in cvxpy:

```python
x0_param = cp.Parameter(n)                 # placeholder for the current state error
U = cp.Variable(N)                         # the N control inputs we're optimising over

prob = cp.Problem(
    cp.Minimize(0.5 * cp.quad_form(U, H) + (F.T @ x0_param).T @ U),
    [U >= -V_max, U <= V_max])
```

The `Problem` object is constructed once. At each 1 ms timestep:

```python
x0_param.value = x0_err                    # inject the current state error
prob.solve(solver=cp.OSQP, warm_start=True, polish=False,
           max_iter=400, eps_abs=1e-4, eps_rel=1e-4)
u = float(U.value[0])                      # apply only the first control (receding horizon)
```

Key solver choices and why they matter for a real-time control loop:

- **`solver=cp.OSQP`** — OSQP is an operator-splitting QP solver (ADMM-based). It's fast for MPC because it exploits the sparsity of the condensed problem and handles warm starts well.
- **`warm_start=True`** — initialises the solver with the previous timestep's solution. Since the state changes by only one 1 ms step, the previous optimal sequence is very close to correct, often cutting iterations by 80%+.
- **`polish=False`** — skips a final refinement step. In a 1 ms control loop, speed matters more than high-precision optimality.
- **`max_iter=400, eps_abs/eps_rel=1e-4`** — cap iteration count and set convergence tolerance. These are tuned for the 1 ms period: if the solver hasn't converged by 400 iterations, the best-so-far solution is good enough.
- Only the **first element** `U.value[0]` is applied to the motor. The rest of the sequence is discarded, and the whole optimisation repeats at the next sample time (receding horizon principle).

#### Summary picture

```
At each 1 ms step:
  ┌─────────────┐     ┌─────────────────────┐      ┌──────────────┐
  │ measure x   │ ──▶ │ condense into QP    │ ──▶  │ OSQP solves  │
  │ error x₀    │     │ H, F precomputed    │      │ min ½UᵀHU +  │
  │             │     │ x₀ → linear term    │      │ (Fᵀx₀)ᵀU     │
  └─────────────┘     └─────────────────────┘      │ s.t. |u|≤V   │
                                                   └──────┬───────┘
                                                          │
                                           apply u₀ only ─┘
```

The crucial distinction from naive saturation: the QP *knows about the constraints* when planning its trajectory — `H` and `F` encode the full dynamics, so the solver can pre-emptively back off the voltage to avoid overshoot. Naive saturation plans as if unlimited voltage exists (calculating gains from the unconstrained Riccati equation), then post-hoc clips — which is why it overshoots and oscillates.

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
