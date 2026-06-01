# Core Problems in Controller Design

**Every controller, from PID to nonlinear MPC, is an answer to a set of fundamental problems. Understanding these problems is understanding why the field evolved the way it did.**

---

## 1. Overview

Strip away the math, and every control system faces the same obstacles standing between you and perfect tracking. Each one has generated an entire branch of control theory:

| # | Problem | What it means | What it generates |
|---|---------|--------------|-------------------|
| 1 | **Measurement** | You don't know the true state — noise, delay, partial sensing | Kalman filter, Luenberger observer, Smith predictor |
| 2 | **Inertia / delay** | The plant doesn't respond immediately — energy must move, mass must accelerate | Damping control, model-based design, feedforward |
| 3 | **Constraints** | The actuator has hard limits — voltage, current, torque, rate | Constrained optimization, MPC, anti-windup |
| 4 | **Model uncertainty** | The model is always wrong — parameters drift, dynamics are unmodeled | Robust control (H∞), adaptive control, sliding mode |
| 5 | **Disturbances** | Unknown external forces act on the plant — wind, load changes, friction | Disturbance observer, ADRC, internal model principle |
| 6 | **Nonlinearity** | The real world isn't linear — friction, saturation, aerodynamics | Gain scheduling, feedback linearization, nonlinear MPC |
| 7 | **Stability** | You need guarantees the system won't blow up | Lyapunov theory, passivity, small-gain theorem |
| 8 | **Computation** | The controller must run in real time on real hardware | Explicit MPC, warm starting, efficient QP solvers |
| 9 | **Multi-objective trade-offs** | You can't have everything — speed vs overshoot, energy vs accuracy | Pareto-optimal control, H₂/H∞ trade-off, Q/R tuning |

These are not separate problems. They compound. A rocket landing has all nine active simultaneously: noisy hypersonic GPS, massive booster inertia, engine throttle limits, uncertain aerodynamics, wind gusts, nonlinear drag, stability-critical failure modes, millisecond control deadlines, and competing fuel-vs-accuracy objectives. No single technique solves all of them — which is why production controllers compose multiple solutions.

---

## 2. Measurement — you don't know the true state

### The problem

A controller computes `u = f(measured state)`. But what you have is:

- **Noise:** sensor readings are corrupted — thermal noise, quantization, vibration
- **Delay:** measurement arrives late — sensor latency, communication bus, computation time
- **Partial measurement:** you measure motor position but not velocity; room temperature at one point but not the whole thermal field

Using the raw measurement as if it were the true state produces wrong control actions. Worse, feeding noisy derivatives (e.g., differencing position to get velocity) amplifies high-frequency noise. Derivative action in PID is famous for this — it's why many industrial PID loops are PI-only.

### The solution: estimate the state

The core idea: run a **model of the plant** in parallel, and correct its state using measurements.

**Luenberger observer** (1964). Run a copy of the plant model with a correction term:

$$\hat{x}_{k+1} = A \hat{x}_k + B u_k + L (y_k - C \hat{x}_k)$$

The gain $L$ is designed by pole placement — place observer poles 2–5× faster than the controller poles. The **separation principle** guarantees you can design estimator and controller independently.

**Kalman filter** (1960). Same structure, but $L$ is chosen *optimally* given statistical noise models:

| | Control (LQR) | Estimation (Kalman) |
|---|---|---|
| **Riccati direction** | Backward | Forward |
| **Weights** | $Q$ (state cost), $R$ (control cost) | $Q$ (process noise), $R$ (measurement noise) |
| **What it minimizes** | State error + control effort | Estimation error covariance |

The Kalman + LQR combination is **LQG** — optimal estimation + optimal control, for linear systems with Gaussian noise. Duality: estimation and control are the same mathematics running in opposite directions.

**Smith predictor** (1957). A specialized solution for pure measurement delay $\tau$. Run a model *without* delay for feedback, add a correction term for the delayed model mismatch. Developed for chemical process control (minutes of pipe transport delay). Still used.

### Why this drives controller evolution

- **PID** assumes measurement = truth. Derivative action amplifies noise; low-pass filtering is ad-hoc.
- **LQR** assumes full state feedback. In practice → LQG (Kalman + LQR).
- **MPC** typically runs a separate estimator (Kalman filter, moving horizon estimation) upstream.

---

## 3. Inertia / delay — the plant doesn't respond immediately

### The problem

Apply voltage to a motor. It doesn't instantly reach speed — the rotor has inertia. Open a valve — the fluid column has momentum. Turn on a heater — thermal mass absorbs energy.

Inertia stores energy and releases it over time. From the controller's perspective, this feels like delay: you command something, nothing happens for a while.

### Why inertia is essentially a delay problem

Phase lag is the common mechanism:

- First-order lag $1/(\tau s + 1)$: phase $\to -90^\circ$
- Second-order system: phase $\to -180^\circ$
- Pure delay $e^{-\tau s}$: phase drops linearly without bound

All three destabilize feedback the same way. The controller pushes, sees no response, pushes harder — by the time the plant moves, the controller has pushed too far. Overshoot, oscillation, instability.

### The solution: damping + prediction

**Damping** — PID's derivative term, LQR's velocity penalty ($q_\omega$), MPC's terminal cost ($P$) — all say the same thing: *slow down before you get there.*

**Feedforward** — instead of waiting for error, compute the control that *should* produce the desired output: $u_{\text{ff}} = G^{-1}(0) \cdot r$. PID's $K_p r$ term is zeroth-order feedforward. Higher-order versions invert the full plant model.

**Prediction** — for large inertia, feedback alone is too slow. By the time you see an error, it's already too late. You must predict where the plant is going. This is why MPC's receding horizon matters: it plans a trajectory that accounts for the plant's sluggishness.

### The model-pays-off threshold

- Small inertia → feedback alone works → PID sufficient
- Moderate inertia → need damping → PID with derivative, LQR
- Large inertia → need prediction → model-based feedforward, MPC
- Massive inertia + delay → full prediction + constraints → MPC + observers

The central insight: **inertia makes the model matter.** When the plant responds quickly, ignorance is cheap. When it's slow, ignorance kills.

---

## 4. Constraints — the actuator has limits

### The problem

Every physical actuator saturates: voltage rails, current ratings, torque limits, temperature ceilings, slew rates. A controller that asks for 30V from a 12V supply is asking the impossible.

### Why naive saturation fails

The obvious fix — compute, then clip — fails for three reasons:

1. **The controller doesn't know it's saturated.** The internal state (integrator, estimator) assumes the commanded $u$ was applied. The plant received less. Model divergence.

2. **Integrator windup.** The integral term accumulates while saturated. When the error crosses zero, the wound-up integrator causes massive overshoot. Anti-windup patches this but doesn't solve the root cause.

3. **The constrained optimum has a different shape.** When the solver knows about the constraint, it finds a *structurally different* control sequence — pre-emptively backing off, avoiding unnecessary saturation. Naive saturation clips; constrained optimization reroutes.

### The solution: bake constraints into the optimization

MPC solves a Quadratic Program at each time step:

$$\min_u J(x, u) \quad \text{s.t.} \quad u_{\min} \leq u \leq u_{\max}$$

The constraints enter the optimization, not the post-processing. The solver sees the walls and plans around them.

**Cost:** LQR solves one Riccati equation offline. MPC solves a QP online at every time step. The history of real-time optimization — interior-point, active-set, operator splitting, warm starting — is the history of making this fast enough.

---

## 5. Model uncertainty — the model is always wrong

### The problem

Every controller uses a model, explicit or implicit. PID's model is "P, I, and D terms will work." LQR's model is the A and B matrices. MPC's model is the full prediction dynamics.

All models are wrong. Parameters drift (motor resistance heats up, battery voltage drops). Dynamics are simplified (flexible modes treated as rigid, friction modeled as linear viscous). Operating points change (aircraft at sea level vs 40,000 ft).

An optimal controller for the wrong model can be worse than a suboptimal controller for the right one. This is the **robustness problem**.

### The solution spectrum

**Robust control** (1980s–). Design the controller to tolerate a specified uncertainty set. If the real plant lies within some bounded perturbation of the nominal model, stability and performance are guaranteed. Tools: H∞ synthesis, μ-synthesis, small-gain theorem, gap metric.

The H∞ problem: find a controller $K$ that minimizes the worst-case gain from disturbance to error, over all plants in the uncertainty set:

$$\min_K \| T_{zw} \|_\infty$$

This produces controllers that are deliberately conservative — they trade nominal performance for guaranteed robustness.

**Adaptive control** (1950s–). Instead of designing for worst-case uncertainty, *estimate the parameters online* and update the controller. Model Reference Adaptive Control (MRAC): force the plant to behave like a reference model by adjusting controller gains in real time. Self-tuning regulators: estimate plant parameters, recompute controller, apply.

The trade-off: robust control guarantees stability but is conservative. Adaptive control can achieve higher performance but risks instability during transients (parameter estimates can drift, excitation can be insufficient).

**Sliding mode control** (1970s–). A different philosophy: instead of modeling uncertainty, *dominate* it. Apply a discontinuous control law that forces the state onto a "sliding surface" and keeps it there, regardless of bounded uncertainty. The price: chattering (high-frequency switching), which mechanical systems hate.

**Gain scheduling.** The practical compromise. Design controllers at multiple operating points, interpolate gains based on a measured scheduling variable (airspeed, altitude, temperature). Widely used in aerospace. Not theoretically elegant — stability between design points isn't guaranteed — but it works.

### Why this matters

A controller that's optimal for the nominal model and fragile to uncertainty is useless in practice. Every real deployment grapples with robustness. LQR's guaranteed margins (≥60° PM, ∞ GM) were a major reason for its adoption — they're *automatic*, not something the designer must verify separately.

---

## 6. Disturbances — the world pushes back

### The problem

Disturbances are external forces you don't control:

- Wind gusts on a drone
- Load torque on a motor (a robot picking up a part)
- Supply voltage fluctuations
- Friction changes (wear, temperature)
- Sensor bias drift over time

Disturbances are different from measurement noise. Noise corrupts what you *see*. Disturbances corrupt what the plant *does*. A Kalman filter handles measurement noise. It does not handle an unexpected load torque — unless you model the disturbance as an augmented state (which is exactly what disturbance observers do).

### The solution: estimate and cancel

**Integral action** is the oldest disturbance rejection technique. The integrator accumulates persistent error and adjusts the control to cancel it. Internal Model Principle: to reject a disturbance, the controller must contain a model of that disturbance. A constant disturbance requires an integrator ($1/s$). A sinusoidal disturbance requires a resonator ($\omega/(s^2 + \omega^2)$).

**Disturbance Observer (DOB)** (1980s–). Estimate the disturbance by comparing the actual plant output to what the model predicted:

$$\hat{d} = G_{\text{model}}^{-1}(y) - u$$

Feed the estimate forward to cancel it. Simple, effective, widely used in motion control (robot joints, hard disk drives).

**Active Disturbance Rejection Control (ADRC)** (Han, 1990s). Extend the plant state with a "total disturbance" term that lumps together unknown dynamics, external forces, and nonlinearities. Estimate everything with an Extended State Observer (ESO). Cancel the disturbance with feedforward. The remaining dynamics are approximately a pure integrator chain — trivial to control. ADRC is remarkably effective with almost no model information.

**Feedforward** from a measured disturbance source. If you can measure the disturbance (wind speed, incoming material temperature), feed it forward to pre-compensate before the feedback loop even sees the error.

### Why this matters

Disturbance rejection is what separates a toy controller from an industrial one. PID's integrator is the minimal viable solution. ADRC and DOB are more principled. MPC can incorporate disturbance models into the prediction — but only if the disturbance is known or estimated.

---

## 7. Nonlinearity — the real world isn't linear

### The problem

Every physical system is nonlinear. Friction has stiction and Coulomb components (not just linear viscous). Aerodynamic drag scales with $v^2$. Chemical reaction rates follow Arrhenius ($e^{-E/RT}$). Transistors saturate. Gears have backlash.

Linear control theory works near an operating point (Taylor expansion: $\sin\theta \approx \theta$ for small $\theta$). Far from that point, it fails. An aircraft at high angle of attack, a robot at high speed, a chemical reactor near runaway — linear assumptions break.

### The solution spectrum

**Gain scheduling** (practical, widely used). Design linear controllers at many operating points, interpolate between them. No theoretical guarantee, but it works in practice (aircraft flight control, engine management).

**Feedback linearization** (1980s–). Find a nonlinear change of variables and a control law that makes the closed-loop system exactly linear. Then apply linear control theory to the linearized system. Requires an accurate model. Fails if the model is uncertain.

**Sliding mode control** (1970s–). Don't model the nonlinearity. Dominate it. Drive the state to a sliding surface with high-gain switching, then slide to the origin. Robust to bounded uncertainty. Chattering is the practical problem.

**Backstepping** (1990s–). For systems in "strict feedback form" (cascaded integrators with nonlinearities at each stage), recursively design Lyapunov functions and virtual controls from the innermost loop outward. Systematic and constructive — unlike feedback linearization, it can handle some classes of uncertainty.

**Nonlinear MPC** (2000s–). Solve the full nonlinear optimal control problem online. At each step, minimize $J$ subject to $x_{k+1} = f(x_k, u_k)$ and constraints — using DDP (Differential Dynamic Programming), SQP (Sequential Quadratic Programming), or direct collocation. Computationally expensive but increasingly practical (SpaceX rocket landing, Boston Dynamics robots).

### Why this matters

Nonlinearity is the frontier. PID, LQR, and linear MPC handle systems near equilibrium. When the system moves through large regions of state space — swinging up a pendulum, landing a rocket, aggressive maneuvering — you need nonlinear tools. The progression from linear MPC to nonlinear MPC mirrors the progression from LQR to linear MPC: each step handles more problems simultaneously, and pays with computation.

---

## 8. Stability — you need guarantees

### The problem

It's not enough for a controller to work in simulation. You need to *prove* the closed-loop system won't blow up, even when:

- The model is approximate (robust stability)
- The system starts far from equilibrium (region of attraction)
- Disturbances are present (input-to-state stability)
- The controller is implemented in discrete time on a continuous plant (sampled-data stability)

A controller without stability guarantees is a liability. PID can be destabilized by aggressive tuning — there's no automatic protection. This is why LQR was revolutionary: it guarantees 60° phase margin and infinite gain margin *automatically*, no matter what Q and R you choose.

### The solution: Lyapunov theory and its descendants

**Lyapunov stability** (1892). Find a scalar function $V(x) > 0$ for $x \neq 0$ that decreases along trajectories ($\dot{V} < 0$). If such a function exists, the origin is stable. This is the foundation of nearly all nonlinear stability analysis.

For LQR, the optimal cost $V(x) = x^T P x$ *is* a Lyapunov function — the Riccati equation guarantees it decreases. This is one of the most elegant results in control theory: optimality implies stability.

**Passivity** (1970s–). If the plant is passive (it doesn't generate energy), and the controller is passive, the interconnection is stable. Passivity-based control designs controllers to shape the energy function, especially for mechanical and electrical systems.

**Small-gain theorem** (1960s). If the loop gain is less than 1, the closed loop is stable. This generalizes the Bode intuition (gain margin) to nonlinear and interconnected systems. Used heavily in robust control.

**Barrier functions** (2010s–). For safety-critical systems, stability isn't enough — you need to guarantee the state never enters an unsafe region. Control Barrier Functions (CBFs) constrain the control input to keep the system in the safe set. Combined with Control Lyapunov Functions (CLFs), they produce controllers that are both stable and safe — used in autonomous driving, legged robots, drone swarms.

### Why this matters

Stability is non-negotiable. Aerospace, medical devices, autonomous vehicles — failure means death. The progression from "works in simulation" to "provably safe" is the hardest and most important step in controller design. It's also where control theory earns its keep: without Lyapunov, you're just hoping.

---

## 9. Computation — the controller has limits too

### The problem

The controller runs on real hardware with finite resources:

- **Time:** the control loop has a deadline (1 ms, 100 µs, 10 µs)
- **Memory:** embedded microcontrollers have kilobytes, not gigabytes
- **Power:** battery-powered devices can't run heavy optimization
- **Numerical precision:** fixed-point DSPs, single-precision floats

A controller that's mathematically perfect but takes 10 ms to compute on a 1 ms loop is useless. Computation is a constraint on the controller itself.

### The solution spectrum

**Offline pre-computation.** LQR's appeal is that the Riccati equation is solved once offline — the online controller is just $u = -Kx$ (a dot product). This is why LQR runs on anything.

**Explicit MPC** (2002). Pre-solve the QP for *all possible* states offline. The result is a partition of the state space into polyhedral regions, each with a pre-computed affine control law $u = K_i x + b_i$. Online: find which region the state is in (point location), apply the law. Handles constraints with LQR-level online cost. Explodes in complexity for large problems (number of regions grows exponentially with horizon).

**Warm starting.** The state changes by one time step. Last step's optimal control sequence is very close to this step's. Initialize the solver there — often cuts iterations by 80%+. Both OSQP and DAQP support it.

**Efficient solvers.** The choice of QP solver dominates online computation:
- **Active-set methods** (qpOASES, DAQP): few iterations, good for small problems
- **Interior-point methods** (CVXOPT, ECOS): more iterations but predictable
- **Operator splitting** (OSQP, SCS): first-order, certifiable, GPU-friendly

**Event-triggered and self-triggered control.** Don't compute at a fixed rate. Compute only when the state has changed enough to warrant it. Reduces average computation by 10–100× in sparse-event regimes.

**Multi-rate control.** Run the fast inner loop (current control) at 100 kHz with a simple PI controller. Run the slow outer loop (position MPC) at 100 Hz. The fast loop hides the slow loop's latency.

### Why this matters

The gap between "proven in MATLAB" and "running on a microcontroller" is where most academic controllers die. Real-time implementation is a first-class design constraint, not an afterthought. This is why PID survived for a century — it costs essentially zero computation.

---

## 10. Multi-objective trade-offs — you can't have everything

### The problem

Every controller balances competing objectives:

- **Speed vs overshoot** — faster rise means more overshoot (waterbed effect in linear systems)
- **Tracking vs energy** — aggressive tracking burns power (LQR's Q vs R encodes this)
- **Robustness vs nominal performance** — designing for worst-case uncertainty sacrifices best-case performance
- **Simplicity vs optimality** — PID is simple but suboptimal; nonlinear MPC is near-optimal but complex
- **Exploration vs exploitation** — in learning-based control, trying new actions risks poor immediate performance

These are not just implementation details. They are incommensurable — there is no single number that captures "goodness." You must choose where to sit on each Pareto frontier.

### The solution: make the trade-offs explicit

**LQR weights** (Q, R) are the most elegant encoding of a trade-off in control theory. $Q$ penalizes state error, $R$ penalizes control effort. The ratio $Q/R$ determines aggressiveness. The Riccati solution finds the mathematically optimal compromise. You don't need to *design* the compromise — you declare what you care about, and the math returns the best possible controller given those priorities.

**Multi-objective MPC.** Instead of a scalar cost, optimize over multiple objectives with explicit weighting. Some formulations use lexicographic ordering (safety first, then performance, then comfort). Others use constrained optimization (minimize energy subject to tracking error ≤ ε).

**H₂ / H∞ trade-off.** H₂ optimizes nominal performance (expected behavior). H∞ optimizes worst-case robustness. Mixed H₂/H∞ designs balance the two — good nominal performance with guaranteed robustness to a specified uncertainty bound.

**The fundamental problem:** no controller can transcend the trade-offs imposed by the plant physics. Bode's integral formula says the integral of the sensitivity function is conserved — reducing error at one frequency increases it at another. The waterbed is a law of nature, not an engineering limitation. Good controller design is not about eliminating trade-offs. It's about making them visible and choosing deliberately.

---

## 11. How these problems interact

The problems compound. Each one makes the others harder:

| Scenario | Measurement | Inertia | Constraints | Model uncert. | Disturb. | Nonlinearity | What you need |
|----------|-------------|---------|-------------|---------------|----------|-------------|---------------|
| Fast motor, good encoder, oversized amp | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | PID |
| Same, noisy encoder | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | PI + filter, or LQG |
| Heavy arm, good encoder | ✗ | ✓ | ✗ | ✗ | ✗ | ✗ | LQR with feedforward |
| Heavy arm, variable load | ✗ | ✓ | ✗ | ✗ | ✓ | ✗ | LQR + disturbance observer |
| Motor with tight voltage limit | ✗ | ✗ | ✓ | ✗ | ✗ | ✗ | MPC or PID + anti-windup |
| Aging battery (varying params) | ✗ | ✗ | ✓ | ✓ | ✗ | ✗ | Adaptive MPC |
| Wind turbine (flexible, gusty) | ✓ | ✓ | ✓ | ✓ | ✓ | ✗ | Gain-scheduled robust MPC |
| Rocket landing | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | Nonlinear MPC + EKF |

### Where the separation principle breaks

The separation principle (estimator + controller designed independently = stable combined system) holds for linear systems. It breaks when:

- **Constraints are active** — the controller didn't apply what it said. The estimator assumes it did. Estimate diverges.
- **The model is wrong** — the estimator and controller use the same wrong model. Errors correlate and amplify.
- **The system is nonlinear** — there is no general separation principle for nonlinear systems.

This is why constrained MPC + Kalman filter can fail despite both components being perfectly tuned in isolation. And why robust MPC and moving horizon estimation (MHE) are active research areas — they address the interaction explicitly.

---

## 12. The evolutionary arc

Control theory didn't evolve randomly. Each generation is a response to a growing set of problems it can handle:

| Generation | Meas. | Inertia | Constraints | Model uncert. | Disturb. | Nonlinearity | Key tools |
|------------|-------|---------|-------------|---------------|----------|-------------|-----------|
| **PID** (1920s–) | Assumes clean | D-term damping | Anti-windup (patch) | Manual retuning | Integrator | None | 3-term error feedback |
| **LQR/LQG** (1960s–) | Kalman filter | Riccati → optimal damping | None | Guaranteed margins (automatic) | Integrator (LQI) | None | State-space, Riccati |
| **Robust** (1980s–) | H∞ estimation | Structured uncertainty | Included in design | **Core focus** | H∞ disturbance atten. | Small-gain theorem | H∞ synthesis, μ-synthesis |
| **Adaptive** (1950s–90s) | Online param est. | Adjusts to changing inertia | Can adapt to sat. limits | **Core focus** | Estimates & cancels | Some classes (MRAC) | Online estimation, MRAC |
| **QP-MPC** (1980s–) | Separate estimator | Prediction over horizon | **Core focus** | Min-max variants | Measured disturb. model | None (linear only) | Receding-horizon QP |
| **Nonlinear MPC** (2000s–) | Moving horizon est. | Nonlinear prediction | Nonlinear constraints | Robust NMPC variants | Disturbance models | **Core focus** | DDP, SQP, real-time iter. |
| **Learning-based** (2010s–) | Learned observation | Learned dynamics | Learned safety filters | Learned uncertainty sets | Learned disturbance patterns | **Handles implicitly** | RL, GP-MPC, neural ODEs |

The progression is toward handling more problems simultaneously, paying with computation. 1960 couldn't solve a QP at 1 kHz. 2020 can. The question is always: *given the problems my system actually has, what's the simplest controller that handles them?*

Most systems don't need the full stack. A temperature controller with slow dynamics and loose specs is fine with PI. An automotive engine needs gain-scheduled MPC. A rocket landing needs everything.

---

## 13. Further reading

**General:**
- Åström, K.J. & Murray, R.M. (2021). *Feedback Systems.* Princeton. — The best single-volume introduction to the full landscape.

**Measurement:**
- Simon, D. (2006). *Optimal State Estimation.* Wiley. — Kalman filter, EKF, UKF, particle filters.
- Smith, O.J.M. (1957). "Closer control of loops with dead time." *Chemical Engineering Progress.*

**Model uncertainty:**
- Zhou, K., Doyle, J.C., Glover, K. (1996). *Robust and Optimal Control.* Prentice-Hall. — H∞, μ-synthesis, the definitive reference.
- Åström, K.J. & Wittenmark, B. (2013). *Adaptive Control.* Dover. — MRAC, self-tuning regulators.

**Disturbances:**
- Han, J. (2009). "From PID to Active Disturbance Rejection Control." *IEEE Trans. Industrial Electronics.*
- Chen, W.H. et al. (2016). "Disturbance Observer-Based Control." *IEEE Trans. Industrial Electronics.*

**Nonlinearity:**
- Khalil, H.K. (2015). *Nonlinear Control.* Pearson. — Lyapunov, feedback linearization, sliding mode, backstepping.
- Slotine, J.J. & Li, W. (1991). *Applied Nonlinear Control.* Prentice-Hall.

**Constraints:**
- Rawlings, J.B., Mayne, D.Q., Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design.* Nob Hill.

**Stability & safety:**
- Sontag, E.D. (1998). *Mathematical Control Theory.* Springer. — Lyapunov theory, input-to-state stability.
- Ames, A.D. et al. (2019). "Control Barrier Functions." *Annual Review of Control, Robotics, and Autonomous Systems.*

**This project (related docs):**
- `bellman_to_lqr.md` — DP to Riccati derivation
- `care_vs_dare.md` — discrete vs continuous Riccati
- `from_lp_to_qp_to_lqr.md` — the optimization engine under MPC
- `nonlinear_mpc.md` — what happens when dynamics go nonlinear
- `trajectory_tracking_lqr_mpc.md` — tracking vs regulation
- `poles_zeros_ode.md` — the transfer-function view of dynamics
