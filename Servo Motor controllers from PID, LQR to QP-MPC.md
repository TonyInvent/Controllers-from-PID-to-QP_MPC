# Servo Motor Controllers: From PID, LQR to QP-MPC

**A podcast transcript — tracing the century-long evolution of feedback control through the lens of a DC motor servo.**

---

## Episode Outline

| Slide | Section |
|-------|---------|
| 0 | Prologue — A field guide to this project |
| 1 | Title Card — The servo motor problem |
| 2 | Act I: PID (1910s–1940s) — Intuition becomes mathematics |
| 3 | Interlude — Poles, Zeros & ODEs: the math that bridges intuition to optimality |
| 4 | Act II: State-Space & LQR (1958–1960) — The Space Race rewrites control theory |
| 5 | Act III: MPC & QP-MPC (1970s–1990s) — Constraints break optimality, and computers fix it |
| 6 | Beyond Linearity — Nonlinear MPC and the current frontier |
| 7 | Epilogue — The motor on your desk |
| 8 | Appendix — One motor, five ways to control it |
| 9 | References & Further Reading |

---

## Slide 0 — Prologue: A Field Guide to This Project

**Visual:** A grid of cards — the welcome page of the project. Five simulators fade in: PID Explorer, Servo Motor PID, LQR Explorer, QP-MPC Explorer, Zero-Effect Explorer. Then a "Deep Dives" shelf slides in: Poles/Zeros & ODEs, Trajectory Tracking, Nonlinear MPC. Finally the two podcast covers: "Servo Motor Controllers" and "The Century of Feedback."

---

**NARRATOR:**

Before we start the story, a word about what you're looking at.

This repository started as three simulators — one for PID, one for LQR, one for QP-MPC — and a podcast script to accompany them. It has since grown into something closer to a small control-theory curriculum.

The simulators are still at the center:
- **PID Explorer** — a 2nd-order plant where you drag Kp, Kd, Ki and watch the pole-zero map update in real time. Build intuition before touching real motors.
- **Servo Motor PID** — the same PID, now on a brushed DC motor with real parameters: resistance, inductance, torque constant, inertia, friction. Voltage saturation, anti-windup, disturbance torque injection.
- **LQR Explorer** — tune cost weights Q and R instead of gains. The CARE solver returns the mathematically optimal state-feedback matrix. Overlay PID to see how classical tuning compares to optimal control.
- **QP-MPC Explorer** — hard constraints enter the optimization. A QP solver finds the best control sequence that respects voltage and current limits, and a parallel LQR simulation proves the constrained optimum is structurally different from simply saturating LQR.
- **Zero-Effect Explorer** — deep-dive into why derivative action speeds things up, why RHP zeros impose hard bandwidth limits, and how multiple zeros compound.

Surrounding the simulators are **Deep Dives** — markdown explainers that go deeper than any simulator can:
- *Poles, Zeros & Differential Equations* — every transfer function is a differential equation in disguise. Learn why a pole at −σ + jω means oscillation at ω rad/s with decay envelope e^(−σt).
- *Trajectory Tracking with LQR & MPC* — regulation asks "push to zero"; tracking asks "follow this moving target." The feedback gain doesn't change at all.
- *Beyond Linearity: Nonlinear MPC* — what happens when x_{k+1} = f(x_k, u_k) instead of Ax_k + Bu_k. DDP, SQP, and the tools that replace Riccati equations.

And there are two podcast scripts:
- This one — *Servo Motor Controllers: From PID, LQR to QP-MPC* — tracing the century of feedback through one motor.
- *The Century of Feedback* — a 50-minute deep-dive into the 250-year history, from James Watt's flyballs to SpaceX's rocket-landing QP-MPC.

Everything in this project is self-contained. Every simulator is a single HTML file — no build step, no server, no framework. Open any `.html` in a browser and it runs. The idea is that you shouldn't need to install anything to understand control theory.

This script traces the central story. The rest is there when you're ready to go deeper.

---

## Slide 1 — Title Card

**Visual:** A brushed DC motor spinning. Three labels appear beside it — *PID*, *LQR*, *QP-MPC* — each fading into the next. Subtitle: "One motor. One tuning problem. A hundred years of ideas."

---

**NARRATOR:**

If you want to make a DC motor turn to a specific angle and hold it there — a servo — you have to answer one question: given where the shaft is now and where you want it to be, what voltage should you apply?

It sounds simple. But answering that question well has driven a century of control theory. Three generations of engineers looked at the same motor and came up with three radically different answers — not because the motor changed, but because *what they had available* changed. The math they knew. The computers they could touch. The problems they were actually being paid to solve.

This is the story of those three answers. PID. LQR. QP-MPC. And the historical currents that carried each one ashore.

---

## Slide 2 — Act I: PID (1910s–1940s)

### Intuition becomes mathematics

**Visual:** A ship's steering engine, circa 1910. Then a pneumatic PID controller from a 1930s refinery. Overlay: the three-term equation `u = Kp·e + Ki·∫e dt + Kd·de/dt`.

---

**NARRATOR:**

The earliest feedback controllers weren't designed — they were *discovered*. A nineteenth-century millwright would adjust a float valve and notice that too much gain made the water level oscillate. He didn't know why. He just backed off the screw until it stopped.

The first systematic analysis came from observing ships. In the 1860s, James Clerk Maxwell — yes, that Maxwell — wrote a paper called "On Governors." He showed that a mechanical feedback loop could be described by differential equations, and that stability was about where the roots of those equations landed. This was radical: *feedback had a mathematics.*

But the three-term controller we now call PID didn't crystallize until the 1910s and 1920s. The driving force was **marine autopilots**. Ships needed to hold a course for hours. A human helmsman could do it, but he fatigued. The Sperry Gyroscope Company built mechanical autopilots that implemented what we'd now recognize as proportional and derivative action — responding to both heading error and rate of turn. Later, integral action was added to correct persistent offsets from wind and current.

The name "PID" and the tuning rules that made it accessible came later still. In 1942, John Ziegler and Nathaniel Nichols — two engineers at Taylor Instrument Companies in Rochester, New York — published their famous tuning paper. Their insight was pragmatic: *most industrial processes can be approximated by a first-order lag plus dead time, and their PID gains can be read off a simple step-response experiment.* No transfer function needed. No differential equation. Just bump the plant, measure the reaction, and look up the numbers in a table.

This was the killer feature of PID. Not optimality. Not mathematical elegance. **Accessibility.** A technician with a stopwatch could tune a controller. And for the overwhelming majority of industrial loops — temperature, pressure, flow, level — it was good enough.

The limitations, though, were baked in from the start:

- **It's single-input, single-output.** PID has no native way to coordinate multiple actuators or balance competing objectives. You can tune for position or you can tune for velocity, but you can't express *both* as explicit design goals.
- **It's reactive.** The derivative term gives you some anticipation, but PID has no model of the plant's future. It cannot plan ahead.
- **Constraints are a hack.** If your motor amplifier saturates at ±12V, PID will keep piling up integral error during saturation — the integrator "winds up" — and then overshoot catastrophically when the actuator comes out of saturation. The fix, anti-windup clamping, is an admission that the core algorithm doesn't know the actuator exists.

There's also a deeper question that PID alone can't answer: *why* does derivative action reduce oscillation? *Why* does adding a zero to the controller speed up the response? These are questions about transfer functions and their zeros — and answering them requires going below the surface of PID to the differential equations underneath. The **Zero-Effect Explorer** (`zero_effect_explorer.html`) in this project lets you see exactly this: place zeros in the left or right half-plane and watch how the step response changes. A left-half-plane zero pulls the response faster (it's "derivative-like"), while a right-half-plane zero causes the output to initially move in the *wrong* direction — a non-minimum-phase limitation that no amount of tuning can fix.

By the 1950s, PID was everywhere. And that's exactly when a completely different problem blew the doors off control theory.

---

## Slide 3 — Interlude: Poles, Zeros & the Differential Equation

### The math that bridges intuition to optimality

**Visual:** A transfer function $G(s) = \omega_n^2/(s^2 + 2\zeta\omega_n s + \omega_n^2)$ written on a blackboard. An arrow connects it to the ODE $\ddot{y} + 2\zeta\omega_n\dot{y} + \omega_n^2 y = \omega_n^2 u$. Then the same ODE, but now with PID substituted in — the coefficients change, the poles migrate, the step response reshapes.

---

**NARRATOR:**

Before we leave PID for LQR, we need to face something. The PID Explorer in this project shows you poles moving on a map and step responses changing shape. You can develop real intuition this way — "increasing Kd damps the oscillation" — but intuition without mechanism is fragile. The question *why* has an answer, and it lives in differential equations.

Every transfer function is a differential equation in disguise. When you see:

$$G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$

the $s$ is the Laplace-domain stand-in for $d/dt$. Cross-multiply and reverse the transform and you get:

$$\ddot{y}(t) + 2\zeta\omega_n\,\dot{y}(t) + \omega_n^2\,y(t) = \omega_n^2\,u(t)$$

The poles of $G(s)$ are the roots of the left-hand side's characteristic equation — they determine the homogeneous solution, the natural modes of the system. A pole at $-\sigma$ contributes a mode $e^{-\sigma t}$. A pair at $-\sigma \pm j\omega$ contributes $e^{-\sigma t}\sin(\omega t + \phi)$. **The real part is the decay envelope. The imaginary part is the oscillation frequency.**

The zeros of $G(s)$ are what the right-hand side does to the input. A zero at $-z$ means the forcing term involves $\dot{u} + zu$ — the derivative of the input. Zeros don't create modes; they determine *how strongly each mode is excited*. This is why derivative action speeds things up: it adds zeros that shift energy toward faster-decaying components through impulse-like kicks at $t = 0$.

When you drag the $K_p$ slider in the PID Explorer and watch $\zeta_{\text{eff}}$ drop, you are watching the discriminant of the characteristic equation turn negative. The math is:

$$\lambda_{1,2} = -(\zeta\omega_n + \tfrac{1}{2}K_d\omega_n^2) \pm \sqrt{(\zeta\omega_n + \tfrac{1}{2}K_d\omega_n^2)^2 - \omega_n^2(1+K_p)}$$

As $K_p$ grows, the term under the square root goes negative, the roots become complex, and oscillation appears. As $K_d$ grows, the real part becomes more negative, the decay accelerates, and oscillation is suppressed. **This is not magic. It is algebra.** A deeper treatment — with all the steps — is in `poles_zeros_ode.md` in this project.

Why does this matter for our story? Because LQR is about to replace "drag the sliders until it looks right" with "declare what you care about and let the math compute the optimal answer." The mental leap from PID to LQR is exactly the leap from *tuning* to *optimization*. And you can't make that leap unless you first understand that the sliders were always moving the roots of a differential equation.

---

## Slide 4 — Act II: State-Space & LQR (1958–1960)

### The Space Race rewrites control theory

**Visual:** Sputnik beeping in orbit (1957). A Saturn V on the pad. Kalman at his blackboard. The Riccati equation. Transition to a state-space block diagram: `ẋ = Ax + Bu`, `y = Cx`.

---

**NARRATOR:**

On October 4, 1957, the Soviet Union put a radio beacon in orbit. Sputnik didn't *do* much — it beeped — but it meant Soviet rockets could deliver a payload anywhere on Earth. The American response was immediate and well-funded. The problem was that you couldn't guide a rocket to the Moon with PID.

The fundamental issue was that a rocket is **MIMO** — multiple inputs, multiple outputs — and its dynamics are coupled. You can't tune the pitch channel independently of the yaw channel because they interact through the airframe and through the actuator saturations. And you can't tune them sequentially because the cross-coupling means the second loop destabilizes the first.

Worse, the objective wasn't "minimize overshoot." It was something more formal: *given limited fuel and a target trajectory, find the control history that minimizes the integrated squared error.* This is an **optimization problem**, not a tuning problem.

The mathematical foundations dropped fast. Bellman's dynamic programming principle (1957). Pontryagin's maximum principle (1956, translated to English in 1962). And in 1960, Rudolf Kalman published a paper that redefined the field.

Kalman's contribution was twofold. First, he showed that the natural language for control systems is the **state-space representation**: `ẋ = Ax + Bu`, with all the dynamics encoded in the matrices A and B, and all the outputs as linear combinations of the state. This was general. PID was a special case — a 1×1 controller architecture. State-space could describe a rocket with 20 states and 5 actuators just as naturally.

Second, he posed the control problem as an explicit cost minimization. Choose the control `u(t)` to minimize a quadratic cost:

```
J = ∫ (xᵀQx + uᵀRu) dt
```

where Q penalizes state error and R penalizes control effort. The solution is a linear state feedback **u = −Kx**, where K is computed by solving an algebraic Riccati equation — the CARE. This is the **Linear Quadratic Regulator**.

The Riccati equation had been known since the 1720s — Count Riccati studied it in the context of differential equations — but Kalman repurposed it as the engine of optimal control. Solving it gives you a gain matrix K that is *provably optimal* for the specified Q and R. There is no tuning in the traditional sense. You declare what you care about — position accuracy, control effort, integrator action — as weights, and the math returns the best possible gains. The resulting closed-loop system has guaranteed stability margins: at least 60° phase margin and infinite gain margin in each channel.

This is what flew the Apollo missions. The LQR was used for the lunar module's descent and ascent guidance. Not PID. The problem demanded optimality, MIMO coordination, and mathematical guarantees — and LQR delivered all three.

**A natural extension: trajectory tracking.** Standard LQR solves the *regulation* problem — drive the state to zero. But a motor servo tracks a position command, a drone follows a waypoint path, a robot arm traces a desired trajectory. These are *tracking* problems: given a reference $\{r_0, r_1, \ldots, r_N\}$, find controls that make $x_k \approx r_k$. The good news: the feedback gain $K_k$ is identical to the regulation case. You only add a feedforward term $u_k^{\text{ff}}$ that translates the reference into control space. Define the tracking error $e_k = x_k - r_k$, and the same LQR machinery applies. Tracking adds surprisingly little machinery on top of regulation — but it unlocks almost every real-world application. The explainer `trajectory_tracking_lqr_mpc.md` in this project walks through the derivation step by step.

But LQR has a blind spot, and it's a big one: **constraints.** The formulation `min ∫ (xᵀQx + uᵀRu)` assumes that u can be anything. It can't. Real motors have voltage limits. Real amplifiers saturate. Real currents must stay below thermal limits. LQR gives you the unconstrained optimal gain, and if that gain commands 30V from a 12V supply, the answer is simply wrong.

Some extensions — LQI (LQR with integral action), anti-windup heuristics — patch the problem. But they're patches. LQR's mathematical purity cracks when it meets physics. And a whole industry was about to feel that crack acutely.

---

## Slide 4 — Act III: MPC & QP-MPC (1970s–1990s)

### Constraints break optimality, and computers fix it

**Visual:** An oil refinery at night — miles of pipes and distillation columns. A control room filled with CRT monitors. A graph showing a long prediction horizon with constraints drawn as horizontal lines. The QP formulation: `min ½UᵀHU + fᵀU s.t. u_min ≤ u_k ≤ u_max`.

---

**NARRATOR:**

While aerospace was busy with LQR, the process industries — oil refineries, chemical plants, paper mills — had a different problem. These plants run continuously. They're slow (time constants of minutes to hours). They're MIMO with strong interactions. And most importantly, **they live on their constraints.**

A refinery makes the most money when it's pushed right up against its physical limits — maximum temperature, maximum pressure, maximum throughput. The economically optimal operating point is almost always at a constraint boundary. If your controller can't explicitly handle constraints, you have to back off to a "safe" operating point, and every degree of backoff is lost profit.

The academic control theory of the 1960s — elegant, optimal, unconstrained — was useless for this. Plant operators knew it. They kept using PID, tuned conservatively, leaving money on the table.

The breakthrough came from industry, not academia. In 1978, Jacques Richalet published a paper on "Model Predictive Heuristic Control." The name tells you the philosophy: **model** (you use a dynamic model of the plant), **predictive** (you simulate forward over a horizon to see where you're going), **heuristic** (the implementation was pragmatic, not mathematically pristine). Around the same time, Charlie Cutler and Brian Ramaker at Shell Oil developed Dynamic Matrix Control (DMC), which used step-response models (easy to obtain from plant data) and solved a constrained optimization online.

The key insight was this: instead of computing one set of fixed gains (like LQR), **solve an optimization problem at every time step.** Use a model to predict the plant's response over a finite horizon. Find the sequence of control moves that minimizes the cost while respecting all constraints. Apply the first move. Then repeat — re-measure, re-optimize, re-apply. This is **receding-horizon control**, and it's the core of MPC.

Why did this happen in the 1970s and not earlier? Three preconditions converged:

1. **Computers became fast enough.** Solving a constrained quadratic program (QP) online — at every sample time — requires real-time optimization. A 10-step horizon with voltage constraints means solving a 10-variable QP. By the late 1970s, minicomputers could do this for slow processes. By the 1990s, microprocessors could do it for fast electromechanical systems. By the 2010s, you could run MPC on a motor servo at kilohertz rates.

2. **The QP solvers matured.** The constrained QP at the heart of MPC is a convex optimization problem. Efficient algorithms — active-set methods, interior-point methods, and fast gradient projection — were developed through the 1980s and 1990s. Today, tailored solvers like qpOASES and OSQP can solve small QPs in microseconds.

3. **There was money in it.** Shell and other process companies funded MPC development because it directly increased profit margins. Academic interest followed industrial success, not the other way around.

What does MPC give you that LQR doesn't?

- **Hard constraint handling.** You specify `u_min` and `u_max`, and the optimizer guarantees each control move respects them. No saturation, no windup, no patches. If the optimal unconstrained solution would exceed a limit, the controller finds the *constrained optimum* — the best you can do without violating physics.

- **Preview and feedforward.** Because MPC simulates the model forward over the horizon, it can see reference changes coming and start moving *before* the error appears. Feedforward is baked into the formulation, not bolted on.

- **Constraint boundary economics.** When constraints are inactive, QP-MPC reproduces the LQR solution exactly. When a constraint becomes active, the controller shifts to a different optimal trajectory — one that rides the limit — and then smoothly returns to the unconstrained control law when the constraint relaxes. This is automatic. No mode switching. No gain scheduling.

The tradeoff is computational. LQR is a one-time matrix solve — compute K once, apply forever. QP-MPC requires solving a QP at every sample time. For a DC motor servo, the QP is small enough that a modern microcontroller handles it easily. For a process plant with hundreds of variables and constraints, specialized QP solvers running on dedicated hardware are standard.

But QP-MPC — for all its power — still assumes a *linear* model: $x_{k+1} = A x_k + B u_k$. What happens when that assumption breaks? Rocket aerodynamics at high angle of attack. Chemical reactors with Arrhenius kinetics. Walking robots with contact dynamics. These are nonlinear, and the QP formulation — with its fixed Hessian, its convexity guarantee, its one-shot Riccati factorization — no longer applies. The next frontier is nonlinear MPC.

---

## Slide 6 — Beyond Linearity: Nonlinear MPC

### The current frontier

**Visual:** A smooth nonlinear manifold replacing the linear hyperplane. A trajectory snaking through it. DDP iterations visualized — forward rollout, backward Riccati-like sweep (now state-dependent). Labels: DDP, iLQR, SQP, Real-Time Iteration.

---

**NARRATOR:**

Everything so far — PID, LQR, QP-MPC — assumed the plant is linear: $\dot{x} = Ax + Bu$. That assumption bought us a lot. Closed-form Riccati solutions. Convex quadratic programs with guaranteed global optimality. A Hessian matrix independent of the state, factorized once and reused forever.

Reality is nonlinear. And when $x_{k+1} = f(x_k, u_k)$ with nonlinear $f$, all three pillars collapse:

1. **The cost is no longer quadratic in the control sequence.** With linear dynamics, condensing produced $J = \frac{1}{2}U^T H U + f^T U$. With nonlinear dynamics, the state trajectory becomes a nonlinear function of $U$, and the cost inherits that nonlinearity. No Riccati recursion can decompose it.

2. **The QP is no longer convex.** $H \succeq 0$ is not guaranteed. Local minima become possible. The solver can get stuck.

3. **The Hessian depends on the state.** You can't factorize it once. Every time step brings a new linearization, a new Hessian, and a new QP — or you abandon QP entirely.

The field responded with three classes of methods:

**DDP / iLQR (Differential Dynamic Programming / iterative LQR).** Instead of condensing into a single QP, you linearize the dynamics and quadratize the cost *around the current trajectory guess*, then solve the resulting LQR problem backward (the Riccati sweep) and roll forward with a line search. Repeat. Each iteration is an LQR problem, but the A and B matrices are recomputed at each time step along the trajectory. This is the workhorse of modern robotics — Boston Dynamics uses variants of this for their walking controllers.

**SQP (Sequential Quadratic Programming).** Treat the full nonlinear optimal control problem as a nonlinear program. At each iteration, form a QP approximation — linearize constraints, quadratize the Lagrangian — solve the QP for a step direction, take the step (with merit functions and line search), and repeat. SQP is more general than DDP (it handles arbitrary equality and inequality constraints, not just dynamics), but heavier.

**Real-Time Iteration (RTI).** The key insight: in a receding-horizon setting, you don't need to converge fully at each time step. The plant is moving. One or two SQP iterations per sample time, warm-started from the previous solution shifted by one step, are often enough to track a trajectory. RTI trades per-iteration accuracy for temporal consistency — and for fast systems, it's the only option.

Nonlinear MPC has landed real hardware. Chemical reactors. Autonomous racing drones that re-plan trajectories at 100 Hz. Rocket landing — SpaceX's Grasshopper and Falcon 9 boosters use nonlinear MPC for the terminal landing phase, where aerodynamics and throttling nonlinearities can't be ignored.

The frontier is moving fast. Learned dynamics (fitting $f$ from data using neural networks). Reinforcement learning as approximate stochastic optimal control. But those are stories for another podcast. The explainer `nonlinear_mpc.md` in this project picks up where this slide leaves off.

---

## Slide 7 — Epilogue

### The motor on your desk

**Visual:** The same DC motor from Slide 1. Three controller blocks arranged in a row: PID, LQR, QP-MPC. Below each: *Trial-and-error tuning*, *Optimal unconstrained gains*, *Constrained online optimization*. The QP-MPC block lights up. Caption: "Each generation's answer to the same question."

---

**NARRATOR:**

So why did it take a century to figure out how to control a motor?

It didn't. The motor was never the hard part. The hard part was knowing what question to ask.

PID asks: *what error correction feels right?* It's fast, intuitive, and surprisingly robust. It powered the industrial revolution and still controls 90-plus percent of all feedback loops today. If your process is single-loop, well-behaved, and constraints are loose, PID is the right answer.

LQR asks: *what control policy is mathematically optimal?* It emerged from the Space Race because rockets were MIMO, MIMO demanded state-space, and state-space naturally led to optimization. If your system has a good linear model and constraints aren't tight, LQR gives you provable optimality with guaranteed stability margins.

QP-MPC asks: *what is the best I can do within my physical limits?* It emerged from the process industries because oil refineries live on their constraints, and leaving slack costs real money. If your system has hard limits — voltage, current, torque, temperature — QP-MPC explicitly respects them while staying as close to optimal as physics allows.

Nonlinear MPC asks: *what if the physics itself is nonlinear?* When aerodynamics, chemical kinetics, or contact dynamics make $Ax + Bu$ a poor approximation, you need DDP, SQP, or real-time iteration to find a control sequence that respects the true dynamics. It's the current frontier — computationally heavier, but necessary when linearity is the wrong assumption.

Each controller is the right answer for the problem as it was understood in its era. PID was enabled by mechanical feedback and empirical tuning in the age of steam. LQR was enabled by state-space theory and the Cold War's appetite for optimal guidance. QP-MPC was enabled by cheap computation and the process industry's need to operate profitably at constraint boundaries. Nonlinear MPC is enabled by faster solvers, automatic differentiation, and the demands of robots, rockets, and autonomous systems that can't be linearized away.

The motor hasn't changed. But we have. And this project — the simulators, the deep-dives, the podcast you're reading — is meant to be a hands-on path through that entire evolution. Start with `pid_explorer.html`. End with `nonlinear_mpc.md`. In between, you'll have touched every major idea in modern control theory, on one motor, with no installation required.

---

## Slide 8 — Appendix: One Motor, Five Ways to Control It

**Visual:** A grid of five simulation panels from the project's simulators. Top row: PID Explorer (pole-zero map + step response), Servo Motor PID (3rd-order motor with voltage traces), LQR Explorer (Q/R tuning with CARE solver). Bottom row: QP-MPC Explorer (hard constraint boundaries visible as horizontal limits), Zero-Effect Explorer (step responses under different zero placements). Overlay caption: "Five interactive simulators. One motor. No installation required."

---

### Project File Map

| File | What it is | Open with |
|---|---|---|
| `welcome.html` | Landing page — links to everything | Browser |
| `pid_explorer.html` | PID on 2nd-order plant with pole-zero map | Browser |
| `servo_motor_pid.html` | PID on DC motor with real physics (R, L, Kt, J, B) | Browser |
| `lqr_explorer.html` | LQR/LQI optimal control with CARE solver | Browser |
| `servo_qp_mpc.html` | QP-MPC with hard V/I constraints, LQR comparison | Browser |
| `zero_effect_explorer.html` | How zeros shape step response (LHP, RHP, multiple) | Browser |
| `poles_zeros_ode.md` | Poles and zeros explained through differential equations | Text / `mdviewer.html` |
| `trajectory_tracking_lqr_mpc.md` | Tracking vs. regulation, feedforward design | Text / `mdviewer.html` |
| `nonlinear_mpc.md` | Beyond linearity: DDP, SQP, real-time iteration | Text / `mdviewer.html` |
| `The Century of Feedback - A History of Control Theory.md` | 50-min deep-dive into 250 years of feedback control | Text / `mdviewer.html` |

### Controller Comparison Table

| | PID | LQR / LQI | QP-MPC | Nonlinear MPC |
|---|---|---|---|---|
| **Era** | 1910s–1940s | 1958–1960 | 1970s–1990s | 1990s–present |
| **Driven by** | Industrial automation, marine navigation | Space Race, missile guidance | Process industries (oil, chemicals) | Robotics, rocketry, autonomous systems |
| **Key insight** | Three-term error feedback is good enough for most loops | Optimal control as a quadratic minimization with guaranteed margins | Receding-horizon optimization with explicit constraint handling | Sequential linearization + LQR solves handle nonlinear dynamics |
| **Math** | Heuristic tuning (Ziegler-Nichols) | Riccati equation (CARE/DARE) | Online quadratic programming | DDP, SQP, real-time iteration |
| **MIMO?** | No (SISO) | Yes | Yes | Yes |
| **Constraints?** | Anti-windup patches | Saturation (destroys optimality) | Hard constraints, guaranteed satisfaction | Hard constraints on linearized subproblems |
| **Model** | None (error only) | Linear state-space | Linear state-space | Nonlinear state-space $x_{k+1}=f(x_k,u_k)$ |
| **Computation** | Negligible | One-time matrix solve | QP solved every sample time | Multiple QPs + line search every sample time |
| **Best for** | Simple, well-behaved loops | MIMO systems with loose constraints | Systems operating near physical limits | Systems where linearity is the wrong assumption |
| **The motor answer** | "Adjust Kp until it looks right" | "Here's the mathematically optimal gain" | "Here's the best I can do given the 12V supply" | "Here's the best trajectory given the true nonlinear physics" |

---

## Slide 9 — References & Further Reading

### Foundational papers

1. **Ziegler, J.G. & Nichols, N.B. (1942).** "Optimum Settings for Automatic Controllers." *Transactions of the ASME.* — The paper that gave us PID tuning rules.

2. **Kalman, R.E. (1960).** "Contributions to the Theory of Optimal Control." *Boletín de la Sociedad Matemática Mexicana.* — The birth of LQR and the Kalman filter, in the same year, by the same author.

3. **Richalet, J. et al. (1978).** "Model Predictive Heuristic Control: Applications to Industrial Processes." *Automatica.* — The paper that launched MPC as an industrial practice.

4. **Cutler, C.R. & Ramaker, B.L. (1980).** "Dynamic Matrix Control — A Computer Control Algorithm." *Proceedings of the Joint Automatic Control Conference.* — Shell's MPC, the most widely deployed variant in process control.

### MPC and optimization

5. **Qin, S.J. & Badgwell, T.A. (2003).** "A Survey of Industrial Model Predictive Control Technology." *Control Engineering Practice.* — A comprehensive look at MPC's evolution from research to deployed technology.

6. **Maciejowski, J.M. (2002).** *Predictive Control with Constraints.* Prentice Hall. — The standard textbook on MPC theory and implementation.

7. **Boyd, S. & Vandenberghe, L. (2004).** *Convex Optimization.* Cambridge University Press. — The optimization theory that underpins modern QP solvers for MPC.

### Nonlinear MPC and trajectory optimization

8. **Mayne, D.Q. et al. (2000).** "Constrained Model Predictive Control: Stability and Optimality." *Automatica.* — The definitive survey establishing stability and optimality guarantees for MPC.

9. **Tassa, Y., Erez, T. & Todorov, E. (2012).** "Synthesis and Stabilization of Complex Behaviors through Online Trajectory Optimization." *IROS.* — iLQR / DDP for real-time nonlinear control; the approach behind much of modern legged robotics.

10. **Diehl, M. et al. (2002).** "Real-Time Optimization and Nonlinear Model Predictive Control of Processes Governed by Differential-Algebraic Equations." *Journal of Process Control.* — The real-time iteration (RTI) scheme that makes nonlinear MPC fast enough for online use.

11. **Rawlings, J.B., Mayne, D.Q. & Diehl, M. (2017).** *Model Predictive Control: Theory, Computation, and Design.* Nob Hill. — The modern comprehensive MPC textbook, covering linear and nonlinear MPC.

---

*This document accompanies the interactive simulators and explainers in the `Controllers-from-PID-to-QP_MPC` repository. Start with `pid_explorer.html`, move to `lqr_explorer.html` and `servo_qp_mpc.html`, and end with `nonlinear_mpc.md`. The historical progression is also the pedagogical one.*
