# Servo Motor Controllers: From PID, LQR to QP-MPC

**A podcast transcript — tracing the century-long evolution of feedback control through the lens of a DC motor servo.**

---

## Episode Outline

| Slide | Section |
|-------|---------|
| 1 | Title Card — The servo motor problem |
| 2 | Act I: PID (1910s–1940s) — Intuition becomes mathematics |
| 3 | Act II: State-Space & LQR (1958–1960) — The Space Race rewrites control theory |
| 4 | Act III: MPC & QP-MPC (1970s–1990s) — Constraints break optimality, and computers fix it |
| 5 | Epilogue — The motor on your desk |

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

By the 1950s, PID was everywhere. And that's exactly when a completely different problem blew the doors off control theory.

---

## Slide 3 — Act II: State-Space & LQR (1958–1960)

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

---

## Slide 5 — Epilogue

### The motor on your desk

**Visual:** The same DC motor from Slide 1. Three controller blocks arranged in a row: PID, LQR, QP-MPC. Below each: *Trial-and-error tuning*, *Optimal unconstrained gains*, *Constrained online optimization*. The QP-MPC block lights up. Caption: "Each generation's answer to the same question."

---

**NARRATOR:**

So why did it take a century to figure out how to control a motor?

It didn't. The motor was never the hard part. The hard part was knowing what question to ask.

PID asks: *what error correction feels right?* It's fast, intuitive, and surprisingly robust. It powered the industrial revolution and still controls 90-plus percent of all feedback loops today. If your process is single-loop, well-behaved, and constraints are loose, PID is the right answer.

LQR asks: *what control policy is mathematically optimal?* It emerged from the Space Race because rockets were MIMO, MIMO demanded state-space, and state-space naturally led to optimization. If your system has a good linear model and constraints aren't tight, LQR gives you provable optimality with guaranteed stability margins.

QP-MPC asks: *what is the best I can do within my physical limits?* It emerged from the process industries because oil refineries live on their constraints, and leaving slack costs real money. If your system has hard limits — voltage, current, torque, temperature — QP-MPC explicitly respects them while staying as close to optimal as physics allows.

Each controller is the right answer for the problem as it was understood in its era. PID was enabled by mechanical feedback and empirical tuning in the age of steam. LQR was enabled by state-space theory and the Cold War's appetite for optimal guidance. QP-MPC was enabled by cheap computation and the process industry's need to operate profitably at constraint boundaries.

The motor hasn't changed. But we have.

---

## Slide 6 — Appendix: One Motor, Three Controllers

**Visual:** Side-by-side simulation traces from the three explorers in this repository. Position tracking, voltage, and current. The LQR trace exceeding constraints while QP-MPC clips cleanly at ±Vmax. PID oscillating slightly. Caption: "PID: tuned by hand. LQR: optimal but unconstrained. QP-MPC: optimal and constraint-aware."

---

### Summary Table

| | PID | LQR / LQI | QP-MPC |
|---|---|---|---|
| **Era** | 1910s–1940s | 1958–1960 | 1970s–1990s |
| **Driven by** | Industrial automation, marine navigation | Space Race, missile guidance | Process industries (oil, chemicals) |
| **Key insight** | Three-term error feedback is good enough for most loops | Optimal control as a quadratic minimization with guaranteed margins | Receding-horizon optimization with explicit constraint handling |
| **Math** | Heuristic tuning (Ziegler-Nichols) | Riccati equation (CARE/DARE) | Online quadratic programming |
| **MIMO?** | No (SISO) | Yes | Yes |
| **Constraints?** | Anti-windup patches | Saturation (destroys optimality) | Hard constraints, guaranteed satisfaction |
| **Computation** | Negligible | One-time matrix solve | QP solved every sample time |
| **Best for** | Simple, well-behaved loops | MIMO systems with loose constraints | Systems operating near physical limits |
| **The motor answer** | "Adjust Kp until it looks right" | "Here's the mathematically optimal gain" | "Here's the best I can do given the 12V supply" |

---

## Slide 7 — References & Further Reading

1. **Ziegler, J.G. & Nichols, N.B. (1942).** "Optimum Settings for Automatic Controllers." *Transactions of the ASME.* — The paper that gave us PID tuning rules.

2. **Kalman, R.E. (1960).** "Contributions to the Theory of Optimal Control." *Boletín de la Sociedad Matemática Mexicana.* — The birth of LQR and the Kalman filter, in the same year, by the same author.

3. **Richalet, J. et al. (1978).** "Model Predictive Heuristic Control: Applications to Industrial Processes." *Automatica.* — The paper that launched MPC as an industrial practice.

4. **Cutler, C.R. & Ramaker, B.L. (1980).** "Dynamic Matrix Control — A Computer Control Algorithm." *Proceedings of the Joint Automatic Control Conference.* — Shell's MPC, the most widely deployed variant in process control.

5. **Qin, S.J. & Badgwell, T.A. (2003).** "A Survey of Industrial Model Predictive Control Technology." *Control Engineering Practice.* — A comprehensive look at MPC's evolution from research to deployed technology.

6. **Maciejowski, J.M. (2002).** *Predictive Control with Constraints.* Prentice Hall. — The standard textbook on MPC theory and implementation.

7. **Boyd, S. & Vandenberghe, L. (2004).** *Convex Optimization.* Cambridge University Press. — The optimization theory that underpins modern QP solvers for MPC.

---

*This document accompanies the interactive simulators in the `zero-effect-explorer` repository: `servo_motor_pid.html`, `lqr_explorer.html`, and `servo_qp_mpc.html`. Start with PID, move to LQR, and end with QP-MPC — the historical progression is also the pedagogical one.*
