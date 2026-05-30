# The difference between CARE and DARE

**Two algebraic Riccati equations, one idea — why the equations look different, when to use each, and how they connect.**

---

## 1. What they are

Both are steady-state solutions to the Linear Quadratic Regulator problem. The only difference is the time domain.

| | Discrete-time | Continuous-time |
|---|---|---|
| **Equation** | **DARE** | **CARE** |
| **Cost** | $J = \sum x_k^T Q x_k + u_k^T R u_k$ | $J = \int (x^T Q x + u^T R u)\, dt$ |
| **Dynamics** | $x_{k+1} = A x_k + B u_k$ | $\dot{x} = A x + B u$ |

The Riccati equation is what you get when you run Bellman's principle of optimality backwards to steady-state, assuming a quadratic value function $V(x) = x^T P x$. The time domain determines the algebraic form.

---

## 2. Side by side

**DARE** (Discrete-time Algebraic Riccati Equation):

$$P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q$$

$$K = (R + B^T P B)^{-1} B^T P A$$

**CARE** (Continuous-time Algebraic Riccati Equation):

$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

$$K = R^{-1} B^T P$$

---

## 3. Why the forms differ

The difference traces directly to how "next state" works in each domain.

### Discrete: predict one step ahead

The Bellman equation in discrete time is:

$$V_k(x) = \min_u \Big[ x^T Q x + u^T R u + V_{k+1}(Ax + Bu) \Big]$$

The next state $x_{k+1} = Ax + Bu$ gets wrapped in $V_{k+1}$, producing $A^T P A$ — the state transitions through $A$ **before** hitting the cost matrix $P$. The control cost denominator gets a $B^T P B$ term because the control affects the next state, which then incurs cost through $P$.

### Continuous: instantaneous rate

In continuous time, you take the limit $\Delta t \to 0$. The value function satisfies a differential equation:

$$-\dot{P} = A^T P + P A - P B R^{-1} B^T P + Q$$

As $t \to \infty$, $\dot{P} \to 0$, leaving $A^T P + P A$. The $P A$ term comes from differentiating $x^T P x$ — it's the instantaneous rate of change, not a step-ahead prediction. There is no $A^T P A$ because in continuous time there is no "one step ahead" — the dynamics $\dot{x} = Ax + Bu$ are instantaneous.

---

## 4. The control gain tells the story

| | Gain formula | Why |
|---|---|---|
| **DARE** | $K = (R + B^T P B)^{-1} B^T P A$ | Control affects the next state through $A$, so $A$ appears explicitly |
| **CARE** | $K = R^{-1} B^T P$ | Control affects $\dot{x}$ instantaneously, no $A$ needed |

The DARE gain asks: "What control now minimizes cost *plus* the cost of the state I'll land in next?" — so $A$ appears because $x_{k+1} = Ax_k + Bu_k$.

The CARE gain asks: "What control instantaneously counteracts the state's drift?" — no transition matrix needed.

---

## 5. How they connect: the $\Delta t \to 0$ limit

You can derive the CARE from the DARE by taking the sampling time to zero.

Let $A_d \approx I + A \Delta t$ and $B_d \approx B \Delta t$ (forward Euler discretization). Substitute into the DARE:

- $A_d^T P A_d \approx P + (A^T P + P A) \Delta t + O(\Delta t^2)$
- $A_d^T P B_d \approx P B \Delta t + O(\Delta t^2)$
- $B_d^T P B_d \approx B^T P B \Delta t^2$

As $\Delta t \to 0$:
- $A^T P A$ collapses to $P + (A^T P + P A)\Delta t$ — the $O(1)$ term cancels, leaving $A^T P + P A$
- $B^T P B$ shrinks to zero relative to $R$, so $(R + B^T P B)^{-1} \to R^{-1}$
- $A$ in the feedback path $B^T P A \to B^T P$

The DARE gain converges to the CARE gain: $K_{\text{dare}} \to R^{-1} B^T P = K_{\text{care}}$.

---

## 6. Which one should you use?

**In practice, use DARE.** Every real control system today runs on digital hardware — microcontrollers, DSPs, FPGAs. Even when the plant is a continuous physical system (motor, robot, drone, chemical process), the controller is a discrete-time algorithm executing at a fixed sample rate. You discretize the plant model and solve the DARE on the discrete matrices.

CARE still matters in three scenarios:

| Scenario | Why CARE? |
|---|---|
| **Academic analysis** | Continuous-time proofs are often cleaner — eigenvalues of Hamiltonian matrices, stability margin guarantees, passivity arguments |
| **Simulation and education** | It's natural to formulate the motor + controller in continuous time and simulate together with an ODE solver (`lqr_explorer.html` does this) |
| **Initial design, then discretize** | Design the LQR in continuous time where intuition is easier (pole locations, bandwidth, damping ratio), then discretize the resulting $K$ for implementation |

But if you're writing firmware for a real product — motor drive, drone flight controller, satellite attitude control — you'll solve the DARE on the discretized plant. That's the equation that matches what the microcontroller actually does: sample, compute, apply, repeat.

In this project:

- **`servo_qp_mpc.py`** uses DARE — ZOH-discretized motor at 1 ms, MPC solves a discrete QP with the DARE $P$ as terminal cost. This is what a real implementation looks like.
- **`lqr_explorer.html`** uses CARE (via Newton-Kleinman iteration) — the design and simulation are formulated in continuous time for pedagogical clarity, solved alongside RK4 integration.

---

## 7. Further reading

- **Bellman to LQR** (`bellman_to_lqr.md`) — walks through the DP recurrence that produces both equations; Sections 4 and 5 cover the DARE-to-CARE transition.
- **Anderson, B.D.O. & Moore, J.B. (1990).** *Optimal Control: Linear Quadratic Methods.* — Chapters 2–3 cover continuous and discrete Riccati equations in depth.
- **Bertsekas, D.P. (2012).** *Dynamic Programming and Optimal Control, Vol. I.* — Chapter 4 treats discrete-time LQR rigorously; Appendix A covers the continuous-time limit.
