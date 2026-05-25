# From LP to QP to LQR: A History of Optimization in Control

**LP solves "find the cheapest way." QP solves "find the cheapest way, but prefer small moves." LQR solves "find the best sequence of moves for a dynamic system." MPC connects them all by reducing LQR to a QP at every time step.**

---

## 1. The optimization landscape

Imagine you're solving three problems:

1. **A shipping problem.** You have 5 factories and 200 warehouses. Each factory→warehouse route has a different cost per ton. How many tons should each factory ship to each warehouse to meet demand at minimum total cost?

2. **A robot reaching for an object.** The robot has 7 joints. You want the end-effector to reach a target pose, but you also want the joint movements to be small (no jerky motion) and the joints must not hit their angle limits.

3. **A motor servo tracking a trajectory.** You apply voltage, the shaft moves. You want minimal position error and minimal control effort over the next 100 time steps. The amplifier saturates at ±12V. What voltage sequence do you apply?

Problem 1 is a **Linear Program (LP)**. Problem 2 is a **Quadratic Program (QP)**. Problem 3 is a **finite-horizon LQR** — and, as we'll see, it can be converted into a QP.

These three problem classes form a hierarchy of increasing expressiveness. LP is the simplest. QP adds a quadratic term that encodes "prefer small adjustments." LQR is QP applied to a dynamic system over time — and when constraints enter, it becomes the QP at the heart of Model Predictive Control.

This document traces that hierarchy, from Dantzig's simplex method in 1947 to modern QP-based MPC and robot inverse kinematics.

---

## 2. Linear Programming (LP)

### 2.1 The standard form

A linear program in standard form looks like this:

$$\begin{aligned}
\min_x \quad & c^T x \\
\text{s.t.} \quad & A x = b \\
& x \geq 0
\end{aligned}$$

where $x \in \mathbb{R}^n$ is the decision variable, $c$ is the cost vector, $A \in \mathbb{R}^{m \times n}$ defines equality constraints, and $b \in \mathbb{R}^m$ is the right-hand side. The constraint $x \geq 0$ means each component of $x$ is non-negative.

More general LPs allow inequality constraints $A x \leq b$ and unrestricted variables, but these can always be converted to standard form (by adding slack variables and splitting free variables into positive and negative parts).

**The defining feature of LP:** both the objective and all constraints are *linear* in $x$. There are no squares, no products, no exponentials — just dot products.

### 2.2 Historical: Dantzig and the simplex method (1947)

During World War II, the US military faced massive logistics problems — literally the "ship supplies to the front" problem from Section 1. These were formulated as linear programs, but no one knew how to solve them efficiently.

In 1947, George Dantzig — then a mathematical advisor to the US Air Force — developed the **simplex method**. His key geometric insight: the feasible set of an LP is a **convex polytope** (a high-dimensional polyhedron), and if an optimal solution exists, at least one optimal solution lies at a **vertex** (corner) of that polytope.

The simplex method exploits this. It starts at one vertex, then "pivots" to an adjacent vertex that improves the objective. It repeats until no improving neighbor exists — at which point the current vertex is globally optimal. The algorithm walks along the edges of the polytope, always moving downhill.

```
Given: A, b, c (LP in standard form)

1. Find an initial basic feasible solution (a vertex).
2. Compute the "reduced costs" — how much the objective would
   change by moving along each edge away from this vertex.
3. If all reduced costs are ≥ 0: STOP (optimal).
4. Otherwise: pick a negative reduced cost, move along that edge
   until hitting a boundary (another vertex), go to step 2.
```

The simplex method typically solves LPs with thousands of variables and constraints in practice, despite having exponential worst-case complexity. Its average-case performance is remarkably good — it typically takes $O(m)$ to $O(3m)$ iterations, where $m$ is the number of constraints.

Dantzig's simplex method made LP the workhorse of operations research. By the 1950s, oil companies were using it to schedule refineries. Airlines used it for crew scheduling. It remains one of the most important algorithms ever invented.

### 2.3 Geometric intuition

Consider a simple LP in two variables:

$$\begin{aligned}
\max_{x_1, x_2} \quad & 3x_1 + 2x_2 \\
\text{s.t.} \quad & x_1 + x_2 \leq 4 \\
& 2x_1 + x_2 \leq 5 \\
& x_1, x_2 \geq 0
\end{aligned}$$

The feasible region is a polygon in the $(x_1, x_2)$ plane. The objective $3x_1 + 2x_2$ defines a family of parallel lines. The optimal solution is the point in the polygon that touches the "highest" line — always a vertex.

This geometric picture generalizes to $n$ dimensions: the feasible set is a convex polytope, the objective is a hyperplane, and the optimum (if unique) is a vertex. The simplex method is systematic vertex-hopping.

> **Companion demo:** `lp_geometry_demo.py` plots this exact LP — the feasible polygon, objective contours, the simplex pivot path O → A → B, and an inset zoom showing the KKT optimality condition at the optimum vertex. Run it with `.venv/Scripts/python.exe lp_geometry_demo.py`; output goes to `lp_geometry_demo.png`.

### 2.4 Interior point methods (Karmarkar, 1984)

In 1984, Narendra Karmarkar at Bell Labs published a fundamentally different algorithm for LP. Instead of walking along the boundary from vertex to vertex, his **interior point method** cuts through the middle of the polytope.

The idea: start at a point strictly inside the feasible region. Transform the problem so the current point is at the "center" of the polytope (using a projective transformation). Take a step in the direction of steepest descent. Transform back. Repeat.

Karmarkar proved polynomial-time convergence — $O(n^{3.5} L)$ where $L$ is the bit-length of the input — and, more provocatively, claimed his method beat simplex on large problems.

This sparked a revolution. Variants like Mehrotra's predictor-corrector method (1992) became the foundation of modern LP solvers. Today, both simplex and interior-point solvers coexist: simplex excels at resolving (re-optimizing after small data changes), interior point excels at large, sparse problems solved from scratch.

> **Companion demo:** `interior_point_demo.py` implements the primal logarithmic-barrier method on the same LP — tracing the central path from the analytic centre $(0.76, 1.12)$ to the LP optimum $(1, 3)$ as $t \to \infty$, with Newton centering steps shown at intermediate $t$. Run it with `.venv/Scripts/python.exe interior_point_demo.py`; output goes to `interior_point_demo.png`.

### 2.5 LP is static

A critical observation: **LP solves a static problem.** It finds a single vector $x$ that minimizes $c^T x$ subject to constraints. There is no time. No dynamics. No "what happens after I apply this decision." LP is a snapshot — optimal resource allocation at one instant.

This is both its power and its limitation. For the shipping problem, it's perfect — you just need one allocation. For controlling a motor over time, you need something more.

---

## 3. Quadratic Programming (QP)

### 3.1 What QP adds

A quadratic program generalizes LP by adding a quadratic term to the objective:

$$\begin{aligned}
\min_x \quad & \frac{1}{2} x^T H x + c^T x \\
\text{s.t.} \quad & A x = b \\
& G x \leq h
\end{aligned}$$

where $H \in \mathbb{R}^{n \times n}$ is symmetric and (for convex QP) positive semidefinite. The constraints remain linear — only the objective gains curvature.

If $H = 0$, this is exactly an LP. So **QP strictly generalizes LP.**

### 3.2 Why the quadratic term matters

The linear term $c^T x$ says "push in this direction." The quadratic term $\frac{1}{2} x^T H x$ says "but not too far — the further you go, the more it costs."

This encodes a fundamental preference: **small adjustments are better than large ones.** In the robot example, $c^T x$ encodes the end-effector error (reach the target), while $x^T H x$ penalizes large joint movements (move smoothly). The QP finds the balance.

Concretely, if $H$ is diagonal, each term $\frac{1}{2} H_{ii} x_i^2$ grows quadratically with $x_i$. The optimizer will spread the "effort" across multiple variables rather than letting any one variable dominate — a property called **regularization**.

### 3.3 LP vs QP: a visual contrast

Consider minimizing $f(x) = \frac{1}{2} x^2 + c x$ subject to $-1 \leq x \leq 1$.

- If the linear term $c$ is large and negative ($c = -5$), the optimum is at the constraint boundary $x = 1$. The quadratic term "wants" $x = 0$, but the linear term overpowers it all the way to the wall.

- If $c$ is small ($c = -0.5$), the unconstrained minimum would be at $x = 0.5$ — inside the feasible region. The quadratic curvature stops the solution from hitting the boundary.

This is the key behavior: **LP solutions live on constraint boundaries. QP solutions can live in the interior.** LP is "all or nothing" — the optimum is always at a vertex. QP allows "some, but not too much" — the optimum can be anywhere in the feasible set, determined by the trade-off between the linear push and the quadratic pull-back.

This matters enormously in control. An LP controller would bang against limits constantly. A QP controller only touches limits when necessary.

### 3.4 Solving QPs

The solution methods for QP parallel those for LP, with added complexity from the quadratic term:

**Active-set methods** generalize simplex. At each iteration, they guess which inequality constraints are "active" (binding as equalities), solve an equality-constrained QP (which reduces to a linear system via the KKT conditions), then update the active set. These are the workhorses for small-to-medium QPs where warm-starting matters.

**Interior point methods** generalize Karmarkar's LP algorithm to QP. They replace inequality constraints with logarithmic barrier terms added to the objective, then trace the "central path" as the barrier weight decreases. These excel at large, sparse QPs.

**Operator splitting methods** (ADMM, OSQP) decompose the problem into simpler subproblems and iterate. OSQP (Stellato et al., 2020) uses an alternating-direction method that's particularly fast for the QPs arising in MPC — it can solve small problems in microseconds and handles warm-starting well.

**Active-set for small QPs** include DAQP (dual active-set QP), explicitly designed for small-to-medium embedded QPs. It uses a dual active-set approach that works directly with box constraints — exactly the structure that appears in MPC with input limits. Unlike general-purpose solvers, DAQP exploits the fact that MPC constraints are often simple bounds $u_{\min} \leq u_k \leq u_{\max}$.

---

## 4. LQR as dynamic optimization

### 4.1 Brief recap: Bellman → Riccati → u = -Kx

The Linear Quadratic Regulator solves:

$$\begin{aligned}
\min_{u_0, \ldots, u_{N-1}} \quad & x_N^T Q_f x_N + \sum_{k=0}^{N-1} \left( x_k^T Q x_k + u_k^T R u_k \right) \\
\text{s.t.} \quad & x_{k+1} = A x_k + B u_k
\end{aligned}$$

This is **dynamic optimization** — the decision variables $(u_0, \ldots, u_{N-1})$ are linked through the dynamics constraint. Each $u_k$ affects not just the cost at step $k$, but all future states $x_{k+1}, x_{k+2}, \ldots$ through the recurrence $x_{k+1} = A x_k + B u_k$.

Bellman's principle of optimality decomposes this into a backwards recursion, yielding the Riccati equation and the linear feedback law $u_k = -K_k x_k$. For the infinite-horizon case, $K$ is constant and computed once. (See `bellman_to_lqr.md` for the full derivation.)

### 4.2 The unconstrained assumption

The LQR derivation has no inequality constraints. The optimal $u_k = -K_k x_k$ can be any real number. If $K$ demands 100V from a 12V amplifier, LQR doesn't care — it gives you the mathematically optimal answer for the *unconstrained* problem.

There is no term in the Riccati equation for $u_{\min}$ or $u_{\max}$. The cost function $J = \sum (x^T Q x + u^T R u)$ is purely quadratic; the minimization over $u$ is unconstrained and solved by setting a gradient to zero. Constraints break this closed-form solution.

---

## 5. Why $u = -Kx$ is rarely used directly in practice

### 5.1 Constraints are everywhere

Every physical system has limits:

| Limit | What it constrains |
|-------|-------------------|
| Amplifier saturation | $u_{\min} \leq u_k \leq u_{\max}$ |
| Slew rate | $|u_{k+1} - u_k| \leq \Delta u_{\max}$ |
| Current limit | $|i| \leq i_{\max}$ (state constraint) |
| Position limits | $\theta_{\min} \leq \theta_k \leq \theta_{\max}$ |
| Thermal limits | $\sum |u_k|^2 \leq P_{\text{thermal}}$ |

LQR ignores all of them. The gain matrix $K$ is computed assuming the control can take any value. When it can't, the closed-loop behavior degrades — sometimes catastrophically.

The naive fix is saturation: compute $u = -Kx$, then clip it to $[u_{\min}, u_{\max}]$. But this has two problems:

1. **Windup:** The integrator (in LQI) keeps accumulating error during saturation, then overshoots when the actuator comes out of saturation.
2. **The controller doesn't know it's saturated.** It computes $u$ as if unlimited authority exists, so it doesn't plan around the constraint. The result is overshoot and oscillation — as demonstrated in `servo_qp_mpc.py`.

### 5.2 Model accuracy is never perfect

$A$ and $B$ are models, not reality. The motor has cogging torque, friction nonlinearities, temperature-dependent resistance, and unmodeled dynamics. The LQR gain $K$ is optimal for the model — but on the real hardware, it may perform worse than a hand-tuned PID that was adjusted empirically.

This gap between model and reality is why adaptive and robust control exist. But it also motivates MPC: **by re-solving the optimization at every time step using the latest measurement, MPC can compensate for model error.** The receding-horizon feedback provides a degree of robustness that the open-loop optimal $u = -Kx$ lacks.

### 5.3 Infinite vs. finite horizon

The "textbook" LQR — $u = -Kx$ with constant $K$ — is infinite-horizon. It assumes the cost is accumulated over $t \in [0, \infty)$. This is mathematically elegant but practically limiting:

- Real tasks have finite duration (move to this position in 1 second).
- Real references change (the motor doesn't just regulate to zero).
- Real objectives change (tracking vs. regulation vs. path following).

Finite-horizon LQR with time-varying $K_k$ is what's actually needed. And finite-horizon LQR is exactly what MPC solves — with the addition of constraints.

---

## 6. From LQR to QP: Condensing

### 6.1 The key insight

Consider the discrete-time finite-horizon LQR problem with $N$ steps:

$$\begin{aligned}
\min_{u_0, \ldots, u_{N-1}} \quad & x_N^T P x_N + \sum_{k=0}^{N-1} \left( x_k^T Q x_k + u_k^T R u_k \right) \\
\text{s.t.} \quad & x_{k+1} = A x_k + B u_k
\end{aligned}$$

The decision variables are $(u_0, \ldots, u_{N-1})$ — because, given $x_0$ and the controls, the states are fully determined by the dynamics. We can **eliminate the state variables** and express the entire cost purely as a quadratic function of the control sequence.

This is called **condensing.** It converts the dynamic optimization into a static QP in $N \cdot m$ variables (where $m$ is the number of control inputs).

### 6.2 The math: state prediction over the horizon

Given initial state $x_0$ and control sequence $U = [u_0^T, u_1^T, \ldots, u_{N-1}^T]^T$, we can write the state at every future step:

$$\begin{aligned}
x_1 &= A x_0 + B u_0 \\
x_2 &= A^2 x_0 + A B u_0 + B u_1 \\
x_3 &= A^3 x_0 + A^2 B u_0 + A B u_1 + B u_2 \\
&\vdots \\
x_N &= A^N x_0 + A^{N-1} B u_0 + \cdots + B u_{N-1}
\end{aligned}$$

Stack all predicted states into $\mathbf{X} = [x_1^T, \ldots, x_N^T]^T$. In matrix form:

$$\mathbf{X} = \mathcal{A} x_0 + \mathcal{B} U$$

where $\mathcal{A} \in \mathbb{R}^{N n \times n}$ and $\mathcal{B} \in \mathbb{R}^{N n \times N m}$ are the **condensed prediction matrices:**

$$\mathcal{A} = \begin{bmatrix} A \\ A^2 \\ \vdots \\ A^N \end{bmatrix}, \qquad
\mathcal{B} = \begin{bmatrix}
B & 0 & \cdots & 0 \\
A B & B & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
A^{N-1} B & A^{N-2} B & \cdots & B
\end{bmatrix}$$

$\mathcal{B}$ is block lower-triangular — causality in matrix form. $u_k$ only affects states $x_{k+1}$ and later, never earlier states.

### 6.3 Substituting into the cost

The cost function in terms of $\mathbf{X}$ and $U$:

$$J = x_0^T Q x_0 + \mathbf{X}^T \bar{Q} \mathbf{X} + U^T \bar{R} U$$

where $\bar{Q} = \text{blkdiag}(Q, Q, \ldots, Q, P)$ and $\bar{R} = \text{blkdiag}(R, R, \ldots, R)$ are block-diagonal matrices repeating the stage cost matrices $N$ times, with terminal cost $P$ in the last block of $\bar{Q}$.

Substitute $\mathbf{X} = \mathcal{A} x_0 + \mathcal{B} U$:

$$\begin{aligned}
J &= x_0^T Q x_0 + (\mathcal{A} x_0 + \mathcal{B} U)^T \bar{Q} (\mathcal{A} x_0 + \mathcal{B} U) + U^T \bar{R} U \\
&= x_0^T Q x_0 + x_0^T \mathcal{A}^T \bar{Q} \mathcal{A} x_0 + 2 x_0^T \mathcal{A}^T \bar{Q} \mathcal{B} U + U^T (\mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R}) U
\end{aligned}$$

The terms $x_0^T Q x_0$ and $x_0^T \mathcal{A}^T \bar{Q} \mathcal{A} x_0$ don't depend on $U$, so they can be dropped from the minimization. We get:

$$\min_U \quad \frac{1}{2} U^T H U + (F^T x_0)^T U$$

where:

$$H = 2 (\mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R}), \qquad F = 2 \mathcal{B}^T \bar{Q} \mathcal{A}$$

This is a **standard QP** in $N \cdot m$ variables. The Hessian $H$ is $N m \times N m$ and positive definite (since $R \succ 0$). The linear term $F^T x_0$ encodes the initial state — as $x_0$ changes at each time step, only the linear term changes, not $H$.

### 6.4 Adding constraints

Now the crucial step: adding box constraints on the controls:

$$u_{\min} \leq u_k \leq u_{\max}, \quad k = 0, \ldots, N-1$$

These are just linear inequality constraints on $U$. The full QP becomes:

$$\begin{aligned}
\min_U \quad & \frac{1}{2} U^T H U + (F^T x_0)^T U \\
\text{s.t.} \quad & u_{\min} \leq u_k \leq u_{\max}, \quad k = 0, \ldots, N-1
\end{aligned}$$

This is what the QP solver actually solves at each MPC step. When no constraints are active, the solution is $U^* = -H^{-1} F^T x_0$, and $u_0^*$ exactly matches the LQR gain $u_0 = -K_0 x_0$. When constraints become active, the solver finds the constrained optimum — the best you can do within the limits.

You can also add state constraints ($x_{\min} \leq x_k \leq x_{\max}$) and rate constraints ($|u_{k+1} - u_k| \leq \Delta u_{\max}$), all as linear inequalities on $U$.

### 6.5 Condensing in code

The Python implementation in `servo_qp_mpc.py:118-138` does exactly this:

```python
# Prediction matrices (condensed)
A_aug = [A; A^2; ...; A^N]          # stacked vertically
B_aug = lower-triangular as above    # N blocks × N blocks

# Block-diagonal cost matrices
Qbar = blkdiag(Q, Q, ..., Q, P)      # terminal P in last block
Rbar = blkdiag(R, R, ..., R)

# QP matrices
H = B_aug.T @ Qbar @ B_aug + Rbar
H = 0.5 * (H + H.T)                  # ensure symmetry
F = A_aug.T @ Qbar @ B_aug           # maps x0 into linear cost term

# At each time step, solve:
#   min  0.5 * U^T H U + (F^T x0)^T U
#   s.t. u_min <= U <= u_max
```

$H$ is computed once (it doesn't depend on $x_0$). Each time step, only the linear term $F^T x_0$ changes. This is the key efficiency: the QP's Hessian is fixed, so solvers can factorize $H$ once and reuse the factorization.

### 6.6 This is MPC

The condensed QP is the computational engine of **Model Predictive Control:**

```
At each time step k:
  1. Measure (or estimate) current state x_k
  2. Solve the condensed QP:  min ½U^T H U + (F^T x_k)^T U
                               s.t. constraints
  3. Apply only u*_0 (the first control in the optimal sequence)
  4. k ← k+1, go to step 1
```

This is **receding-horizon control.** You plan a full $N$-step trajectory, but you only execute the first step. Then you re-measure, re-optimize, and re-apply. The feedback from re-measuring provides robustness to model error. The re-optimization ensures the control policy adapts to the actual state, not the predicted one.

Without constraints, MPC exactly reproduces LQR. With constraints, MPC extends LQR into the physically realistic regime where limits exist.

---

## 7. QP in robot inverse kinematics

### 7.1 The IK problem as optimization

A robot arm with $n$ joints has configuration $q \in \mathbb{R}^n$. The forward kinematics map joint angles to end-effector pose: $p = f(q)$ where $p \in \mathbb{R}^6$ (position + orientation).

**Inverse kinematics (IK)** asks: given a desired end-effector pose $p^*$, find joint angles $q$ such that $f(q) = p^*$.

For redundant manipulators ($n > 6$), there are infinitely many solutions. You need to pick one — and this is where QP enters.

### 7.2 Formulating IK as a QP

Linearize the forward kinematics around the current configuration $q_0$:

$$f(q) \approx f(q_0) + J(q_0) \cdot \Delta q$$

where $J = \frac{\partial f}{\partial q}$ is the $6 \times n$ Jacobian. The IK problem becomes: find $\Delta q$ such that $J \Delta q = p^* - f(q_0)$.

This is an underdetermined linear system (more variables than equations). The QP formulation:

$$\begin{aligned}
\min_{\Delta q} \quad & \frac{1}{2} \Delta q^T W \Delta q \\
\text{s.t.} \quad & J \Delta q = \Delta p \\
& q_{\min} - q_0 \leq \Delta q \leq q_{\max} - q_0
\end{aligned}$$

where $W$ is a weighting matrix (often diagonal — heavier joints move less) and the box constraints enforce joint limits. This is a **convex QP** — and it's structurally identical to the MPC problem, just with a different $H$ and constraints.

### 7.3 Mink and DAQP

[Mink](https://github.com/kevinzakka/mink) is a Python library for inverse kinematics that formulates IK as a constrained optimization problem. Its default solver is **DAQP** (Dual Active-set Quadratic Programming) — a specialized QP solver for small-to-medium problems with simple constraints.

DAQP is well-suited to IK because:

- **IK QPs are small.** A 7-DOF arm has 7 variables. The Jacobian is $6 \times 7$. The QP is tiny by optimization standards.
- **Constraints are mostly bounds.** Joint limits are box constraints $q_{\min} \leq q \leq q_{\max}$. These are the simplest possible QP constraints and can be handled very efficiently by dual active-set methods.
- **Warm-starting is critical.** At each control cycle (often 100 Hz–1 kHz), the QP is solved from the previous solution. The active set changes slowly. DAQP exploits this.

The connection to MPC is direct: both solve a convex QP with box constraints at each time step, both use warm-starting from the previous solution, and both need microsecond-level solve times for real-time operation.

---

## 8. The big picture

### 8.1 A unified view

| Class | Objective | Constraints | Time | Solution lies at |
|-------|-----------|-------------|------|-----------------|
| **LP** | Linear | Linear | Static | A vertex |
| **QP** | Quadratic + Linear | Linear | Static | Anywhere in feasible set |
| **LQR** (infinite-horizon) | Quadratic (dynamic) | None | Dynamic (solved once) | $u = -Kx$, linear feedback |
| **MPC** (condensed QP) | Quadratic (dynamic) | Linear (box/inequality) | Dynamic (solved each step) | Receding-horizon constrained optimum |

The progression is one of increasing generality:

- **LP → QP:** The objective gains curvature, allowing "small moves preferred" regularization. Solutions are no longer forced to vertices.
- **Static QP → LQR:** The optimization spans a time horizon. The decision is no longer a single vector but a sequence linked by dynamics. Eliminating the states gives a static QP (condensing).
- **LQR → MPC:** The optimization becomes online and constrained. At each time step, a condensed QP is solved. Constraints are handled explicitly, not patched around.

### 8.2 LP and QP are the workhorses of static optimization

Nearly every resource allocation, scheduling, portfolio optimization, or network flow problem is an LP or QP. LPs are solved by simplex or interior-point methods. QPs add the quadratic term that encodes cost-of-adjustment — essential whenever you want smooth, distributed solutions rather than bang-bang extremes.

### 8.3 LQR and MPC are QP applied to dynamic systems

The finite-horizon LQR is a QP in $N \cdot m$ variables — you just don't see it in that form because the Riccati recursion solves it more elegantly for the unconstrained case. But the Riccati approach cannot handle inequality constraints, which is why MPC solves the QP directly.

**LQR is the special case of MPC when constraints are inactive.** This is easy to verify: run an unconstrained QP-MPC and compare $u_0^*$ to $-K_0 x_0$. They are identical.

### 8.4 Why this matters

If you understand LP and QP, you understand the computational engine of modern control. MPC is not a fundamentally new mathematical object — it's a QP, solved online, with the dynamics folded into the Hessian through condensing. The "magic" of MPC is not the optimization (which is standard) but the architecture: model, predict, optimize with constraints, recede.

Similarly, if you understand QP, you understand modern robot IK. The only difference is what $H$, $F$, and the constraints encode — end-effector error vs. state regulation, joint limits vs. voltage limits. The solver doesn't care.

---

## 9. References

1. **Dantzig, G.B. (1947).** "Maximization of a linear function of variables subject to linear inequalities." Chapter XXI in *Activity Analysis of Production and Allocation* (T.C. Koopmans, ed.). — The original simplex method.

2. **Karmarkar, N. (1984).** "A New Polynomial-Time Algorithm for Linear Programming." *Combinatorica.* — The paper that launched interior-point methods.

3. **Wright, S.J. (1997).** *Primal-Dual Interior-Point Methods.* SIAM. — The definitive reference on interior-point algorithms for LP and QP.

4. **Nocedal, J. & Wright, S.J. (2006).** *Numerical Optimization.* Springer. — Comprehensive coverage of optimization algorithms, including QP methods.

5. **Boyd, S. & Vandenberghe, L. (2004).** *Convex Optimization.* Cambridge University Press. — The modern foundation for convex optimization, including LP and QP.

6. **Maciejowski, J.M. (2002).** *Predictive Control with Constraints.* Prentice Hall. — The standard MPC textbook; covers condensing and constrained QP formulation.

7. **Stellato, B., Banjac, G., Goulart, P., Bemporad, A., & Boyd, S. (2020).** "OSQP: An Operator Splitting Solver for Quadratic Programs." *Mathematical Programming Computation.* — The OSQP solver, widely used in embedded MPC.

8. **Arnström, D., Bemporad, A., & Axehill, D. (2022).** "A Dual Active-Set Solver for Embedded Quadratic Programming." *arXiv:2203.02599.* — The DAQP solver used in Mink and other embedded applications.

9. **Zakka, K. et al. (2024).** "Mink: A Python Library for Inverse Kinematics." *GitHub: kevinzakka/mink.* — IK via constrained QP; see in particular the DAQP integration.

10. **Bertsekas, D.P. (2012).** *Dynamic Programming and Optimal Control, Vol. I.* Athena Scientific. — Connects the dots from Bellman through LQR to constrained optimal control.

11. **Borrelli, F., Bemporad, A., & Morari, M. (2017).** *Predictive Control for Linear and Hybrid Systems.* Cambridge University Press. — Modern MPC, including explicit MPC and hybrid systems.

---

*This document is part of the `Controllers-from-PID-to-QP_MPC` repository. For the LQR derivation from Bellman's principle, see `bellman_to_lqr.md`. For the interactive simulators, open `lqr_explorer.html` (unconstrained LQR) and `servo_qp_mpc.html` (constrained QP-MPC). The Python demo `servo_qp_mpc.py` shows the condensing implementation and benchmarks OSQP vs. DAQP.*
