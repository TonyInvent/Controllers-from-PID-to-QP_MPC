# Trajectory Tracking with LQR and MPC

**Regulation asks "push the state to zero." Tracking asks "follow this moving target." The feedback gain doesn't change at all — you just add a feedforward term that translates the reference into the control space.**

---

## 1. The difference between regulation and tracking

Standard LQR solves the **regulation problem**: drive the state to zero while penalizing control effort. The optimal control is $u_k = -K_k x_k$ — pure negative feedback from the current state.

But almost no real system regulates to zero. A motor servo tracks a position command. A drone follows a waypoint path. A robot arm traces a desired end-effector trajectory. These are **tracking problems**: given a reference $\{r_0, r_1, \ldots, r_N\}$, find controls that make $x_k \approx r_k$ for all $k$.

The good news: tracking adds surprisingly little machinery on top of regulation. The feedback gain $K_k$ is identical. You only need a feedforward term.

---

## 2. LQR tracking

### 2.1 Derivation via error coordinates

Given a linear system:

$$x_{k+1} = A x_k + B u_k$$

and a reference trajectory $\{r_0, r_1, \ldots, r_N\}$ that satisfies the same dynamics with some nominal control $u_k^{\text{ff}}$:

$$r_{k+1} = A r_k + B u_k^{\text{ff}}$$

Define the tracking error $e_k = x_k - r_k$ and the control deviation $\tilde{u}_k = u_k - u_k^{\text{ff}}$. Subtracting the reference dynamics from the system dynamics:

$$e_{k+1} = A e_k + B \tilde{u}_k$$

This is identical in form to the original dynamics. The error system with input $\tilde{u}_k$ has the same $A$ and $B$. So the tracking problem on $(x, u)$ becomes a **regulation problem on $(e, \tilde{u})$:**

$$\min_{\tilde{u}_0, \ldots, \tilde{u}_{N-1}} \quad e_N^T P e_N + \sum_{k=0}^{N-1} \big( e_k^T Q e_k + \tilde{u}_k^T R \tilde{u}_k \big)$$

The optimal solution is:

$$\tilde{u}_k = -K_k e_k$$

where $K_k$ comes from the **exact same Riccati recurrence** as the regulation problem. Unfolding back to the original variables:

$$u_k = u_k^{\text{ff}} - K_k (x_k - r_k)$$

### 2.2 The two-term structure

$$\underbrace{u_k}_{\text{applied control}} = \underbrace{u_k^{\text{ff}}}_{\text{feedforward}} - \underbrace{K_k (x_k - r_k)}_{\text{feedback correction}}$$

| Term | Role | Depends on |
|------|------|------------|
| $u_k^{\text{ff}}$ | Nominal control that would keep the system on the reference if everything were perfect | $r_k, r_{k+1}, A, B$ |
| $-K_k (x_k - r_k)$ | Corrects deviations caused by disturbances, initial offsets, or model mismatch | $Q, R$ (the LQR tuning) |

The feedforward does the heavy lifting — it knows where the trajectory is going. The feedback only cleans up the residuals. If the model were perfect and there were no disturbances, the feedback term would be zero and the feedforward alone would track perfectly.

### 2.3 Computing the feedforward

The ideal feedforward requires $r_{k+1} = A r_k + B u_k^{\text{ff}}$, so:

$$u_k^{\text{ff}} = B^+ (r_{k+1} - A r_k)$$

where $B^+$ is the Moore-Penrose pseudoinverse. This is exact when $r_{k+1} - A r_k$ lies in the column space of $B$ (the reference is "dynamically feasible").

For many practical references — e.g., a step change in position — $r_{k+1} - A r_k$ is not in the column space of $B$ exactly. Common workarounds:

- **Steady-state feedforward:** Compute $u_{\text{ss}}$ from the DC gain. For a constant reference $r$, the steady-state condition is $r = A r + B u_{\text{ss}}$, so $u_{\text{ss}} = B^+ (I - A) r$.
- **LQI (integral augmentation):** Augment the state with the integral of the tracking error. The LQR then automatically builds the feedforward into the augmented gain. This is what `lqr_explorer.html` implements for the motor servo.
- **Preview feedforward:** If the full future reference is known, compute $u_k^{\text{ff}}$ from the reference dynamics at each step using the pseudoinverse. This is exact when the reference obeys the dynamics.

### 2.4 Infinite-horizon steady-state case

For constant reference $r$ and infinite-horizon LQR ($K_k \to K$ as $k \to \infty$):

$$u_k = u_{\text{ss}} - K (x_k - r)$$

where $u_{\text{ss}} = B^+ (I - A) r$. The closed-loop system:

$$x_{k+1} = A x_k + B [u_{\text{ss}} - K (x_k - r)] = (A - BK) x_k + B(K r + u_{\text{ss}})$$

At steady state, $x_\infty = r$ (the closed-loop DC gain from $r$ to $x_\infty$ is identity when $u_{\text{ss}}$ is exact, or close to identity with LQI augmentation).

---

## 3. MPC tracking

### 3.1 The regulation QP (recap)

From the condensing derivation in `from_lp_to_qp_to_lqr.md`, the regulation MPC solves:

$$\min_U \quad \frac{1}{2} U^T H U + (F^T x_0)^T U \quad \text{s.t. constr.}$$

where $H = 2(\mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R})$ and $F = 2 \mathcal{B}^T \bar{Q} \mathcal{A}$. $H$ is fixed; only the linear term changes with $x_0$.

### 3.2 Adding the reference

Define the stacked reference $\mathbf{R} = [r_1^T, r_2^T, \ldots, r_N^T]^T \in \mathbb{R}^{N n}$. The tracking cost penalizes $\mathbf{X} - \mathbf{R}$ instead of just $\mathbf{X}$:

$$\begin{aligned}
J &= (\mathbf{X} - \mathbf{R})^T \bar{Q} (\mathbf{X} - \mathbf{R}) + U^T \bar{R} U \\
&= \mathbf{X}^T \bar{Q} \mathbf{X} - 2 \mathbf{R}^T \bar{Q} \mathbf{X} + \mathbf{R}^T \bar{Q} \mathbf{R} + U^T \bar{R} U
\end{aligned}$$

Substitute $\mathbf{X} = \mathcal{A} x_0 + \mathcal{B} U$:

$$\begin{aligned}
J &= (\mathcal{A} x_0 + \mathcal{B} U)^T \bar{Q} (\mathcal{A} x_0 + \mathcal{B} U) - 2 \mathbf{R}^T \bar{Q} (\mathcal{A} x_0 + \mathcal{B} U) + \text{const} + U^T \bar{R} U \\
&= U^T (\mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R}) U + 2 x_0^T \mathcal{A}^T \bar{Q} \mathcal{B} U - 2 \mathbf{R}^T \bar{Q} \mathcal{B} U + \text{(terms indep. of }U\text{)}
\end{aligned}$$

Collecting the linear-in-$U$ terms:

$$J = \frac{1}{2} U^T H U + \big( F^T x_0 - 2 \mathcal{B}^T \bar{Q} \mathbf{R} \big)^T U + \text{const}$$

So the QP becomes:

$$\boxed{\min_U \quad \frac{1}{2} U^T H U + \big( F^T x_0 + g_{\text{ref}} \big)^T U \quad \text{s.t. constraints}}$$

where:

$$g_{\text{ref}} = -2 \,\mathcal{B}^T \bar{Q} \,\mathbf{R}$$

### 3.3 What changes vs. what stays

| Component | Regulation | Tracking | Changed? |
|-----------|-----------|----------|----------|
| $H$ | $2(\mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R})$ | same | **No** |
| $F$ | $2 \mathcal{B}^T \bar{Q} \mathcal{A}$ | same | **No** |
| Linear cost | $F^T x_0$ | $F^T x_0 + g_{\text{ref}}$ | **Yes** — one extra vector |
| Constraints | $u_{\min} \leq U \leq u_{\max}$ | same (or extended) | **Usually no** |

$H$ is the heavy part — it's $N m \times N m$, and factorizing it dominates the solve time. The fact that $H$ does not change when switching from regulation to tracking is the key practical insight: **you pre-factorize $H$ once, and tracking only adds a cheap vector addition to the linear term each time step.**

### 3.4 What $g_{\text{ref}}$ actually does

$g_{\text{ref}} = -2 \mathcal{B}^T \bar{Q} \mathbf{R}$ encodes the entire future reference trajectory's influence on the optimal control sequence. Expanding:

$$\mathcal{B}^T \bar{Q} \mathbf{R} = \begin{bmatrix}
B^T Q r_1 + B^T A^T Q r_2 + B^T (A^2)^T Q r_3 + \cdots \\
B^T Q r_2 + B^T A^T Q r_3 + \cdots \\
\vdots \\
B^T Q r_N
\end{bmatrix}$$

Each component of $g_{\text{ref}}$ sums the effect of all future reference points that a given control $u_k$ can reach through the dynamics. A reference point $r_j$ influences $u_k$ only if $j > k$ (causality: $u_k$ can only affect future states, and the cost compares those future states to their references). The weight decays with the distance $j - k$ through powers of $A$.

Without constraints, the optimal tracking control is $U^* = -H^{-1}(F^T x_0 + g_{\text{ref}})$. The first control $u_0^*$ is:

$$u_0^* = -\underbrace{[H^{-1} F^T]_{0} x_0}_{\text{regulation feedback}} \;-\; \underbrace{[H^{-1}]_{0}\, g_{\text{ref}}}_{\text{feedforward from future reference}}$$

where $[H^{-1}]_0$ denotes the first $m$ rows of $H^{-1}$. This makes explicit that $u_0^*$ is a sum of state feedback and reference feedforward — exactly the two-term structure from the LQR case, but now the feedforward looks ahead over the full horizon.

### 3.5 Code sketch

The only change from regulation MPC is one extra line per time step:

```python
# —— Precompute (once) ——
H = 2 * (B_aug.T @ Qbar @ B_aug + Rbar)
F = 2 * B_aug.T @ Qbar @ A_aug       # same as regulation

# —— Each time step ——
ref_stack = build_reference_stack(k)  # [r_{k+1}, ..., r_{k+N}]
g_ref = -2 * B_aug.T @ Qbar @ ref_stack

prob = QP(H, F.T @ x0 + g_ref, u_min, u_max)   # <— g_ref is the only addition
U_opt = prob.solve()
u = U_opt[0]
```

That's it. The same QP solver. The same Hessian. One extra vector.

---

## 4. State constraints on tracking error

In regulation MPC, state constraints take the form $x_{\min} \leq x_k \leq x_{\max}$. In tracking, you often want the tracking error to stay within bounds: $|x_k - r_k| \leq e_{\max}$, or equivalently $r_k - e_{\max} \leq x_k \leq r_k + e_{\max}$.

Since $\mathbf{X} = \mathcal{A} x_0 + \mathcal{B} U$, the state constraint becomes a time-varying linear inequality on $U$:

$$\begin{bmatrix} \mathcal{B} \\ -\mathcal{B} \end{bmatrix} U \leq \begin{bmatrix} \mathbf{R} + \mathbf{e}_{\max} - \mathcal{A} x_0 \\ -\mathbf{R} + \mathbf{e}_{\max} + \mathcal{A} x_0 \end{bmatrix}$$

The constraint matrix ($\mathcal{B}$ stacked with $-\mathcal{B}$) is still fixed. Only the right-hand side changes with $\mathbf{R}$ and $x_0$.

---

## 5. Summary

| | LQR regulation | LQR tracking | MPC regulation | MPC tracking |
|---|---|---|---|---|
| **Gain / Hessian** | $K_k$ from Riccati | Same $K_k$ | $H, F$ from condensing | Same $H$, same $F$ |
| **What's added** | — | $u_k^{\text{ff}}$ | — | $g_{\text{ref}}$ in linear cost |
| **Structure** | $u_k = -K_k x_k$ | $u_k = u_k^{\text{ff}} - K_k (x_k - r_k)$ | $\min \frac{1}{2}U^T H U + (F^T x_0)^T U$ | $\min \frac{1}{2}U^T H U + (F^T x_0 + g_{\text{ref}})^T U$ |
| **Online cost** | None (offline) | None (offline) | Solve QP each step | Same QP cost + one vector add |

The pattern is universal: **tracking = regulation + reference translation.** The feedback machinery (Riccati gains, QP Hessian) doesn't know or care about the reference — it only knows how to stabilize deviations. The reference enters separately, through a feedforward path that maps the desired trajectory into the control space. This separation is one of the cleanest structural properties of linear-quadratic optimal control.

---

## 6. References

1. **Anderson, B.D.O. & Moore, J.B. (1990).** *Optimal Control: Linear Quadratic Methods.* Prentice-Hall. — Chapters 3–4 cover the tracking extension of LQR in detail.

2. **Borrelli, F., Bemporad, A., & Morari, M. (2017).** *Predictive Control for Linear and Hybrid Systems.* Cambridge University Press. — Chapter 9 covers reference tracking in MPC, including the condensed QP formulation.

3. **Maciejowski, J.M. (2002).** *Predictive Control with Constraints.* Prentice Hall. — Chapters 2–3 derive the tracking MPC formulation and discuss feedforward design.

4. **Rawlings, J.B., Mayne, D.Q., & Diehl, M. (2017).** *Model Predictive Control: Theory, Computation, and Design.* Nob Hill. — Chapter 1 covers the regulation-to-tracking transition; Chapter 5 covers state constraints on tracking error.

---

*This document is part of the `Controllers-from-PID-to-QP_MPC` repository. For the LQR derivation, see `bellman_to_lqr.md`. For the condensing derivation that produces $H$ and $F$, see `from_lp_to_qp_to_lqr.md` (Section 6). The interactive tracker `lqr_explorer.html` implements LQI — LQR tracking with integral augmentation for steady-state reference following.*
