# Eigenvalues and Eigenvectors in Control Engineering

**They torment every control engineering student, and for good reason. Modern control theory is fundamentally asking one question: inside a multi-dimensional system, which directions stay pointing the same way when the dynamics act on them? And along those directions, does the system decay, grow, oscillate, or hold steady? The answer is eigenvalues and eigenvectors.**

---

## 1. The core intuition: a matrix is not a table of numbers — it's a transformation

Take a matrix $A$ acting on a vector $x$:

$$Ax$$

Don't read this as multiplication. Read it as:

> The matrix $A$ takes a vector $x$ in space and turns it into another vector $Ax$.

In a 2D plane, a matrix can:

- Stretch some directions
- Compress others
- Rotate the whole plane
- Shear space
- Flip certain directions
- Mix different coordinates together

Generally, when a matrix acts on a vector, the direction changes:

$$x \quad \longrightarrow \quad Ax$$

But some very special vectors, when acted on by $A$, **do not change direction — only their length changes**.

These are the eigenvectors.

---

## 2. Definition of eigenvector and eigenvalue

If there exists a non-zero vector $v$ such that:

$$Av = \lambda v$$

then:

- $v$ is an **eigenvector** of $A$
- $\lambda$ is the corresponding **eigenvalue**

In words:

> When the matrix $A$ acts along the direction $v$, the result stays along $v$ — only scaled by a factor of $\lambda$.

$v$ cannot be the zero vector, because the zero vector has no direction.

---

## 3. A concrete example

Take:

$$A = \begin{bmatrix} 2 & 0 \\ 0 & 3 \end{bmatrix}$$

It acts on a generic vector as:

$$\begin{bmatrix} x \\ y \end{bmatrix} \longrightarrow \begin{bmatrix} 2x \\ 3y \end{bmatrix}$$

The $x$-direction is stretched by 2×, the $y$-direction by 3×.

For the vector:

$$v_1 = \begin{bmatrix} 1 \\ 0 \end{bmatrix}$$

we have:

$$Av_1 = \begin{bmatrix} 2 \\ 0 \end{bmatrix} = 2\begin{bmatrix} 1 \\ 0 \end{bmatrix}$$

So $\lambda_1 = 2$, with eigenvector along the $x$-axis.

For:

$$v_2 = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$

we have:

$$Av_2 = \begin{bmatrix} 0 \\ 3 \end{bmatrix} = 3\begin{bmatrix} 0 \\ 1 \end{bmatrix}$$

So $\lambda_2 = 3$, with eigenvector along the $y$-axis.

This matrix is easy to understand because it doesn't mix $x$ and $y$.

But control system matrices usually look like this:

$$A = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}$$

It mixes the state variables together. The eigenvector search is asking:

> Even though the coordinates get mixed, are there some *intrinsic directions* that don't get mixed?

There usually are. These directions are the system's **modal directions**.

---

## 4. How to compute eigenvalues

Start from the definition:

$$Av = \lambda v$$

Rearrange:

$$Av - \lambda v = 0$$

Since $\lambda v = \lambda I v$:

$$(A - \lambda I)v = 0$$

For this equation to have a non-zero solution $v$, the matrix $(A - \lambda I)$ must be singular — its determinant must be zero:

$$\det(A - \lambda I) = 0$$

This is the **characteristic equation**. Solve it for $\lambda$, and you have the eigenvalues.

---

## 5. A worked example from control systems

Consider:

$$A = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}$$

This comes from the second-order ODE:

$$\ddot{x} + 3\dot{x} + 2x = 0$$

In state-space form, let $x_1 = x$, $x_2 = \dot{x}$:

$$\dot{x}_1 = x_2$$
$$\dot{x}_2 = -2x_1 - 3x_2$$

So:

$$\dot{x} = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix} x$$

Now find the eigenvalues:

$$A - \lambda I = \begin{bmatrix} -\lambda & 1 \\ -2 & -3-\lambda \end{bmatrix}$$

Determinant:

$$\det(A-\lambda I) = (-\lambda)(-3-\lambda) - (1)(-2) = \lambda(3+\lambda) + 2 = \lambda^2 + 3\lambda + 2$$

Set to zero:

$$\lambda^2 + 3\lambda + 2 = 0$$

Factor:

$$(\lambda+1)(\lambda+2)=0$$

Therefore:

$$\lambda_1 = -1,\qquad \lambda_2 = -2$$

These two eigenvalues tell us:

> This system has two natural modes. One decays as $e^{-t}$, the other decays as $e^{-2t}$.

This is already deeply control-theoretic.

---

## 6. How to compute eigenvectors

Start with $\lambda_1 = -1$. Substitute into $(A-\lambda I)v = 0$:

$$A - (-1)I = A + I = \begin{bmatrix} 1 & 1 \\ -2 & -2 \end{bmatrix}$$

Let $v = \begin{bmatrix} v_1 \\ v_2 \end{bmatrix}$. Then:

$$\begin{bmatrix} 1 & 1 \\ -2 & -2 \end{bmatrix} \begin{bmatrix} v_1 \\ v_2 \end{bmatrix} = 0$$

The first row gives $v_1 + v_2 = 0$, so $v_2 = -v_1$. Taking $v_1 = 1$:

$$v^{(1)} = \begin{bmatrix} 1 \\ -1 \end{bmatrix}$$

So $\lambda_1 = -1$, with $v^{(1)} = \begin{bmatrix} 1 \\ -1 \end{bmatrix}$.

Now $\lambda_2 = -2$:

$$A - (-2)I = A + 2I = \begin{bmatrix} 2 & 1 \\ -2 & -1 \end{bmatrix}$$

The equation: $2v_1 + v_2 = 0$, so $v_2 = -2v_1$. Taking $v_1 = 1$:

$$v^{(2)} = \begin{bmatrix} 1 \\ -2 \end{bmatrix}$$

So $\lambda_2 = -2$, with $v^{(2)} = \begin{bmatrix} 1 \\ -2 \end{bmatrix}$.

---

## 7. What these eigenvectors mean in a control system

The system is:

$$\dot{x} = Ax$$

If the initial state lies exactly along the first eigenvector:

$$x(0) = c\begin{bmatrix} 1 \\ -1 \end{bmatrix}$$

then the motion stays on that direction forever, only its magnitude decays as $e^{-t}$:

$$x(t) = c e^{-t}\begin{bmatrix} 1 \\ -1 \end{bmatrix}$$

If the initial state lies along the second eigenvector:

$$x(0) = c\begin{bmatrix} 1 \\ -2 \end{bmatrix}$$

then:

$$x(t) = c e^{-2t}\begin{bmatrix} 1 \\ -2 \end{bmatrix}$$

So:

- **Eigenvectors** are the system's **natural directions of motion**.
- **Eigenvalues** are the **natural rates of motion** along those directions.

---

## 8. This is what "modes" means

Control theory constantly talks about:

- modes
- modal decomposition
- modal coordinates
- dominant pole
- fast mode / slow mode
- unstable mode
- oscillatory mode

All of these are about eigenvalues and eigenvectors.

| Eigenvalue | Mode behavior |
|---|---|
| $\lambda = -1$ | Decays as $e^{-t}$ |
| $\lambda = -10$ | Decays quickly as $e^{-10t}$ |
| $\lambda = 2$ | Blows up as $e^{2t}$ — unstable |
| $\lambda = -1 \pm j5$ | Oscillates at ~5 rad/s with envelope $e^{-t}$ |

A complex conjugate pair $\lambda = -1 \pm j5$ gives two real solution components:

$$e^{-t}\cos(5t),\qquad e^{-t}\sin(5t)$$

The system oscillates at roughly $5\text{ rad/s}$ while the envelope decays as $e^{-t}$.

---

## 9. Continuous-time: eigenvalues and stability

For a continuous-time system:

$$\dot{x} = Ax$$

Stability is determined by the eigenvalues of $A$.

- If **all** eigenvalues have negative real parts: $\operatorname{Re}(\lambda_i) < 0$ → **asymptotically stable**.
- If **any** eigenvalue has positive real part: $\operatorname{Re}(\lambda_i) > 0$ → **unstable**.
- If eigenvalues lie on the imaginary axis, e.g. $\lambda = \pm j\omega$: further analysis needed (repeated roots, Jordan structure). In simple cases: constant-amplitude oscillation; in degenerate cases: possible instability.

**Continuous-time stability region: the open left half-plane.**

---

## 10. Discrete-time: eigenvalues and stability

For a discrete-time system:

$$x_{k+1} = A x_k$$

Each time step multiplies by $A$ once.

- If an eigenvalue is $\lambda = 0.8$: each step shrinks the mode to 80% → decays.
- If $\lambda = 1.2$: each step grows it to 120% → diverges.

**Discrete-time stability condition:** $|\lambda_i| < 1$ for all $i$.

All eigenvalues must lie inside the unit circle.

The mapping between continuous and discrete domains is:

$$z = e^{sT}$$

The continuous left half-plane maps to the interior of the unit circle.

---

## 11. Why eigenvalues are poles

This is critical.

For a state-space system:

$$\dot{x} = Ax + Bu$$
$$y = Cx + Du$$

The transfer function is:

$$G(s) = C(sI - A)^{-1}B + D$$

The term $(sI - A)^{-1}$ blows up precisely when:

$$\det(sI - A) = 0$$

And $\det(sI - A) = 0$ is exactly the characteristic equation of $A$, with $s$ in place of $\lambda$.

Therefore:

> The eigenvalues of the state matrix $A$ are the **internal poles** of the system dynamics.

More precisely:

- The eigenvalues of $A$ are the system's **internal poles**.
- If some modes are uncontrollable or unobservable, they may be hidden in the input-output transfer function — but they still exist inside the state.
- This is why state-space control is "deeper" than classical transfer functions: it sees hidden internal modes.

---

## 12. The real power of eigenvectors: change of coordinates

### 12.1 Where we are

We have the original system in **physical coordinates**:

$$\dot{x} = Ax$$

Here $x$ is the physical state (positions, velocities, voltages — whatever the system is made of), and $A$ is a matrix that **couples** those states together. In general, every $\dot{x}_i$ depends on multiple $x_j$ — the system is tangled.

What we want:

> Find a **smarter set of coordinates** where the system becomes simple.

We want to change variables. Not because $x$ is wrong — but because $x$ is inconvenient.

### 12.2 The key idea: use eigenvectors as new coordinate axes

Suppose $A$ has $n$ linearly independent eigenvectors $v_1, v_2, \dots, v_n$. Stack them as columns into a matrix:

$$V = \begin{bmatrix} | & | & & | \\ v_1 & v_2 & \cdots & v_n \\ | & | & & | \end{bmatrix}$$

$V$ is not just any matrix. **Each column of $V$ is a new coordinate axis.** The axes are the directions the system "likes" — its eigenvectors. Together they span a new coordinate system.

Because each column is an eigenvector ($A v_i = \lambda_i v_i$), stacking them gives the compact relationship:

$$AV = V\Lambda$$

where:

$$\Lambda = \begin{bmatrix} \lambda_1 & 0 & \cdots & 0 \\ 0 & \lambda_2 & \cdots & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \cdots & \lambda_n \end{bmatrix}$$

Thus:

$$A = V\Lambda V^{-1}$$

This is **diagonalization**.

### 12.3 So what is $z$? (This is where textbooks lose people)

Here is the step that many textbooks skip. We **define a new variable $z$** by:

$$\boxed{x = Vz}$$

or equivalently:

$$\boxed{z = V^{-1}x}$$

This is not pulling $z$ out of thin air. It is a **coordinate transformation**:

- $x$ — describes the state in the **original coordinate system** (physical axes)
- $z$ — describes the **same state** in the **eigenvector coordinate system** (modal axes)

> $z$ is the projection of $x$ onto the eigenvector basis. Each $z_i$ is "how much of mode $i$ is present in the current state."

**Analogy.** This is exactly like switching coordinate systems in geometry:

| Transformation | Original coordinates | New coordinates |
|---|---|---|
| Cartesian → polar | $(x, y)$ | $(r, \theta)$ |
| World → robot joint | world-frame position | joint angles |
| Physical → modal | $x$ (coupled states) | $z$ (decoupled modes) |

The only difference: here the "axes" we're rotating to are the eigenvectors — the directions the system naturally moves along.

### 12.4 Derivation: how $z$ emerges from the algebra

We don't know ahead of time that $z$ will work. We *try* the transformation $x = Vz$ and see what happens.

**Step 1 — Differentiate the transformation.** From $x = Vz$, take the time derivative ( $V$ is constant, so it passes through):

$$\dot{x} = V\dot{z}$$

**Step 2 — Substitute into the original system.** The original system is $\dot{x} = Ax$. Substitute $x = Vz$ on the right and $\dot{x} = V\dot{z}$ on the left:

$$V\dot{z} = A(Vz)$$

**Step 3 — Left-multiply by $V^{-1}$.** This isolates $\dot{z}$:

$$\dot{z} = V^{-1}AV\,z$$

**Step 4 — The eigenvector magic happens.** Because $AV = V\Lambda$, multiplying on the left by $V^{-1}$ gives:

$$V^{-1}AV = V^{-1}V\Lambda = \Lambda$$

**Step 5 — The result.** The system in $z$-coordinates is:

$$\dot{z} = \Lambda z$$

Writing it out row by row:

$$\dot{z}_1 = \lambda_1 z_1$$
$$\dot{z}_2 = \lambda_2 z_2$$
$$\cdots$$
$$\dot{z}_n = \lambda_n z_n$$

Each $z_i$ evolves independently of all the others. The coupled system has become $n$ separate first-order systems.

### 12.5 Why this decoupling happens

In the original $x$-space, $A$ mixes all the states together. $\dot{x}_1$ depends on $x_2$, $x_3$, and so on.

In the $z$-space, $\Lambda$ is **diagonal**. A diagonal matrix means:

> Each row only references its own variable.

$\dot{z}_1$ depends only on $z_1$. $\dot{z}_2$ depends only on $z_2$. The cross-coupling is gone.

In the $x$ coordinate system, the axes are arbitrary (position, velocity, etc.) and the dynamics couple across them. In the $z$ coordinate system, the axes are the system's own natural motion directions — and along each axis, the dynamics are self-contained.

### 12.6 What $z$ physically represents

Now we can say precisely what $z_i$ is:

> $z_i$ = the component of the system's state along the $i$-th eigenvector direction.

These are called **modal coordinates**. In control engineering, the $z_i$ are the quantities behind every term you hear:

- **slow mode** — the $z_i$ with eigenvalue near the imaginary axis
- **fast mode** — the $z_i$ with eigenvalue far into the left half-plane
- **unstable mode** — the $z_i$ with eigenvalue in the right half-plane
- **oscillatory mode** — a pair of $z_i$ corresponding to a complex conjugate eigenvalue pair

When you do modal analysis, vibration decoupling, or system decomposition, you are working in $z$-space — whether the textbook calls it that or not.

### 12.7 The one-line summary

$$\boxed{z = V^{-1}x}$$

$z$ is not a variable that appears from nowhere. It is the natural result of asking: *what if we described the system using its own preferred directions instead of our arbitrary physical coordinates?* The answer is: the system becomes diagonal, and each mode becomes independent.

---

## 13. Why control engineers frame everything as "choosing eigenvalues"

Because the closed-loop response is governed by the eigenvalues of the closed-loop matrix.

Open-loop system:

$$\dot{x} = Ax + Bu$$

Apply state feedback:

$$u = -Kx$$

Substitute:

$$\dot{x} = Ax + B(-Kx) = (A - BK)x$$

The matrix:

$$A_{cl} = A - BK$$

is the **closed-loop system matrix**.

Stability, speed, oscillation — all determined by the eigenvalues of $A - BK$.

So one of the central problems of controller design is:

> Choose $K$ to place the eigenvalues of $A - BK$ at desired locations.

This is **pole placement**.

---

## 14. Pole placement design

Given:

$$\dot{x} = Ax + Bu$$

Design:

$$u = -Kx$$

Goal: make the eigenvalues of $A - BK$ equal to specified closed-loop poles.

For example, if we want closed-loop poles at $-4, -5$, we need:

$$\det(sI - (A - BK)) = (s+4)(s+5) = s^2 + 9s + 20$$

Choose $K$ to make the closed-loop characteristic equation match this.

---

## 15. A complete worked pole-placement example

Same second-order system:

$$A = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix},\qquad B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$

Let $u = -Kx$ with $K = \begin{bmatrix} k_1 & k_2 \end{bmatrix}$.

Then:

$$BK = \begin{bmatrix} 0 \\ 1 \end{bmatrix}\begin{bmatrix} k_1 & k_2 \end{bmatrix} = \begin{bmatrix} 0 & 0 \\ k_1 & k_2 \end{bmatrix}$$

$$A - BK = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix} - \begin{bmatrix} 0 & 0 \\ k_1 & k_2 \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -2-k_1 & -3-k_2 \end{bmatrix}$$

Its characteristic equation:

$$\det(sI - (A-BK)) = \det\begin{bmatrix} s & -1 \\ 2+k_1 & s+3+k_2 \end{bmatrix} = s(s+3+k_2) + (2+k_1)$$

$$= s^2 + (3+k_2)s + (2+k_1)$$

Target: poles at $-4, -5$. Desired characteristic equation:

$$(s+4)(s+5) = s^2 + 9s + 20$$

Match coefficients:

$$3 + k_2 = 9 \quad\Rightarrow\quad k_2 = 6$$
$$2 + k_1 = 20 \quad\Rightarrow\quad k_1 = 18$$

Therefore:

$$K = \begin{bmatrix} 18 & 6 \end{bmatrix}$$

Closed-loop matrix:

$$A - BK = \begin{bmatrix} 0 & 1 \\ -20 & -9 \end{bmatrix}$$

Its eigenvalues are exactly $-4, -5$. That's pole placement.

---

## 16. Can you place poles anywhere? No.

The key condition: the system must be **controllable**.

Controllability matrix:

$$\mathcal{C} = \begin{bmatrix} B & AB & A^2B & \cdots & A^{n-1}B \end{bmatrix}$$

If $\operatorname{rank}(\mathcal{C}) = n$, the system is fully controllable.

This means:

> The input $u$ can influence **all** internal modes.

If an eigenmode is uncontrollable, you cannot move it with feedback.

> The controller does not directly control state variables — it influences system modes through the input. If a mode is unreachable by the input, its eigenvalue cannot be moved.

---

## 17. Eigenvectors and controllability: the PBH test

Controllability isn't just about eigenvalues — eigenvectors matter too.

The **PBH (Popov-Belevitch-Hautus) test**: the system $(A, B)$ is controllable iff for every eigenvalue $\lambda$ of $A$:

$$\operatorname{rank}\begin{bmatrix} \lambda I - A & B \end{bmatrix} = n$$

Intuition:

> For each natural mode, the input matrix $B$ must be able to "reach into" it.

There is also a left-eigenvector interpretation. If $w^T A = \lambda w^T$ (so $w$ is a left eigenvector) and:

$$w^T B = 0$$

then that mode is completely unreachable by the input — uncontrollable.

This gives better physical intuition than the controllability matrix alone:

> If the actuator force direction is perfectly orthogonal to a particular internal vibration direction, you cannot control that vibration.

---

## 18. Observer design

In real systems, we rarely measure the full state $x$.

System:

$$\dot{x} = Ax + Bu$$
$$y = Cx$$

The goal of an observer is to construct an estimated state $\hat{x}$.

The classic **Luenberger observer**:

$$\dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x})$$

where:

- $\hat{x}$ is the estimated state
- $y - C\hat{x}$ is the measurement error (the **innovation**)
- $L$ is the observer gain matrix

Define estimation error $e = x - \hat{x}$. The error dynamics:

$$\dot{e} = \dot{x} - \dot{\hat{x}} = (Ax + Bu) - \big(A\hat{x} + Bu + LC(x - \hat{x})\big)$$
$$\dot{e} = A(x - \hat{x}) - LC(x - \hat{x}) = (A - LC)e$$

So whether the observer error converges depends on the eigenvalues of:

$$A - LC$$

If all eigenvalues of $A - LC$ are in the left half-plane (and far enough left), estimation error decays quickly.

Observer design is also pole placement:

> Choose $L$ to place the eigenvalues of $A - LC$ at desired locations.

---

## 19. Controller and observer are duals

| | Controller | Observer |
|---|---|---|
| Closed-loop matrix | $A - BK$ | $A - LC$ |
| Input/output matrix | $B$ (how input enters) | $C$ (how state is measured) |
| Design condition | $(A, B)$ controllable | $(A, C)$ observable |

Observability matrix:

$$\mathcal{O} = \begin{bmatrix} C \\ CA \\ CA^2 \\ \vdots \\ CA^{n-1} \end{bmatrix}$$

If $\operatorname{rank}(\mathcal{O}) = n$, the system is fully observable.

Meaning:

> All internal modes leave a trace in the output $y$.

If a mode is invisible to the output, the observer cannot estimate it.

---

## 20. Eigenvectors and observability: the PBH test

System $(A, C)$ is observable iff for every eigenvalue $\lambda$ of $A$:

$$\operatorname{rank}\begin{bmatrix} \lambda I - A \\ C \end{bmatrix} = n$$

Intuition:

> Every natural mode must be visible to the sensors.

If a right eigenvector $v$ satisfies $Av = \lambda v$ but:

$$Cv = 0$$

then that mode exists inside the system but produces zero output. It is **unobservable**.

Example: a mechanical structure has an internal vibration mode, but your sensor is mounted exactly at the node of that mode — it cannot see that vibration.

---

## 21. Observer-based controller: the separation principle

When you cannot measure $x$ directly, use the estimate $\hat{x}$ for feedback:

$$u = -K\hat{x}$$

Combined system — controller + observer:

$$u = -K\hat{x}$$
$$\dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x})$$

The overall closed-loop eigenvalues have a beautiful property:

$$\text{eig(total)} = \text{eig}(A - BK) \;\cup\; \text{eig}(A - LC)$$

This is the **separation principle**.

> You can design $K$ first to make the control loop stable, then design $L$ to make the estimation error converge fast. In linear systems, these two designs are independent.

This is one of the most important results in modern control theory.

---

## 22. Eigenvalues in LQR

LQR does not directly specify eigenvalues. It minimizes a cost function:

$$J = \int_0^\infty (x^T Q x + u^T R u)\,dt$$

Solving the algebraic Riccati equation yields $P$, and the optimal gain is:

$$K = R^{-1} B^T P$$

The closed-loop matrix is still $A - BK$, and LQR produces a set of closed-loop eigenvalues.

The difference from pole placement:

- **Pole placement**: you say "put the poles here."
- **LQR**: you say "I care about state error and control effort — please find the best trade-off automatically," and LQR picks the eigenvalues for you.

The result still comes down to $\operatorname{eig}(A - BK)$:

- Eigenvalues too far left → fast response, potentially large control effort
- Eigenvalues near the imaginary axis → slow response, more energy-efficient

---

## 23. Eigenvalues in the Kalman filter

The Kalman filter is the statistically optimal observer under noise assumptions. Its structure mirrors the Luenberger observer:

$$\dot{\hat{x}} = A\hat{x} + Bu + K_f(y - C\hat{x})$$

where $K_f$ is the Kalman gain.

Error dynamics:

$$\dot{e} = (A - K_f C)e + \text{noise terms}$$

The filter's convergence speed and estimation error dynamics are governed by the eigenvalues of $A - K_f C$.

- Trust measurements more → larger Kalman gain → faster observer poles → more noise enters the estimate
- Trust the model more → smaller gain → smoother estimate → slower response

Duality:

| | LQR | Kalman Filter |
|---|---|---|
| Trade-off | State error vs. control energy | Model prediction vs. measurement noise |
| Gain from | Riccati equation | Riccati equation |
| Closed-loop | $A - BK$ | $A - K_f C$ |

Combined, they form **LQG** (Linear Quadratic Gaussian) control.

---

## 24. Complex eigenvalues: control interpretation

Many control systems are second-order oscillators. If the closed-loop poles are:

$$\lambda = -\sigma \pm j\omega_d$$

the response looks like:

$$e^{-\sigma t}\cos(\omega_d t)$$

where:

- $\sigma$ — decay rate (more negative = faster decay)
- $\omega_d$ — oscillation frequency (larger = faster oscillation)

In second-order standard form:

$$s^2 + 2\zeta\omega_n s + \omega_n^2$$

Poles:

$$s = -\zeta\omega_n \pm j\omega_n\sqrt{1 - \zeta^2}$$

where $\zeta$ is the damping ratio and $\omega_n$ is the natural frequency.

Reading eigenvalues at a glance:

| Eigenvalues | Interpretation |
|---|---|
| $-2 \pm j10$ | High oscillation frequency, moderate decay |
| $-10 \pm j2$ | Fast decay, light oscillation |
| $-1 \pm j1$ | Balanced — decay and oscillation at similar rates |
| $0.1 \pm j5$ | Very slow decay, persistent high-frequency oscillation |

---

## 25. Common misconception: eigenvalue ≠ eigenvector

Many students confuse the two.

**Eigenvalue is a number.**

$$\lambda = -3$$

It tells you the temporal behavior of that mode: $e^{-3t}$.

**Eigenvector is a direction.**

$$v = \begin{bmatrix} 1 \\ -3 \end{bmatrix}$$

It tells you the shape of that mode in state space.

In a mechanical system:

- Eigenvalue ≈ "this mode's frequency and damping"
- Eigenvector ≈ "what this mode shape looks like"

For a multi-degree-of-freedom spring-mass system:

- One mode: both masses move in phase
- Another mode: masses move out of phase

These "in-phase / out-of-phase shapes" are the physical meaning of eigenvectors.

---

## 26. Another misconception: eigenvectors are not unique

If $v$ is an eigenvector, then so are:

$$2v,\quad 10v,\quad -v$$

They all point in the same direction — just different lengths.

An eigenvector fundamentally represents a **direction**, not a fixed-length vector.

$$\begin{bmatrix} 1 \\ -2 \end{bmatrix} \quad\text{and}\quad \begin{bmatrix} 10 \\ -20 \end{bmatrix}$$

represent the same eigen-direction.

---

## 27. Yet another: not all matrices are diagonalizable

If $A$ has $n$ linearly independent eigenvectors, it can be diagonalized:

$$A = V\Lambda V^{-1}$$

But some matrices don't have enough eigenvectors. For example:

$$A = \begin{bmatrix} 1 & 1 \\ 0 & 1 \end{bmatrix}$$

It has only one independent eigenvector and cannot be diagonalized in the usual way. This requires the **Jordan canonical form**.

In control, Jordan blocks are messier, but the intuition is:

> The system response isn't just pure $e^{\lambda t}$ — you may get terms like $t e^{\lambda t}$, $t^2 e^{\lambda t}$, etc.

This is why repeated roots and non-diagonalizable matrices produce more complicated dynamics.

---

## 28. Connection to poles and the homogeneous solution

For the unforced system (homogeneous system):

$$\dot{x} = Ax$$

The solution is:

$$x(t) = e^{At}x(0)$$

If $A$ is diagonalizable ($A = V\Lambda V^{-1}$):

$$e^{At} = V e^{\Lambda t} V^{-1}$$

where:

$$e^{\Lambda t} = \begin{bmatrix} e^{\lambda_1 t} & 0 & \cdots \\ 0 & e^{\lambda_2 t} & \cdots \\ \vdots & \vdots & \ddots \end{bmatrix}$$

So the natural solution is a sum of modal contributions:

$$\boxed{x(t) = \sum_i c_i\, e^{\lambda_i t}\, v_i}$$

where:

- $c_i$ — how much this mode is excited (depends on initial conditions)
- $\lambda_i$ — how this mode evolves in time (exponential rate / frequency)
- $v_i$ — the shape of this mode in state space

This is the most beautiful summary:

$$\boxed{\text{System response} = \sum \text{(modal amplitude)} \times e^{\lambda_i t} \times \text{(modal shape)}}$$

---

## 29. How control engineers actually use eigenvalues and eigenvectors

### Category 1: Assess natural stability

Compute $\operatorname{eig}(A)$. All in the left half-plane → open-loop stable. Any in the right half-plane → open-loop unstable.

### Category 2: Assess response speed and oscillation

Read eigenvalue positions:

- More negative real part → faster decay
- Real part near zero → slow response
- Larger imaginary part → higher oscillation frequency
- Right half-plane → divergence
- Near the origin → slow modes
- **Dominant poles** (closest to the imaginary axis) determine the overall response shape

### Category 3: Design state feedback

Choose $K$ to shape the eigenvalues of $A - BK$. This is the core of pole placement and LQR.

### Category 4: Design observers

Choose $L$ to shape the eigenvalues of $A - LC$. Make estimation error decay fast.

### Category 5: Check controllability and observability

Use eigenvectors (via PBH tests) to determine:

- Which modes the input can reach → controllable
- Which modes the output can see → observable

Uncontrollable modes cannot be moved by $K$. Unobservable modes cannot be estimated by $L$.

### Category 6: Modal analysis

In mechanical, structural, robotics, and aerospace systems, eigenvectors reveal mode shapes:

- Which part of a flexible robot joint vibrates
- Aircraft short-period and phugoid modes
- Power system low-frequency oscillation modes
- Coupled vibrations at a particular robot arm posture
- Mode shapes of multi-mass-spring systems

---

## 30. The intuition every control engineer should develop

When you see:

$$\dot{x} = Ax$$

think:

> The system's natural dynamics are hidden in the eigenvalues and eigenvectors of $A$.

When you see:

$$\dot{x} = Ax + Bu$$

think:

> Can the input $B$ reach those eigenmodes? If so, I can move them with feedback.

When you see:

$$y = Cx$$

think:

> Can the output $C$ see those eigenmodes? If so, I can design an observer to estimate them.

When you see:

$$u = -Kx$$

think:

$$A \rightarrow A - BK$$

The controller changed the system's eigenvalues.

When you see:

$$\dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x})$$

think:

$$A \rightarrow A - LC$$

The observer changed the estimation error's eigenvalues.

---

## 31. The one-sentence summary

**Eigenvectors are the directions the system naturally likes to move along.**

**Eigenvalues are the growth rate, decay rate, or oscillation frequency along each of those directions.**

In modern control:

| Matrix | Meaning |
|---|---|
| $A$ | Determines open-loop modes |
| $A - BK$ | Determines closed-loop (controller) modes |
| $A - LC$ | Determines observer error modes |

Modern control theory is built on eigenvalues and eigenvectors because the essence of control is:

> Find the system's own modes. Judge whether they're stable. Use inputs to move bad modes to good locations. Use sensors to estimate hidden modes.
