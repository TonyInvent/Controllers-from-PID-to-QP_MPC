# Observer Design: Bridging State Feedback to Output Feedback

**LQR gives you $u = -Kx$. But you never measure $x$. You measure $y = Cx$ through noisy, delayed, incomplete sensors. The observer is not an add-on — it is half the controller.**

---

## 1. The lie at the heart of state feedback

Every state-space controller — LQR, pole placement, MPC — begins with the same assumption: **you know the full state vector $x$.** The control law is $u = -Kx$. It's a dot product. It's beautiful, it's optimal, it comes with guarantees (≥60° phase margin for LQR), and it rests on a fiction.

In reality, you never have the full state. The sensor tells you $y$, not $x$:

- **You measure position, not velocity.** A quadrature encoder gives you $\theta$. To get $\dot{\theta}$, you differentiate — which amplifies noise by $\omega$ in the frequency domain. At 10 kHz sampling, encoder quantization noise differentiated into "velocity" is unusable without heavy filtering.
- **You measure one temperature, not the whole thermal field.** A thermocouple at one point of a reactor tells you nothing about the temperature 10 cm away, or the temperature gradient, or the heat flux. But the thermal PDE has hundreds of states.
- **You measure acceleration, not attitude.** An IMU gives you accelerometer and gyro readings. Roll and pitch angles are *inferred* by integrating gyro rates and fusing with the gravity vector from the accelerometer — that's an observer.

Using $y$ as if it were $x$ is wrong in two directions. First, $y$ is lower-dimensional — $C$ projects the state down to what the sensors see. Second, $y$ is corrupted — sensor noise, quantization, communication delay. Feeding $u = -Ky$ into your carefully designed LQR gives you a controller that is neither optimal nor, in many cases, stable.

**The observer is the answer.** It reconstructs $\hat{x}$ from $y$ and $u$ using a model of the plant. Then the controller uses $\hat{x}$ as if it were $x$. This is **output feedback**: measure $y$, estimate $x$, control with $\hat{x}$.

---

## 2. Open-loop simulation: why you need correction

The simplest idea: run a copy of the plant model in software.

$$\hat{x}_{k+1} = A \hat{x}_k + B u_k$$

You know $u_k$ (you computed it). You know $A$ and $B$ (from the model). Start $\hat{x}_0$ at some guess, and let the model run.

This works — for about three time steps. Then three things ruin it:

1. **Unknown initial condition.** The plant starts at some $x_0$ you don't know. If $\hat{x}_0 \neq x_0$, the estimate is wrong from the start, and the error never goes away because there's no correction mechanism.
2. **Model mismatch.** $A$ and $B$ are approximations. The real motor has friction, cogging torque, parameter drift from heating. These unmodeled dynamics cause $\hat{x}$ to diverge from $x$ over time.
3. **Disturbances.** An unexpected load torque, a gust of wind, a voltage sag — these change $x$ in ways the open-loop model never sees.

An open-loop simulator integrates errors, it doesn't correct them. What we need is a way to **pull the estimate toward the truth** using measurements.

---

## 3. The Luenberger observer: run the model, correct with measurements

David Luenberger (1964) had the key insight. Run the model, but add a correction term proportional to the **measurement error** — the difference between what you actually measure ($y_k$) and what your model predicts you should measure ($C\hat{x}_k$):

$$\hat{x}_{k+1} = \underbrace{A \hat{x}_k + B u_k}_{\text{prediction (open-loop)}} + \underbrace{L\,(y_k - C \hat{x}_k)}_{\text{correction}}$$

The signal $y_k - C\hat{x}_k$ is the **innovation** — the part of the measurement that your model didn't predict. If the model is perfect and $\hat{x} = x$, the innovation is zero (except for measurement noise). If $\hat{x}$ is wrong, the innovation tells you *in what direction* and *by how much*.

$L$ is the **observer gain matrix**. It determines how aggressively you trust the measurement vs. the model:

- $L$ large → trust measurements heavily, converge fast, but amplify sensor noise
- $L$ small → trust the model, smooth noise, but converge slowly
- $L = 0$ → open-loop simulation, no correction

The error dynamics tell the story. Define $e_k = x_k - \hat{x}_k$:

$$e_{k+1} = x_{k+1} - \hat{x}_{k+1} = (A x_k + B u_k) - \big(A \hat{x}_k + B u_k + L(C x_k - C \hat{x}_k)\big)$$
$$e_{k+1} = (A - LC)\, e_k$$

The observer error evolves according to the matrix $(A - LC)$. If all eigenvalues of $(A - LC)$ lie inside the unit circle (discrete-time), the error converges to zero **regardless of the initial guess**. The observer is stable. The rate of convergence is determined by how far inside the unit circle those eigenvalues are.

In continuous time, the Luenberger observer is:

$$\dot{\hat{x}} = A \hat{x} + B u + L (y - C \hat{x})$$

and the error dynamics are $\dot{e} = (A - LC) e$. Stability requires all eigenvalues of $(A - LC)$ to have negative real parts.

---

## 4. Observer pole placement: faster than the controller, but not too fast

Designing $L$ is a pole placement problem. The eigenvalues of $(A - LC)$ are the **observer poles** — they determine how fast the estimate converges.

The rule of thumb: **place observer poles 2–5× faster than the controller poles.**

| If controller poles are at... | Place observer poles at... | Rationale |
|---|---|---|
| $s = -10$ | $s = -20$ to $-50$ | The observer converges before the controller's transient matters |
| $s = -2 \pm j3$ | $s = -4 \pm j6$ to $-10 \pm j15$ | The estimation error dies out 2–5× faster than the closed-loop response |

Why "faster" is good:

- The controller acts on $\hat{x}$. If $\hat{x}$ is still converging while the plant is already responding, you're controlling a moving target — the effective dynamics are degraded.
- During startup, $\hat{x}$ starts wrong. You want it to converge before the reference step arrives, or at least within the first few samples.
- For LQR, the separation principle (Section 7) guarantees that the closed-loop poles are the **union** of the controller poles (from $A-BK$) and the observer poles (from $A-LC$). If the observer poles are much faster, the closed-loop response is dominated by the controller poles — exactly what you designed $K$ for.

Why "too fast" is bad:

- Fast observer poles → large $L$ gains → the correction term $L(y_k - C\hat{x}_k)$ amplifies measurement noise. Sensor noise enters the estimate directly, scaled by $L$.
- Fast poles → wide bandwidth → the observer tracks high-frequency sensor artifacts, producing a noisy $\hat{x}$ that the controller then acts on. You get jittery control signals, increased actuator wear, and excitation of unmodeled high-frequency plant modes.
- There's a fundamental trade-off: **convergence speed vs. noise sensitivity.** The Kalman filter (Section 5) resolves this optimally.

**Computing $L$ by pole placement:**

The eigenvalues of $(A - LC)$ are the same as the eigenvalues of $(A^T - C^T L^T)$. This is exactly the same form as the controller pole placement problem for $(A^T - C^T K^T)$ — place the poles of $A^T - C^T \tilde{K}$ with $\tilde{K} = L^T$. This is **duality** (Section 6): design $L$ by applying pole placement to the transposed system.

In MATLAB: `L = place(A', C', poles)'` — transpose, design a "controller" for the dual system, transpose back.

---

## 5. The Kalman filter: when you know the noise statistics

The Luenberger observer places poles by hand. The Kalman filter (Rudolf Kalman, 1960) chooses $L$ **optimally** by asking a statistical question:

> Given that the plant is driven by random process noise $w_k \sim \mathcal{N}(0, Q)$ and the sensor is corrupted by measurement noise $v_k \sim \mathcal{N}(0, R)$, what gain $L_k$ minimizes the expected squared estimation error $\mathbb{E}[\|x_k - \hat{x}_k\|^2]$ at every time step?

The system model:

$$x_{k+1} = A x_k + B u_k + w_k, \qquad w_k \sim \mathcal{N}(0, Q)$$
$$y_k = C x_k + v_k, \qquad v_k \sim \mathcal{N}(0, R)$$

$Q$ is the **process noise covariance** — how much you trust the model. Large $Q$ means the model is unreliable (unmodeled dynamics, disturbances); the filter should trust measurements more.

$R$ is the **measurement noise covariance** — how noisy the sensors are. Large $R$ means the sensors are bad; the filter should trust the model more.

The Kalman filter has the **exact same structure** as the Luenberger observer:

$$\hat{x}_{k+1} = A \hat{x}_k + B u_k + \underbrace{L_k}_{\text{optimal gain}} (y_k - C \hat{x}_k)$$

But $L_k$ is not constant — it's time-varying and computed from a **forward Riccati recursion**:

1. **Predict** the error covariance: $P_{k+1}^- = A P_k A^T + Q$
2. **Compute** the optimal gain: $L_k = P_k^- C^T (C P_k^- C^T + R)^{-1}$
3. **Update** the covariance: $P_k = (I - L_k C) P_k^-$

In steady state, $P_k \to P_\infty$ and $L_k \to L_\infty$, and the filter becomes a steady-state Luenberger observer with the Kalman gain. The steady-state covariance $P_\infty$ solves the **Filter Algebraic Riccati Equation (FARE)**:

$$P = A P A^T - A P C^T (C P C^T + R)^{-1} C P A^T + Q$$

Compare this to the DARE (controller Riccati):

| | Control (LQR) | Estimation (Kalman) |
|---|---|---|
| **Riccati variable** | $P$ (cost-to-go) | $P$ (error covariance) |
| **Direction** | Backward in time | Forward in time |
| **A appears as** | $A^T P A$ | $A P A^T$ |
| **Weighting** | $Q$ = state cost, $R$ = control cost | $Q$ = process noise, $R$ = measurement noise |
| **Gain formula** | $K = (R + B^T P B)^{-1} B^T P A$ | $L = A P C^T (C P C^T + R)^{-1}$ |

The Kalman gain $L$ emerges from $Q$ and $R$ the way the LQR gain $K$ emerges from $Q$ and $R$. In both cases, you specify what you care about, and the Riccati equation gives you the optimal gain. The difference: LQR solves *backward* from the terminal cost; Kalman solves *forward* from the initial uncertainty.

**Tuning the Kalman filter in practice:**

You almost never know $Q$ and $R$ exactly. They become **design knobs**:

- $Q/R$ ratio large → trust measurements, faster convergence, noisier estimate
- $Q/R$ ratio small → trust model, smoother estimate, slower convergence
- Diagonal $Q$ entries correspond to how much you expect each state to be perturbed by unknown effects. A motor's velocity state gets a larger $Q$ entry than its position state (velocity is affected by load torque; position is just integrated velocity, which the model handles well).
- Diagonal $R$ entries correspond to sensor noise variance. An encoder with $\pm 1$ count quantization gets $R = 1/12$ (uniform distribution variance). An analog sensor with 10 mV RMS noise gets $R = (0.01)^2$.

---

## 6. Duality: estimation and control are the same problem, transposed

There is a deep structural relationship between control and estimation. The mathematics are identical — you just transpose the matrices and swap the direction of time.

**Primal (control):** Design $K$ so that $(A - BK)$ has eigenvalues inside the unit circle.

$$x_{k+1} = A x_k + B u_k, \qquad u_k = -K x_k$$
$$x_{k+1} = (A - BK) x_k$$

**Dual (estimation):** Design $L$ so that $(A - LC)$ has eigenvalues inside the unit circle.

$$e_{k+1} = (A - LC) e_k$$

But the eigenvalues of $(A - LC)$ are exactly the eigenvalues of $(A^T - C^T L^T)$. So designing an observer for $(A, C)$ is **exactly** designing a controller for the transposed system $(A^T, C^T)$:

| | Controller for $(A, B)$ | Observer for $(A, C)$ |
|---|---|---|
| **System matrix** | $A$ | $A^T$ |
| **Input matrix** | $B$ | $C^T$ |
| **Gain** | $K$ | $L^T$ |
| **Closed loop** | $A - BK$ | $A^T - C^T L^T$ |
| **Riccati** | $P = A^T P A - \ldots + Q$ | $P = A P A^T - \ldots + Q$ |

This duality means:

- Any algorithm for controller design (pole placement, LQR, H∞) can be reused for observer design by transposing the system.
- The MATLAB command `dlqr(A, B, Q, R)` gives you $K$. `dlqr(A', C', Q, R)` gives you $L^T$ — then $L$ is its transpose. The Kalman gain is the LQR gain of the dual system.
- The separation principle (Section 7) is a direct consequence: the closed-loop eigenvalues of the combined system are the eigenvalues of $(A-BK)$ union the eigenvalues of $(A-LC)$. The controller and observer poles don't interact — they're decoupled by duality.

---

## 7. The separation principle: design them independently, combine them

The separation principle is the theorem that makes output feedback tractable. It says:

> Design $K$ as if you had full state feedback. Design $L$ as if the controller didn't exist. Then combine them: $u_k = -K \hat{x}_k$, where $\hat{x}$ comes from the observer. The closed-loop system is stable, and its poles are exactly the controller poles (from $A-BK$) plus the observer poles (from $A-LC$).

**Why this works.** Write the combined state $[x_k; e_k]$ where $e_k = x_k - \hat{x}_k$:

$$x_{k+1} = A x_k + B(-K \hat{x}_k) = A x_k - B K (x_k - e_k) = (A - BK) x_k + BK e_k$$
$$e_{k+1} = (A - LC) e_k$$

In matrix form:

$$\begin{bmatrix} x_{k+1} \\ e_{k+1} \end{bmatrix} = \begin{bmatrix} A - BK & BK \\ 0 & A - LC \end{bmatrix} \begin{bmatrix} x_k \\ e_k \end{bmatrix}$$

The block-triangular structure means the eigenvalues are the union of the eigenvalues of the diagonal blocks: $\text{eig}(A-BK) \cup \text{eig}(A-LC)$. The controller and observer are **decoupled** in the error dynamics. The $BK$ term in the upper-right means the estimation error *does* affect the state — but it doesn't affect stability, because the error itself is stable and decays to zero.

**When it holds (linear case):**

- Linear time-invariant systems with no constraints: the separation principle is exact and proven.
- LQG (Linear Quadratic Gaussian): LQR controller + Kalman filter = optimal output feedback for linear systems with Gaussian noise. You design $K$ via DARE, $L$ via FARE, combine them. That's LQG.

**When it breaks:**

| Scenario | What happens |
|---|---|
| **Actuator saturation** | $u_k = \text{sat}(-K \hat{x}_k)$. The actual $u_k$ differs from what the observer assumes. Model mismatch → estimation error does not decay as predicted. Integrator windup and observer windup compound. |
| **Nonlinear plants** | The error dynamics are no longer $\dot{e} = (A - LC)e$ — they involve higher-order terms in $e$. The separation doesn't hold. Extended Kalman Filters (EKF) linearize at each step, but the separation is only approximate. |
| **Model mismatch** | If the observer's $(A, B, C)$ differ from the true plant, the error dynamics are driven by the mismatch. The separation principle assumes perfect model knowledge. |
| **Correlated process and measurement noise** | Kalman's derivation assumes $w_k$ and $v_k$ are independent. If they're correlated (e.g., a vibration source affects both the plant and the sensor), the separation holds for stability but optimality is lost. |

**The practical takeaway:** for linear systems far from saturation, the separation principle works and you can treat controller design and observer design as independent tasks. This is why LQR + Luenberger is the standard architecture in motion control, aerospace, and process control. For nonlinear systems or systems that saturate, you need more sophisticated combinations (moving horizon estimation + nonlinear MPC, for instance).

---

## 8. The observer in practice: what the textbook doesn't tell you

### 8.1 The model is always wrong

The observer runs a model in parallel. If $A$ and $B$ are wrong, the prediction step produces garbage. The correction term $L(y - C\hat{x})$ can compensate — but only within the bandwidth of the observer. Slow parameter drift (motor heating) the observer can track. Unmodeled nonlinearities (static friction, backlash) it cannot — they produce persistent innovation that the observer tries to correct, resulting in a biased estimate.

**Remedy:** treat parameter uncertainty as process noise. Increase the corresponding diagonal entries in $Q$ (Kalman) or slow down the observer poles (Luenberger). You'll get a smoother estimate that lags reality — a better trade-off than a fast, wrong estimate.

### 8.2 Sensor bias

Most sensors have DC offsets. A gyro has a bias that drifts with temperature. An accelerometer has a gravity component that depends on mounting alignment. An encoder has an unknown zero position.

The innovation $y_k - C\hat{x}_k$ contains a persistent bias term. The observer cannot distinguish sensor bias from a genuine state error — both produce the same innovation signature. The result: the estimate converges to a biased value.

**Remedy: augment the state.** Add the bias as an unknown constant state:

$$x_{\text{aug}} = \begin{bmatrix} x \\ b \end{bmatrix}, \qquad b_{k+1} = b_k$$

The observer now estimates both the plant state and the sensor bias simultaneously. This is **bias estimation** and it's the standard approach in attitude estimation (estimating gyro bias along with attitude quaternion) and GPS/INS integration.

### 8.3 Missing measurements

Sensors fail. GPS drops out in tunnels. Encoders skip counts. Cameras are occluded. The observer must handle periods with no measurement.

During a measurement dropout, the observer runs open-loop: $\hat{x}_{k+1} = A\hat{x}_k + Bu_k$ (set $L = 0$ for that sample). The error grows as $e_{k+1} = A e_k$. If $A$ is stable (the plant itself is stable), the error is bounded. If $A$ is unstable (inverted pendulum, rocket), the error grows without bound — you need backup sensors or a different strategy.

**Multi-rate Kalman filters** handle measurements arriving at different rates. GPS at 1 Hz, IMU at 100 Hz, camera at 30 Hz. The filter predicts at the IMU rate (100 Hz) and only applies the correction step when a measurement arrives. Each sensor gets its own $C$ matrix and $R$ covariance. This is the architecture of every drone flight controller.

### 8.4 Observer windup

When the actuator saturates, the actual $u_k$ is less than commanded. The observer runs with the commanded $u_k$ — but the plant received less. The innovation spikes. The observer overcorrects. The estimate diverges.

**Remedy:** feed the **actual** (saturated) $u_k$ to the observer, not the commanded $u_k$. This is observer anti-windup and it's as important as integrator anti-windup. Any system with saturation must do this.

---

## 9. Connection to this project

| Doc | The observer connection |
|---|---|
| `lqr_explorer.html` | Assumes full state feedback — $u = -Kx$ with both position and velocity "measured." This is pedagogically clean but physically impossible. To make the LQR explorer work on real hardware, you'd add a Luenberger observer between the encoder reading and the LQR gain, estimating velocity from position. |
| `servo_motor_pid.html` | PID uses $y$ (position) directly. Derivative action approximates velocity by differencing — which is a trivial observer (model = zero-order hold of position). The noise amplification you see when increasing $K_d$ is the observer problem in miniature: poor noise model, ad-hoc filtering. |
| `servo_qp_mpc.html` | The MPC uses full state in its prediction model. A real MPC deployment runs a Kalman filter upstream — the MPC's initial condition is $\hat{x}_{k \mid k}$ from the estimator, not $x_k$ (which is unknown). |
| `core_problems_controller_design.md` | Problem #1 (Measurement) — the observer is the solution. The table in that doc maps the entire measurement problem space: Kalman, Luenberger, Smith predictor. This doc dives deep into the first two. |
| `care_vs_dare.md` | The DARE solves for $K$ in LQR. The FARE (Filter ARE) solves for $P$ in the Kalman filter — same algebraic structure, transposed. The DARE-to-FARE duality is one of the most elegant results in control. |
| `trajectory_tracking_lqr_mpc.md` | Tracking requires knowing the current state to compute the error from the reference trajectory. An observer provides that state from measurements. Without it, trajectory tracking is open-loop prediction. |
| `bellman_to_lqr.md` | The DP recursion that produces LQR starts from full-state knowledge. The observer is the missing piece that makes DP-based control realizable from partial measurements. |
| `kalman_filter_basics.md` | (planned) The Kalman filter as a standalone topic: derivation from Bayes' rule, the update equations, the information form. This doc provides the structural context — why Kalman filters exist in the first place. |

**The deeper pattern:** Every controller in this project — PID, LQR, MPC — needs to know the state. PID gets away with using $y$ directly because industrial processes are often first-order (position *is* the state, no hidden dynamics). LQR exposes the gap explicitly: it demands $x$, forcing you to confront the measurement problem. MPC's prediction horizon makes the gap even wider: wrong initial state → wrong prediction → wrong control. The observer is not optional infrastructure — it is the mechanism that connects the mathematical controller to the physical plant.

---

## 10. Further reading

**Foundational — the original papers:**
- Luenberger, D.G. (1964). "Observing the State of a Linear System." *IEEE Trans. Military Electronics*, 8(2), 74–80. — The paper that introduced the observer structure. Remarkably readable.
- Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems." *ASME J. Basic Engineering*, 82(1), 35–45. — The Kalman filter. Dense but rewarding. The appendix proves the optimality of the gain.

**Textbook treatments — from applied to theoretical:**
- Franklin, G.F., Powell, J.D., & Workman, M.L. (1998). *Digital Control of Dynamic Systems*, 3rd ed. Ellis-Kagle Press. — Chapters 8–9 cover observer design and the separation principle with worked examples. The most practical treatment.
- Åström, K.J. & Wittenmark, B. (1997). *Computer-Controlled Systems*, 3rd ed. Prentice-Hall. — Chapter 6 on pole-placement design with observers; Chapter 7 on the Kalman filter.
- Anderson, B.D.O. & Moore, J.B. (1979). *Optimal Filtering*. Prentice-Hall. — The definitive text on Kalman filtering. Rigorous, complete. Chapters 3–4 cover the discrete-time filter and its properties.

**Duality and the separation principle:**
- Doyle, J.C., Francis, B.A., & Tannenbaum, A.R. (1992). *Feedback Control Theory*. Macmillan. — Chapter 6 presents the separation principle as a unified result within the Youla parameterization framework.
- Zhou, K., Doyle, J.C., & Glover, K. (1996). *Robust and Optimal Control*. Prentice-Hall. — Chapter 14 on LQG and the separation principle; the mathematical structure in full generality.

**Practical estimation — implementation-focused:**
- Grewal, M.S. & Andrews, A.P. (2014). *Kalman Filtering: Theory and Practice Using MATLAB*, 4th ed. Wiley. — Implementation details: covariance propagation, numerical stability (Joseph form, UD factorization, square-root filtering), multi-rate and sequential updates.
- Brown, R.G. & Hwang, P.Y.C. (2012). *Introduction to Random Signals and Applied Kalman Filtering*, 4th ed. Wiley. — Accessible introduction with MATLAB code. Good treatment of sensor bias estimation and GPS/INS integration.

**The observer in motion control:**
- Ellis, G. (2012). *Control System Design Guide*, 4th ed. Butterworth-Heinemann. — Chapter 13 on the Luenberger observer in servo drives: velocity estimation from encoder position, practical tuning rules, acceleration feedforward.
