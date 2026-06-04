# System Identification: Where Models Come From

**Every controller needs a model. LQR needs A, B, C, D. PID needs a plant to tune against. MPC needs a prediction model. None of these come from first principles — they come from data. System identification is the missing first chapter of control theory.**

---

## 1. The problem you didn't know you had

Open `servo_motor_pid.html`. Look at the parameters: armature resistance $R = 2.0\ \Omega$, torque constant $K_t = 0.12\ \text{N·m/A}$, rotor inertia $J = 0.0001\ \text{kg·m}^2$. Where did these numbers come from?

Not from Maxwell's equations. Not from first principles of electromagnetism. Somebody measured them — or guessed, or read a datasheet, or copied them from a similar motor. If the numbers are wrong, the simulated PID, LQR, and MPC are all optimizing for a motor that doesn't exist.

Now imagine you're not simulating. You have a real motor on a bench. A microcontroller. A power stage. An encoder. You want to build a controller. **Step zero is not tuning gains. Step zero is getting the model.**

System identification answers one question: given a pile of input-output data $\{u[k], y[k]\}_{k=1}^N$, what is the system that generated it?

---

## 2. What the data looks like

Hook up your motor. Apply a voltage sequence $u[k]$ (the input). Measure the resulting position $y[k]$ (the output). You get two columns of numbers:

| $k$ | $u[k]$ (volts) | $y[k]$ (encoder counts) |
|-----|----------------|-------------------------|
| 1 | 0.0 | 0 |
| 2 | 3.0 | 12 |
| 3 | 3.0 | 47 |
| 4 | 3.0 | 108 |
| 5 | 3.0 | 197 |
| 6 | 3.0 | 312 |
| ... | ... | ... |

This is all you have. No physics. No differential equations. Just numbers. The question is: can you recover the differential equation that connects $u$ to $y$ from these numbers alone?

The answer is yes — under conditions we'll understand — and the process is system identification.

**The core assumption:** there exists some relationship, however approximate, of the form:

$$y[k] + a_1 y[k-1] + \cdots + a_{n_a} y[k-n_a] = b_1 u[k-1] + \cdots + b_{n_b} u[k-n_b] + e[k]$$

where $e[k]$ is "everything we can't explain" — noise, unmodeled dynamics, nonlinearities. The job is to find $a_i$ and $b_i$ (and the structure — $n_a$, $n_b$, and what to do about $e[k]$) that make this equation hold as closely as possible for the data we have.

This is called a **linear difference equation** model. The coefficients $a_i, b_i$ encode the system's dynamics. From them, you can reconstruct poles, zeros, natural frequencies, damping ratios, and DC gain. Everything a controller needs.

---

## 3. Model structures: ARX, ARMAX, OE

The noise term $e[k]$ is where the three main model families diverge. They differ in what they assume about how noise enters the system — and the choice matters more than most people realize.

### 3.1 ARX: the workhorse

**ARX** = AutoRegressive with eXogenous input. The noise enters *before* the plant dynamics:

$$A(q^{-1}) y[k] = B(q^{-1}) u[k] + e[k]$$

where $q^{-1}$ is the backward shift operator ($q^{-1} y[k] = y[k-1]$), and:

$$A(q^{-1}) = 1 + a_1 q^{-1} + \cdots + a_{n_a} q^{-n_a}$$
$$B(q^{-1}) = b_1 q^{-1} + \cdots + b_{n_b} q^{-n_b}$$

Rewriting:

$$y[k] = \frac{B(q^{-1})}{A(q^{-1})} u[k] + \frac{1}{A(q^{-1})} e[k]$$

The noise filter shares the plant's denominator $A(q^{-1})$. This means the noise is colored by the same dynamics as the plant.

**The great advantage of ARX:** the predictor $\hat{y}[k \mid \theta]$ is **linear in the parameters**:

$$\hat{y}[k] = -a_1 y[k-1] - \cdots - a_{n_a} y[k-n_a] + b_1 u[k-1] + \cdots + b_{n_b} u[k-n_b]$$

This is a linear regression. Stack $N$ data points, and the parameters fall out from a single least-squares solve. No iteration. No local minima. Global optimum in one shot.

**The catch:** ARX is biased when the noise doesn't actually enter through $1/A(q^{-1})$. The bias can be large — the estimates converge to the wrong values even with infinite data.

**When to use ARX:** when you want something fast and you don't have strong noise coloring. First pass on any data. Good enough for many control applications. If the model order is high enough, ARX can approximate any linear system arbitrarily well (the bias shrinks as $n_a, n_b$ increase).

### 3.2 ARMAX: noise has its own dynamics

**ARMAX** = ARX with Moving Average noise:

$$A(q^{-1}) y[k] = B(q^{-1}) u[k] + C(q^{-1}) e[k]$$

where $C(q^{-1}) = 1 + c_1 q^{-1} + \cdots + c_{n_c} q^{-n_c}$.

The noise gets its *own* filter $C(q^{-1})$, independent of $A(q^{-1})$. This is more realistic: measurement noise often has different spectral characteristics than the plant dynamics.

The predictor becomes:

$$\hat{y}[k] = \frac{B(q^{-1})}{C(q^{-1})} u[k] + \frac{C(q^{-1}) - A(q^{-1})}{C(q^{-1})} y[k]$$

This is **nonlinear in the parameters** ($c_i$ appear in denominators). You need iterative optimization — typically prediction error minimization (PEM) — which means local minima, initialization sensitivity, and more computation.

**When to use ARMAX:** when the noise spectrum is clearly different from the plant spectrum, and you need accurate parameter estimates (not just good prediction). Common in econometrics, less common in control where OE is often preferred.

### 3.3 OE: Output Error — what control engineers actually want

**OE** = Output Error. The noise enters *after* the plant dynamics:

$$y[k] = \frac{B(q^{-1})}{F(q^{-1})} u[k] + e[k]$$

where $F(q^{-1}) = 1 + f_1 q^{-1} + \cdots + f_{n_f} q^{-n_f}$. Note: $F(q^{-1})$ is used instead of $A(q^{-1})$ to emphasize that it's the plant denominator, not the noise denominator.

The noise is **white at the output** — uncorrelated, honest measurement noise. This is the physically natural assumption for most control applications.

The predictor is the *simulated* output, not the one-step-ahead prediction:

$$\hat{y}[k] = \frac{B(q^{-1})}{F(q^{-1})} u[k]$$

**This is also nonlinear in the parameters.** The predictor involves past *simulated* outputs, not past measured outputs. You need iterative PEM.

**When to use OE:** when you care about the *simulation* properties of your model (which you do, for control design). OE directly minimizes simulation error. ARX minimizes one-step-ahead prediction error — which can produce models that predict well one step ahead but simulate poorly over a horizon. For MPC, you need a model that simulates well. Use OE.

### 3.4 The bias-variance trade-off in model structure

| Structure | Parameter count | Bias risk | Variance | Solver |
|-----------|----------------|-----------|----------|--------|
| ARX | $n_a + n_b$ | High (wrong noise model) | Low | Linear least-squares |
| ARMAX | $n_a + n_b + n_c$ | Medium | Medium | Iterative PEM |
| OE | $n_f + n_b$ | Low (correct noise structure) | Higher | Iterative PEM |
| High-order ARX | Large $n_a + n_b$ | Low (approximates OE) | High | Linear least-squares |

The practical truth: for control, use high-order ARX first (it's one shot, it's unbiased asymptotically), then validate. If the residual test fails, switch to OE. ARMAX is rarely the right choice in control applications — the extra noise modeling flexibility doesn't help you build a better controller.

---

## 4. Least-squares estimation: the engine underneath

ARX estimation is ordinary least squares. Let's walk through it concretely.

### 4.1 The setup

We have the ARX predictor (order $n_a = 2, n_b = 2$):

$$\hat{y}[k] = \underbrace{\begin{bmatrix} -y[k-1] & -y[k-2] & u[k-1] & u[k-2] \end{bmatrix}}_{\varphi^T[k]} \underbrace{\begin{bmatrix} a_1 \\ a_2 \\ b_1 \\ b_2 \end{bmatrix}}_{\theta}$$

For $N$ data points, stack them:

$$\underbrace{\begin{bmatrix} \hat{y}[3] \\ \hat{y}[4] \\ \vdots \\ \hat{y}[N] \end{bmatrix}}_{\hat{Y}} = \underbrace{\begin{bmatrix} -y[2] & -y[1] & u[2] & u[1] \\ -y[3] & -y[2] & u[3] & u[2] \\ \vdots & \vdots & \vdots & \vdots \\ -y[N-1] & -y[N-2] & u[N-1] & u[N-2] \end{bmatrix}}_{\Phi} \underbrace{\begin{bmatrix} a_1 \\ a_2 \\ b_1 \\ b_2 \end{bmatrix}}_{\theta}$$

$$\hat{Y} = \Phi \theta$$

### 4.2 The normal equations

We want $\theta$ that minimizes the sum of squared one-step-ahead prediction errors:

$$J(\theta) = \sum_{k=n+1}^{N} (y[k] - \hat{y}[k])^2 = \|Y - \Phi\theta\|^2$$

This is a quadratic in $\theta$. Set the gradient to zero:

$$\frac{\partial J}{\partial \theta} = -2\Phi^T(Y - \Phi\theta) = 0$$

$$\Phi^T \Phi \theta = \Phi^T Y$$

$$\boxed{\hat{\theta} = (\Phi^T \Phi)^{-1} \Phi^T Y}$$

These are the **normal equations**. The matrix $\Phi^T \Phi$ is (up to scaling) the sample covariance of the regressors. It must be invertible — which requires *persistent excitation* (the input must be rich enough to probe all the dynamics).

### 4.3 A concrete 2nd-order example

Suppose the true system is a mass-spring-damper sampled at $T_s = 0.01$ s:

$$G(s) = \frac{100}{s^2 + 2s + 100} \quad \longrightarrow \quad G(z) = \frac{0.0049 z^{-1} + 0.0048 z^{-2}}{1 - 1.97 z^{-1} + 0.98 z^{-2}}$$

The true parameters are $a_1 = -1.97$, $a_2 = 0.98$, $b_1 = 0.0049$, $b_2 = 0.0048$.

Now run an experiment: apply a random input (uniform white noise, ±1 V), measure the output with small measurement noise ($\sigma = 0.001$), 1000 samples.

The regressor matrix for the first few rows:

$$\Phi_{1:5} = \begin{bmatrix} -y[2] & -y[1] & u[2] & u[1] \\ -y[3] & -y[2] & u[3] & u[2] \\ -y[4] & -y[3] & u[4] & u[3] \\ -y[5] & -y[4] & u[5] & u[4] \\ -y[6] & -y[5] & u[6] & u[5] \end{bmatrix}$$

Solving $(\Phi^T \Phi)^{-1} \Phi^T Y$ gives:

$$\hat{\theta} = \begin{bmatrix} -1.968 \\ 0.979 \\ 0.00491 \\ 0.00482 \end{bmatrix}$$

Compare to truth: $a_1$ off by 0.1%, $a_2$ off by 0.1%, $b_1$ off by 0.2%, $b_2$ off by 0.4%. From 1000 noisy data points, we recovered the exact system to within half a percent.

This is the power of least squares: when the model structure is right and the excitation is adequate, consistency is guaranteed.

### 4.4 The hat matrix and the residual

The predicted output is:

$$\hat{Y} = \Phi \hat{\theta} = \Phi(\Phi^T\Phi)^{-1}\Phi^T Y = H Y$$

$H = \Phi(\Phi^T\Phi)^{-1}\Phi^T$ is the **hat matrix**. It projects $Y$ onto the column space of $\Phi$. The diagonal elements $h_{ii}$ measure the *leverage* of data point $i$ — how much it influences its own prediction.

The residual:

$$\varepsilon = Y - \hat{Y} = (I - H)Y$$

should be white (uncorrelated) if the model is correct. The sample autocorrelation of $\varepsilon[k]$ is the primary diagnostic.

**Estimating the parameter covariance.** Under white Gaussian noise with variance $\sigma^2$:

$$\text{Cov}(\hat{\theta}) = \sigma^2 (\Phi^T \Phi)^{-1}$$

$$\hat{\sigma}^2 = \frac{\|Y - \Phi\hat{\theta}\|^2}{N - \dim(\theta)}$$

This gives you standard errors on every parameter. A parameter whose estimate is smaller than its standard error is statistically indistinguishable from zero — your model order is probably too high.

---

## 5. PRBS design: how to excite the system

You can't identify a system from data where the input never changes. You need **persistent excitation**. The input spectrum must have power at all frequencies where the plant has dynamics.

### 5.1 Why pseudo-random binary

A **PRBS** (Pseudo-Random Binary Sequence) switches between two levels ($+A, -A$) in a pattern that looks random but is deterministic and reproducible. It's generated by a shift register with feedback:

```
bit 0 → bit 1 → bit 2 → ... → bit n → [XOR feedback] → bit 0
```

A maximum-length PRBS of order $m$ has period $2^m - 1$.

**Why binary?** Because real actuators are amplitude-limited. A binary signal at $\pm A$ has the maximum possible power for a given amplitude constraint. This maximizes signal-to-noise ratio. If you're stuck with $\pm 12$ V, use all of it.

**Why pseudo-random?** Because a true random sequence would be different every experiment — you couldn't average runs, compare models, or reproduce results. PRBS is deterministic: the same seed gives the same sequence.

### 5.2 How long, how fast

Two rules of thumb:

**Rule 1 — Period > settling time.** The PRBS period must exceed the plant's settling time. Otherwise, the plant never reaches steady state between transitions, and you can't identify the DC gain. For a motor with settling time $T_{\text{settle}} \approx 0.1$ s, you need:

$$T_{\text{period}} = (2^m - 1) \cdot T_{\text{bit}} > T_{\text{settle}}$$

**Rule 2 — Bit rate > 5× bandwidth.** The clock period $T_{\text{bit}}$ must be fast enough to excite the plant's dynamics. The PRBS has power from roughly $1/T_{\text{period}}$ to $1/(3 T_{\text{bit}})$. To cover the plant bandwidth $\omega_B$:

$$\frac{1}{T_{\text{bit}}} > 5 \cdot \omega_B$$

For the motor: $\omega_B \approx 100$ rad/s. So $T_{\text{bit}} < 1/500 = 2$ ms. A PRBS-7 ($m=7$, period 127) at 2 ms gives $T_{\text{period}} = 0.254$ s, which easily covers the 0.1 s settling time.

**Rule 3 — Amplitude.** As large as the system can safely handle. Higher amplitude → higher SNR → better estimates. But don't break anything. For the servo motor, ±12 V is the supply rail — use it.

### 5.3 The experiment

Run the PRBS for 5–10 periods. Discard the first period (transient from initial conditions). The remaining data forms your identification set. Save one period for validation (never used in fitting).

Total data: $(2^m - 1) \times N_{\text{periods}}$ points. For $m=7$, 8 periods, discard 1, use 6 for identification, 1 for validation: 762 identification points, 127 validation points. Plenty for a 2nd-order model.

---

## 6. Model order selection: the art

You don't know $n_a, n_b$ in advance. You must choose them.

### 6.1 The information criteria

Fit ARX models of increasing order. For each, compute:

**AIC** (Akaike Information Criterion):

$$\text{AIC} = N \ln(\hat{\sigma}^2) + 2p$$

**BIC** (Bayesian Information Criterion, also called MDL — Minimum Description Length):

$$\text{BIC} = N \ln(\hat{\sigma}^2) + p \ln(N)$$

where $p = n_a + n_b$ is the number of parameters and $\hat{\sigma}^2$ is the residual variance.

AIC and BIC penalize complexity differently: BIC penalizes harder (via $\ln N$), favoring simpler models for large $N$. Both balance fit quality against parameter count.

Plot AIC vs model order. The minimum is your candidate. But don't just pick the minimum — look for the "elbow" where adding more parameters stops helping meaningfully.

### 6.2 The danger of overfitting

Here's a trap that has trapped everyone. Fit a 20th-order ARX model to 1000 data points:

$$\hat{\sigma}^2_{\text{train}} = 0.0003 \quad \text{(looks excellent!)}$$

Now compute the simulation error on the validation set:

$$\text{RMSE}_{\text{val}} = 0.47 \quad \text{(terrible)}$$

What happened? The 20th-order model has 40 parameters. With 1000 data points, that's only 25 points per parameter. The model fitted the noise, not the signal. It memorized the training data but learned nothing about the system.

This is the **bias-variance trade-off**: low-order models have high bias (systematic error from insufficient flexibility) but low variance (stable across different data sets). High-order models have low bias but high variance — they change wildly with different data realizations.

**The universal sign of overfitting:** excellent one-step-ahead prediction, terrible multi-step simulation. The model predicts $y[k+1]$ from the true $y[k]$ perfectly — because it memorized the noise pattern. But when you simulate (feeding predicted outputs back as inputs), the errors compound and explode. This is why OE validation is essential: simulate the model, don't just predict one step.

---

## 7. Validation: is your model any good?

A model that fits the training data is not necessarily a good model. A model that passes validation tests is.

### 7.1 Residual whiteness test

Compute the one-step-ahead residuals:

$$\varepsilon[k] = y[k] - \hat{y}[k \mid k-1]$$

If the model captures all the dynamics, $\varepsilon[k]$ should be white — uncorrelated with itself at any lag. Compute the normalized autocorrelation:

$$\hat{\rho}_\varepsilon(\tau) = \frac{\sum_{k=\tau+1}^{N} \varepsilon[k] \varepsilon[k-\tau]}{\sum_{k=1}^{N} \varepsilon^2[k]}$$

For white noise, $\hat{\rho}_\varepsilon(\tau) \approx 0$ for $\tau \geq 1$, with 95% of values falling within $\pm 1.96 / \sqrt{N}$.

If you see significant autocorrelation at small lags — your model is missing dynamics. Increase the order. If you see correlation at a specific lag — there's an unmodeled resonance. Add a notch.

### 7.2 Cross-correlation test

The residuals should also be uncorrelated with *past* inputs (the system can't anticipate the future, so correlation with future inputs is unphysical, but correlation with past inputs means the model left information on the table):

$$\hat{\rho}_{\varepsilon u}(\tau) = \frac{\sum_{k} \varepsilon[k] u[k-\tau]}{\sqrt{\sum \varepsilon^2[k] \sum u^2[k]}}$$

for $\tau \geq 0$. If significant at any $\tau \geq 0$, the model is missing input-output dynamics. This is a stronger test than the whiteness test — it specifically checks for unmodeled transfer function dynamics.

### 7.3 Cross-validation: the gold standard

Split your data: 70% for identification, 30% for validation. Fit on the identification set. Then evaluate on the validation set using **simulation** (not one-step-ahead prediction):

$$\text{Fit} = 100 \times \left(1 - \frac{\|y_{\text{val}} - \hat{y}_{\text{sim}}\|}{\|y_{\text{val}} - \bar{y}_{\text{val}}\|}\right)$$

A fit of 100% means perfect simulation. Above 90% is excellent. Below 70% is suspect. Below 50% — throw it out.

Never validate on the same data you trained on. This is the cardinal rule of system identification. Training fit always improves with model order. Validation fit peaks, then degrades as overfitting sets in. The peak is your model order.

---

## 8. From discrete to continuous: the d2c problem

Your ARX model is discrete-time:

$$y[k] + a_1 y[k-1] + a_2 y[k-2] = b_1 u[k-1] + b_2 u[k-2]$$

But LQR and most control design tools want a continuous-time state-space model:

$$\dot{x} = A_c x + B_c u$$

How do you get from one to the other?

### 8.1 Zero-order hold inverse

If the data was collected with a ZOH (constant input between samples, which is what every DAC does), then the discrete-time system is:

$$A_d = e^{A_c T_s}, \qquad B_d = \int_0^{T_s} e^{A_c \tau} B_c \, d\tau$$

To invert: from $A_d$, compute $A_c = \frac{1}{T_s} \ln(A_d)$ — the matrix logarithm. From $B_d$, solve $B_c = (A_d - I)^{-1} A_c B_d$. This requires that $T_s$ is small enough that the logarithm is well-defined (no aliasing of fast poles).

### 8.2 Transfer function approach

Convert the ARX polynomial to a discrete transfer function:

$$G(z) = \frac{b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}} = \frac{b_1 z + b_2}{z^2 + a_1 z + a_2}$$

Then apply the Tustin (bilinear) transform: $z = \frac{1 + s T_s/2}{1 - s T_s/2}$, or ZOH equivalent: solve $G(s)$ from $G(z) = \mathcal{Z}\{ \text{ZOH} \cdot G(s) \}$.

### 8.3 The practical answer

For the motor model used in this project, the continuous-time parameters were chosen to be physically plausible, not identified from hardware. But if you had a real motor, the workflow would be:

1. Apply PRBS voltage signal, record position
2. Fit ARX(2,2) via least squares → $\hat{a}_1, \hat{a}_2, \hat{b}_1, \hat{b}_2$
3. Validate: residual whiteness, cross-validation simulation fit
4. Convert to continuous: $A_c, B_c$ from ZOH inverse
5. Use $A_c, B_c$ in LQR design; discretize again for MPC

The model only needs to be good enough to control. A 5% error in $J$ (inertia) means the LQR gains are suboptimal by a few percent — not catastrophic. System identification is robust: you don't need perfection, you need adequacy.

---

## 9. Why identification matters: GIGO

**Garbage In, Garbage Out.** This is not a slogan — it's a theorem. If your model is wrong, your controller optimizes for the wrong plant. The mismatch can produce:

- **Sluggish response:** the controller thinks the plant is heavier than it is, so it's too cautious
- **Oscillation:** the controller thinks the plant responds faster than it does, so it overdrives
- **Steady-state error:** wrong DC gain means the feedforward term is wrong
- **Instability:** worst case — the controller pushes at a frequency where the true plant has 180° phase lag that the model doesn't capture

Every controller in this project — PID in `servo_motor_pid.html`, LQR in `lqr_explorer.html`, MPC in `servo_qp_mpc.html` — is designed for a plant with parameters $R = 2.0\ \Omega$, $L = 0.005$ H, $K_t = 0.12$ N·m/A, $J = 0.0001$ kg·m$^2$, $B = 0.0005$ N·m·s/rad.

If the real motor has $J = 0.0002$ (twice the inertia), the LQR gains are too aggressive by roughly a factor of $\sqrt{2}$. The response overshoots. If $R = 4.0\ \Omega$ (twice the resistance), the current loop has half the bandwidth — high-frequency performance degrades. These are real effects that system identification catches before you waste time tuning a controller for a motor that doesn't exist.

**The hierarchy of modeling effort:**

| Approach | Effort | Model quality | When to use |
|----------|--------|--------------|-------------|
| Datasheet values | Zero | Approximate | Prototyping, education |
| Physics + hand tuning | Low | Okay | When physics is simple and parameters are few |
| System identification | Medium | Good | When you have hardware and need a trustworthy model |
| Physics + identification (grey-box) | High | Best | When you know the structure but not the parameters |

The simulators in this project use datasheet-like parameters — the educational sweet spot where the numbers are physically meaningful but not individually measured. A real deployment would run identification.

---

## 10. Connection to this project

| Document / Simulator | How system identification connects |
|----------------------|-----------------------------------|
| `servo_motor_pid.html` | Motor parameters ($R, L, K_t, J, B$) are the ground truth. In a real build, these would come from identification — PRBS voltage → position data → ARX → transfer function → physics mapping |
| `servo_qp_mpc.html` | The discrete-time prediction model ($A_d, B_d$) is derived from continuous-time physics. With real hardware, you'd identify $A_d, B_d$ directly from data, bypassing the physics entirely (black-box approach) |
| `lqr_explorer.html` | LQR design needs $A, B$. The explorer uses the physics model. On hardware, you'd identify $A, B$ first, then design LQR — the identification step is logically prior to the control step |
| `core_problems_controller_design.md` | Problem #4 (Model Uncertainty) is why identification matters. Every model has uncertainty; identification quantifies it (parameter covariance, residual spectrum) |
| `bellman_to_lqr.md` | LQR solves Bellman on a *given* model. Where does the model come from? Identification. The missing chapter before the Riccati equation |
| `care_vs_dare.md` | Discrete-time LQR (DARE) operates on $A_d, B_d$ — which are exactly what discrete-time identification produces. The continuous-time CARE operates on $A_c, B_c$ from the d2c step |
| `youla_parameterization.md` | The nominal model $G(s)$ in Youla comes from identification. The uncertainty set (how much $G$ can deviate) is also estimated from identification residuals |
| `from_lp_to_qp_to_lqr.md` | The QP in MPC condenses a prediction model. The prediction model's accuracy determines MPC performance. Identification → model → condensing → QP → control |

---

## 11. Further reading

**Start here — accessible and practical:**
- Ljung, L. (1999). *System Identification: Theory for the User*, 2nd ed. Prentice Hall. — The bible. Chapter 4 (ARX/ARMAX/OE) and Chapter 7 (model structure selection) are the essential sections. Ljung's writing is clear, and the MATLAB System Identification Toolbox follows this book exactly.

**The applied reference:**
- Ljung, L. & Glad, T. (1994). *Modeling of Dynamic Systems.* Prentice Hall. — Shorter than Ljung (1999), more focused on the "how" than the "why." Excellent for engineers who need to get an identification experiment running.

**PRBS and experiment design:**
- Godfrey, K.R. (1993). *Perturbation Signals for System Identification.* Prentice Hall. — Everything about PRBS, multi-sine, chirp, and step signals. Chapter 3 covers PRBS design rules in detail.

**The theoretical foundations:**
- Söderström, T. & Stoica, P. (1989). *System Identification.* Prentice Hall. — The complete statistical treatment. Consistency, asymptotic normality, Cramér-Rao bounds. For when you need proofs, not just recipes.

**Grey-box identification (physics + data):**
- Bohlin, T. (2006). *Practical Grey-box Process Identification.* Springer. — When you know the differential equation structure (from physics) but not the parameters. Much more efficient than black-box ARX when the structure is right.

**Online identification and adaptive control:**
- Åström, K.J. & Wittenmark, B. (1995). *Adaptive Control*, 2nd ed. Addison-Wesley. — Chapters 2–4 cover recursive least squares and online identification. The bridge from offline identification to controllers that update their own models in real time.

**The original Ljung classic:**
- Ljung, L. (1978). "Convergence analysis of parametric identification methods." *IEEE Trans. Automatic Control*, 23(5), 770–783. — The paper that proved PEM converges under weak conditions. The mathematical foundation for everything in MATLAB's `ssest()` and `pem()`.

**For this project specifically:**
- The motor simulators (`servo_motor_pid.html`, `lqr_explorer.html`, `servo_qp_mpc.html`) use a 3rd-order motor model ($RLC$ + mechanical). A real identification experiment would recover this as a 3rd-order ARX or OE model. The d2c conversion would recover a 3rd-order state-space model, which could then be compared to the physics-based parameters. The agreement (or disagreement) between identified and physics-based parameters is itself a validation of both approaches.
