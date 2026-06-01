# The IP Controller: Killing Overshoot by Moving the Proportional Term

**Add a first-order low-pass filter to the reference input of a PI controller, and you get an IP controller. Not approximately — exactly. The filter cancels the closed-loop zero that causes "proportional kick," turning a jumpy step response into a smooth one without touching the disturbance rejection.**

---

## 1. The problem: proportional kick

In a standard PI controller, the error is $E(s) = R(s) - Y(s)$ — reference minus feedback. The control output is:

$$U(s) = \left(K_p + \frac{K_i}{s}\right) E(s) = K_p(R(s) - Y(s)) + \frac{K_i}{s}(R(s) - Y(s))$$
### 1.1 Deriving the closed-loop transfer function

The derivation follows the signal flow through the feedback loop. Define the signals (dropping $(s)$ for brevity):

- $R$: reference command
- $Y$: plant output (feedback)
- $E = R - Y$: tracking error
- $U$: controller output (control effort)
- $C$: controller transfer function
- $P$: plant transfer function

**Step 1 — Plant equation.** The output is the plant's response to the control signal:

$$Y = P \cdot U$$

**Step 2 — Controller equation.** The control signal is the controller's response to the error:

$$U = C \cdot E$$

Substituting into Step 1:

$$Y = P \cdot C \cdot E$$

**Step 3 — Error definition.** Feedback control defines error as reference minus output: $E = R - Y$. Substitute:

$$Y = P \cdot C \cdot (R - Y)$$

**Step 4 — Collect $Y$ terms.** Expand and gather all $Y$ terms on one side:

$$Y = P C R - P C Y \quad\Rightarrow\quad Y + P C Y = P C R \quad\Rightarrow\quad Y(1 + P C) = P C R$$

**Step 5 — The universal feedback formula.** Divide through by $(1 + PC)$:

$$T(s) = \frac{Y(s)}{R(s)} = \frac{P(s) C(s)}{1 + P(s) C(s)}$$

This is the single most important formula in classical control. Every linear SISO feedback system — PID, LQR, H∞ — reduces to this structure.

### 1.2 Substituting the PI controller

The PI controller in Laplace form is:

$$C(s) = K_p + \frac{K_i}{s} = \frac{K_p s + K_i}{s}$$

Substitute into the universal feedback formula:

$$T_{PI}(s) = \frac{P(s) \cdot \frac{K_p s + K_i}{s}}{1 + P(s) \cdot \frac{K_p s + K_i}{s}}$$

**Clear the nested fraction.** Multiply numerator and denominator by $s$:

- Numerator: $P(s) \cdot (K_p s + K_i)$
- Denominator: $s \cdot 1 + s \cdot \left[P(s) \frac{K_p s + K_i}{s}\right] = s + P(s)(K_p s + K_i)$

Result:

$$T_{PI}(s) = \frac{Y(s)}{R(s)} = \frac{P(s)(K_p s + K_i)}{s + P(s)(K_p s + K_i)}$$

### 1.3 The zero reveals itself

Look at the numerator: $K_p s + K_i$. The $K_p s$ term is a **closed-loop zero**. When the reference $R(s)$ steps, $s R(s)$ produces an impulse-like component. The proportional gain $K_p$ amplifies the instantaneous error and creates a control spike — **proportional kick**. The result is overshoot.
This is not a tuning failure. It's structural. The zero is baked into the PI topology because the proportional term acts on the reference *and* the feedback through the same error signal.

---

## 2. The IP controller: decouple proportional from reference

The IP (Integral-Proportional) controller fixes this with a one-line structural change:

| | PI | IP |
|---|---|---|
| **Integral** | $K_i \int e\,dt$ | $K_i \int e\,dt$ |
| **Proportional** | $K_p e$ | $-K_p y$ |
| **Derivative** | (same pattern) | (same pattern) |

The proportional term acts on the **feedback only**, not the error. The integrator still acts on the error (to eliminate steady-state error). The control law:

$$U(s) = \frac{K_i}{s}(R(s) - Y(s)) - K_p Y(s)$$

Deriving the closed-loop transfer function:

$$T_{IP}(s) = \frac{Y(s)}{R(s)} = \frac{P(s) K_i}{s + P(s)(K_p s + K_i)}$$

Compare $T_{PI}$ and $T_{IP}$:

| | Numerator | Denominator |
|---|---|---|
| **PI** | $P(s)(K_p s + K_i)$ | $s + P(s)(K_p s + K_i)$ |
| **IP** | $P(s) K_i$ | $s + P(s)(K_p s + K_i)$ |

**The denominators are identical.** The characteristic equation, pole locations, and disturbance rejection properties are unchanged. The only difference: the numerator loses the $K_p s$ zero. The step response goes from overshooting to monotonic — same stability margins, same stiffness, zero overshoot.

---

## 3. The filter equivalence: PI + low-pass = IP

Here is the result that makes this more than a wiring trick.

Suppose you cannot modify the controller internals — the PI structure is baked into firmware. Instead, you place a filter $F(s)$ on the reference input: $R^*(s) = F(s) R(s)$. The filtered reference enters a standard PI controller. What $F(s)$ makes the overall system identical to IP?

Set the filtered-PI closed loop equal to the IP closed loop:

$$F(s) \cdot T_{PI}(s) = T_{IP}(s)$$

Substitute the transfer functions:

$$F(s) \cdot \frac{P(s)(K_p s + K_i)}{s + P(s)(K_p s + K_i)} = \frac{P(s) K_i}{s + P(s)(K_p s + K_i)}$$

Cancel the common denominator and $P(s)$:

$$F(s)(K_p s + K_i) = K_i$$

Solve for $F(s)$:

$$F(s) = \frac{K_i}{K_p s + K_i} = \frac{1}{\frac{K_p}{K_i} s + 1}$$

**$F(s)$ is a first-order low-pass filter** with time constant $\tau = K_p / K_i$ — exactly the integral time constant $T_i$ of the PI controller. Cutoff frequency: $\omega_c = K_i / K_p$.

This is exact, not an approximation. No Taylor expansion, no "close enough." The algebra cancels perfectly.

---

## 4. Why do this externally instead of rewriting the controller?

If IP is better for step response, why not just code it directly?

### 4.1 Closed systems

Commercial servo drives, motor controllers, and PLC function blocks ship with fixed PID/PI structures. You cannot modify the firmware. You *can* filter the reference signal before it enters the controller. A first-order IIR filter costs two multiply-adds per sample — essentially free.

### 4.2 Two-degree-of-freedom (2-DOF) design

The filter equivalence reveals a 2-DOF architecture:

```
    r ──→ [F(s)] ──→ [PI] ──→ u ──→ [P(s)] ──→ y
                      ↑                         │
                      └─────────────────────────┘
```

You now have two independent knobs:

1. **The PI gains $(K_p, K_i)$** — tune these for disturbance rejection. Make them aggressive. High stiffness, fast load recovery. The filter will handle the overshoot.

2. **The filter $F(s)$** — set $\tau = K_p/K_i$ for IP-equivalent smooth tracking. Or deliberately set $\tau$ longer for even softer starts. Or set it to 1 (bypass) during aggressive maneuvers where overshoot is acceptable. Or schedule it — tight filter for fine positioning, loose filter for large slews.

This is **independent tuning of servo and regulation performance** — the core promise of 2-DOF control. The PI + filter architecture achieves it without changing a line of controller code.

### 4.3 Bumpless transfer

Switching between "aggressive PI" and "smooth IP" behavior is just a filter parameter change. No gain switching, no integrator reset, no transient. Change $\tau$ and the response shape changes smoothly. This matters in multi-mode systems (coarse approach → fine positioning, speed control → torque control).

---

## 5. What about the D term?

Extending to full PID: the IPD (or I-PD) controller moves **both** proportional and derivative terms to the feedback path:

$$U(s) = \frac{K_i}{s}(R(s) - Y(s)) - K_p Y(s) - K_d s Y(s)$$

The derivative-on-feedback pattern avoids "derivative kick" — the same problem as proportional kick, but worse because differentiating a step produces an infinite spike (a delta function in theory, saturated by rate limits in practice).

The filter equivalence for full PID is slightly different:

$$F(s) = \frac{K_i}{K_d s^2 + K_p s + K_i}$$

This is a second-order low-pass filter. The same principle applies: $K_p$ and $K_d$ in the numerator create zeros; the filter cancels them.

---

## 6. How the filter cutoff relates to tuning

The filter's cutoff frequency $\omega_c = K_i / K_p$ emerges from the PI gains. This gives a design rule:

| If you tune PI to be... | Then $K_p/K_i$ is... | The filter cutoff is... |
|---|---|---|
| **Fast / aggressive** | Small $T_i$ | High — little filtering, keep the speed |
| **Slow / conservative** | Large $T_i$ | Low — heavy filtering, but you already tuned slow |
| **Ziegler-Nichols** | $T_i \approx 0.85\,T_u$ | Around $1.18/T_u$ — moderate filtering |

The filter adapts automatically if you derive it from the PI gains. Tune $K_p$ and $K_i$ for disturbance rejection, then compute $\tau = K_p/K_i$ for the reference filter. The filter never needs independent tuning.

---

## 7. Implementation: three lines of code

In discrete time (sample period $T_s$), the first-order low-pass filter is an exponential moving average:

```c
// PI controller (unchanged)
error = reference_filtered - measurement;
integral += Ki * Ts * error;
u = Kp * error + integral;   // standard PI

// Reference filter (added — two lines)
alpha = Ts / (Kp/Ki + Ts);   // or precompute
reference_filtered = alpha * reference + (1 - alpha) * reference_filtered;
```

If $K_p/K_i$ changes online (adaptive tuning), update `alpha` in the same cycle. The filter state is already valid — it's tracking the current filtered value, which is physically meaningful.

---

## 8. What the filter does not do

The filter equivalence has limits worth knowing:

| Concern | Answer |
|---|---|
| **Does the filter slow down disturbance rejection?** | No. The filter is on the reference path. Disturbances enter at the plant input or output — they bypass the filter entirely. The PI loop sees disturbances with full bandwidth. |
| **Does the filter add phase lag inside the loop?** | No. The filter is outside the feedback loop. Loop gain, phase margin, and gain margin are unchanged. |
| **Does this work with nonlinear plants?** | The derivation assumes linearity, but the *architecture* (proportional-on-feedback, integral-on-error) works on any plant. The filter equivalence is exact for linear plants; for nonlinear plants, it's still the right structure — it just won't be an exact algebraic identity. |
| **What about measurement noise?** | IP and filtered-PI have identical noise properties to standard PI. The proportional term acts on $y$ in both cases (in PI it acts on $r-y$, but the noise comes through $y$ regardless). No difference. |

---

## 9. Connection to this project

The IP controller sits at the intersection of several threads in this repository:

| Doc | Connection |
|---|---|
| `zero_effect_explorer.html` | The proportional kick is a **zero effect** — an LHP zero in the closed-loop transfer function that accelerates response at the cost of overshoot. IP removes that zero. The zero-effect simulator lets you see exactly what happens when you add or remove a zero at $s = -K_i/K_p$. |
| `core_problems_controller_design.md` | Problem 9 (multi-objective trade-offs): IP is a structural solution to the speed-vs-overshoot trade-off. Instead of compromising on a single $K_p$, you get both — aggressive disturbance rejection *and* smooth tracking. |
| `youla_parameterization.md` | The PI → IP transition is a special case of Youla: you're selecting a different $Q(s)$ that removes the zero from the complementary sensitivity. The 2-DOF structure is exactly the IMC architecture with a particular choice of $Q$. |
| `trajectory_tracking_lqr_mpc.md` | LQR tracking adds a feedforward term $u_{\text{ff}} = K_r r$ that translates the reference into control space. The IP filter $F(s)$ is a *dynamic* feedforward — it shapes *how* the reference enters the loop rather than just scaling it. |
| `servo_motor_pid.html` | The PID simulator's anti-windup handles saturation. IP + filter adds another layer: even before saturation, the reference is shaped to avoid exciting the zero that causes overshoot. |

### The deeper pattern

The PI → IP transition is an instance of a recurring theme in this project: **separating what you can separate.** PI couples tracking and regulation through a single error signal. IP decouples them — proportional for regulation, integral for tracking. MPC decouples further — constraints, prediction, and optimization each get their own mechanism. The progression from PI to IP to 2-DOF to MPC is a progression of *decoupling* — pulling apart concerns that were bundled together.

---

## 10. Historical note

The IP controller emerged from industrial motion control in the 1980s–1990s, particularly in Japan. Yaskawa, Mitsubishi, and Fanuc servo drives offered "I-P control" as a configuration option alongside standard PI. The term "I-P" (rather than "IP") is common in Japanese documentation to avoid confusion with "Intellectual Property."

The input-filter equivalence was known to practitioners earlier but was formalized in the academic literature by:

- **Han, J.** (1995). The PID controller with set-point filter. *Control Engineering (Chinese)*. — Han's work on ADRC began from the same observation: move the proportional term off the reference.
- **Åström, K.J. & Hägglund, T.** (1995). *PID Controllers: Theory, Design, and Tuning.* ISA. — Section 3.4 covers set-point weighting as a 2-DOF parameterization, of which IP ($\beta = 0$ in their notation) is the extreme case.

The set-point weighting formulation generalizes the idea:

$$U(s) = K_p(\beta R(s) - Y(s)) + \frac{K_i}{s}(R(s) - Y(s))$$

- $\beta = 1$ → standard PI (proportional on full error)
- $\beta = 0$ → IP (proportional on feedback only)
- $0 < \beta < 1$ → partial set-point weighting (some overshoot, faster rise time)

The filter equivalence $F(s) = 1/((K_p/K_i)s + 1)$ is exactly $\beta = 0$, plus the dynamics of a low-pass filter that the integrator provides.

---

## 11. Further reading

- **Åström, K.J. & Hägglund, T.** (2006). *Advanced PID Control.* ISA. — Chapter 4 on set-point weighting and 2-DOF structures; the definitive reference.
- **Ellis, G.** (2012). *Control System Design Guide*, 4th ed. Butterworth-Heinemann. — Chapter 8 covers IP control in the context of motion systems; strong practical emphasis.
- **Visioli, A.** (2006). *Practical PID Control.* Springer. — Chapter 2 on set-point filtering; includes tuning rules for the filter time constant.
- **Han, J.** (2009). "From PID to Active Disturbance Rejection Control." *IEEE Trans. Industrial Electronics*, 56(3), 900–906. — Traces the philosophical arc from PID through set-point filtering to ADRC.
