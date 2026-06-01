# Why Pure PD Is Physically Unimplementable — and Why Your MCU Code Works Anyway

**Pure PD is non-causal and has infinite high-frequency gain. Theory says it cannot exist. Yet you write `D_out = Kd * (error - last_error)` on an STM32 and the servo runs fine. Both are true. Here's why.**

---

## 1. The theoretical argument: non-causal and non-proper

In the Laplace domain, a pure PD controller is:

$$C(s) = K_p + K_d s$$

The transfer function has **numerator degree 1, denominator degree 0**. This is an *improper transfer function* — the numerator outranks the denominator.

A physically realizable system must be **proper**: denominator degree ≥ numerator degree. The reason is causality. The derivative operation:

$$s X(s) \quad\longleftrightarrow\quad \frac{dx}{dt}$$

requires knowing the instantaneous rate of change. In the frequency domain, $s = j\omega$. As $\omega \to \infty$, the magnitude $|j\omega| \to \infty$. An ideal differentiator has **infinite bandwidth** — it responds to a step with a delta function, which requires infinite energy and infinite slew rate. No physical component (op-amp, transistor, mechanical linkage) can deliver this.

A proper transfer function's denominator provides *roll-off* at high frequencies. The missing denominator on pure PD means no roll-off — the gain climbs without bound.

---

## 2. The engineering argument: noise amplification

This is the one that actually kills it in practice.

Replace $s \to j\omega$. The D term contribution:

$$|K_d \cdot j\omega| = K_d |\omega|$$

| Frequency $\omega$ | D-term gain |
|---|---|
| 1 rad/s | $K_d$ |
| 100 rad/s | $100 \cdot K_d$ |
| $10^4$ rad/s (16 kHz) | $10^4 \cdot K_d$ |
| $\to \infty$ | $\to \infty$ |

Every real sensor signal carries high-frequency noise — thermal noise in the ADC, EMI from nearby switching converters, vibration coupled through the mechanical structure, quantization error from finite bit depth. The noise amplitude might be tiny (a few millivolts, a single ADC count), but its frequency content reaches into the hundreds of kHz.

A pure differentiator multiplies that tiny noise by $\omega$. At 100 kHz, a 1 mV noise spike becomes a 628 V control command. The actuator saturates, the motor screams, the FETs overheat. This isn't a tuning problem — it's structural.

**This is why many industrial PID loops are PI-only.** They disable the D term entirely rather than risk the noise amplification, accepting slower response as the cost of reliability.

---

## 3. The real solution: filtered PD (practical derivative)

No one deploys pure PD. What ships is **filtered PD** — a differentiator cascaded with a first-order low-pass filter:

$$C(s) = K_p + \frac{K_d s}{1 + T_f s}$$

where $T_f$ is the filter time constant, sometimes expressed as $T_d / N$ with $N = 5\sim20$.

Now numerator degree = 1, denominator degree = 1 → **proper**. At low frequencies ($\omega \ll 1/T_f$):

$$\frac{K_d s}{1 + T_f s} \approx K_d s$$

It behaves like a pure differentiator. At high frequencies ($\omega \gg 1/T_f$):

$$\frac{K_d s}{1 + T_f s} \to \frac{K_d}{T_f}$$

The gain **saturates** at a constant $K_d / T_f$. High-frequency noise is amplified by a fixed factor, not by $\omega$. The noise floor rises — you still pay a penalty — but it doesn't diverge to infinity.

### Discrete implementation

In a digital control loop with sample period $T_s$, the filtered derivative is a two-step update:

```c
// Backward-Euler discretization of Kd*s/(1 + Tf*s)
// D_state is the internal filter state

D_state = D_state + (Ts / Tf) * (Kd * (error - prev_error) - D_state);
u = Kp * error + D_state;
```

This is the difference between `(error - last_error)` multiplied by an infinite-gain differentiator, and the same difference passed through a low-pass filter. The code looks similar — both use `error - last_error` — but the filter state bounds the output.

---

## 4. Then why does my bare-metal `D_out = Kd*(error - last_error)` work?

This is the question every embedded developer asks, and the answer reveals something important: **your hardware already contains implicit low-pass filters.** You didn't add them — they were always there.

### 4.1 The sampling period $T_s$ is a frequency ceiling

Continuous-time theory lets $\omega \to \infty$. Your MCU's timer interrupt (e.g., 1 kHz) caps what frequencies can even *exist* in the discrete-time system. The discrete derivative:

$$D[k] = K_d \frac{e[k] - e[k-1]}{T_s}$$

has a bounded output for any bounded input sequence. The Nyquist frequency $f_s/2$ is a hard cutoff — no frequency above it can be represented. The difference between $e[k]$ and $e[k-1]$ is bounded by the signal's maximum slew rate between samples, which is finite because the physical plant has finite velocity.

**The sampler itself acts as an anti-aliasing filter.** Frequencies above Nyquist are aliased, not passed through with infinite gain. This alone prevents the unbounded amplification that the continuous-time theory predicts.

### 4.2 The motor is a mechanical low-pass filter

Suppose your differential term *did* produce a spiky, high-frequency voltage command. What happens next?

- **Electrical:** The motor winding has inductance $L$. The current cannot change instantaneously — $V = L\,di/dt$ limits the slew rate. The inductance is a low-pass filter with cutoff $\omega_c = R/L$.
- **Mechanical:** The rotor has inertia $J$. Torque produces angular acceleration $\alpha = \tau/J$. A 1 ms voltage spike barely moves the rotor before the next sample arrives. The mechanical time constant $\tau_m = J R / (K_t K_e)$ for a typical hobby servo is 10–100 ms — an order of magnitude slower than the control loop.

The motor's transfer function $P(s) = K / (s(Js + b)(Ls + R))$ is strictly proper (denominator degree 3, numerator degree 0). It rolls off at −60 dB/decade after the electrical pole. High-frequency control commands are attenuated mechanically before they produce observable motion.

### 4.3 Hardware filtering on the sensor path

Before the ADC value even reaches your PID loop:

- **RC filter** on the potentiometer or analog input pin (often 10 kΩ + 100 nF → $f_c \approx 160$ Hz)
- **Sampling capacitor** in the ADC's sample-and-hold circuit
- **Oversampling + averaging** if your firmware reads the ADC multiple times per control cycle

Even if you didn't explicitly design these, they're present on most development boards and reference designs. The signal entering `error = setpoint - adc_value` is already band-limited.

### 4.4 Quantization floors the noise

For a potentiometer-based servo with a 10-bit ADC and 270° range:

$$\text{Resolution} = \frac{270^\circ}{1024} \approx 0.26^\circ$$

A vibration smaller than 0.26° produces **zero change** in the ADC reading. The quantization acts as a dead-zone — sub-LSB noise never reaches the differentiator. Your `error - last_error` is 0 most of the time, and jumps by ±1 count when the shaft actually moves.

---

## 5. When your direct difference *will* fail

The implicit filtering works when the mechanical system is slow and the sensor is quiet. It stops working when:

| Scenario | Why it breaks |
|---|---|
| **Drone flight controller** | Frame is light (low inertia), motors spin at 10k+ RPM (fast response), gyro samples at 8 kHz with 16-bit resolution. Mechanical filtering is gone — the 500 Hz frame vibrations pass straight through. Without a D-term LPF, the ESCs overheat and the drone oscillates violently. |
| **Voice coil / linear motor stage** | Friction is near-zero, inertia is low, bandwidth is 100+ Hz. The actuator *can* respond to high-frequency commands. Differential noise produces audible squeal and micron-level jitter. |
| **High-resolution encoders** | A 20-bit optical encoder resolves sub-arcsecond motion. Quantization no longer floors the noise — the sensor is good enough to *see* the vibration, and the D term amplifies it. |
| **Very fast control loops** | At 20+ kHz sample rates, Nyquist moves up. More noise spectrum enters the loop. The sampler provides less free filtering. |
| **Direct-drive motors (no gearbox)** | Gearboxes multiply inertia by $N^2$ — a 100:1 gearbox gives the motor a 10,000× mechanical advantage over the load. Direct-drive removes this filtering. Every torque ripple shows up in the output. |

The common thread: **when the mechanical time constant approaches the control loop period, implicit filtering disappears.** A 1 ms control loop on a system with $\tau_m = 2$ ms is fundamentally different from a 1 ms loop on a system with $\tau_m = 100$ ms.

---

## 6. The deep point: what "physically unimplementable" actually means

The phrase is precise, not rhetorical. It means three things simultaneously:

1. **Causality.** An improper transfer function requires knowledge of future inputs (the derivative at time $t$ depends on the signal at $t$ and $t + \varepsilon$). No physical device can violate causality.

2. **Energy.** Differentiating a step requires infinite instantaneous power. The energy in a step discontinuity is spread across all frequencies; capturing it all means infinite bandwidth, which means infinite energy. Conservation of energy forbids it.

3. **Approximation is always present.** Every "pure" PD implementation is actually filtered PD in disguise — the filter is just implemented by physics (inductance, inertia, sampling, quantization) rather than by code. The question is never *if* there's a filter, but whether the implicit filter's cutoff is well-placed for your application.

The theory isn't wrong — it's explaining what would happen if you removed every physical constraint. Your MCU code works because physics won't let you remove them.

---

## 7. Practical recommendations

| Your system | Recommendation |
|---|---|
| **Hobby servo, < 100 Hz loop** | Direct difference is fine. The mechanics filter everything above 10 Hz. |
| **DC motor with gearbox, 1 kHz loop** | Direct difference still works. Add a moving-average filter on the ADC for safety. |
| **BLDC FOC, 10 kHz current loop** | Filtered derivative is mandatory. $T_f = T_d / 10$ is a reasonable starting point. |
| **Drone, 8 kHz IMU** | Multiple cascaded filters are standard — first-order on D, plus notch filters at the frame resonance. This is not optional. |
| **High-precision CNC** | The D term is usually avoided entirely. Feedforward + PI does the job with zero noise amplification. |

---

## 8. Connection to this project

| Doc | Connection |
|---|---|
| `ip_controller.md` | The IP controller removes the proportional term from the reference path for the same reason — $K_p s$ in the numerator = overshoot. IP removes the P zero; filtered PD tames the D zero. Same structural insight. |
| `zero_effect_explorer.html` | A pure differentiator adds a zero at the origin ($s$). Adding the low-pass filter $1/(1 + T_f s)$ turns that into a zero at the origin *and* a pole at $s = -1/T_f$. The zero-effect simulator shows exactly how this shapes the step response. |
| `core_problems_controller_design.md` | Problem #1 (measurement noise) is the direct motivation for filtering the D term. Problem #8 (computation) is why the filter must be cheap — a two-line IIR is the right trade-off. |
| `servo_motor_pid.html` | The PID simulator models the 3rd-order motor physics ($L$, $J$, $b$) that provide the implicit filtering described in Section 4. The simulator shows that even with `Kd * (error - last_error)`, the motor's own dynamics prevent unbounded output. |

---

## 9. References

- **Åström, K.J. & Hägglund, T.** (2006). *Advanced PID Control.* ISA. — Chapter 3: "Filtering the Derivative." The canonical treatment of practical derivative implementation.
- **Åström, K.J. & Murray, R.M.** (2021). *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd ed. Princeton. — Chapter 10 on transfer functions and properness; the clearest undergraduate exposition of why improper systems are non-causal.
- **Ellis, G.** (2012). *Control System Design Guide*, 4th ed. Butterworth-Heinemann. — Chapter 5: "The D Term and Low-Pass Filters." Practical guidance on choosing $T_f$ for motion systems.
- **Franklin, G.F., Powell, J.D., & Emami-Naeini, A.** (2019). *Feedback Control of Dynamic Systems*, 8th ed. Pearson. — Section 4.4 on PID implementation; covers the $T_d/N$ formulation.
- **Dorf, R.C. & Bishop, R.H.** (2016). *Modern Control Systems*, 13th ed. Pearson. — Section 7.7 on the practical derivative with filtering.
