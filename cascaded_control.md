# Cascaded Control Loops: Why Real Motor Drives Have Three Controllers, Not One

**You already know that the DC motor in `servo_motor_pid.html` has three states — position, velocity, and current — and one PID controller that stares at the position error and outputs a voltage. That works. But walk into any factory, open any industrial servo drive, and you'll find *three* separate controllers arranged in a chain: current, velocity, position. Why triple the complexity? Because cascading isn't just a design choice — it's a way to make the control problem tractable. Each loop fights exactly the disturbances at its own timescale, and no single controller has to see everything at once. The cascade turns one hard problem into three easy ones.**

---

## 1. The problem with one big controller

You have a DC motor. You want to control its shaft position $\theta$. The plant is third-order:

$$\frac{\theta(s)}{V(s)} = \frac{K_t}{s\left[(Ls + R)(Js + B) + K_t K_e\right]}$$

In `servo_motor_pid.html`, a single PID controller handles this. It works — the simulator proves it. But look closer at what PID has to do:

- **Reject load torque disturbances** that hit the mechanical dynamics ($Js + B$) — timescale: milliseconds to tens of milliseconds.
- **Push current fast enough** to overcome the electrical dynamics ($Ls + R$) — timescale: microseconds to milliseconds.
- **Keep the integrator from winding up** when the voltage saturates at ±Vmax.
- **Maintain stability** across three orders of poles — the electrical pole at $R/L$ (typically hundreds of rad/s), the mechanical pole at $B/J$ (tens of rad/s), and the integrator at the origin.

One PID controller does all of this with three gains. The result is always a compromise. If you tune Kp aggressively for fast response, you risk exciting the electrical dynamics. If you add derivative action to damp the mechanical resonance, you amplify measurement noise. If you add integral action for zero steady-state error, you introduce windup.

The fundamental issue: **a single controller has a single bandwidth.** It sees the plant as one monolithic transfer function. Every design degree of freedom — gain, phase, bandwidth — is shared across all the physics. You can't independently set how fast the current responds, how damped the velocity is, and how accurately the position tracks. Everything couples through three knob turns.

Cascaded control solves this by decomposing the plant along its natural timescale boundaries. Instead of one controller for everything, you get a hierarchy of controllers, each with its own bandwidth and its own disturbance rejection task.

---

## 2. The idea: slice the plant at its timescale boundaries

The DC motor has a natural cascade structure. The voltage $V$ produces current $i$ through the electrical dynamics. The current $i$ produces torque, which drives velocity $\omega$ through the mechanical dynamics. The velocity $\omega$ integrates to position $\theta$. Physically, everything flows one way:

$$V \longrightarrow i \longrightarrow \tau \longrightarrow \omega \longrightarrow \theta$$

If the electrical dynamics are much faster than the mechanical dynamics — and they almost always are, because $L/R \ll J/B$ — we can treat the current loop and the velocity loop as operating at different timescales. From the velocity loop's perspective, the current loop is "essentially instantaneous." From the position loop's perspective, the velocity loop is "essentially instantaneous."

This is the cascade principle: **design the inner loop first, close it, simplify its closed-loop response to a low-order approximate model, then design the next outer loop against that simpler model.** Repeat outward.

| Loop | Controls | Bandwidth (typical) | Sees |
|------|----------|---------------------|------|
| Current (innermost) | Motor current $i$ | 1–5 kHz | Electrical dynamics ($L$, $R$) |
| Velocity (middle) | Shaft speed $\omega$ | 100–500 Hz | Mechanical dynamics ($J$, $B$) |
| Position (outermost) | Shaft angle $\theta$ | 10–50 Hz | Pure integrator + velocity loop |

Each loop only has to handle the physics at its own timescale. The current loop fights back-EMF and supply voltage fluctuations. The velocity loop fights load torque disturbances and friction. The position loop fights reference tracking error. No single controller is stretched across three orders of magnitude in dynamics.

---

## 3. The current loop: making torque on demand

### 3.1 Why it must be the fastest

The current loop is the innermost loop — the one that directly commands the power electronics. Its job is simple: make the actual motor current $i$ track a commanded current $i_{\text{ref}}$ provided by the velocity controller. If the current loop can't follow $i_{\text{ref}}$ quickly, the velocity controller is issuing torque commands the motor can't execute, and the whole cascade degrades.

The electrical dynamics of a brushed DC motor:

$$L \frac{di}{dt} + R i + K_e \omega = V$$

where $K_e \omega$ is the back-EMF term — a disturbance from the current loop's perspective. The transfer function from voltage to current:

$$\frac{i(s)}{V(s)} = \frac{1}{Ls + R} \qquad (\text{ignoring back-EMF for design})$$

The pole is at $s = -R/L$. For a typical servo motor ($R = 4\,\Omega$, $L = 0.02\,\text{H}$), this is 200 rad/s — fast, but not instantaneous. A 12V step produces a current rise with time constant $L/R = 5\,\text{ms}$.

### 3.2 PI control for the current loop

Current control is almost always PI — no derivative, because the electrical dynamics are first-order and there's no mechanical resonance to damp:

$$C_i(s) = K_{p,i} + \frac{K_{i,i}}{s}$$

The integral term ensures zero steady-state current error even with back-EMF disturbance. The closed-loop current response becomes:

$$T_i(s) = \frac{i(s)}{i_{\text{ref}}(s)} \approx \frac{1}{\tau_i s + 1}$$

where $\tau_i$ is set by the PI gains to be approximately $1/(2\pi \cdot 2000)\, \text{s}$ — a first-order lag with bandwidth around 2 kHz. This is the key simplification: from the velocity loop's perspective, the current loop is just a first-order lag with a pole you placed. You designed it, so you know its dynamics exactly.

### 3.3 Saturation at the current level

The current loop also provides the first line of saturation defense. Every motor has a maximum current $i_{\max}$ (set by thermal limits, demagnetization risk, or power electronics). The current loop's output $V_{\text{cmd}}$ is clamped to the DC bus voltage $\pm V_{\text{bus}}$, but the *command* $i_{\text{ref}}$ can also be clamped to $\pm i_{\max}$. This means the velocity loop never sees the electrical limits — it just sees a current source that saturates cleanly at $\pm i_{\max}$. Each loop manages its own actuator limits.

---

## 4. The velocity loop: damping the mechanics

### 4.1 What it sees

With the current loop closed, the plant seen by the velocity controller is:

$$G_v(s) = T_i(s) \cdot \frac{K_t}{Js + B}$$

If we've tuned the current loop to be fast enough — 5–10× the desired velocity loop bandwidth — we can approximate $T_i(s) \approx 1$, ignoring the small first-order lag. The velocity loop sees a simple first-order plant:

$$G_v(s) \approx \frac{K_t}{Js + B}$$

This is the payoff of cascading. A third-order plant has been reduced to a first-order one for the velocity designer.

### 4.2 PI control for velocity

The velocity loop is also typically PI:

$$C_v(s) = K_{p,v} + \frac{K_{i,v}}{s}$$

Why PI and not PID? The plant $K_t/(Js + B)$ is first-order, so a PI controller produces a second-order closed loop — we can place both poles exactly where we want them. No derivative term needed; the integral handles steady-state error from friction and load torque.

The tuning rule comes from pole placement. With the approximate plant $G_v(s) = K_t / (Js + B)$ and PI controller $K_{p,v} + K_{i,v}/s$, the closed-loop characteristic equation is:

$$s^2 + \frac{B + K_t K_{p,v}}{J} s + \frac{K_t K_{i,v}}{J} = 0$$

Match this to a standard second-order form $s^2 + 2\zeta \omega_n s + \omega_n^2$:

$$K_{p,v} = \frac{2\zeta \omega_n J - B}{K_t}, \qquad K_{i,v} = \frac{\omega_n^2 J}{K_t}$$

Choose $\zeta$ (typically 0.7–1.0) and $\omega_n$ (the desired velocity loop bandwidth, e.g., $2\pi \cdot 200$ rad/s). The gains fall out.

### 4.3 Disturbance rejection at the mechanical level

The velocity loop's key job is rejecting load torque disturbances. A load torque $\tau_L$ enters between the current and the velocity:

$$\omega(s) = \frac{K_t}{Js + B}\, i(s) - \frac{1}{Js + B}\, \tau_L(s)$$

The velocity loop sees $\tau_L$ as an input disturbance. Its PI controller fights it — the integral term forces the average velocity error from a constant load to zero, and the proportional term provides immediate reaction. Because the velocity loop runs at 200+ Hz, it can reject torque disturbances that change over tens of milliseconds — a robot arm encountering varying load, a spindle cutting into material.

Critically: the position loop never sees these disturbances. They're rejected at the velocity level, long before they integrate into a position error. This is the second structural advantage of cascading — **disturbance rejection at the earliest possible point in the physical chain.**

---

## 5. The position loop: tracking the reference

### 5.1 The simplest plant of all

After closing the velocity loop and approximating its closed-loop response as a first-order lag:

$$T_v(s) = \frac{\omega(s)}{\omega_{\text{ref}}(s)} \approx \frac{1}{\tau_v s + 1}$$

The position loop sees a cascaded plant:

$$G_p(s) = T_v(s) \cdot \frac{1}{s} = \frac{1}{s(\tau_v s + 1)}$$

This is a second-order plant — a pure integrator followed by the velocity loop's lag. No motor parameters appear at all. The physics have been abstracted away by the inner loops.

### 5.2 P control (often enough)

Because the plant already includes a pure integrator (position from velocity), the position loop often uses pure proportional control:

$$C_p(s) = K_{p,p}$$

The closed-loop position response becomes:

$$T_p(s) = \frac{K_{p,p}}{s(\tau_v s + 1) + K_{p,p}} = \frac{\omega_p^2}{s^2 + 2\zeta_p \omega_p s + \omega_p^2}$$

where $\omega_p = \sqrt{K_{p,p} / \tau_v}$ and $\zeta_p = 1/(2\sqrt{K_{p,p} \tau_v})$. This is a standard second-order system. The position gain $K_{p,p}$ directly sets the bandwidth: larger $K_{p,p}$ → faster response → less damping.

When is integral needed in the position loop? Only when the velocity loop isn't perfect — if there's stiction that causes the velocity loop integrator to saturate, or if the velocity measurement has significant offset. In well-designed servo drives, the velocity loop's integral action provides the zero-steady-state-error guarantee, and the position loop stays P-only. Adding integral action at the outermost loop is risky: it adds a slow mode that can wind up and is hard to tame.

### 5.3 Feedforward for tracking

The position loop can be dramatically improved with feedforward. If the reference trajectory $\theta_{\text{ref}}(t)$ is known in advance — a CNC toolpath, a robot joint profile — you can compute the required velocity and inject it directly:

$$\omega_{\text{ref}} = K_{p,p} (\theta_{\text{ref}} - \theta) + \dot{\theta}_{\text{ref}}$$

The feedforward term $\dot{\theta}_{\text{ref}}$ provides the velocity the motor needs to track the trajectory. The feedback term $K_{p,p} (\theta_{\text{ref}} - \theta)$ only has to cancel the tracking error, not supply the entire control effort. This is standard in industrial motion control — velocity feedforward + acceleration feedforward (injected at the current level) can reduce tracking errors by orders of magnitude.

---

## 6. The bandwidth separation rule — derived, not recited

The cascade works because the loops operate at different timescales. The universal rule:

$$\boxed{\text{inner loop bandwidth} \geq 5\times \text{next outer loop bandwidth}}$$

This is not an empirical guideline. It falls out of a clean derivation using **pole-zero cancellation** — the standard industrial tuning method for cascaded loops — which reduces the cascaded system to exactly second order, where the damping ratio $\zeta$ is well-defined.

### 6.1 Tune the inner loop with pole-zero cancellation

Start with an inner loop whose plant is first-order — exactly what PI current or velocity control sees after the next-inner loop has been closed:

$$G_{\text{in}}(s) = \frac{K}{\tau s + 1}$$

Apply a PI controller written in the form $C_{\text{in}}(s) = K_p \cdot \dfrac{\tau s + 1}{\tau s}$:

$$C_{\text{in}}(s) = \frac{K_p (\tau s + 1)}{\tau s}$$

This is pole-zero cancellation: the controller zero at $s = -1/\tau$ cancels the plant pole at the same location. (In practice the cancellation is never perfect — parameter uncertainty leaves a small residual — but it is close enough that the resulting dynamics are dominated by the intended design.)

The open-loop transfer function of the inner loop becomes:

$$L_{\text{in}}(s) = C_{\text{in}}(s) \cdot G_{\text{in}}(s) = \frac{K_p (\tau s + 1)}{\tau s} \cdot \frac{K}{\tau s + 1} = \frac{K_p K}{\tau s}$$

The $(\tau s + 1)$ terms cancel exactly. The inner open loop is a pure integrator. The closed-loop inner transfer function is therefore first-order:

$$T_{\text{in}}(s) = \frac{L_{\text{in}}(s)}{1 + L_{\text{in}}(s)} = \frac{K_p K / \tau s}{1 + K_p K / \tau s} = \frac{1}{\tau_{\text{in}} s + 1}$$

where the inner closed-loop time constant is:

$$\tau_{\text{in}} = \frac{\tau}{K_p K}$$

The inner loop's closed-loop bandwidth is $\omega_{\text{in}} = 1 / \tau_{\text{in}} = K_p K / \tau$. By choosing $K_p$, you set the bandwidth directly — no iteration, no root-locus sketching, one design parameter produces one pole location.

### 6.2 Close the outer loop — the cascaded system becomes second order

The outer loop sees the inner closed-loop as its plant, followed by a pure integrator (position from velocity, or velocity from current):

$$G_{\text{out}}(s) = T_{\text{in}}(s) \cdot \frac{1}{s} = \frac{1}{s(\tau_{\text{in}} s + 1)}$$

Apply a simple proportional controller $C_{\text{out}}(s) = K_{p,\text{out}}$. The closed-loop transfer function of the outer loop is:

$$T_{\text{out}}(s) = \frac{K_{p,\text{out}}}{s(\tau_{\text{in}} s + 1) + K_{p,\text{out}}} = \frac{K_{p,\text{out}} / \tau_{\text{in}}}{s^2 + s / \tau_{\text{in}} + K_{p,\text{out}} / \tau_{\text{in}}}$$

This is a standard second-order system. Write it in the canonical form $T(s) = \omega_n^2 / (s^2 + 2\zeta \omega_n s + \omega_n^2)$:

$$\omega_n^2 = \frac{K_{p,\text{out}}}{\tau_{\text{in}}}, \qquad 2\zeta \omega_n = \frac{1}{\tau_{\text{in}}}$$

Eliminate $\omega_n$ to find the relationship between the outer loop's damping ratio $\zeta$ and the key design parameters:

$$\zeta = \frac{1}{2\sqrt{K_{p,\text{out}} \, \tau_{\text{in}}}}$$

Now define the **separation ratio** $N$ — the ratio of inner closed-loop bandwidth to outer natural frequency:

$$N \equiv \frac{\omega_{\text{in}}}{\omega_n} = \frac{1/\tau_{\text{in}}}{\sqrt{K_{p,\text{out}} / \tau_{\text{in}}}} = \frac{1}{\sqrt{K_{p,\text{out}} \, \tau_{\text{in}}}}$$

Substituting into the expression for $\zeta$:

$$\boxed{\zeta = \frac{N}{2}}$$

This is the result. The damping ratio of the cascaded outer loop is exactly half the separation ratio. No approximations. No cubic characteristic equation. No Routh–Hurwitz. Pole-zero cancellation makes the cascade a second-order system, and second-order systems have a damping ratio.

### 6.3 What damping ratio requires what separation

$$\zeta = \frac{N}{2} \quad\Longrightarrow\quad N = 2\zeta$$

| Desired $\zeta$ | Required $N = \omega_{\text{in}} / \omega_n$ | Step response character |
|-----------------|---------------------------------------------|------------------------|
| 0.5 | 1.0 | Oscillatory — 16% overshoot, visible ringing |
| 0.7 | 1.4 | Clean — 5% overshoot, fast settling |
| 1.0 | 2.0 | Critically damped — no overshoot, slower rise |

The minimum ratio to avoid visible oscillation is $N \approx 1.4$ ($\zeta = 0.7$). Below that, the outer loop overshoots noticeably.

**Why then does industrial practice say 5×, not 1.4×?** Three reasons, each adding a factor:

1. **Cancellation is never perfect.** The PI controller zero at $-1/\tau$ only approximates the plant pole. Motor resistance drifts with temperature. Inductance varies with saturation. The residual uncanceled dynamics add phase lag that erodes the effective damping. A factor of ~2× margin recovers the intended $\zeta$.

2. **The inner loop is not truly first-order.** Even with pole-zero cancellation, unmodeled dynamics — PWM delay in the current loop, encoder quantization in the velocity loop, anti-aliasing filters — push the actual order higher. Each extra lag adds phase loss at the outer loop's crossover. A factor of ~1.5× accounts for these.

3. **Bandwidth is not $\omega_n$.** In practice, the "bandwidth" quoted on datasheets is the $-3$ dB closed-loop bandwidth $\omega_B$, which for second-order systems is approximately $\omega_n$ (exactly $\omega_n$ at $\zeta = 0.7$). But the separation ratio in the field is computed using *open-loop crossover frequencies*, which are lower than $\omega_n$ for damped systems. Switching back to the crossover-based definition inflates $N$ by roughly $1.5\times$.

Multiply these together: $1.4 \times 2 \times 1.5 \times 1.5 \approx 6.3$. The rounded, conservative value is 5. The $N = 2\zeta$ derivation gives the **theoretical floor**. The factor of 5 is the **practical recommendation** that works on hardware.

### 6.4 The phase-loss check (quick verification)

The same $N \geq 5$ can be verified quickly in the frequency domain. With the inner closed-loop $T_{\text{in}}(s) = 1/(\tau_{\text{in}} s + 1)$, the outer open-loop is $L_{\text{out}}(s) = K_{p,\text{out}} / (s(\tau_{\text{in}} s + 1))$. The outer crossover $\omega_{c,\text{out}} \approx K_{p,\text{out}}$ (valid when the inner lag is fast enough that its phase contribution is small). Using the pole-zero cancellation relationships:

$$K_{p,\text{out}} = \omega_n^2 \tau_{\text{in}} = \frac{\omega_{\text{in}}^2}{N^2} \cdot \frac{1}{\omega_{\text{in}}} = \frac{\omega_{\text{in}}}{N^2}$$

So at the outer crossover, the inner loop's phase contribution is:

$$\Delta\phi = -\arctan\!\left(\frac{\omega_{c,\text{out}}}{\omega_{\text{in}}}\right) = -\arctan(1/N^2)$$

| $N$ | $N^2$ | Phase loss $\arctan(1/N^2)$ | Impact |
|-----|-------|---------------------------|--------|
| 1.4 | 2.0 | $-26.6^\circ$ | Too much — outer loop sluggish |
| 2.0 | 4.0 | $-14.0^\circ$ | Borderline |
| 3.0 | 9.0 | $-6.3^\circ$ | Acceptable |
| **5.0** | **25.0** | **$-2.3^\circ$** | **Negligible — inner loop is transparent** |

At $N = 5$, the inner loop costs only $2.3^\circ$ at the outer crossover — the cascade is effectively decoupled. At $N = 2$, the $14^\circ$ loss is significant enough that you must redesign the outer loop to compensate.

### 6.5 The practical numbers

The clean pole-zero cancellation derivation gives $N = 2\zeta$ as the theoretical relationship between separation ratio and outer-loop damping. For $\zeta = 0.7$, the theoretical minimum is $N = 1.4$. With practical safety factors for imperfect cancellation, unmodeled dynamics, and the conservative definition of bandwidth used in industry, the recommended value rounds to $N = 5$. In practice:

| Loop | Bandwidth at $N=5$ | Bandwidth at $N=10$ |
|------|---------------------|----------------------|
| Current (innermost) | 500 Hz | 2000 Hz |
| Velocity | 100 Hz | 200 Hz |
| Position | 20 Hz | 20 Hz |

At $N=5$, each inner loop is fast enough to be a clean first-order approximation for the next outer loop. At $N=10$, you have headroom for robustness against parameter drift. Below $N=3$, the approximations break, the simple "design each loop independently" procedure stops working, and you must design the loops jointly — which is exactly what LQR does with its full state-feedback gain matrix.
## 7. Tuning the cascade: inside-out

Tuning a cascade is procedural — no iteration across loops:

**Step 1: Tune the current loop.** Disconnect the outer loops. Apply a current step and tune the PI gains for fastest response without overshoot (no current ringing allowed — it overheats the motor). Bandwidth target: as fast as the PWM frequency allows.

**Step 2: Close and approximate.** With the current loop tuned, measure or compute its closed-loop bandwidth $\omega_{c,i}$. Approximate it as a first-order lag with time constant $\tau_i = 1 / \omega_{c,i}$.

**Step 3: Tune the velocity loop.** Using the approximate plant $G_v(s) = (1/(\tau_i s + 1)) \cdot (K_t/(Js + B))$, design the velocity PI gains for the desired damping and bandwidth. The inner loop's lag $\tau_i$ is treated as part of the plant — small enough not to matter much, but included for accuracy.

**Step 4: Close and approximate.** With velocity loop tuned, its closed-loop bandwidth $\omega_{c,v}$ gives $\tau_v = 1 / \omega_{c,v}$ for the position designer.

**Step 5: Tune the position loop.** Using $G_p(s) = 1/(s(\tau_v s + 1))$, set $K_{p,p}$ for the desired position tracking bandwidth. Optionally add velocity and acceleration feedforward.

**Step 6: Verify end-to-end.** Simulate the full three-loop cascade. Check that the inner loop approximations held up. If the current loop's phase lag reduced the velocity loop's phase margin, back off the velocity bandwidth slightly. This verification step is usually a formality — the 5× separation rule makes the approximations reliable.

Notice what *didn't* happen: no trial-and-error across loops, no re-tuning the current loop because the position loop oscillates, no guessing. The cascade decomposes a single coupled tuning problem into three independent ones.

---

## 8. Anti-windup in cascaded loops

### 8.1 Each loop saturates differently

In a cascaded drive, saturation happens at multiple levels:

- **Current loop:** the voltage command $V_{\text{cmd}}$ hits $\pm V_{\text{bus}}$. The current PI integrator winds up.
- **Velocity loop:** the current command $i_{\text{ref}}$ hits $\pm i_{\max}$. The velocity PI integrator winds up.
- **Position loop:** (if it has a PI) the velocity command $\omega_{\text{ref}}$ hits $\pm \omega_{\max}$.

The structural beauty of cascading: **each loop only needs to know about its own saturation.** The current loop has a back-calculation anti-windup that tracks the difference $V_{\text{cmd}} - V_{\text{actual}}$. The velocity loop has a back-calculation that tracks $i_{\text{ref}} - i_{\text{limited}}$. Neither needs to know the other's state.

Compare this to a monolithic controller — LQR or a single PID — where windup in one state bleeds into all others. The cascade naturally isolates saturation effects.

### 8.2 The standard anti-windup pattern at each level

For a PI controller at any level, back-calculation works the same way:

$$u_{\text{unlim}} = K_p e + I$$
$$u_{\text{lim}} = \text{clamp}(u_{\text{unlim}}, u_{\min}, u_{\max})$$
$$I_{k+1} = I_k + \left(K_i e - \frac{u_{\text{unlim}} - u_{\text{lim}}}{T_t}\right) \Delta t$$

The tracking time constant $T_t$ determines how fast the integrator backs off. A reasonable choice is $T_t \approx \sqrt{T_i T_d}$ for PID, or simply $T_t = 1 / K_i$ for PI — the integrator time constant itself.

In the cascade, each loop runs this identical pattern with its own $u_{\min}$, $u_{\max}$. The position loop limits velocity commands. The velocity loop limits current commands. The current loop limits voltage commands. No cross-coupling, no shared integrator states, no cascading failure modes.

### 8.3 The deeper insight: saturation is "model mismatch" at each stage

When the velocity loop commands $i_{\text{ref}} = 5\,\text{A}$ but the current loop can only deliver $3\,\text{A}$ (because the back-EMF at high speed has eaten the voltage headroom), the velocity controller's integrator believes $5\,\text{A}$ of torque was applied. The plant model in the velocity controller's internal state diverges from reality. Anti-windup corrects this divergence by making the integrator consistent with what the plant *actually received*.

This is why MPC doesn't need anti-windup: the plant model is explicit in the optimization, and the constraint $|u| \leq u_{\max}$ is part of the solver's search. There is no internal state mismatch to correct. The cascade with back-calculation approximates this — each loop corrects its internal state to match the constrained reality, independently.

---

## 9. Cascade vs LQR: structured vs optimal

### 9.1 LQR on the full motor

Consider the same DC motor with full-state feedback via LQR. The control law is:

$$u = -Kx = -[k_\theta \ k_\omega \ k_i] \begin{bmatrix} \theta \\ \omega \\ i \end{bmatrix}$$

This is a single unified gain vector $K$ computed by solving the CARE. The LQR controller sees all three states simultaneously and computes one optimal voltage. It makes no distinction between "electrical" and "mechanical" dynamics — everything is coupled through the $K$ matrix.

### 9.2 The cascade is a structured LQR

Now look at the cascade from a state-space perspective:

```
Position error → K_p,p → ω_ref → Velocity PI → i_ref → Current PI → V_cmd
```

Expanding the cascade: the position P controller produces $\omega_{\text{ref}} = K_{p,p} (\theta_{\text{ref}} - \theta)$. The velocity PI produces $i_{\text{ref}} = K_{p,v} (\omega_{\text{ref}} - \omega) + K_{i,v} \int (\omega_{\text{ref}} - \omega)$. The current PI produces the voltage.

Written as one big state-feedback law, the cascade *is* an LQR controller — but with a highly structured gain matrix. Instead of a full $1 \times 3$ matrix where every gain can be anything, the cascade constrains the structure:

$$K_{\text{cascade}} \text{ has zeros in specific off-diagonal positions}$$

The structure reflects the physical directionality: voltage → current → velocity → position. The cascade pre-commits to this directionality. LQR is free to use *any* linear combination of all states.

### 9.3 When the cascade matches LQR

If you solve the LQR problem with a diagonal $Q$ (no cross-weighting between states) and the plant is exactly a chain of integrators, the optimal $K$ will naturally have the cascade structure — LQR "discovers" cascading. The electrical state only enters through the current gain $k_i$, the mechanical state only through $k_\omega$, and the position state only through $k_\theta$. The optimal control respects the physical topology.

When the cascade won't match LQR:

- **Strong cross-coupling** ($K_e$, back-EMF) means current state affects velocity in the voltage equation — LQR can use $k_i$ to partially cancel back-EMF, which the cascade can't (the current loop only sees $i_{\text{ref}}$, not $i$ directly).
- **Non-chain topology** — if the plant isn't a cascade of first-order lags, the structured controller can't capture the optimal coupling.
- **Non-diagonal Q weighting** — if you care about combinations like $\theta + 2\omega$, LQR uses coupling that the cascade structure prohibits.

### 9.4 Why cascade persists despite LQR's optimality

If LQR is strictly better (it solves the optimal control problem exactly), why do industrial drives use cascaded PI loops instead of LQR?

1. **Tuning.** Three independent PI loops with intuitive bandwidth targets → an engineer can tune a cascade on the factory floor without knowing what a Riccati equation is. LQR requires model matrices, cost weight selection, and a solver.

2. **Saturation handling.** Each loop's anti-windup is simple, independent, and proven. LQR saturation handling requires a separate anti-windup framework or MPC.

3. **Diagnostics.** If the position is oscillating, the technician checks the velocity loop. If the motor sounds rough, check the current loop. The cascade provides a direct physical interpretation for every symptom.

4. **Implementation.** Three PI loops = six gains, zero matrix operations. Runs on a two-dollar microcontroller.

5. **Guaranteed stability at each stage.** If each inner loop is stable and 5× faster, the whole cascade is stable. The proof is structural, not numerical.

The cascade is an example of something deep: **optimality is not the only design goal.** Interpretability, tunability, and robustness to individual loop failures all matter. A suboptimal controller that a technician can tune is better than an optimal one that requires a PhD to adjust.

---

## 10. Connection to this project

| Doc | Connection |
|-----|-----------|
| `servo_motor_pid.html` | The PID simulator treats the motor as a single controller. This deep-dive shows how it *could* be decomposed into three nested controllers — current → velocity → position — each handling its own timescale. The simulator's anti-windup on voltage saturation is the current-loop anti-windup; velocity and position anti-windup would be the natural next steps. |
| `lqr_explorer.html` | LQR on the full motor produces a single optimal gain vector $K = [k_\theta, k_\omega, k_i]$. The cascade is a *structured approximation* to this — it imposes zeros on certain off-diagonal gains to respect the physical directionality. LQR can use current feedback ($k_i$) to partially cancel back-EMF; the cascade can't. |
| `servo_qp_mpc.html` | MPC handles constraints in the optimization. The cascade handles them independently at each level with anti-windup. MPC is the theoretically clean solution; cascading with back-calculation is the practical one that runs on a two-dollar microcontroller. |
| `anti_windup.md` | Back-calculation anti-windup appears at every level of the cascade — current, velocity, and position — each with its own tracking time constant. The cascade naturally isolates saturation effects so a saturated inner loop doesn't corrupt outer loop integrators. |
| `core_problems_controller_design.md` | Problem #2 (inertia/delay) is *why* we cascade — instead of one controller fighting three orders of dynamics, three controllers each fight one. Problem #3 (constraints) is managed by anti-windup at each level. Problem #8 (computation) explains why three PIs beat one MPC on real hardware. |
| `bellman_to_lqr.md` | The cascade is a value-function decomposition: the current loop's value depends only on current error, the velocity loop's on velocity error, the position loop's on position error. LQR's value function couples all states. The cascade approximates optimality by assuming the value function is separable. |
| `lead_lag_compensator_design.md` | The cascade's "approximate inner loop as first-order lag" is exactly the loop-shaping philosophy applied hierarchically: design the inner Bode plot, simplify, move outward. |
| `digital_control_sampling.md` | The bandwidth separation rule (5–10×) determines multi-rate sampling: current loop at 100 kHz, velocity at 10 kHz, position at 1 kHz. Each loop's sample rate is set by its bandwidth requirement, not by the fastest loop. |

---

## 11. Further reading

**The standard industrial reference:**
- Ellis, G. (2012). *Control System Design Guide*, 4th ed. Butterworth-Heinemann. — Chapters 4–7 on the current loop, velocity loop, and position loop, with real tuning procedures and servo drive examples. Written by an engineer who designed Kollmorgen servo drives. This is *the* book.

**Academic treatment of nested loops:**
- Åström, K.J. & Hägglund, T. (2006). *Advanced PID Control*. ISA. — Chapter 8 on cascade control: when it helps, when it doesn't, tuning rules, anti-windup in the cascade structure.

**State-space perspective on cascading:**
- Franklin, G.F., Powell, J.D., & Emami-Naeini, A. (2019). *Feedback Control of Dynamic Systems*, 8th ed. Pearson. — Chapter 7 on state-space design. Shows the equivalence between cascaded PI and full-state feedback with structured gains.

**The original cascade control paper:**
- Franks, R.G. & Worley, C.W. (1956). "Quantitative Analysis of Cascade Control." *Industrial & Engineering Chemistry*, 48(6), 1074–1079. — The paper that introduced cascading to process control. Motor cascading came later but the principle is identical.

**Motor drive design (power electronics side):**
- Krishnan, R. (2001). *Electric Motor Drives: Modeling, Analysis, and Control*. Prentice Hall. — Chapters 6–7 on current control design (hysteresis, ramp-comparison, space-vector PWM) and the velocity/position cascade for DC, PMSM, and induction motors.

**Advanced: MPC as cascade replacement:**
- Bolognani, S., Bolognani, S., Peretti, L., & Zigliotto, M. (2009). "Design and Implementation of Model Predictive Control for Electrical Motor Drives." *IEEE Trans. Industrial Electronics*, 56(6), 1925–1936. — Shows when MPC can replace the entire cascade, and why the cascade remains more popular in practice.
