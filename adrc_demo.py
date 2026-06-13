#!/usr/bin/env python3
"""
ADRC (Active Disturbance Rejection Control) Demo — 2nd-order plant.

Compares ADRC against PID on the same plant with configurable disturbance
and parameter shift.  Fixed-step RK4 for the full 5-state system
(plant 2 + ESO 3).

Usage:   .venv/bin/pip install numpy matplotlib
         .venv/bin/python3 adrc_demo.py
"""

import numpy as np
from matplotlib import pyplot as plt

# ═══════════════════════════════════════════════════════════════════
# 1. Plant — 2nd order
# ═══════════════════════════════════════════════════════════════════

wn, zeta = 2.0, 0.7
b0_true = wn * wn          # true input gain = 4


def disturbance(t, dist_type='ramp', amp=30.0):
    if dist_type == 'step':   return amp if t >= 1.0 else 0.0
    if dist_type == 'sine':   return amp * np.sin(5 * t) if t >= 0.5 else 0.0
    if dist_type == 'ramp':
        if t < 1.0:  return 0.0
        if t < 3.0:  return amp * (t - 1.0) / 2.0
        return amp
    if dist_type == 'pulse':  return amp if (t % 2.0) < 0.4 else 0.0
    return 0.0


def plant_params(t, param_shift=0.0, shift_time=1.5):
    if param_shift > 0 and t >= shift_time:
        f = 1.0 + param_shift
        return wn * np.sqrt(f), zeta / np.sqrt(f)
    return wn, zeta


# ═══════════════════════════════════════════════════════════════════
# 2. ADRC — Correct ESO:  ż1 = z2 + l1·e
#                         ż2 = z3 + b0·u + l2·e
#                         ż3 = l3·e
# ═══════════════════════════════════════════════════════════════════

def simulate_adrc(t_span, dt, wc, wo, b0_adrc,
                  dist_type='ramp', dist_amp=30.0,
                  param_shift=0.0, shift_time=1.5):
    n_steps = int((t_span[1] - t_span[0]) / dt) + 1
    t = np.linspace(t_span[0], t_span[1], n_steps)

    y_hist = np.zeros(n_steps)
    u_hist = np.zeros(n_steps)
    fh_hist = np.zeros(n_steps)
    ft_hist = np.zeros(n_steps)

    state = np.zeros(5)  # [y_plant, ẏ_plant, z1=ŷ, z2=ẏ̂, z3=f̂]
    lo = np.array([3 * wo, 3 * wo * wo, wo * wo * wo])
    kc = np.array([wc * wc, 2 * wc])
    ref = 1.0

    for i in range(n_steps):
        tv = t[i]
        wn_cur, zeta_cur = plant_params(tv, param_shift, shift_time)
        a0, a1 = wn_cur * wn_cur, 2 * zeta_cur * wn_cur

        z1, z2, z3 = state[2], state[3], state[4]
        u_raw = (kc[0] * (ref - z1) - kc[1] * z2 - z3) / b0_adrc
        u = np.clip(u_raw, -30.0, 30.0)
        u_hist[i] = u
        y_hist[i] = state[0]

        d_val = disturbance(tv, dist_type, dist_amp)
        ft_hist[i] = -(a1 * state[1] + a0 * state[0]) + d_val
        fh_hist[i] = z3

        if i == n_steps - 1:
            break

        # RK4 over one dt step — plant + ESO integrated together
        sub, dtF = 5, dt / 5
        for _ in range(sub):
            yp, vp, z1_, z2_, z3_ = state
            e = yp - z1_
            dy1 = vp
            dy2 = -a1 * vp - a0 * yp + b0_true * u + d_val
            dz1 = z2_ + lo[0] * e
            dz2 = z3_ + b0_adrc * u + lo[1] * e
            dz3 = lo[2] * e
            k1 = np.array([dy1, dy2, dz1, dz2, dz3])

            s2 = state + 0.5 * dtF * k1
            yp2, vp2, z1_2, z2_2, z3_2 = s2
            e2 = yp2 - z1_2
            dz1_2 = z2_2 + lo[0] * e2
            dz2_2 = z3_2 + b0_adrc * u + lo[1] * e2
            k2 = np.array([vp2, -a1 * vp2 - a0 * yp2 + b0_true * u + d_val, dz1_2, dz2_2, lo[2] * e2])

            s3 = state + 0.5 * dtF * k2
            yp3, vp3, z1_3, z2_3, z3_3 = s3
            e3 = yp3 - z1_3
            dz1_3 = z2_3 + lo[0] * e3
            dz2_3 = z3_3 + b0_adrc * u + lo[1] * e3
            k3 = np.array([vp3, -a1 * vp3 - a0 * yp3 + b0_true * u + d_val, dz1_3, dz2_3, lo[2] * e3])

            s4 = state + dtF * k3
            yp4, vp4, z1_4, z2_4, z3_4 = s4
            e4 = yp4 - z1_4
            dz1_4 = z2_4 + lo[0] * e4
            dz2_4 = z3_4 + b0_adrc * u + lo[1] * e4
            k4 = np.array([vp4, -a1 * vp4 - a0 * yp4 + b0_true * u + d_val, dz1_4, dz2_4, lo[2] * e4])

            state += (dtF / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    return t, y_hist, u_hist, fh_hist, ft_hist


# ═══════════════════════════════════════════════════════════════════
# 3. PID — with anti-windup + derivative LPF
# ═══════════════════════════════════════════════════════════════════

def simulate_pid(t_span, dt, Kp, Kd, Ki,
                 dist_type='ramp', dist_amp=30.0,
                 param_shift=0.0, shift_time=1.5):
    n_steps = int((t_span[1] - t_span[0]) / dt) + 1
    t = np.linspace(t_span[0], t_span[1], n_steps)

    y_hist = np.zeros(n_steps)
    u_hist = np.zeros(n_steps)
    y_plant = np.zeros(2)
    integ, deriv, tau_f, Tt = 0.0, 0.0, 0.01, np.sqrt(1.0 / max(Ki, 1e-6))
    ref = 1.0

    for i in range(n_steps):
        tv = t[i]
        wn_cur, zeta_cur = plant_params(tv, param_shift, shift_time)
        a0, a1 = wn_cur * wn_cur, 2 * zeta_cur * wn_cur

        e = ref - y_plant[0]
        u_raw = Kp * e + deriv + integ                 # -Kd·ẏ via deriv filter
        u = np.clip(u_raw, -30.0, 30.0)
        u_hist[i] = u
        y_hist[i] = y_plant[0]

        if i == n_steps - 1:  break

        # RK4 plant sub-steps
        sub, dtF, d_val = 5, dt / 5, disturbance(tv, dist_type, dist_amp)
        for _ in range(sub):
            yp, vp = y_plant
            k1y = vp
            k1v = -a1 * vp - a0 * yp + b0_true * u + d_val
            k2y = vp + 0.5 * dtF * k1v
            k2v = -a1 * (vp + 0.5 * dtF * k1v) - a0 * (yp + 0.5 * dtF * k1y) + b0_true * u + d_val
            k3y = vp + 0.5 * dtF * k2v
            k3v = -a1 * (vp + 0.5 * dtF * k2v) - a0 * (yp + 0.5 * dtF * k2y) + b0_true * u + d_val
            k4y = vp + dtF * k3v
            k4v = -a1 * (vp + dtF * k3v) - a0 * (yp + dtF * k3y) + b0_true * u + d_val
            y_plant += (dtF / 6.0) * np.array([k1y + 2*k2y + 2*k3y + k4y,
                                                k1v + 2*k2v + 2*k3v + k4v])

        d_raw = -Kd * y_plant[1]
        integ += (Ki * e - (u_raw - u) / Tt) * dt
        deriv += ((Kd * d_raw - deriv) / tau_f) * dt

    return t, y_hist, u_hist


# ═══════════════════════════════════════════════════════════════════
# 4. Run and plot
# ═══════════════════════════════════════════════════════════════════

t_span = (0.0, 8.0)
dt = 0.0005

wc, wo, b0_adrc = 4.0, 20.0, b0_true
Kp, Kd, Ki = 6.0, 1.0, 8.0
dist_type, dist_amp = 'ramp', 8.0
param_shift, shift_time = 0.8, 3.0

print("ADRC:  ωc=%.0f  ωo=%.0f  b0=%.0f" % (wc, wo, b0_adrc))
print("PID:   Kp=%.1f  Kd=%.1f  Ki=%.0f" % (Kp, Kd, Ki))
print("Plant: ωn=%.0f  ζ=%.2f  true-b0=%.0f" % (wn, zeta, b0_true))
print("Dist:  %s amp=%.0f  param-shift=%.0f%% at t=%.1fs" %
      (dist_type, dist_amp, param_shift * 100, shift_time))
print("Simulating...")

t_a, y_a, u_a, f_h, f_t = simulate_adrc(
    t_span, dt, wc, wo, b0_adrc, dist_type, dist_amp, param_shift, shift_time)
t_p, y_p, u_p = simulate_pid(
    t_span, dt, Kp, Kd, Ki, dist_type, dist_amp, param_shift, shift_time)

ise_a = np.sum((1 - y_a) ** 2) * dt
ise_p = np.sum((1 - y_p) ** 2) * dt
print("ADRC ISE: %.3f  max|u|: %.1f" % (ise_a, np.max(np.abs(u_a))))
print("PID  ISE: %.3f  max|u|: %.1f" % (ise_p, np.max(np.abs(u_p))))

# ── Plot ──
fig, (ax_y, ax_f, ax_u) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

ax_y.plot(t_a, y_a, '#06d6a0', lw=2, label='ADRC')
ax_y.plot(t_p, y_p, '#f72585', lw=2, label='PID')
ax_y.axhline(1, color='gray', ls='--', lw=0.8)
if param_shift > 0:
    ax_y.axvline(shift_time, color='orange', ls=':', lw=1, alpha=0.7, label='param shift')
ax_y.set_ylabel('y(t)')
ax_y.legend(loc='lower right');  ax_y.grid(True, alpha=0.15)
ax_y.set_title('ADRC vs PID — step response + %.0f%% param shift at t=%.0fs + %s disturbance' %
               (param_shift * 100, shift_time, dist_type), fontweight='bold')

ax_f.plot(t_a, f_t, '#8b949e', lw=1.5, label='true f(t)')
ax_f.plot(t_a, f_h, '#ffd166', lw=2, label='ESO estimate f̂(t)')
ax_f.set_ylabel('f(t)');  ax_f.legend(loc='upper left');  ax_f.grid(True, alpha=0.15)

ax_u.plot(t_a, u_a, '#06d6a0', lw=1.5, label='ADRC')
ax_u.plot(t_p, u_p, '#f72585', lw=1.5, label='PID')
ax_u.axhline(30, color='red', ls='--', lw=0.6, alpha=0.4)
ax_u.axhline(-30, color='red', ls='--', lw=0.6, alpha=0.4)
ax_u.set_ylabel('u(t)');  ax_u.set_xlabel('time [s]')
ax_u.legend(loc='upper right');  ax_u.grid(True, alpha=0.15)

fig.tight_layout(pad=1.0)
fig.savefig('adrc_demo.png', dpi=150, facecolor='white')
print("Plot saved → adrc_demo.png")
plt.show()
