#!/usr/bin/env python3
"""
Lead / Lag Compensator Design Demo.

A compensator reshapes the open-loop frequency response to meet closed-loop
specs — more phase margin (lead), better steady-state accuracy (lag), or
both (lead-lag).  This demo works through all three on the same plant and
also discretises each compensator at 1 kHz for digital implementation.

           ┌──────────┐     ┌──────────┐
    r ──→  │ C(s)     │──→  │ G(s)     │──→ y
           │compensator│     │  plant   │
           └──────────┘     └──────────┘

Usage:   .venv/bin/pip install control matplotlib numpy scipy
         .venv/bin/python3 lead_lag_compensator_demo.py
"""

import numpy as np
from matplotlib import pyplot as plt
import control as ct

# ═══════════════════════════════════════════════════════════════════
# 1. The plant — three cascaded lags (type-0, finite DC gain, lots of phase lag)
# ═══════════════════════════════════════════════════════════════════
#
# Three first-order lags in series: a fast actuator, a mechanical time
# constant, and a slow thermal/filter pole.  Type-0 means finite DC gain
# → steady-state error exists → lag compensation is meaningful.
# The three poles stack phase lag → phase margin is poor → lead helps.

tau1 = 0.02    # fast actuator / electrical time constant
tau2 = 0.2     # mechanical time constant
tau3 = 0.8     # slow filter / sensor lag
K_plant = 5.0  # DC gain

num_G = [K_plant]
# (τ1 s + 1)(τ2 s + 1)(τ3 s + 1) in coefficient form
den_G = [tau1*tau2*tau3,
         tau1*tau2 + tau1*tau3 + tau2*tau3,
         tau1 + tau2 + tau3,
         1]
G = ct.tf(num_G, den_G)

print("Plant G(s):")
print(f"  poles  = {np.round(ct.poles(G), 3)}")
print(f"  DCgain = {float(ct.dcgain(G)):.3f}")

# ═══════════════════════════════════════════════════════════════════
# 2. Analyse the bare plant
# ═══════════════════════════════════════════════════════════════════

gm, pm, wcg, wcp = ct.margin(G)
print(f"\nBare plant margins:")
print(f"  GM  = {gm:.2f} ({20*np.log10(gm):.1f} dB)  at ω = {wcg:.2f} rad/s")
print(f"  PM  = {pm:.1f}°  at ω = {wcp:.2f} rad/s")
print(f"  bandwidth (0 dB cross) ≈ {wcp:.2f} rad/s")
print(f"  DC gain = {float(ct.dcgain(G)):.3f}  → steady-state error = "
      f"{1/(1 + float(ct.dcgain(G))):.1%} for unit feedback")

# ═══════════════════════════════════════════════════════════════════
# 3. Lead compensator — add phase near the crossover
# ═══════════════════════════════════════════════════════════════════
#
# Lead:  C(s) = Kc · (τ s + 1) / (α τ s + 1),   0 < α < 1
#
# Zero at −1/τ, pole at −1/(ατ). α<1 ⇒ pole further from origin ⇒ phase lead.
# Phase boost peaks at ω_max = 1/(τ√α), height = arcsin((1−α)/(1+α)).
# We want ≈ 50° extra phase at ω_max ≈ 1.5× bare crossover.


def design_lead(plant, desired_pm_extra=50, crossover_mult=1.5):
    """Return C_lead(s) and a diagnostics dict."""
    _, pm0, _, wcp0 = ct.margin(plant)
    phi = np.deg2rad(desired_pm_extra)         # phase boost we want
    alpha = (1 - np.sin(phi)) / (1 + np.sin(phi))  # 0 < α < 1
    w_max = crossover_mult * wcp0               # centre the bump above current BW
    tau = 1 / (w_max * np.sqrt(alpha))

    num = tau, 1                               # τs + 1
    den = alpha * tau, 1                       # ατs + 1
    C = ct.tf([tau, 1], [alpha * tau, 1])

    # Scale so that |C(j wcp0)| ≈ 1 — keep the old crossover gain unchanged
    mag_at_wcp0 = np.abs(ct.evalfr(C, 1j * wcp0))
    Kc = 1 / mag_at_wcp0
    C *= Kc

    info = dict(alpha=alpha, tau=tau, w_max=w_max, Kc=Kc,
                phi_max=np.rad2deg(phi))
    return C, info


C_lead, info_lead = design_lead(G)
print(f"\nLead compensator: α={info_lead['alpha']:.2f}, "
      f"τ={info_lead['tau']:.4f}, φ_max={info_lead['phi_max']:.1f}° "
      f"at ω_max={info_lead['w_max']:.2f}")
print(f"  C_lead(s) = {C_lead}")

# ═══════════════════════════════════════════════════════════════════
# 4. Lag compensator — boost low-frequency gain without hurting PM
# ═══════════════════════════════════════════════════════════════════
#
# Lag:  C(s) = Kc · (τ s + 1) / (β τ s + 1),   β > 1
#
# Pole at −1/(βτ) is closer to origin than zero at −1/τ ⇒ phase lag.
# This is a LOW-PASS: DC gain = 1, HF gain = 1/β. Cascaded with an
# overall loop gain K, the DC loop gain becomes K (boosted) while the
# lag rolls off before crossover, preserving PM.


def design_lag(plant, dc_gain_boost=10, decade_below=10):
    """Return C_lag(s) and a diagnostics dict."""
    _, _, _, wcp0 = ct.margin(plant)
    beta = dc_gain_boost
    # Place the zero a decade below crossover so the phase lag has faded
    wz = wcp0 / decade_below
    tau = 1 / wz
    C = ct.tf([tau, 1], [beta * tau, 1])

    # The lag network has |C(jω_cp)| ≈ 1/β at the original crossover.
    # Boost overall gain by β so the compensated crossover stays near ω_cp.
    C *= beta

    return C, dict(beta=beta, tau=tau, wz=wz)


C_lag, info_lag = design_lag(G)
print(f"\nLag compensator:  β={info_lag['beta']:.2f}, "
      f"τ={info_lag['tau']:.4f}, zero at ωz={info_lag['wz']:.3f}")
print(f"  C_lag(s) = {C_lag}")

# ═══════════════════════════════════════════════════════════════════
# 5. Lead-lag — best of both
# ═══════════════════════════════════════════════════════════════════

C_leadlag = C_lead * C_lag

print(f"\nLead-lag compensator:")
print(f"  C_leadlag(s) = C_lead(s) · C_lag(s)")
print(f"  DC boost factor ≈ {info_lag['beta']:.0f}×")
print(f"  phase boost ≈ {info_lead['phi_max']:.0f}° at ω ≈ {info_lead['w_max']:.2f}")

# ═══════════════════════════════════════════════════════════════════
# 6. Digital implementations — Tustin (bilinear) at 1 kHz
# ═══════════════════════════════════════════════════════════════════

fs = 1000.0          # Hz
Ts = 1 / fs

def discretise_tustin(C_ct, Ts):
    """Bilinear (Tustin) discretisation of a continuous transfer function."""
    if isinstance(C_ct, (int, float)):
        return C_ct                     # scalar gain — no dynamics to discretise
    C_dt = ct.sample_system(C_ct, Ts, method='tustin')
    return C_dt

C_lead_d   = discretise_tustin(C_lead, Ts)
C_lag_d    = discretise_tustin(C_lag, Ts)
C_leadlag_d = discretise_tustin(C_leadlag, Ts)

print(f"\nDigital compensators (Tustin, fs={fs:.0f} Hz, Ts={Ts*1e3:.1f} ms):")
print(f"  C_lead_d(z)   degree = {len(C_lead_d.num[0][0]) - 1}")
print(f"  C_lag_d(z)    degree = {len(C_lag_d.num[0][0]) - 1}")
print(f"  C_leadlag_d(z) degree = {len(C_leadlag_d.num[0][0]) - 1}")

# ═══════════════════════════════════════════════════════════════════
# 7. Closed-loop comparisons
# ═══════════════════════════════════════════════════════════════════

systems = {
    'bare plant (unit gain)':    (1, G),
    'lead':                      (C_lead, G),
    'lag':                       (C_lag, G),
    'lead-lag':                  (C_leadlag, G),
}

# Compute margins and step response for each
results = {}
for name, (C, plant) in systems.items():
    L = C * plant
    gm, pm, wcg, wcp = ct.margin(L)
    T = ct.feedback(L, 1)
    t, y = ct.step_response(T, T=np.linspace(0, 3, 3000))
    ss_err = 1 / (1 + np.abs(ct.dcgain(L)))
    results[name] = dict(gm=gm, pm=pm, wcp=wcp, t=t, y=y, ss_err=ss_err, L=L)

# ═══════════════════════════════════════════════════════════════════
# 8. Plot
# ═══════════════════════════════════════════════════════════════════

fig, axes = plt.subplots(2, 2, figsize=(13, 9))

# ——— Bode ———
ax_mag, ax_phase = axes[0, 0], axes[1, 0]
omega = np.logspace(-1, 3, 2000)
colors = ['#8b949e', '#00bcd4', '#f0883e', '#7c4dff']
for (name, res), c in zip(results.items(), colors):
    mag, phase, _ = ct.bode(res['L'], omega, plot=False)
    ax_mag.semilogx(omega, 20*np.log10(mag), color=c, lw=1.5, label=name)
    ax_phase.semilogx(omega, np.rad2deg(phase), color=c, lw=1.5)
    # Mark PM on phase plot
    if res['wcp'] > 0 and res['wcp'] < omega[-1]:
        pm_phase = np.interp(res['wcp'], omega, np.rad2deg(phase))
        ax_phase.plot(res['wcp'], pm_phase, 'o', color=c, ms=4)

ax_mag.axhline(0, color='red', alpha=0.2, ls='--', lw=0.8)
ax_mag.set_ylabel('magnitude [dB]')
ax_mag.legend(fontsize=7.5, loc='lower left')
ax_mag.grid(True, alpha=0.12)
ax_phase.axhline(-180, color='red', alpha=0.2, ls='--', lw=0.8)
ax_phase.set_ylabel('phase [°]')
ax_phase.set_xlabel('frequency [rad/s]')
ax_phase.grid(True, alpha=0.12)
ax_mag.set_title('Open-loop Bode — lead adds PM, lag boosts LF gain',
                 fontweight='bold')

# ——— Step response ———
ax_step = axes[0, 1]
for (name, res), c in zip(results.items(), colors):
    ax_step.plot(res['t'], res['y'], color=c, lw=1.5, label=name)
ax_step.axhline(1, color='gray', alpha=0.3, ls='--', lw=0.8)
ax_step.set_xlabel('time [s]')
ax_step.set_ylabel('y(t)')
ax_step.set_title('Closed-loop step response', fontweight='bold')
ax_step.legend(fontsize=7.5, loc='lower right')
ax_step.grid(True, alpha=0.12)
ax_step.set_ylim(-0.05, 1.4)

# ——— Continuous vs Digital (Tustin) step response ———
ax_dig = axes[1, 1]
t_dig = np.arange(0, 3 + Ts, Ts)
for (name, (C_ct, plant)), c in zip(systems.items(), colors):
    # Continuous closed-loop
    T_ct = ct.feedback(C_ct * plant, 1)
    _, y_ct = ct.step_response(T_ct, T=np.linspace(0, 3, 1000))
    ax_dig.plot(np.linspace(0, 3, 1000), y_ct, color=c, lw=0.8, alpha=0.3)

    # Digital closed-loop — discretise compensator, keep plant continuous
    C_dt = discretise_tustin(C_ct, Ts)
    G_dt = ct.sample_system(plant, Ts, method='zoh')
    if isinstance(C_dt, (int, float)):
        L_dt = C_dt * G_dt
    else:
        L_dt = C_dt * G_dt
    T_dt = ct.feedback(L_dt, 1)
    _, y_dt = ct.step_response(T_dt, T=t_dig)
    ax_dig.step(t_dig, y_dt, color=c, lw=1.5, where='post', label=f'{name} (Tustin)')

ax_dig.axhline(1, color='gray', alpha=0.3, ls='--', lw=0.8)
ax_dig.set_xlabel('time [s]')
ax_dig.set_ylabel('y(t)')
ax_dig.set_title(f'Continuous (faint) vs Digital Tustin (solid, fs={fs:.0f} Hz)',
                 fontweight='bold')
ax_dig.legend(fontsize=7, loc='lower right')
ax_dig.grid(True, alpha=0.12)
ax_dig.set_ylim(-0.05, 1.4)

fig.tight_layout(pad=1.5)
fig.savefig('lead_lag_demo.png', dpi=150, facecolor='white')
print("\nPlot saved → lead_lag_demo.png")

# ═══════════════════════════════════════════════════════════════════
# 9. Summary table
# ═══════════════════════════════════════════════════════════════════

print(f"\n{'':>22} {'PM [°]':>8} {'GM [dB]':>8} {'ω_cp [rad/s]':>14} {'ss err':>8}")
print(f"{'':─>52}")
for name, res in results.items():
    gm_db = 20*np.log10(res['gm']) if res['gm'] > 0 else float('-inf')
    print(f"{name:>22} {res['pm']:8.1f} {gm_db:8.1f} {res['wcp']:14.2f} "
          f"{res['ss_err']:7.1%}")

print(f"\nKey takeaway:")
print(f"  • Lead — adds {info_lead['phi_max']:.0f}° phase near "
      f"{info_lead['w_max']:.1f} rad/s → higher PM, faster response, less overshoot")
print(f"  • Lag — boosts DC gain {info_lag['beta']:.0f}× → "
      f"steady-state error drops by ~{info_lag['beta']:.0f}× without hurting PM")
print(f"  • Lead-lag — combines both: fast AND accurate")
print(f"  • Tustin discretisation at {fs:.0f} Hz faithfully reproduces "
      f"continuous behaviour (check plot)")

plt.show()
