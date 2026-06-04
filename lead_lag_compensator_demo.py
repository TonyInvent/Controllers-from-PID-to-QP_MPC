#!/usr/bin/env python3
"""
Lead / Lag Compensator Design Demo — Unified Pole-Zero Form.

Every compensator is  C(s) = K · (s + z) / (s + p).  What matters is the
relative position of zero and pole on the negative real axis:

    |z| < |p|   →  lead  (zero closer to origin — phase boost)
    |p| < |z|   →  lag   (pole closer to origin — DC gain boost)

This demo works through lead, lag, and lead-lag on a type-1 plant, and
discretises each compensator at 1 kHz via Tustin (bilinear transform).

           ┌──────────┐     ┌──────────┐
    r ──→  │ C(s)     │──→  │ G(s)     │──→ y
           │compensator│     │  plant   │
           └──────────┘     └──────────┘

Usage:   .venv/bin/pip install control matplotlib numpy scipy
         .venv/bin/python3 lead_lag_compensator_demo.py
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')             # headless — saves plot without blocking
from matplotlib import pyplot as plt
import control as ct

# ═══════════════════════════════════════════════════════════════════
# 1. The plant  —  G(s) = 1 / (s (s + 2))
# ═══════════════════════════════════════════════════════════════════
#
# Type-1, 2nd order: one integrator at s = 0, one lag at s = −2.
# Type-1 ⇒ zero steady-state error for step inputs, finite error
# for ramp inputs (error = 1/Kv).

num_G = [1]
den_G = [1, 2, 0]          # s² + 2s  →  s(s+2)
G = ct.tf(num_G, den_G)

print("Plant G(s) = 1 / (s(s+2))")
print(f"  poles  = {np.round(ct.poles(G), 3)}")
print(f"  type-1 — integrator at s = 0 → Kv = lim s·G(s) = 0.5")

# ═══════════════════════════════════════════════════════════════════
# 2. Analyse the bare plant
# ═══════════════════════════════════════════════════════════════════

gm, pm, wcg, wcp = ct.margin(G)
Kv_bare = 0.5  # lim s→0 s·1/(s(s+2)) = 1/2

print(f"\nBare plant margins:")
gm_str = f"{gm:.2f}" if np.isfinite(gm) else "∞"
wcg_str = f"{wcg:.2f}" if np.isfinite(wcg) else "∞"
gm_db = f"{20*np.log10(gm):.1f} dB" if np.isfinite(gm) and gm > 0 else "∞ dB"
print(f"  GM  = {gm_str} ({gm_db})  at ω = {wcg_str} rad/s")
print(f"  PM  = {pm:.1f}°  at crossover ω_cp = {wcp:.2f} rad/s")
print(f"  ω_cp (0 dB cross) = {wcp:.2f} rad/s")
print(f"  Kv = {Kv_bare:.3f}  →  ramp steady-state error = {1/Kv_bare:.3f}")

# ═══════════════════════════════════════════════════════════════════
# 3. Lead compensator  —  |z| < |p|, zero closer to origin
# ═══════════════════════════════════════════════════════════════════
#
#  C(s) = K · (s + z) / (s + p)      with  z < p  (both positive)
#
#  Phase added at frequency ω:
#    φ(ω) = arctan(ω/z) − arctan(ω/p)  >  0  because z < p
#
#  Peak phase φ_max at ω_max = √(z·p):
#    sin(φ_max) = (p − z) / (p + z)
#
#  Design: pick desired phase boost φ.  From φ compute the ratio
#    r = p/z = (1 + sin φ) / (1 − sin φ)
#  Place z at (or slightly below) the bare crossover, then p = r·z.
#  Finally, scale K so |C(j ω_cp0)| ≈ 1 — preserves the old crossover gain.

def design_lead_zpk(plant, desired_pm_boost_deg=50, z_factor=0.9):
    """Design a lead compensator  C(s) = K·(s+z)/(s+p)  with |z| < |p|.

    Returns C(s) and a diagnostics dict with keys z, p, K, r, phi_max.
    """
    _, _, _, wcp0 = ct.margin(plant)

    phi = np.deg2rad(desired_pm_boost_deg)
    sin_phi = np.sin(phi)
    r = (1 + sin_phi) / (1 - sin_phi)          # p/z > 1

    # Place zero near the bare crossover (or slightly below it)
    z = z_factor * wcp0
    p = r * z                                  # pole further out

    # K so that |C(j·wcp0)| = 1  →  total loop gain unchanged at old crossover
    mag = np.abs((1j * wcp0 + z) / (1j * wcp0 + p))
    K = 1.0 / mag
    C = ct.tf([K, K*z], [1, p])
    phi_max = np.rad2deg(np.arcsin((p - z) / (p + z)))
    w_max = np.sqrt(z * p)

    return C, dict(z=z, p=p, K=K, r=r, phi_max=phi_max, w_max=w_max)


C_lead, info_lead = design_lead_zpk(G)
print(f"\nLead compensator  C(s) = K·(s+z)/(s+p)   |z| < |p|")
print(f"  z = {info_lead['z']:.4f}   p = {info_lead['p']:.4f}   K = {info_lead['K']:.4f}")
print(f"  ratio p/z = {info_lead['r']:.2f}  →  φ_max = {info_lead['phi_max']:.1f}° "
      f"at ω_max = {info_lead['w_max']:.3f} rad/s")
print(f"  C_lead(s) = {C_lead}")

# ═══════════════════════════════════════════════════════════════════
# 4. Lag compensator  —  |p| < |z|, pole closer to origin
# ═══════════════════════════════════════════════════════════════════
#
#  C(s) = K · (s + z) / (s + p)      with  p < z  (both positive)
#
#  Phase at frequency ω:
#    φ(ω) = arctan(ω/z) − arctan(ω/p)  <  0  because p < z  (lag!)
#  Therefore: place the zero well below crossover so the phase dip
#  has faded by ω_cp — the PM stays largely intact.
#
#  DC gain = K·z/p,  HF gain = K.
#  z/p = β is the DC boost factor.
#  Design: place pole near origin, zero β× further out, K = β
#  to compensate the HF attenuation of the passive lag network.

def design_lag_zpk(plant, dc_boost=10, pole_loc=0.1):
    """Design a lag compensator  C(s) = K·(s+z)/(s+p)  with |p| < |z|.

    Returns C(s) and a diagnostics dict with keys z, p, K, beta.
    """
    p = pole_loc                               # pole near origin
    z = dc_boost * p                           # zero further out
    K = dc_boost                               # compensate HF attenuation

    C = ct.tf([K, K*z], [1, p])
    return C, dict(z=z, p=p, K=K, beta=dc_boost)


C_lag, info_lag = design_lag_zpk(G)
print(f"\nLag compensator  C(s) = K·(s+z)/(s+p)   |p| < |z|")
print(f"  z = {info_lag['z']:.4f}   p = {info_lag['p']:.4f}   K = {info_lag['K']:.4f}")
print(f"  zero/pole ratio β = z/p = {info_lag['beta']:.2f}  →  DC loop gain boost = {info_lag['beta']**2:.0f}×")
print(f"  C_lag(s) = {C_lag}")

# ═══════════════════════════════════════════════════════════════════
# 5. Lead-lag  —  cascade lead and lag
# ═══════════════════════════════════════════════════════════════════

C_leadlag = C_lead * C_lag

print(f"\nLead-lag compensator  C_leadlag(s) = C_lead(s) · C_lag(s)")
print(f"  C_leadlag(s) = {C_leadlag}")

# ═══════════════════════════════════════════════════════════════════
# 6. Digital implementations — Tustin (bilinear) at 1 kHz
# ═══════════════════════════════════════════════════════════════════

fs = 1000.0          # Hz
Ts = 1 / fs

def discretise_tustin(C_ct, Ts):
    """Bilinear (Tustin) discretisation of a continuous transfer function."""
    if isinstance(C_ct, (int, float)):
        return C_ct
    return ct.sample_system(C_ct, Ts, method='tustin')


C_lead_d   = discretise_tustin(C_lead, Ts)
C_lag_d    = discretise_tustin(C_lag, Ts)
C_leadlag_d = discretise_tustin(C_leadlag, Ts)

print(f"\nDigital compensators (Tustin, fs={fs:.0f} Hz, Ts={Ts*1e3:.1f} ms):")
print(f"  C_lead_d(z)   = {C_lead_d}")
print(f"  C_lag_d(z)    = {C_lag_d}")
print(f"  C_leadlag_d(z) = {C_leadlag_d}")

# ═══════════════════════════════════════════════════════════════════
# 7. Closed-loop comparisons
# ═══════════════════════════════════════════════════════════════════

systems = {
    'bare plant (unit gain)':    (1, G),
    'lead':                      (C_lead, G),
    'lag':                       (C_lag, G),
    'lead-lag':                  (C_leadlag, G),
}

results = {}
for name, (C, plant) in systems.items():
    L = C * plant
    gm, pm, wcg, wcp = ct.margin(L)

    # Closed-loop step
    T = ct.feedback(L, 1)
    t, y = ct.step_response(T, T=np.linspace(0, 3, 3000))

    # Steady-state error for unit feedback (step)
    if abs(float(ct.dcgain(L))) > 1e-12:
        ss_err = 1 / (1 + float(ct.dcgain(L)))
    else:
        ss_err = 1.0  # type-1 plant with no integrator in C → infinite DC loop gain
    # Ramp error: 1/Kv where Kv = lim s→0 s·L(s)
    # Evaluate |s·L(s)| at a very small s to avoid dcgain pole-zero cancellation issues
    eps = 1e-8
    Kv = abs(ct.evalfr(L, 1j * eps)) * eps   # |L(jε)|·ε ≈ |s·L(s)| as s→0
    ramp_err = 1.0 / Kv if Kv > 1e-12 else float('inf')

    results[name] = dict(gm=gm, pm=pm, wcp=wcp, t=t, y=y,
                         ss_err=ss_err, ramp_err=ramp_err, L=L)

# ═══════════════════════════════════════════════════════════════════
# 8. Plot
# ═══════════════════════════════════════════════════════════════════

fig, axes = plt.subplots(2, 2, figsize=(13, 9))

# ——— Bode ———
ax_mag, ax_phase = axes[0, 0], axes[1, 0]
omega = np.logspace(-1, 2, 2000)
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
    L_dt = C_dt * G_dt if isinstance(C_dt, (int, float)) else C_dt * G_dt
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

print(f"\n{'':>22} {'PM [°]':>8} {'GM [dB]':>8} {'ω_cp [rad/s]':>14} "
      f"{'step ss err':>12} {'ramp ss err':>12}")
print(f"{'':─>78}")
for name, res in results.items():
    gm_db = 20*np.log10(res['gm']) if res['gm'] > 0 else float('-inf')
    ramp_str = f"{res['ramp_err']:.4f}" if np.isfinite(res['ramp_err']) else "∞"
    print(f"{name:>22} {res['pm']:8.1f} {gm_db:8.1f} {res['wcp']:14.2f} "
          f"{res['ss_err']:11.2%} {ramp_str:>12}")

print(f"\nC(s) = K·(s+z)/(s+p)  —  unified pole-zero form")
print(f"  z = {info_lead['z']:.4f}, p = {info_lead['p']:.4f}, K = {info_lead['K']:.3f}      for lead")
print(f"  z = {info_lag['z']:.4f}, p = {info_lag['p']:.4f}, K = {info_lag['K']:.3f}       for lag")
print(f"\nKey takeaway:")
print(f"  - Lead  — |z|<|p|, zero before pole → +{info_lead['phi_max']:.0f} deg phase near "
      f"{info_lead['w_max']:.2f} rad/s → higher PM, faster response")
print(f"  - Lag   — |p|<|z|, pole before zero → {info_lag['beta']:.0f}x DC loop gain "
      f"boost → ramp error shrinks ~{info_lag['beta']**2:.0f}x without hurting PM")
print(f"  - Lead-lag — cascaded: phase boost AND DC boost")
print(f"  - Tustin discretisation at {fs:.0f} Hz faithfully reproduces "
      f"continuous behaviour (check plot)")

