"""
Demonstrate the effect of zeros on a second-order system's step response.

Zeros are the roots of the numerator polynomial. For a second-order system,
adding a zero introduces a derivative component that can dramatically change
the transient response — including overshoot amplification, faster rise time,
or (for RHP zeros) initial undershoot.

Base system:  G(s) = ω_n² / (s² + 2ζω_n s + ω_n²)
With one zero: G(s) = (1 + s/z) · ω_n² / (s² + 2ζω_n s + ω_n²)
With two zeros: G(s) = (1 + s/z1)(1 + s/z2) · ...  (with proper pole count)
"""

import control as ct
import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # interactive; switch to "Agg" to save files only
import matplotlib.pyplot as plt


# ── System parameters ──────────────────────────────────────────────
zeta = 0.5          # damping ratio (underdamped)
wn = 2.0            # natural frequency [rad/s]

# ── Build the base second-order denominator (no zeros) ─────────────
den_base = [1, 2 * zeta * wn, wn**2]  # s² + 2ζω_n s + ω_n²

# ── Helper: create a tf and compute step response ──────────────────
def step_response(num, den, T=None):
    """Return (time, output) for a step response of num/den."""
    if T is None:
        T = np.linspace(0, 8, 1000)
    G = ct.tf(num, den)
    t, y = ct.step_response(G, T)
    return t, y


# ── Figure 1: One zero — varying its location ──────────────────────
fig1, (ax1a, ax1b) = plt.subplots(1, 2, figsize=(12, 5))

T = np.linspace(0, 8, 1000)

# --- Left subplot: LHP zeros (minimum-phase) ---
zeros_lhp = [None, -10, -4, -1.5, -0.8]  # None = no-zero baseline

for z in zeros_lhp:
    if z is None:
        num = [wn**2]                          # no zero
        label = "no zero (baseline)"
    else:
        num = [wn**2 / abs(z), wn**2]           # (1 + s/|z|) · ω_n²
        label = f"z = {z:.1f}"

    t, y = step_response(num, den_base, T)
    ax1a.plot(t, y, linewidth=1.5, label=label)

ax1a.axhline(1, color='gray', linestyle=':', linewidth=0.8)
ax1a.set_title("Left-half-plane (LHP) zeros\n(minimum-phase, stable numerator)")
ax1a.set_xlabel("Time [s]")


# --- Right subplot: RHP zeros (non-minimum-phase) ---
zeros_rhp = [None, 6, 2.5, 1.2, 0.7]  # positive = RHP

for z in zeros_rhp:
    if z is None:
        num = [wn**2]
        label = "no zero (baseline)"
        t, y = step_response(num, den_base, T)
    else:
        # (1 - s/z) = (-1/z s + 1); num = [ -wn²/z,  wn² ]
        num = [-wn**2 / z, wn**2]
        label = f"z = +{z:.1f}"

        # RHP zeros can go negative — extend time to see recovery
        t, y = step_response(num, den_base, T)

    ax1b.plot(t, y, linewidth=1.5, label=label)

ax1b.axhline(1, color='gray', linestyle=':', linewidth=0.8)
ax1b.set_title("Right-half-plane (RHP) zeros\n(non-minimum-phase, initial undershoot)")
ax1b.set_xlabel("Time [s]")

for ax in (ax1a, ax1b):
    ax.set_ylabel("Output")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.6, 2.2)

fig1.suptitle("Effect of a single zero on a 2nd-order system (ζ=0.5, ωₙ=2)", fontsize=13, y=1.02)
fig1.tight_layout()


# ── Figure 2: Two zeros ────────────────────────────────────────────
fig2, ax2 = plt.subplots(figsize=(9, 5.5))

# Base denominator — bump to 3rd-order so two-zero system is proper
# Use a fast pole at s = -10 to approximate pure 2-zero/2-pole behavior
den_3rd = np.convolve(den_base, [1, 10])   # (s² + …)(s + 10)

zero_pairs = {
    "no zeros (baseline)": [],                          # just ω_n² × 10
    "one LHP zero, z = −3": [3],
    "two LHP zeros, z = −3, −5": [3, 5],
    "two LHP zeros, z = −1.5, −3": [1.5, 3],
}

for label, z_vals in zero_pairs.items():
    # Numerator: ω_n² × 10 × ∏(1 + s/|z|)   [with proper scaling]
    num = [wn**2 * 10]          # baseline gain (include the fast pole gain)
    for zv in z_vals:
        num = np.convolve(num, [1 / zv, 1])     # (s/z + 1)
    t, y = step_response(num, den_3rd, T)
    ax2.plot(t, y, linewidth=1.5, label=label)

ax2.axhline(1, color='gray', linestyle=':', linewidth=0.8)
ax2.set_title("Multiple zeros on a (mostly) 2nd-order system\n(fast 3rd pole at s=−10 added for properness)")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Output")
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3)
ax2.set_ylim(-0.3, 2.5)
fig2.tight_layout()


# ── Figure 3: Zero–Pole map ───────────────────────────────────────
fig3, ax3 = plt.subplots(figsize=(6, 6))

# Draw a zero-pole map for one example: LHP zero at z=-1.5
num_demo = [wn**2 / 1.5, wn**2]   # (1 + s/1.5)·ω_n²
G_demo = ct.tf(num_demo, den_base)

poles = ct.poles(G_demo)
zeros = ct.zeros(G_demo)

ax3.scatter(np.real(poles), np.imag(poles), marker='x', s=100,
            color='red', linewidths=2, zorder=5, label=f"poles: {np.round(poles, 2)}")
ax3.scatter(np.real(zeros), np.imag(zeros), marker='o', s=100,
            facecolors='none', edgecolors='blue', linewidths=2, zorder=5,
            label=f"zero: {np.round(zeros, 2)}")

ax3.axhline(0, color='gray', linewidth=0.5)
ax3.axvline(0, color='gray', linewidth=0.5)
ax3.set_xlabel("Real")
ax3.set_ylabel("Imag")
ax3.set_title(f"Pole–Zero Map: 2nd-order system with one LHP zero at s=−1.5")
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3)
ax3.set_aspect('equal')

# Draw damping lines
ang = np.arccos(zeta)
rmax = wn * 1.8
ax3.plot([0, -rmax * zeta / wn], [0, rmax * np.sqrt(1 - zeta**2) / wn],
         'k--', linewidth=0.6, alpha=0.4)
ax3.plot([0, -rmax * zeta / wn], [0, -rmax * np.sqrt(1 - zeta**2) / wn],
         'k--', linewidth=0.6, alpha=0.4)
fig3.tight_layout()


fig1.savefig("zero_effect_one_zero.png", dpi=150, bbox_inches="tight")
fig2.savefig("zero_effect_two_zeros.png", dpi=150, bbox_inches="tight")
fig3.savefig("zero_effect_pzmap.png", dpi=150, bbox_inches="tight")
print("Figures saved to:")
print("  • zero_effect_one_zero.png")
print("  • zero_effect_two_zeros.png")
print("  • zero_effect_pzmap.png")
plt.show()


# ── Text summary ───────────────────────────────────────────────────
print("=" * 70)
print("SUMMARY — How zeros affect a 2nd-order step response")
print("=" * 70)
print()
print("1. LHP zero (minimum-phase):")
print("   • Closer to the origin → larger overshoot (derivative kick)")
print("   • Can reduce rise time (the zero 'pulls' the response up early)")
print("   • A zero far left (|z| >> ω_n) has negligible effect")
print()
print("2. RHP zero (non-minimum-phase):")
print("   • Causes initial undershoot — the output first goes the WRONG way")
print("   • Closer to the origin → deeper/longer undershoot")
print("   • Common in systems with competing dynamics (e.g., boiler level")
print("     control: adding cold water temporarily drops level before")
print("     steam bubbles collapse and level rises)")
print()
print("3. Multiple zeros:")
print("   • Each additional zero adds another derivative term")
print("   • Two LHP zeros amplify overshoot more than one")
print("   • A real system must be proper (den order ≥ num order), so")
print("     we add a fast pole to keep 2nd-order character")
