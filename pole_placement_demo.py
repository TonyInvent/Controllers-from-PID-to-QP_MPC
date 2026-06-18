#!/usr/bin/env python3
"""
Pole Placement Demo — Eigenvalue Design for a 2nd-Order Plant
==============================================================

This is the readable Python version of pole_placement_explorer.html.
It walks through the same concepts step by step:

  1. Define the plant (same A, B, C from eigenvalues_eigenvectors.md)
  2. Compute open-loop eigenvalues and step response
  3. Choose desired closed-loop poles
  4. Compute the feedback gain K via coefficient matching
  5. Simulate the closed-loop step response
  6. Plot: s-plane pole map + time-domain response

Plant:  x'' + 3x' + 2x = u   →   x' = Ax + Bu
  A = [[0, 1], [-2, -3]]      B = [[0], [1]]      C = [1, 0]
  Open-loop eigenvalues: -1, -2

Usage:
  python pole_placement_demo.py              # run all presets
  python pole_placement_demo.py --help       # see options
"""

import sys
import numpy as np
from numpy.linalg import eig, inv, matrix_rank
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


# =============================================================================
# PLANT DEFINITION
# =============================================================================
# From eigenvalues_eigenvectors.md Sections 5 & 15
# x'' + 3x' + 2x = u   (a 2nd-order ODE)
# State: x1 = position, x2 = velocity
A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
B = np.array([[0.0],
              [1.0]])
C = np.array([[1.0, 0.0]])

# --- open-loop eigenvalues ---
ol_eigs = eig(A)[0]
print("Open-loop eigenvalues of A:  %.3f, %.3f\n" % (ol_eigs[0].real, ol_eigs[1].real))


# =============================================================================
# HELPERS
# =============================================================================

def char_poly(M):
    """Return coefficients of s^2 + a1*s + a0 for a 2x2 matrix."""
    tr = np.trace(M)
    det = M[0, 0] * M[1, 1] - M[0, 1] * M[1, 0]
    return -tr, det           # (a1, a0)


def fmt_eig(val):
    """Pretty-print an eigenvalue (real or complex)."""
    if abs(val.imag) < 1e-9:
        return "%.3f" % val.real
    sign = "+" if val.imag >= 0 else "-"
    return "%.3f %s j%.3f" % (val.real, sign, abs(val.imag))


def zeta_omega(eigvals):
    """Damping ratio and natural frequency from dominant eigenvalue pair."""
    # find the dominant (slowest) pole
    if abs(eigvals[0].imag) > 1e-9:
        sigma = -eigvals[0].real
        wn = abs(eigvals[0])
        return sigma / wn, wn
    # both real — use the two real values
    r1, r2 = abs(eigvals[0].real), abs(eigvals[1].real)
    slow, fast = min(r1, r2), max(r1, r2)
    if slow < 1e-9:
        return 1.0, fast
    return (slow + fast) / (2 * np.sqrt(slow * fast)), np.sqrt(slow * fast)


# =============================================================================
# POLE PLACEMENT — coefficient matching for 2x2 SISO
# =============================================================================

def closed_loop_A(k1, k2):
    """A_cl = A - B K   where K = [k1, k2]."""
    return A - B @ np.array([[k1, k2]])


def compute_K_from_poles(sigma, omega):
    """
    Given desired closed-loop poles at  -sigma +/- j*omega,
    compute K = [k1, k2] by matching characteristic polynomials.

    Desired:    (s + sigma + jw)(s + sigma - jw) = s^2 + 2*sigma*s + (sigma^2 + w^2)
    Closed-loop char poly of A-BK:  s^2 + (3 + k2)*s + (2 + k1)

    Match:  3 + k2 = 2*sigma          =>  k2 = 2*sigma - 3
            2 + k1 = sigma^2 + w^2    =>  k1 = sigma^2 + w^2 - 2
    """
    a1 = 2.0 * sigma
    a0 = sigma * sigma + omega * omega
    return float(a0 - 2.0), float(a1 - 3.0)   # k1, k2


def compute_K_place(A_mat, B_mat, desired_poles):
    """
    General pole placement using Ackermann's formula via scipy or manual.
    Falls back to coefficient matching for 2x2 when scipy is unavailable.
    """
    try:
        from scipy.signal import place_poles
        result = place_poles(A_mat, B_mat, desired_poles)
        return result.gain_matrix[0]  # [k1, k2, ...]
    except ImportError:
        pass

    # Manual Ackermann for 2x2 SISO -------------------------------------------------
    n = A_mat.shape[0]
    # Desired characteristic polynomial coefficients (highest power first)
    poly = np.poly(desired_poles)          # [1, a_{n-1}, ..., a_0]
    # Controllability matrix
    Ctrb = B_mat.copy()
    Ak = A_mat.copy()
    for _ in range(1, n):
        Ctrb = np.hstack([Ctrb, Ak @ B_mat])
        Ak = Ak @ A_mat
    if np.linalg.matrix_rank(Ctrb) < n:
        raise ValueError("System is not controllable — cannot place poles arbitrarily.")

    # Ackermann: K = [0 ... 0 1] * Ctrb^{-1} * poly(A)
    poly_A = np.zeros_like(A_mat)
    for i, coeff in enumerate(poly):
        poly_A = poly_A + coeff * np.linalg.matrix_power(A_mat, n - i)
    e_n = np.zeros((1, n)); e_n[0, -1] = 1.0
    K = e_n @ np.linalg.inv(Ctrb) @ poly_A
    return K[0]


# =============================================================================
# SIMULATION — Runge-Kutta 4
# =============================================================================

def simulate(Acl, t_max=5.0, dt=0.001, use_ic=True):
    """
    Simulate closed-loop response.
    use_ic=True:  initial condition x(0) = [1, 0],  u = -Kx
    use_ic=False: zero initial, reference step r=1,  u = -Kx + r
    """
    n_steps = int(t_max / dt)
    t = np.linspace(0, t_max, n_steps)
    y = np.zeros(n_steps)
    x = np.array([1.0, 0.0]) if use_ic else np.array([0.0, 0.0])

    def f(xv, uv):
        return A @ xv + B.flatten() * uv

    for i in range(n_steps):
        y[i] = float((C @ x)[0])
        # Control
        K_implied = np.array([-(Acl[1, 0] + 2.0), -(Acl[1, 1] + 3.0)])
        u = float(-K_implied @ x + (0.0 if use_ic else 1.0))
        # RK4 step
        k1v = f(x, u)
        k2v = f(x + dt / 2 * k1v, u)
        k3v = f(x + dt / 2 * k2v, u)
        k4v = f(x + dt * k3v, u)
        x = x + dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)

    return t, y


# =============================================================================
# PLOTTING
# =============================================================================

def make_figure(k1, k2, Acl, cl_eigs, t, y, title, subtitle):
    """Draw a dual-panel figure: s-plane pole map + step response."""
    fig = plt.figure(figsize=(12, 5.5))
    gs = GridSpec(1, 2, figure=fig, width_ratios=[1, 1.1])

    # --- Left: s-plane pole map ---
    ax_pz = fig.add_subplot(gs[0])
    ax_pz.axhline(y=0, color="#444444", linewidth=0.8)
    ax_pz.axvline(x=0, color="#dd5555", linewidth=0.8, linestyle="--", alpha=0.6)
    # LHP / RHP shading
    ax_pz.axvspan(-20, 0, alpha=0.03, color="green")
    ax_pz.axvspan(0, 5, alpha=0.04, color="red")

    # Open-loop poles (x)
    ax_pz.plot(ol_eigs[0].real, ol_eigs[0].imag, "x", color="#4cc9f0", markersize=12,
               markeredgewidth=2, label="Open-loop  $\\lambda(A)$")
    ax_pz.plot(ol_eigs[1].real, ol_eigs[1].imag, "x", color="#4cc9f0", markersize=12,
               markeredgewidth=2)

    # Closed-loop poles (o)
    for i, ev in enumerate(cl_eigs):
        ax_pz.plot(ev.real, ev.imag, "o", color="#f0883e", markersize=10,
                   markeredgewidth=2, fillstyle="none",
                   label="Closed-loop  $\\lambda(A-BK)$" if i == 0 else "")
        # Migration arrow
        dx = ev.real - ol_eigs[i].real
        dy = ev.imag - ol_eigs[i].imag
        if abs(dx) > 0.02 or abs(dy) > 0.02:
            ax_pz.annotate("", xy=(ev.real, ev.imag),
                           xytext=(ol_eigs[i].real, ol_eigs[i].imag),
                           arrowprops=dict(arrowstyle="->", color="#888888",
                                           linewidth=1, linestyle="dotted"))
        ax_pz.annotate(" $\\lambda_%d$" % (i + 1), (ev.real, ev.imag),
                       textcoords="offset points", xytext=(18, -14 if i == 0 else 14),
                       fontsize=9, color="#f0883e")

    ax_pz.set_xlim(-12, 3)
    ax_pz.set_ylim(-10, 10)
    ax_pz.set_xlabel("Re(s)")
    ax_pz.set_ylabel("Im(s)")
    ax_pz.set_title("s-Plane Pole Map")
    ax_pz.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax_pz.set_aspect("equal")
    ax_pz.grid(True, alpha=0.2)

    # --- Right: step response ---
    ax_step = fig.add_subplot(gs[1])
    target = 1.0 if "ref" in subtitle.lower() else 0.0
    ax_step.plot(t, y, color="#58a6ff", linewidth=1.8, label="y(t)")
    # +/- 2 % bands
    ax_step.axhline(target + 0.02, color="#444444", linewidth=0.6, linestyle="--")
    ax_step.axhline(target - 0.02, color="#444444", linewidth=0.6, linestyle="--")
    ax_step.set_xlabel("Time [s]")
    ax_step.set_ylabel("y(t)")
    ax_step.set_title("Closed-Loop Response")
    ax_step.legend(loc="upper right", fontsize=8)
    ax_step.grid(True, alpha=0.2)

    # --- Readout text box ---
    zeta, wn = zeta_omega(cl_eigs)
    textstr = (
        "$K = [%.1f,\\ %.1f]$\n"
        "$\\lambda_1 = %s$\n"
        "$\\lambda_2 = %s$\n"
        "$\\zeta = %.3f$    $\\omega_n = %.2f$"
        % (k1, k2, fmt_eig(cl_eigs[0]), fmt_eig(cl_eigs[1]), zeta, wn)
    )
    props = dict(boxstyle="round,pad=0.4", facecolor="#1a1a2e", edgecolor="#444444", alpha=0.9)
    ax_step.text(0.97, 0.97, textstr, transform=ax_step.transAxes, fontsize=9,
                 fontfamily="monospace", verticalalignment="top", horizontalalignment="right",
                 bbox=props, color="#d0d0e0")

    fig.suptitle(title, fontsize=13, fontweight="bold", y=0.99)
    if subtitle:
        fig.text(0.5, 0.93, subtitle, ha="center", fontsize=9, color="#888888")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    return fig


# =============================================================================
# PRESET CONFIGURATIONS
# =============================================================================

PRESETS = {
    "Open-Loop":      {"k1": 0.0,  "k2": 0.0,  "title": "Open-Loop  (K = 0)"},
    "Doc Example":    {"k1": 18.0, "k2": 6.0,  "title": "Doc Example  (poles at -4, -5)"},
    "Slow":           {"k1": 2.0,  "k2": 1.0,  "title": "Slow  (poles near open-loop)"},
    "Fast":           {"k1": 30.0, "k2": 12.0, "title": "Fast  (poles far left)"},
    "Oscillatory":    {"k1": 8.0,  "k2": -1.0, "title": "Oscillatory  (low damping)"},
    "Barely Stable":  {"k1": -1.0, "k2": -2.0, "title": "Barely Stable  (poles near origin)"},
    "Aggressive":     {"k1": 50.0, "k2": 16.0, "title": "Aggressive  (very far left)"},
}


# =============================================================================
# MAIN — run & plot
# =============================================================================

def run_preset(name, cfg):
    k1, k2 = cfg["k1"], cfg["k2"]
    Acl = closed_loop_A(k1, k2)
    cl_eigs = eig(Acl)[0]

    # Simulate initial-condition response
    t_ic, y_ic = simulate(Acl, t_max=5.0, use_ic=True)

    zeta, wn = zeta_omega(cl_eigs)
    stability = "STABLE" if all(e.real < -1e-9 for e in cl_eigs) else \
                ("UNSTABLE" if any(e.real > 1e-9 for e in cl_eigs) else "MARGINAL")

    print("─" * 60)
    print("  %s" % cfg["title"])
    print("  K = [%.1f, %.1f]    %s    zeta=%.3f   wn=%.2f rad/s"
          % (k1, k2, stability, zeta, wn))
    print("  Closed-loop eigenvalues: %s, %s" % (fmt_eig(cl_eigs[0]), fmt_eig(cl_eigs[1])))
    a1, a0 = char_poly(Acl)
    print("  Char poly: s^2 + %.2f s + %.2f" % (a1, a0))
    print("  A-BK = [[%.1f, %.1f], [%.1f, %.1f]]" %
          (Acl[0, 0], Acl[0, 1], Acl[1, 0], Acl[1, 1]))

    fig = make_figure(k1, k2, Acl, cl_eigs, t_ic, y_ic,
                      cfg["title"],
                      "initial condition  x(0)=[1, 0]")
    return fig


def run_place_poles_example():
    """
    Demonstrate 'Place Poles' mode: choose desired pole locations,
    compute the needed K, then simulate.
    """
    print("\n" + "=" * 60)
    print("  PLACE-POLES EXAMPLE")
    print("  Desired closed-loop poles at  -3 +/- j4  (underdamped)")
    print("=" * 60)

    sigma, omega = 3.0, 4.0
    desired = np.array([-sigma + 1j * omega, -sigma - 1j * omega])
    k1, k2 = compute_K_from_poles(sigma, omega)
    print("  Computed K = [%.1f, %.1f]" % (k1, k2))

    Acl = closed_loop_A(k1, k2)
    cl_eigs = eig(Acl)[0]
    print("  Actual closed-loop eigenvalues: %s, %s" % (fmt_eig(cl_eigs[0]), fmt_eig(cl_eigs[1])))

    # Also demonstrate general Ackermann
    try:
        K_acker = compute_K_place(A, B, desired)
        print("  Ackermann K =", np.round(K_acker, 3))
    except ValueError as e:
        print("  Ackermann:", e)

    t_ic, y_ic = simulate(Acl, t_max=6.0, use_ic=True)
    fig = make_figure(k1, k2, Acl, cl_eigs, t_ic, y_ic,
                      "Place Poles: desired at -3 +/- j4",
                      "initial condition  x(0)=[1, 0]")
    return fig


def main():
    print("Pole Placement Demo — Eigenvalue Design")
    print("Plant: x'' + 3x' + 2x = u")
    print("A = [[0,1],[-2,-3]]   B = [[0],[1]]   C = [1,0]")
    print()

    figs = []

    # --- Run all presets ---
    for name, cfg in PRESETS.items():
        fig = run_preset(name, cfg)
        figs.append(fig)

    # --- Place-poles example ---
    fig_pp = run_place_poles_example()
    figs.append(fig_pp)

    # --- Controllability check ---
    print("\n" + "─" * 60)
    Ctrb = np.hstack([B, A @ B])
    print("  Controllability matrix rank: %d / %d  (full rank = can place poles anywhere)"
          % (np.linalg.matrix_rank(Ctrb), A.shape[0]))

    plt.show()


if __name__ == "__main__":
    main()
