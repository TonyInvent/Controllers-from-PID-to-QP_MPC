#!/usr/bin/env python3
"""
QP-MPC Servo Motor Demo — constrained QP beats naive saturation.

A properly solved QP knows the amplifier can only deliver ±V_max, so it plans
a control sequence that stays within limits *while accounting for the effect on
future states*.  Naive saturation ("just clip LQR") doesn't — it keeps computing
controls as if unlimited voltage existed, producing overshoot and oscillation.

The demo runs three controllers on the same 2nd-order DC motor:
  A. Unconstrained LQR  — fast, but demands 100+ V (impossible)
  B. Naive saturation   — clips LQR at ±12 V, overshoots because the
                           controller doesn't KNOW it's saturated
  C. QP-MPC             — solves a constrained QP at each step, pre-emptively
                           backs off the voltage to avoid overshoot

This directly illustrates the README's central argument:
  "Naive saturation says 'do your best, I'll clean up the mess.'
   Constrained optimisation says 'given the walls, find the best path
   that never touches them unnecessarily.'"

Usage:   .venv/bin/python3 servo_qp_mpc.py
"""

import numpy as np
from scipy.linalg import solve_discrete_are as dare
from matplotlib import pyplot as plt
import cvxpy as cp

# ═══════════════════════════════════════════════════════════════════
# 1. Motor model — 2nd-order DC servo (θ, ω)
# ═══════════════════════════════════════════════════════════════════

J  = 0.001       # rotor inertia [kg·m²]
b  = 0.01        # viscous friction [N·m·s/rad]
Kt = 0.1         # torque constant [N·m/A]
Rm = 1.0         # winding resistance [Ω]

# Continuous-time:  ẋ = Ac·x + Bc·u
A_eff = Kt**2 / Rm + b
Ac = np.array([[0.0, 1.0],
               [0.0, -A_eff / J]])
Bc = np.array([[0.0],
               [Kt / (Rm * J)]])
Cc = np.array([[1.0, 0.0]])

# ZOH discretisation at 1 ms
Ts = 0.001
n, m = 2, 1
M = np.zeros((n + m, n + m))
M[:n, :n] = Ac
M[:n, n:] = Bc
phi = np.linalg.matrix_power(np.eye(n+m) + M*Ts / 256, 256)
Ad = phi[:n, :n]
Bd = phi[:n, n:]

print(f"Motor: 2nd-order, Ts={Ts*1000:.0f} ms")
print(f"Ad eigenvalues: {np.linalg.eigvals(Ad)}")


# ═══════════════════════════════════════════════════════════════════
# 2. LQR design — aggressive tuning to force heavy saturation
# ═══════════════════════════════════════════════════════════════════

# Very aggressive:  huge penalty on position error, almost no damping,
# tiny control penalty → LQR will demand huge voltages
Q_lqr = np.diag([1000.0, 0.01])    # q_θ huge, q_ω tiny → oscillatory
R_lqr = np.array([[0.001]])        # almost free control → huge gains

P_lqr = dare(Ad, Bd, Q_lqr, R_lqr)
K_lqr = np.linalg.solve(R_lqr + Bd.T @ P_lqr @ Bd, Bd.T @ P_lqr @ Ad)

print(f"LQR gain: K = [{K_lqr[0,0]:.1f}, {K_lqr[0,1]:.4f}]")
print(f"closed-loop poles: {np.linalg.eigvals(Ad - Bd @ K_lqr)}")


# ═══════════════════════════════════════════════════════════════════
# 3. Run three controllers side-by-side
# ═══════════════════════════════════════════════════════════════════

T_total = 1.5
target  = 2.0                    # moderate step
V_max   = 12.0
n_steps = int(T_total / Ts)

ref = np.full(n_steps, target)
ref[:3] = 0.0

# ——— A. Unconstrained LQR ———
def run_lqr():
    x = np.zeros(2)
    h = dict(t=[], pos=[], volt=[])
    for k in range(n_steps):
        rk = ref[k];  xref = np.array([rk, 0.0])
        u = -(K_lqr @ (x - xref)).item()
        x = Ad @ x + Bd.flatten() * u
        h['t'].append(k*Ts); h['pos'].append(x[0]); h['volt'].append(u)
    return h

# ——— B. Naive saturation ———
def run_naive():
    x = np.zeros(2)
    h = dict(t=[], pos=[], volt=[])
    for k in range(n_steps):
        rk = ref[k];  xref = np.array([rk, 0.0])
        u_raw = -(K_lqr @ (x - xref)).item()
        u = np.clip(u_raw, -V_max, V_max)
        x = Ad @ x + Bd.flatten() * u
        h['t'].append(k*Ts); h['pos'].append(x[0]); h['volt'].append(u)
    return h

# ——— C. QP-MPC ———
def run_qp_mpc():
    N = 12   # prediction horizon

    # Condense once
    A_aug = np.vstack([np.linalg.matrix_power(Ad, k+1) for k in range(N)])
    B_aug = np.zeros((N*n, N*m))
    for row in range(N):
        for col in range(row + 1):
            B_aug[row*n:(row+1)*n, col] = (
                np.linalg.matrix_power(Ad, row-col) @ Bd).flatten()

    Qbar = np.kron(np.eye(N), Q_lqr)
    Qbar[-n:, -n:] = P_lqr           # terminal cost (stability guarantee)
    Rbar = np.kron(np.eye(N), R_lqr)

    H = B_aug.T @ Qbar @ B_aug + Rbar
    H = 0.5 * (H + H.T)
    F = A_aug.T @ Qbar @ B_aug       # maps x0 into linear cost term

    x0_param = cp.Parameter(n)
    U = cp.Variable(N)
    prob = cp.Problem(
        cp.Minimize(0.5 * cp.quad_form(U, H) + (F.T @ x0_param).T @ U),
        [U >= -V_max, U <= V_max])

    x = np.zeros(2)
    h = dict(t=[], pos=[], volt=[])
    for k in range(n_steps):
        rk = ref[k];  xref = np.array([rk, 0.0])
        x0_err = x - xref              # state error for regulation

        x0_param.value = x0_err
        prob.solve(solver=cp.OSQP, warm_start=True, polish=False,
                   max_iter=400, eps_abs=1e-4, eps_rel=1e-4)
        u = float(U.value[0]) if U.value is not None else 0.0
        u = np.clip(u, -V_max, V_max)

        x = Ad @ x + Bd.flatten() * u
        h['t'].append(k*Ts); h['pos'].append(x[0]); h['volt'].append(u)
    return h

h_lqr   = run_lqr()
h_naive = run_naive()
h_mpc   = run_qp_mpc()


# ═══════════════════════════════════════════════════════════════════
# 4. Plot — these traces ARE the lesson
# ═══════════════════════════════════════════════════════════════════

fig, (ax_pos, ax_volt) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)

t = h_lqr['t']
# Position
ax_pos.plot(t, ref,           'k--', alpha=0.25, lw=1, label='reference (2 rad)')
ax_pos.plot(t, h_lqr['pos'],  '#f0883e', lw=1.2, label='LQR (unconstrained)')
ax_pos.plot(t, h_naive['pos'],'#f85149', lw=1.5, label='naive sat. — clip LQR')
ax_pos.plot(t, h_mpc['pos'],  '#00bcd4', lw=2.0, label='QP-MPC — constrained QP')
ax_pos.set_ylabel('θ [rad]')
ax_pos.legend(fontsize=8, loc='lower right')
ax_pos.grid(True, alpha=0.15)
ax_pos.set_ylim(-0.3, target * 1.4)

# Voltage
ax_volt.plot(t, h_lqr['volt'],  '#f0883e', lw=1.0)
ax_volt.plot(t, h_naive['volt'],'#f85149', lw=1.5)
ax_volt.plot(t, h_mpc['volt'],  '#00bcd4', lw=2.0)
ax_volt.axhline(+V_max, color='red',  alpha=0.25, ls='--', lw=1)
ax_volt.axhline(-V_max, color='red',  alpha=0.25, ls='--', lw=1)
ax_volt.set_ylabel('V [V]')
ax_volt.set_xlabel('time [s]')
ax_volt.grid(True, alpha=0.15)

ax_pos.set_title(
    f'LQR servo — {target:.0f} rad step, ±{V_max:.0f} V limit  |  '
    f'K=[{K_lqr[0,0]:.0f}, {K_lqr[0,1]:.2f}]  |  N={12}',
    fontweight='bold')

fig.tight_layout(pad=1.0)
fig.savefig('qp_mpc_demo.png', dpi=150, facecolor='white')
print("\nPlot saved → qp_mpc_demo.png")
plt.show()


# ═══════════════════════════════════════════════════════════════════
# 5. Metrics — quantify the difference
# ═══════════════════════════════════════════════════════════════════

def metrics(h, name):
    pos = np.array(h['pos']);  volt = np.array(h['volt'])
    t90  = np.argmax(pos >= 0.9 * target) * Ts * 1e3
    ovs  = (max(pos) / target - 1) * 100
    peak = max(abs(volt))
    # oscillation count
    e = pos - target
    n_cross = np.sum(np.diff(np.sign(e)) != 0)
    settling = None
    for i in range(len(pos)-1, 0, -1):
        if abs(pos[i] - target) / target > 0.02:
            settling = (i + 1) * Ts * 1e3 if i+1 < len(pos) else None
            break
    return name, t90, ovs, peak, n_cross

rows = [metrics(h_lqr, 'LQR (unconstrained)'),
        metrics(h_naive, 'naive saturation'),
        metrics(h_mpc, 'QP-MPC')]

print(f"\n{'':>22} {'rise [ms]':>10} {'overshoot':>10} {'peak |V|':>10} {'oscill.':>8}")
for name, t90, ovs, peak, n_cross in rows:
    print(f"{name:>22} {t90:10.0f} {ovs:9.1f}% {peak:10.1f} {n_cross:8d}")

print(f"\nKey takeaway:")
print(f"  • LQR demands {rows[0][3]:.0f} V — completely unrealistic")
print(f"  • Naive saturation clips to ±{V_max:.0f} V but overshoots {rows[1][2]:.1f}% "
      f"(controller doesn't know it's saturated)")
print(f"  • QP-MPC pre-emptively backs off → only {rows[2][2]:.1f}% overshoot")
print(f"  • The voltage PROFILE is structurally different — not just clipped")
