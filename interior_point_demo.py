#!/usr/bin/env python3
"""
Interior-point method illustrated on the 2-variable LP from section 2.4.

Same LP as lp_geometry_demo.py (section 2.3):

    max  3*x1 + 2*x2
    s.t. x1 + x2 ≤ 4
         2*x1 + x2 ≤ 5
         x1, x2 ≥ 0

Optimal solution: x1 = 1, x2 = 3, objective = 9.

This script implements the primal logarithmic-barrier method:

  1. Replace each inequality a_i^T x ≤ b_i with a barrier term -ln(b_i - a_i^T x).
  2. Solve  min  t·c^T x - Σ ln(b_i - a_i^T x)  for increasing t.
  3. As t → ∞, the barrier minimiser x*(t) traces the *central path*
     from the analytic centre (t≈0) to the LP optimum.

The left panel shows the central path through the feasible region.  The
right panel shows how the barrier function distorts the objective at an
intermediate t — the barrier "walls" push the minimum away from the
constraint boundaries toward the interior.

Run:   .venv/Scripts/python.exe interior_point_demo.py
"""

import matplotlib
matplotlib.use('Agg')
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import LineCollection

# ═══════════════════════════════════════════════════════════════════════
# 1. LP data — identical to lp_geometry_demo.py & section 2.3
# ═══════════════════════════════════════════════════════════════════════

# minimize  c^T x   (c = negative of the max coefs)
c_obj = np.array([-3.0, -2.0])

# Inequality constraints:  A @ x <= b
A = np.array([[ 1.0,  1.0],    # x1 + x2 <= 4
              [ 2.0,  1.0],    # 2*x1 + x2 <= 5
              [-1.0,  0.0],    # -x1 <= 0  (x1 >= 0)
              [ 0.0, -1.0]])   # -x2 <= 0  (x2 >= 0)
b = np.array([4.0, 5.0, 0.0, 0.0])
m = 4                          # number of inequality constraints

# Feasible polygon vertices
O = np.array([0.0, 0.0])
A_vert = np.array([2.5, 0.0])
B_vert = np.array([1.0, 3.0])       # LP optimum
C_vert = np.array([0.0, 4.0])
poly_vertices = np.array([O, A_vert, B_vert, C_vert])


# ═══════════════════════════════════════════════════════════════════════
# 2. Logarithmic barrier & its derivatives
# ═══════════════════════════════════════════════════════════════════════

def slacks(x):
    """Slack variables s_i = b_i - a_i^T x.  Must all be > 0 for feasibility."""
    return b - A @ x


def barrier(x, t):
    """t * c^T x - Σ ln(s_i).  Returns inf if any slack ≤ 0."""
    s = slacks(x)
    if np.any(s <= 0):
        return np.inf
    return float(t * (c_obj @ x) - np.sum(np.log(s)))


def grad_barrier(x, t):
    """∇_x B(x, t) = t·c + Σ a_i / s_i."""
    s = slacks(x)
    return t * c_obj + A.T @ (1.0 / s)


def hess_barrier(x):
    """∇²B = Σ (a_i a_i^T) / s_i².  (Independent of t.)"""
    s = slacks(x)
    H = np.zeros((2, 2))
    for i in range(m):
        a_i = A[i]
        H += np.outer(a_i, a_i) / s[i]**2
    return H


def newton_decrement(x, t):
    """λ(x,t) = sqrt(∇B^T ∇²B^{-1} ∇B) — scaled stopping criterion."""
    g = grad_barrier(x, t)
    H = hess_barrier(x)
    dx = -np.linalg.solve(H, g)
    return float(np.sqrt(-g @ dx))


# ═══════════════════════════════════════════════════════════════════════
# 3. Analytic centre  (t → 0 limit of the central path)
# ═══════════════════════════════════════════════════════════════════════

def analytic_center(x0, max_iter=50):
    """Minimise the pure barrier  -Σ ln(s_i)  (t = 0)."""
    x = x0.copy()
    for _ in range(max_iter):
        g = A.T @ (1.0 / slacks(x))           # gradient of -Σ ln(s)
        H = hess_barrier(x)                     # Hessian
        dx = -np.linalg.solve(H, g)
        # Backtracking line search
        step = 1.0
        while True:
            xn = x + step * dx
            if np.all(slacks(xn) > 0):
                break
            step *= 0.5
            if step < 1e-14:
                return x
        x = xn
        if np.linalg.norm(grad_barrier(x, 1e-8)) < 1e-8:
            break
    return x


# ═══════════════════════════════════════════════════════════════════════
# 4. Newton centering step  (inner loop of the barrier method)
# ═══════════════════════════════════════════════════════════════════════

def newton_center(x0, t, max_iter=40, gtol=1e-8):
    """Starting from x0, find x*(t) = argmin B(·, t) via Newton's method.

    Returns (x_final, [list of iterates for visualisation]).
    """
    x = x0.copy()
    path = [x.copy()]
    for _ in range(max_iter):
        g = grad_barrier(x, t)
        H = hess_barrier(x)
        dx = -np.linalg.solve(H, g)
        λ2 = float(-g @ dx)          # squared Newton decrement

        # Backtracking line search with Armijo condition
        step = 1.0
        Bx = barrier(x, t)
        while True:
            xn = x + step * dx
            s_new = slacks(xn)
            if np.all(s_new > 0) and barrier(xn, t) <= Bx + 1e-4 * step * (g @ dx):
                break
            step *= 0.5
            if step < 1e-14:
                break
        x = xn
        path.append(x.copy())
        if λ2 < 2 * gtol:
            break
    return x, path


# ═══════════════════════════════════════════════════════════════════════
# 5. Barrier method — trace the full central path
# ═══════════════════════════════════════════════════════════════════════

def central_path(x0, t0=0.3, mu=3.0, num_outer=14, gtol=1e-8):
    """Run the primal barrier method, recording x*(t) at each outer iteration.

    Outer loop:  t ← μ·t   (warm-start Newton from previous x*(t)).
    Returns:  t_values, x_stars, all_newton_paths.
    """
    t = t0
    x = x0.copy()
    t_vals = []
    x_stars = []
    newton_paths = []

    for _ in range(num_outer):
        x, npath = newton_center(x, t, gtol=gtol)
        t_vals.append(t)
        x_stars.append(x.copy())
        newton_paths.append(npath)
        t *= mu

    return t_vals, x_stars, newton_paths


# ═══════════════════════════════════════════════════════════════════════
# 6. Run the method
# ═══════════════════════════════════════════════════════════════════════

# Start from the analytic centre
x_start = np.array([0.5, 0.5])
x_ac = analytic_center(x_start)
print(f"Analytic centre: ({x_ac[0]:.4f}, {x_ac[1]:.4f})")

t_vals, x_stars, newton_paths = central_path(x_ac, t0=0.3, mu=3.0, num_outer=14)
x_final = x_stars[-1]
print(f"Barrier-method optimum: ({x_final[0]:.4f}, {x_final[1]:.4f})")
print(f"Objective value: {-c_obj @ x_final:.4f}  (true LP opt: 9.0)")
print(f"Duality gap ~ m / t_max = {m / t_vals[-1]:.2e}")

# Print central-path progression
print(f"\n{'outer':>6} {'t':>10} {'x1':>8} {'x2':>8} {'obj':>8}")
for i, (tv, xs) in enumerate(zip(t_vals, x_stars)):
    print(f"{i:>6} {tv:>10.2f} {xs[0]:>8.4f} {xs[1]:>8.4f} {-c_obj@xs:>8.4f}")


# ═══════════════════════════════════════════════════════════════════════
# 7. Plot
# ═══════════════════════════════════════════════════════════════════════

fig, (ax_central, ax_barrier) = plt.subplots(1, 2, figsize=(15, 6.5))

# ── Colour map for t progression ─────────────────────────────────────
cmap = plt.cm.viridis
norm = plt.Normalize(np.log(t_vals[0]), np.log(t_vals[-1]))

# ── 7a. Left panel: central path ─────────────────────────────────────
ax = ax_central

# Feasible polygon
poly = Polygon(poly_vertices, closed=True, facecolor='#37474f',
               edgecolor='#78909c', linewidth=2, alpha=0.5, zorder=1)
ax.add_patch(poly)

# Constraint lines
xx = np.linspace(-0.5, 5.5, 150)
ax.plot(xx, 4 - xx,      '--', color='#ffb74d', lw=1.5, alpha=0.7,
        label=r'$x_1 + x_2 = 4$')
ax.plot(xx, 5 - 2 * xx,  '--', color='#4fc3f7', lw=1.5, alpha=0.7,
        label=r'$2x_1 + x_2 = 5$')

# Central path — colour-coded by log(t)
x_arr = np.array(x_stars)
for i in range(len(x_arr) - 1):
    c1 = cmap(norm(np.log(t_vals[i])))
    c2 = cmap(norm(np.log(t_vals[i + 1])))
    ax.plot(x_arr[i:i+2, 0], x_arr[i:i+2, 1], '-', color=c2, lw=3, alpha=0.85)

# Scatter dots on the central path
sc = ax.scatter(x_arr[:, 0], x_arr[:, 1], c=np.log(t_vals), cmap=cmap,
                norm=norm, s=50, edgecolors='white', linewidth=0.8,
                zorder=5, label='central path  $x^*(t)$')

# Analytic centre
ax.plot(x_ac[0], x_ac[1], 'D', color='#ce93d8', markersize=10,
        markeredgecolor='white', markeredgewidth=1, zorder=6)
ax.annotate('analytic centre\n    $(t \\to 0)$', xy=x_ac,
            xytext=(x_ac[0] + 1.6, x_ac[1] + 0.1),
            fontsize=8.5, color='#ce93d8',
            arrowprops=dict(arrowstyle='->', color='#ce93d8', lw=0.8))

# LP optimum
ax.plot(B_vert[0], B_vert[1], 'o', color='#00e676', markersize=13,
        markeredgecolor='white', markeredgewidth=2, zorder=6)
ax.annotate(rf'LP optimum' + '\n' + rf'$x^*=({B_vert[0]:.0f}, {B_vert[1]:.0f})$',
            xy=B_vert, xytext=(B_vert[0] + 1.0, B_vert[1] - 0.2),
            fontsize=9, color='#00e676',
            arrowprops=dict(arrowstyle='->', color='#00e676', lw=1.0))

# ── Newton steps at an intermediate t ─────────────────────────────────
# Pick an outer iteration to show Newton's method converging
show_outer = 4               # t ≈ 0.3 * 3^4 ≈ 24.3 — midway along path
npath = newton_paths[show_outer]
npath_arr = np.array(npath)
t_show = t_vals[show_outer]

ax.plot(npath_arr[:, 0], npath_arr[:, 1], 'o-', color='#ff7043', lw=2,
        markersize=6, markerfacecolor='#ffab91', markeredgecolor='white',
        markeredgewidth=0.5, zorder=7,
        label=rf'Newton steps at $t={t_show:.1f}$')

# Mark the centering target
x_t_show = x_stars[show_outer]
ax.plot(x_t_show[0], x_t_show[1], 's', color='#ff7043', markersize=10,
        markeredgecolor='white', markeredgewidth=1, zorder=7)

# Vertex labels
for name, pt, off in [('O', O, (-0.35, -0.35)),
                        ('A', A_vert, (0.15, -0.35)),
                        ('C', C_vert, (-0.35, 0.2))]:
    ax.annotate(name, xy=pt, xytext=(pt[0]+off[0], pt[1]+off[1]),
                fontsize=9, color='#90a4ae')

ax.text(1.2, 1.2, 'feasible\nregion', fontsize=9, color='#b0bec5',
        ha='center', va='center')
ax.set_xlabel(r'$x_1$', fontsize=12)
ax.set_ylabel(r'$x_2$', fontsize=12)
ax.set_xlim(-0.5, 5.5)
ax.set_ylim(-0.5, 5.0)
ax.set_xticks([0, 1, 2, 3, 4, 5])
ax.set_yticks([0, 1, 2, 3, 4])
ax.set_aspect('equal')
ax.grid(True, alpha=0.12)
ax.legend(loc='upper right', fontsize=7.5, framealpha=0.85,
          ncol=1, columnspacing=0.5)
ax.set_title('Central path & Newton centering steps',
             fontweight='bold', fontsize=12)

# Colour bar for t
cbar = fig.colorbar(sc, ax=ax, shrink=0.65, pad=0.02)
cbar.set_label(r'$\log(t)$', fontsize=10)
cbar.ax.tick_params(labelsize=7)

# ── 7b. Right panel: barrier landscape at t = 2 ──────────────────────
ax2 = ax_barrier

# Mesh for contour
n_grid = 300
x1g = np.linspace(0.02, 4.8, n_grid)
x2g = np.linspace(0.02, 4.3, n_grid)
X1, X2 = np.meshgrid(x1g, x2g)

t_demo = 2.0

# Evaluate barrier on grid (mask infeasible region)
B_grid = np.full_like(X1, np.nan)
for i in range(n_grid):
    for j in range(n_grid):
        xx = np.array([X1[i, j], X2[i, j]])
        s = slacks(xx)
        if np.all(s > 0):
            B_grid[i, j] = barrier(xx, t_demo)

# Contour fill of the barrier function
levels = np.linspace(np.nanmin(B_grid) + 0.5, np.nanmin(B_grid) + 20, 30)
cs = ax2.contourf(X1, X2, B_grid, levels=levels, cmap='plasma',
                  alpha=0.85)

# Barrier minimiser at t = t_demo
x_t_demo, np_demo = newton_center(x_ac, t_demo)
print(f"\nx*({t_demo}) = ({x_t_demo[0]:.4f}, {x_t_demo[1]:.4f})")

# Constraint boundaries
ax2.plot(xx, 4 - xx,      color='#ffb74d', lw=2, ls='--', alpha=0.8,
         label=r'$x_1+x_2=4$')
ax2.plot(xx, 5 - 2 * xx,  color='#4fc3f7', lw=2, ls='--', alpha=0.8,
         label=r'$2x_1+x_2=5$')

# LP objective contours (3x1+2x2 = const, recall c_obj = [-3,-2])
obj_levels = np.linspace(1, 9, 9)
ax2.contour(X1, X2, -c_obj[0]*X1 - c_obj[1]*X2, levels=obj_levels,
            colors='white', linewidths=0.5, alpha=0.35, linestyles='dotted')

# Barrier minimiser at this t
ax2.plot(x_t_demo[0], x_t_demo[1], 'D', color='#00e676', markersize=10,
         markeredgecolor='white', markeredgewidth=1.5, zorder=8)
ax2.annotate(rf'$x^*({t_demo})$' + '\nbarrier\nminimum',
             xy=x_t_demo, xytext=(x_t_demo[0]+1.0, x_t_demo[1]-0.8),
             fontsize=9, color='#00e676',
             arrowprops=dict(arrowstyle='->', color='#00e676', lw=1.2))

# True LP optimum
ax2.plot(B_vert[0], B_vert[1], 'o', color='white', markersize=12,
         markeredgecolor='#00e676', markeredgewidth=2, zorder=8)

# Infeasible region — grey hatching
ax2.fill_between(xx, 4 - xx, 5.5, alpha=0.15, color='#ffb74d')
ax2.fill_between(xx, 5 - 2*xx, 5.5, alpha=0.15, color='#4fc3f7')
# x1 < 0  (left of x1=0)
ax2.axvspan(-1, 0, alpha=0.08, color='grey')
# x2 < 0  (below x2=0)
ax2.axhspan(-1, 0, alpha=0.08, color='grey')

# Annotation: "barrier walls"
ax2.annotate('barrier "walls"\n(blow up near\nconstraints)',
             xy=(0.1, 3.5), fontsize=8.5, color='#ffcc80',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#263238',
                       edgecolor='#546e7a', alpha=0.7))

ax2.set_xlabel(r'$x_1$', fontsize=12)
ax2.set_ylabel(r'$x_2$', fontsize=12)
ax2.set_xlim(-0.5, 5.2)
ax2.set_ylim(-0.5, 4.5)
ax2.set_xticks([0, 1, 2, 3, 4, 5])
ax2.set_yticks([0, 1, 2, 3, 4])
ax2.set_aspect('equal')
ax2.legend(loc='upper right', fontsize=8, framealpha=0.8)
cbar2 = fig.colorbar(cs, ax=ax2, shrink=0.65, pad=0.02)
cbar2.set_label(rf'$B(x,\,t={t_demo:.0f})$', fontsize=10)
cbar2.ax.tick_params(labelsize=7)
ax2.set_title(rf'Barrier function landscape at $t = {t_demo:.0f}$',
              fontweight='bold', fontsize=12)

fig.suptitle(
    'Interior-point method — the central path from analytic centre to LP optimum',
    fontsize=14, fontweight='bold', y=1.01)
fig.tight_layout(pad=2.5)
fig.savefig('interior_point_demo.png', dpi=150, facecolor='white',
            bbox_inches='tight')
print("\nSaved → interior_point_demo.png")
