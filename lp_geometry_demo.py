#!/usr/bin/env python3
"""
Geometric illustration of the simplex method on a 2-variable LP.

From section 2.3 of from_lp_to_qp_to_lqr.md:

    max  3*x1 + 2*x2
    s.t. x1 + x2 ≤ 4
         2*x1 + x2 ≤ 5
         x1, x2 ≥ 0

Optimal solution: x1 = 1, x2 = 3, objective = 9.

This script draws the feasible polytope, objective contours, and the
simplex pivot path from the origin to the optimum.
"""

import matplotlib
matplotlib.use('Agg')          # non-interactive — save to file only
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon

# ── LP data ───────────────────────────────────────────────────────────
#   max 3*x1 + 2*x2
#   s.t. x1 + x2 ≤ 4,   2*x1 + x2 ≤ 5,   x1, x2 ≥ 0

c = np.array([3.0, 2.0])

# Vertices of the feasible polygon, computed as intersections of
# constraint-line pairs.  Walk CCW around the boundary:
#   O:  x1=0 ∩ x2=0           → (0, 0)
#   A:  2*x1+x2=5 ∩ x2=0      → (2.5, 0)
#   B:  x1+x2=4 ∩ 2*x1+x2=5   → (1, 3)
#   C:  x1+x2=4 ∩ x1=0        → (0, 4)

O  = np.array([0.0, 0.0])
A  = np.array([2.5, 0.0])
B  = np.array([1.0, 3.0])
C  = np.array([0.0, 4.0])
sorted_vertices = np.array([O, A, B, C])

# ── Plot ──────────────────────────────────────────────────────────────
fig, (ax_static, ax_simplex) = plt.subplots(1, 2, figsize=(14, 6))

# Common extent
x1_range = np.linspace(-0.5, 5.5, 200)
X1, X2 = np.meshgrid(x1_range, np.linspace(-0.5, 5.5, 200))

for ax, title in [(ax_static, 'Feasible region & objective contours'),
                   (ax_simplex, 'Simplex pivot path (O → A → B)')]:
    # Feasible polygon
    poly = Polygon(sorted_vertices, closed=True, facecolor='#37474f',
                   edgecolor='#78909c', linewidth=2, alpha=0.55, zorder=1)
    ax.add_patch(poly)

    # Constraint lines
    xx = np.linspace(-0.5, 5.5, 100)
    # x1 + x2 = 4
    ax.plot(xx, 4 - xx, color='#ffb74d', lw=1.5, ls='--', alpha=0.8,
            label=r'$x_1 + x_2 = 4$')
    # 2*x1 + x2 = 5
    ax.plot(xx, 5 - 2*xx, color='#4fc3f7', lw=1.5, ls='--', alpha=0.8,
            label=r'$2x_1 + x_2 = 5$')

    # Shade the "infeasible" side lightly
    ax.fill_between(xx, 4 - xx, 6, alpha=0.06, color='#ffb74d')
    ax.fill_between(xx, 5 - 2*xx, 6, alpha=0.06, color='#4fc3f7')

    # Objective contours
    contour_levels = np.linspace(0, 12, 7)
    ax.contour(X1, X2, c[0]*X1 + c[1]*X2, levels=contour_levels,
               colors='#e0e0e0', linewidths=0.5, alpha=0.5, linestyles='dotted')

    # Objective gradient direction (arrow from origin)
    grad_len = 1.8
    ax.annotate('', xy=(c[0]*grad_len/np.linalg.norm(c),
                         c[1]*grad_len/np.linalg.norm(c)),
                xytext=(0, 0),
                arrowprops=dict(arrowstyle='->', color='#ef5350',
                                lw=2.5, shrinkA=0, shrinkB=0))
    ax.text(c[0]*grad_len/np.linalg.norm(c) + 0.15,
            c[1]*grad_len/np.linalg.norm(c) - 0.15,
            r'$c = (3, 2)$', color='#ef5350', fontsize=10, fontweight='bold')

    # Optimal point
    ax.plot(B[0], B[1], 'o', color='#00e676', markersize=12, zorder=5,
            markeredgecolor='white', markeredgewidth=1.5)
    ax.annotate(rf'$x^* = ({B[0]:.0f}, {B[1]:.0f})$' + '\n' + rf'$c^T x^* = {c@B:.0f}$',
                xy=(B[0], B[1]), xytext=(B[0]+1.2, B[1]+0.5),
                fontsize=10, color='#00e676',
                arrowprops=dict(arrowstyle='->', color='#00e676', lw=1.2))

    # Feasible region label
    ax.text(1.2, 1.2, 'feasible\nregion', fontsize=9, color='#b0bec5',
            ha='center', va='center')

    ax.set_xlabel(r'$x_1$', fontsize=12)
    ax.set_ylabel(r'$x_2$', fontsize=12)
    ax.set_xlim(-0.5, 5.2)
    ax.set_ylim(-0.5, 5.2)
    ax.set_xticks([0, 1, 2, 3, 4, 5])
    ax.set_yticks([0, 1, 2, 3, 4, 5])
    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.8)
    ax.grid(True, alpha=0.15)
    ax.set_title(title, fontweight='bold', fontsize=12)

# ── Left panel: label all vertices ────────────────────────────────────
for name, pt, offset in [('O (0,0)', O, (-0.4, -0.35)),
                          ('A (2.5, 0)', A, (0.15, -0.35)),
                          ('C (0, 4)', C, (-0.5, 0.2))]:
    ax_static.annotate(name, xy=pt, xytext=(pt[0]+offset[0], pt[1]+offset[1]),
                       fontsize=9, color='#b0bec5',
                       arrowprops=dict(arrowstyle='->', color='#546e7a', lw=0.8))
# B already labelled as optimum

# ── Right panel: show the simplex pivot path ──────────────────────────
# Label vertices
for name, pt, offset in [('O', O, (-0.25, -0.35)),
                          ('A', A, (0.15, -0.35)),
                          ('B (opt)', B, (-0.5, 0.25)),
                          ('C', C, (-0.35, 0.2))]:
    ax_simplex.annotate(name, xy=pt, xytext=(pt[0]+offset[0], pt[1]+offset[1]),
                        fontsize=9, color='#b0bec5')

# Draw simplex path O → A → B
path_vertices = np.array([O, A, B])
ax_simplex.plot(path_vertices[:, 0], path_vertices[:, 1],
                'o-', color='#ff7043', lw=3, markersize=10,
                markerfacecolor='#ffab91', markeredgecolor='white',
                markeredgewidth=1.5, zorder=6)

# Annotate the pivots
for i, (fr, to, label) in enumerate([
    (O, A, 'Pivot 1\nO → A\nobj = 0 → 7.5'),
    (A, B, 'Pivot 2\nA → B\nobj = 7.5 → 9')]):
    mid = (fr + to) / 2
    mid += np.array([0.5, 0.1]) if i == 0 else np.array([-0.4, 0.3])
    ax_simplex.annotate(label, xy=(fr+to)/2, xytext=mid,
                        fontsize=8, color='#ffcc80',
                        arrowprops=dict(arrowstyle='->', color='#ff7043',
                                        lw=0.8, alpha=0.6))

# Annotate "this vertex is not optimal — keep going" at A
ax_simplex.plot(A[0], A[1], 'o', color='#ff5252', markersize=10,
                zorder=5, markeredgecolor='white', markeredgewidth=1)
ax_simplex.annotate('reduced cost < 0\n→ keep pivoting',
                    xy=A, xytext=(A[0]+1.6, A[1]-0.6),
                    fontsize=8, color='#ff5252',
                    arrowprops=dict(arrowstyle='->', color='#ff5252', lw=0.8))

# ── Add the "why vertex?" inset explanation ──────────────────────────
# Draw a small blown-up view of the optimum corner showing that the
# objective contour is tangent to the feasible region boundary.
inset_ax = ax_simplex.inset_axes([0.52, 0.12, 0.44, 0.44])
inset_ax.set_xlim(0.3, 2.5)
inset_ax.set_ylim(2.2, 4.2)

# Mini feasible region near B
inset_poly = Polygon([(0, 4), (1, 3), (2.5, 0), (0, 0)], closed=True,
                     facecolor='#37474f', edgecolor='#78909c',
                     linewidth=1.5, alpha=0.55)
inset_ax.add_patch(inset_poly)

# Constraint lines near B
xx_i = np.linspace(0.3, 2.5, 50)
inset_ax.plot(xx_i, 4 - xx_i, '--', color='#ffb74d', lw=1.2, alpha=0.9)
inset_ax.plot(xx_i, 5 - 2*xx_i, '--', color='#4fc3f7', lw=1.2, alpha=0.9)

# Dense objective contours near B to show tangency
obj_val = c @ B  # 9
for dv in np.linspace(-0.4, 0.4, 5):
    level = obj_val + dv
    # contour: 3*x1 + 2*x2 = level
    xxc = np.linspace(0.3, 2.5, 50)
    x2c = (level - 3*xxc) / 2
    inset_ax.plot(xxc, x2c, color='#e0e0e0', lw=0.6, alpha=0.6)

# The optimal contour (level = 9) in bold
inset_ax.plot(xxc, (obj_val - 3*xxc)/2, color='#00e676', lw=2, alpha=0.9,
              label=r'$3x_1+2x_2=9$')
inset_ax.plot(B[0], B[1], 'o', color='#00e676', markersize=10, zorder=5,
              markeredgecolor='white', markeredgewidth=1.5)

# Normal vectors to show why it's optimal
n1 = np.array([1.0, 1.0])   # normal to x1+x2=4
n2 = np.array([2.0, 1.0])   # normal to 2*x1+x2=5
scale = 0.6
for n, color_i, label_i in [(n1, '#ffb74d', r'$n_1$'),
                              (n2, '#4fc3f7', r'$n_2$')]:
    n_unit = n / np.linalg.norm(n) * scale
    inset_ax.annotate('', xy=(B[0]+n_unit[0], B[1]+n_unit[1]), xytext=B,
                      arrowprops=dict(arrowstyle='->', color=color_i, lw=1.5))
    inset_ax.text(B[0]+n_unit[0]+0.1, B[1]+n_unit[1]+0.05,
                  label_i, color=color_i, fontsize=8)

# c as positive combination of normals
inset_ax.annotate(r'$c = \frac{1}{1}(n_1 + n_2)$' + '\n' + r'$c^T d \leq 0$ ' + r'$\forall$ feasible $d$',
                  xy=(0.25, 0.92), xycoords='axes fraction',
                  fontsize=8.5, color='#eeeeee', ha='left', va='top',
                  bbox=dict(boxstyle='round,pad=0.4', facecolor='#263238',
                            edgecolor='#546e7a', alpha=0.8))
inset_ax.legend(fontsize=7, loc='lower left')
inset_ax.set_xticks([])
inset_ax.set_yticks([])
inset_ax.set_title('Optimum: c is a positive\ncombination of active normals',
                   fontsize=8.5, color='#90a4ae')

fig.suptitle('Simplex geometry: walking the edges of the feasible polytope',
             fontsize=14, fontweight='bold', y=1.01)
fig.tight_layout(pad=2)
fig.savefig('lp_geometry_demo.png', dpi=150, facecolor='white',
            bbox_inches='tight')
print("Saved → lp_geometry_demo.png")
