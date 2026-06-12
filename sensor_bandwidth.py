import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import control as ct

# Sensor Bandwidth Limit Demo — interactive slider exploration
#
# Two closed-loop step responses overlaid:
#   Blue — ideal sensor H(s)=1 (instant, perfect, all frequencies)
#   Red  — real sensor H(s)=ω_s/(s+ω_s) (bandlimited low-pass)
#
# Slide K_p up or ω_s down and watch the red trace diverge.
# The sensor bandwidth sets a hard ceiling on achievable loop gain.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import control as ct

# ---- time vector --------------------------------------------------
t = np.linspace(0, 5, 500)
init_Kp = 10.0
init_ws = 20.0

# ---- figure & axes -------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(left=0.1, bottom=0.35)

ax.set_title('Sensor Bandwidth Limit — Step Response', fontsize=14)
ax.set_xlabel('Time [s]')
ax.set_ylabel('Amplitude')
ax.grid(True, linestyle='--', alpha=0.7)

line_ideal,  = ax.plot([], [], 'b-',  lw=2,   label='ideal sensor  H(s) = 1')
line_actual, = ax.plot([], [], 'r-',  lw=1.5, label=f'bandlimited  H(s) = ω_s/(s+ω_s)')
ax.legend(loc='upper right')
ax.set_xlim(0, 5)

# ---- sliders -------------------------------------------------------
axcolor = 'lightgray'
ax_Kp   = plt.axes([0.15, 0.20, 0.65, 0.03], facecolor=axcolor)
ax_ws   = plt.axes([0.15, 0.10, 0.65, 0.03], facecolor=axcolor)

slider_Kp = Slider(ax_Kp, 'Kp (controller gain)', 1.0, 50.0,
                   valinit=init_Kp, valstep=0.5)
slider_ws = Slider(ax_ws, 'ω_s (sensor bandwidth rad/s)', 5.0, 100.0,
                   valinit=init_ws, valstep=1.0)

# ---- update --------------------------------------------------------
def update(val):
    Kp = slider_Kp.val
    ws = slider_ws.val

    # Plant:  G(s) = Kp / (s² + 2s + 1)   (2nd-order, DC gain = Kp)
    G = ct.tf([Kp], [1, 2, 1])

    # Ideal:  unity feedback  H(s) = 1
    sys_ideal = ct.feedback(G, 1)

    # Real:   bandlimited sensor  H(s) = ω_s / (s + ω_s)
    H = ct.tf([ws], [1, ws])
    sys_actual = ct.feedback(G, H)

    # Step responses
    t_ideal,  y_ideal  = ct.step_response(sys_ideal,  T=t)
    t_actual, y_actual = ct.step_response(sys_actual, T=t)

    line_ideal.set_data(t_ideal, y_ideal)
    line_actual.set_data(t_actual, y_actual)

    # Auto-scale y-axis, clamped to keep runaway traces readable
    max_y = max(np.max(y_ideal), np.max(y_actual))
    min_y = min(np.min(y_ideal), np.min(y_actual))
    if max_y > 4:  max_y = 4
    if min_y < -2: min_y = -2
    ax.set_ylim(min_y - 0.2, max_y + 0.5)

    fig.canvas.draw_idle()

slider_Kp.on_changed(update)
slider_ws.on_changed(update)
update(None)
plt.show()
