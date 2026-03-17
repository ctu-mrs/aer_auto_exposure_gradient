import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.gridspec import GridSpec

# --- Weight function ---
def soft_percentile_weights(p, k, n=1000):
    x = np.linspace(0, 1, n)
    w = np.where(
        x <= p,
        np.sin(np.pi * x / (2 * p)) ** k,
        np.sin(np.pi / 2 - np.pi * (x - p) / (2 * (1 - p))) ** k
    )
    # Normalize
    #w /= w.sum()   # not needed anymore, since the code is working with non-normalized function
    return x, w

# --- Initial values ---
p0, k0 = 0.75, 5.0

fig = plt.figure(figsize=(9, 6), facecolor="#1a1a2e")
fig.suptitle("Soft Percentile Weight Function", color="white", fontsize=14, fontweight="bold", y=0.97)

gs = GridSpec(2, 1, figure=fig, height_ratios=[5, 1], hspace=0.05)
ax = fig.add_subplot(gs[0])
ax.set_facecolor("#16213e")
for spine in ax.spines.values():
    spine.set_color("#444466")
ax.tick_params(colors="white")
ax.xaxis.label.set_color("white")
ax.yaxis.label.set_color("white")
ax.set_xlabel("Normalized gradient rank  x = i / S", labelpad=8)
ax.set_ylabel("Weight  W(x)", labelpad=8)
ax.set_xlim(0, 1)
ax.set_ylim(0)
ax.grid(True, color="#2a2a4a", linewidth=0.8)

x, w = soft_percentile_weights(p0, k0)
(line_left,) = ax.plot(x[x <= p0], w[x <= p0], color="#e94560", linewidth=2.5, label="ascending branch")
(line_right,) = ax.plot(x[x > p0],  w[x > p0],  color="#0f9be8", linewidth=2.5, label="descending branch")
vline = ax.axvline(p0, color="#f5a623", linewidth=1.2, linestyle="--", alpha=0.8, label=f"p = {p0:.2f}")
ax.legend(facecolor="#1a1a2e", edgecolor="#444466", labelcolor="white", fontsize=9)

# --- Sliders ---
slider_ax_p = fig.add_axes([0.15, 0.10, 0.70, 0.03], facecolor="#16213e")
slider_ax_k = fig.add_axes([0.15, 0.04, 0.70, 0.03], facecolor="#16213e")

slider_p = Slider(slider_ax_p, "p", 0.05, 0.99, valinit=p0, color="#e94560", track_color="#2a2a4a")
slider_k = Slider(slider_ax_k, "k", 0.0, 20.0, valinit=k0, color="#0f9be8", track_color="#2a2a4a")

for sl in (slider_p, slider_k):
    sl.label.set_color("white")
    sl.valtext.set_color("white")

def update(_):
    p = slider_p.val
    k = slider_k.val
    x, w = soft_percentile_weights(p, k)
    mask = x <= p
    line_left.set_xdata(x[mask]);  line_left.set_ydata(w[mask])
    line_right.set_xdata(x[~mask]); line_right.set_ydata(w[~mask])
    vline.set_xdata([p, p])
    ax.set_ylim(0, w.max() * 1.12)
    vline.set_label(f"p = {p:.2f}")
    ax.legend(facecolor="#1a1a2e", edgecolor="#444466", labelcolor="white", fontsize=9)
    fig.canvas.draw_idle()

slider_p.on_changed(update)
slider_k.on_changed(update)

update(None)  # draw correct initial state

plt.show()
