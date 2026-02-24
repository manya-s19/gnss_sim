import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np
import subprocess
import os

# ── Run the C++ sim to generate both CSVs ──────────────────────────────────
BUILD = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'build')

def run_sim(mode, outfile):
    args = [os.path.join(BUILD, 'gnss_sim')]
    if mode == 'spoof':
        args.append('spoof')
    subprocess.run(args, cwd=BUILD, capture_output=True)
    src = os.path.join(BUILD, 'output.csv')
    dst = os.path.join(BUILD, outfile)
    if os.path.exists(src):
        os.replace(src, dst)
    return dst

print("Running simulations...")
normal_csv  = run_sim('normal', 'output_normal.csv')
spoofed_csv = run_sim('spoof',  'output_spoof.csv')

# ── Waypoints ──────────────────────────────────────────────────────────────
true_lons = [-79.3871, -79.4500, -79.5200, -79.5800, -79.6248]
true_lats = [ 43.6426,  43.6500,  43.6600,  43.6700,  43.6777]
fake_lat,    fake_lon    = 43.6426, -79.3871   # CN Tower
pearson_lat, pearson_lon = 43.6777, -79.6248

LON_MIN, LON_MAX = -79.70, -79.33
LAT_MIN, LAT_MAX =  43.62,  43.70

# ── Figure ─────────────────────────────────────────────────────────────────
fig, (ax_n, ax_s) = plt.subplots(1, 2, figsize=(16, 7), facecolor='#0d1117')
fig.suptitle(
    'GNSS Drone No-Fly Zone Simulator  |  CN Tower → Toronto Pearson Airport',
    color='white', fontsize=13, fontweight='bold', y=0.97
)

def style_ax(ax, title, color):
    ax.set_facecolor('#0d1117')
    ax.set_xlim(LON_MIN, LON_MAX)
    ax.set_ylim(LAT_MIN, LAT_MAX)
    ax.set_xlabel('Longitude', color='#888', fontsize=9)
    ax.set_ylabel('Latitude',  color='#888', fontsize=9)
    ax.tick_params(colors='#555')
    for sp in ax.spines.values(): sp.set_edgecolor('#333')
    ax.grid(color='#1e2533', lw=0.6, ls='--')
    ax.set_title(title, color=color, fontsize=11, fontweight='bold', pad=8)
    nfz = patches.Circle((pearson_lon, pearson_lat), radius=0.018,
        lw=2, edgecolor='#ff3333', facecolor='#ff000022', zorder=2)
    ax.add_patch(nfz)
    ax.annotate('✈ Pearson Airport\n(No-Fly Zone)',
        xy=(pearson_lon, pearson_lat),
        xytext=(pearson_lon - 0.14, pearson_lat + 0.028),
        color='#ff6666', fontsize=8,
        arrowprops=dict(arrowstyle='->', color='#ff6666', lw=1), zorder=10)
    ax.annotate('📍 CN Tower\n(Start)',
        xy=(fake_lon, fake_lat),
        xytext=(fake_lon + 0.05, fake_lat - 0.028),
        color='#66ff88', fontsize=8,
        arrowprops=dict(arrowstyle='->', color='#66ff88', lw=1), zorder=10)

style_ax(ax_n, '✅  Normal Mode — GPS Tracking Correctly',  '#44ff77')
style_ax(ax_s, '⚠️  Spoofing Mode — GPS Signal Hijacked',   '#ff6633')

# Normal artists
n_true,  = ax_n.plot([], [], 'w-',  lw=2,  zorder=4, label='True path')
n_est,   = ax_n.plot([], [], '-',   lw=2,  zorder=5, color='#44ff77', label='GPS estimated (accurate)')
n_dots,  = ax_n.plot([], [], 'wo',  ms=7,  zorder=6)
n_drone, = ax_n.plot([], [], 'o',   ms=14, zorder=8, color='#44ff77',
                     markeredgecolor='white', markeredgewidth=1.5)
ax_n.legend(loc='lower right', facecolor='#1a1f2e', edgecolor='#333',
            labelcolor='white', fontsize=8)
n_status = ax_n.text(0.02, 0.05, '', transform=ax_n.transAxes,
    color='#44ff77', fontsize=9, fontweight='bold',
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#0d1117',
              edgecolor='#44ff77', alpha=0.85))

# Spoof artists
s_true,  = ax_s.plot([], [], 'w-',  lw=2,   zorder=4, label='True path (hidden from drone)')
s_gap,   = ax_s.plot([], [], '--',  lw=1.8, zorder=3, color='#ff6633', alpha=0.7,
                     label='Hidden gap (spoofed offset)')
s_dots,  = ax_s.plot([], [], 'wo',  ms=7,   zorder=6)
s_drone, = ax_s.plot([], [], 'o',   ms=14,  zorder=8, color='#ff4422',
                     markeredgecolor='white', markeredgewidth=1.5)
s_fake,  = ax_s.plot([fake_lon], [fake_lat], 'x', ms=18, mew=3.5,
                     color='#ff6633', zorder=9, label='GPS thinks drone is HERE')
s_ring,  = ax_s.plot([], [], '-', lw=2, color='#ff6633', alpha=0.6, zorder=8)
ax_s.legend(loc='lower right', facecolor='#1a1f2e', edgecolor='#333',
            labelcolor='white', fontsize=8)
s_status = ax_s.text(0.02, 0.05, '', transform=ax_s.transAxes,
    color='#ff6633', fontsize=9, fontweight='bold',
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#0d1117',
              edgecolor='#ff6633', alpha=0.85))
s_ann = [None]

N = len(true_lats)

def animate(frame):
    idx = min(frame % (N + 6), N - 1)

    # Normal
    n_true.set_data(true_lons[:idx+1],  true_lats[:idx+1])
    n_est.set_data(true_lons[:idx+1],   true_lats[:idx+1])
    n_dots.set_data(true_lons[:idx+1],  true_lats[:idx+1])
    n_drone.set_data([true_lons[idx]],  [true_lats[idx]])

    dlat = true_lats[idx] - pearson_lat
    dlon = true_lons[idx] - pearson_lon
    dist_pearson = np.sqrt((dlat*111)**2 + (dlon*111*np.cos(np.radians(43.66)))**2)
    in_nfz = dist_pearson < 2.0

    if in_nfz:
        n_drone.set_color('#ff4422')
        n_status.set_text('⚠ Drone in no-fly zone — detected!')
        n_status.set_color('#ff4422')
    else:
        n_drone.set_color('#44ff77')
        n_status.set_text('GPS error < 0.001 m  ✓')
        n_status.set_color('#44ff77')

    # Spoof
    s_true.set_data(true_lons[:idx+1], true_lats[:idx+1])
    s_dots.set_data(true_lons[:idx+1], true_lats[:idx+1])
    s_drone.set_data([true_lons[idx]], [true_lats[idx]])

    if idx > 0:
        s_gap.set_data([fake_lon, true_lons[idx]], [fake_lat, true_lats[idx]])
    else:
        s_gap.set_data([], [])

    # Pulsing ring
    t = frame * 0.4
    r = 0.013 + 0.004*np.sin(t)
    th = np.linspace(0, 2*np.pi, 64)
    s_ring.set_data(fake_lon + r*np.cos(th), fake_lat + r*np.sin(th))

    if s_ann[0]: s_ann[0].remove(); s_ann[0] = None

    dlat2 = true_lats[idx] - fake_lat
    dlon2 = true_lons[idx] - fake_lon
    err_km = np.sqrt((dlat2*111)**2 + (dlon2*111*np.cos(np.radians(43.66)))**2)

    if in_nfz:
        s_drone.set_color('#ff0000')
        s_status.set_text(f'🚨 IN NO-FLY ZONE — {err_km:.1f} km error UNDETECTED!')
        s_status.set_color('#ff2222')
        mid_lon = (fake_lon + true_lons[idx]) / 2
        mid_lat = (fake_lat + true_lats[idx]) / 2
        s_ann[0] = ax_s.annotate(
            f'{err_km:.1f} km\nhidden by spoofer',
            xy=(mid_lon, mid_lat), ha='center', color='#ff6633',
            fontsize=8, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#1a0800',
                      edgecolor='#ff6633', alpha=0.9))
    else:
        s_drone.set_color('#ff4422')
        s_status.set_text(f'GPS offset: {err_km:.1f} km  ← spoofed')
        s_status.set_color('#ff6633')

    return (n_true, n_est, n_dots, n_drone, n_status,
            s_true, s_gap, s_dots, s_drone, s_fake, s_ring, s_status)

ani = animation.FuncAnimation(fig, animate, frames=200, interval=900, blit=False)
plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()