"""Plot the saved numerical entry trajectories; no simulation or interpolation."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
ROOT = Path(__file__).resolve().parents[1]
font = FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
plt.rcParams['axes.unicode_minus'] = False
colors = ['#8b95a8', '#ad7dde', '#db6b61', '#d59b32', '#248da2', '#4474c1']
fig, axes = plt.subplots(1, 2, figsize=(13.6, 6.3), dpi=150)
for ax, geometry, speeds in [(axes[0], 'paddles', [0,5,8,10,15,20]), (axes[1], 'four_skis', [0,5,8,10,15])]:
    for i, speed in enumerate(speeds):
        name = f'lowentry_20261027_{geometry}_v{speed}_p176'
        if geometry == 'four_skis' and speed in [10,15]:
            name = f'pln_four_skis_v{speed}_p176_30s'
        a = np.load(ROOT/'physics/runs'/name/'trajectory.npz')
        bad = np.flatnonzero(a['qpos'][:,2] < -1.5)
        end = int(bad[0])+1 if len(bad) else len(a['t'])
        mask = (np.arange(len(a['t'])) < end) & (a['t'] <= 8)
        ax.plot(a['t'][mask], a['qvel'][mask,0], color=colors[i], lw=2, label=f'{speed} m/s')
        if len(bad):
            ax.scatter(a['t'][end-1], a['qvel'][end-1,0], marker='x', s=70, linewidths=2, color=colors[i])
    ax.set_title('仅轮桨' if geometry == 'paddles' else '四滑板 + 轮桨', fontproperties=font, fontsize=14)
    ax.set_xlabel('入水后的时间 / s', fontproperties=font)
    ax.set_ylabel('初始前进方向的速度 / (m/s)', fontproperties=font)
    ax.grid(alpha=.22)
    ax.set_xlim(0,8)
    ax.set_ylim(-2,22)
    ax.legend(title='Initial speed', fontsize=9, ncol=2)
fig.suptitle('同176 kW上限：入水速度与低速起滑补测', fontproperties=font, fontsize=17)
fig.text(.5,.035,'×：车体参考原点低于水面1.5 m，随后继续下潜；曲线来自未校准的参数模型。', ha='center', fontproperties=font, fontsize=10)
fig.tight_layout(rect=(0,.075,1,.93))
fig.savefig(ROOT/'research/entry_speed_supplement.png', facecolor='white')
plt.close(fig)
