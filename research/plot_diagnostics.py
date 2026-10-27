from pathlib import Path
import argparse,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'figure.facecolor':'#f1f4f5','axes.facecolor':'white'})
ap=argparse.ArgumentParser();ap.add_argument('cases',nargs='*',default=['bike_flat_v2','car_large_floats_v2']);args=ap.parse_args()
fig,axs=plt.subplots(3,len(args.cases),figsize=(15,10),squeeze=False)
for j,name in enumerate(args.cases):
    path=ROOT/'physics/runs'/name;a=np.load(path/'trajectory.npz');m=json.loads((path/'metadata.json').read_text());t=a['t'];mass=m['total_mass_kg'];groups=m['force_groups']
    ax=axs[0,j];ax.plot(t,a['qpos'][:,2],color='#007b9b',label='车身参考高度');ax.plot(t,a['qvel'][:,0],color='#d67a12',label='向前速度');ax.legend();ax.set_title(name+'\n'+f'{mass:g} kg · MuJoCo state');ax.set_ylabel('m / (m/s)')
    ax=axs[1,j]
    for field,label,color in [('buoyancy','静浮力','#007b9b'),('panel_pressure','板面动支撑','#d67a12')]:ax.plot(t,a['force'][:,groups.index(field),2]/(mass*9.81),label=label,color=color)
    ax.axhline(1,color='#6d7680',ls='--',label='整车重量');ax.legend();ax.set_ylabel('向上力 / 重量')
    ax=axs[2,j];ax.plot(t,a['roll_deg'],label='横摇');ax.plot(t,a['pitch_deg'],label='俯仰');ax.legend();ax.set_ylabel('度');ax.set_xlabel('求解时间 / s')
    for ax in axs[:,j]:ax.grid(alpha=.2)
fig.suptitle('同一水动力模型，运动由受力积分得到',fontsize=20);fig.tight_layout(rect=(0,.025,1,.95));fig.text(.04,.015,'公开参数实验；质量、体积与水动力系数见各 run 的 params.json。曲线为求解输出。',fontsize=11)
fig.savefig(ROOT/'research/physical_sample_diagnostics.png',dpi=150)
