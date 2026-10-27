"""User-requested 15 m/s sensitivity checks, preserving the frozen model and baselines."""
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
import os,sys,json,hashlib,argparse
os.environ['OMP_NUM_THREADS']='1'
os.environ['OPENBLAS_NUM_THREADS']='1'
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'physics'))
import numpy as np
from hydro import run

FROZEN='20357f4c7d29238af982a7a1d9416462776ae0e1d8c0f97f9d1de6b017610693'
VARIANTS=['no_mod','paddles','small_floats','large_floats']
def read(p):return json.loads(p.read_text(encoding='utf8'))
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def job(entry):
    name,p=entry;dest=ROOT/'physics/runs'/name
    if (dest/'metadata.json').exists():
        assert read(dest/'params.json')==p
        assert read(dest/'metadata.json')['source_sha256']==FROZEN
    else:run(p,dest)
    return name

def summarize(name):
    path=ROOT/'physics/runs'/name;p=read(path/'params.json');m=read(path/'metadata.json');a=np.load(path/'trajectory.npz')
    t=a['t'];sel=t>=t[-1]-2;f=a['force'];groups=m['force_groups'];weight=m['total_mass_kg']*9.81
    return {'name':name,'variant':p['variant'],'initial_speed_m_s':p['initial_speed'],'wheel_target_rad_s':p['wheel_target_rad_s'],
        'duration_s':float(t[-1]),'stop':m['stop'],'end_horizontal_speed_m_s':float(np.linalg.norm(a['qvel'][-1,:2])),
        'final_z_m':float(a['qpos'][-1,2]),'final_vertical_speed_m_s':float(a['qvel'][-1,2]),
        'tail_mean_horizontal_speed_m_s':float(np.mean(np.linalg.norm(a['qvel'][sel,:2],axis=1))),
        'tail_buoyancy_weight_ratio':float(np.mean(f[sel,groups.index('buoyancy'),2])/weight),
        'tail_panel_support_weight_ratio':float(np.mean(f[sel,groups.index('panel_pressure'),2])/weight),
        'initial_motor_power_w':float(a['motor_power'][0]),'tail_mean_paddle_x_n':float(np.mean(f[sel,groups.index('paddles'),0])),
        'max_abs_pitch_deg':m['max_abs_pitch_deg'],'energy_balance_relative_error':m['energy_balance_relative_error'],
        'max_dissipative_relative_power_w':m['max_dissipative_relative_power_w'],
        'trajectory_sha256':m['trajectory_sha256'],'source_sha256':m['source_sha256'],
        'finite_states':bool(np.isfinite(a['qpos']).all() and np.isfinite(a['qvel']).all()),
        'observed_outcome':'submerged' if a['qpos'][-1,2]<-1.5 else 'afloat_at_end'}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workers',type=int,default=4);ap.add_argument('--summarize-only',action='store_true');args=ap.parse_args()
    assert digest(ROOT/'physics/hydro.py')==FROZEN
    jobs=[];changes=[]
    for variant in VARIANTS:
        base=read(ROOT/f'physics/runs/car_{variant}_final/params.json')
        for high_target in ([False] if variant=='no_mod' else [False,True]):
            p=json.loads(json.dumps(base));p['initial_speed']=15.
            if high_target:p['wheel_target_rad_s']=60.
            name=f'car_{variant}_entry15'+('_target60' if high_target else '')
            changed={k:{'baseline':base[k],'new':v} for k,v in p.items() if v!=base[k]}
            assert set(changed)==({'initial_speed','wheel_target_rad_s'} if high_target else {'initial_speed'})
            jobs.append((name,p));changes.append({'name':name,'baseline':f'car_{variant}_final','changed_parameters':changed})
    if not args.summarize_only:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            futures=[pool.submit(job,item) for item in jobs]
            for future in as_completed(futures):print('Completed '+future.result(),flush=True)
    report={'question':'Can the car enter at 15 m/s rather than the chosen 5 m/s?',
        'scope':'Same frozen quasi-steady water-force model; only initial speed changes in primary comparison. Separate control sensitivity raises wheel-speed target while keeping torque and power budgets unchanged.',
        'source_sha256':FROZEN,'changed_parameters':changes,'baselines':[summarize(f'car_{v}_final') for v in VARIANTS],
        'new_cases':[summarize(name) for name,_ in jobs],
        'controller_note':'Baseline wheel target is 29 rad/s, blade radius 0.405 m. A 15 m/s launch initializes the wheels at 15/0.34=44.12 rad/s, above that target, so the baseline motor controller initially brakes. Target60 cases distinguish this controller choice from initial-speed sensitivity. Torque remains 260 Nm and power 22 kW per driven wheel.',
        'physical_limits':'These are uncalibrated parameter experiments, not an optimized planing-car design or real-car performance predictions.',
        'new_resolution_comparison':'not yet performed'}
    report['all_states_finite']=all(r['finite_states'] for r in report['new_cases'])
    (ROOT/'research/car_entry_speed_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False})
    fig,axes=plt.subplots(2,4,figsize=(16,7),sharex='col',layout='constrained')
    labels=['原车','只加桨叶','小浮筒','大浮筒']
    for col,(variant,label) in enumerate(zip(VARIANTS,labels)):
        names=[(f'car_{variant}_final','5 m/s，原轮速目标','#718292'),(f'car_{variant}_entry15','15 m/s，原轮速目标','#d17e00')]
        if variant!='no_mod':names.append((f'car_{variant}_entry15_target60','15 m/s，提高轮速目标','#077b87'))
        for name,legend,color in names:
            a=np.load(ROOT/'physics/runs'/name/'trajectory.npz')
            axes[0,col].plot(a['t'],np.linalg.norm(a['qvel'][:,:2],axis=1),label=legend,color=color)
            axes[1,col].plot(a['t'],a['qpos'][:,2],color=color)
        axes[0,col].set_title(label);axes[0,col].legend(fontsize=7);axes[0,col].set_ylabel('水平速度 (m/s)')
        axes[1,col].set_ylabel('车体参考点高度 (m)');axes[1,col].set_xlabel('实际求解时间 (s)')
        for row in range(2):axes[row,col].grid(alpha=.25)
    fig.suptitle('入水初速 5 → 15 m/s：保留原车结构、质量、扭矩和功率\n另列轮速目标对照，避免把控制器主动减速混同为车体滑行能力',fontsize=14)
    fig.savefig(ROOT/'research/car_entry_speed_comparison.png',dpi=150);plt.close(fig)
    print(json.dumps(report,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
