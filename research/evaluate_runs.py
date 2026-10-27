"""Summarize observed outcomes and compare independent numerical resolutions."""
from pathlib import Path
import json,argparse
import numpy as np
ROOT=Path(__file__).resolve().parents[1];RUNS=ROOT/'physics/runs'

def load(name):
    path=RUNS/name
    return np.load(path/'trajectory.npz'),json.loads((path/'metadata.json').read_text()),json.loads((path/'params.json').read_text())

def summary(name):
    source=name
    if name=='car_stop_large_floats_final' and (RUNS/'car_stop_large_floats_long/metadata.json').exists():source='car_stop_large_floats_long'
    a,m,p=load(source);t=a['t'];sel=t>=t[-1]-2;f=a['force'];mass=m['total_mass_kg'];q=a['qpos'][:,3:7]
    tilt=np.rad2deg(np.arccos(np.clip(1-2*(q[:,1]**2+q[:,2]**2),-1,1)))
    r=dict(name=name,vehicle=p['vehicle'],variant=p['variant'],water=p['water']['kind'],duration_s=float(t[-1]),mass_kg=mass,capacity_ratio=m['capacity_ratio'],max_displacement_m3=m['max_displacement_m3'],initial_speed_m_s=p['initial_speed'],drive_torque_nm_per_wheel=p['drive_torque_nm'],drive_power_w_total=p['total_drive_power_w'],final_z_m=m['final_z_m'],end_speed_m_s=m['final_speed_m_s'],tail_horizontal_speed_m_s=float(np.mean(np.linalg.norm(a['qvel'][sel,:2],axis=1))),tail_vertical_speed_m_s=float(np.mean(a['qvel'][sel,2])),tail_z_range_m=float(np.ptp(a['qpos'][sel,2])),max_tilt_deg=float(tilt.max()),end_tilt_deg=float(tilt[-1]),tail_angular_speed_rad_s=float(np.max(np.linalg.norm(a['qvel'][sel,3:6],axis=1))),tail_total_vertical_force_over_weight=float(np.mean(f[sel,:,2].sum(axis=1))/(mass*9.81)),tail_buoyancy_over_weight=m['final_mean_buoyancy_weight_ratio'],tail_panel_support_over_weight=m['final_mean_dynamic_vertical_weight_ratio'],energy_relative_residual=m['energy_balance_relative_error'],dissipation_max_relative_power_w=m['max_dissipative_relative_power_w'],stop=m['stop'])
    is_static=p['initial_speed']==0 and p['drive_off_s']==0
    settled=(abs(r['tail_vertical_speed_m_s'])<.005 and r['tail_z_range_m']<.025 and abs(r['tail_total_vertical_force_over_weight']-1)<.02 and r['tail_angular_speed_rad_s']<.03 and r['end_tilt_deg']<15)
    if r['final_z_m'] < -1.5:r['observed_outcome']='submerged'
    elif r['end_tilt_deg']>60:r['observed_outcome']='large_attitude_loss'
    elif is_static:r['observed_outcome']='settled_floating' if settled else 'floating_still_settling'
    else:r['observed_outcome']='continued_motion'
    r['static_settling_check']=settled if is_static else None
    r['source_run']=source
    r['needs_longer_static_run']=is_static and not settled and r['observed_outcome']!='submerged'
    return r

def interp(a,t,key):
    values=a[key];flat=values.reshape(len(values),-1)
    return np.stack([np.interp(t,a['t'],flat[:,j]) for j in range(flat.shape[1])],axis=1).reshape((len(t),)+values.shape[1:])

def compare(first,second,max_time=6.):
    a,ma,pa=load(first);b,mb,pb=load(second);end=min(max_time,a['t'][-1],b['t'][-1]);tt=np.arange(0,end+.0001,1/48)
    va=interp(a,tt,'qvel');vb=interp(b,tt,'qvel');qa=interp(a,tt,'qpos');qb=interp(b,tt,'qpos')
    speed_a=np.linalg.norm(va[:,:2],axis=1);speed_b=np.linalg.norm(vb[:,:2],axis=1)
    vrel=abs(np.mean(speed_a)-np.mean(speed_b))/max(.1,np.mean(speed_b));zrms=float(np.sqrt(np.mean((qa[:,2]-qb[:,2])**2)))
    angle={}
    for key in ['roll_deg','pitch_deg']:
        da=interp(a,tt,key);db=interp(b,tt,key);dd=(da-db+180)%360-180;angle[key]=float(np.sqrt(np.mean(dd**2)))
    fa=interp(a,tt,'force');fb=interp(b,tt,'force');ia=np.trapezoid(fa,tt,axis=0);ib=np.trapezoid(fb,tt,axis=0)
    impulse_error=np.linalg.norm(ia-ib,axis=1)/np.maximum(.01*mb['total_mass_kg']*9.81*end,np.linalg.norm(ib,axis=1))
    r=dict(first=first,second=second,compared_time_s=end,mean_horizontal_speed_relative_difference=float(vrel),z_trajectory_rms_difference_m=zrms,angle_rms_difference_deg=angle,group_impulse_relative_difference=dict(zip(mb['force_groups'],impulse_error.tolist())),max_group_impulse_relative_difference=float(impulse_error.max()))
    spatial=ma['grid_scale']!=mb['grid_scale'];threshold=.05 if spatial else .03
    r['passed']=bool(vrel<.02 and zrms<.02 and max(angle.values())<1 and impulse_error.max()<threshold)
    return r

if __name__=='__main__':
    report={'cases':[summary(p.name) for p in sorted(RUNS.glob('*_final')) if (p/'metadata.json').exists()], 'comparisons':[]}
    for case in ['bike_flat','bike_cut_power','bike_cross']:
        coarse=case+'_pilot' if case=='bike_flat' else case+'_dt2'
        for aa,bb in [(coarse,case+'_final'),(case+'_final',case+'_dt05')]:
            if all((RUNS/n/'metadata.json').exists() for n in [aa,bb]):report['comparisons'].append(compare(aa,bb))
    if all((RUNS/n/'metadata.json').exists() for n in ['bike_cross_final','bike_cross_grid2']):report['comparisons'].append(compare('bike_cross_final','bike_cross_grid2'))
    report['validation_complete']=len(report['cases'])==25 and len(report['comparisons'])==7 and all(x['passed'] for x in report['comparisons']) and not any(x['needs_longer_static_run'] for x in report['cases'])
    (ROOT/'research/scenario_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'completed_cases':len(report['cases']),'completed_comparisons':len(report['comparisons']),'comparison_failures':[x for x in report['comparisons'] if not x['passed']],'longer_static':[x['name'] for x in report['cases'] if x['needs_longer_static_run']],'validated':report['validation_complete']},ensure_ascii=False,indent=2))
