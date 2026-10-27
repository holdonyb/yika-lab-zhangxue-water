"""Read-only replay of final domain screens and transparent energy accounting."""
from pathlib import Path
import sys, json, hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'physics'))
import hydro_planing as h
from planing_surface import dynamic_normal_forces


def stat(values):
    a=np.asarray(values,dtype=float)
    return dict(min=float(a.min()),mean=float(a.mean()),max=float(a.max()))


def replay_domain(name):
    dest=ROOT/'physics/runs'/name
    p=json.loads((dest/'params.json').read_text());m=json.loads((dest/'metadata.json').read_text())
    assert m['source_sha256']==hashlib.sha256((ROOT/'physics/hydro_planing.py').read_bytes()).hexdigest()
    assert m['planing_surface_source_sha256']==hashlib.sha256((ROOT/'physics/planing_surface.py').read_bytes()).hexdigest()
    a=np.load(dest/'trajectory.npz');indices=np.flatnonzero(a['t']>=a['t'][-1]-5.)
    e=h.Experiment(p);d=e.data
    groups={};replay_error=[]
    for i in indices:
        d.time=float(a['t'][i]);d.qpos[:]=a['qpos'][i];d.qvel[:]=a['qvel'][i]
        h.mujoco.mj_forward(e.model,d);out=e.apply()
        replay_error.append(abs(out['planing_valid_z']-float(a['planing_valid_z'][i]))+abs(out['planing_extrapolated_z']-float(a['planing_extrapolated_z'][i])))
        for b in e.panels:
            spec=b['spec'];bid=b['bid'];R=d.xmat[bid].reshape(3,3)
            wp=b['points']@R.T+d.xpos[bid];eta,u,_=e.water.evaluate(wp,d.time)
            v=e.velocity[bid,3:]+np.cross(e.velocity[bid,:3],wp-d.xipos[bid])-u
            n=R@b['normal'];lon=R@b['longitudinal'];lat=R@b['lateral']
            wet=np.clip((eta-wp[:,2])/spec['wet_transition_m'],0,1)
            fn,diag=dynamic_normal_forces(e.water.rho,spec['size'][1],b['area'],b['chord'],wet,v,n,lon,lat,b['grid'])
            fz=fn*n[2];total=float(fz.sum())
            if total<=1e-9:continue
            speed=np.linalg.norm(v,axis=1);vn=v@n
            attack=np.rad2deg(np.arctan2(np.maximum(-vn,0),np.sqrt(np.maximum(speed**2-vn**2,0))))
            forward=v@lon;sideways=v@lat;cv=speed/np.sqrt(9.81*spec['size'][1])
            weights=wet*np.maximum(b['chord'],0)**2*((vn<0)&(wet>0));weights/=weights.sum()
            wetgrid=wet.reshape(b['grid']);spread=float(np.max(np.ptp(wetgrid,axis=1)))
            mean_attack=float(weights@attack)
            masks=dict(Cv_range=(cv>=.6)&(cv<=13),effective_attack_range=(attack>=2)&(attack<=15),
                lambda_range=diag['lambda']<=4,forward_positive=forward>0,
                sideslip_3deg=np.abs(sideways)<=np.maximum(forward,0)*np.tan(np.deg2rad(3)),
                continuous_wet_from_transom=diag['contiguous_transom_wet'],full_width_wet=diag['full_width_wet'],
                roll_5deg=abs(lat[2])<=np.sin(np.deg2rad(5)),speed_spread_10pct=diag['speed_relative_spread']<=.1,
                attack_spread_2deg=diag['attack_spread_deg']<=2,
                geometric_trim_range=2<=diag['geometric_trim_deg']<=15,
                vertical_flow_2deg=abs(diag['mean_vertical_flow_angle_deg'])<=2,
                effective_geometric_trim_difference_2deg=abs(mean_attack-diag['geometric_trim_deg'])<=2)
            g=groups.setdefault(spec['name'],dict(active_samples=0,lift_sample_sum_n=0.,valid_sample_sum_n=0.,failed_lift_sample_sum_n={k:0. for k in masks},
                sideslip_deg=[],geometric_trim_deg=[],vertical_flow_deg=[],wet_width_fraction_spread=[],roll_screen_deg=[],cp_error_m=[]))
            g['active_samples']+=1;g['lift_sample_sum_n']+=total;g['valid_sample_sum_n']+=diag['valid_force_z']
            for key,mask in masks.items():g['failed_lift_sample_sum_n'][key]+=float(np.sum(fz*np.logical_not(mask)))
            g['sideslip_deg'].append(float(weights@np.rad2deg(np.arctan2(sideways,forward))))
            g['geometric_trim_deg'].append(diag['geometric_trim_deg']);g['vertical_flow_deg'].append(diag['mean_vertical_flow_angle_deg'])
            g['wet_width_fraction_spread'].append(spread);g['roll_screen_deg'].append(float(np.rad2deg(np.arcsin(np.clip(lat[2],-1,1)))))
            g['cp_error_m'].append(diag['cp_error_m'])
    for g in groups.values():
        total=g['lift_sample_sum_n'];g['valid_fraction']=g['valid_sample_sum_n']/total
        g['failed_lift_fraction']={k:v/total for k,v in g.pop('failed_lift_sample_sum_n').items()}
        for key in ['sideslip_deg','geometric_trim_deg','vertical_flow_deg','wet_width_fraction_spread','roll_screen_deg','cp_error_m']:g[key]=stat(g[key])
    tail=indices
    course=np.rad2deg(np.arctan2(a['qvel'][tail,1],a['qvel'][tail,0]))
    heading_minus_course=(a['yaw_deg'][tail]-course+180)%360-180
    return dict(name=name,samples=len(indices),time_interval_s=[float(a['t'][indices[0]]),float(a['t'][indices[-1]])],
                roll_deg=stat(a['roll_deg'][tail]),pitch_deg=stat(a['pitch_deg'][tail]),yaw_deg=stat(a['yaw_deg'][tail]),course_deg=stat(course),
                heading_minus_course_deg=stat(heading_minus_course),yaw_change_deg=float(a['yaw_deg'][tail[-1]]-a['yaw_deg'][tail[0]]),
                replay_max_domain_force_error_n=max(replay_error),surfaces=groups)


def energy(name):
    dest=ROOT/'physics/runs'/name;m=json.loads((dest/'metadata.json').read_text());p=json.loads((dest/'params.json').read_text())
    a=np.load(dest/'trajectory.npz');t=a['t'];trap=np.trapezoid
    positive=np.maximum(a['motor_power'],0);dt=np.diff(t)
    motor_trap=float(trap(positive,t));motor_left=float(dt@positive[:-1]);motor_right=float(dt@positive[1:])
    sampled_net=np.sum(a['world_power'],axis=1)+a['motor_power']+a['steering_power']+a['passive_joint_power']
    sampled_work=float(trap(sampled_net,t));res=m['energy_balance_residual_j'];e0=m['initial_energy_j']
    return dict(name=name,duration_s=float(t[-1]),dt_s=p['dt'],output_hz=p['output_hz'],
        initial_energy_j=e0,mechanical_energy_change_j=m['mechanical_energy_change_j'],
        integrated_non_gravity_work_j=m['integrated_non_gravity_work_j'],signed_energy_residual_j=res,absolute_energy_residual_j=abs(res),
        original_energy_balance_relative_error=m['energy_balance_relative_error'],
        original_denominator_value=max(m['total_mass_kg']*9.81,abs(m['integrated_non_gravity_work_j'])),
        positive_motor_work_sampled_trapezoid_j=motor_trap,positive_motor_work_sampled_left_j=motor_left,positive_motor_work_sampled_right_j=motor_right,
        absolute_residual_over_initial_energy=abs(res)/abs(e0),absolute_residual_over_sampled_positive_motor_work=abs(res)/motor_trap,
        sampled_net_work_j=sampled_work,sampled_minus_solver_step_net_work_j=sampled_work-m['integrated_non_gravity_work_j'],
        note='Net work and residual are original solver-step metadata. Positive motor work is a trapezoid estimate from saved 48 Hz snapshots, with left/right sampled estimates; it is not the unavailable exact per-step positive-work integral. The alternative ratios do not replace the original metric.',
        group_world_work_sampled_j={g:float(trap(a['world_power'][:,i],t)) for i,g in enumerate(m['force_groups'])},
        source_sha256=m['source_sha256'],trajectory_sha256=m['trajectory_sha256'])


if __name__=='__main__':
    names=['pln_four_skis_v15_p176_30s','pln_four_skis_v10_p176_30s','pln_four_skis_v15_p176_roll1_30s','pln_four_skis_v15_p176_head',
           'pln_four_skis_v15_t0_p176','pln_four_skis_v15_p176_dt05']
    report=dict(scope='Read-only final-state domain replay and energy-ledger interpretation. No new trajectory or solver edits.',
                domain_replays=[replay_domain(n) for n in ['pln_four_skis_v15_p176_roll1_30s','pln_four_skis_v15_p176_30s']],
                energies=[energy(n) for n in names])
    (ROOT/'research/planing_final_result_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
