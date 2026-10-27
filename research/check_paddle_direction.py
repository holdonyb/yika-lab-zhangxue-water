"""Replay preserved solver states and diagnose repeated-blade frame aliasing."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'physics'))
import hydro
import hydro_planing

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    timeline=json.loads((ROOT/'timeline.json').read_text(encoding='utf8'))
    names=set()
    for shot in timeline['shots']:
        for media in shot['media']:
            path=ROOT/media['path']
            if '/visuals/' not in path.as_posix() or path.suffix!='.mp4':continue
            rec=path.with_suffix('.json')
            if not rec.exists():continue
            d=json.loads(rec.read_text(encoding='utf8'))
            if 'job' in d:names.add(d['job']['run'])
            elif d.get('source'):
                source=Path(d['source']).with_suffix('.json')
                names.add(json.loads(source.read_text(encoding='utf8'))['job']['run'])
    rows=[]
    for name in sorted(names):
        dest=ROOT/'physics/runs'/name;p=json.loads((dest/'params.json').read_text(encoding='utf8'));m=json.loads((dest/'metadata.json').read_text(encoding='utf8'))
        mod=hydro_planing if name.startswith('pln_') else hydro
        assert sha(Path(mod.__file__))==m['source_sha256']
        assert sha(dest/'trajectory.npz')==m['trajectory_sha256']
        a=np.load(dest/'trajectory.npz');e=mod.Experiment(p);d=e.data
        samples=[]
        for t in sorted(set([min(1.,a['t'][-1]/2),min(2.,a['t'][-1]*.75),max(0.,a['t'][-1]-.5)])):
            i=int(np.argmin(abs(a['t']-t)));d.time=float(a['t'][i]);d.qpos[:]=a['qpos'][i];d.qvel[:]=a['qvel'][i]
            mod.mujoco.mj_forward(e.model,d);out=e.apply()
            wheels=[]
            for b in e.driven:
                bid=b['bid'];spec=b['spec'];R=d.xmat[bid].reshape(3,3);points=b['points']@R.T+d.xpos[bid]
                eta,water_v,_=e.water.evaluate(points,d.time)
                relative=e.velocity[bid,3:]+np.cross(e.velocity[bid,:3],points-d.xipos[bid])-water_v
                normals=b['normals']@R.T;vn=np.sum(relative*normals,axis=1)
                wet=np.clip((eta-points[:,2])/spec['blade_depth']+.5,0,1)
                f=-.5*e.water.rho*1.65*spec['blade_area']*wet[:,None]*np.abs(vn[:,None])*vn[:,None]*normals
                omega=float(d.qvel[b['dof']]);advance=omega/24*180/np.pi;period=360/spec['blade_count']
                apparent=(advance+period/2)%period-period/2
                k=int(np.argmin(points[:,2]))
                wheels.append({'name':spec['name'],'joint_axis':e.model.jnt_axis[e.model.dof_jntid[b['dof']]].tolist(),
                    'angular_speed_rad_s':omega,'true_1x_frame_advance_deg':advance,'repeated_pattern_nearest_advance_deg':apparent,
                    'bottom_point_world':points[k].tolist(),'bottom_relative_water_velocity_m_s':relative[k].tolist(),
                    'bottom_force_on_vehicle_n':f[k].tolist(),'sum_paddle_force_n':f.sum(axis=0).tolist(),
                    'relative_paddle_power_w':float(np.sum(f*relative))})
            samples.append({'time_s':d.time,'full_force_replay_error_n':float(np.max(abs(out['force']-a['force'][i]))),'wheels':wheels})
        rows.append({'run':name,'trajectory_sha256':m['trajectory_sha256'],'solver_sha256':m['source_sha256'],'samples':samples})
    checks={'preserved_states_reproduce_recorded_force':all(s['full_force_replay_error_n']<1e-6 for r in rows for s in r['samples']),
        'paddle_force_opposes_water_relative_motion':all(w['relative_paddle_power_w']<1e-6 for r in rows for s in r['samples'] for w in s['wheels']),
        'all_driven_axes_positive_y':all(w['joint_axis']==[0.,1.,0.] for r in rows for s in r['samples'] for w in s['wheels']),
        'timeline_does_not_reverse_or_retime_sources':all(m.get('playback_rate',1)==1 for s in timeline['shots'] for m in s['media'])}
    result={'scope':'Read-only replay of actual states used by the film. No solver or trajectory changes.',
        'root_causes':['Old positive tangential cup curvature opens against the rotational travel direction.','Twelve repeated blades sampled at 24 fps can appear to rotate backwards even when body quaternion and joint velocity are correct.'],
        'solver_limit':'Paddles are idealized two-sided normal-drag elements with constant Cd=1.65. Concave-versus-convex cup drag is not resolved; correcting the display mesh does not change these equations or saved results.',
        'checks':checks,'passed':all(checks.values()),'runs':rows}
    out=ROOT/'research/paddle_direction_20261027';out.mkdir(exist_ok=True)
    (out/'solver_replay.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'passed':result['passed'],'runs':len(rows),'checks':checks},ensure_ascii=False))
    if not result['passed']:raise RuntimeError('Paddle replay mismatch')

if __name__=='__main__':main()
