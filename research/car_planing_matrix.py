"""Registered, resumable free-6DOF car experiments; never overwrites old runs."""
from pathlib import Path
from copy import deepcopy
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse, hashlib, json, os, sys
os.environ['OMP_NUM_THREADS']='1'
os.environ['OPENBLAS_NUM_THREADS']='1'
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'physics'))
import numpy as np
import hydro_planing as h

def read(path):return json.loads(path.read_text(encoding='utf8'))
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def candidate(geometry='paddles',speed=15.,trim=3.,power_factor=1.,duration=10.,grid=40):
    p=h.parameters('car','paddles')
    p.update(variant='planing_'+geometry,initial_speed=float(speed),initial_pitch_deg=float(trim),duration=duration,
             drive_control='positive_power_envelope',wheel_target_rad_s=100.,drive_torque_nm=260.*power_factor,
             drive_power_w=22000.*power_factor,total_drive_power_w=44000.*power_factor)
    p['power_sensitivity_note']='Power/torque factor is an isolated rating sensitivity at fixed mass, not an engineered engine upgrade.'
    if geometry=='four_skis':
        size=np.array([1.4,.65,.04]);mount=6.;rotation=h.ry(-np.deg2rad(mount))
        mass=float(4*2*(size[0]*size[1]+size[0]*size[2]+size[1]*size[2])*.0015*2700.+12.)
        centers=[]
        for x in [-1.3,1.3]:
            for side in [-1,1]:
                bottom=np.array([x,side*1.30,-.80]);center=bottom+rotation[:,2]*size[2]/2;centers.append(center)
                p['volumes'].append(h.box(f'ski_solid_{x}_{side}',center.tolist(),size.tolist(),cd=(.18,.85,0.),grid=(14,5,2),pitch_deg=mount))
                p['panels'].append(h.panel(f'ski_bottom_{x}_{side}',bottom.tolist(),size[:2].tolist(),pitch_deg=mount,grid=(grid,6),cf=.006,thickness=float(size[2])))
        old_mass=p['chassis_mass'];old_com=np.array(p['chassis_com']);new_mass=old_mass+mass
        com=(old_mass*old_com+mass*np.mean(centers,axis=0))/new_mass
        def parallel(m,r):return m*(np.eye(3)*np.dot(r,r)-np.outer(r,r))
        inertia=np.diag(p['chassis_inertia'])+parallel(old_mass,old_com-com)
        sx,sy,sz=size;own=rotation@np.diag([sy*sy+sz*sz,sx*sx+sz*sz,sx*sx+sy*sy])@rotation.T*(mass/4)/12
        for center in centers:inertia+=own+parallel(mass/4,center-com)
        p.update(chassis_mass=new_mass,chassis_com=com.tolist(),chassis_inertia=np.diag(inertia).tolist(),float_mass_kg=mass)
        p['ski_mass_estimate']={'mass_kg':mass,'aluminium_density_kg_m3':2700.,'shell_thickness_m':.0015,'brackets_kg':12.,'calculated_inertia_tensor_kg_m2':inertia.tolist(),'model_note':'Diagonal inertia approximation; off-diagonal terms recorded. Separated fore/aft boards have unmodelled wake interaction.'}
    elif geometry!='paddles':
        if geometry in ['wide_low','wide_high','wide_low_clear','wide_high_clear']:size=[3.,.75,.10]
        elif geometry=='narrow_low':size=[3.,.25,.30]
        else:raise ValueError(geometry)
        bottom=-.80 if '_low' in geometry else -.55
        z=bottom+size[2]/2;y=1.35 if geometry.endswith('_clear') else 1.30
        # Declared shell estimate, including eight kilograms of brackets.
        area=4*(size[0]*size[1]+size[0]*size[2]+size[1]*size[2])
        mass=area*.0015*2700.+8.
        h.add_symmetric_float_mass(p,mass,size,y,z)
        p['float_mass_estimate']={'material_density_kg_m3':2700.,'shell_thickness_m':.0015,'brackets_kg':8.,'total_kg':mass,'note':'Mass assumption only; no structural certification.'}
        for side in [-1,1]:
            p['volumes'].append(h.box(f'side_float_{side}',[0,side*y,z],size,cd=(.18,.85,0.),grid=(20,5,4)))
            p['panels'].append(h.panel(f'float_bottom_{side}',[0,side*y,bottom],size[:2],pitch_deg=0,grid=(grid,6),cf=.006,thickness=size[2]))
    for panel in p['panels']:
        panel['grid']=[grid,6]
        # Slender floats remain explicitly outside the selected closure domain.
        if panel['size'][0]/panel['size'][1]<=4:panel['pressure_model']='savitsky_dynamic'
        else:panel['pressure_model']='original_local_pressure_unvalidated_for_planing'
    # Same 2 cm clearance of the lowest physical component in each entry test.
    # Geometry and pitch set the initial height; no constraint persists thereafter.
    rot=h.ry(-np.deg2rad(trim));lowest=0.
    for panel in p['panels']:
        for x in [-panel['size'][0]/2,panel['size'][0]/2]:
            point=np.array(panel['center'])+h.ry(-np.deg2rad(panel['pitch_deg']))@np.array([x,0,0])
            lowest=min(lowest,float((rot@point)[2]))
    for wheel in p['wheels']:
        lowest=min(lowest,float((rot@np.array(wheel['center']))[2])-max(wheel['radius'],wheel.get('blade_radius',0) if wheel['paddles'] else 0))
    p['initial_z']=.02-lowest
    p['initial_clearance_note']='2 cm clearance above calm water for the lowest board or wheel/blade envelope; free entry thereafter.'
    return p

def entries(stage):
    cases=[]
    def add(name,p):cases.append((name,p))
    if stage=='coarse':
        for geom in ['paddles','wide_low']:
            for speed,trim in [(5,3),(10,3),(15,0),(15,3),(15,6),(20,3)]:
                add(f'pln_{geom}_v{speed}_t{trim}_p44',candidate(geom,speed,trim))
    elif stage=='power_geometry':
        for geom in ['wide_low','wide_high','narrow_low']:
            for factor in [1,2,4]:
                add(f'pln_{geom}_v15_t3_p{44*factor}',candidate(geom,15,3,factor))
    elif stage=='four_skis':
        for speed,factor in [(15,1),(15,2),(15,4),(10,1),(10,4),(20,1),(20,4)]:
            add(f'pln_four_skis_v{speed}_t0_p{44*factor}',candidate('four_skis',speed,0,factor))
        for factor in [1,4]:
            p=candidate('four_skis',15,0,factor)
            for panel in p['panels']:panel['pressure_model']='original_local_pressure_unvalidated_for_planing'
            add(f'pln_four_skis_v15_t0_p{44*factor}_local',p)
    elif stage=='confirmation':
        add('pln_four_skis_v15_p176_30s',candidate('four_skis',15,0,4,duration=30.))
        p=candidate('four_skis',15,0,4);p['dt']=.0005
        add('pln_four_skis_v15_p176_dt05',p)
        add('pln_four_skis_v15_p176_grid80',candidate('four_skis',15,0,4,grid=80))
        p=candidate('four_skis',15,0,4,grid=80)
        for panel in p['panels']:panel['wet_transition_m']=.006
        add('pln_four_skis_v15_p176_grid80_wet6',p)
    elif stage=='stress':
        add('pln_four_skis_v10_p176_30s',candidate('four_skis',10,0,4,duration=30.))
        p=candidate('four_skis',15,0,4,duration=30.);p['initial_roll_deg']=1.
        add('pln_four_skis_v15_p176_roll1_30s',p)
        p=candidate('four_skis',15,0,4,duration=25.);p['drive_off_s']=12.
        add('pln_four_skis_v15_p176_off12',p)
        p=candidate('wide_low',15,3,1,grid=80);p['dt']=.0005
        add('pln_wide_low_v15_t3_p44_refined',p)
        for water in ['head','cross','swell','irregular']:
            p=candidate('four_skis',15,0,4,duration=12.);p['water']=h.wave_spec(water)
            p['wave_interpretation']='Quasi-steady prescribed incident waves only; impact, wake, added mass and free-surface feedback are not resolved. Exploratory stress test.'
            add(f'pln_four_skis_v15_p176_{water}',p)
    elif stage=='clearance':
        for factor in [1,4]:
            add(f'pln_wide_low_clear_v15_t3_p{44*factor}',candidate('wide_low_clear',15,3,factor))
    elif stage=='wave_check':
        p=candidate('four_skis',15,0,4,duration=12.);p['water']=h.wave_spec('head');p['dt']=.0005
        p['wave_interpretation']='Half-step comparison prompted by the head-wave energy residual; quasi-steady prescribed incident waves remain exploratory.'
        add('pln_four_skis_v15_p176_head_dt05',p)
    else:raise ValueError(stage)
    return cases

def job(entry):
    name,p=entry;dest=ROOT/'physics/runs'/name
    if (dest/'metadata.json').exists():
        m=read(dest/'metadata.json');assert read(dest/'params.json')==p
        assert m['source_sha256']==digest(ROOT/'physics/hydro_planing.py')
        assert m['planing_surface_source_sha256']==digest(ROOT/'physics/planing_surface.py')
    else:h.run(p,dest)
    return name

def summarize(name):
    path=ROOT/'physics/runs'/name;p=read(path/'params.json');m=read(path/'metadata.json');a=np.load(path/'trajectory.npz')
    gaps=[]
    for volume in p['volumes']:
        if not volume['name'].startswith(('side_float_','ski_solid_')):continue
        for wheel in p['wheels']:
            if not wheel['paddles'] or volume['center'][1]*wheel['center'][1]<0:continue
            if abs(volume['center'][0]-wheel['center'][0])>volume['size'][0]/2+wheel['blade_radius']:continue
            gaps.append(abs(volume['center'][1])-volume['size'][1]/2-abs(wheel['center'][1])-wheel.get('blade_width',wheel['width'])/2)
    clearance=min(gaps) if gaps else None
    t=a['t'];tail=t>=t[-1]-min(5.,t[-1]/2);early=(t>=t[-1]-10)&(t<t[-1]-5);late=t>=t[-1]-5
    # Forward velocity in body heading, excluding sink rate; calm water in coarse runs.
    yaw=np.deg2rad(a['yaw_deg']);current=np.array(p['water']['current'])
    forward=(a['qvel'][:,0]-current[0])*np.cos(yaw)+(a['qvel'][:,1]-current[1])*np.sin(yaw)
    groups=m['force_groups'];f=a['force'];weight=m['total_mass_kg']*9.81
    support={g:float(f[tail,groups.index(g),2].mean()/weight) for g in groups}
    drift=abs(float(forward[late].mean()-forward[early].mean()))/max(abs(float(forward[early].mean())),.1) if early.any() else None
    valid=float(a['planing_valid_z'][tail].sum());extra=float(a['planing_extrapolated_z'][tail].sum())
    within=valid/max(valid+extra,1e-9)
    bounded_angles=bool(early.any() and all(np.ptp(a[key][late])<=max(1.2*np.ptp(a[key][early]),.5) for key in ['pitch_deg','roll_deg']))
    stable=(t[-1]>=29.9 and drift is not None and drift<.05 and abs(float(a['qvel'][tail,2].mean()))<.02 and np.ptp(a['qpos'][tail,2])<.1 and np.max(np.abs(a['roll_deg'][tail]))<15 and np.max(np.abs(a['pitch_deg'][tail]))<15 and abs(sum(support.values())-1)<.1 and bounded_angles)
    if a['qpos'][-1,2]<-1.5:outcome='submerged'
    elif np.max(np.abs(a['roll_deg'][tail]))>30 or np.max(np.abs(a['pitch_deg'][tail]))>30:outcome='unstable_attitude'
    elif stable and support['panel_pressure']>.5 and within>.9:outcome='sustained_planing_in_declared_model'
    elif stable and support['buoyancy']>.5:outcome='displacement_or_mixed_floating'
    else:outcome='transient_or_unconfirmed'
    return {'name':name,'geometry':p['variant'],'minimum_lateral_board_blade_envelope_gap_m':clearance,'mechanical_envelope_overlap':bool(clearance is not None and clearance<0),'initial_speed_m_s':p['initial_speed'],'initial_trim_deg':p['initial_pitch_deg'],'total_power_kw':p['total_drive_power_w']/1000,
        'duration_s':float(t[-1]),'mass_kg':m['total_mass_kg'],'end_forward_speed_m_s':float(forward[-1]),'tail_mean_forward_speed_m_s':float(forward[tail].mean()),
        'tail_speed_relative_drift':drift,'tail_angle_amplitudes_not_growing':bounded_angles,'final_z_m':float(a['qpos'][-1,2]),'tail_z_range_m':float(np.ptp(a['qpos'][tail,2])),
        'tail_mean_vertical_speed_m_s':float(a['qvel'][tail,2].mean()),'tail_mean_pitch_deg':float(a['pitch_deg'][tail].mean()),'tail_pitch_range_deg':float(np.ptp(a['pitch_deg'][tail])),
        'tail_mean_roll_deg':float(a['roll_deg'][tail].mean()),'tail_support_weight_ratios':support,'tail_valid_fraction_of_tagged_dynamic_lift':within,
        'tail_mean_motor_power_kw':float(a['motor_power'][tail].mean()/1000),'tail_mean_paddle_x_n':float(f[tail,groups.index('paddles'),0].mean()),
        'tail_mean_resistance_x_n':{g:float(f[tail,groups.index(g),0].mean()) for g in groups if g!='paddles'},
        'energy_balance_relative_error':m['energy_balance_relative_error'],'max_dissipative_relative_power_w':m['max_dissipative_relative_power_w'],
        'finite_states':bool(np.isfinite(a['qpos']).all() and np.isfinite(a['qvel']).all()),'trajectory_sha256':m['trajectory_sha256'],'outcome':outcome}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--stage',default='coarse');ap.add_argument('--workers',type=int,default=4);ap.add_argument('--summarize-only',action='store_true');args=ap.parse_args()
    cases=entries(args.stage)
    manifest={'stage':args.stage,'note':'Registered before execution; all outcomes retained. Coarse 10 s cases are not sufficient to claim sustained planing.','cases':[{'name':n,'parameters':p} for n,p in cases],
              'source_hashes':{n:digest(ROOT/'physics'/n) for n in ['hydro.py','hydro_planing.py','planing_surface.py']}}
    dest=ROOT/f'research/car_planing_{args.stage}_manifest.json'
    if dest.exists():assert read(dest)==manifest
    else:dest.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
    if not args.summarize_only:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            for future in as_completed([pool.submit(job,c) for c in cases]):print('Completed '+future.result(),flush=True)
    report={'stage':args.stage,'scope':'Uncalibrated hybrid model: dynamic planing correlation + geometric static buoyancy. Domain load fractions reported separately.','cases':[summarize(n) for n,p in cases]}
    (ROOT/f'research/car_planing_{args.stage}_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    for row in report['cases']:print(json.dumps(row,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
