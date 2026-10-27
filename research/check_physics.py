"""Independent short diagnostics. Reads physics/hydro.py; writes only research output."""
from pathlib import Path
import importlib.util
import hashlib
import json
import sys
import numpy as np
import mujoco
from exact_box_clip import polygon_wet_rectangle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'physics'))
source = ROOT / 'physics' / 'hydro.py'
spec = importlib.util.spec_from_file_location('reviewed_hydro', source)
h = importlib.util.module_from_spec(spec)
spec.loader.exec_module(h)

def minimal(mass=1000):
    p = h.parameters('bike')
    p.update(chassis_mass=mass, chassis_inertia=[200., 200., 200.], wheels=[], volumes=[], panels=[], initial_speed=0., initial_z=0., initial_roll_deg=0., initial_pitch_deg=0., drive_off_s=0.)
    p.pop('front_fork', None)
    return p

def group(e, name):
    return e.last['force'][h.GROUPS.index(name)]

results = {'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'clipped_box_sha256':hashlib.sha256((ROOT/'physics'/'clipped_box.py').read_bytes()).hexdigest(), 'mujoco_version': mujoco.__version__, 'checks': {}}

# Force application and point speed, with both an offset COM and an articulated wheel.
p = h.parameters('bike')
e = h.Experiment(p)
e.data.qvel[:] = np.linspace(-.7, .8, e.model.nv)
mujoco.mj_forward(e.model, e.data)
mapping = []
for bid in range(1, e.model.nbody):
    pt = e.data.xpos[bid] + np.array([.13, -.21, .16])
    vel = np.zeros(6)
    mujoco.mj_objectVelocity(e.model, e.data, mujoco.mjtObj.mjOBJ_BODY, bid, vel, 0)
    calc = vel[3:] + np.cross(vel[:3], pt - e.data.xipos[bid])
    jp, jr = np.zeros((3, e.model.nv)), np.zeros((3, e.model.nv))
    mujoco.mj_jac(e.model, e.data, jp, jr, pt, bid)
    force = np.array([13., -22., 47.])
    qforce = np.zeros(e.model.nv)
    mujoco.mj_applyFT(e.model, e.data, force, np.zeros(3), pt, bid, qforce)
    mapping.append({'body': e.names[bid-1], 'velocity_abs_error': float(np.max(np.abs(calc-jp@e.data.qvel))), 'power_abs_error': float(abs(force@calc-qforce@e.data.qvel))})
results['checks']['point_velocity_and_power_mapping'] = mapping
e.apply()
first = e.data.xfrc_applied.copy()
e.apply()
results['checks']['repeated_force_abs_error'] = float(np.max(np.abs(first-e.data.xfrc_applied)))

# Exact horizontal box displacement and zero torque around its symmetric COM.
buoy = []
for draft in [0., .25, .5, 1.]:
    p = minimal()
    p['initial_z'] = .5-draft
    p['volumes'] = [h.box('reference_box', [0, 0, 0], [1, 1, 1], grid=(8,8,8))]
    e = h.Experiment(p); e.apply()
    buoy.append({'draft_m': draft, 'Fz_N': float(group(e, 'buoyancy')[2]), 'expected_N': 1000*9.81*draft, 'torque_Nm': e.last['torque'][h.GROUPS.index('buoyancy')].tolist()})
results['checks']['horizontal_box'] = buoy

# Restore torque for a broad rectangular barge, known positive GM.
restore = []
mass, length, width, height, draft = 1000., 2., 2., 1., .25
GM = draft/2 + length*width**3/(12*(mass/1000)) - height/2
for grid_scale in [1,2,4]:
    for angle in [1.,3.,5.,15.]:
        p = minimal(mass)
        p.update(initial_z=height/2-draft, initial_roll_deg=angle)
        p['volumes'] = [h.box('reference_barge', [0,0,0], [length,width,height], grid=(8,8,4))]
        e = h.Experiment(p, grid_scale=grid_scale); e.apply()
        actual = float(e.last['torque'][h.GROUPS.index('buoyancy'),0])
        linear = -mass*9.81*GM*np.deg2rad(angle)
        area, centroid = polygon_wet_rectangle(width,height,np.deg2rad(angle),height/2-draft)
        exact_moment = 1000*9.81*length*area*centroid[0]
        restore.append({'grid_scale': grid_scale, 'roll_deg': angle, 'Mx_Nm': actual, 'linear_reference_Nm': linear, 'exact_polygon_reference_Nm': exact_moment, 'relative_to_exact_polygon':abs((actual-exact_moment)/exact_moment), 'relative_to_small_angle_reference': abs((actual-linear)/linear)})
results['checks']['barge_restore'] = restore

# Current single-surface pressure test: panel's lower face is wet, upper face would be dry.
p = minimal(100)
p.update(initial_z=-.006)
p['panels'] = [h.panel('thin_board', [0,0,0], [1,1], pitch_deg=0, grid=(4,4), cn=2.)]
e = h.Experiment(p)
e.data.qvel[2] = 1.
mujoco.mj_forward(e.model,e.data);e.apply()
results['checks']['bottom_wet_top_dry_upward_motion'] = {'assumed_thickness_m': .028, 'bottom_z_m': -.006, 'top_z_m': .022, 'velocity_z_m_s': 1., 'normal_force_z_N': float(group(e, 'panel_pressure')[2]), 'expected_for_one_sided_no_suction_model_N': 0.}

# For a rotating axisymmetric tire, submerged volume should be phase invariant.
p = h.parameters('bike');p.update(initial_speed=0., initial_z=.455)
p['volumes'] = [v for v in p['volumes'] if v['name'] in ['rear_tire_sealed','rear_wheel_sealed']];p['panels']=[]
e = h.Experiment(p)
rear = e.driven[0]
phase = []
for a in [0.,15.,30.,45.,60.,75.,90.]:
    e.data.qpos[rear['qadr']] = np.deg2rad(a)
    e.data.qpos[2] = .455 + .09
    mujoco.mj_forward(e.model,e.data);e.apply()
    phase.append({'spin_deg':a, 'wet_volume_m3':e.last['wet_volume'], 'Fz_N':float(group(e,'buoyancy')[2])})
results['checks']['tire_rotation_displacement'] = phase

# Airy field temporal derivatives at fixed world positions; below mean water level.
w = h.Water(h.wave_spec('head'))
pts = np.array([[.3,.2,-.4],[.8,-.1,0.],[-.3,.1,-.2]])
t,eps = .7,1e-5
eta,u,a = w.evaluate(pts,t)
ep,up,_ = w.evaluate(pts,t+eps)
em,um,_ = w.evaluate(pts,t-eps)
results['checks']['airy_temporal_derivative'] = {'acceleration_abs_error_m_s2':float(np.max(np.abs((up-um)/(2*eps)-a))), 'kinematic_surface_abs_error_m_s':float(abs((ep[1]-em[1])/(2*eps)-u[1,2]))}

# Passive normal/skin/drag terms should remain dissipative at arbitrary finite states.
p = h.parameters('bike', water='cross');e=h.Experiment(p)
rng=np.random.default_rng(20261026);peak=-float('inf')
for j in range(20):
    e.data.qpos[2]=rng.uniform(-.5,1.3)
    e.data.qvel[:]=rng.normal(0,3,e.model.nv)
    q=rng.normal(size=4);e.data.qpos[3:7]=q/np.linalg.norm(q)
    mujoco.mj_forward(e.model,e.data);e.apply()
    ids=[h.GROUPS.index(s) for s in ['volume_drag','panel_pressure','skin_drag','paddles','air_drag']]
    peak=max(peak,float(np.max(e.last['relative_power'][ids])))
results['checks']['random_state_max_passive_relative_power_W']=peak

# Independent single-panel force law and U^2 / area scaling.
plate=[]
for speed,area in [(4.,1.),(8.,1.),(8.,.5)]:
    p=minimal(1000);p.update(initial_speed=speed,initial_z=-1.)
    p['panels']=[h.panel('reference_plate',[0,0,0],[1.,area],pitch_deg=6.,grid=(4,4),cn=2.,cf=0.)]
    e=h.Experiment(p);e.apply();F=group(e,'panel_pressure')
    a=np.deg2rad(6.);lift=.5*1000*2*area*speed**2*np.sin(a)*np.cos(a)
    plate.append(dict(speed_m_s=speed,area_m2=area,Fx_N=float(F[0]),Fz_N=float(F[2]),expected_lift_N=float(lift),normal_drag_lift=float(-F[0]/F[2]),expected_normal_drag_lift=float(np.tan(a))))
results['checks']['single_plate']=plate

# Internal drive torque with no external load must conserve whole-system angular momentum.
p=h.parameters('bike');p.update(initial_speed=0.);e=h.Experiment(p)
e.model.opt.gravity[:]=0.;e.model.dof_damping[:]=0.
drive=e.driven[0]['aid']
for _ in range(1000):
    mujoco.mj_step1(e.model,e.data)
    e.data.xfrc_applied[:]=0.;e.data.qfrc_applied[:]=0.;e.data.ctrl[:]=0.;e.data.ctrl[drive]=1.
    mujoco.mj_step2(e.model,e.data)
mujoco.mj_forward(e.model,e.data);mujoco.mj_subtreeVel(e.model,e.data)
H=e.data.subtree_angmom[e.ids['chassis']]
results['checks']['internal_rotor_torque']=dict(duration_s=float(e.data.time),torque_Nm=1.,whole_system_angular_momentum_kg_m2_s=H.tolist(),momentum_norm=float(np.linalg.norm(H)),root_angular_velocity_rad_s=e.data.qvel[3:6].tolist(),wheel_relative_velocity_rad_s=float(e.data.qvel[e.driven[0]['dof']]))

c=results['checks']
tire_f=np.array([x['Fz_N'] for x in c['tire_rotation_displacement']])
summary={
    'point_velocity_and_power_mapping':max(max(x['velocity_abs_error'],x['power_abs_error']) for x in c['point_velocity_and_power_mapping'])<1e-9,
    'force_evaluation_not_accumulated':c['repeated_force_abs_error']<1e-12,
    'archimedes_horizontal_box':max(abs(x['Fz_N']-x['expected_N']) for x in c['horizontal_box'])<1e-6,
    'restore_torque_exact_polygon':max(x['relative_to_exact_polygon'] for x in c['barge_restore'])<1e-9,
    'dry_top_no_downward_pressure':abs(c['bottom_wet_top_dry_upward_motion']['normal_force_z_N'])<1e-9,
    'tire_phase_variation_below_0_1_percent':float(np.ptp(tire_f)/np.mean(tire_f))<.001,
    'airy_time_derivative':max(c['airy_temporal_derivative'].values())<1e-5,
    'passive_relative_power':c['random_state_max_passive_relative_power_W']<=1e-9,
    'plate_force_analytic':max(abs(x['Fz_N']/x['expected_lift_N']-1) for x in c['single_plate'])<1e-9,
    'internal_rotor_angular_momentum':c['internal_rotor_torque']['momentum_norm']<1e-4,
}
summary={k:bool(v) for k,v in summary.items()}
results['short_benchmark_status']=summary
results['all_short_benchmarks_passed']=all(summary.values())
results['scope']='Short implementation diagnostics only; full scenario stability, time-step convergence and wave-space convergence are separate.'

(ROOT/'research'/'physics_checks.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(results,ensure_ascii=False,indent=2))
