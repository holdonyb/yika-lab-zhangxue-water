"""Constrained steady force/moment diagnostic, never a free-motion success test."""
from pathlib import Path
import os
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import argparse
import hashlib
import json
import time
import sys
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'physics'))
sys.path.insert(0, str(ROOT / 'research'))
import hydro_planing as h
from car_planing_matrix import candidate
from planing_surface import dynamic_normal_forces


class Bench:
    def __init__(self, geometry, speed, power_factor, phases=4):
        self.geometry, self.speed, self.power_factor = geometry, speed, power_factor
        self.p = candidate(geometry, speed, 6., power_factor, grid=40)
        self.e = h.Experiment(self.p)
        self.phases = phases
        self.calls = 0
        m = self.e.model
        self.wheels = []
        for spec in self.p['wheels']:
            j = h.mujoco.mj_name2id(m, h.mujoco.mjtObj.mjOBJ_JOINT, spec['name'] + '_spin')
            self.wheels.append(dict(spec=spec, bid=self.e.ids[spec['name']], qadr=int(m.jnt_qposadr[j]), dof=int(m.jnt_dofadr[j])))
        self.weight = self.e.mass * 9.81
        self.scale = np.array([self.weight, self.weight * 1.5, 260 * power_factor, 100.])

    def evaluate(self, x, details=False):
        self.calls += 1
        z, trim, rear_speed, front_speed = map(float, x)
        angle = np.deg2rad(trim)
        e, m, d = self.e, self.e.model, self.e.data
        forces, moments, rear_torques, front_torques, powers, valid, extra, wetareas, cps = [], [], [], [], [], [], [], [], []
        surface_rows = []
        for phase in (np.arange(self.phases) + .5) * (2 * np.pi / 12) / self.phases:
            d.qpos[:3] = [0., 0., z]
            d.qpos[3:7] = [np.cos(angle / 2), 0., -np.sin(angle / 2), 0.]
            d.qvel[:] = 0.
            d.qvel[0] = self.speed
            for wheel in self.wheels:
                d.qpos[wheel['qadr']] = phase
                d.qvel[wheel['dof']] = rear_speed if wheel['spec']['paddles'] else front_speed
            h.mujoco.mj_forward(m, d)
            out = e.apply()
            rear, front = [], []
            for wheel in self.wheels:
                bid, dof = wheel['bid'], wheel['dof']
                axis = d.xmat[bid].reshape(3, 3)[:, 1]
                hydro_torque = float(np.dot(d.xfrc_applied[bid, 3:], axis))
                net_torque = hydro_torque - .02 * d.qvel[dof]
                if wheel['spec']['paddles']:
                    aid = next(b['aid'] for b in e.driven if b['bid'] == bid)
                    rear.append(net_torque + d.ctrl[aid])
                else:
                    front.append(net_torque)
            forces.append(out['force'].copy()); moments.append(out['torque'].copy())
            rear_torques.append(np.mean(rear)); front_torques.append(np.mean(front))
            powers.append(out['motor_power']); valid.append(out['planing_valid_z']); extra.append(out['planing_extrapolated_z'])
            wetareas.append(out['wet_panel_area']); cps.append(out['planing_cp_error_m'])
        f = np.mean(forces, axis=0); moment = np.mean(moments, axis=0)
        residual = np.array([f[:, 2].sum() - self.weight, moment[:, 1].sum(), np.mean(rear_torques), np.mean(front_torques)])
        if not details:
            return residual / self.scale
        # Calm water + zero body angular velocity means panel loads are phase independent.
        for b in e.panels:
            bid, spec = b['bid'], b['spec']
            rotation = d.xmat[bid].reshape(3, 3)
            world = b['points'] @ rotation.T + d.xpos[bid]
            eta, u, _ = e.water.evaluate(world, 0.)
            wet = np.clip((eta-world[:, 2])/spec['wet_transition_m'], 0., 1.)
            vr = np.tile([self.speed, 0., 0.], (len(world), 1)) - u
            normal = rotation @ b['normal']
            top = world + spec['thickness'] * normal
            center = rotation @ np.array(spec['center']) + d.xpos[bid]
            half_height = .5 * spec['size'][0] * abs((rotation @ b['longitudinal'])[2]) + .5 * spec['size'][1] * abs((rotation @ b['lateral'])[2])
            bottom_range = [float(center[2]-half_height),float(center[2]+half_height)]
            top_range = [z + float(spec['thickness']*normal[2]) for z in bottom_range]
            fn, diag = dynamic_normal_forces(e.water.rho, spec['size'][1], b['area'], b['chord'], wet, vr, normal,
                                              rotation @ b['longitudinal'], rotation @ b['lateral'], b['grid'])
            surface_rows.append(dict(name=spec['name'], beam_m=spec['size'][1], full_length_m=spec['size'][0],
                                     bottom_z_range_m=bottom_range, top_z_range_m=top_range,
                                     top_sample_submerged_fraction=float(np.mean(top[:,2]<0)),
                                     whole_sampled_top_submerged=bool(np.all(top[:,2]<0)),
                                     whole_top_below_mean_water=bool(top_range[1]<0),
                                     lift_n=float(fn.sum()*normal[2]), **diag))
        total_f = f.sum(axis=0); total_f[2] -= self.weight
        required_tow = -float(total_f[0])
        return dict(z_m=z, trim_deg=trim, rear_wheel_rad_s=rear_speed, front_wheel_rad_s=front_speed,
                    residual=dict(vertical_n=float(residual[0]), pitch_nm=float(residual[1]), rear_per_wheel_nm=float(residual[2]), front_per_wheel_nm=float(residual[3])),
                    normalized_max_residual=float(np.max(np.abs(residual/self.scale))),
                    net_external_force_including_weight_n=total_f.tolist(), external_moment_about_whole_com_nm=moment.sum(axis=0).tolist(),
                    force_by_group_n={name:f[i].tolist() for i,name in enumerate(h.GROUPS)},
                    moment_by_group_nm={name:moment[i].tolist() for i,name in enumerate(h.GROUPS)},
                    motor_power_kw=float(np.mean(powers)/1000), installed_power_limit_kw=self.p['total_drive_power_w']/1000,
                    required_additional_horizontal_force_n=required_tow, external_tow_power_kw=required_tow*self.speed/1000,
                    paddle_thrust_n=float(f[h.GROUPS.index('paddles'),0]),
                    nonpaddle_horizontal_resistance_n=-float(sum(f[i,0] for i,name in enumerate(h.GROUPS) if name!='paddles')),
                    fraction_outside_tagged_dynamic_domain=float(np.mean(extra)/max(np.mean(extra)+np.mean(valid),1e-9)),
                    tagged_dynamic_lift_n=float(np.mean(valid)+np.mean(extra)),
                    buoyancy_weight_fraction=float(f[h.GROUPS.index('buoyancy'),2]/self.weight),
                    panel_pressure_weight_fraction=float(f[h.GROUPS.index('panel_pressure'),2]/self.weight),
                    phase_pitch_moment_peak_to_peak_nm=float(np.ptp(np.sum(moments,axis=1)[:,1])),
                    phase_rear_net_torque_peak_to_peak_nm=float(np.ptp(rear_torques)),
                    phase_samples=self.phases, surfaces=surface_rows)


def solve_case(geometry, speed, power_factor):
    started = time.perf_counter()
    bench = Bench(geometry, speed, power_factor)
    bounds = ([-.5, 2.05, 0., -10.], [1.35, 14.95, 110., 110.])
    found, attempts = [], []
    for trim in [3., 6., 10., 14.]:
        bottom = .8 if geometry == 'wide_low' else .48
        z = bottom*np.cos(np.deg2rad(trim)) + np.sin(np.deg2rad(trim))*(1.5-1.4)
        x0 = [z, trim, speed/.405*1.15, speed/.30]
        result = least_squares(bench.evaluate, x0, bounds=bounds, x_scale=[.3,6,50,40], diff_step=1e-4,
                               ftol=2e-9, xtol=2e-9, gtol=2e-9, max_nfev=100)
        error = float(np.max(np.abs(bench.evaluate(result.x))))
        attempts.append(dict(start=x0, final=result.x.tolist(), normalized_max_residual=error, nfev=result.nfev, status=int(result.status)))
        print(json.dumps(dict(progress=[geometry,speed,power_factor,trim], residual=error, calls=bench.calls)), flush=True)
        if error < 2e-4 and not any(np.linalg.norm((result.x-x)/np.array([.1,2,10,10])) < .05 for x in found):
            found.append(result.x)
    solutions=[]
    bench.phases=16
    for x in found:
        result=least_squares(bench.evaluate,x,bounds=bounds,x_scale=[.3,6,50,40],diff_step=1e-4,
                             ftol=2e-10,xtol=2e-10,gtol=2e-10,max_nfev=80)
        row=bench.evaluate(result.x,details=True)
        row['accepted_constrained_equilibrium']=row['normalized_max_residual']<2e-4
        solutions.append(row)
    best=min(attempts,key=lambda a:a['normalized_max_residual'])
    return dict(geometry=geometry,speed_m_s=speed,power_factor=power_factor,mass_kg=bench.e.mass,grid=[40,6],wet_transition_m=.012,
                search_bounds=dict(z_m=[-.5,1.35],trim_deg=[2.05,14.95],rear_rad_s=[0,110],front_rad_s=[-10,110]),
                assumptions='Calm water; zero heave/pitch/roll/yaw velocities; horizontal speed constrained. Symmetric wheels, four phase search refined to 16 phases over one paddle pitch. Rear wheels solve actual controller plus hydro torque and bearing damping; front wheels solve hydro torque and damping. Engine and hinge torques are internal to the complete assembly. Constraint tow acts at whole COM and is reported, not applied. Fixed mass across power ratings.',
                solutions=solutions,attempts=attempts,best_unaccepted=bench.evaluate(best['final'],details=True) if not solutions else None,
                calls=bench.calls,elapsed_s=time.perf_counter()-started)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--speeds',nargs='+',type=float,default=[15.]);args=ap.parse_args()
    dest=ROOT/'research/car_planing_equilibrium.json'
    report=dict(scope='Constrained quasi-steady vertical/pitch/rotor balance search, never free-motion success; no solver or trajectory edits', cases=[],
                source_hashes={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in [ROOT/'research/car_planing_matrix.py',ROOT/'physics/hydro_planing.py',ROOT/'physics/planing_surface.py']})
    if dest.exists():
        old=json.loads(dest.read_text())
        if old['source_hashes']!=report['source_hashes']:raise RuntimeError('Source fingerprints changed; use a new result destination')
        report=old
    for speed in args.speeds:
        for geometry in ['paddles','wide_low']:
            for factor in [1.,4.]:
                if any(c['geometry']==geometry and c['speed_m_s']==speed and c['power_factor']==factor for c in report['cases']):continue
                row=solve_case(geometry,speed,factor);report['cases'].append(row)
                dest.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
                print(json.dumps(dict(completed=[geometry,speed,factor],solutions=len(row['solutions']),elapsed_s=row['elapsed_s'])),flush=True)


if __name__=='__main__':main()
