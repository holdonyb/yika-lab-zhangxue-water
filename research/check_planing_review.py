"""Focused independent review probes; does not run or modify vehicle trajectories."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'physics'))
import hydro_planing as h
from planing_surface import dynamic_normal_forces


def domain_probes():
    count = 100
    chord = np.repeat((np.arange(count) + .5) * 3.3 / count, 2)
    angle = np.deg2rad(6.)
    normal = np.array([-np.sin(angle), 0., np.cos(angle)])
    longitudinal = np.array([np.cos(angle), 0., np.sin(angle)])
    lateral = np.array([0., 1., 0.])
    cases = [
        ('forward', [15., 0., 0.], np.ones(2 * count), True),
        ('crossflow_descent', [1., 15., -1.58], np.ones(2 * count), False),
        ('reverse_descent', [-15., 0., -3.15], np.ones(2 * count), False),
        ('disconnected_wet', [15., 0., 0.], ((chord < .8) | (chord > 2.5)).astype(float), False),
        ('half_width_wet', [15., 0., 0.], np.tile([1., 0.], count), False),
        ('horizontal_uniform_descent', [15., 0., -1.58], np.ones(2 * count), False),
    ]
    rows = []
    for name, velocity, wet, expected_valid in cases:
        vr = np.tile(velocity, (2 * count, 1))
        case_normal = np.array([0., 0., 1.]) if name == 'horizontal_uniform_descent' else normal
        case_longitudinal = np.array([1., 0., 0.]) if name == 'horizontal_uniform_descent' else longitudinal
        fn, diag = dynamic_normal_forces(1000., 1.4, 3.3 * 1.4 / (2 * count), chord, wet, vr, case_normal, case_longitudinal, lateral, (count, 2))
        relative_power = float(np.sum(fn * (vr @ case_normal)))
        valid = diag['valid_force_z'] > 0 and diag['extrapolated_force_z'] < 1e-9
        rows.append(dict(name=name, expected_valid=expected_valid, diagnostic=diag, relative_power_w=relative_power,
                         passed=bool(valid == expected_valid and relative_power <= 0)))
    return rows


def production_probe(trim, lam, grid):
    beam, length, speed = 1.4, 3.3, 15.
    angle = np.deg2rad(trim)
    p = h.parameters('car', 'no_mod')
    p.update(initial_speed=speed, initial_pitch_deg=trim,
             initial_z=.48 * np.cos(angle) + np.sin(angle) * (length / 2 - lam * beam))
    p['volumes'] = []
    p['panels'][0].update(grid=[grid, 2], wet_transition_m=.012, pressure_model='savitsky_dynamic')
    e = h.Experiment(p)
    out = e.apply()
    i = h.GROUPS.index('panel_pressure')
    force, torque = out['force'][i], out['torque'][i]
    bid = e.ids['chassis']
    rotation = e.data.xmat[bid].reshape(3, 3)
    fb, tb = rotation.T @ force, rotation.T @ torque
    com = rotation.T @ (e.data.subtree_com[bid] - e.data.xpos[bid])
    cp = float(-tb[1] / fb[2] + com[0] + length / 2)
    wet_length = float(out['wet_panel_area'] / beam)
    reference_lift = .5 * 1000 * speed**2 * beam**2 * .012 * lam**.5 * trim**1.1
    reference_cp = .75 * lam * beam
    return dict(trim_deg=trim, lambda_requested=lam, grid=grid, wet_transition_m=.012,
                actual_wet_length_m=wet_length, geometric_wet_length_m=lam * beam,
                lift_n=float(force[2]), lift_reference_n=reference_lift,
                lift_relative_error=float(force[2] / reference_lift - 1),
                cp_m=cp, cp_geometric_reference_m=reference_cp,
                cp_error_geometric_m=cp-reference_cp, cp_error_effective_wet_length_m=cp-.75*wet_length,
                valid_force_z=float(out['planing_valid_z']), extrapolated_force_z=float(out['planing_extrapolated_z']))


if __name__ == '__main__':
    domain = domain_probes()
    production = [production_probe(trim, lam, grid) for trim, lam in [(3., .75), (6., 1.5), (9., 3.3 / 1.4)] for grid in [20, 40, 80]]
    report = dict(scope='Domain guards and production wet-transition/grid probes; no trajectory or empirical validation',
                  domain_passed=all(r['passed'] for r in domain), domain_probes=domain, production_grid_probes=production,
                  source_hashes={name: hashlib.sha256((ROOT / 'physics' / name).read_bytes()).hexdigest() for name in ['hydro.py', 'hydro_planing.py', 'planing_surface.py']})
    (ROOT / 'research/planing_review_checks.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(report, indent=2))
