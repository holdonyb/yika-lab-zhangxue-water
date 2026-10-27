"""Registered low-speed entry supplement; unchanged solver and no imposed speed."""
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse
import json
import numpy as np
from car_planing_matrix import ROOT, candidate, job, summarize, digest, read


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', choices=['low_speed_entry', 'matched_power_entry', 'entry_halfstep'], default='low_speed_entry')
    args = parser.parse_args()
    stage = args.stage
    cases = []
    geometries = ['paddles', 'four_skis'] if stage in ['low_speed_entry', 'entry_halfstep'] else ['paddles']
    speeds = [0, 5, 8] if stage == 'low_speed_entry' else [10, 15, 20]
    for geometry in geometries:
        entry_speeds = [15 if geometry == 'paddles' else 8] if stage == 'entry_halfstep' else speeds
        for speed in entry_speeds:
            name = f'lowentry_20261027_{geometry}_v{speed}_p176'
            p = candidate(geometry, speed, 0, 4, duration=10.)
            if stage == 'entry_halfstep':
                name += '_dt05'
                p['dt'] = .0005
            cases.append((name, p))
    manifest = {
        'question': 'Can the same wheel-paddle drive start or reach planing from low entry speed?',
        'scope': 'Free 6DOF entry from the same 2 cm geometric clearance. No road/ramp support or speed constraint; 176 kW rating sensitivity at fixed mass. Not an amphibious shore-launch test.',
        'criteria': 'Report every result; ten seconds cannot establish sustained planing. Compare both geometries at identical initial speed and power/torque budgets. Retain actual power, sinking, attitude, closure-domain and energy diagnostics.',
        'source_hashes': {n: digest(ROOT/'physics'/n) for n in ['hydro.py', 'hydro_planing.py', 'planing_surface.py', 'clipped_box.py']},
        'cases': [{'name': n, 'parameters': p} for n, p in cases],
    }
    path = ROOT/f'research/{stage}_manifest.json'
    if path.exists():
        assert read(path) == manifest
    else:
        path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf8')
    with ProcessPoolExecutor(max_workers=2) as pool:
        for future in as_completed([pool.submit(job, c) for c in cases]):
            print('Completed ' + future.result(), flush=True)
    results = []
    for name, p in cases:
        row = summarize(name)
        folder = ROOT/'physics/runs'/name
        a = np.load(folder/'trajectory.npz')
        m = read(folder/'metadata.json')
        row.update(peak_forward_speed_m_s=float(a['qvel'][:, 0].max()),
                   max_abs_roll_deg=float(abs(a['roll_deg']).max()),
                   max_abs_pitch_deg=float(abs(a['pitch_deg']).max()),
                   energy_balance_residual_j=m['energy_balance_residual_j'],
                   stop=m['stop'])
        assert row['finite_states']
        assert m['trajectory_sha256'] == digest(folder/'trajectory.npz')
        assert m['source_sha256'] == manifest['source_hashes']['hydro_planing.py']
        assert np.isfinite(a['force']).all()
        assert a['motor_power'].max() <= p['total_drive_power_w'] + 1e-6
        row['peak_motor_power_kw'] = float(a['motor_power'].max()/1000)
        results.append(row)
    report = {'manifest_sha256': digest(path), 'scope': manifest['scope'], 'cases': results}
    (ROOT/f'research/{stage}_results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    lines = ['# 低速入水补测', '', manifest['scope'], '',
             '采用既有求解器；相同初始离水间隙、轮端功率及扭矩预算，自由六自由度运动。0 m/s 表示从水面上方释放且持续驱动，不含岸坡或道路支撑。十秒结果用于检查起滑过程，不作为长期稳定证明。', '',
             '| 方案 | 初速 m/s | 实际运行 s | 末段均速 m/s | 最大纵倾 ° | 末段实际功率 kW | 结果 |',
             '|---|---:|---:|---:|---:|---:|---|']
    for r in results:
        lines.append(f"| {r['geometry']} | {r['initial_speed_m_s']:g} | {r['duration_s']:.2f} | {r['tail_mean_forward_speed_m_s']:.2f} | {r['max_abs_pitch_deg']:.1f} | {r['tail_mean_motor_power_kw']:.1f} | {r['outcome']} |")
    lines += ['', '参数、源码指纹与全部失败结果见 low_speed_entry_manifest.json、low_speed_entry_results.json；每组轨迹保留在 physics/runs/lowentry_20261027_*。模型未校准实车，杯口两侧阻力差及滑板尾流没有求解。']
    (ROOT/f'research/{stage.upper()}_RESULTS.md').write_text('\n'.join(lines)+'\n', encoding='utf8')
    print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
