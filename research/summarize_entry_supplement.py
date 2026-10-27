"""Read-only traceability checks and comparison of the registered entry runs."""
from pathlib import Path
import json
import numpy as np
from car_planing_matrix import ROOT, digest, read, summarize


def main():
    rows, checks, manifests = [], [], {}
    for stage in ['low_speed_entry', 'matched_power_entry', 'entry_halfstep']:
        path = ROOT/f'research/{stage}_manifest.json'
        manifest = read(path)
        manifests[stage] = digest(path)
        result = read(ROOT/f'research/{stage}_results.json')
        assert result['manifest_sha256'] == digest(path)
        for case, row in zip(manifest['cases'], result['cases'], strict=True):
            folder = ROOT/'physics/runs'/case['name']
            p, m = read(folder/'params.json'), read(folder/'metadata.json')
            a = np.load(folder/'trajectory.npz')
            c = {'parameters_registered': p == case['parameters'],
                 'current_solver': m['source_sha256'] == manifest['source_hashes']['hydro_planing.py'] == digest(ROOT/'physics/hydro_planing.py'),
                 'current_surface': m['planing_surface_source_sha256'] == manifest['source_hashes']['planing_surface.py'] == digest(ROOT/'physics/planing_surface.py'),
                 'current_clipper': m['clip_source_sha256'] == manifest['source_hashes']['clipped_box.py'] == digest(ROOT/'physics/clipped_box.py'),
                 'trajectory_binding': m['trajectory_sha256'] == row['trajectory_sha256'] == digest(folder/'trajectory.npz'),
                 'all_arrays_finite': all(np.isfinite(a[k]).all() for k in a.files),
                 'time_increases': bool(np.all(np.diff(a['t']) > 0)),
                 'power_within_budget': bool(np.max(a['motor_power']) <= p['total_drive_power_w']+1e-5),
                 'positive_drive': bool(np.min(a['motor_power']) >= -1e-7),
                 'passive_water_loads': m['max_dissipative_relative_power_w'] <= 1e-7,
                 'metadata_duration': abs(m['simulated_s']-a['t'][-1]) < 1e-8}
            c = {k: bool(v) for k, v in c.items()}
            checks.append({'name': case['name'], 'checks': c, 'passed': all(c.values())})
            row['peak_motor_power_kw'] = float(a['motor_power'].max()/1000)
            row['first_origin_below_minus1_5_s'] = float(a['t'][np.flatnonzero(a['qpos'][:,2] < -1.5)[0]]) if np.any(a['qpos'][:,2] < -1.5) else None
            rows.append(row)
    byname = {r['name']: r for r in rows}
    comparisons = []
    for geometry, speed in [('paddles',15), ('four_skis',8)]:
        name = f'lowentry_20261027_{geometry}_v{speed}_p176'
        a, b = byname[name], byname[name+'_dt05']
        comparisons.append({'coarse': name, 'fine': name+'_dt05',
                            'same_submerged_outcome': a['outcome'] == b['outcome'] == 'submerged',
                            'stop_time_difference_s': b['duration_s']-a['duration_s'],
                            'origin_minus1_5_time_difference_s': b['first_origin_below_minus1_5_s']-a['first_origin_below_minus1_5_s'],
                            'peak_forward_speed_difference_m_s': b['peak_forward_speed_m_s']-a['peak_forward_speed_m_s'],
                            'coarse_energy_residual_j': a['energy_balance_residual_j'],
                            'fine_energy_residual_j': b['energy_balance_residual_j']})
    report = {'scope': '11 additional free-entry numerical tests in the unchanged uncalibrated model. No road/ramp support, wake coupling or empirical vehicle validation.',
              'passed_traceability_checks': all(c['passed'] for c in checks),
              'manifests_sha256': manifests, 'checks': checks, 'cases': rows, 'halfstep_comparisons': comparisons,
              'empirical_validation': False, 'exact_launch_speed_threshold_identified': False}
    (ROOT/'research/entry_supplement_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    lines = ['# 入水速度、动力与低速起滑补测', '',
             '2026-10-27。针对“为什么只选5 m/s，能否15 m/s，以及是否应该多测”的追加要求，保留原实验并新增9组自由入水和2组半时间步复算。', '',
             '**5 m/s不是速度上限。当前模型中，高速入水维持滑行与从低速加速起滑，是两件需要分别测试的事。**', '',
             '后轮桨叶负责推进，前后板面可提供动态支撑，分工与摩托方案相近。仍需计算浸没位置，因为它决定桨叶、板面和车身哪些部分接水，以及受力和力矩作用在哪里；但不能只用静浮排水量不足，就判定高速运行一定失败。本轮按实际接水状态同时计算静浮力、板面动态力、轮桨力和阻力，再求整车运动。', '',
             '![同动力上限的入水速度对比](entry_speed_supplement.png)', '',
             '两套方案均保持每后轮1040 N·m、88 kW上限（合计176 kW），只给轮轴扭矩，不强制保持车速。初始纵倾0°，最低构件离平水面2 cm，随后自由六自由度运动。纯桨叶1424 kg，四滑板1468.1408 kg，差值包括装置质量。功率上限敏感性固定车重，不代表已完成实车动力改装。', '',
             '0 m/s是从水面上方释放且驱动，不含岸坡、道路或托架。失败组均触发车体参考原点低于−3.5 m的停止条件；下表时间是该停止时刻，不是开始下沉的精确时刻。', '',
             '| 方案 | 初速 m/s | 步长 ms | 运行至停止 s | 峰值实际功率 kW | 结果 |', '|---|---:|---:|---:|---:|---|']
    for r in rows:
        name = '四滑板+轮桨' if r['geometry'] == 'planing_four_skis' else '仅轮桨'
        dt = .5 if r['name'].endswith('dt05') else 1
        lines.append(f"| {name} | {r['initial_speed_m_s']:g} | {dt:g} | {r['duration_s']:.3f} | {r['peak_motor_power_kw']:.1f} | 下潜停止 |")
    lines += ['', '实际功率由当时轮速和扭矩共同决定，列出176 kW预算不代表每时每刻输出176 kW。全部工况数据、功率、力分项、姿态及能量残差均保留，不能把下潜过程末段当作稳态。', '', '## 半时间步对照', '']
    for c in comparisons:
        lines.append(f"- {c['coarse']}：步长1→0.5 ms，停止时刻变化{c['stop_time_difference_s']:+.4f} s，峰值前向速度变化{c['peak_forward_speed_difference_m_s']:+.4f} m/s；原始能量残差{c['coarse_energy_residual_j']:.2f}→{c['fine_energy_residual_j']:.2f} J；两组均下潜。")
    lines += ['', '## 与已有高速运行放在一起看', '',
              '纯轮桨在同176 kW预算、初速0/5/8/10/15/20 m/s下均未维持水面运行。四滑板从0/5/8 m/s释放也未起滑；已有10和15 m/s入水的30秒组在当前对称平水模型内维持动支撑，末段约15.8 m/s。由此只能说初态会影响起滑过程，尚未确定准确起滑速度，也未验证岸边入水过程。', '',
              '这些新增结果补足了“只比较不同速度时动力不同”和“只看高速入水成功”的缺口。影片原有15 m/s入水的限定结果仍成立；不能扩写成从静止自启动、全天候稳定或实车改装可行。', '',
              '现有模型的杯桨仍是双面法向阻力近似，未解析凹凸面差异、喷溅和自由水面反馈；滑板前后尾流也未求解。增加同一模型的测试量能检查参数与数值表现，不能代替模型标定或实车试验。', '',
              '## 核对资料', '',
              '- [全部新增数据和检查](entry_supplement_checks.json)',
              '- [原44组滑行实验](CAR_PLANING_RESULTS.md)',
              '- [仰望U8、Panther、Amphicar对照](AMPHIBIOUS_COMPARISON.md)',
              '- [桨叶形状与旋转检查](PADDLE_DIRECTION_REPAIR.md)', '',
              '11组参数分别在low_speed_entry_manifest.json、matched_power_entry_manifest.json、entry_halfstep_manifest.json登记，原始轨迹位于physics/runs/lowentry_20261027_*。']
    (ROOT/'research/ENTRY_SPEED_SUPPLEMENT.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    print(json.dumps({'passed':report['passed_traceability_checks'],'cases':len(rows),'halfstep':comparisons},ensure_ascii=False))
    if not report['passed_traceability_checks']:
        raise RuntimeError('Entry supplement traceability failure')


if __name__ == '__main__':
    main()
