"""Bind the added film claims to preserved runs; this is not empirical validation."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
from car_planing_matrix import ROOT,read

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    report=read(ROOT/'research/car_planing_results.json')
    rows={r['name']:r for r in report['cases']}
    manifests=[read(p) for p in sorted((ROOT/'research').glob('car_planing_*_manifest.json'))]
    expected={c['name']:c['parameters'] for m in manifests for c in m['cases']}
    frozen='20357f4c7d29238af982a7a1d9416462776ae0e1d8c0f97f9d1de6b017610693'
    sources={n:sha(ROOT/'physics'/n) for n in ['hydro.py','hydro_planing.py','planing_surface.py','clipped_box.py']}
    audits=[]
    for name,params in expected.items():
        dest=ROOT/'physics/runs'/name;m=read(dest/'metadata.json');a=np.load(dest/'trajectory.npz')
        checks={'registered_parameters':params==read(dest/'params.json'),
            'source_hashes':m['source_sha256']==sources['hydro_planing.py'] and m['planing_surface_source_sha256']==sources['planing_surface.py'] and m['clip_source_sha256']==sources['clipped_box.py'],
            'trajectory_hash':m['trajectory_sha256']==sha(dest/'trajectory.npz')==rows[name]['trajectory_sha256'],
            'finite_recorded_arrays':all(np.isfinite(a[k]).all() for k in a.files),
            'time_increasing':bool(np.all(np.diff(a['t'])>0)),
            'duration_metadata':abs(a['t'][-1]-m['simulated_s'])<1e-8,
            'passive_relative_power':m['max_dissipative_relative_power_w']<=1e-7,
            'recorded_power_within_budget':bool(a['motor_power'].max()<=params['total_drive_power_w']+1e-5),
            'positive_drive_no_active_braking':bool(a['motor_power'].min()>=-1e-7)}
        checks={k:bool(v) for k,v in checks.items()}
        audits.append({'name':name,'checks':checks,'passed':all(checks.values()),'outcome':rows[name]['outcome'],
            'mechanical_envelope_overlap':rows[name]['mechanical_envelope_overlap']})
    base=rows['pln_four_skis_v15_p176_30s'];start10=rows['pln_four_skis_v10_p176_30s'];off=rows['pln_four_skis_v15_p176_off12'];roll=rows['pln_four_skis_v15_p176_roll1_30s']
    focused=read(ROOT/'research/planing_final_result_checks.json')
    domain={r['name']:r for r in focused['domain_replays']}
    claims={
        'four_entry_speeds_paddles_fail_in_tested_setup':all(rows[f'pln_paddles_v{v}_t3_p44']['outcome']=='submerged' for v in [5,10,15,20]),
        'two_lower_power_four_skis_fail':all(rows[f'pln_four_skis_v15_t0_p{power}']['outcome']=='submerged' for power in [44,88]),
        'clear_wide_structure_still_fails':all(rows[f'pln_wide_low_clear_v15_t3_p{power}']['outcome']=='submerged' and not rows[f'pln_wide_low_clear_v15_t3_p{power}']['mechanical_envelope_overlap'] for power in [44,176]),
        'symmetric_calm_30s_candidate':all(r['outcome']=='sustained_planing_in_declared_model' for r in [base,start10]),
        'tail_speed_about_57_kmh':abs(base['tail_mean_forward_speed_m_s']*3.6-57)<.5,
        'dynamic_support_dominates':base['tail_support_weight_ratios']['panel_pressure']>.95 and base['tail_support_weight_ratios']['buoyancy']<.04,
        'power_off_then_submerges_about18_4s':off['outcome']=='submerged' and abs(off['duration_s']-18.4)<.1,
        'calm_numeric_and_wet_band_comparisons_reported':len(report['comparisons'])==3 and all(c['within_declared_comparison_thresholds'] for c in report['comparisons']),
        'roll_perturbation_not_misclassified':roll['outcome']=='transient_or_unconfirmed' and roll['tail_valid_fraction_of_tagged_dynamic_lift']==0 and domain[roll['name']]['yaw_change_deg']>6,
        'saved_domain_replay_matches':all(r['replay_max_domain_force_error_n']<1e-8 for r in domain.values()),
        'wave_results_remain_exploratory':all(rows[f'pln_four_skis_v15_p176_{w}']['outcome']=='transient_or_unconfirmed' for w in ['head','cross','swell','irregular','head_dt05']),
        'same_wave_halfstep_completed':bool(report['head_wave_check'])}
    checks={'original_solver_frozen':sources['hydro.py']==frozen,
        'all_registered_cases_preserved':set(expected)==set(rows) and not report['pending'],
        'all_case_data_consistent':all(r['passed'] for r in audits),
        'surface_formula_benchmark':read(ROOT/'research/planing_surface_benchmark.json')['passed'],
        'domain_guards_benchmark':read(ROOT/'research/planing_review_checks.json')['domain_passed'],
        'stated_film_claims_match_runs':all(claims.values()),
        'new_geometry_pose_water_checks':read(ROOT/'visuals/planing/delivery_validation.json')['pass']}
    result={'scope':'Traceability, implementation checks and narrowly stated model-film claims. No full-vehicle empirical validation or proof of seaworthiness.',
        'delivery_claims_supported':all(checks.values()),'empirical_vehicle_validation':False,'wave_capability_validated':False,
        'case_count':len(audits),'checks':checks,'claim_checks':claims,'cases':audits,
        'allowed_result':'Symmetric calm-water candidate sustains model motion for 30 seconds under the declared four-board geometry and 176 kW power budget; speed is a model prediction.',
        'limitations':['Fore/aft board wake interaction, spray, added mass, impact and free-surface feedback unresolved.','Power-rating sweep holds vehicle mass fixed and is not a finished powertrain design.','1 degree initial roll leads to yaw/sideslip outside declared straight-flow screen.','Wave runs have extrapolated loads and only 12-second duration.','Residuals are preserved, including the head-wave half-step comparison; no alternative denominator is substituted for the original metric.'],
        'source_hashes':sources,
        'evidence_sha256':{n:sha(ROOT/n) for n in ['research/car_planing_results.json','research/planing_final_result_checks.json','research/planing_surface_benchmark.json','research/planing_review_checks.json','visuals/planing/delivery_validation.json','script_final.txt']}}
    (ROOT/'research/planing_delivery_checks.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'supported':result['delivery_claims_supported'],'checks':checks,'claim_checks':claims,'failed_cases':[r for r in audits if not r['passed']]},ensure_ascii=False))
    if not result['delivery_claims_supported']:raise RuntimeError('Planing evidence checks failed')

if __name__=='__main__':main()
