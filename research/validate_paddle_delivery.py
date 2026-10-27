"""Validate the changed film dependencies and actual directed-mesh evidence."""
from pathlib import Path
import hashlib,json
import numpy as np
ROOT=Path(__file__).resolve().parents[1];VIS=ROOT/'visuals'
OUT=ROOT/'research/paddle_direction_20261027'
def read(p):return json.loads(p.read_text(encoding='utf8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    rows=[]
    for branch in ['base','planing']:
        folder=VIS/('planing/clips' if branch=='planing' else 'clips')
        renderer=VIS/('render_scene_planing.py' if branch=='planing' else 'render_scene.py')
        for job in read(VIS/f'rotation_repair_20261027/{branch}_jobs.json'):
            stem=folder/job['id'];d=read(stem.with_suffix('.json'));r=d['render_receipt']
            states=read(Path(d['frames_file']));check=read(stem.with_suffix('.planing_check.json' if branch=='planing' else '.pose_check.json'))
            run=ROOT/'physics/runs'/job['run'];m=read(run/'metadata.json');a=np.load(run/'trajectory.npz')
            times=[s['simulation_time_s'] for s in states];expected=job['start']+np.arange(round(job['duration']*24))/24*job['scale']
            mb=r['motion_blur'];subframes=[t for _,f,t in mb['evaluated_times'] if abs(f)>1e-5]
            checks={'current_mesh_source':r['paddle_geometry_sha256']==sha(VIS/'paddle_geometry.py')==d['inputs_sha256']['paddle_geometry'],
                'current_renderer':r['render_code_sha256']==sha(renderer)==d['inputs_sha256']['renderer'],
                'current_assembly':d['inputs_sha256']['geometry']==sha(VIS/'motorcycle_geometry.py'),
                'unchanged_solver_and_trajectory':m['source_sha256']==sha(ROOT/'physics'/('hydro_planing.py' if branch=='planing' else 'hydro.py')) and m['trajectory_sha256']==sha(run/'trajectory.npz')==d['inputs_sha256']['trajectory'],
                'current_encoded_source':d['video_sha256']==sha(stem.with_suffix('.mp4')) and d['full_decode_pass'],
                'all_requested_frames':len(states)==round(job['duration']*24) and [s['frame'] for s in states]==list(range(1,len(states)+1)),
                'forward_solver_time':len(times)==len(expected) and bool(np.max(abs(np.array(times)-expected))<1e-6),
                'saved_joint_advance_below_pi':bool(np.max(abs(np.diff(a['qpos'][:,7:],axis=0)))<np.pi),
                'poses_and_water_match':check['pass'] if branch=='planing' else check['pose_transfer_pass'] and check['wave_parameter_identity'],
                'real_temporal_exposure_or_readable_slow_motion':(mb['enabled'] and mb['shutter_frames']==1 and mb['steps']>=3 and len(subframes)>len(states)*2) if job['scale']>.2 else (not mb['enabled'] and job['scale']<=.1)}
            rows.append({'id':job['id'],'run':job['run'],'branch':branch,'video_sha256':d['video_sha256'],'renderer_sha256':r['render_code_sha256'],
                'trajectory_sha256':m['trajectory_sha256'],'maximum_saved_joint_advance_rad':float(np.max(abs(np.diff(a['qpos'][:,7:],axis=0)))),
                'checks':{k:bool(v) for k,v in checks.items()},'passed':all(checks.values()),'real_shutter_samples':len(mb['evaluated_times'])})
    probes=[read(OUT/(name+'_final_mesh_probe.json')) for name in ['bike','car','planing']]
    mesh_checks=all(p['passed'] and p['renderer_sha256']==sha(VIS/p['renderer']) and p['paddle_source_sha256']==sha(VIS/'paddle_geometry.py') for p in probes)
    stills=[]
    for rel in [*[f'visuals/stills/{j["id"]}' for j in read(VIS/'still_jobs.json')],*[f'visuals/planing/stills/{j["id"]}' for j in read(VIS/'planing_still_jobs.json')],
                'visuals/candidates/motorcycle_cutout','visuals/candidates/car_cutout','visuals/film_stills/bike_buoyant']:
        base=ROOT/rel;r=read(base.with_suffix('.render.json'));renderer=VIS/('render_scene_planing.py' if '/planing/' in rel else 'render_scene.py')
        stills.append({'path':rel+'.png','sha256':sha(base.with_suffix('.png')),'passed':r['paddle_geometry_sha256']==sha(VIS/'paddle_geometry.py') and r['render_code_sha256']==sha(renderer)})
    checks={'all_23_used_sources_rebuilt':len(rows)==23 and all(r['passed'] for r in rows),
        'all_directly_derived_stills_current':all(r['passed'] for r in stills),
        'actual_evaluated_mesh_direction':mesh_checks,'saved_state_force_replay':read(OUT/'solver_replay.json')['passed'],
        'canonical_audio_unchanged':sha(ROOT/'tts/narration.wav')=='73d68add5884f11d8bc46d361f53180f24942fd72b31b9e5e2e0a1a7b26379c3',
        'timeline_unchanged':sha(ROOT/'timeline.json')==sha(ROOT/'archive_versions/rotation_before_20261027/timeline.json')}
    result={'scope':'Changed visual geometry, continuous orientation and temporal exposure; preserved physical data and narration.',
        'passed':all(checks.values()),'checks':checks,'sources':rows,'stills':stills,
        'source_sha256':{name:sha(VIS/name) for name in ['paddle_geometry.py','motorcycle_geometry.py','render_scene.py','render_scene_planing.py']},
        'probe_sha256':{name:sha(OUT/name) for name in ['bike_final_mesh_probe.json','car_final_mesh_probe.json','planing_final_mesh_probe.json','solver_replay.json']},
        'physical_model_scope':'The paddle load remains idealized two-sided normal drag; concave/convex cup drag differences are not resolved. No new empirical vehicle claim.',
        'continuous_human_watch':False,'visual_review':'Actual final encoded sequences are reviewed separately and bound to the final MP4.'}
    (OUT/'delivery_checks.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'passed':result['passed'],'checks':checks,'sources':len(rows),'failed':[r for r in rows if not r['passed']]},ensure_ascii=False))
    if not result['passed']:raise RuntimeError('Paddle delivery check failed')

if __name__=='__main__':main()
