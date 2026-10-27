"""Finish derived local assets after the owned serial render queue completes.

Stops at pending actual visual inspection; never marks pictures reviewed.
"""
from pathlib import Path
import hashlib, json, subprocess, sys, time
ROOT=Path(__file__).resolve().parents[1]
VIS=ROOT/'visuals'

def read(p): return json.loads(p.read_text(encoding='utf8'))
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def run(script,*args):
    print('STAGE '+script,flush=True)
    subprocess.run([sys.executable,str(ROOT/script),*args],cwd=ROOT,check=True)

def main():
    sources={name:sha(VIS/name) for name in ['paddle_geometry.py','motorcycle_geometry.py','render_scene.py','render_scene_planing.py']}
    last=-1
    while True:
        assert all(sha(VIS/name)==h for name,h in sources.items()), 'Frozen render source changed'
        ready=0
        for branch in ['base','planing']:
            folder=VIS/('planing/clips' if branch=='planing' else 'clips')
            renderer='render_scene_planing.py' if branch=='planing' else 'render_scene.py'
            for job in read(VIS/f'rotation_repair_20261027/{branch}_jobs.json'):
                d=read(folder/(job['id']+'.json'));inputs=d.get('inputs_sha256',{})
                if (inputs.get('paddle_geometry')==sources['paddle_geometry.py']
                    and inputs.get('geometry')==sources['motorcycle_geometry.py']
                    and inputs.get('renderer')==sources[renderer]
                    and d.get('render_receipt',{}).get('paddle_geometry_sha256')==sources['paddle_geometry.py']):ready+=1
        if ready!=last:print(f'CURRENT_SOURCES {ready}/23',flush=True);last=ready
        if ready==23:break
        time.sleep(10)
    run('visuals/render_rotation_repair.py','--stage','stills')
    for job in read(VIS/'planing_still_jobs.json'):
        run('visuals/check_planing_delivery.py',str(VIS/'planing/stills'/(job['id']+'.json')))
    run('visuals/postprocess_rotation_sources.py')
    run('graphics/parameter_cards.py')
    run('graphics/four_cars.py')
    run('graphics/planing_graphics.py','--parts','cards')
    run('visuals/build_rotation_evidence.py')
    print('AWAITING_OWNER_ACTUAL_IMAGE_INSPECTION',flush=True)

if __name__=='__main__': main()
