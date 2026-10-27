"""Bounded repair queue: only film-used sources and directly derived stills."""
from pathlib import Path
import argparse,hashlib,json,subprocess,sys
ROOT=Path(__file__).resolve().parents[1];HERE=ROOT/'visuals'
BLENDER='C:/Program Files/Blender Foundation/Blender 5.1/blender.exe'
OUT=HERE/'rotation_repair_20261027';OUT.mkdir(exist_ok=True)

def read(path):return json.loads(path.read_text(encoding='utf8'))
def run(args,log=None):
    if log:
        with log.open('w',encoding='utf8') as f:subprocess.run(args,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
    else:subprocess.run(args,cwd=ROOT,check=True)

def manifest():
    found={}
    for shot in read(ROOT/'timeline.json')['shots']:
        for m in shot['media']:
            path=ROOT/m['path']
            if not m['path'].startswith('visuals/') or path.suffix!='.mp4':continue
            d=read(path.with_suffix('.json'))
            if 'job' not in d:d=read(Path(d['source']).with_suffix('.json'))
            found[d['id']]=d['job']
    for branch in ['base','planing']:
        jobs=[j for j in found.values() if j['run'].startswith('pln_')==(branch=='planing')]
        target=OUT/f'{branch}_jobs.json'
        if not target.exists():target.write_text(json.dumps(jobs,ensure_ascii=False,indent=2),encoding='utf8')
    return found

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--stage',choices=['previews','formal','stills'],required=True);args=parser.parse_args()
    if args.stage=='previews':
        for name,renderer,case,camera,t,scale in [
            ('car_normal','render_scene.py','car_large_floats_final','cutaway',2,1),
            ('planing_normal','render_scene_planing.py','pln_four_skis_v15_p176_30s','cutaway',20,1),
            ('planing_slow','render_scene_planing.py','pln_four_skis_v15_p176_30s','cutaway',20,.05)]:
            print('PREVIEW '+name,flush=True)
            run([BLENDER,'-b','-t','4','-P',str(HERE/renderer),'--','--run',str(ROOT/'physics/runs'/case),'--render','--duration','1','--start-time',str(t),'--time-scale',str(scale),'--width','960','--samples','8','--camera',camera,'--out',str(OUT/name)],OUT/(name+'.log'))
            run(['ffmpeg','-y','-v','error','-framerate','24','-i',str(OUT/name/'frame_%04d.png'),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/(name+'.mp4'))])
    elif args.stage=='formal':
        found=manifest();print('FILM_SOURCE_COUNT '+str(len(found)),flush=True)
        for branch,script in [('base','render_batch.py'),('planing','render_planing_batch.py')]:
            run([sys.executable,str(HERE/script),'--manifest',str(OUT/f'{branch}_jobs.json')])
    else:
        run([sys.executable,str(HERE/'render_stills.py')])
        run([sys.executable,str(HERE/'render_planing_stills.py')])
        for name,case,camera,at,transparent in [
            ('candidates/motorcycle_cutout','bike_flat_final','front',0,True),
            ('candidates/car_cutout','car_large_floats_final','front',0,True),
            ('film_stills/bike_buoyant','bike_stop_buoyant_final','side',9,False)]:
            command=[BLENDER,'-b','-t','4','-P',str(HERE/'render_scene.py'),'--','--run',str(ROOT/'physics/runs'/case),'--preview','--start-time',str(at),'--camera',camera,'--width','1920','--samples','24','--out',str(HERE/name)]
            if transparent:command+=['--transparent','--save-blend']
            run(command,OUT/(Path(name).name+'.log'))
        current={'scope':'Current editable static scenes from corrected paddle sources; historical candidate blends excluded.',
            'source_sha256':{n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ['paddle_geometry.py','motorcycle_geometry.py','render_scene.py']},
            'scenes':[{'path':'visuals/candidates/'+n+'.blend','sha256':hashlib.sha256((HERE/'candidates'/(n+'.blend')).read_bytes()).hexdigest(),
                       'render_receipt':'visuals/candidates/'+n+'.render.json'} for n in ['motorcycle_cutout','car_cutout']],
            'saved_scene_reopen_check':'pending'}
        (HERE/'candidates/current_scene_manifest.json').write_text(json.dumps(current,ensure_ascii=False,indent=2),encoding='utf8')
        run([BLENDER,'-b','-t','4','-P',str(HERE/'check_saved_rotation_scenes.py')],OUT/'saved_scene_reopen.log')

if __name__=='__main__':main()
