"""Serial GPU rendering; deterministic frame cache and per-shot media checks."""
from pathlib import Path
import subprocess,json,hashlib,time,argparse,shutil
HERE=Path(__file__).resolve().parent
BLENDER=Path('C:/Program Files/Blender Foundation/Blender 5.1/blender.exe')
p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,default=HERE/'formal_jobs.json');p.add_argument('--only');args=p.parse_args()
jobs=json.loads(args.manifest.read_text(encoding='utf8'))
if args.only:jobs=[j for j in jobs if j['id']==args.only]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for sub in ['clips','frames','logs']: (HERE/sub).mkdir(exist_ok=True)
for job in jobs:
    rp=HERE.parent/'physics/runs'/job['run'];logged_wait=False
    while not all((rp/n).exists() for n in ['metadata.json','params.json','trajectory.npz']):
        if not logged_wait:print('WAITING '+job['run'],flush=True);logged_wait=True
        time.sleep(5)
    meta=json.loads((rp/'metadata.json').read_text(encoding='utf8'))
    trajectory_hash=sha(rp/'trajectory.npz')
    if meta['trajectory_sha256']!=trajectory_hash:raise RuntimeError('Trajectory hash disagrees with completed solver metadata: '+job['run'])
    hashes={name:sha(path) for name,path in {'trajectory':rp/'trajectory.npz','params':rp/'params.json','renderer':HERE/'render_scene.py','geometry':HERE/'motorcycle_geometry.py','paddle_geometry':HERE/'paddle_geometry.py'}.items()}
    key=hashlib.sha256(json.dumps([job,hashes],sort_keys=True).encode()).hexdigest()[:12]
    frames=HERE/'frames'/f"{job['id']}_{key}";dest=HERE/'clips'/f"{job['id']}.mp4";receipt=dest.with_suffix('.json')
    if receipt.exists() and dest.exists():
        previous=json.loads(receipt.read_text(encoding='utf8'))
        if previous.get('cache_key')==key and previous.get('full_decode_pass'):
            print('CACHED '+job['id'],flush=True);continue
    expected=round(job['duration']*24)
    if len(list(frames.glob('frame_*.png')))!=expected or not frames.with_suffix('.render.json').exists():
        cmd=[str(BLENDER),'-b','-t','4','-P',str(HERE/'render_scene.py'),'--','--run',str(rp),'--render','--duration',str(job['duration']),
             '--start-time',str(job['start']),'--time-scale',str(job['scale']),'--width',str(job['width']),'--samples',str(job['samples']),
             '--camera',job['camera'],'--out',str(frames)]
        print('RENDER '+job['id'],flush=True);started=time.monotonic()
        with (HERE/'logs'/f"{job['id']}.log").open('w',encoding='utf8') as log:subprocess.run(cmd,cwd=HERE,stdout=log,stderr=subprocess.STDOUT,check=True)
        print('RENDERED '+job['id']+f' {time.monotonic()-started:.1f}s',flush=True)
    if len(list(frames.glob('frame_*.png')))!=expected:raise RuntimeError('Frame count mismatch: '+job['id'])
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-framerate','24','-i',str(frames/'frame_%04d.png'),'-c:v','libx264',
                    '-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(dest)],check=True)
    probe=subprocess.run(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(dest)],capture_output=True,text=True,check=True)
    subprocess.run(['ffmpeg','-v','error','-i',str(dest),'-f','null','-'],check=True)
    data={'id':job['id'],'job':job,'cache_key':key,'inputs_sha256':hashes,'video_sha256':sha(dest),'full_decode_pass':True,
          'metadata':json.loads(probe.stdout),'render_receipt':json.loads(frames.with_suffix('.render.json').read_text(encoding='utf8')),
          'frames_dir':str(frames),'visual_review':'pending representative frame inspection'}
    for suffix in ['anchors','frames']:
        source=frames.with_suffix(f'.{suffix}.json')
        if source.exists():
            target=dest.with_suffix(f'.{suffix}.json');shutil.copyfile(source,target);data[suffix+'_file']=str(target)
    receipt.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
    print('READY '+str(dest),flush=True)
