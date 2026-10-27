from pathlib import Path
import json,subprocess,hashlib,time
from PIL import Image
HERE=Path(__file__).resolve().parent
OUT=HERE/'stills';OUT.mkdir(exist_ok=True)
jobs=json.loads((HERE/'still_jobs.json').read_text(encoding='utf8'))
for j in jobs:
    run=HERE.parent/'physics/runs'/j['run']
    while not all((run/f).exists() for f in ['trajectory.npz','params.json','metadata.json']):time.sleep(5)
    meta=json.loads((run/'metadata.json').read_text(encoding='utf8'));h=hashlib.sha256((run/'trajectory.npz').read_bytes()).hexdigest()
    if h!=meta['trajectory_sha256']:raise RuntimeError('Source trajectory incomplete: '+j['run'])
    if j['time']>meta['simulated_s']+1e-5:raise RuntimeError('Requested still exceeds simulation time: '+j['id'])
    dest=OUT/j['id'];print('STILL '+j['id'],flush=True)
    cmd=['C:/Program Files/Blender Foundation/Blender 5.1/blender.exe','-b','-t','4','-P',str(HERE/'render_scene.py'),'--',
         '--run',str(run),'--preview','--start-time',str(j['time']),'--width','1280','--samples','24','--camera',j['camera'],'--out',str(dest)]
    with (HERE/'logs'/f"{j['id']}.log").open('w',encoding='utf8') as log:subprocess.run(cmd,cwd=HERE,stdout=log,stderr=subprocess.STDOUT,check=True)
    im=Image.open(dest.with_suffix('.png'));im.load()
    if im.size!=(1280,720):raise RuntimeError('Still image size mismatch')
    record={'id':j['id'],'job':j,'trajectory_sha256':h,'image_sha256':hashlib.sha256(dest.with_suffix('.png').read_bytes()).hexdigest(),
            'size':list(im.size),'render_receipt':json.loads(dest.with_suffix('.render.json').read_text(encoding='utf8')),'image_decode_pass':True}
    dest.with_suffix('.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf8')
    print('READY '+str(dest.with_suffix('.png')),flush=True)
