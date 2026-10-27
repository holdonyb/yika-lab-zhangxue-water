"""Verify only newly completed sources, then refresh their instrument overlays."""
from pathlib import Path
import json,subprocess,sys,hashlib,argparse
ROOT=Path(__file__).resolve().parents[1];HERE=ROOT/'visuals'
def read(p):return json.loads(p.read_text(encoding='utf8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--ready-only',action='store_true');args=parser.parse_args()
    current=sha(HERE/'paddle_geometry.py');ready=[]
    for branch in ['base','planing']:
        folder=HERE/('planing/clips' if branch=='planing' else 'clips')
        for job in read(HERE/f'rotation_repair_20261027/{branch}_jobs.json'):
            base=folder/job['id'];receipt=base.with_suffix('.json');d=read(receipt)
            fresh=d.get('render_receipt',{}).get('paddle_geometry_sha256')==current and d.get('inputs_sha256',{}).get('paddle_geometry')==current
            if not fresh:
                if args.ready_only:continue
                raise RuntimeError('Direction repair not rendered: '+job['id'])
            check=base.with_suffix('.planing_check.json' if branch=='planing' else '.pose_check.json')
            old=read(check) if check.exists() else {}
            old_sha=old.get('source_video_sha256') if branch=='planing' else old.get('video_sha256')
            if old_sha!=d['video_sha256']:
                script='check_planing_delivery.py' if branch=='planing' else 'check_pose_transfer.py'
                subprocess.run([sys.executable,str(HERE/script),str(receipt)],cwd=ROOT,check=True)
            ready.append((branch,job['id']))
    ids=[name for branch,name in ready if branch=='base']
    if ids:subprocess.run([sys.executable,str(ROOT/'graphics/batch_hud.py'),'--only',*ids],cwd=ROOT,check=True)
    if any(branch=='planing' for branch,_ in ready):
        # The planing helper is content-addressed and skips unchanged overlays.
        subprocess.run([sys.executable,str(ROOT/'graphics/planing_graphics.py'),'--parts','hud'],cwd=ROOT,check=True)
    print(json.dumps({'ready_source_count':len(ready),'ready':[name for _,name in ready]}),flush=True)

if __name__=='__main__':main()
