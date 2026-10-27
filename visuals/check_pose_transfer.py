"""Check exported Blender poses against their frozen MuJoCo source arrays."""
import argparse,json,hashlib,math
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw

HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('receipt',type=Path);args=p.parse_args()
d=json.loads(args.receipt.read_text(encoding='utf8'));r=d['render_receipt'];job=d['job']
rp=HERE.parent/'physics/runs'/job['run'];raw=(rp/'trajectory.npz').read_bytes()
if hashlib.sha256(raw).hexdigest()!=d['inputs_sha256']['trajectory']:raise RuntimeError('Input trajectory changed')
source=np.load(rp/'trajectory.npz');meta=json.loads((rp/'metadata.json').read_text(encoding='utf8'))
states=json.loads(Path(d['frames_file']).read_text(encoding='utf8'))
pos_error=[];angle_error=[]
for row in states:
    t=row['simulation_time_s'];a=int(np.clip(np.searchsorted(source['t'],t)-1,0,len(source['t'])-2))
    u=(t-source['t'][a])/(source['t'][a+1]-source['t'][a])
    for name,body in row['bodies'].items():
        bi=meta['body_names'].index(name);expected=(1-u)*source['body_pos'][a,bi]+u*source['body_pos'][a+1,bi]
        pos_error.append(float(np.linalg.norm(expected-np.array(body['position']))))
        q0=source['body_quat'][a,bi];q1=source['body_quat'][a+1,bi]
        if np.dot(q0,q1)<0:q1=-q1
        angle=np.arccos(np.clip(np.dot(q0,q1),-1,1))
        q=((1-u)*q0+u*q1) if abs(angle)<1e-8 else (np.sin((1-u)*angle)*q0+np.sin(u*angle)*q1)/np.sin(angle)
        q=q/np.linalg.norm(q);actual=np.array(body['quaternion_wxyz']);actual/=np.linalg.norm(actual)
        angle_error.append(float(2*np.arccos(np.clip(abs(np.dot(q,actual)),-1,1))))
expected_count=round(job['duration']*24)
anchors=json.loads(Path(d['anchors_file']).read_text(encoding='utf8'))
result={'id':d['id'],'frames':len(states),'expected_frames':expected_count,'anchor_frames':len(anchors['frames']),
        'max_position_error_m':max(pos_error),'max_orientation_error_rad':max(angle_error),
        'trajectory_sha256':d['inputs_sha256']['trajectory'],'video_sha256':d['video_sha256'],
        'pose_transfer_pass':len(states)==expected_count and len(anchors['frames'])==expected_count and max(pos_error)<1e-4 and max(angle_error)<1e-4,
        'wave_parameter_identity':r['wave_parameters']==json.loads((rp/'params.json').read_text(encoding='utf8'))['water']}
args.receipt.with_suffix('.pose_check.json').write_text(json.dumps(result,indent=2),encoding='utf8')
frames=Path(d['frames_dir']);indices=[1,expected_count//2,expected_count]
sheet=Image.new('RGB',(1440,290),(10,20,26));draw=ImageDraw.Draw(sheet)
for col,idx in enumerate(indices):
    im=Image.open(frames/f'frame_{idx:04d}.png').convert('RGB');im.thumbnail((480,270));sheet.paste(im,(480*col,20));draw.text((480*col+12,5),f"{d['id']} / frame {idx}",fill='white')
args.receipt.with_suffix('.contact.jpg').parent.mkdir(exist_ok=True)
sheet.save(args.receipt.with_suffix('.contact.jpg'),quality=92)
print(json.dumps(result))
if not result['pose_transfer_pass']:raise SystemExit(2)
