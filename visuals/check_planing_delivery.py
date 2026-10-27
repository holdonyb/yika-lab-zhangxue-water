"""Verify new planing geometry, exported poses, water vertices and camera data."""
import argparse, json, hashlib, math
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('receipt',type=Path);a=p.parse_args()
d=json.loads(a.receipt.read_text(encoding='utf8'));r=d['render_receipt'];run=Path(r['run'])
params=json.loads((run/'params.json').read_text(encoding='utf8'))
metadata=json.loads((run/'metadata.json').read_text(encoding='utf8'))
source=np.load(run/'trajectory.npz')
states=json.loads(Path(r['rendered_body_states']).read_text(encoding='utf8'))
anchors=json.loads(Path(r['overlay_anchors']).read_text(encoding='utf8'))
expected=[v for v in params['volumes'] if v['name'].startswith(('side_float_','ski_solid_'))]
clearances=[abs(v['center'][1])-v['size'][1]/2-(abs(w['center'][1])+w.get('blade_width',w['width'])/2)
            for v in expected for w in params['wheels'] if w.get('paddles') and v['center'][1]*w['center'][1]>0]
geometry_errors=[];bottom_errors=[];water_errors=[];position_errors=[];angle_errors=[]
for v in r['contact_geometry']:
    original=next(x for x in expected if x['name']==v['name'])
    geometry_errors.extend([float(np.max(np.abs(np.array(original['center'])-v['blender_local_center']))),
                            float(np.max(np.abs(np.array(original['size'])-v['blender_local_size']))),
                            abs(v['blender_rotation_y_deg']+original.get('pitch_deg',0))])
    if original['name'].startswith('ski_solid_'):
        angle=-math.radians(original.get('pitch_deg',0));c=math.cos(angle);s=math.sin(angle)
        bottom=np.array(v['blender_local_center'])+np.array([[c,0,s],[0,1,0],[-s,0,c]])@np.array([0,0,-original['size'][2]/2])
        panel=next(x for x in params['panels'] if x['name']==original['name'].replace('ski_solid_','ski_bottom_'))
        bottom_errors.append(float(np.max(np.abs(bottom-panel['center']))))
def height(x,y,t):
    return params['water'].get('mean_z',0)+sum(w['amplitude']*math.cos(w['kx']*x+w['ky']*y-w['omega']*t+w['phase']) for w in params['water'].get('components',[]))
for row in states:
    t=row['simulation_time_s'];idx=int(np.clip(np.searchsorted(source['t'],t)-1,0,len(source['t'])-2));u=(t-source['t'][idx])/(source['t'][idx+1]-source['t'][idx])
    for name,body in row['bodies'].items():
        bi=metadata['body_names'].index(name);pos=(1-u)*source['body_pos'][idx,bi]+u*source['body_pos'][idx+1,bi]
        position_errors.append(float(np.linalg.norm(pos-np.array(body['position']))))
        q0=source['body_quat'][idx,bi];q1=source['body_quat'][idx+1,bi]
        if np.dot(q0,q1)<0:q1=-q1
        theta=np.arccos(np.clip(np.dot(q0,q1),-1,1))
        q=(1-u)*q0+u*q1 if abs(theta)<1e-8 else (np.sin((1-u)*theta)*q0+np.sin(u*theta)*q1)/np.sin(theta)
        q/=np.linalg.norm(q);actual=np.array(body['quaternion_wxyz']);actual/=np.linalg.norm(actual)
        angle_errors.append(float(2*np.arccos(np.clip(abs(np.dot(q,actual)),-1,1))))
    for x,y,z in row['water_samples_world']:water_errors.append(abs(z-height(x,y,t)))
for row in anchors['frames']:
    for anchor in row['anchors'].values():
        x,y,z=anchor['water_surface_world'];water_errors.append(abs(z-height(x,y,row['simulation_time_s'])))
count=round(d['job']['duration']*24) if 'duration' in d['job'] else 1
result={'id':d['id'],'frames':len(states),'expected_frames':count,'geometry_count':len(expected),
        'source_video_sha256':d.get('video_sha256'),'source_image_sha256':d.get('image_sha256'),
        'render_source_sha256':r['render_code_sha256'],
        'minimum_paddle_plate_lateral_clearance_m':min(clearances) if clearances else None,
        'lateral_clearance_pass':min(clearances,default=0)>=-1e-8,
        'geometry_pass':len(expected)==len(r['contact_geometry']) and max(geometry_errors,default=0)<1e-5 and max(bottom_errors,default=0)<1e-6,
        'max_geometry_error':max(geometry_errors,default=0),'max_ski_bottom_error_m':max(bottom_errors,default=0),
        'max_water_height_error_m':max(water_errors,default=0),'water_samples_checked':len(water_errors),
        'water_pass':len(water_errors)==count*5 and max(water_errors,default=1)<1e-5 and r['wave_parameters']==params['water'],
        'max_position_error_m':max(position_errors),'max_orientation_error_rad':max(angle_errors),
        'pose_pass':max(position_errors)<1e-4 and max(angle_errors)<1e-4 and len(states)==count and len(anchors['frames'])==count,
        'trajectory_hash_pass':hashlib.sha256((run/'trajectory.npz').read_bytes()).hexdigest()==r['trajectory_sha256'],
        'params_hash_pass':hashlib.sha256((run/'params.json').read_bytes()).hexdigest()==r['geometry_params_sha256'],
        'camera_position':r['camera_world_position'],'camera_rotation_euler':r['camera_world_rotation_euler'],'camera_lens_mm':r['camera_lens_mm']}
result['pass']=all(result[k] for k in ['geometry_pass','lateral_clearance_pass','water_pass','pose_pass','trajectory_hash_pass','params_hash_pass'])
a.receipt.with_suffix('.planing_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(result,ensure_ascii=False))
if not result['pass']:raise SystemExit(2)
