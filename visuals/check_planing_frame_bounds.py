"""Check each plate's projected corners across recorded Blender poses."""
import argparse,json,math,itertools
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('receipt',type=Path);args=p.parse_args()
d=json.loads(args.receipt.read_text(encoding='utf8'));r=d['render_receipt']
states=json.loads(Path(r['rendered_body_states']).read_text(encoding='utf8'))
params=json.loads((Path(r['run'])/'params.json').read_text(encoding='utf8'))
contacts=[v for v in params['volumes'] if v['name'].startswith(('ski_solid_','side_float_'))]
rx,ry,rz=r['camera_world_rotation_euler'];cx,sx=math.cos(rx),math.sin(rx);cy,sy=math.cos(ry),math.sin(ry);cz,sz=math.cos(rz),math.sin(rz)
camera_R=np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])@np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
offset=np.array(r['camera_world_position'])-np.array(states[0]['bodies']['chassis']['position'])
width=d['job']['width'];height=round(width*9/16);f=r['camera_lens_mm']/36*width
signs=np.array(list(itertools.product([-1,1],repeat=3)));rows=[]
for row in states:
    body=row['bodies']['chassis'];pos=np.array(body['position']);w,x,y,z=body['quaternion_wxyz']
    body_R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    projected=[]
    for v in contacts:
        theta=-math.radians(v.get('pitch_deg',0));c,s=math.cos(theta),math.sin(theta);plate_R=np.array([[c,0,s],[0,1,0],[-s,0,c]])
        local=np.array(v['center'])+(signs*np.array(v['size'])/2)@plate_R.T
        world=pos+local@body_R.T
        camera=(world-(pos+offset))@camera_R
        depth=-camera[:,2];pixels=np.column_stack([width/2+f*camera[:,0]/depth,height/2-f*camera[:,1]/depth])
        projected.extend(pixels.tolist())
    points=np.array(projected);left,top=points.min(axis=0);right,bottom=points.max(axis=0)
    margin=min(left,top,width-right,height-bottom)
    rows.append({'frame':row['frame'],'simulation_time_s':row['simulation_time_s'],'plate_bounds_pixels':[float(left),float(top),float(right),float(bottom)],'minimum_margin_pixels':float(margin)})
worst=min(rows,key=lambda x:x['minimum_margin_pixels'])
result={'id':d['id'],'contact_geometry_count':len(contacts),'frame_count':len(rows),'all_plates_inside_frame':worst['minimum_margin_pixels']>=0,'minimum_margin_pixels':worst['minimum_margin_pixels'],'worst_frame':worst,'method':'Recorded Blender body poses and fixed-relative camera; plate corners from exact params geometry; default 36 mm horizontal sensor.', 'frames':rows}
args.receipt.with_suffix('.framing_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k!='frames'}))
if not result['all_plates_inside_frame']:raise SystemExit(2)
