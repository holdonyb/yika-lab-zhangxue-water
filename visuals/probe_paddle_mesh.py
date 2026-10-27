"""Blender: sample actual evaluated paddle vertices through continuous motion."""
import argparse,json,sys,math,hashlib
from pathlib import Path
import bpy
import numpy as np
from mathutils import Vector
HERE=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--renderer',default='render_scene.py');parser.add_argument('--run',required=True);parser.add_argument('--start',type=float,default=2);parser.add_argument('--seconds',type=float,default=.5);parser.add_argument('--out',required=True)
opt=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
source=HERE/opt.renderer;dest=Path(opt.out).resolve();dest.parent.mkdir(parents=True,exist_ok=True)
sys.argv=[sys.argv[0],'--','--run',opt.run,'--start-time',str(opt.start),'--duration','1','--out',str(dest.with_suffix('.scene'))]
ns={'__file__':str(source),'__name__':'__main__'};exec(compile(source.read_text(encoding='utf8'),str(source),'exec'),ns)
scene=ns['scene'];objects=[ob for ob in bpy.data.objects if ob.type=='MESH' and ob.get('paddle_index')==0]
graph=bpy.context.evaluated_depsgraph_get();mount_checks=[]
for ob in objects:
    is_bike=ob.name.startswith('Rear cupped')
    prefix='Paddle attachment 0 ' if is_bike else ob.name.rsplit(' paddle ',1)[0]+' paddle attachment 0 '
    tyre=bpy.data.objects['Rear tire' if is_bike else ob.name.rsplit(' paddle ',1)[0]+' tire']
    cup_eval=ob.evaluated_get(graph);tyre_eval=tyre.evaluated_get(graph)
    for side in [-1,1]:
        tube=bpy.data.objects[prefix+str(side)];actual=tube.evaluated_get(graph)
        zs=[v.co.z for v in tube.data.vertices]
        ends=[actual.matrix_world@Vector((0,0,z)) for z in [min(zs),max(zs)]]
        cup_dist=[];tyre_signed=[]
        for point in ends:
            local=cup_eval.matrix_world.inverted()@point
            hit,nearest,normal,_=cup_eval.closest_point_on_mesh(local)
            cup_dist.append(float((local-nearest).length) if hit else 1e9)
            local=tyre_eval.matrix_world.inverted()@point
            hit,nearest,normal,_=tyre_eval.closest_point_on_mesh(local)
            tyre_signed.append(float((local-nearest).dot(normal)) if hit else 1e9)
        passed=min(cup_dist)<.01 and min(tyre_signed)<-.002 and tube.parent==ob.parent==tyre.parent
        mount_checks.append({'paddle':ob.name,'attachment':tube.name,'cap_end_distance_to_cup_m':cup_dist,
            'cap_end_signed_distance_to_tyre_m':tyre_signed,'same_wheel_parent':tube.parent==ob.parent==tyre.parent,'passed':bool(passed)})
records=[]
for seconds in np.linspace(0,opt.seconds,121):
    frame=1+seconds*24;whole=int(frame);scene.frame_set(whole,subframe=float(frame-whole));graph=bpy.context.evaluated_depsgraph_get()
    row={'time_s':opt.start+float(seconds),'paddles':[]}
    for ob in objects:
        actual=ob.evaluated_get(graph);me=actual.to_mesh();indices=list(ob['probe_vertex_indices'])
        points=[np.array(actual.matrix_world@me.vertices[i].co) for i in indices]
        center,left,right=points;origin=np.array(actual.matrix_world.translation);axis=np.array(actual.matrix_world.to_3x3()@Vector((0,1,0)))
        aperture=(left+right)/2-center;aperture/=np.linalg.norm(aperture)
        tangent=np.cross(axis,center-origin);tangent/=np.linalg.norm(tangent)
        parent=ns['bike'];local=np.array(parent.matrix_world.inverted()@Vector(center))-np.array(parent.matrix_world.inverted()@Vector(origin))
        phase=float(math.atan2(-local[2],local[0]));actual.to_mesh_clear()
        row['paddles'].append({'object':ob.name,'actual_mesh_vertices_world':[v.tolist() for v in points],
            'axle_world':origin.tolist(),'cup_aperture_world':aperture.tolist(),'positive_axle_tangent_world':tangent.tolist(),
            'aperture_dot_positive_rotation':float(aperture@tangent),'marker_phase_relative_chassis_rad':phase})
    records.append(row)
checks={'has_actual_paddles':bool(objects),'cups_face_rotational_travel':all(p['aperture_dot_positive_rotation']>.99 for r in records for p in r['paddles'])}
checks['actual_mounts_connect_rubber_and_cup']=all(m['passed'] for m in mount_checks)
phase_checks=[]
for ob in objects:
    phase=np.unwrap([next(p['marker_phase_relative_chassis_rad'] for p in r['paddles'] if p['object']==ob.name) for r in records])
    phase_checks.append({'object':ob.name,'unwrapped_marker_advance_rad':float(phase[-1]-phase[0]),'minimum_sample_advance_rad':float(np.min(np.diff(phase)))})
checks['continuous_positive_marked_mesh_motion']=all(p['minimum_sample_advance_rad']>0 for p in phase_checks)
report={'scope':'Actual evaluated Blender mesh vertices, including Solidify; 121 contiguous samples, no playback reversal or angle sign override.',
    'renderer':opt.renderer,'renderer_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
    'paddle_source_sha256':hashlib.sha256((HERE/'paddle_geometry.py').read_bytes()).hexdigest(),
    'run':str(Path(opt.run).resolve()),'checks':checks,'passed':all(checks.values()),'mount_checks':mount_checks,'phase_checks':phase_checks,'records':records}
dest.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print('PADDLE_MESH_PROBE '+json.dumps({'passed':report['passed'],'checks':checks,'phase':phase_checks}))
if not report['passed']:raise RuntimeError('Actual paddle mesh direction failed')
