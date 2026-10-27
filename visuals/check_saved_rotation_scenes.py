"""Reopen only the two delivered editable scenes and inspect their saved cups."""
from pathlib import Path
import hashlib,json
import bpy
from mathutils import Vector
ROOT=Path(__file__).resolve().parents[1]
path=ROOT/'visuals/candidates/current_scene_manifest.json'
manifest=json.loads(path.read_text(encoding='utf8'))
rows=[]
for item in manifest['scenes']:
    scene=ROOT/item['path']
    assert hashlib.sha256(scene.read_bytes()).hexdigest()==item['sha256']
    bpy.ops.wm.open_mainfile(filepath=str(scene))
    graph=bpy.context.evaluated_depsgraph_get()
    cups=[o for o in bpy.data.objects if 'paddle_index' in o]
    dots=[]
    for cup in cups:
        actual=cup.evaluated_get(graph);mesh=actual.to_mesh()
        center,left,right=[actual.matrix_world@mesh.vertices[i].co for i in cup['probe_vertex_indices']]
        aperture=((left+right)/2-center).normalized()
        origin=actual.matrix_world.translation
        axis=actual.matrix_world.to_3x3()@Vector((0,1,0))
        tangent=axis.cross(center-origin).normalized()
        dots.append(float(aperture.dot(tangent)))
        actual.to_mesh_clear()
    expected=12 if 'motorcycle' in scene.name else 24
    passed=len(cups)==expected and min(dots)>.99
    rows.append({'path':item['path'],'sha256':item['sha256'],'actual_saved_cup_count':len(cups),
                 'minimum_aperture_dot_positive_tangent':min(dots),'passed':passed})
    assert passed
manifest['saved_scene_reopen_check']={'passed':True,'scope':'Saved static cup mesh and count, not animation or physical validation.','scenes':rows}
path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(manifest['saved_scene_reopen_check']),flush=True)
