"""Independent polygon / scipy ConvexHull reference for clip_cells."""
from pathlib import Path
import hashlib
import importlib.util
import itertools
import json
import time
import numpy as np
from scipy.spatial import ConvexHull
from exact_box_clip import polygon_wet_rectangle

ROOT=Path(__file__).resolve().parents[1]
path=ROOT/'physics'/'clipped_box.py'
spec=importlib.util.spec_from_file_location('clip_module',path)
clip_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(clip_module)
clip_cells=clip_module.clip_cells

def reference(center,edges,level):
    signs=np.array(list(itertools.product([-.5,.5],repeat=3)))
    vertices=center+signs@edges
    z=vertices[:,2]
    if z.max()<=level:return 1.,center.copy()
    if z.min()>=level:return 0.,center.copy()
    points=list(vertices[z<=level])
    for i,j in itertools.combinations(range(8),2):
        if np.count_nonzero(signs[i]!=signs[j])!=1:continue
        if (z[i]<level<z[j]) or (z[j]<level<z[i]):
            points.append(vertices[i]+(vertices[j]-vertices[i])*(level-z[i])/(z[j]-z[i]))
    points=np.asarray(points)
    hull=ConvexHull(points)
    interior=points.mean(axis=0)
    triangles=points[hull.simplices]
    v=np.abs(np.einsum('ij,ij->i',triangles[:,0]-interior,np.cross(triangles[:,1]-interior,triangles[:,2]-interior)))/6
    vol=v.sum()
    centroid=np.sum(v[:,None]*(triangles.sum(axis=1)+interior)/4,axis=0)/vol
    return float(vol/abs(np.linalg.det(edges))),centroid

rollchecks=[]
for angle in [0.,.01,1.,3.,5.,15.,45.,89.9,90.,-5.,-15.]:
    a=np.deg2rad(angle);c,s=np.cos(a),np.sin(a)
    R=np.array([[1,0,0],[0,c,-s],[0,s,c]])
    edges=np.diag([2,2,1])@R.T
    f,centroid=clip_cells(np.array([[0.,0.,.25]]),edges,0.)
    area,reference_center=polygon_wet_rectangle(2,1,a,.25)
    rollchecks.append(dict(roll_deg=angle,volume_abs_error_m3=float(abs(f[0]*4-area*2)),centroid_abs_error_m=float(np.max(np.abs(centroid[0,1:]-reference_center)))))

rng=np.random.default_rng(20261026)
centers=[];edge_list=[];levels=[]
for i in range(250):
    R,_=np.linalg.qr(rng.normal(size=(3,3)))
    if np.linalg.det(R)<0:R[:,0]*=-1
    size=10**rng.uniform(-1,1,3)
    center=rng.uniform(-3,3,3)
    ed=np.diag(size)@R.T
    extent=np.sum(np.abs(ed[:,2]))
    level=center[2]+rng.uniform(-.51,.51)*extent
    centers.append(center);edge_list.append(ed);levels.append(level)
# Near-horizontal and nearly one-axis projections, as well as near-empty/full.
for roll in [1e-12,1e-9,1e-6,1e-3,.1]:
    for pitch in [1e-12,1e-7,.07]:
        cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch)
        R=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
        ed=np.diag([1.7,.23,.031])@R.T
        for f in [1e-6,.01,.3,.5,.99,1-1e-6]:
            extent=np.sum(np.abs(ed[:,2]));center=np.zeros(3)
            centers.append(center);edge_list.append(ed);levels.append((f-.5)*extent)

centers=np.asarray(centers);edge_list=np.asarray(edge_list);levels=np.asarray(levels)
fractions,centroids=clip_cells(centers,edge_list,levels)
max_f_error=0.;max_c_error=0.;worst=[]
for i,(c,e,l) in enumerate(zip(centers,edge_list,levels)):
    f,cc=reference(c,e,l)
    errf=abs(f-fractions[i]);errc=float(np.max(np.abs(cc-centroids[i])))
    max_f_error=max(max_f_error,errf);max_c_error=max(max_c_error,errc)
    if errf>1e-10 or errc>1e-7:worst.append(dict(i=i,frac_error=errf,centroid_error=errc,reference_fraction=f,computed_fraction=float(fractions[i])))
batch_c=np.tile(centers[0],(10000,1));batch_e=np.tile(edge_list[0],(10000,1,1));batch_l=np.linspace(np.min(batch_c[:,2])-1,np.max(batch_c[:,2])+1,10000)
t=time.perf_counter();clip_cells(batch_c,batch_e,batch_l);elapsed=time.perf_counter()-t
out=dict(source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),cases=len(centers),reference='scipy.spatial.ConvexHull of original wet vertices plus exact box-edge/plane intersections; positive tetrahedral volume and centroid',roll_polygon_checks=rollchecks,max_fraction_abs_error=max_f_error,max_centroid_abs_error_m=max_c_error,failures=worst,passed=not worst and max(x['volume_abs_error_m3'] for x in rollchecks)<1e-10 and max(x['centroid_abs_error_m'] for x in rollchecks)<1e-9,batch_10000_wall_s=elapsed)
(ROOT/'research'/'clipped_box_checks.json').write_text(json.dumps(out,indent=2),encoding='utf8')
print(json.dumps(out,indent=2))
