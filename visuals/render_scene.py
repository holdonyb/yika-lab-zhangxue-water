"""Blender Eevee visualization of recorded MuJoCo body states.

The geometry-only preview uses a static assembly pose. Motion rendering requires
trajectory.npz, metadata.json and params.json; no fallback motion is generated.
"""
import argparse, json, math, sys, time, hashlib, io
from pathlib import Path
import bpy
import numpy as np
from mathutils import Vector, Quaternion, Matrix
from bpy_extras.object_utils import world_to_camera_view

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from paddle_geometry import cup_mesh, stamp_cup, cup_mount_points
p = argparse.ArgumentParser()
p.add_argument('--vehicle', choices=['motorcycle','car'],default='motorcycle')
p.add_argument('--variant', choices=['no_mod','paddles','small_floats','large_floats'],default='large_floats')
p.add_argument('--look', choices=['day','dusk'],default='day')
p.add_argument('--run', type=Path)
p.add_argument('--out', default='preview')
p.add_argument('--start-time', type=float, default=0)
p.add_argument('--duration', type=float, default=8)
p.add_argument('--time-scale', type=float, default=1)
p.add_argument('--width', type=int, default=1280)
p.add_argument('--samples',type=int,default=24)
p.add_argument('--render',action='store_true')
p.add_argument('--preview',action='store_true')
p.add_argument('--frame',type=int,default=1)
p.add_argument('--camera',choices=['front','side','rear','high','wide','bow','top','waterline','paddle','cutaway'],default='front')
p.add_argument('--save-blend',action='store_true')
p.add_argument('--transparent',action='store_true')
p.add_argument('--benchmark-frames',type=int,default=0)
args = p.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
params_path=(args.run/'params.json') if args.run else HERE/f'geometry_{"bike" if args.vehicle=="motorcycle" else "car"}_{"base" if args.vehicle=="motorcycle" else args.variant}.json'
params_bytes=params_path.read_bytes();params=json.loads(params_bytes.decode('utf8'))
params_sha256=hashlib.sha256(params_bytes).hexdigest()
args.vehicle='motorcycle' if params.get('vehicle')=='bike' else 'car'
args.variant=params.get('variant',args.variant)
render_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
STARTED=time.time()
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene
scene.render.engine='BLENDER_EEVEE'
scene.render.resolution_x=args.width; scene.render.resolution_y=round(args.width*9/16)
scene.render.resolution_percentage=100; scene.render.fps=24
scene.render.image_settings.file_format='PNG';scene.render.image_settings.color_mode='RGB'
scene.render.image_settings.compression=15
scene.render.film_transparent=False
if args.transparent:scene.render.film_transparent=True;scene.render.image_settings.color_mode='RGBA'
scene.render.use_file_extension=True
scene.eevee.taa_render_samples=args.samples
scene.eevee.use_raytracing=True
scene.render.use_motion_blur=bool(args.render and args.time_scale>.2)
scene.render.motion_blur_shutter=1.0
scene.eevee.motion_blur_max=512
scene.eevee.motion_blur_steps=3
scene.eevee.ray_tracing_options.resolution_scale='2'
scene.world.use_nodes=True
wn=scene.world.node_tree.nodes;wn.clear()
sky=wn.new('ShaderNodeTexSky');sky.sky_type='MULTIPLE_SCATTERING'
sky.sun_elevation=math.radians(28 if args.look=='day' else 6)
sky.sun_rotation=math.radians(135);sky.altitude=.1;sky.air_density=1.1;sky.aerosol_density=1.2
bg=wn.new('ShaderNodeBackground');bg.inputs['Strength'].default_value=.18 if args.look=='day' else .13
worldout=wn.new('ShaderNodeOutputWorld')
scene.world.node_tree.links.new(sky.outputs['Color'],bg.inputs[0]);scene.world.node_tree.links.new(bg.outputs[0],worldout.inputs[0])
scene.view_settings.view_transform='AgX'
scene.view_settings.look='AgX - Medium High Contrast'
scene.view_settings.exposure=-.45 if args.look=='day' else -.10

def material(name,color,metallic=0,roughness=.4,coat=0):
    m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
    bs=m.node_tree.nodes.get('Principled BSDF')
    bs.inputs['Base Color'].default_value=(*color,1)
    bs.inputs['Metallic'].default_value=metallic;bs.inputs['Roughness'].default_value=roughness
    bs.inputs['Coat Weight'].default_value=coat
    return m
def mat(name,color):
    return material(name,color,roughness=.35)
M={
 'orange':material('Cobalt paint',(.018,.125,.62),.4,.22,.5),
 'cream':material('Ivory enamel',(.78,.84,.88),.25,.27,.4),
 'black':material('Tire rubber',(.014,.018,.023),0,.64),
 'dark':material('Graphite polymer',(.022,.03,.042),.15,.42),
 'metal':material('Brushed aluminium',(.35,.42,.47),.88,.3),
 'chrome':material('Polished steel',(.70,.79,.84),.93,.19),
 'board':material('Satin aluminium planing boards',(.55,.65,.72),.85,.25),
 'up':material('Gold detail',(.99,.48,.035),.5,.25),
 'flow':material('Cyan',(.07,.6,.74),.2,.3),
 'push':material('Orange',(.95,.14,.045),.3,.3),
 'wet':material('Paddle rubber',(.035,.043,.05),0,.5),
 'foam':material('Surface foam',(.69,.87,.9),0,.55),
}

# The motorcycle assembly has no update handler and contains no motion code.
exec(compile((HERE/'motorcycle_geometry.py').read_text(encoding='utf-8'),str(HERE/'motorcycle_geometry.py'),'exec'))
motorcycle_root=bike
motorcycle_pivots=list(pivots)

def remove_tree(ob):
    for ch in list(ob.children):remove_tree(ch)
    bpy.data.objects.remove(ob,do_unlink=True)

def car_body(stations, name, material_key, parent):
    # Cross-sections include shoulder and rocker transitions, not a box hull.
    vv=[]
    for x,w,bot,shoulder,top in stations:
        rocker=bot+min(.10,(top-bot)*.30)
        vv.extend([(x,-w*.87,bot),(x,-w,rocker),(x,-w,shoulder),(x,-w*.91,top),
                   (x,w*.91,top),(x,w,shoulder),(x,w,rocker),(x,w*.87,bot)])
    ff=[tuple(reversed(range(8)))]
    for i in range(len(stations)-1):
        for j in range(8):ff.append((i*8+j,i*8+(j+1)%8,(i+1)*8+(j+1)%8,(i+1)*8+j))
    ff.append(tuple(range((len(stations)-1)*8,len(stations)*8)))
    bevel=min(.045,min(s[4]-s[2] for s in stations)*.20)
    ob=mesh(name,vv,ff,material_key,parent,bevel)
    weighted=ob.modifiers.new('Stable panel shading','WEIGHTED_NORMAL');weighted.keep_sharp=True;weighted.weight=30
    return ob

def add_paddle_wheel(root, radius, width, prefix):
    torus(prefix+' tire',(0,0,0),radius-.075,.075,'black',root)
    for sg in [-1,1]:
        yy=sg*width*.46
        torus(prefix+f' rim {sg}',(0,yy,0),radius-.14,.024,'chrome',root)
        for j in range(6):
            a=j*math.tau/6
            tube(prefix+f' spoke {sg} {j}',(0,yy,0),((radius-.15)*math.cos(a),yy,(radius-.15)*math.sin(a)),.023,'metal',root)
    tube(prefix+' hub',(0,-width*.55,0),(0,width*.55,0),.065,'metal',root)
    if args.variant!='no_mod' and prefix.startswith('rear'):
        w=next(q for q in params['wheels'] if q['name']==prefix)
        for j in range(w['blade_count']):
            phase=j*math.tau/w['blade_count']
            vv,ff=cup_mesh(w,phase)
            ob=mesh(prefix+f' paddle {j}',vv,ff,'up' if j==0 else 'wet',root)
            stamp_cup(ob,w,j)
            md=ob.modifiers.new('Cup thickness','SOLIDIFY');md.thickness=.012
            for sign in [-1,1]:
                start,end=cup_mount_points(w,phase,radius-.075,.075,sign)
                tube(prefix+f' paddle attachment {j} {sign}',start,end,.010,'wet',root,8)


def build_car():
    global bike
    remove_tree(motorcycle_root)
    bike=empty('chassis');ws={}
    M['glass']=material('Tinted automotive glass',(.018,.045,.065),.30,.14,.5)
    M['lamp']=material('Headlight glass',(.83,.94,1),.15,.15,.5)
    M['tail']=material('Tail light red',(.65,.016,.01),.25,.18,.6)
    M['float']=material('Orange pontoons',(.95,.22,.035),.25,.28,.4)
    body=car_body([(-2.05,.66,.35,.57,.67),(-1.75,.84,.31,.72,.82),(-1.25,.87,.30,.75,.85),
                   (.0,.88,.30,.77,.88),(1.20,.85,.33,.75,.86),(1.82,.76,.38,.65,.74),(2.10,.62,.44,.56,.65)],'car continuous body','orange',bike)
    # Actual wheel arch openings: one Boolean cutter per axle through both sides.
    for xx in [-1.30,1.30]:
        bpy.ops.mesh.primitive_cylinder_add(vertices=48,radius=.41,depth=3,location=(xx,0,.34),rotation=(math.pi/2,0,0))
        cutter=bpy.context.object
        md=body.modifiers.new('Wheel arch','BOOLEAN');md.operation='DIFFERENCE';md.object=cutter
        bpy.context.view_layer.objects.active=body;bpy.ops.object.modifier_apply(modifier=md.name)
        bpy.data.objects.remove(cutter,do_unlink=True)
    car_body([(-1.34,.69,.79,.86,.92),(-.79,.63,.83,1.30,1.43),(.42,.62,.83,1.32,1.45),
              (1.18,.68,.81,.84,.91)],'glazed cabin','glass',bike)
    car_body([(-.82,.63,1.38,1.42,1.45),(-.57,.64,1.42,1.46,1.49),(.40,.62,1.42,1.46,1.49),
              (.61,.59,1.35,1.39,1.42)],'roof panel','orange',bike)
    for sg in [-1,1]:
        tube(f'A pillar {sg}',(1.17,sg*.68,.86),(.44,sg*.61,1.44),.035,'orange',bike)
        tube(f'C pillar {sg}',(-1.31,sg*.69,.85),(-.78,sg*.63,1.44),.052,'orange',bike)
        tube(f'B pillar {sg}',(-.17,sg*.70,.84),(-.17,sg*.64,1.46),.038,'dark',bike)
        path(f'Shoulder trim {sg}',[(-1.52,sg*.83,.80),(-.8,sg*.88,.83),(.7,sg*.87,.83),(1.65,sg*.80,.74)],.009,'chrome',bike)
        path(f'Rocker strip {sg}',[(-.8,sg*.89,.36),(.80,sg*.89,.36)],.025,'dark',bike)
        for xx in [-.72,.40]:
            path(f'Door seam {sg} {xx}',[(xx,sg*.887,.39),(xx,sg*.887,.80)],.004,'dark',bike)
            cube(f'Door handle {sg} {xx}',(xx+.22,sg*.895,.76),(.15,.025,.025),'chrome',bike,.009)
        tube(f'Mirror arm {sg}',(.76,sg*.66,1.02),(.74,sg*.93,1.05),.026,'dark',bike)
        cube(f'Mirror {sg}',(.74,sg*1.0,1.06),(.21,.16,.11),'orange',bike,.045)
        cube(f'Headlamp {sg}',(2.035,sg*.43,.65),(.10,.31,.095),'lamp',bike,.025)
        cube(f'Tail lamp {sg}',(-2.025,sg*.48,.64),(.075,.25,.095),'tail',bike,.024)
        for xx in [-1.30,1.30]:
            prefix=('front' if xx>0 else 'rear')+('_left_wheel' if sg>0 else '_right_wheel')
            w=next(q for q in params['wheels'] if q['name']==prefix)
            loc=np.array(w['center'])+np.array(params['visual_origin_to_com'])
            r=empty(prefix,loc);r.parent=bike;ws[prefix]=r
            add_paddle_wheel(r,w['radius'],w['width'],prefix)
    cube('front grille',(2.10,0,.52),(.045,.73,.12),'dark',bike,.02)
    for sg in [-1,1]:
        path(f'Hood shut line {sg}',[(1.23,sg*.62,.865),(1.72,sg*.60,.765),(1.98,sg*.52,.687)],.004,'dark',bike)
        for z in [.485,.52,.555]:tube(f'Grille strip {sg} {z}',(2.13,sg*.05,z),(2.13,sg*.34,z),.006,'metal',bike)
        cube(f'Front intake {sg}',(2.06,sg*.50,.49),(.04,.18,.095),'dark',bike,.015)
    cube('Front plate',(2.135,0,.51),(.018,.23,.065),'cream',bike,.004)
    cube('rear diffuser',(-2.05,0,.41),(.04,.85,.10),'dark',bike,.015)
    tube('Roof aerial',(-.66,.0,1.48),(-.80,0,1.66),.008,'dark',bike)
    if args.variant in ['small_floats','large_floats']:
        for sg in [-1,1]:
            v=next(q for q in params['volumes'] if q['name']==f'side_float_{sg}')
            length,width,height=v['size'];loc=np.array(v['center'])+np.array(params['visual_origin_to_com']);yy=abs(loc[1]);zz=loc[2]
            fo=cube(f'float {sg}',loc,v['size'],'float',bike,.008)
            for xx in [-.85,.85]:
                mountz=max(.48,zz+height*.5+.04)
                tube(f'float transverse mount {sg} {xx}',(xx,sg*.74,mountz),(xx,sg*yy,mountz),.045,'metal',bike)
                tube(f'float vertical mount {sg} {xx}',(xx,sg*yy,mountz),(xx,sg*yy,zz+height*.5),.040,'metal',bike)
                tube(f'float diagonal brace {sg} {xx}',(xx,sg*.76,.40),(xx+.20,sg*yy,zz+height*.35),.024,'chrome',bike)
            for xx in [-1.2,1.2]:
                if abs(xx)<length*.45: path(f'float band {sg} {xx}',[(xx,sg*yy-width*.51,zz),(xx,sg*yy-width*.51,zz+height*.51),(xx,sg*yy+width*.51,zz+height*.51),(xx,sg*yy+width*.51,zz)],.013,'dark',bike)
    return bike,ws

if args.vehicle=='motorcycle':
    body_roots={'chassis':bike,'rear_wheel':motorcycle_pivots[0],'front_fork':steering,'front_wheel':motorcycle_pivots[1]}
    default_com=np.array([0,0,.95])
else:
    bike,ws=build_car();body_roots={'chassis':bike,**ws};default_com=np.array([0,0,.75])

cut_components=[]
if args.vehicle=='car' and args.camera in ['cutaway','paddle']:
    # The near pontoon remains a wire enclosure at its full solver dimensions.
    # This exposes the wheel inside the silhouette without changing any dynamics.
    M['outline']=material('Cutaway pontoon outline',(.95,.42,.055),.2,.35)
    for ob in list(bpy.data.objects):
        if ob.name=='float -1' or ob.name.startswith('float band -1'):
            ob.hide_render=True;cut_components.append(ob.name)
    floats=[v for v in params['volumes'] if v['name']=='side_float_-1']
    if floats:
        v=floats[0];c=np.array(v['center'])+np.array(params['visual_origin_to_com']);half=np.array(v['size'])/2
        corners=[c+half*np.array([sx,sy,sz]) for sx in [-1,1] for sy in [-1,1] for sz in [-1,1]]
        for a in range(8):
            for b in range(a+1,8):
                if bin(a^b).count('1')==1:tube(f'Float cutaway edge {a} {b}',corners[a],corners[b],.007,'outline',bike)

# Directional sunlight and a large cool bounce keep both dark tire and metal readable.
def light(name,kind,location,power,color,size=5):
    data=bpy.data.lights.new(name,kind);data.energy=power;data.color=color
    if kind=='AREA':data.shape='DISK';data.size=size;data.specular_factor=.12
    ob=bpy.data.objects.new(name,data);scene.collection.objects.link(ob);ob.location=location
    ob.rotation_euler=(Vector((0,0,.5))-ob.location).to_track_quat('-Z','Y').to_euler();return ob
sun=light('Sun','SUN',(3,-5,7),2.0 if args.look=='day' else 1.6,(1,.89,.73) if args.look=='day' else (1,.47,.21));sun.data.angle=math.radians(6)
light('Sky softbox','AREA',(-2,-4,6),350,(.49,.74,1),8)
light('Rim softbox','AREA',(1,5,4),450,(.68,.84,1),6)

watermat=material('Water surface',(.008,.065,.090),.06,.21,.25)
bs=watermat.node_tree.nodes.get('Principled BSDF');bs.inputs['IOR'].default_value=1.333
ns=watermat.node_tree.nodes;ls=watermat.node_tree.links
noise=ns.new('ShaderNodeTexNoise');noise.inputs['Scale'].default_value=5;noise.inputs['Detail'].default_value=3;noise.inputs['Roughness'].default_value=.66
tex=ns.new('ShaderNodeTexCoord');ls.new(tex.outputs['Object'],noise.inputs['Vector'])
bump=ns.new('ShaderNodeBump');bump.inputs['Strength'].default_value=.20;bump.inputs['Distance'].default_value=.025
ls.new(noise.outputs['Fac'],bump.inputs['Height']);ls.new(bump.outputs['Normal'],bs.inputs['Normal'])
M['water']=watermat
size=90
outer=[100,200,500,1500,3000]
xx=np.concatenate([-np.array(outer[::-1]),np.linspace(-size/2,size/2,241),np.array(outer)])
yy=np.concatenate([-np.array(outer[::-1]),np.linspace(-size/2,size/2,201),np.array(outer)])
nx=len(xx)-1;ny=len(yy)-1
xgrid,ygrid=np.meshgrid(xx,yy);xy=np.column_stack([xgrid.ravel(),ygrid.ravel()])
wv=np.zeros((len(xy),3));wv[:,:2]=xy
wf=[]
for j in range(ny):
    for i in range(nx):
        if args.camera in ['cutaway','paddle'] and yy[j]<0:continue
        a=j*(nx+1)+i;wf.append((a,a+1,a+nx+2,a+nx+1))
water=mesh('Airy water surface',wv.tolist(),wf,'water');water.data.polygons.foreach_set('use_smooth',[True]*len(wf))
M['deep']=material('Deep water',(.008,.047,.065),.1,.3)
depthwater=cube('Water depth',(0,0,-6),(200,200,.2),'deep',None,0)
if args.transparent:water.hide_render=True;depthwater.hide_render=True
sectionwater=None;sectionline=None
if args.camera in ['cutaway','paddle']:
    depthwater.hide_render=True
    M['sectionwater']=material('Water cut section',(.018,.12,.17),0,.72)
    secnodes=M['sectionwater'].node_tree.nodes;secnodes.clear()
    secbs=secnodes.new('ShaderNodeEmission');secbs.inputs['Color'].default_value=(.018,.12,.17,1);secbs.inputs['Strength'].default_value=.7
    secout=secnodes.new('ShaderNodeOutputMaterial');M['sectionwater'].node_tree.links.new(secbs.outputs[0],secout.inputs['Surface'])
    sv=[];sf=[]
    for x in np.linspace(-50,50,201):sv.extend([(x,0,-30),(x,0,0)])
    for j in range(200):sf.append((2*j,2*j+2,2*j+3,2*j+1))
    sectionwater=mesh('Water section',sv,sf,'sectionwater')
    sectionline=path('Water line',[(x,0,.004) for x in np.linspace(-50,50,201)],.006,'flow')

bpy.ops.object.camera_add();cam=bpy.context.object;scene.camera=cam;cam.data.lens=50
cam.data.clip_end=5000;cam.data.clip_start=.05
run_data=None;metadata={};geometry_bindings=[]
wave=params.get('water',{'mean_z':0,'components':[]})

# Convert original exterior to the model COM coordinate system, then replace all
# water-contacting boards with the exact declared solver dimensions and centers.
com=np.array(params.get('visual_origin_to_com',default_com),dtype=float)
for ob in list(bike.children):ob.location-=Vector(com)
bpy.context.view_layer.update()
if args.vehicle=='motorcycle':
    for ob in list(bpy.data.objects):
        if ob.name.startswith(('Front board','Rear board')):bpy.data.objects.remove(ob,do_unlink=True)
    # The steering body rotates around its solver pivot; preserve rest assembly.
    mats={ch:ch.matrix_world.copy() for ch in list(steering.children)}
    steering.location=params['front_fork']['center']
    bpy.context.view_layer.update()
    for ch,matrix in mats.items():ch.matrix_world=matrix
    for v in params['volumes']:
        if '_ski_solid_' not in v['name'] and not v['name'].startswith('reserve_float_'):continue
        owner=body_roots[v['body']]
        o=cube(v['name'],v['center'],v['size'],'board',owner,.003)
        o.rotation_euler[1]=-math.radians(v.get('pitch_deg',0))
        x,y,z=v['center']
        axle=(-.86,math.copysign(.11,y),-.455) if v['body']=='chassis' else (0,math.copysign(.09,y),0)
        tube(v['name']+' bracket',axle,(x,y,z+.03),.018,'metal',owner)
    for w in params['wheels']:
        ob=body_roots[w['name']]
        ob.location=w['center']
        # All wheel detail uses the same local axle center and spin transform.
        old_radius=.395 if w['name']=='rear_wheel' else .425
        for ch in list(ob.children):
            ch.location*=w['radius']/old_radius
            ch.scale*=w['radius']/old_radius
bpy.context.view_layer.update()
# Static pose places the COM exactly as the model's initial condition.
bike.location=(0,0,float(params.get('initial_z',com[2])))
bpy.context.view_layer.update()
if args.run:
    rp=args.run.resolve()
    trajectory_bytes=(rp/'trajectory.npz').read_bytes()
    trajectory_sha256=hashlib.sha256(trajectory_bytes).hexdigest()
    run_data=np.load(io.BytesIO(trajectory_bytes))
    metadata=json.loads((rp/'metadata.json').read_text(encoding='utf-8'))
    required=['t','body_pos','body_quat']
    if any(k not in run_data for k in required):raise ValueError('Trajectory lacks required rigid body fields')
    if len(run_data['t'])<2:raise ValueError('Trajectory has fewer than two samples')
    requested_end=args.start_time+(max(1,round(args.duration*24))-1)/24*args.time_scale
    if args.render and requested_end>run_data['t'][-1]+1e-6:
        raise ValueError(f'Requested clip ends at {requested_end:.3f}s but solver trajectory ends at {run_data["t"][-1]:.3f}s')
    names=metadata['body_names']
    if 'chassis' not in names:raise ValueError('body_names must include chassis')
    wave=params.get('water',params.get('waves',wave))
    for name,ob in body_roots.items():
        if name=='chassis':continue
        # Imported body states are world transforms, so wheel roots are not parented to chassis.
        matrix=ob.matrix_world.copy();ob.parent=None;ob.matrix_world=matrix
    for name in names:
        if name in body_roots:geometry_bindings.append((names.index(name),body_roots[name]))
    missing=[n for n in body_roots if n not in names]
    if missing:raise ValueError('Trajectory body names missing rendered objects: '+str(missing))
elif args.render:
    raise ValueError('Animation render requires a solver trajectory --run')

def eta(x,y,t):
    out=np.zeros_like(x,dtype=float)+float(wave.get('mean_z',0))
    for c in wave.get('components',[]):
        out+=float(c['amplitude'])*np.cos(float(c['kx'])*x+float(c['ky'])*y-float(c['omega'])*t+float(c.get('phase',0)))
    return out

frame_state_records={};anchor_records={};motion_sample_times=set()
def update(sc):
    t=args.start_time+(sc.frame_current+sc.frame_subframe-1)/24*args.time_scale
    center=np.array([0.,0.,.65 if args.vehicle=='motorcycle' else .45])
    if run_data is not None:
        tt=run_data['t'];t=float(np.clip(t,tt[0],tt[-1]));idx=int(np.clip(np.searchsorted(tt,t)-1,0,len(tt)-2));u=float((t-tt[idx])/(tt[idx+1]-tt[idx]))
        motion_sample_times.add((int(sc.frame_current),float(sc.frame_subframe),float(t)))
        for bi,ob in geometry_bindings:
            ob.location=(1-u)*run_data['body_pos'][idx,bi]+u*run_data['body_pos'][idx+1,bi]
            ob.rotation_mode='QUATERNION';ob.rotation_quaternion=Quaternion(run_data['body_quat'][idx,bi]).slerp(Quaternion(run_data['body_quat'][idx+1,bi]),u)
        center=np.array(bike.location)
        if args.camera not in ['cutaway','paddle']:center[2]=max(center[2],.15)
        if sc.frame_subframe==0:
            frame_state_records[int(sc.frame_current)]={'frame':int(sc.frame_current),'simulation_time_s':t,
                'bodies':{name:{'position':list(ob.location),'quaternion_wxyz':list(ob.rotation_quaternion)} for name,ob in body_roots.items()}}
    # Water grid follows camera in XY, but phase always uses world coordinates.
    wv[:,0]=xy[:,0]+center[0];wv[:,1]=xy[:,1]+center[1];wv[:,2]=eta(wv[:,0],wv[:,1],t)
    water.data.vertices.foreach_set('co',wv.ravel());water.data.update()
    if sectionwater is not None:
        secxs=np.linspace(center[0]-50,center[0]+50,201);secys=np.full(201,center[1])
        seczs=eta(secxs,secys,t);sv=np.zeros((402,3))
        sv[::2,0]=secxs;sv[1::2,0]=secxs;sv[:,1]=center[1]
        sv[::2,2]=-30;sv[1::2,2]=seczs
        sectionwater.data.vertices.foreach_set('co',sv.ravel());sectionwater.data.update()
        sp=sectionline.data.splines[0]
        for pt,x,y,z in zip(sp.points,secxs,secys,seczs):pt.co=(x,y-.002,z+.004,1)
    scale=1 if args.vehicle=='motorcycle' else 1.55
    offsets={'front':(4.0,-6.0,2.7),'side':(.15,-7.3,1.8),'rear':(-4.2,-6.0,2.4),'high':(4.5,-6.5,5.3),'wide':(6,-9,3.4),
             'bow':(6.5,0,1.0),'top':(0,-.01,7.5),'waterline':(.05,-6.4,-.45),'paddle':(-2.7,-3.0,-.25),'cutaway':(.05,-6.4,.50)}
    off=np.array(offsets[args.camera])*scale
    cam.location=center+off;target=center+np.array([0,0,.20 if args.vehicle=='motorcycle' else .05])
    if args.camera=='paddle':target=center+np.array([-.80,0,-.36]);cam.data.lens=65
    if args.camera=='waterline':target=center+np.array([0,0,-.40]);cam.data.lens=50
    if args.camera=='cutaway':target=center+np.array([0,0,.10]);cam.data.lens=45
    cam.rotation_euler=(Vector(target)-cam.location).to_track_quat('-Z','Y').to_euler()
    cam.matrix_world=Matrix.Translation(cam.location)@cam.rotation_euler.to_matrix().to_4x4()
    if run_data is not None and sc.frame_subframe==0:
        def pixel(point):
            pp=world_to_camera_view(scene,cam,Vector(point))
            return [float(pp.x*scene.render.resolution_x),float((1-pp.y)*scene.render.resolution_y),float(pp.z)]
        rear_name='rear_wheel' if 'rear_wheel' in body_roots else 'rear_right_wheel'
        points={'chassis':np.array(bike.location),'rear_wheel':np.array(body_roots[rear_name].location)}
        anchors={k:{'body_name':'chassis' if k=='chassis' else rear_name,'world':v.tolist(),'pixel_xy_depth':pixel(v)} for k,v in points.items()}
        actual=(1-u)*run_data['force'][idx]+u*run_data['force'][idx+1]
        groups=metadata['force_groups'];forces={}
        for key in ['buoyancy','panel_pressure','paddles','gravity']:
            f=np.array([0,0,-metadata['total_mass_kg']*wave.get('gravity',9.81)]) if key=='gravity' else actual[groups.index(key)]
            anchor='rear_wheel' if key=='paddles' else 'chassis';mag=float(np.linalg.norm(f));direction=f/max(mag,1e-12)
            forces[key]={'anchor':anchor,'force_world_N':f.tolist(),'magnitude_N':mag,'unit_endpoint_pixel_xy_depth':pixel(points[anchor]+direction)}
        anchor_records[int(sc.frame_current)]={'frame':int(sc.frame_current),'simulation_time_s':t,
            'trajectory_sha256':trajectory_sha256,'anchors':anchors,'forces':forces}

scene.frame_start=1;scene.frame_end=max(1,round(args.duration*24))
bpy.app.handlers.frame_change_pre.clear();bpy.app.handlers.frame_change_pre.append(update)
scene.frame_set(args.frame)
out=HERE/args.out;out.parent.mkdir(parents=True,exist_ok=True)
if args.save_blend:bpy.ops.wm.save_as_mainfile(filepath=str(out.with_suffix('.blend')))
if args.preview:
    scene.render.filepath=str(out.with_suffix('.png'));bpy.ops.render.render(write_still=True)
if args.render:
    out.mkdir(parents=True,exist_ok=True)
    frame_times=[]
    if args.benchmark_frames:
        scene.frame_end=args.benchmark_frames
        for f in range(1,args.benchmark_frames+1):
            started_frame=time.perf_counter();scene.frame_set(f)
            scene.render.filepath=str(out/f'frame_{f:04d}.png');bpy.ops.render.render(write_still=True)
            frame_times.append(time.perf_counter()-started_frame)
            print(f'BENCH_FRAME {f} {frame_times[-1]:.4f}',flush=True)
    else:
        scene.render.filepath=str(out/'frame_');bpy.ops.render.render(animation=True)
receipt={'vehicle':args.vehicle,'variant':args.variant,'look':args.look,'run':str(args.run) if args.run else None,
         'motion_source':'MuJoCo trajectory body transforms' if args.run else 'static assembly pose',
         'width':args.width,'fps':24,'samples':args.samples,'engine':'BLENDER_EEVEE','wall_s':time.time()-STARTED,
         'wave_parameters':wave,'rendered_frame_count':scene.frame_end if args.render else (1 if args.preview else 0),
         'camera':args.camera,'time_scale':args.time_scale,'start_time_s':args.start_time,
         'coordinate_system':'+X forward, +Y left, +Z up; quaternions wxyz',
         'body_bindings':list(body_roots),'component_geometry_source':str(params_path),
         'geometry_params_sha256':params_sha256,
         'cutaway_components':cut_components,
         'paddle_geometry_sha256':hashlib.sha256((HERE/'paddle_geometry.py').read_bytes()).hexdigest(),
         'motion_blur':{'enabled':scene.render.use_motion_blur,'shutter_frames':scene.render.motion_blur_shutter,'steps':scene.eevee.motion_blur_steps,'evaluated_times':sorted(motion_sample_times)},
         'render_code_sha256':render_source_sha256}
if args.run:receipt['trajectory_sha256']=trajectory_sha256
if args.run:
    state_path=out.with_suffix('.frames.json')
    state_path.write_text(json.dumps(list(frame_state_records.values()),ensure_ascii=False,indent=2),encoding='utf8')
    receipt['rendered_body_states']=str(state_path)
    anchor_path=out.with_suffix('.anchors.json')
    anchor_path.write_text(json.dumps({'resolution':[args.width,round(args.width*9/16)],'coordinate_origin':'top left',
        'direction_endpoint_distance_m':1,'trajectory_sha256':trajectory_sha256,'params_sha256':params_sha256,
        'frames':list(anchor_records.values())},ensure_ascii=False,indent=2),encoding='utf8')
    receipt['overlay_anchors']=str(anchor_path)
if args.benchmark_frames:
    receipt['frame_wall_s']=frame_times
    receipt['first_frame_s']=frame_times[0];receipt['subsequent_mean_s']=float(np.mean(frame_times[1:]))
out.with_suffix('.render.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(receipt,ensure_ascii=False))
