# Original authored motorcycle geometry, rebuilt with physically based materials.
# Geometry only: no animation or prescribed motion.
objects = {}
def finish(obj, name, material, parent=None, bevel=0):
    obj.name=name; obj.data.materials.append(M[material]); objects[name]=obj
    if parent: obj.parent=parent
    if bevel:
        mod=obj.modifiers.new('Soft manufactured edges','BEVEL'); mod.width=bevel; mod.segments=3
    if obj.type=='MESH':
        for poly in obj.data.polygons: poly.use_smooth=True
    return obj
def empty(name, at=(0,0,0)):
    o=bpy.data.objects.new(name,None); bpy.context.collection.objects.link(o); o.location=at; return o
bike=empty('chassis')
def cube(name, at, scale, material, parent=None, bevel=.02):
    bpy.ops.mesh.primitive_cube_add(size=1,location=at); o=bpy.context.object; o.dimensions=scale
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    return finish(o,name,material,parent,bevel)
def tube(name,a,b,r,material,parent=None,vertices=16):
    a,b=Vector(a),Vector(b); d=b-a
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices,radius=r,depth=d.length,location=(a+b)/2)
    o=bpy.context.object; o.rotation_euler=d.to_track_quat('Z','Y').to_euler()
    return finish(o,name,material,parent,.004)
def path(name, points, radius, material, parent=None):
    cu=bpy.data.curves.new(name,'CURVE'); cu.dimensions='3D'; cu.bevel_depth=radius; cu.bevel_resolution=2
    sp=cu.splines.new('POLY'); sp.points.add(len(points)-1)
    for p,v in zip(sp.points,points): p.co=(*v,1)
    o=bpy.data.objects.new(name,cu); bpy.context.collection.objects.link(o)
    return finish(o,name,material,parent)
def torus(name,at,major,minor,material,parent=None):
    bpy.ops.mesh.primitive_torus_add(major_segments=48,minor_segments=12,location=at,major_radius=major,minor_radius=minor,rotation=(math.pi/2,0,0))
    return finish(bpy.context.object,name,material,parent)
def mesh(name,verts,faces,material,parent=None,bevel=0):
    me=bpy.data.meshes.new(name); me.from_pydata(verts,[],faces); me.update()
    o=bpy.data.objects.new(name,me); bpy.context.collection.objects.link(o)
    return finish(o,name,material,parent,bevel)
def hull(name, stations, material):
    verts=[]
    for x,w,b,t in stations: verts += [(x,-w,b),(x,w,b),(x,w,t),(x,-w,t)]
    faces=[(3,2,1,0)]
    for i in range(len(stations)-1):
        for j in range(4): faces.append((i*4+j,i*4+(j+1)%4,(i+1)*4+(j+1)%4,(i+1)*4+j))
    n=(len(stations)-1)*4; faces.append((n,n+1,n+2,n+3))
    return mesh(name,verts,faces,material,bike,.035)

# Motorcycle proportions and all geometry below are original explanatory choices.
# +X forward, +Z up, Y is the axle direction; water lies approximately at Z=0.
pivots=[]
rear_paddles=[]
for front,wx,wz,r in [(False,-.86,.405,.395),(True,.91,.445,.425)]:
    prefix='Front' if front else 'Rear'
    pivot=empty(prefix+' wheel pivot',(wx,0,wz)); pivot.parent=bike; pivots.append(pivot)
    torus(prefix+' tire',(0,0,0),r-.080,.080 if front else .096,'black',pivot)
    torus(prefix+' rim',(0,0,0),r-.158,.019,'chrome',pivot)
    tube(prefix+' hub',(0,-.12,0),(0,.12,0),.058,'metal',pivot)
    for i in range(16):
        a=i*math.tau/16
        tube(prefix+f' spoke {i}',(0,-.025,0),((r-.158)*math.cos(a),-.025,(r-.158)*math.sin(a)),.008,'chrome',pivot,8)
    if front:
        for i in range(30):
            a=i*math.tau/30
            o=cube(f'Front tread {i}',(r*math.cos(a),0,r*math.sin(a)),(.055,.16,.030),'black',pivot,.007)
            o.rotation_euler[1]=math.pi/2-a
    else:
        wheel_spec=next((w for w in params['wheels'] if w['name']=='rear_wheel'),None)
        for i in range(wheel_spec['blade_count'] if wheel_spec else 0):
            phase=i*math.tau/wheel_spec['blade_count']
            verts,faces=cup_mesh(wheel_spec,phase)
            o=mesh(f'Rear cupped paddle {i}',verts,faces,'up' if i==0 else 'dark',pivot)
            stamp_cup(o,wheel_spec,i)
            sol=o.modifiers.new('Rubber scoop thickness','SOLIDIFY');sol.thickness=.012
            for sign in [-1,1]:
                start,end=cup_mount_points(wheel_spec,phase,r-.080,.096,sign)
                tube(f'Paddle attachment {i} {sign}',start,end,.010,'dark',pivot,8)
            rear_paddles.append(o)
    tube(prefix+' axle',(wx,-.18,wz),(wx,.18,wz),.025,'chrome',bike)

for y in [-.13,.13]:
    joints=[(-.78,y,.75),(-.20,y,.43),(.53,y,.75),(.60,y,1.20),(-.25,y,1.09),(-.78,y,.75)]
    path(f'Continuous frame {y}',joints,.028,'orange',bike)
    tube(f'Subframe {y}',(-.78,y,.75),(-.95,y,1.15),.023,'metal',bike)
    tube(f'Swingarm {y}',(-.21,y,.58),(-.86,y,.405),.032,'metal',bike)
    tube(f'Front fork upper {y}',(.60,y,1.25),(.75,y,.84),.039,'dark',bike)
    tube(f'Front fork lower {y}',(.75,y,.86),(.91,y,.445),.026,'chrome',bike)
    tube(f'Footrest {y}',(-.25,y,.56),(-.25,y*2.5,.56),.025,'black',bike)
cube('Engine body',(-.03,0,.72),(.40,.34,.38),'metal',bike,.075)
tube('Engine round cover',(-.13,-.205,.68),(-.13,.205,.68),.16,'chrome',bike,32)
cube('Engine cylinder',(.10,0,.89),(.25,.28,.26),'dark',bike,.035)
cube('Radiator',(.39,0,.94),(.08,.36,.31),'metal',bike,.015)
for z in [.84,.90,.96,1.02]: tube(f'Radiator slat {z}',(.44,-.18,z),(.44,.18,z),.012,'dark',bike)
hull('Tank and front fairing',[(-.27,.12,1.02,1.23),(.04,.20,1.02,1.32),(.32,.18,.99,1.30),(.51,.09,1.04,1.16)],'orange')
hull('Long narrow seat',[(-.96,.09,1.11,1.20),(-.73,.14,1.16,1.26),(-.29,.14,1.17,1.28),(-.10,.09,1.20,1.27)],'dark')
hull('Rear fender',[(-1.20,.15,1.13,1.18),(-.95,.17,1.13,1.18),(-.70,.12,1.11,1.16)],'cream')
hull('Front fender',[(.45,.12,.98,1.02),(.75,.17,1.03,1.07),(1.10,.16,1.02,1.06),(1.39,.10,.94,.98)],'orange')
for y in [-.205,.205]:
    o=cube(f'Side number panel {y}',(-.53,y,1.01),(.40,.035,.24),'cream',bike,.04); o.rotation_euler[1]=-.15
tube('Steering stem',(.59,0,1.22),(.55,0,1.43),.038,'metal',bike)
path('Handlebar',[(.55,-.37,1.40),(.52,-.21,1.46),(.55,0,1.43),(.52,.21,1.46),(.55,.37,1.40)],.023,'chrome',bike)
for y in [-1,1]: tube(f'Handlebar grip {y}',(.55,y*.27,1.41),(.55,y*.41,1.39),.032,'dark',bike)
o=cube('Front number plate',(.69,0,1.22),(.035,.28,.30),'cream',bike,.035); o.rotation_euler[1]=-.24
path('Exhaust pipe',[(.15,.18,.79),(.30,.24,.72),(.17,.26,.56),(-.35,.24,.67),(-.64,.22,1.02)],.035,'metal',bike)
tube('Silencer',(-.63,.22,1.02),(-1.04,.22,1.08),.067,'chrome',bike,24)
path('Chain loop',[(-.21,-.185,.61),(-.85,-.185,.47),(-1.00,-.185,.40),(-.87,-.185,.32),(-.19,-.185,.48),(-.12,-.185,.54),(-.21,-.185,.61)],.012,'dark',bike)
for i in range(7):
    z=.70+i*.042
    torus(f'Rear shock spring {i}',(-.47,0,z),.055,.011,'up',bike).rotation_euler=(0,0,0)

# Two generic slotted planing boards; wheel openings preserve rear wheel contact.
boards=[]
def board_strip(name,x0,x1,y0,y1):
    xs=[x0,x0+(x1-x0)*.65,x0+(x1-x0)*.85,x1]
    zs=[.015,.025,.075,.17]
    verts=[]
    for x,z in zip(xs,zs): verts += [(x,y0,z),(x,y1,z),(x,y1,z-.035),(x,y0,z-.035)]
    faces=[(3,2,1,0)]
    for i in range(3):
        for j in range(4): faces.append((i*4+j,i*4+(j+1)%4,(i+1)*4+(j+1)%4,(i+1)*4+j))
    faces.append((12,13,14,15))
    o=mesh(name,verts,faces,'board',bike,.012); boards.append(o)
for prefix,x0,x1,half,slot in [('Front',.27,1.65,.39,.14),('Rear',-1.52,-.28,.42,.185)]:
    board_strip(prefix+' board left',x0,x1,slot,half)
    board_strip(prefix+' board right',x0,x1,-half,-slot)
    # Crosspiece only ahead of the tire; the rear paddle wheel remains exposed to water.
    cross_start=1.30 if prefix=='Front' else -.48
    board_strip(prefix+' board forward bridge',cross_start,x1,-slot,slot)
    axle=.91 if prefix=='Front' else -.86
    for sign in [-1,1]:
        tube(prefix+f' board bracket {sign}',(axle,sign*.13,.43),(axle-.10,sign*(half-.06),.04),.018,'metal',bike)

# Additional detailed rider, generated from original local geometry, not a scan.
M['suit']=mat('Rider | cobalt textile',(.055,.22,.48))
M['red']=mat('Rider | vermilion',(.88,.095,.055))
M['visor']=mat('Helmet | reflective blue',(.035,.075,.12))
M['white']=mat('Helmet | pearl',(.9,.94,.97))

def sphere(name,at,scale,material,parent=bike):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=24,ring_count=16,location=at)
    o=bpy.context.object; o.scale=scale
    return finish(o,name,material,parent)
# Seated forward posture: hips, knees, ankles, elbows and hands form continuous limbs.
sphere('Rider pelvis',(-.49,0,1.34),(.20,.18,.17),'suit')
o=sphere('Rider torso',(-.28,0,1.60),(.20,.205,.35),'suit'); o.rotation_euler[1]=.43
sphere('Rider back armour',(-.40,.02,1.70),(.095,.19,.25),'dark')
for sg in [-1,1]:
    y=.19*sg
    tube(f'Rider thigh {sg}',(-.48,y,1.35),(-.05,sg*.23,1.05),.105,'suit',bike)
    sphere(f'Rider knee {sg}',(-.05,sg*.23,1.05),(.11,.10,.11),'red')
    tube(f'Rider shin {sg}',(-.05,sg*.23,1.05),(-.26,sg*.27,.65),.075,'suit',bike)
    cube(f'Rider boot {sg}',(-.16,sg*.27,.61),(.28,.13,.15),'white',bike,.045)
    tube(f'Rider upper arm {sg}',(-.15,sg*.20,1.80),(.15,sg*.32,1.56),.075,'red',bike)
    sphere(f'Rider elbow {sg}',(.15,sg*.32,1.56),(.08,.08,.08),'dark')
    tube(f'Rider forearm {sg}',(.15,sg*.32,1.56),(.54,sg*.32,1.42),.063,'suit',bike)
    sphere(f'Rider glove {sg}',(.54,sg*.32,1.42),(.075,.055,.06),'dark')
sphere('Rider neck',(-.08,0,1.91),(.065,.065,.10),'dark')
sphere('Rider helmet',(.005,0,2.02),(.17,.16,.19),'white')
sphere('Rider visor',(.135,0,2.045),(.08,.151,.095),'visor')
cube('Helmet chin guard',(.115,0,1.925),(.19,.25,.07),'red',bike,.025)
cube('Helmet peak',(.12,0,2.17),(.33,.32,.025),'white',bike,.015)
# Cable and mechanical details, all author-chosen.
for sg in [-1,1]:
    path(f'Control cable {sg}',[(.53,.3*sg,1.42),(.65,.22*sg,1.26),(.67,.15*sg,.86),(.91,.15*sg,.47)],.008,'black',bike)
for i in range(6): cube(f'Engine cooling fin {i}',(.08,0,.81+i*.035),(.27,.32,.012),'chrome',bike,.003)
# Rear wheel uses paddles, broad scoops rather than road tread; selected geometry remains conceptual.
# Steerable fork + front ski, independent of wheel spin. Interview source verifies linkage.
steering=empty('Front steering assembly',(.60,0,1.25)); steering.parent=bike
for ob in list(bike.children):
    if ob==steering: continue
    if ob.name.startswith(('Front fork','Front board','Front axle','Front fender','Front number')) or ob==pivots[1]:
        ob.parent=steering; ob.location-=steering.location

