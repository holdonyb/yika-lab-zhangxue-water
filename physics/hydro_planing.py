"""Experimental planing branch; frozen baseline remains in hydro.py.

Tagged bottom surfaces use the Savitsky dynamic term and front-loaded pressure.
Geometric static buoyancy is separate. This hybrid quasi-steady model is not
the complete Savitsky hull model or a calibrated real-car prediction.

No vehicle pose is prescribed after initial conditions. External water loads are
evaluated from the current state and applied as world wrenches at each body's COM.
The fluid is a documented engineering closure, not a Navier-Stokes computation.
"""
from pathlib import Path
from copy import deepcopy
import argparse, hashlib, json, math, time
import numpy as np
import mujoco
from clipped_box import clip_cells
from planing_surface import dynamic_normal_forces

ROOT=Path(__file__).resolve().parent
G=9.81
GROUPS=('buoyancy','wave_pressure','volume_drag','panel_pressure','skin_drag','paddles','air_drag')

def ry(a):
    c,s=np.cos(a),np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def wave_spec(kind='calm',seed=17,density=1000.):
    components=[]
    def add(a,wavelength,direction,phase=0):
        k=2*np.pi/wavelength;ang=np.deg2rad(direction)
        components.append(dict(amplitude=a,kx=k*np.cos(ang),ky=k*np.sin(ang),omega=np.sqrt(G*k),phase=phase))
    if kind=='head':add(.075,4.,180)
    elif kind=='cross':add(.075,4.,90)
    elif kind=='swell':add(.075,22.,180)
    elif kind=='irregular':
        rng=np.random.default_rng(seed)
        for a,L,d in zip([.035,.055,.040,.025,.018],[3.1,5.3,9.2,15.,24.],[160,195,95,170,70]):add(a,L,d,float(rng.uniform(0,2*np.pi)))
    return dict(kind=kind,mean_z=0.,density=density,current=[0.,0.,0.],components=components,seed=seed,gravity=G)

class Water:
    def __init__(self,spec):
        self.spec=spec;self.rho=spec['density'];self.current=np.array(spec['current'],float)
        cs=spec['components'];self.a=np.array([c['amplitude'] for c in cs]);self.kx=np.array([c['kx'] for c in cs]);self.ky=np.array([c['ky'] for c in cs]);self.omega=np.array([c['omega'] for c in cs]);self.phase=np.array([c['phase'] for c in cs]);self.k=np.hypot(self.kx,self.ky)
    def evaluate(self,p,t):
        p=np.asarray(p);n=len(p);eta=np.full(n,self.spec['mean_z']);u=np.tile(self.current,(n,1));acc=np.zeros((n,3))
        if not len(self.a):return eta,u,acc
        theta=p[:,0,None]*self.kx+p[:,1,None]*self.ky-t*self.omega+self.phase
        co,si=np.cos(theta),np.sin(theta)
        eta+=co@self.a
        # Linear deep-water field, continued to mean level for points above it.
        e=np.exp(np.minimum(p[:,2,None]-self.spec['mean_z'],0)*self.k)
        horiz=e*co*(self.a*self.omega)
        u[:,0]+=horiz@(self.kx/self.k);u[:,1]+=horiz@(self.ky/self.k);u[:,2]+=(e*si)@(self.a*self.omega)
        horiz_acc=e*si*(self.a*self.omega**2)
        acc[:,0]+=horiz_acc@(self.kx/self.k);acc[:,1]+=horiz_acc@(self.ky/self.k);acc[:,2]-=(e*co)@(self.a*self.omega**2)
        return eta,u,acc

def box(name,center,size,body='chassis',cd=(.8,1.,1.),grid=(8,4,4),pitch_deg=0):
    return dict(name=name,body=body,center=center,size=size,cd=list(cd),grid=list(grid),pitch_deg=pitch_deg)

def panel(name,center,size,body='chassis',pitch_deg=6,grid=(16,4),cn=2.,cf=.008,thickness=.028):
    return dict(name=name,body=body,center=center,size=size,pitch_deg=pitch_deg,grid=list(grid),cn=cn,cf=cf,wet_transition_m=.012,thickness=thickness)

def tire_volume(w):
    ro=w['radius']-.03;ri=w['radius']-.11;width=w['width']
    return dict(name=w['name']+'_sealed',body=w['name'],center=[0,0,0],size=[2*ro,width,2*ro],shape='annular_cylinder',outer_radius=ro,inner_radius=ri,width=width,cd=[.8,1.,.8],grid=[32,2,2],pitch_deg=0.)

def volume_of(v):
    return float(np.pi*(v['outer_radius']**2-v['inner_radius']**2)*v['width']) if v.get('shape')=='annular_cylinder' else float(np.prod(v['size']))

def add_symmetric_float_mass(p,total_mass,size,y,z):
    """Two axis-aligned equal float masses; exact parallel-axis assembly."""
    old_mass=p['chassis_mass'];new_mass=old_mass+total_mass;cz=total_mass*z/new_mass
    sx,sy,sz=size
    own=np.array([sy*sy+sz*sz,sx*sx+sz*sz,sx*sx+sy*sy])*total_mass/12
    offset=np.array([y*y+(z-cz)**2,(z-cz)**2,y*y])*total_mass
    old_offset=np.array([cz*cz,cz*cz,0.])*old_mass
    p['chassis_inertia']=(np.array(p['chassis_inertia'])+own+offset+old_offset).tolist()
    p['chassis_mass']=new_mass;p['chassis_com']=[0.,0.,cz]
    p['float_mass_kg']=total_mass

def parameters(vehicle='bike',variant='base',water='calm'):
    p=dict(vehicle=vehicle,variant=variant,water=wave_spec(water),duration=12.,dt=.001,output_hz=48.,initial_speed=9.,initial_roll_deg=0.,initial_pitch_deg=0.,initial_z=.86,drive_off_s=None,steer_target_deg=0.,steer_start_s=0.,parameter_origin='declared experimental model, not a measurement of the news vehicle',volumes=[],panels=[],wheels=[])
    if vehicle=='bike':
        p.update(chassis_mass=220.,chassis_inertia=[65.,110.,85.],visual_origin_to_com=[0,0,.86],drive_torque_nm=300.,drive_power_w=15000.,wheel_target_rad_s=36.)
        p['wheels']=[dict(name='rear_wheel',parent='chassis',center=[-.86,0,-.455],radius=.395,width=.22,mass=12.,inertia=[.70,1.10,.70],paddles=True,blade_area=.038,blade_count=12,blade_radius=.46,blade_depth=.09),dict(name='front_wheel',parent='front_fork',center=[0,0,0],radius=.425,width=.16,mass=8.,inertia=[.50,.9,.50],paddles=False)]
        p['front_fork']=dict(center=[.91,0,-.415],mass=2.)
        # Non-overlapping equivalent watertight volumes for the experimental body.
        p['volumes']=[box('engine_and_structure',[-.12,0,-.16],[.65,.36,.28]),box('rider_effective',[0,0,.48],[.32,.34,.55])]+[tire_volume(w) for w in p['wheels']]
        for x,body in [(-.78,'chassis'),(0.,'front_fork')]:
            z=-.82 if body=='chassis' else -.405
            for side in [-1,1]:
                y=side*(.34 if body=='chassis' else .20);center=[x,y,z]
                p['panels'].append(panel(f'{body}_ski_{side}',center,[1.12,.22],body))
                solid_center=(np.array(center)+ry(-np.deg2rad(6))[:,2]*.014).tolist()
                p['volumes'].append(box(f'{body}_ski_solid_{side}',solid_center,[1.12,.22,.028],body,cd=(.7,.9,0.),grid=(12,3,2),pitch_deg=6))
        if variant=='low_torque':p['drive_torque_nm']=145.
        if variant=='light':p['chassis_mass']*=.8;p['chassis_inertia']=[a*.8 for a in p['chassis_inertia']]
        if variant=='wide':
            for q in p['panels']:q['size'][1]*=1.7;q['center'][1]*=1.5
            for q in p['volumes']:
                if '_ski_' in q['name']:q['size'][1]*=1.7;q['center'][1]*=1.5
            p['chassis_mass']+=6.
        if variant=='buoyant':
            for side in [-1,1]:p['volumes'].append(box(f'reserve_float_{side}',[0,side*.52,-.60],[1.45,.28,.32],cd=(.3,.8,.9),grid=(12,4,5)))
            add_symmetric_float_mass(p,10.,[1.45,.28,.32],.52,-.60)
    else:
        p.update(chassis_mass=1352.,chassis_inertia=[650.,2100.,2250.],visual_origin_to_com=[0,0,.78],drive_torque_nm=260.,drive_power_w=22000.,wheel_target_rad_s=29.,initial_speed=5.,initial_z=.82)
        for rear,x in [(True,-1.30),(False,1.30)]:
            for side,y in [('left',.78),('right',-.78)]:
                name=('rear_' if rear else 'front_')+side+'_wheel'
                p['wheels'].append(dict(name=name,parent='chassis',center=[x,y,-.39],radius=.34,width=.22,mass=12.,inertia=[.52,.82,.52],paddles=rear and variant!='no_mod',blade_area=.034,blade_width=.34,blade_count=12,blade_radius=.405,blade_depth=.10))
                p['volumes'].append(tire_volume(p['wheels'][-1]))
        # An open/flooded passenger cabin is assumed in every car variant.
        p['cabin_condition']='open/flooded cabin; only listed solid/sealed components displace water'
        p['volumes'].append(box('drivetrain_solids',[0,0,-.05],[1.30,.55,.21],cd=(.85,1.,1.),grid=(12,5,3)))
        p['panels'].append(panel('underbody',[0,0,-.48],[3.3,1.4],pitch_deg=0,grid=(20,8),cn=1.5,cf=.012))
        if variant!='no_mod':
            for w in p['wheels']:
                if w['paddles']:
                    w['mass']+=12.
                    w['inertia']=(np.array(w['inertia'])+np.array([.5,1.,.5])*12*w['blade_radius']**2).tolist()
        if variant in ['small_floats','large_floats']:
            large=variant=='large_floats';size=[3.7,.52,.68] if large else [3.,.25,.30]
            add_symmetric_float_mass(p,120. if large else 36.,size,1.30,-.40)
            for side in [-1,1]:
                p['volumes'].append(box(f'side_float_{side}',[0,side*1.30,-.40],size,cd=(.18,.85,0.),grid=(20,5,6)))
                p['panels'].append(panel(f'float_bottom_{side}',[0,side*1.30,-.40-size[2]/2],[size[0],size[1]],pitch_deg=0,grid=(20,5),cn=1.6,cf=.006,thickness=size[2]))
    p['drive_rating_scope']='per driven wheel; total budget is per-wheel rating times driven wheel count'
    p.setdefault('chassis_com',[0.,0.,0.])
    p['reference_frame_note']='Free body origin is a fixed geometry reference; chassis_com is its physical mass-center offset. Modifications include their mass and parallel-axis inertia.'
    p['total_drive_power_w']=p['drive_power_w']*sum(w['paddles'] for w in p['wheels'])
    return p

def model_xml(p):
    def wheel(w):
        pos=' '.join(map(str,w['center']));inertia=' '.join(map(str,w['inertia']));r=w['radius'];width=w['width']
        return f'<body name="{w["name"]}" pos="{pos}"><joint name="{w["name"]}_spin" type="hinge" axis="0 1 0" damping=".02"/><inertial pos="0 0 0" mass="{w["mass"]}" diaginertia="{inertia}"/><geom type="cylinder" size="{r} {width/2}" quat=".70710678 .70710678 0 0" contype="0" conaffinity="0" rgba=".08 .09 .1 1"/></body>'
    body=''.join(wheel(w) for w in p['wheels'] if w['parent']=='chassis')
    if 'front_fork' in p:
        f=p['front_fork'];pos=' '.join(map(str,f['center']))
        body+=f'<body name="front_fork" pos="{pos}"><joint name="steer" axis="0 0 1" damping="1" limited="true" range="-.6 .6"/><inertial mass="{f["mass"]}" pos="0 0 0" diaginertia=".05 .05 .05"/>'+''.join(wheel(w) for w in p['wheels'] if w['parent']=='front_fork')+'</body>'
    actuators=''.join(f'<motor name="drive_{w["name"]}" joint="{w["name"]}_spin"/>' for w in p['wheels'] if w['paddles'])
    if 'front_fork' in p:actuators+='<motor name="steering" joint="steer"/>'
    return f'''<mujoco model="water_{p['vehicle']}"><compiler angle="radian"/><option timestep="{p['dt']}" gravity="0 0 -9.81" density="0" viscosity="0" integrator="implicitfast"><flag energy="enable"/></option><worldbody><body name="chassis" pos="0 0 {p['initial_z']}"><freejoint name="free"/><inertial pos="{' '.join(map(str,p.get('chassis_com',[0,0,0])))}" mass="{p['chassis_mass']}" diaginertia="{' '.join(map(str,p['chassis_inertia']))}"/><geom type="box" size=".4 .15 .3" contype="0" conaffinity="0"/>{body}</body></worldbody><actuator>{actuators}</actuator></mujoco>'''

class Experiment:
    def __init__(self,p,grid_scale=1):
        self.p=deepcopy(p);self.water=Water(p['water']);self.xml=model_xml(p)
        self.model=mujoco.MjModel.from_xml_string(self.xml);self.data=mujoco.MjData(self.model);m,d=self.model,self.data
        self.names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,i) for i in range(1,m.nbody)]
        self.ids={name:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,name) for name in self.names}
        self.mass=float(m.body_mass.sum());self.boxes=[];self.panels=[];self.driven=[]
        for b in p['volumes']:
            size=np.array(b['size']);grid=np.maximum(1,np.array(b['grid'])*grid_scale).astype(int);cell=size/grid
            rot=ry(-np.deg2rad(b.get('pitch_deg',0)))
            if b.get('shape')=='annular_cylinder':
                nt,nr,ny=grid;angles=(np.arange(nt)+.5)*2*np.pi/nt
                rb=np.sqrt(np.linspace(b['inner_radius']**2,b['outer_radius']**2,nr+1));rc=np.sqrt((rb[:-1]**2+rb[1:]**2)/2)
                yy=(np.arange(ny)+.5)/ny*b['width']-b['width']/2
                aa,rr,yyy=np.meshgrid(angles,rc,yy,indexing='ij');aa=aa.ravel();rr=rr.ravel();yyy=yyy.ravel()
                pts=np.column_stack([rr*np.cos(aa),yyy,rr*np.sin(aa)])+np.array(b['center'])
                dr=np.broadcast_to(np.diff(rb)[None,:,None],(nt,nr,ny)).ravel()
                radial=np.column_stack([np.cos(aa)*dr,np.zeros(len(aa)),np.sin(aa)*dr])
                tangent=np.column_stack([-np.sin(aa)*rr*2*np.pi/nt,np.zeros(len(aa)),np.cos(aa)*rr*2*np.pi/nt])
                axial=np.tile([0,b['width']/ny,0],(len(aa),1));edges=np.stack([radial,axial,tangent],axis=1)
            else:
                axes=[(np.arange(n)+.5)/n*s-s/2 for s,n in zip(size,grid)]
                xyz=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3);pts=xyz@rot.T+np.array(b['center'])
                edges=np.tile((rot@np.diag(cell)).T,(len(pts),1,1))
            count=len(pts);areas=np.array([size[1]*size[2],size[0]*size[2],size[0]*size[1]])
            self.boxes.append(dict(spec=b,bid=self.ids[b['body']],points=pts,rot=rot,cell=cell,edges=edges,vol=volume_of(b)/count,area=areas/count,cd=np.array(b['cd'])))
        for b in p['panels']:
            size=np.array(b['size']);grid=np.maximum(1,np.array(b['grid'])*grid_scale).astype(int)
            x=(np.arange(grid[0])+.5)/grid[0]*size[0]-size[0]/2;y=(np.arange(grid[1])+.5)/grid[1]*size[1]-size[1]/2
            xx,yy=np.meshgrid(x,y,indexing='ij');pts=np.column_stack([xx.ravel(),yy.ravel(),np.zeros(xx.size)]);rot=ry(-np.deg2rad(b['pitch_deg']));pts=pts@rot.T+np.array(b['center'])
            self.panels.append(dict(spec=b,bid=self.ids[b['body']],points=pts,normal=rot[:,2],longitudinal=rot[:,0],lateral=rot[:,1],grid=tuple(grid),chord=xx.ravel()+size[0]/2,area=float(np.prod(size)/len(pts))))
        for w in p['wheels']:
            if not w['paddles']:continue
            theta=np.arange(w['blade_count'])*2*np.pi/w['blade_count'];r=w['blade_radius']
            # Rotation about +Y: velocity at bottom is toward -X.
            pts=np.column_stack([r*np.cos(theta),np.zeros(len(theta)),r*np.sin(theta)])
            normals=np.column_stack([np.sin(theta),np.zeros(len(theta)),-np.cos(theta)])
            j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,w['name']+'_spin')
            aid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,'drive_'+w['name'])
            self.driven.append(dict(spec=w,bid=self.ids[w['name']],points=pts,normals=normals,dof=int(m.jnt_dofadr[j]),qadr=int(m.jnt_qposadr[j]),aid=aid))
        d.qvel[0]=p['initial_speed'];roll=np.deg2rad(p['initial_roll_deg']);pitch=np.deg2rad(p['initial_pitch_deg'])
        qroll=np.array([np.cos(roll/2),np.sin(roll/2),0.,0.]);qpitch=np.array([np.cos(pitch/2),0.,-np.sin(pitch/2),0.]);q=np.zeros(4);mujoco.mju_mulQuat(q,qpitch,qroll);d.qpos[3:7]=q
        for w in p['wheels']:
            j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,w['name']+'_spin');d.qvel[m.jnt_dofadr[j]]=p['initial_speed']/w['radius']
        mujoco.mj_forward(m,d)
        self.capacity=sum(volume_of(v) for v in p['volumes'])*self.water.rho/self.mass
        self.velocity=np.zeros((m.nbody,6));self.last=None
    def apply(self):
        m,d=self.model,self.data;rho=self.water.rho;t=d.time
        d.xfrc_applied[:]=0;d.qfrc_applied[:]=0;d.ctrl[:]=0
        fsum=np.zeros((len(GROUPS),3));tsum=np.zeros_like(fsum);power=np.zeros(len(GROUPS));world_power=np.zeros(len(GROUPS));wetvol=0.;wetarea=0.;rootcom=d.subtree_com[self.ids['chassis']]
        for bid in range(1,m.nbody):mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,bid,self.velocity[bid],0)
        def state(bid,pts):
            R=d.xmat[bid].reshape(3,3);wp=pts@R.T+d.xpos[bid];v=self.velocity[bid,3:]+np.cross(self.velocity[bid,:3],wp-d.xipos[bid]);eta,u,acc=self.water.evaluate(wp,t)
            return R,wp,v-u,eta,acc
        def add(group,bid,pts,forces,relvel):
            idx=GROUPS.index(group);F=np.sum(forces,axis=0);T=np.sum(np.cross(pts-d.xipos[bid],forces),axis=0)
            d.xfrc_applied[bid,:3]+=F;d.xfrc_applied[bid,3:]+=T;fsum[idx]+=F;tsum[idx]+=np.sum(np.cross(pts-rootcom,forces),axis=0);power[idx]+=float(np.sum(forces*relvel))
            point_velocity=self.velocity[bid,3:]+np.cross(self.velocity[bid,:3],pts-d.xipos[bid]);world_power[idx]+=float(np.sum(forces*point_velocity))
        for b in self.boxes:
            bid=b['bid'];R,wp,vr,eta,acc=state(bid,b['points']);br=R@b['rot']
            frac,centroid=clip_cells(wp,b['edges']@R.T,eta)
            dv=b['vol']*frac;wetvol+=float(dv.sum())
            # Velocities and wave acceleration are re-evaluated at wet centroids.
            vel=self.velocity[bid,3:]+np.cross(self.velocity[bid,:3],centroid-d.xipos[bid]);_,u,acc=self.water.evaluate(centroid,t);vr=vel-u
            buoy=np.zeros_like(wp);buoy[:,2]=rho*G*dv
            add('buoyancy',bid,centroid,buoy,vr)
            add('wave_pressure',bid,centroid,rho*dv[:,None]*acc,vr)
            vl=vr@br;fd=-.5*rho*frac[:,None]*b['cd']*b['area']*np.abs(vl)*vl
            add('volume_drag',bid,centroid,fd@br.T,vr)
        planing_valid_z=0.;planing_extrapolated_z=0.;planing_cp_error=0.
        for b in self.panels:
            bid=b['bid'];spec=b['spec']
            for side in [0,1]:
                pts=b['points']+side*spec['thickness']*b['normal']
                R,wp,vr,eta,acc=state(bid,pts);n=(1-2*side)*(R@b['normal'])
                wet=np.clip((eta-wp[:,2])/spec['wet_transition_m'],0,1)
                if side==0:wetarea+=float(wet.sum()*b['area'])
                vn=vr@n;speed=np.linalg.norm(vr,axis=1)
                fn=.5*rho*b['area']*spec['cn']*wet*speed*np.maximum(-vn,0.)
                if side==0 and spec.get('pressure_model')=='savitsky_dynamic':
                    fn,diagnostic=dynamic_normal_forces(rho,spec['size'][1],b['area'],b['chord'],wet,vr,n,R@b['longitudinal'],R@b['lateral'],b['grid'])
                    planing_valid_z+=diagnostic['valid_force_z']
                    planing_extrapolated_z+=diagnostic['extrapolated_force_z']
                    planing_cp_error=max(planing_cp_error,abs(diagnostic['cp_error_m']))
                add('panel_pressure',bid,wp,fn[:,None]*n,vr)
                vt=vr-vn[:,None]*n;ft=-.5*rho*b['area']*spec['cf']*wet[:,None]*np.linalg.norm(vt,axis=1)[:,None]*vt
                add('skin_drag',bid,wp,ft,vr)
        for b in self.driven:
            bid=b['bid'];R,wp,vr,eta,acc=state(bid,b['points']);normals=b['normals']@R.T;spec=b['spec']
            wet=np.clip((eta-wp[:,2])/spec['blade_depth']+.5,0,1);vn=np.sum(vr*normals,axis=1)
            fn=-.5*rho*1.65*spec['blade_area']*wet*np.abs(vn)*vn
            add('paddles',bid,wp,fn[:,None]*normals,vr)
            speed=d.qvel[b['dof']];target=self.p['wheel_target_rad_s']
            if self.p.get('drive_control')=='positive_power_envelope':
                torque=self.p['drive_torque_nm']*np.clip((target-speed)/max(.15*target,1.),0.,1.)
            else:torque=np.clip((target-speed)*22.,-self.p['drive_torque_nm'],self.p['drive_torque_nm'])
            # Finite power applies to drive and engine braking; no root propulsive force.
            torque=float(np.clip(torque,-self.p['drive_power_w']/max(abs(speed),1.),self.p['drive_power_w']/max(abs(speed),1.)))
            if self.p['drive_off_s'] is not None and t>=self.p['drive_off_s']:torque=0.
            d.ctrl[b['aid']]=torque
        if 'front_fork' in self.p:
            j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'steer');aid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,'steering');target=np.deg2rad(self.p['steer_target_deg']) if t>=self.p['steer_start_s'] else 0.
            d.ctrl[aid]=np.clip(80*(target-d.qpos[m.jnt_qposadr[j]])-8*d.qvel[m.jnt_dofadr[j]],-35,35)
        bid=self.ids['chassis'];vp=self.velocity[bid,3:];fa=-.5*1.225*.7*np.linalg.norm(vp)*vp
        add('air_drag',bid,d.xipos[bid][None,:],fa[None,:],vp[None,:])
        motor_power=sum(float(d.ctrl[b['aid']]*d.qvel[b['dof']]) for b in self.driven)
        steer_power=0.
        if 'front_fork' in self.p:steer_power=float(d.ctrl[aid]*d.qvel[m.jnt_dofadr[j]])
        passive_joint_power=float(np.dot(d.qfrc_passive,d.qvel))
        self.last=dict(force=fsum,torque=tsum,relative_power=power,world_power=world_power,wet_volume=wetvol,wet_panel_area=wetarea,motor_power=motor_power,steering_power=steer_power,passive_joint_power=passive_joint_power,ctrl=d.ctrl.copy(),planing_valid_z=planing_valid_z,planing_extrapolated_z=planing_extrapolated_z,planing_cp_error_m=planing_cp_error)
        return self.last
    def step(self):
        mujoco.mj_step1(self.model,self.data);self.apply();mujoco.mj_step2(self.model,self.data)
    def snapshot(self):
        m,d=self.model,self.data;mujoco.mj_forward(m,d);loads=self.apply();q=d.qpos[3:7];w,x,y,z=q
        roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y));pitch=np.arcsin(np.clip(2*(w*y-z*x),-1,1));yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
        potential=-float(np.sum(m.body_mass[:,None]*d.xipos*m.opt.gravity));mujoco.mj_energyVel(m,d)
        return dict(t=d.time,qpos=d.qpos.copy(),qvel=d.qvel.copy(),body_pos=d.xpos[1:].copy(),body_quat=d.xquat[1:].copy(),kinetic_energy_j=float(d.energy[1]),potential_energy_j=potential,roll_deg=np.rad2deg(roll),pitch_deg=-np.rad2deg(pitch),yaw_deg=np.rad2deg(yaw),**loads)

def run(p,dest,grid_scale=1):
    dest=Path(dest);dest.mkdir(parents=True,exist_ok=True);e=Experiment(p,grid_scale);samples=[];started=time.perf_counter();nextout=0.;stop='duration'
    n=round(p['duration']/p['dt']);integrated_work=0.
    for step in range(n+1):
        if e.data.time+1e-9>=nextout:
            sample=e.snapshot();samples.append(sample);nextout+=1/p['output_hz']
            if not np.isfinite(e.data.qpos).all() or not np.isfinite(e.data.qvel).all():raise RuntimeError('Non-finite solver state')
            if e.data.qpos[2]<-3.5:stop='body reference origin below -3.5 m; experiment stopped';break
        if step<n:
            e.step();integrated_work+=p['dt']*(float(np.sum(e.last['world_power']))+e.last['motor_power']+e.last['steering_power']+e.last['passive_joint_power'])
            if np.any(e.data.warning.number):raise RuntimeError('MuJoCo warning: '+str(e.data.warning.number))
    arrays={k:np.asarray([s[k] for s in samples]) for k in samples[0]};np.savez_compressed(dest/'trajectory.npz',**arrays)
    (dest/'params.json').write_text(json.dumps(p,ensure_ascii=False,indent=2),encoding='utf8');(dest/'model.xml').write_text(e.xml,encoding='utf8')
    time_axis=arrays['t'];last=min(len(time_axis),int(p['output_hz']*2));force=arrays['force'];bidx=GROUPS.index('buoyancy');pidx=GROUPS.index('panel_pressure')
    dissipative=[GROUPS.index(x) for x in ['volume_drag','panel_pressure','skin_drag','paddles','air_drag']]
    result=dict(solver='MuJoCo '+mujoco.__version__,body_names=e.names,force_groups=list(GROUPS),elapsed_wall_s=time.perf_counter()-started,simulated_s=float(time_axis[-1]),stop=stop,grid_scale=grid_scale,total_mass_kg=e.mass,max_displacement_m3=sum(volume_of(v) for v in p['volumes']),capacity_ratio=e.capacity,final_speed_m_s=float(np.linalg.norm(arrays['qvel'][-1,:3])),final_z_m=float(arrays['qpos'][-1,2]),max_abs_roll_deg=float(np.max(np.abs(arrays['roll_deg']))),max_abs_pitch_deg=float(np.max(np.abs(arrays['pitch_deg']))),final_mean_buoyancy_weight_ratio=float(np.mean(force[-last:,bidx,2])/(e.mass*G)),final_mean_dynamic_vertical_weight_ratio=float(np.mean(force[-last:,pidx,2])/(e.mass*G)),max_dissipative_relative_power_w=float(np.max(arrays['relative_power'][:,dissipative])),initial_energy_j=float(arrays['kinetic_energy_j'][0]+arrays['potential_energy_j'][0]),trajectory_sha256=hashlib.sha256((dest/'trajectory.npz').read_bytes()).hexdigest(),source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),clip_source_sha256=hashlib.sha256((ROOT/'clipped_box.py').read_bytes()).hexdigest())
    delta_energy=float(arrays['kinetic_energy_j'][-1]+arrays['potential_energy_j'][-1]-arrays['kinetic_energy_j'][0]-arrays['potential_energy_j'][0])
    result.update(integrated_non_gravity_work_j=integrated_work,mechanical_energy_change_j=delta_energy,energy_balance_residual_j=delta_energy-integrated_work,energy_balance_relative_error=abs(delta_energy-integrated_work)/max(e.mass*G,abs(integrated_work)),final_vertical_speed_m_s=float(arrays['qvel'][-1,2]),final_mean_vertical_speed_m_s=float(np.mean(arrays['qvel'][-last:,2])),total_drive_power_budget_w=p['total_drive_power_w'])
    result['planing_surface_source_sha256']=hashlib.sha256((ROOT/'planing_surface.py').read_bytes()).hexdigest()
    result['closure']='Hybrid: Savitsky dynamic term on tagged surfaces, separate geometric buoyancy, original other loads'
    (dest/'metadata.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    header=['t','x','y','z','vx','vy','vz','roll_deg','pitch_deg','yaw_deg','buoyancy_N','dynamic_z_N','paddle_x_N','wet_volume_m3','wet_panel_area_m2','motor_power_W']
    columns=np.column_stack([time_axis,arrays['qpos'][:,:3],arrays['qvel'][:,:3],arrays['roll_deg'],arrays['pitch_deg'],arrays['yaw_deg'],force[:,bidx,2],force[:,pidx,2],force[:,GROUPS.index('paddles'),0],arrays['wet_volume'],arrays['wet_panel_area'],arrays['motor_power']])
    np.savetxt(dest/'trace.csv',columns,delimiter=',',header=','.join(header),comments='',fmt='%.8g')
    print(json.dumps(dict(name=dest.name,**{k:result[k] for k in ['simulated_s','elapsed_wall_s','capacity_ratio','final_z_m','final_speed_m_s','max_abs_roll_deg','max_abs_pitch_deg','max_dissipative_relative_power_w']})),flush=True)
    return result,arrays

def scenarios():
    out={}
    for name,variant,water in [('bike_low_torque','low_torque','calm'),('bike_flat','base','calm'),('bike_head','base','head'),('bike_cross','base','cross'),('bike_swell','base','swell'),('bike_irregular','base','irregular'),('bike_light','light','calm'),('bike_wide','wide','calm'),('bike_reserve_buoyancy','buoyant','calm')]:out[name]=parameters('bike',variant,water)
    for variant in ['base','buoyant']:
        p=parameters('bike',variant);p.update(initial_speed=0.,duration=15.,drive_off_s=0.);out['bike_stop_'+variant]=p
    p=parameters('bike');p.update(drive_off_s=3.,duration=16.);out['bike_cut_power']=p
    p=parameters('bike');p.update(initial_roll_deg=3.,duration=10.);out['bike_roll_recovery']=p
    for variant in ['no_mod','paddles','small_floats','large_floats']:
        p=parameters('car',variant);out['car_'+variant]=p
        p=deepcopy(p);p.update(initial_speed=0.,drive_off_s=0.,duration=15.,initial_roll_deg=3.);out['car_stop_'+variant]=p
    for water in ['head','cross','swell','irregular']:
        p=parameters('car','large_floats',water);p['initial_speed']=2.;out['car_large_'+water]=p
    return out

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--cases',default='bike_flat,car_large_floats');ap.add_argument('--duration',type=float);ap.add_argument('--dt',type=float);ap.add_argument('--grid-scale',type=int,default=1);ap.add_argument('--suffix',default='');args=ap.parse_args();allcases=scenarios();selected=list(allcases) if args.cases=='all' else args.cases.split(',')
    for name in selected:
        p=allcases[name]
        if args.duration:p['duration']=args.duration
        if args.dt:p['dt']=args.dt
        run(p,ROOT/'runs'/(name+args.suffix),args.grid_scale)
