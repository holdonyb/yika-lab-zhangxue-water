"""Dynamic-only flat planing-surface closure for an explicitly audited branch.

Savitsky (1964): CLd=0.012*sqrt(lambda)*trim_deg**1.1, referenced to beam².
Ideal dynamic pressure center: 0.75 wet length from transom. Source domain:
0.6<=Cv<=13, 2<=trim<=15 degrees, lambda<=4. Extrapolation is recorded,
not silently treated as validated. No empirical hydrostatic term is added.
"""
import numpy as np

def dynamic_normal_forces(rho,beam,cell_area,chord,wet,relative_velocity,normal,longitudinal,lateral,grid):
    chord=np.asarray(chord);wet=np.asarray(wet);vr=np.asarray(relative_velocity);normal=np.asarray(normal)
    vn=vr@normal;speed=np.linalg.norm(vr,axis=1)
    tangential=np.sqrt(np.maximum(0.,speed**2-vn**2))
    attack=np.rad2deg(np.arctan2(np.maximum(-vn,0.),np.maximum(tangential,1e-12)))
    wet_length=float(wet.sum()*cell_area/beam);lam=wet_length/beam
    eligible=(vn<0)&(wet>0)
    weights=wet*np.maximum(chord,0.)**2*eligible
    fn=np.zeros(len(wet));diag={'valid_force_z':0.,'extrapolated_force_z':0.,'cp_error_m':0.,'lambda':lam,'wet_length_m':wet_length,'target_cp_m':.75*wet_length,'actual_cp_m':None}
    if weights.sum()<=0 or normal[2]<=.1:return fn,diag
    weights/=weights.sum()
    cl=.012*np.sqrt(max(lam,0.))*attack**1.1
    fn=.5*rho*beam**2*speed**2*cl*weights/normal[2]
    cv=speed/np.sqrt(9.81*beam)
    # Conservative declared screens for the straight, symmetric, quasi-steady
    # source setting. These tolerances are screening choices, not experimentally
    # established extensions to yaw, roll, disconnected wetted patches or waves.
    forward=vr@np.asarray(longitudinal);sideways=vr@np.asarray(lateral)
    wet_grid=wet.reshape(grid);wet_chord=wet_grid.mean(axis=1)
    contiguous=wet_chord[0]>0 and np.max(np.diff(wet_chord),initial=0.)<=.05
    full_width=np.max(np.ptp(wet_grid,axis=1),initial=0.)<=.15
    mean_speed=float(np.dot(weights,speed));mean_attack=float(np.dot(weights,attack))
    speed_spread=float(np.sqrt(np.dot(weights,(speed-mean_speed)**2)))/max(mean_speed,1e-9)
    attack_spread=float(np.sqrt(np.dot(weights,(attack-mean_attack)**2)))
    geometric_trim=float(np.rad2deg(np.arctan2(longitudinal[2],np.hypot(longitudinal[0],longitudinal[1]))))
    mean_v=np.sum(weights[:,None]*vr,axis=0)
    vertical_flow_angle=float(np.rad2deg(np.arctan2(mean_v[2],np.hypot(mean_v[0],mean_v[1]))))
    geometric_ok=contiguous and full_width and abs(float(lateral[2]))<=np.sin(np.deg2rad(5)) and speed_spread<=.10 and attack_spread<=2. and 2.<=geometric_trim<=15. and abs(vertical_flow_angle)<=2. and abs(mean_attack-geometric_trim)<=2.
    valid=(cv>=.6)&(cv<=13)&(attack>=2)&(attack<=15)&(lam<=4)&(forward>0)&(np.abs(sideways)<=np.maximum(forward,0)*np.tan(np.deg2rad(3)))&geometric_ok
    diag.update(contiguous_transom_wet=bool(contiguous),full_width_wet=bool(full_width),speed_relative_spread=speed_spread,attack_spread_deg=attack_spread,geometric_trim_deg=geometric_trim,mean_vertical_flow_angle_deg=vertical_flow_angle)
    diag['valid_force_z']=float(fn[valid].sum()*normal[2])
    diag['extrapolated_force_z']=float(fn[~valid].sum()*normal[2])
    if fn.sum()>0:
        cp=float(np.dot(fn,chord)/fn.sum());diag['actual_cp_m']=cp;diag['cp_error_m']=cp-.75*wet_length
    return fn,diag
