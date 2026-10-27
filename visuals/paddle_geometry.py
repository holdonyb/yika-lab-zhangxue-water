"""Cup mesh in the solver's +X forward / +Y axle / +Z up coordinates.

Positive rotation about Y travels along -e_theta.  The concave aperture must
therefore open along -e_theta, including at the bottom where it faces -X.
This display mesh represents the idealized paddle area; it does not add a CFD
model of cup-side-dependent drag to the preserved solver.
"""
import math

def cup_mesh(spec, phase, rows=6, columns=8):
    radius=spec['blade_radius'];depth=spec['blade_depth']
    width=spec.get('blade_width',spec['blade_area']/depth)
    cup_depth=.35*depth
    er=(math.cos(phase),0.,math.sin(phase))
    et=(-math.sin(phase),0.,math.cos(phase))
    vertices=[]
    for j in range(rows+1):
        lateral=2*j/rows-1
        for k in range(columns+1):
            radial=2*k/columns-1
            rr=radius+depth*.5*radial
            tangential=-cup_depth*(radial**2+.25*lateral**4)
            vertices.append((rr*er[0]+tangential*et[0],width*.5*lateral,rr*er[2]+tangential*et[2]))
    faces=[]
    for j in range(rows):
        for k in range(columns):
            q=j*(columns+1)+k
            # Concave/front face normal is -e_theta at the central force point.
            faces.append((q,q+columns+1,q+columns+2,q+1))
    return vertices,faces

def stamp_cup(ob,spec,index):
    phase=index*math.tau/spec['blade_count']
    ob['paddle_index']=index
    ob['aperture_local']=[math.sin(phase),0.,-math.cos(phase)]
    ob['force_point_local']=[spec['blade_radius']*math.cos(phase),0.,spec['blade_radius']*math.sin(phase)]
    ob['probe_vertex_indices']=[3*9+4,3*9,3*9+8]
    ob['direction_marker']=index==0

def cup_mount_points(spec,phase,tyre_major,tyre_minor,sign):
    """Connect inside the actual torus rubber to the actual curved cup root."""
    width=spec.get('blade_width',spec['blade_area']/spec['blade_depth'])
    end_y=sign*width*.30
    start_y=sign*min(spec['width']*.25,tyre_minor*.5)
    start_r=tyre_major+math.sqrt(tyre_minor**2-start_y**2)-.008
    end_r=spec['blade_radius']-spec['blade_depth']/2
    lateral=2*end_y/width
    end_t=-.35*spec['blade_depth']*(1+.25*lateral**4)
    er=(math.cos(phase),0.,math.sin(phase));et=(-math.sin(phase),0.,math.cos(phase))
    start=(start_r*er[0],start_y,start_r*er[2])
    end=(end_r*er[0]+end_t*et[0],end_y,end_r*er[2]+end_t*et[2])
    return start,end
