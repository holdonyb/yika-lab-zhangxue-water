"""Independent reference: exact volume/centroid of an oriented box below z=eta.

Local box size is full length, R maps local offsets to world. `depth` is eta
minus the WORLD z of the box center. All depth entries share size and R.
Returns volume fraction and wet-centroid offset in the local box frame.
Derivation: inclusion/exclusion integrals over a clipped positive orthant.
"""
import itertools
import math
import numpy as np

def clip_box_horizontal(size, R, depth):
    size=np.asarray(size,dtype=float)
    a=np.asarray(R,dtype=float)[2]
    depth=np.atleast_1d(depth).astype(float)
    projection=np.abs(a)*size
    active=np.flatnonzero(projection > max(1e-12, float(np.max(projection))*1e-9))
    b=projection[active]
    n=len(active)
    s=depth+.5*float(np.sum(b))
    frac=np.zeros(len(depth))
    offset=np.zeros((len(depth),3))
    full=s>=float(np.sum(b))
    frac[full]=1.
    inside=(s>0)&~full
    if not np.any(inside):return frac,offset
    si=s[inside]
    measure=np.zeros_like(si)
    moments=np.zeros((len(si),n))
    for bits in itertools.product([0,1],repeat=n):
        bits=np.array(bits)
        sign=(-1.)**int(np.sum(bits))
        rem=np.maximum(si-float(bits@b),0.)
        vol=rem**n/math.factorial(n)
        mom=rem**(n+1)/math.factorial(n+1)
        measure+=sign*vol
        moments+=sign*(mom[:,None]+vol[:,None]*(bits*b))
    frac[inside]=np.clip(measure/np.prod(b),0.,1.)
    means=moments/np.maximum(measure[:,None],1e-300)
    local=(means/b-.5)*size[active]*np.sign(a[active])
    offset[np.ix_(inside,active)]=local
    return frac,offset

def polygon_wet_rectangle(width,height,roll,zcenter):
    """Independent 2-D polygon clipping and shoelace centroid for a prism."""
    c,s=np.cos(roll),np.sin(roll)
    poly=[]
    for y,z in [(-width/2,-height/2),(width/2,-height/2),(width/2,height/2),(-width/2,height/2)]:
        poly.append(np.array([c*y-s*z,s*y+c*z+zcenter]))
    wet=[]
    for p,q in zip(poly,poly[1:]+poly[:1]):
        pin,qin=p[1]<=0,q[1]<=0
        if pin:wet.append(p)
        if pin!=qin:wet.append(p+(q-p)*(-p[1]/(q[1]-p[1])))
    if len(wet)<3:return 0.,np.zeros(2)
    P=np.asarray(wet);Q=np.roll(P,-1,axis=0)
    cross=P[:,0]*Q[:,1]-Q[:,0]*P[:,1]
    area=.5*np.sum(cross)
    centroid=np.sum((P+Q)*cross[:,None],axis=0)/(6*area)
    return abs(float(area)),centroid

if __name__=='__main__':
    checks=[]
    for angle in [0.,.01,1.,3.,5.,15.,45.,89.9,90.]:
        r=np.deg2rad(angle);c,s=np.cos(r),np.sin(r)
        R=np.array([[1,0,0],[0,c,-s],[0,s,c]])
        frac,local=clip_box_horizontal([2,2,1],R,[-.25])
        area,centroid=polygon_wet_rectangle(2,1,r,.25)
        world=R@local[0]+[0,0,.25]
        checks.append(dict(angle=angle,volume_error=float(frac[0]*4-area*2),centroid_error=float(np.max(np.abs(world[1:]-centroid))),Fz_N=float(9810*area*2),Mx_Nm=float(9810*area*2*centroid[0])))
    import json
    print(json.dumps(checks,indent=2))
