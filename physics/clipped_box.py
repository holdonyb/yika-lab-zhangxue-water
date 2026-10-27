"""Exact horizontal-plane clipping of oriented parallelepiped cells.

Cell edges are full WORLD edge vectors stored as rows. Centers, edges and water
levels are batched. The algorithm integrates piecewise polynomial cross sections
analytically (two-point Gauss is exact on each cubic segment). Unlike direct
truncated-power inclusion/exclusion, it stays well conditioned as one or two
vertical edge projections approach zero.

Returns wet volume fraction and WORLD wet centroid. Empty cells return their
original center; their force contribution is zero. This clips against each
cell's supplied horizontal plane; spatially varying waves are still discretized.
"""
from __future__ import annotations
import numpy as np


def _cdf2(p, q, s):
    """Volume and unnormalized centered first moments; 0 < p <= q.

Integrate unit-square coordinates (u,v) satisfying p*u + q*v <= s.
Moments are integrals of (u-.5) and (v-.5), not conditional means.
"""
    p,q,s=np.broadcast_arrays(p,q,s)
    complement=s>.5*(p+q)
    r=np.where(complement,p+q-s,s)
    r=np.maximum(r,0.)
    triangle=r<=p
    f_tri=.5*(r/p)*(r/q)
    mp_tri=f_tri*(r/(3*p)-.5)
    mq_tri=f_tri*(r/(3*q)-.5)
    f_trap=(r-.5*p)/q
    mp_trap=-p/(12*q)
    mq_trap=.5*f_trap*(f_trap-1)+(p/q)**2/24
    f=np.where(triangle,f_tri,f_trap)
    mp=np.where(triangle,mp_tri,mp_trap)
    mq=np.where(triangle,mq_tri,mq_trap)
    f=np.where(complement,1-f,f)
    outside=(s<=0)|(s>=p+q)
    f=np.clip(f,0.,1.)
    mp=np.where(outside,0.,mp)
    mq=np.where(outside,0.,mq)
    return f,mp,mq


def clip_cells(centers, edges, water_levels):
    """Return `(wet_fraction[N], wet_centroid[N,3])` without modifying inputs.

Parameters
----------
centers : (N,3) world cell centers
edges : (N,3,3) world edge vectors, rows; (3,3) also broadcasts
water_levels : (N,) world water z at each cell, or scalar
"""
    centers=np.asarray(centers,dtype=np.float64)
    if centers.ndim!=2 or centers.shape[1]!=3:
        raise ValueError('centers must have shape (N,3)')
    n=len(centers)
    edges=np.broadcast_to(np.asarray(edges,dtype=np.float64),(n,3,3))
    levels=np.broadcast_to(np.asarray(water_levels,dtype=np.float64),(n,))
    projection=edges[:,:,2]
    b=np.abs(projection)
    extent=np.sum(b,axis=1)
    if np.any(extent<=0):
        raise ValueError('cell must have nonzero vertical extent')
    depth=levels-centers[:,2]
    frac=np.zeros(n,dtype=float)
    centroid=centers.copy()
    full=depth>=extent/2
    dry=depth<=-extent/2
    frac[full]=1.
    partial=~(full|dry)
    if not np.any(partial):return frac,centroid

    indices=np.flatnonzero(partial)
    bp=b[indices]
    ep=edges[indices]
    # Dropped projections are at most a few ulps of normal cell dimensions.
    active=bp>np.max(bp,axis=1)[:,None]*1e-13
    bp=np.where(active,bp,0.)
    order=np.argsort(bp,axis=1)
    bs=np.take_along_axis(bp,order,axis=1)
    count=np.sum(active,axis=1)
    s=depth[indices]+.5*np.sum(bp,axis=1)
    f=np.zeros(len(indices))
    moments=np.zeros((len(indices),3))  # sorted-coordinate unit-cube moments

    one=count==1
    if np.any(one):
        f1=np.clip(s[one]/bs[one,2],0.,1.)
        f[one]=f1
        moments[one,2]=-.5*f1*(1-f1)

    two=count==2
    if np.any(two):
        f2,m1,m2=_cdf2(bs[two,1],bs[two,2],s[two])
        f[two]=f2;moments[two,1]=m1;moments[two,2]=m2

    three=count==3
    if np.any(three):
        small,p,q=bs[three].T
        st=s[three]
        # Integrate u along the smallest projected edge; remaining cross
        # section changes polynomial at levels 0,p,q,p+q.
        cuts=np.column_stack([np.zeros(len(st)),np.ones(len(st)),
                              st/small,(st-p)/small,(st-q)/small,(st-p-q)/small])
        cuts=np.sort(np.clip(cuts,0.,1.),axis=1)
        f3=np.zeros(len(st));m0=np.zeros(len(st));m1=np.zeros(len(st));m2=np.zeros(len(st))
        invsqrt3=1/np.sqrt(3.)
        for j in range(5):
            mid=.5*(cuts[:,j]+cuts[:,j+1]);half=.5*(cuts[:,j+1]-cuts[:,j])
            for sign in (-1.,1.):
                u=mid+sign*half*invsqrt3
                fc,mc1,mc2=_cdf2(p,q,st-small*u)
                f3+=half*fc;m0+=half*(u-.5)*fc;m1+=half*mc1;m2+=half*mc2
        f[three]=f3;moments[three,0]=m0;moments[three,1]=m1;moments[three,2]=m2

    f=np.clip(f,0.,1.)
    # Map sorted integration coordinates back to original edge coordinates.
    unsorted=np.zeros_like(moments)
    np.put_along_axis(unsorted,order,moments,axis=1)
    coeff=np.divide(unsorted,f[:,None],out=np.zeros_like(unsorted),where=f[:,None]>1e-300)
    coeff*=np.sign(projection[indices])
    displacement=np.einsum('ni,nij->nj',coeff,ep)
    frac[indices]=f
    centroid[indices]=centers[indices]+displacement
    return frac,centroid
