"""Force/moment bench against the published flat-surface dynamic correlation."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'physics'))
import hydro,hydro_planing

def measure(module,speed,trim,lam,grid=512):
    p=hydro.parameters('car','no_mod');b=1.4;length=3.3;angle=np.deg2rad(trim);wet_length=lam*b
    p.update(initial_speed=speed,initial_pitch_deg=trim,initial_z=.48*np.cos(angle)+np.sin(angle)*(length/2-wet_length))
    p['volumes']=[];p['panels'][0].update(grid=[grid,2],wet_transition_m=1e-7)
    if module is hydro_planing:p['panels'][0]['pressure_model']='savitsky_dynamic'
    e=module.Experiment(p);out=e.apply();i=module.GROUPS.index('panel_pressure');force=out['force'][i];moment=out['torque'][i]
    bid=e.ids['chassis'];rot=e.data.xmat[bid].reshape(3,3);fb=rot.T@force;tb=rot.T@moment
    com=rot.T@(e.data.subtree_com[bid]-e.data.xpos[bid]);cp=float(-tb[1]/fb[2]+com[0]+length/2)
    return {'lift_n':float(force[2]),'dynamic_cp_from_transom_m':cp,'relative_power_w':float(out['relative_power'][i]),'wet_length_m':float(out['wet_panel_area']/b),'reported_cp_error_m':out.get('planing_cp_error_m')}

def main():
    cases=[(15.,tau,lam) for tau in [3.,6.,9.] for lam in [.75,1.5,3.3/1.4]]+[(u,6.,1.5) for u in [5.,10.,20.]]
    rows=[]
    for speed,trim,lam in cases:
        # Published reference, independent of the implementation under test.
        expected_lift=.5*1000*speed**2*1.4**2*.012*lam**.5*trim**1.1
        expected_cp=.75*lam*1.4
        old=measure(hydro,speed,trim,lam);new=measure(hydro_planing,speed,trim,lam)
        lift_error=abs(new['lift_n']/expected_lift-1);cp_error=abs(new['dynamic_cp_from_transom_m']-expected_cp)
        rows.append({'speed_m_s':speed,'trim_deg':trim,'lambda':lam,'Cv':speed/np.sqrt(9.81*1.4),
            'reference_dynamic_lift_n':expected_lift,'reference_dynamic_cp_m':expected_cp,'frozen_local_closure':old,'planing_branch':new,
            'lift_relative_error':lift_error,'cp_absolute_error_m':cp_error,
            'passed':bool(lift_error<.01 and cp_error<.02 and new['relative_power_w']<=0)})
    report={'reference':'Savitsky 1964 flat-surface dynamic term and dynamic center of pressure; geometric static buoyancy deliberately excluded from this bench',
        'source_pdf':'research/savitsky1964_review.pdf','surface':'3.3 m x 1.4 m, zero deadrise, 512 x 2 integration cells, finite-thickness top/bottom treatment',
        'scope':'Agreement of integrated dynamic force and moment with published correlation; not experimental validation of the hybrid vehicle model or waves',
        'passed':all(r['passed'] for r in rows),'cases':rows,
        'source_hashes':{n:hashlib.sha256((ROOT/'physics'/n).read_bytes()).hexdigest() for n in ['hydro.py','hydro_planing.py','planing_surface.py']}}
    (ROOT/'research/planing_surface_benchmark.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'passed':report['passed'],'cases':len(rows),'max_lift_error':max(r['lift_relative_error'] for r in rows),'max_cp_error_m':max(r['cp_absolute_error_m'] for r in rows),'first_case':rows[0]},ensure_ascii=False))
    if not report['passed']:raise SystemExit(1)

if __name__=='__main__':main()
