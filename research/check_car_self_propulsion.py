"""One bounded self-propulsion equilibrium diagnostic plus a low-power continuation."""
from pathlib import Path
import hashlib
import json
import time
import numpy as np
from scipy.optimize import least_squares
from check_car_planing_equilibrium import Bench, ROOT


def main():
    previous=json.loads((ROOT/'research/car_planing_equilibrium.json').read_text())
    case=next(c for c in previous['cases'] if c['geometry']=='wide_low' and c['power_factor']==4 and c['speed_m_s']==15)
    seed=case['solutions'][0]
    bench=Bench('wide_low',15.,4.,phases=16)
    def residual(x):
        bench.speed=float(x[4])
        row=bench.evaluate(x[:4],details=True)
        r=row['residual']
        return np.array([r['vertical_n'],r['pitch_nm'],r['rear_per_wheel_nm'],r['front_per_wheel_nm'],
                         row['net_external_force_including_weight_n'][0]])/np.r_[bench.scale,bench.weight]
    x0=[seed['z_m'],seed['trim_deg'],seed['rear_wheel_rad_s'],seed['front_wheel_rad_s'],15.]
    bounds=([-.5,2.05,0.,-10.,5.],[1.35,14.95,110.,110.,20.])
    start=time.perf_counter()
    result=least_squares(residual,x0,bounds=bounds,x_scale=[.3,6,50,40,10],diff_step=1e-4,
                         ftol=2e-10,xtol=2e-10,gtol=2e-10,max_nfev=120)
    err=float(np.max(np.abs(residual(result.x))))
    solution=bench.evaluate(result.x[:4],details=True)
    solution.update(speed_m_s=float(result.x[4]),accepted_constrained_self_propulsion_equilibrium=err<2e-4,
                    normalized_five_equation_max_residual=err,elapsed_s=time.perf_counter()-start)
    solution['release_state']=dict(z_m=solution['z_m'],trim_deg=solution['trim_deg'],speed_m_s=solution['speed_m_s'],
                                   rear_wheel_rad_s=solution['rear_wheel_rad_s'],front_wheel_rad_s=solution['front_wheel_rad_s'],
                                   suggested_phase_rad=0.,note='Use these only as initial conditions, then release every body DOF. Phase-averaged torque equilibrium permits instantaneous oscillations; examine a second initial wheel phase.')
    # Controller/geometry parameters are unchanged; initial pose and wheel rates above
    # replace the entry setup only for the explicitly requested free-release diagnostic.
    solution['candidate_parameters']=bench.p
    report=dict(scope='Five-equation constrained steady-state root; does not establish dynamic stability or sustained free motion.',
                source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'research/car_planing_matrix.py',ROOT/'physics/hydro_planing.py',ROOT/'physics/planing_surface.py']},
                wide_low_176kw_self_propulsion=solution)
    dest=ROOT/'research/car_self_propulsion_equilibrium.json'
    dest.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(dict(self_propulsion_solution=solution['release_state'],accepted=solution['accepted_constrained_self_propulsion_equilibrium'],error=err)),flush=True)
    # Continue the already found paddles-only equilibrium toward the original torque
    # rating, instead of declaring nonexistence from failed remote initial guesses.
    old=next(c for c in previous['cases'] if c['geometry']=='paddles' and c['power_factor']==4 and c['speed_m_s']==15)['solutions'][0]
    x=np.array([old['z_m'],old['trim_deg'],old['rear_wheel_rad_s'],old['front_wheel_rad_s']])
    rows=[]
    for factor in [3.,2.,1.]:
        b=Bench('paddles',15.,factor,phases=16)
        r=least_squares(b.evaluate,x,bounds=([-.5,2.05,0,-10],[1.35,14.95,110,110]),x_scale=[.3,6,50,40],diff_step=1e-4,
                        ftol=2e-10,xtol=2e-10,gtol=2e-10,max_nfev=100)
        row=b.evaluate(r.x,details=True);row['power_factor']=factor
        row['accepted_constrained_equilibrium']=row['normalized_max_residual']<2e-4
        rows.append(row);x=r.x
        report['paddles_15m_s_power_continuation']=rows
        dest.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
        print(json.dumps(dict(continuation_factor=factor,accepted=row['accepted_constrained_equilibrium'],error=row['normalized_max_residual'],trim_deg=row['trim_deg'])),flush=True)


if __name__=='__main__':main()
