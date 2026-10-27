"""Bounded continuation of one already found hull/rotor equilibrium branch."""
import json
import numpy as np
from scipy.optimize import least_squares
from check_car_planing_equilibrium import Bench, ROOT


def main():
    original=json.loads((ROOT/'research/car_planing_equilibrium.json').read_text())
    dest=ROOT/'research/car_self_propulsion_equilibrium.json'
    report=json.loads(dest.read_text())
    seed=next(c for c in original['cases'] if c['geometry']=='wide_low' and c['power_factor']==4)['solutions'][0]
    x=np.array([seed['z_m'],seed['trim_deg'],seed['rear_wheel_rad_s'],seed['front_wheel_rad_s']])
    bench=Bench('wide_low',15.,4.,phases=16)
    rows=[]
    # Fixed list: enough to bracket a possible Fx root on the discovered branch,
    # without expanding geometry, power, or searching a highest attainable speed.
    for speed in [15.,14.,13.,12.,10.,8.,5.,20.]:
        if speed==20.:
            x=np.array([seed['z_m'],seed['trim_deg'],seed['rear_wheel_rad_s'],seed['front_wheel_rad_s']])
        bench.speed=speed
        r=least_squares(bench.evaluate,x,bounds=([-.5,2.05,0.,-10.],[1.35,14.95,110.,110.]),x_scale=[.3,6,50,40],diff_step=1e-4,
                        ftol=2e-10,xtol=2e-10,gtol=2e-10,max_nfev=100)
        row=bench.evaluate(r.x,details=True);row.update(speed_m_s=speed,accepted_constrained_equilibrium=row['normalized_max_residual']<2e-4)
        rows.append(row)
        if row['accepted_constrained_equilibrium']:x=r.x
        report['wide_low_176kw_fixed_speed_branch']=rows
        dest.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
        print(json.dumps(dict(speed=speed,accepted=row['accepted_constrained_equilibrium'],error=row['normalized_max_residual'],tow_n=row['required_additional_horizontal_force_n'],trim=row['trim_deg'])),flush=True)
    # Re-evaluate the three original accepted constrained states with exact corner
    # heights, preserving the old result file and its original source fingerprint.
    report['original_15m_s_geometry_recheck']=[]
    for c in original['cases']:
        for old in c['solutions']:
            b=Bench(c['geometry'],15.,c['power_factor'],phases=16)
            row=b.evaluate([old['z_m'],old['trim_deg'],old['rear_wheel_rad_s'],old['front_wheel_rad_s']],details=True)
            row.update(geometry=c['geometry'],power_factor=c['power_factor'])
            report['original_15m_s_geometry_recheck'].append(row)
    dest.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')


if __name__=='__main__':main()
