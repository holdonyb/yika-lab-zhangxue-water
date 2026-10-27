"""Bounded parallel execution; each case has independent immutable output."""
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
import argparse,json,os
os.environ['OMP_NUM_THREADS']='1';os.environ['OPENBLAS_NUM_THREADS']='1'
from hydro import scenarios,run,ROOT

def job(entry):
    name,p,scale=entry
    result,_=run(p,ROOT/'runs'/name,scale)
    return name,result

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--cases',default='all');ap.add_argument('--workers',type=int,default=4);ap.add_argument('--duration',type=float);ap.add_argument('--dt',type=float);ap.add_argument('--suffix',default='_final');ap.add_argument('--grid-scale',type=int,default=1)
    a=ap.parse_args();ss=scenarios();names=list(ss) if a.cases=='all' else a.cases.split(',');jobs=[]
    for name in names:
        p=ss[name]
        if a.duration:p['duration']=a.duration
        if a.dt:p['dt']=a.dt
        jobs.append((name+a.suffix,p,a.grid_scale))
    results={}
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        futures={pool.submit(job,j):j[0] for j in jobs}
        for future in as_completed(futures):
            name,result=future.result();results[name]=result
            (ROOT/('batch'+a.suffix+'.json')).write_text(json.dumps(results,indent=2),encoding='utf8')
