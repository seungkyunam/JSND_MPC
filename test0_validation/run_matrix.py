#!/usr/bin/env python3
import argparse, concurrent.futures, subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'test0_validation/sim_env.bash'
CASES=[('pp','pp'),('mpc','legacy057'),('mpc','final')]
SCENARIOS=['clear','obstacles','obstacles_realistic']

def one(args):
    controller,profile,scenario,out,laps=args
    dest=out/f'{controller}_{profile}__{scenario}'
    cmd=[str(RUN),str(ROOT/'test0_validation/run_trial.py'),'--controller',controller,'--profile',profile,
         '--scenario',scenario,'--laps',str(laps),'--output',str(dest)]
    done=subprocess.run(cmd,text=True,capture_output=True)
    (out/f'{controller}_{profile}__{scenario}.log').write_text(done.stdout+done.stderr)
    return dest.name,done.returncode

def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); p.add_argument('--laps',type=int,default=11); p.add_argument('--jobs',type=int,default=3)
    p.add_argument('--scenarios',nargs='+',choices=SCENARIOS,default=SCENARIOS); p.add_argument('--resume',action='store_true')
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=a.resume)
    jobs=[(c,p,s,a.output,a.laps) for s in a.scenarios for c,p in CASES]
    with concurrent.futures.ProcessPoolExecutor(max_workers=a.jobs) as ex:
        results=list(ex.map(one,jobs))
    print('\n'.join(f'{name}: {code}' for name,code in results))
    return int(any(code for _,code in results))
if __name__=='__main__': raise SystemExit(main())
