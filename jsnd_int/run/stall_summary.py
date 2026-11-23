#!/usr/bin/env python3
"""Corner-stall summary from a race_mpc*.log (controller_manager [MPC] status-change lines).
usage: python3 stall_summary.py <log> [<log> ...]   (pass yesterday's log too for before/after)
STOP   = '[MPC] STOP requested: <reason>'  -> speed 0 command (the stall)
CONT   = '[MPC] <status>' warn with success  -> step 17 plan continuation (no stall)
Rates are per minute of controller activity (first->last [MPC] line)."""
import re, sys, collections
pat = re.compile(r'\[controller_manager[^\]]*\] \[(\w+)\] \[([\d.]+)\] \[controller_manager\]: \[MPC\] (.*)')
for path in sys.argv[1:]:
    stop, cont, ts = collections.Counter(), collections.Counter(), []
    track = 0
    for line in open(path, errors='replace'):
        m = pat.search(line)
        if not m: continue
        lvl, t, msg = m.group(1), float(m.group(2)), m.group(3).strip()
        ts.append(t)
        if msg.startswith('STOP requested:'): stop[msg.split(':', 1)[1].strip()] += 1
        elif msg.startswith('tracking'): track += 1
        elif lvl == 'WARN': cont[msg] += 1
    dur = (ts[-1] - ts[0]) if len(ts) > 1 else 0.0
    rate = lambda n: f"{n / (dur / 60):.1f}/min" if dur > 0 else "-"
    print(f"== {path}\n   span {dur:.0f} s | STOP {sum(stop.values())} ({rate(sum(stop.values()))}) | "
          f"continuation {sum(cont.values())} | recover->tracking {track}")
    for k, v in stop.most_common(6): print(f"   STOP  {v:5d}  {k}")
    for k, v in cont.most_common(4): print(f"   CONT  {v:5d}  {k}")
