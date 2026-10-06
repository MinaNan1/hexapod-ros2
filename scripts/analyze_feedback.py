#!/usr/bin/env python3
"""
analyze_feedback.py — compare COMMANDED vs ACTUAL servo positions from a recording
made by the bridge (record_on/record_off → feedback_log.csv).

Answers the key questions for the no-sag closed-loop:
  • which servos track worst (biggest error)?
  • does the error GROW over the run? (= progressive sag, the servos slowly
    falling behind / drooping under sustained load)
  • per-leg, which joints droop?

Usage:  python3 analyze_feedback.py [feedback_log.csv]
(pull the CSV off the Jetson first, or run on the Jetson.)
"""
import sys, csv

CSV = sys.argv[1] if len(sys.argv) > 1 else "feedback_log.csv"
# servo id → "leg joint"
NAME = {1:"rightFront coxa",2:"rightFront femur",3:"rightFront tibia",
        4:"rightMiddle coxa",5:"rightMiddle femur",6:"rightMiddle tibia",
        7:"rightBack coxa",8:"rightBack femur",9:"rightBack tibia",
        10:"leftBack coxa",11:"leftBack femur",12:"leftBack tibia",
        13:"leftMiddle coxa",14:"leftMiddle femur",15:"leftMiddle tibia",
        16:"leftFront coxa",17:"leftFront femur",18:"leftFront tibia"}

rows = []
with open(CSV) as f:
    r = csv.DictReader(f)
    cols = r.fieldnames
    for row in r:
        rows.append(row)
if not rows:
    print("empty CSV"); sys.exit(1)

ids = sorted(int(c[3:]) for c in cols if c.startswith("act"))
n = len(rows)
q = max(1, n // 4)
dur = float(rows[-1]['t']) - float(rows[0]['t'])
print(f"{n} samples over {dur:.1f}s ({n/max(dur,1e-9):.0f} Hz). units: 1000=240deg (~4.2/deg)\n")

def err_series(sid):
    out = []
    for row in rows:
        c, a = row.get(f'cmd{sid}',''), row.get(f'act{sid}','')
        if c not in ('', None) and a not in ('', None):
            out.append(int(c) - int(a))     # signed: + = actual LAGS BELOW command
    return out

stats = []
for sid in ids:
    e = err_series(sid)
    if not e: continue
    absmean = sum(abs(x) for x in e)/len(e)
    first = sum(abs(x) for x in e[:q])/max(1,len(e[:q]))
    last  = sum(abs(x) for x in e[-q:])/max(1,len(e[-q:]))
    bias  = sum(e)/len(e)
    stats.append((sid, absmean, first, last, last-first, max(abs(x) for x in e), bias))

# rank by END error (the droopers at the end of the walk)
stats.sort(key=lambda s: -s[3])
print(f"{'servo':22} {'|err|':>6} {'start':>6} {'end':>6} {'grow':>6} {'max':>5} {'bias':>6}   (units)")
print("-"*78)
for sid, am, fi, la, gr, mx, bi in stats:
    flag = "  <-- GROWS (progressive)" if gr > 6 and la > 10 else ""
    print(f"{NAME[sid]:22} {am:6.1f} {fi:6.1f} {la:6.1f} {gr:+6.1f} {mx:5.0f} {bi:+6.1f}{flag}")

growers = [s for s in stats if s[4] > 6 and s[3] > 10]
print("\nVERDICT:")
if growers:
    print(f"  → error GROWS over the run on {len(growers)} servo(s): "
          + ", ".join(NAME[s[0]] for s in growers))
    print("    = progressive — the servos fall further behind the longer it walks.")
    print("    If 'bias' is consistently one sign, they're drooping in that direction.")
    print("    Closed-loop 'wait for catch-up' should help IF they eventually reach")
    print("    the target between strides; if 'end' error stays huge, they're load-")
    print("    saturated → reduce load (lift/stride/height) instead of gating.")
else:
    print("  → error is roughly STEADY (not growing). The sag is a constant tracking")
    print("    lag, not accumulation — a fixed comp/feed-forward offset would fix it.")
