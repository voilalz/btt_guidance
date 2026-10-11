"""Exercise the real CLI's numeric input validation and output timing."""
import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile

exe = str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory() as tmp:
    out = Path(tmp) / "case.csv"
    for bad in (["invalid", str(out)], ["constant", str(out), "nan"],
                ["constant", str(out), "inf"], ["constant", str(out), "0"],
                ["constant", str(out), "1", "0"], ["constant", str(out), "1", "-1"],
                ["constant", str(out), "1junk"], ["constant", str(out), "1", ".01", "0"],
                ["constant", str(out), "1", ".01", "-1"],
                ["constant", str(Path(tmp)/"missing"/"case.csv")]):
        result = subprocess.run([exe, *bad], capture_output=True, text=True)
        assert result.returncode != 0, f"invalid input accepted: {bad}"
    for duration, dt, expected in ((1e-12,.01,2),(.123,.01,4),(.1,.01,3)):
        subprocess.run([exe,"constant",str(out),str(duration),str(dt),"5"],check=True,capture_output=True)
        with out.open() as f:
            rows = list(csv.DictReader(f))
        assert len(rows)==expected, (duration,len(rows),expected)
        times=[float(r["t_s"]) for r in rows]
        assert times[0]==0 and times[-1]==duration, times
        assert all(a<b for a,b in zip(times,times[1:])), times
        assert all(math.isfinite(float(v)) for r in rows for v in r.values())
        for r in rows:
            assert abs(float(r["y_m"])+float(r["D_m"]))<1e-11
            assert abs(float(r["z_m"])-float(r["E_m"]))<1e-11
    print("PASS: CLI invalid inputs, tiny duration, exact final sample and output coordinates")
