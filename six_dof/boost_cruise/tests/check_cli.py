import csv
from pathlib import Path
import subprocess
import sys
import tempfile
import math

exe=str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory() as tmp:
    out=Path(tmp)/"test.csv"
    for args in ([],[str(out),"nan"],[str(out),"0"],[str(out),"1","0"],
                 [str(out),"inf"],[str(out),"1","-1"],[str(out),"1x"],
                 [str(out),"1",".005","0"],[str(out),"1",".005","-1"],
                 [str(Path(tmp)/"missing"/"test.csv")]):
        p=subprocess.run([exe,*args],capture_output=True)
        assert p.returncode!=0,args
    for duration,dt,n in ((1e-12,.005,2),(.123,.005,4),(.1,.005,3)):
        subprocess.run([exe,str(out),str(duration),str(dt),"10"],check=True,capture_output=True)
        with out.open() as f:rows=list(csv.DictReader(f))
        assert len(rows)==n,(duration,len(rows),n)
        assert float(rows[0]["y_m"])==0 and float(rows[0]["speed_mps"])==0
        assert float(rows[-1]["t_s"])==duration
        assert all(float(a["t_s"])<float(b["t_s"]) for a,b in zip(rows,rows[1:]))
        assert all(math.isfinite(float(v)) for row in rows for v in row.values())
    print("PASS: CLI numeric/file errors, static initial row and tiny/final output times")
