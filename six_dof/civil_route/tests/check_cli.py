import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

exe=str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory() as tmp:
    out=Path(tmp)/"out.csv"
    for args in ([],["unknown",str(out)],["high",str(out),"nan"],["high",str(out),"0"],
                 ["high",str(out),"1","-1"],["high",str(out),"inf"],
                 ["high",str(out),"1",".005","0"],["high",str(out),"1",".005","-1"],
                 ["high",str(out),"1",".005","10","invalid"],
                 ["high",str(Path(tmp)/"missing"/"out.csv")]):
        assert subprocess.run([exe,*args],capture_output=True).returncode!=0,args
    for duration,n in ((1e-12,2),(.123,4),(.1,3)):
        subprocess.run([exe,"high",str(out),str(duration),".005","10"],check=True,capture_output=True)
        with out.open() as f:rows=list(csv.DictReader(f))
        assert len(rows)==n,(duration,len(rows))
        assert float(rows[0]["height_m"])==0 and float(rows[0]["speed_mps"])==0
        assert float(rows[-1]["t_s"])==duration
        assert all(float(a["t_s"])<float(b["t_s"]) for a,b in zip(rows,rows[1:]))
        assert all(math.isfinite(float(v)) for r in rows for v in r.values())
        report=json.loads(Path(str(out)+".json").read_text())
        assert report["end_time_s"]==duration and report["termination"]=="duration"
        assert report.get("dt_s")==.005 and report.get("output_stride")==10,"run timing metadata absent"
    subprocess.run([exe,"high",str(out),"12",".005","10","direct","slew","launch"],check=True,capture_output=True)
    report=json.loads(Path(str(out)+".json").read_text())
    assert report["controllers"]==["direct","slew","launch"]
    print("PASS: CLI numeric/file/controller errors, tiny duration, final time and callback selection")
