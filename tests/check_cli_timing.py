"""校核实际命令行运行的极短合法时长，避免固定绝对时间容差跳过积分。"""
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

exe=Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory() as tmp:
    out=Path(tmp)/'tiny.csv'
    subprocess.run([str(exe),'constant',str(out),'1e-11','1e-11'],check=True,capture_output=True)
    with out.open() as f:
        rows=list(csv.DictReader(f))
    assert len(rows)==2, f'Expected initial and final rows, got {len(rows)}'
    assert abs(float(rows[-1]['t_s'])-1e-11)<1e-25
print('PASS: tiny positive duration still executes one integration step')
