"""Verify preserved blobs and run original checks in a temporary copy.

The old numerical verifier writes its own JSON report, so it runs outside the
delivery tree. No baseline result or original source is rewritten here.
"""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[2]
def main():
    baseline=json.loads((Path(__file__).parent/"three_dof_baseline.json").read_text())
    for path,expected in baseline.items():
        data=(ROOT/path).read_bytes()
        actual=hashlib.sha1(f"blob {len(data)}\0".encode()+data).hexdigest()
        assert actual==expected,f"protected file changed: {path}"
    print(f"PASS: {len(baseline)} protected original/spec blobs unchanged",flush=True)
    with tempfile.TemporaryDirectory() as tmp:
        copy=Path(tmp)/"three_dof";copy.mkdir()
        for folder in ("c","tests","results"):
            shutil.copytree(ROOT/folder,copy/folder)
        exe=copy/"demo"
        subprocess.run(["gcc","-std=c99","-O2","-Wall","-Wextra","-Werror",
                        str(copy/"c/main.c"),str(copy/"c/trajectory.c"),"-lm","-o",str(exe)],check=True)
        for script,args in (("run_checks.py",[]),("check_cli_timing.py",[str(exe)]),
                            ("verify_outputs.py",[str(exe)])):
            result=subprocess.run([sys.executable,str(copy/"tests"/script),*args],check=True,
                                  capture_output=True,text=True)
            if script=="verify_outputs.py":
                report=json.loads((copy/"results/verification.json").read_text())
                print("PASS: original independent DOP853, convergence and CLI boundaries",flush=True)
                for mode,case in report["cases"].items():
                    print(f"  {mode}: DOP853 position max error {case['independent_DOP853_position_error_max_m']:.3g} m")
            else:print(result.stdout.strip(),flush=True)

if __name__=="__main__":main()
