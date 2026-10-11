"""Strict build and real C model/CLI checks (no NumPy needed)."""
from pathlib import Path
import subprocess
import sys
import tempfile

BC=Path(__file__).resolve().parents[1]
SD6=BC.parent
def build(name,output):
    assert (BC/"c/boost_cruise.c").is_file(),"boost/cruise model not implemented"
    source=BC/("c/main.c" if name=="demo" else "tests/check_model.c")
    assert source.is_file(),f"boost/cruise {name} not implemented"
    subprocess.run(["gcc","-std=c99","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(SD6/"c"),"-I",str(BC/"c"),str(source),
                    str(BC/"c/boost_cruise.c"),str(SD6/"c/six_dof.c"),"-lm","-o",str(output)],check=True)

def main():
    with tempfile.TemporaryDirectory() as tmp:
        exe=Path(tmp)/"model";build("model",exe);subprocess.run([str(exe)],check=True)
        if (BC/"c/main.c").is_file():
            exe=Path(tmp)/"demo";build("demo",exe)
            subprocess.run([sys.executable,str(BC/"tests/check_cli.py"),str(exe)],check=True)

if __name__=="__main__":main()
