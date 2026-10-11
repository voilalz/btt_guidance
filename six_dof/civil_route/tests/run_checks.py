"""Strict C build and actual component/CLI checks."""
from pathlib import Path
import subprocess
import sys
import tempfile

MODULE=Path(__file__).resolve().parents[1]
SD6=MODULE.parent

def build(name,exe):
    assert (MODULE/"c/civil_route.c").exists(),"civil route model not implemented"
    src=MODULE/("c/main.c" if name=="demo" else "tests/check_components.c")
    assert src.exists(),f"civil route {name} not implemented"
    subprocess.run(["gcc","-std=c99","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(MODULE/"c"),"-I",str(SD6/"boost_cruise/c"),"-I",str(SD6/"c"),str(src),
                    str(MODULE/"c/controllers.c"),str(MODULE/"c/civil_route.c"),
                    str(SD6/"boost_cruise/c/boost_cruise.c"),str(SD6/"c/six_dof.c"),"-lm","-o",str(exe)],check=True)

def main():
    with tempfile.TemporaryDirectory() as tmp:
        exe=Path(tmp)/"components";build("components",exe);subprocess.run([str(exe)],check=True)
        if (MODULE/"c/main.c").exists():
            exe=Path(tmp)/"demo";build("demo",exe)
            subprocess.run([sys.executable,str(MODULE/"tests/check_cli.py"),str(exe)],check=True)

if __name__=="__main__":main()
