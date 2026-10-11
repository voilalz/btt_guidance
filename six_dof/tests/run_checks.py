"""Build with strict C99 warnings and run the selected analytic checks."""
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SD6 = ROOT / "six_dof"

def build(name, output):
    source = SD6 / ("c/main.c" if name == "demo" else f"tests/check_{name}.c")
    assert source.is_file(), f"missing six-DOF {name} implementation/test"
    assert (SD6 / "c/six_dof.c").is_file(), "six-DOF dynamics core not implemented"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I", str(SD6 / "c"), str(source), str(SD6 / "c/six_dof.c"),
                    "-lm", "-o", str(output)], check=True)

def main():
    group = sys.argv[1] if len(sys.argv) > 1 else "all"
    assert group in ("core", "events", "all")
    with tempfile.TemporaryDirectory() as tmp:
        for name in ("core", "events"):
            if group in (name, "all"):
                exe = Path(tmp) / name
                build(name, exe)
                subprocess.run([str(exe)], check=True)
        if group in ("events", "all"):
            exe = Path(tmp) / "demo"
            build("demo", exe)
            subprocess.run([sys.executable, str(SD6 / "tests/check_cli.py"), str(exe)], check=True)

if __name__ == "__main__":
    main()
