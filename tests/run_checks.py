"""编译并运行解析物理算例；需要Python标准库和C99编译器。"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class CorePhysicsTests(unittest.TestCase):
    def test_analytic_physics_cases(self):
        self.assertTrue((ROOT / "c/trajectory.c").is_file(), "C动力学核心尚未实现")
        with tempfile.TemporaryDirectory() as tmp:
            exe = str(Path(tmp) / "check_core")
            subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "c"), str(ROOT / "tests/check_core.c"),
                            str(ROOT / "c/trajectory.c"), "-lm", "-o", exe], check=True)
            result = subprocess.run([exe], check=True, capture_output=True, text=True)
            print(result.stdout, end="")

if __name__ == "__main__":
    unittest.main()
