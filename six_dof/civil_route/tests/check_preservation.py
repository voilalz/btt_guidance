from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[3]
def main():
    baseline=json.loads((Path(__file__).parent/"previous_files_baseline.json").read_text())
    for path,expected in baseline.items():
        raw=(ROOT/path).read_bytes()
        actual=hashlib.sha1(f"blob {len(raw)}\0".encode()+raw).hexdigest()
        assert actual==expected,f"previous file changed: {path}"
    print(f"PASS: {len(baseline)} previous core, test, model and result blobs unchanged")

if __name__=="__main__":main()
