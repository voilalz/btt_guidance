"""Run real C scenes, verify saved bytes and compare a half-step simulation."""
from pathlib import Path
import json
import subprocess
import sys
import tempfile
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from run_checks import MODULE,build

SCENES=("high","crossing","monitor")
def read(path):
    a=np.genfromtxt(path,delimiter=",",names=True)
    assert len(a)>100 and all(np.isfinite(a[n]).all() for n in a.dtype.names)
    assert a["t_s"][0]==0 and a["t_s"][-1]==120 and np.all(np.diff(a["t_s"])>0)
    assert a["height_m"][0]==0 and a["speed_mps"][0]==0
    assert set(a["phase"].astype(int))=={0,1,2}
    assert np.max(abs(a["mass_kg"]-11.7-a["boost_fuel_kg"]-a["cruise_fuel_kg"]))<1e-12
    assert np.min(a["boost_fuel_kg"])>=0 and np.min(a["cruise_fuel_kg"])>=0
    q=np.column_stack([a[n] for n in ("quat_a","quat_b","quat_c","quat_d")])
    assert np.max(abs(np.linalg.norm(q,axis=1)-1))<1e-12
    sep=np.sqrt((a["traffic_x_m"]-a["x_m"])**2+(a["traffic_height_m"]-a["height_m"])**2+(a["traffic_z_m"]-a["z_m"])**2)
    assert np.max(abs(sep-a["separation_m"]))<1e-9
    summary=json.loads(Path(str(path)+".json").read_text())
    assert summary["end_time_s"]==120 and summary["termination"]=="duration"
    assert all(x>0 for x in summary["controller_calls"])
    assert summary["minimum_separation_m"]<=float(a["separation_m"].min())+1e-8
    assert np.all(a["traffic_height_m"]==summary["traffic_height_m"])
    return a,summary

def main():
    generate="--generate" in sys.argv[1:];folder=MODULE/"results"
    if not generate:assert (folder/"high.csv").exists(),"civil trajectory not generated"
    else:folder.mkdir(exist_ok=True)
    reports={}
    with tempfile.TemporaryDirectory() as tmp:
        tmp=Path(tmp);exe=tmp/"demo";build("demo",exe)
        for scene in SCENES:
            path=folder/(scene+".csv");fresh=path if generate else tmp/(scene+".csv")
            subprocess.run([str(exe),scene,str(fresh),"120",".005","10"],check=True,capture_output=True)
            if not generate:
                assert fresh.read_bytes()==path.read_bytes(),"saved trajectory differs: "+scene
                assert Path(str(fresh)+".json").read_bytes()==Path(str(path)+".json").read_bytes(),"saved summary differs: "+scene
            a,r=read(path);reports[scene]=r
            if scene=="high":assert r["traffic_height_m"]==10000 and r["safety_activations"]==0
            if scene=="crossing":assert r["minimum_separation_m"]>=80 and r["safety_activations"]>0
            if scene=="monitor":assert r["safety_activations"]==0
            fine=tmp/(scene+"_fine.csv")
            subprocess.run([str(exe),scene,str(fine),"120",".0025","20"],check=True,capture_output=True)
            b,rf=read(fine)
            ia=np.flatnonzero(abs(a["t_s"]/.05-np.rint(a["t_s"]/.05))<1e-8)
            ib=np.flatnonzero(abs(b["t_s"]/.05-np.rint(b["t_s"]/.05))<1e-8)
            assert len(ia)==len(ib)==2401
            pos=np.column_stack([a[n][ia]-b[n][ib] for n in ("x_m","height_m","z_m")])
            vel=np.column_stack([a[n][ia]-b[n][ib] for n in ("vx_mps","vh_mps","vz_mps")])
            r["half_step_position_m"]=float(np.linalg.norm(pos,axis=1).max())
            r["half_step_velocity_mps"]=float(np.linalg.norm(vel,axis=1).max())
            assert r["half_step_position_m"]<.1 and r["half_step_velocity_mps"]<.02,r
            assert abs(r["minimum_separation_m"]-rf["minimum_separation_m"])<.1
        assert reports["crossing"]["minimum_separation_m"]>reports["monitor"]["minimum_separation_m"]+50
        alternate=tmp/"alternate.csv"
        subprocess.run([str(exe),"high",str(alternate),"120",".005","10","direct","slew","direct"],check=True,capture_output=True)
        _,ra=read(alternate);assert ra["controllers"]==["direct","slew","direct"] and all(x>0 for x in ra["controller_calls"])
        report={"scenes":reports,"alternate_controllers":ra["controllers"],"matlab_executed":False,"cmake_executed":False}
        out=folder/"verification.json"
        if generate:out.write_text(json.dumps(report,indent=2)+"\n")
        else:assert json.loads(out.read_text())==report,"verification metadata differs"
        print(json.dumps(report,indent=2))
        print("PASS: all phases, scene geometry, actual separation, callbacks, saved bytes and half-step comparisons")

if __name__=="__main__":main()
