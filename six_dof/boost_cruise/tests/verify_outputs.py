"""Generate/check the static-launch default and independently run half-step C."""
from pathlib import Path
import json
import subprocess
import sys
import tempfile
import numpy as np

BC=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path(__file__).parent))
from run_checks import build

def read(path):
    assert path.exists(),"boost/cruise trajectory not generated"
    a=np.genfromtxt(path,delimiter=",",names=True)
    assert all(np.isfinite(a[n]).all() for n in a.dtype.names)
    assert len(a)>2 and a["t_s"][0]==0 and np.all(np.diff(a["t_s"])>0)
    assert a["y_m"][0]==0 and a["speed_mps"][0]==0
    assert np.max(abs(a["mass_kg"]-11.7-a["boost_fuel_kg"]-a["cruise_fuel_kg"]))<1e-12
    assert np.min(a["boost_fuel_kg"])>=0 and np.min(a["cruise_fuel_kg"])>=0
    assert np.all(np.diff(a["mass_kg"])<=1e-12)
    q=np.column_stack([a[n] for n in ("quat_a","quat_b","quat_c","quat_d")])
    assert np.max(abs(np.linalg.norm(q,axis=1)-1))<=1e-12
    ign=np.where(a["event_mask"].astype(int)&1)[0];burn=np.where(a["event_mask"].astype(int)&2)[0]
    assert len(ign)==1 and len(burn)==1
    i,j=int(ign[0]),int(burn[0]);assert a["y_m"][i]>=40-1e-8 and a["speed_mps"][i]>=25-1e-8
    assert a["thrust_N"][i]==0 and a["cruise_ignited"][i]==1
    assert np.all(a["cruise_ignited"][i:]==1)
    assert abs(a["t_s"][j]-4)<1e-12 and a["boost_thrust_N"][j]==0 and a["boost_fuel_kg"][j]==0
    assert np.all(a["boost_thrust_N"][a["t_s"]<4]==260)
    assert np.all(a["boost_thrust_N"][a["t_s"]>=4]==0)
    assert np.max(a["thrust_N"][a["stage"]==1])>0,"no actual propulsion overlap"
    assert np.all(a["thrust_cmd_N"][a["cruise_ignited"]==1]>=3-1e-12),"ignited idle command missing"
    assert a["t_s"][-1]==120,"default did not sustain the requested cruise"
    assert np.max(abs(a["y_m"][a["t_s"]>=60]-40))<2,"height stabilization not demonstrated"
    assert np.max(abs(a["speed_mps"][a["t_s"]>=60]-25))<1,"speed stabilization not demonstrated"
    return a,q,i,j

def main():
    generate="--generate" in sys.argv[1:];results=BC/"results";out=results/"boost_cruise.csv"
    if not generate:assert out.exists(),"boost/cruise trajectory not generated"
    else:results.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        exe=Path(tmp)/"demo";build("demo",exe)
        if generate:subprocess.run([str(exe),str(out),"120",".005","10"],check=True)
        else:
            reproduced=Path(tmp)/"coarse.csv"
            subprocess.run([str(exe),str(reproduced),"120",".005","10"],check=True,capture_output=True)
            assert reproduced.read_bytes()==out.read_bytes(),"saved C trajectory not byte-reproducible"
        fine_path=Path(tmp)/"fine.csv"
        subprocess.run([str(exe),str(fine_path),"120",".0025","20"],check=True,capture_output=True)
        a,q,i,j=read(out);b,r,ib,jb=read(fine_path)
        ca=np.where(abs(a["t_s"]/.05-np.rint(a["t_s"]/.05))<1e-8)[0]
        cb=np.where(abs(b["t_s"]/.05-np.rint(b["t_s"]/.05))<1e-8)[0]
        assert len(ca)==2401 and len(cb)==2401
        assert np.max(abs(a["t_s"][ca]-b["t_s"][cb]))<1e-9
        pos=np.column_stack([a[n][ca]-b[n][cb] for n in ("x_m","y_m","z_m")])
        vel=np.column_stack([a[n][ca]-b[n][cb] for n in ("vx_mps","vy_mps","vz_mps")])
        signs=np.where(np.sum(q[ca]*r[cb],axis=1)<0,-1,1)
        angle=np.rad2deg(4*np.arcsin(np.clip(np.linalg.norm(q[ca]-signs[:,None]*r[cb],axis=1)/2,0,1)))
        conv={"position_m":float(np.max(np.linalg.norm(pos,axis=1))),
              "velocity_mps":float(np.max(np.linalg.norm(vel,axis=1))),"attitude_deg":float(np.max(angle))}
        assert conv["position_m"]<.05 and conv["velocity_mps"]<.01 and conv["attitude_deg"]<.05,conv
        assert abs(a["t_s"][i]-b["t_s"][ib])<1e-4
        report={"dt_s":.005,"comparison_dt_s":.0025,"output_stride":10,"duration_s":120,
                "matlab_executed":False,"cmake_executed":False,"samples":len(a),"common_grid_samples":2401,
                "initial_height_m":0,"initial_speed_mps":0,"initial_mass_kg":float(a["mass_kg"][0]),
                "boost_thrust_N":260,"burn_duration_s":4,"cruise_idle_thrust_N":3,
                "ignition_height_m":40,"ignition_speed_mps":25,"cruise_height_m":40,"cruise_speed_mps":25,
                "ignition":{"t_s":float(a["t_s"][i]),"height_m":float(a["y_m"][i]),"speed_mps":float(a["speed_mps"][i])},
                "burnout":{"t_s":float(a["t_s"][j]),"height_m":float(a["y_m"][j]),"speed_mps":float(a["speed_mps"][j]),
                           "cruise_actual_thrust_N":float(a["thrust_N"][j])},
                "height_range_m":[float(a["y_m"].min()),float(a["y_m"].max())],
                "speed_range_mps":[float(a["speed_mps"].min()),float(a["speed_mps"].max())],
                "final_position_m":[float(a[n][-1]) for n in ("x_m","y_m","z_m")],
                "final_speed_mps":float(a["speed_mps"][-1]),"final_mass_kg":float(a["mass_kg"][-1]),
                "final_boost_fuel_kg":float(a["boost_fuel_kg"][-1]),"final_cruise_fuel_kg":float(a["cruise_fuel_kg"][-1]),
                "post_60s_max_height_error_m":float(np.max(abs(a["y_m"][a["t_s"]>=60]-40))),
                "post_60s_max_speed_error_mps":float(np.max(abs(a["speed_mps"][a["t_s"]>=60]-25))),
                "max_quaternion_norm_error":float(np.max(abs(np.linalg.norm(q,axis=1)-1))),
                "convergence":conv,"termination":"duration"}
        if generate:(results/"verification.json").write_text(json.dumps(report,indent=2)+"\n")
        else:assert json.loads((results/"verification.json").read_text())==report,"summary not reproducible"
        print(json.dumps(report,indent=2))
        print("PASS: actual lift-off, AND-gate handoff, burnout, sustained cruise and half-step comparison")

if __name__=="__main__":main()
