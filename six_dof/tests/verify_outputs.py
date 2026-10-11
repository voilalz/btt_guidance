"""Validate final C CSVs against a half-step run; --generate creates defaults.

All comparisons use regular common output times, plus separate event checks in
check_events.c. No interpolation of attitude or extrapolation after termination.
"""
from pathlib import Path
import json
import subprocess
import sys
import tempfile
import numpy as np

SD6 = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from run_checks import build

def read(path):
    assert path.is_file(), f"six-DOF result not generated: {path}"
    a=np.genfromtxt(path,delimiter=",",names=True)
    assert len(a)>=2
    assert all(np.isfinite(a[n]).all() for n in a.dtype.names)
    assert a["t_s"][0]==0 and np.all(np.diff(a["t_s"])>0)
    assert np.all(a["mass_kg"]>=11.7-1e-10)
    assert np.all((a["thrust_N"]>=0)&(a["thrust_N"]<=60+1e-9))
    assert np.all((a["throttle"]>=0)&(a["throttle"]<=1))
    q=np.column_stack([a[n] for n in ("quat_a","quat_b","quat_c","quat_d")])
    err=float(np.max(abs(np.linalg.norm(q,axis=1)-1)))
    assert err<=1e-12
    assert np.max(a["quat_norm_error"])<=1e-12
    assert np.max(abs(a["y_m"]+a["D_m"]))<1e-11
    assert np.max(abs(a["z_m"]-a["E_m"]))<1e-11
    assert abs(a["alpha_rad"][0]-np.deg2rad(2.438596874881059))<1e-12
    assert abs(a["thrust_N"][0]-13.232733930207198)<1e-9
    assert abs(a["pitch_rad"][0]-a["alpha_rad"][0])<1e-12
    assert abs(a["vx_mps"][0]-25)<1e-10 and abs(a["vy_mps"][0])<1e-10
    return a,q,err

def compare(coarse,fine,qc,qf):
    # dt=.01 stride=5 and dt=.005 stride=10 have a shared 0.05 s grid.
    ic=np.where(abs(coarse["t_s"]/.05-np.rint(coarse["t_s"]/.05))<1e-8)[0]
    jf=np.where(abs(fine["t_s"]/.05-np.rint(fine["t_s"]/.05))<1e-8)[0]
    count=min(len(ic),len(jf));ic=ic[:count];jf=jf[:count]
    assert count>2 and np.max(abs(coarse["t_s"][ic]-fine["t_s"][jf]))<1e-9
    pos=np.column_stack([coarse[n][ic]-fine[n][jf] for n in ("x_m","y_m","z_m")])
    vel=np.column_stack([coarse[n][ic]-fine[n][jf] for n in ("vx_mps","vy_mps","vz_mps")])
    # Chord length avoids arccos round-off near identical quaternions.
    sign=np.where(np.sum(qc[ic]*qf[jf],axis=1)<0,-1.,1.)
    chord=np.linalg.norm(qc[ic]-sign[:,None]*qf[jf],axis=1)
    angle=np.rad2deg(4*np.arcsin(np.clip(chord/2,0,1)))
    result={"max_position_difference_m":float(np.max(np.linalg.norm(pos,axis=1))),
            "max_velocity_difference_mps":float(np.max(np.linalg.norm(vel,axis=1))),
            "max_attitude_difference_deg":float(np.max(angle)),"common_samples":count}
    assert result["max_position_difference_m"]<.05,result
    assert result["max_velocity_difference_mps"]<.01,result
    assert result["max_attitude_difference_deg"]<.05,result
    # Ending time includes located ground time if that event occurs.
    assert abs(coarse["t_s"][-1]-fine["t_s"][-1])<.001
    return result

def main():
    generate="--generate" in sys.argv[1:]
    results=SD6/"results"
    if not generate:
        assert (results/"constant.csv").is_file(),"six-DOF result not generated"
        assert (results/"variable.csv").is_file(),"six-DOF result not generated"
    else:
        results.mkdir(exist_ok=True)
    report={"implementation":"C99, GCC strict warnings", "duration_requested_s":120,
            "dt_s":.01,"output_stride":5,"comparison_dt_s":.005,
            "matlab_executed":False,"cmake_executed":False,
            "parameters":{"mass0_kg":12,"mass_dry_kg":11.7,"speed0_mps":25,"height0_m":200,
                "area_m2":.8,"span_m":2,"chord_m":.4,"inertia_kgm2":[1,1.6,2.2],
                "g_mps2":9.80665,"rho0_kgm3":1.225,"scale_height_m":8500,
                "thrust_max_N":60,"thrust_tau_s":1,"fuel_c0_kgps":5e-5,"fuel_cT_kg_per_Ns":1.5e-5},
            "cases":{}}
    data={}
    with tempfile.TemporaryDirectory() as tmp:
        exe=Path(tmp)/"demo";build("demo",exe)
        for mode in ("constant","variable"):
            coarse_path=results/f"{mode}.csv";fine_path=Path(tmp)/f"{mode}.csv"
            if generate:
                subprocess.run([str(exe),mode,str(coarse_path),"120",".01","5"],check=True)
            subprocess.run([str(exe),mode,str(fine_path),"120",".005","10"],check=True,capture_output=True)
            coarse,qc,err=read(coarse_path);fine,qf,_=read(fine_path)
            convergence=compare(coarse,fine,qc,qf)
            data[mode]=coarse
            report["cases"][mode]={"samples":len(coarse),"end_time_s":float(coarse["t_s"][-1]),
                "termination":"ground" if coarse["y_m"][-1]==0 else "duration",
                "final_position_m":[float(coarse[n][-1]) for n in ("x_m","y_m","z_m")],
                "height_range_m":[float(np.min(coarse["y_m"])),float(np.max(coarse["y_m"]))],
                "speed_range_mps":[float(np.min(np.hypot(np.hypot(coarse["u_mps"],coarse["v_mps"]),coarse["w_mps"]))),float(np.max(np.hypot(np.hypot(coarse["u_mps"],coarse["v_mps"]),coarse["w_mps"])))],
                "final_mass_kg":float(coarse["mass_kg"][-1]),"max_quaternion_norm_error":err,
                "convergence":convergence}
            print(f"PASS: {mode} finite state, trim, unit quaternion and half-step convergence: {convergence}")
    a,b=data["constant"],data["variable"]
    count=min(len(a),len(b))
    assert np.max(abs(a["thrust_N"][:count]-b["thrust_N"][:count]))>1
    assert np.max(abs(a["y_m"][:count]-b["y_m"][:count]))>.01
    report["initial_trim"]={"alpha_rad":float(a["alpha_rad"][0]),"elevator_rad":float(a["delta_e_rad"][0]),
                            "throttle":float(a["throttle"][0]),"thrust_N":float(a["thrust_N"][0])}
    if generate:
        (results/"verification.json").write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n")
    else:
        saved=json.loads((results/"verification.json").read_text())
        assert saved==report,"saved verification metadata differs from current result checks"
    print("PASS: constant and variable thrust/trajectories differ; reproducible verification metadata")

if __name__=="__main__":main()
