"""交付前的独立校核：解析气动力、SciPy DOP853、步长收敛与CLI边界。
仅此校核脚本依赖numpy/scipy；用户运行C程序不需要它们。
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
EXE = Path(sys.argv[1]).resolve()
G, MASS0, MASS_DRY = 9.80665, 12.0, 11.7
RHO0, H, AREA, V0, Y0 = 1.225, 8500.0, 0.8, 25.0, 200.0
QS0 = 0.5 * RHO0*np.exp(-Y0/H)*V0**2*AREA
ALPHA0 = brentq(lambda a: QS0*(0.2+4.5*a)+12*np.sin(a)-MASS0*G, 0, 0.2)

def independent_rhs(t, s, variable):
    # 从文档的矢量方程直接建立，不调用C核心。
    vel=s[3:6]; speed=np.linalg.norm(vel); ev=vel/speed
    vertical=np.array([0.0,1.0,0.0])
    up=vertical-np.dot(vertical,ev)*ev; up/=np.linalg.norm(up)
    side=np.cross(ev,up)
    alpha=ALPHA0+np.deg2rad(0.25)*np.sin(2*np.pi*t/30)
    bank=np.deg2rad(8)*np.sin(2*np.pi*t/60)
    en=np.cos(bank)*up+np.sin(bank)*side
    rho=RHO0*np.exp(-max(s[1],0.0)/H)
    cl=np.clip(0.2+4.5*alpha,-0.6,1.2); cd=0.035+0.06*cl**2
    qs=0.5*rho*speed**2*AREA
    throttle=0.2
    if variable:
        throttle+=0.15*np.sin(2*np.pi*t/40)+0.05*np.sin(2*np.pi*t/15)
    command=60*np.clip(throttle,0.0,1.0)
    thrust=s[7]
    force=(thrust*np.cos(alpha)-qs*cd)*ev+(thrust*np.sin(alpha)+qs*cl)*en
    acceleration=force/s[6]+np.array([0.0,-G,0.0])
    return np.r_[vel,acceleration,-(5e-5+1.5e-5*thrust),command-thrust]

report={"runtime":"GCC/C99 (MATLAB/Octave unavailable)","cases":{},"checks":[]}
with tempfile.TemporaryDirectory() as tmp:
    for mode in ('constant','variable'):
        outputs={}
        for dt in (0.2,0.1,0.05,0.02,0.01):
            out=Path(tmp)/f'{mode}_{dt}.csv'
            subprocess.run([str(EXE),mode,str(out),str(dt),'120'],check=True,capture_output=True)
            outputs[dt]=np.genfromtxt(out,delimiter=',',names=True)
        a=outputs[0.02]
        assert len(a)==6001 and abs(a['t_s'][-1]-120)<1e-9
        assert all(np.isfinite(a[name]).all() for name in a.dtype.names)
        assert np.all(np.diff(a['mass_kg'])<=1e-10)
        assert np.min(a['mass_kg'])>=MASS_DRY and np.min(a['y_m'])>0
        assert np.min(a['thrust_N'])>=0 and np.max(a['thrust_N'])<=60
        initial=np.array([0,Y0,0,V0,0,0,MASS0,12],dtype=float)
        ref=solve_ivp(lambda t,s: independent_rhs(t,s,mode=='variable'),[0,120],initial,
                      method='DOP853',rtol=2e-12,atol=1e-12)
        assert ref.success,ref.message
        cols=['x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','mass_kg','thrust_N']
        end=np.array([a[name][-1] for name in cols])
        ref_err=np.abs(end-ref.y[:,-1])
        assert max(ref_err[:3])<2e-5,ref_err
        coarse=np.array([outputs[0.1][name][-1] for name in cols])
        medium=np.array([outputs[0.05][name][-1] for name in cols])
        fine=np.array([outputs[0.01][name][-1] for name in cols])
        assert max(np.abs(end[:3]-fine[:3]))<1e-4
        # 120 s内每个正弦恰好走整数个周期，指令平均值均为12 N。
        trapz=getattr(np,'trapezoid',np.trapz)
        assert abs(trapz(a['thrust_cmd_N'],a['t_s'])/120-12)<1e-8
        report['cases'][mode]={
            'rows':len(a),'time_s':float(a['t_s'][-1]),
            'end_position_m':[float(a[name][-1]) for name in ('x_m','y_m','z_m')],
            'end_speed_mps':float(a['speed_mps'][-1]),
            'height_minmax_m':[float(a['y_m'].min()),float(a['y_m'].max())],
            'speed_minmax_mps':[float(a['speed_mps'].min()),float(a['speed_mps'].max())],
            'fuel_used_kg':float(MASS0-a['mass_kg'][-1]),
            'independent_DOP853_position_error_max_m':float(max(ref_err[:3])),
            'dt_002_vs_001_position_error_max_m':float(max(np.abs(end[:3]-fine[:3]))),
            'dt_01_vs_002_position_error_max_m':float(max(np.abs(coarse[:3]-end[:3]))),
            'dt_005_vs_002_position_error_max_m':float(max(np.abs(medium[:3]-end[:3])))}
    for args in [('invalid','0.02','120'),('variable','0','120'),('variable','-1','120'),
                 ('variable','nan','120'),('constant','0.02','0'),('constant','0.02','inf')]:
        mode,dt,end=args
        proc=subprocess.run([str(EXE),mode,str(Path(tmp)/'invalid.csv'),dt,end],capture_output=True)
        assert proc.returncode==2,(args,proc.returncode)
report['checks']=['two complete finite trajectories','monotone bounded mass','bounded thrust',
                  'equal mean thrust commands','step-size convergence','independent DOP853 agreement',
                  'six invalid command-line cases rejected']
(ROOT/'results/verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
