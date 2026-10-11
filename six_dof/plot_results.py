"""Create scientific figures from the shared C/MATLAB CSV format."""
from pathlib import Path
import argparse
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("results",nargs="?",type=Path,default=Path(__file__).resolve().parent/"results")
    args=parser.parse_args();folder=args.results
    cases={mode:np.genfromtxt(folder/f"{mode}.csv",delimiter=",",names=True)
           for mode in ("constant","variable")}
    colors={"constant":"#2369a8","variable":"#dc6928"}
    labels={"constant":"Constant command","variable":"Variable command"}
    plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False,
                         "axes.grid":True,"grid.alpha":.25,"lines.linewidth":1.6})
    fig=plt.figure(figsize=(10,7));ax=fig.add_subplot(111,projection="3d")
    for mode,c in cases.items():
        ax.plot(c["x_m"],c["z_m"],c["y_m"],color=colors[mode],label=labels[mode])
    ax.set(xlabel="Forward x (m)",ylabel="Right z (m)",zlabel="Height y (m)",title="Six-DOF trajectory")
    ax.view_init(elev=25,azim=-60);ax.legend();ax.set_box_aspect((2,1,1))
    # Reserve room for the projected 3D height label; tight bbox can omit it.
    fig.subplots_adjust(left=.01,right=.85,bottom=.06,top=.92)
    fig.savefig(folder/"trajectory_3d.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(12,11),layout="constrained")
    for mode,c in cases.items():
        color=colors[mode];label=labels[mode];t=c["t_s"]
        axes[0,0].plot(c["x_m"],c["y_m"],color=color,label=label)
        axes[0,1].plot(t,c["thrust_N"],color=color,label=label+" actual")
        axes[0,1].plot(t,c["thrust_cmd_N"],"--",color=color,alpha=.7,label=label+" requested")
        speed=np.hypot(np.hypot(c["u_mps"],c["v_mps"]),c["w_mps"])
        axes[1,0].plot(t,speed,color=color,label=label)
        axes[1,1].plot(t,c["mass_kg"],color=color,label=label)
        axes[2,0].plot(t,np.rad2deg(c["alpha_rad"]),color=color,label=label+" alpha")
        axes[2,0].plot(t,np.rad2deg(c["beta_rad"]),"--",color=color,label=label+" beta")
        axes[2,1].plot(t,c["throttle"],color=color,label=label)
    axes[0,0].set(xlabel="Forward x (m)",ylabel="Height y (m)",title="Vertical trajectory")
    axes[0,1].set(xlabel="Time (s)",ylabel="Thrust (N)",title="First-order propulsion response")
    axes[1,0].set(xlabel="Time (s)",ylabel="Speed (m/s)",title="Airspeed (zero wind)")
    axes[1,1].set(xlabel="Time (s)",ylabel="Mass (kg)",title="Fuel consumption from actual thrust")
    axes[2,0].set(xlabel="Time (s)",ylabel="Angle (deg)",title="Aerodynamic angles")
    axes[2,1].set(xlabel="Time (s)",ylabel="Throttle",title="Continuous throttle input",ylim=(0,1))
    for ax in axes.flat:ax.legend(fontsize=8,loc="best")
    fig.suptitle("Six-DOF simulation | fictional parameters | dt = 0.01 s")
    fig.savefig(folder/"overview.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(12,10),layout="constrained")
    for row,(name,rate) in enumerate(zip(("roll","pitch","yaw"),("p","q","r"))):
        for mode,c in cases.items():
            axes[row,0].plot(c["t_s"],np.rad2deg(c[name+"_rad"]),color=colors[mode],label=labels[mode])
            axes[row,1].plot(c["t_s"],np.rad2deg(c[rate+"_radps"]),color=colors[mode],label=labels[mode])
        axes[row,0].set(xlabel="Time (s)",ylabel=f"{name.title()} (deg)")
        axes[row,1].set(xlabel="Time (s)",ylabel=f"{rate} (deg/s)")
        axes[row,0].legend(fontsize=8);axes[row,1].legend(fontsize=8)
    fig.suptitle("Attitude and body angular rates")
    fig.savefig(folder/"attitude_rates.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(11,7),layout="constrained",sharex=True)
    for ax,(field,label) in zip(axes,(("delta_a_rad","Aileron"),("delta_e_rad","Elevator"),("delta_r_rad","Rudder"))):
        for mode,c in cases.items():ax.plot(c["t_s"],np.rad2deg(c[field]),color=colors[mode],label=labels[mode])
        ax.set(ylabel=f"{label} (deg)");ax.legend(fontsize=8)
    axes[-1].set_xlabel("Time (s)");fig.suptitle("Identical open-loop control inputs in both cases")
    fig.savefig(folder/"controls.png",dpi=160);plt.close(fig)
    print("Created: trajectory_3d.png, overview.png, attitude_rates.png, controls.png")

if __name__=="__main__":main()
