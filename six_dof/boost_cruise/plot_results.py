"""Scientific figures from the actual C/MATLAB boost-to-cruise CSV."""
from pathlib import Path
import argparse
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

BLUE="#2369a8"
ORANGE="#d87924"
GREEN="#258770"

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("results",nargs="?",type=Path,default=Path(__file__).resolve().parent/"results")
    folder=parser.parse_args().results
    a=np.genfromtxt(folder/"boost_cruise.csv",delimiter=",",names=True)
    t=a["t_s"];ign=float(a["ignition_time_s"][-1])
    burn_rows=np.flatnonzero(a["event_mask"].astype(int)&2)
    burn=float(t[burn_rows[0]]) if len(burn_rows) else None
    plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False,
                         "axes.grid":True,"grid.alpha":.25,"lines.linewidth":1.6})
    def marks(ax):
        if ign>=0:ax.axvline(ign,color=GREEN,ls=":",lw=1.2)
        if burn is not None:ax.axvline(burn,color=ORANGE,ls="-.",lw=1.2)
        ax.set_xlabel("Time (s)")
    def trajectory(ax):
        ax.plot(a["x_m"],a["y_m"],color=BLUE,label="Trajectory")
        if ign>=0:
            j=np.flatnonzero(a["event_mask"].astype(int)&1)[0]
            ax.scatter(a["x_m"][j],a["y_m"][j],color=GREEN,s=38,zorder=4,label="Cruise ignition")
        if burn is not None:
            j=burn_rows[0];ax.scatter(a["x_m"][j],a["y_m"][j],color=ORANGE,s=38,zorder=4,label="Solid burnout")
        ax.axhline(40,color="gray",ls="--",lw=1,label="Cruise height target")
        ax.set(xlabel="Forward x (m)",ylabel="Height y (m)")
        ax.legend(fontsize=8)
    fig,ax=plt.subplots(figsize=(11,4.7),layout="constrained")
    trajectory(ax);ax.set_title("Static launch, solid boost and sustained cruise | six DOF, fictional parameters")
    fig.savefig(folder/"trajectory.png",dpi=170);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(12,10.5),layout="constrained")
    trajectory(axes[0,0]);axes[0,0].set_title("Vertical trajectory (symmetric case)")
    axes[0,1].plot(t,a["y_m"],color=BLUE,label="Height")
    axes[0,1].axhline(40,color="gray",ls="--",label="Gate / target = 40 m")
    axes[0,1].set(ylabel="Height (m)",title="Climb and settling")
    axes[1,0].plot(t,a["speed_mps"],color=BLUE,label="Speed")
    axes[1,0].axhline(25,color="gray",ls="--",label="Gate / target = 25 m/s")
    axes[1,0].set(ylabel="Speed (m/s)",title="Booster overspeed and cruise recovery")
    axes[1,1].plot(t,a["boost_thrust_N"],color=ORANGE,label="Solid actual")
    axes[1,1].plot(t,a["thrust_N"],color=GREEN,label="Cruise actual")
    axes[1,1].plot(t,a["thrust_cmd_N"],color=GREEN,ls="--",label="Cruise command")
    axes[1,1].set(ylabel="Thrust (N)",title="Propulsion handoff")
    axes[2,0].plot(t,a["boost_fuel_kg"],color=ORANGE,label="Solid propellant")
    axes[2,0].plot(t,a["cruise_fuel_kg"],color=GREEN,label="Cruise fuel")
    axes[2,0].set(ylabel="Remaining inventory (kg)",title="Separate consumables; booster casing retained")
    axes[2,1].step(t,a["stage"],where="post",color=BLUE,label="Mode")
    axes[2,1].set(yticks=range(5),yticklabels=["Boost","Overlap","Coast","Cruise","Empty"],
                  ylim=(-.2,4.2),ylabel="Stage",title="Latched height AND speed ignition")
    for ax in list(axes.flat)[1:]:marks(ax);ax.legend(fontsize=8)
    fig.suptitle(f"Boost-to-cruise | ignition {ign:.3f} s | burnout {burn:.3f} s" if burn is not None else "Boost-to-cruise")
    fig.savefig(folder/"overview.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,7.8),layout="constrained")
    axes[0,0].plot(t,a["y_m"],color=BLUE,label="Height")
    axes[0,0].axhline(40,color="gray",ls="--",label="Height gate")
    axes[0,0].set(ylabel="Height (m)",title="Ignition requires BOTH gates")
    axes[0,1].plot(t,a["speed_mps"],color=BLUE,label="Speed")
    axes[0,1].axhline(25,color="gray",ls="--",label="Speed gate")
    axes[0,1].set(ylabel="Speed (m/s)",title="Speed alone does not ignite cruise")
    axes[1,0].plot(t,a["boost_thrust_N"],color=ORANGE,label="Solid actual")
    axes[1,0].plot(t,a["total_thrust_N"],color=BLUE,ls="--",label="Total actual")
    axes[1,0].set(ylabel="Thrust (N)",title="Solid burns to its preset end")
    axes[1,1].plot(t,a["thrust_N"],color=GREEN,label="Cruise actual")
    axes[1,1].plot(t,a["thrust_cmd_N"],color=GREEN,ls="--",label="Cruise command")
    axes[1,1].set(ylabel="Thrust (N)",title="Continuous response from zero to 3 N idle")
    for ax in axes.flat:marks(ax);ax.set_xlim(0,min(12,t[-1]));ax.legend(fontsize=8)
    fig.suptitle("First 12 seconds | dotted: cruise ignition | dash-dot: solid burnout")
    fig.savefig(folder/"handoff.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,7.8),layout="constrained")
    axes[0,0].plot(t,np.rad2deg(a["pitch_rad"]),color=BLUE,label="Pitch actual")
    axes[0,0].plot(t,np.rad2deg(a["pitch_cmd_rad"]),color=BLUE,ls="--",label="Pitch command")
    axes[0,0].set(ylabel="Pitch (deg)",title="Smooth attitude command handoff")
    axes[0,1].plot(t,np.rad2deg(a["flight_path_rad"]),color=GREEN,label="Flight path actual")
    axes[0,1].plot(t,np.rad2deg(a["gamma_cmd_rad"]),color=GREEN,ls="--",label="Flight path command")
    axes[0,1].set(ylabel="Flight path (deg)")
    axes[1,0].plot(t,np.rad2deg(a["alpha_rad"]),color=BLUE,label="Alpha")
    axes[1,0].plot(t,np.rad2deg(a["delta_e_rad"]),color=ORANGE,label="Elevator")
    axes[1,0].set(ylabel="Angle (deg)",title="Fictional aerodynamic and control model")
    axes[1,1].plot(t,np.rad2deg(a["q_radps"]),color=BLUE,label="Body pitch rate")
    axes[1,1].set(ylabel="q (deg/s)")
    for ax in axes.flat:marks(ax);ax.legend(fontsize=8)
    fig.suptitle("Six-DOF attitude evolution; angles plotted in degrees")
    fig.savefig(folder/"attitude.png",dpi=160);plt.close(fig)
    print("Created: trajectory.png, overview.png, handoff.png, attitude.png")

if __name__=="__main__":main()
