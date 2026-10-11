"""Plot civilian fixed-route and independent traffic simulation results."""
from pathlib import Path
import argparse
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

BLUE="#2369a8";GREEN="#27846f";ORANGE="#d57526";RED="#b23e47"

def main():
    parser=argparse.ArgumentParser();parser.add_argument("results",nargs="?",type=Path,default=Path(__file__).resolve().parent/"results")
    folder=parser.parse_args().results
    cases={n:np.genfromtxt(folder/(n+".csv"),delimiter=",",names=True) for n in ("high","crossing","monitor")}
    report=json.loads((folder/"verification.json").read_text())["scenes"]
    plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False,
                         "axes.grid":True,"grid.alpha":.23,"lines.linewidth":1.7})
    a=cases["high"];t=a["t_s"]
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout="constrained")
    axes[0,0].plot(a["x_m"],a["height_m"],color=BLUE,label="Own fixed route")
    axes[0,0].scatter([800,1800,2200,2800],[40,60,40,30],color=ORANGE,s=32,label="Waypoint reference")
    axes[0,0].set(xlabel="Forward x (m)",ylabel="Own height (m)",title="Launch, waypoint flight and approach")
    axes[0,1].plot(t,a["traffic_height_m"]/1000,color=GREEN,label="Independent traffic")
    axes[0,1].set(xlabel="Time (s)",ylabel="Traffic height (km)",ylim=(9.5,10.5),title="Traffic remains at 10 km")
    axes[1,0].plot(t,a["x_m"],color=BLUE,label="Own aircraft")
    axes[1,0].plot(t,a["traffic_x_m"],color=GREEN,label="Traffic")
    axes[1,0].set(xlabel="Time (s)",ylabel="Forward x (m)",title="Independent motion along the forward axis")
    axes[1,1].plot(t,a["separation_m"]/1000,color=BLUE,label="Actual 3D separation")
    axes[1,1].set(xlabel="Time (s)",ylabel="Separation (km)",title="No safety override required")
    for ax in axes.flat:ax.legend(fontsize=8)
    fig.suptitle("Civil route with high-altitude traffic | fictional aircraft parameters")
    fig.savefig(folder/"high_traffic.png",dpi=160);plt.close(fig)
    a=cases["crossing"];b=cases["monitor"];t=a["t_s"]
    fig,axes=plt.subplots(2,2,figsize=(12,8.5),layout="constrained")
    axes[0,0].plot(a["x_m"],a["z_m"],color=BLUE,label="Own aircraft")
    axes[0,0].plot(a["traffic_x_m"],a["traffic_z_m"],color=GREEN,label="Crossing traffic")
    axes[0,0].set(xlabel="Forward x (m)",ylabel="Right z (m)",title="Horizontal paths")
    axes[0,1].plot(t,a["height_m"],color=BLUE,label="Safety enabled")
    axes[0,1].plot(b["t_s"],b["height_m"],color=ORANGE,label="Monitor only")
    axes[0,1].plot(t,a["traffic_height_m"],color=GREEN,ls="--",label="Traffic")
    axes[0,1].plot(t,a["command_height_m"],color=BLUE,ls=":",label="Applied reference")
    axes[0,1].set(xlabel="Time (s)",ylabel="Height (m)",title="Vertical separation followed by route return")
    for n,color,label in (("crossing",BLUE,"Safety enabled"),("monitor",ORANGE,"Monitor only")):
        c=cases[n];r=report[n]
        axes[1,0].plot(c["t_s"],c["separation_m"],color=color,label=f"{label}: min {r['minimum_separation_m']:.2f} m")
        axes[1,0].scatter(r["minimum_separation_time_s"],r["minimum_separation_m"],color=color,s=35,zorder=4)
    axes[1,0].axhline(80,color=RED,ls="--",label="Teaching check = 80 m")
    axes[1,0].set(xlabel="Time (s)",ylabel="Actual separation (m)",xlim=(20,80),ylim=(0,850),title="Minimum measured at every accepted step")
    axes[1,1].step(t,a["safety_active"],where="post",color=BLUE,label="Safety override")
    axes[1,1].set(xlabel="Time (s)",ylabel="Safety mode",ylim=(-.1,1.2),yticks=[0,1],yticklabels=["Off","On"],title="Hold and separating-clearance release")
    for ax in axes.flat:ax.legend(fontsize=8)
    fig.suptitle("Civil crossing traffic | same fixed route | 120 s C simulation")
    fig.savefig(folder/"crossing_avoidance.png",dpi=170);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(12,10),layout="constrained")
    axes[0,0].plot(t,a["speed_mps"],color=BLUE,label="Actual")
    axes[0,0].plot(t,a["command_speed_mps"],color=BLUE,ls="--",label="Reference")
    axes[0,0].set(ylabel="Speed (m/s)")
    axes[0,1].plot(t,a["boost_thrust_N"],color=ORANGE,label="Solid boost")
    axes[0,1].plot(t,a["cruise_thrust_N"],color=GREEN,label="Cruise actual")
    axes[0,1].plot(t,a["cruise_cmd_N"],color=GREEN,ls="--",label="Cruise requested")
    axes[0,1].set(ylabel="Thrust (N)")
    axes[1,0].step(t,a["phase"],where="post",color=BLUE,label="Flight phase")
    axes[1,0].set(ylabel="Mission phase",yticks=[0,1,2],yticklabels=["Departure","Enroute","Approach"])
    axes[1,1].step(t,a["controller_id"],where="post",color=BLUE,label="Selected controller")
    axes[1,1].set(ylabel="Controller ID",yticks=[0,1,2],yticklabels=["launch","direct","slew"])
    axes[2,0].plot(t,np.rad2deg(a["pitch_rad"]),color=BLUE,label="Pitch")
    axes[2,0].plot(t,np.rad2deg(a["pitch_cmd_rad"]),color=BLUE,ls="--",label="Pitch reference")
    axes[2,0].set(ylabel="Pitch (deg)")
    axes[2,1].plot(t,a["boost_fuel_kg"],color=ORANGE,label="Solid inventory")
    axes[2,1].plot(t,a["cruise_fuel_kg"],color=GREEN,label="Cruise inventory")
    axes[2,1].set(ylabel="Remaining inventory (kg)")
    for ax in axes.flat:ax.set_xlabel("Time (s)");ax.legend(fontsize=8)
    fig.suptitle("Full six-DOF state integration | replaceable civilian reference generators")
    fig.savefig(folder/"flight_states.png",dpi=160);plt.close(fig)
    print("Created: high_traffic.png, crossing_avoidance.png, flight_states.png")

if __name__=="__main__":main()
