"""读取C程序的CSV并绘制科学曲线。可选依赖：numpy、matplotlib。"""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results'
a=np.genfromtxt(OUT/'constant.csv',delimiter=',',names=True)
b=np.genfromtxt(OUT/'variable.csv',delimiter=',',names=True)
BLUE='#2369AE'; RED='#D85130'; GRID='#D8E0E7'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,
                     'axes.spines.top':False,'axes.spines.right':False,
                     'axes.labelcolor':'#283746','text.color':'#283746',
                     'xtick.color':'#526172','ytick.color':'#526172',
                     'axes.edgecolor':'#C3CDD7','figure.facecolor':'white',
                     'axes.titleweight':'bold','savefig.facecolor':'white'})

def pair(ax,x,y,xlabel,ylabel,title):
    ax.plot(a[x],a[y],color=BLUE,lw=1.8,label='Constant thrust')
    ax.plot(b[x],b[y],color=RED,lw=1.8,label='Continuous variable thrust')
    ax.set(xlabel=xlabel,ylabel=ylabel,title=title)
    ax.grid(color=GRID,alpha=.75,lw=.6)

fig,axs=plt.subplots(2,3,figsize=(14.2,8.0))
fig.subplots_adjust(left=.07,right=.975,bottom=.09,top=.815,hspace=.40,wspace=.32)
fig.suptitle('Continuous variable thrust: trajectory simulation',fontsize=19,fontweight='bold',y=.974)
fig.text(.5,.925,'Fictitious parameters  |  3-DOF point mass  |  Same initial state and angle schedules  |  dt = 0.02 s',
         ha='center',fontsize=10,color='#627084')
pair(axs[0,0],'x_m','y_m','Forward x [m]','Height y [m]','Vertical projection')
pair(axs[0,1],'x_m','z_m','Forward x [m]','Right z [m]','Horizontal projection')
axs[0,1].set_aspect('equal',adjustable='datalim')
pair(axs[0,2],'t_s','speed_mps','Time [s]','Speed [m/s]','Speed response')
pair(axs[1,0],'t_s','y_m','Time [s]','Height [m]','Height response')
ax=axs[1,1]
ax.plot(a['t_s'],a['thrust_N'],color=BLUE,lw=1.7,label='Constant actual')
ax.plot(b['t_s'],b['thrust_cmd_N'],color='#939DA8',ls='--',lw=1.2,label='Variable command')
ax.plot(b['t_s'],b['thrust_N'],color=RED,lw=1.8,label='Variable actual')
ax.set(xlabel='Time [s]',ylabel='Thrust [N]',title='Command and engine response')
ax.grid(color=GRID,alpha=.75,lw=.6); ax.legend(fontsize=8,loc='lower left',frameon=False)
ax=axs[1,2]
ax.plot(a['t_s'],1000*(12-a['mass_kg']),color=BLUE,lw=1.8)
ax.plot(b['t_s'],1000*(12-b['mass_kg']),color=RED,lw=1.8)
ax.set(xlabel='Time [s]',ylabel='Consumed fuel [g]',title='Mass depletion')
ax.grid(color=GRID,alpha=.75,lw=.6)
handles,labels=axs[0,0].get_legend_handles_labels()
fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.897),ncol=2,frameon=False,fontsize=10)
fig.text(.07,.024,'Thrust commands have equal mean values over 120 s. This is a workflow demonstration, not a performance prediction.',
         fontsize=9,color='#627084')
fig.savefig(OUT/'overview.png',dpi=170)
plt.close(fig)

fig=plt.figure(figsize=(10.5,7.4))
ax=fig.add_subplot(111,projection='3d')
for data,col,label in [(a,BLUE,'Constant thrust'),(b,RED,'Continuous variable thrust')]:
    ax.plot(data['x_m'],data['z_m'],data['y_m'],color=col,lw=2,label=label)
    ax.scatter(data['x_m'][-1],data['z_m'][-1],data['y_m'][-1],color=col,s=28)
ax.scatter([0],[0],[200],color='#243D4D',s=35,label='Initial state')
ax.set(xlabel='Forward x [m]',ylabel='Right z [m]',zlabel='Height y [m]')
ax.view_init(elev=24,azim=-58); ax.set_box_aspect((1.25,1,.62))
ax.legend(loc='upper left',frameon=False,fontsize=9)
ax.set_title('3D flight trajectories',fontsize=18,pad=22)
fig.text(.5,.04,'Plot coordinates: (x, z, y). Vertical scale enlarged for readability.',ha='center',fontsize=10,color='#627084')
fig.subplots_adjust(left=.03,right=.94,bottom=.10,top=.90)
fig.savefig(OUT/'trajectory_3d.png',dpi=170)
plt.close(fig)
print('Saved overview.png and trajectory_3d.png')
