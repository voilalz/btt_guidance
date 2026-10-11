# 持续可变动力飞行器六自由度教学模型

本模型在现有三自由度点质量示例之外增加刚体姿态动力学。它不需要真实机型气动数据库：用虚构的质量、惯量、气动力和力矩系数先验证建模、积分、事件处理及输出流程。默认对象是小型固定翼试验飞行器，推力可随油门连续变化。

原三自由度模型的迎角和倾侧角由时间函数直接给出。这里将姿态作为状态，迎角、侧滑角由机体速度计算，气动力矩和舵面输入决定角速度及姿态。六自由度是三个平移与三个转动自由度；四元数含一个单位范数约束，再加质量、推力状态，因此代码使用 15 个状态量。

## 1. 坐标和状态

内部地面系采用右手 N/E/D：N 为初始前方，E 为初始右方，D 为向下；它是局部平面坐标。机体系 b 的 x/y/z 分别为前/右/下。输出沿用原示例的坐标：

$$x=N,\qquad y=-D,\qquad z=E.$$

注意，输出坐标的轴排列不能直接用于右手刚体叉乘。叉乘和姿态矩阵始终在内部右手坐标中计算。

状态为

$$s=[N,E,D,u,v,w,a,b,c,d,p,q,r,m,T]^T.$$

其中 \(\mathbf v_b=[u,v,w]^T\)，\(\boldsymbol\omega_b=[p,q,r]^T\)，\(Q=[a,b,c,d]^T\) 为标量在前的 Hamilton 四元数。\(R_{nb}\) 把机体向量转换为地面向量：

$$R_{nb}=\begin{bmatrix}
a^2+b^2-c^2-d^2&2(bc-ad)&2(bd+ac)\\
2(bc+ad)&a^2-b^2+c^2-d^2&2(cd-ab)\\
2(bd-ac)&2(cd+ab)&a^2-b^2-c^2+d^2
\end{bmatrix}.$$

每个接受的积分步将四元数归一化。RK4 子阶段以归一化副本计算旋转矩阵，以原子阶段四元数计算四元数导数；范数过小或非有限则报错。

## 2. 平移、转动及姿态

$$\dot{\mathbf r}_n=R_{nb}\mathbf v_b,$$
$$\dot{\mathbf v}_b=\frac{\mathbf F_{a,b}+[T,0,0]^T}{m}
+R_{nb}^T[0,0,g]^T-\boldsymbol\omega_b\times\mathbf v_b,$$
$$J\dot{\boldsymbol\omega}_b+\boldsymbol\omega_b\times(J\boldsymbol\omega_b)=\mathbf M_{a,b}.$$

惯量 \(J=\operatorname{diag}(J_x,J_y,J_z)\)，因此

$$\dot p=\frac{M_x+(J_y-J_z)qr}{J_x},\quad
\dot q=\frac{M_y+(J_z-J_x)pr}{J_y},\quad
\dot r=\frac{M_z+(J_x-J_y)pq}{J_z}.$$

$$\dot Q=\frac12\begin{bmatrix}
0&-p&-q&-r\\p&0&r&-q\\q&-r&0&p\\r&q&-p&0
\end{bmatrix}Q.$$

输出欧拉角采用 ZYX 次序，仅用于显示；不以欧拉角积分。航迹倾角为 \(\operatorname{atan2}(-V_D,\sqrt{V_N^2+V_E^2})\)，沿用原示例的航迹方位角 \(\psi_v=\operatorname{atan2}(-V_E,V_N)\)，其正号与常规右转航向相反。

假设地球平坦、重力恒定、无风、重心固定、惯量固定。质量变化只进入平移方程及耗油状态，不考虑燃料引起的惯量/重心变化和流出角动量。推力穿过重心，不产生附加力矩。\(T\) 是发动机对飞行器的净推力，不能再叠加同一股排气的反作用项。这些是本教学模型的边界。

## 3. 虚构气动关系

$$V=\sqrt{u^2+v^2+w^2},\quad \alpha=\operatorname{atan2}(w,u),\quad
\beta=\operatorname{atan2}(v,\sqrt{u^2+w^2}),$$
$$\rho=\rho_0\exp[-\max(-D,0)/H],\qquad \bar q=\tfrac12\rho V^2.$$

在机体系表示风轴基向量：

$$\mathbf e_{xw}=[\cos\alpha\cos\beta,\sin\beta,\sin\alpha\cos\beta]^T,$$
$$\mathbf e_{zw}=[-\sin\alpha,0,\cos\alpha]^T,\qquad
\mathbf e_{yw}=[-\cos\alpha\sin\beta,\cos\beta,-\sin\alpha\sin\beta]^T.$$

$$\mathbf F_{a,b}=-D_a\mathbf e_{xw}+Y_a\mathbf e_{yw}-L\mathbf e_{zw}.$$

这里 \(D_a\) 为阻力，避免与向下位置 D 混淆。该变换使升力、侧向力与速度垂直，气动力功率为 \(-D_aV\)，可以用来验证符号。

默认系数（所有角度按 rad 输入）：

$$C_L=\operatorname{clip}(0.2+4.5\alpha,-0.6,1.2),\quad
C_D=0.035+0.06C_L^2,\quad C_Y=-0.5\beta+0.12\delta_r.$$

令 \(\hat p=pb/(2V_s),\hat q=qc/(2V_s),\hat r=rb/(2V_s)\)，\(V_s=\max(V,10^{-3})\)：

$$C_l=-0.08\beta-0.45\hat p+0.10\hat r+0.12\delta_a,$$
$$C_m=0.03-0.80\alpha-5\hat q-\delta_e,$$
$$C_n=0.08\beta-0.04\hat p-0.20\hat r-0.08\delta_r.$$

力为 \((L,D_a,Y_a)=\bar q S(C_L,C_D,C_Y)\)，力矩为 \(\bar q S(bC_l,cC_m,bC_n)\)。正副翼产生正滚转力矩；正升降舵产生负俯仰力矩；正方向舵产生负偏航力矩。\(V<10^{-8}\) 时气动力及气动力矩置零，观察量迎角、侧滑角置零，重力和按姿态确定的推力仍计算。升力限幅是虚构模型的一部分，不代表真实失速。

## 4. 连续可变动力与燃料

$$T_{cmd}=T_{max}\operatorname{clip}(\eta(t),0,1),\quad
\dot T=(T_{cmd}-T)/\tau_T,\quad \dot m=-(c_0+c_TT).$$

耗油取决于实际推力；\(c_0\) 表示发动机运行时的基础消耗。恒定模式采用初始配平油门 \(\eta_0\)；可变模式为

$$\eta(t)=\operatorname{clip}\left[\eta_0+0.15\sin(2\pi t/40)+0.05\sin(2\pi t/15),0,1\right].$$

燃料耗尽时二分定位 \(m=m_{dry}\) 的时刻，立即置 \(T=0\)，剩余子步关闭发动机且保持干质量。该关机模型不模拟残余推力。`consume_fuel=0` 只供定质量解析验证，允许发动机运行但不改变质量。

## 5. 默认参数及初始全配平

| 参数 | 数值 |
|---|---:|
| 初始/干质量 | 12 / 11.7 kg |
| 初始速度/高度 | 25 m/s / 200 m |
| 翼面积/翼展/参考弦长 | 0.8 m² / 2 m / 0.4 m |
| \(J_x,J_y,J_z\) | 1, 1.6, 2.2 kg·m² |
| 重力/海平面密度/密度尺度高度 | 9.80665 m/s² / 1.225 kg/m³ / 8500 m |
| 最大推力/响应时间常数 | 60 N / 1 s |
| \(c_0,c_T\) | \(5\times10^{-5}\) kg/s, \(1.5\times10^{-5}\) kg/(N·s) |
| 仿真时长/步长/输出间隔 | 120 s / 0.01 s / 每 5 个常规步 |

先求水平飞行、零角速度的初始配平，推力随机体俯仰有竖直分量：

$$L(\alpha_0)+D_a(\alpha_0)\tan\alpha_0=m_0g,\quad
T_0=D_a(\alpha_0)/\cos\alpha_0,$$
$$\delta_{e0}=-(C_{m0}+C_{m\alpha}\alpha_0)/C_{m\delta_e},\quad \eta_0=T_0/T_{max}.$$

在 ±15° 内二分求根；不存在根、推力越界或配平表达式奇异时明确失败。默认得到 \(\alpha_0=2.438596874881°\)，\(\delta_{e0}=-0.232004114512°\)，\(T_0=13.232733930207\) N，\(\eta_0=0.220545565503\)。初始机体速度为 \([V_0\cos\alpha_0,0,V_0\sin\alpha_0]^T\)，四元数为 \([\cos(\alpha_0/2),0,\sin(\alpha_0/2),0]^T\)，惯性速度为水平 25 m/s。

两种动力模式使用相同初始状态与舵面输入。除固定配平升降舵外，副翼在 20–40 s、方向舵在 45–65 s 各施加小扰动：

$$W(t;t_1,t_2)=\sin^2[\pi(t-t_1)/(t_2-t_1)]\quad(t_1<t<t_2),$$

区间外 W=0，\(\delta_a=0.2°W(t;20,40)\sin[2\pi(t-20)/8]\)，\(\delta_r=0.1°W(t;45,65)\sin[2\pi(t-45)/10]\)。程序转换为 rad。这是开环演示输入，不包含航迹跟踪控制器。初始配平后质量仍会随耗油改变，所以恒定指令案例也会逐渐偏离初始严格平飞。

## 6. 积分与输出

使用定步长 RK4，最后一步缩短至请求终止时刻；触地时停止。一个 RK4 子步内推进模式固定，燃料事件先分段处理，触地定位的每次探测也调用同样的燃料分段逻辑。这样同一步内可以正确处理先耗油后触地。事件搜索以子步末端跨越为条件，默认小步长适合本演示；任意大步长不保证检出同一步内穿越再返回的多次事件，也不能保证积分稳定。

CSV 包括位置、惯性与机体速度、四元数、角速度、欧拉角、气动角、航迹角、质量、实际/指令推力、油门、舵面、气动力、力矩和四元数范数误差。全部 CSV 角度单位是 rad；图中转换为 deg 并标明单位。极短正时长及最后不足一个输出间隔的状态也会输出。

C 与 MATLAB 独立实现同一模型；Python 只验证和绘图，不代替 C 积分。实际执行记录见 [VALIDATION_6DOF_zh.md](VALIDATION_6DOF_zh.md)，完整设计见 [规格文档](superpowers/specs/2026-10-11-six-dof-design.md)。

## 7. 一手参考

刚体机体系方程及四元数运动学可对照 MathWorks 的 [6DOF (Euler Angles)](https://www.mathworks.com/help/aeroblks/6dofeulerangles.html) 和 [6DOF (Quaternion)](https://www.mathworks.com/help/aeroblks/6dofquaternion.html)。本文明确指定机体到地面的矩阵方向，独立实现各方程；虚构系数和演示输入不来自真实机型数据库。
