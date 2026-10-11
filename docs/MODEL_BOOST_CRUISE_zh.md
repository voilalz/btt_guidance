# 固体助推与巡航接管六自由度数学模型

在 [原六自由度模型](MODEL_6DOF_zh.md) 上增加固体助推、两类推进剂库存及条件点火事件。原刚体方程和气动系数不改；C 直接复用原 RHS，MATLAB 独立实现。模型使用虚构民用试验飞行器参数，忽略风、地球曲率、重心与惯量变化、接地反力和助推壳体抛离。

## 坐标与状态

内部位置为右手 NED，机体系为前、右、下。令 $R(Q)$ 把机体向量变换到 NED，$Q=[a,b,c,d]^T$ 为单位四元数，$\mathbf v_b=[u,v,w]^T$，$\boldsymbol\omega=[p,q,r]^T$。CSV 映射为 $x=N,y=-D,z=E$，因此 $y$ 为高度。

连续状态共 17 个数：

$$s=[N,E,D,u,v,w,a,b,c,d,p,q,r,m,T_c,m_b,m_c]^T.$$

$T_c$ 是巡航实际净推力，$m_b,m_c$ 分别是剩余助推推进剂及巡航燃料；总质量始终满足

$$m=m_d+m_b+m_c.$$

$m_d$ 包括保留的干机体与壳体。离散标志记录助推是否燃烧、巡航是否已点火、巡航燃料是否耗尽、是否已离地，以及点火时刻 $t_i$。六自由度指三轴平移与三轴转动；17 维是加入姿态表示、质量及发动机状态后的积分向量维数。

## 运动与姿态

两种动力均沿机体前轴并穿过重心，总净推力 $T=T_b+T_c$，不额外产生力矩。令 $\mathbf F_a,\mathbf M_a$ 为原模型的气动力与力矩：

$$\dot{\mathbf r}_n=R\mathbf v_b,$$
$$\dot{\mathbf v}_b=\frac{\mathbf F_a+[T,0,0]^T}{m}+R^T[0,0,g]^T-\boldsymbol\omega\times\mathbf v_b,$$
$$\dot{\boldsymbol\omega}=J^{-1}\left(\mathbf M_a-\boldsymbol\omega\times(J\boldsymbol\omega)\right),$$
$$\dot Q=\frac12\begin{bmatrix}
0&-p&-q&-r\\p&0&r&-q\\q&-r&0&p\\r&q&-p&0
\end{bmatrix}Q.$$

$J=\operatorname{diag}(1,1.6,2.2)$ kg·m² 固定。每个接受步归一化四元数。净推力已包含发动机排气反作用，不再叠加 $-\dot m\mathbf v_b$。这是固定惯量近似的变质量平动模型，未建立排气角动量模型。

沿用虚构大气 $\rho=1.225\exp(-\max(h,0)/8500)$，$h=-D$，$\bar q=\rho V^2/2$。$V=\|\mathbf v_b\|$，$\alpha=\operatorname{atan2}(w,u)$，$\beta=\operatorname{atan2}(v,\sqrt{u^2+w^2})$。当 $V<10^{-8}$ m/s 时，气动力/力矩以及记录的迎角/侧滑角置零；推力与重力仍起作用。

主要系数为 $C_L=\operatorname{clip}(0.2+4.5\alpha,-0.6,1.2)$，$C_D=0.035+0.06C_L^2$，$C_Y=-0.5\beta$，翼面积 0.8 m²。滚转、俯仰、偏航阻尼及完整力轴变换见原模型文档。启动过渡中可能出现较大迎角；本简化系数并非经试验验证的失速模型。

## 两类动力及库存

固体助推只采用预设的平顶净推力：

$$T_b=\begin{cases}260\ \mathrm N,&\text{助推燃烧标志有效},\\0,&\text{已自然燃尽},\end{cases}\qquad
\dot m_b=\begin{cases}-0.5/4\ \mathrm{kg/s},&\text{燃烧中},\\0,&\text{其余} .\end{cases}$$

自然燃尽事件为 $t_b=4$ s；事件后 $m_b=0,T_b=0$。巡航油门不提前关闭固体助推。默认 $t=0$ 就给出助推净推力，不模拟固体点火建立时间。

点火判据同时检查高度与速度：

$$h\ge40\ \mathrm m\quad\land\quad V\ge25\ \mathrm{m/s}.$$

第一次满足即锁存 $t_i$，点火时 $T_c(t_i)=0$，后续不因门限重新变假而关闭发动机。点火后的巡航响应及消耗为

$$\dot T_c=(T_{c,\mathrm{cmd}}-T_c)/\tau_c,\qquad\tau_c=1\ \mathrm s,$$
$$\dot m_c=-(c_0+c_TT_c),\qquad c_0=5\times10^{-5}\ \mathrm{kg/s},\quad c_T=1.5\times10^{-5}\ \mathrm{kg/(N\,s)}.$$

点火前巡航推力/指令/流量均为零；巡航燃料耗尽后将库存与推力置零，并停止巡航消耗。质量导数为两库存导数之和。默认两种初始库存为 0.5/0.3 kg，干质量 11.7 kg，初始总质量 12.5 kg。

| 阶段编号 | 条件 | 动力 |
|---:|---|---|
| 0 | 助推有效、巡航关闭 | 固体助推 |
| 1 | 助推有效、巡航开启 | 两种动力重叠 |
| 2 | 助推燃尽、巡航尚未点火 | 无动力滑行，继续检测门限 |
| 3 | 助推燃尽、巡航开启 | 持续巡航 |
| 4 | 助推燃尽、已点火巡航燃料耗尽 | 无动力滑行 |

阶段编号用于记录动力模式；是否达到高度/速度门限另有字段。门限未达到就保持相应模式，不能假设必然成功接管。

## 起飞释放与巡航稳定器

默认从 $h=0,V=0$、俯仰 $35^\circ$ 自由释放；零滚转/偏航/角速度。初始向上加速度必须为正：

$$\frac{T_b\sin35^\circ}{12.5}-g>0.$$

若该条件不成立，初始化报告无法离地。模型没有轮胎、导轨和地面约束，只描述释放后的飞行；离地标志在高度首次为正时锁存，之后触地终止。零速时航迹角记录为零，仅是无速度方向时的数值约定。

巡航目标 $h_r=40$ m、$V_r=25$ m/s。参考迎角 $\alpha_r$ 由原配平函数在助推燃尽质量、目标高度/速度下求取；这只计算控制参考，不覆盖当前的位置、速度、四元数等状态。用基础比例控制演示可持续巡航：

$$\gamma_{\mathrm{cmd}}=\operatorname{clip}(0.02(h_r-h),-8^\circ,8^\circ),$$
$$\lambda=\operatorname{clip}((t-t_i)/3,0,1),\qquad B=3\lambda^2-2\lambda^3,$$
$$\vartheta_{\mathrm{cmd}}=(1-B)35^\circ+B(\alpha_r+\gamma_{\mathrm{cmd}}),$$
$$\delta_e=\operatorname{clip}\left(-\frac{C_{m0}+C_{m\alpha}\alpha}{C_{m\delta_e}}
+0.8(\vartheta-\vartheta_{\mathrm{cmd}})+0.25q,-20^\circ,20^\circ\right),$$
$$T_{c,\mathrm{cmd}}=\operatorname{clip}\left(\frac{D_a+mg\sin\gamma_{\mathrm{cmd}}}{\max(\cos\alpha,0.8)}+5(V_r-V),3,60\right)\ \mathrm N.$$

$D_a$ 为阻力大小，避免与 NED 的 $D$ 坐标混淆。点火前保持发射俯仰指令。3 N 是虚构的点火后最低怠速指令，保证助推造成超速时巡航推力仍连续建立；巡航实际推力不在点火瞬间跳变。副翼/方向舵为零，因此默认轨迹保持对称平面、横向位移为零，但仍积分全部六自由度。所有控制计算角度为 rad，这些增益是教学输入。

## 积分及事件顺序

默认 RK4 步长 0.005 s。一个 RK4 子步内动力开关固定；助推燃尽先把区间截到 4 s，巡航点火、巡航燃料耗尽及触地以二分法定位。推进接口返回最早事件及实际用时，外层继续积分该常规步剩余时间，可处理同一窗口内多次事件。触地优先终止。事件会修改对应模式/库存，点火与助推燃尽保持全部运动状态连续。

常规网格每 0.05 s 输出，额外保留准确事件行；同一时刻合并。CSV 保存事件处理后的状态及位掩码，所有角度为 rad。事件定位依赖步内穿越被小步长解析捕捉；不要把任意长步长的门限扫描视为可靠算法。解析事件检查及步长减半对照见验证文档。

默认点火时速度已高于巡航目标，随后固体继续燃烧，故出现明显超调：最高高度约 96.32 m、最大速度约 57.13 m/s。控制器逐渐回到 40 m、25 m/s，这些数值不代表真实起飞或巡航性能。两类库存有限，“持续巡航”表示接管后的受控持续飞行阶段，不表示无限续航。

## 参考资料

NASA [Solid Rocket Engine](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/solid-rocket-engine/) 与 [Rocket Thrust Equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/rocket-thrust-equation/) 用于一般动力概念。这里的推力曲线、库存、气动系数和控制参数均为虚构设定，没有使用真实发动机或飞行器数据。
