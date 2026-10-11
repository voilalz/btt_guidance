# Six-DOF Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 保留既有三自由度内容，新增虚构参数的六自由度 C/MATLAB 仿真，并实际输出恒定与持续可变动力轨迹。

**Architecture:** 独立 `six_dof/` 子系统。机体速度、Hamilton 四元数、角速度、质量和推力组成 15 维状态；RK4 分段处理燃料耗尽及触地。C 为本环境可执行的验证实现，MATLAB 独立实现相同方程。

**Tech Stack:** C99 + libm，MATLAB 基础函数，Python/NumPy/Matplotlib 用于结果验证和绘图。

**Spec:** `docs/superpowers/specs/2026-10-11-six-dof-design.md`

## Global Constraints

- 原 `c/`、`matlab/`、`tests/`、`results/`、原模型文档及原绘图脚本保持字节一致；根 README/CMake 只增添新入口。
- 全部 SI 单位，CSV 角度为 rad。内部 N/E/D 与机体前/右/下均右手；输出沿用 x 前、y 上、z 右。
- 固定惯量、重心不动、零风、常重力，推力穿过重心。不引入制导或真实机型参数。
- 每个物理功能先写解析测试并观察失败，再实现并观察通过；保存实际结果，不能预报测试通过。
- 当前环境缺少 MATLAB/Octave/CMake：直接 GCC 编译执行 C；如环境仍缺少这些程序，明确说明未执行对应检查。
- 按用户已确认的仓库上传授权，通过 GitHub 连接器增量提交，移动 main 时使用 expected SHA，拒绝覆盖并发更新。

## Review Focus

1. NED 与旧输出坐标互换、四元数乘法顺序、重力投影及风轴气动力的符号。
2. 低速、无效四元数和非有限输入是否产生清晰失败；初始配平是否同时满足力及力矩平衡。
3. 同一步内燃料耗尽与触地、极短正时长、最后不足一个输出间隔的样本是否处理正确。
4. C/MATLAB 方程、默认参数和事件语义是否一致；MATLAB 未执行的局限是否准确披露。
5. 结果及摘要是否由最终源码重现；原三自由度文件是否保留，远端提交是否只修改授权路径。

---

## Task 1: 刚体方程、配平和核心解析检查

**Files:** `six_dof/c/six_dof.h`, `six_dof/c/six_dof.c`, `six_dof/tests/check_core.c`, `six_dof/tests/run_checks.py`。

**Interfaces:** `SD6State.a[15]` 的顺序与规格一致；`SD6Params` 保存全部参数和配平值；`SD6Observation` 保存惯性速度、气动力、力矩、姿态及输入。公开 `sd6_default`, `sd6_initial`, `sd6_rotation`, `sd6_observe`, `sd6_rhs`, `sd6_step`。除默认参数函数外均返回 0 成功、负数失败。`sd6_step` 是固定推进模式的单个 RK4 子步，不能用于跨越燃料事件；外部调用应使用 Task 2 的 `sd6_advance`。

- [x] 写真实解析测试：恒等/偏航/滚转矩阵、正交及行列式、气动力功率、初始全配平、自由落体、球形自由刚体、非球形能量与惯性角动量、推力阶跃。
- [x] 运行 `python six_dof/tests/run_checks.py core`。Expected: 新核心尚未存在的断言失败；接口声明后再检查各待实现物理断言的失败。
- [x] 实现规格中的四元数与刚体方程、风轴气动力、二分配平、推力和耗油、RK4，每个接受步归一化。
- [x] 同一命令及 `python tests/run_checks.py`。Expected: 全部解析检查和原物理检查通过，GCC `-Werror` 编译通过。
- [x] 本地提交此任务；把真实测试结果写入执行记录。

## Task 2: 事件推进、CLI 与时序检查

**Files:** `six_dof/c/six_dof.c`, `six_dof/c/six_dof.h`, `six_dof/c/main.c`, `six_dof/tests/check_events.c`, `six_dof/tests/check_cli.py`, `CMakeLists.txt`。

**Interfaces:** `sd6_advance(t,dt,state,params,mode,&used_dt)` 返回 0 正常、1 触地、负数失败；燃料事件内部完成，不终止轨迹。CLI `six_dof_demo constant|variable OUTPUT.csv [duration_s [dt_s [output_stride]]]`。CSV 字段遵照规格；初始及终止样本总是输出。

- [x] 写燃料耗尽解析解、触地解析时间、同一步两事件、事件步长收敛及 CLI 无效参数/极短时长/最后样本测试。
- [x] 运行 `python six_dof/tests/run_checks.py events`。Expected: 缺少事件处理或 CLI 导致对应断言失败。
- [x] 实现燃料事件二分定位、燃料事件前后分段积分、触地二分、状态检查及 CSV CLI；增加独立 CMake 目标。
- [x] 运行 core/events 和原检查。Expected: 各事件解析误差满足测试容差，极短时长输出两个样本，错误输入返回非零。
- [x] 本地提交任务并记真实验证结果。

## Task 3: MATLAB、结果曲线与模型文档

**Files:** `six_dof/matlab/run_six_dof_demo.m`, `six_dof/matlab/plot_saved_csv.m`, `six_dof/plot_results.py`, `six_dof/tests/verify_outputs.py`, `six_dof/results/*`, `six_dof/README_zh.md`, `docs/MODEL_6DOF_zh.md`, 根 README 两份。

**Interfaces:** 两种模式均输出相同 CSV 表头；MATLAB 使用同一 15 维状态和 RK4 事件分段。Python 验证读取 C CSV，绘图读取任一实现的相同格式数据。

- [x] 先写输出检查：共同网格步长减半比较、有限值、四元数、质量/推力边界、配平值、两种模式确有差异及元数据。
- [x] 运行 `python six_dof/tests/verify_outputs.py`。Expected: 结果尚未生成的断言失败。
- [x] 编写独立 MATLAB 实现及绘图入口、Python 绘图及摘要生成，补充完整模型与使用文档。
- [x] GCC 编译并运行默认 120 s、dt=0.01 s 两案例及 dt=0.005 s 验证案例，运行输出检查。Expected: 最大位置差 <0.05 m、速度差 <0.01 m/s、姿态差 <0.05°，单位四元数误差 ≤1e-12。
- [x] 实际生成 3D/高度、姿态/角速率、气动角/动力/质量/舵面曲线并目视检查；摘要注明 MATLAB 是否实际执行。
- [x] 本地提交任务并记结果。

## Task 4: 回归、最终审查和连接器提交

**Files:** `docs/VALIDATION_6DOF_zh.md`，其余只修审查发现的实质问题。

- [x] 运行全部新旧物理、时序、输出检查，核对受保护文件与基线的内容哈希。Expected: 全部通过且原文件不变。
- [x] 整分支新上下文审查，覆盖 Review Focus；实质发现先用测试复现失败，再修复并跑完整检查。
- [x] 写真实验证文档，最终检查及本地提交。Expected: 验证记录与最终结果一致。
- [ ] 用连接器读取远端最新 head/tree，只上传新/授权修改文件；创建提交并以 expected SHA 更新 main。
- [ ] 重新读取远端树并比较 blob SHA。Expected: 新增文件一致，原受保护文件一致，返回可访问提交链接。

执行方式：依用户“进入实现计划与代码阶段”的明确指令，在当前会话逐项执行；所有上传操作沿用此前已确认的连接器授权。
