# Civil Route and Traffic Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现已确认的固定民用航路跟踪、三阶段可替换控制器和独立交通避碰仿真。

**Architecture:** 新 `six_dof/civil_route/` 复用既有 BC 六自由度动力积分；控制器读取本机/固定航路，安全监督单独覆盖高度参考。旧源码与结果保持。

**Tech Stack:** C99/libm、Python NumPy/Matplotlib、基础 MATLAB 绘图与 C 调用入口。

**Spec:** `docs/superpowers/specs/2026-10-11-civil-route-design.md`

## Global Constraints

- 内置 launch/direct/slew，出发/航路/进近独立配置回调；默认 launch/direct/slew。
- 本机参考高度 1–300 m、速度 15–40 m/s；固定航路点 x/h/V 为 800/40/25、1800/60/25、2200/40/25、2800/30/20。
- 动力与已有虚构助推模型相同；出发阶段 t≥8 s、巡航点火且助推燃尽后切换。
- 高空交通恒高 10000 m；交叉交通 x1500/h40/z−1000，vz20 m/s。
- 监督前视 30 s、水平触发 120 m、高度触发 100 m、提高到交通高度+120 m；至少保持 40 s，远离且水平间距>300 m 才释放。
- 默认 dt=0.005 s/120 s/stride10；输出事件行，接收步统计最小间距，避碰验收≥80 m。
- 原 70 文件除三个入口追加外字节不变；MATLAB/CMake 未安装，不能声明执行通过。

## Review Focus

1. 同一时间零用时动力事件、阶段/安全切换不能造成重复 CSV 行或消耗虚假控制器时间。
2. 自定义控制器非法参考必须拒绝，不得保留上一次参考掩盖失败。
3. 零/远离相对运动不能触发错误避碰；安全模式要有保持及退出条件。
4. 最小间距须用接受步统计，交通时间必须与输出时间一致，阶段切换不重置飞行状态。
5. 回调在三个阶段确实被调用且可替换，旧三自由度/六自由度/助推情形保持。

## Task 1: 控制接口、交通和安全监督

**Files:** Create `six_dof/civil_route/c/civil_route.h`, `controllers.c`, `civil_route.c`, `tests/check_components.c`, `tests/run_checks.py`。

**Interfaces:** `CivilController.evaluate(const CivilFlightView*,const CivilSegment*,const CivilControllerMemory*,CivilReference*)`；`civil_controller(name)`，`civil_default(config,scene)`，`civil_initial(config,state)`，`civil_update(t,state,config,observation)`；`civil_traffic(t,config,traffic)`，`civil_cpa(view,traffic,horizon,prediction)`。State 内嵌 BCState、BCParams、航路/阶段/安全状态。

- [x] 先写缺失模型及组件验收：解析交通/CPA、保持/释放、非法/自定义回调、阶段状态连续与平滑参考。运行 `python six_dof/civil_route/tests/run_checks.py`，Expected: 新模块未实现断言失败。
- [x] 实现纯回调、固定航路、交通解析、安全监督、参考配平与更新接口。
- [x] 同一检查 Expected: GCC -Werror、解析/接口/边界检查全部通过。本地提交。

## Task 2: CLI、完整运行、图和文档

**Files:** Create C `main.c`，`tests/check_cli.py`、`verify_outputs.py`、`check_preservation.py`、baseline JSON，`plot_results.py`、`matlab/run_civil_route_demo.m`、`plot_saved_civil_route.m`、`results/*`、`README_zh.md`、`docs/MODEL_CIVIL_ROUTE_zh.md`、`VALIDATION_CIVIL_ROUTE_zh.md`；Append root CMake/root README/six_dof README。

**Interfaces:** CLI `civil_route_demo SCENE OUTPUT.csv [duration [dt [stride [departure [enroute [approach]]]]]]`；59 数值列 CSV 与 `OUTPUT.csv.json` 接受步摘要；选择 high/crossing/monitor，默认控制器 launch/direct/slew。

- [x] 写 CLI 与结果验收，Expected: main 或结果缺失断言失败。
- [x] 实现流式 CSV、摘要、阶段事件、参数检查；实际运行三场景、控制器替换及半步长对照，Expected: 三阶段可调用、实际避碰间距≥80 m并改善、公共网格差异满足阈值。
- [x] 生成三幅科学图并目视检查，写模型/接口/运行和实测文档；提供 MATLAB 调用 C 与 CSV 绘图。
- [x] 运行新旧检查、CSV 字节复现、原保护文件校验，Expected: 全部通过。本地提交。

## Task 3: 最终审查与连接器交付

- [x] 一次新上下文整分支审查，重要问题复现 RED→GREEN 并重跑完整检查。
- [x] 完善实测验证和未运行记录，提交最终源码/文档。
- [ ] 按现有授权连接器增量提交 main，预期 SHA 更新并核对所有远端 blob。
- [ ] 同步完成记录，核对最终提交，交付运行链接与曲线。

沿用本会话直接实现和连接器交付方式，用户此次“可以”确认了民用航路与避碰范围。
