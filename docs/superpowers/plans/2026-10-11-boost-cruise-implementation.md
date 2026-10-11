# Boost-to-Cruise Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 在现有六自由度仿真中新增静止释放、固体助推、条件点火巡航的可运行情形和曲线。

**Architecture:** 独立 `six_dof/boost_cruise/` 复用原刚体 RHS，扩展两类燃料及离散阶段；外层 RK4 与最早事件推进。已有核心与两种旧案例保持。

**Tech Stack:** C99/libm、基础 MATLAB、Python/NumPy/Matplotlib。

**Spec:** `docs/superpowers/specs/2026-10-11-boost-cruise-design.md`

## Global Constraints

- 17 维连续状态，质量等于干质量与两个库存之和，原 T 状态是巡航推力。
- 门限高度及速度为 AND，首次满足锁存；固体发动机按预设 260 N/4 s 自然燃尽。
- 巡航点火后最低指令 3 N，一阶响应连续建立；点火当刻实际巡航推力保持零。
- 默认静止、地面、俯仰 35°，库存 0.5/0.3 kg，门限与巡航目标 40 m/25 m/s。
- 默认 dt=0.005 s、120 s、输出间隔 10；原 CSV 和核心不改。
- 每个最早事件结束子步，剩余时间再次推进；事件时刻输出，角度 rad。
- C 必须实际运行；MATLAB/Octave/CMake 当前不可用，不能声明运行通过。

## Review Focus

1. 零速地面释放的触地判定不能立即停止，也不能掩盖无法离地。
2. 点火必须同时满足高度和速度，并保持锁存；没有达到门限的案例报告真实状态。
3. RK4 子阶段不能提前关固体动力；点火/燃尽交接不重置运动状态。
4. 两类库存的消耗、总质量和燃尽事件一致，同一步多个事件不漏掉。
5. MATLAB 方程/时序与 C 一致，CSV 事件行与常规行无重复时间，旧案例保持。

## Task 1: 物理、库存及最早事件推进

**Files:** 新 `six_dof/boost_cruise/c/boost_cruise.h`, `boost_cruise.c`，`tests/check_model.c`, `tests/run_checks.py`。

**Interfaces:** `BCState.a[17]`，离散 boost_active/cruise_ignited/fuel_exhausted/airborne/ignition_time；`BCParams` 嵌入原 `SD6Params`。`bc_default`, `bc_initial`, `bc_observe`, `bc_rhs`, `bc_advance(t,dt,state,params,&used)`；推进返回事件位掩码或负数失败。控制和阶段观察量集中由 observe 计算。

- [x] 先写缺失实现断言及解析/门限/燃尽/触地测试；运行 `python six_dof/boost_cruise/tests/run_checks.py`，Expected: 缺失功能或物理断言失败。
- [x] 实现固定模式 RK4、控制观察量、燃料状态、最早事件定位；每个接受步归一化四元数及校准总质量。
- [x] 运行同一命令，Expected: GCC -Werror 与全部新解析/边界检查通过；运行旧 core 检查通过。
- [x] 本地提交并记录结果。

## Task 2: CLI、MATLAB、结果与文档

**Files:** 新 C `main.c`、MATLAB `run_boost_cruise_demo.m`、`plot_saved_boost_cruise.m`、Python `plot_results.py`、`tests/verify_outputs.py`、`results/*`、`README_zh.md`；根 CMake/README及六自由度 README 增添入口；新 `docs/MODEL_BOOST_CRUISE_zh.md`。

**Interfaces:** CLI `boost_cruise_demo OUTPUT.csv [duration_s [dt_s [output_stride]]]`，输出原 41 字段加阶段/库存/动力/控制字段；事件行覆盖同时间常规行。MATLAB 独立实现相同方程；Python 基于 C CSV 验证、绘图。

- [x] 先写输出存在、点火/燃尽事件、库存/范数、极短时长、公共网格半步长及实际巡航的验收；运行检查，Expected: 未生成结果断言失败。
- [x] 实现 CLI 和 MATLAB，运行默认与半步长 C 案例；Expected: 实际完成助推与巡航、输出有限，差异满足规格阈值。
- [x] 绘图并目视检查，写实测摘要、数学模型及使用说明。
- [x] 运行新旧全套检查并核对原文件；Expected: 全部通过、旧结果字节不变。本地提交。

## Task 3: 审查及连接器交付

- [x] 新上下文整分支审查；实质问题先复现失败，再修正并跑完整检查。
- [x] 记录实际验证及未运行项，提交最终源码/文档。
- [ ] 按此前仓库上传授权，通过连接器只提交新增/授权修改文件，以 expected SHA 更新 main。
- [ ] 核对远端所有 blob SHA，记录交付完成。

用户本次明确要求仿真实现；沿用当前会话逐项实现及仓库连接器交付方式。
