# 民用航路与交通避碰实测验证

本记录对应 [模型与接口](MODEL_CIVIL_ROUTE_zh.md) 和 [运行入口](../six_dof/civil_route/README_zh.md)。实际运行的是 GCC 编译的 C 六自由度参考实现，参数为虚构教学值；此次新增范围为固定航路跟踪与独立交通避碰。

## 运行条件与结果

默认三场景均运行 120 s，积分步长 0.005 s、常规 CSV 输出间隔 0.05 s，控制器依次为 `launch/direct/slew`。所有场景均经历出发、航路、进近，并通过最后航路点的前向坐标平面。固体助推保持原模型，巡航高度与速度 AND 门限的实际点火时刻均为 3.114495988 s。

| 实测量 | high | crossing | monitor |
|---|---:|---:|---:|
| 交通高度 / m | 10000 | 40 | 40 |
| 最小三维间距 / m | 9960.012910 | 126.443396 | 21.522425 |
| 最小间距时刻 / s | 76.705 | 51.555 | 50.310 |
| 安全模式启动次数 | 0 | 1 | 0 |
| 120 s 末前向位置 / m | 3078.349193 | 3242.139521 | 3078.349193 |
| 120 s 末高度 / m | 30.000049 | 30.002147 | 30.000049 |
| 120 s 末速度 / (m/s) | 20.000007 | 19.963612 | 20.000007 |
| 接受积分步数 | 24001 | 24001 | 24001 |
| CSV 行数（不计表头） | 2406 | 2408 | 2406 |

`crossing` 在 15.685 s 发出 160 m 安全高度参考，在 60.845 s 满足保持及远离条件后恢复固定航路。该场景实测最小间距超过本案例 80 m 验收值；相同交通轨迹的仅监测对照为 21.522425 m。该对照反映这组理想输入的结果，不能推断任意交通条件下的安全性能。`high` 中交通恒高 10 km，本机仍按低空教学航路飞行。

三组均在 8 s 从出发切换至航路；进入进近的时刻分别为 78.135、78.795、78.135 s，最终前向平面通过时刻分别为 106.085、100.610、106.085 s。`route_complete` 仅表示平面通过，不表示精确到达、着陆或所有状态误差为零。

## 数值和接口检查

在公共 0.05 s 输出网格上，将积分步长减半为 0.0025 s，与默认积分比较：

| 最大差异 | high | crossing | monitor | 验收上限 |
|---|---:|---:|---:|---:|
| 三维位置范数 / m | 0.006331 | 0.013643 | 0.006331 | 0.1 |
| 三维速度范数 / (m/s) | 0.000628 | 0.001763 | 0.000628 | 0.02 |

这是两种步长的一致性检查；本次没有为新场景运行独立高阶参考积分器。四元数归一化误差的逐接受步最大值为 `1.1102230246251565e-16`。最小间距由每个接受步统计，包含事件子步；仍属于有限时间步长的离散极值估计。

以下命令实际执行并通过：

```sh
python six_dof/civil_route/tests/run_checks.py
python six_dof/civil_route/tests/verify_outputs.py
python six_dof/civil_route/tests/check_preservation.py
python six_dof/boost_cruise/tests/run_checks.py
python six_dof/boost_cruise/tests/verify_outputs.py
python six_dof/tests/run_checks.py all
python six_dof/tests/verify_outputs.py
python six_dof/tests/check_regression.py
python -m py_compile six_dof/civil_route/plot_results.py six_dof/civil_route/tests/*.py
git diff --check
```

- 新 C 源码使用 `-std=c99 -O2 -Wall -Wextra -Werror` 编译。组件检查覆盖交通解析、最近接近预测、相对静止/远离边界、安全模式保持/释放（含保持期后相对静止仍须覆盖）、高空高度隔离、非法参考，以及三个阶段的自定义回调调用。
- CLI 检查覆盖非法场景/数值/控制器/路径、极短正时长、末时刻、递增时间和 JSON 摘要。控制器回调或参数失败不会保留上一条参考掩盖错误。
- 完整三场景验证检查 59 个字段有限、解析交通与时间一致、姿态/质量库存、阶段/事件、实际间距、控制器调用和最终航路平面。默认 CSV 与逐步摘要重新运行后字节一致。
- `direct/slew/direct` 的另一组三阶段配置实际运行 120 s，三个阶段均被调用且输出有限。
- 原有 70 个文件中，除三个入口文件追加内容外，其余 67 个文件的 Git blob 校验值与基线一致。原三自由度、六自由度和助推巡航检查通过，原六自由度 DOP853 参考检查仍通过。
- 三张 PNG 由交付 CSV 生成并目视检查，保存为 [高空交通](../six_dof/civil_route/results/high_traffic.png)、[交叉交通对照](../six_dof/civil_route/results/crossing_avoidance.png)、[飞行状态](../six_dof/civil_route/results/flight_states.png)。完整精度结果见 [verification.json](../six_dof/civil_route/results/verification.json)。

## 未运行范围

环境未安装 MATLAB、Octave 或 CMake，未执行新增 MATLAB 前端、MATLAB 绘图或 CMake 构建。MATLAB 前端调用已经编译的 C 程序；另一个函数读取保存的 CSV 绘图。原各案例的独立 MATLAB 源码保持，新场景没有第二套独立 MATLAB 动力学实现。

默认路线使用纵向高度/速度参考，横向对称；六自由度状态仍完整积分。交通为理想已知匀速真值；未引入风、传感器误差、通信或执行延迟。当前检查支持这组教学流程及程序接口，不构成实际飞行适航验证。

## 交付记录

完成一次新上下文的整分支审查，发现安全监督退出条件把相对静止归为远离。新增回归先复现失败，再把释放条件改为水平相对位置与速度点积严格为正；组件/CLI、完整三场景字节复现、半步长、控制器替换、新旧回归及 67 个保护文件校验重新通过。默认三场景结果与修复前字节一致。

已通过 GitHub 连接器提交到 `voilalz/btt_guidance` 的 `main`，代码/文档/结果提交为 [33835a7829ad92e18c53ff0a70db438bbd0d04b9](https://github.com/voilalz/btt_guidance/commit/33835a7829ad92e18c53ff0a70db438bbd0d04b9)。基于原提交 `5d96fff6bbf67906d3a546cf9864eb9d9edda70f` 增量写入 31 个新增/修改文件，更新引用使用预期 SHA 且未强制覆盖。提交后读取远端引用和完整树，核对全部 98 个文件的 Git blob 校验值及 67 个原保护文件；连接器另成功读取运行说明与提交页面。随后通过文档提交同步本记录及计划完成状态。
