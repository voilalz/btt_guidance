# 固定民用航路、分阶段控制器与交通避碰

本情形从地面静止释放，使用原虚构固体助推和巡航动力，按固定航路点飞行；出发、航路、进近三阶段独立选择控制器。另一架飞行器使用解析交通轨迹，安全监督器根据预测风险临时提高本机高度参考，满足保持及释放条件后恢复航路。

参考实现为 C，积分完整 17 维助推六自由度状态。当前航路为前向/高度/速度参考，横向保持对称；原三自由度、原六自由度、原助推情形保留。详细方程及接口见 [MODEL_CIVIL_ROUTE_zh.md](../../docs/MODEL_CIVIL_ROUTE_zh.md)，实测检查见 [VALIDATION_CIVIL_ROUTE_zh.md](../../docs/VALIDATION_CIVIL_ROUTE_zh.md)。全部参数与安全门限均为教学输入。

## 编译与运行

在仓库根目录执行，需要 C99 编译器和 libm；仿真不依赖 Python、MATLAB 或真实气动文件。

```sh
mkdir -p build
gcc -std=c99 -O2 -Wall -Wextra -Werror -Isix_dof/c -Isix_dof/boost_cruise/c -Isix_dof/civil_route/c six_dof/civil_route/c/main.c six_dof/civil_route/c/controllers.c six_dof/civil_route/c/civil_route.c six_dof/boost_cruise/c/boost_cruise.c six_dof/c/six_dof.c -lm -o build/civil_route_demo
./build/civil_route_demo high six_dof/civil_route/results/high.csv
./build/civil_route_demo crossing six_dof/civil_route/results/crossing.csv
./build/civil_route_demo monitor six_dof/civil_route/results/monitor.csv
python six_dof/civil_route/plot_results.py
```

Windows/MinGW 将输出名改成 `build/civil_route_demo.exe`，使用 `.\build\civil_route_demo.exe` 运行。CMake 提供 `civil_route_core`、`civil_route_demo`、`check_civil_route_components` 目标，旧目标保留；本次未安装 CMake，实际使用以上 GCC 路径。

CLI 格式：

```text
civil_route_demo SCENE OUTPUT.csv [duration [dt [stride [departure [enroute [approach]]]]]]
```

默认 `120 0.005 10 launch direct slew`。CSV 输出目录须已存在；时长/步长为有限正数，时长≤3600 s、积分步数≤2000000，输出间隔为正整数。输出 CSV 和同名 `.csv.json` 摘要；数值/控制器/文件错误返回非零状态，失败时 CSV 可能不完整。

## 三阶段更换控制器

内置控制器 `launch`、`direct`、`slew` 均可配置到三个阶段：

| 控制器 | 编号 | 行为 |
|---|---:|---|
| launch | 0 | 固定 40 m、25 m/s 参考 |
| direct | 1 | 当前固定航路点的高度与速度参考 |
| slew | 2 | 从段进入参考按高度 1 m/s、速度 0.5 m/s² 平滑过渡 |

分别替换出发/航路/进近：

```sh
./build/civil_route_demo high my.csv 120 0.005 10 direct slew direct
```

C 库接口在 `c/civil_route.h`：

```c
typedef int (*CivilControllerFn)(const CivilFlightView *own,
                                const CivilSegment *segment,
                                const CivilControllerMemory *memory,
                                CivilReference *output);
```

回调只接收本机状态、固定航路点及阶段/航路段进入参考；生成 `height_m`、`speed_mps`，返回 0 成功或负数失败。输出范围高度 1–300 m、速度 15–40 m/s，非有限或超界参考被拒绝。回调应无副作用，同时间重复调用须得到相同参考。

加入自定义函数并按阶段传入，例如：

```c
static int my_reference(const CivilFlightView *own,
                        const CivilSegment *segment,
                        const CivilControllerMemory *memory,
                        CivilReference *output) {
    (void)own; (void)memory;
    output->height_m = segment->height_m;
    output->speed_mps = segment->speed_mps;
    return 0;
}
static const CivilController my_controller = {101, "my_reference", my_reference};

/* cfg is a CivilConfig initialized by civil_default(). */
cfg.controllers[CIVIL_ENROUTE] = &my_controller;
```

库调用直接传函数指针；CLI 新增名字时在 `controllers.c` 的注册表追加条目，并把新函数源文件加入编译。三个配置槽分别为 `CIVIL_DEPARTURE`、`CIVIL_ENROUTE`、`CIVIL_APPROACH`。内部高度/速度反馈与俯仰/升降舵计算沿用 BC 稳定器；此回调层负责航路参考生成。具有积分状态的控制器需另设状态提交/复位接口，当前回调为无副作用参考生成器。

## 情形与实际结果

| 情形 | 交通轨迹 | 120 s 实测最小三维间距 |
|---|---|---:|
| high | 初态 x=6000 m、高度 10000 m，vx=−50 m/s | 9960.013 m |
| crossing | x=1500 m、高度 40 m，z=−1000+20t m；安全监督开启 | 126.443 m |
| monitor | 同一交叉交通，仅记录风险 | 21.522 m |

三组均实际完成出发、航路、进近阶段，并通过最后固定航路点的前向坐标平面。`route_complete` 表示完成坐标平面通过检查，不包含着陆；高度/速度跟踪误差另行记录。高空交通恒高，本机保持现有低空教学航路。

`crossing` 中安全监督启动一次，临时发出 160 m 高度参考，交通离开后恢复航路；120 s 末高度 30.0021 m、速度 19.9636 m/s。最小间距由每个接受积分步统计，采样步长 0.005 s；CSV 常规输出间隔 0.05 s。

## MATLAB 入口

新 MATLAB 前端调用已经编译的 C 程序，使用相同参考实现；另提供读取保存 CSV 的绘图函数。默认输出到 `results_matlab/`。

```matlab
addpath('six_dof/civil_route/matlab');
% executablePath 使用实际已编译文件路径：
summary = run_civil_route_demo(fullfile(pwd,'build','civil_route_demo'));
% Windows 示例：fullfile(pwd,'build','civil_route_demo.exe')
% 更换三个阶段：
% summary = run_civil_route_demo(executablePath,'my_results','high',120,0.005,10,'direct','slew','direct');
% 不运行 C，直接绘制已交付结果：
plot_saved_civil_route('six_dof/civil_route/results');
```

本次无 MATLAB/Octave，未执行新前端或绘图。原各案例的独立 MATLAB 动力学源码保留；新情形的 C 已实际编译运行，交付 PNG 由 Python/Matplotlib 生成。

## 检查及文件

```sh
python six_dof/civil_route/tests/run_checks.py
python six_dof/civil_route/tests/verify_outputs.py
python six_dof/civil_route/tests/check_preservation.py
# 重生成三组 C CSV、摘要与验证记录：
python six_dof/civil_route/tests/verify_outputs.py --generate
python six_dof/civil_route/plot_results.py
```

组件/CLI 检查只需 Python 标准库及 GCC，输出验证需 NumPy，绘图需 Matplotlib。

| 文件 | 责任 |
|---|---|
| `c/civil_route.h` | 公共控制器、航路、交通、监督与观察接口 |
| `c/controllers.c` | 内置无副作用参考生成器和名字注册 |
| `c/civil_route.c` | 阶段/航路切换、解析交通、监督、配平参考 |
| `c/main.c` | 既有 BC 积分、流式 CSV、逐接受步统计与 JSON |
| `results/*.csv`, `*.csv.json` | 三组实际轨迹及逐步摘要 |
| `results/verification.json` | 字节复现、半步长、控制器替换实测 |
| `results/high_traffic.png` | 高空交通与低空航路 |
| `results/crossing_avoidance.png` | 低空交叉交通及安全监督对照 |
| `results/flight_states.png` | 动力、阶段、控制器、姿态和库存 |

CSV 共 59 个数值字段，角度为 rad，所有位置/速度为输出的前向 x、高度、右向 z 分量。阶段为 0 出发、1 航路、2 进近；事件位 1/2/4/8 沿用动力点火/固体燃尽/巡航耗尽/触地，16 阶段切换、32 航路点推进、64/128 安全模式开/关、256 航路完成。事件行保存处理后的参考/阶段，同时间行合并。

![交叉交通与避碰对照](results/crossing_avoidance.png)
