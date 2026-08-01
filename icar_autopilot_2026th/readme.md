# ICAR 智能汽车自动驾驶系统（2026th）

> 基于 OpenCV + AI 视觉推理 + 有限状态机的智能汽车自动驾驶解决方案。
> 支持语音/自然语言指令输入，自动规划多圈行驶任务。

---

## 目录

- [项目简介](#项目简介)
- [快速开始](#快速开始)
- [项目结构](#项目结构)
- [核心模块说明](#核心模块说明)
- [配置文件说明](#配置文件说明)
- [常用工具](#常用工具)
- [常见问题](#常见问题)

---

## 项目简介

本项目是一个面向教育/竞赛场景的智能汽车自动驾驶系统，核心功能包括：

- **实时车道线识别**：基于二值化图像的边缘搜索与贝塞尔曲线拟合
- **AI 目标检测**：集成 YOLO 系列模型，识别道闸、锥桶、行人、限速标志等
- **多场景状态机**：停车场、施工区、岔路口、Y 型路口、斑马线、停靠站等
- **语音智能交互**：通过大语言模型（LLM）解析自然语言指令，自动映射为多圈任务配置
- **视觉场景识别**：通过视觉大模型识别当前场景标签（限速/施工区/停车场等）

---

## 快速开始

### 1. 环境准备

#### 系统要求

- **操作系统**：Linux（推荐 Ubuntu 18.04/20.04/22.04，ARM 或 x86）
- **编译器**：GCC 7+，支持 C++17
- **CMake**：3.4 以上

#### 依赖库

| 依赖 | 用途 |
|------|------|
| OpenCV 4.x | 图像采集、预处理、显示 |
| PPNC | AI 推理运行时（百度 Paddle 端侧推理库） |
| ONNX Runtime | ONNX 模型推理 |
| libserial | UART 串口通信 |
| glib-2.0 | 系统底层支持 |
| Python 3.6+ | 语音/视觉模块运行环境 |

安装系统依赖示例：

```bash
sudo apt update
sudo apt install -y cmake build-essential pkg-config \
    libopencv-dev libserial-dev libglib2.0-dev
```

Python 依赖：

```bash
pip install requests pillow opencv-python-headless
```

### 2. 编译项目

```bash
cd icar_autopilot_2026th
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

编译完成后，会在 `build/` 目录下生成以下可执行文件：

| 可执行文件 | 说明 |
|-----------|------|
| `icar` | **主程序**：自动驾驶核心 |
| `boot` | 开机自启守护进程 |
| `camera` | 摄像头调试工具 |
| `imging` | 图像预处理测试 |
| `detection` | AI 目标检测测试 |
| `calibration` | 相机标定工具 |
| `collection` | 遥控手柄图像采集 |
| `img2video` | 图片合成视频工具 |

### 3. 运行主程序

#### 方式一：直接运行 C++ 主控

```bash
cd build
./icar
```

#### 方式二：通过 Python 启动器运行（推荐）

支持语音指令 + 摄像头场景识别 + 自动配置：

```bash
cd src
python3 start.py
```

运行流程：
1. 输入语音/文字指令（如"先去停车场 3 号位，再经过施工区出口，最后走岔路口左侧"）
2. 系统自动调用 LLM 解析为结构化任务
3. 摄像头拍照识别当前场景
4. 写入 `config.json` 配置
5. 启动 `icar` 主程序

### 4. 配置文件准备

首次运行前，确保 `res/config.json` 存在并按需修改参数。详见 [配置文件说明](#配置文件说明)。

---

## 项目结构

```
icar_autopilot_2026th/
├── CMakeLists.txt          # CMake 构建配置
├── README.md               # 本文件
├── build/                  # 编译输出目录（需自行创建）
├── include/                # C++ 头文件
│   ├── icar.hpp            # 顶层入口类
│   ├── ctrl/               # 控制模块头文件
│   │   ├── center.hpp      # 控制中心计算
│   │   ├── motion.hpp      # 运动控制（速度/舵机）
│   │   ├── predeal.hpp     # 图像预处理
│   │   ├── track.hpp       # 车道线检测
│   │   └── mapping.hpp     # 透视变换
│   ├── fsm/                # 有限状态机头文件
│   │   ├── park.hpp        # 停车场
│   │   ├── busy.hpp        # 施工区/避障
│   │   ├── stop.hpp        # 停车区（道闸）
│   │   ├── cross.hpp       # 斑马线
│   │   ├── slow.hpp        # 慢行区
│   │   ├── station.hpp     # 停靠站
│   │   ├── fork.hpp        # T 型岔路
│   │   ├── yfork.hpp       # Y 型岔路
│   │   ├── obstacle.hpp    # 全局障碍物
│   │   └── manualControl.hpp # 远程手动接管
│   ├── com/                # 通信模块
│   │   ├── uart.hpp        # UART 串口通信
│   │   ├── server.hpp      # TCP 服务端（Boot）
│   │   └── client.hpp      # TCP 客户端
│   └── utils/              # 工具类
│       ├── detection.hpp   # AI 推理封装
│       ├── params.hpp      # 全局参数/配置结构
│       ├── tools.hpp       # 通用工具函数
│       ├── show.hpp        # UI 显示窗口
│       └── json.hpp        # JSON 解析
├── src/
│   ├── icar.cpp            # C++ 主入口
│   ├── start.py            # Python 启动器（语音+视觉）
│   ├── ctrl/               # 控制模块实现
│   ├── fsm/                # 状态机实现
│   ├── tool/               # 工具程序实现
│   ├── speech/             # Python 语音/LLM 模块
│   │   ├── llm.py          # 大语言模型接口
│   │   ├── speech.py       # 指令解析与配置更新
│   │   └── libs/
│   └── visual/             # Python 视觉大模型模块
│       ├── visual.py       # 百度一见视觉识别
│       └── libs/
└── res/                    # 资源文件
    ├── config.json         # 运行时主配置
    ├── models/             # AI 模型文件
    │   └── yolov3_mobilenet_v1/
    ├── samples/            # 示例视频/图像
    └── calibration/        # 相机标定数据
```

---

## 核心模块说明

### 1. 车道线识别 (`ctrl/track`)

- 输入：二值化后的赛道图像
- 输出：左右边缘点集 `pointsEdgeLeft` / `pointsEdgeRight`
- 原理：从图像底部向上扫描每行的黑白跳变点，提取赛道边界

### 2. 控制中心计算 (`ctrl/center`)

- 基于左右边缘点拟合赛道中心线
- 使用三阶贝塞尔曲线平滑路径
- 输出车辆应行驶的 `center` 坐标与舵机 PWM 值

### 3. 有限状态机 (`fsm/*`)

每个场景对应一个独立的 FSM 类：

| FSM | 触发条件 | 行为 |
|-----|---------|------|
| `FsmStop` | 检测到道闸 | 停车等待，重合度计算 |
| `FsmSlow` | 检测到限速标志 | 减速行驶 |
| `FsmCross` | 检测到斑马线 | 起点/终点停车 |
| `FsmPark` | 检测到停车场 | 入库/出库/指定车位 |
| `FsmBusy` | 检测到施工区 | 避障、手动接管、停靠 |
| `FsmFork` | 检测到岔路箭头 | T 型岔路补线 |
| `FsmYfork` | 检测到 Y 型路口 | 按配置左转/右转 |
| `FsmStation`| 检测到停靠框 | 施工区内按框停车 |
| `FsmObstacle`| 锥桶/行人 | 动态路径重规划 |

状态切换由 `getMode()` 决定，主循环按优先级依次执行各 FSM。

### 4. AI 目标检测 (`utils/detection`)

- 后端：PPNC（Paddle 端侧推理）+ ONNX Runtime
- 模型：`yolov3_mobilenet_v1`
- 检测目标：道闸、锥桶、行人、限速标志、斑马线、停车场、施工区等

### 5. 语音指令解析 (`speech/llm.py`)

- 支持国内主流大模型（百度千帆、阿里千问、智谱 AI 等）
- 统一使用 OpenAI 兼容 API 格式
- 通过结构化 System Prompt 将自然语言转为 JSON 任务列表

示例指令映射：

| 用户输入 | 解析结果 |
|---------|---------|
| "先去施工区中间，再去停车场 3 号位" | `[{"type":"construction","stop":1}, {"type":"park","spot":3}]` |
| "走岔路口右侧，再穿过停车位" | `[{"type":"fork","direction":"right"}, {"type":"park","spot":0}]` |

### 6. 视觉场景识别 (`visual/visual.py`)

- 基于百度一见视觉大模型
- 支持标签：`cone`/`person`/`busy`/`limit`/`unlimit`/`park`
- 对 "限速/解除限速" 提供二阶段确认机制，降低误识别率

---

## 配置文件说明

主配置文件：`res/config.json`

### 通用配置参数

| 参数 | 类型 | 说明 | 示例 |
|------|------|------|------|
| `velLow` | float | 最低速度 (m/s) | 0.6 |
| `velHigh` | float | 最高速度 (m/s) | 0.8 |
| `velSlow` | float | 慢行区速度 | 0.35 |
| `velPark` | float | 停车场速度 | 0.5 |
| `debug` | bool | 调试模式（显示窗口） | false |
| `saveImg` | bool | 保存原始图像 | false |
| `binary` | int | 二值化阈值（-1 表示大津法） | -1 |
| `model` | string | AI 模型路径 | `"../res/models/..."` |
| `video` | string | 视频源（调试时可用） | `"../res/samples/sample.mp4"` |
| `alertTarget` | string | 视觉识别目标标签 | `"park"` |

### 圈数配置

```json
{
  "圈数配置": {
    "totalLaps": 3,
    "crossStop": 1
  }
}
```

- `totalLaps`：总行驶圈数
- `crossStop`：斑马线停车次数（达到后不再触发）

### 每圈功能使能配置

每圈独立配置，字段说明：

| 字段 | 类型 | 说明 |
|------|------|------|
| `park` | bool | 是否启用停车场 |
| `parkSpot` | int | 目标停车位（0=穿过，1~4=指定车位） |
| `busy` | bool | 是否启用施工区 |
| `busyStopEnable` | bool | 施工区内是否停靠 |
| `busyStopPoint` | int | 停靠框序号（1=中间，2=出口） |
| `yfork` | bool | 是否启用 Y 型岔路 |
| `yforkLeft` | bool | Y 型岔路方向（true=左，false=右） |
| `slow` | bool | 是否启用慢行区 |
| `cross` | bool | 是否启用斑马线停车 |
| `stop` | bool | 是否启用道闸停车 |
| `station` | bool | 是否启用停靠站 |
| `manualTakeover` | bool | 是否允许手动接管 |

---

## 常用工具

### 相机标定

使用棋盘格拍摄 10 张以上标定图片，运行：

```bash
./calibration
```

标定结果自动保存到 `res/calibration/valid/calibration.xml`。

### 遥控手柄采图

连接手柄后运行：

```bash
./collection
```

按键说明：
- `按键 2`：连续采图
- `按键 3`：单张采图
- `按键 0`：停止采图

### AI 检测测试

```bash
./detection
```

实时显示摄像头画面及 AI 检测结果，用于验证模型部署是否正常。

### 图像预处理测试

```bash
./imging
```

按 `空格键` 保存当前帧的矫正图和二值化图。

---

## 常见问题

**Q1：编译时报错找不到 `ppnc` 或 `onnx`？**

> 确保已安装对应的推理库，并在 `CMakeLists.txt` 中正确配置 `PKG_CONFIG_PATH`：
> ```bash
> export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
> ```

**Q2：运行 `icar` 时提示找不到摄像头？**

> 检查摄像头设备节点是否存在：
> ```bash
> ls /dev/video*
> ```
> 若使用视频文件调试，修改 `config.json` 中的 `video` 字段。

**Q3：LLM 解析失败？**

> 在 `src/speech/llm.py` 中配置有效的 `api_key`，支持百度千帆、阿里千问等 OpenAI 兼容接口。

**Q4：如何切换为实车模式？**

> 将 `config.json` 中的 `debug` 设为 `false`，程序将通过 UART 向底层控制板发送速度/舵机指令。

**Q5：如何添加新的场景状态机？**

> 1. 在 `include/fsm/` 下新建 `xxx.hpp`，继承 `FSMState`
> 2. 在 `src/fsm/` 下实现 `xxx.cpp`
> 3. 在 `include/utils/params.hpp` 的 `FsmMode` 枚举中添加新模式
> 4. 在 `src/tool/running.cpp` 的 `runFsm()` 中注册并调用

---

## 技术栈

- **C++17** / **CMake** / **OpenCV**
- **YOLO** / **PaddlePaddle (PPNC)** / **ONNX**
- **UART** / **TCP Socket**
- **Python 3** / **LLM API** / **Requests**

---

> 本代码为北京赛曙科技有限公司内部教学/竞赛用示例代码，开源仅供学习参考。
>
> 代码持续更新，欢迎关注相关开源渠道获取最新版本。
