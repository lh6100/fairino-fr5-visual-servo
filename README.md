# FAIRINO FR5 Visual Servo System

基于RealSense D435i相机的法奥FR5机械臂视觉伺服系统（Eye-in-hand配置）

## 项目结构

```
d435i_fr5/
├── src/                          # 源代码
│   ├── visual_servo/             # 核心模块
│   │   ├── aruco_detector.py     # ArUco标记检测
│   │   ├── controller.py         # 视觉伺服控制器
│   │   ├── fr5_driver.py         # FR5机器人驱动
│   │   ├── realtime_plotter.py   # 实时数据绘图
│   │   ├── utils_math.py         # 数学工具函数
│   │   └── handeye_io.py         # 手眼标定I/O
│   └── main_vs.py                # 主程序
│
├── config/                       # 配置文件
│   ├── config.yaml               # 当前配置
│   └── config.yaml.example       # 配置模板
│
├── data/                         # 数据文件
│   ├── handeye_result.txt        # 手眼标定结果
│   ├── handeye_samples.npz       # 标定样本数据
│   └── handeye_logs/             # 标定日志
│
├── scripts/                      # 工具脚本
│   ├── handeye_calibration_eye_in_hand.py  # 手眼标定脚本
│   └── prepare_github.sh         # GitHub发布准备
│
├── tests/                        # 测试文件
│   └── test_plotter.py           # 绘图测试
│
├── docs/                         # 文档
│   ├── README.md                 # 项目说明
│   ├── CALIBRATION_GUIDE.md      # 标定指南
│   ├── PROJECT_FILES.md          # 文件说明
│   └── ROLL_*.md                 # 调试文档
│
├── logs/                         # 日志目录（运行时生成）
├── lib/                          # C++库文件
├── include/                      # C++头文件
├── fairino/                      # Python绑定库
│
├── run_visual_servo.py           # 快速启动脚本
└── requirements.txt              # Python依赖

```

## 快速开始

### 1. 安装依赖
```bash
pip install -r requirements.txt
```

### 2. 配置系统
```bash
cp config/config.yaml.example config/config.yaml
# 编辑 config/config.yaml 设置机器人IP等参数
```

### 3. 运行视觉伺服
```bash
python3 run_visual_servo.py
```

或直接运行：
```bash
python3 src/main_vs.py --config config/config.yaml
```

### 4. 键盘控制
- `S` - 启动伺服
- `P` - 暂停伺服
- `R` - 重置控制器
- `Q` - 退出系统

## 手眼标定

```bash
python3 scripts/handeye_calibration_eye_in_hand.py
```

## 测试

```bash
# 测试实时绘图
python3 tests/test_plotter.py
```

## 特性

- ✅ 6自由度视觉伺服（IBVS）
- ✅ 自适应增益控制（远距离快速，近距离精确）
- ✅ 实时速度/误差曲线监测
- ✅ 手眼标定工具
- ✅ 多线程架构（30Hz视觉 + 125Hz控制）

## 依赖

- Python >= 3.8
- RealSense SDK 2.50+
- OpenCV 4.5+
- NumPy >= 1.23
- Matplotlib >= 3.5

## 许可

MIT License
