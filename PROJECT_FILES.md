# 项目文件清单与说明

## 📦 GitHub 可上传文件（核心代码）

### Python 核心模块
```
main_vs.py                          # 主程序：双线程视觉伺服系统
fr5_driver.py                       # 机器人驱动：FAIRINO FR5 SDK封装
aruco_detector.py                   # ArUco检测：标签检测和位姿估计
controller.py                       # 控制器：IBVS视觉伺服控制律
handeye_calibration_eye_in_hand.py  # 手眼标定：眼在手标定工具
handeye_io.py                       # 手眼IO：标定数据加载保存
utils_math.py                       # 数学工具：旋转矩阵、限幅等
```

### 工具脚本
```
convert_tool_frame.py               # 坐标系转换：手眼矩阵→机器人工具坐标
test_servo_axes.py                  # 测试工具：单轴运动测试
verify_rgb_intrinsics.py            # 相机工具：相机内参验证
aruco_depth_viewer.py               # 调试工具：ArUco深度可视化
```

### 配置文件
```
config.yaml                         # 系统配置：所有参数集中管理
requirements.txt                    # Python依赖：pip安装清单
```

### 文档
```
README.md                           # 项目说明：快速入门和使用指南
CALIBRATION_GUIDE.md                # 标定指南：手眼标定详细步骤
TEST_X_AXIS.md                      # 测试文档：X轴测试指南
PROJECT_FILES.md                    # 本文件：项目文件清单
```

### 构建文件
```
CMakeLists.txt                      # CMake配置（如果有C++代码）
.gitignore                          # Git忽略规则（需要创建）
```

---

## 🚫 不应上传到 GitHub 的文件

### 运行时生成的数据
```
handeye_samples.npz                 # 手眼标定数据（个人标定结果）
handeye_result.txt                  # 手眼标定矩阵（个人配置）
tool_frame_params.txt               # 工具坐标参数（个人配置）
realsense_neican.txt                # 相机内参（特定硬件）
```

### 日志和输出
```
handeye_logs/                       # 标定日志文件夹
  ├── samples.jsonl                 # 标定样本数据
  └── images/                       # 标定图像
servo_logs/                         # 伺服日志文件夹
  └── servo_log.jsonl               # 运行日志
```

### Python 缓存和虚拟环境
```
__pycache__/                        # Python字节码缓存
.venv/                              # 虚拟环境
*.pyc                               # 编译的Python文件
```

### 第三方库和编译产物
```
fairino/                            # FAIRINO SDK（第三方）
lib/                                # 共享库文件
  ├── libfairino.so.2
  ├── libfairino.so.2.3.1
  └── test_fairino
include/                            # 头文件（第三方SDK）
  ├── robot_error.h
  ├── robot_types.h
  └── robot.h
```

### 临时或测试文件
```
test1.py                            # 临时测试脚本
aruco_2d_visual_servo_direction.md  # 临时笔记
```

---

## 📁 推荐的 GitHub 项目结构

```
fairino-fr5-visual-servo/
├── README.md                           # 项目介绍
├── requirements.txt                    # Python依赖
├── config.yaml.example                 # 配置模板（不含个人数据）
├── .gitignore                          # Git忽略规则
│
├── docs/                               # 文档目录
│   ├── CALIBRATION_GUIDE.md           # 标定指南
│   ├── INSTALLATION.md                # 安装说明
│   ├── USAGE.md                       # 使用教程
│   └── API.md                         # API文档
│
├── src/                                # 源代码目录
│   ├── main_vs.py                     # 主程序
│   ├── fr5_driver.py                  # 机器人驱动
│   ├── aruco_detector.py              # ArUco检测
│   ├── controller.py                  # 控制器
│   ├── handeye_io.py                  # 手眼IO
│   └── utils_math.py                  # 数学工具
│
├── scripts/                            # 工具脚本
│   ├── handeye_calibration.py         # 手眼标定
│   ├── convert_tool_frame.py          # 坐标转换
│   ├── test_servo_axes.py             # 轴测试
│   └── verify_camera.py               # 相机验证
│
├── config/                             # 配置目录
│   └── default_config.yaml            # 默认配置
│
├── examples/                           # 示例代码
│   └── simple_demo.py                 # 简单演示
│
├── tests/                              # 单元测试
│   ├── test_detector.py
│   └── test_controller.py
│
└── data/                               # 数据目录（.gitignore）
    ├── calibration/                   # 标定数据
    └── logs/                          # 运行日志
```

---

## 🔧 需要创建的文件

### 1. .gitignore
```gitignore
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
.venv/
venv/
ENV/

# 数据文件
*.npz
*.jsonl
data/
handeye_logs/
servo_logs/
*.txt
!requirements.txt
!README.txt

# 配置（个人）
config.yaml
realsense_neican.txt
handeye_result.txt
tool_frame_params.txt

# 第三方库
fairino/
lib/
include/

# IDE
.vscode/
.idea/
*.swp
*.swo
*~

# 临时文件
test*.py
!test_servo_axes.py
aruco_2d_visual_servo_direction.md
```

### 2. config.yaml.example
将当前config.yaml复制为模板，移除个人标定数据：
```yaml
# 示例配置（请根据实际情况修改）
robot:
  ip: "192.168.1.200"  # 修改为你的机器人IP
  cmdT: 0.008

camera:
  fps: 30
  width: 640
  height: 480

aruco:
  dictionary: "DICT_4X4_50"
  tag_size: 0.05  # 修改为你的Tag实际尺寸（米）
  target_id: 0

target:
  z_des: 0.40  # 期望距离（米）
  enable_yaw: false

controller:
  use_handeye_transform: true
  k_x: 1.0
  k_y: 1.0
  k_z: 1.2
  k_yaw: 0.5
  max_trans_mm_per_tick: 0.15
  max_rot_deg_per_tick: 0.1
  deadband_px: 2.0
  deadband_mm: 2.0
  deadband_deg: 1.0
  stale_timeout_ms: 50
  decay_tau_s: 0.10

handeye:
  # 使用 handeye_calibration_eye_in_hand.py 标定后填入
  T_tool_cam:
    R:
      - [1.0, 0.0, 0.0]
      - [0.0, 1.0, 0.0]
      - [0.0, 0.0, 1.0]
    t:
      - 0.0
      - 0.0
      - 0.0

debug:
  enable_viz: true
  print_freq_hz: 2.0
  save_log: true
  log_dir: "servo_logs"
```

### 3. LICENSE
选择合适的开源协议（如MIT、Apache 2.0、GPL等）

---

## 📋 迁移步骤

### 步骤1：清理当前目录
```bash
# 删除临时文件
rm -f test1.py
rm -f aruco_2d_visual_servo_direction.md

# 备份个人数据
mkdir -p ~/backup_d435i_fr5
cp handeye_samples.npz ~/backup_d435i_fr5/
cp handeye_result.txt ~/backup_d435i_fr5/
cp config.yaml ~/backup_d435i_fr5/
```

### 步骤2：创建.gitignore
```bash
cd /home/lh/yuntai/d435i_fr5
# 创建.gitignore文件（内容见上方）
```

### 步骤3：创建配置模板
```bash
cp config.yaml config.yaml.example
# 编辑config.yaml.example，移除个人标定数据
```

### 步骤4：初始化Git仓库
```bash
git init
git add -A
git commit -m "Initial commit: FAIRINO FR5 Visual Servo System"
```

### 步骤5：推送到GitHub
```bash
# 在GitHub创建仓库后
git remote add origin https://github.com/lh6100/fairino-fr5-visual-servo.git

git branch -M main
git push -u origin main
```

---

## 📊 文件统计

### 核心代码（应上传）
- Python模块：8个
- 工具脚本：4个
- 配置文件：2个
- 文档文件：4个
- **总计：18个文件**

### 排除文件（不应上传）
- 运行数据：4个文件
- 日志目录：2个文件夹
- Python缓存：1个文件夹
- 第三方库：3个文件夹
- 临时文件：2个文件
- **总计：12个文件/文件夹**

---

## 🎯 建议的README.md结构

```markdown
# FAIRINO FR5 + RealSense D435i Visual Servo

纯Python实现的机械臂视觉伺服系统（不依赖ROS2）

## 特性
- ✅ 双线程架构（30Hz视觉 + 125Hz控制）
- ✅ 基于图像的视觉伺服（IBVS）
- ✅ 手眼标定工具
- ✅ 实时轨迹可视化
- ✅ 安全的键盘控制

## 硬件要求
- FAIRINO FR5机械臂
- Intel RealSense D435i相机
- Ubuntu 22.04 / Python 3.8+

## 快速开始
[安装、标定、运行步骤]

## 文档
- [标定指南](docs/CALIBRATION_GUIDE.md)
- [使用教程](docs/USAGE.md)
- [API文档](docs/API.md)

## License
MIT
```

---

## ✅ 操作建议

1. **立即执行**：创建.gitignore文件
2. **清理临时文件**：删除test1.py等
3. **备份个人数据**：handeye_samples.npz等
4. **创建配置模板**：config.yaml.example
5. **优化README**：添加更多使用示例
6. **添加单元测试**：提高代码质量
7. **完善文档**：API说明和故障排除

---

**建议的GitHub仓库名称：**
- `fairino-fr5-visual-servo`
- `fr5-realsense-visual-servoing`
- `eye-in-hand-visual-servo`
