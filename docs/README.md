# FAIRINO FR5 + RealSense D435i 视觉伺服系统

纯Python实现的眼在手（Eye-in-Hand）视觉伺服系统，采用30Hz视觉外环 + 125Hz机器人内环的多速率闭环架构，实现稳定、快速、可调的Tag跟踪控制。

---

## 目录

- [系统架构](#系统架构)
- [安装步骤](#安装步骤)
- [配置说明](#配置说明)
- [运行方法](#运行方法)
- [调参指南](#调参指南)
- [方向自检](#方向自检)
- [手眼标定](#手眼标定)
- [常见问题](#常见问题)

---

## 系统架构

### 多速率闭环设计

```
外环（视觉线程 30Hz）
  ↓ 采集图像
  ↓ ArUco检测
  ↓ 计算误差（像素+距离）
  ↓ 生成twist_cam（带时间戳）
  ↓ 放入队列

内环（控制线程 125Hz @ 8ms）
  ↓ 读取最新twist（非阻塞）
  ↓ ZOH保持 + Stale衰减
  ↓ 坐标变换（相机系->工具系）
  ↓ 限幅 + 死区
  ↓ ServoCart(mode=2, delta, cmdT=0.008)
```

### 稳定性保证机制

1. **ZOH（零阶保持）**: 两帧之间保持上一条指令，避免空白周期
2. **Stale超时衰减**: 视觉数据超时后指数衰减到0，避免突然停止
3. **增量限幅**: 每8ms的位移/旋转增量严格限制
4. **误差死区**: 到位后小误差不输出指令，消除微抖
5. **坐标变换**: 通过手眼外参T_tool_cam正确转换速度方向

---

## 安装步骤

### 1. 系统要求

- Ubuntu 22.04 (或其他Linux发行版)
- Python 3.8+
- Intel RealSense D435i 相机
- FAIRINO FR5 机械臂（控制器可访问）

### 2. 安装依赖

#### (1) Intel RealSense SDK

```bash
# 添加 Intel RealSense 仓库
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"

# 安装
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# 验证
realsense-viewer
```

#### (2) Python 依赖

```bash
cd /home/lh/yuntai/d435i_fr5
pip install -r requirements.txt
```

#### (3) FAIRINO Python SDK

将FAIRINO官方SDK中的 `frrpc.py` 复制到项目目录或放入 `PYTHONPATH`：

```bash
# 假设SDK已下载到 ~/fairino_sdk/
cp ~/fairino_sdk/frrpc.py /home/lh/yuntai/d435i_fr5/
```

### 3. 打印ArUco Tag

使用在线工具生成并打印ArUco Tag：
- 访问：https://chev.me/arucogen/
- 字典选择：4x4 (50, 100, 250, 1000)
- ID选择：5（或其他）
- 尺寸：打印后测量实际边长（例如100mm = 0.10m）

**重要**: 必须精确测量打印后的Tag边长，并在 `config.yaml` 中设置 `aruco.tag_size`！

---

## 配置说明

主配置文件：`config.yaml`

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot.ip` | 192.168.1.200 | 机器人控制器IP |
| `robot.cmdT` | 0.008 | 伺服周期（秒），固定8ms |
| `aruco.dictionary` | DICT_4X4_50 | ArUco字典 |
| `aruco.tag_size` | 0.10 | Tag边长（米），**必须精确测量** |
| `aruco.target_id` | -1 | 目标ID（-1=自动，>=0=指定） |
| `target.z_des` | 0.40 | 期望距离（米） |
| `controller.k_xy` | 1.0 | 横向增益 |
| `controller.k_z` | 1.2 | 深度增益 |
| `controller.max_trans_mm_per_tick` | 0.2 | 平移限幅（mm/8ms） |
| `controller.max_rot_deg_per_tick` | 0.1 | 旋转限幅（deg/8ms） |
| `controller.deadband_px` | 2.0 | 像素死区 |
| `handeye.T_tool_cam` | 示例矩阵 | 手眼标定外参（**必须标定**） |

---

## 运行方法

### 1. 基本运行

```bash
cd /home/lh/yuntai/d435i_fr5
python3 main_vs.py
```

### 2. 命令行参数

```bash
# 指定配置文件
python3 main_vs.py --config my_config.yaml

# 覆盖参数
python3 main_vs.py --target-id 5 --z-des 0.35

# 使用不同字典
python3 main_vs.py --dictionary DICT_6X6_250
```

### 3. 运行时操作

- **退出**: 按 `Ctrl+C` 或在图像窗口按 `q`
- **观察**: 实时窗口显示检测结果、误差、距离
- **终端输出**: 每0.5秒打印状态（FPS、误差、距离）

---

## 调参指南

### 调参原则

1. **从保守开始**: 先用小增益、小限幅，确保安全
2. **逐轴调试**: 先Z轴，再X轴，再Y轴，最后yaw（如需要）
3. **观察响应**: 响应过慢→增大增益；抖动→减小增益/增大死区
4. **迭代优化**: 每次只调一个参数，观察效果

### 核心参数调整

#### 1. 控制增益

| 参数 | 调整效果 | 推荐范围 |
|------|----------|----------|
| `k_xy` | 横向响应速度 | 0.5 ~ 2.0 |
| `k_z` | 深度响应速度 | 0.8 ~ 2.5 |
| `k_yaw` | 旋转响应速度 | 0.3 ~ 1.0 |

**调整步骤**:
1. 初始设为保守值（k_xy=0.6, k_z=0.8）
2. 手动小幅移动Tag，观察机器人跟随速度
3. 太慢→增大10%；抖动→减小20%
4. 重复直到满意

#### 2. 限幅

| 参数 | 调整效果 | 推荐范围 |
|------|----------|----------|
| `max_trans_mm_per_tick` | 单步最大平移 | 0.1 ~ 0.5 mm |
| `max_rot_deg_per_tick` | 单步最大旋转 | 0.05 ~ 0.3 deg |

**调整步骤**:
- 限幅太小→响应慢、难收敛
- 限幅太大→可能超调、不平滑
- 推荐从0.2mm开始

#### 3. 死区

| 参数 | 调整效果 | 推荐范围 |
|------|----------|----------|
| `deadband_px` | 像素误差死区 | 1.0 ~ 5.0 px |
| `deadband_mm` | 距离误差死区 | 1.0 ~ 5.0 mm |

**调整步骤**:
- 死区太小→到位后微抖
- 死区太大→精度不足
- 观察到位后抖动，适当增大

#### 4. 过时处理

| 参数 | 调整效果 | 推荐范围 |
|------|----------|----------|
| `stale_timeout_ms` | 视觉超时判定 | 30 ~ 100 ms |
| `decay_tau_s` | 衰减时间常数 | 0.05 ~ 0.2 s |

**调整步骤**:
- 遮挡Tag观察：指令应平滑衰减到0
- 衰减太快→急停感；太慢→响应迟钝

---

## 方向自检

**目的**: 验证坐标系方向是否正确，避免反向运动。

### 自检步骤

#### 1. Z轴自检（深度）

```bash
# 运行系统
python3 main_vs.py --z-des 0.40

# 观察：
# - 机器人距离Tag > 0.40m 时，应该向前靠近（Z+）
# - 机器人距离Tag < 0.40m 时，应该向后退远（Z-）
```

**预期**: 机器人自动调整到距离Tag约0.40m并稳定。

**异常**:
- 反向运动：检查手眼标定外参 `T_tool_cam` 的旋转部分
- 不动：检查增益 `k_z` 是否过小

#### 2. X轴自检（左右）

```bash
# Tag位于图像中心，轻轻向左移动Tag
# 观察：机器人应该向左跟随

# Tag向右移动
# 观察：机器人应该向右跟随
```

**预期**: 机器人跟随Tag左右移动，保持Tag在图像中心。

**异常**:
- 反向：检查 `T_tool_cam` 的旋转矩阵X方向
- 抖动：减小 `k_xy` 或增大 `deadband_px`

#### 3. Y轴自检（上下）

```bash
# Tag向上移动
# 观察：机器人应该向上跟随

# Tag向下移动
# 观察：机器人应该向下跟随
```

**预期**: 机器人跟随Tag上下移动。

#### 4. Yaw自检（旋转，可选）

```bash
# 修改 config.yaml: target.enable_yaw = true
python3 main_vs.py

# 旋转Tag（绕Tag的Z轴）
# 观察：机器人应该跟随旋转，保持Tag摆正
```

### 方向修正

如果某轴方向反了：

1. **检查手眼标定**: 重新进行手眼标定（见下节）
2. **临时翻转**: 在 `T_tool_cam` 中对应旋转矩阵的列取负
3. **增益取负**: 在 `controller.py` 中对应增益取负（不推荐）

---

## 手眼标定

### 为什么需要手眼标定？

手眼标定求解工具坐标系到相机坐标系的变换 `T_tool_cam`，确保控制指令方向正确。

### 数据采集（手动）

#### 方法一：使用机器人示教

```bash
# 1. 在不同位置示教机器人，使Tag在相机视野内
# 2. 记录每个位置的：
#    - 机器人TCP位姿（通过示教器读取，或代码读取）
#    - Tag的rvec, tvec（运行检测程序）

# 3. 保存数据（使用handeye_io.py）
```

示例代码（保存单个样本）：

```python
from handeye_io import save_calibration_sample

tcp_pose = {'xyz': [300, 100, 500], 'rpy': [0, 0, 1.57]}  # 单位mm, deg
tag_pose = {'rvec': [0.1, 0.2, 0.3], 'tvec': [0, 0, 0.4]}  # 单位弧度, 米

save_calibration_sample(
    save_dir='calib_data',
    tcp_pose=tcp_pose,
    tag_pose=tag_pose,
    tag_id=5
)
```

#### 方法二：编写自动采集脚本

参考以下伪代码：

```python
# calib_collect.py
from fr5_driver import FR5Driver
from aruco_detector import ArucoDetector
from handeye_io import save_calibration_sample
import pyrealsense2 as rs
import time

# 初始化机器人、相机、检测器
robot = FR5Driver("192.168.1.200")
robot.connect()

# ... 初始化相机和检测器 ...

# 定义多个采样位姿（手动示教后记录）
sample_poses = [
    # [joint_pos, desc_pos]
    [[0, -90, 90, 0, 90, 0], [300, 0, 500, 180, 0, 0]],
    # ... 更多姿态
]

for i, (joint_pos, desc_pos) in enumerate(sample_poses):
    print(f"移动到位姿 {i+1}/{len(sample_poses)}")
    robot.move_j(joint_pos, desc_pos)
    time.sleep(2.0)  # 等待稳定
    
    # 读取TCP位姿
    tcp_pose = robot.get_tcp_pose()
    
    # 采集图像并检测
    # ... 获取detection结果 ...
    
    # 保存样本
    tag_pose = {'rvec': detection['rvec'], 'tvec': detection['tvec']}
    save_calibration_sample('calib_data', tcp_pose, tag_pose, tag_id=5)

robot.disconnect()
```

### 手眼求解

使用OpenCV `calibrateHandEye()` 求解（需15+组样本）：

```python
import numpy as np
import cv2
from handeye_io import load_calibration_samples, export_for_opencv_handeye

# 1. 导出为OpenCV格式
samples = load_calibration_samples('calib_data/samples_20260121.json')
export_for_opencv_handeye(samples, 'calib_data')

# 2. 加载并求解
data = np.load('calib_data/handeye_data_XXXXXX.npz')
R_gripper2base = data['R_gripper2base']
t_gripper2base = data['t_gripper2base']
R_target2cam = data['R_target2cam']
t_target2cam = data['t_target2cam']

# 3. 调用OpenCV求解（Eye-in-Hand）
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base=[R for R in R_gripper2base],
    t_gripper2base=[t.reshape(3,1) for t in t_gripper2base],
    R_target2cam=[R for R in R_target2cam],
    t_target2cam=[t.reshape(3,1) for t in t_target2cam],
    method=cv2.CALIB_HAND_EYE_TSAI
)

# 4. 转为 T_tool_cam（注意：cam2gripper = tool2cam 的逆）
from utils_math import build_transform_matrix, invert_transform
T_cam2tool = build_transform_matrix(R_cam2gripper, t_cam2gripper)
T_tool_cam = invert_transform(T_cam2tool)

print("求解的 T_tool_cam:")
print(T_tool_cam)

# 5. 更新到 config.yaml
```

---

## 常见问题

### 1. 无法连接机器人

**症状**: `[ERROR] 连接机器人异常`

**解决**:
- 检查网络：`ping 192.168.1.200`
- 检查SDK：确认 `frrpc.py` 在PYTHONPATH中
- 检查防火墙：关闭或添加规则

### 2. 检测不到Tag

**症状**: `No Tag Detected`

**解决**:
- 检查光照：避免过曝、阴影、反光
- 检查Tag质量：打印清晰、边缘锐利
- 检查距离：Tag不能太远（<1m）或太近（>0.1m）
- 检查字典：确认Tag ID与字典匹配

### 3. 机器人不动

**症状**: 检测到Tag但机器人无动作

**解决**:
- 检查伺服模式：确认 `ServoMoveStart()` 成功
- 检查增益：可能过小（k_xy, k_z）
- 检查限幅：可能过小（max_trans_mm_per_tick）
- 检查死区：误差在死区内

### 4. 机器人抖动

**症状**: 机器人到位后持续微动

**解决**:
- 增大死区：`deadband_px`, `deadband_mm`
- 减小增益：`k_xy`, `k_z`
- 启用低通滤波：`enable_lpf: true`, 调整 `lpf_alpha`

### 5. 方向反了

**症状**: 向左移动Tag，机器人向右

**解决**:
- 重新进行手眼标定
- 检查 `T_tool_cam` 的旋转矩阵
- 临时修正：对应列取负

### 6. 收敛慢

**症状**: 机器人移动很慢

**解决**:
- 增大增益：`k_xy`, `k_z`
- 增大限幅：`max_trans_mm_per_tick`
- 检查距离：z_des是否合理

### 7. 超调/振荡

**症状**: 机器人来回摆动

**解决**:
- 减小增益：`k_xy`, `k_z`
- 减小限幅：避免步长过大
- 增大死区：提前停止

---

## 性能优化建议

### 1. 提高视觉帧率

- 降低分辨率：640x480 → 320x240（精度下降）
- 使用GPU加速（需CUDA版OpenCV）
- 简化检测参数

### 2. 减少延迟

- 启用延迟预测：`enable_prediction: true`
- 减小 `stale_timeout_ms`

### 3. 提高鲁棒性

- 启用低通滤波：平滑噪声
- 增大 `decay_tau_s`：避免急停

---

## 安全注意事项

1. **急停**: 随时准备按下机器人急停按钮
2. **工作空间**: 清空机器人周围障碍物
3. **限幅**: 首次运行使用最保守限幅（0.1mm/tick）
4. **监控**: 全程观察机器人运动，异常立即停止
5. **手眼标定**: 标定错误会导致危险运动，标定后先小范围测试

---

## 文件说明

| 文件 | 功能 |
|------|------|
| `main_vs.py` | 主程序，启动双线程视觉伺服 |
| `fr5_driver.py` | FR5机器人驱动封装 |
| `aruco_detector.py` | ArUco检测与位姿估计 |
| `controller.py` | 视觉伺服控制器（误差→指令） |
| `handeye_io.py` | 手眼标定数据IO |
| `utils_math.py` | 数学工具（旋转、限幅等） |
| `config.yaml` | 系统配置文件 |

---

## 许可与引用

本项目仅供学习研究使用。使用时请遵守：
- FAIRINO SDK许可协议
- Intel RealSense SDK许可协议
- OpenCV许可协议

如有问题或建议，欢迎提交Issue。

---

**最后更新**: 2026-01-21
