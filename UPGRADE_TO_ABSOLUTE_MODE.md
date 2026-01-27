# 视觉伺服改为mode=0（基坐标系绝对运动）修改说明

## 需要修改的文件
1. src/main_vs.py - 主控制循环
2. 添加: src/visual_servo/controller_absolute.py - 已创建

## main_vs.py 修改步骤

### 1. 在文件开头添加导入
```python
from visual_servo.controller_absolute import AbsolutePoseController
from scipy.spatial.transform import Rotation as R
```

### 2. 在 VisualServoSystem.__init__() 中添加成员变量
```python
self.absolute_controller = None  # 绝对位姿控制器
self.use_absolute_mode = True    # 使用绝对位姿模式
```

### 3. 在 _init_controller() 方法中初始化绝对控制器
```python
def _init_controller(self):
    # ... 现有代码 ...
    
    # 初始化绝对位姿控制器
    if self.use_absolute_mode:
        self.absolute_controller = AbsolutePoseController(
            max_vel_mm_s=50.0,      # 最大速度50mm/s
            max_rot_deg_s=20.0,     # 最大旋转速度20deg/s
            acc_mm_s2=200.0,        # 加速度200mm/s²
            acc_rot_deg_s2=80.0     # 旋转加速度80deg/s²
        )
        print("[INFO] 绝对位姿控制器已初始化 (mode=0)")
```

### 4. 修改 control_loop() 中的控制逻辑
替换这段代码：
```python
# 原代码（mode=2增量）
if self.servo_enabled:
    desc_pos = self.controller.get_servo_command(dt=dt)
    if desc_pos is not None:
        success = self.robot.servo_cart(desc_pos)
```

改为：
```python
# 新代码（mode=0绝对）
if self.servo_enabled:
    if self.use_absolute_mode:
        # 获取当前TCP位姿
        tcp_pose_dict = self.robot.get_tcp_pose()
        if tcp_pose_dict is None:
            continue
        
        current_pose = np.hstack([tcp_pose_dict['xyz'], tcp_pose_dict['rpy']])
        
        # 获取期望twist（相机系）
        with self.vision_lock:
            det = self.latest_detection.copy()
        
        if det['detected']:
            twist_cam, ts = self.controller.compute_twist_from_detection(det)
            self.controller.update_twist(twist_cam, ts)
            
            if twist_cam is not None:
                # 计算相机到基座的旋转矩阵
                # 需要: R_base_cam = R_base_tool @ R_tool_cam
                R_base_tool = R.from_euler('xyz', np.deg2rad(tcp_pose_dict['rpy'])).as_matrix()
                R_tool_cam = self.controller.R_tool_cam
                R_cam_base = R_base_tool @ R_tool_cam
                
                # 计算目标绝对位姿
                target_pose = self.absolute_controller.compute_target_pose_from_twist(
                    current_pose, twist_cam, R_cam_base, dt
                )
                
                # 执行绝对位姿伺服
                success = self.robot.servo_cart_absolute(target_pose)
                
                # 调试输出
                if tick_count % 125 == 0:
                    print(f"[DEBUG] 目标绝对位姿(mm,deg): "
                          f"[{target_pose[0]:.1f}, {target_pose[1]:.1f}, {target_pose[2]:.1f}, "
                          f"{target_pose[3]:.2f}, {target_pose[4]:.2f}, {target_pose[5]:.2f}]")
    else:
        # 保留原增量模式（向后兼容）
        desc_pos = self.controller.get_servo_command(dt=dt)
        if desc_pos is not None:
            success = self.robot.servo_cart(desc_pos)
```

### 5. 在config.yaml中添加配置选项（可选）
```yaml
controller:
  # ... 现有配置 ...
  
  # 绝对位姿模式配置
  use_absolute_mode: true
  absolute_max_vel_mm_s: 50.0
  absolute_max_rot_deg_s: 20.0
  absolute_acc_mm_s2: 200.0
  absolute_acc_rot_deg_s2: 80.0
```

## 关键坐标系转换说明

1. **相机系 → 工具系**: 使用手眼标定矩阵 `R_tool_cam`
2. **工具系 → 基座系**: 使用当前TCP姿态 `R_base_tool`
3. **相机系 → 基座系**: `R_cam_base = R_base_tool @ R_tool_cam`

## 优点

1. ✅ **消除抖动**: 基坐标系绝对运动 + 速度平滑
2. ✅ **平滑加减速**: 限制加速度，避免突变
3. ✅ **避免错误码14**: pos_gain=0.5，更保守的执行
4. ✅ **实时反馈**: 保持视觉伺服的响应性

## 测试步骤

1. 先运行servrcart_test/test.py验证mode=0工作正常
2. 修改main_vs.py
3. 运行 `python3 src/main_vs.py`
4. 观察是否还有抖动和错误14

## 回退方案

如果新方案有问题，设置 `self.use_absolute_mode = False` 即可回到原增量模式
