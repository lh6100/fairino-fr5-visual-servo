#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
controller.py - 视觉伺服控制器
实现误差计算、控制律、限幅、死区、过时处理
"""

import numpy as np
import time
from utils_math import (
    clamp, apply_deadband, exponential_decay, 
    extract_yaw_from_rvec, normalize_angle, lpf_update
)


class VisualServoController:
    """视觉伺服控制器（IBVS - Image-Based Visual Servoing）"""
    
    def __init__(self, config, T_tool_cam):
        """
        初始化控制器
        
        Args:
            config: 配置字典
            T_tool_cam: 4x4 手眼标定外参（工具系->相机系）
        """
        self.config = config
        ctrl_cfg = config['controller']
        target_cfg = config['target']
        camera_cfg = config['camera']
        
        # 控制增益（独立控制每个轴）
        self.k_x = ctrl_cfg.get('k_x', ctrl_cfg.get('k_xy', 1.0))  # 兼容旧配置
        self.k_y = ctrl_cfg.get('k_y', ctrl_cfg.get('k_xy', 1.0))  # 兼容旧配置
        self.k_z = ctrl_cfg.get('k_z', 1.0)
        self.k_yaw = ctrl_cfg.get('k_yaw', 0.5)
        
        # 目标
        self.z_des = target_cfg.get('z_des', 0.40)  # 米
        self.enable_yaw = target_cfg.get('enable_yaw', False)
        
        # 限幅（每 tick = 8ms）
        self.max_trans_mm_per_tick = ctrl_cfg.get('max_trans_mm_per_tick', 0.2)
        self.max_rot_deg_per_tick = ctrl_cfg.get('max_rot_deg_per_tick', 0.1)
        
        # 死区
        self.deadband_px = ctrl_cfg.get('deadband_px', 2.0)
        self.deadband_mm = ctrl_cfg.get('deadband_mm', 2.0)
        self.deadband_deg = ctrl_cfg.get('deadband_deg', 1.0)
        
        # 过时处理
        self.stale_timeout_ms = ctrl_cfg.get('stale_timeout_ms', 50)
        self.decay_tau_s = ctrl_cfg.get('decay_tau_s', 0.10)
        
        # 延迟预测（可选）
        self.enable_prediction = ctrl_cfg.get('enable_prediction', False)
        self.prediction_horizon_ms = ctrl_cfg.get('prediction_horizon_ms', 20)
        
        # 低通滤波（可选）
        self.enable_lpf = ctrl_cfg.get('enable_lpf', False)
        self.lpf_alpha = ctrl_cfg.get('lpf_alpha', 0.3)
        
        # 是否使用手眼变换（测试时可关闭）
        self.use_handeye_transform = ctrl_cfg.get('use_handeye_transform', True)
        
        # 手眼外参 (注意：calibrateHandEye返回T_cam_tool)
        self.T_tool_cam = T_tool_cam
        self.R_tool_cam = T_tool_cam[0:3, 0:3]
        # 速度变换需要用转置（从相机系到工具系）
        self.R_cam_to_tool = self.R_tool_cam.T
        
        # 相机内参（从外部设置）
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # 状态变量
        self.last_twist_cam = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        self.last_update_time = 0
        self.is_initialized = False
        
        # 低通滤波状态
        self.lpf_error = None  # [ex, ey, ez, yaw_err]
        
        print("[INFO] 视觉伺服控制器初始化完成")
        print(f"  增益: k_x={self.k_x}, k_y={self.k_y}, k_z={self.k_z}, k_yaw={self.k_yaw}")
        print(f"  目标: z_des={self.z_des}m, enable_yaw={self.enable_yaw}")
        print(f"  限幅: trans={self.max_trans_mm_per_tick}mm/tick, rot={self.max_rot_deg_per_tick}deg/tick")
    
    def set_camera_intrinsics(self, fx, fy, cx, cy):
        """设置相机内参"""
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        print(f"[INFO] 相机内参已设置: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
    
    def compute_twist_from_detection(self, detection_result):
        """
        根据视觉检测结果计算期望 twist（相机坐标系）
        
        Args:
            detection_result: aruco_detector 返回的检测结果
        Returns:
            twist_cam: 6D numpy array [vx, vy, vz, wx, wy, wz]（相机系，单位 m/s, rad/s）
            timestamp: 时间戳
        """
        if not detection_result['detected']:
            return None, detection_result['timestamp']
        
        if self.fx is None:
            print("[ERROR] 相机内参未设置！")
            return None, detection_result['timestamp']
        
        # 提取检测数据
        u, v = detection_result['center_px']
        tvec = detection_result['tvec']  # 相机系下的 Tag 位置（米）
        rvec = detection_result['rvec']
        
        if tvec is None:
            return None, detection_result['timestamp']
        
        z = tvec[2]  # Tag 距离（米）
        
        # 计算误差
        ex = u - self.cx  # 像素
        ey = v - self.cy  # 像素
        ez = z - self.z_des  # 米
        
        # yaw 误差（可选）
        yaw_err = 0.0
        if self.enable_yaw and rvec is not None:
            yaw = extract_yaw_from_rvec(rvec)
            yaw_des = 0.0  # 期望 yaw 角（可参数化）
            yaw_err = normalize_angle(yaw - yaw_des)
        
        # 低通滤波（可选）
        if self.enable_lpf:
            raw_error = np.array([ex, ey, ez, yaw_err])
            if self.lpf_error is None:
                self.lpf_error = raw_error
            else:
                self.lpf_error = lpf_update(self.lpf_error, raw_error, self.lpf_alpha)
            ex, ey, ez, yaw_err = self.lpf_error
        
        # 延迟预测（可选）
        if self.enable_prediction and self.is_initialized:
            # 简单线性预测：e_pred = e + e_dot * dt_pred
            dt_pred = self.prediction_horizon_ms / 1000.0
            # 估算 e_dot（有限差分）
            # 这里简化：假设误差变化率恒定
            # 实际可用卡尔曼滤波或移动平均
            pass  # 暂不实现，可后续添加
        
        # 应用死区
        ex = apply_deadband(ex, self.deadband_px)
        ey = apply_deadband(ey, self.deadband_px)
        ez = apply_deadband(ez, self.deadband_mm / 1000.0)  # 转为米
        yaw_err = apply_deadband(yaw_err, np.deg2rad(self.deadband_deg))
        
        # 控制律（相机坐标系）
        # 注意：像素误差 -> 速度的转换需要考虑深度
        # v = -k * (u - u_des) / f  （近似）
        vx_cam = -self.k_x * ex / self.fx
        vy_cam = -self.k_y * ey / self.fy
        vz_cam = -self.k_z * ez
        
        # 角速度（只控制 yaw，绕 Z 轴）
        wx_cam = 0.0
        wy_cam = 0.0
        wz_cam = -self.k_yaw * yaw_err if self.enable_yaw else 0.0
        
        twist_cam = np.array([vx_cam, vy_cam, vz_cam, wx_cam, wy_cam, wz_cam])
        
        return twist_cam, detection_result['timestamp']
    
    def update_twist(self, twist_cam, timestamp):
        """
        更新最新的 twist 指令（外环调用）
        
        Args:
            twist_cam: 6D twist（相机系）
            timestamp: 时间戳
        """
        if twist_cam is not None:
            self.last_twist_cam = twist_cam
            self.last_update_time = timestamp
            self.is_initialized = True
    
    def get_servo_command(self, dt=0.008):
        """
        获取当前时刻的伺服指令（内环调用，125Hz）
        
        Args:
            dt: 时间步长（秒），默认 0.008
        Returns:
            desc_pos: 6D numpy array [dx, dy, dz, drx, dry, drz]（工具系增量，单位 mm, deg）
                     如果返回 None，表示无指令
        """
        if not self.is_initialized:
            return None
        
        current_time = time.time()
        dt_since_update = current_time - self.last_update_time
        
        # 检查数据是否过时
        twist_tool = self.last_twist_cam.copy()
        
        if dt_since_update > (self.stale_timeout_ms / 1000.0):
            # 数据过时，进行指数衰减
            decay_dt = dt_since_update - (self.stale_timeout_ms / 1000.0)
            decay_factor = np.exp(-decay_dt / self.decay_tau_s)
            twist_tool *= decay_factor
        
        # 坐标变换：相机系 -> 工具系
        v_cam = twist_tool[0:3]
        w_cam = twist_tool[3:6]
        
        if self.use_handeye_transform:
            # 使用手眼标定变换（注意：速度变换用R的转置）
            # v_tool = R_cam_to_tool @ v_cam = R_tool_cam.T @ v_cam
            v_tool = self.R_cam_to_tool @ v_cam
            w_tool = self.R_cam_to_tool @ w_cam
        else:
            # 不变换（假设相机系=工具系，用于测试）
            v_tool = v_cam
            w_tool = w_cam
        
        # 调试信息（显示变换前后对比）
        if self.use_handeye_transform:
            pass  # 在下方统一输出
            # print(f"[DEBUG] v_cam={v_cam*1000:.3f}mm/s -> v_tool={v_tool*1000:.3f}mm/s")
        
        # 计算增量
        delta_pos = v_tool * dt  # 米
        delta_rot = w_tool * dt  # 弧度
        
        # 限幅（按范数）
        delta_pos_norm = np.linalg.norm(delta_pos)
        if delta_pos_norm > (self.max_trans_mm_per_tick / 1000.0):
            delta_pos *= (self.max_trans_mm_per_tick / 1000.0) / delta_pos_norm
        
        delta_rot_norm = np.linalg.norm(delta_rot)
        if delta_rot_norm > np.deg2rad(self.max_rot_deg_per_tick):
            delta_rot *= np.deg2rad(self.max_rot_deg_per_tick) / delta_rot_norm
        
        # 单位转换：米 -> mm，弧度 -> 度
        delta_pos_mm = delta_pos * 1000.0
        delta_rot_deg = np.rad2deg(delta_rot)
        
        desc_pos = np.hstack([delta_pos_mm, delta_rot_deg])
        
        return desc_pos
    
    def reset(self):
        """重置控制器状态"""
        self.last_twist_cam = np.zeros(6)
        self.is_initialized = False
        self.lpf_error = None
        print("[INFO] 控制器已重置")


if __name__ == "__main__":
    # 单元测试
    print("=== controller.py 单元测试 ===")
    
    # 模拟配置
    test_config = {
        'controller': {
            'k_xy': 1.0,
            'k_z': 1.0,
            'k_yaw': 0.5,
            'max_trans_mm_per_tick': 0.2,
            'max_rot_deg_per_tick': 0.1,
            'deadband_px': 2.0,
            'deadband_mm': 2.0,
            'deadband_deg': 1.0,
            'stale_timeout_ms': 50,
            'decay_tau_s': 0.10,
            'enable_prediction': False,
            'enable_lpf': False
        },
        'target': {
            'z_des': 0.40,
            'enable_yaw': False
        },
        'camera': {}
    }
    
    # 模拟手眼外参（单位阵）
    T_tool_cam = np.eye(4)
    
    # 创建控制器
    controller = VisualServoController(test_config, T_tool_cam)
    controller.set_camera_intrinsics(fx=600, fy=600, cx=320, cy=240)
    
    # 模拟检测结果
    test_detection = {
        'detected': True,
        'tag_id': 5,
        'center_px': (330, 250),  # 偏移 (10, 10) 像素
        'tvec': np.array([0, 0, 0.42]),  # 距离 0.42m（偏离目标 0.40m）
        'rvec': np.array([0, 0, 0]),
        'timestamp': time.time()
    }
    
    # 计算 twist
    twist_cam, ts = controller.compute_twist_from_detection(test_detection)
    print("计算的 twist_cam:", twist_cam)
    
    # 更新
    controller.update_twist(twist_cam, ts)
    
    # 获取伺服指令
    desc_pos = controller.get_servo_command(dt=0.008)
    print("伺服指令 desc_pos (mm, deg):", desc_pos)
    
    print("测试完成！")
