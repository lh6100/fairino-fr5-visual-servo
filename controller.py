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
    extract_yaw_from_rvec, extract_pitch_from_rvec, extract_roll_from_rvec,
    normalize_angle, lpf_update, lpf_update_angle
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
        self.k_pitch = ctrl_cfg.get('k_pitch', 0.5)
        self.k_roll = ctrl_cfg.get('k_roll', 0.5)
        
        # 自适应增益（远距离快速收敛，近距离平稳）
        self.enable_adaptive_gain = ctrl_cfg.get('enable_adaptive_gain', False)
        self.k_x_far = ctrl_cfg.get('k_x_far', self.k_x * 2.0)  # 默认为2倍
        self.k_y_far = ctrl_cfg.get('k_y_far', self.k_y * 2.0)
        self.adaptive_threshold_px = ctrl_cfg.get('adaptive_threshold_px', 30.0)
        
        # 目标
        self.z_des = target_cfg.get('z_des', 0.40)  # 米
        self.enable_yaw = target_cfg.get('enable_yaw', False)
        self.enable_pitch = target_cfg.get('enable_pitch', False)
        self.enable_roll = target_cfg.get('enable_roll', False)
        
        # 限幅（每 tick = 8ms）
        self.max_trans_mm_per_tick = ctrl_cfg.get('max_trans_mm_per_tick', 0.2)
        self.max_rot_deg_per_tick = ctrl_cfg.get('max_rot_deg_per_tick', 0.1)
        
        # 死区
        self.deadband_px = ctrl_cfg.get('deadband_px', 2.0)
        self.deadband_mm = ctrl_cfg.get('deadband_mm', 2.0)
        self.deadband_deg = ctrl_cfg.get('deadband_deg', 1.0)
        self.deadband_roll_deg = ctrl_cfg.get('deadband_roll_deg', self.deadband_deg)  # roll专用死区
        
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
        self.lpf_error = None  # [ex, ey, ez, yaw_err, pitch_err, roll_err]
        
        # Roll调试：记录上一次角度，检测突变
        self.last_roll_raw = None
        self.roll_jump_threshold = np.deg2rad(30)  # 突变阈值30度
        
        # Roll振荡检测
        self.roll_err_history = []  # 记录最近的roll误差
        self.roll_history_len = 10  # 记录长度
        
        # 日志级别控制
        self.debug_log = config.get('debug', {}).get('verbose_log', False)
        
        print("[INFO] 视觉伺服控制器初始化完成")
        print(f"  增益: k_x={self.k_x}, k_y={self.k_y}, k_z={self.k_z}, k_yaw={self.k_yaw}")
        print(f"       k_pitch={self.k_pitch}, k_roll={self.k_roll}")
        print(f"  目标: z_des={self.z_des}m, enable_yaw={self.enable_yaw}, enable_pitch={self.enable_pitch}, enable_roll={self.enable_roll}")
        print(f"  限幅: trans={self.max_trans_mm_per_tick}mm/tick, rot={self.max_rot_deg_per_tick}deg/tick")
        print(f"  死区: px={self.deadband_px}, mm={self.deadband_mm}, deg={self.deadband_deg}, roll_deg={self.deadband_roll_deg}")
        if self.enable_roll:
            print(f"\n  [ROLL配置] 启用Roll控制")
            print(f"    增益 k_roll = {self.k_roll}")
            print(f"    死区 = {self.deadband_roll_deg}°")
            print(f"    手眼变换矩阵 R_cam_to_tool:")
            print(f"      [{self.R_cam_to_tool[0,0]:+7.4f}, {self.R_cam_to_tool[0,1]:+7.4f}, {self.R_cam_to_tool[0,2]:+7.4f}]")
            print(f"      [{self.R_cam_to_tool[1,0]:+7.4f}, {self.R_cam_to_tool[1,1]:+7.4f}, {self.R_cam_to_tool[1,2]:+7.4f}]")
            print(f"      [{self.R_cam_to_tool[2,0]:+7.4f}, {self.R_cam_to_tool[2,1]:+7.4f}, {self.R_cam_to_tool[2,2]:+7.4f}]")
    
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
        
        # pitch 误差（可选）
        pitch_err = 0.0
        if self.enable_pitch and rvec is not None:
            pitch = extract_pitch_from_rvec(rvec)
            pitch_des = 0.0  # 期望 pitch 角
            pitch_err = normalize_angle(pitch - pitch_des)
        
        # roll 误差（可选）
        roll_err = 0.0
        roll_raw = 0.0
        if self.enable_roll and rvec is not None:
            roll_raw = extract_roll_from_rvec(rvec)
            roll_des = 0.0  # 默认期望 roll 角
            
            # ⭐ 关键修复：当roll非常接近±180°时，自动调整目标为最近的±180°
            # 这样可以避免从-178°到0°需要转178°的问题
            # 只有在真正接近边界(±175°)时才调整，避免误判
            if abs(roll_raw) > np.deg2rad(175):  # 当|roll| > 175°时
                # 将目标设为最近的±180°
                if roll_raw > 0:
                    roll_des = np.pi  # +180°
                else:
                    roll_des = -np.pi  # -180°
                print(f"[ROLL_DEBUG] 角度接近边界(>175°)，目标调整为: {np.rad2deg(roll_des):+.1f}°")
            
            # ★关键：使用normalize_angle确保计算最短角度差
            roll_err = normalize_angle(roll_raw - roll_des)
            
            # 检测角度突变
            if self.last_roll_raw is not None:
                roll_diff = normalize_angle(roll_raw - self.last_roll_raw)
                if abs(roll_diff) > self.roll_jump_threshold:
                    print(f"[ROLL_WARNING] 检测到角度突变！diff={np.rad2deg(roll_diff):+7.2f}° (上次={np.rad2deg(self.last_roll_raw):+7.2f}° -> 当前={np.rad2deg(roll_raw):+7.2f}°)")
            
            self.last_roll_raw = roll_raw
            if self.debug_log:
                print(f"[ROLL_DEBUG] 原始角度: roll={np.rad2deg(roll_raw):+7.2f}° | 目标: {np.rad2deg(roll_des):+7.2f}° | 误差(归一化后): roll_err={np.rad2deg(roll_err):+7.2f}°")
        
        # 低通滤波（可选）
        if self.enable_lpf:
            raw_error = np.array([ex, ey, ez, yaw_err, pitch_err, roll_err])
            if self.lpf_error is None:
                self.lpf_error = raw_error
            else:
                # 对位置误差使用普通滤波
                self.lpf_error[0] = lpf_update(self.lpf_error[0], raw_error[0], self.lpf_alpha)
                self.lpf_error[1] = lpf_update(self.lpf_error[1], raw_error[1], self.lpf_alpha)
                self.lpf_error[2] = lpf_update(self.lpf_error[2], raw_error[2], self.lpf_alpha)
                # ⭐ 关键修复：对角度误差使用专用的角度滤波器（处理±180°边界）
                self.lpf_error[3] = lpf_update_angle(self.lpf_error[3], raw_error[3], self.lpf_alpha)  # yaw
                self.lpf_error[4] = lpf_update_angle(self.lpf_error[4], raw_error[4], self.lpf_alpha)  # pitch
                self.lpf_error[5] = lpf_update_angle(self.lpf_error[5], raw_error[5], self.lpf_alpha)  # roll
            ex, ey, ez, yaw_err, pitch_err, roll_err = self.lpf_error
            
            if self.enable_roll and self.debug_log:
                print(f"[ROLL_DEBUG] 低通滤波后(角度专用滤波器): roll_err={np.rad2deg(roll_err):+7.2f}°")
        
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
        pitch_err = apply_deadband(pitch_err, np.deg2rad(self.deadband_deg))
        
        # Roll专用死区处理（带详细日志）
        roll_err_before_deadband = roll_err
        roll_err = apply_deadband(roll_err, np.deg2rad(self.deadband_roll_deg))  # 使用roll专用死区
        if self.enable_roll:
            # 记录roll误差历史
            self.roll_err_history.append(roll_err)
            if len(self.roll_err_history) > self.roll_history_len:
                self.roll_err_history.pop(0)
            
            # 检测振荡：如果最近几次误差符号频繁变化
            if len(self.roll_err_history) >= 5:
                sign_changes = 0
                for i in range(1, len(self.roll_err_history)):
                    if abs(self.roll_err_history[i]) > 1e-6 and abs(self.roll_err_history[i-1]) > 1e-6:
                        if np.sign(self.roll_err_history[i]) != np.sign(self.roll_err_history[i-1]):
                            sign_changes += 1
                if sign_changes >= 3:  # 如果有3次或更多符号变化
                    print(f"[ROLL_WARNING] 检测到振荡！最近{len(self.roll_err_history)}次误差符号变化{sign_changes}次")
            
            if self.debug_log:
                print(f"[ROLL_DEBUG] 死区处理: 前={np.rad2deg(roll_err_before_deadband):+7.2f}° -> 后={np.rad2deg(roll_err):+7.2f}° (死区={self.deadband_roll_deg:.1f}°)")
        
        # 控制律（相机坐标系）
        # 注意：像素误差 -> 速度的转换需要考虑深度
        # v = -k * (u - u_des) / f  （近似）
        
        # 自适应增益：根据误差大小动态调整
        k_x_adaptive = self.k_x
        k_y_adaptive = self.k_y
        if self.enable_adaptive_gain:
            # 计算欧氏距离误差
            err_magnitude = np.sqrt(ex**2 + ey**2)
            if err_magnitude > self.adaptive_threshold_px:
                # 远距离：使用大增益快速接近
                k_x_adaptive = self.k_x_far
                k_y_adaptive = self.k_y_far
                if self.debug_log:
                    print(f"[ADAPTIVE] 远距离模式: err={err_magnitude:.1f}px > {self.adaptive_threshold_px:.1f}px, 使用增益 k_x={k_x_adaptive:.1f}, k_y={k_y_adaptive:.1f}")
            else:
                # 近距离：使用小增益精确定位
                if self.debug_log:
                    print(f"[ADAPTIVE] 近距离模式: err={err_magnitude:.1f}px ≤ {self.adaptive_threshold_px:.1f}px, 使用增益 k_x={k_x_adaptive:.1f}, k_y={k_y_adaptive:.1f}")
        
        vx_cam = -k_x_adaptive * ex / self.fx
        vy_cam = -k_y_adaptive * ey / self.fy
        vz_cam = -self.k_z * ez
        
        # 角速度（roll绕X轴, pitch绕Y轴, yaw绕Z轴）
        # ★已验证正确方向★：
        #   Rx(roll):  wx_cam = +k_roll * roll_err   (正号，已确认)
        #   Ry(pitch): wy_cam = -k_pitch * pitch_err (负号，已确认)
        #   Rz(yaw):   wz_cam = +k_yaw * yaw_err     (正号，已确认)
        wx_cam = self.k_roll * roll_err if self.enable_roll else 0.0
        wy_cam = -self.k_pitch * pitch_err if self.enable_pitch else 0.0
        wz_cam = self.k_yaw * yaw_err if self.enable_yaw else 0.0
        
        # Roll控制律详细日志
        if self.enable_roll and self.debug_log:
            print(f"[ROLL_DEBUG] 控制律: wx_cam = k_roll({self.k_roll}) * roll_err({np.rad2deg(roll_err):+7.2f}°) = {wx_cam:+7.4f} rad/s = {np.rad2deg(wx_cam):+7.2f} deg/s")
        
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
            
            # Roll坐标变换详细日志
            if self.debug_log and np.any(np.abs(w_cam[0]) > 1e-6):  # wx_cam非零时才打印
                print(f"[ROLL_DEBUG] 坐标变换: w_cam=[{w_cam[0]:+7.4f}, {w_cam[1]:+7.4f}, {w_cam[2]:+7.4f}] rad/s")
                print(f"[ROLL_DEBUG]          -> w_tool=[{w_tool[0]:+7.4f}, {w_tool[1]:+7.4f}, {w_tool[2]:+7.4f}] rad/s")
                print(f"[ROLL_DEBUG]          -> w_tool_deg=[{np.rad2deg(w_tool[0]):+7.2f}, {np.rad2deg(w_tool[1]):+7.2f}, {np.rad2deg(w_tool[2]):+7.2f}] deg/s")
        else:
            # 不变换（假设相机系=工具系，用于测试）
            v_tool = v_cam
            w_tool = w_cam
        
        # 计算增量
        delta_pos = v_tool * dt  # 米
        delta_rot = w_tool * dt  # 弧度
        delta_rot_before_clamp = delta_rot.copy()
        
        # 限幅（按范数）
        delta_pos_norm = np.linalg.norm(delta_pos)
        if delta_pos_norm > (self.max_trans_mm_per_tick / 1000.0):
            delta_pos *= (self.max_trans_mm_per_tick / 1000.0) / delta_pos_norm
        
        delta_rot_norm = np.linalg.norm(delta_rot)
        was_clamped = False
        if delta_rot_norm > np.deg2rad(self.max_rot_deg_per_tick):
            delta_rot *= np.deg2rad(self.max_rot_deg_per_tick) / delta_rot_norm
            was_clamped = True
        
        # Roll增量和限幅日志
        if self.debug_log and np.abs(delta_rot_before_clamp[0]) > 1e-6:
            print(f"[ROLL_DEBUG] 增量计算: delta_rot_x = w_tool_x({w_tool[0]:+7.4f}) * dt({dt}) = {delta_rot_before_clamp[0]:+7.5f} rad = {np.rad2deg(delta_rot_before_clamp[0]):+7.3f}°")
            if was_clamped:
                print(f"[ROLL_DEBUG] 限幅: 前={np.rad2deg(delta_rot_before_clamp[0]):+7.3f}° -> 后={np.rad2deg(delta_rot[0]):+7.3f}° (范数限制={self.max_rot_deg_per_tick}°/tick)")
        
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
