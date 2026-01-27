#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
controller_absolute.py - 绝对位姿视觉伺服控制器扩展
添加mode=0（基坐标系绝对运动）支持，包含速度平滑
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


class AbsolutePoseController:
    """绝对位姿控制器 - 添加速度平滑和目标位姿计算"""
    
    def __init__(self, max_vel_mm_s=50.0, max_rot_deg_s=20.0, acc_mm_s2=200.0, acc_rot_deg_s2=80.0, 
                 tracking_threshold_mm=50.0, near_target_threshold_mm=30.0):
        """
        初始化绝对位姿控制器
        
        Args:
            max_vel_mm_s: 最大平移速度 (mm/s)
            max_rot_deg_s: 最大旋转速度 (deg/s)
            acc_mm_s2: 平移加速度 (mm/s²)
            acc_rot_deg_s2: 旋转加速度 (deg/s²)
            tracking_threshold_mm: 位置跟踪保护阈值（mm），只有超过此值才重置（防止发散）
            near_target_threshold_mm: 接近目标阈值（mm），小于此值时从实际位置计算（避免震荡）
        """
        self.max_vel_trans = max_vel_mm_s / 1000.0  # 转为m/s
        self.max_vel_rot = np.deg2rad(max_rot_deg_s)
        self.acc_trans = acc_mm_s2 / 1000.0  # 转为m/s²
        self.acc_rot = np.deg2rad(acc_rot_deg_s2)
        self.tracking_threshold = tracking_threshold_mm / 1000.0  # 转为m
        self.near_target_threshold = near_target_threshold_mm / 1000.0  # 转为m
        
        # 当前速度状态（用于加减速平滑）
        self.current_vel_trans = np.zeros(3)  # [vx, vy, vz] m/s
        self.current_vel_rot = np.zeros(3)    # [wx, wy, wz] rad/s
        
        # 上一次目标位姿（用于保持连续性）
        self.last_target_pos = None  # [x, y, z] m
        self.last_target_rpy = None  # [rx, ry, rz] rad
        
        print(f"[AbsoluteCtrl] 初始化: v_max={max_vel_mm_s}mm/s, w_max={max_rot_deg_s}deg/s")
        print(f"[AbsoluteCtrl] 加速度: a={acc_mm_s2}mm/s², α={acc_rot_deg_s2}deg/s²")
        print(f"[AbsoluteCtrl] 跟踪保护阈值: {tracking_threshold_mm}mm（防发散）")
        print(f"[AbsoluteCtrl] 近目标阈值: {near_target_threshold_mm}mm（防震荡）")
    
    def compute_target_pose_from_twist(self, current_pose, desired_twist_cam, R_cam_to_base, dt):
        """
        从twist计算目标绝对位姿（基坐标系）- 混合跟踪策略
        
        Args:
            current_pose: 当前TCP位姿 [x, y, z, rx, ry, rz] (mm, deg, 基坐标系)
            desired_twist_cam: 期望twist [vx, vy, vz, wx, wy, wz] (m/s, rad/s, 相机系)
            R_cam_to_base: 相机系到基座系的旋转矩阵 (3x3)
            dt: 时间步长 (s)
        Returns:
            target_pose: 目标位姿 [x, y, z, rx, ry, rz] (mm, deg, 基坐标系)
        """
        # 当前实际位姿
        current_pos = np.array(current_pose[0:3]) / 1000.0  # mm -> m
        current_rpy = np.deg2rad(np.array(current_pose[3:6]))  # deg -> rad
        
        # 选择起始位姿（混合策略：默认从目标位置累积，只在误差过大时重置）
        if self.last_target_pos is not None:
            # 计算上次目标与当前实际位置的误差
            tracking_error = np.linalg.norm(self.last_target_pos - current_pos)
            
            if tracking_error > self.tracking_threshold:
                # 误差过大：从实际位置重新计算（防止发散）
                start_pos = current_pos
                start_rpy = current_rpy
                print(f"[AbsoluteCtrl] ⚠️ 跟踪误差过大: {tracking_error*1000:.1f}mm > {self.tracking_threshold*1000:.0f}mm，重置到实际位置")
                # 重置速度状态，避免突变
                self.current_vel_trans *= 0.3
                self.current_vel_rot *= 0.3
            else:
                # 正常跟踪：始终从上次目标位置出发（像test.py一样保持连续性）
                start_pos = self.last_target_pos
                start_rpy = self.last_target_rpy
                # 只在误差较大时打印，避免刷屏
                # if tracking_error * 1000 > 20:
                #     print(f"[AbsoluteCtrl] 跟踪: err={tracking_error*1000:.1f}mm，从目标位置累积")
        else:
            # 首次调用：从实际位置出发
            start_pos = current_pos
            start_rpy = current_rpy
        
        # 提取期望速度（相机系）
        v_cam = np.array(desired_twist_cam[0:3])  # m/s
        w_cam = np.array(desired_twist_cam[3:6])  # rad/s
        
        # 转换到基座标系
        v_base = R_cam_to_base @ v_cam
        w_base = R_cam_to_base @ w_cam
        
        # 速度平滑（加减速限制）
        v_base_smooth = self._smooth_velocity(v_base, self.current_vel_trans, 
                                               self.acc_trans, self.max_vel_trans, dt)
        w_base_smooth = self._smooth_velocity(w_base, self.current_vel_rot,
                                               self.acc_rot, self.max_vel_rot, dt)
        
        # 更新当前速度状态
        self.current_vel_trans = v_base_smooth
        self.current_vel_rot = w_base_smooth
        
        # 计算目标位置（从起始位姿 + 平滑速度增量）
        target_pos = start_pos + v_base_smooth * dt
        
        # 计算目标姿态（从起始姿态 + 平滑角速度增量）
        R_start = R.from_euler('xyz', start_rpy).as_matrix()
        delta_R = R.from_rotvec(w_base_smooth * dt).as_matrix()
        R_target = delta_R @ R_start
        target_rpy = R.from_matrix(R_target).as_euler('xyz')
        
        # 保存本次目标位姿（供下次使用）
        self.last_target_pos = target_pos.copy()
        self.last_target_rpy = target_rpy.copy()
        
        # 组合目标位姿
        target_pose = np.hstack([target_pos * 1000.0, np.rad2deg(target_rpy)])  # 转回mm, deg
        
        return target_pose
    
    def _smooth_velocity(self, desired_vel, current_vel, acc, max_vel, dt):
        """
        速度平滑（加减速限制）- 优化版：保持方向不变
        
        Args:
            desired_vel: 期望速度向量
            current_vel: 当前速度向量
            acc: 加速度标量
            max_vel: 最大速度标量
            dt: 时间步长
        Returns:
            smooth_vel: 平滑后的速度向量
        """
        # 计算期望速度大小和方向
        desired_speed = np.linalg.norm(desired_vel)
        current_speed = np.linalg.norm(current_vel)
        
        # 如果期望速度为0，直接减速到0
        if desired_speed < 1e-6:
            # 减速
            new_speed = max(0.0, current_speed - acc * dt)
            if current_speed < 1e-6:
                return np.zeros(3)
            return current_vel * (new_speed / current_speed)
        
        desired_dir = desired_vel / desired_speed
        
        # 计算速度变化（标量）
        delta_speed = desired_speed - current_speed
        
        # 限制加速度（标量）
        max_delta_speed = acc * dt
        if abs(delta_speed) > max_delta_speed:
            delta_speed = np.sign(delta_speed) * max_delta_speed
        
        # 计算新速度大小
        new_speed = current_speed + delta_speed
        
        # 限制最大速度
        new_speed = min(new_speed, max_vel)
        new_speed = max(new_speed, 0.0)
        
        # 返回新速度向量（保持期望方向）
        return desired_dir * new_speed
    
    def reset(self):
        """重置速度状态和目标位姿记录"""
        self.current_vel_trans = np.zeros(3)
        self.current_vel_rot = np.zeros(3)
        self.last_target_pos = None
        self.last_target_rpy = None
        print("[AbsoluteCtrl] 速度状态和目标位姿已重置")


if __name__ == "__main__":
    print("=== controller_absolute.py 单元测试 ===")
    
    # 创建控制器
    ctrl = AbsolutePoseController(
        max_vel_mm_s=50.0,
        max_rot_deg_s=20.0,
        acc_mm_s2=200.0,
        acc_rot_deg_s2=80.0
    )
    
    # 模拟当前位姿
    current_pose = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]  # mm, deg
    
    # 模拟期望twist（相机系）
    desired_twist_cam = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.1])  # m/s, rad/s
    
    # 模拟旋转矩阵（简化为单位矩阵）
    R_cam_to_base = np.eye(3)
    
    # 计算目标位姿
    dt = 0.008
    for i in range(10):
        target_pose = ctrl.compute_target_pose_from_twist(
            current_pose, desired_twist_cam, R_cam_to_base, dt
        )
        print(f"Iter {i+1}: target_pose = {target_pose}")
        current_pose = target_pose  # 更新当前位姿
    
    print("测试完成！")
