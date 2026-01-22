#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
utils_math.py - 数学工具函数
提供旋转矩阵、欧拉角转换、限幅、归一化等工具
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def rotation_matrix_to_euler(R_mat, seq='xyz'):
    """
    旋转矩阵转欧拉角
    
    Args:
        R_mat: 3x3 旋转矩阵
        seq: 旋转顺序，默认 'xyz'
    Returns:
        (rx, ry, rz) 弧度
    """
    r = R.from_matrix(R_mat)
    return r.as_euler(seq, degrees=False)


def euler_to_rotation_matrix(rx, ry, rz, seq='xyz'):
    """
    欧拉角转旋转矩阵
    
    Args:
        rx, ry, rz: 欧拉角（弧度）
        seq: 旋转顺序
    Returns:
        3x3 旋转矩阵
    """
    r = R.from_euler(seq, [rx, ry, rz], degrees=False)
    return r.as_matrix()


def rvec_to_rotation_matrix(rvec):
    """
    旋转向量（Rodrigues）转旋转矩阵
    
    Args:
        rvec: 3x1 旋转向量
    Returns:
        3x3 旋转矩阵
    """
    import cv2
    R_mat, _ = cv2.Rodrigues(rvec)
    return R_mat


def rotation_matrix_to_rvec(R_mat):
    """
    旋转矩阵转旋转向量
    
    Args:
        R_mat: 3x3 旋转矩阵
    Returns:
        3x1 旋转向量
    """
    import cv2
    rvec, _ = cv2.Rodrigues(R_mat)
    return rvec


def normalize_angle(angle):
    """
    角度归一化到 [-pi, pi]
    
    Args:
        angle: 弧度
    Returns:
        归一化后的角度（弧度）
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


def clamp(value, min_val, max_val):
    """
    限幅函数
    
    Args:
        value: 输入值
        min_val: 最小值
        max_val: 最大值
    Returns:
        限幅后的值
    """
    return max(min_val, min(value, max_val))


def clamp_vector(vec, max_norm):
    """
    向量按范数限幅
    
    Args:
        vec: numpy array
        max_norm: 最大范数
    Returns:
        限幅后的向量
    """
    norm = np.linalg.norm(vec)
    if norm > max_norm:
        return vec * (max_norm / norm)
    return vec


def apply_deadband(value, deadband):
    """
    死区函数
    
    Args:
        value: 输入值
        deadband: 死区半径
    Returns:
        应用死区后的值
    """
    if abs(value) < deadband:
        return 0.0
    return value


def exponential_decay(value, dt, tau):
    """
    指数衰减
    
    Args:
        value: 当前值
        dt: 时间增量（秒）
        tau: 时间常数（秒）
    Returns:
        衰减后的值
    """
    return value * np.exp(-dt / tau)


def build_transform_matrix(R_mat, t_vec):
    """
    构建4x4齐次变换矩阵
    
    Args:
        R_mat: 3x3 旋转矩阵
        t_vec: 3x1 平移向量
    Returns:
        4x4 齐次变换矩阵
    """
    T = np.eye(4)
    T[0:3, 0:3] = R_mat
    T[0:3, 3] = t_vec.flatten()
    return T


def invert_transform(T):
    """
    求齐次变换矩阵的逆
    
    Args:
        T: 4x4 齐次变换矩阵
    Returns:
        逆变换矩阵
    """
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R.T
    T_inv[0:3, 3] = -R.T @ t
    return T_inv


def extract_yaw_from_rvec(rvec):
    """
    从旋转向量中提取 yaw 角（绕Z轴旋转）
    简化方法：转为旋转矩阵后提取 atan2(R21, R11)
    
    Args:
        rvec: 3x1 旋转向量
    Returns:
        yaw 角（弧度）
    """
    R_mat = rvec_to_rotation_matrix(rvec)
    # 假设 ZYX 欧拉角顺序，yaw = atan2(R10, R00)
    yaw = np.arctan2(R_mat[1, 0], R_mat[0, 0])
    return yaw


def extract_pitch_from_rvec(rvec):
    """
    从旋转向量中提取 pitch 角（绕Y轴旋转）
    
    Args:
        rvec: 3x1 旋转向量
    Returns:
        pitch 角（弧度）
    """
    R_mat = rvec_to_rotation_matrix(rvec)
    # ZYX欧拉角顺序，pitch = -asin(R20)
    pitch = -np.arcsin(np.clip(R_mat[2, 0], -1.0, 1.0))
    return pitch


def extract_roll_from_rvec(rvec):
    """
    从旋转向量中提取 roll 角（绕X轴旋转）
    
    Args:
        rvec: 3x1 旋转向量
    Returns:
        roll 角（弧度）
    """
    R_mat = rvec_to_rotation_matrix(rvec)
    # ZYX欧拉角顺序，roll = atan2(R21, R22)
    roll = np.arctan2(R_mat[2, 1], R_mat[2, 2])
    return roll


def lpf_update(prev_value, new_value, alpha):
    """
    一阶低通滤波
    output = alpha * new + (1-alpha) * prev
    
    Args:
        prev_value: 上一时刻滤波值
        new_value: 当前原始值
        alpha: 滤波系数 (0-1)，越大越接近新值
    Returns:
        滤波后的值
    """
    return alpha * new_value + (1.0 - alpha) * prev_value


if __name__ == "__main__":
    # 单元测试
    print("=== utils_math.py 单元测试 ===")
    
    # 测试旋转矩阵<->欧拉角
    rx, ry, rz = 0.1, 0.2, 0.3
    R_mat = euler_to_rotation_matrix(rx, ry, rz)
    rx2, ry2, rz2 = rotation_matrix_to_euler(R_mat)
    print(f"欧拉角往返: ({rx}, {ry}, {rz}) -> ({rx2:.4f}, {ry2:.4f}, {rz2:.4f})")
    
    # 测试限幅
    vec = np.array([3.0, 4.0, 0.0])
    vec_clamped = clamp_vector(vec, 2.0)
    print(f"向量限幅: {vec} -> {vec_clamped}, norm={np.linalg.norm(vec_clamped):.2f}")
    
    # 测试死区
    print(f"死区测试: apply_deadband(1.5, 2.0) = {apply_deadband(1.5, 2.0)}")
    print(f"死区测试: apply_deadband(3.0, 2.0) = {apply_deadband(3.0, 2.0)}")
    
    # 测试衰减
    val = 10.0
    dt = 0.05
    tau = 0.1
    val_decayed = exponential_decay(val, dt, tau)
    print(f"指数衰减: {val} -> {val_decayed:.4f} (dt={dt}, tau={tau})")
    
    print("测试完成！")
