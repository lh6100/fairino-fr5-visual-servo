#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
handeye_io.py - 手眼标定数据IO模块
负责加载外参、保存标定采样数据
"""

import os
import json
import numpy as np
from datetime import datetime
import yaml


def load_handeye_transform(config):
    """
    从配置文件加载手眼标定外参 T_tool_cam
    
    Args:
        config: 配置字典
    Returns:
        T_tool_cam: 4x4 numpy array（工具系到相机系的齐次变换矩阵）
    """
    handeye_cfg = config.get('handeye', {})
    T_cfg = handeye_cfg.get('T_tool_cam', {})
    
    R_list = T_cfg.get('R', [])
    t_list = T_cfg.get('t', [])
    
    if not R_list or not t_list:
        print("[WARNING] 未找到手眼标定外参，使用单位矩阵（相机与工具重合）")
        return np.eye(4)
    
    R = np.array(R_list, dtype=np.float64)
    t = np.array(t_list, dtype=np.float64)
    
    T_tool_cam = np.eye(4)
    T_tool_cam[0:3, 0:3] = R
    T_tool_cam[0:3, 3] = t
    
    print("[INFO] 成功加载手眼标定外参 T_tool_cam:")
    print(T_tool_cam)
    
    return T_tool_cam


def save_calibration_sample(save_dir, tcp_pose, tag_pose, tag_id, timestamp=None):
    """
    保存一组手眼标定采样数据
    
    Args:
        save_dir: 保存目录
        tcp_pose: 机器人 TCP 位姿，字典 {'xyz': [x,y,z], 'rpy': [rx,ry,rz]} 或 4x4 矩阵
        tag_pose: Tag 位姿（相机系），字典 {'rvec': [...], 'tvec': [...]} 或 4x4 矩阵
        tag_id: Tag ID
        timestamp: 时间戳（可选）
    Returns:
        保存的文件路径
    """
    os.makedirs(save_dir, exist_ok=True)
    
    if timestamp is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    
    # 构造样本数据
    sample = {
        'timestamp': timestamp,
        'tag_id': int(tag_id),
        'tcp_pose': {},
        'tag_pose': {}
    }
    
    # 处理 TCP 位姿
    if isinstance(tcp_pose, dict):
        sample['tcp_pose'] = {
            'xyz': [float(x) for x in tcp_pose.get('xyz', [0, 0, 0])],
            'rpy': [float(x) for x in tcp_pose.get('rpy', [0, 0, 0])]
        }
    elif isinstance(tcp_pose, np.ndarray) and tcp_pose.shape == (4, 4):
        # 如果是4x4矩阵，转为xyz+rpy
        from utils_math import rotation_matrix_to_euler
        xyz = tcp_pose[0:3, 3].tolist()
        R = tcp_pose[0:3, 0:3]
        rpy = rotation_matrix_to_euler(R, seq='xyz').tolist()
        sample['tcp_pose'] = {'xyz': xyz, 'rpy': rpy}
    
    # 处理 Tag 位姿
    if isinstance(tag_pose, dict):
        sample['tag_pose'] = {
            'rvec': [float(x) for x in tag_pose.get('rvec', [0, 0, 0])],
            'tvec': [float(x) for x in tag_pose.get('tvec', [0, 0, 0])]
        }
    elif isinstance(tag_pose, np.ndarray) and tag_pose.shape == (4, 4):
        # 如果是4x4矩阵，转为rvec+tvec
        from utils_math import rotation_matrix_to_rvec
        tvec = tag_pose[0:3, 3].tolist()
        R = tag_pose[0:3, 0:3]
        rvec = rotation_matrix_to_rvec(R).flatten().tolist()
        sample['tag_pose'] = {'rvec': rvec, 'tvec': tvec}
    
    # 保存为JSON（追加模式）
    filename = f"samples_{datetime.now().strftime('%Y%m%d')}.json"
    filepath = os.path.join(save_dir, filename)
    
    # 读取已有数据
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                data = {'samples': []}
    else:
        data = {'samples': []}
    
    # 追加新样本
    data['samples'].append(sample)
    
    # 写回文件
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"[INFO] 标定样本已保存: {filepath} (共 {len(data['samples'])} 个样本)")
    return filepath


def load_calibration_samples(filepath):
    """
    加载手眼标定样本数据
    
    Args:
        filepath: JSON文件路径
    Returns:
        samples 列表
    """
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        return []
    
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    samples = data.get('samples', [])
    print(f"[INFO] 成功加载 {len(samples)} 个标定样本")
    return samples


def export_for_opencv_handeye(samples, output_dir):
    """
    将采样数据导出为 OpenCV hand-eye calibration 格式
    生成两个numpy数组：R_gripper2base, t_gripper2base 和 R_target2cam, t_target2cam
    
    Args:
        samples: 样本列表
        output_dir: 输出目录
    Returns:
        导出的文件路径列表
    """
    os.makedirs(output_dir, exist_ok=True)
    
    from utils_math import euler_to_rotation_matrix, rvec_to_rotation_matrix
    
    R_gripper2base_list = []
    t_gripper2base_list = []
    R_target2cam_list = []
    t_target2cam_list = []
    
    for sample in samples:
        # TCP pose (base->tool)
        tcp = sample['tcp_pose']
        xyz = np.array(tcp['xyz'])
        rpy = np.array(tcp['rpy'])
        R_tool = euler_to_rotation_matrix(*rpy)
        t_tool = xyz
        
        R_gripper2base_list.append(R_tool)
        t_gripper2base_list.append(t_tool)
        
        # Tag pose (cam->tag)
        tag = sample['tag_pose']
        rvec = np.array(tag['rvec'])
        tvec = np.array(tag['tvec'])
        R_tag = rvec_to_rotation_matrix(rvec)
        t_tag = tvec
        
        R_target2cam_list.append(R_tag)
        t_target2cam_list.append(t_tag)
    
    # 保存为numpy格式
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    np.savez(
        os.path.join(output_dir, f"handeye_data_{timestamp}.npz"),
        R_gripper2base=np.array(R_gripper2base_list),
        t_gripper2base=np.array(t_gripper2base_list),
        R_target2cam=np.array(R_target2cam_list),
        t_target2cam=np.array(t_target2cam_list)
    )
    
    print(f"[INFO] 已导出 {len(samples)} 组数据到 {output_dir}/handeye_data_{timestamp}.npz")
    print("[INFO] 可使用 cv2.calibrateHandEye() 进行求解")
    
    return os.path.join(output_dir, f"handeye_data_{timestamp}.npz")


if __name__ == "__main__":
    # 测试代码
    print("=== handeye_io.py 单元测试 ===")
    
    # 测试加载配置
    test_config = {
        'handeye': {
            'T_tool_cam': {
                'R': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                't': [0.05, 0.0, 0.08]
            }
        }
    }
    T = load_handeye_transform(test_config)
    print("加载的外参:\n", T)
    
    # 测试保存样本
    test_tcp = {'xyz': [0.3, 0.1, 0.5], 'rpy': [0, 0, 1.57]}
    test_tag = {'rvec': [0.1, 0.2, 0.3], 'tvec': [0, 0, 0.4]}
    save_calibration_sample("/tmp/test_calib", test_tcp, test_tag, tag_id=5)
    
    print("测试完成！")
