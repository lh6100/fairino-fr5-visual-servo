#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
aruco_detector.py - ArUco Tag 检测与位姿估计模块
"""

import cv2
import numpy as np
import time


class ArucoDetector:
    """ArUco Tag 检测器"""
    
    def __init__(self, dictionary_name="DICT_4X4_50", tag_size=0.10, camera_matrix=None, dist_coeffs=None):
        """
        初始化 ArUco 检测器
        
        Args:
            dictionary_name: ArUco 字典名称（字符串）
            tag_size: Tag 边长（米）
            camera_matrix: 相机内参矩阵 3x3
            dist_coeffs: 畸变系数 (k1, k2, p1, p2, k3)
        """
        self.dictionary_name = dictionary_name
        self.tag_size = tag_size
        
        # 加载 ArUco 字典
        aruco_dict_attr = getattr(cv2.aruco, dictionary_name, None)
        if aruco_dict_attr is None:
            raise ValueError(f"不支持的 ArUco 字典: {dictionary_name}")
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_attr)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # 相机内参
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)
        
        print(f"[INFO] ArUco 检测器初始化: {dictionary_name}, tag_size={tag_size}m")
    
    def set_camera_intrinsics(self, camera_matrix, dist_coeffs=None):
        """
        设置相机内参
        
        Args:
            camera_matrix: 3x3 内参矩阵
            dist_coeffs: 畸变系数
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)
    
    def detect(self, image, target_id=-1):
        """
        检测 ArUco Tag
        
        Args:
            image: BGR 图像
            target_id: 目标 ID（-1 表示自动选择最大面积的 Tag）
        Returns:
            检测结果字典，包含：
            {
                'detected': bool,
                'tag_id': int,
                'corners': numpy array (4x2),
                'center_px': (u, v),
                'rvec': 旋转向量 (3x1),
                'tvec': 平移向量 (3x1, 单位米),
                'area': 像素面积,
                'timestamp': 时间戳
            }
            如果未检测到，返回 {'detected': False}
        """
        timestamp = time.time()
        
        # 转为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # 检测 Markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is None or len(ids) == 0:
            return {'detected': False, 'timestamp': timestamp}
        
        # 目标选择
        selected_idx = -1
        selected_id = -1
        
        if target_id >= 0:
            # 跟随指定 ID
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == target_id:
                    selected_idx = i
                    selected_id = marker_id
                    break
            if selected_idx == -1:
                # 未找到目标 ID
                return {'detected': False, 'timestamp': timestamp, 'reason': f'target_id={target_id} not found'}
        else:
            # 自动选择：优先最大面积
            max_area = 0
            for i, corner in enumerate(corners):
                area = cv2.contourArea(corner[0])
                if area > max_area:
                    max_area = area
                    selected_idx = i
                    selected_id = ids[i][0]
        
        # 提取选中的 Tag
        selected_corners = corners[selected_idx][0]  # shape (4, 2)
        
        # 计算中心像素坐标
        center_u = np.mean(selected_corners[:, 0])
        center_v = np.mean(selected_corners[:, 1])
        
        # 位姿估计
        rvec, tvec = None, None
        if self.camera_matrix is not None:
            # 使用 solvePnP（更稳定）
            object_points = np.array([
                [-self.tag_size/2,  self.tag_size/2, 0],
                [ self.tag_size/2,  self.tag_size/2, 0],
                [ self.tag_size/2, -self.tag_size/2, 0],
                [-self.tag_size/2, -self.tag_size/2, 0]
            ], dtype=np.float32)
            
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                selected_corners.astype(np.float32),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            if not success:
                print(f"[WARNING] solvePnP 失败 for tag_id={selected_id}")
                return {'detected': False, 'timestamp': timestamp, 'reason': 'solvePnP failed'}
        
        # 计算面积
        area = cv2.contourArea(selected_corners)
        
        result = {
            'detected': True,
            'tag_id': int(selected_id),
            'corners': selected_corners,
            'center_px': (center_u, center_v),
            'rvec': rvec.flatten() if rvec is not None else None,
            'tvec': tvec.flatten() if tvec is not None else None,
            'area': area,
            'timestamp': timestamp
        }
        
        return result
    
    def draw_detection(self, image, detection_result, cx=None, cy=None, trajectory=None):
        """
        在图像上绘制检测结果
        
        Args:
            image: BGR 图像
            detection_result: detect() 返回的结果
            cx, cy: 相机主点（可选，用于绘制目标十字）
            trajectory: Tag中心轨迹历史列表 [(u1,v1), (u2,v2), ...]
        Returns:
            绘制后的图像
        """
        vis = image.copy()
        
        # === 绘制轨迹线 ===
        if trajectory is not None and len(trajectory) > 1:
            # 绘制轨迹线（青色）
            pts = np.array(trajectory, dtype=np.int32)
            for i in range(1, len(pts)):
                # 渐变颜色：旧的轨迹点更淡
                alpha = i / len(pts)  # 0到1
                color = (255, int(255 * alpha), 0)  # 青色到亮青色
                thickness = max(1, int(2 * alpha))  # 越新越粗
                cv2.line(vis, tuple(pts[i-1]), tuple(pts[i]), color, thickness)
            
            # 标注起点（绿色圆圈）
            cv2.circle(vis, tuple(pts[0]), 8, (0, 255, 0), 2)
            cv2.putText(vis, "START", (pts[0][0] + 10, pts[0][1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        if not detection_result['detected']:
            # 绘制 "No Tag Detected"
            cv2.putText(vis, "No Tag Detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return vis
        
        corners = detection_result['corners']
        tag_id = detection_result['tag_id']
        center_u, center_v = detection_result['center_px']
        
        # 绘制边界
        cv2.polylines(vis, [corners.astype(np.int32)], True, (0, 255, 0), 2)
        
        # 绘制中心（当前位置，红色）
        cv2.circle(vis, (int(center_u), int(center_v)), 7, (0, 0, 255), -1)
        
        # 绘制 ID
        cv2.putText(vis, f"ID:{tag_id}", (int(corners[0][0]), int(corners[0][1])-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # 绘制坐标轴（如果有位姿）
        if detection_result['rvec'] is not None and self.camera_matrix is not None:
            cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs,
                             detection_result['rvec'], detection_result['tvec'], 
                             self.tag_size * 0.5)
        
        # 绘制相机主点十字（目标位置，黄色）
        if cx is not None and cy is not None:
            cv2.drawMarker(vis, (int(cx), int(cy)), (0, 255, 255), 
                          cv2.MARKER_CROSS, 30, 2)
            cv2.putText(vis, "Target", (int(cx) + 15, int(cy) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # 显示距离信息
        if detection_result['tvec'] is not None:
            z = detection_result['tvec'][2]
            cv2.putText(vis, f"Z:{z*1000:.1f}mm", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        return vis


if __name__ == "__main__":
    # 测试代码（需要相机）
    print("=== aruco_detector.py 单元测试 ===")
    
    # 创建检测器
    detector = ArucoDetector(dictionary_name="DICT_4X4_50", tag_size=0.10)
    
    # 模拟相机内参
    fx, fy = 600.0, 600.0
    cx, cy = 320.0, 240.0
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    detector.set_camera_intrinsics(camera_matrix)
    
    # 生成测试图像（打印 ArUco Tag 后拍摄，或使用生成的图像）
    print("[INFO] 检测器已创建，运行需要实际相机图像")
    print("[INFO] 字典:", detector.dictionary_name)
    print("[INFO] Tag尺寸:", detector.tag_size, "m")
    
    print("测试完成！")
