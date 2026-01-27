#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_vs.py - FAIRINO FR5 + RealSense D435i 视觉伺服主程序
实现 30Hz 视觉外环 + 125Hz 机器人内环的多速率闭环控制
"""

import sys
import os
import argparse
import yaml
import time
import threading
import cv2
import numpy as np
import pyrealsense2 as rs

from visual_servo.fr5_driver import FR5Driver
from visual_servo.aruco_detector import ArucoDetector
from visual_servo.controller import VisualServoController
from visual_servo.handeye_io import load_handeye_transform, save_calibration_sample
from visual_servo.utils_math import build_transform_matrix
from visual_servo.realtime_plotter import RealtimePlotter


class VisualServoSystem:
    """视觉伺服系统主类"""
    
    def __init__(self, config_path):
        """
        初始化系统
        
        Args:
            config_path: 配置文件路径
        """
        # 加载配置
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        print("=" * 60)
        print("  FAIRINO FR5 + RealSense D435i 视觉伺服系统")
        print("=" * 60)
        
        # 初始化组件
        self.robot = None
        self.pipeline = None
        self.detector = None
        self.controller = None
        self.plotter = None  # 实时绘图器
        self.plotter = None  # 实时绘图器
        
        # 线程控制
        self.running = False
        self.servo_enabled = False  # 伺服使能标志（按键控制）
        self.vision_thread = None
        self.control_thread = None
        self.vision_lock = threading.Lock()
        
        # 共享数据
        self.latest_detection = {'detected': False, 'timestamp': 0}
        
        # 轨迹记录
        self.trajectory = []  # [(u, v), ...]
        self.max_trajectory_len = 200  # 最多保存200个点
        
        # 统计
        self.vision_fps = 0.0
        self.control_hz = 0.0
        self.last_print_time = 0
        
        # Roll调试统计
        self.roll_debug_counter = 0
        
    def initialize(self):
        """初始化所有硬件和软件组件"""
        print("\n[1/5] 初始化机器人连接...")
        self._init_robot()
        
        print("\n[2/5] 初始化相机...")
        self._init_camera()
        
        print("\n[3/5] 初始化ArUco检测器...")
        self._init_detector()
        
        print("\n[4/5] 初始化控制器...")
        self._init_controller()
        
        print("\n[5/5] 启动伺服模式...")
        if not self.robot.servo_move_start():
            raise RuntimeError("启动伺服模式失败！")
        
        print("\n✓ 系统初始化完成！\n")
    
    def _init_robot(self):
        """初始化机器人"""
        robot_cfg = self.config['robot']
        self.robot = FR5Driver(robot_cfg['ip'], cmdT=robot_cfg['cmdT'])
        
        if not self.robot.connect():
            raise RuntimeError("无法连接到机器人！")
    
    def _init_camera(self):
        """初始化 RealSense 相机"""
        camera_cfg = self.config['camera']
        
        # 创建 pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # 配置流
        config.enable_stream(
            rs.stream.color,
            camera_cfg.get('width', 640),
            camera_cfg.get('height', 480),
            rs.format.bgr8,
            camera_cfg.get('fps', 30)
        )
        
        # 启动 pipeline
        profile = self.pipeline.start(config)
        
        # 获取相机内参
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        self.camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)
        
        print(f"  相机内参: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}, "
              f"cx={intrinsics.ppx:.1f}, cy={intrinsics.ppy:.1f}")
        
        # 预热相机（丢弃前几帧）
        for _ in range(10):
            self.pipeline.wait_for_frames()
        
        print("  相机预热完成")
    
    def _init_detector(self):
        """初始化 ArUco 检测器"""
        aruco_cfg = self.config['aruco']
        
        self.detector = ArucoDetector(
            dictionary_name=aruco_cfg['dictionary'],
            tag_size=aruco_cfg['tag_size'],
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs
        )
    
    def _init_controller(self):
        """初始化控制器"""
        # 加载手眼标定外参
        T_tool_cam = load_handeye_transform(self.config)
        
        # 创建控制器
        self.controller = VisualServoController(self.config, T_tool_cam)
        
        # 设置相机内参
        intrinsics = self.camera_matrix
        self.controller.set_camera_intrinsics(
            fx=intrinsics[0, 0],
            fy=intrinsics[1, 1],
            cx=intrinsics[0, 2],
            cy=intrinsics[1, 2]
        )
    
    def vision_loop(self):
        """视觉线程（30Hz 外环）"""
        print("[INFO] 视觉线程已启动")
        
        frame_count = 0
        start_time = time.time()
        target_id = self.config['aruco']['target_id']
        enable_viz = self.config['debug'].get('enable_viz', True)
        
        while self.running:
            try:
                # 采集图像
                frames = self.pipeline.wait_for_frames(timeout_ms=100)
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    continue
                
                # 转为 numpy
                image = np.asanyarray(color_frame.get_data())
                
                # ArUco 检测
                detection = self.detector.detect(image, target_id=target_id)
                
                # 记录轨迹（仅当伺服启用时）
                if self.servo_enabled and detection['detected']:
                    center_u, center_v = detection['center_px']
                    self.trajectory.append((int(center_u), int(center_v)))
                    # 限制轨迹长度
                    if len(self.trajectory) > self.max_trajectory_len:
                        self.trajectory.pop(0)
                
                # 计算 twist
                if detection['detected']:
                    twist_cam, timestamp = self.controller.compute_twist_from_detection(detection)
                    
                    # 更新控制器
                    self.controller.update_twist(twist_cam, timestamp)
                
                # 更新共享数据
                with self.vision_lock:
                    self.latest_detection = detection
                
                # 可视化
                if enable_viz:
                    vis_img = self.detector.draw_detection(
                        image, detection,
                        cx=self.camera_matrix[0, 2],
                        cy=self.camera_matrix[1, 2],
                        trajectory=self.trajectory if self.servo_enabled else None
                    )
                    
                    # 显示伺服状态
                    status_text = "SERVO: ON" if self.servo_enabled else "SERVO: OFF"
                    status_color = (0, 255, 0) if self.servo_enabled else (0, 0, 255)
                    cv2.putText(vis_img, status_text, (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
                    
                    # 显示按键提示
                    cv2.putText(vis_img, "S-Start P-Pause R-Reset Q-Quit", (10, image.shape[0]-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                    
                    cv2.imshow("Visual Servo", vis_img)
                    
                    # 键盘控制
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("\n[INFO] 检测到 'q' 键，准备退出...")
                        self.running = False
                        # 立即关闭OpenCV窗口，避免卡顿
                        cv2.destroyWindow("Visual Servo")
                        break  # 退出视觉循环
                        # 立即关闭OpenCV窗口，避免卡顿
                        cv2.destroyWindow("Visual Servo")
                        break  # 退出视觉循环
                    elif key == ord('s'):
                        self.servo_enabled = True
                        self.trajectory = []  # 清空轨迹，重新记录
                        print("[INFO] 伺服已启动 (按 'p' 暂停)")
                    elif key == ord('p'):
                        self.servo_enabled = False
                        self.controller.reset()
                        print("[INFO] 伺服已暂停 (按 's' 启动)")
                    elif key == ord('r'):
                        self.controller.reset()
                        self.trajectory = []  # 清空轨迹
                        print("[INFO] 控制器和轨迹已重置")
                
                # 统计
                frame_count += 1
                elapsed = time.time() - start_time
                if elapsed >= 1.0:
                    self.vision_fps = frame_count / elapsed
                    frame_count = 0
                    start_time = time.time()
            
            except Exception as e:
                print(f"[ERROR] 视觉线程异常: {e}")
                time.sleep(0.01)
        
        print("[INFO] 视觉线程已退出")
    
    def control_loop(self):
        """控制线程（125Hz 内环）"""
        print("[INFO] 控制线程已启动")
        
        dt = self.config['robot']['cmdT']
        tick_count = 0
        start_time = time.time()
        error_count = 0  # ServoCart错误计数
        error_threshold = 10  # 连续错误阈值
        
        while self.running:
            tick_start = time.perf_counter()
            
            try:
                # 获取伺服指令（仅当伺服使能时）
                if self.servo_enabled:
                    desc_pos = self.controller.get_servo_command(dt=dt)
                    
                    # 下发指令
                    if desc_pos is not None:
                        # 记录数据到绘图器（如果启用）
                        if self.plotter is not None:
                            with self.vision_lock:
                                det = self.latest_detection
                            if det['detected']:
                                # 计算速度（mm/s）
                                vx = desc_pos[0] / dt  # mm/s
                                vy = desc_pos[1] / dt
                                vz = desc_pos[2] / dt
                                
                                # 获取图像误差
                                u, v = det['center_px']
                                ex = u - self.controller.cx
                                ey = v - self.controller.cy
                                ez = (det['tvec'][2] - self.controller.z_des) * 1000.0  # mm
                                
                                self.plotter.add_data(vx, vy, vz, ex, ey, ez, time.time())
                        
                        # 调试：每1秒打印一次指令
                        if tick_count % 125 == 0:  # 125Hz -> 每秒1次
                            print(f"[DEBUG] 工具系增量(mm,deg): "
                                  f"[{desc_pos[0]:+6.3f}, {desc_pos[1]:+6.3f}, {desc_pos[2]:+6.3f}, "
                                  f"{desc_pos[3]:+6.3f}, {desc_pos[4]:+6.3f}, {desc_pos[5]:+6.3f}]")
                        
                        # 执行伺服运动
                        success = self.robot.servo_cart(desc_pos)
                        
                        # Roll状态监控：每30个tick（约0.24秒）输出一次状态（仅在verbose模式）
                        if self.config.get('debug', {}).get('verbose_log', False):
                            self.roll_debug_counter += 1
                            if self.roll_debug_counter >= 30 and self.config['target'].get('enable_roll', False):
                                with self.vision_lock:
                                    det = self.latest_detection.copy()
                                if det['detected'] and det.get('rvec') is not None:
                                    from visual_servo.utils_math import extract_roll_from_rvec
                                    roll = extract_roll_from_rvec(det['rvec'])
                                    print(f"[ROLL_STATUS] 当前状态: roll={np.rad2deg(roll):+7.2f}° | 最后命令: drx={desc_pos[3]:+7.3f}° | 控制增益: k_roll={self.controller.k_roll}")
                                self.roll_debug_counter = 0
                        
                        # 错误处理
                        if not success:
                            error_count += 1
                            if error_count == 1:  # 第一次出错时提示
                                print(f"[WARNING] ServoCart失败，可能接近工作空间边界或奇异点")
                            if error_count >= error_threshold:
                                print(f"[ERROR] 连续{error_count}次ServoCart失败，自动暂停伺服")
                                self.servo_enabled = False
                                error_count = 0
                        else:
                            error_count = 0  # 成功则清零
                else:
                    # 伺服未使能时，重置控制器（清除累积状态）
                    if self.controller.is_initialized:
                        # 不频繁重置，避免影响性能
                        pass
                
                # 统计
                tick_count += 1
                elapsed = time.time() - start_time
                if elapsed >= 1.0:
                    self.control_hz = tick_count / elapsed
                    tick_count = 0
                    start_time = time.time()
                
                # 定时打印
                self._print_status()
            
            except Exception as e:
                print(f"[ERROR] 控制线程异常: {e}")
            
            # 精确定时（补偿执行时间）
            tick_elapsed = time.perf_counter() - tick_start
            sleep_time = dt - tick_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                print(f"[WARNING] 控制循环超时: {tick_elapsed*1000:.2f}ms > {dt*1000:.2f}ms")
        
        print("[INFO] 控制线程已退出")
    
    def _print_status(self):
        """打印状态信息"""
        current_time = time.time()
        print_freq = self.config['debug'].get('print_freq_hz', 2.0)
        
        if current_time - self.last_print_time >= (1.0 / print_freq):
            with self.vision_lock:
                detection = self.latest_detection.copy()
            
            status_prefix = "[SERVO ON]" if self.servo_enabled else "[SERVO OFF]"
            
            if detection['detected']:
                u, v = detection['center_px']
                z = detection['tvec'][2] if detection['tvec'] is not None else 0
                tag_id = detection['tag_id']
                rvec = detection.get('rvec')
                
                ex = u - self.camera_matrix[0, 2]
                ey = v - self.camera_matrix[1, 2]
                ez = (z - self.config['target']['z_des']) * 1000  # mm
                
                # 提取姿态角（如果启用）
                roll_str = ""
                pitch_str = ""
                yaw_str = ""
                if rvec is not None:
                    from visual_servo.utils_math import extract_roll_from_rvec, extract_pitch_from_rvec, extract_yaw_from_rvec
                    if self.config['target'].get('enable_roll', False):
                        roll = extract_roll_from_rvec(rvec)
                        roll_str = f" Roll:{np.rad2deg(roll):+6.1f}°"
                    if self.config['target'].get('enable_pitch', False):
                        pitch = extract_pitch_from_rvec(rvec)
                        pitch_str = f" Pitch:{np.rad2deg(pitch):+6.1f}°"
                    if self.config['target'].get('enable_yaw', False):
                        yaw = extract_yaw_from_rvec(rvec)
                        yaw_str = f" Yaw:{np.rad2deg(yaw):+6.1f}°"
                        yaw = extract_yaw_from_rvec(rvec)
                        yaw_str = f" Yaw:{np.rad2deg(yaw):+6.1f}°"
                
                print(f"{status_prefix} FPS:{self.vision_fps:.1f} Hz:{self.control_hz:.1f} | "
                      f"ID:{tag_id} | err_px:[{ex:+6.1f}, {ey:+6.1f}] err_z:{ez:+6.1f}mm | "
                      f"Z:{z*1000:.1f}mm{roll_str}{pitch_str}{yaw_str}")
            else:
                print(f"{status_prefix} FPS:{self.vision_fps:.1f} Hz:{self.control_hz:.1f} | "
                      f"No Tag Detected")
            
            self.last_print_time = current_time
    
    def start(self):
        """启动双线程视觉伺服"""
        print("[INFO] 启动视觉伺服系统...")
        
        self.running = True
        
        # 启动实时绘图器（如果配置启用）
        if self.config['debug'].get('enable_realtime_plot', True):
            print("\n[启动] 实时监测曲线绘图器...")
            self.plotter = RealtimePlotter(max_points=300, update_interval=50)
            self.plotter.start()
        
        # 启动视觉线程
        self.vision_thread = threading.Thread(target=self.vision_loop, daemon=True)
        self.vision_thread.start()
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        print("\n" + "="*60)
        print("  系统运行中...")
        print("  按键控制:")
        print("    S - 启动伺服")
        print("    P - 暂停伺服")
        print("    R - 重置控制器")
        print("    Q - 退出系统 (或 Ctrl+C)")
        print("="*60 + "\n")
        
        try:
            # 主线程等待
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n[INFO] 检测到 Ctrl+C，准备退出...")
            self.running = False
    
    def stop(self):
        """停止系统"""
        print("\n[INFO] 停止视觉伺服系统...")
        
        self.running = False
        
        # 先关闭绘图器（后台线程，快速）
        if self.plotter:
            self.plotter.stop()
            print("  ✓ 绘图器已关闭")
        
        # 立即关闭所有OpenCV窗口
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # 强制刷新事件队列
        print("  ✓ 图像窗口已关闭")
        
        # 等待线程结束（缩短超时时间）
        if self.vision_thread is not None:
            self.vision_thread.join(timeout=0.5)
        if self.control_thread is not None:
            self.control_thread.join(timeout=0.5)
        
        # 关闭硬件
        if self.robot is not None:
            self.robot.servo_move_end()
            self.robot.disconnect()
        
        if self.pipeline is not None:
            self.pipeline.stop()
        
        print("\n[INFO] 系统已安全退出\n")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="FAIRINO FR5 视觉伺服系统")
    parser.add_argument('--config', type=str, default='config/config.yaml',
                       help='配置文件路径')
    parser.add_argument('--dictionary', type=str, default=None,
                       help='ArUco字典（覆盖配置文件）')
    parser.add_argument('--target-id', type=int, default=None,
                       help='目标Tag ID（-1=自动，>=0=指定ID）')
    parser.add_argument('--z-des', type=float, default=None,
                       help='期望距离（米，覆盖配置文件）')
    
    args = parser.parse_args()
    
    # 检查配置文件
    if not os.path.exists(args.config):
        print(f"[ERROR] 配置文件不存在: {args.config}")
        sys.exit(1)
    
    # 加载配置
    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)
    
    # 命令行参数覆盖
    if args.dictionary is not None:
        config['aruco']['dictionary'] = args.dictionary
    if args.target_id is not None:
        config['aruco']['target_id'] = args.target_id
    if args.z_des is not None:
        config['target']['z_des'] = args.z_des
    
    # 保存修改后的配置
    temp_config_path = '/tmp/vs_runtime_config.yaml'
    with open(temp_config_path, 'w') as f:
        yaml.dump(config, f)
    
    # 创建系统
    system = VisualServoSystem(temp_config_path)
    
    try:
        # 初始化
        system.initialize()
        
        # 启动
        system.start()
    
    except Exception as e:
        print(f"\n[ERROR] 系统异常: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 停止
        system.stop()


if __name__ == "__main__":
    main()
