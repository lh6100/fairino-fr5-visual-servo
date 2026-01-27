#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fr5_driver.py - FAIRINO FR5 机械臂驱动封装
基于官方 Python SDK (frrpc.py)
"""

import time
import numpy as np


class FR5Driver:
    """FAIRINO FR5 机械臂驱动封装类"""
    
    def __init__(self, robot_ip, cmdT=0.008):
        """
        初始化机器人连接
        
        Args:
            robot_ip: 机器人控制器IP地址
            cmdT: 伺服周期（秒），默认 0.008
        """
        self.robot_ip = robot_ip
        self.cmdT = cmdT
        self.robot = None
        self.is_servo_started = False
        
        print(f"[INFO] FR5 驱动初始化: IP={robot_ip}, cmdT={cmdT}s")
    
    def connect(self):
        """建立与机器人的连接"""
        try:
            # 导入 FAIRINO Python SDK
            # 尝试两种导入方式：fairino 或 frrpc
            try:
                from fairino import Robot
                self.robot = Robot.RPC(self.robot_ip)
            except ImportError:
                import frrpc
                self.robot = frrpc.RPC(self.robot_ip)
            
            # 测试连接
            err, version = self.robot.GetSDKVersion()
            if err == 0:
                print(f"[INFO] 成功连接到机器人: {self.robot_ip}")
                print(f"[INFO] SDK版本: {version}")
                return True
            else:
                print(f"[ERROR] 连接失败: 错误码 {err}")
                return False
        except Exception as e:
            print(f"[ERROR] 连接机器人异常: {e}")
            print("[HINT] 请确保已安装 FAIRINO Python SDK")
            print("[HINT] 可通过以下方式之一安装:")
            print("       1. pip install fairino (如果有官方包)")
            print("       2. 将 frrpc.py 放入当前目录或 PYTHONPATH")
            return False
    
    def disconnect(self):
        """断开与机器人的连接"""
        if self.is_servo_started:
            self.servo_move_end()
        
        if self.robot is not None:
            # SDK 无显式断开方法，直接置空
            self.robot = None
            print("[INFO] 已断开机器人连接")
    
    def servo_move_start(self):
        """启动伺服模式"""
        if self.robot is None:
            print("[ERROR] 机器人未连接！")
            return False
        
        try:
            # 先清除所有错误状态
            self.robot.ResetAllError()
            time.sleep(0.05)  # 等待错误清除生效
            
            err = self.robot.ServoMoveStart()
            if err == 0:
                self.is_servo_started = True
                print("[INFO] 伺服模式已启动")
                return True
            else:
                print(f"[ERROR] 启动伺服失败: 错误码 {err}")
                return False
        except Exception as e:
            print(f"[ERROR] 启动伺服异常: {e}")
            return False
    
    def servo_move_end(self):
        """结束伺服模式"""
        if self.robot is None:
            return
        
        try:
            err = self.robot.ServoMoveEnd()
            if err == 0:
                self.is_servo_started = False
                print("[INFO] 伺服模式已结束")
            else:
                print(f"[WARNING] 结束伺服返回: 错误码 {err}")
        except Exception as e:
            print(f"[ERROR] 结束伺服异常: {e}")
    
    def servo_cart_absolute(self, target_pose):
        """
        笛卡尔伺服（绝对位姿模式）
        
        Args:
            target_pose: 目标绝对位姿 [x, y, z, rx, ry, rz]
                        平移单位 mm，旋转单位 deg，基坐标系
        Returns:
            bool: 是否成功
        """
        if not self.is_servo_started:
            print("[ERROR] 伺服模式未启动！")
            return False
        
        try:
            # 调用 SDK 接口
            # ServoCart(mode, desc_pos, pos_gain, acc, vel, cmdT, filterT, gain)
            # mode=0: 基坐标系绝对位姿
            err = self.robot.ServoCart(
                mode=0,
                desc_pos=target_pose.tolist() if isinstance(target_pose, np.ndarray) else target_pose,
                pos_gain=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5],  # 降低增益避免抖动
                acc=0.0,
                vel=0.0,
                cmdT=self.cmdT,
                filterT=0.0,
                gain=0.0
            )
            
            if err != 0:
                if err == 14:
                    try:
                        self.robot.ResetAllError()
                    except:
                        pass
                    print(f"[WARNING] ServoCart 返回错误码: {err}，已尝试清除错误")
                elif err == 112:
                    pass  # 静默处理
                else:
                    print(f"[WARNING] ServoCart 返回错误码: {err}")
                return False
            
            return True
        except Exception as e:
            print(f"[ERROR] ServoCart 异常: {e}")
            return False
    
    def servo_cart(self, desc_pos):
        """
        笛卡尔伺服（增量模式）- 保留向后兼容
        
        Args:
            desc_pos: 增量位姿 [dx, dy, dz, drx, dry, drz]
                     平移单位 mm，旋转单位 deg
        Returns:
            bool: 是否成功
        """
        if not self.is_servo_started:
            print("[ERROR] 伺服模式未启动！")
            return False
        
        try:
            # 调用 SDK 接口
            # ServoCart(mode, desc_pos, pos_gain, acc, vel, cmdT, filterT, gain)
            # mode=2: 工具坐标系增量
            err = self.robot.ServoCart(
                mode=2,
                desc_pos=desc_pos.tolist() if isinstance(desc_pos, np.ndarray) else desc_pos,
                pos_gain=[0.3, 0.3, 0.3, 0.3, 0.3, 0.3],  # 降低增益避免错误14
                acc=0.0,  # 加速度（由机器人内部控制）
                vel=0.0,  # 速度（由机器人内部控制）
                cmdT=self.cmdT,
                filterT=0.0,  # 滤波时间常数（0 表示不滤波）
                gain=0.0  # 额外增益
            )
            
            if err != 0:
                # 错误码14：指令执行失败 - 可能是之前的错误状态未清除
                if err == 14:
                    # 尝试清除错误状态
                    try:
                        self.robot.ResetAllError()
                    except:
                        pass
                    print(f"[WARNING] ServoCart 返回错误码: {err}，已尝试清除错误")
                # 错误码112：目标位姿无法到达（可能接近奇异点或工作空间边界）
                elif err == 112:
                    # 不打印过多警告，返回False让控制器自动降速
                    pass  # 静默处理，避免刷屏
                else:
                    print(f"[WARNING] ServoCart 返回错误码: {err}")
                return False
            
            return True
        except Exception as e:
            print(f"[ERROR] ServoCart 异常: {e}")
            return False
    
    def get_tcp_pose(self, tool=0, user=0):
        """
        获取当前 TCP 位姿
        
        Args:
            tool: 工具坐标系编号
            user: 用户坐标系编号
        Returns:
            位姿字典 {'xyz': [x, y, z], 'rpy': [rx, ry, rz]}
            失败返回 None
        """
        if self.robot is None:
            return None
        
        try:
            err, pose = self.robot.GetActualTCPPose(0)
            if err == 0:
                # pose 格式: [x, y, z, rx, ry, rz]（单位 mm, deg）
                return {
                    'xyz': pose[0:3],
                    'rpy': pose[3:6]
                }
            else:
                print(f"[WARNING] 获取TCP位姿失败: 错误码 {err}")
                return None
        except Exception as e:
            print(f"[ERROR] 获取TCP位姿异常: {e}")
            return None
    
    def get_joint_positions(self):
        """
        获取当前关节角度
        
        Returns:
            关节角度列表（度），失败返回 None
        """
        if self.robot is None:
            return None
        
        try:
            err, joint_pos = self.robot.GetActualJointPosDegree()
            if err == 0:
                return joint_pos
            else:
                print(f"[WARNING] 获取关节角度失败: 错误码 {err}")
                return None
        except Exception as e:
            print(f"[ERROR] 获取关节角度异常: {e}")
            return None
    
    def move_j(self, joint_pos, desc_pos, tool=0, user=0, vel=20.0, acc=100.0, ovl=100.0):
        """
        关节运动（用于标定采样等移动任务）
        
        Args:
            joint_pos: 关节角度 [j1, j2, j3, j4, j5, j6]（度）
            desc_pos: 笛卡尔位姿 [x, y, z, rx, ry, rz]（mm, deg）
            vel: 速度百分比
            acc: 加速度百分比
            ovl: 速度缩放
        Returns:
            bool: 是否成功
        """
        if self.robot is None:
            return False
        
        try:
            err = self.robot.MoveJ(joint_pos, desc_pos, tool, user, vel, acc, ovl, 
                                   epos=[0,0,0,0], search=0, offset_flag=0, offset_pos=[0,0,0,0,0,0])
            if err == 0:
                return True
            else:
                print(f"[WARNING] MoveJ 返回错误码: {err}")
                return False
        except Exception as e:
            print(f"[ERROR] MoveJ 异常: {e}")
            return False


if __name__ == "__main__":
    # 测试代码
    print("=== fr5_driver.py 单元测试 ===")
    
    # 创建驱动（需要实际机器人）
    driver = FR5Driver("192.168.1.200", cmdT=0.008)
    
    # 尝试连接
    if driver.connect():
        print("[TEST] 连接成功！")
        
        # 获取 TCP 位姿
        tcp_pose = driver.get_tcp_pose()
        if tcp_pose:
            print(f"[TEST] 当前TCP位姿: {tcp_pose}")
        
        # 获取关节角度
        joint_pos = driver.get_joint_positions()
        if joint_pos:
            print(f"[TEST] 当前关节角度: {joint_pos}")
        
        # 断开连接
        driver.disconnect()
    else:
        print("[TEST] 连接失败（请检查机器人IP和SDK安装）")
    
    print("测试完成！")
