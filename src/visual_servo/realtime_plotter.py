#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
realtime_plotter.py - 实时数据曲线绘制器
用于监测视觉伺服系统的速度、误差等关键指标
"""

import numpy as np
import matplotlib
# 使用Agg后端避免线程问题，然后手动切换到TkAgg
matplotlib.use('Agg')  # 先使用非GUI后端
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time


class RealtimePlotter:
    """实时曲线绘制器"""
    
    def __init__(self, max_points=300, update_interval=50):
        """
        初始化绘制器
        
        Args:
            max_points: 最大显示点数（时间窗口）
            update_interval: 更新间隔（毫秒）
        """
        self.max_points = max_points
        self.update_interval = update_interval
        
        # 数据缓存（使用deque实现滑动窗口）
        self.time_data = deque(maxlen=max_points)
        self.vx_data = deque(maxlen=max_points)
        self.vy_data = deque(maxlen=max_points)
        self.vz_data = deque(maxlen=max_points)
        self.ex_data = deque(maxlen=max_points)
        self.ey_data = deque(maxlen=max_points)
        self.ez_data = deque(maxlen=max_points)
        self.err_magnitude_data = deque(maxlen=max_points)
        
        # 线程安全
        self.data_lock = threading.Lock()
        
        # 时间基准
        self.start_time = None
        
        # 图形界面
        self.fig = None
        self.ax1 = None
        self.ax2 = None
        self.lines = {}
        self.animation = None
        
        # 线程控制
        self.plot_thread = None
        self.running = False
        
    def add_data(self, vx, vy, vz, ex, ey, ez, timestamp):
        """
        添加新数据点
        
        Args:
            vx, vy, vz: XYZ方向速度（mm/s）
            ex, ey, ez: XYZ方向误差（像素或mm）
            timestamp: 时间戳
        """
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp
                print(f"[Realtime Plot] Start recording data, time: {timestamp:.3f}")
            
            t = timestamp - self.start_time
            self.time_data.append(t)
            self.vx_data.append(vx)
            self.vy_data.append(vy)
            self.vz_data.append(vz)
            self.ex_data.append(ex)
            self.ey_data.append(ey)
            self.ez_data.append(ez)
            
            # 计算误差幅值
            err_mag = np.sqrt(ex**2 + ey**2)
            self.err_magnitude_data.append(err_mag)
            
            # # 每10个点打印一次统计
            # if len(self.time_data) % 10 == 0:
            #     print(f"[实时绘图] 已记录 {len(self.time_data)} 个数据点 | 最新: vx={vx:.1f}, vy={vy:.1f}, vz={vz:.1f} mm/s")
    
    def setup_plot(self):
        """设置图形界面"""
        # 切换到TkAgg后端用于显示
        try:
            matplotlib.use('TkAgg', force=True)
        except:
            pass
        
        # 创建figure和subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # 子图1：速度曲线
        self.ax1.set_title('Flange Velocity (Tool Frame)', fontsize=12, fontweight='bold')
        self.ax1.set_xlabel('Time (s) [0s = Current]')
        self.ax1.set_ylabel('Velocity (mm/s)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axvline(x=0, color='k', linestyle='--', linewidth=1, alpha=0.5)  # Current time marker
        self.lines['vx'], = self.ax1.plot([], [], 'r-', linewidth=2, label='Vx')
        self.lines['vy'], = self.ax1.plot([], [], 'g-', linewidth=2, label='Vy')
        self.lines['vz'], = self.ax1.plot([], [], 'b-', linewidth=2, label='Vz')
        self.ax1.legend(loc='upper right')
        self.ax1.set_xlim(-2.0, 0.2)
        self.ax1.set_ylim(-200, 200)
        
        # 子图2：误差曲线
        self.ax2.set_title('Image Error', fontsize=12, fontweight='bold')
        self.ax2.set_xlabel('Time (s) [0s = Current]')
        self.ax2.set_ylabel('Error')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.axvline(x=0, color='k', linestyle='--', linewidth=1, alpha=0.5)  # Current time marker
        self.lines['ex'], = self.ax2.plot([], [], 'r--', linewidth=1.5, alpha=0.7, label='err_x (px)')
        self.lines['ey'], = self.ax2.plot([], [], 'g--', linewidth=1.5, alpha=0.7, label='err_y (px)')
        self.lines['ez'], = self.ax2.plot([], [], 'b-', linewidth=2, label='err_z (mm)')
        self.lines['err_mag'], = self.ax2.plot([], [], 'm-', linewidth=2.5, label='|err_xy| (px)')
        self.ax2.legend(loc='upper right')
        self.ax2.set_xlim(-2.0, 0.2)
        self.ax2.set_ylim(-150, 150)
        
        # 调整布局
        plt.tight_layout()
        
        # 设置窗口标题
        try:
            self.fig.canvas.manager.set_window_title('Visual Servo Real-time Monitor')
        except:
            pass
        
        # 显示窗口（非阻塞）
        plt.show(block=False)
        plt.pause(0.001)  # 确保窗口显示出来
    
    def update_plot(self, frame):
        """更新图形（动画回调）"""
        with self.data_lock:
            if len(self.time_data) == 0:
                return []
            
            # 转换为numpy数组
            t_abs = np.array(self.time_data)
            vx = np.array(self.vx_data)
            vy = np.array(self.vy_data)
            vz = np.array(self.vz_data)
            ex = np.array(self.ex_data)
            ey = np.array(self.ey_data)
            ez = np.array(self.ez_data)
            err_mag = np.array(self.err_magnitude_data)
            
            # 计算相对当前时刻的时间（0s = 当前时刻）
            current_time = t_abs[-1]
            t_relative = t_abs - current_time
            
            # 只显示最近4秒的数据（-2s到+2s窗口）
            time_window = 2.0  # 秒
            mask = t_relative >= -time_window
            
            t_plot = t_relative[mask]
            vx_plot = vx[mask]
            vy_plot = vy[mask]
            vz_plot = vz[mask]
            ex_plot = ex[mask]
            ey_plot = ey[mask]
            ez_plot = ez[mask]
            err_mag_plot = err_mag[mask]
            
            # 更新速度曲线
            self.lines['vx'].set_data(t_plot, vx_plot)
            self.lines['vy'].set_data(t_plot, vy_plot)
            self.lines['vz'].set_data(t_plot, vz_plot)
            
            # 更新误差曲线
            self.lines['ex'].set_data(t_plot, ex_plot)
            self.lines['ey'].set_data(t_plot, ey_plot)
            self.lines['ez'].set_data(t_plot, ez_plot)
            self.lines['err_mag'].set_data(t_plot, err_mag_plot)
            
            # 固定x轴范围：-2秒到+0.2秒（留一点余量）
            self.ax1.set_xlim(-time_window, 0.2)
            self.ax2.set_xlim(-time_window, 0.2)
            
            # 自动调整y轴范围（基于数据范围）
            if len(vx_plot) > 2:
                v_max = max(abs(vx_plot).max(), abs(vy_plot).max(), abs(vz_plot).max(), 10) * 1.3
                self.ax1.set_ylim(-v_max, v_max)
            
            if len(ex_plot) > 2:
                e_max = max(abs(ex_plot).max(), abs(ey_plot).max(), err_mag_plot.max(), 10) * 1.3
                ez_max = max(abs(ez_plot).max(), 10) * 1.3
                combined_max = max(e_max, ez_max)
                self.ax2.set_ylim(-combined_max, combined_max)
            
            # 强制重绘
            self.ax1.relim()
            self.ax1.autoscale_view(scalex=False, scaley=False)
            self.ax2.relim()
            self.ax2.autoscale_view(scalex=False, scaley=False)
        
        return []
    
    def _plot_thread_func(self):
        """绘图线程函数（在独立线程中运行）"""
        try:
            # 在绘图线程中初始化matplotlib
            self.setup_plot()
            
            # 创建动画（自动更新）
            self.animation = FuncAnimation(
                self.fig,
                self.update_plot,
                interval=self.update_interval,
                blit=False,  # 关闭blit，确保完整更新
                cache_frame_data=False
            )
            
            print("[Realtime Plot] Plot thread started")
            
            # 保持matplotlib事件循环运行
            while self.running:
                plt.pause(0.01)  # 使用小间隔的pause代替show()阻塞
            
        except Exception as e:
            print(f"[Realtime Plot] Plot thread exception: {e}")
        finally:
            self.running = False
            # 清理资源
            try:
                if self.animation:
                    self.animation.event_source.stop()
                if self.fig:
                    plt.close(self.fig)
            except:
                pass
            print("[Realtime Plot] Plot thread exited")
    
    def start(self):
        """启动实时绘制（在独立线程中）"""
        if self.running:
            print("[Realtime Plot] Already running")
            return
        
        self.running = True
        
        # 创建并启动绘图线程
        self.plot_thread = threading.Thread(target=self._plot_thread_func, daemon=True)
        self.plot_thread.start()
        
        # 等待一小段时间确保窗口创建
        time.sleep(0.5)
        
        print("[Realtime Plot] Started (separate thread)")
        print("[Tip] Press 's' to start servo, curves will begin plotting")
    
    def stop(self):
        """停止绘制"""
        if not self.running:
            return
        
        self.running = False
        
        # 关闭matplotlib窗口
        try:
            if self.animation:
                self.animation.event_source.stop()
        except:
            pass
        
        try:
            if self.fig:
                plt.close(self.fig)
        except:
            pass
        
        # 等待绘图线程结束
        if self.plot_thread and self.plot_thread.is_alive():
            self.plot_thread.join(timeout=1.0)
        
        # 强制清理matplotlib资源
        try:
            plt.close('all')
        except:
            pass
        
        print("[Realtime Plot] Stopped")
