#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_plotter.py - 测试实时绘图器
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import time
import numpy as np
from visual_servo.realtime_plotter import RealtimePlotter

def main():
    print("测试实时绘图器...")
    print("将显示模拟的速度和误差曲线")
    print("按 Ctrl+C 退出")
    
    # 创建绘图器
    plotter = RealtimePlotter(max_points=200, update_interval=50)
    plotter.start()
    
    # 模拟数据
    t = 0
    try:
        while True:
            # 生成模拟数据（模拟收敛过程）
            decay = np.exp(-t / 5.0)  # 指数衰减
            
            vx = 100 * decay * np.sin(t * 2)
            vy = 80 * decay * np.cos(t * 1.5)
            vz = 50 * decay * np.sin(t * 0.8)
            
            ex = 50 * decay * np.sin(t)
            ey = 40 * decay * np.cos(t * 1.2)
            ez = 30 * decay * np.sin(t * 0.5)
            
            # 添加数据
            plotter.add_data(vx, vy, vz, ex, ey, ez, time.time())
            
            t += 0.05
            time.sleep(0.05)  # 20Hz
            
    except KeyboardInterrupt:
        print("\n停止测试...")
        plotter.stop()
        print("完成！")

if __name__ == "__main__":
    main()
