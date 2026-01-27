#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_visual_servo.py - 视觉伺服系统启动脚本
"""

import sys
import os

# 添加src目录到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# 导入并运行主程序
from main_vs import main

if __name__ == "__main__":
    main()
