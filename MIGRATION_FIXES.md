# 项目结构迁移 - 快速修复指南

## 已修复的问题

### 1. Import路径问题 ✅
**问题**：`No module named 'utils_math'`

**原因**：项目重构后，部分文件内部的import路径未更新

**已修复文件**：
- `src/visual_servo/controller.py` - 改用相对导入 `.utils_math`
- `src/main_vs.py` - 两处动态import改为 `visual_servo.utils_math`

### 2. Matplotlib线程问题 ✅
**问题**：退出时出现 `RuntimeError: main thread is not in main loop`

**原因**：matplotlib的GUI必须在主线程运行，但我们在daemon线程中运行

**解决方案**：
- 改用Agg后端初始化，运行时切换到TkAgg
- 增强异常处理，确保资源正确清理
- 添加 `plt.close('all')` 强制清理

## 使用新结构

### 运行系统
```bash
python3 run_visual_servo.py
```

### 如果遇到import错误
确保从项目根目录运行，启动脚本会自动添加src到Python路径。

## 文件变更摘要
- ✅ 所有核心模块移至 `src/visual_servo/`
- ✅ 主程序在 `src/main_vs.py`
- ✅ 配置文件在 `config/config.yaml`
- ✅ 快速启动 `run_visual_servo.py`
- ✅ 所有import路径已更新
- ✅ 线程安全问题已修复
