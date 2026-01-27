# 项目结构整理说明

## 2026-01-27 更新

项目已重新组织为标准的Python项目结构。

### 目录结构

```
├── src/                  # 源代码
│   ├── visual_servo/    # 核心模块包
│   └── main_vs.py       # 主程序入口
├── config/              # 配置文件
├── data/                # 数据文件（标定结果等）
├── scripts/             # 工具脚本
├── tests/               # 测试文件
├── docs/                # 文档
├── logs/                # 日志（运行时生成）
└── run_visual_servo.py  # 快速启动脚本
```

### 使用方法

**旧的运行方式：**
```bash
python3 main_vs.py
```

**新的运行方式（推荐）：**
```bash
python3 run_visual_servo.py
```

或指定配置文件：
```bash
python3 run_visual_servo.py --config config/config.yaml
```

### 导入变化

**模块内部**：相对导入
```python
from .utils_math import *
from .controller import *
```

**主程序**：绝对导入
```python
from visual_servo.controller import VisualServoController
from visual_servo.realtime_plotter import RealtimePlotter
```

### 文件映射

| 旧位置 | 新位置 |
|--------|--------|
| `main_vs.py` | `src/main_vs.py` |
| `controller.py` | `src/visual_servo/controller.py` |
| `aruco_detector.py` | `src/visual_servo/aruco_detector.py` |
| `fr5_driver.py` | `src/visual_servo/fr5_driver.py` |
| `realtime_plotter.py` | `src/visual_servo/realtime_plotter.py` |
| `utils_math.py` | `src/visual_servo/utils_math.py` |
| `handeye_io.py` | `src/visual_servo/handeye_io.py` |
| `config.yaml` | `config/config.yaml` |
| `test_plotter.py` | `tests/test_plotter.py` |
| `handeye_calibration_eye_in_hand.py` | `scripts/handeye_calibration_eye_in_hand.py` |
| `README.md` 等 | `docs/README.md` 等 |

### Git 操作

```bash
# 添加所有更改
git add .

# 提交
git commit -m "重构: 整理项目结构为标准Python包布局"

# 推送
git push
```
