# FAIRINO FR5 手眼标定步骤指南（Eye-in-Hand）

## 目标
标定**法兰坐标系（Flange）到相机坐标系（Camera）**的变换矩阵 `T_flange_cam`

---

## 准备工作

### 1. 打印ArUco Tag
- 字典：DICT_4X4_50
- ID：0（或其他）
- 尺寸：精确测量并记录（例如：50mm）
- 打印质量：清晰、无反光、平整

### 2. 固定ArUco Tag
- 将Tag固定在稳定平面上（桌面、墙壁等）
- 确保Tag在整个标定过程中不会移动
- 光照均匀，避免阴影和反光

### 3. 检查机器人与相机
- 机器人能正常连接（192.168.1.200）
- 相机能正常工作（`realsense-viewer`测试）
- 机器人工作空间内能看到Tag

---

## 标定命令

```bash
cd /home/lh/yuntai/d435i_fr5

python3 handeye_calibration_eye_in_hand.py \
  --robot-ip 192.168.1.200 \
  --dictionary DICT_4X4_50 \
  --target-id 0 \
  --marker-length-mm 50 \
  --use-flange \
  --min-samples 15
```

**关键参数说明：**
- `--use-flange`：使用法兰坐标系（重要！）
- `--marker-length-mm 50`：Tag实际尺寸（毫米）
- `--min-samples 15`：采集15组数据（建议15-20组）

---

## 采样策略（重要！）

采样质量直接影响标定精度，请按以下原则操作：

### 原则1：姿态多样性
手动示教机器人到不同位置，确保：
- ✅ **距离变化**：Tag距离相机 300mm ~ 600mm
- ✅ **角度变化**：绕X、Y、Z轴旋转（±30°~±60°）
- ✅ **位置变化**：Tag在图像不同区域（左、右、上、下、中心）
- ❌ **避免**：所有姿态都很相似

### 原则2：Tag完全可见
每个位置必须满足：
- ✅ Tag完全在相机视野内
- ✅ Tag四个角点清晰可见
- ✅ 图像清晰，无模糊
- ❌ **避免**：Tag被遮挡、超出视野、过近或过远

### 原则3：稳定后采集
- 移动到位置后，等待1-2秒让机器人稳定
- 观察图像中Tag检测正常（绿色边框）
- 按空格键采集

---

## 操作流程

### 启动标定程序
```bash
python3 handeye_calibration_eye_in_hand.py --robot-ip 192.168.1.200 \
  --dictionary DICT_4X4_50 --target-id 0 --marker-length-mm 50 --use-flange
```

### 采样循环（15次）
对每个样本：
1. **手动示教**机器人到新位置
2. 确认Tag在图像中完全可见且清晰
3. 观察终端显示检测成功
4. **按空格键**保存当前样本
5. 重复直到采集15个样本

### 建议的15个采样位置

| 样本 | 距离 | 角度 | 位置 | 描述 |
|------|------|------|------|------|
| 1 | 400mm | 0° | 中心 | 正对Tag，基准位置 |
| 2 | 400mm | X+30° | 中心 | 绕X轴旋转 |
| 3 | 400mm | X-30° | 中心 | 绕X轴反向 |
| 4 | 400mm | Y+30° | 中心 | 绕Y轴旋转 |
| 5 | 400mm | Y-30° | 中心 | 绕Y轴反向 |
| 6 | 400mm | Z+30° | 中心 | 绕Z轴旋转 |
| 7 | 300mm | 0° | 左侧 | 近距离 |
| 8 | 500mm | 0° | 右侧 | 远距离 |
| 9 | 400mm | X+20°,Y+20° | 左上 | 组合旋转 |
| 10 | 400mm | X-20°,Y-20° | 右下 | 组合旋转 |
| 11 | 350mm | X+40° | 上方 | 大角度 |
| 12 | 450mm | Y+40° | 下方 | 大角度 |
| 13 | 400mm | X+15°,Z+30° | 中心 | 组合旋转 |
| 14 | 400mm | Y-15°,Z-30° | 中心 | 组合旋转 |
| 15 | 400mm | X+30°,Y-20° | 左下 | 综合位置 |

**注意**：以上只是参考，实际可根据机器人工作空间灵活调整

---

## 标定完成

### 查看结果
标定完成后会显示：
```
Calibration result saved to:
  handeye_logs/samples_YYYYMMDD_HHMMSS.jsonl
  handeye_samples.npz
  handeye_result.txt (4x4变换矩阵)
```

### 检查标定质量
```bash
# 查看4x4变换矩阵
cat handeye_result.txt

# 查看.npz文件内容
python3 -c "import numpy as np; data=np.load('handeye_samples.npz'); print(data.files); print(f'采集样本数: {data['r_gb'].shape[0]}')"
```

### 标定结果格式
`handeye_result.txt` 包含4x4齐次变换矩阵：
```
R11  R12  R13  tx
R21  R22  R23  ty
R31  R32  R33  tz
0    0    0    1
```
- 左上角3x3：旋转矩阵
- 右上角3x1：平移向量（米）

---

## 标定质量评估

### 好的标定特征：
✅ 重投影误差 < 2 像素
✅ 姿态多样性好（覆盖不同角度和距离）
✅ Tag检测稳定（所有样本都成功检测）

### 需要重新标定：
❌ 重投影误差 > 5 像素
❌ 姿态过于集中（都在相似位置）
❌ 部分样本Tag检测失败

---

## 常见问题

### Q1: Tag检测不到
- 检查光照（避免过曝、阴影）
- 检查Tag打印质量
- 调整距离（300-600mm）
- 确认字典匹配（DICT_4X4_50）

### Q2: 采样时机器人位置跳变
- 示教模式下移动机器人
- 移动到位后等待稳定
- 观察图像稳定后再按空格

### Q3: 标定精度不高
- 增加采样数量（20-25个）
- 提高姿态多样性
- 确保Tag在图像中完全可见
- 重新精确测量Tag尺寸

### Q4: 程序报错
```bash
# 检查机器人连接
ping 192.168.1.200

# 检查相机
realsense-viewer

# 检查Python依赖
pip install -r requirements.txt
```

---

## 下一步

标定完成后：
1. 将 `handeye_result.txt` 中的矩阵填入 `config.yaml`
2. 运行 `python3 convert_tool_frame.py` 转换为机器人格式
3. 在示教器网页设置工具坐标系
4. 运行视觉伺服测试

---

## 完整命令速查

```bash
# 1. 运行标定（使用法兰坐标系）
python3 handeye_calibration_eye_in_hand.py \
  --robot-ip 192.168.1.200 \
  --dictionary DICT_4X4_50 \
  --target-id 0 \
  --marker-length-mm 50 \
  --use-flange \
  --min-samples 15

# 2. 查看标定结果
cat handeye_result.txt

# 3. 转换为机器人工具坐标系格式
python3 convert_tool_frame.py

# 4. 查看转换结果
cat tool_frame_params.txt
```

---

**标定日期：** _____________  
**操作人员：** _____________  
**Tag尺寸：** _____________mm  
**样本数量：** _____________  
**标定质量：** ☐优秀 ☐良好 ☐需改进
