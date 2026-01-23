# Roll方向震荡问题分析与解决

## 问题现象

从日志可以看到roll角度在±180°边界附近剧烈震荡：

```
roll=+178.07° → roll=-177.98° → roll=+179.98° → roll=-177.78° → roll=+178.98° ...
```

每次角度变化后，控制命令的方向也随之改变：
- 当检测到roll=-177.98°时，命令为drx=-0.100°（向负方向）
- 当检测到roll=+178.98°时，命令为drx=+0.100°（向正方向）

## 根本原因

### 1. ±180°边界的角度跳变

**物理事实**：roll=-178°和roll=+178°实际上只相差**4°**（非常接近）

**问题**：ArUco检测或欧拉角提取在±180°边界可能产生±360°的跳变显示，但这是正常的数学表示，关键是**误差计算要使用最短角度差**。

### 2. 误差计算错误（已修复）

**错误的计算**：
```python
roll_err = roll_raw - roll_des  # roll_raw=-177.98°, roll_des=0°
# 结果: roll_err = -177.98°  ❌ 错误！
```

**正确的计算**：
```python
roll_err = normalize_angle(roll_raw - roll_des)  # 使用最短角度差
# 当roll_raw=-177.98°时，normalize_angle(-177.98° - 0°) = +2.02° ✓ 正确！
```

从-177.98°到0°，最短路径是**顺时针+2.02°**，而不是逆时针-177.98°。

### 3. 低通滤波累积角度超出范围（已修复）

**问题**：低通滤波器对角度误差进行加权平均：
```python
lpf_error = alpha * new_error + (1-alpha) * old_error
```

如果`old_error=+175°`，`new_error=-175°`（实际相差10°），滤波后可能得到接近0°或超出±180°的值。

**解决**：在滤波后对角度重新归一化：
```python
self.lpf_error[5] = normalize_angle(self.lpf_error[5])  # roll归一化
```

### 4. 死区处理前的角度值异常

日志中出现的奇怪数值：
```
死区处理: 前= +44.48° -> 后= +44.48°
死区处理: 前= +85.13° -> 后= +85.13°
```

这是因为低通滤波累积了未归一化的角度，导致超出±180°范围。修复滤波后归一化可以解决。

## 代码修复

### 修复点1: 确保误差计算使用最短角度差

```python
# controller.py, line ~155
roll_err = normalize_angle(roll_raw - roll_des)  # ✓ 已经正确
```

这一点代码本身是对的，但需要确认`normalize_angle`函数正确。

### 修复点2: 低通滤波后重新归一化

```python
# controller.py, line ~175
if self.enable_lpf:
    raw_error = np.array([ex, ey, ez, yaw_err, pitch_err, roll_err])
    if self.lpf_error is None:
        self.lpf_error = raw_error
    else:
        self.lpf_error = lpf_update(self.lpf_error, raw_error, self.lpf_alpha)
        # ⭐ 关键修复：重新归一化角度误差
        self.lpf_error[3] = normalize_angle(self.lpf_error[3])  # yaw
        self.lpf_error[4] = normalize_angle(self.lpf_error[4])  # pitch
        self.lpf_error[5] = normalize_angle(self.lpf_error[5])  # roll
    ex, ey, ez, yaw_err, pitch_err, roll_err = self.lpf_error
```

## 验证方法

重新运行程序后，观察以下日志：

### 1. 原始角度显示
```
[ROLL_DEBUG] 原始角度: roll=-177.98° | 误差(归一化后): roll_err=+2.02°
```
✓ 误差应该是小角度（±10°以内），而不是170-180°

### 2. 低通滤波后
```
[ROLL_DEBUG] 低通滤波后(归一化): roll_err=+1.85°
```
✓ 滤波后角度仍在合理范围

### 3. 死区处理
```
[ROLL_DEBUG] 死区处理: 前= +1.85° -> 后= +0.00° (死区=30.0°)
```
✓ 小于30°被过滤为0

### 4. 控制命令稳定
```
[ROLL_STATUS] 当前状态: roll=+178.0° | 最后命令: drx= +0.000° | 控制增益: k_roll=-0.5
```
✓ 角度接近180°时，命令应该趋于0，而不是来回跳变

## 其他可能需要的优化

如果修复后仍然震荡，可以尝试：

### 1. 增大Roll死区（最简单有效）
```yaml
# config.yaml
controller:
  deadband_roll_deg: 50.0  # 从30增加到50
```

### 2. 降低Roll增益
```yaml
controller:
  k_roll: -0.3  # 从-0.5降低到-0.3
```

### 3. 增强低通滤波
```yaml
controller:
  lpf_alpha: 0.2  # 从0.3降低到0.2（更平滑，但响应变慢）
```

### 4. 限制Roll旋转速度
```yaml
controller:
  max_rot_deg_per_tick: 0.05  # 从0.1降低到0.05
```

### 5. 使用专用的Roll角度滤波器

在`controller.py`中为roll单独使用更强的滤波：

```python
# 在__init__中添加
self.roll_lpf_alpha = 0.1  # 比其他轴更强的滤波

# 在compute_twist_from_detection中
if self.enable_roll and rvec is not None:
    roll_raw = extract_roll_from_rvec(rvec)
    roll_err = normalize_angle(roll_raw - roll_des)
    
    # Roll专用低通滤波
    if self.last_roll_err is None:
        self.last_roll_err = roll_err
    else:
        roll_err = self.roll_lpf_alpha * roll_err + (1 - self.roll_lpf_alpha) * self.last_roll_err
        roll_err = normalize_angle(roll_err)  # 滤波后再归一化
        self.last_roll_err = roll_err
```

## 预期效果

修复后，应该看到：

1. **Roll角度稳定在180°附近**（±2°小幅波动）
2. **误差值为小角度**（±5°以内）
3. **控制命令趋于0**（在死区内）
4. **不再出现正负跳变**

如果角度确实需要从+180°转到0°（或-180°），控制器会：
- 识别最短路径（例如+180° → -179° → -178° ... → 0°）
- 平滑地旋转，而不是跳变

## 物理约束考虑

如果修复后roll仍然难以收敛，可能是**物理限制**：

1. **机械耦合**：roll旋转会影响相机视角，导致ArUco检测精度下降
2. **奇异点**：某些姿态下roll与其他轴耦合严重
3. **刚性不足**：末端执行器绕自身轴旋转的机械刚性较低

此时建议：
- **临时禁用roll控制**，只控制xyz + yaw/pitch
- **手动调整初始姿态**，避开±180°边界（例如初始roll=0°）
- **增加机械刚性**或使用更稳定的相机安装方式

## 总结

**核心问题**：低通滤波累积角度时未归一化，导致角度超出±180°范围，产生错误的控制命令。

**解决方案**：在低通滤波后对角度误差重新归一化到[-π, π]。

**修复位置**：`controller.py`第175行左右，`lpf_update`之后添加`normalize_angle`。

现在重新运行程序测试！
