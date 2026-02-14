# AirExo2 坐标变换逻辑总结

## 概述

本文档整理了 AirExo2 项目中机械臂渲染的完整坐标变换逻辑，主要基于 `reconstruct_two_arm.ipynb` 中"完全照抄原仓库renderer的变换逻辑"cell 的正确实现，并结合 `airexo/helpers/renderer.py`、`airexo/helpers/visualizer.py`、`airexo/calibration/calib_info.py` 和 `airexo/helpers/constants.py` 等核心代码。

---

## 核心变换矩阵定义

### 1. O3D_RENDER_TRANSFORMATION
```python
# 位置: airexo/helpers/constants.py L120-127
O3D_RENDER_TRANSFORMATION = np.array([
    [1, 0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, -1, 0],
    [0, 0, 0, 1]
], dtype = np.float32)
```
**作用**: 将 Open3D 的右手坐标系转换为适合渲染的坐标系，翻转 Y 轴和 Z 轴。

### 2. ROBOT_PREDEFINED_TRANSFORMATION
```python
# 位置: airexo/helpers/constants.py L91-96
ROBOT_PREDEFINED_TRANSFORMATION = np.array([
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
], dtype = np.float32)
```
**作用**: URDF 基座到真实机器人基座的变换（坐标轴重排）。

### 3. LEFT_ROBOT_PREDEFINED_TRANSFORMATION
```python
# 位置: airexo/helpers/constants.py L98-103
LEFT_ROBOT_PREDEFINED_TRANSFORMATION = np.array([
    [1, 0, 0, 0],
    [0, np.sqrt(2) / 2, np.sqrt(2) / 2, 0],
    [0, -np.sqrt(2) / 2, np.sqrt(2) / 2, 0],
    [0, 0, 0, 1]
], dtype = np.float32)
```
**作用**: 左臂 URDF 基座到真实左臂基座的变换（45°旋转）。

### 4. RIGHT_ROBOT_PREDEFINED_TRANSFORMATION
```python
# 位置: airexo/helpers/constants.py L105-110
RIGHT_ROBOT_PREDEFINED_TRANSFORMATION = np.array([
    [1, 0, 0, 0],
    [0, np.sqrt(2) / 2, -np.sqrt(2) / 2, 0],
    [0, np.sqrt(2) / 2, np.sqrt(2) / 2, 0],
    [0, 0, 0, 1]
], dtype = np.float32)
```
**作用**: 右臂 URDF 基座到真实右臂基座的变换（45°旋转）。

### 5. AIREXO_PREDEFINED_TRANSFORMATION
```python
# 位置: airexo/helpers/constants.py L112-117
AIREXO_PREDEFINED_TRANSFORMATION = np.array([
    [0, 1, 0, 0],
    [1, 0, 0, 0],
    [0, 0, -1, 0],
    [0, 0, 0, 1]
], dtype = np.float32)
```
**作用**: AirExo URDF 基座到真实 AirExo 基座的变换。

---

## 坐标系层级关系

```
┌─────────────────────────────────────────────────────────────────┐
│                    Open3D 渲染空间                                │
│                  (O3D_RENDER_TRANSFORMATION)                     │
└─────────────────────────────────────────────────────────────────┘
                              ↑
┌─────────────────────────────────────────────────────────────────┐
│                    相机坐标系                                      │
│                  (cam_to_base)                                    │
└─────────────────────────────────────────────────────────────────┘
                              ↑
┌─────────────────────────────────────────────────────────────────┐
│              真实机器人基座 (Real Robot Base)                     │
│            (ROBOT_PREDEFINED_TRANSFORMATION)                     │
└─────────────────────────────────────────────────────────────────┘
                              ↑
┌─────────────────────────────────────────────────────────────────┐
│                  URDF 基座 (URDF Base)                            │
│                  (前向运动学计算结果)                              │
└─────────────────────────────────────────────────────────────────┘
                              ↑
┌─────────────────────────────────────────────────────────────────┐
│                    Link 坐标系                                     │
│                  (transform.matrix())                             │
└─────────────────────────────────────────────────────────────────┘
                              ↑
┌─────────────────────────────────────────────────────────────────┐
│                  Mesh 坐标系 (v.offset.matrix())                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 完整变换链（单机械臂）

### 机械臂 Mesh 变换公式

```python
# 位置: airexo/helpers/renderer.py L207, L241
# 位置: airexo/helpers/visualizer.py L931, L991
tf = O3D_RENDER_TRANSFORMATION @ cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
```

**变换链详解（从右到左应用）**:

| 变换 | 含义 | 代码位置 |
|------|------|----------|
| `v.offset.matrix()` | Mesh 相对于 Link 的偏移变换 | URDF visual 标签 |
| `transform.matrix()` | Link 相对于 URDF 基座的变换（来自前向运动学） | [`forward_kinematic()`](airexo/helpers/urdf_robot.py:112) |
| `ROBOT_PREDEFINED_TRANSFORMATION` | URDF 基座到真实机器人基座的变换 | [`constants.py:91`](airexo/helpers/constants.py:91) |
| `cam_to_base` | 相机到 URDF 基座的变换（来自标定） | [`calib_info.py:81`](airexo/calibration/calib_info.py:81) |
| `O3D_RENDER_TRANSFORMATION` | 真实基座到 Open3D 渲染空间的变换 | [`constants.py:120`](airexo/helpers/constants.py:120) |

**完整变换链**:
```
Mesh → Link → URDF基座 → 真实机器人基座 → 相机坐标系 → Open3D渲染空间
```

### 点云变换公式

```python
# 位置: reconstruct_two_arm.ipynb L388
pcd_cam.transform(O3D_RENDER_TRANSFORMATION)
```

**变换链**:
```
相机坐标系 → O3D_RENDER_TRANSFORMATION → Open3D渲染空间
```

---

## cam_to_base 的获取逻辑

### get_camera_to_base 函数

```python
# 位置: airexo/calibration/calib_info.py L81-96
def get_camera_to_base(self, serial, real_base = False):
    if self.calib_type == "airexo":
        return self.extrinsics[serial] @ np.linalg.inv(AIREXO_BASE_TO_MARKER)
    elif self.calib_type == "airexo_upd":
        if serial == self.upd_serial:
            return self.upd_camera_to_base
        else:
            return self.extrinsics[serial] @ np.linalg.inv(self.extrinsics[self.upd_serial]) @ self.upd_camera_to_base
    else:  # robot
        left_cam_to_base = self.get_camera_to_robot_left_base(serial, real_base = True) @ ROBOT_LEFT_REAL_BASE_TO_REAL_BASE
        right_cam_to_base = self.get_camera_to_robot_right_base(serial, real_base = True) @ ROBOT_RIGHT_REAL_BASE_TO_REAL_BASE
        cam_to_base = average_xyz_rot_quat(left_cam_to_base, right_cam_to_base, rotation_rep = "matrix")
        if not real_base:
            cam_to_base = cam_to_base @ np.linalg.inv(ROBOT_PREDEFINED_TRANSFORMATION)
        return cam_to_base
```

**关键点**:
- 当 `real_base=False`（默认值）时，返回的是相机到 **URDF 基座** 的变换
- 当 `real_base=True` 时，返回的是相机到 **真实机器人基座** 的变换
- `cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION` = 相机到真实基座

---

## 双机械臂变换链（SeparateRobotRenderer）

### 左臂 Mesh 变换公式

```python
# 位置: airexo/helpers/renderer.py L337, L390
tf = O3D_RENDER_TRANSFORMATION @ cam_to_left_base @ ROBOT_PREDEFINED_TRANSFORMATION @ LEFT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
```

**完整变换链**:
```
Mesh → Link → 左臂URDF基座 → 真实左臂基座 → 真实机器人基座 → 相机坐标系 → Open3D渲染空间
```

### 右臂 Mesh 变换公式

```python
# 位置: airexo/helpers/renderer.py L351, L400
tf = O3D_RENDER_TRANSFORMATION @ cam_to_right_base @ ROBOT_PREDEFINED_TRANSFORMATION @ RIGHT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
```

**完整变换链**:
```
Mesh → Link → 右臂URDF基座 → 真实右臂基座 → 真实机器人基座 → 相机坐标系 → Open3D渲染空间
```

---

## AirExo 变换链

### AirExo Mesh 变换公式

```python
# 位置: airexo/helpers/renderer.py L80, L116
tf = O3D_RENDER_TRANSFORMATION @ cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
```

**完整变换链**:
```
Mesh → Link → AirExo URDF基座 → 真实AirExo基座 → 相机坐标系 → Open3D渲染空间
```

---

## 前向运动学

### forward_kinematic 函数

```python
# 位置: airexo/helpers/urdf_robot.py L112-154
def forward_kinematic(
    left_joint, 
    right_joint,
    left_joint_cfgs,
    right_joint_cfgs,
    is_rad = True,
    urdf_file = os.path.join("airexo", "urdf_models", "robot", "robot.urdf"),
    with_visuals_map = True,
    **kwargs
):
    model_chain = kp.build_chain_from_urdf(open(urdf_file).read().encode('utf-8'))
    joint_states = convert_joint_states(
        left_joint = left_joint,
        right_joint = right_joint,
        left_joint_cfgs = left_joint_cfgs,
        right_joint_cfgs = right_joint_cfgs,
        is_rad = is_rad,
        seperate = False,
        **kwargs
    )
    if with_visuals_map:
        return model_chain.forward_kinematics(joint_states), model_chain.visuals_map()
    else:
        return model_chain.forward_kinematics(joint_states)
```

**返回值**:
- `cur_transforms`: dict, key=link名称, value=该link相对于URDF基座的变换矩阵
- `visuals_map`: dict, key=link名称, value=该link的visual信息列表

---

## 完整代码示例（reconstruct_two_arm.ipynb）

```python
# ========== 1. 读取关节数据 ==========
with h5py.File(f"{lowdim_path}/robot_left.h5", 'r') as f:
    left_joint = f['joint_pos'][0]
with h5py.File(f"{lowdim_path}/robot_right.h5", 'r') as f:
    right_joint = f['joint_pos'][0]
with h5py.File(f"{lowdim_path}/gripper_left.h5", 'r') as f:
    left_gripper = f['width'][0] if 'width' in f else 0.05
with h5py.File(f"{lowdim_path}/gripper_right.h5", 'r') as f:
    right_gripper = f['width'][0] if 'width' in f else 0.05

left_joint = np.concatenate([left_joint, [left_gripper]])
right_joint = np.concatenate([right_joint, [right_gripper]])

# ========== 2. 加载标定数据 ==========
calib_info = CalibrationInfo(
    calib_path=calib_path,
    calib_timestamp=1737548651048
)
camera_serial = list(calib_info.camera_serials_global)[0]
cam_to_base = calib_info.get_camera_to_base(camera_serial)
intrinsic = calib_info.get_intrinsic(camera_serial)

# ========== 3. 前向运动学 ==========
left_cfg = OmegaConf.load("airexo/configs/joint/left/robot.yaml")
right_cfg = OmegaConf.load("airexo/configs/joint/right/robot.yaml")

cur_transforms, visuals_map = forward_kinematic(
    left_joint=left_joint,
    right_joint=right_joint,
    left_joint_cfgs=left_cfg,
    right_joint_cfgs=right_cfg,
    is_rad=True,
    urdf_file="airexo/urdf_models/robot/robot_inhand.urdf",
    with_visuals_map=True
)

# ========== 4. 读取深度图并转换为点云 ==========
depth_img = o3d.io.read_image(depth_path)
rgb_img = o3d.io.read_image(rgb_path)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
    rgb_img, depth_img,
    depth_scale=1000.0,
    convert_rgb_to_intensity=False
)

intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
    width=int(w), height=int(h),
    fx=float(fx), fy=float(fy),
    cx=float(cx), cy=float(cy)
)

# 从RGBD图像创建点云，此时点云在相机坐标系中
pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic_o3d)

# 将点云从相机坐标系变换到Open3D渲染空间
pcd_cam.transform(O3D_RENDER_TRANSFORMATION)

# ========== 5. 渲染机械臂 ==========
for link, transform in cur_transforms.items():
    if link not in visuals_map: continue
    
    for v in visuals_map[link]:
        if v.geom_param is None: continue
        
        mesh_path = os.path.join("airexo/urdf_models/robot", v.geom_param)
        if not os.path.exists(mesh_path):
            continue

        mesh = trimesh.load(mesh_path, force='mesh')
        
        # ✅ 完全照抄官方公式
        tf = O3D_RENDER_TRANSFORMATION @ cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
        mesh.apply_transform(tf)
        
        plot += k3d.mesh(mesh.vertices.astype(np.float32), 
                         mesh.faces.astype(np.uint32),
                         color=0xaaaaaa)

# ========== 6. 渲染点云 ==========
pcd_points = np.asarray(pcd_cam.points).astype(np.float32)
pcd_colors = np.asarray(pcd_cam.colors).astype(np.float32)
pcd_colors_uint32 = (pcd_colors * 255).astype(np.uint32)
pcd_colors_packed = (pcd_colors_uint32[:, 0] << 16) | \
                    (pcd_colors_uint32[:, 1] << 8) | \
                     pcd_colors_uint32[:, 2]

plot += k3d.points(pcd_points, 
                   colors=pcd_colors_packed,
                   point_size=0.002,
                   shader='flat')
```

---

## 关键代码位置索引

| 功能 | 文件 | 行号 |
|------|------|------|
| RobotRenderer 初始化变换 | [`airexo/helpers/renderer.py`](airexo/helpers/renderer.py:207) | L207 |
| RobotRenderer 更新变换 | [`airexo/helpers/renderer.py`](airexo/helpers/renderer.py:241) | L241 |
| SeparateRobotRenderer 左臂变换 | [`airexo/helpers/renderer.py`](airexo/helpers/renderer.py:337) | L337, L390 |
| SeparateRobotRenderer 右臂变换 | [`airexo/helpers/renderer.py`](airexo/helpers/renderer.py:351) | L351, L400 |
| AirExoRenderer 变换 | [`airexo/helpers/renderer.py`](airexo/helpers/renderer.py:80) | L80, L116 |
| RobotVisualizer 初始变换 | [`airexo/helpers/visualizer.py`](airexo/helpers/visualizer.py:931) | L931 |
| RobotVisualizer 更新变换 | [`airexo/helpers/visualizer.py`](airexo/helpers/visualizer.py:991) | L991 |
| get_camera_to_base | [`airexo/calibration/calib_info.py`](airexo/calibration/calib_info.py:81) | L81-96 |
| forward_kinematic | [`airexo/helpers/urdf_robot.py`](airexo/helpers/urdf_robot.py:112) | L112-154 |
| ROBOT_PREDEFINED_TRANSFORMATION | [`airexo/helpers/constants.py`](airexo/helpers/constants.py:91) | L91-96 |
| O3D_RENDER_TRANSFORMATION | [`airexo/helpers/constants.py`](airexo/helpers/constants.py:120) | L120-127 |

---

## 注意事项

1. **变换顺序**: 矩阵乘法从右到左应用，注意变换链的顺序
2. **real_base 参数**: `get_camera_to_base(serial, real_base=False)` 默认返回到 URDF 基座的变换
3. **点云变换**: 点云只需要应用 `O3D_RENDER_TRANSFORMATION`，不需要 `cam_to_base`
4. **TCP 变换**: TCP 坐标也需要应用 `ROBOT_PREDEFINED_TRANSFORMATION`（见 [`visualizer.py:951`](airexo/helpers/visualizer.py:951)）
5. **双机械臂**: 使用 `SeparateRobotRenderer` 时，左右臂分别使用 `cam_to_left_base` 和 `cam_to_right_base`

---

## 变换链图示

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Open3D 渲染空间                                      │
│                    (O3D_RENDER_TRANSFORMATION)                               │
└─────────────────────────────────────────────────────────────────────────────┘
                                    ↑
┌─────────────────────────────────────────────────────────────────────────────┐
│                              相机坐标系                                        │
│                            (cam_to_base)                                      │
└─────────────────────────────────────────────────────────────────────────────┘
                                    ↑
┌─────────────────────────────────────────────────────────────────────────────┐
│                        真实机器人基座 (Real Base)                             │
│                  (ROBOT_PREDEFINED_TRANSFORMATION)                           │
└─────────────────────────────────────────────────────────────────────────────┘
                                    ↑
┌─────────────────────────────────────────────────────────────────────────────┐
│                          URDF 基座 (URDF Base)                                │
│                      (forward_kinematic 结果)                                 │
└─────────────────────────────────────────────────────────────────────────────┘
                                    ↑
┌─────────────────────────────────────────────────────────────────────────────┐
│                            Link 坐标系                                        │
│                          (transform.matrix())                                 │
└─────────────────────────────────────────────────────────────────────────────┘
                                    ↑
┌─────────────────────────────────────────────────────────────────────────────┐
│                        Mesh 坐标系 (v.offset.matrix())                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 总结

AirExo2 的坐标变换逻辑遵循以下核心公式：

```python
# 单机械臂
tf = O3D_RENDER_TRANSFORMATION @ cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()

# 双机械臂 - 左臂
tf = O3D_RENDER_TRANSFORMATION @ cam_to_left_base @ ROBOT_PREDEFINED_TRANSFORMATION @ LEFT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()

# 双机械臂 - 右臂
tf = O3D_RENDER_TRANSFORMATION @ cam_to_right_base @ ROBOT_PREDEFINED_TRANSFORMATION @ RIGHT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()

# AirExo
tf = O3D_RENDER_TRANSFORMATION @ cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()

# 点云
pcd.transform(O3D_RENDER_TRANSFORMATION)
```

这些变换确保了机械臂模型和点云在 Open3D 渲染空间中的正确对齐。
