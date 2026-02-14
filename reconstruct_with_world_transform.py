"""
使用 FLIPPING_v3 的标定数据，通过 world-base 变换矩阵转换为 AirExo-2 坐标系
实现点云对齐可视化

FLIPPING_v3 的 pose_in_link 是 T_world_cam (world → camera)
通过 T_world_base 转换为 T_cam_base (camera → robot_base)
然后使用 AirExo-2 的渲染逻辑进行对齐
"""

import h5py
import numpy as np
import k3d
import trimesh
import os
from omegaconf import OmegaConf
from airexo.helpers.urdf_robot import forward_kinematic
from airexo.helpers.constants import ROBOT_PREDEFINED_TRANSFORMATION, O3D_RENDER_TRANSFORMATION
import open3d as o3d

# ========== 1. 定义变换矩阵 ==========

# FLIPPING_v3 的 pose_in_link (T_world_cam, wxyz 格式)
pose_in_link = np.array([
    0.07783932332093665,
    0.2078814260418823,
    0.34723683952957585,
    0.2273133855008057,
    -0.6785647482083789,
    0.6637673415778982,
    -0.21746591345696367
])

# 四元数转旋转矩阵 (wxyz 格式)
def quat_to_mat_wxyz(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

# 7D pose转4x4矩阵 (wxyz 格式)
def pose7d_to_T_wxyz(pose7d):
    xyz = pose7d[:3]
    q = pose7d[3:]
    R = quat_to_mat_wxyz(q)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T

# 转换为 T_world_cam
T_world_cam = pose7d_to_T_wxyz(pose_in_link)
T_cam_world = np.linalg.inv(T_world_cam)

# world → robot_base 变换矩阵 (从 find_world_base_transform.py 计算得到)
T_world_base_urdf = np.array([
    [ 0.70771504, -0.6363821,  -0.30685198,  0.78460977],
    [-0.27343178, -0.64720033,  0.71159471,  0.32980268],
    [-0.6514406,  -0.4197032,  -0.63204052,  0.66463799],
    [ 0.,          0.,          0.,          1.        ]
], dtype=np.float32)

# 计算 T_cam_base (camera → URDF base)
# T_cam_base = T_cam_world @ T_world_base
T_cam_base = T_cam_world @ T_world_base_urdf

print("=" * 80)
print("坐标系转换")
print("=" * 80)
print()
print("FLIPPING_v3 的 T_world_cam (world → camera):")
print(T_world_cam)
print()
print("T_world_base_urdf (world → URDF base):")
print(T_world_base_urdf)
print()
print("转换后的 T_cam_base (camera → URDF base):")
print(T_cam_base)
print()

# ========== 2. 读取关节数据 ==========

scene_path = "/data/haoxiang/data/airexo2/task_0013/train/scene_0001"
lowdim_path = os.path.join(scene_path, "lowdim")

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

print(f"左臂关节: {left_joint}")
print(f"右臂关节: {right_joint}")
print()

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

# 使用相机序列号 105422061350
camera_serial = "105422061350"
cam_path = os.path.join(scene_path, f"cam_{camera_serial}")
depth_path = os.path.join(cam_path, "depth", "1737546126606.png")
rgb_path = os.path.join(cam_path, "color", "1737546126606.png")

depth_img = o3d.io.read_image(depth_path)
rgb_img = o3d.io.read_image(rgb_path)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
    rgb_img, depth_img,
    depth_scale=1000.0,
    convert_rgb_to_intensity=False
)

# 使用 AirExo-2 的内参
intrinsic = np.array([
    [912.4466, 0.0, 633.4127],
    [0.0, 911.4704, 364.21265],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

fx = intrinsic[0, 0]
fy = intrinsic[1, 1]
cx = intrinsic[0, 2]
cy = intrinsic[1, 2]
h, w = np.asarray(rgb_img).shape[:2]

intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
    width=int(w), height=int(h),
    fx=float(fx), fy=float(fy),
    cx=float(cx), cy=float(cy)
)

pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic_o3d)

# ========== 5. K3D 可视化 ==========

plot = k3d.plot()

# 5.1 渲染机械臂
# 使用转换后的 T_cam_base 进行渲染
for link, transform in cur_transforms.items():
    if link not in visuals_map: continue
    
    for v in visuals_map[link]:
        if v.geom_param is None: continue
        
        mesh_path = os.path.join("airexo/urdf_models/robot", v.geom_param)
        if not os.path.exists(mesh_path):
            continue

        mesh = trimesh.load(mesh_path, force='mesh')
        
        # 使用转换后的 T_cam_base
        # 变换链: mesh → link → URDF基座 → 相机坐标系 → Open3D渲染空间
        tf = O3D_RENDER_TRANSFORMATION @ T_cam_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
        mesh.apply_transform(tf)
        
        plot += k3d.mesh(mesh.vertices.astype(np.float32), 
                         mesh.faces.astype(np.uint32),
                         color=0xaaaaaa)

# 5.2 渲染点云
pcd_cam.transform(O3D_RENDER_TRANSFORMATION)

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

# 5.3 添加坐标轴
axis_size = 0.3
axes = k3d.vectors(
    origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    vectors=[[axis_size, 0, 0], [0, axis_size, 0], [0, 0, axis_size]],
    colors=[0xff0000, 0x00ff00, 0x0000ff],
    line_width=0.01
)
plot += axes

print("=" * 80)
print("可视化说明")
print("=" * 80)
print()
print("1. 使用 FLIPPING_v3 的 pose_in_link (T_world_cam)")
print("2. 通过 T_world_base_urdf 转换为 T_cam_base (AirExo-2 坐标系)")
print("3. 使用 AirExo-2 的渲染逻辑进行点云对齐")
print()
print("变换链:")
print("  机械臂: mesh → link → URDF基座 → 相机坐标系 → Open3D渲染空间")
print("  点云: 相机坐标系 → Open3D渲染空间")
print()
print("关键变换:")
print("  T_cam_base = T_cam_world @ T_world_base_urdf")
print("  机械臂变换 = O3D_RENDER_TRANSFORMATION @ T_cam_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform @ offset")
print("  点云变换 = O3D_RENDER_TRANSFORMATION")
print()

plot.display()
