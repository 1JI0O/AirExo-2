"""
计算 world 坐标系和 robot_base 坐标系之间的变换关系

FLIPPING_v3 的 pose_in_link 是 T_world_cam (world → camera)
AirExo-2 的 cam_to_base 是 T_cam_base (camera → robot_base)

我们需要找到 T_world_base (world → robot_base) 或 T_base_world (robot_base → world)

关系：
T_world_cam = T_world_base @ T_base_cam
T_cam_base = T_cam_world @ T_world_base

因此：
T_world_base = T_world_cam @ T_cam_base
T_base_world = inv(T_world_base)
"""

import numpy as np

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

# 加载 AirExo-2 标定数据
calib = np.load('/data/haoxiang/data/airexo2/task_0013/calib/1737548651048.npy', allow_pickle=True).item()

# FLIPPING_v3 的 pose_in_link (T_world_cam)
pose_in_link = np.array([
    0.07783932332093665,
    0.2078814260418823,
    0.34723683952957585,
    0.2273133855008057,
    -0.6785647482083789,
    0.6637673415778982,
    -0.21746591345696367
])

# 转换为 T_world_cam (wxyz 格式)
T_world_cam = pose7d_to_T_wxyz(pose_in_link)
T_cam_world = np.linalg.inv(T_world_cam)

# AirExo-2 的 T_cam_base
# 从 calib_info.py 的 get_camera_to_base 方法中提取
T_cam_marker = calib['extrinsics']['105422061350']
T_marker_cam_inhand = np.linalg.inv(calib['extrinsics'][calib['camera_serial_inhand_left']])

ROBOT_LEFT_CAM_TO_TCP = np.array([
    [-0.01240050, 0.99905890, 0.04157753, -0.09342833],
    [-0.99978572, -0.01307791, 0.01605635, 0.02158097],
    [0.01658444, -0.04136945, 0.99900651, -0.00380356],
    [0.00000000, 0.00000000, 0.00000000, 1.00000000]
], dtype=np.float32)

T_cam_tcp = ROBOT_LEFT_CAM_TO_TCP

# tcp_pose 转换为 T_tcp_base
tcp_pose_left = calib['robot_left']['tcp_pose']
def quat_to_mat_xyzw(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

def pose7d_to_T_xyzw(pose7d):
    xyz = pose7d[:3]
    q = pose7d[3:]
    R = quat_to_mat_xyzw(q)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T

T_base_tcp = pose7d_to_T_xyzw(tcp_pose_left)
T_tcp_base = np.linalg.inv(T_base_tcp)

# 完整的 T_cam_base (real_base=True，即相机到真实机器人基座)
T_cam_base_real = T_cam_marker @ T_marker_cam_inhand @ T_cam_tcp @ T_tcp_base

# 完整的 T_cam_base (real_base=False，即相机到URDF基座)
ROBOT_PREDEFINED_TRANSFORMATION = np.array([
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
], dtype=np.float32)

T_cam_base_urdf = T_cam_base_real @ np.linalg.inv(ROBOT_PREDEFINED_TRANSFORMATION)

print("=" * 80)
print("计算 world 坐标系和 robot_base 坐标系之间的变换关系")
print("=" * 80)
print()

print("FLIPPING_v3 的 T_world_cam (world → camera):")
print(T_world_cam)
print()

print("FLIPPING_v3 的 T_cam_world (camera → world):")
print(T_cam_world)
print()

print("AirExo-2 的 T_cam_base_real (camera → real robot base):")
print(T_cam_base_real)
print()

print("AirExo-2 的 T_cam_base_urdf (camera → URDF base):")
print(T_cam_base_urdf)
print()

print("=" * 80)
print("计算 T_world_base (world → robot_base)")
print("=" * 80)
print()

# 方法1: T_world_base = T_world_cam @ T_cam_base
T_world_base_real = T_world_cam @ T_cam_base_real
T_world_base_urdf = T_world_cam @ T_cam_base_urdf

print("方法1: T_world_base = T_world_cam @ T_cam_base")
print()
print("T_world_base_real (world → real robot base):")
print(T_world_base_real)
print()

print("T_world_base_urdf (world → URDF base):")
print(T_world_base_urdf)
print()

# 方法2: T_world_base = inv(T_cam_world) @ T_cam_base
T_world_base_real2 = np.linalg.inv(T_cam_world) @ T_cam_base_real
T_world_base_urdf2 = np.linalg.inv(T_cam_world) @ T_cam_base_urdf

print("方法2: T_world_base = inv(T_cam_world) @ T_cam_base")
print()
print("T_world_base_real (world → real robot base):")
print(T_world_base_real2)
print()

print("T_world_base_urdf (world → URDF base):")
print(T_world_base_urdf2)
print()

# 验证两种方法是否一致
print("=" * 80)
print("验证两种方法是否一致")
print("=" * 80)
print()

diff_real = np.linalg.norm(T_world_base_real - T_world_base_real2, 'fro')
diff_urdf = np.linalg.norm(T_world_base_urdf - T_world_base_urdf2, 'fro')

print(f"T_world_base_real 两种方法的差异: {diff_real:.10f}")
print(f"T_world_base_urdf 两种方法的差异: {diff_urdf:.10f}")
print()

# 计算逆变换
print("=" * 80)
print("计算逆变换 T_base_world (robot_base → world)")
print("=" * 80)
print()

T_base_world_real = np.linalg.inv(T_world_base_real)
T_base_world_urdf = np.linalg.inv(T_world_base_urdf)

print("T_base_world_real (real robot base → world):")
print(T_base_world_real)
print()

print("T_base_world_urdf (URDF base → world):")
print(T_base_world_urdf)
print()

# 验证变换链
print("=" * 80)
print("验证变换链")
print("=" * 80)
print()

# 验证: T_world_cam = T_world_base @ T_base_cam
T_base_cam_real = np.linalg.inv(T_cam_base_real)
T_base_cam_urdf = np.linalg.inv(T_cam_base_urdf)

T_world_cam_check_real = T_world_base_real @ T_base_cam_real
T_world_cam_check_urdf = T_world_base_urdf @ T_base_cam_urdf

diff_check_real = np.linalg.norm(T_world_cam - T_world_cam_check_real, 'fro')
diff_check_urdf = np.linalg.norm(T_world_cam - T_world_cam_check_urdf, 'fro')

print("验证: T_world_cam = T_world_base @ T_base_cam")
print()
print(f"real robot base: 差异 = {diff_check_real:.10f}")
print(f"URDF base: 差异 = {diff_check_urdf:.10f}")
print()

# 验证: T_cam_base = T_cam_world @ T_world_base
T_cam_base_check_real = T_cam_world @ T_world_base_real
T_cam_base_check_urdf = T_cam_world @ T_world_base_urdf

diff_check2_real = np.linalg.norm(T_cam_base_real - T_cam_base_check_real, 'fro')
diff_check2_urdf = np.linalg.norm(T_cam_base_urdf - T_cam_base_check_urdf, 'fro')

print("验证: T_cam_base = T_cam_world @ T_world_base")
print()
print(f"real robot base: 差异 = {diff_check2_real:.10f}")
print(f"URDF base: 差异 = {diff_check2_urdf:.10f}")
print()

# 总结
print("=" * 80)
print("总结")
print("=" * 80)
print()

print("关键变换矩阵:")
print()
print("1. T_world_base_real (world → real robot base):")
print(T_world_base_real)
print()

print("2. T_world_base_urdf (world → URDF base):")
print(T_world_base_urdf)
print()

print("3. T_base_world_real (real robot base → world):")
print(T_base_world_real)
print()

print("4. T_base_world_urdf (URDF base → world):")
print(T_base_world_urdf)
print()

print("使用方法:")
print()
print("如果要将 FLIPPING_v3 的标定数据转换为 AirExo-2 的坐标系:")
print("  T_cam_base_airexo = T_cam_world_flipping @ T_world_base")
print()
print("如果要将 AirExo-2 的标定数据转换为 FLIPPING_v3 的坐标系:")
print("  T_cam_world_flipping = T_cam_base_airexo @ T_base_world")
