"""
检查 FLIPPING_v3 的 pose_in_link 矩阵能否对应上 AirExo-2 变换链中的某种组合

变换链：
T_cam_base = T_cam_marker @ T_marker_cam_inhand @ T_cam_tcp @ T_tcp_base
"""

import numpy as np

# 四元数转旋转矩阵 (xyzw 格式)
def quat_to_mat_xyzw(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

# 四元数转旋转矩阵 (wxyz 格式)
def quat_to_mat_wxyz(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

# 7D pose转4x4矩阵 (xyzw 格式)
def pose7d_to_T_xyzw(pose7d):
    xyz = pose7d[:3]
    q = pose7d[3:]
    R = quat_to_mat_xyzw(q)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T

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

# FLIPPING_v3 的 pose_in_link
pose_in_link = np.array([
    0.07783932332093665,
    0.2078814260418823,
    0.34723683952957585,
    0.2273133855008057,
    -0.6785647482083789,
    0.6637673415778982,
    -0.21746591345696367
])

# AirExo-2 变换链中的各个部分
T_cam_marker = calib['extrinsics']['105422061350']  # 相机 → ArUco标记
T_marker_cam_inhand = np.linalg.inv(calib['extrinsics'][calib['camera_serial_inhand_left']])  # 标记 → 手持相机

# ROBOT_LEFT_CAM_TO_TCP 常量
ROBOT_LEFT_CAM_TO_TCP = np.array([
    [-0.01240050, 0.99905890, 0.04157753, -0.09342833],
    [-0.99978572, -0.01307791, 0.01605635, 0.02158097],
    [0.01658444, -0.04136945, 0.99900651, -0.00380356],
    [0.00000000, 0.00000000, 0.00000000, 1.00000000]
], dtype=np.float32)

T_cam_tcp = ROBOT_LEFT_CAM_TO_TCP  # 手持相机 → TCP

# tcp_pose 转换为 T_tcp_base
tcp_pose_left = calib['robot_left']['tcp_pose']
T_base_tcp = pose7d_to_T_xyzw(tcp_pose_left)
T_tcp_base = np.linalg.inv(T_base_tcp)  # TCP → 机器人基座

print("=" * 80)
print("检查 FLIPPING_v3 的 pose_in_link 矩阵能否对应上 AirExo-2 变换链中的某种组合")
print("=" * 80)
print()

# 变换链中的各个部分
print("变换链中的各个部分:")
print()
print("1. T_cam_marker (相机 → ArUco标记):")
print(T_cam_marker)
print()

print("2. T_marker_cam_inhand (标记 → 手持相机):")
print(T_marker_cam_inhand)
print()

print("3. T_cam_tcp (手持相机 → TCP):")
print(T_cam_tcp)
print()

print("4. T_tcp_base (TCP → 机器人基座):")
print(T_tcp_base)
print()

# 计算完整的 T_cam_base
T_cam_base = T_cam_marker @ T_marker_cam_inhand @ T_cam_tcp @ T_tcp_base
print("5. T_cam_base (相机 → 机器人基座, 完整变换链):")
print(T_cam_base)
print()

# 测试 pose_in_link 的两种格式
print("=" * 80)
print("测试 pose_in_link 的两种格式")
print("=" * 80)
print()

# xyzw 格式
T_world_cam_xyzw = pose7d_to_T_xyzw(pose_in_link)
T_cam_world_xyzw = np.linalg.inv(T_world_cam_xyzw)

print("pose_in_link (xyzw 格式) -> T_world_cam:")
print(T_world_cam_xyzw)
print()

print("pose_in_link (xyzw 格式) -> T_cam_world:")
print(T_cam_world_xyzw)
print()

# wxyz 格式
T_world_cam_wxyz = pose7d_to_T_wxyz(pose_in_link)
T_cam_world_wxyz = np.linalg.inv(T_world_cam_wxyz)

print("pose_in_link (wxyz 格式) -> T_world_cam:")
print(T_world_cam_wxyz)
print()

print("pose_in_link (wxyz 格式) -> T_cam_world:")
print(T_cam_world_wxyz)
print()

# 检查 pose_in_link 是否对应变换链中的某个部分
print("=" * 80)
print("检查 pose_in_link 是否对应变换链中的某个部分")
print("=" * 80)
print()

def compare_matrices(name, T1, T2):
    """比较两个矩阵的差异"""
    rot_diff = np.linalg.norm(T1[:3, :3] - T2[:3, :3], 'fro')
    trans_diff = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
    total_diff = np.linalg.norm(T1 - T2, 'fro')
    print(f"{name}:")
    print(f"  旋转差异 (Frobenius): {rot_diff:.6f}")
    print(f"  平移差异 (L2): {trans_diff:.6f}")
    print(f"  总差异 (Frobenius): {total_diff:.6f}")
    return total_diff

# 测试 xyzw 格式
print("xyzw 格式:")
print()

# 检查 T_world_cam_xyzw 是否对应某个部分
print("检查 T_world_cam_xyzw 是否对应某个部分:")
diff1 = compare_matrices("  vs T_cam_marker", T_world_cam_xyzw, T_cam_marker)
diff2 = compare_matrices("  vs T_marker_cam_inhand", T_world_cam_xyzw, T_marker_cam_inhand)
diff3 = compare_matrices("  vs T_cam_tcp", T_world_cam_xyzw, T_cam_tcp)
diff4 = compare_matrices("  vs T_tcp_base", T_world_cam_xyzw, T_tcp_base)
diff5 = compare_matrices("  vs T_cam_base", T_world_cam_xyzw, T_cam_base)
print()

# 检查 T_cam_world_xyzw 是否对应某个部分
print("检查 T_cam_world_xyzw 是否对应某个部分:")
diff6 = compare_matrices("  vs T_cam_marker", T_cam_world_xyzw, T_cam_marker)
diff7 = compare_matrices("  vs T_marker_cam_inhand", T_cam_world_xyzw, T_marker_cam_inhand)
diff8 = compare_matrices("  vs T_cam_tcp", T_cam_world_xyzw, T_cam_tcp)
diff9 = compare_matrices("  vs T_tcp_base", T_cam_world_xyzw, T_tcp_base)
diff10 = compare_matrices("  vs T_cam_base", T_cam_world_xyzw, T_cam_base)
print()

# 测试 wxyz 格式
print("wxyz 格式:")
print()

# 检查 T_world_cam_wxyz 是否对应某个部分
print("检查 T_world_cam_wxyz 是否对应某个部分:")
diff11 = compare_matrices("  vs T_cam_marker", T_world_cam_wxyz, T_cam_marker)
diff12 = compare_matrices("  vs T_marker_cam_inhand", T_world_cam_wxyz, T_marker_cam_inhand)
diff13 = compare_matrices("  vs T_cam_tcp", T_world_cam_wxyz, T_cam_tcp)
diff14 = compare_matrices("  vs T_tcp_base", T_world_cam_wxyz, T_tcp_base)
diff15 = compare_matrices("  vs T_cam_base", T_world_cam_wxyz, T_cam_base)
print()

# 检查 T_cam_world_wxyz 是否对应某个部分
print("检查 T_cam_world_wxyz 是否对应某个部分:")
diff16 = compare_matrices("  vs T_cam_marker", T_cam_world_wxyz, T_cam_marker)
diff17 = compare_matrices("  vs T_marker_cam_inhand", T_cam_world_wxyz, T_marker_cam_inhand)
diff18 = compare_matrices("  vs T_cam_tcp", T_cam_world_wxyz, T_cam_tcp)
diff19 = compare_matrices("  vs T_tcp_base", T_cam_world_wxyz, T_tcp_base)
diff20 = compare_matrices("  vs T_cam_base", T_cam_world_wxyz, T_cam_base)
print()

# 检查变换链中的部分组合
print("=" * 80)
print("检查变换链中的部分组合")
print("=" * 80)
print()

# 计算部分组合
T_cam_marker_inhand = T_cam_marker @ T_marker_cam_inhand
T_cam_marker_inhand_tcp = T_cam_marker @ T_marker_cam_inhand @ T_cam_tcp
T_marker_inhand_tcp = T_marker_cam_inhand @ T_cam_tcp
T_marker_inhand_tcp_base = T_marker_cam_inhand @ T_cam_tcp @ T_tcp_base

print("部分组合:")
print()
print("1. T_cam_marker @ T_marker_cam_inhand:")
print(T_cam_marker_inhand)
print()

print("2. T_cam_marker @ T_marker_cam_inhand @ T_cam_tcp:")
print(T_cam_marker_inhand_tcp)
print()

print("3. T_marker_cam_inhand @ T_cam_tcp:")
print(T_marker_inhand_tcp)
print()

print("4. T_marker_cam_inhand @ T_cam_tcp @ T_tcp_base:")
print(T_marker_inhand_tcp_base)
print()

# 检查 pose_in_link 是否对应部分组合
print("检查 pose_in_link (xyzw 格式) 是否对应部分组合:")
print()
diff21 = compare_matrices("  vs T_cam_marker @ T_marker_cam_inhand", T_world_cam_xyzw, T_cam_marker_inhand)
diff22 = compare_matrices("  vs T_cam_marker @ T_marker_cam_inhand @ T_cam_tcp", T_world_cam_xyzw, T_cam_marker_inhand_tcp)
diff23 = compare_matrices("  vs T_marker_cam_inhand @ T_cam_tcp", T_world_cam_xyzw, T_marker_inhand_tcp)
diff24 = compare_matrices("  vs T_marker_cam_inhand @ T_cam_tcp @ T_tcp_base", T_world_cam_xyzw, T_marker_inhand_tcp_base)
print()

print("检查 pose_in_link (wxyz 格式) 是否对应部分组合:")
print()
diff25 = compare_matrices("  vs T_cam_marker @ T_marker_cam_inhand", T_world_cam_wxyz, T_cam_marker_inhand)
diff26 = compare_matrices("  vs T_cam_marker @ T_marker_cam_inhand @ T_cam_tcp", T_world_cam_wxyz, T_cam_marker_inhand_tcp)
diff27 = compare_matrices("  vs T_marker_cam_inhand @ T_cam_tcp", T_world_cam_wxyz, T_marker_inhand_tcp)
diff28 = compare_matrices("  vs T_marker_cam_inhand @ T_cam_tcp @ T_tcp_base", T_world_cam_wxyz, T_marker_inhand_tcp_base)
print()

# 总结
print("=" * 80)
print("总结")
print("=" * 80)
print()

all_diffs = [
    ("xyzw T_world_cam vs T_cam_marker", diff1),
    ("xyzw T_world_cam vs T_marker_cam_inhand", diff2),
    ("xyzw T_world_cam vs T_cam_tcp", diff3),
    ("xyzw T_world_cam vs T_tcp_base", diff4),
    ("xyzw T_world_cam vs T_cam_base", diff5),
    ("xyzw T_cam_world vs T_cam_marker", diff6),
    ("xyzw T_cam_world vs T_marker_cam_inhand", diff7),
    ("xyzw T_cam_world vs T_cam_tcp", diff8),
    ("xyzw T_cam_world vs T_tcp_base", diff9),
    ("xyzw T_cam_world vs T_cam_base", diff10),
    ("wxyz T_world_cam vs T_cam_marker", diff11),
    ("wxyz T_world_cam vs T_marker_cam_inhand", diff12),
    ("wxyz T_world_cam vs T_cam_tcp", diff13),
    ("wxyz T_world_cam vs T_tcp_base", diff14),
    ("wxyz T_world_cam vs T_cam_base", diff15),
    ("wxyz T_cam_world vs T_cam_marker", diff16),
    ("wxyz T_cam_world vs T_marker_cam_inhand", diff17),
    ("wxyz T_cam_world vs T_cam_tcp", diff18),
    ("wxyz T_cam_world vs T_tcp_base", diff19),
    ("wxyz T_cam_world vs T_cam_base", diff20),
    ("xyzw T_world_cam vs T_cam_marker @ T_marker_cam_inhand", diff21),
    ("xyzw T_world_cam vs T_cam_marker @ T_marker_cam_inhand @ T_cam_tcp", diff22),
    ("xyzw T_world_cam vs T_marker_cam_inhand @ T_cam_tcp", diff23),
    ("xyzw T_world_cam vs T_marker_cam_inhand @ T_cam_tcp @ T_tcp_base", diff24),
    ("wxyz T_world_cam vs T_cam_marker @ T_marker_cam_inhand", diff25),
    ("wxyz T_world_cam vs T_cam_marker @ T_marker_cam_inhand @ T_cam_tcp", diff26),
    ("wxyz T_world_cam vs T_marker_cam_inhand @ T_cam_tcp", diff27),
    ("wxyz T_world_cam vs T_marker_cam_inhand @ T_cam_tcp @ T_tcp_base", diff28),
]

all_diffs.sort(key=lambda x: x[1])

print("最接近的匹配（按差异从小到大排序）:")
print()
for i, (name, diff) in enumerate(all_diffs[:10]):
    print(f"{i+1}. {name}: {diff:.6f}")
