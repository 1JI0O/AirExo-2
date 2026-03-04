"""
Calib transform utility.

Extracted from dataset_transform.py calibration conversion logic.

Usage:
  python -m airexo.adaptor.calib_transform_tool \
      --input-calib /path/to/airexo_calib.npy \
      --output-calib /path/to/rise2_calib.npy
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict

import numpy as np

# 兼容两种调用方式：
# 1) python -m airexo.adaptor.calib_transform_tool
# 2) python calib_transform_tool.py（在 adaptor 目录下直跑）
if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from airexo.helpers.constants import ROBOT_LEFT_CAM_TO_TCP, ROBOT_RIGHT_CAM_TO_TCP

def _as_matrix4(value: Any) -> np.ndarray:
    """Normalize a matrix-like object to a 4x4 float32 numpy array."""
    if isinstance(value, list):
        if len(value) == 0:
            raise ValueError("Empty list encountered when parsing calibration matrix.")
        value = value[0]
    mat = np.asarray(value, dtype=np.float32)
    if mat.shape != (4, 4):
        raise ValueError(f"Expected a 4x4 matrix, but got shape {mat.shape}.")
    return mat


def _quat_wxyz_to_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] to a 3x3 rotation matrix."""
    q = np.asarray(quat_wxyz, dtype=np.float32).reshape(-1)
    if q.shape[0] != 4:
        raise ValueError(f"Expected quaternion with 4 elements, but got shape {q.shape}.")

    w, x, y, z = q
    n = float(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("Quaternion norm is too small.")

    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float32,
    )


def _tcp_pose7_to_mat(tcp_pose: Any) -> np.ndarray:
    """Convert tcp pose [x, y, z, qw, qx, qy, qz] to 4x4 matrix."""
    pose = np.asarray(tcp_pose, dtype=np.float32).reshape(-1)
    if pose.shape[0] != 7:
        raise ValueError(f"Expected tcp pose with 7 elements, but got shape {pose.shape}.")

    mat = np.eye(4, dtype=np.float32)
    mat[:3, :3] = _quat_wxyz_to_rotmat(pose[3:])
    mat[:3, 3] = pose[:3]
    return mat


def _camera_to_robot_left_base(calib: Dict[str, Any], serial: str) -> np.ndarray:
    extrinsics = calib["extrinsics"]
    inhand_left = calib["camera_serial_inhand_left"]
    left_tcp_pose = calib["robot_left"]["tcp_pose"]

    cam_to_global_marker = _as_matrix4(extrinsics[serial])
    inhand_to_global_marker = _as_matrix4(extrinsics[inhand_left])
    left_tcp_pose_mat = _tcp_pose7_to_mat(left_tcp_pose)

    cam_to_left_base = (
        cam_to_global_marker
        @ np.linalg.inv(inhand_to_global_marker)
        @ ROBOT_LEFT_CAM_TO_TCP
        @ np.linalg.inv(left_tcp_pose_mat)
    )
    return cam_to_left_base.astype(np.float32)


def _camera_to_robot_right_base(calib: Dict[str, Any], serial: str) -> np.ndarray:
    extrinsics = calib["extrinsics"]
    inhand_right = calib["camera_serial_inhand_right"]
    right_tcp_pose = calib["robot_right"]["tcp_pose"]

    cam_to_global_marker = _as_matrix4(extrinsics[serial])
    inhand_to_global_marker = _as_matrix4(extrinsics[inhand_right])
    right_tcp_pose_mat = _tcp_pose7_to_mat(right_tcp_pose)

    cam_to_right_base = (
        cam_to_global_marker
        @ np.linalg.inv(inhand_to_global_marker)
        @ ROBOT_RIGHT_CAM_TO_TCP
        @ np.linalg.inv(right_tcp_pose_mat)
    )
    return cam_to_right_base.astype(np.float32)


def convert_airexo_calib_to_rise2(input_calib_path: str, output_calib_path: str) -> Dict[str, Any]:
    """
    Convert one AirExo-style calibration npy to RISE2-style calibration npy.

    Args:
        input_calib_path: Path to AirExo-style .npy calibration file.
        output_calib_path: Output path for converted RISE2-style .npy calibration file.

    Returns:
        Converted calibration dictionary.
    """
    input_path = Path(input_calib_path)
    output_path = Path(output_calib_path)

    if not input_path.exists():
        raise FileNotFoundError(f"Input calibration file does not exist: {input_path}")

    calib = np.load(str(input_path), allow_pickle=True).item()
    if not isinstance(calib, dict):
        raise TypeError("Loaded calibration file is not a dict.")

    required_keys = [
        "type",
        "camera_serials",
        "camera_serials_global",
        "camera_serial_inhand_left",
        "camera_serial_inhand_right",
        "intrinsics",
    ]
    for key in required_keys:
        if key not in calib:
            raise KeyError(f"Missing required key in input calibration: {key}")

    calib_type = calib["type"]
    is_robot = calib_type == "robot"

    if is_robot:
        robot_required = ["extrinsics", "robot_left", "robot_right"]
        for key in robot_required:
            if key not in calib:
                raise KeyError(f"Missing required key for robot calibration: {key}")

    calib_res: Dict[str, Any] = {
        "type": calib_type,
        "camera_serials": calib["camera_serials"],
        "camera_serials_global": calib["camera_serials_global"],
        "camera_serial_inhand_left": calib["camera_serial_inhand_left"],
        "camera_serial_inhand_right": calib["camera_serial_inhand_right"],
        "intrinsics": calib["intrinsics"],
        "camera_to_robot_left": {},
        "camera_to_robot_right": {},
    }

    for serial in calib["camera_serials_global"]:
        if is_robot:
            calib_res["camera_to_robot_left"][serial] = _camera_to_robot_left_base(calib, serial)
            calib_res["camera_to_robot_right"][serial] = _camera_to_robot_right_base(calib, serial)
        else:
            calib_res["camera_to_robot_left"][serial] = np.eye(4, dtype=np.float32)
            calib_res["camera_to_robot_right"][serial] = np.eye(4, dtype=np.float32)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(str(output_path), calib_res, allow_pickle=True)
    return calib_res


def _build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert AirExo-style calibration npy into RISE2-style calibration npy."
    )
    parser.add_argument(
        "--input-calib",
        required=True,
        help="Path to AirExo-style calibration npy file.",
    )
    parser.add_argument(
        "--output-calib",
        required=True,
        help="Output path for converted RISE2-style calibration npy file.",
    )
    return parser


def main() -> None:
    args = _build_argparser().parse_args()
    convert_airexo_calib_to_rise2(
        input_calib_path=args.input_calib,
        output_calib_path=args.output_calib,
    )


if __name__ == "__main__":
    main()
