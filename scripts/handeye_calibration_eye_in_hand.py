#!/usr/bin/env python3
import argparse
import json
import math
import time
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs

try:
    from fairino import Robot
except Exception:  # pragma: no cover - allow running without SDK installed
    Robot = None


@dataclass
class PoseSample:
    r_gripper2base: np.ndarray
    t_gripper2base: np.ndarray
    r_target2cam: np.ndarray
    t_target2cam: np.ndarray


def get_aruco_dict(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def create_detector_params():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


def detect_markers(gray, aruco_dict, params):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)


def rs_intrinsics(profile):
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()
    k = np.array(
        [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    d = np.array(intr.coeffs, dtype=np.float64).reshape(1, -1)
    return k, d


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rmat = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    return rmat


def pose_to_gripper2base(
    pose_mm_deg: List[float], angle_type: str
) -> Tuple[np.ndarray, np.ndarray]:
    """
    将机器人姿态转换为 base→gripper(flange) 的变换
    
    Args:
        pose_mm_deg: GetActualToolFlangePose() 返回的 [x,y,z,rx,ry,rz]
                     已经是 base→flange，无需求逆！
    Returns:
        r_gripper2base: 旋转矩阵 (base→gripper)
        t_gripper2base: 平移向量 (base→gripper，单位米)
    """
    x, y, z, rx, ry, rz = pose_mm_deg
    t_base_gripper = np.array([x, y, z], dtype=np.float64).reshape(3, 1) / 1000.0

    if angle_type == "rpy":
        rmat = rpy_to_matrix(
            math.radians(rx), math.radians(ry), math.radians(rz)
        )
    elif angle_type == "rotvec":
        rvec = np.radians(np.array([rx, ry, rz], dtype=np.float64)).reshape(3, 1)
        rmat, _ = cv2.Rodrigues(rvec)
    else:
        raise ValueError("angle_type must be 'rpy' or 'rotvec'")

    # 关键修正：GetActualToolFlangePose() 已经返回 base→flange，直接使用！
    r_gripper2base = rmat  # 修正：不要转置
    t_gripper2base = t_base_gripper  # 修正：不要取反
    return r_gripper2base, t_gripper2base


def save_samples(path: str, samples: List[PoseSample]) -> None:
    r_gb = np.stack([s.r_gripper2base for s in samples], axis=0)
    t_gb = np.stack([s.t_gripper2base for s in samples], axis=0)
    r_tc = np.stack([s.r_target2cam for s in samples], axis=0)
    t_tc = np.stack([s.t_target2cam for s in samples], axis=0)
    np.savez(path, r_gb=r_gb, t_gb=t_gb, r_tc=r_tc, t_tc=t_tc)


def save_handeye_result(path: str, r_cam2gripper: np.ndarray, t_cam2gripper: np.ndarray) -> None:
    t = t_cam2gripper.reshape(3, 1)
    t_cam2gripper = t
    t_mat = np.eye(4, dtype=np.float64)
    t_mat[:3, :3] = r_cam2gripper
    t_mat[:3, 3:4] = t_cam2gripper
    np.savetxt(path, t_mat, fmt="%.8f")


def load_samples(path: str) -> List[PoseSample]:
    data = np.load(path)
    samples = []
    for i in range(data["r_gb"].shape[0]):
        samples.append(
            PoseSample(
                r_gripper2base=data["r_gb"][i],
                t_gripper2base=data["t_gb"][i],
                r_target2cam=data["r_tc"][i],
                t_target2cam=data["t_tc"][i],
            )
        )
    return samples


def calibrate_handeye(samples: List[PoseSample]):
    r_gb = [s.r_gripper2base for s in samples]
    t_gb = [s.t_gripper2base for s in samples]
    r_tc = [s.r_target2cam for s in samples]
    t_tc = [s.t_target2cam for s in samples]

    r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        r_gb,
        t_gb,
        r_tc,
        t_tc,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    return r_cam2gripper, t_cam2gripper


def parse_args():
    parser = argparse.ArgumentParser(
        description="Eye-in-hand hand-eye calibration with FR5 + D435i + ArUco"
    )
    parser.add_argument("--robot-ip", type=str, default="192.168.1.200")
    parser.add_argument("--dictionary", type=str, default="DICT_4X4_50")
    parser.add_argument("--target-id", type=int, default=0)
    parser.add_argument("--marker-length-mm", type=float, default=50.0)
    parser.add_argument("--color-width", type=int, default=1280)
    parser.add_argument("--color-height", type=int, default=720)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--angle-type", choices=["rpy", "rotvec"], default="rpy")
    parser.add_argument("--use-flange", action="store_true")
    parser.add_argument("--min-samples", type=int, default=12)
    parser.add_argument("--save", type=str, default="handeye_samples.npz")
    parser.add_argument("--save-result", type=str, default="handeye_result.txt")
    parser.add_argument("--log-dir", type=str, default="handeye_logs")
    parser.add_argument("--load", type=str, default="")
    parser.add_argument("--no-live", action="store_true")
    return parser.parse_args()


def ensure_log_dir(path: str) -> Path:
    log_dir = Path(path)
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / "images").mkdir(parents=True, exist_ok=True)
    return log_dir


def append_jsonl(path: Path, record: dict) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")


def main():
    args = parse_args()

    if args.load:
        samples = load_samples(args.load)
        r_cam2gripper, t_cam2gripper = calibrate_handeye(samples)
        print("Loaded samples:", len(samples))
        print("R_cam2gripper:\n", r_cam2gripper)
        print("t_cam2gripper (m):", t_cam2gripper.reshape(3))
        if args.save_result:
            save_handeye_result(args.save_result, r_cam2gripper, t_cam2gripper)
            print("Saved hand-eye result to", args.save_result)
        return

    if Robot is None:
        raise RuntimeError("fairino SDK not found. Install and retry.")

    robot = Robot.RPC(args.robot_ip)

    aruco_dict = get_aruco_dict(args.dictionary)
    detector_params = create_detector_params()
    marker_len = args.marker_length_mm / 1000.0

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.color,
        args.color_width,
        args.color_height,
        rs.format.bgr8,
        args.fps,
    )
    config.enable_stream(
        rs.stream.depth,
        args.depth_width,
        args.depth_height,
        rs.format.z16,
        args.fps,
    )

    profile = pipeline.start(config)
    k, d = rs_intrinsics(profile)
    align = rs.align(rs.stream.color)

    log_dir = ensure_log_dir(args.log_dir)
    log_path = log_dir / "samples.jsonl"
    idx = 0

    samples: List[PoseSample] = []

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detect_markers(gray, aruco_dict, detector_params)

            display = color_image.copy()
            detected = False
            r_target2cam = None
            t_target2cam = None

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
                for corner, marker_id in zip(corners, ids.flatten()):
                    if args.target_id >= 0 and marker_id != args.target_id:
                        continue
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corner], marker_len, k, d
                    )
                    rvec = rvecs[0].reshape(3, 1)
                    tvec = tvecs[0].reshape(3, 1)
                    r_target2cam, _ = cv2.Rodrigues(rvec)
                    t_target2cam = tvec
                    cv2.drawFrameAxes(display, k, d, rvec, tvec, marker_len * 0.5)
                    detected = True
                    break

            status = f"samples: {len(samples)}"
            if detected:
                status += " | marker detected"
            cv2.putText(
                display,
                status,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                display,
                "SPACE: capture  ESC/q: quit",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            if not args.no_live:
                cv2.imshow("handeye_capture", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break
            if key == ord(" "):
                if not detected or r_target2cam is None or t_target2cam is None:
                    print("No marker detected. Move robot or adjust view.")
                    continue

                if args.use_flange:
                    err, pose = robot.GetActualToolFlangePose(0)
                else:
                    err, pose = robot.GetActualTCPPose(0)
                if err != 0:
                    print("Robot pose error:", err)
                    continue

                r_gb, t_gb = pose_to_gripper2base(pose, args.angle_type)
                ts = time.time()
                image_name = f"images/sample_{idx:03d}.png"
                image_path = log_dir / image_name
                cv2.imwrite(str(image_path), color_image)
                record = {
                    "index": idx,
                    "timestamp": ts,
                    "robot_ip": args.robot_ip,
                    "angle_type": args.angle_type,
                    "use_flange": bool(args.use_flange),
                    "dictionary": args.dictionary,
                    "target_id": int(args.target_id),
                    "marker_length_m": marker_len,
                    "image": image_name,
                    "pose_mm_deg": [float(v) for v in pose],
                    "r_gripper2base": r_gb.tolist(),
                    "t_gripper2base_m": t_gb.reshape(3).tolist(),
                    "r_target2cam": r_target2cam.tolist(),
                    "t_target2cam_m": t_target2cam.reshape(3).tolist(),
                }
                append_jsonl(log_path, record)
                samples.append(
                    PoseSample(
                        r_gripper2base=r_gb,
                        t_gripper2base=t_gb,
                        r_target2cam=r_target2cam,
                        t_target2cam=t_target2cam,
                    )
                )
                idx += 1
                print(f"Captured {len(samples)} samples")

            if len(samples) >= args.min_samples:
                r_cam2gripper, t_cam2gripper = calibrate_handeye(samples)
                print("R_cam2gripper:\n", r_cam2gripper)
                print("t_cam2gripper (m):", t_cam2gripper.reshape(3))
                if args.save_result:
                    save_handeye_result(args.save_result, r_cam2gripper, t_cam2gripper)
                    print("Saved hand-eye result to", args.save_result)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        try:
            robot.CloseRPC()
        except Exception:
            pass

    if samples:
        save_samples(args.save, samples)
        print(f"Saved {len(samples)} samples to {args.save}")


if __name__ == "__main__":
    main()
