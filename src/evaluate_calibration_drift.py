"""Đánh giá định lượng ảnh hưởng của Calibration Drift (Góc quay Yaw/Pitch và Dịch chuyển Translation)
lên phép chiếu LiDAR-Camera.

Hỗ trợ đo lường:
1. Tỷ lệ điểm LiDAR trong ảnh (% inside FOV).
2. Tỷ lệ điểm rơi đúng vào 2D Bounding Box của xe/người theo cự ly gần (<20m) và xa (>=20m).
3. Độ dịch chuyển pixel trung bình (Mean Pixel Displacement).
4. Đo lường độ trễ thực thi (Latency p50 và p95) sau 25 lần chạy (bỏ lần đầu tiên).
5. Xuất bảng số liệu CSV và đồ thị phân tích đa chiều.
"""
from __future__ import annotations

import argparse
import platform
import sys
import time
from pathlib import Path

# Cấu hình UTF-8 cho Windows console
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from starter.datasets import dataset_type, load_frame
from starter.kitti_io import KittiCalib, KittiObject
from starter.projection import (
    draw_box2d,
    overlay_points,
    perturb_extrinsic,
    project_velo_to_image,
    velo_to_cam,
)


def extract_object_points(points_raw: np.ndarray, calib: KittiCalib,
                          objects: list[KittiObject], margin: float = 0.1) -> list[dict]:
    """Trích xuất danh sách các điểm LiDAR 3D thực sự thuộc về từng vật thể theo GT 3D bounding box."""
    pts_cam = velo_to_cam(points_raw[:, :3], calib)
    extracted = []

    for idx, obj in enumerate(objects):
        if obj.type == "DontCare":
            continue
        c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        p_rel = pts_cam - obj.location
        p_loc = p_rel @ R  # Transform sang local box coordinate
        h, w, l = obj.dimensions
        mask = (
            (np.abs(p_loc[:, 0]) <= l / 2 + margin)
            & (p_loc[:, 1] >= -h - margin)
            & (p_loc[:, 1] <= margin)
            & (np.abs(p_loc[:, 2]) <= w / 2 + margin)
        )
        obj_pts = points_raw[mask]
        if len(obj_pts) > 0:
            extracted.append({
                "idx": idx,
                "type": obj.type,
                "distance": float(obj.location[2]),
                "bbox": obj.bbox,
                "points": obj_pts,
                "count": len(obj_pts),
            })
    return extracted


def measure_drift_metrics(frame_data: dict, calib_perturbed: KittiCalib,
                          calib_gt: KittiCalib, obj_list: list[dict]) -> dict:
    """Tính toán các chỉ số khi calibration bị thay đổi."""
    points_raw = frame_data["points"]
    img_shape = frame_data["image"].shape
    H, W = img_shape[:2]

    # 1. Toàn bộ điểm
    uv_p, depth_p, mask_p = project_velo_to_image(points_raw, calib_perturbed, img_shape)
    pct_in_fov = float(mask_p.sum() / len(points_raw) * 100.0)

    # 2. Pixel displacement so với GT calib trên các điểm cùng nhìn thấy
    uv_gt, depth_gt, mask_gt = project_velo_to_image(points_raw, calib_gt, img_shape)
    common_mask = mask_p & mask_gt
    if common_mask.sum() > 0:
        # Lấy toạ độ điểm chung trên ảnh
        uv_p_all, _, _ = project_velo_to_image(points_raw[common_mask], calib_perturbed, img_shape)
        uv_gt_all, _, _ = project_velo_to_image(points_raw[common_mask], calib_gt, img_shape)
        diff = np.linalg.norm(uv_p_all - uv_gt_all, axis=1)
        mean_disp_px = float(np.mean(diff))
    else:
        mean_disp_px = 0.0

    # 3. Tỷ lệ điểm rơi đúng vào 2D box
    near_total, near_in = 0, 0
    far_total, far_in = 0, 0
    all_total, all_in = 0, 0

    for item in obj_list:
        obj_pts = item["points"]
        x1, y1, x2, y2 = item["bbox"]
        is_near = item["distance"] < 20.0

        uv_obj, _, _ = project_velo_to_image(obj_pts, calib_perturbed, img_shape)
        if len(uv_obj) > 0:
            in_box = (
                (uv_obj[:, 0] >= x1)
                & (uv_obj[:, 0] <= x2)
                & (uv_obj[:, 1] >= y1)
                & (uv_obj[:, 1] <= y2)
            ).sum()
        else:
            in_box = 0

        total_pts = item["count"]
        all_total += total_pts
        all_in += in_box
        if is_near:
            near_total += total_pts
            near_in += in_box
        else:
            far_total += total_pts
            far_in += in_box

    in_box_near = (near_in / near_total * 100.0) if near_total > 0 else 100.0
    in_box_far = (far_in / far_total * 100.0) if far_total > 0 else 100.0
    in_box_all = (all_in / all_total * 100.0) if all_total > 0 else 100.0

    return {
        "pct_in_fov": pct_in_fov,
        "in_box_near_pct": in_box_near,
        "in_box_far_pct": in_box_far,
        "in_box_all_pct": in_box_all,
        "mean_disp_px": mean_disp_px,
    }


def measure_latency(frame_data: dict, calib: KittiCalib, n_runs: int = 25) -> tuple[float, float, list[float]]:
    """Đo latency projection (bỏ lần chạy đầu), tính p50 và p95."""
    latencies_ms = []
    points = frame_data["points"]
    img_shape = frame_data["image"].shape

    # Lần chạy đầu tiên (warm-up, loại bỏ)
    _ = project_velo_to_image(points, calib, img_shape)

    for _ in range(n_runs):
        t0 = time.perf_counter()
        _ = project_velo_to_image(points, calib, img_shape)
        t1 = time.perf_counter()
        latencies_ms.append((t1 - t0) * 1000.0)

    p50 = float(np.percentile(latencies_ms, 50))
    p95 = float(np.percentile(latencies_ms, 95))
    return p50, p95, latencies_ms


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark định lượng ảnh hưởng của Calibration Drift (Topic A)"
    )
    parser.add_argument("--data-root", default="data/kitti_mini", help="Đường dẫn dataset (kitti_mini)")
    parser.add_argument("--frames", default="000011,000012,000049", help="Danh sách frame IDs cách nhau dấu phẩy")
    parser.add_argument("--out-csv", default="results/calibration_drift_sweep.csv", help="Đường dẫn file CSV kết quả")
    parser.add_argument("--out-fig", default="results/figures/calibration_drift_analysis.png", help="Đường dẫn ảnh đồ thị")
    parser.add_argument("--latency-runs", type=int, default=25, help="Số lần đo latency (mặc định 25)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed cố định")
    args = parser.parse_args()

    np.random.seed(args.seed)
    frame_ids = [f.strip() for f in args.frames.split(",") if f.strip()]
    Path(args.out_csv).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out_fig).parent.mkdir(parents=True, exist_ok=True)

    print(f"=== ĐÁNH GIÁ CALIBRATION DRIFT TRÊN DATASET: {args.data_root} ===")
    print(f"Danh sách frame: {frame_ids}")

    frames_loaded = []
    for fid in frame_ids:
        fr = load_frame(args.data_root, fid)
        objs = extract_object_points(fr["points"], fr["calib"], fr["labels"])
        frames_loaded.append({"frame_id": fid, "data": fr, "objects": objs})
        print(f"Frame {fid}: {len(fr['points'])} points, {len(objs)} vật thể có điểm 3D.")

    # 1. Thí nghiệm 1: Sweep Yaw Drift (0.0° -> 3.0°)
    yaw_levels = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0]
    records = []

    print("\n--- 1. Sweep Yaw Drift ---")
    for yaw in yaw_levels:
        metrics_list = []
        for item in frames_loaded:
            calib_p = perturb_extrinsic(item["data"]["calib"], yaw_deg=yaw)
            m = measure_drift_metrics(item["data"], calib_p, item["data"]["calib"], item["objects"])
            metrics_list.append(m)

        # Tính trung bình trên các frame
        avg_fov = np.mean([m["pct_in_fov"] for m in metrics_list])
        avg_near = np.mean([m["in_box_near_pct"] for m in metrics_list])
        avg_far = np.mean([m["in_box_far_pct"] for m in metrics_list])
        avg_all = np.mean([m["in_box_all_pct"] for m in metrics_list])
        avg_disp = np.mean([m["mean_disp_px"] for m in metrics_list])

        rec = {
            "perturb_type": "yaw",
            "level": yaw,
            "unit": "deg",
            "pct_in_fov": round(avg_fov, 2),
            "in_box_near_pct": round(avg_near, 2),
            "in_box_far_pct": round(avg_far, 2),
            "in_box_all_pct": round(avg_all, 2),
            "mean_disp_px": round(avg_disp, 2),
        }
        records.append(rec)
        print(f"Yaw {yaw:4.1f}° | FOV: {avg_fov:.1f}% | Near (<20m): {avg_near:.1f}% | Far (>=20m): {avg_far:.1f}% | All: {avg_all:.1f}% | Disp: {avg_disp:.1f}px")

    # 2. Thí nghiệm 2: Sweep Pitch Drift (0.0° -> 3.0°)
    pitch_levels = [0.5, 1.0, 1.5, 2.0, 3.0]
    print("\n--- 2. Sweep Pitch Drift ---")
    for pitch in pitch_levels:
        metrics_list = []
        for item in frames_loaded:
            calib_p = perturb_extrinsic(item["data"]["calib"], pitch_deg=pitch)
            m = measure_drift_metrics(item["data"], calib_p, item["data"]["calib"], item["objects"])
            metrics_list.append(m)

        rec = {
            "perturb_type": "pitch",
            "level": pitch,
            "unit": "deg",
            "pct_in_fov": round(np.mean([m["pct_in_fov"] for m in metrics_list]), 2),
            "in_box_near_pct": round(np.mean([m["in_box_near_pct"] for m in metrics_list]), 2),
            "in_box_far_pct": round(np.mean([m["in_box_far_pct"] for m in metrics_list]), 2),
            "in_box_all_pct": round(np.mean([m["in_box_all_pct"] for m in metrics_list]), 2),
            "mean_disp_px": round(np.mean([m["mean_disp_px"] for m in metrics_list]), 2),
        }
        records.append(rec)
        print(f"Pitch {pitch:4.1f}° | In-box All: {rec['in_box_all_pct']:.1f}% | Disp: {rec['mean_disp_px']:.1f}px")

    # 3. Thí nghiệm 3: Sweep Translation Drift (t_x, t_y)
    trans_levels = [0.05, 0.10, 0.20]
    print("\n--- 3. Sweep Translation Drift (t_y mét) ---")
    for ty in trans_levels:
        metrics_list = []
        for item in frames_loaded:
            calib_p = perturb_extrinsic(item["data"]["calib"], t_xyz_m=(0.0, ty, 0.0))
            m = measure_drift_metrics(item["data"], calib_p, item["data"]["calib"], item["objects"])
            metrics_list.append(m)

        rec = {
            "perturb_type": "trans_y",
            "level": ty,
            "unit": "m",
            "pct_in_fov": round(np.mean([m["pct_in_fov"] for m in metrics_list]), 2),
            "in_box_near_pct": round(np.mean([m["in_box_near_pct"] for m in metrics_list]), 2),
            "in_box_far_pct": round(np.mean([m["in_box_far_pct"] for m in metrics_list]), 2),
            "in_box_all_pct": round(np.mean([m["in_box_all_pct"] for m in metrics_list]), 2),
            "mean_disp_px": round(np.mean([m["mean_disp_px"] for m in metrics_list]), 2),
        }
        records.append(rec)
        print(f"Trans_y {ty:4.2f}m | In-box All: {rec['in_box_all_pct']:.1f}% | Disp: {rec['mean_disp_px']:.1f}px")

    # 4. Đo Latency (p50, p95)
    print("\n--- 4. Benchmark Latency Projection ---")
    p50, p95, lats = measure_latency(frames_loaded[0]["data"], frames_loaded[0]["data"]["calib"], args.latency_runs)
    print(f"Hardware: {platform.processor()} | OS: {platform.system()} {platform.release()}")
    print(f"Runs: {len(lats)} (warm-up dropped) | Latency p50: {p50:.2f} ms | p95: {p95:.2f} ms")

    # Xuất ra CSV
    df = pd.DataFrame(records)
    df.to_csv(args.out_csv, index=False)
    print(f"\n-> Đã lưu bảng số liệu vào: {args.out_csv}")

    # 5. Vẽ đồ thị phân tích
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("LiDAR-Camera Projection Calibration Drift Analysis (Topic A)", fontsize=16, fontweight="bold")

    # Panel 1: Yaw vs In-Box Retention (Near vs Far)
    df_yaw = df[df["perturb_type"] == "yaw"]
    ax1 = axes[0, 0]
    ax1.plot(df_yaw["level"], df_yaw["in_box_near_pct"], "o-", color="#1f77b4", linewidth=2, label="Gần (< 20m)")
    ax1.plot(df_yaw["level"], df_yaw["in_box_far_pct"], "s--", color="#d62728", linewidth=2, label="Xa (>= 20m)")
    ax1.plot(df_yaw["level"], df_yaw["in_box_all_pct"], "^-.", color="#2ca02c", linewidth=2, label="Toàn bộ vật thể")
    ax1.axhline(75, color="gray", linestyle=":", label="Ngưỡng cảnh báo (-25%)")
    ax1.axhline(50, color="red", linestyle=":", label="Ngưỡng gãy đổ (-50%)")
    ax1.set_xlabel("Độ lệch Yaw (°)", fontsize=11)
    ax1.set_ylabel("Tỷ lệ điểm rơi trong 2D Box (%)", fontsize=11)
    ax1.set_title("1. Suy giảm điểm trong Box theo góc Yaw (Near vs Far)", fontsize=12, fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="lower left", fontsize=10)

    # Panel 2: So sánh Yaw vs Pitch vs Translation
    ax2 = axes[0, 1]
    df_pitch = df[df["perturb_type"] == "pitch"]
    df_trans = df[df["perturb_type"] == "trans_y"]
    ax2.plot(df_yaw["level"], df_yaw["in_box_all_pct"], "o-", color="#1f77b4", linewidth=2, label="Yaw Drift (°)")
    ax2.plot(df_pitch["level"], df_pitch["in_box_all_pct"], "s-", color="#ff7f0e", linewidth=2, label="Pitch Drift (°)")
    ax2.set_xlabel("Độ lệch góc xoay (°)", fontsize=11)
    ax2.set_ylabel("Tỷ lệ điểm trong Box (%)", fontsize=11)
    ax2.set_title("2. Độ nhạy: Yaw Drift vs Pitch Drift", fontsize=12, fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="lower left", fontsize=10)

    # Panel 3: Độ dịch chuyển Pixel trung bình (Mean Displacement)
    ax3 = axes[1, 0]
    ax3.plot(df_yaw["level"], df_yaw["mean_disp_px"], "o-", color="#9467bd", linewidth=2, label="Yaw Drift")
    ax3.plot(df_pitch["level"], df_pitch["mean_disp_px"], "s-", color="#8c564b", linewidth=2, label="Pitch Drift")
    ax3.set_xlabel("Độ lệch góc xoay (°)", fontsize=11)
    ax3.set_ylabel("Độ dịch chuyển trung bình (Pixels)", fontsize=11)
    ax3.set_title("3. Độ dịch chuyển điểm trên ảnh Camera (Pixels)", fontsize=12, fontweight="bold")
    ax3.grid(True, linestyle="--", alpha=0.6)
    ax3.legend(loc="upper left", fontsize=10)

    # Panel 4: Phân phối thời gian xử lý (Latency)
    ax4 = axes[1, 1]
    ax4.hist(lats, bins=10, color="#2ca02c", edgecolor="black", alpha=0.7)
    ax4.axvline(p50, color="blue", linestyle="--", linewidth=2, label=f"p50 = {p50:.2f} ms")
    ax4.axvline(p95, color="red", linestyle=":", linewidth=2, label=f"p95 = {p95:.2f} ms")
    ax4.set_xlabel("Thời gian xử lý mỗi frame (ms)", fontsize=11)
    ax4.set_ylabel("Tần suất (lần chạy)", fontsize=11)
    ax4.set_title(f"4. Phân bố độ trễ Projection (25 lần lặp, CPU)", fontsize=12, fontweight="bold")
    ax4.grid(True, linestyle="--", alpha=0.6)
    ax4.legend(loc="upper right", fontsize=10)

    plt.tight_layout()
    plt.savefig(args.out_fig, dpi=150)
    plt.close()
    print(f"-> Đã lưu biểu đồ vào: {args.out_fig}")

    # 6. Tạo các ảnh trực quan hóa cho báo cáo
    print("\n--- 5. Xuất các ảnh trực quan cho Báo cáo (Frame 000011) ---")
    f0 = frames_loaded[0]["data"]
    for y_deg in [0.0, 1.0, 2.0, 3.0]:
        cal_p = perturb_extrinsic(f0["calib"], yaw_deg=y_deg)
        uv, depth, _ = project_velo_to_image(f0["points"], cal_p, f0["image"].shape)
        vis = overlay_points(f0["image"], uv, depth)
        for obj in f0["labels"]:
            if obj.type != "DontCare":
                vis = draw_box2d(vis, obj.bbox, label=obj.type)
        out_img_path = Path("results/figures") / f"overlay_000011_yaw_{y_deg:.1f}deg.png"
        cv2.imwrite(str(out_img_path), vis)
        print(f"-> Lưu ảnh: {out_img_path}")

    print("\n=== HOÀN THÀNH TOÀN BỘ THÍ NGHIỆM CP3 ===")


if __name__ == "__main__":
    main()
