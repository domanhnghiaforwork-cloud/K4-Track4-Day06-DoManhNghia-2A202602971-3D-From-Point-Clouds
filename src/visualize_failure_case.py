"""Tạo ảnh minh hoạ chi tiết cho Failure Case (CP4):
So sánh trực quan giữa Calibration Chuẩn (0.0°) và Calibration Bị Lệch (Yaw Drift 2.0°).
Tập trung zoom cận cảnh vào xe ở cự ly 26.6m và người đi bộ ở cự ly 34.1m (Frame 000011).
"""
from __future__ import annotations

import sys
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

from starter.datasets import load_frame
from starter.projection import (
    draw_box2d,
    overlay_points,
    perturb_extrinsic,
    project_velo_to_image,
)


def generate_failure_visualization(frame_id: str = "000011",
                                   out_path: str = "results/figures/fail_01_yaw_drift_mismatch.png"):
    fr = load_frame("data/kitti_mini", frame_id)
    img_bgr = fr["image"]
    calib_gt = fr["calib"]
    calib_fail = perturb_extrinsic(calib_gt, yaw_deg=2.0)

    # 1. Overlay với GT calib
    uv_gt, depth_gt, _ = project_velo_to_image(fr["points"], calib_gt, img_bgr.shape)
    vis_gt = overlay_points(img_bgr, uv_gt, depth_gt)

    # 2. Overlay với Yaw drift 2.0 deg
    uv_fail, depth_fail, _ = project_velo_to_image(fr["points"], calib_fail, img_bgr.shape)
    vis_fail = overlay_points(img_bgr, uv_fail, depth_fail)

    # Vẽ bounding box lên cả 2 ảnh
    for obj in fr["labels"]:
        if obj.type != "DontCare":
            vis_gt = draw_box2d(vis_gt, obj.bbox, color=(0, 255, 0), label=f"{obj.type} ({obj.location[2]:.1f}m)")
            vis_fail = draw_box2d(vis_fail, obj.bbox, color=(0, 0, 255), label=f"{obj.type} [MISMATCH]")

    # BGR to RGB cho Matplotlib
    vis_gt_rgb = cv2.cvtColor(vis_gt, cv2.COLOR_BGR2RGB)
    vis_fail_rgb = cv2.cvtColor(vis_fail, cv2.COLOR_BGR2RGB)

    # Vùng Crop Zoom-in (tập trung vào Car 26.6m và Pedestrian 34.1m: x: 420-720, y: 130-280)
    y1, y2, x1, x2 = 130, 280, 420, 720
    crop_gt = vis_gt_rgb[y1:y2, x1:x2]
    crop_fail = vis_fail_rgb[y1:y2, x1:x2]

    # Tạo biểu đồ đối sánh 2 hàng x 2 cột
    fig, axes = plt.subplots(2, 2, figsize=(15, 9), gridspec_kw={"height_ratios": [1.2, 1]})
    fig.suptitle("FAILURE CASE: Lệch Extrinsic Calibration (Yaw Drift +2.0°) Gây Phá Vỡ Không Gian Chiếu",
                 fontsize=14, fontweight="bold", color="darkred")

    # Toàn cảnh GT
    axes[0, 0].imshow(vis_gt_rgb)
    axes[0, 0].set_title("[A] Toàn cảnh: Calibration Chuẩn (Yaw = 0.0°)\nĐiểm LiDAR bám khít hình học vật thể", fontsize=11)
    axes[0, 0].axis("off")

    # Toàn cảnh Fail
    axes[0, 1].imshow(vis_fail_rgb)
    axes[0, 1].set_title("[B] Toàn cảnh: Calibration Drift (Yaw = +2.0°)\nĐiểm LiDAR bị lệch ngang sang phải ~31 pixels", fontsize=11, color="red")
    axes[0, 1].axis("off")

    # Zoom-in GT
    axes[1, 0].imshow(crop_gt)
    axes[1, 0].set_title("[C] Zoom Cận Cảnh (Chuẩn): Điểm đỏ/vàng nằm trọn trong xe (26.6m) & người (34.1m)", fontsize=11)
    axes[1, 0].axis("off")

    # Zoom-in Fail
    axes[1, 1].imshow(crop_fail)
    axes[1, 1].set_title("[D] Zoom Cận Cảnh (Fail): Điểm LiDAR trượt ra ngoài 2D Box -> Mất 57% điểm xe, 100% điểm người!", fontsize=11, color="red")
    axes[1, 1].axis("off")

    plt.tight_layout()
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=160)
    plt.close()
    print(f"-> Đã tạo ảnh Failure Case tại: {out_path}")


if __name__ == "__main__":
    generate_failure_visualization()
