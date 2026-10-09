# Báo cáo Day 6: Kiểm tra Calibration LiDAR-Camera bằng Projection và Phân tích Drift

- **Họ tên:** Đỗ Mạnh Nghĩa
- **MSSV:** 2A202602971
- **Lớp:** VinUni AI20K - Track 4
- **Link repo:** https://github.com/domanhnghiaforwork-cloud/K4-Track4-Day06-DoManhNghia-2A202602971-3D-From-Point-Clouds
- **Topic:** A — Kiểm tra calibration LiDAR-camera bằng projection
- **Dataset:** data/kitti_mini, data/synthetic, data/nuscenes_mini_subset
- **Các frame đã dùng:** 000011, 000012, 000020 (KITTI), 000000 (synthetic), scene-0103_010 (nuScenes)

> Hãy viết ngắn: mỗi mục từ 3 đến 8 dòng, ưu tiên số liệu và hình ảnh.

## 1. Claim

Độ lệch góc xoay (extrinsic calibration drift) theo trục yaw từ 1.0° trở lên làm trên 25% điểm LiDAR thuộc vật thể trượt ra ngoài bounding box 2D của xe ở cự ly trên 20 m; khi góc lệch đạt 2.0°, tỷ lệ điểm rơi đúng vào 2D box sụt giảm trên 50%, phá vỡ tính tương thích không gian giữa cảm biến LiDAR và camera.


## 2. Evidence

Bảng hoặc plot số liệu, kèm ảnh/video demo. Ghi rõ đường dẫn file trong `results/`.

| Cấu hình / mức perturb | Metric 1 | Metric 2 | Ghi chú |
|---|---|---|---|
| [ĐIỀN] | | | |

![demo](../results/figures/[ĐIỀN].png)

## 3. Failure case

Nêu khi nào hệ thống hoặc phương pháp fail, vì sao fail, và liên hệ tới lớp nào trong 6 lớp debug: I/O, Geometry, Time, Preprocess, Model, Metric.

![failure](../results/figures/fail_[ĐIỀN].png)

[ĐIỀN]

## 4. Khuyến nghị nếu triển khai thật

Use-case cụ thể (ADAS / robot / drone), trade-off và bước tiếp theo.

[ĐIỀN]

## 5. Cách chạy lại

Các lệnh tái tạo lại toàn bộ kết quả từ repo sạch:

```bash
# 1. Chạy demo chiếu điểm LiDAR lên ảnh camera (CP2)
python -m starter.projection --data-root data/synthetic --frame 000000
python -m starter.projection --data-root data/kitti_mini --frame 000011
python -m starter.projection --data-root data/nuscenes_mini_subset --frame scene-0103_010
```

## 6. Khai báo sử dụng AI

Ghi rõ đã dùng công cụ AI nào, dùng vào việc gì, và bạn đã tự kiểm chứng kết quả đó bằng cách nào. Nếu không dùng AI, ghi "Không sử dụng". Xem quy định ở `RULES.md` mục 2.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| [ĐIỀN] | | |
