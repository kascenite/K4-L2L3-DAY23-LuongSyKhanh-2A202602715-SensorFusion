# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Lương Sỹ Khánh
- MSSV: 2A202602715
- Email: khanh.sylg@gmail.com
- Link repo (fork): https://github.com/kascenite/K4-L2L3-DAY23-LuongSyKhanh-2A202602715-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): 8e119df97b4dc41ba6c19dad8a2e1ae0c67b17cd

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, frames `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, seed `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision 0.9701, recall 0.7004, tp 519 / fp 16 / fn 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse 0.1503 m, matches 502, sum_sq_err 11.3436, ghost 0, miss 239, mean_confirmed 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse 0.1359 m, matches 502, sum_sq_err 9.2668, ghost 0, miss 239, mean_confirmed 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: Hai mode có cùng 502 matches, 0 ghost, 239 misses và tập track confirmed; LiDAR quản lý track, camera chỉ cập nhật EKF. Fused giảm RMSE từ 0.1503 xuống 0.1359 m trên cùng các cặp ghép (sum squared error: 11.34 → 9.27 m²). Precision track là 1.0, coverage là 0.967; 239 misses gồm 222 detection bỏ sót và khoảng 17 frame track chưa được xác nhận. Camera dùng tâm hộp ground-truth có nhiễu, không phải detector ảnh, nên kết quả không đại diện cho hiệu quả camera thực.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E-H — tự viết)

1. **Đo lidar và camera.** LiDAR đo vị trí 3D nên hàm đo tuyến tính, `H` hằng số và `R` tính bằng m². Camera đo pixel qua phép chiếu pinhole phi tuyến nên EKF dùng Jacobian. Camera không đo độ sâu và `R` tính bằng pixel².
2. **Gating Mahalanobis.** `d² = γᵀS⁻¹γ` xét cả innovation lẫn độ bất định `S = HPHᵀ + R`, không như khoảng cách Euclid. So với ngưỡng χ² theo số chiều đo giúp loại ghép sai trong khi đo ngoài FOV hoặc có độ sâu không hợp lệ bị bỏ qua.
3. **Track-then-fuse.** Mỗi frame predict một lần, LiDAR ghép và quản lý vòng đời và sau đó camera chỉ cập nhật trạng thái. Log cho thấy hai mode có cùng 502 matches, 0 ghost, 239 misses nhưng fused có RMSE thấp hơn (0.1359 m so với 0.1503 m).
4. **Camera lệch calibration.** Lệch extrinsic tạo innovation có bias: lệch nhỏ có thể làm state và RMSE xấu đi; lệch lớn bị Mahalanobis gating loại. Camera không quản lý vòng đời nên track vẫn do LiDAR quyết định. Chưa kiểm chứng bằng thực nghiệm.
5. **Sensor tường minh và vai trò LiDAR.** Khi không có detection, danh sách đo rỗng nên phải truyền rõ sensor để xử lý miss LiDAR. Chỉ LiDAR cập nhật score, tạo và xoá track, còn camera chỉ cập nhật trạng thái vì phép đo 2D nhiễu không đủ tin cậy để quyết định vòng đời.
6. **Vòng đời.** Track bắt đầu với score `1/6`; cần 4 hit LiDAR tiếp theo để vượt `0.8` và được xác nhận. Miss trong FOV trừ `1/6`; track bị xoá nếu covariance vượt ngưỡng, confirmed có score `< 0.6`, hoặc tentative có score `<= 0`.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code
- Dùng cho phần nào (hàm, câu hỏi, debug): Claude Code viết code Part E-H, chạy test và lần chạy Waymo `--fusion compare --seed 0`, soạn bản nháp phần tóm tắt kết quả cùng 6 câu giải thích ở file này từ `metrics.json`/`grade_run.log`.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): chạy `pytest student/tests -q` có 128 passed, 0 xfailed; chạy `fusion-run-lab --fusion compare --seed 0` trên frame 0-198, số liệu ở trên chép từ `student/artifacts/metrics.json` và kiểm tra lại `grade_run.log`; đối chiếu công thức EKF/Mahalanobis/pinhole với `docs/HUONG_DAN_KY_THUAT.md` và các gợi ý `# vi: TODO`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [ ] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
