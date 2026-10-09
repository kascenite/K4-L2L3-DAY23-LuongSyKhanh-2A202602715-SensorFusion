# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Lương Sỹ Khánh
- MSSV: 2A202602715
- Email:
- Link repo (fork): https://github.com/kascenite/K4-L2L3-DAY23-LuongSyKhanh-2A202602715-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`):

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, frames `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, seed `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision 0.9701, recall 0.7004, tp 519 / fp 16 / fn 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse 0.1503 m, matches 502, sum_sq_err 11.3436, ghost 0, miss 239, mean_confirmed 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse 0.1359 m, matches 502, sum_sq_err 9.2668, ghost 0, miss 239, mean_confirmed 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: Hai mode có cùng `matches` = 502, cùng ghost = 0, cùng miss = 239 và cùng `mean_confirmed_tracks`: tập track confirmed giống hệt nhau, vì chỉ lượt LiDAR sinh, xác nhận và xoá track, còn camera chỉ chỉnh trạng thái EKF. Khác biệt duy nhất là sai số vị trí: RMSE giảm từ 0.1503 m xuống 0.1359 m (`rmse_fused − rmse_lidar` = −0.0145 m, tức fused không xấu hơn LiDAR), `sum_sq_err` giảm từ 11.34 xuống 9.27 m² trên cùng 502 cặp, nên mức giảm thật sự đến từ sai số nhỏ hơn chứ không phải từ việc ghép khác đi. Cả hai mode đạt `precision_track` = 502/(502+0) = 1.0 và `coverage` = 502/519 = 0.967. 239 frame-GT bị miss gồm 222 xe detector bỏ sót (`detection.fn`) cộng khoảng 17 lần xe đã được detect nhưng chưa có track confirmed ở các frame đầu (track mới cần 4 lần hit LiDAR liên tiếp để vượt ngưỡng 0.8 nên frame 0–3 `confirmed` = 0 trong `grade_run.log`). Giới hạn: đo camera là tâm hộp 2D ground-truth cộng nhiễu seed 0, không phải detector ảnh, nên mức cải thiện RMSE của fused chỉ cho thấy EKF dùng được đo thứ hai, không chứng minh chất lượng một camera thật.

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

## Giải thích ngắn (Parts E–H — tự viết)

1. **Đo lidar 3D và camera 2D.** LiDAR cho `z = (x, y, z)` mét trong hệ cảm biến, `R = diag(0.1², 0.1², 0.1²)`, và `h(x)` là phép đổi hệ toạ độ tuyến tính nên `H` (3×6) là hằng số (phần xoay, cột vận tốc bằng 0). Camera cho `z = (u, v)` pixel, `R = diag(5², 5²)` pixel², `h(x)` là phép chiếu pinhole `u = c_i − f_i·y_s/x_s`, `v = c_j − f_j·z_s/x_s` (`camera_fusion.camera_measurement_prediction`), phi tuyến, nên EKF dùng `H` là Jacobian tại trạng thái hiện tại (platform tính bằng chain rule). Camera không đo độ sâu, chỉ ràng buộc hướng nhìn; hai độ lệch chuẩn khác đơn vị nên không thể trộn chung một `R`.
2. **Vì sao gating Mahalanobis.** `d² = γᵀ S⁻¹ γ` chuẩn hoá innovation theo `S = H P Hᵀ + R`: khi `P` lớn (track mới, vận tốc chưa biết, σ=50) cùng khoảng cách Euclid được chấp nhận rộng hơn, khi `P` nhỏ thì chặt hơn; Euclid không phân biệt được điều đó và lại trộn mét với pixel. `d²` tuân theo χ² với `dim_meas` bậc tự do, nên ngưỡng `chi2.ppf(0.995, dim_meas)` (≈ 12.84 cho lidar 3D) giữ 99.5% đo đúng và loại đo ngoại lai trước khi gán greedy/update, tránh một đo sai kéo lệch state hoặc ăn mất track khác. Cặp ngoài FOV bị loại trước khi chiếu vì chiếu điểm sau lưng camera (độ sâu ≤ 1e-6) là không xác định.
3. **Track-then-fuse.** Chỉ có một danh sách track; mỗi frame EKF predict một lần, rồi AssocL (update bằng đo lidar, quản lý track), rồi AssocC (update bằng đo camera, không đổi vòng đời). Không gộp dữ liệu thô trước detection (đó mới là fuse-then-track). Trên log: `grade_run.log` có đúng một record cho mỗi `(mode, frame)`; ở mode `fused` số `confirmed`, `matches`, `ghosts`, `misses` giống hệt mode `lidar` ở mọi frame (tổng matches 502 ở cả hai, ghost 0, miss 239) trong khi `sum_sq_err` khác (ví dụ frame 4: lidar 0.01128, fused 0.03199), đúng là camera chỉ đổi trạng thái chứ không đổi tập track.
4. **Camera lệch calibration.** Extrinsic lệch làm `h(x)` chiếu track ra pixel sai chỗ một lượng có hệ thống, nên innovation `γ = z − h(x)` có trung bình khác 0 (bias, cùng dấu qua nhiều frame và lớn dần với độ lệch/khoảng cách) thay vì dao động quanh 0 với phương sai `S`; `d²` tăng. Nếu lệch vừa, EKF vẫn nhận và kéo state lệch theo (RMSE tăng); nếu lệch lớn thì `d²` vượt ngưỡng χ² (2 bậc tự do) và gating chặn đo, camera không còn đóng góp. Vì camera không đổi score, track không bị xoá oan, nhưng độ chính xác tốt nhất chỉ còn bằng LiDAR. (Phần này chưa kiểm chứng bằng thực nghiệm lệch extrinsic; tôi không làm bonus.)
5. **Sensor tường minh & vai trò LiDAR.** Ở frame LiDAR không có detection, `meas_list` rỗng nên không có `meas.sensor` để suy ra đang ở lượt nào; `associate_and_update` vẫn phải gọi `manager.manage_tracks(unassigned_tracks, [], sensor)` để mọi track trong FOV LiDAR bị trừ `1/window` và track hết score bị xoá (`test_empty_lidar_frame_scores_then_deletes_exhausted_track`). Đồng thời với `sensor=camera` hàm `manage_tracks` trả về ngay, không cộng/trừ score, không xoá, không sinh track. LiDAR quyết định tồn tại vì nó đo vị trí 3D trực tiếp, FOV ±90° và là nguồn duy nhất sinh `Measurement` có kích thước hộp để khởi tạo track; đo camera ở đây là hộp GT có nhiễu 2D, bị che khuất/ngoài ảnh là chuyện thường nên nếu để nó quyết định score thì track sẽ bị trừ oan hoặc sống nhờ đo giả.
6. **Vòng đời.** Track mới: `score = 1/window = 1/6`, `state = initialized`. Mỗi hit LiDAR: `score = min(1, score + 1/6)`; nếu `score > 0.8` thì `confirmed`, ngược lại `tentative` (track đã confirmed không bị hạ). Mỗi miss trong FOV LiDAR: `score −= 1/6`, state giữ nguyên, nên một miss không làm mất confirmed (ví dụ 1.0 → 0.833). Xoá (OR): `P[0,0]` hoặc `P[1,1] > max_P = 9`; hoặc confirmed có `score < 0.6`; hoặc chưa confirmed có `score <= 0`. Từ lúc khởi tạo cần 4 hit nữa (1/6 → 5/6 > 0.8) để confirmed, khớp với `confirmed = 0` ở frame 0–3 trong log, và confirmed bị xoá sau 3 miss liên tiếp (1.0 → 0.5 < 0.6).

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (mô hình Claude Sonnet 5.5)
- Dùng cho phần nào (hàm, câu hỏi, debug): Claude Code viết code Part E–H (`kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`), chạy test và lần chạy Waymo `--fusion compare --seed 0`, và soạn bản nháp phần tóm tắt kết quả cùng 6 câu giải thích ở file này từ `metrics.json`/`grade_run.log` thật. Không có phần nào do tôi tự viết lại; tôi cần đọc, hiểu và tự chịu trách nhiệm trước khi nộp.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` → 128 passed, 0 xfailed; chạy `fusion-run-lab --fusion compare --seed 0` trên frame 0–198, mọi số liệu ở trên chép từ `student/artifacts/metrics.json` và kiểm tra lại từ `grade_run.log`; đối chiếu công thức EKF/Mahalanobis/pinhole với `docs/HUONG_DAN_KY_THUAT.md` và các gợi ý `# vi: TODO`.

## Checklist nộp

- [ ] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [ ] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [ ] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [ ] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [ ] Đã điền đủ file này, gồm khai báo AI
- [ ] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [ ] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
