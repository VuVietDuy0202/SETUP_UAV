# CLAUDE.md

Hướng dẫn cho Claude Code khi làm việc trong dự án này. Đọc file này trước khi thực hiện thay đổi.

## Tổng quan dự án

- **Mục tiêu:** Firmware flight controller cho UAV (drone).
- **Phần cứng:** ESP32-S3 (board `esp32-s3-devkitm-1`).
- **Framework:** Arduino trên PlatformIO (`platform = espressif32@6.9.0`).
- **Cảm biến/ngoại vi:** IMU MPU (I2C), receiver RC, ESC/motor qua ESP32Servo.

> ⚠️ **AN TOÀN:** Đây là firmware điều khiển motor thật. Luôn giả định người dùng đang test với phần cứng có điện. Không bao giờ đề xuất chạy motor ở throttle cao khi chưa tháo cánh quạt. Ưu tiên safety check (arming, failsafe, timeout).

## Cấu trúc thư mục

```
src/
  main.cpp              # Vòng lặp chính: đọc receiver, đọc IMU, PID, xuất ESC
  receiver/             # Đọc tín hiệu RC (rx_config, getReceiverValues, ...)
  calib_mpu/            # Hiệu chỉnh gyro/accel MPU
  test_motor/           # Test motor riêng lẻ
include/common.h        # #define chân (pin), biến extern dùng chung giữa các module
lib/                    # Thư viện nội bộ (mỗi lib 1 thư mục)
test/                   # Unit test (PlatformIO test)
platformio.ini          # Cấu hình board, framework, lib_deps
```

## Lệnh thường dùng

```bash
pio run                              # Build
pio run -t upload                    # Build + nạp firmware
pio run -t upload -t monitor         # Nạp + mở serial monitor
pio device monitor -b 115200         # Mở serial monitor
pio run -t clean                     # Dọn build
```

## Quy tắc code

- **Ngôn ngữ:** C++ (Arduino style). Comment có thể viết tiếng Việt hoặc tiếng Anh, nhưng giữ nhất quán với file đang sửa.
- **Chân (pin) & hằng số:** khai báo tập trung trong `include/common.h` bằng `#define`. Không hard-code số chân rải rác trong logic.
- **Biến dùng chung giữa các module:** khai báo `extern` trong header, định nghĩa 1 lần trong `main.cpp`.
- **Module mới:** tạo thư mục riêng trong `src/<tên>/` với cặp `.h` + `.cpp`, expose hàm qua header.
- **Kiểu dữ liệu:** dùng kiểu cố định (`int16_t`, `uint8_t`, ...) thay vì `int` chung chung khi liên quan tới thanh ghi/cảm biến.
- **Không blocking trong vòng lặp bay:** tránh `delay()` dài trong loop điều khiển; giữ chu kỳ loop ổn định (thời gian thực).
- **Không đổi tần số loop / thứ tự đọc cảm biến** nếu không được yêu cầu — nó ảnh hưởng tới độ ổn định bay.

## Khi Claude làm việc

- Sửa đúng phạm vi được yêu cầu; không tự refactor rộng nếu chưa hỏi.
- Trước khi commit thay đổi firmware: chạy `pio run` để chắc chắn build sạch.
- Nếu thay đổi liên quan pin/PID/failsafe, nêu rõ rủi ro cho người dùng.
- Giữ nguyên phần "Terms of use" / "Safety note" ở đầu `main.cpp`.
- Không commit thư mục `.pio/` (đã có trong `.gitignore`).

## Git

- Branch chính: `main`.
- Commit message ngắn gọn, mô tả thay đổi (tiếng Việt hoặc Anh đều được).
- Không commit file build/output (`build_output.log`, `.pio/`).

## Chưa rõ thì hỏi

Nếu yêu cầu ảnh hưởng tới thông số bay (PID, giới hạn throttle, failsafe, tần số loop) và không chắc chắn, hãy hỏi lại người dùng trước khi sửa.
