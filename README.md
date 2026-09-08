# SETUP_UAV — Firmware kiểm tra & hiệu chỉnh drone

> **UAV VVD — HUS-VNU** · Tài liệu dành cho học sinh

Firmware chạy trên ESP32-S3, dùng để **kiểm tra và hiệu chỉnh từng bộ phận của drone
trước khi nạp firmware bay**. Toàn bộ thao tác qua **Serial Monitor**: gõ một ký tự để
chạy một bài kiểm tra.

Đây **không phải** firmware bay — không có PID, không có vòng điều khiển ổn định. Mục
tiêu của nó là trả lời câu hỏi: *"Từng bộ phận của drone có hoạt động đúng không?"*

---

## Mục lục

**Phần I — Sử dụng**
1. [⚠️ An toàn](#1-️-an-toàn--đọc-trước-khi-cắm-pin)
2. [Phần cứng & sơ đồ chân](#2-phần-cứng--sơ-đồ-chân)
3. [Cài đặt & nạp firmware](#3-cài-đặt--nạp-firmware)
4. [Bảng lệnh Serial](#4-bảng-lệnh-serial)
5. [Quy trình kiểm tra 5 bước](#5-quy-trình-kiểm-tra-5-bước)

**Phần II — Hiểu code**

6. [Kiến thức nền cần có](#6-kiến-thức-nền-cần-có)
7. [Kiến trúc tổng thể](#7-kiến-trúc-tổng-thể)
8. [Khối `common.h` — Bộ khai báo chung](#8-khối-commonh--bộ-khai-báo-chung)
9. [Khối `main.cpp` — Bộ não điều phối](#9-khối-maincpp--bộ-não-điều-phối)
10. [Khối `receiver` — Đọc tay điều khiển](#10-khối-receiver--đọc-tay-điều-khiển)
11. [Khối `calib_mpu` — Cảm biến & tính góc](#11-khối-calib_mpu--cảm-biến--tính-góc)
12. [Khối `test_motor` — Điều khiển động cơ](#12-khối-test_motor--điều-khiển-động-cơ)
13. [Xử lý sự cố](#13-xử-lý-sự-cố)
14. [Bài tập tự làm](#14-bài-tập-tự-làm)

---
---

# PHẦN I — SỬ DỤNG

## 1. ⚠️ An toàn — đọc trước khi cắm pin

> **THÁO CÁNH QUẠT** trước mọi thao tác có cấp nguồn cho ESC. Không có ngoại lệ.
>
> Một cánh quạt drone quay ở 10.000 vòng/phút có thể cắt đứt gân tay.

Firmware này điều khiển motor thật. Ba điểm bắt buộc phải biết:

### 1. Motor có thể quay NGAY khi cấp nguồn

Sau khi khởi động, vòng lặp chính đưa **tay ga (kênh 3) thẳng ra cả 4 motor**, không
qua bước arming nào ([`src/main.cpp:155`](src/main.cpp#L155)). Nếu tay ga trên tay điều
khiển đang ở vị trí cao lúc bạn cắm pin, cả 4 motor sẽ quay ngay lập tức.

**Quy tắc:** bật tay điều khiển và hạ tay ga xuống thấp nhất **trước** khi cắm pin.

Chỉ cần **gõ bất kỳ ký tự nào vào Serial Monitor**, chế độ truyền thẳng này sẽ tắt
(`disable_throttle = 1`) và cả 4 motor bị ép về 1000 µs cho đến khi khởi động lại.

### 2. Không có failsafe khi mất sóng

Nếu tay điều khiển tắt hoặc mất sóng giữa chừng, firmware **giữ nguyên giá trị kênh
cuối cùng đọc được** — motor sẽ tiếp tục quay ở mức đó. Luôn để tay gần giắc pin.

### 3. Đừng chạy liên tục quá ~70 phút

Bộ đếm `micros()` tràn sau khoảng 71 phút, có thể làm vòng lặp test bị treo trong khi
motor giữ nguyên lệnh cuối. Rút pin và khởi động lại giữa các buổi thực hành.

---

## 2. Phần cứng & sơ đồ chân

| Thành phần | Chi tiết |
|---|---|
| Vi điều khiển | ESP32-S3 (board `esp32-s3-devkitm-1`) |
| Cảm biến | MPU-6050 (I2C, địa chỉ `0x68`) |
| Điều khiển xa | Receiver RC 6 kênh, tín hiệu PWM |
| Động lực | 4 × ESC + motor (giao tiếp qua thư viện ESP32Servo) |

### Sơ đồ chân

| Chức năng | GPIO | Khai báo tại |
|---|---|---|
| LED đỏ | 2 | [`include/common.h:9`](include/common.h#L9) |
| LED xanh | 3 | [`include/common.h:10`](include/common.h#L10) |
| Motor 1 (trước phải, CCW) | 4 | [`src/test_motor/test_motor.h:8`](src/test_motor/test_motor.h#L8) |
| Motor 2 (sau phải, CW) | 5 | [`src/test_motor/test_motor.h:9`](src/test_motor/test_motor.h#L9) |
| Motor 3 (sau trái, CCW) | 6 | [`src/test_motor/test_motor.h:10`](src/test_motor/test_motor.h#L10) |
| Motor 4 (trước trái, CW) | 7 | [`src/test_motor/test_motor.h:11`](src/test_motor/test_motor.h#L11) |
| I2C SCL | 10 | [`src/main.cpp:94`](src/main.cpp#L94) |
| I2C SDA | 11 | [`src/main.cpp:94`](src/main.cpp#L94) |
| RC kênh 1 (Roll) | 37 | [`src/receiver/receiver.cpp:6`](src/receiver/receiver.cpp#L6) |
| RC kênh 2 (Pitch) | 38 | 〃 |
| RC kênh 3 (Throttle) | 39 | 〃 |
| RC kênh 4 (Yaw) | 40 | 〃 |
| RC kênh 5 | 41 | 〃 |
| RC kênh 6 | 42 | 〃 |

### Bố trí motor

Nhìn từ trên xuống, mũi drone hướng lên:

```
        MŨI
   M4          M1
  (CW)        (CCW)
      \      /
       \    /
        \  /
        /  \
       /    \
      /      \
   M3          M2
  (CCW)       (CW)
        ĐUÔI
```

> **Tại sao 2 motor quay chiều này, 2 motor quay chiều kia?**
>
> Mỗi motor quay tạo ra một **mô-men phản lực** (định luật III Newton) làm thân drone
> có xu hướng xoay ngược lại. Nếu cả 4 motor cùng chiều, drone sẽ tự xoay tít quanh
> trục đứng. Bố trí 2 CW + 2 CCW theo đường chéo giúp các mô-men **triệt tiêu nhau**.
>
> Muốn drone xoay (yaw) sang phải? Tăng tốc 2 motor CCW và giảm 2 motor CW — tổng lực
> nâng không đổi nhưng mô-men mất cân bằng làm drone xoay.

---

## 3. Cài đặt & nạp firmware

Cần [PlatformIO](https://platformio.org/) (khuyến nghị cài qua extension của VS Code).

```bash
# Build
pio run -e esp32-s3-devkitm-1

# Build + nạp
pio run -e esp32-s3-devkitm-1 -t upload

# Mở Serial Monitor
pio device monitor -b 57600
```

> ### ⚠️ Hai lưu ý về lệnh build
>
> **1. Baud rate là 57600, không phải 115200.**
> Firmware dùng `Serial.begin(57600)` nhưng `platformio.ini` khai báo
> `monitor_speed = 115200`. Nếu mở monitor bằng `pio device monitor` không có tham số,
> hoặc bằng nút Monitor của VS Code, **màn hình sẽ toàn ký tự rác**. Luôn thêm `-b 57600`.
>
> **2. Luôn chỉ định `-e esp32-s3-devkitm-1`.**
> Dự án khai báo 2 môi trường. Nếu chạy `pio run` không tham số, PlatformIO build cả
> môi trường `esp32dev` (ESP32 đời cũ) — môi trường này **không dùng được** vì các chân
> RC 40, 41, 42 không tồn tại trên ESP32 classic.

---

## 4. Bảng lệnh Serial

Sau khi nạp xong, mở Serial Monitor và gõ **một ký tự** (không cần Enter):

| Phím | Chức năng | Cần cắm pin? |
|:---:|---|:---:|
| `b` | Quét bus I2C — tìm thiết bị | Không |
| `c` | Đọc giá trị gyro thô | Không |
| `d` | Đọc giá trị accelerometer thô | Không |
| `h` | Hiệu chỉnh gyro + accel (thủ công) | Không |
| `a` | Đọc tín hiệu 6 kênh receiver | Không |
| `e` | Đọc góc nghiêng Pitch / Roll / Yaw | Không |
| `1` | Test **motor 1** — trước phải, CCW | ⚠️ **Có** |
| `2` | Test **motor 2** — sau phải, CW | ⚠️ **Có** |
| `3` | Test **motor 3** — sau trái, CCW | ⚠️ **Có** |
| `4` | Test **motor 4** — trước trái, CW | ⚠️ **Có** |
| `5` | Test **cả 4 motor** + đo độ rung | ⚠️ **Có** |
| `q` | Thoát bài test đang chạy | — |

---

## 5. Quy trình kiểm tra 5 bước

Làm **đúng thứ tự**. Mỗi bước chỉ chuyển sang bước sau khi đã đạt kết quả mong đợi.

### Bước 1 — Kiểm tra IMU *(chỉ cắm USB, KHÔNG cắm pin)*

**`b` — Quét I2C.** Kết quả đúng:

```
I2C device found at address 0x68
```

Nếu không thấy gì → kiểm tra dây SDA (GPIO 11), SCL (GPIO 10), nguồn 3.3 V và GND.

**`c` — Gyro thô.** Đặt drone nằm yên hoàn toàn.

Kết quả đúng: cả 3 trục dao động quanh **0** (±10 là bình thường). Xoay drone bằng tay
→ trục tương ứng đổi rõ rệt và trở về 0 khi dừng.

**`d` — Accelerometer thô.** Đặt drone nằm phẳng.

Kết quả đúng: `ACC_z` ≈ **4096** (tương ứng 1 g — chính là trọng lực), `ACC_x` và
`ACC_y` ≈ 0. Lật nghiêng 90° → trục tương ứng lên ~4096, trục Z về ~0.

### Bước 2 — Hiệu chỉnh IMU

**`h` — Hiệu chỉnh thủ công.** Đặt drone **nằm phẳng và tuyệt đối yên**.

Mất khoảng **24 giây** (2000 mẫu làm nóng + 4000 mẫu đo, LED xanh nhấp nháy). Không
chạm vào drone trong suốt quá trình.

Kết quả in ra:

```
manual_acc_pitch_cal_value = 196
manual_acc_roll_cal_value = 3
manual_gyro_pitch_cal_value = -14
manual_gyro_roll_cal_value = 61
manual_gyro_yaw_cal_value = 9
```

> #### 📋 Phải chép tay các giá trị này vào code
>
> Firmware **không tự lưu** kết quả hiệu chỉnh. Bạn phải:
>
> 1. Chép 5 giá trị vừa in vào [`src/main.cpp`](src/main.cpp#L27-L33) (dòng 27–33).
> 2. Đổi `use_manual_calibration` thành **`true`** ([`src/main.cpp:30`](src/main.cpp#L30)).
> 3. Build và nạp lại firmware.
>
> Nếu bỏ qua bước 2, các bài `c` và `e` sẽ **tự hiệu chỉnh lại từ đầu** mỗi lần chạy và
> phớt lờ giá trị bạn vừa chép vào — đây là nguyên nhân phổ biến của tình trạng
> *"em đã hiệu chỉnh rồi mà không thấy gì thay đổi"*.

### Bước 3 — Kiểm tra tay điều khiển *(vẫn chưa cắm pin)*

**`a` — Đọc tín hiệu receiver.** Bật tay điều khiển trước. Mỗi 250 ms in một dòng:

```
Start:0  Roll:-+-1500  Pitch:-+-1500  Throttle:vvv1000  Yaw:-+-1500  CH5:1000  CH6:1000
```

Cần kiểm tra:

- Cả 4 kênh chính về **~1500** khi cần điều khiển ở giữa.
- Tay ga kéo hết xuống → **~1000**, đẩy hết lên → **~2000**.
- Gạt cần **Roll sang phải** → chỉ số tăng và hiện `>>>`; sang trái hiện `<<<`.
- Đẩy cần **Pitch về trước** → hiện `^^^`; kéo về sau hiện `vvv`.
- Nếu một kênh **đứng im ở 1500 hoặc 1000** → dây tín hiệu kênh đó chưa nối đúng GPIO.

Cột `Start` thể hiện chuỗi arming (chỉ hiển thị, chưa điều khiển motor):
`0` → hạ ga + yaw trái → `1` → yaw về giữa → `2` (đã arm).

### Bước 4 — Kiểm tra góc nghiêng *(vẫn chưa cắm pin)*

**`e` — Đọc góc.** Nếu chưa bật `use_manual_calibration`, firmware tự hiệu chỉnh gyro
trong ~8 giây trước — giữ drone đứng yên.

```
Pitch: 0.3 Roll: -0.1 Yaw: 0 Temp: 28.4
```

Cần kiểm tra:

- Đặt phẳng → Pitch và Roll đều **gần 0°**.
- Nghiêng ~45° → đọc ~45°, dấu nhất quán với hướng nghiêng.
- Xoay quanh trục đứng → **Yaw** (đơn vị **°/s**, không phải góc tích lũy) khác 0 khi
  đang xoay, về 0 khi dừng.
- `Temp` ~25–35 °C. Ra số vô lý → sai giao tiếp I2C.

### Bước 5 — ⚠️ Kiểm tra motor *(THÁO CÁNH QUẠT, rồi mới cắm pin)*

> **Kiểm tra lần cuối trước khi cắm pin:**
>
> - [ ] Đã **tháo hết 4 cánh quạt**
> - [ ] Tay điều khiển đã **bật**, tay ga ở **vị trí thấp nhất**
> - [ ] Drone được giữ chặt hoặc đặt trên bề mặt không trơn
> - [ ] Không có ai đứng trong tầm với của drone

Gõ `1`–`4` để test từng motor. Firmware chờ 2.5 giây rồi vào bài test.

Bài test **yêu cầu tay ga dưới 1050 µs** mới bắt đầu. Nếu chưa, nó in:

```
Throttle is not in the lowest position.
Throttle value is: 1520
Waiting 10 seconds:....
```

và cho 10 giây để hạ ga. Hết 10 giây mà tay ga vẫn cao → bài test **tự huỷ**. Đây là
cơ chế an toàn, không phải lỗi.

Khi đã vào test, **đẩy tay ga từ từ**. Cần xác nhận:

- **Đúng motor** quay theo sơ đồ (bấm `1` → motor trước phải quay).
- **Đúng chiều quay**. Sai chiều → đảo 2 trong 3 dây nối giữa ESC và motor.

Gõ `5` để chạy **cả 4 motor** và **đo độ rung**:

| Giá trị | Ý nghĩa |
|---|---|
| < 10 | Tốt — khung và motor cân bằng |
| 10 – 30 | Chấp nhận được, kiểm tra lại độ chặt của ốc |
| > 30 | **Có vấn đề** — motor cong trục, bạc đạn hỏng, hoặc khung lỏng |

---
---

# PHẦN II — HIỂU CODE

## 6. Kiến thức nền cần có

Bốn khái niệm dưới đây xuất hiện xuyên suốt firmware. Đọc kỹ phần này trước sẽ giúp các
phần sau dễ hiểu hơn nhiều.

### 6.1. Tín hiệu PWM trong RC — "độ rộng xung là giá trị"

Receiver RC và ESC đều nói chuyện bằng cùng một ngôn ngữ: **độ rộng xung**.

```
Điện áp
  ↑
3V│  ┌──────┐                      ┌──────┐
  │  │      │                      │      │
0V└──┘      └──────────────────────┘      └────►  thời gian
     │◄────►│                      │
      1000-2000 µs                 │
     │◄─────── 20 ms (50 Hz) ──────►│
```

- Cứ **20 ms** (50 lần/giây) lại có một xung.
- **Độ rộng** của xung mới mang thông tin, không phải điện áp hay tần số:

| Độ rộng | Ý nghĩa (cần điều khiển) | Ý nghĩa (ESC) |
|---|---|---|
| 1000 µs | Hết về một phía / ga nhỏ nhất | Motor dừng |
| 1500 µs | Ở giữa | Motor ~50% |
| 2000 µs | Hết về phía kia / ga lớn nhất | Motor tối đa |

Vì vậy toàn bộ code làm việc với **con số 1000–2000**, và nhiệm vụ của khối `receiver`
chỉ là: *đo xem xung dài bao nhiêu micro-giây*.

### 6.2. Giá trị thô (raw) và hệ số quy đổi

Cảm biến MPU-6050 không trả về "độ" hay "g". Nó trả về **số nguyên 16 bit** (từ −32768
đến +32767). Muốn ra đơn vị vật lý phải chia cho **hệ số nhạy** (sensitivity).

Firmware cấu hình cảm biến ở thang đo:

| Thông số | Thang đo | Hệ số quy đổi | Nghĩa là |
|---|---|---|---|
| Gyro | ±500 °/s | **65.5 LSB / (°/s)** | Đọc được 655 → đang xoay 10 °/s |
| Accel | ±8 g | **4096 LSB / g** | Đọc được 4096 → chịu gia tốc 1 g |

> **Vì sao chọn thang đo rộng?** Thang càng hẹp thì càng chính xác nhưng dễ "bão hoà"
> (vượt ngưỡng đo). Drone khi bay có thể xoay rất nhanh và rung mạnh, nên chọn ±500 °/s
> và ±8 g để không bị mất dữ liệu ở những lúc quan trọng nhất.

### 6.3. Gyro và Accel — hai cảm biến bù khuyết cho nhau

Đây là ý tưởng cốt lõi của mọi flight controller:

| | **Gyroscope** (con quay hồi chuyển) | **Accelerometer** (gia tốc kế) |
|---|---|---|
| Đo cái gì | **Tốc độ xoay** (°/s) | **Gia tốc** (bao gồm cả trọng lực) |
| Muốn ra góc thì | Phải **cộng dồn theo thời gian** (tích phân) | Tính trực tiếp từ hướng trọng lực |
| Ưu điểm | Rất mượt, phản ứng nhanh, không bị nhiễu bởi rung | Không bao giờ trôi — trọng lực luôn hướng xuống |
| Nhược điểm | **Trôi (drift)** — sai số nhỏ cộng dồn thành sai số lớn | Nhiễu nặng khi motor rung hoặc drone tăng tốc |

Không cảm biến nào dùng một mình được. Giải pháp: **bộ lọc bù** (mục 11.4).

### 6.4. Vòng lặp thời gian thực 250 Hz

Firmware chạy các bài test ở **đúng 250 lần mỗi giây** (chu kỳ 4000 µs = 4 ms):

```cpp
loop_timer = micros() + 4000;   // Đặt mốc: 4 ms nữa

// ... làm mọi việc: đọc cảm biến, tính toán, xuất ra motor ...

while (loop_timer > micros());  // Chờ cho đủ 4 ms rồi mới lặp tiếp
```

**Tại sao phải cố định tần số?** Vì công thức tích phân gyro dùng `dt = 0.004 s` như
một **hằng số**. Nếu vòng lặp lúc nhanh lúc chậm, góc tính ra sẽ sai. Đây cũng là lý do
quy tắc dự án cấm dùng `delay()` dài trong vòng lặp điều khiển.

---

## 7. Kiến trúc tổng thể

### 7.1. Bốn khối và quan hệ giữa chúng

```
                     ┌────────────────────────┐
                     │      include/          │
                     │       common.h         │   ← Khai báo dùng chung
                     │  #define chân, extern  │      (mọi khối đều include)
                     └───────────┬────────────┘
                                 │
         ┌───────────────────────┼───────────────────────┐
         │                       │                       │
┌────────▼────────┐   ┌──────────▼─────────┐   ┌─────────▼────────┐
│    receiver/    │   │     calib_mpu/     │   │   test_motor/    │
│                 │   │                    │   │                  │
│ Đọc xung PWM    │   │ Đọc MPU-6050 (I2C) │   │ Xuất xung ra ESC │
│ từ 6 kênh RC    │   │ Hiệu chỉnh, tính   │   │ Đo độ rung       │
│ bằng ngắt (ISR) │   │ góc nghiêng        │   │                  │
│                 │   │                    │   │                  │
│ Bài test: 'a'   │   │ Bài: b,c,d,e,h     │   │ Bài test: 1-5    │
└────────┬────────┘   └──────────┬─────────┘   └─────────┬────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                     ┌───────────▼────────────┐
                     │       src/main.cpp     │
                     │                        │
                     │  setup()  — khởi tạo   │
                     │  loop()   — menu Serial│
                     │  Định nghĩa biến global│
                     └────────────────────────┘
```

**Chiều phụ thuộc:** `main.cpp` gọi xuống 3 module. Các module **không gọi lẫn nhau**
(trừ `test_motor` gọi `gyro_signalen()` của `calib_mpu` để đo rung, và gọi
`getReceiverValues()` của `receiver` để lấy tay ga).

### 7.2. Luồng dữ liệu khi chạy một bài test motor

Đây là ví dụ đầy đủ nhất, đi qua cả 3 module:

```
① Tay điều khiển  ──sóng radio──►  ② Receiver RC
                                        │ xung PWM 1000-2000 µs
                                        ▼
                                   ③ GPIO 39 của ESP32
                                        │ mỗi lần đổi mức → sinh ngắt
                                        ▼
                              ④ ISR: handle_channel(2)
                                   đo độ rộng xung bằng micros()
                                        │
                                        ▼
                              ⑤ ReceiverValue[2] = 1450
                                        │ getReceiverValues() sao chép an toàn
                                        ▼
                              ⑥ check_motor_vibrations()
                                   constrain(1450, 1000, 2000)
                                        │
                                        ▼
                              ⑦ mot1.writeMicroseconds(1450)
                                        │ ESP32Servo tạo xung ở GPIO 4
                                        ▼
                              ⑧ ESC  ──►  Motor quay
                                        │
                                        │ (motor rung → drone rung)
                                        ▼
                              ⑨ MPU-6050 cảm nhận rung
                                        │ I2C
                                        ▼
                              ⑩ gyro_signalen() → acc_axis[]
                                        │
                                        ▼
                              ⑪ Tính độ rung → in ra Serial
```

Vòng ⑦→⑧→⑨→⑩→⑪ chính là **vòng phản hồi**: ta ra lệnh cho motor, rồi dùng cảm biến để
đo xem hậu quả vật lý là gì. Firmware bay hoạt động đúng theo nguyên lý này, chỉ khác
là nó dùng phản hồi để **tự động điều chỉnh** (PID) thay vì chỉ in ra màn hình.

### 7.3. Cấu trúc thư mục

```
src/
  main.cpp                  # setup(), loop(), menu Serial, định nghĩa biến toàn cục
  receiver/
    receiver.h              # Khai báo hàm public của module
    receiver.cpp            # Đọc PWM 6 kênh bằng ngắt, bài test 'a'
  calib_mpu/
    calib_mpu.h
    calib_mpu.cpp           # Quét I2C, đọc MPU, hiệu chỉnh, tính góc
  test_motor/
    test_motor.h
    test_motor.cpp          # Điều khiển ESC, đo rung
include/
  common.h                  # #define chân, khai báo extern dùng chung
platformio.ini              # Cấu hình board, thư viện
```

**Quy ước:** mỗi module là một thư mục, có **cặp `.h` + `.cpp`**.

- File `.h` (header) = **bản hợp đồng**: liệt kê những hàm mà module cho phép bên ngoài
  gọi. Ai muốn dùng module thì `#include` file này.
- File `.cpp` = **phần thực thi**: chứa code thật, kể cả những hàm phụ nội bộ mà bên
  ngoài không cần biết.

Cách chia này gọi là **đóng gói (encapsulation)**: người dùng `receiver` chỉ cần biết
"gọi `getReceiverValues()` sẽ có 6 giá trị", không cần biết bên trong dùng ngắt hay
thanh ghi gì.

---

## 8. Khối `common.h` — Bộ khai báo chung

📁 [`include/common.h`](include/common.h) · 71 dòng

Đây là file **không chứa code thực thi**, chỉ chứa khai báo. Nhiệm vụ: cho phép các
module khác nhau cùng dùng chung một tập biến và hằng số.

### 8.1. `#define` — đặt tên cho con số

```cpp
#define RED_LED_PIN 2
#define GREEN_LED_PIN 3
```

`#define` là lệnh cho **bộ tiền xử lý** (preprocessor): trước khi biên dịch, mọi chỗ
xuất hiện chữ `RED_LED_PIN` sẽ được thay bằng số `2`.

**Tại sao không viết thẳng số 2?** Giả sử bạn đổi LED sang GPIO 8. Nếu dùng `#define`,
sửa **một dòng**. Nếu viết số 2 rải rác 20 chỗ, bạn phải tìm đủ 20 chỗ — và chắc chắn
sẽ sót, hoặc sửa nhầm một số 2 không liên quan.

> ⚠️ Quy tắc dự án yêu cầu **mọi số chân phải khai báo trong `common.h`**. Hiện code còn
> vi phạm ở [`main.cpp:94`](src/main.cpp#L94): `Wire.begin(11, 10, 400000)` viết thẳng
> số chân I2C vào giữa logic. Đây là một bài tập tốt để sửa (xem mục 14).

### 8.2. `extern` — "biến này có ở đâu đó, không phải ở đây"

```cpp
extern int32_t channel_1;   // trong common.h  — chỉ KHAI BÁO
```
```cpp
int32_t channel_1;          // trong main.cpp  — ĐỊNH NGHĨA thật (cấp bộ nhớ)
```

Phân biệt hai từ này rất quan trọng:

| | **Khai báo** (`extern`) | **Định nghĩa** |
|---|---|---|
| Làm gì | Báo cho trình biên dịch biết *tên và kiểu* của biến | Thực sự **cấp phát bộ nhớ** |
| Ở đâu | Trong file `.h` | Trong **đúng một** file `.cpp` |
| Được phép lặp lại | Có, bao nhiêu lần cũng được | **Không** — lặp lại sẽ lỗi linker |

Nếu bỏ chữ `extern` trong header, mỗi file `.cpp` include nó sẽ tạo ra một biến
`channel_1` riêng → khi liên kết (link) chương trình, trình liên kết thấy nhiều biến
trùng tên và báo lỗi `multiple definition`.

### 8.3. `#pragma once` — chống include trùng

Dòng đầu file. Nếu `main.cpp` include `common.h`, rồi lại include `receiver.h` (mà
`receiver.h` cũng include `common.h`), nội dung `common.h` sẽ bị dán vào hai lần → lỗi
"khai báo trùng". `#pragma once` bảo trình biên dịch: *file này chỉ dán vào một lần thôi*.

### 8.4. Danh mục biến toàn cục

| Nhóm biến | Kiểu | Ý nghĩa |
|---|---|---|
| `acc_axis[4]`, `gyro_axis[4]` | `int16_t` | Số đọc thô từ MPU. **Chỉ số 1, 2, 3 = X, Y, Z**; chỉ số 0 bỏ trống |
| `temperature` | `int16_t` | Nhiệt độ thô của MPU |
| `channel_1..6` | `int32_t` | Độ rộng xung 6 kênh RC (µs) |
| `angle_pitch`, `angle_roll` | `float` | Góc nghiêng sau bộ lọc bù (độ) |
| `angle_pitch_acc`, `angle_roll_acc` | `float` | Góc tính riêng từ accel (độ) |
| `manual_*_cal_value` | `int16_t` | Giá trị hiệu chỉnh chép tay |
| `data` | `uint8_t` | **Ký tự lệnh vừa gõ** — biến quan trọng nhất, điều khiển toàn bộ menu |
| `disable_throttle` | `uint8_t` | Cờ khoá tay ga |
| `loop_timer` | `uint32_t` | Mốc thời gian giữ nhịp 250 Hz |
| `cal_int` | `int32_t` | Bộ đếm vòng lặp hiệu chỉnh |

> **Vì sao mảng bỏ trống chỉ số 0?** `acc_axis[1]` = trục X, `[2]` = Y, `[3]` = Z. Cách
> này giúp đọc code tự nhiên hơn (trục 1, 2, 3) nhưng lãng phí 1 phần tử. Đây là quy ước
> kế thừa từ code gốc — không phải cách viết tối ưu, nhưng phải nhớ khi đọc code.

> **Vì sao dùng `int16_t` chứ không phải `int`?** Vì `int` có kích thước khác nhau tuỳ
> nền tảng (16 hoặc 32 bit). Khi làm việc với thanh ghi cảm biến — nơi từng bit có ý
> nghĩa — phải dùng kiểu **cố định kích thước**: `int16_t` luôn đúng 16 bit. MPU-6050
> trả về đúng 16 bit có dấu, nên `int16_t` là kiểu khớp chính xác.

---

## 9. Khối `main.cpp` — Bộ não điều phối

📁 [`src/main.cpp`](src/main.cpp) · 268 dòng

Nhiệm vụ: khởi tạo phần cứng, hiển thị menu, và **điều hướng** ký tự người dùng gõ tới
đúng hàm test. Bản thân nó không chứa thuật toán nào phức tạp.

### 9.1. `setup()` — chạy đúng một lần khi khởi động

```cpp
void setup()
{
  Serial.begin(57600);                    // ① Mở cổng Serial

  pinMode(RED_LED_PIN, OUTPUT);           // ② Đặt 2 chân LED làm đầu ra
  pinMode(GREEN_LED_PIN, OUTPUT);

  rx_config();                            // ③ Gắn ngắt cho 6 chân receiver

  Wire.begin(11, 10, 400000);             // ④ Mở I2C: SDA=11, SCL=10, 400 kHz

  // ⑤ Cấu hình MPU-6050 qua 4 thanh ghi (xem bảng bên dưới)

  motors_init();                          // ⑥ Gắn 4 ESC, gửi tín hiệu arm
  print_intro();                          // ⑦ In menu
}
```

**Thứ tự có ý nghĩa:** phải `rx_config()` trước khi vào `loop()`, nếu không mảng
`ReceiverValue[]` sẽ không bao giờ được cập nhật. Phải `Wire.begin()` trước khi ghi
thanh ghi MPU. Phải `motors_init()` để ESC nhận tín hiệu 1000 µs — nếu ESC không nhận
được tín hiệu hợp lệ ngay từ đầu, nhiều loại ESC sẽ kêu bíp báo lỗi và từ chối chạy.

### 9.2. Cấu hình MPU-6050 — ghi vào thanh ghi

MPU-6050 giống một con chip có **bảng công tắc bên trong**. Muốn đổi chế độ, ta ghi giá
trị vào đúng ô nhớ (thanh ghi) của nó. Mỗi lần ghi theo mẫu 3 bước:

```cpp
Wire.beginTransmission(gyro_address);  // Gọi thiết bị ở địa chỉ 0x68
Wire.write(0x1B);                      // "Tôi muốn ghi vào thanh ghi 0x1B"
Wire.write(0x08);                      // "Giá trị là 0x08"
Wire.endTransmission();                // Gửi đi
```

Bốn thanh ghi được cấu hình trong `setup()`:

| Thanh ghi | Tên | Giá trị | Tác dụng |
|:---:|---|:---:|---|
| `0x6B` | PWR_MGMT_1 | `0x00` | **Đánh thức cảm biến.** Khi cấp nguồn, MPU ở chế độ ngủ; không ghi dòng này thì đọc mãi ra số 0 |
| `0x1B` | GYRO_CONFIG | `0x08` | Đặt thang gyro **±500 °/s** → hệ số 65.5 LSB/(°/s) |
| `0x1C` | ACCEL_CONFIG | `0x10` | Đặt thang accel **±8 g** → hệ số 4096 LSB/g |
| `0x1A` | CONFIG | `0x03` | Bật **bộ lọc thông thấp ~44 Hz** |

> **Bộ lọc thông thấp (DLPF) làm gì?** Nó loại bỏ các dao động **tần số cao** — chính là
> rung do motor và cánh quạt. Drone nghiêng người thì chậm (vài Hz), còn motor rung thì
> nhanh (hàng trăm Hz). Lọc bỏ phần nhanh giữ lại phần chậm = giữ lại thông tin có ích.
>
> Nhưng lọc mạnh quá thì cảm biến phản ứng chậm, drone bay lờ đờ và mất ổn định. 44 Hz
> là mức cân bằng thường dùng.

### 9.3. `loop()` — máy trạng thái điều khiển bằng biến `data`

`loop()` chạy lặp vô hạn. Mỗi vòng làm 4 việc:

```cpp
void loop()
{
  delay(10);                          // ① Nghỉ 10 ms (~100 vòng/giây)

  { uint16_t ch[6];                   // ② Cập nhật 6 kênh RC
    getReceiverValues(ch);
    channel_1 = ch[0];  /* ... */  channel_6 = ch[5]; }

  if (Serial.available() > 0)         // ③ Có ai gõ phím không?
  {
    data = Serial.read();             //    → Lưu ký tự vào biến `data`
    delay(100);
    while (Serial.available() > 0)    //    → Xả sạch phần thừa trong bộ đệm
      loop_counter = Serial.read();
    disable_throttle = 1;             //    → KHOÁ tay ga
  }

  if (!disable_throttle) { /* xuất channel_3 ra 4 motor */ }   // ④ (nguy hiểm)
  else                  { /* ép cả 4 motor về 1000 µs   */ }

  if (data == 'a') { /* gọi reading_receiver_signals() */ }    // ⑤ Điều hướng
  if (data == 'b') { /* gọi i2c_scanner()              */ }
  // ... 10 nhánh tương tự
}
```

**Biến `data` là trung tâm của toàn bộ firmware.** Nó không chỉ dùng ở `main.cpp` — mọi
hàm test đều lặp `while (data != 'q')`, nghĩa là chúng tự thoát khi thấy `data` thành
`'q'`. Đây là cách firmware cho phép người dùng thoát một bài test đang chạy: gõ `q` →
`Serial.read()` bên trong hàm test đọc được → vòng lặp kết thúc.

**Vì sao phải "xả bộ đệm"?** Nhiều Serial Monitor tự thêm ký tự xuống dòng (`\n`) sau
mỗi lần gửi. Nếu không xả, ký tự `\n` này sẽ được đọc ở vòng sau và bị hiểu nhầm thành
một lệnh mới. Vòng `while` xả sạch phần thừa, chỉ giữ lại ký tự đầu tiên.

> ### 🔴 Điểm nguy hiểm ở bước ④
>
> `disable_throttle` là biến toàn cục nên C++ khởi tạo nó bằng **0**. Vậy ngay sau khi
> khởi động, `!disable_throttle` là **đúng** → firmware đưa tay ga thẳng ra cả 4 motor,
> **không qua bất kỳ kiểm tra an toàn nào**.
>
> Nó chỉ chuyển sang 1 khi người dùng gõ phím đầu tiên, và **không bao giờ trở về 0**.
> Xem lại mục An toàn ở Phần I.

---

## 10. Khối `receiver` — Đọc tay điều khiển

📁 [`src/receiver/receiver.cpp`](src/receiver/receiver.cpp) · 146 dòng

**Bài toán:** đo độ rộng của 6 xung PWM đến từ receiver, cùng lúc, chính xác tới
micro-giây, mà không làm chậm chương trình chính.

### 10.1. Vì sao phải dùng ngắt (interrupt)?

Cách ngây thơ là dùng `pulseIn()` — hàm này **đứng chờ** cho tới khi xung kết thúc:

```cpp
ch1 = pulseIn(37, HIGH);   // Đứng chờ tới 2 ms
ch2 = pulseIn(38, HIGH);   // Rồi lại chờ tiếp...
// ... 6 kênh × 2 ms = 12 ms lãng phí mỗi vòng, và các kênh đo lệch thời điểm nhau
```

12 ms là quá nhiều với vòng lặp 4 ms. Giải pháp: **ngắt phần cứng**.

> **Ngắt là gì?** Hãy tưởng tượng bạn đang làm bài tập (chương trình chính) và có chuông
> cửa (ngắt). Khi chuông reo, bạn *dừng bút ngay lập tức*, ra mở cửa (chạy hàm ISR), rồi
> quay lại làm bài đúng chỗ đang dở. Bạn không cần liên tục chạy ra cửa xem có ai không.
>
> Ở đây "chuông cửa" là **mỗi lần chân GPIO đổi mức điện áp**, và "ra mở cửa" là ghi lại
> thời điểm đó.

### 10.2. Cách đo độ rộng xung

```cpp
void IRAM_ATTR handle_channel(uint8_t ch)
{
    uint32_t t = micros();          // ① Chụp thời điểm NGAY LẬP TỨC
    uint8_t pin = channel_pins[ch];
    uint8_t state;

    if (pin < 32)                   // ② Đọc mức chân từ thanh ghi phần cứng
        state = (GPIO.in >> pin) & 0x1;
    else
        state = (GPIO.in1.val >> (pin - 32)) & 0x1;

    if (state)                      // ③ Sườn LÊN → ghi nhớ điểm bắt đầu
    {
        timer_ch[ch] = t;
    }
    else                            // ④ Sườn XUỐNG → tính hiệu = độ rộng
    {
        uint32_t width = t - timer_ch[ch];
        if (width > 900 && width < 2100)      // ⑤ Kiểm tra hợp lệ
            ReceiverValue[ch] = (uint16_t)width;
    }
}
```

Minh hoạ:

```
        ┌──────────────┐
        │              │
────────┘              └────────
        ▲              ▲
        │              │
   ISR chạy lần 1  ISR chạy lần 2
   state = 1       state = 0
   timer_ch = 12345   width = 13845 - 12345 = 1500 µs ✓
```

Ngắt được gắn ở chế độ `CHANGE` — nghĩa là kích hoạt ở **cả sườn lên lẫn sườn xuống**.
Lần đầu ghi mốc, lần sau lấy hiệu.

**Bước ⑤ rất quan trọng:** nhiễu điện có thể tạo ra xung giả rất ngắn hoặc rất dài. Chỉ
những giá trị nằm trong khoảng 900–2100 µs mới được chấp nhận. Xung ngoài khoảng này bị
**vứt bỏ hoàn toàn** và giá trị cũ được giữ nguyên.

> ⚠️ Chính cơ chế "giữ nguyên giá trị cũ" này tạo ra **lỗ hổng failsafe**: khi mất sóng,
> không có xung nào đến, `ReceiverValue[]` giữ mãi giá trị cuối cùng, và motor cứ quay.

### 10.3. Hai chi tiết kỹ thuật quan trọng

**`IRAM_ATTR` — bắt hàm phải nằm trong RAM**

```cpp
void IRAM_ATTR handle_channel(uint8_t ch)
```

Bình thường code ESP32 nằm ở bộ nhớ flash ngoài, truy cập chậm và **có lúc bị khoá** (ví
dụ khi đang ghi flash). Nếu ngắt xảy ra đúng lúc đó, chip sẽ **crash**. `IRAM_ATTR` yêu
cầu trình biên dịch đặt hàm này vào RAM nội — luôn sẵn sàng, luôn nhanh.

Quy tắc: **mọi hàm ISR trên ESP32 đều phải có `IRAM_ATTR`.**

**Đọc thanh ghi `GPIO.in` thay vì `digitalRead()`**

```cpp
state = (GPIO.in >> pin) & 0x1;      // Nhanh: đọc thẳng thanh ghi phần cứng
// state = digitalRead(pin);         // Chậm hơn nhiều: qua nhiều lớp thư viện
```

`GPIO.in` là một thanh ghi 32 bit, **mỗi bit là mức của một chân**. Phép `>> pin` dịch
bit cần quan tâm về vị trí 0, phép `& 0x1` giữ lại đúng bit đó. Vì ESP32-S3 có hơn 32
chân nên cần thanh ghi thứ hai `GPIO.in1` cho các chân từ 32 trở lên.

Trong ISR, mỗi micro-giây đều quý — vì vậy phải viết ở mức thấp như thế này.

### 10.4. `getReceiverValues()` — đọc dữ liệu an toàn

```cpp
void getReceiverValues(uint16_t *ch)
{
    noInterrupts();                         // ① TẠM KHOÁ ngắt
    for (int i = 0; i < NUM_CHANNELS; i++)
        ch[i] = ReceiverValue[i];           // ② Sao chép cả 6 giá trị
    interrupts();                           // ③ Mở lại ngắt
}
```

**Vì sao phải khoá ngắt?** Vì ISR có thể nhảy vào **giữa chừng** vòng sao chép. Kịch bản
xấu: bạn vừa chép xong kênh 1–3, ISR chen vào cập nhật kênh 1 và kênh 5, rồi bạn chép
tiếp kênh 4–6. Kết quả: kênh 1 là dữ liệu **cũ**, kênh 5 là dữ liệu **mới** — một ảnh
chụp không nhất quán về thời gian.

Với dữ liệu điều khiển bay, sự không nhất quán này có thể gây hành vi bất thường. Việc
khoá ngắt chỉ kéo dài vài micro-giây, hoàn toàn chấp nhận được.

**Từ khoá `volatile`:**

```cpp
volatile uint16_t ReceiverValue[NUM_CHANNELS];
```

`volatile` báo cho trình biên dịch: *biến này có thể bị thay đổi bởi thứ gì đó ngoài
luồng chương trình bình thường (ở đây là ISR), đừng tối ưu hoá nó.*

Không có `volatile`, trình biên dịch có thể "thông minh" quá mức: thấy trong vòng lặp
không có dòng nào gán `ReceiverValue`, nó bèn đọc một lần rồi cache lại trong thanh ghi
CPU và dùng mãi giá trị cũ. Kết quả: chương trình không bao giờ thấy dữ liệu mới.

**Quy tắc: mọi biến được ISR ghi vào đều phải là `volatile`.**

### 10.5. `reading_receiver_signals()` — bài test `a`

Vòng lặp in giá trị 6 kênh mỗi 250 ms, kèm ký hiệu trực quan:

```cpp
if (channel_1 - 1480 < 0)       Serial.print("<<<");   // Lệch trái
else if (channel_1 - 1520 > 0)  Serial.print(">>>");   // Lệch phải
else                            Serial.print("-+-");   // Ở giữa
```

Khoảng 1480–1520 được coi là "giữa" — đây là **vùng chết (deadband)** 40 µs. Cần điều
khiển thật không bao giờ về đúng 1500; luôn có sai lệch cơ khí vài chục µs. Vùng chết
tránh việc drone bị trôi nhẹ khi người lái đã buông cần.

---

## 11. Khối `calib_mpu` — Cảm biến & tính góc

📁 [`src/calib_mpu/calib_mpu.cpp`](src/calib_mpu/calib_mpu.cpp) · 359 dòng

Đây là khối nhiều toán học nhất. Nó có 5 hàm ứng với 5 bài test: `b`, `c`, `d`, `e`, `h`.

### 11.1. `i2c_scanner()` — bài test `b`

Nguyên lý rất đơn giản: **thử gọi tất cả 126 địa chỉ, xem ai trả lời**.

```cpp
for (address = 1; address < 127; address++)
{
    Wire.beginTransmission(address);
    error = Wire.endTransmission();     // Trả 0 nếu có thiết bị phản hồi
    if (error == 0) { /* Tìm thấy! */ }
}
```

Chuẩn I2C quy định: khi master gọi một địa chỉ, thiết bị mang địa chỉ đó phải kéo đường
dữ liệu xuống để "vâng ạ" (tín hiệu ACK). Không ai kéo → master hiểu là không có thiết
bị.

Đây là **công cụ chẩn đoán đầu tiên** khi mạch không hoạt động: nếu không thấy `0x68`,
vấn đề là phần cứng (dây, nguồn), không phải phần mềm. Đừng phí thời gian sửa code.

### 11.2. `gyro_signalen()` — đọc dữ liệu thô

Hàm được gọi nhiều nhất trong firmware (250 lần/giây). Nó đọc **14 byte liên tiếp**
trong một lần giao dịch I2C:

```cpp
Wire.beginTransmission(gyro_address);
Wire.write(0x3B);                    // Bắt đầu từ thanh ghi ACCEL_XOUT_H
Wire.endTransmission();
Wire.requestFrom(gyro_address, 14);  // Xin 14 byte
```

MPU-6050 sắp xếp dữ liệu liền nhau từ địa chỉ `0x3B`, nên một lệnh đọc lấy hết:

| Byte | Nội dung |
|---|---|
| 1–2 | Accel X (cao, thấp) |
| 3–4 | Accel Y |
| 5–6 | Accel Z |
| 7–8 | Nhiệt độ |
| 9–10 | Gyro X |
| 11–12 | Gyro Y |
| 13–14 | Gyro Z |

**Ghép 2 byte thành 1 số 16 bit:**

```cpp
acc_axis[1] = Wire.read() << 8 | Wire.read();
```

Cảm biến gửi từng byte 8 bit, nhưng giá trị thật là 16 bit. Byte đầu là **byte cao**
(MSB), byte sau là **byte thấp** (LSB):

```
Byte cao:  0001 0000                    (= 16)
Byte thấp:           0000 0000          (= 0)
                ↓ dịch trái 8 bit rồi OR
Kết quả:   0001 0000 0000 0000          (= 4096)
```

Phép `<< 8` đẩy byte cao lên 8 vị trí, phép `|` (OR bit) ghép byte thấp vào chỗ trống.

**Đổi dấu trục:**

```cpp
gyro_axis[2] *= -1;   // Pitch
gyro_axis[3] *= -1;   // Yaw
```

Chip có thể được hàn theo hướng bất kỳ trên bo mạch. Hai dòng này đảm bảo quy ước:
*chúc mũi lên = pitch dương, xoay phải = yaw dương*. Nếu bạn thay bo mạch khác hoặc xoay
cảm biến 180°, đây là chỗ cần sửa.

**Trừ giá trị hiệu chỉnh:**

```cpp
acc_axis[1] -= manual_acc_pitch_cal_value;
gyro_axis[1] -= manual_gyro_roll_cal_value;
// ...
```

Giải thích ở mục tiếp theo.

### 11.3. `manual_imu_calibration()` — bài test `h`

**Vấn đề cần giải quyết:** đặt cảm biến nằm yên tuyệt đối, gyro lẽ ra phải đọc 0. Thực
tế nó đọc ra 61, hoặc −14. Đó là **độ lệch không (bias/offset)** — sai số cố hữu của
từng con chip do dung sai sản xuất.

Bias nhỏ nhưng cực kỳ nguy hiểm, vì góc được tính bằng **tích phân**:

```
Bias = 5 LSB  →  5 / 65.5 ≈ 0.076 °/s
Sau 1 phút:  0.076 × 60  ≈ 4.6°   ← drone tưởng mình nghiêng 4.6° dù đang nằm phẳng
Sau 5 phút:  0.076 × 300 ≈ 23°    ← không thể bay được
```

**Cách đo bias:** lấy trung bình rất nhiều mẫu khi cảm biến đứng yên. Giá trị thật là 0,
nên trung bình đo được **chính là bias**.

```cpp
// Giai đoạn 1 — Làm nóng: 2000 mẫu, VỨT BỎ
for (cal_int = 0; cal_int < 2000; cal_int++) {
    gyro_signalen();
    delay(4);
}

// Giai đoạn 2 — Đo thật: 4000 mẫu, CỘNG DỒN
for (cal_int = 0; cal_int < 4000; cal_int++) {
    gyro_signalen();
    gyro_axis_cal[1] += gyro_axis[1] + manual_gyro_roll_cal_value;
    // ...
    delay(4);
}

gyro_axis_cal[1] /= 4000;    // Chia lấy trung bình
```

**Vì sao cần 2000 mẫu làm nóng?** Cảm biến MEMS có **độ trôi theo nhiệt độ**. Lúc mới
cấp nguồn, chip còn nguội; sau vài giây nó nóng lên và ổn định. Đo trong giai đoạn chưa
ổn định sẽ ra bias sai. 2000 mẫu × 4 ms = **8 giây làm nóng**.

**Vì sao lại cộng lại `manual_gyro_roll_cal_value`?** Vì `gyro_signalen()` **đã trừ** giá
trị hiệu chỉnh cũ rồi. Muốn đo lại từ đầu, phải cộng nó trở lại để có số thô nguyên bản.
Nếu không, mỗi lần hiệu chỉnh sẽ trừ chồng lên lần trước.

Tổng thời gian: (2000 + 4000) × 4 ms = **24 giây**.

### 11.4. `check_imu_angles()` — bài test `e` (phần khó nhất)

Hàm này biến số đọc thô thành **góc nghiêng thực tế**, qua 3 giai đoạn:

#### Giai đoạn A — Góc từ gyro (tích phân)

```cpp
#define GYRO_SCALE_FACTOR 0.0000611f    // = 1 / (250 Hz × 65.5 LSB/(°/s))

angle_pitch += gyro_axis[2] * GYRO_SCALE_FACTOR;
angle_roll  += gyro_axis[1] * GYRO_SCALE_FACTOR;
```

Nguồn gốc con số `0.0000611`:

```
Gyro cho ta TỐC ĐỘ xoay, ta muốn GÓC.
Góc = tốc độ × thời gian.

Tốc độ (°/s)  = giá trị thô ÷ 65.5
Thời gian (s) = 1 ÷ 250 = 0.004

Góc mỗi vòng = giá trị thô ÷ 65.5 × 0.004
             = giá trị thô × (0.004 ÷ 65.5)
             = giá trị thô × 0.0000611
```

Dấu `+=` chính là **tích phân số**: cộng dồn từng mẩu góc nhỏ, 250 lần mỗi giây.

> **Vì sao nhân sẵn thành một hằng số?** Vì phép nhân nhanh hơn phép chia rất nhiều trên
> vi điều khiển. Tính trước `0.004 ÷ 65.5` một lần lúc viết code, thay vì bắt chip chia
> 250 lần mỗi giây. Đây là kỹ thuật tối ưu phổ biến trong firmware thời gian thực.

#### Giai đoạn B — Chuyển đổi giữa các trục khi xoay

```cpp
angle_pitch -= angle_roll  * sin(gyro_axis[3] * GYRO_YAW_RAD);
angle_roll  += angle_pitch * sin(gyro_axis[3] * GYRO_YAW_RAD);
```

Hiện tượng: nếu drone đang **nghiêng phải 30°** rồi **xoay 90° sang phải**, thì cái từng
là "nghiêng phải" bây giờ đã trở thành "chúc mũi xuống". Góc không tự đổi, nhưng **hệ
quy chiếu đã xoay**.

Hai dòng trên chuyển một phần góc roll sang pitch và ngược lại, tỉ lệ với lượng yaw vừa
xảy ra. `GYRO_YAW_RAD = GYRO_SCALE_FACTOR × π/180` vì hàm `sin()` nhận radian.

#### Giai đoạn C — Góc từ accelerometer

```cpp
acc_axis[1] = constrain(acc_axis[1], -4096, 4096);
angle_pitch_acc = asin((float)acc_axis[1] / 4096.0f) * 57.296f;
```

Nguyên lý: **trọng lực luôn chỉ xuống**. Đo xem vectơ trọng lực rơi vào trục nào bao
nhiêu, ta biết cảm biến đang nghiêng bao nhiêu.

```
   Nằm phẳng                      Nghiêng 30°
   ┌─────────┐                      ┌─────────┐
   │ cảm biến│                     /  cảm biến/
   └─────────┘                    └─────────┘
        │ g                            ╲ │ g
        ▼                               ╲▼
   ACC_z = 4096 (toàn bộ ở Z)      ACC_x = 4096·sin(30°) = 2048
   ACC_x = 0                       ACC_z = 4096·cos(30°) = 3547

   → Đảo ngược lại: góc = asin(ACC_x / 4096) = asin(0.5) = 30° ✓
```

- `constrain(...)` chặn giá trị trong ±4096, vì `asin()` chỉ nhận đối số trong khoảng
  [−1, 1]. Rung mạnh có thể làm số đọc vượt ngưỡng → `asin()` trả về `NaN` → toàn bộ
  phép tính hỏng.
- `57.296` = 180/π, đổi radian sang độ.

#### Giai đoạn D — Bộ lọc bù (Complementary filter)

```cpp
angle_pitch = angle_pitch * 0.9996f + angle_pitch_acc * 0.0004f;
```

Đây là **trái tim của thuật toán ước lượng góc**. Chỉ một dòng, nhưng kết hợp được ưu
điểm của cả hai cảm biến:

- **99.96%** tin vào góc từ gyro → giữ được sự **mượt mà và phản ứng nhanh**.
- **0.04%** tin vào góc từ accel → mỗi vòng kéo nhẹ về phía "sự thật", **triệt tiêu drift**.

```
Góc thật:  ────────────────────────────
Gyro:      ─────────────╱╱╱╱╱ (trôi dần lên, mượt)
Accel:     ∿∿∿∿∿∿∿∿∿∿∿∿∿ (đúng trung bình, nhưng nhiễu loạn)
Lọc bù:    ──────────────────────────── (mượt VÀ không trôi ✓)
```

**Hằng số thời gian** của bộ lọc:

```
τ = dt × (a / (1 − a)) = 0.004 × (0.9996 / 0.0004) ≈ 10 giây
```

Nghĩa là: nhiễu accel ngắn hơn 10 giây gần như bị bỏ qua hoàn toàn (tốt — rung motor là
nhiễu tần số cao), còn sai lệch kéo dài hơn 10 giây sẽ được sửa dần (tốt — drift gyro là
sai số chậm).

Chỉnh hệ số:
- Tăng 0.9996 → 0.9999: mượt hơn nhưng chống drift kém hơn.
- Giảm 0.9996 → 0.98: bám accel nhanh hơn nhưng góc giật theo mỗi cú rung.

**Khởi tạo lần đầu:**

```cpp
if (first_angle) {
    angle_pitch = angle_pitch_acc;    // Lấy thẳng góc accel làm điểm xuất phát
    first_angle = false;
}
```

Nếu bắt đầu từ 0 mà drone đang nghiêng 20°, bộ lọc phải mất **hàng chục giây** mới bò
tới giá trị đúng (vì mỗi vòng chỉ sửa 0.04%). Lấy accel làm điểm xuất phát giúp đúng
ngay lập tức.

#### Kỹ thuật in ra Serial trải đều

```cpp
switch (loop_counter) {
    case 0: Serial.print("Pitch: ");     break;
    case 1: Serial.print(angle_pitch, 1); break;
    case 2: Serial.print(" Roll: ");     break;
    // ... tới case 7
}
loop_counter++;
if (loop_counter == 60) loop_counter = 0;
```

**Vì sao chia nhỏ ra như vậy?** `Serial.print()` **chậm** — ở 57600 baud, mỗi ký tự mất
~0.17 ms. In cả dòng 40 ký tự mất ~7 ms, trong khi vòng lặp chỉ có **4 ms**. In hết một
lần sẽ **phá vỡ nhịp 250 Hz**, làm phép tích phân sai.

Giải pháp: mỗi vòng chỉ in **một mẩu nhỏ**, trải qua 8 vòng. Chu kỳ 60 vòng = 240 ms →
mắt người thấy khoảng 4 dòng/giây, vừa đủ đọc, mà nhịp vòng lặp vẫn được giữ.

Đây là một kỹ thuật rất đáng học: **khi một việc quá chậm cho vòng lặp thời gian thực,
hãy chia nhỏ nó ra nhiều vòng.**

---

## 12. Khối `test_motor` — Điều khiển động cơ

📁 [`src/test_motor/test_motor.cpp`](src/test_motor/test_motor.cpp) · 173 dòng

### 12.1. ESC và cách ra lệnh

**ESC (Electronic Speed Controller)** là mạch trung gian giữa vi điều khiển và motor:

```
ESP32          ESC                     Motor không chổi than
  │  xung PWM   │  3 pha điện           (Brushless)
  │  1000-2000  │  công suất lớn
  └────────────►├──────────────────────►  ⚙
     (tín hiệu) │      (động lực)
                │
             Pin LiPo
```

ESP32 không thể điều khiển motor trực tiếp — motor cần dòng hàng chục ampe và điện 3 pha
đúng thứ tự. ESC lo phần đó; ta chỉ cần gửi cho nó một xung PWM giống hệt xung của
receiver.

**Thư viện ESP32Servo:**

```cpp
mot1.attach(MOT1_PIN, 1000, 2000);   // Chân 4, xung ngắn nhất 1000, dài nhất 2000 µs
mot1.writeMicroseconds(1500);        // Ra lệnh: 50% ga
```

Thư viện tên "Servo" vì servo và ESC dùng **chung một chuẩn tín hiệu**.

### 12.2. `motors_init()` — nghi thức khởi động ESC

```cpp
void motors_init(void)
{
    mot1.attach(MOT1_PIN, 1000, 2000);   // ... 4 motor

    mot1.writeMicroseconds(1000);        // Gửi tín hiệu GA THẤP NHẤT
    // ... cả 4 motor

    delay(3000);                         // Chờ 3 giây
}
```

**Vì sao phải gửi 1000 µs rồi chờ 3 giây?** Đây là **cơ chế an toàn của chính ESC**. Khi
mới có điện, ESC không chạy ngay mà chờ xem tín hiệu đầu vào là gì:

- Nếu thấy **ga thấp nhất** → "người dùng đã sẵn sàng" → kêu bíp và cho phép chạy.
- Nếu thấy **ga cao** → "nguy hiểm! có thể do lỗi" → **từ chối chạy**, kêu bíp báo lỗi
  liên tục.

Cơ chế này ngăn motor bật vọt lên ngay khi cắm pin. `delay(3000)` cho ESC đủ thời gian
hoàn tất nghi thức này.

> Đây là một trong số ít chỗ `delay()` dài được chấp nhận, vì nó nằm trong `setup()` chứ
> không phải vòng lặp điều khiển.

### 12.3. `check_motor_vibrations()` — bài test `1`–`5`

#### Bảo vệ tay ga trước khi bắt đầu

```cpp
wait_timer = millis() + 10000;              // Hạn chót: 10 giây nữa

if (throttle_us > 1050) {
    Serial.println(F("Throttle is not in the lowest position."));
}

while (millis() < wait_timer && !throttle_init_ok) {
    getReceiverValues(ch);
    throttle_us = ch[2];
    if (throttle_us < 1050) throttle_init_ok = 1;   // Đạt! Cho phép chạy
    delay(500);
    Serial.print(F("."));
}

if (!throttle_init_ok) data = 'q';          // Hết giờ → HUỶ bài test
```

Đây là **safety check quan trọng nhất của firmware**. Nó đảm bảo motor không thể vọt lên
ngay khi vào bài test. Nếu người dùng không hạ ga trong 10 giây, bài test tự huỷ.

#### Đo độ rung

Ý tưởng: **độ lớn của vectơ gia tốc** phải luôn bằng 1 g (4096) khi drone đứng yên. Rung
làm giá trị này dao động quanh 4096. Dao động càng mạnh = rung càng nhiều.

```cpp
// ① Tính độ lớn vectơ: √(x² + y² + z²)
vibration_array[0] = (int32_t)sqrt(
    (float)acc_axis[1]*acc_axis[1] +
    (float)acc_axis[2]*acc_axis[2] +
    (float)acc_axis[3]*acc_axis[3]);

// ② Dịch mảng và tính trung bình 16 mẫu gần nhất
average_vibration_level = 0;
for (array_counter = VIB_SAMPLES-1; array_counter > 0; array_counter--) {
    vibration_array[array_counter] = vibration_array[array_counter-1];
    average_vibration_level += vibration_array[array_counter];
}
average_vibration_level /= (VIB_SAMPLES - 1);

// ③ Cộng dồn ĐỘ LỆCH giữa mẫu hiện tại và trung bình
vibration_total_result += abs(vibration_array[0] - average_vibration_level);
```

Cấu trúc ở bước ② gọi là **cửa sổ trượt (sliding window)**: mảng 17 phần tử, mỗi vòng
đẩy toàn bộ lùi một ô, mẫu mới nhất vào ô 0, mẫu cũ nhất rơi ra ngoài.

```
Vòng N:    [ mới ][ 1 ][ 2 ][ 3 ] ... [ 16 ]
                    ↓ dịch phải
Vòng N+1:  [ mới ][mới cũ][ 1 ][ 2 ] ... [ 15 ]  ← ô 16 bị đẩy ra, biến mất
```

**Vì sao so với trung bình chứ không so với 4096?** Vì nếu drone hơi nghiêng, hoặc cảm
biến có bias, giá trị nền có thể là 4100 hay 4050. Ta không quan tâm giá trị nền là bao
nhiêu — ta chỉ quan tâm nó **dao động** nhiều hay ít. So với trung bình động sẽ tự loại
bỏ ảnh hưởng của giá trị nền.

> **Con số 50 ở dòng `Serial.println(vibration_total_result / 50)`** là hệ số chia tuỳ ý
> kế thừa từ code gốc (dù chỉ cộng dồn 20 mẫu). Nó chỉ ảnh hưởng tới thang hiển thị, nên
> hãy dùng bảng ngưỡng ở Phần I (< 10 là tốt) thay vì tự suy diễn ý nghĩa vật lý.

#### Tắt motor an toàn khi thoát

```cpp
mot1.writeMicroseconds(1000);   // ... cả 4 motor
```

Dòng cuối hàm. **Mọi hàm điều khiển motor đều phải kết thúc bằng lệnh tắt motor** — kể
cả khi thoát bình thường. Đây là một thói quen bắt buộc trong lập trình điều khiển.

---

## 13. Xử lý sự cố

| Hiện tượng | Nguyên nhân thường gặp |
|---|---|
| Serial Monitor toàn ký tự rác | Sai baud rate — phải dùng `-b 57600` |
| `No I2C devices found` | Sai dây SDA/SCL, thiếu nguồn 3.3 V cho MPU, hoặc thiếu điện trở kéo lên |
| Đọc MPU toàn số 0 | Cảm biến chưa được đánh thức — thanh ghi `0x6B` chưa ghi (xem 9.2) |
| Kênh RC đứng im không đổi | Dây tín hiệu chưa cắm đúng GPIO, hoặc receiver chưa bind với tay điều khiển |
| Góc Pitch/Roll trôi liên tục | Chưa hiệu chỉnh, hoặc quên đặt `use_manual_calibration = true` |
| Góc nhảy loạn khi motor chạy | Rung quá mạnh — kiểm tra cân bằng cánh, đệm chống rung cho IMU |
| Bài test motor tự thoát ngay | Tay ga không xuống dưới 1050 µs trong 10 giây — cơ chế an toàn |
| ESC kêu bíp liên tục, motor không quay | Tay ga đang cao lúc cấp nguồn — rút pin, hạ ga, cắm lại |
| Motor quay ngay khi cắm pin | Tay ga đang ở vị trí cao — xem mục An toàn ở Phần I |
| Motor sai chiều quay | Đảo vị trí 2 trong 3 dây giữa ESC và motor |
| Build báo lỗi chân GPIO không hợp lệ | Đang build nhầm môi trường — thêm `-e esp32-s3-devkitm-1` |

---

## 14. Bài tập tự làm

Xếp theo độ khó tăng dần. Các bài 1–3 an toàn tuyệt đối (không đụng tới motor).

**Bài 1 — Sửa lỗi baud rate.**
Cho `Serial.begin()` trong [`main.cpp:85`](src/main.cpp#L85) khớp với `monitor_speed`
trong [`platformio.ini`](platformio.ini#L14). Chọn một con số và làm cho cả hai giống
nhau. *Học được: cấu hình phần mềm phải nhất quán giữa các file.*

**Bài 2 — Gom chân I2C về `common.h`.**
Dòng `Wire.begin(11, 10, 400000)` viết thẳng số chân vào logic, vi phạm quy tắc dự án.
Tạo `#define I2C_SDA_PIN 11` và `#define I2C_SCL_PIN 10` trong `common.h` rồi dùng chúng.
*Học được: vì sao hằng số phải tập trung một chỗ.*

**Bài 3 — Tách hàm đọc accelerometer.**
Bài `d` hiện gọi `read_gyro_values()` và hàm này tự phân nhánh dựa vào biến `data`. Hãy
viết `read_accel_values()` riêng. *Học được: một hàm nên làm một việc.*

**Bài 4 — Thêm số thứ tự kênh vào bài test `a`.**
Sửa `reading_receiver_signals()` để in thêm số hiệu GPIO của từng kênh, giúp học sinh
khác dò dây nhanh hơn. *Học được: đọc và sửa code hiển thị.*

**Bài 5 — Sửa lỗi tràn `micros()`.**
Đổi `while (loop_timer > micros());` thành dạng an toàn với tràn số:
`while (micros() - loop_timer < 4000);`. Giải thích vì sao phép trừ số không dấu lại xử
lý đúng cả khi tràn. *Học được: số học không dấu và lỗi tràn biến.*

**Bài 6 — Thêm failsafe cho receiver.**
Ghi lại thời điểm mỗi lần ISR cập nhật kênh. Trong `getReceiverValues()`, nếu một kênh
không có xung mới trong 500 ms, trả về giá trị an toàn (1000 cho tay ga). *Học được:
thiết kế hệ thống chịu lỗi.*

**Bài 7 — Lưu giá trị hiệu chỉnh vào flash.**
Dùng thư viện `Preferences` của ESP32 để `manual_imu_calibration()` tự lưu kết quả, và
`setup()` tự đọc lại. Bỏ hẳn khâu chép tay. *Học được: bộ nhớ không mất khi tắt nguồn
(NVS).*

**Bài 8 — Khoá menu theo bậc an toàn.**
Thêm mảng `step_passed[]`. Từ chối chạy bài test motor (`1`–`5`) nếu học sinh chưa hoàn
thành các bài kiểm tra IMU và receiver. *Học được: thiết kế máy trạng thái và tư duy an
toàn.*

---

## Giấy phép

Phần mềm được cung cấp "nguyên trạng" (AS IS), không kèm bảo hành dưới bất kỳ hình thức
nào. Xem đầy đủ ở phần **Terms of use** đầu file [`src/main.cpp`](src/main.cpp#L1-L16).
