# STEM-KCBot
Cute codes for a cute car

## Cài Môi trường Code
- B1: Cài [Arduino v2](https://downloads.arduino.cc/arduino-ide/arduino-ide_2.0.4_Windows_64bit.exe).
- B2: Cài [Easycode](https://drive.google.com/drive/u/1/folders/1xCd9tZ0VadZD4rjVmXd_SSLT3-WnPcSv).
- B3: Mở Easycode, chọn chế độ code C++, vào menu Cài đặt/Cài Driver cho Arduino.
- B4: Copy folder libraries vào Documents/Arduino trên máy của bạn.
- B5: Kết nối xe với máy tính, đặt cổng COM, chuyển board thành Arduino Uno.

## Giải thích
- `v1, v2`: Phiên bản
- `5V, 9V`: Thiết đặt cho từng tốc độ
- `BT`: Bluetooth
- `LT`: Dò line
- `DualLT`: Dò line bằng 2 cách (8 mắt - 4 mắt cấp sẵn)
- `Light`: Đèn

## Giải thích file code
- `v1`: File bản đầu tiên (STEM 2021), xử lý dò line 4 mắt một cách cơ bản, không hiệu chỉnh
- `v2`: File bản thứ hai (STEM 2023 - QT 2023), tính toán vị trí dò line 4 mắt / 8 mắt kỹ càng, có hiệu chỉnh, chỉnh đồng thời hai chế độ không cần nạp lại file.
- `v2.LHP`: File bản thứ hai (LHP 2023), tối ưu tốc độ của động cơ và xử lý xe để có thể quay góc gấp 90 độ (chỉ sử dụng nếu đường đi thẳng và có góc cua 90 độ).
