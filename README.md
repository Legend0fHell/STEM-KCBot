# STEM-KCBot
- Code cho xe sử dụng mạch EasyCode (Arduino Uno)
- Code chiến thắng toàn bộ cuộc thi liên quan đến xe dò line, xe tự hành tại tỉnh Yên Bái từ 2021-2023.

## Tải code
- B1: Nhấn `Code` ở phía trên, chọn `Download ZIP`.
- B2: Giải nén file `.zip` mới được tải về. Toàn bộ code, thư viện, tài liệu sẽ nằm trong file đó.

## Cài Môi trường Code
- B1: Cài Arduino tại link sau: [Arduino v2](https://downloads.arduino.cc/arduino-ide/arduino-ide_2.0.4_Windows_64bit.exe).
- B2: Cài thư viện cho board:
  - Vào Arduino, menu `Tools/Board/Boards Manager`.
  - Cài đặt (Install) mục `Arduino AVR Boards` trong mục `Boards Manager` ở cạnh trái (hoặc phải).
  - Kiểm tra xem có bị lỗi `Missing FBQN` không: [Link](https://support.arduino.cc/hc/en-us/articles/4412950941074-Compilation-error-Missing-FQBN-Fully-Qualified-Board-Name-)
- B3: Cài Easycode: [Easycode](https://drive.google.com/drive/u/1/folders/1xCd9tZ0VadZD4rjVmXd_SSLT3-WnPcSv).
- B4: Mở Easycode, chọn chế độ code C++, vào menu `Cài đặt/Cài Driver` cho Arduino.
- B5: Copy folder tên `libraries` ở file `zip` vào thư mục `Documents/Arduino` trên máy của bạn.
![image](https://github.com/Legend0fHell/STEM-KCBot/assets/56226847/931caa09-3865-407a-b2a8-72c498a30c12)
- B6: Kết nối xe với máy tính, đặt cổng COM, chuyển board thành Arduino Uno.

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
