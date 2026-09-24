# 驗證紀錄

## 已完成

- Earle Philhower Arduino-Pico **5.6.0** 實際交叉編譯、連結成功。
- FQBN：`rp2040:rp2040:generic_rp2350:arch=arm,freq=150,usbstack=picosdk`。
- ARM Cortex-M33，150 MHz，Pico SDK USB；預設 1000 Hz、兩套融合、50 Hz Plotter。
- Flash：**84,244 bytes**；全域/靜態 RAM：**14,616 bytes**。這是編譯器報告，並非實機最大 stack / heap 使用量。
- 工具鏈與核心下載檔的 SHA-256 與 Arduino-Pico 發行索引相符。
- 兩套算法皆通過主機測試：靜止穩定、90° yaw 積分、30° roll 收斂、30° pitch 初始化、無加速度修正的 gyro 積分、拒絕 NaN / 過長 dt、變動 dt 與 quaternion 正規化。
- BMI088.h/.cpp 及 Bosch 原檔沿用成功的 v1.2，未修改。主程式預設仍跳過 gyro soft reset。

數學測試可在本資料夾執行：

```sh
g++ -std=c++11 -Wall -Wextra -Werror tests/fusion_test.cpp -o /tmp/fusion_test
/tmp/fusion_test
```

## 尚待實機驗證

本次未連接 CURIO / BMI088 硬體，尚未量測加入兩套算法後的 1 kHz 處理時間、長時間漏樣率、USB Plotter 顯示及實際姿態誤差。既有感測器範例的硬體成功紀錄不等同這一版融合程式已經實測。

上板先保持靜止完成校正，再依次緩慢轉動 X/Y/Z 軸，確認對應 Roll/Pitch/Yaw 與實際安裝方向一致。可啟用 `DIAGNOSTICS=true` 檢查 OK、late、skip、cross、fail、bus、work_us；確認後改回 false 繪圖。
