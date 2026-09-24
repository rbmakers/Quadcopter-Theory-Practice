# v1.2 本次驗證

- 新增預設省略 Gyro reset、明確啟用 reset、reset 後 register TX timeout 的模擬案例，全部通過。
- 失敗時保留錯誤並指出具體 stage；不把 timeout 當成功。
- -Wall -Wextra -Werror、AddressSanitizer、UndefinedBehaviorSanitizer 通過。
- 本次未重跑 Arduino 目標編譯或實板測試。
- 詳見 DEBUG_NOTES_v1_2.md。以下是歷史紀錄。

# v1.1 本次驗證

- 真實 BMI088.cpp 與隨附 Bosch C API 通過主機 C/C++ 編譯與 mock Wire 測試。
- wrapper 使用 -Wall -Wextra -Werror；測試使用 AddressSanitizer / UndefinedBehaviorSanitizer，關閉容器不支援的 leak 檢查。
- 模擬 Accel 仍在 active mode：舊流程直接 reset 回傳通訊失敗；新流程先 suspend + disable，回讀確認後 reset 成功。
- 模擬 PWR_CONF 寫入失敗：初始化停止，不送 reset。
- 模擬電源設定未生效：回讀失敗，初始化停止，不送 reset。
- 模擬預處理完成但 reset 仍然 NACK：初始化停止，保留 -2；沒有吞掉錯誤。
- 回歸驗證同步 config 384 次分塊寫入、INT1/INT2/INT3 配置、六軸換算、短讀/NACK 不發布新資料，以及 1kHz 模式。
- 本次未重新安裝 RP2350 toolchain，未重跑 Arduino 目標編譯；v1.0 的目標編譯紀錄如下。
- 未連接使用者實板，ACK 波形與實際根因仍須新 log／量測確認。模擬測試不證明實板故障已排除。

---

# 驗證紀錄

## v1.0 歷史驗證（以下編譯數值不代表 v1.1）

- Arduino CLI 實際編譯成功，非僅語法檢查。
- Board package：Earle Philhower Arduino-Pico 5.6.0。
- Target：`rp2040:rp2040:generic_rp2350:arch=arm,freq=150,usbstack=picosdk`。
- 預設程式：400 Hz 同步輸出、I²C 400 kHz、Accel ±6 g、Gyro ±2000 deg/s。
- Program storage：78,820 bytes；靜態 RAM：10,008 bytes。
- Bosch vendor 檔案與來源 commit 的內容一致；授權隨附。
- 使用主機端 mock Wire 執行真實 driver + Bosch C API 的測試，並開啟 AddressSanitizer / UndefinedBehaviorSanitizer（容器不支援 LeakSanitizer，已關閉 leak 檢查）。

測試案例：

1. 6144-byte config 以 384 個 16-byte payload 送出。
2. INT2 輸入／INT1 同步輸出／INT3 DRDY 映射及量程回讀。
3. 從同步 Accel 特殊暫存器讀出 signed XYZ，正確換算 m/s² / deg/s。
4. Accel Z 短讀時回報失敗，保持 caller Sample 不變。
5. Gyro NACK 時回報失敗，保持 caller Sample 不變。
6. config 前段 chunk 失敗、後續成功時，初始化仍拒絕成功。
7. 1 kHz mode 的 Accel ODR 與 Gyro BW/ODR 設定。
8. 錯誤 chip ID 阻止初始化。

## 尚須實板驗證

- 使用者實板的 RP2354A、實際 SDO 位址、電源與 I²C pull-up。
- GPIO22/23 實際 pulse 頻率／相位、INT2–INT3 短接與電氣方向。
- 感測器 config stream 在實體 BMI088 上完成載入。
- 六軸正負方向、靜止 norm、運動中的同步品質。
- 真實 I²C 耗時、漏中斷、USB 影響、長時間運作與故障復原。

mock Wire 只能驗證程式流程／錯誤傳遞，不能模擬 Bosch 內部同步演算法或證明硬體取樣時間一致。
未提供預編譯韌體；請依實際板設定在 Arduino IDE 編譯上傳。
