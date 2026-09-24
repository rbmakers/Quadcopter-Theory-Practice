# v1.2：定位 Gyro reset 之後的 I²C timeout

## 本次實板紀錄

```text
IDs OK: ACC=0x1E GYR=0x0F
INIT FAILED at register read error=-2
phase=READ_REGISTER_TX addr=0x18 reg=0x7C WireStatus=5
```

已查 Arduino-Pico **5.6.0** 的 Wire.cpp：`endTransmission()` 在底層回傳 `PICO_ERROR_TIMEOUT` 時回傳5。這筆不是已辨識出的 NACK。
原始碼：https://raw.githubusercontent.com/earlephilhower/arduino-pico/5.6.0/libraries/Wire/src/Wire.cpp

v1.1 執行順序是 chip IDs → Gyro soft reset → Accel PWR_CONF 讀取 → Accel suspend → Accel reset。
因此這筆是在 Gyro reset 後，送出 Accel 暫存器位址時逾時，尚未執行 Accel reset 前置修正。不能據此判斷 v1.1 的 Accel 修正有效或無效。
也尚未到同步 config 或 GPIO 中斷設定，不應先修改 INT2–INT3 接線。

## 本版改動

- 預設 `RESET_GYRO_ON_STARTUP=false`，依先前查核的 Crazyflie 正常啟動方式，省略 Gyro soft reset。
- 在任何 Gyro reset 前先讀 Accel `0x7C/0x7D`，建立存取基準。
- 可將 `RESET_GYRO_ON_STARTUP=true` 作明確對照；會在 Gyro reset 後立即再讀 Accel `0x7C`，並給出特定失敗 stage。
- 省略 Gyro reset 後仍顯式設定 normal power、量程、ODR/BW、INT3/INT4 routing，重要設定回讀確認後才開始取樣。
- 保留 Accel suspend-before-reset、Bosch config stream 與 INT2–INT3 同步。
- 失敗時立即取樣 SDA/SCL 邏輯電位，之後再列印。
- 保持 I²C400kHz／timeout3ms，以便先對照 Gyro reset 的影響。沒有延長 timeout、降速、吞掉錯誤或自動改模式。

這是針對觀察到的故障點做的隔離版本；尚不能斷言 Gyro reset 是電氣根因。舊設定、匯流排狀態、供電或訊號品質仍可能相關。

## 測試順序

1. 完整替換 sketch，使用 v1.2 預設設定，不另改位址、clock 或 timeout。
2. **USB 和電池都移除數秒再供電**，讓 MCU 及 BMI088 一起重新上電；只 reset MCU 不一定清除 sensor 或匯流排狀態。
3. 確認標頭 `v1.2 ... GYRO RESET ISOLATION`，回傳完整開機 log。
4. 若預設版本成功，再選擇將 `RESET_GYRO_ON_STARTUP=true`，重新上傳、完整斷電後比較。沒有必要在失敗後不斷連續 reset。

預期主流程的關鍵訊息（非量測結果）：

```text
I2C pre-init: SDA=1 SCL=1
IDs OK: ACC=0x1E GYR=0x0F
ACC baseline: PWR_CONF=0x... PWR_CTRL=0x...
GYR reset: SKIPPED (Crazyflie-style startup)
ACC pre-reset: PWR_CONF=0x3 PWR_CTRL=0x0
ACC reset ACK OK
Post-reset IDs OK; loading sync config...
```

## 判讀

| 新結果 | 可以推論與下一步 |
|---|---|
| baseline 就 timeout | 不需 Gyro reset 即可重現；應查 I²C 交易／省電狀態／線路，不能歸因 Gyro reset |
| skip 正常，reset=true 後立即失敗 | 故障與 Gyro reset 路徑有強關聯；可進一步量測 reset 後 SDA/SCL |
| skip 仍於其他 register 失敗 | 依具體 stage/register 繼續，不把所有錯誤統稱 reset 問題 |
| 失敗時 SDA=0 或 SCL=0 | 當下有低電位，可能 bus 被拉住；需波形確認是誰驅動及持續多久 |
| 失敗時 SDA=1、SCL=1 | 僅表示取樣瞬間兩線高，不能排除先前 timeout、尖峰、上升時間或控制器狀態問題 |

若仍 timeout，下一個受控測試可單獨比較 clock 或 timeout，但不在這版同時更改，以保留判讀依據。

## 本次驗證

真實 driver + Bosch C API 的主機 mock Wire 測試通過；新增「Gyro reset 後 Accel register address TX 回5」情境，確認 default 路徑不送 Gyro reset、explicit reset 路徑在精確 stage 停止。mock 情境是人工注入，不能證明實板物理根因。
保留 v1.1 的 reset前置、錯誤傳遞、短讀、六軸換算及同步設定回歸測試。
本次未做 RP2350 目標重編譯或實板驗證。
