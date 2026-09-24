# v1.1：Accel reset error=-2 與 Crazyflie 對照

## 日誌已證實什麼

v1.0 停在 `accel reset error=-2`，代表前面的兩個 chip ID 檢查與 Gyro reset 已通過。
此版 Bosch API 的 -2 是 `BMI08_E_COM_FAIL`。此时尚未載入同步 config、設定 INT2 同步輸入或啟用 MCU IRQ，因此不能把這筆錯誤判作 GPIO22/23 的中斷故障。
ID 可讀不等於每一筆寫入都可靠；缺少底層 Wire status，不能單憑原日誌證實是 NACK、timeout 或電氣故障。

## 官方依據與修正

BMI088 datasheet rev1.9 §4.8.1（PDF 頁18–19）描述：Accel 非省電模式時的 reset，可能在 ACK 階段立即釋放 SDA。需要完整 ACK 的主機應先啟用省電並停用感測器及 auxiliary interface。
本版透過 Bosch `bmi08a_set_power_mode()` 使用文件中合法的完整 register 值：

1. `ACC_PWR_CONF (0x7C) = 0x03`，等待 5ms。
2. `ACC_PWR_CTRL (0x7D) = 0x00`，等待 5ms。
3. 回讀確認這兩個值，任何失敗即停止。
4. `ACC_SOFTRESET (0x7E) = 0xB6`。
5. 即使 reset 交易失敗也先等 5ms；失敗仍停止，沒有忽略 -2。
6. 成功時再查 chip ID、電源 reset 狀態，之後才載入 Bosch 同步 config。

這修正上一版未處理的已知 reset/ACK 條件；是否就是使用者實板的根因，須由 v1.1 的新日誌驗證。
I²C 仍維持 400kHz，timeout 仍維持3ms；避免同時更動多個因素而無法判讀。

## 附件與 Crazyflie 主流程的角色

| 檔案 | 本次確認內容 |
|---|---|
| 附件 sensors_bmi088_i2c.c | 只綁定 Accel/Gyro address、I²C interface、read/write/delay callbacks，並非完整初始化 |
| 附件 sensors_bmi088_common.h | callback 宣告，沒有實作 |
| 附件 sensors.c | BMI088 選項指向 sensorsBmi088Bmp3xxInit_I2C 及對應 callback |
| 附件 sensors_bosch.c | 有 BMI088 include／常數，但檢視的裝置初始化主要是 BMI160/BMI055 等；不能用此檔代表 BMI088 實際啟動路徑 |
| 附件 bmi088_accel.c | 舊 Bosch API 的 soft reset 仍會把 callback 通訊錯誤往上傳；軟體 cfg 的 suspend 預設不代表硬體已被寫入 suspend |
| 附件 bmi088_gyro.c、bmi088_fifo.c | Gyro data-ready 與 FIFO 支援函式的存在，不代表主程式有啟用所有功能 |
| 附件 HTML | 描述 Gyro IRQ 驅動 sensorsTask；文中的工作排程同步，不等於两顆 die 的硬體資料同步 |

附件沒有包含 `sensors_bmi088_bmp3xx.c`，本次另讀 Bitcraze 官方 master 的該檔補足主流程。它可能與附件對應的舊版不同；例如附件 HTML 說明 semaphore，這次讀取的官方 callback 使用 task notification。共同重點是 ISR 通知工作，I²C 在 task 內執行。

## 與本版同步方案的差別

| 項目 | 本次讀取的 Crazyflie BMI088 主流程 | CURIO v1.1 |
|---|---|---|
| 正常啟動的 Accel soft reset | 沒有呼叫 | 保留；先 suspend/disable |
| Accel 啟動 | enable、等5ms、init、active、等10ms、設定量程/ODR | 重置後載入同步 config，再 active 與同步設定 |
| 讀取觸發 | Gyro INT3，1kHz | 同步資料就緒 INT1，預設400Hz |
| Accel / Gyro ODR | Accel1600Hz、OSR4；Gyro1000Hz、BW116Hz | Bosch400Hz同步配對；亦可選1kHz同步配對 |
| 資料同步 | task 依同一事件依序讀兩個 sensor | INT3→INT2 + Bosch同步引擎 |
| config stream | 正常初始化未呼叫載入 | 必須載入並驗證 |
| 匯流排錯誤 | backend read/write 回傳連線錯誤 | 保留回傳，新增 phase/address/register/Wire status/長度 |

因此不直接搬入 Crazyflie 舊 driver 或刪除同步設定；否則會改掉原本的 INT2–INT3 資料同步需求。也沒有在 reset 失敗後靜默改用非同步模式。
Crazyflie 的 sensorsAccelGet wrapper 未把 read 回傳值傳給上層，本版仍維持每組資料的成功／失敗檢查。

## 上傳後觀察

解壓縮至新資料夾，用整份 v1.1 sketch 上傳。開機應顯示 `v1.1 ... RESET ACK FIX`。
初始電源值依是否只重啟 MCU 或整板重新供電而不同；其後應見：

```text
IDs OK: ACC=0x1E GYR=0x0F
ACC initial: PWR_CONF=0x... PWR_CTRL=0x...
ACC pre-reset: PWR_CONF=0x3 PWR_CTRL=0x0
ACC reset ACK OK
Post-reset IDs OK; loading sync config...
```

這是预期格式，並非已量測輸出。若失敗，請回傳從 v1.1 標頭到最後一行的完整 log，尤其 `Last I2C failure`。
`WireStatus` 為 Arduino core 原始狀態，`WRITE_TX completed` 表示成功加入 TX buffer 的 payload bytes，**不代表 sensor 已 ACK 那些 bytes**。
若仍在 `accel reset after suspend` 出錯，下一步才依 WireStatus 檢查實際 ACK／SDA、SCL 波形，或做100kHz初始化對照；不要直接忽略錯誤。

## 來源

- https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf （rev1.9 §4.8.1）
- https://community.bosch-sensortec.com/mems-sensors-forum-jrmujtaw/post/bmi088-soft-reset-failure-NLONx65SuHbHtSj
- https://github.com/bitcraze/crazyflie-firmware/blob/master/src/hal/src/sensors_bmi088_bmp3xx.c

本次以附件及上述官方主流程作對照，沒有把 Crazyflie GPL 原始碼複製進 CURIO driver。隨附 Bosch BSD 原始檔保持不變。
