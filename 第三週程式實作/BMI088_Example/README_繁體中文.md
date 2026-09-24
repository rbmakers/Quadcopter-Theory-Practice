# CURIO BMI088 v1.2：I²C、中斷與硬體資料同步

本次針對 v1.1 的 `READ_REGISTER_TX ... reg=0x7C WireStatus=5` 逾時，預設省略 Gyro soft reset 並加入對照開關與電位診斷。請先閱讀 `DEBUG_NOTES_v1_2.md`；v1.1 記錄保留作沿革。

本版將原本輪詢讀值的三個檔案改為「Bosch Data Synchronization + GPIO 中斷觸發讀取」。
使用者訊息的 RP2454A 先按 CURIO 的 **RP2354A（RP2350 系列）** 處理。
這是獨立感測器驗證範例，沒有馬達控制、offset 校正或姿態融合。

## 1. 安裝與開啟

1. 完整解壓縮 ZIP，保留 `BMI088_Example` 資料夾及 `src/bosch` 子資料夾。
2. Arduino IDE 開啟 `BMI088_Example/BMI088_Example.ino`。
3. 使用 Earle Philhower 的 **Raspberry Pi Pico/RP2040/RP2350** core；原 v1.0 曾以 **5.6.0** 編譯；v1.2 的本次驗證範圍見 VALIDATION.md。
4. 自製 RP2354A 板可選 Generic RP2350，Architecture=ARM、150 MHz、USB Stack=Pico SDK，Flash size 依實板設定。RP2354A 內建 Flash 為 2 MB 時選相應選項。
5. 確認下表接線及 SDO 位址後上傳，開啟 115200 序列監控。USB CDC 的 baud 設定不決定 I²C 速度。
6. 不必安裝另一套 BMI088 函式庫；Bosch 原始碼、6144-byte 同步設定資料與授權均已附在 `src/bosch`。

請使用整份 ZIP。只下載 `.ino/.h/.cpp` 三個檔案會缺少同步引擎的必要相依檔案。
檔案名稱統一為 `BMI088.h`、`BMI088.cpp`，不要在同一 sketch 內留下舊版 `BMI088(2).cpp`，避免重複定義。

## 2. 接線：INT2–INT3 短接保留

| CURIO / MCU | BMI088 | 方向與作用 |
|---|---|---|
| GPIO20 | SDA | 保留附件原設定，I²C0 資料 |
| GPIO25 | SCL | 保留附件原設定，I²C0 時鐘 |
| GPIO22 / INT_ACC | INT1 | BMI088 → MCU，同步資料就緒 |
| GPIO23 / INT_GYR | INT3 | BMI088 → MCU，Gyro DRDY 計數 |
| 與 INT3 同一個 net | INT2 | **INT3 → INT2；INT2 必須是輸入** |
| VDDIO | PS、CSB1 | 按 BMI088 I²C 接線要求維持高電位 |
| GND | GND/GNDIO | 共地；供電與 I²C pull-up 按實板設計 |

GPIO20 與 GPIO25 可分別作為 I²C0 SDA、SCL，不必相鄰。GPIO22、23 均設定 `INPUT`，由感測器主動驅動。
GPIO23 只旁聽 INT3 訊號；MCU 不產生同步脈衝。請避免舊版韌體曾將相連的 INT2 與 INT3 同時設成輸出。

附件 `.ino` 使用 Accel=`0x18`、Gyro=`0x69`，本版沿用。
Accel SDO1 若實際拉高，將 `ACC_ADDRESS` 改為 `0x19`；Gyro SDO2 拉低則為 `0x68`。
不要只依舊 header 裡的 `0x19` 巨集推斷目前硬體位址，應以實際 SDO 接法為準。

## 3. 「使用中斷」與「感測器資料同步」的差別

單純對兩個 GPIO 執行 `attachInterrupt()`，只是讓 MCU 得知各自資料就緒。
INT2 與 INT3 短接提供同步訊號的實體路徑；還必須載入 Bosch 設定資料、啟用同步模式、設定輸入／輸出及讀取正確的資料暫存器。

本版初始化順序：

1. 確認 Accel ID=`0x1E` 與 Gyro ID=`0x0F`；Gyro 預設不重置（可用開關對照），先將 Accel `0x7C=0x03`、`0x7D=0x00`，遵守等待時間並回讀確認，再執行 Accel 軟重置。
2. `bmi08a_load_config_file()` 載入 Bosch 同步設定資料並檢查 INTERNAL_STATUS。
3. 設定 Accel ±6 g、Gyro ±2000 deg/s 及同步模式。
4. INT2=`BMI08_ACCEL_SYNC_INPUT`，作為 Gyro 的同步輸入。
5. INT1=`BMI08_ACCEL_INT_SYNC_DATA_RDY`，輸出同步資料就緒。
6. INT3=`BMI08_GYRO_INT_DATA_RDY`，輸出給 INT2 及 GPIO23；INT4 不映射 DRDY。
7. 回讀重要方向、範圍與中斷映射，再安裝 MCU 上升緣中斷。

Bosch 同步引擎會依 Gyro 的時間基準處理／插值 Accel 資料及補償群延遲。
不能將此解讀為 MCU 同時啟動兩個 ADC，也不能以兩個中斷頻率相等就證明所有資料已同步。
INT1 與 INT3 可以有固定相位差，無須邊緣完全重疊。

## 4. ISR 與主迴圈分工

`onAccReady()` 只記錄 MCU 端邊緣時間及累計次數；`onGyroReady()` 只累計次數。
ISR callback 本體標示為 SRAM 執行，但 Arduino core 的 GPIO 中斷分派仍有其延遲，這不代表完整中斷路徑均已搬到 SRAM。

`loop()` 原子擷取計數／時間戳後呼叫 `readSynchronized()`。
ISR 內沒有 Wire、Serial、浮點換算或 delay。關中斷只用於極短的計數 snapshot，I²C 交易期間中斷仍開啟。
本例所有 driver / Wire 存取均在 Core0；未使用 Core1，也未提供跨核心 I²C mutex。

同步讀取由 Bosch `bmi08a_get_synchronized_data()` 完成：

| 資料 | I²C die | 暫存器 |
|---|---|---|
| 同步 Accel X/Y | Accel | `0x1E–0x21` |
| 同步 Accel Z | Accel | `0x27–0x28` |
| Gyro X/Y/Z | Gyro | `0x02–0x07` |

一般 Accel `0x12–0x17` 是原始資料路徑；不能在此同步模式中拿它代替同步輸出。
原本每 50 ms 讀一次且 delay 的流程已移除；序列摘要每秒一次。

## 5. 更新率與 I²C 負載

預設 `SYNC_RATE = BMI088::SyncRate::Hz400`；I²C=400 kHz。

| 模式 | Accel 原生 ODR | Gyro ODR / BW | 同步輸出 | 本範例 |
|---|---:|---:|---:|---|
| 400 Hz | 400 Hz | 400 Hz / 47 Hz | 400 Hz | 預設 |
| 1000 Hz | 800 Hz | 1000 Hz / 116 Hz | 1000 Hz | 可選，需看漏讀統計 |
| 2000 Hz | 1600 Hz | 2000 Hz / 230 Hz | 2000 Hz | 此 I²C 範例不開放 |

表中 ODR/BW 由本次附帶的 Bosch API 設定。1 kHz 同步 Accel 由內部處理提供，並非 Accel 原生 ODR 是 1 kHz。
不要在 `beginSync()` 後自行更改單獨的 ODR/BW，否則不再符合同步模式設定。

Bosch 的同步讀取有三筆 I²C 交易，共讀取 12 bytes；加上每筆的位址及 register address，理想匯流排時間約為
`(12 + 3×3)×9 / 400000 = 472.5 µs`，尚未含軟體、起停條件等開銷。
2 kHz 週期僅 500 µs，餘裕不足；若之後要 1–2 kHz 飛控整合，需重新量測整體時序，2 kHz 優先考慮 SPI。
本例的 `us` 是同步讀取呼叫觀測到的最長時間，不是整個 loop 時間，也不是 ISR latency。

## 6. 錯誤與資料新鮮度

- I²C callback 檢查每筆 `endTransmission()` 狀態、回傳長度與可讀長度。
- 同步設定上傳以 16 bytes 分塊；除了 Bosch 狀態碼，再检查上傳期間是否有任何 I²C 錯誤，避免上游迴圈後續成功覆蓋前面的錯誤。
- 三軸／六軸換算完成後才發布新 `Sample`；部分讀取失敗時 caller 的輸出不更新。
- pending 中斷數大於 1 時，只讀最新暫存器一次；舊事件計為 `skip`，不假裝可補回歷史樣本。本例沒有 FIFO。
- 超過四分之一輸出週期才開始服務事件，計為 `late` 並等下一筆。
- 讀取期間出現新 INT1 或 INT3，或讀取跨過一個週期，計為 `cross`，不發布這組資料。
- 超過五個輸出週期沒有有效樣本，顯示 `fresh=0`，六軸印為 `nan`，避免把靜止的舊值當健康資料。
- USB 未連線或 TX buffer 不足時丟棄該次報告。此策略減少阻塞，但 USB/IRQ 本身仍有執行成本。

這些是依 MCU 觀察到的邊緣做的保守防護，不是硬體原子快照的證明；漏掉實體脈衝、長時間 IRQ 遮蔽等仍須示波器／邏輯分析儀實測。

## 7. 上板驗收

開機印出的重要暫存器通常應包含以下值（未列者依 mode 或保留位不同）：

| die / register | 預期 | 含義 |
|---|---|---|
| ACC `0x2A` | `0x01` | config 初始化成功 |
| ACC `0x41` | 低 2 bits=`1` | ±6 g |
| ACC `0x53` | `0x0A` | INT1 高有效 push-pull output |
| ACC `0x54` | `0x13` | INT2 edge input、output 關閉 |
| ACC `0x56` | bit0=`1` | 同步 DRDY 映射 INT1 |
| ACC `0x58` | DRDY bits=`0` | 未映射普通 Accel DRDY |
| GYR `0x15` | bit7=`1` | DRDY enable |
| GYR `0x16` | 低 2 bits=`1` | INT3 高有效 push-pull |
| GYR `0x18` | bit0=`1`、bit7=`0` | DRDY 只映射 INT3 |

每秒一行的欄位：

| 欄位 | 判讀 |
|---|---|
| `A` | GPIO22 中斷頻率；400 Hz 模式時約 400 |
| `G` | GPIO23 中斷頻率；400 Hz 模式時約 400 |
| `OK` | 實際成功且未被跨週期檢查拒絕的六軸資料組數／秒 |
| `skip` | MCU 未逐筆服務的已觀察到事件數，累計 |
| `late` | 太晚才開始讀取而丟棄，累計 |
| `cross` | 讀取期間更新，該組資料丟棄，累計 |
| `fail` | 同步讀取失敗，累計 |
| `bus` | 所有 driver I²C 交易錯誤，包含初始化／溫度／diagnostic，累計 |
| `us` | 同步讀取最大耗時，µs |
| `fresh` | 最近是否仍有有效資料；不等同完整 IMU 健康認證 |
| `a` | ax,ay,az，m/s²，含重力、未扣 offset |
| `g` | gx,gy,gz，deg/s |
| `T` | 溫度，°C |

靜置時加速度向量長度應接近 9.81 m/s²；哪一軸正／負取決於感測器安裝方向。
Gyro 靜置應接近 0，但尚未扣除偏置，不能要求每一軸完全為零。

若 G≈400、A=0：優先檢查 INT3–INT2 連線、INT2 方向、同步 config 和 GPIO22 接線。
若 A/G 都正常而 OK 明顯較低：查看 `late/cross/us`，以邏輯分析儀量測 INT1、INT3 與 I²C 讀取相位；不要直接刪掉跨週期保護。
若 `bus/fail` 持續增加：優先檢查供電、pull-up、I²C 電平與接線品質。
斷開 INT3–INT2 可作為同步路徑驗證：此時不應繼續宣稱同步 Accel 資料健康；請在斷電狀態變更接線。

## 8. 與原程式的 API 差異

保留類別名稱 `BMI088` 與接受 `TwoWire&` 的 constructor，但同步版**不是舊 driver 的全部 API 相容替代品**。
原本 `initialize()`、獨立 `getAcceleration()/getGyroscope()` 和個別 ODR setters 不再公開，避免呼叫端誤用非同步路徑。

```cpp
BMI088 imu(Wire, 0x18, 0x69);
// setup(): Wire already configured
bool initialized = imu.beginSync(BMI088::SyncRate::Hz400);
// loop(): ONLY after synchronized INT1 event, with timing checks as in sketch
BMI088::Sample s;
if (initialized && imu.readSynchronized(s)) {
    // a: m/s², g: deg/s; then apply application calibration / fusion
}
```

要整合進 CURIO 飛控時，應再處理座標轉換、gyro/accel 校正、估測器 dt、Core0 時序、sensor failure 與控制器 failsafe。本例本身不做此整合。

## 9. 來源與版本

- Bosch 官方 SensorAPI：https://github.com/boschsensortec/BMI08x_SensorAPI
- 隨附版本 commit：`c1ed227e7bb7da1fa600bbd4e5c82d0da1eb416a`
- 同步說明：https://github.com/boschsensortec/BMI08x_SensorAPI/blob/c1ed227e7bb7da1fa600bbd4e5c82d0da1eb416a/DataSync.md
- BMI088 datasheet：https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
- Arduino-Pico Wire：https://arduino-pico.readthedocs.io/en/latest/wire.html

`src/bosch` 原始檔保留原作者 BSD-3-Clause 授權，未修改其實作。
本版的接脚方向選擇與範圍換算以實際 API／register 行為為準；官方 example 中部分註解與選定 range 不一致，未照抄其換算常數。

完整驗證狀態請見 `VALIDATION.md`。本次沒有連接使用者的 CURIO/BMI088 實板，編譯及軟體測試不等同電氣訊號或硬體同步驗證。
