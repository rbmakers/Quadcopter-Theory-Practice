# CURIO RP2354A BMI088：Mahony / Madgwick 六軸姿態融合

開啟 `BMI088_Fusion/BMI088_Fusion.ino`。同資料夾已包含驅動、Bosch 同步設定與兩套算法，不需另裝 AHRS 函式庫。

## 接線與開發環境

使用 Earle Philhower Arduino-Pico、RP2350 ARM 核心（既有 CURIO 板設定優先）。保留已成功的 v1.2 BMI088 驅動：不執行 Gyro soft reset。

| 功能 | 接線 / 設定 |
|---|---|
| I²C SDA / SCL | GPIO20 / GPIO25；400 kHz |
| Accel / Gyro I²C 位址 | 0x18 / 0x69 |
| Accel INT1 | GPIO22，同步資料就緒 |
| Gyro INT3 | GPIO23，並與 Accel INT2 短接 |
| Accel INT2 | 感測器輸入；不可設成輸出 |
| 預設融合資料率 | Bosch 同步 1000 Hz |
| 預設繪圖速率 | 約 50 Hz；顯示最新結果 |

## 使用 Serial Plotter

1. 上傳程式；保持板子完全靜止。開機先暖機 2 秒，再收集至少 3 秒資料估算 gyro 三軸零偏。校正期間偵測移動會重新收集；板子不一定要完全水平，但不要晃動。長時間沒有曲線時，先在 Serial Monitor 確認初始化沒有失敗，再重置並保持靜止。
2. 打開 Arduino IDE 的 **Tools → Serial Plotter**，選擇 **115200**。初始化及校正完成後，只輸出 Plotter 格式。若 Plotter 開啟觸發重置，重新靜置約 5 秒以上。
3. 緩慢繞感測器 X、Y、Z 軸旋轉，分別觀察 Roll、Pitch、Yaw。預設六條曲線：`Mahony_Roll`、`Mahony_Pitch`、`Mahony_Yaw`、`Madgwick_Roll`、`Madgwick_Pitch`、`Madgwick_Yaw`。單位都是度。
4. 兩套算法相近時曲線可能重疊；可在 Plotter 勾選曲線，或修改下方設定只輸出一套。

Serial Plotter 的橫軸是連續資料點，並非帶真實秒數的時間戳。50 Hz 時每點約 20 ms；USB 延遲或丟棄顯示資料時，不能用點數作精密時間量測。沒有將時間戳當成第七條曲線，以免拉大縱軸。

## 常用設定

在 `.ino` 上方修改：

```cpp
constexpr BMI088::SyncRate SYNC_RATE = BMI088::SyncRate::Hz1000;
constexpr uint32_t PLOT_HZ = 50;
constexpr int PLOT_MODE = 0;       // 0=兩套，1=Mahony，2=Madgwick
constexpr bool DIAGNOSTICS = false;
```

需要回到已驗證的 400 Hz 時，只把 `Hz1000` 改成 `Hz400`；取樣週期與校正樣本數會一起調整。I²C 仍維持 400 kHz。不要用 `delay(1)` 將融合頻率固定為 1 kHz：本例依已接受樣本的中斷時間戳計算實際 dt。

`Fusion6D.h` 的預設增益：Mahony `kp=2.0, ki=0.0`，Madgwick `beta=0.05`。這是初始調校值，需按震動與運動情況實測。提高 kp 或 beta 通常更快修正傾角，也更容易受非重力加速度干擾。Mahony 使用完整向量叉積，kp / ki 定義可能與其他函式庫的 twoKp / twoKi 不同。

## 角度與限制

- 採感測器原生右手座標，沒有自動改成飛控 FRD 軸。平放且 Z 軸朝上時，az 約 +9.81 m/s²；Roll 繞 X、Pitch 繞 Y、Yaw 繞 Z。若模組安裝軸不同，必須對 accel 與 gyro 使用相同、保持右手性的軸映射。
- Quaternion 表示 sensor → reference 的旋轉，輸出 ZYX Euler。Roll / Yaw 約 -180°～+180°，Pitch -90°～+90°。跨越 ±180° 時會跳變；Pitch 接近 ±90° 時 Euler 表示有奇異性，不代表 Quaternion 失效。
- 初始 roll / pitch 由靜止平均重力方向決定，初始 yaw=0。BMI088 無磁力計，無法取得絕對航向；Yaw 長時間漂移是六軸融合的限制，兩套算法都無法以重力校正航向。
- 陀螺儀由驅動的 deg/s 轉成 rad/s，扣除開機零偏後才更新融合。加速度保留重力，不會把靜止的 Z 軸重力扣掉。
- 加速度大小不在 0.75g～1.25g 時暫停重力修正，只積分 gyro。這個簡單門檻不能排除所有線性加速度；急加速、震動仍可能造成傾角誤差。
- 靜止校正只檢查有限門檻，不能辨識所有緩慢運動；校正時確實保持不動。溫漂仍需另外處理。

## 執行架構與偵錯

Core0 處理 I²C、IRQ 樣本一致性檢查、校正與兩套融合。ISR 只計數及記錄時間。Core1 負責浮點轉字串與 USB 輸出，使用短時間 mutex 複製最新顯示資料；Core0 取不到鎖就丟棄該次顯示更新，不等待 USB。不在取樣路徑讀取溫度。

六條曲線來自同一筆已接受的同步樣本。I²C 讀取跨過下一個 IRQ、服務太晚或讀取失敗時，不更新算法。短暫遺失樣本以實際 dt 積分目前 gyro，無法還原中間未知運動。間隔超過 20 ms 時跳過該步並保留原姿態，`gaps` 增加；嚴重中斷後姿態可能不準，請靜置並重置重新校正。停止收到新資料時，停止送出曲線點，避免假裝舊姿態仍是新資料。

若要確認 1 kHz 效能，把 `DIAGNOSTICS=true`，改用 Serial Monitor。每秒顯示：

- `OK`：兩套融合成功更新次數/秒（第一行為 0，第二行起才可比較）。
- `skip/late/cross/fail/bus`：累積跳過、服務過晚、跨 IRQ、不成功更新/讀取、I²C 錯誤。
- `gaps`：超過 20 ms 或無效時間間隔。
- `read_us`：最大感測器讀取時間；`work_us`：最大讀取加處理時間，包含校正，可能包含中斷開銷。

這些效能值需要在您的實際板子驗證；既有純感測器測試的約 602 µs 讀取時間不代表加入兩套融合後總時間也相同。若 late/skip 明顯累積，可先回到 400 Hz 比較。診斷模式與 Plotter 模式互斥，避免統計值干擾角度曲線。

## 來源與授權

兩套融合採獨立的方程式實作：Mahony 重力叉積回授與 Madgwick 原始 IMU-only 梯度下降（不是新版 x-io Fusion API）。算法參考入口：https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/ 。

Bosch 原始驅動保留原檔及 `src/bosch/LICENSE`。沿用專案之 BMI088.h/.cpp 保留其既有聲明。本次新寫的 Fusion6D.h、BMI088_Fusion.ino 與驗證測試依附帶 LICENSE_FUSION.txt 的 MIT 授權提供。
