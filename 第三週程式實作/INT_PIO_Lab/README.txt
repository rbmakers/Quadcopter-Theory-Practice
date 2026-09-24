INT vs PIO 教學實驗 v1.0
Arduino-Pico 5.6.0 / Generic RP2350 / ARM / 150 MHz / Pico SDK USB。
每個 Lab 資料夾是獨立 Arduino sketch；開啟同名 .ino。勿合併三個資料夾。
Lab01、Lab02：GPIO2 輸出 PWM，跳線至 GPIO6 與 GPIO7。需確認板上腳位可用。
Lab03：BMI088 SDA20 SCL25 ACC0x18 GYR0x69；INT1→22，INT3→23 並連 INT2。
GPIO2 不得接到 BMI088 INT 輸出。程式沒有馬達控制。
Serial Monitor 115200：n 一般、b CPU忙碌但IRQ開啟、m 關中斷1200us、r 重複、d 匯出。
先 n，再 b，再 m，各重複至少20次。量測期間不輸出Serial。
CAPTURE valid=1、rxstall=0、dma_error=0、remaining=0，所有 OVERFLOW=0 才採信資料。
PIO 1MHz、65536 samples、65.536ms，16KiB DMA緩衝。排除前後2ms統計。
三份sketch已交叉編譯；尚未實機驗證。Lab03僅初始化同步並量測事件，不執行資料讀取或融合。
完整理論、限制與實驗流程請見 HTML 教材。
