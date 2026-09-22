RP2354A 三色 LED 教學範例
GPIO7=Blue、GPIO8=Red、GPIO9=Yellow；預設 HIGH 點亮，各 LED 需串聯限流電阻。

Arduino IDE 2.x + Earle Philhower Arduino-Pico 5.6.0。
自製 RP2354A 板：Generic RP2350 / RP2350A / 2MB (no FS) / ARM / 150 MHz。
每個資料夾是一份獨立 Sketch：開啟同名 .ino，一次上傳一份；不要合併到同一個 Sketch。

01_While_SoftwarePWM：while 忙等，64-slot 軟體 PWM，交錯呼吸。
02_TimerIRQ_SoftwarePWM：50us repeating timer IRQ，軟體 PWM + 查表呼吸。
03_PIO_PWM：3個 PIO SM，7條共用指令，CPU 更新包絡，PIO 自主輸出載波。
04_Millis_NeonChase：非阻塞開關式霓虹跑馬燈，無 PWM。
05_HardwarePWM_Breathe：analogWrite 硬體 PWM + millis 交錯呼吸。

完整原理、完整程式、接線、PIO 指令、公式、比較與實驗請閱讀同包 HTML。
五份程式已實際交叉編譯；未在使用者實機燒錄或量測。
PIO 衍生程式授權見 THIRD_PARTY_NOTICES.txt。
