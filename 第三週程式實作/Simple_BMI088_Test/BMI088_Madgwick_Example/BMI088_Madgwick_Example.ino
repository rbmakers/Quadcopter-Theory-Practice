#include "BMI088.h"

// 1. 硬體引腳定義 (RP2354A) [cite: 1]
const int I2C0_SDA = 20; // [cite: 1]
const int I2C0_SCL = 25; // [cite: 1]
#define BMI088_ACC_ADDR  0x18  // [cite: 2]
#define BMI088_GYRO_ADDR 0x69  // [cite: 2]

// 2. 感測器原始與轉換變數 [cite: 2, 3]
float ax = 0, ay = 0, az = 0; // [cite: 2]
float gx = 0, gy = 0, gz = 0; // [cite: 3]
int16_t temp = 0; // [cite: 3]

// 3. 校正偏置變數 (Offsets)
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// 4. Madgwick 濾波器參數與變數
#define beta 0.1f   // 演算法增益 (Beta)，控制加速度計修正陀螺儀的強度。數值愈大收斂愈快，但易受震動干擾

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元數姿態
unsigned long lastUpdate = 0;                      // 計算時間差 Delta t

// 宣告驅動實例 [cite: 4]
BMI088 bmi088(Wire, BMI088_ACC_ADDR, BMI088_GYRO_ADDR); // [cite: 4]

// 函式宣告
void calibrateSensors();
void MadgwickAHRSupdateIMU(float dt, float gx, float gy, float gz, float ax, float ay, float az);

void setup(void) { 
    // 設定 RP2354A 的 I2C 引腳並啟動 [cite: 5]
    Wire.setSDA(I2C0_SDA); // [cite: 5]
    Wire.setSCL(I2C0_SCL); // 
    Wire.begin(); // 

    Serial.begin(115200); // 
    while (!Serial); // 
    Serial.println("BMI088 IMU 初始化 (Madgwick)..."); 

    while (1) { 
        if (bmi088.isConnection()) { // 
            bmi088.initialize(); // 
            Serial.println("BMI088 連線成功！"); // [cite: 7]
            break; 
        } else {
            Serial.println("BMI088 連線失敗，請檢查接線..."); // [cite: 7]
        } 
        delay(2000); // [cite: 8]
    }

    // 執行水平靜置校正
    calibrateSensors();
    lastUpdate = micros();
}

void loop(void) {
    // 1. 讀取感測器數據 [cite: 8]
    bmi088.getAcceleration(&ax, &ay, &az); // [cite: 8]
    bmi088.getGyroscope(&gx, &gy, &gz); // [cite: 9]
    temp = bmi088.getTemperature(); // [cite: 9]

    // 2. 套用校正補償
    float ax_cal = ax - ax_offset;
    float ay_cal = ay - ay_offset;
    float az_cal = az - az_offset; 
    
    // Madgwick 演算法需要弧度制的角速度 (rad/s)
    float gx_rad = (gx - gx_offset) * DEG_TO_RAD;
    float gy_rad = (gy - gy_offset) * DEG_TO_RAD;
    float gz_rad = (gz - gz_offset) * DEG_TO_RAD;

    // 將扣除的重力加回，因為融合演算法需要完整的重力向量來識別「下」的方向
    float ax_fusion = ax_cal;
    float ay_fusion = ay_cal;
    float az_fusion = az_cal + 9.80665f; 

    // 3. 計算時間差 Delta t
    unsigned long now = micros();
    float dt = ((float)(now - lastUpdate)) / 1000000.0f;
    lastUpdate = now;

    if (dt <= 0.0f) dt = 0.001f; // 避免極端異常的 dt

    // 4. 更新 Madgwick 姿態融合
    MadgwickAHRSupdateIMU(dt, gx_rad, gy_rad, gz_rad, ax_fusion, ay_fusion, az_fusion);

    // 5. 將四元數轉換為歐拉角 (Roll, Pitch, Yaw) 
    float roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    float pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * RAD_TO_DEG;
    float yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

    // 6. 輸出至 Serial Plotter (格式：名稱1:數值1,名稱2:數值2,...)
    Serial.print("Roll:");  Serial.print(roll);  Serial.print(",");
    Serial.print("Pitch:"); Serial.print(pitch); Serial.print(",");
    Serial.print("Yaw:");   Serial.println(yaw);

    delay(10); // [cite: 10]
}

// ----------------------------------------------------------------
// 1) 靜置水平校正函式
// ----------------------------------------------------------------
void calibrateSensors() {
    Serial.println("正在進行水平靜置校正，請勿晃動飛控板...");
    delay(500);

    long num_samples = 500;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < num_samples; i++) {
        float raw_ax, raw_ay, raw_az;
        float raw_gx, raw_gy, raw_gz;
        
        bmi088.getAcceleration(&raw_ax, &raw_ay, &raw_az);
        bmi088.getGyroscope(&raw_gx, &raw_gy, &raw_gz);

        sum_ax += raw_ax;
        sum_ay += raw_ay;
        sum_az += raw_az;
        sum_gx += raw_gx;
        sum_gy += raw_gy;
        sum_gz += raw_gz;
        delay(4);
    }

    ax_offset = sum_ax / num_samples;
    ay_offset = sum_ay / num_samples;
    az_offset = (sum_az / num_samples) - 9.80665f; // 扣除標準重力加速度

    gx_offset = sum_gx / num_samples;
    gy_offset = sum_gy / num_samples;
    gz_offset = sum_gz / num_samples;

    Serial.println("校正完成！");
    delay(1000);
}

// ----------------------------------------------------------------
// 2) Madgwick IMU 姿態融合演算法 (不含磁力計版本) - 修正版
// ----------------------------------------------------------------
void MadgwickAHRSupdateIMU(float dt, float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    // 這裡補上了 _4q3 宣告
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _4q3, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 如果加速度計皆為 0 則跳過修正（避免除以零）
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 1. 數據單位化
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 重複運算項優化（這裡補上了 _4q3 的計算）
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _4q3 = 4.0f * q3; // <--- 修正：補上這一行
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 2. 梯度下降法 (Gradient Descent) 求解目標函數與雅可比矩陣
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = _4q3 * q1q1 - _2q1 * ax + _4q3 * q2q2 - _2q2 * ay;
        
        // 單位化梯度
        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 3. 結合陀螺儀變換率與梯度修正量
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;
    } else {
        // 如果無加速度修正，僅使用陀螺儀積分
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    }

    // 4. 四元數積分更新
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // 5. 規範化四元數
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}