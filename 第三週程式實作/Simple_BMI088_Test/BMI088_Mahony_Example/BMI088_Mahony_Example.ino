//火箭鳥創客倉庫

#include "BMI088.h"

// 1. 硬體引腳定義 (RP2354A)
const int I2C0_SDA = 20;
const int I2C0_SCL = 25;
#define BMI088_ACC_ADDR  0x18  
#define BMI088_GYRO_ADDR 0x69 

// 2. 感測器原始與轉換變數
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;

// 3. 校正偏置變數 (Offsets)
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// 4. Mahony 濾波器參數與變數
#define kp 2.0f      // 比例增益（控制加速度計修正陀螺儀的速度）
#define ki 0.005f    // 積分增益（控制消除陀螺儀常數漂移的速度）

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元數
float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;    // 積分誤差
unsigned long lastUpdate = 0;                      // 計算時間差 Delta t

// 宣告驅動實例
BMI088 bmi088(Wire, BMI088_ACC_ADDR, BMI088_GYRO_ADDR);

// 函式宣告
void calibrateSensors();
void MahonyAHRSupdateIMU(float dt, float gx, float gy, float gz, float ax, float ay, float az);

void setup(void) { 
    // 設定 RP2354A 的 I2C 引腳並啟動
    Wire.setSDA(I2C0_SDA);
    Wire.setSCL(I2C0_SCL);
    Wire.begin(); 

    Serial.begin(115200); 
    while (!Serial); 
    Serial.println("BMI088 IMU 初始化中..."); 

    while (1) { 
        if (bmi088.isConnection()) { 
            bmi088.initialize();
            Serial.println("BMI088 連線成功！"); 
            break; 
        } else {
            Serial.println("BMI088 連線失敗，請檢查接線...");
        } 
        delay(2000); 
    }

    // 執行水平靜置校正
    calibrateSensors();
    lastUpdate = micros();
}

void loop(void) {
    // 1. 讀取感測器數據
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz); 
    temp = bmi088.getTemperature(); 

    // 2. 套用校正補償
    float ax_cal = ax - ax_offset;
    float ay_cal = ay - ay_offset;
    float az_cal = az - az_offset; // 已在校正中扣除重力，此時水平靜置應接近 0
    
    // Mahony 演算法需要弧度制的角速度 (rad/s)
    float gx_rad = (gx - gx_offset) * DEG_TO_RAD;
    float gy_rad = (gy - gy_offset) * DEG_TO_RAD;
    float gz_rad = (gz - gz_offset) * DEG_TO_RAD;

    // 若要維持 Mahony 標準輸入（加速度計含重力），需將扣除的重力加回
    float ax_fusion = ax_cal;
    float ay_fusion = ay_cal;
    float az_fusion = az_cal + 9.80665f; 

    // 3. 計算時間差 Delta t
    unsigned long now = micros();
    float dt = ((float)(now - lastUpdate)) / 1000000.0f;
    lastUpdate = now;

    // 避免極端異常的 dt
    if (dt <= 0.0f) dt = 0.001f;

    // 4. 更新 Mahony 姿態融合
    MahonyAHRSupdateIMU(dt, gx_rad, gy_rad, gz_rad, ax_fusion, ay_fusion, az_fusion);

    // 5. 將四元數轉換為歐拉角 (Roll, Pitch, Yaw) 以利觀測
    float roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    float pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * RAD_TO_DEG;
    float yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

    // 6. 輸出至 Serial Plotter (格式：名稱1:數值1,名稱2:數值2,...)
    Serial.print("Roll:");  Serial.print(roll);  Serial.print(",");
    Serial.print("Pitch:"); Serial.print(pitch); Serial.print(",");
    Serial.print("Yaw:");   Serial.println(yaw);

    delay(10); // 約 100Hz 更新率，適合 Serial Plotter 繪圖與姿態收斂
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

    // 計算平均偏置
    ax_offset = sum_ax / num_samples;
    ay_offset = sum_ay / num_samples;
    // 假設校正時晶片絕對水平朝上，Z 軸會量到 +1G 的重力 (約 9.80665 m/s^2)
    az_offset = (sum_az / num_samples) - 9.80665f; 

    gx_offset = sum_gx / num_samples;
    gy_offset = sum_gy / num_samples;
    gz_offset = sum_gz / num_samples;

    Serial.println("校正完成！");
    Serial.print("Acc Offsets: "); Serial.print(ax_offset); Serial.print(", "); Serial.print(ay_offset); Serial.print(", "); Serial.println(az_offset);
    Serial.print("Gyro Offsets: "); Serial.print(gx_offset); Serial.print(", "); Serial.print(gy_offset); Serial.print(", "); Serial.println(gz_offset);
    delay(1000);
}

// ----------------------------------------------------------------
// 2) Mahony IMU 姿態融合演算法
// ----------------------------------------------------------------
void MahonyAHRSupdateIMU(float dt, float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 如果加速度計皆為 0 則不進行修正（避免除以零）
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 單位化加速度計量測值
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 計算估計的重力方向（從目前四元數推導地理座標系的 Z 軸）
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 誤差為「量測重力方向」與「估計重力方向」的外積 (Cross Product)
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 計算並套用積分項（消除常數漂移）
        if(ki > 0.0f) {
            eIntX += halfex * ki * dt;
            eIntY += halfey * ki * dt;
            eIntZ += halfez * ki * dt;
            gx += eIntX;
            gy += eIntY;
            gz += eIntZ;
        } else {
            eIntX = 0.0f;
            eIntY = 0.0f;
            eIntZ = 0.0f;
        }

        // 套用比例項修正角速度
        gx += kp * halfex;
        gy += kp * halfey;
        gz += kp * halfez;
    }

    // 積分四元數並更新
    dt = 0.5f * dt;
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * dt;
    q1 += (qa * gx + qc * gz - q3 * gy) * dt;
    q2 += (qa * gy - qb * gz + q3 * gx) * dt;
    q3 += (qa * gz + qb * gy - qc * gx) * dt;

    // 規範化四元數
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
