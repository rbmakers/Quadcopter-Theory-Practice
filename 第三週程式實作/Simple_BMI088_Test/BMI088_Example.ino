// 製作 : 火箭鳥創客倉庫

#include "BMI088.h"

// Hardware Pin Definitions for the CURIO setup
const int I2C0_SDA = 20;
const int I2C0_SCL = 25;

#define BMI088_ACC_ADDR  0x18  
#define BMI088_GYRO_ADDR 0x69 

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0; 

// Initialize the driver passing Wire (i2c0 block) and your explicitly defined addresses
BMI088 bmi088(Wire, BMI088_ACC_ADDR, BMI088_GYRO_ADDR);

void setup(void) { 
    // Configure RP2354A specific I2C0 hardware pins before starting the bus
    Wire.setSDA(I2C0_SDA);
    Wire.setSCL(I2C0_SCL);
    Wire.begin(); 

    Serial.begin(115200); 
    while (!Serial); 
    Serial.println("BMI088 Raw Data Initialization..."); 

    while (1) { 
        if (bmi088.isConnection()) { 
            bmi088.initialize(); 
            Serial.println("BMI088 is successfully connected!"); 
            break; 
        } else {
            Serial.println("BMI088 connection failed. Checking pins..."); 
        } 
        delay(2000); 
    }
}

void loop(void) {
    bmi088.getAcceleration(&ax, &ay, &az); 
    bmi088.getGyroscope(&gx, &gy, &gz); 
    temp = bmi088.getTemperature(); 

    // Print Formatted IMU outputs
    Serial.print(ax); 
    Serial.print(","); 
    Serial.print(ay); 
    Serial.print(","); 
    Serial.print(az); 
    Serial.print(","); 

    Serial.print(gx); 
    Serial.print(","); 
    Serial.print(gy); 
    Serial.print(","); 
    Serial.print(gz); 
    Serial.print(","); 

    Serial.print(temp); 
    Serial.println(); 
    
    delay(50); // Loop execution speed optimized for telemetry readout 
}
