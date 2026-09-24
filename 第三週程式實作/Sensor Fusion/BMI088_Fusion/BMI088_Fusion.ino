/* RP2354A / Arduino-Pico: BMI088 synchronized interrupts + two 6DOF filters.
 * Sensor frame, right-handed; level acceleration +Z; gyro deg/s -> rad/s.
 * Core0: sensor + filters. Core1: USB plotting. No Wire inside ISR/core1.
 */
#include "BMI088.h"
#include "Fusion6D.h"
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <pico/platform.h>
#include <pico/mutex.h>
#include <atomic>
#include <stdio.h>

bool core1_separate_stack=true; // independent 8 KiB stacks for both cores
constexpr BMI088::SyncRate SYNC_RATE=BMI088::SyncRate::Hz1000;
constexpr uint32_t PERIOD_US=1000000UL/static_cast<uint16_t>(SYNC_RATE);
constexpr uint32_t PLOT_HZ=50;
constexpr int PLOT_MODE=0; // 0=both (6 curves), 1=Mahony, 2=Madgwick
static_assert(PLOT_HZ>0 && PLOT_HZ<=static_cast<uint16_t>(SYNC_RATE), "Invalid plot frequency");
static_assert(PLOT_MODE>=0 && PLOT_MODE<=2, "Invalid plot mode");
constexpr bool DIAGNOSTICS=false; // true: Serial Monitor stats, not Plotter
constexpr float GRAVITY=9.80665f;
constexpr uint32_t CAL_SAMPLES=3*static_cast<uint16_t>(SYNC_RATE);
BMI088 imu(Wire,0x18,0x69);
Fusion6D::Mahony mahony;
Fusion6D::Madgwick madgwick;
volatile uint32_t accCount=0,gyrCount=0,accUs=0;
void __not_in_flash_func(accISR)(){accUs=time_us_32();++accCount;}
void __not_in_flash_func(gyrISR)(){++gyrCount;}
struct Irqs {uint32_t a,g,us;};
Irqs irqs();
Irqs irqs(){uint32_t s=save_and_disable_interrupts();Irqs v{accCount,gyrCount,accUs};restore_interrupts(s);return v;}
struct Frame {
    Fusion6D::Euler m,d;
    uint32_t us=0,seq=0,skip=0,late=0,cross=0,fail=0,bus=0,gaps=0,readMax=0,workMax=0;
};
mutex_t frameMutex;
Frame sharedFrame;
std::atomic<bool> plottingReady{false};
uint32_t consumed=0,previousUs=0,accepted=0,skipped=0,late=0,crossed=0,failed=0,gaps=0;
uint32_t maxRead=0,maxWork=0,startMs=0,lastPublishUs=0;
bool calibrated=false;
uint32_t calN=0;
double sum[6]={},sqGyro[3]={};
float bias[3]={},firstAccel[3]={};
void clearCalibration(){calN=0;for(int i=0;i<6;++i)sum[i]=0;for(int i=0;i<3;++i)sqGyro[i]=0;}
bool calibrate(const BMI088::Sample &s);
void process(const BMI088::Sample &s,uint32_t stamp);

bool calibrate(const BMI088::Sample &s){
    if(millis()-startMs<2000) return false; // short startup warm-up
    float a[3]={s.ax,s.ay,s.az},g[3]={s.gx,s.gy,s.gz};
    float norm=sqrtf(s.ax*s.ax+s.ay*s.ay+s.az*s.az);
    bool stationary=isfinite(norm)&&fabsf(norm-GRAVITY)<0.8f;
    for(int i=0;i<3;++i) stationary &= isfinite(g[i])&&fabsf(g[i])<3.0f;
    if(calN)for(int i=0;i<3;++i)stationary &= fabsf(a[i]-firstAccel[i])<0.35f;
    if(!stationary){clearCalibration();return false;}
    if(!calN)for(int i=0;i<3;++i)firstAccel[i]=a[i];
    for(int i=0;i<3;++i){sum[i]+=a[i];sum[i+3]+=g[i];sqGyro[i]+=double(g[i])*g[i];}
    if(++calN<CAL_SAMPLES)return false;
    for(int i=0;i<3;++i){
        double mean=sum[i+3]/calN;
        if(sqGyro[i]/calN-mean*mean>0.5*0.5){clearCalibration();return false;}
        bias[i]=mean;
    }
    mahony.reset(sum[0]/calN,sum[1]/calN,sum[2]/calN);
    madgwick.reset(sum[0]/calN,sum[1]/calN,sum[2]/calN);
    return true;
}
void process(const BMI088::Sample &s,uint32_t stamp){
    if(!calibrated){
        if(calibrate(s)){calibrated=true;previousUs=stamp;plottingReady.store(true,std::memory_order_release);}
        return;
    }
    uint32_t elapsed=stamp-previousUs;previousUs=stamp;
    // Preserve orientation across a long outage; do not integrate unknown motion.
    if(elapsed==0||elapsed>20000){++gaps;return;}
    float dt=elapsed*1e-6f;
    float gx=(s.gx-bias[0])*Fusion6D::RAD,gy=(s.gy-bias[1])*Fusion6D::RAD,gz=(s.gz-bias[2])*Fusion6D::RAD;
    float an=sqrtf(s.ax*s.ax+s.ay*s.ay+s.az*s.az);
    bool useAccel=isfinite(an)&&an>0.75f*GRAVITY&&an<1.25f*GRAVITY;
    bool okM=mahony.update(gx,gy,gz,s.ax,s.ay,s.az,dt,useAccel);
    bool okD=madgwick.update(gx,gy,gz,s.ax,s.ay,s.az,dt,useAccel);
    if(!okM||!okD){++failed;return;}
    ++accepted;
    if(stamp-lastPublishUs<1000000UL/PLOT_HZ)return;
    lastPublishUs=stamp;
    Frame f;
    f.m=mahony.q.euler();f.d=madgwick.q.euler();f.us=stamp;f.seq=accepted;
    f.skip=skipped;f.late=late;f.cross=crossed;f.fail=failed;f.bus=imu.i2cErrors();f.gaps=gaps;
    f.readMax=maxRead;f.workMax=maxWork;
    // Latest-only mailbox: never stall the sampling core for USB/other core.
    if(mutex_try_enter(&frameMutex,nullptr)){sharedFrame=f;mutex_exit(&frameMutex);}
}
void setup(){
    mutex_init(&frameMutex);
    Serial.begin(115200);
    uint32_t t=millis();while(!Serial&&millis()-t<3000)delay(10);
    Serial.println("CURIO BMI088 Mahony + Madgwick 6DOF v1.0");
    Serial.println("Keep completely STILL: 2 s warm-up + >=3 s gyro calibration; movement restarts calibration.");
    Serial.println("After calibration: open Serial Plotter 115200. Yaw is relative and drifts without magnetometer.");
    if(!Wire.setSDA(20)||!Wire.setSCL(25)){Serial.println("I2C pin error");while(true)delay(1000);}
    Wire.begin();Wire.setClock(400000);Wire.setTimeout(3);
    pinMode(22,INPUT);pinMode(23,INPUT);
    // Retain tested v1.2 startup: do NOT soft-reset gyro.
    if(!imu.beginSync(SYNC_RATE,&Serial,false)){
        Serial.print("INIT FAILED: ");Serial.println(imu.lastStage());imu.printLastBusError(Serial);
        while(true)delay(1000);
    }
    Serial.println("Calibrating... Plot output begins automatically when stable.");
    Serial.flush(); // only before acquisition starts
    attachInterrupt(digitalPinToInterrupt(22),accISR,RISING);
    attachInterrupt(digitalPinToInterrupt(23),gyrISR,RISING);
    consumed=irqs().a;startMs=millis();
}
void loop(){
    Irqs before=irqs();uint32_t pending=before.a-consumed;
    if(!pending)return;
    consumed=before.a;if(pending>1)skipped+=pending-1;
    uint32_t started=time_us_32();
    if(started-before.us>PERIOD_US/4){++late;return;}
    BMI088::Sample s;
    bool ok=imu.readSynchronized(s);
    uint32_t readTime=time_us_32()-started;if(readTime>maxRead)maxRead=readTime;
    Irqs after=irqs();
    if(!ok){++failed;return;}
    if(after.a!=before.a||after.g!=before.g||time_us_32()-before.us>=PERIOD_US){++crossed;return;}
    process(s,before.us);
    uint32_t work=time_us_32()-started;if(work>maxWork)maxWork=work;
}
void setup1(){} // Arduino-Pico launches core1; readiness uses release/acquire.
void loop1(){
    if(!plottingReady.load(std::memory_order_acquire)){delay(10);return;}
    static uint32_t lastSeq=0,lastStatsMs=0,oldSeq=0;
    Frame f;
    mutex_enter_blocking(&frameMutex);f=sharedFrame;mutex_exit(&frameMutex);
    if(f.seq==lastSeq||time_us_32()-f.us>100000){delay(1);return;}
    lastSeq=f.seq;
    char line[240];int n=0;
    if(DIAGNOSTICS){
        uint32_t now=millis(),ms=now-lastStatsMs;
        if(ms<1000){delay(1);return;}
        float hz=lastStatsMs?(f.seq-oldSeq)*1000.0f/ms:0;
        oldSeq=f.seq;lastStatsMs=now;
        n=snprintf(line,sizeof(line),"OK=%.1f skip=%lu late=%lu cross=%lu fail=%lu bus=%lu gaps=%lu read_us=%lu work_us=%lu\n",hz,
          (unsigned long)f.skip,(unsigned long)f.late,(unsigned long)f.cross,(unsigned long)f.fail,
          (unsigned long)f.bus,(unsigned long)f.gaps,(unsigned long)f.readMax,(unsigned long)f.workMax);
    }else if(PLOT_MODE==1){
        n=snprintf(line,sizeof(line),"Mahony_Roll:%.2f\tMahony_Pitch:%.2f\tMahony_Yaw:%.2f\n",f.m.roll,f.m.pitch,f.m.yaw);
    }else if(PLOT_MODE==2){
        n=snprintf(line,sizeof(line),"Madgwick_Roll:%.2f\tMadgwick_Pitch:%.2f\tMadgwick_Yaw:%.2f\n",f.d.roll,f.d.pitch,f.d.yaw);
    }else{
        n=snprintf(line,sizeof(line),"Mahony_Roll:%.2f\tMahony_Pitch:%.2f\tMahony_Yaw:%.2f\tMadgwick_Roll:%.2f\tMadgwick_Pitch:%.2f\tMadgwick_Yaw:%.2f\n",
          f.m.roll,f.m.pitch,f.m.yaw,f.d.roll,f.d.pitch,f.d.yaw);
    }
    // USB writes may wait, but only on core1. No unbounded telemetry queue.
    if(n>0&&n<(int)sizeof(line)&&Serial)Serial.write((const uint8_t*)line,n);
    delay(1);
}
