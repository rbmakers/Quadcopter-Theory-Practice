#pragma once
#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include "TimingAnalysis.h"
#if USE_BMI088
#include "BMI088.h"
BMI088 imu(Wire,0x18,0x69);
#endif
// Standalone lab owns one SM on PIO0 and PIO0 IRQ flag 0. No PIO CPU IRQ used.
constexpr uint32_t SAMPLE_HZ=1000000,WORDS=4096,SAMPLES=WORDS*16;
constexpr uint32_t EDGE_CAP=512,GUARD_US=2000;
constexpr uint32_t BASE_PIN=USE_BMI088?22:6;
constexpr uint32_t TEST_OUT=2; // synthetic labs only: jumper GPIO2 -> GPIO6 AND GPIO7
constexpr uint32_t MASK_US=1200,MASK_EVERY_US=7000;
static_assert(SAMPLES%16==0,"Full words required");
static_assert(BASE_PIN+1<30,"Two consecutive input pins required");
PIO capturePio=pio0;
int captureSm=-1,dmaCh=-1;
uint programOffset=0;
float actualSampleHz=0;
uint32_t windowUs=65536;
#if ENABLE_PIO
alignas(4) uint32_t packed[WORDS];
uint32_t pioEdges[2][EDGE_CAP];
uint32_t pioN[2]={};bool pioEdgeOverflow[2]={};
#endif
volatile uint32_t irqEdges[2][EDGE_CAP];
volatile uint32_t irqN[2]={},irqOverflow[2]={};
volatile bool recording=false;
volatile uint32_t epochUs=0;
uint32_t storedIrq[2][EDGE_CAP],storedN[2]={};
uint32_t lastIrqOverflow[2]={},maskCount=0;
bool captureValid=false,haveCapture=false;
char mode='n';
const uint16_t samplerInstructions[]={
    (uint16_t)pio_encode_pull(false,true),
    (uint16_t)pio_encode_mov(pio_x,pio_osr),
    (uint16_t)pio_encode_in(pio_pins,2),
    (uint16_t)pio_encode_jmp_x_dec(2),
    (uint16_t)pio_encode_irq_wait(false,0)
};
const struct pio_program samplerProgram={samplerInstructions,5,-1};
void __not_in_flash_func(recordEdge)(uint32_t ch){
    if(!recording)return;
    uint32_t t=time_us_32()-epochUs;
    if(t>=windowUs)return;
    uint32_t n=irqN[ch];
    if(n<EDGE_CAP){irqEdges[ch][n]=t;irqN[ch]=n+1;}else ++irqOverflow[ch];
}
void __not_in_flash_func(onA)(){recordEdge(0);}
void __not_in_flash_func(onG)(){recordEdge(1);}
void fatal(const char *msg){Serial.println(msg);while(true)delay(1000);}
void initSampler(){
#if ENABLE_PIO
    captureSm=pio_claim_unused_sm(capturePio,false);
    if(captureSm<0||!pio_can_add_program(capturePio,&samplerProgram))fatal("PIO resource unavailable");
    programOffset=pio_add_program(capturePio,&samplerProgram);
    dmaCh=dma_claim_unused_channel(false);if(dmaCh<0)fatal("DMA resource unavailable");
    // Integer divider: IN + JMP = two state-machine cycles per sample.
    uint32_t sys=clock_get_hz(clk_sys);
    uint32_t div=sys/(2*SAMPLE_HZ);
    if(div<1||div>65535||sys%(2*SAMPLE_HZ))fatal("Choose CPU clock divisible by 2 MHz (150 MHz works)");
    actualSampleHz=float(sys)/(2*div);
    windowUs=uint32_t(double(SAMPLES)*1e6/actualSampleHz);
#else
    actualSampleHz=SAMPLE_HZ;
#endif
}
void prepareSampler(){
#if ENABLE_PIO
    pio_sm_set_enabled(capturePio,captureSm,false);
    pio_interrupt_clear(capturePio,0);
    pio_sm_config c=pio_get_default_sm_config();
    sm_config_set_wrap(&c,programOffset,programOffset+4);
    sm_config_set_in_pins(&c,BASE_PIN);
    sm_config_set_in_shift(&c,true,true,32); // right shift; autopush 32
    sm_config_set_clkdiv(&c,float(clock_get_hz(clk_sys))/(2*SAMPLE_HZ));
    // Keep pins as SIO INPUT: PIO inputs can observe GPIO independently.
    // Do not call pio_gpio_init or configure any PIO output on BMI088 IRQ lines.
    pio_sm_init(capturePio,captureSm,programOffset,&c);
    pio_sm_clear_fifos(capturePio,captureSm);
    pio_sm_put_blocking(capturePio,captureSm,SAMPLES-1);
    capturePio->fdebug=1u<<(PIO_FDEBUG_RXSTALL_LSB+captureSm); // W1C
    dma_channel_config d=dma_channel_get_default_config(dmaCh);
    channel_config_set_transfer_data_size(&d,DMA_SIZE_32);
    channel_config_set_read_increment(&d,false);
    channel_config_set_write_increment(&d,true);
    channel_config_set_dreq(&d,pio_get_dreq(capturePio,captureSm,false));
    dma_channel_configure(dmaCh,&d,packed,&capturePio->rxf[captureSm],WORDS,true);
#endif
}
void exerciseCPU(uint32_t elapsed,uint32_t &nextMask){
    if(mode=='b')busy_wait_us_32(40); // busy, but IRQs remain enabled
    else if(mode=='m'&&elapsed>=nextMask&&elapsed+MASK_US+GUARD_US<windowUs){
        uint32_t s=save_and_disable_interrupts();
        busy_wait_us_32(MASK_US);
        restore_interrupts(s);++maskCount;nextMask=elapsed+MASK_EVERY_US;
    }
}
void captureWindow(){
    recording=false;haveCapture=false;captureValid=false;maskCount=0;
    irqN[0]=irqN[1]=irqOverflow[0]=irqOverflow[1]=0;
    prepareSampler();
    uint32_t saved=save_and_disable_interrupts();
    gpio_acknowledge_irq(BASE_PIN,GPIO_IRQ_EDGE_RISE);
    gpio_acknowledge_irq(BASE_PIN+1,GPIO_IRQ_EDGE_RISE);
    epochUs=time_us_32();recording=true;
#if ENABLE_PIO
    pio_sm_clkdiv_restart(capturePio,captureSm);
    pio_sm_set_enabled(capturePio,captureSm,true);
#endif
    restore_interrupts(saved);
    bool timeout=false;uint32_t nextMask=5000;
    while(true){
        uint32_t elapsed=time_us_32()-epochUs;
#if ENABLE_PIO
        if(pio_interrupt_get(capturePio,0)&&!dma_channel_is_busy(dmaCh))break;
        if(elapsed>windowUs+20000){timeout=true;break;}
#else
        if(elapsed>=windowUs)break;
#endif
        exerciseCPU(elapsed,nextMask);
    }
    saved=save_and_disable_interrupts();recording=false;restore_interrupts(saved);
#if ENABLE_PIO
    pio_sm_set_enabled(capturePio,captureSm,false);
    if(dma_channel_is_busy(dmaCh))dma_channel_abort(dmaCh);
    bool stalled=(capturePio->fdebug&(1u<<(PIO_FDEBUG_RXSTALL_LSB+captureSm)))!=0;
    bool dmaError=(dma_channel_hw_addr(dmaCh)->ctrl_trig&DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS)!=0;
    bool complete=dma_channel_hw_addr(dmaCh)->transfer_count==0;
    captureValid=!timeout&&!stalled&&!dmaError&&complete;
    Serial.printf("CAPTURE,valid=%u,timeout=%u,rxstall=%u,dma_error=%u,remaining=%lu\n",
        captureValid,timeout,stalled,dmaError,(unsigned long)dma_channel_hw_addr(dmaCh)->transfer_count);
    if(captureValid)for(uint32_t c=0;c<2;++c)pioN[c]=extractEdges(packed,SAMPLES,c,pioEdges[c],EDGE_CAP,pioEdgeOverflow[c]);
    else for(uint32_t c=0;c<2;++c){pioN[c]=0;pioEdgeOverflow[c]=false;}
#else
    captureValid=!timeout;
#endif
    for(uint32_t c=0;c<2;++c){storedN[c]=irqN[c];lastIrqOverflow[c]=irqOverflow[c];
        for(uint32_t i=0;i<storedN[c];++i)storedIrq[c][i]=irqEdges[c][i];}
    haveCapture=true;
}
void printStats(const char *kind,uint32_t channel,const uint32_t *edges,uint32_t n,double scale){
    TimingStats s;uint32_t accepted=0;double prev=0;
    for(uint32_t i=0;i<n;++i){double t=edges[i]*scale;
        if(t<GUARD_US||t>=windowUs-GUARD_US)continue;
        if(accepted)s.add(t-prev);prev=t;++accepted;
    }
    if(!s.count)Serial.printf("STAT,%s,%lu,%lu,0,NA,NA,NA,NA\n",kind,(unsigned long)channel,(unsigned long)accepted);
    else Serial.printf("STAT,%s,%lu,%lu,%lu,%.3f,%.3f,%.3f,%.3f\n",kind,(unsigned long)channel,
       (unsigned long)accepted,(unsigned long)s.count,s.mean,s.min,s.max,s.sd());
}
void report(){
    Serial.printf("META,mode=%c,source=%s,fs=%.1f,window_us=%lu,mask_count=%lu\n",mode,
        USE_BMI088?"BMI088":"PWM",actualSampleHz,(unsigned long)windowUs,(unsigned long)maskCount);
    Serial.println("HEADER,source,channel,edges_in_guard,intervals,mean_us,min_us,max_us,sd_us");
    for(uint32_t c=0;c<2;++c){
        printStats("INT",c,storedIrq[c],storedN[c],1.0);
        Serial.printf("INT_OVERFLOW,%lu,%lu\n",(unsigned long)c,(unsigned long)lastIrqOverflow[c]);
#if ENABLE_PIO
        if(captureValid){printStats("PIO",c,pioEdges[c],pioN[c],1e6/actualSampleHz);
            Serial.printf("PIO_EDGE_OVERFLOW,%lu,%u\n",(unsigned long)c,pioEdgeOverflow[c]);}
#endif
    }
#if ENABLE_PIO
    if(captureValid&&!pioEdgeOverflow[0]&&!pioEdgeOverflow[1]){
        TimingStats phase;uint32_t j=0;bool havePrev=false;uint32_t prev=0;
        for(uint32_t i=0;i<pioN[0];++i){uint32_t a=pioEdges[0][i];
            while(j<pioN[1]&&pioEdges[1][j]<=a){prev=pioEdges[1][j++];havePrev=true;}
            double t=a*1e6/actualSampleHz;
            if(havePrev&&t>=GUARD_US&&t<windowUs-GUARD_US)phase.add((a-prev)*1e6/actualSampleHz);
        }
        if(phase.count)Serial.printf("PHASE_A_AFTER_PREV_G,n=%lu,mean_us=%.3f,min_us=%.3f,max_us=%.3f\n",
            (unsigned long)phase.count,phase.mean,phase.min,phase.max);
    }
#endif
    Serial.println("READY: n=normal, b=CPU busy IRQ enabled, m=mask IRQ, r=repeat, d=dump last window");
}
void dump(){
    if(!haveCapture){Serial.println("No capture; send n first");return;}
    Serial.println("TRACE,source,channel,index,t_us");
    for(uint32_t c=0;c<2;++c){
        for(uint32_t i=0;i<storedN[c];++i)Serial.printf("TRACE,INT,%lu,%lu,%lu\n",(unsigned long)c,(unsigned long)i,(unsigned long)storedIrq[c][i]);
#if ENABLE_PIO
        if(captureValid)for(uint32_t i=0;i<pioN[c];++i)Serial.printf("TRACE,PIO,%lu,%lu,%.3f\n",(unsigned long)c,(unsigned long)i,pioEdges[c][i]*1e6/actualSampleHz);
#endif
    }
    Serial.println("END_TRACE");
}
void setup(){
    Serial.begin(115200);uint32_t t=millis();while(!Serial&&millis()-t<4000)delay(10);
    Serial.println("CURIO INT vs PIO lab v1.0 - timing only, no fusion or motor control");
    pinMode(BASE_PIN,INPUT);pinMode(BASE_PIN+1,INPUT);
#if USE_BMI088
    if(!Wire.setSDA(20)||!Wire.setSCL(25))fatal("I2C pin mapping error");
    Wire.begin();Wire.setClock(400000);Wire.setTimeout(3);
    if(!imu.beginSync(BMI088::SyncRate::Hz1000,&Serial,false)){
        imu.printLastBusError(Serial);fatal("BMI088 init failed");}
    delay(100); // initialization only, before capture
#else
    gpio_pull_down(BASE_PIN);gpio_pull_down(BASE_PIN+1);
    gpio_set_function(TEST_OUT,GPIO_FUNC_PWM);
    uint slice=pwm_gpio_to_slice_num(TEST_OUT);pwm_config cfg=pwm_get_default_config();
    uint32_t clock=clock_get_hz(clk_sys);uint32_t divider=clock/1000000;
    if(clock%1000000||divider<1||divider>255)fatal("Unsupported PWM divider; use 150 MHz");
    pwm_config_set_clkdiv(&cfg,float(divider));pwm_config_set_wrap(&cfg,999);
    pwm_init(slice,&cfg,false);pwm_set_gpio_level(TEST_OUT,500);pwm_set_enabled(slice,true);
    Serial.println("Synthetic: GPIO2 -> GPIO6 AND GPIO7; no connection to BMI088 outputs");
#endif
    initSampler();
    attachInterrupt(digitalPinToInterrupt(BASE_PIN),onA,RISING);
    attachInterrupt(digitalPinToInterrupt(BASE_PIN+1),onG,RISING);
    Serial.printf("Inputs=%lu,%lu PIO=%u capture=%lu us. Send n/b/m/r/d.\n",(unsigned long)BASE_PIN,
        (unsigned long)(BASE_PIN+1),ENABLE_PIO,(unsigned long)windowUs);
}
void loop(){
    if(!Serial.available()){delay(1);return;}
    char c=Serial.read();
    if(c=='d'){dump();return;}
    if(c=='n'||c=='b'||c=='m')mode=c;
    else if(c!='r')return;
    Serial.flush();captureWindow();report();
}
