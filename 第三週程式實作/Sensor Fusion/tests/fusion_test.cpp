#include "../BMI088_Fusion/Fusion6D.h"
#include <cassert>
#include <cstdio>
using namespace Fusion6D;
template<class F> void run(){
 F f;f.reset(0,0,9.80665f);
 for(int i=0;i<10000;++i)assert(f.update(0,0,0,0,0,9.80665f,.001f));
 assert(fabsf(f.q.euler().roll)<.001f);
 for(int i=0;i<1000;++i)f.update(0,0,PI_F/2,0,0,9.80665f,.001f);
 assert(fabsf(f.q.euler().yaw-90)<.02f);
 f.reset(0,0,1);
 for(int i=0;i<30000;++i)f.update(0,0,0,0,.5f,.8660254f,.001f);
 assert(fabsf(f.q.euler().roll-30)<.1f);
 f.reset(-.5f,0,.8660254f);assert(fabsf(f.q.euler().pitch-30)<.001f);
 f.reset(0,0,1);
 for(int i=0;i<1000;++i)f.update(0,PI_F/6,0,0,0,0,.001f,false);
 assert(fabsf(f.q.euler().pitch-30)<.02f);
 Quaternion old=f.q;assert(!f.update(NAN,0,0,0,0,1,.001f));assert(f.q.w==old.w);
 assert(!f.update(0,0,0,0,0,1,.1f));
 f.reset(0,0,1);
 for(int i=0;i<1000;++i)f.update(PI_F/6,0,0,0,0,0,(i%2)?.0015f:.0005f,false);
 assert(fabsf(f.q.euler().roll-30)<.02f);
 float norm=f.q.w*f.q.w+f.q.x*f.q.x+f.q.y*f.q.y+f.q.z*f.q.z;assert(fabsf(norm-1)<1e-6f);
}
int main(){run<Mahony>();run<Madgwick>();puts("PASS: both filters stationary, yaw integration, tilt convergence, tilt initialization, gyro-only, invalid input, variable dt, normalization");}
