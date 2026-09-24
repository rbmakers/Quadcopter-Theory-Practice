#pragma once
#include <math.h>
// Independent equation-based implementations. Units: rad/s, arbitrary accel
// units, seconds. Quaternion rotates sensor vectors into the reference frame.
namespace Fusion6D {
constexpr float PI_F=3.14159265358979323846f;
constexpr float RAD=PI_F/180.0f, DEG=180.0f/PI_F;
struct Euler { float roll, pitch, yaw; };
struct Quaternion {
    float w=1, x=0, y=0, z=0;
    void normalize() {
        float n=sqrtf(w*w+x*x+y*y+z*z);
        if (isfinite(n) && n>1e-12f) { w/=n; x/=n; y/=n; z/=n; }
        else { w=1; x=y=z=0; }
    }
    void fromAccel(float ax,float ay,float az) {
        float r=atan2f(ay,az), p=atan2f(-ax,sqrtf(ay*ay+az*az));
        float cr=cosf(r/2),sr=sinf(r/2),cp=cosf(p/2),sp=sinf(p/2);
        w=cr*cp; x=sr*cp; y=cr*sp; z=-sr*sp; normalize();
    }
    Euler euler() const {
        float s=2*(w*y-z*x); s=fmaxf(-1,fminf(1,s));
        return {DEG*atan2f(2*(w*x+y*z),1-2*(x*x+y*y)),
                DEG*asinf(s),DEG*atan2f(2*(w*z+x*y),1-2*(y*y+z*z))};
    }
    void integrate(float gx,float gy,float gz,float dt,
                   float sw=0,float sx=0,float sy=0,float sz=0) {
        float dw=0.5f*(-x*gx-y*gy-z*gz)-sw;
        float dx=0.5f*(w*gx+y*gz-z*gy)-sx;
        float dy=0.5f*(w*gy-x*gz+z*gx)-sy;
        float dz=0.5f*(w*gz+x*gy-y*gx)-sz;
        w+=dt*dw; x+=dt*dx; y+=dt*dy; z+=dt*dz; normalize();
    }
};
inline bool validStep(float gx,float gy,float gz,float dt) {
    return isfinite(gx)&&isfinite(gy)&&isfinite(gz)&&isfinite(dt)&&dt>0&&dt<=0.02f;
}
inline bool unitAccel(float &ax,float &ay,float &az) {
    float n=sqrtf(ax*ax+ay*ay+az*az);
    if (!isfinite(n)||n<1e-6f) return false;
    ax/=n; ay/=n; az/=n; return true;
}
class Mahony {
public:
    Quaternion q;
    float kp=2.0f,ki=0.0f;
    void reset(float ax,float ay,float az) {q.fromAccel(ax,ay,az); ix=iy=iz=0;}
    bool update(float gx,float gy,float gz,float ax,float ay,float az,float dt,bool useAccel=true) {
        if (!validStep(gx,gy,gz,dt)) return false;
        if (ki<=0) ix=iy=iz=0;
        if (useAccel && unitAccel(ax,ay,az)) {
            float vx=2*(q.x*q.z-q.w*q.y),vy=2*(q.w*q.x+q.y*q.z);
            float vz=1-2*(q.x*q.x+q.y*q.y);
            float ex=ay*vz-az*vy,ey=az*vx-ax*vz,ez=ax*vy-ay*vx;
            if(ki>0) {ix=limit(ix+ki*ex*dt);iy=limit(iy+ki*ey*dt);iz=limit(iz+ki*ez*dt);}
            gx+=kp*ex; gy+=kp*ey; gz+=kp*ez;
        }
        q.integrate(gx+ix,gy+iy,gz+iz,dt); return true;
    }
private:
    float ix=0,iy=0,iz=0;
    static float limit(float x) {return fmaxf(-0.2f,fminf(0.2f,x));}
};
class Madgwick {
public:
    Quaternion q;
    float beta=0.05f;
    void reset(float ax,float ay,float az) {q.fromAccel(ax,ay,az);}
    bool update(float gx,float gy,float gz,float ax,float ay,float az,float dt,bool useAccel=true) {
        if (!validStep(gx,gy,gz,dt)) return false;
        float sw=0,sx=0,sy=0,sz=0;
        if(useAccel && unitAccel(ax,ay,az)) {
            // f(q) = predicted gravity - normalized measured acceleration.
            // Gradient J^T f, original IMU-only Madgwick objective.
            float f1=2*(q.x*q.z-q.w*q.y)-ax;
            float f2=2*(q.w*q.x+q.y*q.z)-ay;
            float f3=1-2*(q.x*q.x+q.y*q.y)-az;
            sw=-2*q.y*f1+2*q.x*f2;
            sx=2*q.z*f1+2*q.w*f2-4*q.x*f3;
            sy=-2*q.w*f1+2*q.z*f2-4*q.y*f3;
            sz=2*q.x*f1+2*q.y*f2;
            float n=sqrtf(sw*sw+sx*sx+sy*sy+sz*sz);
            if(n>1e-9f) {float k=beta/n;sw*=k;sx*=k;sy*=k;sz*=k;}
            else sw=sx=sy=sz=0;
        }
        q.integrate(gx,gy,gz,dt,sw,sx,sy,sz); return true;
    }
};
}
