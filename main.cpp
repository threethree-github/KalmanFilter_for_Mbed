#include "mbed.h"
#include "math.h"
#include "MPU6050.h"
#include "Kalman.h"

#define RadToDeg 57.295779513082320876798154814105
MPU6050 mpu(p9,p10);
Serial pc(USBTX, USBRX);
KalmanFilter gKfx, gKfy; 
Timer timer;

float gCalibrateX; // 初期化時の角度。（＝静止角とみなす）
float gCalibrateY;
float gPrevMicros; 

int main() {
    timer.start();
    float a[3];
    float g[3];
    
    pc.baud(115200);
    mpu.setAcceleroRange(2);
    mpu.setGyroRange(2);

    mpu.getAccelero(a);
    mpu.getGyro(g);
    float degRoll  = atan2(a[1], a[2]) * RadToDeg;
    float degPitch = atan(-a[0] / sqrt(a[1] * a[1] + a[2] * a[2])) * RadToDeg;

    gKfx.setAngle(degRoll);
    gKfy.setAngle(degPitch);
    gCalibrateY = degPitch;
    gCalibrateX = degRoll;
    gPrevMicros = timer.read();
    
    while(1) {
        mpu.getAccelero(a);
        mpu.getGyro(g);
        degRoll  = atan2(a[1], a[2]) * RadToDeg;
        degPitch = atan(-a[0] / sqrt(a[1] * a[1] + a[2] * a[2])) * RadToDeg;
        
        float dpsX = g[0] * RadToDeg;
        float dpsY = g[1] * RadToDeg;
        float dpsZ = g[2] * RadToDeg;
        
        float curMicros = timer.read();
        float dt = curMicros - gPrevMicros;
        gPrevMicros = curMicros;

        float degX = gKfx.calcAngle(degRoll, dpsX, dt);
        float degY = gKfy.calcAngle(degPitch, dpsY, dt);
        degY -= gCalibrateY;
        degX -= gCalibrateX;

        pc.printf("%f,%f,%f\r\n",dt,-degX,degY);
        wait(0.001);
    }
}