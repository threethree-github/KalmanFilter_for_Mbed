#ifndef _KALMAN_H
#define _KALMAN_H

class KalmanFilter {
private:
    float angle;
    float bias;
    float P[2][2];
public:
    KalmanFilter();
    void setAngle(float newAngle);
    float calcAngle(float newAngle, float newRate, float dt);
};

#endif