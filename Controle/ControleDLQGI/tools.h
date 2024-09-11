#ifndef TOOLS
#define TOOLS

#include <Wire.h>
#include <MPU6050.h>

#define ENCODER_C1_R 32
#define ENCODER_C2_R 33 

#define ENCODER_C1_L 14 
#define ENCODER_C2_L 27

#define IN1 16
#define IN2 17
#define ENA 4

#define IN3 18
#define IN4 19
#define ENB 23

#define L_CHANNEL 0
#define R_CHANNEL 1

#define MIN_PWM 20

// #define USE_WIFI

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
typedef enum { INIT, CONFIG, RUN, BUFFER_OVERFLOW } Modes;
void motor(int PWM, int chanel);

class Kalman {
  public:
    Kalman(float _dt) {
      angle = 0;
      bias = 0;
      dt = _dt;
      P[0][0] = 1;
      P[0][1] = 0;
      P[1][0] = 0;
      P[1][1] = 1;
    }

    float getAngle(float newAngle, float newRate) {
      float rate = newRate - bias;
      angle += dt * rate;

      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      float S = P[0][0] + R_measure;
      float K[2];
      K[0] = P[0][0] / S;
      K[1] = P[1][0] / S;

      float y = newAngle - angle;
      angle += K[0] * y;
      bias += K[1] * y;

      float P00_temp = P[0][0];
      float P01_temp = P[0][1];

      P[0][0] -= K[0] * P00_temp;
      P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp;
      P[1][1] -= K[1] * P01_temp;

      return angle;
    }

    float bias;
  private:
    float dt; 
    float angle;
    float P[2][2];
    float Q_angle = 0.001;    // angular data confidence
    float Q_bias = 0.005;     // angular velocity confidence
    float R_measure = 1.0;    // measure confidence
    // float Q_angle = 0.001;
    // float Q_bias = 0.003;
    // float R_measure = 0.03;
};

#endif