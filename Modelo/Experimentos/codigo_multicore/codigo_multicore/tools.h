#ifndef TOOLS
#define TOOLS

#include <Wire.h>
#include <MPU6050.h>

#define ENCODER_DA_R 14
#define ENCODER_DB_R 27 

#define ENCODER_DA_L 32
#define ENCODER_DB_L 33 

#define IN1 16
#define IN2 17
#define ENA 4

#define IN3 18
#define IN4 19
#define ENB 23

#define L_CHANNEL 0
#define R_CHANNEL 1

#define FORMAT_SPIFFS_IF_FAILED true

#define BUFFER_SIZE 2000
#define SIMULATIONS 4

#define FILE_COUNT_BKP "/file_count.bkp"

#define MAX_PWM 50

#define X_ACCEL_OFFSET 761.77
#define Y_ACCEL_OFFSET -1035.07
#define Z_ACCEL_OFFSET 570.21
#define X_GYRO_OFFSET -387.96
#define Y_GYRO_OFFSET 48.65
#define Z_GYRO_OFFSET 191.20

typedef struct {
  float t;
  int u;
  float yr;
  float yl;
  float t_trig;
  float freq;
  float theta;
} Data;

typedef enum { INIT, CONFIG, RUN, BUFFER_OVERFLOW } Modes;

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void motor(int PWM, int chanel);
int fileCountInit();
// float updatePitch(float currentAngle, float loop_period);
// void getAccelGyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
// float getAccelPitch();

#endif