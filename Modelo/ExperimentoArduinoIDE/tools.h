#ifndef TOOLS
#define TOOLS

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <ESP32Encoder.h>
#include <math.h>
#include "FS.h"
#include "SPIFFS.h"

// Pinos do encoderes
#define ENCODER_DA_R 14
#define ENCODER_DB_R 27 

#define ENCODER_DA_L 32
#define ENCODER_DB_L 33 


// Pinos dos motores
#define IN1 16
#define IN2 17
#define ENA 4

#define IN3 18
#define IN4 19
#define ENB 23

// Canais de PWM
#define L_CHANNEL 0
#define R_CHANNEL 1

// Definições para o arquivo na flash
#define FORMAT_SPIFFS_IF_FAILED true
#define BUFFER_SIZE 2000
#define FILE_COUNT_BKP "/file_count.bkp"

// Offset para calibração da IMU
#define X_ACCEL_OFFSET 761.77
#define Y_ACCEL_OFFSET -1035.07
#define Z_ACCEL_OFFSET 570.21
#define X_GYRO_OFFSET -387.96
#define Y_GYRO_OFFSET 48.65
#define Z_GYRO_OFFSET 191.20

// Microssegundos para segundos
#define usTOs 1000000.0

typedef struct {
  float t;
  int u;
  float yr;
  float yl;
  float t_trig;
  float freq;
  float dtheta;
} Data;

// Modo de operação do ESP32
typedef enum { INIT, CONFIG, RUN, BUFFER_OVERFLOW } Modes;

// GLOBALS VARIABLES
extern ESP32Encoder encoder_r;
extern ESP32Encoder encoder_l;

extern Data *data_array;
extern Modes startMode; 

extern Adafruit_MPU6050 mpu;
extern float g_x_offset;
extern sensors_event_t a, g, temp;
extern int16_t ax_b, ay_b, az_b, gx_b, gy_b, gz_b;
extern float ax, ay, az, gx, gy, gz;
extern float alpha;     // filtro para leitura
extern float dtheta, a_pitch, g_x;   // ângulo lido


// variaveis para arquivo
extern char buffer[50];
extern int count;
extern int flash_count;
extern int file_count;

void listDir(fs::FS &fs, const char * dirname, uint8_t levels); // lista os arquivos presentes na flash
void motor(int PWM, int chanel);                                // aplica o PWM em um motor 
int fileCountInit();                                            // inicializa o arquivo fileCount

float getAccelPitch();                                                                // obtem o ângulo pelo acelerômetro
void getAccelGyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);  // obtem os valores dos eixos do acelerômetro e do giroscópio
float updatePitch(float currentAngle, float loop_period);                             // obtem o ângulo usando o acelerômetro e o giroscópio
void InitSetup ();                                                                    // inicialização

void taskFlash(void* pvParameter);    // thread que salva os valores na flash
void taskBlinkLed(void* pvParameter); // thread que pisca o led e espera o acionamento do botão

#endif // TOOLS