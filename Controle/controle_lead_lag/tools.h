#ifndef TOOLS
#define TOOLS

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <ESP32Encoder.h>
#include <math.h>
#include "FS.h"
#include "SPIFFS.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <driver/pcnt.h>

// Pinos dos encoderes
#define ENCODER_C1_R 32
#define ENCODER_C2_R 33 
#define ENCODER_C1_L 14 
#define ENCODER_C2_L 27

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

#define usTOs 1000000.0
#define BUTTON_BOOT 0
#define BLUE_LED 2
extern volatile bool BlueLED;
extern volatile bool BootPressed;

typedef struct {
  float t;
  float dwheel; //
  float dwheel_f; //
  float dtheta; //
  float theta; //
  float wheel; //
  int u; //
} Data;

// GLOBALS VARIABLES
extern ESP32Encoder encoder_r;
extern ESP32Encoder encoder_l;

extern Data data_array[BUFFER_SIZE];

extern Adafruit_MPU6050 mpu;
extern float g_x_offset;
extern sensors_event_t a, g, temp;
extern float dtheta, a_pitch, g_x;   // ângulo lido

// variaveis para arquivo
extern int count;
extern int count_save;
extern int flash_count;
extern int file_count;

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void motor(int PWM, int chanel);
int fileCountInit();  
void DataSave();

void InitSetup ();                                                                    // inicialização
void calibrate_MPU();
void IRAM_ATTR StopControl();

void taskFlash(void* pvParameter);    // thread que salva os valores na flash

#endif