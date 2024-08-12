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

// Microssegundos para segundos
#define usTOs 1000000.0

typedef struct {
  float t;
  int u;
  long int yr;
  long int yl;
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

void InitSetup ();                                                                    // inicialização

void taskFlash(void* pvParameter);    // thread que salva os valores na flash
void taskBlinkLed(void* pvParameter); // thread que pisca o led e espera o acionamento do botão

#endif // TOOLS