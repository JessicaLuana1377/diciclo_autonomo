#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <ESP32Encoder.h>
#include <math.h>
#include "FS.h"
#include "SPIFFS.h"
#include <Wire.h>
#include <MPU6050.h>

#include "tools.h"

// CONST VARIABLES
const int ts = 10;
const float f0 = 1;
const float f1 = 20;
const int T = 30;
const int uT = T*1000000;

// GLOBALS VARIABLES
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

Data *data_array;
Modes startMode = INIT; 

MPU6050 mpu;
int16_t ax_b, ay_b, az_b, gx_b, gy_b, gz_b;
float ax, ay, az, gx, gy, gz;
float tau = 0.98;
float theta;

char buffer[50];
int count = 0;
int flash_count = 0;
int file_count = 0;

float getAccelPitch(){
  getAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
  float accel_pitch = atan2(az, ay);
  return accel_pitch;
}

void getAccelGyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
  mpu.getMotion6(&ax_b, &ay_b, &az_b, &gx_b, &gy_b, &gz_b); // gyro (+/- 250 deg/s) accel (+/- 2g)
  // as variaveis ##_b são inteiros de 16 bits, 2^16 - 1 = 65535, a divisão por esse numero é para normalizar de 0 a 1
  // o primeiro 2 é para pegar todo o intervalo, -x a x resulta em um intervalo de x - (-x) = 2x
  ax[0] = 2 * 2   * ((ax_b - X_ACCEL_OFFSET) / 65535.0);                       // g
  ay[0] = 2 * 2   * ((ay_b - Y_ACCEL_OFFSET) / 65535.0);                       // g 
  az[0] = 2 * 2   * ((az_b - Z_ACCEL_OFFSET) / 65535.0);                       // g
  gx[0] = 2 * 250 * ((gx_b - X_GYRO_OFFSET) / (65535.0)) * PI / 180;          // rad/s
  gx[0] = 2 * 250 * ((gy_b - Y_GYRO_OFFSET) / (65535.0)) * PI / 180;          // rad/s
  gz[0] = 2 * 250 * ((gz_b - Z_GYRO_OFFSET) / (65535.0)) * PI / 180;          // rad/s
}

float updatePitch(float currentAngle, float loop_period)
{
  // Get imu data.
  getAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
  // Pitch estimation from accelerometer.
  float accelPitch = getAccelPitch();
  // Complementary filter between acceleration and gyroscopic pitch estimation.
  float pitch_f = (tau) * (currentAngle + (gx)*loop_period) + (1 - tau) * (accelPitch);
  pitch_f = -pitch_f;

  return pitch_f;
}

void PinSetup() {
  // Configuração dos pinos
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuração dos motores
  motor(0, L_CHANNEL);
  motor(0, R_CHANNEL);

  // Configuração do PWM
  ledcSetup(L_CHANNEL, 200, 8);
  ledcSetup(R_CHANNEL, 200, 8);
  ledcAttachPin(ENA, L_CHANNEL);
  ledcAttachPin(ENB, R_CHANNEL);
  ledcWrite(L_CHANNEL, 0);
  ledcWrite(R_CHANNEL, 0);

  // Configuração dos encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
	encoder_r.attachFullQuad(ENCODER_DA_R, ENCODER_DB_R);
	encoder_l.attachFullQuad(ENCODER_DA_L, ENCODER_DB_L);
  encoder_r.clearCount();
  encoder_l.clearCount();

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

void taskFlash(void* pvParameter) {
  String filename = "/arquivo" + String(file_count) + ".csv";
  Serial.println(filename);
  File file = SPIFFS.open(filename, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  String text = "t;u;yr;yl;t_trig;freq;theta\n";
  file.print(text.c_str());

  Data data_read;
  while(1) {
    if (flash_count < count) {
      data_read = data_array[flash_count % BUFFER_SIZE];
      // printf("%f %d\n", data_read.t, count - flash_count);

      if (count - flash_count > BUFFER_SIZE) startMode = BUFFER_OVERFLOW;
      // else startMode = RUN;

      sprintf(buffer, "%.4f;%d;%.4f;%.4f;%.4f;%.4f;%.4f\n", 
        data_read.t, data_read.u, data_read.yr, data_read.yl, data_read.t_trig, data_read.freq, data_read.theta);
      file.print(buffer);

      flash_count++;
    } else if (startMode == CONFIG || (startMode == BUFFER_OVERFLOW && flash_count == count)) {
      file.close();

      file_count++;
      file = SPIFFS.open(FILE_COUNT_BKP, FILE_WRITE);
      if(!file || file.isDirectory()) {
        Serial.println("- failed to open file bkp for something");
      }
      file.write((uint8_t*)&file_count, sizeof(file_count));
      file.close();

      printf("End Flash\n");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  vTaskDelete(NULL);
}

void taskControl(void* pvParameter) {
  pinMode(2, OUTPUT);

  xTaskCreate(&taskFlash, "task_flash", 4096, NULL, 2, NULL);

  Data data;
  TickType_t init_loop_time;
  unsigned long t0 = micros();
  unsigned long time = t0;
  unsigned long last_time = time;
  float theta = getAccelPitch();
  while(1) {
    init_loop_time = xTaskGetTickCount();

    float position_r = encoder_r.getCount();
    float position_l = encoder_l.getCount();
    theta = updatePitch(theta, (micros() - time)/(float)1e6);

    time = micros();
    int temp = (time - t0 + uT) % (2 * uT) - uT;
    float t = abs(temp) / 1000000.0;

    int pwm_value = (int)(MAX_PWM * sin(-2 * PI * f0 * f1 * T / (f1 - f0) * log(1 - (f1 - f0) / (f1 * T) * t)));
    motor(pwm_value, L_CHANNEL);
    motor(pwm_value, R_CHANNEL);
    
    data.t = (time - t0)/1000000.0;
    data.u = pwm_value;
    data.yr = position_r;
    data.yl = position_l;
    data.t_trig = t;
    data.freq = f0 * f1 * T / ((f0 - f1) * t + f1 * T);
    data.theta = theta;
    data_array[count % BUFFER_SIZE] = data;
    count++;

    if (startMode == BUFFER_OVERFLOW) {
      // digitalWrite(2, HIGH);
      // while (startMode == BUFFER_OVERFLOW) {
      //   vTaskDelay(pdMS_TO_TICKS(1));
      // }
      // digitalWrite(2, LOW);

      motor(0, L_CHANNEL);
      motor(0, R_CHANNEL);
      printf("Buffer Overflow! Stop Simulation\n");
      while (1) {
        digitalWrite(2, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(2, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
    }

    if (time - t0 >= uT*SIMULATIONS) {
      motor(0, L_CHANNEL);
      motor(0, R_CHANNEL);

      startMode = CONFIG;
      digitalWrite(2, HIGH);

      printf("End Control\n");
      printf("Configuration Mode (l:list, f:format, r:read)\n");
      break;
    }

    vTaskDelayUntil(&init_loop_time, pdMS_TO_TICKS(ts));
  }

  vTaskDelete(NULL);
}

void taskBlinkLed(void* pvParameter) {
  pinMode(2, OUTPUT);
  pinMode(0, INPUT);

  unsigned long init = millis();
  bool state = true;
  startMode = INIT;

  while (millis() - init <= 5000) {
    digitalWrite(2, state);
    state = !state;

    if (digitalRead(0) == LOW) {
      startMode = RUN;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (startMode == INIT) startMode = CONFIG;

  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  PinSetup();

  Wire.begin();

  delay(1000);
  
  // Inicialização do MPU6050
  Serial.println("Inicializando MPU6050...");
  mpu.initialize();
  delay(1000);

  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("Falha ao montar SPIFFS");
    return;
  }
  file_count = fileCountInit();

  data_array = (Data*)heap_caps_calloc(BUFFER_SIZE, sizeof(Data), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (data_array == NULL) {
    Serial.println("Malloc for 'data_array' has failed!");
    return;
  }

  xTaskCreate(&taskBlinkLed, "blink_led", 2048, NULL, 3, NULL);
  while (startMode == INIT) delay(100);

  if (startMode == RUN) {
    digitalWrite(2, LOW);
    printf("Running Mode...\n");
    xTaskCreate(&taskControl, "task_control", 4096, NULL, 1, NULL);
  }
  else if (startMode == CONFIG) {
    digitalWrite(2, HIGH);
    printf("Configuration Mode (l:list, f:format, r:read)\n");
  }
}

void loop() {
  if (Serial.available() >= 1) {
    char command = Serial.read();

    if (command == 'l') {
      listDir(SPIFFS, "/", 0);
    }

    if (command == 'f') {
      SPIFFS.format();
      Serial.println("Memória formatada");
    }
    
    if (command == 'r' || command == 'R') {
      File root = SPIFFS.open("/");
      if(!root){
        Serial.println("- failed to open directory");
        return;
      }
      if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
      }

      File file = root.openNextFile();
      while(file){
        if (!file.isDirectory()) {
          String filename = "/" + String(file.name());
          File file = SPIFFS.open(filename.c_str());

          Serial.println("Reading: " + filename);

          if(!file || file.isDirectory()){
            Serial.println("- failed to open file for reading");
            return;
          }

          Serial.println("- read from file:");
          while(file.available()){
            Serial.write(file.read());
          }
          file.close();
        }
        file = root.openNextFile();
      }

      if (command == 'r') {
        SPIFFS.format();
        Serial.println("Memória formatada");
      }
    }
  }
}
