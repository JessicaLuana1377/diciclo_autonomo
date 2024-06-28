#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <ESP32Encoder.h>
#include <math.h>
#include "FS.h"
#include "SPIFFS.h"
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <driver/pcnt.h>

#include "tools.h"

// CONST VARIABLES
const int ts = 5;
const float alpha_w = 0.5;
const float alpha_b = 0.95;
const float alpha = 0.98;
const float r_w = 0.03;
const float k_A_theta[3] = {1.0000, -1.8026, 0.8026};
const float k_B_theta[3] = {-67.1593, 124.8036, -57.9757};
const float k_A_x[3]     = {1.0000, -1.1597, 0.3247};
const float k_B_x[3]     = {0, 1.4763, -1.4029};
// const float k_A_x[3]     = {1.0000, 0, 0};
// const float k_B_x[3]     = {0, 1.4763, -1.4029};

// GLOBALS VARIABLES
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

Data *data_array;
Modes startMode = INIT; 

// MPU6050 mpu;
Adafruit_MPU6050 mpu;
float g_x_offset = -0.05 + 0.097;
sensors_event_t a, g, temp;
int16_t ax_b, ay_b, az_b, gx_b, gy_b, gz_b;
float ax, ay, az, gx, gy, gz;

char buffer[50];
int count = 0;
int flash_count = 0;
int file_count = 0;

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
	encoder_r.attachFullQuad(ENCODER_C1_R, ENCODER_C2_R);
	encoder_l.attachFullQuad(ENCODER_C1_L, ENCODER_C2_L);
  encoder_r.clearCount();
  encoder_l.clearCount();

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

void taskFlash(void* pvParameter) {
  Data data_read;
  while(1) {
    if (flash_count < count) {
      data_read = data_array[flash_count % BUFFER_SIZE];
      // printf("-7.5,7.5,%f,%f,%f,%f,%f,%f\n", data_read.theta, data_read.dtheta, data_read.wheel, data_read.dwheel, data_read.dwheel_f, data_read.u);
      // printf("-3.14,3.14,%f,%f,%f\n", error_theta[0], error_theta[1], error_theta[2]);

      if (count - flash_count > BUFFER_SIZE) startMode = BUFFER_OVERFLOW;

      flash_count++;
    } else if (startMode == CONFIG || (startMode == BUFFER_OVERFLOW && flash_count == count)) {
      printf("End Flash\n");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  vTaskDelete(NULL);
}

float position_r, position_l, x, theta;
float last_position_r, last_position_l, last_theta, ur, ul;
float g_x, g_y, g_z;
float a_pitch, a_yaw, a_roll;
float dwheel_f_r, dwheel_f_l, dtheta_f;
int pwm_value;

float u[3] = {0};
float error_theta[3] = {0};
float theta_ref[3] = {-0.062, -0.062, -0.062};
float error_x[3] = {0};

void taskControl(void* pvParameter) {
  pinMode(2, OUTPUT);

  xTaskCreate(&taskFlash, "task_flash", 4096, NULL, 2, NULL);

  Data data;
  TickType_t init_loop_time;
  unsigned long t0 = micros();
  unsigned long time = t0;
  unsigned long last_time = time;
  printf("primeiro\nx:%f,theta:%f,theta_ref:%f,u:%f\nfim\n", x, theta, theta_ref[0], 1*(k_B_x[1]*error_x[1] + k_B_x[2]*error_x[2]));
      
  while(1) {
    init_loop_time = xTaskGetTickCount();

    position_r = 2*PI*encoder_r.getCount()/(float)(4*7*30);
    position_l = 2*PI*encoder_l.getCount()/(float)(4*7*30);
    x = r_w * (position_l - position_r)/2;

    mpu.getEvent(&a, &g, &temp);
    g_x = -(g.gyro.x + g_x_offset);
    a_pitch = atan2(a.acceleration.z, a.acceleration.y);

    // direita
    data.dwheel = (position_r - last_position_r)/(ts/1e3);
    dwheel_f_r = alpha_w*dwheel_f_r + (1-alpha_w)*(position_r - last_position_r)/(ts/1e3);
    data.dwheel_f = dwheel_f_r;
    data.wheel = position_r;
    
    // esquerda
    dwheel_f_l = alpha_w*dwheel_f_l + (1-alpha_w)*(position_l - last_position_l)/(ts/1e3);

    if (count % 10) {
      error_x[0] = 0 - x;

      printf("x:%f,theta:%f,theta_ref:%f,u:%f\n", x, theta, theta_ref[0], 1*(k_B_x[1]*error_x[1] + k_B_x[2]*error_x[2]));
      theta_ref[0] = -1*(k_A_x[1]*theta_ref[1] + k_A_x[2]*theta_ref[2]) + 
        1*(k_B_x[1]*error_x[1] + k_B_x[2]*error_x[2]);
      theta_ref[0] = constrain(theta_ref[0], -PI/4, PI/4);

      error_x[2] = error_x[1];
      error_x[1] = error_x[0];

      theta_ref[2] = theta_ref[1];
      theta_ref[1] = theta_ref[0];

    }

    // corpo
    theta = alpha*(data.theta + g_x*(ts/1e3)) + (1-alpha)*a_pitch;
    dtheta_f = alpha_b*dtheta_f + (1-alpha_b)*(g_x);
    data.dtheta = dtheta_f;
    data.theta = theta;

    error_theta[0] = theta_ref[0] - theta;

    u[0] = -1*(k_A_theta[1]*u[1] + k_A_theta[2]*u[2]) + 
      1*(k_B_theta[0]*error_theta[0] + k_B_theta[1]*error_theta[1] + k_B_theta[2]*error_theta[2]);
    u[0] = constrain(u[0], -7.4, 7.4);

    ul = u[0];
    ur = -ul;

    if (abs(theta) >= PI/4) {
      ur = 0; ul = 0;
      theta_ref[0] = 0;
    }

    data.u = ur;

    pwm_value = map(ur, -7.4, 7.4, -100, 100);  
    motor(pwm_value, R_CHANNEL);

    pwm_value = map(ul, -7.4, 7.4, -100, 100);
    motor(pwm_value, L_CHANNEL);
    
    data_array[count % BUFFER_SIZE] = data;

    last_theta = theta;
    last_position_r = position_r;
    last_position_l = position_l;

    error_theta[2] = error_theta[1];
    error_theta[1] = error_theta[0];

    u[2] = u[1];
    u[1] = u[0];
    
    count++;

    // if (count >= 100) {
    //   motor(0, R_CHANNEL);
    //   motor(0, L_CHANNEL);
    //   while (true) {
    //     delay(10);
    //   }
    // }

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

void calibrate_MPU();

void setup() {
  Serial.begin(921600);
  delay(500);

  PinSetup();

  Wire.begin();

  delay(1000);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  // calibrate_MPU();
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

  // xTaskCreate(&taskBlinkLed, "blink_led", 2048, NULL, 3, NULL);
  // while (startMode == INIT) delay(100);
  delay(100);;
  startMode = RUN;
  if (startMode == RUN) {
    digitalWrite(2, LOW);
    delay(5000);
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
    
    // if (command == 'r' || command == 'R') {
    //   File root = SPIFFS.open("/");
    //   if(!root){
    //     Serial.println("- failed to open directory");
    //     return;
    //   }
    //   if(!root.isDirectory()){
    //     Serial.println(" - not a directory");
    //     return;
    //   }

    //   Data data_read;
    //   File file = root.openNextFile();
    //   while(file){
    //     if (!file.isDirectory()) {
    //       String filename = "/" + String(file.name());

    //       File file = SPIFFS.open(filename.c_str());
          
    //       if(!file || file.isDirectory()){
    //         Serial.println("- failed to open file for reading");
    //         break;
    //       }
    //       if (!filename.equals(String(FILE_COUNT_BKP))) {
    //         Serial.println("Reading: " + filename);

    //         Serial.println("- read from file:");
    //         while(file.available()){
    //           // Serial.write(file.read());
    //           if (file.read((uint8_t*)&data_read, sizeof(Data)) != sizeof(Data)) {
    //             break;
    //           }
    //           sprintf(buffer, "%ld;%d;%d;%d;%d\n", 
    //             data_read.t, data_read.u, data_read.yr, data_read.yl, data_read.dtheta);
    //           Serial.write(buffer);
    //         }
    //       }
    //       file.close();
    //     }
    //     file = root.openNextFile();
    //   }

    //   if (command == 'r') {
    //     SPIFFS.format();
    //     Serial.println("Memória formatada");
    //   }
    // }
  }
}

void calibrate_MPU() {
  float x, y, z;
  for (int i=0; i<3000; i++) {
    mpu.getEvent(&a, &g, &temp);
    x += g.gyro.x;
    y += g.gyro.y;
    z += g.gyro.z;
    delay(3);
  }
  x /= 3000;
  y /= 3000;
  z /= 3000;

  g_x_offset = x;
}