#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <ESP32Encoder.h>
#include <math.h>
#include "FS.h"
#include "SPIFFS.h"
#include <Wire.h>
// #include <MPU6050.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <driver/pcnt.h>

#include "tools.h"

// CONST VARIABLES
const int ts = 10;
const float f0 = 1;
const float f1 = 30;
const int T = 120;
const int uT = T*1000000;
// const float k[4] = {-8.7779, -1.5965, -77.8062, -1};
// const float k[4] = {-7.8196, -1.3265, -70.0481,  0};
// const float k[4] = {-2.8717, -0.5726, -31.1010, 0};
// const float k[4] = {-11.5516, -1.8931, -100.2741, -1};
// const float k[4] = {0, -1.8931, 0, -1};
// const float k[4] = {-7.1133, -1.2254, -64.5476, -1};
// const float k[4] = {-7, -1, -140,  0};


#define n1 1 // 1.1
#define n2 -1 // 0
#define n3 1 // 1.5
//                  dtheta        dwheel        theta         wheel
const float k[4] = {-5.9554*n1,  -1.0491*n2,            -55.2452*n3, 0};

// GLOBALS VARIABLES
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

Data *data_array;
Modes startMode = INIT; 

float ref[4] = {0, 0, 0, 0};
float error[4];

// MPU6050 mpu;
Adafruit_MPU6050 mpu;
float g_x_offset = -0.05 + 0.097;
sensors_event_t a, g, temp;
const float alpha = 0.98;
int16_t ax_b, ay_b, az_b, gx_b, gy_b, gz_b;
float ax, ay, az, gx, gy, gz;
float tau = 0.98;
float theta;

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
	encoder_l.attachFullQuad(ENCODER_C2_L, ENCODER_C1_L);
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
      printf("-3.14,3.14,%f,%f,%f,%f,%f,%f\n", data_read.theta, data_read.dtheta, data_read.wheel, data_read.dwheel, data_read.dwheel_f, data_read.u);
      // printf("-3.14,3.14,%f,%f,%f\n", error[0], error[1], error[2]);

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

float last_position_r, last_position_l, last_theta, ur, ul;
float g_x, g_y, g_z;
float a_pitch, a_yaw, a_roll;
float dwheel_f_r, dwheel_f_l, dtheta_f;
int pwm_value;

// PID velocidade
float Kp = -0.002;
float Ti = 10;
float Td = 1;
float ref_theta = 0.062;
float error_v, error_v_sum = Ti/Kp * ref_theta, last_error_v;

// PI theta
float Kp_theta = -17; //13
float Ti_theta = 20; //20
float error_theta, error_theta_sum = Ti/Kp * ref_theta;
#define LIM 7.4*Ti_theta/(Kp_theta * 5 * ts/1e3)
void taskControl(void* pvParameter) {
  pinMode(2, OUTPUT);

  xTaskCreate(&taskFlash, "task_flash", 4096, NULL, 2, NULL);

  Data data;
  TickType_t init_loop_time;
  unsigned long t0 = micros();
  unsigned long time = t0;
  unsigned long last_time = time;
  while(1) {
    init_loop_time = xTaskGetTickCount();

    float position_r = -2*PI*encoder_r.getCount()/(4*7*30);
    float position_l = -2*PI*encoder_l.getCount()/(4*7*30);
    mpu.getEvent(&a, &g, &temp);
    g_x = -(g.gyro.x + g_x_offset);

    a_pitch = -atan2(-a.acceleration.z, a.acceleration.y);

    // direita
    float alpha_wr = 0.5;
    data.dwheel = (position_r - last_position_r)/(ts/1e3);
    dwheel_f_r = alpha_wr*dwheel_f_r + (1 - alpha_wr)*(data.dwheel);
    data.dwheel_f = dwheel_f_r;
    data.wheel = position_r;
    // error[1] = ref[1] - k[1]*dwheel_f_r;
    // error[3] = ref[3] - k[3]*position_r;
    
    // esquerda
    dwheel_f_l = 0.9*dwheel_f_l + 0.1*(position_l - last_position_l)/(ts/1e3);
    // error[1] = ref[1] - k[1]*dwheel_f_l;
    // error[3] = ref[3] - k[3]*position_l;

    error[1] = ref[1] - k[1]*(dwheel_f_r + dwheel_f_l) / 2;

    // if (count % 10) {
    //   error_v = ((micros() - t0)/1e6 >= 5) * (0 - dwheel_f_r);
    //   error_v_sum += error_v;
    //   ref_theta = Kp * (error_v + 1/Ti*error_v_sum + Td*(error_v - last_error_v)/(10 * ts/1e3));
    //   last_error_v = error_v;
    // }

    // corpo lqr
    data.theta = alpha*(data.theta + g_x*(ts/1e3)) + (1-alpha)*a_pitch;
    dtheta_f = 0.95*dtheta_f + 0.05*(g_x);
    data.dtheta = dtheta_f;
    error[0] = ref[0] - k[0]*dtheta_f;
    error[2] = ref[2] - k[2]*(data.theta - ref_theta);

    //corpo PI
    if(count % 5) {
    error_theta = ref_theta - data.theta;
    error_theta_sum += error_theta;
    error_theta_sum = constrain(error_theta_sum, -LIM, LIM);
    error_theta  =  Kp_theta * (error_theta + 1/Ti_theta*error_theta_sum*(5 * ts/1e3));
    }

    // ur = (error[0] + error[2]);
    ur = (error[0] - error[1] + error[2]) + error_theta;
    // ul = (error[0] + error[1] + error[2] + error[3]);
    ul = ur;

    if (abs(data.theta) >= PI/4) {
      ur = 0; ul = 0;
    }

    data.u = ur;

    pwm_value = map(ur, -7.4, 7.4, -100, 100);
    motor(pwm_value, R_CHANNEL);

    pwm_value = map(ul, -7.4, 7.4, -100, 100);
    motor(pwm_value, L_CHANNEL);
    
    data_array[count % BUFFER_SIZE] = data;

    last_theta = data.theta;
    last_position_r = position_r;
    last_position_l = position_l;
    
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