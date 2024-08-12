// Arquivo simplificado para o fernando

#include "tools.h"

// CONST VARIABLES
// Experimento do corpo
const float f0 = 0.5;       // Frequência inicial
const float f1 = 15;      // Frequência final
#define MAX_PWM 30        // Máximo de PWM aplicado

// Experimento das rodas
// const float f0 = 0.2;       // Frequência inicial
// const float f1 = 20;      // Frequência final
// #define MAX_PWM 50        // Máximo de PWM aplicado

const int ts = 10;
const int T = 120;         // Tempo de uma simulação
const int uT = T*1000000; // T em microsegundos
#define SIMULATIONS 2     // Quantidade de simulações

void setup() {
  
  Serial.begin(115200);
  delay(500);
  
  InitSetup();

  Serial.print("StartMode:");
  Serial.println(startMode);
  
  // Modo simulação - Apertar botão BOOT 
  if (startMode == RUN) {
    digitalWrite(2, LOW);
    printf("Running Mode...\n");

    delay(2000);
    xTaskCreate(&taskControl, "task_control", 4096, NULL, 4, NULL);
  }

  // Modo configuração (leitura, formatar) - esperar 5 segundos
  else if (startMode == CONFIG) {
    digitalWrite(2, HIGH);
    printf("Configuration Mode (l:list, f:format, r:read and format, R:read only)\n");
  }
}

void taskControl(void* pvParameter) {
  pinMode(2, OUTPUT);
  xTaskCreate(&taskFlash, "task_flash", 4096, NULL, 2, NULL); // thread para salvar os dados do experimento na flash

  Data data;                      // struct com os dados do experimento
  TickType_t init_loop_time;      // variavel para manter o tempo de amostragem
  unsigned long t0 = micros();    // tempo inicial da simulação
  unsigned long time = t0;        // tempo usado para calcular o PWM
  mpu.getEvent(&a, &g, &temp);
  a_pitch = atan2(a.acceleration.z, a.acceleration.y);
  float dtheta = a_pitch;  // ângulo inicial do pêndulo
  while(1) {
    init_loop_time = xTaskGetTickCount();
    time = micros();

    float position_r = encoder_r.getCount();
    float position_l = encoder_l.getCount();

    mpu.getEvent(&a, &g, &temp);
    g_x = -(g.gyro.x + g_x_offset);
    dtheta = g_x;

    // calculo do tempo triangular
    int temp = (time - t0 + uT) % (2 * uT) - uT;
    float t = abs(temp) / usTOs;
    // calculo da chirp
    int pwm_value = (int)(MAX_PWM * sin(-2 * PI * f0 * f1 * T / (f1 - f0) * log(1 - (f1 - f0) / (f1 * T) * t)));
    motor(pwm_value, L_CHANNEL);
    motor(-pwm_value, R_CHANNEL);
    
    // salvar dados
    data.t = (time - t0)/usTOs;
    data.u = pwm_value;
    data.yr = position_r;
    data.yl = position_l;
    data.freq = f0 * f1 * T / ((f0 - f1) * t + f1 * T);
    data.dtheta = dtheta;
    data_array[count % BUFFER_SIZE] = data;
    count++;

    // caso ocorra overflow - não costuma acontecer
    if (startMode == BUFFER_OVERFLOW) {
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

    // fim das simulações
    if (time - t0 >= uT*SIMULATIONS) {
      motor(0, L_CHANNEL);
      motor(0, R_CHANNEL);

      startMode = CONFIG;
      digitalWrite(2, HIGH);

      printf("End Control\n");
      printf("Configuration Mode (l:list, f:format, r:read and format, R:read only)\n");
      break;
    }

    vTaskDelayUntil(&init_loop_time, pdMS_TO_TICKS(ts));
  }

  vTaskDelete(NULL);
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

      Data data_read;
      File file = root.openNextFile();
      while(file){
        if (!file.isDirectory()) {
          String filename = "/" + String(file.name());

          File file = SPIFFS.open(filename.c_str());
          
          if(!file || file.isDirectory()){
            Serial.println("- failed to open file for reading");
            break;
          }
          if (!filename.equals(String(FILE_COUNT_BKP))) {
            Serial.println("Reading: " + filename);

            Serial.println("- read from file:");
            Serial.println("t;u;yr;yl;freq;dtheta");
            while(file.available()){
              // Serial.write(file.read());
              if (file.read((uint8_t*)&data_read, sizeof(Data)) != sizeof(Data)) {
                break;
              }
              sprintf(buffer, "%.4f;%d;%ld;%ld;%.4f;%.4f\n", 
                data_read.t, data_read.u, data_read.yr, data_read.yl, data_read.freq, data_read.dtheta);
              Serial.write(buffer);
            }
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
