// Arquivo simplificado para o fernando

#include "tools.h"

// CONST VARIABLES
const int ts = 10;
const float f0 = 1;       // Frequência inicial
const float f1 = 20;      // Frequência final
const int T = 30;         // Tempo de uma simulação
const int uT = T*1000000; // T em microsegundos
#define SIMULATIONS 4     // Quantidade de simulações
#define MAX_PWM 50        // Máximo de PWM aplicado

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
    xTaskCreate(&taskControl, "task_control", 4096, NULL, 1, NULL);
  }

  // Modo configuração (leitura, formatar) - esperar 5 segundos
  else if (startMode == CONFIG) {
    digitalWrite(2, HIGH);
    printf("Configuration Mode (l:list, f:format, r:read)\n");
  }
}

void taskControl(void* pvParameter) {
  pinMode(2, OUTPUT);

  xTaskCreate(&taskFlash, "task_flash", 4096, NULL, 2, NULL); // thread para salvar os dados do experimento na flash

  Data data;                      // struct com os dados do experimento
  TickType_t init_loop_time;      // variavel para manter o tempo de amostragem
  unsigned long t0 = micros();    // tempo inicial da simulação
  unsigned long time = t0;        // tempo usado para calcular o PWM
  float theta = getAccelPitch();  // ângulo inicial do pêndulo
  while(1) {
    init_loop_time = xTaskGetTickCount();

    float position_r = encoder_r.getCount();
    float position_l = encoder_l.getCount();
    theta = updatePitch(theta, (micros() - time)/usTOs);

    // calculo do tempo triangular
    time = micros();
    int temp = (time - t0 + uT) % (2 * uT) - uT;
    float t = abs(temp) / usTOs;
    // calculo da chirp
    int pwm_value = (int)(MAX_PWM * sin(-2 * PI * f0 * f1 * T / (f1 - f0) * log(1 - (f1 - f0) / (f1 * T) * t)));
    motor(pwm_value, L_CHANNEL);
    motor(pwm_value, R_CHANNEL);
    
    // salvar dados
    data.t = (time - t0)/usTOs;
    data.u = pwm_value;
    data.yr = position_r;
    data.yl = position_l;
    data.t_trig = t;
    data.freq = f0 * f1 * T / ((f0 - f1) * t + f1 * T);
    data.theta = theta;
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
      printf("Configuration Mode (l:list, f:format, r:read)\n");
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
