#include <ESP32Encoder.h>
#include <math.h>
#include <stdio.h>

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

// Funções auxiliares
void motor(int PWM, int chanel);
void PinSetup();

// Variáveis Globais
const float TS = 10/1e3; // 10 milisegundos
const float Ti = 1/25.0; 
const float Kp = 3.75;

float t0, t;
unsigned long last_time;
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

float theta_ml, theta_mr; // posição angular do motor esquerdo e direito 
float last_theta_ml, last_theta_mr; 
float omega_ml, omega_mr; // velocidade angular do motor esquerdo e direito 
float u_ml, u_mr; // entrada dos motores
float error_ml, error_mr;
float sum_error_ml = 0, sum_error_mr = 0;

float ref = 0; // rad/s;

void setup() {
  Serial.begin(115200);
  PinSetup();

  motor(0, L_CHANNEL);
  motor(0, R_CHANNEL);

  printf("t;ref;u_ml;omega_ml;u_mr;omega_mr\n");

  delay(100);

  t0 = micros()/1e6;
}

unsigned long diff;
void loop() {
  if ((diff = micros() - last_time) >= TS*1e6) {
    last_time = micros();
    t = micros()/1e6 - t0; // tempo de simulação em segundos

    theta_ml = 2*PI * encoder_l.getCount()/(float)(4*7*30);
    theta_mr = 2*PI * encoder_r.getCount()/(float)(4*7*30);

    // calculo da velocidade: variação da posição sobre a variação do tempo -> (dtheta/dt)
    omega_ml = (theta_ml - last_theta_ml)/TS;
    omega_mr = (theta_mr - last_theta_mr)/TS;

    error_ml = ref - omega_ml;
    error_mr = ref - omega_mr;
    // integral do erro: somatório das áreas dos trapézios -> sum(erro * dt)
    sum_error_ml += error_ml * TS; 
    sum_error_mr += error_mr * TS; 

    u_ml = Kp * (error_ml + sum_error_ml/Ti);
    u_mr = Kp * (error_mr + sum_error_mr/Ti);

    motor(u_ml, L_CHANNEL);
    motor(u_mr, R_CHANNEL);

    // Descomente para observar as saídas no Arduino Serial Plot
    printf("limInf:-30,limSup:30,ref:%.04f,omega_ml:%.04f,omega_mr:%.04f\n", 
      ref, omega_ml, omega_mr);

    // Ou descomente este para ler os dados no CoolTerm
    // printf("%.04f;%.04f;%.04f;%.04f;%.04f;%.04f\n", t, ref, u_ml, omega_ml, u_mr, omega_mr);

    last_theta_ml = theta_ml;
    last_theta_mr = theta_mr;
  }

  if (t >= 3 && t < 6) { // referência 1
    ref = 2*PI;
  } else if (t >= 6 && t < 9) { // referência 2 
    ref = 4*PI;
  } else if (t >= 9 && t < 12) { // referência 3
    ref = 8*PI;
  } else if (t >= 12 && t < 15) { // referência 4
    ref = -8*PI;
  } else { // referência final
    ref = 0;
  }
}

void PinSetup() {
  // Configuração dos pinos
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

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
}

void motor(int PWM, int chanel) {
  int p1, p2;
  if (chanel == L_CHANNEL) {
    p1 = IN1;
    p2 = IN2;
  } else {
    p1 = IN3;
    p2 = IN4;
  }
  PWM = constrain(PWM, -100, 100); // saturador
  PWM = map(PWM, -100, 100, -255, 255); // altera a escala de -100:100 para -255:255
  if (PWM > 0) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
  } else if (PWM < 0) {
    PWM = -PWM;
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
  } else {
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
  }
  ledcWrite(chanel, PWM);
}
