#include <ESP32Encoder.h>
#include <math.h>
#include <stdio.h>
#include <driver/pcnt.h>

#define ENCODER_C1_R 14
#define ENCODER_C2_R 27 

#define ENCODER_C1_L 33 
#define ENCODER_C2_L 32

#define IN1 17
#define IN2 16
#define ENA 4

#define IN3 18
#define IN4 19
#define ENB 23

#define L_CHANNEL 0
#define R_CHANNEL 1

#define SIMULATION_TIME 0.4

// Funções auxiliares
void motor(int PWM, int chanel);
void PinSetup();

// Variáveis Globais
const float TS = 10/1e3; // 10 milisegundos
float t0, t;
unsigned long last_time;
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;
float theta_ml, theta_mr; // posição angular do motor esquerdo e direito 
float u_ml, u_mr; // entrada dos motores

void setup() {
  Serial.begin(115200);
  PinSetup();

  motor(0, L_CHANNEL);
  motor(0, R_CHANNEL);

  delay(100);

  t0 = micros()/1e6;
  Serial.println("t;u_ml;theta_ml;u_mr;theta_mr");
}

unsigned long diff;
void loop() {
  if ((diff = micros() - last_time) >= TS*1e6) {
    last_time = micros();
    t = micros()/1e6 - t0;

    theta_ml = 2*PI * encoder_l.getCount()/(float)(4*7*30);
    theta_mr = 2*PI * encoder_r.getCount()/(float)(4*7*30);

    if (t < SIMULATION_TIME) {
      u_ml = 100;
      u_mr = 100;
    } else if (t < SIMULATION_TIME*2) {
      u_ml = -100;
      u_mr = -100;
    } else {
      motor(0, L_CHANNEL);
      motor(0, R_CHANNEL);

      while (true) delay(1000);
    }

    motor(u_ml, L_CHANNEL);
    motor(u_mr, R_CHANNEL);

    printf("%.04f;%.04f;%.04f;%.04f;%.04f\n", t, u_ml, theta_ml, u_mr, theta_mr);
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
  PWM = constrain(PWM, -100, 100);
  PWM = map(PWM, -100, 100, -255, 255);
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
