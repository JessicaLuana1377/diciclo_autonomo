
#include <ESP32Encoder.h>
// Pinos do encoderes
#define ENCODER_C1_R 32
#define ENCODER_C2_R 33 
#define ENCODER_C1_L 14
#define ENCODER_C2_L 27
#define TS 500 // Tempo de amostragem em milissegundos

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

// Variaveis dos encoder
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

unsigned long t0 = millis();
void setup() {
  Serial.begin(115200);
  delay(100);

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

void loop() {
  if (millis() - t0 > TS){
    t0 = millis();
    float position_r = encoder_r.getCount();
    float position_l = encoder_l.getCount();

    // Comente essas duas linhas para testar o encoder individualmente
    motor(10,L_CHANNEL);
    motor(100,R_CHANNEL);

    Serial.print(" Encocer direito: ");
    Serial.print(position_r);
    Serial.print(" Encocer esquerdo: ");
    Serial.print(position_l);
    Serial.println();
  }
}

void motor(int PWM, int chanel) 
{
  int p1, p2;
  if (chanel == L_CHANNEL) 
  {
    p1 = IN1;
    p2 = IN2;
  } else {
    p1 = IN3;
    p2 = IN4;
  }
  PWM = constrain(PWM, -100, 100);
  PWM = map(PWM, -100, 100, -255, 255);
  if (PWM == 0) 
  {
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
    return;
  } else if (PWM > 0) 
  {
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
  } else 
  {
    PWM = -PWM;
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
  }
  ledcWrite(chanel, PWM);
}
