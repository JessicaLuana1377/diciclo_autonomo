#include <ESP32Encoder.h>
// Pinos do encoderes
#define ENCODER_DA_R 14
#define ENCODER_DB_R 27 
#define ENCODER_DA_L 32
#define ENCODER_DB_L 33
#define TS 500 // Tempo de amostragem em milissegundos

// Variaveis dos encoder
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

unsigned long t0 = millis();
void setup() {
  Serial.begin(115200);
  delay(100);

  // Configuração dos encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
	encoder_r.attachFullQuad(ENCODER_DA_R, ENCODER_DB_R);
	encoder_l.attachFullQuad(ENCODER_DA_L, ENCODER_DB_L);
  encoder_r.clearCount();
  encoder_l.clearCount();
}

void loop() {
  if (millis() - t0 > TS){
    t0 = millis();
    float position_r = encoder_r.getCount();
    float position_l = encoder_l.getCount();

    Serial.print(" Encocer direito: ");
    Serial.print(position_r);
    Serial.print(" Encocer esquerdo: ");
    Serial.print(position_l);
    Serial.println();
  }
}
