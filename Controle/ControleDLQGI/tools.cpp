#include "tools.h"
#include <ESP32Encoder.h>

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
  bool PWM_Zero = PWM == 0 ? true : false;
  PWM = constrain(PWM, -100, 100);
  PWM = (PWM >= 0 ? 1 : -1) * mapfloat(abs(PWM), 0, 100, MIN_PWM, 100);
  PWM = map(PWM, -100, 100, -255, 255);
  if (PWM > 0) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
  } else if (PWM < 0) {
    PWM = -PWM;
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
  } else if (PWM_Zero) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
    PWM = 0;
  }
  ledcWrite(chanel, PWM);
}

