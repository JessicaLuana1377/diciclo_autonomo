#include "SPIFFS.h"
#include "tools.h"

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
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
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
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
  if (PWM == 0) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
    return;
  } else if (PWM > 0) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
  } else {
    PWM = -PWM;
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
  }
  ledcWrite(chanel, PWM);
}

int fileCountInit() {
  int file_count = 0;

  File file = SPIFFS.open(FILE_COUNT_BKP, FILE_READ);
  if(!file || file.isDirectory()){
    file_count = 0;
  }
  while(file.available()){
    file.read((uint8_t*)&file_count, sizeof(file_count));
  }
  file.close();

  Serial.println("File count init = " + String(file_count));

  return file_count;
}
