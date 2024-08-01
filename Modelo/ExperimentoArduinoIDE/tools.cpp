#include "tools.h"

// GLOBALS VARIABLES
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

Data *data_array;
Modes startMode; 

// MPU6050 mpu;
Adafruit_MPU6050 mpu;
float g_x_offset = -0.05 + 0.097;
sensors_event_t a, g, temp;
int16_t ax_b, ay_b, az_b, gx_b, gy_b, gz_b;
float ax, ay, az, gx, gy, gz;
float alpha;
float dtheta, a_pitch, g_x;

char buffer[50];
int count;
int flash_count;
int file_count;

void listDir(fs::FS &fs, const char * dirname, uint8_t levels)
{
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

int fileCountInit() 
{
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

void PinSetup() // realiza o PinOut de todos os dispositivos
{
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

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

void taskFlash(void* pvParameter) 
{
  String filename = "/arquivo" + String(file_count) + ".csv";
  Serial.println(filename);
  File file = SPIFFS.open(filename, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }

  Data data_read;
  while(1) {
    if (flash_count < count) {
      data_read = data_array[flash_count % BUFFER_SIZE];

      if (count - flash_count > BUFFER_SIZE) startMode = BUFFER_OVERFLOW;

      file.write((const uint8_t*)&data_read, sizeof(Data));

      flash_count++;
    } else if (startMode == CONFIG || (startMode == BUFFER_OVERFLOW && flash_count == count)) {
      file.close();

      file_count++;
      file = SPIFFS.open(FILE_COUNT_BKP, FILE_WRITE);
      if(!file || file.isDirectory()) {
        Serial.println("- failed to open file bkp for something");
      }
      file.write((uint8_t*)&file_count, sizeof(file_count));
      file.close();

      printf("End Flash\n");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  vTaskDelete(NULL);
}

void taskBlinkLed(void* pvParameter) 
{
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

void InitSetup()
{

  // Inicialização de variáveis
  startMode = INIT;
  alpha = 0.98;
  count = 0;
  flash_count = 0;
  file_count = 0;

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

  // Serial.println("Criando a TaskBlinkLed");
  xTaskCreate(&taskBlinkLed, "blink_led", 2048, NULL, 3, NULL);
  while (startMode == INIT) delay(100);
}
