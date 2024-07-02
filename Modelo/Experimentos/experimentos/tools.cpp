#include "tools.h"

// GLOBALS VARIABLES
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

Data *data_array;
Modes startMode; 

MPU6050 mpu;
int16_t ax_b, ay_b, az_b, gx_b, gy_b, gz_b;
float ax, ay, az, gx, gy, gz;
float tau;
float theta;

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

float getAccelPitch()
{
  getAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
  float accel_pitch = atan2(az, ay);
  return accel_pitch;
}

void getAccelGyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
  mpu.getMotion6(&ax_b, &ay_b, &az_b, &gx_b, &gy_b, &gz_b); // gyro (+/- 250 deg/s) accel (+/- 2g)
  // as variaveis ##_b são inteiros de 16 bits, 2^16 - 1 = 65535, a divisão por esse numero é para normalizar de 0 a 1
  // o primeiro 2 é para pegar todo o intervalo, -x a x resulta em um intervalo de x - (-x) = 2x
  ax[0] = 2 * 2   * ((ax_b - X_ACCEL_OFFSET) / 65535.0);                       // g
  ay[0] = 2 * 2   * ((ay_b - Y_ACCEL_OFFSET) / 65535.0);                       // g 
  az[0] = 2 * 2   * ((az_b - Z_ACCEL_OFFSET) / 65535.0);                       // g
  gx[0] = 2 * 250 * ((gx_b - X_GYRO_OFFSET) / (65535.0)) * PI / 180;          // rad/s
  gx[0] = 2 * 250 * ((gy_b - Y_GYRO_OFFSET) / (65535.0)) * PI / 180;          // rad/s
  gz[0] = 2 * 250 * ((gz_b - Z_GYRO_OFFSET) / (65535.0)) * PI / 180;          // rad/s
}

float updatePitch(float currentAngle, float loop_period)
{
  // Get imu data.
  getAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
  // Pitch estimation from accelerometer.
  float accelPitch = getAccelPitch();
  // Complementary filter between acceleration and gyroscopic pitch estimation.
  float pitch_f = (tau) * (currentAngle + (gx)*loop_period) + (1 - tau) * (accelPitch);
  pitch_f = -pitch_f;

  return pitch_f;
}

void PinSetup() // realiza o PinOut de todos os dispositivos
{
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
	encoder_r.attachFullQuad(ENCODER_DA_R, ENCODER_DB_R);
	encoder_l.attachFullQuad(ENCODER_DA_L, ENCODER_DB_L);
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
  String text = "t;u;yr;yl;t_trig;freq;theta\n";
  file.print(text.c_str());

  Data data_read;
  while(1) {
    if (flash_count < count) {
      data_read = data_array[flash_count % BUFFER_SIZE];
      // printf("%f %d\n", data_read.t, count - flash_count);

      if (count - flash_count > BUFFER_SIZE) startMode = BUFFER_OVERFLOW;
      // else startMode = RUN;

      sprintf(buffer, "%.4f;%d;%.4f;%.4f;%.4f;%.4f;%.4f\n", 
        data_read.t, data_read.u, data_read.yr, data_read.yl, data_read.t_trig, data_read.freq, data_read.theta);
      file.print(buffer);

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
  tau = 0.98;
  count = 0;
  flash_count = 0;
  file_count = 0;

  PinSetup();

  Wire.begin();

  delay(1000);
  
  // Inicialização do MPU6050
  Serial.println("Inicializando MPU6050...");
  mpu.initialize();
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
