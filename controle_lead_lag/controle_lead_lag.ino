#include "tools.h"

// CONST VARIABLES
const int ts = 5;
const float alpha_w = 0.5;
const float alpha_b = 0.95;
const float r_w = 0.03;
const float k_A_theta[3] = {1.0000, -1.7639, 0.7639};
const float k_B_theta[3] = {-72.9757, 136.5322, -63.8437};
const float k_A_phi[3]     = {1.0000, -1.3270, 0.4037};
const float k_B_phi[3]     = {0, 0.0323, -0.0321};

float position_r, position_l, phi, theta;
float last_position_r, last_position_l, last_theta, ur, ul;
float dwheel_f_r, dwheel_f_l, dtheta_f;
int pwm_value;

float u[3] = {0};
float error_theta[3] = {0};
float theta_ref[3] = {-0.062, -0.062, -0.062};
float error_phi[3] = {0};

void taskControl(void* pvParameter) {
  

  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(921600);
  delay(500);

  InitSetup();

  delay(100);;
  digitalWrite(2, LOW);
  delay(1000);
  printf("Running Mode...\n");
}

void loop() {
  pinMode(2, OUTPUT);

  xTaskCreate(&taskFlash, "task_flash", 4096, NULL, 10, NULL);

  Data data;
  TickType_t init_loop_time;
  unsigned long t0 = micros();
  unsigned long time = t0;
  unsigned long last_time = time;
  printf("primeiro\nx:%f,theta:%f,theta_ref:%f,u:%f\nfim\n", phi, theta, theta_ref[0], 1*(k_B_phi[1]*error_phi[1] + k_B_phi[2]*error_phi[2]));
      
  while(1) {
    init_loop_time = xTaskGetTickCount();

    position_r = 2*PI*encoder_r.getCount()/(float)(4*7*30);
    position_l = 2*PI*encoder_l.getCount()/(float)(4*7*30);
    phi = (position_l - position_r)/2;

    mpu.getEvent(&a, &g, &temp);
    g_x = -(g.gyro.x + g_x_offset);
    a_pitch = atan2(a.acceleration.z, a.acceleration.y);

    // direita
    data.dwheel = (position_r - last_position_r)/(ts/1e3);
    dwheel_f_r = alpha_w*dwheel_f_r + (1-alpha_w)*(position_r - last_position_r)/(ts/1e3);
    data.dwheel_f = dwheel_f_r;
    data.wheel = position_r;
    
    // esquerda
    dwheel_f_l = alpha_w*dwheel_f_l + (1-alpha_w)*(position_l - last_position_l)/(ts/1e3);

    if (true) {
      error_phi[0] = 0 - phi;

      printf("x:%f,theta:%f,theta_ref:%f,u:%f\n", phi, theta, theta_ref[0], 1*(k_B_phi[1]*error_phi[1] + k_B_phi[2]*error_phi[2]));
      theta_ref[0] = -1*(k_A_phi[1]*theta_ref[1] + k_A_phi[2]*theta_ref[2]) + 
        1*(k_B_phi[1]*error_phi[1] + k_B_phi[2]*error_phi[2]);
      theta_ref[0] = constrain(theta_ref[0], -PI/4, PI/4);

      error_phi[2] = error_phi[1];
      error_phi[1] = error_phi[0];

      theta_ref[2] = theta_ref[1];
      theta_ref[1] = theta_ref[0];

    }

    // corpo
    theta = alpha*(data.theta + g_x*(ts/1e3)) + (1-alpha)*a_pitch;
    dtheta_f = alpha_b*dtheta_f + (1-alpha_b)*(g_x);
    data.dtheta = dtheta_f;
    data.theta = theta;

    error_theta[0] = theta_ref[0] - theta;

    u[0] = -1*(k_A_theta[1]*u[1] + k_A_theta[2]*u[2]) + 
      1*(k_B_theta[0]*error_theta[0] + k_B_theta[1]*error_theta[1] + k_B_theta[2]*error_theta[2]);
    u[0] = constrain(u[0], -7.4, 7.4);

    ul = u[0];
    ur = -ul;

    if (abs(theta) >= PI/4) {
      ur = 0; ul = 0;
      theta_ref[0] = 0;
    }

    data.u = ur;

    pwm_value = map(ur, -7.4, 7.4, -100, 100);  
    motor(pwm_value, R_CHANNEL);

    pwm_value = map(ul, -7.4, 7.4, -100, 100);
    motor(pwm_value, L_CHANNEL);
    
    data_array[count % BUFFER_SIZE] = data;

    last_theta = theta;
    last_position_r = position_r;
    last_position_l = position_l;

    error_theta[2] = error_theta[1];
    error_theta[1] = error_theta[0];

    u[2] = u[1];
    u[1] = u[0];
    
    count++;

    // if (count >= 100) {
    //   motor(0, R_CHANNEL);
    //   motor(0, L_CHANNEL);
    //   while (true) {
    //     delay(10);
    //   }
    // }

    vTaskDelayUntil(&init_loop_time, pdMS_TO_TICKS(ts));
  }
}