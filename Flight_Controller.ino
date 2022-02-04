#include <driver/ledc.h>
#include <driver/gpio.h>
#include "Wire.h"
#include <MPU6050_light.h>

#define GPIO_INPUT_IO_0     GPIO_NUM_33
#define GPIO_INPUT_IO_1     GPIO_NUM_25
#define GPIO_INPUT_IO_2     GPIO_NUM_26
#define GPIO_INPUT_IO_3     GPIO_NUM_27
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2)|(1ULL<<GPIO_INPUT_IO_3))
#define ESP_INTR_FLAG_DEFAULT 0

MPU6050 mpu(Wire);
volatile uint16_t timer[4]={1000,1000,1000,1000},prev_time[4];

float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400; 

boolean auto_level = true;   
int start;

float throttle;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
int esc_1,esc_2,esc_3,esc_4;


static void IRAM_ATTR gpio_isr_handler4(void* arg)
{
    timer[3] =  esp_timer_get_time() - prev_time[3] ; 
    prev_time[3] =  esp_timer_get_time();
}
static void IRAM_ATTR gpio_isr_handler1(void* arg)
{
    timer[0] =esp_timer_get_time() - prev_time[0] ; 
    prev_time[0] = esp_timer_get_time();
}
static void IRAM_ATTR gpio_isr_handler2(void* arg)
{
    timer[1] =esp_timer_get_time() - prev_time[1] ; 
    prev_time[1] = esp_timer_get_time();
}
static void IRAM_ATTR gpio_isr_handler3(void* arg)
{
    timer[2] = esp_timer_get_time() - prev_time[2] ; 
    prev_time[2] = esp_timer_get_time();
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  ledcAttachPin(4, 1);
  ledcAttachPin(16, 2);
  ledcAttachPin(17, 3);
  ledcAttachPin(5, 4);

  ledcSetup(1, 500, 11); 
  ledcSetup(2, 500, 11);
  ledcSetup(3, 500, 11);
  ledcSetup(4, 500, 11);

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler1, (void*) GPIO_INPUT_IO_0);
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler2, (void*) GPIO_INPUT_IO_1);
  gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler3, (void*) GPIO_INPUT_IO_2);
  gpio_isr_handler_add(GPIO_INPUT_IO_3, gpio_isr_handler4, (void*) GPIO_INPUT_IO_3);

  ledcWrite(1, timer[0]);
  ledcWrite(2, timer[1]);
  ledcWrite(3, timer[2]);
  ledcWrite(4, timer[3]);
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  delay(1000);
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void loop() {
  
  mpu.update();
  angle_pitch = mpu.getAngleX();
  angle_roll = mpu.getAngleY();
  gyro_pitch_input = mpu.getAngleX();
  gyro_roll_input = mpu.getAngleY();
 
  pid_roll_setpoint = 0;
  pid_roll_setpoint = timer[0] - 1648;
  pid_roll_setpoint /= 10;                                             

  pid_pitch_setpoint = 0;
  pid_pitch_setpoint = timer[1] - 1493;
  pid_pitch_setpoint /= 10.0;                                                

  pid_yaw_setpoint = 0;
  if(timer[2] > 1050){ 
    pid_yaw_setpoint = (timer[3] - 1496)/10.0;
  }
  
  calculate_pid();

  throttle = timer[2];                                      
  if (start == 2){                                                         
    if (throttle > 1800) throttle = 1800;                                   
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
//
//    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
//      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
//      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
//      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
//      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
//    } 

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

   ledcWrite(1, esc_1);
   ledcWrite(2, esc_1);
   ledcWrite(3, esc_1);
   ledcWrite(4, esc_1);

  Serial.print(pid_roll_setpoint);
  Serial.print(" ");
  Serial.print(pid_pitch_setpoint);
  Serial.print(" ");
  Serial.println(pid_yaw_setpoint);

   
}
