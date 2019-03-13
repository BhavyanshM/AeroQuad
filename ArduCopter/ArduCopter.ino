#include <Wire.h>
#include <Servo.h>

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = A5;
const uint8_t sda = A4;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal, accel_pitch_cal, accel_roll_cal, accel_yaw_cal;
double Ax, Ay, Az, T, Gx, Gy, Gz;
double acc_sqrt, acc_angle_roll, acc_angle_pitch, gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw, total_angle_roll, total_angle_pitch, total_angle_yaw;
double rad_deg = 57.295779513;
double past_time, cur_time, elapsed_time;

// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs


// Declare Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5;
int receiver_input_roll, receiver_input_pitch, receiver_input_gear, receiver_input_yaw, receiver_input_throttle;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, esc_timer, current_time;

Servo m1, m2, m3, m4;

void setup() {
  DDRD |= B00111100;
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  PCMSK0 |= (1 << PCINT4);

  Serial.begin(115200);
  Wire.begin();
  MPU6050_Init();

  m1.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  m2.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  m3.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  m4.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

  for(int i = 0; i<2000; i++){
      Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
 
      gyro_pitch_cal += (double)GyroX/GyroScaleFactor;
      gyro_roll_cal += (double)GyroY/GyroScaleFactor;
      gyro_yaw_cal += (double)GyroZ/GyroScaleFactor;

      accel_pitch_cal += (double)AccelX/AccelScaleFactor;
      accel_roll_cal += (double)AccelY/AccelScaleFactor;
      accel_yaw_cal += (double)AccelZ/AccelScaleFactor;

  }

  gyro_pitch_cal /= 2000;
  gyro_roll_cal /=2000;
  gyro_yaw_cal /=2000;

  accel_pitch_cal /= 2000;
  accel_roll_cal /=2000;
  accel_yaw_cal /=2000;
  
//  Serial.print(gyro_pitch_cal); Serial.print("\t");
//  Serial.print(gyro_roll_cal); Serial.print("\t");
//  Serial.print(gyro_yaw_cal); Serial.print("\n");

  
  delay(1000);

//  m1.writeMicroseconds(MIN_PULSE_LENGTH);
//  m2.writeMicroseconds(MIN_PULSE_LENGTH);
//  m3.writeMicroseconds(MIN_PULSE_LENGTH);
//  m4.writeMicroseconds(MIN_PULSE_LENGTH);
  
}



ISR(PCINT0_vect){
  // Channel 1: Pin 7: ROLL
  if(last_channel_1 == 0 && PINB & B00000001 ){
    last_channel_1 = 1;
    timer_1 = micros();
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001)){
    last_channel_1 = 0;
    receiver_input_roll = micros() - timer_1;
  }  

  // Channel 2: Pin 9: PITCH
  if(last_channel_2 == 0 && PINB & B00000010 ){
    last_channel_2 = 1;
    timer_2 = micros();
  }
  else if(last_channel_2 == 1 && !(PINB & B00000010)){
    last_channel_2 = 0;
    receiver_input_pitch = micros() - timer_2;
  } 

  // Channel 3: Pin 10: GEAR
  if(last_channel_3 == 0 && PINB & B00000100 ){
    last_channel_3 = 1;
    timer_3 = micros(); 
  }
  else if(last_channel_3 == 1 && !(PINB & B00000100)){
    last_channel_3 = 0;
    receiver_input_gear = micros() - timer_3;
  } 

  // Channel 4: Pin 11: YAW
  if(last_channel_4 == 0 && PINB & B00001000 ){
    last_channel_4 = 1;
    timer_4 = micros(); 
  }
  else if(last_channel_4 == 1 && !(PINB & B00001000)){
    last_channel_4 = 0;
    receiver_input_yaw = micros() - timer_4;
  } 

  // Channel 5: Pin 12: THROTTLE
  if(last_channel_5 == 0 && PINB & B00010000 ){
    last_channel_5 = 1;
    timer_5 = micros(); 
  }
  else if(last_channel_5 == 1 && !(PINB & B00010000)){
    last_channel_5 = 0;
    receiver_input_throttle = micros() - timer_5;
  }
}
