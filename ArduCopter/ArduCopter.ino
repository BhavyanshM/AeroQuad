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

// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs


// Declare Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5;
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
  Serial.begin(9600);

  m1.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  m2.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  m3.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  m4.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

  delay(1000);

//  m1.writeMicroseconds(MIN_PULSE_LENGTH);
//  m2.writeMicroseconds(MIN_PULSE_LENGTH);
  m3.writeMicroseconds(MIN_PULSE_LENGTH);
//  m4.writeMicroseconds(MIN_PULSE_LENGTH);
  
}



ISR(PCINT0_vect){
  // Channel 1
  if(last_channel_1 == 0 && PINB & B00000001 ){
    last_channel_1 = 1;
    timer_1 = micros();
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001)){
    last_channel_1 = 0;
    receiver_input_channel_1 = micros() - timer_1;
  }  

  // Channel 2
  if(last_channel_2 == 0 && PINB & B00000010 ){
    last_channel_2 = 1;
    timer_2 = micros();
  }
  else if(last_channel_2 == 1 && !(PINB & B00000010)){
    last_channel_2 = 0;
    receiver_input_channel_2 = micros() - timer_2;
  } 

  // Channel 3
  if(last_channel_3 == 0 && PINB & B00000100 ){
    last_channel_3 = 1;
    timer_3 = micros(); 
  }
  else if(last_channel_3 == 1 && !(PINB & B00000100)){
    last_channel_3 = 0;
    receiver_input_channel_3 = micros() - timer_3;
  } 

  // Channel 4
  if(last_channel_4 == 0 && PINB & B00001000 ){
    last_channel_4 = 1;
    timer_4 = micros(); 
  }
  else if(last_channel_4 == 1 && !(PINB & B00001000)){
    last_channel_4 = 0;
    receiver_input_channel_4 = micros() - timer_4;
  } 

  // Channel 4
  if(last_channel_4 == 0 && PINB & B00001000 ){
    last_channel_4 = 1;
    timer_4 = micros(); 
  }
  else if(last_channel_4 == 1 && !(PINB & B00001000)){
    last_channel_4 = 0;
    receiver_input_channel_4 = micros() - timer_4;
  } 

  // Channel 5: Throttle
  if(last_channel_5 == 0 && PINB & B00010000 ){
    last_channel_5 = 1;
    timer_5 = micros(); 
  }
  else if(last_channel_5 == 1 && !(PINB & B00010000)){
    last_channel_5 = 0;
    receiver_input_channel_5 = micros() - timer_5;
  }
}


void print_signals(){
  Serial.print("Roll:");
  if(receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
  else Serial.print(" -+- ");
  Serial.print(receiver_input_channel_1);

  Serial.print("\tPitch:");
  if(receiver_input_channel_2 - 1480 < 0)Serial.print(" ^^^ ");
  else if(receiver_input_channel_2 - 1520 > 0)Serial.print(" vvv ");
  else Serial.print(" -+- ");
  Serial.print(receiver_input_channel_2);

  Serial.print("\tGear:");
  if(receiver_input_channel_3 - 1480 < 0)Serial.print(" vvv ");
  else if(receiver_input_channel_3 - 1520 > 0)Serial.print(" ^^^ ");
  else Serial.print(" -+- ");
  Serial.print(receiver_input_channel_3);

  Serial.print("\tYaw:");
  if(receiver_input_channel_4 - 1480 < 0)Serial.print(" <<< ");
  else if(receiver_input_channel_4 - 1520 > 0)Serial.print(" >>> ");
  else Serial.print(" -+- ");
  Serial.print(receiver_input_channel_4);

  Serial.print("\tThrottle:");
  if(receiver_input_channel_5 - 1480 < 0)Serial.print(" <<< ");
  else if(receiver_input_channel_5 - 1520 > 0)Serial.print(" >>> ");
  else Serial.print(" -+- ");
  Serial.println(receiver_input_channel_5);
}
