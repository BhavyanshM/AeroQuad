

void loop() {  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;

  Gx -= gyro_pitch_cal;
  Gy -= gyro_roll_cal;
  Gz -= gyro_yaw_cal;

  Ax -= accel_pitch_cal;
  Ay -= accel_roll_cal;
//  Az -= accel_yaw_cal;

  // Calculate Time Differential
  past_time = cur_time;
  cur_time = millis();
  elapsed_time = (cur_time - past_time)/1000;
  
  // GYRO: PITCH_X, ROLL_Y, YAW_Z
  gyro_angle_pitch = gyro_angle_pitch + Gx*elapsed_time;
  gyro_angle_roll = gyro_angle_roll + Gy*elapsed_time;
  gyro_angle_yaw = gyro_angle_yaw + Gz*elapsed_time;
  Serial.print(gyro_angle_pitch);Serial.print("\t "); 
  Serial.print(gyro_angle_roll);Serial.print("\t ");
  Serial.print(gyro_angle_yaw);Serial.print("\t ");  

  // ACCEL: ROLL_X, PITCH_Y
  acc_angle_roll = atan(Ax/sqrt(pow(Ay, 2) + pow(Az, 2)))*rad_deg;
  acc_angle_pitch = atan(Ay/sqrt(pow(Ax, 2) + pow(Az, 2)))*rad_deg; 
  Serial.print(acc_angle_roll);
  Serial.print("\t "); Serial.println(acc_angle_pitch);

  // Complementary Filter
  total_angle_roll = 0.98*(total_angle_roll + gyro_angle_roll) + 0.02*(acc_angle_roll);
  total_angle_pitch = 0.98*(total_angle_pitch + gyro_angle_pitch) + 0.02*(acc_angle_pitch);
//  Serial.print(total_angle_roll);
//  Serial.print("\t "); Serial.println(total_angle_pitch);

//  Serial.print("Ax: "); Serial.print(Ax);
//  Serial.print("\tAy: "); Serial.print(Ay);
//  Serial.print("\tAz: "); Serial.print(Az);
//  Serial.print("\tT: "); Serial.print(T);
//  Serial.print("\tGx: "); Serial.print(Gx);
//  Serial.print("\tGy: "); Serial.print(Gy);
//  Serial.print("\tGz: "); Serial.println(Gz);

  if(receiver_input_throttle > 1135){
    m1.writeMicroseconds(receiver_input_throttle);
    m2.writeMicroseconds(receiver_input_throttle);
    m3.writeMicroseconds(receiver_input_throttle);
    m4.writeMicroseconds(receiver_input_throttle);
  }
//  Serial.println(receiver_input_channel_5);


//  print_signals();
}


void print_signals(){
  Serial.print("Roll:");
  if(receiver_input_roll - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_roll - 1520 > 0)Serial.print(">>>");
  else Serial.print(" -+- ");
  Serial.print(receiver_input_roll);

  Serial.print("\tPitch:");
  if(receiver_input_pitch- 1480 < 0)Serial.print(" ^^^ ");
  else if(receiver_input_pitch - 1520 > 0)Serial.print(" vvv ");
  else Serial.print(" -+- ");
  Serial.print(receiver_input_pitch);

  Serial.print("\tGear:");
  if(receiver_input_gear- 1480 < 0)Serial.print(" vvv ");
  else if(receiver_input_gear - 1520 > 0)Serial.print(" ^^^ ");
  else Serial.print(" -+- ");
  Serial.print(receiver_input_gear);

  Serial.print("\tYaw:");
  if(receiver_input_yaw- 1480 < 0)Serial.print(" <<< ");
  else if(receiver_input_yaw - 1520 > 0)Serial.print(" >>> ");
  else Serial.print(" -+- ");
  Serial.print(receiver_input_yaw);

  Serial.print("\tThrottle:");
  if(receiver_input_throttle- 1480 < 0)Serial.print(" <<< ");
  else if(receiver_input_throttle - 1520 > 0)Serial.print(" >>> ");
  else Serial.print(" -+- ");
  Serial.println(receiver_input_throttle);
}

//void loop() {
//  // put your main code here, to run repeatedly:
////  delay(4);
//
////  esc_timer = micros() + 1500;
////  PORTD |= B00111100;
////  while(micros() < esc_timer);
////  PORTD &= B11000011;
//
//  
//  m1.write(receiver_input_channel_5);
//  print_signals();
//}
