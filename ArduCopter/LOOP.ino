void loop() {
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;

//  Serial.print("Ax: "); Serial.print(Ax);
//  Serial.print("\tAy: "); Serial.print(Ay);
//  Serial.print("\tAz: "); Serial.print(Az);
//  Serial.print("\tT: "); Serial.print(T);
//  Serial.print("\tGx: "); Serial.print(Gx);
//  Serial.print("\tGy: "); Serial.print(Gy);
//  Serial.print("\tGz: "); Serial.println(Gz);

  m1.writeMicroseconds(receiver_input_channel_5);
  m2.writeMicroseconds(receiver_input_channel_5);
  m3.writeMicroseconds(receiver_input_channel_5);
  m4.writeMicroseconds(receiver_input_channel_5);
  Serial.println(receiver_input_channel_5);

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
