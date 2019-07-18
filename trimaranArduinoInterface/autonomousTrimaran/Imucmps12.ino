

void setupImuCmps(){
  Serial1.begin(9600);
  nh.advertise(pubImuCmps);
  nh.advertise(pubEuler);

  /*
  Serial1.write(0xE0);
  while(Serial1.available()<1);
  Serial1.read();

  Serial1.write(0xE5);
  while(Serial1.available()<1);
  Serial1.read();

  Serial1.write(0xE2);
  while(Serial1.available()<1);
  Serial1.read();
  */
  
  /*Serial1.write(0xA1);
  while(Serial1.available()<1);
  Serial1.read();
  Serial.println("ooook");

  */
  
}

void updateImuCmps(){
  headingCmps = getheading();
  pitchCmps = getPitch();
  rollCmps = getRoll();

  getAccel();
  getGyro();
  publishImuCmps();
}


void publishImuCmps(){
  long temps = millis();
  imuCmpsMsg.header.stamp.sec = temps/1000;
  imuCmpsMsg.header.stamp.nsec = temps;
  
  
  imuCmpsMsg.linear_acceleration.x = axC;
  imuCmpsMsg.linear_acceleration.y = ayC;
  imuCmpsMsg.linear_acceleration.z = azC;
  
  imuCmpsMsg.angular_velocity.x = gxC;
  imuCmpsMsg.angular_velocity.y = gyC;
  imuCmpsMsg.angular_velocity.z = gzC;
  
  eulerMsg.x = headingCmps;
  eulerMsg.y = pitchCmps;
  eulerMsg.z = rollCmps;

  pubImuCmps.publish(&imuCmpsMsg);
  pubEuler.publish(&eulerMsg);
}




/***************************************************************************/

float getheading(){
  Serial1.write(CMPS_GET_ANGLE16);  // Request and read 16 bit angle
  while(Serial1.available() < 2);
  high_byte = Serial1.read();
  low_byte = Serial1.read();
  float val = float((high_byte<<8)+low_byte)/10;
  //Serial.println(val);
  val = refImu + PI*val/180;
  val = -2*atan(tan(val/2));
  return val;
}

// rend des valeurs coherentes entre -2.20 et 2.20 radian
float getPitch(){
  Serial1.write(CMPS_GET_PITCH);
  while(Serial1.available()<1);
  char buf = Serial1.read();
  return (2*PI*float(buf)/360);
}

// rend des valeurs coherentes entre -pi et pi
float getRoll(){
  Serial1.write(CMPS_GET_ROLL);
  while(Serial1.available()<1);
  char buf = Serial1.read();
  return (2*PI*float(buf)/360);
}

void getAccel(){
  Serial1.write(CMPS_GET_ACCEL);
  while(Serial1.available()<6);
  x_high_byte = Serial1.read();
  x_low_byte = Serial1.read();
  y_high_byte = Serial1.read();
  y_low_byte = Serial1.read();
  z_high_byte = Serial1.read();
  z_low_byte = Serial1.read();

  axC =  float(int16_t(x_high_byte<<8) +int16_t(x_low_byte))/100;
  ayC =  float(int16_t(y_high_byte<<8) +int16_t(y_low_byte))/100;
  azC =  float(int16_t(z_high_byte<<8) +int16_t(z_low_byte))/100;
  
}

void getGyro(){
  Serial1.write(CMPS_GET_GYRO);
  while(Serial1.available()<6);
  x_high_byte = Serial1.read();
  x_low_byte = Serial1.read();
  y_high_byte = Serial1.read();
  y_low_byte = Serial1.read();
  z_high_byte = Serial1.read();
  z_low_byte = Serial1.read();

  gxC =  float(int16_t(x_high_byte<<8) +int16_t(x_low_byte));
  gyC =  float(int16_t(y_high_byte<<8) +int16_t(y_low_byte));
  gzC =  float(int16_t(z_high_byte<<8) +int16_t(z_low_byte));

  gxC = PI*gxC/180;
  gyC = PI*gyC/180;
  gzC = PI*gzC/180;
}
