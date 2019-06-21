
void setupImu(){
 accelgyro.initialize();
 nh.advertise(pubImu);
 Serial.println("Testing device connections...");
 Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 delay(1000);
}


void updateImu(){
  getAccelGyroCompass_Data();
  publishImu();           
}


void publishImu(){
  //nh.loginfo("envoie imu ...");
  imuMsgs.linear_acceleration.x = Axyz[0];
  imuMsgs.linear_acceleration.y = Axyz[1];
  imuMsgs.linear_acceleration.z = Axyz[2];
  imuMsgs.angular_velocity.x = Gxyz[0];
  imuMsgs.angular_velocity.y = Gxyz[1];
  imuMsgs.angular_velocity.z = Gxyz[2];
  imuMsgs.orientation.x = Mxyz[0];
  imuMsgs.orientation.y = Mxyz[1];
  imuMsgs.orientation.z = Mxyz[2];
  pubImu.publish(&imuMsgs);
  
}



/********************************************************************************************/
/* Fonctions pour le fonctionnement de l'imu */



void getAccelGyroCompass_Data(void){
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
  
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
  // degres/sec -> rad/sec
  Gxyz[0] = Gxyz[0] * PI/180;
  Gxyz[1] = Gxyz[0] * PI/180;
  Gxyz[2] = Gxyz[0] * PI/180;
  
  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
  // calibration
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;  
  
}
