/*****************************************
*   CMPS12 Serial example for Arduino    *
*        By James Henderson, 2014        * 
*****************************************/

#include <SoftwareSerial.h>

#define CMPS_GET_ANGLE16 0x13
#define CMPS_GET_PITCH 0x14
#define CMPS_GET_ROLL 0x15
#define CMPS_GET_ACCEL  0x20
#define CMPS_GET_GYRO 0x21


//SoftwareSerial CMPS12 = SoftwareSerial(12,13);

unsigned char high_byte, low_byte;
float pitch, roll, heading;
unsigned int angle16;


unsigned char x_high_byte, x_low_byte;
unsigned char y_high_byte,y_low_byte;
unsigned char z_high_byte,z_low_byte;
float ax,ay,az;
float gx,gy,gz;
float x,y,z,w;

void setup()
{
  Serial.begin(9600);              // Start serial ports
  //Serial1.begin(9600);
  Serial1.begin(9600);
  
}

void loop()
{
  
  /*
  heading = getheading();
  Serial.print("heading : ");
  Serial.print(heading);
 
  pitch = getPitch();
  Serial.print("   Pitch : ");
  Serial.print(pitch);
  
  roll = getRoll();
  Serial.print("   Roll : ");
  Serial.println(float(roll));
  */
  
  getAccel();
  Serial.print("accel: x: ");
  Serial.print(ax);
  Serial.print("  y : ");
  Serial.print(ay);
  Serial.print("  z : ");
  Serial.println(az);
  
  
  /*
  getGyro();
  Serial.print("gyro: x: ");
  Serial.print(gx);
  Serial.print("    y : ");
  Serial.print(gy);
  Serial.print("    z : ");
  Serial.println(gz);
  */
  delay(100);                           // Short delay before next loop
}


float getheading(){
  Serial1.write(CMPS_GET_ANGLE16);  // Request and read 16 bit angle
  while(Serial1.available() < 2);
  high_byte = Serial1.read();
  low_byte = Serial1.read();
  float val = float((high_byte<<8)+low_byte)/10;
  val = PI/2+PI*val/180;
  val = 2*atan(tan(val/2));
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

  ax =  float(int16_t(x_high_byte<<8) +int16_t(x_low_byte))/100;
  ay =  float(int16_t(y_high_byte<<8) +int16_t(y_low_byte))/100;
  az =  float(int16_t(z_high_byte<<8) +int16_t(z_low_byte))/100;
  
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

  gx =  float(int16_t(x_high_byte<<8) +int16_t(x_low_byte));
  gy =  float(int16_t(y_high_byte<<8) +int16_t(y_low_byte));
  gz =  float(int16_t(z_high_byte<<8) +int16_t(z_low_byte));

  gx = PI*gx/180;
  gy = PI*gy/180;
  gz = PI*gz/180;
}

void ToQuaternion(double yaw, double pitch, double roll){
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    w = cy * cp * cr + sy * sp * sr;
    x = cy * cp * sr - sy * sp * cr;
    y = sy * cp * sr + cy * sp * cr;
    z = sy * cp * cr - cy * sp * sr;
}


void ToEulerAngle(double x, double y, double z, double w){
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (w * x + y * z);
  double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (w * y - z * x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (w * z + x * y);
  double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);  
  heading = atan2(siny_cosp, cosy_cosp);
}
