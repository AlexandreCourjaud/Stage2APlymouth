// Define Variables:
//Constant variables relating to pin locations
const int chA=8;
const int chB=2;
const int chC=3;
const int chD=4;
const int chE=5;
const int chF=6;
const int chG=7;
const int chH=36;

//Varibles to store and display the values of each channel
int ch1;
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;
int ch7;
int ch8;


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  // Set input pins
  pinMode(chA, INPUT);
  pinMode(chB,INPUT);
  pinMode(chC,INPUT);
  pinMode(chD,INPUT);
  pinMode(chE,INPUT);
  pinMode(chF,INPUT);
  pinMode(chG,INPUT);
  pinMode(chH,INPUT);
}

//Main Program
void loop() {
  // read the input channels
  ch1 = pulseIn (chA,HIGH);
  Serial.print ("Ch1:");
  Serial.print (ch1);
  Serial.print ("|");

  ch2 = pulseIn (chB,HIGH);
  Serial.print ("Ch2:");
  Serial.print (ch2);
  Serial.print ("|");
 
  ch3 = pulseIn (chC,HIGH);
  Serial.print ("Ch3:");
  Serial.print (ch3);
  Serial.print ("|");
 
  ch4 = pulseIn (chD,HIGH);
  Serial.print ("Ch4:");
  Serial.print (ch4);
  Serial.print ("|");
 
  ch5 = pulseIn (chE,HIGH);
  Serial.print ("Ch5:");
  Serial.print (ch5);
  Serial.print ("|");
 
  ch6 = pulseIn (chF,HIGH);
  Serial.print ("Ch6:");
  Serial.print (ch6);
  Serial.print ("|");

  ch7 = pulseIn (chG,HIGH);
  Serial.print ("Ch7:");
  Serial.print (ch7);
  Serial.print ("|");

  Serial.println(" ");

}
 
