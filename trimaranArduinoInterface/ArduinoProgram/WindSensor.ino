
void setupWind(){
  pinMode(PIN_WIND,INPUT);
  nh.advertise(pubWind);
}


void updateWind(){
  sensorValue = analogRead(A1);
  angleWind = ((sensorValue-ref-MIN_WIND)/(MAX_WIND-MIN_WIND))*2*PI;
  angleWind = atan(tan(angleWind/2));
  publishWind();
}

void publishWind(){
  windMsg.data = angleWind;
  pubWind.publish(&windMsg);
}
