#include "aprs.h"
void setup(){
  Serial.begin(115200);
}
aprs myAprs = aprs("WB9SKY,KC9LIG,KC9LHW"); //TODO: callsign matching
void loop(){
  myAprs.giveAprsString("WB9SKY-11>APBL10,N9IO*,WIDE2-1:!4106.07N/08755.13WO317/025/A=125243Adler High Altitude Balloon");

  //getCallsign: returns a String
  Serial.println(myAprs.getCallsign());

  //getLatitude: returns latitidue in decimal form with N as a positive float and S as a negative float
  Serial.println(myAprs.getLatitude());

  //getLongitude: returns longitude in decimal form with W as a positive float and E as a negative float
  Serial.println(myAprs.getLongitude());

  //getAltitude: returns the altitude as a long
  Serial.println(myAprs.getAltitude());
  delay(5000);  
}
