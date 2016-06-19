#include "Aprs.h"
void setup(){
  Serial.begin(115200);
}
Aprs myAprs = Aprs("WB9SKY,KC9LIG,KC9LHW"); //TODO: callsign matching
void loop(){
  myAprs.putAprsString("WB9SKY-11>APBL10,N9IO*,WIDE2-1:!4106.07N/08755.13WO317/125/A=125243Adler High Altitude Balloon");

  //getCallsign: returns a String
  Serial.print("Callsign: ");
  Serial.println(myAprs.getCallsign());

  //getLatitude: returns latitidue in decimal form with N as a positive float and S as a negative float
  Serial.print("Latitude: ");
  Serial.println(myAprs.getLatitude());

  //getLongitude: returns longitude in decimal form with W as a positive float and E as a negative float
  Serial.print("Longitude: ");
  Serial.println(myAprs.getLongitude());

  //getAltitude: returns the altitude as a long
  Serial.print("Altitude: ");
  Serial.println(myAprs.getAltitude());

  //getGroundspeed: returns the ground speed as an int
  Serial.print("Ground speed: ");
  Serial.println(myAprs.getGroundspeed());

  //getCourse: returns the heading ("Course in the APRS docs") as an int
  Serial.print("Course: ");
  Serial.println(myAprs.getCourse());

  delay(5000);  
}
