#include "aprsParse.h"
void setup(){
  Serial.begin(115200);
}
;
myAprs = aprsParse();
myAprs.giveAprs("WB9SKY-11>APBL10,N9IO*,WIDE2-1:!4106.07N/08755.13WO317/025/A=005243Adler High Altitude Balloon");
void loop(){
  Serial.println(myAprs.getCallsign());
  delay(5000);  
}
