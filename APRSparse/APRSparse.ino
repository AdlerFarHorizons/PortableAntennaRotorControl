// APRS parse
// Prints parsed info to Serial. Callsign, gps lat/lon in decimal form, and altitude are parsed. - KN

//our input aprs string is here:
String in = "WB9SKY-11>APBL10,N9IO*,WIDE2-1:!4106.07N/08755.13WO317/025/A=005243Adler High Altitude Balloon";

void setup() {
  Serial.begin(115200);
  Serial.println(in);
  Serial.println("=======> Parsed:");
}

void loop() {
  //GET CALLSIGN======================================
  String callsign = in.substring(0,in.indexOf('-'));
  Serial.print("Callsign: \t\t");
  Serial.println(callsign);
  //END CALLSIGN======================================

  //GET LATITUDE======================================
  String latDM = in.substring(in.indexOf(':') + 2, 
               in.substring(in.indexOf(':') + 2).indexOf('/') + (in.indexOf(':') + 2));
  Serial.print("latitude DDMM.MM: \t");
  Serial.println(latDM);
  
  //convert latitude to degree decimal format
  String dir = latDM.substring(latDM.length()-1,latDM.length());
  float latDEG;
  if(dir == "S"){
    latDEG = -1 * (latDM.substring(0,2).toInt() + 
                  (latDM.substring(2,latDM.length()-1).toFloat()) / 60)
    ;
  }else{
    latDEG =      latDM.substring(0,2).toInt() + 
                  (latDM.substring(2,latDM.length()-1).toFloat()) / 60
    ;
  }
  Serial.print("latitude decimal: \t");
  Serial.println(latDEG);
  //END LATIUDE

  //GET LOGITUDE================================
  String lonDM = in.substring(in.substring(in.indexOf(':') + 2).indexOf('/') + (in.indexOf(':') + 2) + 1,
              in.substring(in.indexOf(':') + 2).indexOf('/') + (in.indexOf(':') + 2) + 10)
  ;
  Serial.print("longitude DDMM.MM: \t");
  Serial.println(lonDM);

  //convert lonDM to decimal
  String dir2 = lonDM.substring(lonDM.length()-1,lonDM.length());
  float lonDEG;
  if(dir2 == "W"){
    lonDEG = -1 * (lonDM.substring(0,3).toInt() + 
                  (lonDM.substring(3,lonDM.length()-1).toFloat()) / 60)
    ;
  }else{
    lonDEG =      lonDM.substring(0,3).toInt() + 
                  (lonDM.substring(3,lonDM.length()-1).toFloat()) / 60
    ;
  }
  Serial.print("longitude decimal: \t");
  Serial.println(lonDEG);
  //END LONGITUDE

  //GET ALTITUDE================================
  String altString = in.substring(in.indexOf("A=")+2);
  int  alt = 0;
  for(int i=0;i<altString.length();i++){
    if(altString[i]>=48 && altString[i]<=57){
      alt = altString.substring(0,i+1).toInt();
    }
  }
  Serial.print("Altitude + extra info:\t");
  Serial.println(altString);
  Serial.print("Altitude: \t\t");
  Serial.println(alt);
  //END ALTITUDE================================
  
  //NOTES===================================================
  delay(1000000);
  // E/N + 
  // S/W -
  //lat is N/S
}
