#include "aprs.h"
aprs::aprs(String validCallsigns){ 
	_calls = validCallsigns;
}
String aprs::getCallsign(){
	return _callsign;
}
float aprs::getLatitude(){
   if(_latitude != NULL){
     return _latitude; 
   }
   String latDM = _aprsString.substring(_aprsString.indexOf(':') + 2, 
               _aprsString.substring(_aprsString.indexOf(':') + 2).indexOf('/') + (_aprsString.indexOf(':') + 2));
               
  //note: latDM now contains the lat in D.M format
  //convert latitude to degree decimal format
  
  String dir = latDM.substring(latDM.length()-1,latDM.length());
  if(dir == "S"){
    _latitude = -1 * (latDM.substring(0,2).toInt() + 
                  (latDM.substring(2,latDM.length()-1).toFloat()) / 60)
    ;
  }else{
    _latitude =      latDM.substring(0,2).toInt() + 
                  (latDM.substring(2,latDM.length()-1).toFloat()) / 60
    ;
  }
  return _latitude;
}
long aprs::getAltitude(){
  if(_alt != NULL){ //skip processing if we previously calculated the altutide
    return _alt;  
  }
  String altString = _aprsString.substring(_aprsString.indexOf("A=")+2);
  _alt = 0;
  char buffer[10];
  for(int i=0;i<altString.length();i++){
    if(altString[i]>=48 && altString[i]<=57){
      buffer[i] = altString[i];
      _alt = atol(buffer);
    }else{
      break;  
    }
  }
  return _alt;
}
float aprs::getLongitude(){
  if(_longitude != NULL){
    return _longitude;  
  }
  String lonDM = _aprsString.substring(_aprsString.substring(_aprsString.indexOf(':') + 2).indexOf('/') + (_aprsString.indexOf(':') + 2) + 1,
              _aprsString.substring(_aprsString.indexOf(':') + 2).indexOf('/') + (_aprsString.indexOf(':') + 2) + 10)
  ;
  //convert lonDM to decimal
  String dir2 = lonDM.substring(lonDM.length()-1,lonDM.length());
  if(dir2 == "W"){
    _longitude =  -1 * (lonDM.substring(0,3).toInt() + 
                  (lonDM.substring(3,lonDM.length()-1).toFloat()) / 60)
    ;
  }else{
    _longitude =  lonDM.substring(0,3).toInt() + 
                  (lonDM.substring(3,lonDM.length()-1).toFloat()) / 60
    ;
  }
  return _longitude;
}
boolean aprs::giveAprsString(String aprsString){
  _aprsString = aprsString;
  _callsign = _aprsString.substring(0,_aprsString.indexOf('-'));
  //TODO: check the callsigns before proceding
  return true;  

  //TODO: return false if input is not a valid APRS sentence
}
