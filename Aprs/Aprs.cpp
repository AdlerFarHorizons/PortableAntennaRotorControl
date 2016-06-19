#include "Aprs.h"
//===========================================================
Aprs::Aprs(String validCallsigns){ 
	_calls = validCallsigns;
}
//===========================================================
String Aprs::getCallsign(){
	return _callsign;
}
//============================================================
float Aprs::getLatitude(){
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
//============================================================
long Aprs::getAltitude(){
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
//============================================================
float Aprs::getLongitude(){
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

//==========================================================
int Aprs::getGroundspeed(){
  if(_groundspeed != NULL){
    return _groundspeed;  
  }
  char searcher[7];

  //get groups of 7
  for(int i = _aprsString.indexOf(':'); i < _aprsString.length()-5; i++){
    for(int j = 0; j < 7; j++){
      searcher[j] = _aprsString[i+j];
    }

    _course = 0;
    // the brute force versionto check if that group matches our ###/### pattern
    //ASCII 48-57
    boolean flag = true;
    flag = flag && (searcher[0] < 58 && searcher[0] > 47);
    flag = flag && (searcher[1] < 58 && searcher[1] > 47);
    flag = flag && (searcher[2] < 58 && searcher[2] > 47);
    flag = flag && (searcher[3] == '/');
    flag = flag && (searcher[4] < 58 && searcher[4] > 47);
    flag = flag && (searcher[5] < 58 && searcher[5] > 47);
    flag = flag && (searcher[6] < 58 && searcher[6] > 47);
    if(flag){
      return ((String) searcher).substring(0,3).toInt();  //yea... we're assuming that this must happen. Bad code.
    }
  }
}

//GET COURSE (HEADING)========================================
int Aprs::getCourse(){
  if(_course != NULL){
    return _course;  
  }
  char searcher[7];
  for(int i = _aprsString.indexOf(':'); i < _aprsString.length()-5; i++){
    for(int j = 0; j < 7; j++){
      searcher[j] = _aprsString[i+j];
    }


    for(int k = 0; k<7;k++){
      if( k < 3 && (searcher[k] >=57 || searcher[k] <= 48)){
          break;
      }
      if(k==3 && searcher[k] !=47){
          break;
      }
      if(k>3 && (searcher[k] >= 57 || searcher[k] <=48)){
        String returner = (String) searcher[0];
        returner += (String) searcher[1];
        returner += (String) searcher[2];
        _course = returner.toInt();
        return _course;
      }
    }
  }
}
//============================================================
boolean Aprs::giveAprsString(String aprsString){
  _aprsString = aprsString;
  _callsign = _aprsString.substring(0,_aprsString.indexOf('-'));
  //TODO: check the callsigns before proceding
  return true;  

  //TODO: return false if input is not a valid APRS sentence
}
