#include "aprs.h"
aprs::aprs(String validCallsigns){ 
	_calls = validCallsigns;
}
String aprs::getCallsign(){
	return _callsign;
}
boolean aprs::giveAprsString(String aprsString){
  _aprsString = aprsString;
  _callsign = _aprsString.substring(0,_aprsString.indexOf('-'));
  return true;  

  //TODO: return false if input is not a valid APRS sentence
}
