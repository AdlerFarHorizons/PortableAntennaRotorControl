#include "aprs.h"
aprs::aprs(){

}
aprs::getAltitude(){
	
}
aprs::getCallsign(){
	String callsign = _in.substring(0,_in.indexOf('-'));
	return callsign;
}
aprs::getDecLatitude(){

}
aprs::getDecLongitude(){

}
aprs::giveAprs(String aprsInput){
	_in =  aprsInput;
}
