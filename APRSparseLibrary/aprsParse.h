#include "Arduino.h"
#ifndef aprsParse_h
#define aprsParse_h
class aprsParse{
	public:
		aprs(); // to add the callsign checks as arguments
		String getCallsign();
		String getAltitude();
		String getDecLatitude();
		String getDecLongitude():
		void giveAprs();
		
	private:
		String _inputString;
		
}
#endif
