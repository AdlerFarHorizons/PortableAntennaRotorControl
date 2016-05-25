/*
 * This program is for Teensy 3
 */

#include <TimeLib.h>

#define GPSLEN 100 //max length of GPS output to be sent
#define APRSLEN 100 // max length of APRS message packet
#define TERMLEN 100 // max length of terminal message
#define CMDLEN TERMLEN
#define APRSFILEN TERMLEN
#define DEBUG true


const int elDrvPin = 16;
const int elSgnPin = 17;
const int azDrvPin = 18;
const int azSgnPin = 19;
const int ppsPin = 6;
const int updatePer = 1000;
const int eepromAddrStart = 256;
const int syncTimeOffset = 0;
const float xtalTol = 20e-6;
 * Copernicus II configured for two NMEA sentences every 60 seconds:
 *  ZDA:Provides nothing until first fix, then provides full date
 *      based on internal clock until reset. Clock synced to true
 *      UTC at prior PPS output sometime after a fully valid 2D fix.
 *  TF: Provides a complete 3D position and velocity fix but no
 *      date/time. Also indicates whether time at last PPS is true
 *      UTC.
 */
const String gpsConfStr = "$PTNLSNM,0220,60*50\r\n";
String callSgn1 = "KC9LHW-11";
String callSgn2 = "KC9LIG-11";
String callSgn3 = "WB9SKY-11";
String cmdStr = "cmd:";

char gps[GPSLEN], gpsBuf[GPSLEN], gpsZDA[GPSLEN], gpsTF[GPSLEN]; 
int gpsIndex = 0;
boolean gpsRdy, gpsZDAFlg, gpsTFFlg, gpsChkFlg, gpsErrFlg;
boolean gpsPosValid, gpsMsgFlg, gpsTimeFlg, gpsTimeValid;
byte gpsChk;
float lat, lon, alt;
long syncTime;

char aprs[APRSLEN], aprsBuf[APRSLEN];
int aprsIndex = 0;
boolean aprsRdy, aprsChkFlg, aprsErrFlg, aprsBufCurrent, aprsValid;

char term[TERMLEN], aprsfiBuf[APRSFILEN], cmdBuf[CMDLEN];
int termIndex = 0;
boolean termRdy, termChkFlg, termErrFlg, termBufCurrent, termValid;
boolean aprsfiPending, cmdPending, aprsfiBufCurrent, cmdBufCurrent;
boolean cmdRdy, aprsfiRdy;
boolean updateFlg;

int gpsYr, gpsMon, gpsDay, gpsHr, gpsMin, gpsSec;

IntervalTimer updateTimer;

void setup() {
  pinMode( elDrvPin, OUTPUT );
  pinMode( elSgnPin, OUTPUT );
  pinMode( azDrvPin, OUTPUT );
  pinMode( azSgnPin, OUTPUT );
  pinMode( ppsPin, INPUT );
  gpsMsgFlg = false;
  gpsRdy = true;
  updateFlg = false;
  cmdRdy = false;
  cmdBufCurrent = true;
  setSyncProvider( getTeensy3Time );
  Serial.begin( 9600 ); // Monitor and aprs.fi packets
  Serial1.begin( 9600 ); // APRS
  Serial2.begin( 9600 ); // GPS
  while( !Serial || !Serial1 || !Serial2 );
  delay( 100 );
  //Send GPS Configuration messages
  digitalClockDisplay();
  delay( 1000 ); //Wait for GPS to power up.
  // Clear out GPS receive buffer
  while( Serial2.available() ) Serial2.read();
  gpsConfig();
  delay( 1000 ); 
  while( Serial2.available() ) {
    Serial.write( Serial2.read() ); 
  }
  Serial.println( "" );
  
  attachInterrupt( ppsPin, ppsSvc, RISING );
}

void loop() {

  if ( gpsMsgFlg ) updateGPSMsg();
  if ( gpsZDAFlg ) procZDAMsg();
  if ( gpsTFFlg ) procTFMsg();
  if ( !cmdBufCurrent ) updateCmdBuffer();
  if ( Serial.available() ) getTermByte();
  if ( Serial1.available() ) getAPRSByte();
  if ( Serial2.available() ) getGPSByte();

//  if ( !aprsBufCurrent ) updateAPRSBuffer();
//  if ( !aprsfiBufCurrent ) updateAPRSfiBuffer();
//  if ( cmdBufCurrent ) {
//    cmdBufCurrent = false;
//    Serial.println( "Command" );
//  }
//  if ( termRdy && aprsRdy ) {
//    if ( aprsfiPending ) {
//      aprsfiPending = false;
//      Serial.println( "processing aprsfi" );
//    }
//    termRdy = false;
//  }
//  if ( updateFlg ) setRotors();
//  if ( gpsRdy ) updateLocal();
//  if ( aprsRdy ) updateRemote();
  
}

void getGPSByte() {
  char c = Serial2.read();
  if ( c == '$' ) { // '$', start of GPS statement - KN
    gpsIndex = 0;
    gpsRdy = false;
    gpsChk = 0;
    if ( DEBUG ) Serial.println( "GPS msg started..." );
  }
  if ( c == '*' ) { // '*' indicates end of GPS sentence
    gpsChkFlg = true;
    if ( DEBUG ) {
      if ( DEBUG ) Serial.println( gps );
    }
  }
  if ( !gpsRdy ) {
    if ( gpsChkFlg ) {
      gpsErrFlg = gpsChk != c;
      gpsChkFlg = false;
      gpsRdy = true;
      gpsMsgFlg = true;
    } else {
      gps[gpsIndex] = c;
      gpsChk ^= (byte)c;
    }
    gpsIndex++;
    gps[gpsIndex] = 0;
  }
}

void updateGPSMsg() {
  String str = String( gps );
  int i;
  
  if ( str.substring( 3, 6 ) == "ZDA" ) {
    for ( i = 0; i < GPSLEN ; i++ ) {
      gpsZDA[i] = gps[i];
    }
    gpsZDAFlg = true;
  }

  if ( str.substring( 5, 8 ) == "RTF" ) {
    for ( i = 0; i < GPSLEN ; i++ ) {
      gpsTF[i] = gps[i];
    }
    gpsTFFlg = true;
  }

  for ( int i = 0 ; i < GPSLEN ; i++ ) {
      gpsBuf[i] = gps[i];
  }
  if ( DEBUG ) Serial.println( "GPS Buffer updated." );
  gpsMsgFlg = false;
  if ( DEBUG ) {
    Serial.print( "GPS Flags:" );Serial.print( gpsTFFlg );
    Serial.println( gpsZDAFlg );
  }
}

void procZDAMsg() {
    String utc = getField( gpsZDA, 1, ',' );
    gpsHr = utc.substring( 0,2 ).toInt();
    gpsMin = utc.substring( 2,4 ).toInt();
    gpsSec = utc.substring( 4,6 ).toInt();
    gpsDay = getField( gpsZDA, 2, ',' ).toInt();
    gpsMon = getField( gpsZDA, 3, ',' ).toInt();
    gpsYr = getField( gpsZDA, 4, ',' ).toInt();
    gpsZDAFlg = false;
    gpsTimeFlg = true;
  }

 void procTFMsg() {
    // Need UTC offset with at least 2D fix for valid time sync
    gpsTimeValid = (boolean)getField( gpsTF, 15, ',' ).toInt() &&
                   getField( gpsTF, 5, ',' ).toInt() >= 2;
    // Need 3D position fix for proper pointing.
    gpsPosValid = getField( gpsTF, 5, ',' ) == '3';
    if ( DEBUG ) {
      Serial.print( "gpsPosValid:" );Serial.println( gpsPosValid );
      Serial.print( "gpsTimeValid:" );Serial.println( gpsTimeValid );
    }
    if ( gpsPosValid ) {
      // Get position, velocity data
      lat = getField( gpsTF, 6, ',' ).toFloat();
      if ( getField( gpsTF, 7, ',' ) == 'S' ) lat *= -1.0;
      lon = getField( gpsTF, 8, ',' ).toFloat();
      if ( getField( gpsTF, 9, ',' ) == 'W' ) lon *= -1.0;
      alt = getField( gpsTF, 10, ',' ).toFloat();
    }
    gpsTFFlg = false;
  }

String getField( String str, int fieldIndex, char delim ) {
  int startIndex = -1;
  int endIndex = 0;
  for ( int i = 0 ; i <= fieldIndex ; i++ ) {
    startIndex = endIndex + 1;
    endIndex = str.indexOf( delim, startIndex );
  }
  if ( endIndex == -1 ) endIndex = str.length();
  return str.substring( startIndex, endIndex );
}

void gpsConfig() {
  Serial2.print( gpsConfStr );
}

void getAPRSByte() {
  char c = Serial1.read();
  if ( ( c == '\n' || c == '\r' ) ) { // End of APRS msg
    if ( !aprsRdy ) {
      aprsIndex = 0;
      aprsRdy = true;
      aprsBufCurrent = false;
      if ( DEBUG ) Serial.println( "APRS msg ended." );
      if ( DEBUG ) Serial.println( aprs );
    }
  } else {
    if ( DEBUG && aprsRdy ) Serial.println( "APRS msg started..." );
    aprs[aprsIndex] = c;
    aprsRdy = false;
    aprsIndex++;
    aprs[aprsIndex] = 0;
  }
}

void updateAPRSBuffer() {
    for ( int i = 0 ; i < APRSLEN ; i++ ) {
        aprsBuf[i] = aprs[i];
    }
    aprsBufCurrent = true;
    if ( DEBUG ) Serial.println( "APRS Buffer updated." );
}

void getTermByte() {
  char c = Serial.read();
  if ( ( c == '\n' || c == '\r' ) && !termRdy ) { // End of term msg
    termIndex = 0;
    termRdy = true;
    termBufCurrent = false;
    if ( aprsfiPending ) {
      aprsfiRdy = true;
      aprsfiPending = false;
    }
    if ( cmdPending ) {
      cmdRdy = true;
      cmdPending = false;
    }
    if ( DEBUG ) Serial.println( "term msg ended." );
  } else {
    if ( DEBUG && termRdy ) Serial.println( "term msg started..." );
    term[termIndex] = c;
    termRdy = false;
    termIndex++;
    term[termIndex] = 0;
    if ( !aprsfiPending && !cmdPending ) {
      String temp = (String)term;
      if ( temp == callSgn1 || temp == callSgn2 || temp == callSgn3 ) {
        aprsfiPending = true;
        if ( DEBUG ) Serial.println( "aprsfiPending" );
      }
      if ( temp == cmdStr ) {
        cmdPending = true;
        if ( DEBUG ) Serial.println( "cmdPending" );
      }
    }
  }
}

void updateAPRSfiBuffer() {
    for ( int i = 0 ; i < APRSFILEN ; i++ ) {
        aprsfiBuf[i] = term[i];
    }
    aprsfiBufCurrent = true;
    if ( DEBUG ) Serial.println( "APRS.fi Buffer updated." );
}

void updateCmdBuffer() {
    for ( int i = 0 ; i < CMDLEN ; i++ ) {
        cmdBuf[i] = term[i];
    }
    cmdBufCurrent = true;
    if ( DEBUG ) Serial.println( "cmd Buffer updated." );
}

void setRotors() {
  
}

void updateLocal() {
  
}

void updateRemote() {
  
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void ppsSvc() {
  if ( DEBUG ) {
    Serial.print( "." );
  }

  // Set clock(s) to UTC time if a valid fix came in since the last PPS
  if ( gpsTimeValid && gpsTimeFlg ) {
    setTime( gpsHr,gpsMin,gpsSec,gpsDay,gpsMon,gpsYr ); // "Library" time
    setTime( now() + 1 );
    Teensy3Clock.set( now() ); // Teensy RTC
    if ( DEBUG ) {
      Serial.print( "Time Corrected:" );
      digitalClockDisplay();
    }
  }

  // Reset GPS flags
  gpsPosValid = false;
  gpsTimeValid = false;
  gpsTimeFlg = false;
  gpsZDAFlg = false;
  gpsTFFlg = false;
}



