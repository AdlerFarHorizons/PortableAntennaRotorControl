/*
 * This program is for Teensy 3
 * Requires Arduino 1.6.8+ and compatible Teensyduino
 */

#include <TimeLib.h>
#include <EEPROM.h>
#include "Aprs/Aprs.h"
#define GPSLEN 100 //max length of GPS output to be sent
#define APRSLEN 100 // max length of APRS message packet
#define TERMLEN 100 // max length of terminal message
#define CMDLEN TERMLEN
#define APRSFILEN TERMLEN
#define DEBUG true
#define TEST true


const int elDrvPin = 16;
const int elSgnPin = 17;
const int azDrvPin = 18;
const int azSgnPin = 19;
const int ppsPin = 6;
const float xtalTol = 20e-6;
const float maxTimeTol = 0.5; // seconds
const long maxTimeSyncAge = (long)( 0.5 + maxTimeTol / xtalTol );
const long rotorUpdateInterval = 200000;
const float deg2rad = 3.1415927 / 180.0;
const float rEarth = 6.371e6; // meters
const float deg2m = 1.124E5; // meters/deg on great circle at msl
const float m2rad = 1.5696E-7; // radians/m on great circle at msl
const float knots2mps = 0.51444444; // m/s
const float ft2m = 0.3048; //  m
/*
 * Set addresses of EEPROM parameters 
 */
const int eeaddrStart = 256;
const int eeaddrLastGPSTimeSync = eeaddrStart; // time_t
const int eeaddrAzPotEast = eeaddrLastGPSTimeSync + sizeof( time_t );
const int eeaddrAzPotWest = eeaddrAzPotEast + sizeof( int );
const int eeaddrAzPotMin = eeaddrAzPotWest + sizeof( int );
const int eeaddrAzPotMax = eeaddrAzPotMin + sizeof( int );
const int eeaddrElPot0 = eeaddrAzPotMax + sizeof( int );
const int eeaddrElPot90 = eeaddrElPot0 + sizeof( int );

/*
 * Parameters to use with TEST enabled
 */
const float testLat = 41.09 * deg2rad;
const float testLon = -87.9167100 * deg2rad;
const float testAlt = 434.3;

 /*
 * Copernicus II configured for two NMEA sentences every 60 seconds:
 *  ZDA:Provides nothing until first fix, then provides full date
 *      based on internal clock until reset. Clock synced to true
 *      UTC at prior PPS output sometime after a fully valid 2D fix.
 *  TF: Provides a complete 3D position and velocity fix but no
 *      date/time. Also indicates whether time at last PPS is true
 *      UTC.
 */
 
const String gpsConfStr = "$PTNLSNM,0220,60*50\r\n";

const char* callSgns[] = { "KC9LHW", "KC9LIG", "WB9SKY" };
int numCallSgns = sizeof( callSgns ) / sizeof( callSgns[0] );
String cmdStr = "cmd:";

char gps[GPSLEN], gpsBuf[GPSLEN], gpsZDA[GPSLEN], gpsTF[GPSLEN]; 
int gpsIndex = 0;
boolean gpsRdy, gpsZDAFlg, gpsTFFlg, gpsChkFlg, gpsErrFlg;
boolean gpsPosValid, gpsMsgFlg, gpsTimeFlg, gpsTimeValid;
byte gpsChk;
float gpsLat, gpsLon, gpsAlt;
int gpsYr, gpsMon, gpsDay, gpsHr, gpsMin, gpsSec;

char aprs[APRSLEN], aprsBuf[APRSLEN];
int aprsIndex = 0;
boolean aprsRdy, aprsChkFlg, aprsErrFlg, aprsBufCurrent, aprsValid;
boolean aprsFlg, aprsNewFlg;
float aprsLat, aprsLon, aprsVg, aprsHdg;
double aprsAlt;

// Derived aprs parameters
double aprsTime, lastAprsTime;
float aprsVlon, aprsVlat, aprsValt; // Derived from aprsVg, aprsHdg 
double lastAprsAlt = -1; // needed to keep track of prior value to calculate Vz

char term[TERMLEN], aprsfiBuf[APRSFILEN], cmdBuf[CMDLEN];
int termIndex = 0;
boolean termRdy, termChkFlg, termErrFlg, termBufCurrent, termValid;
boolean aprsfiPending, cmdPending, aprsfiBufCurrent, cmdBufCurrent;
boolean cmdRdy, aprsfiRdy;
float apfiLat, apfiLon, apfiAlt, apfiVg, apfiHdg;

boolean targetValid;

long  targetTime; 
float targetLon; //lon radians
float targetLat; //lat radians
double targetAlt; //alt meters
float targetVlon; // rad/s
float targetVlat; // rad/s
float targetValt; // m/s

float targetAz;
float targetEl;

Aprs aprsObj = Aprs( "WB9SKY,KC9LIG,KC9LHW" );

long ppsMillis;

int clockStatus;

IntervalTimer rotorDriveUpdate;

void setup() {
  pinMode( elDrvPin, OUTPUT );
  pinMode( elSgnPin, OUTPUT );
  pinMode( azDrvPin, OUTPUT );
  pinMode( azSgnPin, OUTPUT );
  pinMode( ppsPin, INPUT );
  gpsMsgFlg = false;
  gpsRdy = true;
  aprsBufCurrent = true;
  cmdRdy = false;
  cmdBufCurrent = true;
  setSyncProvider( getTeensy3Time );
  delay( 5000 ); //Wait for GPS to power up.
  Serial.begin( 115200 ); // Monitor and aprs.fi packets
  Serial1.begin( 9600 ); // APRS
  Serial2.begin( 9600 ); // GPS
  delay( 100 ); // Give time for serial TX line to bias up
 // Clear out serial receive buffers
  while( Serial.available() ) Serial.read();
  while( Serial1.available() ) Serial1.read();
  while( Serial2.available() ) Serial2.read();
  // Send GPS Configuration messages
  gpsConfig();
  // Get GPS response
  delay( 200 );
  Serial.println( "waiting for GPS response..." );
  while( Serial2.available() ) {
    Serial.write( Serial2.read() ); 
  }
  Serial.println( "got it." );

  /*
   * Sync library time to RTC. Default is 5 minutes.
   * This is too long.
   */
  setSyncInterval( 10 );
  
  attachInterrupt( ppsPin, ppsSvc, RISING );
//  rotorDriveUpdate.priority( 64 ); // Default is 128, lower is higher
//  rotorDriveUpdate.begin( rotorSvc, rotorUpdateInterval );

  if ( TEST ) {
    gpsLat = testLat;
    gpsLon = testLon;
    gpsAlt = testAlt;
  }
}

void loop() {

  if ( gpsMsgFlg ) updateGPSMsg();
  if ( gpsZDAFlg ) procZDAMsg();
  if ( gpsTFFlg ) procTFMsg();
//  if ( !cmdBufCurrent ) updateCmdBuffer();
//  if ( Serial.available() ) getTermByte();
  if ( Serial1.available() ) getAPRSByte();
  if ( Serial2.available() ) getGPSByte();
  if ( !aprsBufCurrent ) updateAPRSBuffer();
  if ( aprsFlg ) {
    if ( DEBUG ) Serial.print( "APRS msg rcvd" );
    aprsFlg = false;
    procAprs( );
  }
//  if ( !aprsfiBufCurrent ) updateAPRSfiBuffer();
//  if ( cmdBufCurrent ) {
//    cmdBufCurrent = false;
//    Serial.println( "Command" );
//  }
}

void getGPSByte() {
  char c = Serial2.read();
  if ( c == '$' ) { // '$', start of GPS statement - KN
    gpsIndex = 0;
    gpsRdy = false;
    gpsChk = 0;
    if ( DEBUG ) Serial.println( "\nGPS msg started..." );
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
  Serial.println( "GPS msg rcvd" );
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
  digitalClockDisplay(now());
}

void procZDAMsg() {
  Serial.println( gpsZDA );
  String utc = getField( gpsZDA, 1, ',' );
  if ( utc.length() > 0 ) {
    gpsHr = utc.substring( 0,2 ).toInt();
    gpsMin = utc.substring( 2,4 ).toInt();
    gpsSec = utc.substring( 4,6 ).toInt();
    gpsDay = getField( gpsZDA, 2, ',' ).toInt();
    gpsMon = getField( gpsZDA, 3, ',' ).toInt();
    gpsYr = getField( gpsZDA, 4, ',' ).toInt();
    gpsTimeFlg = true;
  }
  gpsZDAFlg = false;
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
      String latStr = getField( gpsTF, 6, ',' );
      gpsLat = latStr.substring( 0,2 ).toFloat();      
      gpsLat = ( gpsLat + latStr.substring( 2 ).toFloat() / 60.0 ) * deg2rad;
      if ( getField( gpsTF, 7, ',' ) == 'S' ) gpsLat *= -1.0;
      String lonStr = getField( gpsTF, 8, ',' );
      gpsLon = lonStr.substring( 0,3 ).toFloat();
      gpsLon = ( gpsLon + lonStr.substring( 3 ).toFloat() / 60.0 ) * deg2rad;
      if ( getField( gpsTF, 9, ',' ) == 'W' ) gpsLon *= -1.0;
      gpsAlt = getField( gpsTF, 10, ',' ).toFloat();
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
      if ( DEBUG ) Serial.println( "\nAPRS msg ended." );
      if ( DEBUG ) Serial.println( aprs );
    }
  } else {
    if ( aprsRdy ) {
      if ( DEBUG ) Serial.println( "\nAPRS msg started..." );
      aprsRdy = false;
    }
  }

  if ( !aprsRdy ) {
    aprs[aprsIndex] = c;
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
    aprsFlg = true;
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
      if ( temp == callSgns[0] || temp == callSgns[1] || temp == callSgns[2] ) {
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

void updateLocal() {
  
}

void updateRemote() {
  
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void digitalClockDisplay(time_t t) {
  // digital clock display of the time
  int hr = hour(t);
  if ( hr < 10 ) Serial.print( '0' );
  Serial.print(hr);
  printDigits(minute(t));
  printDigits(second(t));
  Serial.print(" ");
  Serial.print(day(t));
  Serial.print(" ");
  Serial.print(month(t));
  Serial.print(" ");
  Serial.print(year(t)); 
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
  long tmpMicros;
  if ( DEBUG ) {
    tmpMicros = micros();
    Serial.print( "." );
  }
  
  // Set clock(s) to UTC time if a valid fix came in since the last PPS
  if ( DEBUG ) digitalClockDisplay( now() );
  if ( gpsTimeValid && gpsTimeFlg ) {
    time_t oldTime = getTeensy3Time();
    setTime( gpsHr,gpsMin,gpsSec,gpsDay,gpsMon,gpsYr ); // "Library" time    
    time_t newTime = now() + 1;
    setTime( newTime );
    Teensy3Clock.set( newTime ); // Teensy RTC
    time_t temp;
    EEPROM.get(  eeaddrLastGPSTimeSync, temp );
    if ( ( abs( oldTime - newTime ) >= 1 || 
         ( newTime - temp ) >= maxTimeSyncAge ) ) {
      EEPROM.put( eeaddrLastGPSTimeSync, newTime );
      if ( DEBUG ) {
        Serial.print( "Time Corrected:" );      
      }
    }
  }

  // Reset GPS flags
  gpsPosValid = false;
  gpsTimeValid = false;
  gpsTimeFlg = false;
  gpsZDAFlg = false;
  gpsTFFlg = false;
  updateTarget();
  if ( DEBUG ) {
    Serial.print( "ISR duration: " );
    Serial.print( micros() - tmpMicros );
  }
}

void procAprs() {
  /* Check msg is proper APRS msg from a monitored call sign,
     not some other output from TNC */
  aprsObj.putAprsString( aprsBuf );
  String callSgn = aprsObj.getCallsign();
  boolean callSgnFilt = false;
  for ( int i = 0 ; i < numCallSgns ; i++ ) {
    callSgnFilt |= ( (String)callSgns[i] == callSgn );
  }
  if ( DEBUG ) Serial.print( "\n" + callSgn + ' ');
  if ( DEBUG ) {
    if ( callSgnFilt ) {
      Serial.println( "valid" );
    } else {
      Serial.println( "invalid" );
    }
  }
  if ( callSgnFilt ) {
    // Extract and convert params to m, rad, rad/s
    aprsLat = aprsObj.getLatitude() * deg2rad;
    aprsLon = aprsObj.getLongitude() * deg2rad;
    aprsAlt = (double)aprsObj.getAltitude() * ft2m;
    aprsVg = (float)aprsObj.getGroundspeed() * knots2mps * m2rad;
    aprsHdg = (float)aprsObj.getCourse() * deg2rad;
    aprsNewFlg = true;

    // Derived parameters
    aprsVlon = aprsVg * sin( aprsHdg ) / cos( aprsLat );
    aprsVlat = aprsVg * cos( aprsHdg );

    if ( lastAprsTime == 0 ) {
      lastAprsTime = (double)now();
      aprsTime = lastAprsTime;
      aprsValt = 0.0;
    } else {
      aprsTime = (double)now();
      aprsValt = ( aprsAlt - lastAprsAlt ) / ( aprsTime - lastAprsTime );
      Serial.print( "   :" );Serial.print(aprsTime);
      Serial.print( " " );Serial.println( lastAprsTime );
    }
    double deltaZ = aprsAlt - lastAprsAlt;
    double deltaT = aprsTime - lastAprsTime;
    lastAprsTime = aprsTime;
    lastAprsAlt = aprsAlt;
    if ( DEBUG ) {
      digitalClockDisplay( (long)aprsTime );
      Serial.print("APRS: Lon Lat Alt Vg Hdg Vx Vy Vz:");
      Serial.print( aprsLon, 4 );Serial.print(' ');
      Serial.print( aprsLat, 4 );Serial.print(' ');
      Serial.print( aprsAlt);Serial.print(' ');
      Serial.print( aprsVg );Serial.print(' ');
      Serial.print( aprsHdg );Serial.print(' ');
      Serial.print( aprsVlon );Serial.print(' ');
      Serial.print( aprsVlat );Serial.print(' ');
      Serial.println( aprsValt );
    }

  }
  
}

void updateTarget() {
  // if a new APRS fix came in, use it as last target state
  if ( aprsNewFlg ) {
    if ( DEBUG ) Serial.println( "APRS target update" );
    targetValid = true;
    // Set prior target state to new APRS data
    targetTime = aprsTime;
    targetLon = aprsLon;
    targetLat = aprsLat;
    targetAlt = aprsAlt;
    targetVlon = aprsVlon;
    targetVlat = aprsVlat;
    targetValt = aprsValt;
    aprsNewFlg = false;
  }

  // Update target state if minimum time resolution interval has passed
  double tmpTime = (double)now();
  if ( targetValid && tmpTime > targetTime ) {
    long deltaT = tmpTime - targetTime;
    targetTime = tmpTime; 
    targetLon += ( targetVlon * deltaT );
    targetLat += ( targetVlat * deltaT );
    targetAlt += ( targetValt * deltaT );
  }

  //FOR DEBUG PURPOSES
  Serial.println(targetPosX);
  Serial.println(targetPosY);
  Serial.println(targetPosZ);
  Serial.println(targetVelX);
  Serial.println(targetVelY);
  Serial.println(targetVelZ);
  Serial.println(lastAprsTime);
  Serial.println(targetTime);
  
}


