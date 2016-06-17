#include <MsTimer2.h>
String ident1 = "KC9LHW-11";
String ident2 = "KC9LIG-11";
String prefix = ">APBL10,WIDE1-1,WIDE2-1:!";
String comment = "Adler Planetarium Balloon";
String aprs_str1 = "KC9LHW-11>APBL10,WIDE1-1,WIDE2-1:!4106.18N/08755.31WO309/022/A=005939Adler Planetarium Balloon";
String aprs_str2 = "KC9LIG-11>APBL10,WIDE1-1,WIDE2-1:!4106.20N/08755.20WO311/022/A=006000Adler Planetarium Balloon";
String inStr = "";
byte inByte;
char inChar;
boolean timesUp, altFlg;

float startLat = 41.09; // degrees
float startLon = -87.9167100; // degrees
float startAlt = 434.3; // m

float earth_radius = 6.7E6; // m
float latScale = 1.124E5; // m/deg
float knots2mps = 0.51444444444;
float ft2m = 0.3048;
float deg2rad = 3.1415927 / 180.0;
float vAscent = 6; // m/sec
float vDescent = -12; // m/sec
float altBurst = 3e4; // m
boolean isBurst;
float vWind = 30; // m/sec
float vWindAngle = 105; // 0-360 deg east of north => tan( vx/vy )
float vx, vy, vz; // x=>lon, y=>lat, z=>alt
float position1[3] = {startLon, startLat, startAlt};
float position2[3] = {startLon, startLat, startAlt};
int tUpdate = 30; // seconds
int speedup = 4;

unsigned long timeNow, timeLast;

void setup() {
  MsTimer2::set( round( 1000.0 * tUpdate / ( 1.0 * speedup) ), sendString );
  Serial.begin(9600);
  Serial.flush();
  delay(1);
  timeLast = millis();
  vx = vWind * sin( vWindAngle * deg2rad );
  vy = vWind * cos( vWindAngle * deg2rad );
  vz = vAscent;
  Serial.println( aprs_str( ident1 ) );
  timesUp = false;
  altFlg = false;
  isBurst = false;
  MsTimer2::start();
}

void loop() {

  if ( Serial.available() > 0 ) {

    inChar = (char)Serial.read();
    
    // Check for a CTRL-<x> and loop back something readable
    // unless it's a CR BS or LF. 
    // If CR add a LF in case the terminal isn't sending them.
    // If LF (^J), put it through. 
    // If BS, erase the character
    
    if (inChar == '\r') { //Terminal sent a CR, add LF
    
      Serial.write( '\n' );Serial.write( inChar );
      inStr = "";

     } else if ( inChar == '\n' ) { //Line Feed only, send it.
     
      Serial.write( inChar );
      inStr += inChar;
      
    } else if ( (byte)inChar == (char)0x08 ) { // BS, erase the char.

      Serial.write( inChar );Serial.write( '\x20' );Serial.write( inChar );
      inStr = inStr.substring( 0, inStr.length() - 1 );

    } else if ( (byte)inChar < 0x20 ) {

      inChar = (char)((byte)inChar + 0x40);
      Serial.write( '^' );Serial.write( inChar );
      inStr += '^';inStr += inChar;

    } else {

      Serial.write( inChar );
      inStr += inChar;

    }

    Serial.flush();
  
  }

  if ( timesUp ) {
    updatePosition();
    if ( altFlg ) {
      Serial.println( aprs_str( ident1 ) );
    } else {
      Serial.println( aprs_str( ident2 ) );
    }
    altFlg = !altFlg;
    timesUp = false;
  }

  if ( !isBurst && position1[2] > altBurst ) {
    vz = vDescent;
    isBurst = true;
  } 

  if ( isBurst && position1[2] < startAlt ) {
    vx = 0.0;
    vy = 0.0;
    vz = 0.0;
  }

}

String aprs_str( String ident ) {
  
  return  ident + prefix +
          latToString( position1[1] ) + "/" +
          lonToString( position1[0] ) + "O" +
          headingToString( vx, vy ) + "/" +
          gndSpeedToString( vWind ) + "/A=" +
          altToString( position1[2] ) + comment;
}

String headingToString( float velx, float vely ) {
  float angle = ( atan2( velx, vely ) ) / deg2rad;
  if ( angle < 0 ) angle += 180.0;
  String angleStr = String( round( angle ) );   
  return zeroPad( angleStr, 3 );
}

String gndSpeedToString( float speed ) {
  String speedStr = String( round( speed ) );  
  return zeroPad( speedStr, 3 );
}

String altToString( float alt ) {
  String altStr = String( round( alt / ft2m ) );
  return zeroPad( altStr, 6 );
}

void updatePosition( ) {
  timeNow = millis();
  float deltaT = speedup * ( timeNow - timeLast ) / 1000.0;
  timeLast = timeNow;
  addToLatLonAlt( deltaT * vy, deltaT * vx, deltaT * vz );
}

void addToLatLonAlt( float dy, float dx, float dz ) {
  position1[0] += dx * cos( position1[1] * deg2rad ) / latScale;
  position1[1] += dy / latScale;
  position1[2] += dz;
}

String latToString( float lat ) {
  char dir = 'N';
  if ( lat < 0.0 ) dir = 'S';
  int deg = (int)abs( lat );
  int mins = round( 100.0 * ( 60.0 * ( abs( lat ) - (float)deg ) ) );
  String degStr = String( deg );
  String minStr = String( mins );
  // Leading zero-fills
  degStr = zeroPad( degStr, 2 );
  minStr = zeroPad( minStr, 4 );
  return degStr + minStr.substring( 0, 2 )
         + '.' + minStr.substring( 2 ) + dir;
}

String lonToString( float lon ) {
  char dir = 'E';
  if ( lon < 0.0 ) dir = 'W';
  int deg = (int)abs( lon );
  String degStr = String( deg );
  int mins = (int)round( 100 * ( 60.0 * ( abs( lon ) - deg ) ) );
  String minStr = String( mins );
  // Leading zero-fills
  degStr = zeroPad( degStr, 3 );
  minStr = zeroPad( minStr, 4 );
  return degStr + minStr.substring( 0, 2 )
         + '.' + minStr.substring( 2 ) + dir;  
}

String zeroPad( String str, int strLen ) {  
  int len = str.length();
  for ( int i = 0 ; i < strLen - len ; i++ ) {
    str = "0" + str;
  }
  return str;
}
void sendString() {
  timesUp = true;
}

