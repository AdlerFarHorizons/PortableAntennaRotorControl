//THIS IS A MODIFIED ADAFRUIT EXAMPLE - KN


const int xInput = A0;
const int yInput = A1;
const int zInput = A2;
const int buttonPin = 2;

// Raw Ranges:
// initialize to mid-range and allow calibration to
// find the minimum and maximum for each axis
int xRawMin = 512;
int xRawMax = 512;

int yRawMin = 512;
int yRawMax = 512;

int zRawMin = 512;
int zRawMax = 512;

// Take multiple samples to reduce noise
const int sampleSize = 10;

// Read "sampleSize" samples and report the average
int ReadAxis(int axisPin){
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++){
    reading += analogRead(axisPin);
  }
  return reading/sampleSize;
}

//
// Find the extreme raw readings from each axis
//
void AutoCalibrate(int xRaw, int yRaw, int zRaw){
  if (xRaw < xRawMin){
    xRawMin = xRaw;
  }
  if (xRaw > xRawMax){
    xRawMax = xRaw;
  }
  
  if (yRaw < yRawMin){
    yRawMin = yRaw;
  }
  if (yRaw > yRawMax){
    yRawMax = yRaw;
  }

  if (zRaw < zRawMin){
    zRawMin = zRaw;
  }
  if (zRaw > zRawMax){
    zRawMax = zRaw;
  }
}

void setup() 
{
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);
}

void loop() 
{
  int xRaw = ReadAxis(xInput);
  int yRaw = ReadAxis(yInput);
  int zRaw = ReadAxis(zInput);
  AutoCalibrate(xRaw, yRaw, zRaw);
  Serial.print("X:");
  Serial.print(xRaw);
  Serial.print(", Y:");
  Serial.print(yRaw);
  Serial.print(", Z: ");
  Serial.println(zRaw);

  if (digitalRead(buttonPin) == LOW){
    //RESET VALUES
    xRawMin = 512;
    xRawMax = 512;

    yRawMin = 512;
    yRawMax = 512;

    zRawMin = 512;
    zRawMax = 512;
  }
  delay(250);
  //Serial.print("X:");
  //Serial.print((xRawMin+xRawMax)/2);
  //Serial.print(", Y:");
  //Serial.print((yRawMin+yRawMax)/2);
  //Serial.print(", Z: ");
  //Serial.println((zRawMin+zRawMax)/2);
}
