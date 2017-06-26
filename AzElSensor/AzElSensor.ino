/*
 * Base code for AzElSensor taken from:
 * 
 * https://github.com/chrisspen/homebot/blob/master/src/test/MPU9250-test1/src/mpu9250_arduino.ino
 * 
 * Attribution is not provided, but is apparently based on the
 * Arduino Mega code (apparently) by Phillipe Lucidarme at:
 * 
 * http://www.lucidarme.me/?p=5057
 * 
*/

// 4/15 changes by Ellie:  starting line 129, I made subroutines to read/print accel and gyro data


#include <Wire.h>
#include <TimerOne.h>

const byte MPU9250_ADDRESS = 0x68;
const byte MAG_ADDRESS = 0x0C;

const byte GYRO_FULL_SCALE_250_DPS = 0x00;
const byte GYRO_FULL_SCALE_500_DPS = 0x08;
const byte GYRO_FULL_SCALE_1000_DPS = 0x10;
const byte GYRO_FULL_SCALE_2000_DPS = 0x18;

const byte ACC_FULL_SCALE_2_G = 0x00;
const byte ACC_FULL_SCALE_4_G = 0x08;
const byte ACC_FULL_SCALE_8_G = 0x10;
const byte ACC_FULL_SCALE_16_G = 0x18;

uint8_t Buf[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
uint8_t Mag[7];

boolean cmdPending;
String cmdStr;

void setup() {

  Wire.begin();
  Serial.begin(9600);
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
  pinMode(13, OUTPUT);

  cmdPending = false;
  
}

void loop() {

  if ( Serial.available() ) getInByte();
  if ( cmdPending ) processCmd();
  
}

void processCmd() {

  if ( cmdStr == "read" ) getReadings();
  cmdPending = false;
  cmdStr = "";

}

void getReadings() {

  accelerometer();
  Serial.print( "," );
  magnetometer();
  Serial.print( "," );
  gyro();
  Serial.println( "" );

}

void accelerometer() {

  // Read accelerometer data  
  I2Cread(MPU9250_ADDRESS,0x3B,6,Buf);

  // Create 16 bits values from 8 bits data
  ax=-(Buf[0]<<8 | Buf[1]);
  ay=-(Buf[2]<<8 | Buf[3]);
  az=Buf[4]<<8 | Buf[5];

  //print accelerometer data  
  Serial.print (ax,DEC); 
  Serial.print (",");
  Serial.print (ay,DEC);
  Serial.print (",");
  Serial.print (az,DEC);  

}

void gyro() {

// Read gyroscope data
  I2Cread(MPU9250_ADDRESS,0x43,6,Buf);

// Create 16 bits values from 8 bits data
  int16_t gx=-(Buf[0]<<8 | Buf[1]);
  int16_t gy=-(Buf[2]<<8 | Buf[3]);
  int16_t gz=Buf[4]<<8 | Buf[5];

// print gyroscope data
  Serial.print (gx,DEC); 
  Serial.print (",");
  Serial.print (gy,DEC);
  Serial.print (",");
  Serial.print (gz,DEC);  

}
  
void magnetometer() {

// Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  
  do {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  } while (!(ST1&0x01));
  
// Read magnetometer data   
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

// Create 16 bits values from 8 bits data
  mx=-(Mag[3]<<8 | Mag[2]);
  my=-(Mag[1]<<8 | Mag[0]);
  mz=-(Mag[5]<<8 | Mag[4]);

// print magnetometer data
  Serial.print (mx,DEC);//(mx+200,DEC); 
  Serial.print (",");
  Serial.print (my,DEC);//(my-70,DEC);
  Serial.print (",");
  Serial.print (mz,DEC);//(mz-700,DEC);  

}

void getInByte() {
  char inByte = Serial.read();
  if ( !cmdPending ) {
    if ( inByte == '\r' || inByte == '\n' ) {
      cmdPending = true;
    } else {
      cmdStr += inByte;
    }
  }
}

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


   










