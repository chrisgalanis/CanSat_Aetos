
  
/*
 MPL3115A2 Altitude Sensor Example
 By: A.Weiss, 7/17/2012, changes Nathan Seidle Sept 23rd, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 Hardware Connections (Breakoutboard to Arduino):
 -VCC = 3.3V
 -SDA = A4
 -SCL = A5
 -INT pins can be left unconnected for this demo
 
 Usage:
 -Serial terminal at 9600bps
 -Prints altitude in meters, temperature in degrees C, with 1/16 resolution.
 -software enabled interrupt on new data, ~1Hz with full resolution
 
 During testing, GPS with 9 sattelites reported 5393ft, sensor reported 5360ft (delta of 33ft). Very close!
 
 */
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <assert.h>
#include <Adafruit_MPL3115A2.h>
#include <RadioHead.h>
#include <RHDatagram.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

#define STATUS     0x00
#define OUT_P_MSB  0x01
#define OUT_P_CSB  0x02
#define OUT_P_LSB  0x03
#define OUT_T_MSB  0x04
#define OUT_T_LSB  0x05
#define DR_STATUS  0x06
#define OUT_P_DELTA_MSB  0x07
#define OUT_P_DELTA_CSB  0x08
#define OUT_P_DELTA_LSB  0x09
#define OUT_T_DELTA_MSB  0x0A
#define OUT_T_DELTA_LSB  0x0B
#define WHO_AM_I   0x0C
#define F_STATUS   0x0D
#define F_DATA     0x0E
#define F_SETUP    0x0F
#define TIME_DLY   0x10
#define SYSMOD     0x11
#define INT_SOURCE 0x12
#define PT_DATA_CFG 0x13
#define BAR_IN_MSB 0x14
#define BAR_IN_LSB 0x15
#define P_TGT_MSB  0x16
#define P_TGT_LSB  0x17
#define T_TGT      0x18
#define P_WND_MSB  0x19
#define P_WND_LSB  0x1A
#define T_WND      0x1B
#define P_MIN_MSB  0x1C
#define P_MIN_CSB  0x1D
#define P_MIN_LSB  0x1E
#define T_MIN_MSB  0x1F
#define T_MIN_LSB  0x20
#define P_MAX_MSB  0x21
#define P_MAX_CSB  0x22
#define P_MAX_LSB  0x23
#define T_MAX_MSB  0x24
#define T_MAX_LSB  0x25
#define CTRL_REG1  0x26
#define CTRL_REG2  0x27
#define CTRL_REG3  0x28
#define CTRL_REG4  0x29
#define CTRL_REG5  0x2A
#define OFF_P      0x2B
#define OFF_T      0x2C
#define OFF_H      0x2D

#define MPL3115A2_ADDRESS 0x60 // 7-bit I2C address
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

#define MY_ADDRESS 69
#define BASE_ADDRESS 71
#define SMALL_CANSAT_ADDRESS 75


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 869.5

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram rfManager(rf95, MY_ADDRESS);

const int ZERO_DEGREES = 600;
const int NINETY_DEGREES = 1750;
const int led = 13;
const int LED = 12;
const int led3 = 11;
const int buzzer = 11;
float temperature, pressure,altitude; 
File lastNumber, lastNumber2, logFile;
int sdFileNumber;
bool sdInitialized = false;
String lastNumberFile = "last.txt";
int logCount;
String logFileName;
const int RELAY = 9;

//Servo SERVO;

 
void releaseCansat(){
  float lastHeight = readAltitude();
  float releaseHeight = lastHeight - 10 ;
  
  if(lastHeight > releaseHeight){
    digitalWrite(led3,HIGH);
    digitalWrite(RELAY,LOW);
  }else{
    //servo();
    digitalWrite(led3,LOW);
    digitalWrite(RELAY,HIGH);
    }
  }

void addToSd(String dataToSd){
  String logRow = "";
  logRow += String(dataToSd);
  }
  
void addToLog(){
  if(!sdInitialized)
    return;
  logCount++;
  if(logCount == 10){
    logCount = 0;
    logFile.close();
    logFile = SD.open(logFileName, FILE_WRITE);
  }
  String logRow = "";
  logRow += String(millis());
  logRow += ", ";
  String pressure;
  addToSd(pressure);
  logRow += ", ";
  String temperature;
  addToSd(temperature);
  logRow += ",";
  logFile.println(logRow);
}

void sendStringToBase(String str){
  uint8_t * data = new uint8_t[str.length()];
  for(int i = 0; i < str.length(); i++){
    data[i] = str[i];
  }
  if (rfManager.sendtoWait(data, str.length(), BASE_ADDRESS)) {
    
  } else {
    //Serial.println("Sending failed (no ack)");
  }
  delete[] data;
}



void setup()
{
  Wire.begin();        // join i2c bus
  Serial.begin(57600);  // start serial for output

  if(IIC_Read(WHO_AM_I) == 196) 
    Serial.println("MPL3115A2 online!");
  else
    Serial.println("No response - check connections");

  // Configure the sensor
  setModeAltimeter(); // Measure altitude above sea level in meters
  setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  setOversampleRate(7); // Set Oversample to the recommended 128
  enableEventFlags(); // Enable all three pressure and temp event flags
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(led, OUTPUT);
  pinMode(LED,OUTPUT);
  pinMode(led3,OUTPUT);
  Serial.begin(9600);
  delay(100);

  pinMode(buzzer, OUTPUT);
//  setBuzzer(0);


  pinMode(RELAY, OUTPUT);
  //setRelay(1);
  digitalWrite(RELAY, LOW);


  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if(RELAY == HIGH){
   // SERVO.write(0);
    }
    else{
      //SERVO.write(40);
      }

  while (!rfManager.init()) {
    digitalWrite(LED,HIGH);
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    digitalWrite(LED,HIGH);
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  if(!SD.begin(10)){
    //Serial.println("Initialization Failed!");
  }
  else{
    if(SD.exists(lastNumberFile)){
      lastNumber = SD.open(lastNumberFile);
      while(lastNumber.available()){
        char c = lastNumber.read();
        if('0' <= c && c <= '9'){
          sdFileNumber *= 10;
          sdFileNumber += int(c - '0');
        }
      }
      lastNumber.close();
      SD.remove(lastNumberFile);
    }
    sdFileNumber++;
    lastNumber2 = SD.open(lastNumberFile, FILE_WRITE);
    lastNumber2.println(String(sdFileNumber));
    lastNumber2.close();
    sdInitialized = true;
    logFileName = "d";
    logFileName += String(sdFileNumber);
    logFileName += ".txt";
    //Serial.println(logFileName);
    logFile = SD.open(logFileName, FILE_WRITE);
  }
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

int v = 1;

void loop()
{
 

  float altitude2 = readAltitude();
  float pressure2 = readPressure();
  float temperature2 = readTemp();

if(pressure2 > -900.0f){
    pressure = pressure2;
  }
  if(temperature2 > -900.0f){
    temperature = temperature2;
  }
  if(altitude > -900.0f){
   altitude = altitude2; 
    }


  sendStringToBase("Pressure: " + String(pressure) + "\n");
  sendStringToBase("Temperature: " + String(temperature) + "\n");
  sendStringToBase("Altitude: " + String(altitude) + "\n");
  
  addToLog();
  digitalWrite(led, (v == 1)?HIGH:LOW);
//  setBuzzer(v);
  v = 1 - v;
  //digitalWrite(RELAY, v);
 releaseCansat();

 ;

  delay(1000);
}

//Returns the number of meters above sea level
float readAltitude()
{
  toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

  //Wait for PDR bit, indicates we have new pressure data
  int counter = 0;
  while( (IIC_Read(STATUS) & (1<<1)) == 0)
  {
      if(++counter > 100) return(-999); //Error out
      delay(1);
  }
  
  // Read pressure registers
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(OUT_P_MSB);  // Address of data to get
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
  Wire.requestFrom(MPL3115A2_ADDRESS, 3); // Request three bytes

  //Wait for data to become available
  counter = 0;
  while(Wire.available() < 3)
  {
    if(counter++ > 100) return(-999); //Error out
    delay(1);
  }

  byte msb, csb, lsb;
  msb = Wire.read();
  csb = Wire.read();
  lsb = Wire.read();

  toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

  // The least significant bytes l_altitude and l_temp are 4-bit,
  // fractional values, so you must cast the calulation in (float),
  // shift the value over 4 spots to the right and divide by 16 (since 
  // there are 16 values in 4-bits). 
  float tempcsb = (lsb>>4)/16.0;

  float altitude = (float)( (msb << 8) | csb) + tempcsb;

  return(altitude);
}

//Returns the number of feet above sea level
float readAltitudeFt()
{
  return(readAltitude() * 3.28084);
}

//Reads the current pressure in Pa
//Unit must be set in barometric pressure mode
float readPressure()
{
  toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

  //Wait for PDR bit, indicates we have new pressure data
  int counter = 0;
  while( (IIC_Read(STATUS) & (1<<2)) == 0)
  {
      if(++counter > 100) return(-999); //Error out
      delay(1);
  }

  // Read pressure registers
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(OUT_P_MSB);  // Address of data to get
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
  Wire.requestFrom(MPL3115A2_ADDRESS, 3); // Request three bytes

  //Wait for data to become available
  counter = 0;
  while(Wire.available() < 3)
  {
    if(counter++ > 100) return(-999); //Error out
    delay(1);
  }

  byte msb, csb, lsb;
  msb = Wire.read();
  csb = Wire.read();
  lsb = Wire.read();

  toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

  // Pressure comes back as a left shifted 20 bit number
  long pressure_whole = (long)msb<<16 | (long)csb<<8 | (long)lsb;
  pressure_whole >>= 6; //Pressure is an 18 bit number with 2 bits of decimal. Get rid of decimal portion.

  lsb &= 0b00110000; //Bits 5/4 represent the fractional component
  lsb >>= 4; //Get it right aligned
  float pressure_decimal = (float)lsb/4.0; //Turn it into fraction

  float pressure = (float)pressure_whole + pressure_decimal;

  return(pressure);
}

float readTemp()
{
  toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

  //Wait for TDR bit, indicates we have new temp data
  int counter = 0;
  while( (IIC_Read(STATUS) & (1<<1)) == 0)
  {
      if(++counter > 100) return(-999); //Error out
      delay(1);
  }
  
  // Read temperature registers
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(OUT_T_MSB);  // Address of data to get
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
  Wire.requestFrom(MPL3115A2_ADDRESS, 2); // Request two bytes

  //Wait for data to become available
  counter = 0;
  while(Wire.available() < 2)
  {
    if(++counter > 100) return(-999); //Error out
    delay(1);
  }

  byte msb, lsb;
  msb = Wire.read();
  lsb = Wire.read();
  
  // The least significant bytes l_altitude and l_temp are 4-bit,
  // fractional values, so you must cast the calulation in (float),
  // shift the value over 4 spots to the right and divide by 16 (since 
  // there are 16 values in 4-bits). 
  float templsb = (lsb>>4)/16.0; //temp, fraction of a degree

  float temperature = (float)(msb + templsb);

  return(temperature);
}

//Give me temperature in fahrenheit!
float readTempF()
{
  return((readTemp() * 9.0)/ 5.0 + 32.0); // Convert celsius to fahrenheit
}

//Sets the mode to Barometer
//CTRL_REG1, ALT bit
void setModeBarometer()
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting &= ~(1<<7); //Clear ALT bit
  IIC_Write(CTRL_REG1, tempSetting);
}

//Sets the mode to Altimeter
//CTRL_REG1, ALT bit
void setModeAltimeter()
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting |= (1<<7); //Set ALT bit
  IIC_Write(CTRL_REG1, tempSetting);
}

//Puts the sensor in standby mode
//This is needed so that we can modify the major control registers
void setModeStandby()
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting &= ~(1<<0); //Clear SBYB bit for Standby mode
  IIC_Write(CTRL_REG1, tempSetting);
}

//Puts the sensor in active mode
//This is needed so that we can modify the major control registers
void setModeActive()
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting |= (1<<0); //Set SBYB bit for Active mode
  IIC_Write(CTRL_REG1, tempSetting);
}

//Setup FIFO mode to one of three modes. See page 26, table 31
//From user jr4284
void setFIFOMode(byte f_Mode)
{
  if (f_Mode > 3) f_Mode = 3; // FIFO value cannot exceed 3.
  f_Mode <<= 6; // Shift FIFO byte left 6 to put it in bits 6, 7.

  byte tempSetting = IIC_Read(F_SETUP); //Read current settings
  tempSetting &= ~(3<<6); // clear bits 6, 7
  tempSetting |= f_Mode; //Mask in new FIFO bits
  IIC_Write(F_SETUP, tempSetting);
}

//Call with a rate from 0 to 7. See page 33 for table of ratios.
//Sets the over sample rate. Datasheet calls for 128 but you can set it 
//from 1 to 128 samples. The higher the oversample rate the greater
//the time between data samples.
void setOversampleRate(byte sampleRate)
{
  if(sampleRate > 7) sampleRate = 7; //OS cannot be larger than 0b.0111
  sampleRate <<= 3; //Align it for the CTRL_REG1 register

  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting &= 0b11000111; //Clear out old OS bits
  tempSetting |= sampleRate; //Mask in new OS bits
  IIC_Write(CTRL_REG1, tempSetting);
}

//Clears then sets the OST bit which causes the sensor to immediately take another reading
//Needed to sample faster than 1Hz
void toggleOneShot(void)
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting &= ~(1<<1); //Clear OST bit
  IIC_Write(CTRL_REG1, tempSetting);

  tempSetting = IIC_Read(CTRL_REG1); //Read current settings to be safe
  tempSetting |= (1<<1); //Set OST bit
  IIC_Write(CTRL_REG1, tempSetting);
}

//Enables the pressure and temp measurement event flags so that we can
//test against them. This is recommended in datasheet during setup.
void enableEventFlags()
{
  IIC_Write(PT_DATA_CFG, 0x07); // Enable all three pressure and temp event flags 
}

// These are the two I2C functions in this sketch.
byte IIC_Read(byte regAddr)
{
  // This function reads one byte over IIC
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(regAddr);  // Address of CTRL_REG1
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
  Wire.requestFrom(MPL3115A2_ADDRESS, 1); // Request the data...
  return Wire.read();
}

void IIC_Write(byte regAddr, byte value)
{
  // This function writes one byto over IIC
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission(true);
}
