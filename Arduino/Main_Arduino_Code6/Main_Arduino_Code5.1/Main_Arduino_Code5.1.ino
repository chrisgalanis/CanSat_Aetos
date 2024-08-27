

#include<Servo.h>
#include <SD.h>
#include <SPI.h>
#include <assert.h>
#include <Adafruit_MPL3115A2.h> 
#include <RadioHead.h>
#include <RHDatagram.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

#define MY_ADDRESS 69
#define BASE_ADDRESS 71
#define SMALL_CANSAT_ADDRESS 75

#define RH_ASK_ARDUINO_USE_TIMER2
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

//File lastNumber, lastNumber2, logFile;
//int sdFileNumber;
//bool sdInitialized = false;
//String lastNumberFile = "last.txt";
//int logCount;
//String logFileName;
String gpsData="";
bool isAverage = true;
bool FirstMessures = true;

//const int RELAY = 9;

Servo SERVO;

/*
void servo(int n){
 assert(n ==0 || n==1);
 digitalWrite(led3,HIGH);
  SERVO.write(0);
 delay(1000);
 digitalWrite(led3,LOW);
 SERVO.write(60);
 delay(1000);
}
*/

void releaseCanSat(){
    int altA;
     float alt[]={altitude};
   if (isAverage){
     for(int i=0; i<5; i++){
       altA = (alt[0]+alt[1]+alt[2]+alt[3]+alt[4]) ;
      }
        Serial.print("First A "); Serial.println(altA);
        delay(1000);
         isAverage = false; 
      }
    bool fAltA = altA > altitude; 
    Serial.print("Boolean is: ");Serial.println(fAltA);
    
   if (altA > altitude && altA != altitude){
     Serial.println("Servo works");
        digitalWrite(led3,HIGH);
        SERVO.write(60);
        delay(100);
        SERVO.write(0);
        delay(100);
      
    }
    else{
       digitalWrite(led3,LOW);
      }
   
 }
   
 
    
  
/*void addToSd(String dataToSd){
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
}*/

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

String currGPS;
void getGPS(){
  while(Serial.available()){
    char c = Serial.read();
    if(c == '\n'){
      if(currGPS.length() > 6 && currGPS[0] == '$' && currGPS[3] == 'G' && currGPS[4] == 'G' && currGPS[5] == 'A'){
        gpsData = currGPS;
      }
      currGPS = "";
    }
    else{
      currGPS += c;
    }
  }
}
// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
// See the Wire tutorial for pinouts for each Arduino
// http://arduino.cc/en/reference/wire


void setup() {

  SERVO.attach(10);
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(led, OUTPUT);
  pinMode(LED,OUTPUT);
  pinMode(led3,OUTPUT);
  Serial.begin(9600);

  pinMode(buzzer, OUTPUT);
//  setBuzzer(0);

 
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);



  while (!rfManager.init()) {
    digitalWrite(LED,HIGH);
    SERVO.write(0);
    delay(1000);
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
/*
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
  }*/


}

int16_t packetnum = 0;  // packet counter, we increment per xmission

int v = 1;

void loop() {
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }

float pressure2 = baro.getPressure();
  float altitude2 = baro.getAltitude();
  float temperature2 = baro.getTemperature();
  
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
  sendStringToBase("GPS: " + String(gpsData) + "\n");
  Serial.println(altitude);

  
  if(FirstMessures){
   for(int i=0; i<=5; i++){ 
   Serial.println(altitude);
  
    //servo(v);
    //addToLog();
    FirstMessures = false;
  } 
 }
 
 digitalWrite(led, (v == 1)?HIGH:LOW);
  
 // setBuzzer(v);
  v = 1 - v;
 // digitalWrite(RELAY, v);

 
   if(rfManager.available())
{
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;
    
    if (rfManager.recvfromAck(buf, &len, &from))
    {
      if(from == SMALL_CANSAT_ADDRESS){
        digitalWrite(LED, HIGH);
        buf[len] = 0; // zero out remaining string
        String SmallCansatData = (char*)buf;
      //  addToSd(SmallCansatData);
        Serial.print(SmallCansatData);
        sendStringToBase(SmallCansatData);
        digitalWrite(LED, LOW);
      }
  }
  
}
 
  releaseCanSat();
   delay(250); 
  
  //Serial.println(pressure)
  
  

}
