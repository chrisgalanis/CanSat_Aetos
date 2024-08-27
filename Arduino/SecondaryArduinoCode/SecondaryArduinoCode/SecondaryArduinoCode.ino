#include "sSense_BME680.h" // Include the BME680 Sensor library
#include <RH_RF95.h>
#include <RHReliableDatagram.h>


#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#define RF95_FREQ 869.5

#define MY_ADDRESS 75
#define BASE_ADDRESS 71
#define MAIN_CANSAT_ADDRESS 69

#define led 7

BME680_Class BME680; ///< Create an instance of the BME680
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram rfManager(rf95, MY_ADDRESS);

float temperature, humidity, pressure;
bool hasBME = true;
String gpsData = "";

void read_bmea() {
  if(!hasBME){
    return;
  }
  static int32_t temperature2, humidity2, pressure2, gas2;     // Variable to store readings
  BME680.getSensorData(temperature2,humidity2,pressure2,gas2);
  temperature = temperature2/100.0;
  humidity = humidity2/1000.0;
  pressure = pressure2;
}

void sendStringToMainCanSat(String str){
  uint8_t * data = new uint8_t[str.length()];
  for(int i = 0; i < str.length(); i++){
    data[i] = str[i];
  }
  if (rfManager.sendtoWait(data, str.length(), MAIN_CANSAT_ADDRESS)) {
    
  } else {
    Serial.println("Sending failed (no ack)");
  }
  delete[] data;
}

String currGPS = "";

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


void setup()
{
  //Serial.begin(9600); // Start serial port at Baud rate
  if(!BME680.begin(I2C_STANDARD_MODE)){
    pressure = -999.0;
    temperature = -999.0;
    humidity = -999.0;
    hasBME = false;
  }
  else{
    //Setting 16x oversampling for all sensors
    //digitalWrite(led, LOW);
    BME680.setOversampling(TemperatureSensor,Oversample16); // Use enumerated type values
    BME680.setOversampling(HumiditySensor,   Oversample16);
    BME680.setOversampling(PressureSensor,   Oversample16);
    //Setting IIR filter to a value of 4 samples
    BME680.setIIRFilter(IIR4);
    //Setting gas measurement to 320C for 150ms
    BME680.setGas(320,150); // 320οΏ½c for 150 milliseconds
  }
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  while (!Serial);
  Serial.begin(9600);
  delay(100);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rfManager.init()) {
   // Serial.println("LoRa radio init failed");
   //digitalWrite(led,HIGH);
    while (1);
  }
  //Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    //Serial.println("setFrequency failed");
    //digitalWrite(led,HIGH);
    while (1);
  }
  rf95.setTxPower(23, false);

} // of method setup()

int v =1;
void loop() 
{
  //static uint8_t loopCounter = 0;
  read_bmea() ;
  String results = String(pressure) + ", " + String(temperature) + ", " + String(humidity) + ", #" + gpsData + "#" + "\n";
  sendStringToMainCanSat(results);
  Serial.println(results);

  digitalWrite(led,(v == 1)? HIGH:LOW);
  v = 1-v;
  delay(10);
} 
