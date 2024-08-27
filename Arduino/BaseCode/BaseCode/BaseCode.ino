
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

#define MY_ADDRESS 71
#define MAIN_CANSAT_ADDRESS 69

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 869.5

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHReliableDatagram rfManager(rf95, MY_ADDRESS);
// Blinky on receipt
#define LED 7

void setup() 
{
  pinMode(LED, OUTPUT);     
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
    digitalWrite(LED, HIGH);
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    digitalWrite(LED, HIGH);
    while (1);
  }

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop()
{
  if (rfManager.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;
    
    if (rfManager.recvfromAck(buf, &len, &from))
    {
      if(from == MAIN_CANSAT_ADDRESS){
        digitalWrite(LED, HIGH);
        buf[len] = 0; // zero out remaining string
        
        Serial.print((char*)buf);
        digitalWrite(LED, LOW);
      }
      
    }
  }
}
