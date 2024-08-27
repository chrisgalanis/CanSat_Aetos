


#include "sSense_BME680.h" // Include the BME680 Sensor library

#define SERIAL_SPEED  19200//; ///< Set the baud rate for Serial I/O

BME680_Class BME680; ///< Create an instance of the BME680


float altitude(const float seaLevel=1013.25) 
{
  
  static float Altitude;
  int32_t temp, hum, press, gas;
  BME680.getSensorData(temp,hum,press,gas); // Get the most recent values from the device
  Altitude = 44330.0*(1.0-pow(((float)press/100.0)/seaLevel,0.1903)); // Convert into altitude in meters
  return(Altitude);
} 
float calculate_altitude( float pressure, bool metric = true, float seaLevelPressure = 101325)
{
  float altitude = NAN;
  if (!isnan(pressure) && !isnan(seaLevelPressure)){
    altitude = 1000.0 * ( seaLevelPressure - pressure ) / 3386.3752577878;
  }
  return metric ? altitude * 0.3048 : altitude;
}

float temperatureCompensatedAltitude(int32_t pressure, float temp=21.0 /*Celsius*/, float seaLevel=1013.25) 
{
  
  float Altitude;
  Altitude = (pow((seaLevel/((float)pressure/100.0)), (1/5.257))-1)*(temp + 273.15) / 0.0065; // Convert into altitude in meters
  return(Altitude);  //this are metric value
} 

float temperature, humidity, pressure, gas;

void read_bmea() {
  static int32_t temperature2, humidity2, pressure2, gas2;     // Variable to store readings
  BME680.getSensorData(temperature2,humidity2,pressure2,gas2);
  temperature = temperature2/100.0;
  humidity = humidity2/1000.0;
  pressure = pressure2/100.0;
  gas = gas2/100.0;
}


void setup()
{
  DebugPort.begin(SERIAL_SPEED); // Start serial port at Baud rate

  while(!DebugPort) {delay(10);} // Wait

  

  DebugPort.println("s-Sense BME680 I2C sensor.");
  DebugPort.print("- Initializing BME680 sensor\n");
  while (!BME680.begin(I2C_STANDARD_MODE)) // Start BME680 using I2C protocol
  {
    DebugPort.println("-  Unable to find BME680. Waiting 1 seconds.");
    delay(100);
  } // of loop until device is located
  DebugPort.println("- Setting 16x oversampling for all sensors");
  BME680.setOversampling(TemperatureSensor,Oversample16); // Use enumerated type values
  BME680.setOversampling(HumiditySensor,   Oversample16);
  BME680.setOversampling(PressureSensor,   Oversample16);
  DebugPort.println("- Setting IIR filter to a value of 4 samples");
  BME680.setIIRFilter(IIR4);
  DebugPort.println("- Setting gas measurement to 320C for 150ms");
  BME680.setGas(320,150); // 320οΏ½c for 150 milliseconds
  DebugPort.println();
} // of method setup()

/*!
    @brief    Arduino method for the main program loop
    @details  This is the main program for the Arduino IDE, it is an infinite loop and keeps on repeating. 
    @return   void
*/
void loop() 
{
  //static uint8_t loopCounter = 0;
  read_bmea() ;
  DebugPort.print("\r\nSensor data >>\t\t");                       
  DebugPort.print(temperature);                      
  DebugPort.print("C\t");                          
  DebugPort.print(humidity);                         
  DebugPort.print("%\t");
  DebugPort.print(pressure);                          
  DebugPort.print("hPa\t");
  //DebugPort.print(pressure);                          
  //DebugPort.print("Pa ");
  DebugPort.print(gas);
  DebugPort.println("mOhm");

  DebugPort.println("\r\nCalculated altitude");

  DebugPort.print("temp comp [CASIO equation]: ");

  DebugPort.print(temperatureCompensatedAltitude(pressure, temperature/100.0/*, 1022.0*/),2); 
  DebugPort.print("m\t");


  DebugPort.print("NOAA equation: ");

  DebugPort.print(calculate_altitude((long)pressure,true),2); //calculate_altitude
  //DebugPort.print(calculate_altitude((long)pressure,true, (long)102200.0),2); //calculate_altitude
  DebugPort.print("m\t");

  DebugPort.print("WIKI equation: ");
  DebugPort.print(altitude(),2); 
  DebugPort.println("m \r\n");

  delay(1000);
} 
