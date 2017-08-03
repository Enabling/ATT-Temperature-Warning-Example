#include <Wire.h>
#include <ATT_LoRa_IOT.h>
#include <MicrochipLoRaModem.h>
#include "keys.h"
#include <Adafruit_BME280.h>

#define SERIAL_BAUD 57600

#define sensor TEMPERATURE_SENSOR 

Adafruit_BME280 bme;                          // create an instance of the tph sensor
MicrochipLoRaModem Modem(&Serial1, &Serial);
ATTDevice Device(&Modem, &Serial);

float temperature = 0.0; 

//setup() is run once everytime the device starts running
void setup() 
{
  bme.begin();                                        // connect TPH sensor to the I2C pin (SCL/SDA)
  while((!Serial) && (millis()) < 2000){}             // wait until serial bus is available, so we get the correct logging on screen. If no serial, then blocks for 2 seconds before run
  Serial.begin(SERIAL_BAUD);                          // set baud rate of the default serial debug connection
  Serial1.begin(Modem.getDefaultBaudRate());          // set baud rate of the serial connection between Mbili and LoRa modem
  while(!Device.Connect(DEV_ADDR, APPSKEY, NWKSKEY))
  Serial.println("retrying...");                      
  Serial.println("Ready to send data");
}

//after setup(), loop() is run continuously
void loop() 
{
  SendValue();
  delay(10000); //wait 10s before sending another value
}

bool SendValue()
{
  temperature = bme.readTemperature();                                    // read the value of the sensor
  Serial.print("Temperature: "); Serial.println(bme.readTemperature());   // print the value on the serial monitor
  bool res = Device.Send(temperature, sensor);                            // send the value over the LoRa network
  if(res == false)
    Serial.println("maybe the last couple of packages were sent too quickly after each other? (min of 15 seconds recommended)");
  return res;
}
