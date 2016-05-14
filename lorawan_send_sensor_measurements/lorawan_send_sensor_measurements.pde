/*  
 *  Builds heavily on the LoRaWAN_02_join_abp_send_unconfirmed and
 *  Ga_15_frame_class_utility from the Waspmote Code Examples.
 *  Thanks to David Gascon and Luismi Marti for that!
 *  
 *  Copyright (C) 2016 Hans Henrik Grønsleth http://hanshenrik.com
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *   
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *  
 *  Version:           0.1
 */

#include <WaspLoRaWAN.h>
#include <WaspSensorGas_v20.h>
#include <WaspFrame.h>

////////////////////////////
/*** LoRaWAN parameters ***/
////////////////////////////
// LoRaWAN module socket
uint8_t socket = SOCKET0;

// LoRaWAN backend port (from 1 to 223)
uint8_t PORT = 3;


/////////////////////////////////////
/*** Gas Sensor board parameters ***/
/////////////////////////////////////
#define GAINCO2  7  //GAIN of the sensor stage
#define GAINCO  1      // GAIN of the sensor stage
#define RESISTORCO 100  // LOAD RESISTOR of the sensor stage

float temperature;
float humidity;
float co2Val;
float coVal;
float socketCOVal;
float coPPMVal;
float co2PPMVal;

// Mock calibration data for CO sensor, taken from
// http://www.libelium.com/development/waspmote/examples/ga-13-calibrated-sensor-reading/
int coCalibrationConcentration[3] = {50,100,300};
float coCalibrationOutput[3] = {114.3312 , 40.2036 , 11.7751};


/////////////////////////
/*** Other variables ***/
/////////////////////////
uint8_t error;
char nodeFrameID[] = "222";
uint8_t batteryLevel;
float internalTemperature;


void setup() 
{
  // Checks if we come from a normal reset or a hibernate reset
  PWR.ifHibernate();

  // Set frame ID for all frames to be sent
  frame.setID(nodeFrameID);
}

void loop() 
{
  // If a hibernate has been captured, execute the associated function
  if ( intFlag & HIB_INT )
  {
    intFlag &= ~(HIB_INT);
    delay(100);
  }

  // Get battery level
  batteryLevel = PWR.getBatteryLevel();

  // Get internal board temperature
  RTC.ON();
  internalTemperature = RTC.getTemperature();
  RTC.OFF();

  if (batteryLevel < 30)
  {
    doGasSensorBoardMeasurements();
    makeFrame();
    sendFrameWithLoRaWAN();

    // Longer sleep interval since low battery
//      PWR.deepSleep("00:00:30:00", RTC_OFFSET, RTC_ALM1_MODE1, ALL_OFF);
    PWR.hibernate("00:00:30:00", RTC_OFFSET, RTC_ALM1_MODE2);
  }
  
  doGasSensorBoardMeasurements();
  makeFrame();
  sendFrameWithLoRaWAN();

//  PWR.deepSleep("00:00:09:00", RTC_OFFSET, RTC_ALM1_MODE1, ALL_OFF);
  PWR.hibernate("00:00:09:00", RTC_OFFSET, RTC_ALM1_MODE2);
}


void doGasSensorBoardMeasurements()
{
  // Turn on Gas Sensor board, wait for stabilization and sensor response time
  SensorGasv20.ON();
  delay(15000);
  

  //////////////////////////////////////////////////////////////////////
  // 1. Turn on and configure sensors and wait for stabilization 
  //	and sensor response time
  ////////////////////////////////////////////////////////////////////// 

  // Configure the CO2 sensor socket
  SensorGasv20.configureSensor(SENS_CO2, GAINCO2);
  // Power on the CO2 sensor
  SensorGasv20.setSensorMode(SENS_ON, SENS_CO2);
  // Configure the CO sensor socket
  SensorGasv20.configureSensor(SENS_SOCKET4CO, GAINCO, RESISTORCO);
  
  // Wait for stabilization and sensor response time
  delay(40000);
  

  ///////////////////////////////////////////
  // 2. Read sensors
  ///////////////////////////////////////////  

  // Read the temperature sensor
  temperature = SensorGasv20.readValue(SENS_TEMPERATURE);
  // Read the humidity sensor
  humidity = SensorGasv20.readValue(SENS_HUMIDITY);
  // Read the CO2 sensor
  co2Val = SensorGasv20.readValue(SENS_CO2);
  // Read the CO sensor socket
  socketCOVal = SensorGasv20.readValue(SENS_SOCKET4CO);
  // Conversion from voltage into kiloohms
  coVal = SensorGasv20.calculateResistance( SENS_SOCKET4CO,
                                            socketCOVal,
                                            GAINCO,
                                            RESISTORCO);


  ///////////////////////////////////////////
  // 3. Turn off the sensors
  ///////////////////////////////////////////

  // Power off the CO2 sensor
  SensorGasv20.setSensorMode(SENS_OFF, SENS_CO2);
  
  
  ///////////////////////////////////////////
  // 4. Convert to sensible units
  ///////////////////////////////////////////
  
  // Convert CO from kiloohms to ppm
  coPPMVal = SensorGasv20.calculateConcentration( coCalibrationConcentration,
                                                  coCalibrationOutput,
                                                  coVal);

  // Convert CO2 from volts to ppm
  co2PPMVal = pow(10, ((0.22-co2Val)+158.631)/62.877 );
 

  // Turn off sensor board
  SensorGasv20.OFF(); 
}


void makeFrame()
{
  frame.createFrame(ASCII);
  
  frame.addSensor(SENSOR_BAT, batteryLevel);
  frame.addSensor(SENSOR_IN_TEMP, internalTemperature);
  
  frame.addSensor(SENSOR_GP_TC, temperature);
  frame.addSensor(SENSOR_GP_HUM, humidity);
  frame.addSensor(SENSOR_GP_CO, coPPMVal);
  frame.addSensor(SENSOR_GP_CO2, co2PPMVal);
}


void sendFrameWithLoRaWAN()
{
  // Convert frame to a hexadecimal string
  char sendableString[frame.length*2 + 1];
  Utils.hex2str(frame.buffer, sendableString, frame.length);

  // 1. LoRaWAN switch module on
  error = LoRaWAN.ON(socket);

  // 2. LoRaWAN join network
  error = LoRaWAN.joinABP();

  // Check status
  if( error == 0 ) 
  {
    // 3. LoRaWAN send unconfirmed packet
    error = LoRaWAN.sendUnconfirmed( PORT, sendableString );

    if( error == 0 )
    {
      if (LoRaWAN._dataReceived == true)
      {
        // Do something with received data
      }
    }
  }

  // 4. LoRaWAN switch module off
  error = LoRaWAN.OFF(socket);
}

