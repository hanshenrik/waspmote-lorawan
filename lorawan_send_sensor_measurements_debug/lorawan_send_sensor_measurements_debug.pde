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
  // Set frame ID for all frames to be sent
  frame.setID(nodeFrameID);
  
  // DEV Print stuff to the Serial Monitor
  USB.ON();
}

void loop() 
{
  // Get battery level
  batteryLevel = PWR.getBatteryLevel();
  
  if (batteryLevel < 30) {
    USB.println(F("Battery is below 30%"));

//    TODO:
//    sendLowBatteryFrame();

    internalTemperature = RTC.getTemperature();
    
    doGasSensorBoardMeasurements();
    makeFrame();
    sendFrameWithLoRaWAN();
    
    // Longer sleep interval since low battery
    PWR.deepSleep("00:00:30:00", RTC_OFFSET, RTC_ALM1_MODE1, ALL_OFF);
  }
  
  internalTemperature = RTC.getTemperature();
  
  doGasSensorBoardMeasurements();
  makeFrame();
  sendFrameWithLoRaWAN();

  USB.println();
  PWR.deepSleep("00:00:09:00", RTC_OFFSET, RTC_ALM1_MODE1, ALL_OFF);
}


void doGasSensorBoardMeasurements()
{
  // Blink green LED to indicate doing measurements
  Utils.blinkGreenLED(200, 3);
  
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

  // DEV Print frame
  frame.showFrame();

  USB.print(F("Frame length: "));
  USB.println(frame.length, DEC);
  USB.print(F("Frame max size from frame.getFrameSize(): "));
  USB.println(frame.getFrameSize(), DEC);
}


void sendFrameWithLoRaWAN()
{
  // Blink red LED to indicate sending frame with LoRaWAN
  Utils.blinkRedLED(200, 3);
  
  // Convert frame to a hexadecimal string
  char sendableString[frame.length*2 + 1];
  Utils.hex2str(frame.buffer, sendableString, frame.length);

  USB.print(F("Sendable string: "));
  USB.println(sendableString);
  USB.print(F("strlen(sendableString): "));
  USB.println(strlen(sendableString));
  USB.print(F("sizeof(sendableString): "));
  USB.println(sizeof(sendableString));

  // 1. LoRaWAN switch module on
  error = LoRaWAN.ON(socket);

  // Check status
  if( error == 0 )
  {
    USB.println(F("1. LoRaWAN switch module ON OK"));
  }
  else 
  {
    USB.print(F("1. LoRaWAN switch module ON error = ")); 
    USB.println(error, DEC);
  }

  // 2. LoRaWAN join network
  error = LoRaWAN.joinABP();

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("2. LoRaWAN join network OK"));   

    Utils.blinkGreenLED(1000);
    // 3. LoRaWAN send unconfirmed packet
    error = LoRaWAN.sendUnconfirmed( PORT, sendableString );

    // Error messages:
    /*
     * '6' : Module hasn't joined a network
     * '5' : Sending error
     * '4' : Error with data length	  
     * '3' : Init error
     * '2' : Module didn't response
     * '1' : Module communication error   

     These are defined in Waspmote LoRaWAN networking guide p. 13
     */
    // Check status
    if( error == 0 ) 
    {
      // Get Device EUI
      LoRaWAN.getDeviceAddr();
      USB.print(F("3. LoRaWAN send unconfirmed packet OK from device address: "));
      USB.println(LoRaWAN._devAddr);

      if (LoRaWAN._dataReceived == true)
      { 
        USB.print(F("   There's data on port number "));
        USB.print(LoRaWAN._port,DEC);
        USB.print(F(".\r\n   Data: "));
        USB.println(LoRaWAN._data);
      }
    }
    else 
    {
      USB.print(F("3. LoRaWAN send unconfirmed packet error = ")); 
      USB.println(error, DEC);
    }
  }
  else 
  {
    USB.print(F("2. LoRaWAN join network error = ")); 
    USB.println(error, DEC);
  }
  

  // DEV Print LoRaWAN info
  printLoRaWANInfo();

  
  // 4. LoRaWAN switch module off
  error = LoRaWAN.OFF(socket);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("4. LoRaWAN switch module OFF OK"));     
  }
  else 
  {
    USB.print(F("4. LoRaWAN switch module OFF error = ")); 
    USB.println(error, DEC);
  }
}

void printLoRaWANInfo()
{
  LoRaWAN.getADR();
  USB.print(F("ADR: "));
  USB.println(LoRaWAN._adr, DEC);

  LoRaWAN.getBand();
  USB.print(F("Band: "));
  USB.println(LoRaWAN._band);

  LoRaWAN.getRadioFreq();
  USB.print(F("Radio frequency: "));
  USB.println(LoRaWAN._radioFreq, DEC);

  LoRaWAN.getDataRate();
  USB.print(F("DataRate: "));
  USB.println(LoRaWAN._dataRate, DEC);

  LoRaWAN.getRadioSF();
  USB.print(F("Radio spreading factor: "));
  USB.println(LoRaWAN._radioSF);

  LoRaWAN.getRadioBW();
  USB.print(F("Radio bandwidth: "));
  USB.println(LoRaWAN._radioBW, DEC);

  LoRaWAN.getRadioCR();
  USB.print(F("Radio coding rate: "));
  USB.println(LoRaWAN._radioCR);

  LoRaWAN.getRadioMode();
  USB.print(F("Radio mode: "));
  USB.println(LoRaWAN._radioMode);

  LoRaWAN.getRadioFreqDeviation();
  USB.print(F("Radio frequency deviation: "));
  USB.println(LoRaWAN._radioFreqDev, DEC);

  LoRaWAN.getPower();
  USB.print(F("Power index: "));
  USB.println(LoRaWAN._powerIndex, DEC);

  LoRaWAN.getRadioPower();
  USB.print(F("Radio power: "));
  USB.println(LoRaWAN._radioPower, DEC);

  LoRaWAN.getRadioWDT();
  USB.print(F("Radio watch dog timer's time: "));
  USB.println(LoRaWAN._radioWDT, DEC);

  LoRaWAN.getRadioPreamble();
  USB.print(F("Radio preamble length (LoRaWAN header?): "));
  USB.println(LoRaWAN._preambleLength, DEC);

  for( int i=0; i<3; i++)
  {
    LoRaWAN.getChannelFreq(i);
    LoRaWAN.getChannelDutyCycle(i);
    LoRaWAN.getChannelDRRange(i);
    LoRaWAN.getChannelStatus(i);

    USB.print(F("Channel: "));
    USB.println(i);
    USB.print(F("  Freq: "));
    USB.println(LoRaWAN._freq[i]);
    USB.print(F("  Duty cycle: "));
    USB.println(LoRaWAN._dCycle[i]);
    USB.print(F("  Duty cycle in %: "));
    USB.println(100 / (float)(LoRaWAN._dCycle[i] + 1));
    USB.print(F("  DR min: "));
    USB.println(LoRaWAN._drrMin[i], DEC);
    USB.print(F("  DR max: "));
    USB.println(LoRaWAN._drrMax[i], DEC);
    USB.print(F("  Status: "));
    USB.println(LoRaWAN._status[i], DEC);
    USB.println(F("----------------------------"));
  }
}

