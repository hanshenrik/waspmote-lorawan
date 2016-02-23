/*  
 *  Builds heavily on the LoRaWAN_02_join_abp_send_unconfirmed Waspmote
 *  Code Examples. Thanks to David Gascon and Luismi Marti for that!
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
#include <WaspFrame.h>

// LoRaWAN module socket
uint8_t socket = SOCKET0;

// LoRaWAN backend port (from 1 to 223)
uint8_t PORT = 3;

// Variables
uint8_t error;
char nodeFrameID[] = "WM02032222";
int accX;
int accY;
int accZ;

void doMeasurements()
{
  // Accelerometer
  ACC.ON();
  accX = ACC.getX();
  accY = ACC.getY();
  accZ = ACC.getZ();
  ACC.OFF();
}

void makeFrame()
{
  frame.setID(nodeFrameID);
  frame.createFrame(ASCII);
  frame.addSensor(SENSOR_STR, (char*) "Sensor string value");
  frame.addSensor(SENSOR_ACC, accX, accY, accZ);

  // DEV Print frame
  frame.showFrame();
}

void setup() 
{
  // DEV Print stuff to the Serial Monitor
  USB.ON();
}

void loop() 
{
  doMeasurements();
  makeFrame();

  // Convert frame to a base64 string(?)
  char sendableString[frame.length*2 + 1];
  Utils.hex2str(frame.buffer, sendableString, frame.length);

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

    // 3. LoRaWAN send unconfirmed packet
    error = LoRaWAN.sendUnconfirmed( PORT, sendableString );

    // Error messages:
    /*
     * '6' : Module hasn't joined a network
     * '5' : Sending error
     * '4' : Error with data length	  
     * '2' : Module didn't response
     * '1' : Module communication error   
     */
    // Check status
    if( error == 0 ) 
    {
      USB.println(F("3. LoRaWAN send unconfirmed packet OK")); 
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

  USB.println();
  // TODO: enter deep sleep (or hibernation) mode to save battery!
  // Remember that it may take time to warm up sensors, so if we want to
  // send say every minute, might just sleep for 40s and warm up for 20s...
  delay(60000);

}



