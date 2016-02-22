/*  
 *  ------ LoRaWAN Code Example -------- 
 *  
 *  Explanation: This example shows how to configure the module
 *  and send packets to a LoRaWAN gateway without ACK after join a network
 *  using ABP
 *  
 *  Copyright (C) 2016 Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com 
 *  
 *  This program is free software: you can redistribute it and/or modify  
 *  it under the terms of the GNU General Public License as published by  
 *  the Free Software Foundation, either version 3 of the License, or  
 *  (at your option) any later version.  
 *   
 *  This program is distributed in the hope that it will be useful,  
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of  
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the  
 *  GNU General Public License for more details.  
 *   
 *  You should have received a copy of the GNU General Public License  
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.  
 *  
 *  Version:           0.1
 *  Design:            David Gascon 
 *  Implementation:    Luismi Marti  
 */

#include <WaspLoRaWAN.h>

// socket to use
//////////////////////////////////////////////
uint8_t socket = SOCKET0;
//////////////////////////////////////////////

// Device parameters for Back-End registration
////////////////////////////////////////////////////////////
char DEVICE_EUI[]  = "0102030405060708";
char DEVICE_ADDR[] = "02032222";
char NWK_SESSION_KEY[] = "2B7E151628AED2A6ABF7158809CF4F3C";
char APP_SESSION_KEY[] = "2B7E151628AED2A6ABF7158809CF4F3C";
////////////////////////////////////////////////////////////

// Define port to use in Back-End: from 1 to 223
uint8_t PORT = 3;

// Define data payload to send (maximum is up to data rate)
char data[] = "0102030405060708090A0B0C0D0E0F";

// variable
uint8_t error;



void setup() 
{
  USB.ON();
  USB.println(F("LoRaWAN example - Send Unconfirmed packets (no ACK)\n"));


  USB.println(F("------------------------------------"));
  USB.println(F("Module configuration"));
  USB.println(F("------------------------------------\n"));


  //////////////////////////////////////////////
  // 1. Switch on
  //////////////////////////////////////////////

  error = LoRaWAN.ON(socket);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("1. Switch ON OK"));     
  }
  else 
  {
    USB.print(F("1. Switch ON error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 2. Set Device EUI
  //////////////////////////////////////////////

  error = LoRaWAN.setDeviceEUI(DEVICE_EUI);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("2. Device EUI set OK"));     
  }
  else 
  {
    USB.print(F("2. Device EUI set error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 3. Set Device Address
  //////////////////////////////////////////////

  error = LoRaWAN.setDeviceAddr(DEVICE_ADDR);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("3. Device address set OK"));     
  }
  else 
  {
    USB.print(F("3. Device address set error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 4. Set Network Session Key
  //////////////////////////////////////////////

  error = LoRaWAN.setNwkSessionKey(NWK_SESSION_KEY);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("4. Network Session Key set OK"));     
  }
  else 
  {
    USB.print(F("4. Network Session Key set error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 5. Set Application Session Key
  //////////////////////////////////////////////

  error = LoRaWAN.setAppSessionKey(APP_SESSION_KEY);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("5. Application Session Key set OK"));     
  }
  else 
  {
    USB.print(F("5. Application Session Key set error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 6. Save configuration
  //////////////////////////////////////////////

  error = LoRaWAN.saveConfig();

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("6. Save configuration OK"));     
  }
  else 
  {
    USB.print(F("6. Save configuration error = ")); 
    USB.println(error, DEC);
  }


  USB.println(F("\n------------------------------------"));
  USB.println(F("Module configured"));
  USB.println(F("------------------------------------\n"));

  LoRaWAN.getDeviceEUI();
  USB.print(F("Device EUI: "));
  USB.println(LoRaWAN._devEUI);  

  LoRaWAN.getDeviceAddr();
  USB.print(F("Device Address: "));
  USB.println(LoRaWAN._devAddr);  

  USB.println();  
}



void loop() 
{

  //////////////////////////////////////////////
  // 1. Switch on
  //////////////////////////////////////////////

  error = LoRaWAN.ON(socket);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("1. Switch ON OK"));     
  }
  else 
  {
    USB.print(F("1. Switch ON error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 2. Join network
  //////////////////////////////////////////////

  error = LoRaWAN.joinABP();

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("2. Join network OK"));   

    //////////////////////////////////////////////
    // 3. Send unconfirmed packet 
    //////////////////////////////////////////////

    error = LoRaWAN.sendUnconfirmed( PORT, data);

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
      USB.println(F("3. Send Unconfirmed packet OK")); 
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
      USB.print(F("3. Send Unconfirmed packet error = ")); 
      USB.println(error, DEC);
    }
  }
  else 
  {
    USB.print(F("2. Join network error = ")); 
    USB.println(error, DEC);
  }



  //////////////////////////////////////////////
  // 4. Switch off
  //////////////////////////////////////////////

  error = LoRaWAN.OFF(socket);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("4. Switch OFF OK"));     
  }
  else 
  {
    USB.print(F("4. Switch OFF error = ")); 
    USB.println(error, DEC);
  }


  USB.println();
  delay(10000);



}




