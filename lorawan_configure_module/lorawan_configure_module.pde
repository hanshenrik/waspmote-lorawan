/*  
 *  ------ LoRaWAN Code Example -------- 
 *  
 *  Explanation: This example shows how to configure the module
 *  and all general settings related to back-end registration
 *  process.
 *  
 *  Copyright (C) 2015 Libelium Comunicaciones Distribuidas S.L. 
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

//////////////////////////////////////////////
uint8_t socket = SOCKET0;
//////////////////////////////////////////////

// Device parameters for Back-End registration
////////////////////////////////////////////////////////////
char DEVICE_EUI[]  = "0102030405060708";
char DEVICE_ADDR[] = "02032222";
char NWK_SESSION_KEY[] = "2B7E151628AED2A6ABF7158809CF4F3C";
char APP_SESSION_KEY[] = "2B7E151628AED2A6ABF7158809CF4F3C";
char APP_KEY[] = "000102030405060708090A0B0C0D0E0F";
////////////////////////////////////////////////////////////

// variable
uint8_t error;



void setup() 
{
  USB.ON();
  USB.println(F("LoRaWAN example - Module configuration"));


  //////////////////////////////////////////////
  // 1. switch on
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
  // 2. Reset to factory default values
  //////////////////////////////////////////////

  error = LoRaWAN.factoryReset();

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("2. Reset to factory default values OK"));     
  }
  else 
  {
    USB.print(F("2. Reset to factory error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 3. Set/Get Device EUI
  //////////////////////////////////////////////

  // Set Device EUI
  error = LoRaWAN.setDeviceEUI(DEVICE_EUI);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("3.1. Set Device EUI OK"));     
  }
  else 
  {
    USB.print(F("3.1. Set Device EUI error = ")); 
    USB.println(error, DEC);
  }

  // Get Device EUI
  error = LoRaWAN.getDeviceEUI();

  // Check status
  if( error == 0 ) 
  {
    USB.print(F("3.2. Get Device EUI OK. ")); 
    USB.print(F("Device EUI: "));
    USB.println(LoRaWAN._devEUI);
  }
  else 
  {
    USB.print(F("3.2. Get Device EUI error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 4. Set/Get Device Address
  //////////////////////////////////////////////

  // Set Device Address
  error = LoRaWAN.setDeviceAddr(DEVICE_ADDR);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("4.1. Set Device address OK"));     
  }
  else 
  {
    USB.print(F("4.1. Set Device address error = ")); 
    USB.println(error, DEC);
  }
  
  // Get Device Address
  error = LoRaWAN.getDeviceAddr();

  // Check status
  if( error == 0 ) 
  {
    USB.print(F("4.2. Get Device address OK. ")); 
    USB.print(F("Device address: "));
    USB.println(LoRaWAN._devAddr);
  }
  else 
  {
    USB.print(F("4.2. Get Device address error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 5. Set Network Session Key
  //////////////////////////////////////////////
 
  error = LoRaWAN.setNwkSessionKey(NWK_SESSION_KEY);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("5. Set Network Session Key OK"));     
  }
  else 
  {
    USB.print(F("5. Set Network Session Key error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 6. Set Application Session Key
  //////////////////////////////////////////////

  error = LoRaWAN.setAppSessionKey(APP_SESSION_KEY);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("6. Set Application Session Key OK"));     
  }
  else 
  {
    USB.print(F("6. Set Application Session Key error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 7. Set retransmissions for uplink confirmed packet
  //////////////////////////////////////////////

  // set retries
  error = LoRaWAN.setRetries(7);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("7.1. Set Retransmissions for uplink confirmed packet OK"));     
  }
  else 
  {
    USB.print(F("7.1. Set Retransmissions for uplink confirmed packet error = ")); 
    USB.println(error, DEC);
  }
  
  // Get retries
  error = LoRaWAN.getRetries();

  // Check status
  if( error == 0 ) 
  {
    USB.print(F("7.2. Get Retransmissions for uplink confirmed packet OK. ")); 
    USB.print(F("TX retries: "));
    USB.println(LoRaWAN._retries, DEC);
  }
  else 
  {
    USB.print(F("7.2. Get Retransmissions for uplink confirmed packet error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 8. Set application key
  //////////////////////////////////////////////

  error = LoRaWAN.setAppKey(APP_KEY);

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("8. Application key set OK"));     
  }
  else 
  {
    USB.print(F("8. Application key set error = ")); 
    USB.println(error, DEC);
  }


  //////////////////////////////////////////////
  // 9. Save configuration
  //////////////////////////////////////////////
  
  error = LoRaWAN.saveConfig();

  // Check status
  if( error == 0 ) 
  {
    USB.println(F("9. Save configuration OK"));     
  }
  else 
  {
    USB.print(F("9. Save configuration error = ")); 
    USB.println(error, DEC);
  }


  USB.println(F("------------------------------------"));
  USB.println(F("Now the LoRaWAN module is ready for"));
  USB.println(F("joining networks and send messages."));
  USB.println(F("Please check the next examples..."));
  USB.println(F("------------------------------------\n"));

}


void loop() 
{
  // do nothing
}



