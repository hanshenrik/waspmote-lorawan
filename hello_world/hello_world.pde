void setup()
{
  // Opening UART to show messages using 'Serial Monitor'
  USB.ON();
}

void loop()
{
  // Blinking LEDs
  Utils.blinkLEDs(1000);
 
  // Printing a message, remember to open 'Serial Monitor' to be able to see this message
  USB.println(F("Hello World, this is Waspmote!"));
 
  // A little delay
  delay(5000);
}

