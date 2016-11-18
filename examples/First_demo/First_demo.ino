/*
   Copyright 2016 AllThingsTalk

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <PubSubClient.h>
#include <ATT_IOT.h>              //AllThingsTalk IoT library
#include <SPI.h>                  //required to have support for signed/unsigned long type..
#include "keys.h"                 // Keep all your personal account information in a seperate file


//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager


// select wich pin will trigger the configuraton portal when set to LOW
// ESP-01 users please note: the only pins available (0 and 2), are shared 
// with the bootloader, so always set them HIGH at power-up
#define TRIGGER_PIN 2 // (D4)


ATTDevice Device(DEVICEID, CLIENTID, CLIENTKEY);            //create the object that provides the connection to the cloud to manage the device.
char httpServer[] = "api.AllThingsTalk.io";                   // HTTP API Server host                  
char mqttServer[] = "api.AllThingsTalk.io";                // MQTT Server Address 

int buttonPin = 5;  // (D1)                                     

//required for the device
void callback(char* topic, byte* payload, unsigned int length);
WiFiClient ethClient;
PubSubClient pubsub(mqttServer, 1883, callback, ethClient);  


void setup() {
    pinMode(buttonPin, INPUT);                      // initialize the digital pin as an output.

    /* the power supply of Grove sockets is controlled by a MOSFET switch which is gated by GPIO 15. 
    * So you must pull up GPIO 15 in your Arduino sketch to power on the Grove system: 
    */
    pinMode(15, OUTPUT);
    digitalWrite(15, 1);
    
    pinMode(TRIGGER_PIN, INPUT);                    //the trigger pin is used to reset the wifi at boot (press the button to reset)
  
    Serial.begin(115200);
    Serial.println();
    Serial.println("IoT demonstrator!");
    
    WiFiManager wifiManager;                        //Local intialization. Once its business is done, there is no need to keep it around
    if ( digitalRead(TRIGGER_PIN) == LOW ) {        // is configuration portal requested?
        Serial.println("Resetting WiFi settings to factory default");
        wifiManager.resetSettings();
        delay(500);
    }

    while (!wifiManager.autoConnect("ESP8266Demo")) {
        delay(3000);
        ESP.reset();
        delay(5000);
    }
    
    while(!Device.Connect(&ethClient, httpServer))                // connect the device with the IOT platform.
       Serial.println("retrying");
    Device.AddAsset(buttonPin, "button", "a push button", false, "boolean");   // Create the Digital Actuator asset for your device
    while(!Device.Subscribe(pubsub))                              // make certain that we can receive message from the iot platform (activate mqtt)
       Serial.println("retrying");
}
                                           
unsigned int prevVal = 0;
void loop()
{
   unsigned int buttonRead = digitalRead(buttonPin);       // read from light sensor (photocell)
   if(prevVal != buttonRead){
      if(buttonRead == 0)
         Device.Send("false", buttonPin);
       else 
         Device.Send("true", buttonPin);
         prevVal = buttonRead;
       }
   Device.Process(); 
}


// Callback function: handles messages that were sent from the iot platform to this device.
void callback(char* topic, byte* payload, unsigned int length) 
{ 
  String msgString; 
  {                                                     //put this in a sub block, so any unused memory can be freed as soon as possible, required to save mem while sending data
    char message_buff[length + 1];                      //need to copy over the payload so that we can add a /0 terminator, this can then be wrapped inside a string for easy manipulation.
    strncpy(message_buff, (char*)payload, length);      //copy over the data
    message_buff[length] = '\0';                        //make certain that it ends with a null         
          
    msgString = String(message_buff);
    msgString.toLowerCase();                            //to make certain that our comparison later on works ok (it could be that a 'True' or 'False' was sent)
  }
  int* idOut = NULL;
  {                                                     //put this in a sub block, so any unused memory can be freed as soon as possible, required to save mem while sending data
    int pinNr = Device.GetPinNr(topic, strlen(topic));

    
    Serial.print("Payload: ");                          //show some debugging.
    Serial.println(msgString);
    Serial.print("topic: ");
    Serial.println(topic);
    /*
    if (pinNr == led_TopicID)       
    {
      if (msgString == "false") {
        digitalWrite(ledPin, LOW);                      //change the led   
        idOut = &ledPin;                                
      }
      else if (msgString == "true") {
        digitalWrite(ledPin, HIGH);
        idOut = &led_TopicID;
      }
    }*/
  }
  if(idOut != NULL)                                     //also let the iot platform know that the operation was succesful: give it some feedback. This also allows the iot to update the GUI's correctly & run scenarios.
    Device.Send(msgString, *idOut);    
}
