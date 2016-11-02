#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <PubSubClient.h>
#include <ATT_IOT.h>              //AllThingsTalk IoT library
#include <SPI.h>                  //required to have support for signed/unsigned long type..
#include "keys.h"                 // Keep all your personal account information in a seperate file
#include "Time.h"

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager


// select wich pin will trigger the configuraton portal when set to LOW
// ESP-01 users please note: the only pins available (0 and 2), are shared 
// with the bootloader, so always set them HIGH at power-up
#define TRIGGER_PIN 2 // (D4)


ATTDevice Device(DEVICEID, CLIENTID, CLIENTKEY);            //create the object that provides the connection to the cloud to manage the device.
char httpServer[] = "api.smartliving.io";                   // HTTP API Server host                  
char mqttServer[] = "broker.smartliving.io";                // MQTT Server Address 


int knobPin = A0;                                      // Analog 0 is the input pin
int buttonPin = 3;                                       
int button_TopicID= 5;                                    // identifies the asset on the cloud platform
int knob_TopicID=0;                                    // identifies the asset on the cloud platform

//required for the device
void callback(char* topic, byte* payload, unsigned int length);
WiFiClient ethClient;
PubSubClient client(mqttServer, 1883, callback, ethClient);  


void setup() {
    pinMode(buttonPin, INPUT);                      // initialize the digital pin as an output.
    pinMode(knobPin, INPUT);                        // initialize the Analog pin as an intput.

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
    
    //On nodemcu, creating assets from the device takes too long for the built in watchdog. This can give startup issues. It will usually work eventually, but the process takes very long.
    //best to create the assets from the platform (manually or from a template)
        
    Device.Subscribe(client);                               // make certain that we can receive message from the iot platform (activate mqtt)
}

unsigned long timer;                                            //only send every x amount of time.
unsigned int prevVal = 0;
unsigned int prevVal2 = 0;
void loop()
{
    unsigned long curTime = millis();
    if (curTime > (timer + 1000))                               // publish light reading every 5 seconds to sensor 1
    {
        unsigned int knobRead = analogRead(knobPin);            // read from light sensor (photocell)
        if(prevVal != knobRead){
            Device.Send(String(knobRead), knob_TopicID);
            prevVal = knobRead;
        }
        unsigned int buttonRead = digitalRead(buttonPin);       // read from light sensor (photocell)
        if(prevVal2 != buttonRead){
            if(buttonRead == 0)
                Device.Send("false", button_TopicID);
            else 
                Device.Send("true", button_TopicID);
            prevVal2 = buttonRead;
        }
        timer = curTime;
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
