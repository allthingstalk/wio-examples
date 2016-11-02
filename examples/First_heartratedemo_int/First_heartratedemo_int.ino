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


int heartRateId=0;                                          // identifies the asset on the cloud platform

//the following variables are used for the calculation of the heartrate. These are volatile since they ar globals and used in a function called at interrupt time.
volatile unsigned char counter;
volatile unsigned long temp[21];
volatile unsigned long sub;
volatile bool data_effect=true;
volatile unsigned int heart_rate;                           //the measurement result of heart rate

const int max_heartpluse_duty = 2000;                       //you can change it follow your system's request. 2000 meams 2 seconds. System return error  if the duty overtrip 2 second.
            
//required for the device
void callback(char* topic, byte* payload, unsigned int length);
WiFiClient ethClient;
PubSubClient client(mqttServer, 1883, callback, ethClient);  

/*Function: Initialization for the array(temp)*/
void arrayInit()
{
    for(unsigned char i=0;i < 20;i ++)
        temp[i]=0;
    temp[20]=millis();
}

/*Function: calculate the heart rate*/
void sum()
{
    if(data_effect)
    {
        heart_rate=1200000/(temp[20]-temp[0]);      //60*20*1000/20_total_time 
        Serial.print("Heart_rate_is:\t");
        Serial.println(heart_rate);
    }
    data_effect=1;                                  //sign bit
}

//called at interrup time. Best not to use println in this function
void interrupt()
{
    temp[counter]=millis();
    switch(counter)
    {
        case 0:
            sub=temp[counter]-temp[20];
            break;
        default:
            sub=temp[counter]-temp[counter-1];
        break;
    }
    if(sub>max_heartpluse_duty)                         //set 2 seconds as max heart pluse duty
    {
        data_effect=0;                                  //sign bit
        counter=0;
        arrayInit();
    }
    if (counter==20&&data_effect)
    {
        counter=0;
        sum();
    }
    else if(counter!=20&&data_effect)
        counter++;
    else 
    {
        counter=0;
        data_effect=1;
    }
}

void setup() 
{
    /* the power supply of Grove sockets is controlled by a MOSFET switch which is gated by GPIO 15. 
    * So you must pull up GPIO 15 in your Arduino sketch to power on the Grove system: 
    */
    pinMode(15, OUTPUT);
    digitalWrite(15, 1);

    pinMode(TRIGGER_PIN, INPUT);
  
    Serial.begin(115200);
    Serial.println();
    Serial.println("heartbeat demonstrator!");

    WiFiManager wifiManager;                                    //Local intialization. Once its business is done, there is no need to keep it around
    if ( digitalRead(TRIGGER_PIN) == LOW ) {                    // is configuration portal requested?
        Serial.println("Resetting WiFi settings to factory default");
        wifiManager.resetSettings();
        delay(500);
    } 
    while (!wifiManager.autoConnect("ESP8266Demo")) {
        Serial.println("failed to connect, we should reset as see if it connects");
        delay(3000);
        ESP.reset();
        delay(5000);
    }
        
    //if you get here you have connected to the WiFi
    Serial.println("connected!");
    //assets should be created in the cloud from the UI
    Device.Subscribe(client);                                       // make certain that we can receive message from the iot platform (activate mqtt)
    attachInterrupt(digitalPinToInterrupt(5), interrupt, RISING);   //set interrupt 0,digital port D1. This is called whenever we get a pulse from the heartbeat sensor
}


unsigned int prevheartrate=0;

void loop()
{
    if(heart_rate != 0 && heart_rate != prevheartrate){             //only send the heartbeat when there is a change detected
        Device.Send(String(heart_rate), heartRateId);
        prevheartrate = heart_rate;
    }
    delay(1000);                                                    //at max, send a new value every second. Don't go faster, the cloud does not like this.
    Device.Process();                                               //process incoming messages.
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
    
  }
  if(idOut != NULL)                                     //also let the iot platform know that the operation was succesful: give it some feedback. This also allows the iot to update the GUI's correctly & run scenarios.
    Device.Send(msgString, *idOut);    
}
