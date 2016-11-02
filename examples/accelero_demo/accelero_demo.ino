#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <PubSubClient.h>
#include <ATT_IOT.h>              //AllThingsTalk IoT library
#include <SPI.h>                  //required to have support for signed/unsigned long type..
#include "Time.h"
#include "keys.h"

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9250.h"


// select wich pin will trigger the configuraton portal when set to LOW
// ESP-01 users please note: the only pins available (0 and 2), are shared 
// with the bootloader, so always set them HIGH at power-up
#define TRIGGER_PIN 2 // (D4)


ATTDevice Device(DEVICEID, CLIENTID, CLIENTKEY);            //create the object that provides the connection to the cloud to manage the device.
char httpServer[] = "api.smartliving.io";                   // HTTP API Server host                  
char mqttServer[] = "broker.smartliving.io";                // MQTT Server Address 

//asset ids
#define accelerationId 1
#define gyroId 2
#define magneticId 3
#define headingId 4
#define tiltHeadingId 5


MPU9250 accelgyro;
I2Cdev I2C_M;

float heading;
float tiltheading;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];


#define sample_num_mdate  5000      

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;

volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;

//required for the device
void callback(char* topic, byte* payload, unsigned int length);
WiFiClient ethClient;
PubSubClient client(mqttServer, 1883, callback, ethClient);  // 


void setup() {
    /* the power supply of Grove sockets is controlled by a MOSFET switch which is gated by GPIO 15. 
    * So you must pull up GPIO 15 in your Arduino sketch to power on the Grove system: 
    */
    pinMode(15, OUTPUT);
    digitalWrite(15, 1);
    pinMode(TRIGGER_PIN, INPUT);

    Serial.begin(115200);
    Serial.println();
    Serial.println("accelerometer demonstrator!");
    
    Wire.begin();                                               // join I2C bus (I2Cdev library doesn't do this automatically)
    accelgyro.initialize();                                     // initialize sensor        
    
    //Mxyz_init_calibrated ();                                  //in case we want to calibrate it (requires moving the device around) Best would be to do this 1 time, then store the results in eeprom
        
    WiFiManager wifiManager;
    if ( digitalRead(TRIGGER_PIN) == LOW ) {                    // is configuration portal requested?
        Serial.println("Resetting WiFi settings to factory default");
        wifiManager.resetSettings();
        delay(500);
    } 
    while (!wifiManager.autoConnect("ESP8266Demo")) {           //try to connect to with the currently stored setings, if this fails, go into wifi config mode  
        delay(3000);
        ESP.reset();
        delay(5000);
    }
    //On nodemcu, creating assets from the device takes too long for the built in watchdog. This can give startup issues. It will usually work eventually, but the process takes very long.
    //best to create the assets from the platform (manually or from a template)
    
    Device.Subscribe(client);
    
    /* for creating assets from device:
    if(Device.Connect(&ethClient, httpServer))                       // connect the device with the IOT platform.
    {
        Device.AddAsset(headingId, "heading", "The clockwise angle between the magnetic north and X-Axis", false, "number");
        Device.AddAsset(tiltHeadingId, "tilt heading", "The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane", false, "number");
        Device.AddAsset(accelerationId, "acceleration", "acceleration", false, "{\"type\": \"object\",\"properties\": {\"x\": { \"type\": \"number\" },\"y\": { \"type\": \"number\" },\"z\": { \"type\": \"number\" }}}"); 
        Device.AddAsset(gyroId, "gyroscope", "gyroscope", false, "{\"type\": \"object\",\"properties\": {\"x\": { \"type\": \"number\" },\"y\": { \"type\": \"number\" },\"z\": { \"type\": \"number\" }}}");
        Device.AddAsset(magneticId, "magneto meter", "magneto meter", false, "{\"type\": \"object\",\"properties\": {\"x\": { \"type\": \"number\" },\"y\": { \"type\": \"number\" },\"z\": { \"type\": \"number\" }}}");
        Device.Subscribe(client);                               // make certain that we can receive message from the iot platform (activate mqtt)
    }
    else 
       while(true); */
    
}

unsigned long timer;                                            //only send every x amount of time.

void loop()
{
    unsigned long curTime = millis();
    if (curTime > (timer + 1000))                               // publish light reading every 5 seconds to sensor 1
    {
        getAccel_Data();
        Serial.println("get accelero done");
        getGyro_Data();
        Serial.println("get gyro done");
        getCompassDate_calibrated();                            // compass data should be calibrated. Turned off by default (see setup)
        Serial.println("get compas calibrated done");
        getHeading();                                           //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .                    
        getTiltHeading();           
        printAccelero();
        sendAcceleroData();
        timer = curTime;
    }
    Device.Process(); 
}

void printAccelero()
{
    Serial.print("calibration parameter: ");
    Serial.print(mx_centre);
    Serial.print(", ");
    Serial.print(my_centre);
    Serial.print(", ");
    Serial.println(mz_centre);
    
    
    Serial.print("Acceleration(g) of X,Y,Z: ");
    Serial.print(Axyz[0]); 
    Serial.print(",");
    Serial.print(Axyz[1]); 
    Serial.print(",");
    Serial.println(Axyz[2]); 
    Serial.print("Gyro(degress/s) of X,Y,Z::");
    Serial.print(Gxyz[0]); 
    Serial.print(",");
    Serial.print(Gxyz[1]); 
    Serial.print(",");
    Serial.println(Gxyz[2]); 
    Serial.print("Compass Value of X,Y,Z::");
    Serial.print(Mxyz[0]); 
    Serial.print(",");
    Serial.print(Mxyz[1]); 
    Serial.print(",");
    Serial.println(Mxyz[2]);
    Serial.print("The clockwise angle between the magnetic north and X-Axis: ");
    Serial.print(heading);
    Serial.println(" ");
    Serial.print("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane: ");
    Serial.println(tiltheading);
    Serial.println("   ");
}

void sendAcceleroData()
{
    int length = 40;
    char* message_buff = new char[length];
    
    sprintf(message_buff, "{\"x\":%s,\"y\":%s,\"z\":%s}\0", String(Axyz[0]).c_str(), String(Axyz[1]).c_str(), String(Axyz[2]).c_str());  //no support for float sprintf on nodemcu.
    Device.Send(String(message_buff), accelerationId);                       
    
    sprintf(message_buff, "{\"x\":%s,\"y\":%s,\"z\":%s}\0", String(Gxyz[0]).c_str(), String(Gxyz[1]).c_str(), String(Gxyz[2]).c_str());
    Device.Send(String(message_buff), gyroId);                       
    
    sprintf(message_buff, "{\"x\":%s,\"y\":%s,\"z\":%s}\0", String(Mxyz[0]).c_str(), String(Mxyz[1]).c_str(), String(Mxyz[2]).c_str());
    Device.Send(String(message_buff), magneticId);                       
    
    Device.Send(String(heading), headingId);  
    Device.Send(String(tiltheading), tiltHeadingId);  
    
    delete message_buff;
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


///////////////////////////////////////////////////////////////////////////////////////////
/// accelero functions
///////////////////////////////////////////////////////////////////////////////////////////

void getHeading(void)
{
    heading=180*atan2(Mxyz[1],Mxyz[0])/PI;
    if(heading <0) heading +=360;
}

void getTiltHeading(void)
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1]/cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh)/PI;
    if(yh<0)    tiltheading +=360;
}



void Mxyz_init_calibrated ()
{
    
    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while(!Serial.find("ready"));   
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");
    
    get_calibration_Data ();
    
    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}


void get_calibration_Data ()
{
    for (int i=0; i<sample_num_mdate;i++)
    {
        get_one_sample_date_mxyz();
        /*
        Serial.print(mx_sample[2]);
        Serial.print(" ");
        Serial.print(my_sample[2]);                            //you can see the sample data here .
        Serial.print(" ");
        Serial.println(mz_sample[2]);
        */
            
        if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];         
        if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value            
        if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];     
            
        if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
        if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];
                        
    }
            
    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];          
                
    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];
    

    
    mx_centre = (mx_max + mx_min)/2;
    my_centre = (my_max + my_min)/2;
    mz_centre = (mz_max + mz_min)/2;    
}

void get_one_sample_date_mxyz()
{       
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}   


void getAccel_Data(void)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t   mx, my, mz;
    
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;//16384  LSB/g
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384; 
}

void getGyro_Data(void)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t   mx, my, mz;
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;//131 LSB(бу/s)
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
    int16_t   mx, my, mz;
    uint8_t buffer_m[6];
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
    
    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;  
    
    //Mxyz[0] = (double) mx * 1200 / 4096;
    //Mxyz[1] = (double) my * 1200 / 4096;
    //Mxyz[2] = (double) mz * 1200 / 4096;
    Mxyz[0] = (double) mx * 4800 / 8192;
    Mxyz[1] = (double) my * 4800 / 8192;
    Mxyz[2] = (double) mz * 4800 / 8192;
}

void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;  
}

