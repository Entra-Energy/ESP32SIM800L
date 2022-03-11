/*
 * Project SM_SIM800L_ESP32
 * Description:
 * Author:
 * Date:09.03.2022
 */



// Please select the corresponding model
#define SIM800L_IP5306_VERSION_20190610
#define TINY_GSM_MODEM_SIM800
#include <ArduinoJson.h>
#include <Wire.h>
#include <TinyGsmClient.h>

#include "EmonLib.h"

#include <vector>
#include <sstream>
#include <locale>
#include <iomanip>
#include <map>
#include <string>
#include <cstdio>
#include <algorithm>

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

#define TINY_GSM_DEBUG SerialMon

// Your GPRS credentials, if any
const char apn[] = "internet.vivacom.bg"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "";
const char gprsPass[] = "";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 

// set GSM PIN, if any
#define GSM_PIN ""

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#include <PubSubClient.h>

TinyGsmClient client(modem);
PubSubClient mqtt(client);


uint32_t lastReconnectAttempt = 0;

// MQTT details
const char* broker = "159.89.103.242";                    // Public IP address or domain name
const char* mqttUsername = "REPLACE_WITH_YOUR_MQTT_USER";  // MQTT username
const char* mqttPassword = "REPLACE_WITH_YOUR_MQTT_PASS";  // MQTT password


int ver = 3;
EnergyMonitor emon1;  
EnergyMonitor emon2;
EnergyMonitor emon3; 
float blynkPublish;
float one_min_measure;
float accumulatePow;
int nextPeriod;
float average;
long interval = 2000;
long previousMillis = 0; 

float hourBegining;
float hourConsumption;
float dayBegining;
float dayConsumption;
float monthBegining;
float monthConsumption;

float currPriceH;
float currPriceD;
float currPriceM;
float currPrice;

long currentMillisSend = 0;
long previousMillisSend = 0;
long intervalSend = 5000;
long intervalSendBlynk = 60000;
long previousMillisSendBlynk = 0;
int reportFreq;

int currMin;
int currHour;
int currDay;
int currMonth;
int blynkGridButton;
int16_t currYear;
int16_t prevYear;
uint8_t prevMin;
uint8_t prevDay;
uint8_t prevHour;
uint8_t prevMonth;

#define NUMSAMPLES 5
float tenSec[NUMSAMPLES];


float sixty[60];
float fifteen[15];

float avTenSec = 0;

std::map<String, float>powerMap; 
std::vector< float > measurements;



int trigger = 0;
char data[80]; 

double Irms1; 
double Irms2;
double Irms3; 

int j;

String stampRecieved;

StaticJsonDocument<200>parser; 

const char* deviceID = "sm-0009";


const char* timestampStr = "/timestamp";

String timestampReceived = String(deviceID)+String(timestampStr);

const char* timestampRe = timestampReceived.c_str();

String calibrationKoeff;
const char* cal = "/calibrate";
String calKoeff = String(deviceID)+String(cal);
const char* calibration = calKoeff.c_str();


String adj;
const char* adjust = "/adjustment";
String adjMeasurement = String(deviceID)+String(adjust);
const char* adjTopic = adjMeasurement.c_str();


const char* dashCh = "dash/";
const char* currP = "/currpower/currpower";
String pingDev = String(dashCh)+String(deviceID)+String(currP);
const char* ping = pingDev.c_str();


const char* dataChar = "data/";
const char* fifteenToChar = "/fifteen";
String dataSend = String(dataChar)+String(deviceID)+String(fifteenToChar);
const char* sendFifteen = dataSend.c_str();

const char* er = "error/check/";
String error = String(er)+String(deviceID);
const char* errorSendDash = error.c_str();


// recieve message
void mqttCallback(char* topic, byte* message, unsigned int len) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String p = "";
    for (int i=0;i<len;i++)
    {
        p += (char)message[i];        
    }      
    Serial.println();    
    //test topic
    if (String(topic) == "RED")
    {       
        mqtt.publish("size_plusOnemin", "ECHO ECHO");       
    }

    if (String(topic) == String(deviceID)+"/calibrate")
    {       
        calibrationKoeff = String(p);   
    }
    if (String(topic) == String(deviceID)+"/adjustment")
    {       
        adj = String(p);   
    }


    if (String(topic) == String(deviceID)+"/timestamp")
    {
        stampRecieved = String(p);
        DeserializationError error = deserializeJson(parser, stampRecieved);
        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.f_str());
          return;
        }
        String strValue = parser["time"];;
        float powValue = parser["pow"];
        mqtt.publish("parserPow", (char*) strValue.c_str());
        std::map<String,float>::iterator it;
        it = powerMap.find(strValue);
        if (it != powerMap.end())
        {
            powerMap.erase (it);  
            trigger = 1;   
        }              
    }
}



boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker without username and password
  //boolean status = mqtt.connect("GsmClientN");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect(deviceID);

  if (status == false) {
    SerialMon.println(" fail");
    ESP.restart();
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe("meter/ibexIn");
  //mqtt.subscribe(blynkAuthChar);
  mqtt.subscribe(timestampRe);
  mqtt.subscribe("RED");
//  mqtt.publish(initial,deviceID);
  return mqtt.connected();
}


void setup() { 
  SerialMon.begin(115200);
  delay(10);
  
pinMode(4, OUTPUT);
pinMode(5, OUTPUT);
pinMode(23, OUTPUT);

digitalWrite(4, LOW);
digitalWrite(5, HIGH);
digitalWrite(23, HIGH);

  // Set modem reset, enable, power pins
  SerialAT.begin(115200, SERIAL_8N1, 26, 27); 
  SerialMon.println("Wait...");
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  //modem.restart();
  modem.init();

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }


  

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }


  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }
  
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
    timeConvert();
    prevHour = currHour;
    prevDay = currDay;
    prevMonth = currMonth;
    nextPeriod = currMin + 2;
  }

  
      // MQTT Broker setup
    mqtt.setServer(broker, 1883);
    mqtt.setCallback(mqttCallback);


    emon1.current(12, 90.9); //22 ohm 90.9
    emon2.current(13, 90.9);
    emon3.current(15, 90.9);
    analogReadResolution(ADC_BITS);
    calibrationKoeff = "1";
    adj= "1";
 
    if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
    }    
    powerMap.clear();
   
}


void blynkUpdate()
{
      
      char blynkPub[8];
      
      dtostrf(blynkPublish, 1, 2, blynkPub);
      mqtt.publish(ping,blynkPub);
      //Serial.println(blynkPublish);

      
}

 void measure()
{ 
    
  
    unsigned long currentMillis = millis();  
    
    if(currentMillis - previousMillis > interval)
    { 
        Irms1 = emon1.calcIrms(5500);
       
        Irms2 = emon2.calcIrms(5500);
        Irms3 = emon3.calcIrms(5500);
        double Irms = Irms1 + Irms2 + Irms3;
        double power = Irms*230;  
        
        tenSec[j] = power;         
        j++;        
            if (j > 4)
            {
                for (int k=0; k < 5; k++) 
                {          
                    average += tenSec[k];           
                }          
                average /= 5;
                
                float devCalibrate = stof(calibrationKoeff);
                float devAdj = stof(adj);
                blynkPublish = average*devCalibrate+devAdj;
                measurements.push_back(blynkPublish);//add every 10s                
                average = 0;
                j = 0;
            }          

        previousMillis = currentMillis;
    }   
}


 
void timeConvert (){

String time = modem.getGSMDateTime(DATE_FULL);
int splitT = time.indexOf(",");
int splitY = time.indexOf("/");
String timeStamp = time.substring(splitT+1, time.length());
String timeStampY = time.substring(splitY, 0);
currYear = timeStampY.toInt();
String currHourStr = timeStamp.substring(0,2);
currHour = currHourStr.toInt();

int splitM = timeStamp.indexOf(":");
String timeStampM = timeStamp.substring(splitM+1, timeStamp.length());
String currMinString = timeStampM.substring(0,2);
currMin = currMinString.toInt();
String timeStampMonth = time.substring(splitY+1, time.length());
String currMonthString = timeStampMonth.substring(0,2);
currMonth = currMonthString.toInt();
int splitDay = timeStampMonth.indexOf("/");
String timeStampDay = timeStampMonth.substring(splitDay+1, timeStampMonth.length());
String currDayString = timeStampDay.substring(0,2);
currDay = currDayString.toInt();

}



  String timeGet(struct tm t)
  {

    t = {0};
    t.tm_year = (currYear+2000) - 1900;
    t.tm_mon = currMonth - 1;
    t.tm_mday = currDay;
    t.tm_hour = currHour;
    t.tm_min = currMin;
    t.tm_sec = 00;
    time_t timeSinceEpoch = mktime(&t);    
    int stamp = int(timeSinceEpoch);
    String timestamp = String(stamp);
    return timestamp;    
  } 

  struct timestampPower { 
     void myFunc()
    {
        for (std::map<String, float>::iterator it = powerMap.begin(); it != powerMap.end(); ++it) 
        {
            String timeSt = "\"timestamp\": " + String((*it).first);
            String powerSt = "\"power\": " + String((*it).second);
            String jObj = timeSt +','+ powerSt;
            String payload = "{ \"payload\": {" + jObj + "}}";
            payload.toCharArray(data, (payload.length() + 1));
            Serial.println("++++++++++");
            //Serial.println(data);
            mqtt.publish(sendFifteen, data);  
        }
    }  

  void checkDb(){
  
    if(!powerMap.empty())
    {
    String mapKey = (--powerMap.end())->first;
    float mapValue = (--powerMap.end())->second;
    String timeSt = "\"timestamp\": " + mapKey;
    String powerSt = "\"power\": " + String(mapValue);
    String jObj = timeSt +','+ powerSt;
    String payload = "{ \"payload\": {" + jObj + "}}";
    payload.toCharArray(data, (payload.length() + 1));
    mqtt.publish(errorSendDash, data);
    }

}
}tp;



void loop() {

  
      timeConvert();
      if (!mqtt.connected()) {
        SerialMon.println("=== MQTT NOT CONNECTED ===");
        // Reconnect every 10 seconds
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L) {
          lastReconnectAttempt = t;
        if (mqttConnect()) {
          lastReconnectAttempt = 0;
        }
      }
        delay(100);
        return;
      }  
       
    unsigned long currentMillisSend = millis();  
  
    if(currentMillisSend - previousMillisSend > intervalSend)
    {           
      
        if (trigger == 1 && powerMap.size() > 0)
        {
     
            tp.checkDb();
   
            trigger = 0;
     
        }
        previousMillisSend = currentMillisSend;
    }   

    measure();  

    if(currentMillisSend - previousMillisSendBlynk > intervalSendBlynk)
    {        
        blynkUpdate();
        previousMillisSendBlynk = currentMillisSend;
    } 
     

    if (currMin != prevMin)   
    {     
        one_min_measure = accumulate( measurements.begin(), measurements.end(), 0.0)/measurements.size();
        measurements.clear();
        struct tm t0;
        String timestamp = timeGet(t0); 
        if (!isnan(one_min_measure) && !isinf(one_min_measure))
        {
            powerMap.insert(std::pair<String, float>(timestamp,one_min_measure));
            tp.myFunc();
        }
        
        if (currMin == 0)
        { 
          sixty[59] = one_min_measure;
        }
        else
        {
          sixty[currMin-1] = one_min_measure;
        }    
  
        prevMin = currMin;  
        if (nextPeriod >= 60 )
        {
            nextPeriod = 0;
        }
        Serial.println(nextPeriod); 
    }

    if (currMin == nextPeriod)
        {
            Serial.println("xxxyyy");
            int err_size = powerMap.size();
            String err_string = String(err_size);
                       
            mqtt.publish("boron", (char*) err_string.c_str());

            if (err_size > 100)
            {
              powerMap.clear();
            }
                             
            tp.checkDb();
            trigger = 1;
            nextPeriod += 2;
        }  
        

    // if (currHour != prevHour)
    // {
       
    //     hourBegining = accumulatePow;
    //     currPriceH = 0;
    //     prevHour = currHour;
    // }
    // if (prevDay != currDay)
    // {
    //     dayBegining = accumulatePow;
    //     currPriceD = 0;
    //     prevDay = currDay;
    // }
    // if (prevMonth != currMonth)
    // {
    //     monthBegining = accumulatePow;
    //     currPriceM = 0;
    //     prevMonth = currMonth;
    // }
      

    mqtt.loop();
    
       
}
