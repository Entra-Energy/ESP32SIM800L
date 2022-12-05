/*
 * Project SM_SIM800L_ESP32
 * Description:
 * Author: Georgi Manev
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


const char* deviceID = "sm-0016";


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
//const char* broker = "10.236.108.29";  
const char* broker = "159.89.103.242";                   // Public IP address or domain name
const char* mqttUsername = "REPLACE_WITH_YOUR_MQTT_USER";  // MQTT username
const char* mqttPassword = "REPLACE_WITH_YOUR_MQTT_PASS";  // MQTT password

String reset;
int init_delay = 1;
int start_measure = 0;

int cali = 1;
int ver = 3;
int timer = 0;

int flexiDate;
int flexiDuration;
int flexiPow;
int csq;

String initial = "init";

String latitude;
String longitude;


float powerCorrection;
int providing;
EnergyMonitor emon1;  
EnergyMonitor emon2;
EnergyMonitor emon3; 
float blynkPublish;
float one_min_measure;

float transf1;
float transf2;
float transf3;

int nextPeriod;
float average;
long interval = 2000;
long previousMillis = 0; 

float accumulatePowH;
float accumulatePowD;
float accumulatePowM;
float hourBegining;
float hourConsumption;
float dayBegining;
float dayConsumption;
float monthBegining;
float monthConsumption;

float costPerHour;
float costPerDay;
float costPerMonth;

float currPriceH;
float currPriceD;
float currPriceM;
float currPrice = 0;

long currentMillisSend = 0;
long previousMillisSend = 0;
long previousMillisMeasureDelay = 0;
long intervalSend = 5000;
long intervalSendBlynk = 10000;
long previousMillisSendBlynk = 0;
long previousMillisCorrection = 0;
int reportFreq;

int currSec;
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
uint8_t prevSec;

#define NUMSAMPLES 5
float tenSec[NUMSAMPLES];


std::map<String, float>powerMap; 
std::map<String, float>pingMap;
std::vector< float > measurements;
std::vector< float > measure_delay;


int trigger = 0;
char data[480]; 
char pingData[180];
char transData[80];

char blynk[80];

double Irms1; 
double Irms2;
double Irms3; 

int j;

String stampRecieved;
String corrRecieved;
String flexReq;

String initialRecv;
int supportReady = 0;
int hourlyBudget = 0;
int dailyBudget = 0;
int monthlyBudget = 0;

String blynkName = "";



StaticJsonDocument<200>parser; 
StaticJsonDocument<200>correct_parser;
StaticJsonDocument<400>flexi_parser;
StaticJsonDocument<600>init_parser;

const char* initialStr = "initial/";
String initialReceived = String(initialStr)+String(deviceID);
const char* initialRe = initialReceived.c_str();

//Topic to receive data from db (for db robustness)
const char* timestampStr = "/timestamp";
String timestampReceived = String(deviceID)+String(timestampStr);
const char* timestampRe = timestampReceived.c_str();

//Topic for reset calibration
const char* resetStr = "/reset";
String resetReceived = String(deviceID)+String(resetStr);
const char* resetRe = resetReceived.c_str();

//Topic for time correction
const char* correctionStr = "/correction";
String correctionReceived = String(deviceID)+String(correctionStr);
const char* correctionRe = correctionReceived.c_str();

//Grid support Receiving
const char* gridReadyStr = "/ready";
String readyReceived = String(deviceID)+String(gridReadyStr);
const char* readyRe = readyReceived.c_str();

//Price Receiving
// const char* priceStr = "/price";
// String priceReceived = String(deviceID)+String(priceStr);
// const char* priceRe = priceReceived.c_str();


//hourlyBudget Receiving
const char* subscrHourBudget = "/hourBudget";
String hourBudgetReceived = String(deviceID)+String(subscrHourBudget);
const char* hourlyRe = hourBudgetReceived.c_str();
//dailyBudget Receiving
const char* subscrDayBudget = "/dayBudget";
String dayBudgetReceived = String(deviceID)+String(subscrDayBudget);
const char* dayRe = dayBudgetReceived.c_str();

//monthlyBudget Receiving
const char* subscrMonthBudget = "/monthBudget";
String monthBudgetReceived = String(deviceID)+String(subscrMonthBudget);
const char* monthRe = monthBudgetReceived.c_str();

const char* devBlynkName = "/name";
String topicBlynkName = String(deviceID) + String(devBlynkName);
const char* subscribeBlynkName = topicBlynkName.c_str();


// Topic for flexi recieving/sending
const char* flexiStr = "/flexi";
String flexiReceived = String(deviceID)+String(flexiStr);
const char* flexRe = flexiReceived.c_str();

const char* flexiResponse = "flexiResponse/";
String responseDev = String(flexiResponse)+String(deviceID);
const char* flexiResMsg = responseDev.c_str();

const char* corrResponse = "corrResponse/";
String corrResponseDev = String(corrResponse)+String(deviceID);
const char* corrResMsg = corrResponseDev.c_str();


//Topic to receive calibration
String calibrationKoeff;
const char* cal = "cali/";
String calKoeffTopic = String(cal) + String(deviceID);
const char* calibration = calKoeffTopic.c_str();
float cali_koeff = 1;

String adj;
const char* adjust = "/adjustment";
String adjMeasurement = String(deviceID)+String(adjust);
const char* adjTopic = adjMeasurement.c_str();

//Topic for update status and dash table every 5s
const char* dashCh = "ping/";
String pingDev = String(dashCh)+String(deviceID);
const char* ping = pingDev.c_str();

//Topic for individual transformers data
// const char* transformers = "trans/";
// String pingTransf = String(transformers)+String(deviceID);
// const char* tr = pingTransf.c_str();

//Topic for send data

const char* init_dev = "init/";
String initialize = String(init_dev)+String(deviceID);
const char* init_topic = initialize.c_str();


const char* dataChar = "data/";
const char* fifteenToChar = "/fifteen";
String dataSend = String(dataChar)+String(deviceID)+String(fifteenToChar);
const char* sendFifteen = dataSend.c_str();

//Topic for send data for db check
const char* er = "error/check/";
String error = String(er)+String(deviceID);
const char* errorSendDash = error.c_str();

//Blynk related topics for sending the consumption
const char* blh = "blynkHourConsumption/";
String blynkh = String(blh)+String(deviceID);
const char* blynkHourCons = blynkh.c_str();

const char* bld = "blynkDayConsumption/";
String blynkd = String(bld)+String(deviceID);
const char* blynkDayCons = blynkd.c_str();

const char* blM = "blynkMonthConsumption/";
String blynkm = String(blM)+String(deviceID);
const char* blynkMonthCons = blynkm.c_str();

//Blynk related topics for budgets
const char* hBudget = "blynkHourBudget/";
String blynkHBudget = String(hBudget)+String(deviceID);
const char* hourly = blynkHBudget.c_str();

const char* dBudget = "blynkDayBudget/";
String blynkDBudget = String(dBudget)+String(deviceID);
const char* daily = blynkDBudget.c_str();

const char* mBudget = "blynkMonthBudget/";
String blynkMBudget = String(mBudget)+String(deviceID);
const char* monthly = blynkMBudget.c_str();

//hour,day,month costs
const char* hourCost = "blynkHourCost/";
String blynkHCost = String(hourCost)+String(deviceID);
const char* hCost = blynkHCost.c_str();

const char* dayCost = "blynkDayCost/";
String blynkDCost = String(dayCost)+String(deviceID);
const char* dCost = blynkDCost.c_str();

const char* monthCost = "blynkMonthCost/";
String blynkMCost = String(monthCost)+String(deviceID);
const char* mCost = blynkMCost.c_str();




void get_location(){    

    float lat = 0;
    float lon = 0;
    for (int8_t i = 2; i; i--)
    {        
        
        if (modem.getGsmLocation(&lon, &lat))
        {         
          latitude = String(lat, 8);             
          longitude = String(lon, 8);         
          break;
        }
        else{
           String nul = "null";
           latitude = '"'+nul+'"';
           longitude = '"'+nul+'"';
        }
    }


}



//callback function for receiving mqtt data on different topics
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
  //system topic
  if (String(topic) == "RED")
  {       
      mqtt.publish("size_plusOnemin", "ECHO ECHO");               
  }

  if (String(topic) == calibration)
  {       
    calibrationKoeff = String(p);          
  }
  
  if (String(topic) == String(deviceID)+"/adjustment")
  {       
      adj = String(p);   
  }

  //receive data from db and check if exist into device "array" (db robustness related)
  if (String(topic) == String(deviceID)+"/timestamp")
  {
    stampRecieved = String(p);
    DeserializationError error = deserializeJson(parser, stampRecieved);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    String strValue = parser["time"];
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
  //if received power correction and period for that correction
  if (String(topic) == String(deviceID)+"/correction")
  {
    corrRecieved = String(p);
    DeserializationError error = deserializeJson(correct_parser, corrRecieved);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    powerCorrection = correct_parser["power"];      
    timer = correct_parser["timer"]; 
    providing = 1;

    String power_corr = "\"power\": " + String(powerCorrection);
    String time_corr = "\"period\": " + String(timer);  
    String jObj = power_corr +','+ time_corr;  
    String payload = "{ \"payload\": {" + jObj + "}}";
    char corrRespondObject[80];
    payload.toCharArray(corrRespondObject, (payload.length() + 1));
    Serial.println(corrRespondObject);
    mqtt.publish(corrResMsg, corrRespondObject);

  }

  if (String(topic) == String(deviceID)+"/flexi")
  {
    flexReq = String(p);
    Serial.print("received");
    DeserializationError error = deserializeJson(flexi_parser, flexReq);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
         
    flexiDate = flexi_parser["date"];
    flexiDuration = flexi_parser["duration"];
    flexiPow =  flexi_parser["pow"]; 

    String flex_time = "\"date\": " + String(flexiDate);
    String flex_power = "\"power\": " + String(flexiPow);    
    String flex_dur = "\"duration\": " + String(flexiDuration);
    String jObj = flex_time +','+ flex_power + ',' + flex_dur;
    String payload = "{ \"payload\": {" + jObj + "}}";
    char flexiRespondObject[80];
    payload.toCharArray(flexiRespondObject, (payload.length() + 1));
    Serial.println(flexiRespondObject);
    mqtt.publish(flexiResMsg, flexiRespondObject);

  }

  if (String(topic) == "initial/"+ String(deviceID))
  {
    initialRecv = String(p);
    DeserializationError error = deserializeJson(init_parser, initialRecv);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    accumulatePowH = init_parser["for_hour"];
    accumulatePowD = init_parser["for_today"];
    accumulatePowM = init_parser["for_month"];
    hourlyBudget = init_parser["budgetH"];
    dailyBudget = init_parser["budgetD"];
    monthlyBudget = init_parser["budgetM"];
    supportReady = init_parser["ready"];
  }

  if (String(topic) == String(deviceID)+"/ready")
  {
   
    String ready = String(p);
    supportReady = ready.toInt();    
  }

  if (String(topic) == String(deviceID)+"/hourBudget")
  {   
    String hourBudget = String(p);
    hourlyBudget = hourBudget.toInt();    
  }
  if (String(topic) == String(deviceID)+"/dayBudget")
  {   
    String dayBudget = String(p);
    dailyBudget = dayBudget.toInt();    
  }
  if (String(topic) == String(deviceID)+"/monthBudget")
  {   
    String monthBudget = String(p);
    monthlyBudget = monthBudget.toInt();    
  }

  if (String(topic) == "price")
  {   
    String price = String(p);
    currPrice = price.toFloat();    
  }

  if (String(topic) == String(deviceID)+"/name")
  {   
    String devName = String(p);
    blynkName = devName;  
  }



  //if calibration reset is received
  if (String(topic) == String(deviceID)+"/reset")
  {
    reset = String(p);
    Serial.print("RESET!!");
    if (reset)
    {
      cali_koeff = 1;
    }
  }
}


// initialize and check mqtt connection, subscribe to topics
boolean mqttConnect() 
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);
  boolean status = mqtt.connect(deviceID);
  if (status == false) {
    SerialMon.println("fail");
    ESP.restart();
    return false;
  }
  SerialMon.println("success");
  mqtt.subscribe("meter/ibexIn");
  mqtt.subscribe("correction");
  mqtt.subscribe(timestampRe);
  mqtt.subscribe(flexRe);
  mqtt.subscribe(correctionRe);
  mqtt.subscribe(resetRe);
  mqtt.subscribe(calibration);
  mqtt.subscribe(readyRe);
  mqtt.subscribe("price");

  mqtt.subscribe(hourlyRe);
  mqtt.subscribe(dayRe);
  mqtt.subscribe(monthRe);
  mqtt.subscribe(subscribeBlynkName);
  mqtt.subscribe(initialRe);
  mqtt.subscribe("RED");
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

  csq = modem.getSignalQuality();


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

  //initialize analog pins and constant for measurements
  emon1.current(12, 74.07); //27 ohm 74.07 (50mA)
  emon2.current(13, 74.07);
  emon3.current(15, 74.07);
  analogReadResolution(ADC_BITS);
  calibrationKoeff = "1";
  adj= "1";
 
  //connect to mqtt broker
  if (!mqtt.connected()) {
  SerialMon.println("=== MQTT NOT CONNECTED ===");
  // Reconnect every 10 seconds
  uint32_t t = millis();
  if (t - lastReconnectAttempt > 10000L) {
    lastReconnectAttempt = t;
    if (mqttConnect()) {
      lastReconnectAttempt = 0;
      mqtt.publish(init_topic, "init");
      
      Serial.println("Init");
    }
  }
  delay(100);
  return;
  }    

  
  powerMap.clear();   
  
}

// send data every 10s
void blynkUpdate()
{ 
  
  get_location();
  struct tm t1;
  String timestamp = timeGet(t1);
  pingMap.insert(std::pair<String, float>(timestamp,blynkPublish));
  String prov_now = "\"providing\": " +String(providing);
  String grReady = "\"gridReady\": " + String(supportReady);
  String sigQual = "\"signal\": " + String(csq);
  String timeSt = "\"timestamp\": " + String(pingMap.begin()->first);
  String powerSt = "\"power\": " + String(pingMap.begin()->second);
  String tr1 = "\"trans1\": " + String(transf1);
  String tr2 = "\"trans2\": " + String(transf2);
  String tr3 = "\"trans3\": " + String(transf3);
  String lt = "\"lat\": " + String(latitude);
  String lng = "\"long\": " + String(longitude);
  String b_name = '"' + blynkName + '"';
  String blynkDeviceName = "\"blynkName\": " + b_name;  
  String jObj = timeSt +','+ powerSt +','+ sigQual +','+ grReady +','+ prov_now +',' + lng +','+lt+ ','+ blynkDeviceName + ','+ tr1 + ',' + tr2 + ',' + tr3;  
  String payload = "{ \"payload\": {" + jObj + "}}";
  payload.toCharArray(pingData, (payload.length() + 1));
  mqtt.publish(ping, pingData);
  pingMap.clear();     
}

//measurements
void measure()
{  
  unsigned long currentMillis = millis();  
    
  if(currentMillis - previousMillis > interval)
  { 
    
    Irms1 = emon1.calcIrms(2307);//2307 after calibration for 50ma 
    transf1 = Irms1;
    if (Irms1 <= 0.2)
    {
      Irms1 = 0;
    }       
    Irms2 = emon2.calcIrms(2307);
    transf2 = Irms2;
    if (Irms2 <= 0.2)
    {
      Irms2 = 0;
    } 
    Irms3 = emon3.calcIrms(2307);
    transf3 = Irms3;
    if (Irms3 <= 0.2)
    {
      Irms3 = 0;
    } 
    double Irms = Irms1 + Irms2 + Irms3;
    double power = Irms*230; 

    if (init_delay = 1)
    { 
      measure_delay.push_back(power);//start to keep track on initial measurements first ones have peaks         
    }
      
    int delay_size = measure_delay.size();
    //Serial.println(delay_size);
    if (delay_size > 10 || start_measure == 1) //Start the real measurements after first 30        
    {
      tenSec[j] = power;         
      j++;        
      if (j > 4)
      {
        for (int k=0; k < 5; k++) 
        {          
          average += tenSec[k];           
        }          
        average /= 5000;

        float devCalibrate = calibrationKoeff.toFloat();
        
        if (devCalibrate != 1)
        {                
          cali_koeff = devCalibrate/average;  
          //Serial.println(cali_koeff);
          calibrationKoeff = "1";
        }
        
        average = average*cali_koeff + powerCorrection; 
                    
        blynkPublish = average;
        measurements.push_back(blynkPublish);//add every 10s                
        average = 0;
        j = 0;
      }   
      init_delay = 0;
      if (delay_size > 0)
      {
        measure_delay.clear();
      }
      start_measure = 1;       
    }
    previousMillis = currentMillis;
  }   
}

//get the time, convert and prepear it
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
  String timeStampS = timeStamp.substring(splitM+4, timeStamp.length());
  String currSecString = timeStampS.substring(0,2);
  currSec = currSecString.toInt();
    
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
//time as timestamp
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

struct timestampPower 
{
  //send the measurements or all accumulated data that is not have been sent for some reasons 
  void sendData()
  {
    for (std::map<String, float>::iterator it = powerMap.begin(); it != powerMap.end(); ++it) 
    {
      String timeSt = "\"timestamp\": " + String((*it).first);
      String powerSt = "\"power\": " + String((*it).second);
      String gridReady = "\"gridReady\": " + String(supportReady);
      String signalQ = "\"signal\": " + String(csq);
      String costPerH = "\"costH\": " + String(costPerHour);
      String costPerD = "\"costD\": " + String(costPerDay);
      String costPerM = "\"costM\": " + String(costPerMonth);
      String budgetPerH = "\"budgetH\": " + String(hourlyBudget);
      String budgetPerD = "\"budgetD\": " + String(dailyBudget);
      String budgetPerM = "\"budgetM\": " + String(monthlyBudget);
      String jObj = timeSt +','+ powerSt +','+ gridReady +','+ signalQ+','+ costPerH +','+ costPerD +','+ costPerM +','+ budgetPerH +','+ budgetPerD +','+ budgetPerM;
      String payload = "{ \"payload\": {" + jObj + "}}";
      payload.toCharArray(data, (payload.length() + 1));
      mqtt.publish(sendFifteen, data);  
    }
  }  
  //send the last timestamp-power pair from the accumulated measurements
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
  if (currSec % 10 == 0) //update dash table every 10s
  {
    if (currSec != prevSec)
    {
      blynkUpdate();
      prevSec = currSec;
    }
  }     
  unsigned long currentMillisSend = millis();

  if(currentMillisSend - previousMillisSend > intervalSend)//db robustness related
  {  
    if (trigger == 1 && powerMap.size() > 0)
    {
      tp.checkDb();
      trigger = 0;
    }
    previousMillisSend = currentMillisSend;
  }

     

  if(currentMillisSend - previousMillisCorrection > 1000)//power correction for given period
  {
    
    if (timer > 0)
    {
      timer = timer - 1;               
    }
    else {
      powerCorrection = 0;
      providing = 0;
    }
    previousMillisCorrection = currentMillisSend;
  }
  

  if (currMin != prevMin)   
  {
          
    accumulatePowH += blynkPublish;   
    accumulatePowD += blynkPublish;   
    accumulatePowM += blynkPublish;   
    // Serial.println(accumulatePowH);
    // Serial.println(accumulatePowM);

         
    hourConsumption = (accumulatePowH - hourBegining)/60;    
    char hourConsumptionBlynk[8];
    dtostrf(hourConsumption, 1, 2, hourConsumptionBlynk);
    mqtt.publish(blynkHourCons, hourConsumptionBlynk); 

    costPerHour = hourConsumption*currPrice;
    char costPerHourBlynk[8];
    dtostrf(costPerHour, 1, 2, costPerHourBlynk);
    mqtt.publish(hCost, costPerHourBlynk); 

    dayConsumption = (accumulatePowD - dayBegining )/60;    
    char dayConsumptionBlynk[8];
    dtostrf(dayConsumption, 1, 2, dayConsumptionBlynk);
    mqtt.publish(blynkDayCons, dayConsumptionBlynk);

    costPerDay = dayConsumption*currPrice;
    char costPerDayBlynk[8];
    dtostrf(costPerDay, 1, 2, costPerDayBlynk);
    mqtt.publish(dCost, costPerDayBlynk); 

    monthConsumption = (accumulatePowM - monthBegining)/60; 
    char monthConsumptionBlynk[8];
    dtostrf(monthConsumption, 1, 2, monthConsumptionBlynk);
    mqtt.publish(blynkMonthCons, monthConsumptionBlynk);

    costPerMonth = monthConsumption*currPrice;
    Serial.println(costPerMonth);
    char costPerMonthBlynk[8];
    dtostrf(costPerMonth, 1, 2, costPerMonthBlynk);
    mqtt.publish(mCost, costPerMonthBlynk); 

    //average for min
    one_min_measure = accumulate( measurements.begin(), measurements.end(), 0.0)/measurements.size();
    measurements.clear();
    struct tm t0;
    String timestamp = timeGet(t0); 
    if (!isnan(one_min_measure) && !isinf(one_min_measure))
    {
      powerMap.insert(std::pair<String, float>(timestamp,one_min_measure));
      tp.sendData();
    }
    prevMin = currMin;  
    //set time for send the data for checking db
    if (nextPeriod >= 60)
    {
      nextPeriod = 0;
    }

    //calculate consumption as % of budget
    if (currPrice  > 0 && hourlyBudget > 0)
    {      
      float hourlyConsPercent = (hourConsumption*currPrice)/hourlyBudget*100;
      char hourBudgetBlynk[8];
      dtostrf(hourlyConsPercent, 1, 2, hourBudgetBlynk);
      mqtt.publish(hourly, hourBudgetBlynk);     
    }

    if (currPrice > 0 && dailyBudget > 0)
    {
      float dailyConsPercent = (dayConsumption*currPrice)/dailyBudget*100;
      char dayBudgetBlynk[8];
      dtostrf(dailyConsPercent, 1, 2, dayBudgetBlynk);
      mqtt.publish(daily, dayBudgetBlynk); 
    }

    if (currPrice > 0 && monthlyBudget > 0)
    {
      float monthlyConsPercent = (monthConsumption*currPrice)/monthlyBudget*100;
      char monthBudgetBlynk[8];
      dtostrf(monthlyConsPercent, 1, 2, monthBudgetBlynk);
      mqtt.publish(monthly, monthBudgetBlynk); 
    }

        
  
  }
  //send data for checking if exist into the db
  if (currMin == nextPeriod)
  {    
    int err_size = powerMap.size();
    String err_string = String(err_size);                
    mqtt.publish("boron", (char*) err_string.c_str());//just send the size of the data that has not be sent for some reasons

    if (err_size > 1000)
    {
      powerMap.clear();
    }
                      
    tp.checkDb();
    trigger = 1;
    nextPeriod += 2;
  }  
  if (currHour != prevHour)
  {   
      hourBegining = accumulatePowH;
      prevHour = currHour;
  }
  if (prevDay != currDay)
  {
      dayBegining = accumulatePowD;
      prevDay = currDay;
  }
  if (prevMonth != currMonth)
  {
      monthBegining = accumulatePowM;
      prevMonth = currMonth;
  }   
  measure();   
  mqtt.loop();      
}
