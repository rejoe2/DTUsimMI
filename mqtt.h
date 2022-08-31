#include <Arduino.h>
#include <SPI.h>

#include <ArduinoMqttClient.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <Pinger.h>       // von url=https://www.technologytourist.com   
#else
#endif


static uint8_t MQTT = 0;
const char MQTTbroker[] = MSERVER_IP;
int        MQTTport     = MSERVER_PORT;
float      PowGrid=0;
char       MQTTclientid[] = "MYDTU";



WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

static char TOPIC[]                 = "PV/TSUN-tsol-m800";

static char INVTOT_P[]              = "PV/TSUN/Mi01_totalW";
static char LIMIT_P[]               = "PV/TSUN/Limiting";
static char INV_P_PVNR[]            = "PV/TSUN/Mi01pvPower";
static char INV_UDC_PVNR[]          = "PV/TSUN/Mi01_UPv";
static char INV_IDC_PVNR[]          = "PV/TSUN/Mi01_IPv";
static char INV_Q_PVNR[]            = "PV/TSUN/Mi01EnergiePV";
static char INV_TEMP[]              = "PV/TSUN/WRtemp";
static char INV_STS_PVNR[]          = "PV/TSUN/Mi01StsPort";
static char INV_CONSUM_P[]          = "PV/TSUN/ConsumW";
static char DAY[]                   = "PV/TSUN/Day";
static char TIME[]                  = "PV/TSUN/Time";
static char INFO[]                  = "PV/TSUN/Error";
static char GRID_P[]                = "PV/TSUN/ImpExpW";  //also the topic for read

char ValueStr[20]="";
char TopicStr[20]="";

void onMqttMessage(int messageSize) {
//----------------------------------------------------------------------------------------

  // we received a message, print out the topic and contents
  uint8_t i=0;
  // DEBUG_OUT.println("Received a message with topic '");
  // DEBUG_OUT.print(F(" length:"));
  // DEBUG_OUT.print(messageSize);

  strcpy(TopicStr,(char *)mqttClient.messageTopic().c_str());
  //DEBUG_OUT.print(TopicStr);
  //DEBUG_OUT.print(F(" Watt "));

  //myString.c_str()
  while (mqttClient.available()) {
    ValueStr[i]=(char)mqttClient.read();
    i++;
  }  
  //DEBUG_OUT.print(F(" ValueStr:"));
  //DEBUG_OUT.println(ValueStr);

  if ( strcmp(TopicStr,GRID_P ) ==0){ //true if = 0   Import/Export P Watt
      PowGrid= atof(ValueStr);
      DEBUG_OUT.print(PowGrid,1);
      DEBUG_OUT.print(F("Watt"));
  }

  //DEBUG_OUT.println();
}//----------------------------------------------------------------------------------------

uint8_t setupMQTT(void){
//----------------------------------------------------------------------------------------
  if (!WITHMQTT)
    return 0;
  DEBUG_OUT.printf("Attempting to connect to the MQTT broker: %s",MQTTbroker);
  mqttClient.setId(MQTTclientid);
  if (!mqttClient.connect(MQTTbroker, MQTTport)) {
    DEBUG_OUT.printf("MQTT connection failed! Error code %i",mqttClient.connectError());
    MQTT=0;
  }
  else{
    DEBUG_OUT.printf("MQTT connected (null is ok) %i",mqttClient.connectError());
    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);
    if (ZEROEXP)
        DEBUG_OUT.printf("Subscribing to topics.. %s",GRID_P);
        // subscribe to a topic
        mqttClient.subscribe(GRID_P); //import export
    MQTT=1;
  }
  return MQTT;
}//----------------------------------------------------------------------------------------- 

