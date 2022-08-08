#include <Arduino.h>
#include <SPI.h>

#include <ESP8266WiFi.h>
#include <Pinger.h>       // von url=https://www.technologytourist.com   

#include <ArduinoMqttClient.h>



uint8_t checkAllPV(); //forward decl
uint8_t HopRcvCh();//forward decl

uint8_t MQTT = 0;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char MQTTbroker[] = "192.168.1.11";
int        MQTTport     = 1883;
float P_DTSU=0;



char ValueStr[30]="";
char TopicStr[30]="";

void onMqttMessage(int messageSize) {
//----------------------------------------------------------------------------------------

  // we received a message, print out the topic and contents
  uint8_t i=0;
  // DEBUG_OUT.println("Received a message with topic '");
  // DEBUG_OUT.print(F(" length:"));
  // DEBUG_OUT.print(messageSize);

  strcpy(TopicStr,(char *)mqttClient.messageTopic().c_str());
  DEBUG_OUT.print(TopicStr);
  DEBUG_OUT.print(F(" Watt "));

  //myString.c_str()
  while (mqttClient.available()) {
    ValueStr[i]=(char)mqttClient.read();
    i++;
  }  
  //DEBUG_OUT.print(F(" ValueStr:"));
  //DEBUG_OUT.println(ValueStr);

  if ( strcmp(TopicStr,"ImpExpW" ) ==0){ //true if = 0   Import/Export P Watt
      P_DTSU= atof(ValueStr);
      DEBUG_OUT.print(P_DTSU,1);
      DEBUG_OUT.print(F("Watt  | All PVs received?:"));
      DEBUG_OUT.println(checkAllPV());
  }

  //DEBUG_OUT.println();
}//----------------------------------------------------------------------------------------

uint8_t setupMQTT(void){
//----------------------------------------------------------------------------------------
  if (!WITHMQTT)
    return 0;
  DEBUG_OUT.print(F("Attempting to connect to the MQTT broker: "));
  DEBUG_OUT.println(MQTTbroker);
  if (!mqttClient.connect(MQTTbroker, MQTTport)) {
    DEBUG_OUT.print(F("MQTT connection failed! Error code = "));
    DEBUG_OUT.println(mqttClient.connectError());
    return 0;
  }
  else{
    DEBUG_OUT.print(F("MQTT connected = "));
    DEBUG_OUT.println(mqttClient.connectError());
    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);
    if (ZEROEXP)
        DEBUG_OUT.print(F("Subscribing to topics.. "));
        // subscribe to a topic
        mqttClient.subscribe("ImpExpW"); //import export
    return 1;
  }

}//----------------------------------------------------------------------------------------- 

