#ifndef __SETTINGS_H
#define __SETTINGS_H

#include <stdint.h>
#include <RF24.h>

#define SER_BAUDRATE            (115200)
#define DEBUG                 // output all infos on serial monitor

bool DEBUG_RCV_DATA = 1;      // output all rcv data on serial monitor 
bool DEBUG_TX_DATA  = 0;      // output all tx data on serial monitor
bool CHANNEL_HOP_TX = 1;      //tx ch hopping
bool WITHWIFI       = 1;      // wifi yes/no
bool CHECK_CRC      = 0;      //without crc, more data but may be wrong, must be checked
bool ZEROEXP        = 0;      //grid zero export
bool INTERRUPT      = 0;      //with or without interrupt
bool SNIFFER        = 0;      //as sniffer you just listen everything
bool ONLY_RX        = 0;      //nothing will be sent to inverter, only receive
bool WITHMQTT       = 1;
uint8_t PA_LEVEL    = RF24_PA_MIN;

#define UPDATETICK 8000
#define TOLERANCE 10
//#define WITH_OTA              // mit OTA Support, also update der Firmware über WLan mittels IP/update

//Hardware configuration, CHECK THIS OUT with you board !!!!!!!!!!!!!!!!!!!!!!!!!!
// ESP8266 PIN Setting====================================================================================
#ifdef ESP8266
    #define RF1_CE_PIN  (D2) //(D3)
    #define RF1_CS_PIN   (D8) //(D8)
    #define RF1_IRQ_PIN (D3)
//GPIO0  D3 -> IRQ
//GPIO2  D4 -> CE
//GPIO14 D5 -> SCK
//GPIO12 D6 -> MIso
//GPIO13 D7 -> MOsi
//GPIO15 D8 -> CSN
//#else
//  #define RF1_CE_PIN  (9)
//  #define RF1_CS_PIN  (10)
//  #define RF1_IRQ_PIN (2)
#endif
// ESP8266 PIN Setting====================================================================================


union longlongasbytes {
  uint64_t ull;
  uint8_t bytes[8];
};


uint64_t Serial2RadioID (uint64_t sn) {
//----------------------------------
  longlongasbytes llsn;
  longlongasbytes res;
  llsn.ull = sn;
  res.ull = 0;
  res.bytes[4] = llsn.bytes[0];
  res.bytes[3] = llsn.bytes[1];
  res.bytes[2] = llsn.bytes[2];
  res.bytes[1] = llsn.bytes[3];
  res.bytes[0] = 0x01;
  return res.ull;
}

// DTU ========================================================================================
#define DUMMY_RADIO_ID          ((uint64_t)0xDEADBEEF01ULL) 
#define DTU_RADIO_ID            ((uint64_t)0x8765432101ULL) // <<<<<<<<<<<<<<<<<<<<<<< anpassen

// WR Config ==================================================================================
#define SerialWR                0x10x1xxxxxxxxULL	     // <<<<<<<<<<<<<<<<<<<<<<< anpassen
uint64_t WR1_RADIO_ID           = Serial2RadioID (SerialWR);    // ((uint64_t)0x5279607201ULL);

bool MI300  = 0;     //which model of MI; will be evalutated automatically
bool MI600  = 0;
bool MI1500 = 0;

#define NRofPV  2		//will be evalutated automatically

//-----for zeroexport
#define PVPOWER 350            //each PV power
int MAXPOWER = NRofPV * PVPOWER;   
int MINPOWER = int(MAXPOWER / 10); //watt under this is the WR off
// WR Config =================================================================================

// Webserver
#define WEBSERVER_PORT      80

// Time Server
#define TIMESERVER_NAME "pool.ntp.org"
//#define TIMESERVER_NAME "fritz.box"

// Pinger IP
#define PINGER_IP  {192,168,1,1}

// MQTT
bool MQTT_ON = 1;
#define MSERVER_IP   "192.168.1.11"
#define MSERVER_PORT 1883
#define MQTT_ID      "HOYMILES-DTU"
#define VALUE_TOPIC "inverter1"
#define SET_TOPIC   "inverter1/set"

#ifdef WITH_OTA
    // OTA Einstellungen
    #define UPDATESERVER_PORT   WEBSERVER_PORT+1
    #define UPDATESERVER_DIR    "/update"		// mittels IP:81/update kommt man dann auf die OTA-Seite
    #define UPDATESERVER_USER   "username_für_OTA"	// <<<<<<<<<<<<<<<<<<<<<<< anpassen
    #define UPDATESERVER_PW     "passwort_für_OTA"	// <<<<<<<<<<<<<<<<<<<<<<< anpassen
#endif

// internes WLan
// PREFIXE dienen dazu, die eigenen WLans (wenn mehrere) vonfremden zu unterscheiden
// gehe hier davon aus, dass alle WLans das gleiche Passwort haben. Wenn nicht, dann mehre Passwörter hinterlegen
#define SSID_PREFIX1         "ssid"			// <<<<<<<<<<<<<<<<<<<<<<< anpassen
//#define SSID_PREFIX2         "wlan2-Prefix"		// <<<<<<<<<<<<<<<<<<<<<<< anpassen
#define SSID_PASSWORD        "pw"			// <<<<<<<<<<<<<<<<<<<<<<< anpassen

// zur Berechnung von Sonnenauf- und -untergang
#define  geoBreite  49.2866
#define  geoLaenge  7.3416


#endif
