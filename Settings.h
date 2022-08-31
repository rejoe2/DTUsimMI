#ifndef __SETTINGS_H
#define __SETTINGS_H

#include <stdint.h>
#include <RF24.h>
#include "secrets.h"

#define SER_BAUDRATE            (115200)
#define DEBUG                 // output all infos on serial monitor
bool DEBUG_RCV_DATA = 1;      // output all rcv data on serial monitor
bool DEBUG_TX_DATA  = 1;      // output all tx data on serial monitor
bool WITHWIFI       = 1;      // wifi yes/no
bool CHECK_CRC      = 1;      //without crc, more data but may be wrong, must be checked
bool ZEROEXP        = 0;      //zero export to the grid
bool INTERRUPT      = 0;      //with or without interrupt
bool SNIFFER        = 0;      //as sniffer you just listen everything
bool ONLY_RX        = 0;      //nothing will be sent to inverter, only receive, if you have a dtu
bool WITHMQTT       = 1;      //do you need mqtt?

#define DEFAULT_RF_DATARATE     (RF24_250KBPS)  // Datarate
uint8_t PA_LEVEL    = RF24_PA_LOW;


#define IPServUPDATETICK    15000 //15sek
#define ZEXPUPDTICK         60000 //60sek update zeroexport
#define TXTIMER             700   //send request on every TXTIMER
#define TIMEOUTRXACK        10000 //10sek RxAck timeout
#define TIMERPVCHECK        60000 //40sek RxAck timeout

#define WITH_OTA              // mit OTA Support, also update der Firmware über WLan mittels IP/update

//Hardware configuration, CHECK THIS OUT with you board !!!!!!!!!!!!!!!!!!!!!!!!!!
// ESP8266 PIN Setting====================================================================================
#ifdef ESP8266
    #define RF1_CE_PIN  (D4) //(D2)
    #define RF1_CS_PIN  (D8) //(D8)
    #define RF1_IRQ_PIN (D3)
//GPIO0  D3 -> IRQ
//GPIO2  D4 -> CE
//GPIO14 D5 -> SCK
//GPIO12 D6 -> MIso
//GPIO13 D7 -> MOsi
//GPIO15 D8 -> CSN
#else
  #define RF1_CE_PIN  (9)
  #define RF1_CS_PIN  (10)
  #define RF1_IRQ_PIN (2)
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
#define DTU_RADIO_ID            MY_DTU_PRO

// WR Config =================================================================================
#define SerialWR                MY_MI_WR				// <<<<<<<<<<<<<<<<<<<<<<< anpassen
uint64_t WR1_RADIO_ID           = Serial2RadioID (SerialWR);    // ((uint64_t)0x5279607201ULL);
//uint64_t WR1_RADIO_ID = (uint64_t) 0x6071706301ULL;

bool MI300  = 0;     //chose which model of MI
bool MI600  = 1;
bool MI1500 = 0;
char MIWHAT[25];
#define NRofPV  1

//-----for zeroexport
#define PVPOWER 350  //each PV
int MAXPOWER = NRofPV * PVPOWER;   //for zeroexport
int MINPOWER = int(MAXPOWER / 10); //watt  (10%) under this is the WR off
#define TOLERANCE 15 //watt
// WR Config ===============================================================================

static bool istTag = true;

// Webserver
#define WEBSERVER_PORT      80
static String STARTTIME="";

// Time Server
#define TIMESERVER_NAME "pool.ntp.org"
//#define TIMESERVER_NAME "fritz.box"

// Pinger IP
#define PINGER_IP  {192,168,2,1}

// MQTT
#define MSERVER_IP   "192.168.2.72"
#define MSERVER_PORT 1883

#ifdef WITH_OTA
    // OTA Einstellungen
    #define UPDATESERVER_PORT   WEBSERVER_PORT+1
    #define UPDATESERVER_DIR    "/update"							// mittels IP:81/update kommt man dann auf die OTA-Seite
    #define UPDATESERVER_USER   "username_für_OTA"					// <<<<<<<<<<<<<<<<<<<<<<< anpassen
    #define UPDATESERVER_PW     "passwort_für_OTA"					// <<<<<<<<<<<<<<<<<<<<<<< anpassen
#endif

// internes WLan
// PREFIXE dienen dazu, die eigenen WLans (wenn mehrere) vonfremden zu unterscheiden
// gehe hier davon aus, dass alle WLans das gleiche Passwort haben. Wenn nicht, dann mehre Passwörter hinterlegen
#define SSID_PREFIX1         MY_SSID			// <<<<<<<<<<<<<<<<<<<<<<< anpassen
//#define SSID_PREFIX2         "wlan2-Prefix"	// <<<<<<<<<<<<<<<<<<<<<<< anpassen
#define SSID_PASSWORD        MY_WIFIPW			// <<<<<<<<<<<<<<<<<<<<<<< anpassen

// zur Berechnung von Sonnenauf- und -untergang
#define  geoBreite  MY_BREITE
#define  geoLaenge  MY_LAENGE


#endif
