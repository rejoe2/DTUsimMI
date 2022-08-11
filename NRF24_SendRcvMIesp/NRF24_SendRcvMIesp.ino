/* die orig. SW ist vom Hubi, wurde von mir(Ziyat T.) für den MI-WR abgeaendert.
/* die orig. SW ist vom Hubi, wurde von mir(Ziyat T.) für den MI-WR abgeaendert.
/* die orig. SW ist vom Hubi, wurde von mir(Ziyat T.) für den MI-WR abgeaendert.
Getestet auf ESP8266/ArduinoUNO.
https://www.mikrocontroller.net/topic/525778
https://github.com/hm-soft/Hoymiles-DTU-Simulation

Alle Einstellungen sind in Settings.h !!
*/
#include <stdint.h>
#include <printf.h>
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <RF24_config.h>

#include "Settings.h"
#include "CircularBuffer.h"
#include "hm_crc.h"
#include "hm_packets.h"
#include "Debug.h"
#include "wifi.h"
#include "mqtt.h"

#ifdef ESP8266
    #define DISABLE_EINT noInterrupts()
    #define ENABLE_EINT  interrupts()
#else     // für AVR z.B. ProMini oder Nano
  #define DISABLE_EINT EIMSK = 0x00
  #define ENABLE_EINT EIMSK = 0x01
#endif


#define RF_MAX_ADDR_WIDTH       (5) 
#define MAX_RF_PAYLOAD_SIZE     (32)

#ifdef ESP8266
#define PACKET_BUFFER_SIZE      (30) 
#else
#define PACKET_BUFFER_SIZE      (20) 
#endif

// Startup defaults until user reconfigures it
#define DEFAULT_RECV_CHANNEL    (3)             // 3 = Default channel for Hoymiles
//#define DEFAULT_SEND_CHANNEL  (75)            // 40 = Default channel for Hoymiles, 61
#define DEFAULT_RF_DATARATE     (RF24_250KBPS)  // Datarate

#include "NRF24_sniff_types.h"

static HM_Packets     hmPackets;
static uint32_t       tickMillis;
static uint32_t       UpdateTick;

// Set up nRF24L01 radio on SPI bus plus CE/CS pins
// If more than one RF24 unit is used the another CS pin than 10 must be used
// This pin is used hard coded in SPI library
static RF24 radio1 (RF1_CE_PIN, RF1_CS_PIN);
static NRF24_packet_t bufferData[PACKET_BUFFER_SIZE];
static CircularBuffer<NRF24_packet_t> packetBuffer(bufferData, sizeof(bufferData) / sizeof(bufferData[0]));
static Serial_header_t SerialHdr;

static uint16_t lastCRC;
static uint16_t crc;

uint8_t         channels[]            = {3, 23, 40, 61, 75};   //{1, 3, 6, 9, 11, 23, 40, 61, 75}
uint8_t         channelIdx            = 1;                         // fange mit 23 an
uint8_t         DEFAULT_SEND_CHANNEL  = channels[channelIdx];      // = 23

static unsigned long timeLastPacket = millis();
static unsigned long timeOutChanAck = 60000;                       // wenn zu lange nichts kommt, müssen wir wechseln; 1 Minute?
static unsigned long timeLastAck    = 4294967295 - timeOutChanAck; // wenn ein Hardware-Ack kommt, haben wir vorläufig einen akzeptablen Channel
static uint8_t hoptx = 0;

// Function forward declaration
//static void SendPacket(uint64_t dest, uint8_t *buf, uint8_t len);
char * getChannelName (uint8_t i);
static const int    ANZAHL_VALUES         = 8;
static float        VALUES[4][ANZAHL_VALUES] = {};
static const char   *CHANNEL_NAMES[ANZAHL_VALUES] 
   = {"PanelNr   ",      
      "P [W]  ",
      "Udc [V]",
      "Idc [A]",
      "E [Wh] ",
      "Status ",
      "FCnt   ",
      "FCode  "};
      
//static const uint8_t DIVISOR[ANZAHL_VALUES] = {1,1,1,1,1,1,1,1,1,1,1,1};
static const char BLANK = ' ';

static boolean istTag = true;
char CHANNELNAME_BUFFER[15];

//char WRdata[120];
float  U_DC =0;
float  I_DC =0;
float  U_AC =0;
float  F_AC =0;
float  P_DC =0;
float  Q_DC =0;           
float  TEMP =0;
uint8_t DataOK=0;
int  STAT =0;
int  FCNT = 0;
int  FCODE = 0;
uint8_t  PV = 0;
uint16_t PMI = 0;
//uint16_t P_DSU = 0;
uint16_t LIMIT=0; //for ModWebserver
uint8_t pvCnt[4]={0,0,0,0};

float TotalP[5]={0,0,0,0,0}; //0 is total power, 1-4 are 4 PV power

#ifdef ESP8266
  #include "wifi.h"
  #include "ModWebserver.h"
  //#include "Sonne.h"
#endif

//MI-WR Data
uint8_t sendBuf[MAX_RF_PAYLOAD_SIZE];


//read from Serial for testing 
char SerialIn[10]="";
int Limit=0; //setup 
int SerCmd=0;
int OldLimit = MINPOWER;
uint8_t WRInfo=1;
static uint8_t SendLimitSts=0; //quiet
static uint8_t RcvCH = DEFAULT_RECV_CHANNEL;
static uint8_t TxCH = DEFAULT_RECV_CHANNEL;

//String mStr = "";     // empty string
char cStr[100];



char * getChannelName (uint8_t i) {
//------------------------------------------------------------------------------------------------
  memset (CHANNELNAME_BUFFER, 0, sizeof(CHANNELNAME_BUFFER));
  strcpy (CHANNELNAME_BUFFER, CHANNEL_NAMES[i]);
  //itoa (i, CHANNELNAME_BUFFER, 10);
  return CHANNELNAME_BUFFER;
}//-----getChannelName-----------------------------------------------------------------------------
inline static void dumpData(uint8_t *p, int len) {
//------------------------------------------------------------------------------------------------
  while (len--){
    if (*p < 16)
      DEBUG_OUT.print(F("0"));
    DEBUG_OUT.print(*p++, HEX);
  }
  DEBUG_OUT.print(BLANK);
}//----dumpData------------------------------------------------------------------------------------

#ifdef ESP8266
  IRAM_ATTR
#endif


void ReadRFBuf(void){
//---------------------------------------------------------------------------------------
 static uint16_t lostPacketCount = 0;
 uint8_t pipe;
// Loop until RX buffer(s) contain no more packets.
  while (radio1.available(&pipe)) {
    if (!packetBuffer.full()) {
      NRF24_packet_t *p = packetBuffer.getFront();
      p->timestamp = micros(); // Micros does not increase in interrupt, but it can be used.
      p->packetsLost = lostPacketCount;
      radio1.setChannel(RcvCH);//setChannel(DEFAULT_RECV_CHANNEL);
      //radio1.setChannel(channels[hoptx]);
      uint8_t packetLen = radio1.getPayloadSize();
      if (packetLen > MAX_RF_PAYLOAD_SIZE)
        packetLen = MAX_RF_PAYLOAD_SIZE;

      radio1.read(p->packet, packetLen);

      packetBuffer.pushFront(p);
      lostPacketCount = 0;
      }
    else {
      bool tx_ok, tx_fail, rx_ready; // Buffer full. Increase lost packet counter.
      if (lostPacketCount < 255){
         lostPacketCount++;
         DEBUG_OUT.println(F("Lost packet full buffer"));
         }
      if(INTERRUPT)
        radio1.whatHappened(tx_ok, tx_fail, rx_ready);// Call 'whatHappened' to reset interrupt status.
      radio1.flush_rx();// Flush buffer to drop the packet.
      }
    }
}
//----ReadRFBuf-----------------------------------------------------------------------------------

void ICACHE_RAM_ATTR handleNrf1Irq() {
//---------------------------------------------------------------------------------------

  DISABLE_EINT;
  ReadRFBuf();
  ENABLE_EINT;

}//---handleNrf1Irq-------------------------------------------------------------------

void   setRxPipe(void){
//---------------------------------------------------------------------------------------

  if (SNIFFER){
     radio1.openReadingPipe(0, 0x00aa);
     radio1.openReadingPipe(1, 0x0055);
     }
  else {
      //radio1.openReadingPipe(0, DTU_RADIO_ID);
      radio1.openReadingPipe(1, DTU_RADIO_ID);
      //radio1.openReadingPipe(1, WR1_RADIO_ID);
      }

}//----setRxPipe----------------------------------------------------------------------

static void activateRFConf(void) {
//---------------------------------------------------------------------------------------
  radio1.begin();
    // Use lo PA level, as a higher level will disturb CH340 DEBUG_OUT usb adapter

  radio1.setAutoAck(0);
  radio1.setRetries(0, 0);

  radio1.setDataRate(DEFAULT_RF_DATARATE);
  radio1.disableCRC();
  radio1.setPALevel(PA_LEVEL);
  radio1.setPayloadSize(MAX_RF_PAYLOAD_SIZE);
  radio1.setAddressWidth(5);

  setRxPipe();
  // We want only RX irqs,
  if(INTERRUPT)
    radio1.maskIRQ(true, true, false);
  // Attach interrupt handler to NRF IRQ output. Overwrites any earlier handler.
  if(INTERRUPT)
    attachInterrupt(digitalPinToInterrupt(RF1_IRQ_PIN), handleNrf1Irq, FALLING); // NRF24 Irq pin is active low.
  // Initialize SerialHdr header's address member to promiscuous address.
  uint64_t addr = DTU_RADIO_ID;
  for (int8_t i = sizeof(SerialHdr.address) - 1; i >= 0; --i) {
    SerialHdr.address[i] = addr;
    addr >>= 8;
    } 

  radio1.printDetails();
  delay(2000);
  tickMillis = millis() + 200;
}//--activateRFConf-------------------------------------------------------------------------------------

void setup(void) {
//---------------------------------------------------------------------------------------

  DEBUG_OUT.begin(SER_BAUDRATE);

  DEBUG_OUT.flush();
  DEBUG_OUT.println(F("....................."));
  DEBUG_OUT.println(F("DTU for MI Simulation"));
  DEBUG_OUT.println(F("Setup---------"));

  #ifdef ESP8266
    if (WITHWIFI){
      if(!SNIFFER){
          while (!setupWifi())
            DEBUG_OUT.println(F("Setup Wifi..."));

          DEBUG_OUT.println(F("Setup other IP services..."));
          setupClock();
          MQTT=setupMQTT();
          setupWebServer();
                 // setupUpdateByOTA();
                //calcSunUpDown (getNow());
                // istTag = isDayTime();
                // DEBUG_OUT.print ("Es ist "); DEBUG_OUT.println (istTag?"Tag":"Nacht");
                //hmPackets.SetUnixTimeStamp (getNow());
         }
      }
  #else
    //hmPackets.SetUnixTimeStamp(0x62456430);
  #endif
  delay(2000);
  //---NRF--------------------------
  DEBUG_OUT.println(F("Setup NRF"));
  // Configure nRF IRQ input
  if(INTERRUPT)
    pinMode(RF1_IRQ_PIN, INPUT);
  activateRFConf();

  delay(2000);
  UpdateTick = millis() + 2000;


  if (MI300) strcpy(MIWHAT,"MI-300");
  if (MI600) strcpy(MIWHAT,"MI-600");
  if (MI1500) strcpy(MIWHAT,"MI-1500");

  DEBUG_OUT.print(F("Microinverter is  "));DEBUG_OUT.println(MIWHAT);

  DEBUG_OUT.println(F("Setup finished --------Type 1 for HELP-----------"));


}//---setup------------------------------------------------------------------------------------

static void SendPacket(uint64_t dest, uint8_t *buf, uint8_t len) {
//----------------------------------------------------------------------------------------------------

  if(INTERRUPT) DISABLE_EINT;
  //static uint8_t hoptx = 0;

  radio1.flush_tx();

  if (CHANNEL_HOP_TX){
    if (DEBUG_TX_DATA) {
      DEBUG_OUT.print(millis());
      DEBUG_OUT.print(F(" "));
      if (channels[hoptx]<10)
        DEBUG_OUT.print(F("Send... CH0"));
      else DEBUG_OUT.print(F("Send... CH"));
      DEBUG_OUT.print(channels[hoptx]);
      DEBUG_OUT.print(F(" "));
      }
    TxCH=hoptx;
    if (millis() - timeLastAck > timeOutChanAck) {
      TxCH=++hoptx; // hoptx++;
      if (hoptx >= sizeof(channels))// / sizeof(channels[0]) )
        hoptx = 0;
      }
    }
  else    TxCH=DEFAULT_SEND_CHANNEL;


  if (DEBUG_TX_DATA){ //packet buffer to output
    //DEBUG_OUT.print(millis());
    for (uint8_t i = 0; i < len; i++){
        if (buf[i]==0){DEBUG_OUT.print(F("00"));}
        else { if (buf[i]<0x10) {DEBUG_OUT.print(F("0"));}
             DEBUG_OUT.print(buf[i],HEX);
             }
        }
    }
  radio1.stopListening(); //***************************************
  radio1.setChannel(TxCH);
  radio1.openWritingPipe(dest);
  radio1.setCRCLength(RF24_CRC_16);
  radio1.enableDynamicPayloads();
  radio1.setAutoAck(true);
  radio1.setRetries(3, 15);
  uint8_t res = radio1.write(buf, len);
  if (DEBUG_TX_DATA) {DEBUG_OUT.print(".....send res ");
    DEBUG_OUT.println(res);
    }
  if ( res ) { //we got an hardware ACK!
      timeLastAck = millis();
  }
  //radio1.print_status(radio1.get_status());
  // Try to avoid zero payload acks (has no effect)
  radio1.openWritingPipe(DUMMY_RADIO_ID);
  radio1.setAutoAck(false);
  radio1.setRetries(0, 0);
  radio1.disableDynamicPayloads();
  radio1.setCRCLength(RF24_CRC_DISABLED);

 if(INTERRUPT) ENABLE_EINT;
 //radio1.setChannel(HopRcvCh());
 radio1.setChannel(channels[hoptx]);
 radio1.startListening(); //***************************************

}//----SendPacket-------------------------------------------------------------------------------------------------------

void SerialCmdHandle(void){
//----------------------------------------------------------------------------------------------------------------------
    switch (SerCmd){
        case 1:
          DEBUG_OUT.println(F("1: help 2:Status 3:PA_LOW 4:PA_HIGH 5:PA_MAX 6:Sniffer 7:ZeroEx 8:OnlyRX 9:ShowTX 10:Wifi 11:CRC"));
          DEBUG_OUT.println(F("20:WRinfo 21:Gongfa "));
        break;
        case 2:
          sprintf(cStr,"DEBUG_RCV_DATA %i",DEBUG_RCV_DATA); DEBUG_OUT.println (cStr);
          sprintf(cStr,"DEBUG_TX_DATA %i",DEBUG_TX_DATA); DEBUG_OUT.println (cStr);
          sprintf(cStr,"PA_LEVEL %i",PA_LEVEL); DEBUG_OUT.println (cStr);
          sprintf(cStr,"WITHWIFI %i",WITHWIFI); DEBUG_OUT.println (cStr);
          sprintf(cStr,"ZEROEXP %i",ZEROEXP); DEBUG_OUT.println (cStr);
          sprintf(cStr,"INTERRUPT %i",INTERRUPT); DEBUG_OUT.println (cStr);
          sprintf(cStr,"SNIFFER %i",SNIFFER); DEBUG_OUT.println (cStr);
          sprintf(cStr,"ONLY_RX %i",ONLY_RX); DEBUG_OUT.println (cStr);
          sprintf(cStr,"CHECK_CRC %i",CHECK_CRC); DEBUG_OUT.println (cStr);
        break;
        case 3:
           PA_LEVEL = RF24_PA_LOW;
           radio1.setPALevel(PA_LEVEL);
           DEBUG_OUT.println(F("RF24_PA_LOW"));
           radio1.printDetails();
        break;
        case 4:
          PA_LEVEL = RF24_PA_HIGH;
          radio1.setPALevel(PA_LEVEL);
          DEBUG_OUT.println(F("RF24_PA_HIGH"));
          radio1.printDetails();
        break;
        case 5:
           PA_LEVEL = RF24_PA_MAX;
           radio1.setPALevel(PA_LEVEL);
           DEBUG_OUT.println(F("RF24_PA_MAX"));
           radio1.printDetails();
        break;
        case 6:
              if (SNIFFER)  SNIFFER = 0;
              else  SNIFFER = 1;
              DEBUG_OUT.println(F("CMD Sniffer"));DEBUG_OUT.println(SNIFFER);
              activateRFConf();
        break;
        case 7:
              if (ZEROEXP)  ZEROEXP = 0;
              else  ZEROEXP = 1;
              DEBUG_OUT.print(F("CMD Zeroexport "));DEBUG_OUT.println(ZEROEXP);
        break;
        case 8:
              if (ONLY_RX)  ONLY_RX = 0;
              else  ONLY_RX = 1;
              DEBUG_OUT.print(F("CMD Only RX "));DEBUG_OUT.println(ONLY_RX);
        break;
        case 9:
              if (DEBUG_TX_DATA)  DEBUG_TX_DATA = 0;
              else  DEBUG_TX_DATA = 1;
              DEBUG_OUT.print(F("CMD Out TX Data "));DEBUG_OUT.println(DEBUG_TX_DATA);
        break;
        case 10:
              if (WITHWIFI)  WITHWIFI = 0;
              else  WITHWIFI = 1;
              DEBUG_OUT.print(F("CMD Wifi "));DEBUG_OUT.println(WITHWIFI);
              setup();
        break;
        case 11:
              if (CHECK_CRC)  CHECK_CRC = 0;
              else  CHECK_CRC = 1;
              DEBUG_OUT.print(F("CMD CHECK_CRC "));DEBUG_OUT.println(CHECK_CRC);
        break;

        default:
        break;
        }
    SerCmd=0; //stop command
}
//----SerialCmdHandle-----------------------------------------------------------------------------------------------------

void isTime2Send (void) {
//----------------------------------------------------------------------------------------------------------------------
  static uint8_t MIDataCMD = 0x36;   //begin with first PV
  static uint8_t MI600_DataCMD = 0x09 ;
  static uint8_t telegram = 0;
  int32_t size = 0;
  uint64_t dest = WR1_RADIO_ID;
  uint8_t UsrData[10]; 
  char Cmd = 0;  // Second timer

  if (millis() >= tickMillis) {
    tickMillis += 300;    //200;

    if (telegram > sizeof(channels))    telegram = 0;

    if (MI300) {        // 1 PV
        MIDataCMD=0x09;
        //DEBUG_OUT.println("MI300");
        tickMillis += 4700;    //200;
        }
    if (MI600){         // 2 PVs
        if (MI600_DataCMD == 0x09) {
            MI600_DataCMD=0x11;
        }
        else if (MI600_DataCMD == 0x11) MI600_DataCMD=0x09;
        //DEBUG_OUT.print(MI600_DataCMD);      DEBUG_OUT.println(" MI600");
        MIDataCMD=MI600_DataCMD;
        tickMillis += 4700;    //200;
        }
    if (MI1500){        // 4 PVs
         //DEBUG_OUT.println("MI1500");
        if (MIDataCMD > 0x0039) {
            MIDataCMD= 0x0036;
            tickMillis += 800;    //200;
           }
        }

    switch(telegram) {
      case 0:
        //set SubCmd and  UsrData Limiting
        if ((Limit > 0) && (SendLimitSts))  {        //Limitierung/*&& (abs (P_DTSU) > TOLERANCE))*/
          Cmd=0x51;          
          DEBUG_OUT.print(F("CMD 0x"));DEBUG_OUT.print(Cmd,HEX);DEBUG_OUT.print(F(" Sending Limit:"));
          DEBUG_OUT.println(Limit);
          UsrData[0]=0x5A;UsrData[1]=0x5A;UsrData[2]=100;//0x0a;// 10% limit   
                    
          UsrData[3]=((Limit*10) >> 8) & 0xFF;   UsrData[4]= (Limit*10)  & 0xFF;   //WR needs 1 dec= zB 100.1 W
          size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,5);
          //Limit=0; will be set after ack limiting
          pvCnt[0]=pvCnt[1]=pvCnt[2]=pvCnt[3]=0; //reset PV;sts
          }
        else { ////request WR data HM
          //   #ifdef ESP8266
          //       hmPackets.SetUnixTimeStamp (getNow());
          //   #endif
          //   size = hmPackets.GetTimePacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8);
          UsrData[0]=0x0;//set SubCmd and  UsrData for data request
          size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, MIDataCMD, UsrData,1);
          }
        break;
      case 1:
        switch (SerCmd){ //SrCmd Parts to send
           case 20: //request WR info
              Cmd=0x0f;
              UsrData[0]=0x01;
              DEBUG_OUT.print(F("CMD 0x"));DEBUG_OUT.print(Cmd,HEX);DEBUG_OUT.print(F(" Sending WRInfo Req:0x"));
              DEBUG_OUT.println(WRInfo,HEX);
              size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,1);
            break;
            case 21: //GONGFA
              Cmd=0x02;
              UsrData[0]=0x0;
              DEBUG_OUT.print(F("CMD 0x"));DEBUG_OUT.print(Cmd,HEX);DEBUG_OUT.println(F(" Sending Gongfa"));
              size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, Cmd, UsrData,1);
            break;
            default://request WR data
              UsrData[0]=0x0;//set SubCmd and  UsrData
              size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, MIDataCMD, UsrData,1);
            }
        //SerCmd=0; //stop command
        break;
      default: //request WR data
        UsrData[0]=0x0;//set SubCmd and  UsrData
        size = hmPackets.GetCmdPacket((uint8_t *)&sendBuf, dest >> 8, DTU_RADIO_ID >> 8, MIDataCMD, UsrData,1);
      }  //switch telegram
    SendPacket(dest, (uint8_t *)&sendBuf, size);

    telegram++; 
    MIDataCMD++;
    } //if millis
}//----isTime2Send---------------------------------------------------------------------------------------

void DumpRcvPacket(NRF24_packet_t *p, uint8_t payloadLen) {
//------------------------------------------------------------------------------------------------------
    DEBUG_OUT.print(F("CH")); DEBUG_OUT.print(RcvCH); DEBUG_OUT.print(F(" "));
    // Write  packets lost, address and payload length
    dumpData((uint8_t *)&SerialHdr.packetsLost, sizeof(SerialHdr.packetsLost));
    dumpData((uint8_t *)&SerialHdr.address, sizeof(SerialHdr.address));
    // Trailing bit?!?
    dumpData(&p->packet[0], 2);
    // Payload length from PCF
    dumpData(&payloadLen, sizeof(payloadLen));
    // Packet control field - PID Packet identification
    uint8_t val = (p->packet[1] >> 1) & 0x03;
    DEBUG_OUT.print(val); DEBUG_OUT.print(F("  "));

    if (payloadLen > 9) {
      dumpData(&p->packet[2], 1);
      dumpData(&p->packet[3], 4);
      dumpData(&p->packet[7], 4);
      
      uint16_t remain = payloadLen - 2 - 1 - 4 - 4 + 4;

      if (remain < 32) {
        dumpData(&p->packet[11], remain);
        printf_P(PSTR("%04X "), crc);
        if (((crc >> 8) != p->packet[payloadLen + 2]) || ((crc & 0xFF) != p->packet[payloadLen + 3]))
          DEBUG_OUT.print(0);
        else
          DEBUG_OUT.print(1);
        }
      else {
        DEBUG_OUT.print(F("Ill remain "));
        DEBUG_OUT.print(remain);
        }
      }
    else {
      dumpData(&p->packet[2], payloadLen + 2);
      printf_P(PSTR("%04X "), crc);
      }
    DEBUG_OUT.println();
}//----DumpRcvPacket----------------------------------------------------------------------------

void HandleSerialCmd(void){ //read from serial for WR control cmd's
//-------------------------------------------------------------------------------------------
  static uint8_t InCnt=0;
  int temporary=0;
  
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    if (incomingByte != 13){ //CR
      SerialIn[InCnt]=incomingByte; 
      InCnt++;       
      }
    else {
      SerialIn[InCnt]=0;   //eofl
      temporary = atoi(SerialIn);

      if ((temporary >1999) || (temporary < 30)){  //other cmds. implemented > 2000, request WR info etc.
        SerCmd = temporary;  //this is a serial command
        SendLimitSts=0;      //no nrf send needed
        SerialCmdHandle();
        }
      else{
        Limit = temporary; //Limit is 1-1999
        SendLimitSts=1; //nrf send needed
        }
      sprintf(cStr,"\r\nSerialIn: %s Cmd:%i",SerialIn,temporary); DEBUG_OUT.println (cStr);
      InCnt=0;

      }
  }
}//---HandleSerialCmd-------------------------------------------------------------------------------

void SendMQTTMsg(String topic, String value){
//-------------------------------------------------------------------------------------------
#ifdef WITHWIFI
  if (!checkWifi()) return;
  if (!MQTT) return;
  mqttClient.beginMessage(topic);
  mqttClient.print(value);
  mqttClient.endMessage();
#endif
}//---SendMQTTMsg----------------------------------------------------------------------------------------

void MI1500DataMsg(NRF24_packet_t *p){
//--------------------------------------------------------------------------------------------------
  U_DC =  (float) ((p->packet[11] << 8) + p->packet[12])/10;
  I_DC =  (float) ((p->packet[13] << 8) + p->packet[14])/10;
  U_AC =  (float) ((p->packet[15] << 8) + p->packet[16])/10;
  F_AC =  (float) ((p->packet[17] << 8) + p->packet[18])/100;
  P_DC =  (float)((p->packet[19] << 8) + p->packet[20])/10;
  Q_DC =  (float)((p->packet[21] << 8) + p->packet[22])/1;
  TEMP =  (float) ((p->packet[23] << 8) + p->packet[24])/10;

  if ((30<U_DC<50) && (0<I_DC<15) && (200<U_AC<300) && (45<F_AC<55) && (0<P_DC<420) && (0<TEMP<80))
   DataOK = 1;
  else { DEBUG_OUT.println(F("Data Wrong!!"));DataOK =0; return;}

  STAT = (uint8_t)(p->packet[25] );
  FCNT = (uint8_t)(p->packet[26]);
  FCODE = (uint8_t)(p->packet[27]);

  if (p->packet[2] == 0xB6)  {PV= 0; TotalP[1]=P_DC; pvCnt[0]=1;}//port 1
  if (p->packet[2] == 0xB7)  {PV= 1; TotalP[2]=P_DC; pvCnt[1]=1;}//port 2
  if (p->packet[2] == 0xB8)  {PV= 2; TotalP[3]=P_DC; pvCnt[2]=1;}//port 3
  if (p->packet[2] == 0xB9)  {PV= 3; TotalP[4]=P_DC; pvCnt[3]=1;}//port 4
  TotalP[0]=TotalP[1]+TotalP[2]+TotalP[3]+TotalP[4];//in TotalP[0] is the totalPvW
  if((P_DC>PVPOWER) || (P_DC<0) || (TotalP[0]>MAXPOWER)){// cant be!!
    sprintf(cStr,"Wrong Data.. PV%1i %5sW Total:%5sW ", PV,String(P_DC,1),String(TotalP[0],1) );
    DEBUG_OUT.println(cStr);
    TotalP[0]=0;
    return;
    }
  VALUES[PV][0]=PV;
  VALUES[PV][1]=P_DC;
  VALUES[PV][2]=U_DC;
  VALUES[PV][3]=I_DC;
  VALUES[PV][4]=Q_DC;
  VALUES[PV][5]=STAT;
  VALUES[PV][6]=FCNT;
  VALUES[PV][7]=FCODE;

  PMI=TotalP[0];
  LIMIT=(uint16_t)Limit;
  sprintf(cStr,"CH:%2i PV%1i MI:%4iW Grd:%4iW Lm:%4iW %4sW %4sV %3sA %4iWh %4sACV %3sHz %3sC S:%i ",
  RcvCH,PV,(int)PMI,(int)P_DTSU,Limit,String(P_DC,1),String(U_DC,1),String(I_DC,1),
  (int)Q_DC, String(U_AC,1), String(F_AC,1), String(TEMP,1), STAT);//, (String)getTimeStr(getNow()) );
  DEBUG_OUT.print(millis()); DEBUG_OUT.print(F(" "));
  DEBUG_OUT.println(cStr);
  if (p->packet[2] != 0xB9) tickMillis = millis();
}//--MI1500DataMsg------------------------------------------------------------------------------------------------------

void MI600StsMsg (NRF24_packet_t *p){
  //-----------------------------------------------------------------------
  STAT = (int)((p->packet[11] << 8) + p->packet[12]);
  FCNT = (int)((p->packet[13] << 8) + p->packet[14]);
  FCODE = (int)((p->packet[15] << 8) + p->packet[16]);

  VALUES[PV][5]=STAT;
  VALUES[PV][6]=FCNT;
  VALUES[PV][7]=FCODE;

}//--MI600StsMsg---------------------------------------------------------------------


void MI600DataMsg(NRF24_packet_t *p){
  //--------------------------------------------------------------------------------------------------------------------
  U_DC =  (float) ((p->packet[11] << 8) + p->packet[12])/10;
  I_DC =  (float) ((p->packet[13] << 8) + p->packet[14])/10;
  U_AC =  (float) ((p->packet[15] << 8) + p->packet[16])/10;
  F_AC =  (float) ((p->packet[17] << 8) + p->packet[18])/100;
  P_DC =  (float)((p->packet[19] << 8) + p->packet[20])/10;
  Q_DC =  (float)((p->packet[21] << 8) + p->packet[22])/1;
  TEMP =  (float) ((p->packet[23] << 8) + p->packet[24])/10;

  if ((30<U_DC<50) && (0<I_DC<15) && (200<U_AC<300) && (45<F_AC<55) && (0<P_DC<420) && (0<TEMP<80))
   DataOK = 1;  //we need to check this, if no crc
  else { DEBUG_OUT.println(F("Data Wrong!!"));DataOK =0; return;}

  if (p->packet[2] == 0x89)  {PV= 0; TotalP[1]=P_DC; pvCnt[0]=1;}//port 1
  if (p->packet[2] == 0x91)  {PV= 1; TotalP[2]=P_DC; pvCnt[1]=1;}//port 2

  TotalP[0]=TotalP[1]+TotalP[2]+TotalP[3]+TotalP[4];//in TotalP[0] is the totalPV power
  if((P_DC>400) || (P_DC<0) || (TotalP[0]>MAXPOWER)){// cant be!!
    TotalP[0]=0;
    return;
    }
  VALUES[PV][0]=PV;
  VALUES[PV][1]=P_DC;
  VALUES[PV][2]=U_DC;
  VALUES[PV][3]=I_DC;
  VALUES[PV][4]=Q_DC;

  PMI=TotalP[0];
  LIMIT=(uint16_t)Limit;
  sprintf(cStr,"CH:%2i PV%1i MI:%4iW Grd:%4iW Lm:%4iW %4sW %4sV %3sA %4iWh %4sACV %3sHz %3sC S:%i ",
  RcvCH,PV,(int)PMI,(int)P_DTSU,Limit,String(P_DC,1),String(U_DC,1),String(I_DC,1),
  (int)Q_DC, String(U_AC,1), String(F_AC,1), String(TEMP,1), STAT);//, (String)getTimeStr(getNow()) );
  DEBUG_OUT.print(millis()); DEBUG_OUT.print(F(" "));
  DEBUG_OUT.println(cStr);
  DEBUG_OUT.println(cStr);
  if (p->packet[2] == 0x89) tickMillis = millis();
  timeLastAck = millis();
}//--------------------------------------------------------------------------------------------------

void AnalyseMI1500(NRF24_packet_t *p,uint8_t payloadLen){
//--------------------------------------------------------------------------------------------------
  
  switch (p->packet[2])  {

    case 0xD1:
      DEBUG_OUT.print (F("Limiting(0x51) is ok CMD=")); 
      DEBUG_OUT.println(p->packet[2], HEX);
      OldLimit = Limit;
      //Limit=0; //stop sending Limit
      SendLimitSts=0;
      break;
    case 0x82: //Gongfa
      DEBUG_OUT.print (F("Gongfa(0x2) is ok CMD="));
      DEBUG_OUT.println(p->packet[2], HEX);
      dumpData(&p->packet[3], payloadLen);
      SerCmd=0; //stop sending
      break;
    case 0x8f:
      sprintf(cStr,"WRInfo(0x8f) is ok CMD %x ",p->packet[2]); DEBUG_OUT.print (cStr);
      sprintf(cStr,"WR %x:%x:%x:%x ",p->packet[7],p->packet[8],p->packet[9],p->packet[10]); DEBUG_OUT.print (cStr);
      sprintf(cStr,"HWPN: %i.%i ",p->packet[11],p->packet[12]); DEBUG_OUT.print (cStr);
      sprintf(cStr,"HWVers: %i.%i ",p->packet[13],p->packet[14]); DEBUG_OUT.print (cStr);
      sprintf(cStr,"APPFVers: %i.%i ",p->packet[15],p->packet[16]); DEBUG_OUT.print (cStr);
      sprintf(cStr,"GPFCode: %i.%i ",p->packet[17],p->packet[18]); DEBUG_OUT.print (cStr);
      sprintf(cStr,"GPFVers:: %i.%i ",p->packet[19],p->packet[20]); DEBUG_OUT.print (cStr);
      SerCmd=0; //stop sending WRInfo
      break;
      case 0xB6:    //4 ports
      case 0xB7:    //4 ports
      case 0xB8:    //4 ports
      case 0xB9:    //4 ports
        MI1500DataMsg(p);
      break;

      case 0x89:    //1-2 ports
      case 0x91:    //2 ports                 ev change with 0x91!!!!!!!!!
        MI600DataMsg(p);
      break;
      case 0x88:    //1-2 ports
      case 0x92:    //2 ports                ev change with 0x92!!!!!!!!!
          MI600StsMsg(p);
      break;
    default:
       sprintf(cStr,"New CMD  %x \t",p->packet[2]); DEBUG_OUT.print (cStr);
       DumpRcvPacket (p, payloadLen); //output received data
    }
}//analayseMI1500----------------------------------------------------------------------------------

void RFAnalyse(void) {
//--------------------------------------------------------------------------------------------------

  while (!packetBuffer.empty()) {
    timeLastPacket = millis();
    // One or more records present
    NRF24_packet_t *p = packetBuffer.getBack();

    // Shift payload data due to 9-bit packet control field
    for (int16_t j = sizeof(p->packet) - 1; j >= 0; j--) {
     if (j > 0)
        p->packet[j] = (byte)(p->packet[j] >> 7) | (byte)(p->packet[j - 1] << 1);
     else
        p->packet[j] = (byte)(p->packet[j] >> 7);
     }

    SerialHdr.timestamp   = p->timestamp;
    SerialHdr.packetsLost = p->packetsLost;
    // Check CRC
    crc = 0xFFFF;
    crc = crc16((uint8_t *)&SerialHdr.address, sizeof(SerialHdr.address), crc, 0, BYTES_TO_BITS(sizeof(SerialHdr.address)));
    // Payload length
    uint8_t payloadLen = ((p->packet[0] & 0x01) << 5) | (p->packet[1] >> 3);
    // Add one byte and one bit for 9-bit packet control field
    crc = crc16((uint8_t *)&p->packet[0], sizeof(p->packet), crc, 7, BYTES_TO_BITS(payloadLen + 1) + 1);

    if (CHECK_CRC) {
      // If CRC is invalid only show lost packets
      if (((crc >> 8) != p->packet[payloadLen + 2]) || ((crc & 0xFF) != p->packet[payloadLen + 3])) {
        if (p->packetsLost > 0) {
          DEBUG_OUT.print(F(" Lost: "));
          DEBUG_OUT.println(p->packetsLost);
        }
        packetBuffer.popBack();
        continue;
      }
      // Dump a decoded packet only once
      if (lastCRC == crc) {
        packetBuffer.popBack();
        continue;
      }
      lastCRC = crc;
    }// if checkcrc
    // Don't dump mysterious ack packages
    if (payloadLen == 0) {
      packetBuffer.popBack();
      //DEBUG_OUT.println(F(" mysterious ack "));
      continue;
    }

    if ( (DEBUG_RCV_DATA) || (SNIFFER) )
      DumpRcvPacket (p, payloadLen); //output received data
    
    if (!SNIFFER)
      AnalyseMI1500(p,payloadLen);

    if (p->packetsLost > 0) {
      DEBUG_OUT.print(F(" Lost: "));
      DEBUG_OUT.println(p->packetsLost);
    }
    // Remove record as we're done with it.
    packetBuffer.popBack();
  } //while (!packetBuffer.empty())  GET PACKET
}//-----RFAnalyse-------------------------------------------------------------------------------

uint8_t HopRcvCh(void){
//----------------------------------------------------------------------------------------------------
 static uint8_t hop=-1;

    hop++;
    if (hop >= sizeof(channels))// / sizeof(channels[0]) )
      hop = 0;
    RcvCH = channels[hop];
    return(RcvCH);
}//----HopRcvCh----------------------------------------------------------------------------------------

uint8_t checkAllPV(void){
//-------------------------------------------------------------------------------------------------
  if (pvCnt[0]+pvCnt[1]+pvCnt[2]+pvCnt[3]==NRofPV)
    return 1;
  else
    return 0;
}//----------------------------------------------------------------------------------------------

void DoZeroExport(void){
//-------------------------------------------------------------------------------------------------

    if (!checkAllPV()){//DEBUG_OUT.println(F("PVs not ready:"));
      return; //not all PV data are in
      }

    if (P_DTSU>0){ //Exporting is PLUS zu viel power
      Limit= PMI - abs(P_DTSU);  //327-abs(296)
      }
    else { //importing is minus zu wenig power
      Limit= PMI+ abs(P_DTSU); //327+abs(-296)
      }

  if ((abs (P_DTSU) > TOLERANCE) || (abs (Limit-OldLimit) > TOLERANCE )){//if change more than 10 watt
        if (Limit < MINPOWER) Limit = 100;
        if (Limit > MAXPOWER) Limit = MAXPOWER;
        SendLimitSts=1; //we can send
        }
  else { Limit=OldLimit; }

}//---DoZeroExport----------------------------------------------------------------------------------

void loop(void) {
//===============================================================================================
  //DEBUG_OUT.print(F("loop\b\b\b\b"));
  //radio1.setChannel(HopRcvCh());//setChannel(DEFAULT_RECV_CHANNEL);
  RcvCH = channels[hoptx];
  radio1.setChannel(RcvCH);//setChannel(DEFAULT_RECV_CHANNEL);
  radio1.startListening();

  if(! INTERRUPT)
    ReadRFBuf(); //polling
  //DEBUG_OUT.print(F("0\b"));delay(10);
    //toe if (istTag)
  RFAnalyse();
  //DEBUG_OUT.print(F("LOOP\b\b\b\b"));
  if (! SNIFFER){
    HandleSerialCmd();  //read from serial any command
    if (ZEROEXP)
        DoZeroExport();
    if (!ONLY_RX)
        isTime2Send();//SEND PACKET
    }

  #ifdef ESP8266
    if (WITHWIFI){
     if (!SNIFFER){
       if (!checkWifi()){
         setup();
         //checkUpdateByOTA();
         }
       if (MQTT) mqttClient.poll();

       if (millis() >= UpdateTick){//not overload the mqtt server
         UpdateTick += UPDATETICK;
         //DEBUG_OUT.print(F("Update Web, Mqtt Services. Are PVs ready? : "));
         //DEBUG_OUT.println(checkAllPV());
         //if (checkAllPV())
           webserverHandle();
         if ((MQTT) && (DataOK) && checkAllPV() ){ //&& (checkAllPV())) {
           //mqttClient.poll();
           SendMQTTMsg("Mi01_totalW", (String) PMI);
           SendMQTTMsg("Limiting", (String) Limit);
           SendMQTTMsg("Mi01pvPower"+(String)(PV+1), (String) P_DC);
           SendMQTTMsg("Mi01_UPv"+(String)(PV+1), (String) U_DC);
           SendMQTTMsg("Mi01_IPv"+(String)(PV+1), (String) I_DC);
           SendMQTTMsg("Mi01EnergiePV"+(String)(PV+1), (String) Q_DC);
           SendMQTTMsg("WRtemp", (String) TEMP);
           SendMQTTMsg("Mi01StsPort"+(String)(PV+1), (String) STAT);
           SendMQTTMsg("ConsumW", (String) (abs(P_DTSU) + PMI));
           SendMQTTMsg("Day", (String)getDateStr(getNow()));
           SendMQTTMsg("Time", (String)getTimeStr(getNow()));
           SendMQTTMsg("Error", "NRF24");
           }
         else {
           //DEBUG_OUT.println(F("No MQTT.."));
           if (!MQTT) MQTT=setupMQTT();
           }
         }
         //UpdtCnt=0;
      }//sniffer
     }//wifi
  #endif //esp8266

}//-----loop-----------------------------------------------------------------------------------------
