/*
 This software is CAN communication software using M5Stack and COMMU module.

  Copyright (c) 2019 aho1go / Tatsuya TAKEHISA
  Released under the MIT license
  https://opensource.org/licenses/mit-license.php

----  
 The following libraries are used:
  https://github.com/coryjfowler/MCP_CAN_lib
  If DEBUG_MODE of mcp_can_dfs.h is 1, please set it to 0.
    mcp_can_dfs.h
      #define DEBUG_MODE 1 -> 0
----
*/

#include <M5Stack.h>
#include "Free_Fonts.h" // Include the header file attached to this sketch

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mcp_can.h"

//#define CANDUMMY

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#define ConvBtoA(a) ("0123456789ABCDEF"[a & 0x0F])
#define CONV_AtoB(a) ((a >= 'a') ? (a - 'a' + 10) : (a >= 'A') ? (a - 'A' + 10) : (a >= 0) ? (a - '0') : 0)


// =================
//  list of Task ID
// =================
#define CANLCD_TASKID   0
#define INPUT_TASKID    1
#define OUTPUT_TASKID   2
#define DOS_TASKID      3
#define TASKMAX         4
TaskHandle_t hTask[TASKMAX];

// =================
//  list of Queue
// =================
QueueHandle_t xCANLCDMsgQue;
QueueHandle_t xCANSendMsgQue;
QueueHandle_t xOutputMsgQue;

// =================
//  Definition of strctured value
// =================
typedef struct OutputMessage {    // 12345678901234567890123456789012
  char msg[32];                   // t99981122334455667788FFFFFFFF
} OUTPUTMESSAGE;
#define OUTPUTMESSAGE_QMAX  40

typedef struct CANMessage {
  unsigned long ID;
  byte          DLC;
  byte          Data[8];
} CANMESSAGE;
#define CANMESSAGE_QMAX  40

typedef struct CANLCDMessage {
  byte msg;
} CANLCDMESSAGE;
#define CANLCDMESSAGE_QMAX  20
#define CANLCDMSG_CANOPEN   0
#define CANLCDMSG_CANCLOSE  1
#define CANLCDMSG_CANRECV   2
#define CANLCDMSG_CANSEND   3
#define CANLCDMSG_LCDDISP   4

const char* M5CAN_HWStr = "M5CAN";
const char* M5CAN_HWVer = "V0100";
const char* M5CAN_SWVer = "v0003";


// =================
//  CAN
// =================
static uint8_t can_speed = 6;
static uint8_t can_mode = MCP_LISTENONLY;
static uint8_t can_tsmode = 0;
volatile int can_rxCnt = 0;
volatile int can_txCnt = 0;
static int can_dosmode = 0;
static uint32_t can_doscycle = 0;
static uint32_t can_dostimes = 0;
static bool can_dosincrement = false;
static CANMESSAGE can_dosmsg;

const INT8U can_speedtbl[] = {
  CAN_10KBPS,
  CAN_20KBPS,
  CAN_50KBPS,
  CAN_100KBPS,
  CAN_125KBPS,
  CAN_250KBPS,
  CAN_500KBPS,
  CAN_500KBPS,    //  unsupport 800kbps. <T.B.D>
  CAN_1000KBPS  
};

#define CAN_CSPIN   12    // M5Stack COMMU CS = G12
#define CAN_INTPIN  15    // M5Stack COMMU INT = G15

MCP_CAN CAN(CAN_CSPIN);


// =================
void IRAM_ATTR CAN_RecvIsr() {
  BaseType_t xHigherPriorityTaskWoken;

  CANLCDMESSAGE  msg;
  msg.msg = CANLCDMSG_CANRECV;
  xQueueSendToFrontFromISR(xCANLCDMsgQue, &msg, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// =================
void CAN_setup() {
  void Display_Init();  

  if( CAN.begin(MCP_STDEXT, can_speedtbl[can_speed], MCP_8MHZ) == CAN_OK) {
    can_rxCnt = 0;
    can_txCnt = 0;

    CAN.setMode(can_mode);
    Display_Init();
  }
}

// =================
void CAN_close() {
  CAN.setMode(MCP_SLEEP);
}

// =================
#ifdef CANDUMMY
void CAN_DummyRecv(CANMESSAGE *msg) {
  if(random(100) < 50) {
    msg->ID = random(0x800);
  } else {
    msg->ID = random(0x20000000) | 0x80000000;
  }      
  if(random(100) < 50) {
    msg->ID |= 0x40000000;
  }      
  msg->DLC = random(9);
  for(char i=0; i<msg->DLC; i++) {
    msg->Data[i] = random(256); 
  }
}
#endif

// =================
//  Wi-Fi
// =================
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

const char *ssid = "M5CAN AP";
const char *password = "m5candoit";
char  ssidbuf[32];

volatile bool WIFI_bConnected = false;

WiFiServer server(23);    //  Telnet port
WiFiClient client;

// =================
void WIFIInit() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  sprintf(ssidbuf, "%s-%02X%02X", ssid, mac[4], mac[5]);
  WiFi.softAP(ssidbuf, password);
  
  server.begin();  
}

// =================
bool inline WIFIConnected() {
  return WIFI_bConnected;
}

// =================
char WIFI_recvBuf[32];
char WIFI_recvCnt;

void WIFILoop() {
  
  if(client.connected()) {
    WIFI_bConnected = true;
    
    while(client.available()) {             // if there's bytes to read from the client,
      char c = client.read();             // read a byte, then
      input_slcancmd(WIFI_recvBuf, sizeof(WIFI_recvBuf), &WIFI_recvCnt, c);
    }
  } else {
    if(WIFI_bConnected) {
      WIFI_bConnected = false;
      client.stop();
    } else {
      if(server.hasClient()) {
        client = server.available();   // listen for incoming clients
        //  Serial.println("WiFi Client Connect");
      }
    }
  }
}

// =================
void WIFISend(char* msg) {
  if(WIFIConnected()) {
    client.println(msg);
  }
}


// =================
//  slcan command
// =================
bool parse_slcancmd(char *cmd)
{
  bool bResult = true;
  OUTPUTMESSAGE outmsg;
  CANLCDMESSAGE  evtmsg;

//  Serial.println(cmd);

  outmsg.msg[0] = '\0';
  
  switch (cmd[0])
  {
    case 'S': { //  Setup with standard CAN bit-rates where n is 0-8.
      can_speed = cmd[1] - '0';
      if (can_speed > 9) {
          can_speed = 6; //  set default
          bResult = false;
      }
      break;
    }
    case 'O': { //  Open the CAN channel in normal mode (sending & receiving).
      can_dosmode = 0;
      can_mode = MCP_NORMAL;
      evtmsg.msg = CANLCDMSG_CANOPEN;
      xQueueSendToFront(xCANLCDMsgQue, &evtmsg, NULL);
      break;
    }
    case 'l': { //  Open the CAN channel in loopback mode (loopback & receiving).
      can_dosmode = 0;
      can_mode = MCP_LOOPBACK;
      evtmsg.msg = CANLCDMSG_CANOPEN;
      xQueueSendToFront(xCANLCDMsgQue, &evtmsg, NULL);
      break;
    }
    case 'L': { //  Open the CAN channel in listen only mode (receiving).
      can_dosmode = 0;
      can_mode = MCP_LISTENONLY;
      evtmsg.msg = CANLCDMSG_CANOPEN;
      xQueueSendToFront(xCANLCDMsgQue, &evtmsg, NULL);
      break;
    }
    case 'C': { //  Close the CAN channel.
      evtmsg.msg = CANLCDMSG_CANCLOSE;
      xQueueSendToFront(xCANLCDMsgQue, &evtmsg, NULL);
      break;
    }
    case 'D': { //  Open the CAN channel in DoS and normal mode (sending & receiving)
      //  Dttttttnnnn
      if(strlen(cmd) < 11) {
        bResult = false;
      } else {
        if(strlen(cmd) == 12) {
          can_dosincrement = (cmd[11] == '1' ? true: false);  
          cmd[11] = '\0';
        }
        can_dostimes = atoi(&cmd[7]);
        cmd[7] = '\0';
        can_doscycle = atol(&cmd[1]);
        can_dosmode = 1;
        can_mode = MCP_NORMAL;
        evtmsg.msg = CANLCDMSG_CANOPEN;
        xQueueSendToFront(xCANLCDMsgQue, &evtmsg, NULL);
      }
      break;
    }
    case 't': //  Transmit a standard (11bit) CAN frame.
    case 'T': //  Transmit an extended (29bit) CAN frame.
    case 'r': //  Transmit a standard RTR (11bit) CAN frame.
    case 'R': //  Transmit an extended RTR (29bit) CAN frame.
    {
      CANMESSAGE  canmsg;
  
      bResult = false;
  
      if(cmd[0]=='t' || cmd[0]=='r') {
        if ((strlen(cmd) >= 5) && (strlen(cmd) <= 21)) {
          canmsg.ID = CONV_AtoB(cmd[1]) << 8 | CONV_AtoB(cmd[2]) << 4 | CONV_AtoB(cmd[3]);
          canmsg.DLC = CONV_AtoB(cmd[4]);
          if (canmsg.DLC < 9) {
            if(cmd[0]=='t') {
              if (strlen(cmd) == (size_t)(5 + canmsg.DLC * 2)) {
                for (int i = 0; i < canmsg.DLC; i++) {
                  canmsg.Data[i] = CONV_AtoB(cmd[5 + i * 2 + 0]) << 4 | CONV_AtoB(cmd[5 + i * 2 + 1]);
                }
                canmsg.ID = canmsg.ID | 0x00000000;
                bResult = true;
              }
            } else {
                canmsg.ID = canmsg.ID | 0x40000000;
                bResult = true;
            }
          }
        }
      } else if(cmd[0]=='T' || cmd[0]=='R') {
        if ((strlen(cmd) >= 9) && (strlen(cmd) <= 26)) {
          canmsg.ID = CONV_AtoB(cmd[1]) << 28 | CONV_AtoB(cmd[2]) << 24 | CONV_AtoB(cmd[3]) << 20 |
                       CONV_AtoB(cmd[4]) << 16 | CONV_AtoB(cmd[5]) << 12 | CONV_AtoB(cmd[6]) << 8 |
                       CONV_AtoB(cmd[7]) << 4 | CONV_AtoB(cmd[8]);
          canmsg.DLC = CONV_AtoB(cmd[9]);
          if (canmsg.DLC < 9) {
            if(cmd[0]=='T') {
              if (strlen(cmd) == (size_t)(10 + canmsg.DLC * 2)) {
                for (int i = 0; i < canmsg.DLC; i++) {
                    canmsg.Data[i] = CONV_AtoB(cmd[10 + i * 2 + 0]) << 4 | CONV_AtoB(cmd[10 + i * 2 + 1]);
                }
                canmsg.ID = canmsg.ID | (0x80000000);
                bResult = true;
              }
            } else {
                canmsg.ID = canmsg.ID | (0x40000000|0x80000000);
                bResult = true;
            }
          }
        }
      }
  
      if(bResult) {
        if(can_dosmode == 1) {
          can_dosmsg.ID = canmsg.ID;
          can_dosmsg.DLC = canmsg.DLC;
          memcpy(can_dosmsg.Data, canmsg.Data, sizeof(can_dosmsg.Data)); 
          can_dosmode = 2;
        } else {
          CANLCDMESSAGE  evtmsg;
  
          xQueueSend(xCANSendMsgQue, &canmsg, NULL);
          evtmsg.msg = CANLCDMSG_CANSEND;
          xQueueSendToFront(xCANLCDMsgQue, &evtmsg, NULL);
        }
      }
      break;
    }
    case 'V': //  Get Version number of both CAN232 hardware and software
    {
      strcpy(outmsg.msg, M5CAN_HWVer);
      break;
    }
    case 'v': {
      strcpy(outmsg.msg, M5CAN_SWVer);
      break;
    }
    case 'N': { //  Get Serial number of the CAN232.
      uint8_t mac[6];
      esp_read_mac(mac, ESP_MAC_WIFI_STA);
      sprintf(outmsg.msg, "N%02X%02X", mac[4], mac[5]);
      break;
    }
    case 'Z': { //  Sets Time Stamp ON/OFF for received frames only. EXTENSION to LAWICEL: Z2 - millis() timestamp w/o standard 60000ms cycle
      can_tsmode = cmd[1] - '0';
      break;
    }
    //  Not implements
    case 'P': //  Poll incomming FIFO for CAN frames (single poll)
    case 'A': //  Polls incomming FIFO for CAN frames (all pending frames)
    case 'F': //  Read Status Flags.
    case 'X': //  Sets Auto Poll/Send ON/OFF for received frames.
    case 'U': //  Setup UART with a new baud rate where n is 0-6.
    case 'Q': //  Auto Startup feature (from power on).
    case 's': //  Setup with BTR0/BTR1 CAN bit-rates where xx and yy is a hex value.
    case 'W': //  Filter mode setting. By default CAN232 works in dual filter mode (0) and is backwards compatible with previous CAN232 versions.
    case 'M': //  Sets Acceptance Code Register (ACn Register of SJA1000).
    case 'm': //  Sets Acceptance Mask Register (AMn Register of SJA1000).
    default: {
        break;
    }
  }

  if(strlen(outmsg.msg) > 0) {
    xQueueSend(xOutputMsgQue, &outmsg, NULL);
  } else {
    if(bResult) {
      strcpy(outmsg.msg, "\r");
    } else {
      strcpy(outmsg.msg, "\a");
    }
    xQueueSend(xOutputMsgQue, &outmsg, NULL);
  }

  return bResult;
}

// =================
bool input_slcancmd(char* buf, char bufmax, char* pCnt, char ch) {
  char len = *pCnt;
  
  if(ch == '\r' || ch == '\n') {
    if(len > 0) {
      buf[len] = 0;
      parse_slcancmd(buf);
    }
    len = 0;
    strcpy(buf, "");
  } else {
    if(*pCnt < (bufmax-1)) {
      buf[len++] = ch;
      buf[len] = 0;
    } else {
      len = 0;
      strcpy(buf, "");
    }
  }
  *pCnt = len;
}


// =================
//  Serial
// =================
char serial_recvBuf[32];
char serial_recvCnt = 0;

void SERIALLoop() {
  char ch;

  while(Serial.available()) {
    ch = Serial.read();
    input_slcancmd(serial_recvBuf, sizeof(serial_recvBuf), &serial_recvCnt, ch);
  }
}


// =================
//  Serial/Wi-Fi Input Task
// =================
void Input_Task(void *pvParameters) {
  while(true) {
    SERIALLoop();
    vTaskDelay(1);
    WIFILoop();
    vTaskDelay(1);
  }
}

// =================
//  Serial/Wi-Fi Output Task
// =================
void Output_Task(void *pvParameters) {
  OUTPUTMESSAGE msg;

  while(true) {
    while(xQueueReceive(xOutputMsgQue, &msg, portMAX_DELAY)) {
      if(WIFIConnected()) {
        WIFISend(msg.msg);
      } else {
        Serial.println(msg.msg);
      }
    }
  }
}



// =================
//  Disp 
// =================
static int display_sequencer = 0;

// =================
void Display_Init() {
  display_sequencer = -1;
}

// =================
void Display_Banner() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setFreeFont(FF1);
  M5.Lcd.drawString(M5CAN_HWStr, (int)(M5.Lcd.width()/2), (int)(M5.Lcd.height()/2));
  M5.update();
  vTaskDelay(1000);
  M5.Lcd.fillScreen(BLACK);
  M5.update();
  display_sequencer = -1;
}

// =================
void Display() {
  static int xpos, ypos;
  char buf[40];
  
  switch(display_sequencer) {
    case -1: {
      M5.Lcd.fillScreen(BLACK);
      break;
    }
    case 0: {
      xpos = 0;
      ypos = 0;
      
      //  Indicate Wi-Fi Connection
      if(WIFIConnected()) {
        M5.Lcd.setTextColor(BLACK, GREEN);
      } else {
        M5.Lcd.setTextColor(WHITE, BLACK);
      }
      M5.Lcd.drawString("Wi-Fi", 120, 0);
      break;  
    }
    case 1: {
      M5.Lcd.setFreeFont(FM9);
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(0,24);
      M5.Lcd.drawString(M5CAN_HWStr, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case 2: {
      sprintf(buf,"Rx:%d\r\n", can_rxCnt);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case 3: {
      sprintf(buf,"Tx:%d\r\n", can_txCnt);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case 4: {
      sprintf(buf,"SSID:%s\r\n", ssidbuf);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case 5: {
      IPAddress myIP = WiFi.softAPIP();
      sprintf(buf,"IP:%d.%d.%d.%d\r\n", myIP[0], myIP[1], myIP[2], myIP[3]);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    default: {
      M5.update();
      display_sequencer = -1;
      break;
    }
  }
  display_sequencer ++;
}


// =================
//  CAN/LCD gatekeeper Task
// =================
void CANLCD_Task(void *pvParameters) {
  CANMESSAGE canmsg;
  CANLCDMESSAGE evtmsg;
  OUTPUTMESSAGE outmsg;
  char* msgbuf = outmsg.msg;
  char msgbufIdx;
  unsigned long tm;

  pinMode(CAN_INTPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN_INTPIN), CAN_RecvIsr, FALLING);
  vTaskDelay(10);
  CAN_setup();
 
  while(true) {
    while(xQueueReceive(xCANLCDMsgQue, &evtmsg, portMAX_DELAY)) {
      switch(evtmsg.msg) {
        case CANLCDMSG_CANRECV: {
#ifndef CANDUMMY
          while(CAN.checkReceive()==CAN_MSGAVAIL) {
            CAN.readMsgBuf(&canmsg.ID, &canmsg.DLC, canmsg.Data);
#else
          if(true) {
            CAN_DummyRecv(&canmsg);
#endif
            if(can_tsmode==1) {
              tm = millis();
            } else if(can_tsmode==9) {
              tm = micros();
            }

            msgbufIdx = 0;
            //  Set prefix (t,T,r,R)
            msgbuf[msgbufIdx++] = ((canmsg.ID & 0x80000000) ?
                                    ((canmsg.ID & 0x40000000) ? 'R' : 'T') :
                                    ((canmsg.ID & 0x40000000) ? 'r' : 't'));
            if(canmsg.ID & 0x80000000) {
              //  Extended ID
              canmsg.ID &= 0x1FFFFFFF;
              msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>28); msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>24);
              msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>20); msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>16);
              msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>12); msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>8);
              msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>4);  msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>0);
            } else {
              //  Normal ID
              msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>8);
              msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>4);  msgbuf[msgbufIdx++] = ConvBtoA(canmsg.ID>>0);
            }
            //  Set DLC
            canmsg.DLC = min(canmsg.DLC, (byte)8);
            msgbuf[msgbufIdx++] = ConvBtoA(canmsg.DLC);
            if((canmsg.ID & 0x40000000) == 0) {  // RTR is not contain DATA FIELD
              //  Set Payload Data
              for(int i=0; i<canmsg.DLC; i++) {
                msgbuf[msgbufIdx++] = ConvBtoA(canmsg.Data[i]>>4); msgbuf[msgbufIdx++] = ConvBtoA(canmsg.Data[i]>>0);
              }
            }
            if(can_tsmode==1) {
              tm = tm % 60000;    //  LAWICEL CAN232 - Z command
              msgbuf[msgbufIdx++] = ConvBtoA(tm>>12); msgbuf[msgbufIdx++] = ConvBtoA(tm>>8);
              msgbuf[msgbufIdx++] = ConvBtoA(tm>>4);  msgbuf[msgbufIdx++] = ConvBtoA(tm>>0);
            } else if(can_tsmode==9) {
              msgbuf[msgbufIdx++] = ConvBtoA(tm>>28); msgbuf[msgbufIdx++] = ConvBtoA(tm>>24);
              msgbuf[msgbufIdx++] = ConvBtoA(tm>>20); msgbuf[msgbufIdx++] = ConvBtoA(tm>>16);
              msgbuf[msgbufIdx++] = ConvBtoA(tm>>12); msgbuf[msgbufIdx++] = ConvBtoA(tm>>8);
              msgbuf[msgbufIdx++] = ConvBtoA(tm>>4);  msgbuf[msgbufIdx++] = ConvBtoA(tm>>0);
            }
            //  Set Delimiter and terminator
            msgbuf[msgbufIdx++] = '\r';
            msgbuf[msgbufIdx++] = '\0';

            xQueueSend(xOutputMsgQue, &outmsg, NULL);
            can_rxCnt ++;
          }
          break;
        }
        case CANLCDMSG_CANOPEN: {
          CAN_setup();
          if(can_dosmode != 0) {
            CAN.enOneShotTX();
          } else {
            CAN.disOneShotTX();
          }
          break;
        }
        case CANLCDMSG_CANCLOSE: {
          CAN_close();
          break;
        }
        case CANLCDMSG_CANSEND: {
          xQueueReceive(xCANSendMsgQue, &canmsg, portMAX_DELAY);
          CAN.sendMsgBuf(canmsg.ID, canmsg.DLC, (byte*)canmsg.Data);
          can_txCnt ++;
          break;
        }
        case CANLCDMSG_LCDDISP: {
          Display();
          break;
        }
      }  
    }
  }
}


// =================
//  DoS Task
// =================
void Dos_IncMsg(void) {
  uint64_t  cnt = 0;
  byte*  cnt_wk = (byte*)&cnt;

  if(can_dosmsg.DLC > 0) {
    for(int i=0; i<(int)can_dosmsg.DLC; i++)  cnt_wk[i] = can_dosmsg.Data[can_dosmsg.DLC-1-i];
    cnt ++;
    for(int i=0; i<(int)can_dosmsg.DLC; i++)  can_dosmsg.Data[i] = cnt_wk[can_dosmsg.DLC-1-i];
  }
}

void DoS_Task(void *pvParameters) {
  while(true) {
    if(can_dosmode==2) {
      CANLCDMESSAGE  evtmsg;

      for(uint32_t i=0; i<can_dostimes; i++) {
        xQueueSend(xCANSendMsgQue, &can_dosmsg, NULL);
        evtmsg.msg = CANLCDMSG_CANSEND;
        xQueueSend(xCANLCDMsgQue, &evtmsg, NULL);
        if(can_dosincrement) {
          Dos_IncMsg();
        }
        vTaskDelay(can_doscycle);
      }
      can_dosmode = 0;
    }
    vTaskDelay(100);
  }
}


// =================
//  Initializer
// =================
void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Serial.begin(2000000); //115200*4);
  Serial.println("");
  
  Display_Banner();

  xCANLCDMsgQue = xQueueCreate(CANLCDMESSAGE_QMAX, sizeof(CANLCDMESSAGE));
  xCANSendMsgQue = xQueueCreate(CANMESSAGE_QMAX, sizeof(CANMESSAGE));
  xOutputMsgQue = xQueueCreate(OUTPUTMESSAGE_QMAX, sizeof(OUTPUTMESSAGE));

  WIFIInit();

  xTaskCreatePinnedToCore(Output_Task,"Output_Task", 4096, NULL, 2, &hTask[OUTPUT_TASKID], 1);
  xTaskCreatePinnedToCore(Input_Task,"Input_Task", 4096, NULL, 4, &hTask[INPUT_TASKID], 1);
  xTaskCreatePinnedToCore(CANLCD_Task,"CANLCD_Task", 4096, NULL, 5, &hTask[CANLCD_TASKID], 0);
  xTaskCreatePinnedToCore(DoS_Task,"DoS_Task", 2048, NULL, 3, &hTask[DOS_TASKID], 1);
}

// =================
//  Idle Task
// =================
static int loopCnt = 0;

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.printf("Loop():%d\r\n", loopCnt++);
  CANLCDMESSAGE  msg;
  msg.msg = CANLCDMSG_LCDDISP;
  xQueueSend(xCANLCDMsgQue, &msg, NULL);
  vTaskDelay(100);
#ifdef CANDUMMY
  msg.msg = CANLCDMSG_CANRECV;
  xQueueSendToFront(xCANLCDMsgQue, &msg, NULL);
#endif
}
