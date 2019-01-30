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
#include <esp_wpa2.h>
#include "SPIFFS.h"


// =================
#define ConvBtoA(a) ("0123456789ABCDEF"[a & 0x0F])
#define CONV_AtoB(a) ((a >= 'a') ? (a - 'a' + 10) : (a >= 'A') ? (a - 'A' + 10) : (a >= 0) ? (a - '0') : 0)


// =================
//  list of Task ID
// =================
#define CANLCD_TASKID   0
#define INPUT_TASKID    1
#define OUTPUT_TASKID   2
#define PERIODIC_TASKID      3
#define TASKMAX         4
TaskHandle_t hTask[TASKMAX];

// =================
//  list of Queue
// =================
QueueHandle_t xCANLCDMsgQue;
QueueHandle_t xCANSendMsgQue;
QueueHandle_t xOutputMsgQue;


// =================
//  list of Event
// =================
#define CANLCDEVT_CANOPEN   (1 << 0)
#define CANLCDEVT_CANCLOSE  (1 << 1)
#define CANLCDEVT_CANRECV   (1 << 2)
#define CANLCDEVT_CANSEND   (1 << 3)
#define CANLCDEVT_LCDDISP   (1 << 4)
#define CANLCDEVT_ALL       (CANLCDEVT_CANOPEN | CANLCDEVT_CANCLOSE | CANLCDEVT_CANRECV | CANLCDEVT_CANSEND | CANLCDEVT_LCDDISP)

EventGroupHandle_t  xCANLCDEvtGrp;


// =================
//  Definition of strctured value
// =================
typedef struct OutputMessage {    // 12345678901234567890123456789012
  char msg[32];                   // t99981122334455667788FFFFFFFF
} OUTPUTMESSAGE;
#define OUTPUTMESSAGE_QMAX  100

typedef struct CANMessage {
  unsigned long ID;
  byte          DLC;
  byte          Data[8];
} CANMESSAGE;
#define CANMESSAGE_QMAX  40

File file;
const char* M5CAN_HWStr = "M5CAN";
const char* M5CAN_HWVer = "V0100";
const char* M5CAN_SWVer = "v0006";

#define RECVLOGMAX  5
OUTPUTMESSAGE recvlog[RECVLOGMAX];
int recvlog_idx = 0;


// =================
//  CAN
// =================
static uint8_t can_speed = 6;
static uint8_t can_mode = MCP_LISTENONLY;
static uint8_t can_tsmode = 0;
volatile int can_rxCnt = 0;
volatile int can_txCnt = 0;
volatile int can_rxErrCnt = 0;
volatile int can_txErrCnt = 0;
volatile int can_rxPPS = 0;
static int can_periodicmode = 0;
static uint32_t can_periodiccycle = 0;
static uint32_t can_periodictimes = 0;
static bool can_periodicincrement = false;
static CANMESSAGE can_periodicmsg;

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
  BaseType_t xHigherPriorityTaskWoken, xResult;

  xResult = xEventGroupSetBitsFromISR(xCANLCDEvtGrp, CANLCDEVT_CANRECV, &xHigherPriorityTaskWoken);
  if(xResult != pdFAIL) {
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
//  Beep
// =================
#include <driver/dac.h>
void beep() {
  // Beep (silent)
  dac_output_enable(DAC_CHANNEL_1);
  for(int i=0; i<10; i++) {
    dacWrite(SPEAKER_PIN, 6);
    delayMicroseconds(2272/2);
    dacWrite(SPEAKER_PIN, 0);
    delayMicroseconds(2272/2);
  }
  dac_output_disable(DAC_CHANNEL_1);
}


// =================
//  Wi-Fi
// =================
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

char WIFI_ssid[64];
char WIFI_pass[64];
char WIFI_idet[64];
char WIFI_user[64];

volatile bool WIFI_bConnected = false;
bool WIFI_bClient = false;
bool WIFI_bClientWPA2 = false;
bool WIFI_bFileExist = false;

WiFiServer server(23);    //  Telnet port
WiFiClient client;


// =================
void WIFIInit() {

  WiFi.disconnect(true);

  if(WIFI_bFileExist) {
    WIFI_bClient = true;
    if((strlen(WIFI_idet) == 0) && (strlen(WIFI_user) == 0)) {
      WIFI_bClientWPA2 = false;
    } else {
      WIFI_bClientWPA2 = true;
    }
  }

  if(WIFI_bClient) {
    WiFi.mode(WIFI_STA);
    if(WIFI_bClientWPA2) {
      esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)WIFI_idet, strlen(WIFI_idet)); 
      esp_wifi_sta_wpa2_ent_set_username((uint8_t *)WIFI_user, strlen(WIFI_user)); 
      esp_wifi_sta_wpa2_ent_set_password((uint8_t *)WIFI_pass, strlen(WIFI_pass)); 
      esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT();
      esp_wifi_sta_wpa2_ent_enable(&config);
      WiFi.begin(WIFI_ssid);
    } else {
      WiFi.begin(WIFI_ssid, WIFI_pass); 
    }

    for(int i=0;i<100;i++) {
      if(WiFi.status() == WL_CONNECTED) {
        break;
      }
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.setFreeFont(FM12);
      M5.Lcd.drawRightString("Wait for Wi-Fi Connect", (M5.Lcd.width()/4)*2, M5.Lcd.height()/2, GFXFF);
      M5.update();
      delay(100);
    }
  } else {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(WIFI_ssid, "M5CAN AP-%02X%02X", mac[4], mac[5]);
    strcpy(WIFI_pass, "m5candoit");
    WiFi.softAP(WIFI_ssid, WIFI_pass);
  }

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
    
    while(client.available()) {
      bool input_slcancmd(char* buf, char bufmax, char* pCnt, char ch, bool bFile);
      char c = client.read(); 
      input_slcancmd(WIFI_recvBuf, sizeof(WIFI_recvBuf), &WIFI_recvCnt, c, false);
    }
  } else {
    if(WIFI_bConnected) {
      WIFI_bConnected = false;
      client.stop();
    } else {
      if(server.hasClient()) {
        client = server.available();
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
//  SD
// =================
char file1[128];
char file2[128];
char file3[128];

// =================
bool readFile(fs::FS &fs, const char * path, char* buf, int len, int* plen) {
  File file = fs.open(path);
  bool bResult = false;
  
  if(file) {
    if((file.size()+1) < len) {
      file.readBytes(buf, file.size());
      buf[file.size()] = '\0';
    }
    if(plen != NULL) {
      *plen = file.size();
    }
    file.close();
    bResult = true;
  }
//  Serial.printf("readFile:%s:[%s]\r\n", path, buf);
  return bResult;
}

// =================
bool writeFile(fs::FS &fs, const char * path, char* buf, int len) {
  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    return false;
  }
  file.write((uint8_t*)buf, len);
  file.close();
//  Serial.printf("writeFile:%s:[%s]\r\n", path, buf);
  return true;
}

// =================
String readline(File& file) {
  String str = "";
  
  while(file.available() > 0) {
    char ch = file.read();
    if(ch=='\r' || ch=='\n') {
      if(str.startsWith("#")) {
        str = "";
        continue;
      }
      break;
    }
    str += ch;
  }
//  Serial.printf("readline:[%s]\r\n", str.c_str());
  return str;
}

// =================
void ReadWifiFile(const char* path, char* buf1, int buf1len, char* buf2, int buf2len, char* buf3, int buf3len, char* buf4, int buf4len) {
  char buf[1024];
  int len = sizeof(buf);
  
  memset(buf, 0, len);

  if(SD.exists(path)) {
    int flen;
    if(readFile(SD, path, buf, len, &flen)) {
      writeFile(SPIFFS, path, buf, flen);
    }
  }

  if(SPIFFS.exists(path)) {
    File file = SPIFFS.open(path);
    if(file) {
      String str;
      str = readline(file);
      strncpy(buf1, str.c_str(), buf1len);
      str = readline(file);
      strncpy(buf2, str.c_str(), buf2len);
      str = readline(file);
      strncpy(buf3, str.c_str(), buf3len);
      str = readline(file);
      strncpy(buf4, str.c_str(), buf4len);    
      file.close();
  
      WIFI_bFileExist = true;
    }
  }
}

// =================
void ReadButtonFile(const char* path, char* buf, int len) {
  memset(buf, 0, len);

  if(SD.exists(path)) {
    int flen;
    if(readFile(SD, path, buf, len, &flen)) {
      writeFile(SPIFFS, path, buf, flen);
    }
  }
  readFile(SPIFFS, path, buf, len, NULL);
}

// =================
void FILE_setup(void) {

  SPIFFS.begin(true);
  SD.begin();

  memset(file1, 0, sizeof(file1));
  memset(file2, 0, sizeof(file2));
  memset(file3, 0, sizeof(file3));

  M5.update();
  if(M5.BtnA.isPressed()) {
    beep();
    vTaskDelay(100/portTICK_RATE_MS);
    beep();
    vTaskDelay(100/portTICK_RATE_MS);
    beep();
    vTaskDelay(100/portTICK_RATE_MS);

    SPIFFS.format();
  }
  
  ReadButtonFile("/button1.txt", file1, sizeof(file1));
  ReadButtonFile("/button2.txt", file2, sizeof(file2));
  ReadButtonFile("/button3.txt", file3, sizeof(file3));

  ReadWifiFile("/wifi.txt", 
                WIFI_ssid, sizeof(WIFI_ssid),
                WIFI_pass, sizeof(WIFI_pass),
                WIFI_idet, sizeof(WIFI_idet),
                WIFI_user, sizeof(WIFI_user) );

}


// =================
//  slcan command
// =================
bool parse_slcancmd(char *cmd)
{
  bool bResult = true;
  OUTPUTMESSAGE outmsg;

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
      can_periodicmode = 0;
      can_mode = MCP_NORMAL;
      xEventGroupSetBits(xCANLCDEvtGrp, CANLCDEVT_CANOPEN);
      break;
    }
    case 'l': { //  Open the CAN channel in loopback mode (loopback & receiving).
      can_periodicmode = 0;
      can_mode = MCP_LOOPBACK;
      xEventGroupSetBits(xCANLCDEvtGrp, CANLCDEVT_CANOPEN);
      break;
    }
    case 'L': { //  Open the CAN channel in listen only mode (receiving).
      can_periodicmode = 0;
      can_mode = MCP_LISTENONLY;
      xEventGroupSetBits(xCANLCDEvtGrp, CANLCDEVT_CANOPEN);
      break;
    }
    case 'C': { //  Close the CAN channel.
      xEventGroupSetBits(xCANLCDEvtGrp, CANLCDEVT_CANCLOSE);
      break;
    }
    case 'D': { //  Open the CAN channel in Periodic send and normal mode (sending & receiving)
      //  Dttttttnnnn
      if(strlen(cmd) < 11) {
        bResult = false;
      } else {
        if(strlen(cmd) == 12) {
          can_periodicincrement = (cmd[11] == '1' ? true: false);  
          cmd[11] = '\0';
        }
        can_periodictimes = atoi(&cmd[7]);
        cmd[7] = '\0';
        can_periodiccycle = atol(&cmd[1]);
        can_periodicmode = 1;
        can_mode = MCP_NORMAL;
        xEventGroupSetBits(xCANLCDEvtGrp, CANLCDEVT_CANOPEN);
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
        if(can_periodicmode == 1) {
          can_periodicmsg.ID = canmsg.ID;
          can_periodicmsg.DLC = canmsg.DLC;
          memcpy(can_periodicmsg.Data, canmsg.Data, sizeof(can_periodicmsg.Data)); 
          can_periodicmode = 2;
        } else {
          xQueueSend(xCANSendMsgQue, &canmsg, NULL);
          xEventGroupSetBits(xCANLCDEvtGrp, CANLCDEVT_CANSEND);
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
bool input_slcancmd(char* buf, char bufmax, char* pCnt, char ch, bool bFile) {
  char len = *pCnt;

  if(bFile) {
    if(ch == ',') {
      ch = '\r';
    }
  }
  
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
    input_slcancmd(serial_recvBuf, sizeof(serial_recvBuf), &serial_recvCnt, ch, false);
  }
}


// =================
//  File
// =================
char* file_chrptr = NULL;
char file_recvBuf[32];
char file_recvCnt = 0;

void FILELoop() {
  char ch;

  if(M5.BtnA.wasPressed()) {
    if(file_chrptr == NULL) {
      file_chrptr = file1;
      beep();
      vTaskDelay(100/portTICK_RATE_MS);
    }
  }

  if(M5.BtnB.wasPressed()) {
    if(file_chrptr == NULL) {
      file_chrptr = file2;
      beep();
      vTaskDelay(100/portTICK_RATE_MS);
    }
  }

  if(M5.BtnC.wasPressed()) {
    if(file_chrptr == NULL) {
      file_chrptr = file3;
      beep();
      vTaskDelay(100/portTICK_RATE_MS);
    }
  }
  
  if(file_chrptr != NULL) {
    ch = *file_chrptr ++;
    if(ch == '\0') {
      file_chrptr = NULL;
      ch = '\r';
    }
    input_slcancmd(file_recvBuf, sizeof(file_recvBuf), &file_recvCnt, ch, true);
  }
}


// =================
//  Serial/Wi-Fi/File Input Task
// =================
void Input_Task(void *pvParameters) {
  while(true) {
    SERIALLoop();
    vTaskDelay(1/portTICK_RATE_MS);
    WIFILoop();
    vTaskDelay(1/portTICK_RATE_MS);
    FILELoop();
    vTaskDelay(1/portTICK_RATE_MS);
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
  display_sequencer = -4;
}

// =================
void Display_Banner() {
  int xpos, ypos;
  
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setFreeFont(FM12);
  xpos = (M5.Lcd.width()/4)*3;
  ypos = M5.Lcd.height()/2;
  M5.Lcd.drawRightString(M5CAN_HWStr, xpos, ypos, GFXFF);
  M5.Lcd.setFreeFont(FM9);
  ypos += M5.Lcd.fontHeight(GFXFF)*2;
  M5.Lcd.drawRightString(M5CAN_SWVer, xpos, ypos, GFXFF);
  ypos += M5.Lcd.fontHeight(GFXFF);
  M5.Lcd.drawRightString("Useful CAN Tool", xpos, ypos, GFXFF);
  M5.update();
  vTaskDelay(1000/portTICK_RATE_MS);
  M5.Lcd.fillScreen(BLACK);
  M5.update();
  display_sequencer = -4;
}

// =================
void Display() {
  static int xpos, ypos;
  char buf[40];
  unsigned long tm;

  tm = micros();
  switch(display_sequencer) {
    case -4: {
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextFont(FONT8);
      xpos = 0;
      ypos = 0;
      break;
    }
    case -3: {
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(0,24);
      M5.Lcd.drawString(M5CAN_HWStr, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case -2: {
      sprintf(buf,"SSID:%s", WIFI_ssid);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case -1: {
      IPAddress myIP;
      
      if(WIFI_bClient) {
        sprintf(buf,"IP:%s", WiFi.localIP().toString().c_str());
      } else {
        sprintf(buf,"IP:%s", WiFi.softAPIP().toString().c_str());
      }
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case 0: {
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextFont(FONT8);
      xpos = 0;
      ypos = M5.Lcd.fontHeight(GFXFF)*4;
      //  Indicate Wi-Fi Connection
      if(WIFIConnected()) {
        M5.Lcd.setTextColor(BLACK, GREEN);
      } else {
        M5.Lcd.setTextColor(WHITE, BLACK);
      }
      M5.Lcd.drawString("Wi-Fi", 240, 0, GFXFF);
      M5.Lcd.setTextColor(WHITE, BLACK);
      break;  
    }
    case 1: {
      sprintf(buf,"Rx:%d", can_rxCnt);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case 2: {
      sprintf(buf,"Tx:%d", can_txCnt);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case 3: {
      sprintf(buf,"RxErr:%d", can_rxErrCnt);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case 4: {
      sprintf(buf,"TxErr:%d", can_txErrCnt);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF);
      break;
    }
    case 5: {
      sprintf(buf,"RxPPS:%d", can_rxPPS);
      M5.Lcd.drawString(buf, xpos, ypos, GFXFF);
      ypos += M5.Lcd.fontHeight(GFXFF)*2;
      break;
    }
    case 6:
    case 7:
    case 8:
    case 9:
    case 10: {
      int idx = (recvlog_idx+display_sequencer-6) % RECVLOGMAX;
      
      M5.Lcd.setTextSize(1);
      sprintf(buf,recvlog[idx].msg);
      for(int i=strlen(buf); i<26+8; i++) {
        strcat(buf, " ");
      }
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
  OUTPUTMESSAGE outmsg;
  char* msgbuf = outmsg.msg;
  char msgbufIdx;
  unsigned long tm;

  pinMode(CAN_INTPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN_INTPIN), CAN_RecvIsr, FALLING);
  vTaskDelay(10/portTICK_RATE_MS);
  CAN_setup();
 
  while(true) {
    EventBits_t uxBits;
    
    uxBits = xEventGroupWaitBits(
              xCANLCDEvtGrp,
              CANLCDEVT_ALL,
              pdTRUE,
              pdFALSE,
              portMAX_DELAY );

    if(uxBits & CANLCDEVT_CANRECV) {
      if(CAN.checkError()==CAN_CTRLERROR) {
        can_rxErrCnt += CAN.errorCountRX();
        can_txErrCnt += CAN.errorCountTX();
      }

      while(CAN.checkReceive()==CAN_MSGAVAIL) {
        CAN.readMsgBuf(&canmsg.ID, &canmsg.DLC, canmsg.Data);
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
        canmsg.DLC = (canmsg.DLC < (byte)8) ? canmsg.DLC : (byte)8;
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

        strcpy(recvlog[can_rxCnt % RECVLOGMAX].msg, outmsg.msg);
        if(can_rxCnt >= RECVLOGMAX) {
          recvlog_idx = (recvlog_idx+1) % RECVLOGMAX;
        }

        xQueueSend(xOutputMsgQue, &outmsg, NULL);
        can_rxCnt ++;
      }
    }
    if(uxBits & CANLCDEVT_CANOPEN) {
      CAN_setup();
      if(can_periodicmode != 0) {
        CAN.enOneShotTX();
      } else {
        CAN.disOneShotTX();
      }
      for(int i=0; i<RECVLOGMAX; i++) {
        strcpy(recvlog[recvlog_idx].msg, "");
      }
    }
    if(uxBits & CANLCDEVT_CANCLOSE) {
      CAN_close();
    }
    if(uxBits & CANLCDEVT_CANSEND) {
      xQueueReceive(xCANSendMsgQue, &canmsg, portMAX_DELAY);
      CAN.sendMsgBuf(canmsg.ID, canmsg.DLC, (byte*)canmsg.Data);
      can_txCnt ++;
    }
    if(uxBits & CANLCDEVT_LCDDISP) {
      Display();
    } 
  }
}


// =================
//  Periodic Send Task
// =================
void Periodic_IncMsg(void) {
  uint64_t  cnt = 0;
  byte*  cnt_wk = (byte*)&cnt;

  if(can_periodicmsg.DLC > 0) {
    for(int i=0; i<(int)can_periodicmsg.DLC; i++)  cnt_wk[i] = can_periodicmsg.Data[can_periodicmsg.DLC-1-i];
    cnt ++;
    for(int i=0; i<(int)can_periodicmsg.DLC; i++)  can_periodicmsg.Data[i] = cnt_wk[can_periodicmsg.DLC-1-i];
  }
}

void Periodic_Task(void *pvParameters) {
  while(true) {
    if(can_periodicmode==2) {
      for(uint32_t i=0; i<can_periodictimes; i++) {
        xQueueSend(xCANSendMsgQue, &can_periodicmsg, NULL);
        xEventGroupSetBits(xCANLCDEvtGrp, CANLCDEVT_CANSEND);
        if(can_periodicincrement) {
          Periodic_IncMsg();
        }
        vTaskDelay(can_periodiccycle/portTICK_RATE_MS);
      }
      can_periodicmode = 0;
    }
    vTaskDelay(100/portTICK_RATE_MS);
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

  FILE_setup();

  xCANLCDEvtGrp = xEventGroupCreate();
  xCANSendMsgQue = xQueueCreate(CANMESSAGE_QMAX, sizeof(CANMESSAGE));
  xOutputMsgQue = xQueueCreate(OUTPUTMESSAGE_QMAX, sizeof(OUTPUTMESSAGE));

  WIFIInit();

  xTaskCreatePinnedToCore(Output_Task,"Output_Task", 2048, NULL, 0, &hTask[OUTPUT_TASKID], 1);
  xTaskCreatePinnedToCore(Input_Task,"Input_Task", 4096, NULL, 4, &hTask[INPUT_TASKID], 1);
  xTaskCreatePinnedToCore(CANLCD_Task,"CANLCD_Task", 8192, NULL, 0, &hTask[CANLCD_TASKID], 0);
  xTaskCreatePinnedToCore(Periodic_Task,"Periodic_Task", 2048, NULL, 3, &hTask[PERIODIC_TASKID], 1);
}

// =================
//  Idle Task
// =================
int cnt = 0;
unsigned long can_rxPPStm = 0;
int can_rxCntTmp = 0;

void loop() {
  // put your main code here, to run repeatedly:
  xEventGroupSetBits(xCANLCDEvtGrp, CANLCDEVT_LCDDISP);
  vTaskDelay(100/portTICK_RATE_MS);
  M5.update();

  if(can_rxPPStm == 0) {
    can_rxPPS = 0;
    can_rxPPStm = millis();
    can_rxCntTmp = can_rxCnt;
  } else {
    if( (millis() - can_rxPPStm) >= 1000) {
      can_rxPPS = (int)((double)(can_rxCnt - can_rxCntTmp) / ((double)(millis() - can_rxPPStm) / 1000.0));
      can_rxPPStm = millis();
      can_rxCntTmp = can_rxCnt;
    }
  }
//  Serial.printf("%d\r\n", cnt ++);
}
