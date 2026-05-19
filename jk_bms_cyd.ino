/*
 * JK-BMS CYD Monitor
 * ==================
 * ESP32 Cheap Yellow Display firmware for Jikong (JK) BMS monitoring.
 *
 * Reads JK-BMS data via BLE (NimBLE), serves a JSON API over WiFi,
 * and displays battery status on the CYD 2.4" TFT touchscreen.
 *
 * Based on: https://github.com/peff74/Arduino-jk-bms
 * Modified: Added CYD display + touch controls + multi-BMS pagination
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

// ===================== Configuration =====================
#define DEBUG_ENABLED true

static const char WiFi_SSID[]    = "YOUR_WIFI_SSID";
static const char WiFi_Password[] = "YOUR_WIFI_PASSWORD";
static const char BMS_MAC_1[]    = "c8:47:80:20:69:5f"; // Battery 1 MAC
static const char BMS_MAC_2[]    = "c8:47:80:1f:5f:11"; // Battery 2 MAC
static const int  NUM_BMS        = 2;

#define DISPLAY_REFRESH_INTERVAL 500
#define TOUCH_DEBOUNCE_MS 200
#define BLE_NOTIFY_TIMEOUT 45000UL // 45s before giving up on cell data

// ===================== Debug =====================
#if DEBUG_ENABLED
#define DBG_PRINT(...)     Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...)   Serial.println(__VA_ARGS__)
#define DBG_PRINTF(...)    Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTLN(...)
#define DBG_PRINTF(...)
#endif

// ===================== Display Constants =====================
#define SCREEN_W 320
#define SCREEN_H 240

#define PIN_TFT_MOSI 23
#define PIN_TFT_SCLK 18
#define PIN_TFT_CS   5
#define PIN_TFT_DC   22
#define PIN_TFT_RST  19
#define PIN_TFT_BL   21
#define PIN_TOUCH_CS 33
#define PIN_TOUCH_IRQ 36
#define PIN_TOUCH_CLK 25
#define PIN_TOUCH_MOSI 32
#define PIN_TOUCH_MISO 39

#define TOUCH_MIN_X 200
#define TOUCH_MAX_X 3700
#define TOUCH_MIN_Y 240
#define TOUCH_MAX_Y 3800

#define HEADER_H    32
#define SOC_BAR_H   28
#define CELL_ROWS   4
#define CELL_ROW_H  24
#define STATS_BLOCK_H 56
#define CTRL_BTN_H  36
#define PADDING     8
#define TOP_PAD     (HEADER_H + SOC_BAR_H)

// ===================== Colors =====================
#define CLR_BLACK        0x0000
#define CLR_WHITE        0xFFFF
#define CLR_RED          0xF800
#define CLR_GREEN        0x07E0
#define CLR_BLUE         0x001F
#define CLR_YELLOW       0xFFE0
#define CLR_CYAN         0x07FF
#define CLR_GRAY         0x7BEF
#define CLR_DARK_GRAY    0x5A58
#define CLR_LIGHT_GRAY   0xCE78
#define CLR_AMBER        0xFDA0
#define CLR_ORANGE       0xFA60
#define CLR_DARK_GREEN   0x0320
#define CLR_DARK_BLUE    0x0008
#define CLR_HEADER_BG    0x1A1A2E
#define CLR_CARD_BG      0x16213E

// ===================== Data Structures =====================
struct BMSData {
  float cellVoltage[16] = {0};
  int   cellCount       = 4;
  float avgCellVoltage  = 0;
  float deltaCellVoltage= 0;
  float battVoltage     = 0;
  float battPower       = 0;
  float chargeCurrent   = 0;
  float balanceCurrent  = 0;
  int   percentRemain   = 0;
  float capacityRemain  = 0;
  float nominalCapacity = 0;
  float cycleCount      = 0;
  float battT1          = 0;
  float battT2          = 0;
  float mosTemp         = 0;
  bool  charge          = false;
  bool  discharge       = false;
  bool  balance         = false;
  int   balancingAction = 0;
  uint32_t uptimeSec    = 0;
  uint8_t uptimeDays   = 0;
  uint8_t uptimeHrs    = 0;
  uint8_t uptimeMin    = 0;
  bool  newFrame        = false;
};

// ===================== Touch SPI Bus =====================
SPIClass touchscreenSPI(VSPI);

// ===================== Global Objects =====================
TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen touchscreen(PIN_TOUCH_CS, PIN_TOUCH_IRQ);
WebServer server(80);

BMSData bms[NUM_BMS];  // Multi-BMS data

bool wifiConnected = false;
unsigned long lastDrawTime = 0;
unsigned long lastScanTime = 0;
unsigned long uptimeStart  = 0;
unsigned long totalUptime  = 0;

// Touch
struct { int16_t x = -1, y = -1; bool touched = false; unsigned long lastTime = 0; } touch;

// Page navigation state
struct {
  int currentPage = 0;
  bool writing    = false;
  int  controlType = -1;
} navState;

// ===================== JKBMS Class =====================
class JKBMS {
public:
  JKBMS(const std::string& mac) : targetMAC(mac) {}
  NimBLERemoteCharacteristic* pChr = nullptr;
  const NimBLEAdvertisedDevice* advDevice = nullptr;
  bool doConnect = false, connected = false;
  uint32_t lastNotifyTime = 0;
  std::string targetMAC;
  byte   receivedBytes[320];
  int    frame = 0, ignoreNotifyCount = 0;
  bool   received_start = false, received_complete = false, new_data = false;

  float cellVoltage[16] = {0};
  float Average_Cell_Voltage=0, Delta_Cell_Voltage=0;
  float Battery_Voltage=0, Battery_Power=0, Charge_Current=0;
  float Battery_T1=0, Battery_T2=0, MOS_Temp=0, Balance_Curr=0;
  int Percent_Remain=0, Balancing_Action=0;
  float Capacity_Remain=0, Nominal_Capacity=0, Cycle_Count=0;
  uint32_t Uptime=0; uint8_t sec=0, mi=0, hr=0, days=0;
  bool Charge=false, Discharge=false, Balance=false;
  int cell_count=4;
  float total_battery_capacity=0;

  bool connectToServer();
  void parseData();
  void bms_settings();
  void parseDeviceInfo();
  void writeRegister(uint8_t address, uint32_t value, uint8_t length);
  void handleNotification(uint8_t* pData, size_t length);

private:
  uint8_t crc(const uint8_t data[], uint16_t len) {
    uint8_t c=0; for(uint16_t i=0;i<len;i++) c+=data[i]; return c;
  }
};

JKBMS* jkBmsDevices[NUM_BMS] = { nullptr, nullptr };

// ===================== BLE Callbacks =====================
NimBLEScan* pScan;

class ClientCallbacks : public NimBLEClientCallbacks {
  JKBMS* parent;
public:
  ClientCallbacks(JKBMS* p) : parent(p) {}
  void onConnect(NimBLEClient* pClient) override { DBG_PRINTLN("BLE connected"); }
  void onDisconnect(NimBLEClient* pClient, int reason) override {
    DBG_PRINTF("BLE disconnected, reason: %d\n", reason);
    parent->connected = false;
    parent->doConnect = false;
  }
};

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* adv) override {
    for (int i = 0; i < NUM_BMS; i++) {
      if (jkBmsDevices[i] == nullptr) continue;
      if (adv->getAddress().toString() == jkBmsDevices[i]->targetMAC &&
          !jkBmsDevices[i]->connected && !jkBmsDevices[i]->doConnect) {
        DBG_PRINTF("Found BMS %d: %s\n", i, jkBmsDevices[i]->targetMAC.c_str());
        jkBmsDevices[i]->advDevice = adv;
        jkBmsDevices[i]->doConnect = true;
        NimBLEDevice::getScan()->stop();
        break;
      }
    }
  }
};

static ScanCallbacks scanCallbacksInstance;

void notifyCB(NimBLERemoteCharacteristic* pChr, uint8_t* pData, size_t length, bool isNotify) {
  for (int i = 0; i < NUM_BMS; i++) {
    if (jkBmsDevices[i] != nullptr && jkBmsDevices[i]->pChr == pChr) {
      jkBmsDevices[i]->handleNotification(pData, length);
      break;
    }
  }
}

// ===================== JKBMS Methods =====================
bool JKBMS::connectToServer() {
  // Stop any running scan before connecting — scanning conflicts with active connections
  NimBLEDevice::getScan()->stop();
  DBG_PRINTF("Connecting to %s...\n", targetMAC.c_str());
  NimBLEClient* pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
  if (!pClient) {
    pClient = NimBLEDevice::createClient();
    DBG_PRINTLN("New BLE client");
    pClient->setClientCallbacks(new ClientCallbacks(this), true);
    pClient->setConnectionParams(12, 12, 0, 150);
    pClient->setConnectTimeout(30000);
  }

  if (!pClient->connect(advDevice)) {
    DBG_PRINTF("Connection failed: %s\n", targetMAC.c_str());
    return false;
  }
  DBG_PRINTLN("BLE connected");

  // Negotiate MTU before subscribing — cell data frames are ~200 bytes, default MTU (23) is too small
  // The BMS won't send large cell data notifications unless MTU is high enough
  uint16_t mtu = pClient->getMTU();
  DBG_PRINTF("Current MTU: %d\n", mtu);
  bool mtu_ok = pClient->exchangeMTU();
  DBG_PRINTF("MTU negotiate: %s\n", mtu_ok ? "ok" : "failed");
  mtu = pClient->getMTU();
  DBG_PRINTF("New MTU: %d\n", mtu);

  DBG_PRINTF("Connected! RSSI: %d\n", pClient->getRssi());
  NimBLERemoteService* pSvc = pClient->getService("ffe0");
  if (pSvc) {
    pChr = pSvc->getCharacteristic("ffe1");
    if (pChr && pChr->canNotify()) {
      if (pChr->subscribe(true, notifyCB)) {
        DBG_PRINTF("Subscribed to %s\n", pChr->getUUID().toString().c_str());
        delay(200);
        writeRegister(0x97, 0, 0); // Device info
        DBG_PRINTLN("Wrote device info register (0x97)");
        connected = true;
        return true;
      }
    }
  }
  DBG_PRINTLN("Service/Char not found");
  return false;
}

void JKBMS::handleNotification(uint8_t* pData, size_t length) {
  lastNotifyTime = millis();
  if (ignoreNotifyCount > 0) { ignoreNotifyCount--; return; }

  if (pData[0]==0x55 && pData[1]==0xAA && pData[2]==0xEB && pData[3]==0x90) {
    // New frame header — process previous frame if we have one, then clear buffer
    if (received_start && !received_complete && frame > 0) {
      received_complete = true;
      received_start = false;
      new_data = true;
      switch(receivedBytes[4]) {
        case 0x01: bms_settings(); break;
        case 0x02: parseData(); break;
        case 0x03: parseDeviceInfo(); break;
      }
    }
    // Clear entire buffer to remove stale bytes from old frame
    for (int i = 0; i < 300; i++) receivedBytes[i] = 0;
    frame = 0;
    received_start = true;
    received_complete = false;
    for(size_t i=0;i<length;i++) receivedBytes[frame++]=pData[i];
  } else if (received_start && !received_complete) {
    // Continuation of fragmented frame — accumulate bytes
    for(size_t i=0;i<length;i++) {
      receivedBytes[frame++]=pData[i];
    }
  }
  // else: discard
}

void JKBMS::writeRegister(uint8_t address, uint32_t value, uint8_t length) {
  uint8_t frame[20]={0xAA,0x55,0x90,0xEB,address,length};
  frame[6]=value>>0; frame[7]=value>>8; frame[8]=value>>16; frame[9]=value>>24;
  frame[19]=crc(frame,19);
  if(pChr) pChr->writeValue((uint8_t*)frame,sizeof(frame));
}

void JKBMS::bms_settings() {
  // Cell count is read from the 0x01 settings frame, bytes 114-117
  // Some firmware versions have different layouts — add sanity check
  uint32_t raw_count = (receivedBytes[117]<<24|receivedBytes[116]<<16|receivedBytes[115]<<8|receivedBytes[114]);
  // Sanity: cell count should be between 1 and 16 for any practical battery
  // If it's 0 or suspiciously large (>16), try byte 5 as fallback
  if (raw_count < 1 || raw_count > 16) {
    // Byte 5 in cell data frames is sometimes a cell count, sometimes a frame counter
    // Use it only as fallback and double-check
    uint8_t fb = receivedBytes[5];
    if (fb >= 1 && fb <= 16) {
      cell_count = fb;
    } else {
      // No reliable cell count found — default to 3 for 3S/12V
      cell_count = 3;
    }
  } else {
    cell_count = (int)raw_count;
  }
  total_battery_capacity = ((receivedBytes[133]<<24|receivedBytes[132]<<16|receivedBytes[131]<<8|receivedBytes[130])*0.001);
  DBG_PRINTF("Settings: %d cells, %.2fAh\n", cell_count, total_battery_capacity);
}

void JKBMS::parseDeviceInfo() {
  new_data=false;
  if(frame<134) return;
  Uptime=(receivedBytes[41]<<24)|(receivedBytes[40]<<16)|(receivedBytes[39]<<8)|receivedBytes[38];
  DBG_PRINTF("Device info, uptime=%lu s (cell data next)\n", Uptime);
}

void JKBMS::parseData() {
  DBG_PRINTLN("--- BMS cell data received ---");
  new_data=false; ignoreNotifyCount=10;
  for(int j=0,i=7;i<38&&j<16;j++,i+=2)
    cellVoltage[j]=(uint16_t)(receivedBytes[i]|(receivedBytes[i-1]<<8))*0.001;
  Average_Cell_Voltage  =((uint16_t)(receivedBytes[75]|(receivedBytes[74]<<8)))*0.001;
  Delta_Cell_Voltage    =((uint16_t)(receivedBytes[77]|(receivedBytes[76]<<8)))*0.001;
  MOS_Temp=(((uint16_t)(receivedBytes[145]|(receivedBytes[144]<<8)))*0.1);
  Battery_Voltage =((uint32_t)receivedBytes[153]<<24|(uint32_t)receivedBytes[152]<<16|(uint32_t)receivedBytes[151]<<8|receivedBytes[150])*0.001;
  Charge_Current  =((uint32_t)receivedBytes[161]<<24|(uint32_t)receivedBytes[160]<<16|(uint32_t)receivedBytes[159]<<8|receivedBytes[158])*0.001;
  Battery_Power   =Battery_Voltage*Charge_Current;
  Battery_T1      =(((uint16_t)(receivedBytes[163]|(receivedBytes[162]<<8)))*0.1);
  Battery_T2      =(((uint16_t)(receivedBytes[165]|(receivedBytes[164]<<8)))*0.1);
  if((receivedBytes[171]&0xF0)==0x0) Balance_Curr=((uint16_t)(receivedBytes[171]|(receivedBytes[170]<<8))*0.001);
  else if((receivedBytes[171]&0xF0)==0xF0) Balance_Curr=((int16_t)(((receivedBytes[171]&0x0F)<<8)|receivedBytes[170])*(-0.001));
  Balancing_Action=receivedBytes[172];
  Percent_Remain  =receivedBytes[173];
  Capacity_Remain =((uint32_t)receivedBytes[177]<<24|(uint32_t)receivedBytes[176]<<16|(uint32_t)receivedBytes[175]<<8|receivedBytes[174])*0.001;
  Nominal_Capacity=((uint32_t)receivedBytes[181]<<24|(uint32_t)receivedBytes[180]<<16|(uint32_t)receivedBytes[179]<<8|receivedBytes[178])*0.001;
  Cycle_Count     =((uint32_t)receivedBytes[185]<<24|(uint32_t)receivedBytes[184]<<16|(uint32_t)receivedBytes[183]<<8|receivedBytes[182]);
  Uptime          =receivedBytes[196]<<16|receivedBytes[195]<<8|receivedBytes[194];
  sec=Uptime%60; Uptime/=60; mi=Uptime%60; Uptime/=60; hr=Uptime%24; days=Uptime/24;
  Charge  =receivedBytes[198]>0;
  Discharge=receivedBytes[199]>0;
  Balance =receivedBytes[201]>0;
  DBG_PRINTLN("--- BMS Data ---");
  DBG_PRINTF("V=%.2fV I=%.2fA SOC=%d%%\n", Battery_Voltage, Charge_Current, Percent_Remain);
}

// ===================== Sync BLE → Display =====================
static void syncBMSData(int idx) {
  if (jkBmsDevices[idx] == nullptr) return;
  BMSData& d = bms[idx];
  d.cellCount      = jkBmsDevices[idx]->cell_count;
  for(int i=0;i<d.cellCount && i<16;i++) {
    d.cellVoltage[i] = jkBmsDevices[idx]->cellVoltage[i];
  }
  d.avgCellVoltage   = jkBmsDevices[idx]->Average_Cell_Voltage;
  d.deltaCellVoltage = jkBmsDevices[idx]->Delta_Cell_Voltage;
  d.battVoltage      = jkBmsDevices[idx]->Battery_Voltage;
  d.battPower        = jkBmsDevices[idx]->Battery_Power;
  d.chargeCurrent    = jkBmsDevices[idx]->Charge_Current;
  d.balanceCurrent   = jkBmsDevices[idx]->Balance_Curr;
  d.battT1           = jkBmsDevices[idx]->Battery_T1;
  d.battT2           = jkBmsDevices[idx]->Battery_T2;
  d.mosTemp          = jkBmsDevices[idx]->MOS_Temp;
  d.percentRemain    = jkBmsDevices[idx]->Percent_Remain;
  d.capacityRemain   = jkBmsDevices[idx]->Capacity_Remain;
  d.nominalCapacity  = jkBmsDevices[idx]->Nominal_Capacity;
  d.cycleCount       = jkBmsDevices[idx]->Cycle_Count;
  d.uptimeSec        = jkBmsDevices[idx]->Uptime;
  d.uptimeDays       = jkBmsDevices[idx]->days;
  d.uptimeHrs        = jkBmsDevices[idx]->hr;
  d.uptimeMin        = jkBmsDevices[idx]->mi;
  d.charge           = jkBmsDevices[idx]->Charge;
  d.discharge        = jkBmsDevices[idx]->Discharge;
  d.balance          = jkBmsDevices[idx]->Balance;
  d.balancingAction  = jkBmsDevices[idx]->Balancing_Action;
  d.newFrame         = jkBmsDevices[idx]->new_data;
}

// ===================== Display Drawing =====================
static void drawHeader(const BMSData& d) {
  tft.fillRect(0,0,SCREEN_W,HEADER_H,CLR_HEADER_BG);

  // Battery label centered (font 2, textSize 1 = 8x12 per char)
  char pageLabel[16];
  snprintf(pageLabel, sizeof(pageLabel), "BATTERY %d", navState.currentPage + 1);
  tft.setTextColor(CLR_WHITE, CLR_HEADER_BG);
  int labelX = (SCREEN_W - 8 * strlen(pageLabel)) / 2;
  tft.drawString(pageLabel, labelX, 8, 2);

  // Left arrow button (previous page) - on left edge
  if (navState.currentPage > 0) {
    tft.fillRect(2, 2, 36, 28, CLR_DARK_BLUE);
    tft.drawRoundRect(2, 2, 36, 28, 4, CLR_BLUE);
    tft.setTextColor(CLR_WHITE);
    tft.drawCentreString("<", 2 + 18, 6, 4);
  }

  // Right arrow button (next page) - on right edge
  if (navState.currentPage < NUM_BMS - 1) {
    int ax = SCREEN_W - 38;
    tft.fillRect(ax, 2, 36, 28, CLR_DARK_GREEN);
    tft.drawRoundRect(ax, 2, 36, 28, 4, CLR_GREEN);
    tft.setTextColor(CLR_WHITE);
    tft.drawCentreString(">", ax + 18, 6, 4);
  }

  // Connection status indicator (dot at bottom-center)
  uint16_t dotColor;
  if (wifiConnected) {
    bool anyConnected = false;
    for (int i = 0; i < NUM_BMS; i++) {
      if (jkBmsDevices[i] != nullptr && jkBmsDevices[i]->connected) {
        anyConnected = true; break;
      }
    }
    dotColor = anyConnected ? CLR_GREEN : CLR_AMBER;
  } else {
    dotColor = CLR_RED;
  }
  tft.fillCircle(SCREEN_W/2, 24, 5, dotColor);
  tft.drawCircle(SCREEN_W/2, 24, 5, CLR_HEADER_BG);
}

static void drawSOCBar(const BMSData& d) {
  int y=TOP_PAD, barW=SCREEN_W-PADDING*2, barH=SOC_BAR_H-8, x=PADDING+4;
  char lbl[16];
  snprintf(lbl,sizeof(lbl),"SOC %d%%",d.percentRemain);
  tft.setTextColor(CLR_WHITE);
  tft.drawString(lbl,PADDING,y,1);
  tft.fillRect(x,y+2,barW,barH,CLR_BLACK);
  int fw=(barW*d.percentRemain)/100; fw=constrain(fw,0,barW);
  uint16_t sc=CLR_GREEN;
  if(d.percentRemain<=10) sc=CLR_RED;
  else if(d.percentRemain<=25) sc=CLR_YELLOW;
  if(fw > 0) {
    tft.fillRect(x,y+2,fw,barH,sc);
  }
  tft.drawRect(x,y+2,barW,barH,CLR_LIGHT_GRAY);
}

static uint16_t cellColor(float v) {
  if(v<3.0f) return CLR_RED;
  if(v<3.3f||v>3.9f) return CLR_YELLOW;
  return CLR_GREEN;
}

static void drawCellVoltages(const BMSData& d) {
  int x=PADDING, y=TOP_PAD+SOC_BAR_H;
  tft.setTextColor(CLR_CYAN);
  tft.drawString("CELL VOLTAGES",x,y,1);
  y+=10;
  // Divider line
  tft.drawLine(x,y,SCREEN_W-PADDING,y,CLR_DARK_GRAY);
  y+=4;
  for(int i=0;i<d.cellCount && i<CELL_ROWS;i++) {
    char line[32];
    snprintf(line,sizeof(line),"  C%d: %.3fV",i+1,d.cellVoltage[i]);
    tft.setTextColor(cellColor(d.cellVoltage[i]));
    tft.drawString(line,x+4,y,2);
    y+=CELL_ROW_H;
  }
}

static void drawBatteryStats(const BMSData& d) {
  // Right side panel - compact
  int x=220, y=TOP_PAD+SOC_BAR_H+10;
  tft.fillRect(x,y-6,90,40,CLR_CARD_BG);
  tft.drawRect(x,y-6,90,40,CLR_DARK_GRAY);
  tft.setTextColor(CLR_CYAN,CLR_CARD_BG);
  tft.drawString("STATS",x+6,y,1);
  char s[24];
  snprintf(s,sizeof(s),"%.1fV",d.battVoltage);
  tft.setTextColor(CLR_WHITE,CLR_CARD_BG);
  tft.drawString("V:",x+6,y+12,2);
  tft.drawString(s,x+28,y+12,2);
  snprintf(s,sizeof(s),"%.1fA",d.chargeCurrent);
  tft.drawString("I:",x+6,y+28,2);
  tft.drawString(s,x+28,y+28,2);
}

static void drawTemps(const BMSData& d) {
  // Right side panel - temps below stats (textSize 1 = 12px per line)
  int x=220, y=TOP_PAD+SOC_BAR_H+40+14;
  tft.setTextColor(CLR_CYAN);
  tft.drawString("TEMP",x,y,1);
  y+=10;
  char s[24];
  uint16_t tc=CLR_WHITE;
  if(d.battT1>55.0f) tc=CLR_RED;
  else if(d.battT1<0.0f) tc=CLR_YELLOW;
  snprintf(s,sizeof(s),"T1: %.1f C",d.battT1);
  tft.setTextColor(tc);
  tft.drawString(s,x,y,2);
  y+=14;
  snprintf(s,sizeof(s),"MOS: %.1f C",d.mosTemp);
  tft.drawString(s,x,y,2);
}

static uint16_t btnColor(bool active, int type) {
  if(type==0) return active?0x07C0:0x2400;
  if(type==1) return active?0x001F:0x0008;
  return active?0xFA60:0x6328;
}

static void drawControls(const BMSData& d) {
  int bw=(SCREEN_W-PADDING*3-20)/3, y=SCREEN_H-CTRL_BTN_H-2;
  int bx;

  // Charge
  bx=PADDING;
  tft.fillRect(bx,y,bw,CTRL_BTN_H,btnColor(d.charge,0));
  tft.drawRect(bx,y,bw,CTRL_BTN_H,CLR_GRAY);
  tft.setTextColor(CLR_WHITE);
  tft.drawCentreString("CHARGE",bx+bw/2,y+CTRL_BTN_H/2-6,2);

  // Discharge
  bx=PADDING+bw+10;
  tft.fillRect(bx,y,bw,CTRL_BTN_H,btnColor(d.discharge,1));
  tft.drawRect(bx,y,bw,CTRL_BTN_H,CLR_GRAY);
  tft.drawCentreString("DISCHG",bx+bw/2,y+CTRL_BTN_H/2-6,2);

  // Balance
  bx=PADDING*2+bw*2+10;
  tft.fillRect(bx,y,bw,CTRL_BTN_H,btnColor(d.balance,2));
  tft.drawRect(bx,y,bw,CTRL_BTN_H,CLR_GRAY);
  tft.drawCentreString("BALANCE",bx+bw/2,y+CTRL_BTN_H/2-6,2);
}

static void drawScreen(const BMSData& d) {
  tft.startWrite();
  tft.fillScreen(CLR_BLACK);
  drawHeader(d);
  drawSOCBar(d);
  drawCellVoltages(d);
  drawBatteryStats(d);
  drawTemps(d);
  drawControls(d);
  tft.endWrite();
}

// ===================== Touch =====================
static void readTouch() {
  if(touchscreen.tirqTouched() && touchscreen.touched()) {
    TS_Point p = touchscreen.getPoint();
    touch.x = map(p.x, TOUCH_MIN_X, TOUCH_MAX_X, 0, SCREEN_W);
    touch.y = map(p.y, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, SCREEN_H);
    touch.touched = p.z > 0;
  } else {
    touch.touched = false;
  }
}

static void handleTouch() {
  if(!touch.touched) return;
  if(millis()-touch.lastTime<TOUCH_DEBOUNCE_MS) return;
  touch.lastTime = millis();

  int tx = touch.x, ty = touch.y;

  // Page navigation (header arrows)
  if (ty < HEADER_H) {
    // Left arrow: x from 2 to 38
    if (tx >= 2 && tx <= 38 && navState.currentPage > 0) {
      DBG_PRINTLN("Page: prev");
      navState.currentPage--;
      return;
    }
    // Right arrow: x from (SCREEN_W-38) to SCREEN_W-2
    int rightArrowX = SCREEN_W - 38;
    if (tx >= rightArrowX && tx <= SCREEN_W - 2 && navState.currentPage < NUM_BMS - 1) {
      DBG_PRINTLN("Page: next");
      navState.currentPage++;
      return;
    }
  }

  // Control buttons
  int btnY = SCREEN_H - CTRL_BTN_H - 2;
  int bw = (SCREEN_W-PADDING*3-20)/3;

  // Charge (type 0)
  if (tx>=PADDING && tx<=PADDING+bw && ty>=btnY && ty<=btnY+CTRL_BTN_H) {
    if (!navState.writing) {
      navState.controlType = 0;
      navState.writing = true;
      DBG_PRINTLN("Toggle: CHARGE");
    }
    return;
  }

  // Discharge (type 1)
  if (tx>=PADDING+bw+10 && tx<=PADDING+bw*2+10 && ty>=btnY && ty<=btnY+CTRL_BTN_H) {
    if (!navState.writing) {
      navState.controlType = 1;
      navState.writing = true;
      DBG_PRINTLN("Toggle: DISCHARGE");
    }
    return;
  }

  // Balance (type 2)
  if (tx>=PADDING*2+bw*2+10 && tx<=PADDING*2+bw*2+10+bw && ty>=btnY && ty<=btnY+CTRL_BTN_H) {
    if (!navState.writing) {
      navState.controlType = 2;
      navState.writing = true;
      DBG_PRINTLN("Toggle: BALANCE");
    }
    return;
  }
}

// ===================== BLE Control =====================
static void processControl() {
  if (!navState.writing) return;
  if (jkBmsDevices[navState.currentPage] == nullptr) {
    navState.writing = false;
    return;
  }
  JKBMS* bms = jkBmsDevices[navState.currentPage];

  uint8_t addr;
  if (navState.controlType == 0) addr = 0x1D; // Charge
  else if (navState.controlType == 1) addr = 0x1E; // Discharge
  else addr = 0x1F; // Balance

  bool cur = false;
  if (navState.controlType == 0) cur = bms->Charge;
  else if (navState.controlType == 1) cur = bms->Discharge;
  else cur = bms->Balance;

  uint32_t val = cur ? 0 : 1;
  DBG_PRINTF("BLE write: 0x%02X = %lu\n", addr, val);
  bms->writeRegister(addr, val, 0x04);
  delay(350);
  DBG_PRINTLN("BLE write done");
  navState.writing = false;
}

// ===================== Web Server =====================
static void handleRoot() {
  if(LittleFS.exists("/index.html")) {
    server.sendHeader("Location","/main");
    server.send(302);
  } else {
    server.send(200,"text/html","<html><body style='text-align:center;margin-top:60px;font-family:sans-serif;background:#1a1a2e;color:#eee;'>"
      "<h2>JK-BMS Monitor</h2><p>Upload index.html via /fs</p>"
      "<a href='/fs' style='color:#0af;font-size:1.2em;'>File Manager</a></body></html>");
  }
}

static void handleJSON() {
  DynamicJsonDocument doc(2048);
  doc["num_bms"] = NUM_BMS;
  for (int i = 0; i < NUM_BMS; i++) {
    JsonObject dev = doc.createNestedObject(String("battery_") + (i+1));
    dev["cell_count"] = bms[i].cellCount;
    dev["battery_voltage"] = bms[i].battVoltage;
    dev["battery_power"] = bms[i].battPower;
    dev["charge_current"] = bms[i].chargeCurrent;
    dev["percent_remain"] = bms[i].percentRemain;
    dev["capacity_remain"] = bms[i].capacityRemain;
    dev["nominal_capacity"] = bms[i].nominalCapacity;
    dev["cycle_count"] = bms[i].cycleCount;
    dev["battery_t1"] = bms[i].battT1;
    dev["battery_t2"] = bms[i].battT2;
    dev["mos_temp"] = bms[i].mosTemp;
    dev["charge"] = bms[i].charge;
    dev["discharge"] = bms[i].discharge;
    dev["balance"] = bms[i].balance;
    dev["balancing_action"] = bms[i].balancingAction;
    dev["avg_cell_voltage"] = bms[i].avgCellVoltage;
    dev["delta_cell_voltage"] = bms[i].deltaCellVoltage;
    JsonArray cells = dev.createNestedArray("cell_voltages");
    for (int j = 0; j < bms[i].cellCount && j < 16; j++) cells.add(bms[i].cellVoltage[j]);
    dev["uptime_seconds"] = bms[i].uptimeSec;
    dev["uptime_days"] = bms[i].uptimeDays;
    dev["uptime_hours"] = bms[i].uptimeHrs;
    dev["uptime_minutes"] = bms[i].uptimeMin;
  }
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

static void handleControl() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  DynamicJsonDocument doc(200);
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "text/plain", "Bad JSON"); return; }
  String action = doc["action"];
  String state = doc["state"];
  int targetPage = doc["page"] | 0; // Default to page 0
  if (state != "on" && state != "off") { server.send(400); return; }

  uint8_t addr;
  if (action.startsWith("charging") || action == "charge") addr = 0x1D;
  else if (action.startsWith("discharging") || action == "discharge") addr = 0x1E;
  else addr = 0x1F;

  uint32_t val = (state == "on") ? 1 : 0;
  DBG_PRINTF("Web control page %d: 0x%02X=%lu\n", targetPage, addr, val);

  if (targetPage < NUM_BMS && jkBmsDevices[targetPage] != nullptr) {
    jkBmsDevices[targetPage]->writeRegister(addr, val, 0x04);
    delay(350);
  }
  server.send(200, "text/plain", "OK");
}

static void handleSketchInfo() {
  DynamicJsonDocument doc(200);
  doc["core_version"] = String(ESP_ARDUINO_VERSION_MAJOR)+"."+ESP_ARDUINO_VERSION_MINOR+"."+ESP_ARDUINO_VERSION_PATCH;
  doc["compile_date"] = __DATE__;
  doc["compile_time"] = __TIME__;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

static void handleFreeHeap() {
  DynamicJsonDocument doc(100);
  uint32_t f = ESP.getFreeHeap();
  char s[16];
  snprintf(s, sizeof(s), "%.1fKB", (float)f/1024);
  doc["free_heap"] = s;
  doc["free_heap_bytes"] = f;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

static void handleUptime() {
  DynamicJsonDocument doc(100);
  doc["uptime_seconds"] = totalUptime;
  char s[32];
  snprintf(s, sizeof(s), "%lud %02lu:%02lu:%02lu",
    totalUptime/86400, (totalUptime%86400)/3600, (totalUptime%3600)/60, totalUptime%60);
  doc["uptime_formatted"] = s;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

static void handleFileList() {
  String files;
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  while(f) {
    char sz[16];
    if(f.size()<1024) snprintf(sz,sizeof(sz),"%zuB",f.size());
    else if(f.size()<1048576) snprintf(sz,sizeof(sz),"%.1fKB",(float)f.size()/1024);
    else snprintf(sz,sizeof(sz),"%.1fMB",(float)f.size()/1048576);
    files += "<div style='background:#16213e;padding:10px;margin:5px 0;border-radius:4px;display:flex;justify-content:space-between;'>"
    "<span>"+String(f.name())+"</span><span style='color:#888;margin:0 10px'>"+String(sz)+"</span>"
    "<a href='/view?file="+String(f.name())+"' style='color:#0af;text-decoration:none'>view</a> "
    "<a href='/delete?file="+String(f.name())+"' style='color:#f44;text-decoration:none'>del</a></div>";
    f = root.openNextFile();
  }
  server.send(200, "text/html",
    "<html><head><style>body{font-family:sans-serif;background:#1a1a2e;color:#eee;padding:20px;}a{color:#0af;text-decoration:none;}</style></head><body>"
    "<h2>LittleFS Manager</h2>"
    "<form method='post' action='/upload' enctype='multipart/form-data'>"
    "<input type='file' name='upload[]' multiple><input type='submit' value='Upload'></form><br>"
    "<a href='/format' style='color:#f44'>Format LittleFS</a><br><br>"
    "<h3>Files</h3>"+files+"<br><a href='/'>Home</a></body></html>");
}

static void handleFileUpload() {
  if (server.uri() != "/upload") return;
  HTTPUpload& upload = server.upload();
  static File file;
  if (upload.status == UPLOAD_FILE_START) {
    file = LittleFS.open("/"+upload.filename, "w");
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (file) file.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (file) file.close();
    server.sendHeader("Location", "/fs");
    server.send(303);
  }
}

static void handleFileDelete() {
  String fn = server.arg("file");
  LittleFS.remove(fn);
  server.sendHeader("Location", "/fs");
  server.send(303);
}

static void handleFileView() {
  String fn = server.arg("file");
  if (!fn.startsWith("/")) fn = "/" + fn;
  File f = LittleFS.open(fn, "r");
  if (!f) { server.send(404); return; }
  String c = f.readString();
  f.close();
  server.send(200, "text/html", "<html><body style='background:#1a1a2e;color:#eee;font-family:monospace;padding:20px;'>"
    "<h2>"+fn+"</h2><pre>"+c+"</pre><br><a href='/fs' style='color:#0af'>Back</a></body></html>");
}

static void handleFormat() { LittleFS.format(); server.sendHeader("Location", "/fs"); server.send(303); }

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  DBG_PRINTLN("\n=== JK-BMS CYD Monitor ===");

  // TFT
  tft.init();
  tft.setSwapBytes(true);
  tft.setRotation(1);
  tft.fillScreen(CLR_BLACK);
  tft.setTextColor(CLR_CYAN);
  tft.setTextSize(2);
  tft.drawCentreString("JK-BMS", SCREEN_W/2, 100, 4);
  tft.setTextColor(CLR_WHITE);
  tft.setTextSize(1);
  tft.drawCentreString("CYD Monitor", SCREEN_W/2, 160, 2);
  tft.drawString("Initializing...", PADDING, 230, 2);

  // Touch SPI bus + init
  touchscreenSPI.begin(PIN_TOUCH_CLK, PIN_TOUCH_MISO, PIN_TOUCH_MOSI, PIN_TOUCH_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(1);

  // LittleFS
  if (!LittleFS.begin(true)) {
    DBG_PRINTLN("LittleFS mount failed");
  } else {
    DBG_PRINTLN("LittleFS mounted");
  }

  // Initialize BMS objects
  jkBmsDevices[0] = new JKBMS(std::string(BMS_MAC_1));
  jkBmsDevices[1] = new JKBMS(std::string(BMS_MAC_2));

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WiFi_SSID, WiFi_Password);
  DBG_PRINTLN("Connecting WiFi...");
  int att = 0;
  while (WiFi.status() != WL_CONNECTED && att < 30) {
    delay(500); Serial.print("."); att++;
    if (att % 6 == 0) {
      char s[40];
      snprintf(s, sizeof(s), "WiFi %d/30", att);
      tft.drawString(s, PADDING, 230, 2);
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    DBG_PRINTLN("\nWiFi connected!");
    DBG_PRINTF("IP: %s\n", WiFi.localIP().toString().c_str());
    tft.drawString("WiFi OK", PADDING, 230, 2);
    tft.setTextColor(CLR_GREEN);
    tft.drawString(WiFi.localIP().toString().c_str(), PADDING, 250, 2);
  } else {
    DBG_PRINTLN("\nWiFi failed!");
    tft.setTextColor(CLR_RED);
    tft.drawString("WiFi FAILED", PADDING, 230, 2);
  }

  // Web server
  server.serveStatic("/main", LittleFS, "/index.html");
  server.serveStatic("/style.css", LittleFS, "/style.css", "text/css");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleJSON);
  server.on("/control", HTTP_POST, handleControl);
  server.on("/sketchinfo", HTTP_GET, handleSketchInfo);
  server.on("/freeheap", HTTP_GET, handleFreeHeap);
  server.on("/uptime", HTTP_GET, handleUptime);
  server.on("/fs", HTTP_GET, handleFileList);
  server.on("/upload", HTTP_POST, handleFileUpload);
  server.on("/delete", HTTP_GET, handleFileDelete);
  server.on("/view", HTTP_GET, handleFileView);
  server.on("/format", HTTP_GET, handleFormat);
  server.begin();
  DBG_PRINTLN("Web server started");

  // NimBLE
  DBG_PRINTLN("Initializing NimBLE...");
  NimBLEDevice::init("JK-BMS-CYD");
  NimBLEDevice::setPower(3);
  pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacksInstance);
  pScan->setInterval(1024);   // Less aggressive — don't hog BLE airtime from active connections
  pScan->setWindow(50);
  pScan->setActiveScan(false); // Passive scan — don't interfere with active connections

  uptimeStart = millis();
  drawScreen(bms[0]);
}

// ===================== Loop =====================
void loop() {
  unsigned long now = millis();

  server.handleClient();

  // BLE connection for each device
  for (int i = 0; i < NUM_BMS; i++) {
    if (jkBmsDevices[i] == nullptr) continue;
    if (jkBmsDevices[i]->doConnect && !jkBmsDevices[i]->connected) {
      // Stop scanning while connecting — don't steal BLE airtime
      pScan->stop();
      if (jkBmsDevices[i]->connectToServer()) {
        DBG_PRINTF("BMS %d connected!\n", i);
      }
      jkBmsDevices[i]->doConnect = false;
    }

    // Connection timeout — no notification data in 45s, disconnect and resume scanning
    if (jkBmsDevices[i]->connected && (now - jkBmsDevices[i]->lastNotifyTime > BLE_NOTIFY_TIMEOUT)) {
      DBG_PRINTF("BMS %d timeout (no cell data in %lu ms)\n", i, (unsigned long)(now - jkBmsDevices[i]->lastNotifyTime));
      jkBmsDevices[i]->connected = false;
      NimBLEClient* pc = NimBLEDevice::getClientByPeerAddress(jkBmsDevices[i]->advDevice->getAddress());
      if (pc) pc->disconnect();
    }
  }

  // Sync data for all devices
  for (int i = 0; i < NUM_BMS; i++) {
    if (jkBmsDevices[i] != nullptr) {
      bool anyChanged = false;
      for (int j = 0; j < NUM_BMS; j++) {
        if (jkBmsDevices[j] != nullptr && jkBmsDevices[j]->new_data) {
          anyChanged = true; break;
        }
      }
      syncBMSData(i);
    }
  }

  // Process control
  processControl();

  // Touch
  readTouch();
  handleTouch();

  // Draw current page — throttle redraws to every 1s so touch stays responsive
  BMSData& d = bms[navState.currentPage];
  static unsigned long lastDrawTime = 0;
  if ((d.newFrame || navState.writing) && (now - lastDrawTime > 1000)) {
    drawScreen(d);
    d.newFrame = false;
    lastDrawTime = now;
  }

  // Rescan if needed
  bool anyConnected = false;
  for (int i = 0; i < NUM_BMS; i++) {
    if (jkBmsDevices[i] != nullptr && jkBmsDevices[i]->connected) {
      anyConnected = true; break;
    }
  }
  // Only scan when NO BMS is connected — scanning steals BLE airtime from active links
  if (!anyConnected && (now - lastScanTime >= 10000)) {
    DBG_PRINTLN("Scanning for BMS...");
    pScan->start(5000, false, true);
    lastScanTime = now;
  }

  // Uptime
  totalUptime = (now - uptimeStart) / 1000;

  delay(10);
}
