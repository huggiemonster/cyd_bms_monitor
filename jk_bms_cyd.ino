/*
 * JK-BMS CYD Monitor
 * ==================
 * ESP32 Cheap Yellow Display firmware for Jikong (JK) BMS monitoring.
 *
 * Reads JK-BMS data via BLE (NimBLE), serves a JSON API over WiFi,
 * and displays battery status on the CYD 2.4" TFT touchscreen.
 *
 * Based on: https://github.com/peff74/Arduino-jk-bms
 * Modified: Added CYD display + touch controls
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
static const char BMS_MAC[]      = "20:22:08:25:26:8b"; // Change to your BMS MAC

#define DISPLAY_REFRESH_INTERVAL 500
#define TOUCH_DEBOUNCE_MS 200

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
#define SCREEN_W 240
#define SCREEN_H 320

#define PIN_TFT_MOSI 23
#define PIN_TFT_SCLK 18
#define PIN_TFT_CS   5
#define PIN_TFT_DC   22
#define PIN_TFT_RST  19
#define PIN_TFT_BL   21
#define PIN_TOUCH_CS 33
#define PIN_TOUCH_IRQ 36

#define TOUCH_MIN_X 200
#define TOUCH_MAX_X 3700
#define TOUCH_MIN_Y 240
#define TOUCH_MAX_Y 3800

#define HEADER_H    32
#define SOC_BAR_H   28
#define CELL_ROWS   4
#define CELL_ROW_H  26
#define STATS_BLOCK_H 68
#define CTRL_BTN_H  36
#define PAD 8
#define TOP_PAD (HEADER_H + SOC_BAR_H)

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
  float wireResist[16]  = {0};
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
  float cycleCapacity   = 0;
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

struct BMSData bms;

// ===================== Global Objects =====================
TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen touchscreen(PIN_TOUCH_CS);
WebServer server(80);

bool wifiConnected  = false;
bool bmsConnected   = false;
unsigned long lastDrawTime = 0;
unsigned long lastScanTime = 0;
unsigned long uptimeStart  = 0;
unsigned long totalUptime  = 0;

// Touch
struct { int16_t x = -1, y = -1; bool touched = false; unsigned long lastTime = 0; } touch;

// Control
struct { int type = -1; bool pending = false; bool writing = false; } ctrlPending;

// ===================== Helpers =====================
static int16_t mapTx(int16_t r) { return map(r, TOUCH_MIN_X, TOUCH_MAX_X, 0, SCREEN_W); }
static int16_t mapTy(int16_t r) { return map(r, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, SCREEN_H); }

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

  float cellVoltage[16] = {0}, wireResist[16] = {0};
  float Average_Cell_Voltage=0, Delta_Cell_Voltage=0;
  float Battery_Voltage=0, Battery_Power=0, Charge_Current=0;
  float Battery_T1=0, Battery_T2=0, MOS_Temp=0, Balance_Curr=0;
  int Percent_Remain=0, Balancing_Action=0;
  float Capacity_Remain=0, Nominal_Capacity=0, Cycle_Count=0, Cycle_Capacity=0;
  uint32_t Uptime=0; uint8_t sec=0, mi=0, hr=0, days=0;
  bool Charge=false, Discharge=false, Balance=false;
  int cell_count=4;
  float total_battery_capacity=0, balance_starting_voltage=0;
  float cell_voltage_undervoltage_protection=0;
  float cell_voltage_undervoltage_recovery=0;
  float cell_voltage_overvoltage_protection=0;
  float cell_voltage_overvoltage_recovery=0;
  float max_charge_current=0;
  float max_discharge_current=0;

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

JKBMS jkBms(BMS_MAC);

// ===================== BLE Callbacks =====================
NimBLEScan* pScan;

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override { DBG_PRINTLN("BLE connected"); }
  void onDisconnect(NimBLEClient* pClient, int reason) override {
    DBG_PRINTF("BLE disconnected, reason: %d\n", reason);
    jkBms.connected = false; jkBms.doConnect = false;
  }
};

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* adv) override {
    if (adv->getAddress().toString() == jkBms.targetMAC && !jkBms.connected && !jkBms.doConnect) {
      DBG_PRINTF("Found BMS: %s\n", BMS_MAC);
      jkBms.advDevice = adv;
      jkBms.doConnect = true;
      NimBLEDevice::getScan()->stop();
    }
  }
};

static ScanCallbacks scanCallbacksInstance;

void notifyCB(NimBLERemoteCharacteristic* pChr, uint8_t* pData, size_t length, bool isNotify) {
  jkBms.handleNotification(pData, length);
}

// ===================== JKBMS Methods =====================
bool JKBMS::connectToServer() {
  DBG_PRINTF("Connecting to %s...\n", targetMAC.c_str());
  NimBLEClient* pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
  if (!pClient) {
    pClient = NimBLEDevice::createClient();
    DBG_PRINTLN("New BLE client");
    pClient->setClientCallbacks(new ClientCallbacks(), true);
    pClient->setConnectionParams(12, 12, 0, 150);
    pClient->setConnectTimeout(5000);
  }
  if (!pClient->connect(advDevice)) {
    DBG_PRINTF("Connection failed: %s\n", targetMAC.c_str());
    return false;
  }
  DBG_PRINTF("Connected! RSSI: %d\n", pClient->getRssi());
  NimBLERemoteService* pSvc = pClient->getService("ffe0");
  if (pSvc) {
    pChr = pSvc->getCharacteristic("ffe1");
    if (pChr && pChr->canNotify()) {
      if (pChr->subscribe(true, notifyCB)) {
        DBG_PRINTF("Subscribed to %s\n", pChr->getUUID().toString().c_str());
        delay(500);
        writeRegister(0x97, 0, 0); // Device info
        delay(500);
        writeRegister(0x96, 0, 0); // Cell data
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
    frame=0; received_start=true; received_complete=false;
    for(size_t i=0;i<length;i++) receivedBytes[frame++]=pData[i];
  } else if (received_start && !received_complete) {
    for(size_t i=0;i<length;i++) {
      receivedBytes[frame++]=pData[i];
      if(frame>=300) {
        received_complete=true; received_start=false; new_data=true;
        switch(receivedBytes[4]) {
          case 0x01: bms_settings(); break;
          case 0x02: parseData(); break;
          case 0x03: parseDeviceInfo(); break;
        }
        break;
      }
    }
  }
}

void JKBMS::writeRegister(uint8_t address, uint32_t value, uint8_t length) {
  uint8_t frame[20]={0xAA,0x55,0x90,0xEB,address,length};
  frame[6]=value>>0; frame[7]=value>>8; frame[8]=value>>16; frame[9]=value>>24;
  frame[19]=crc(frame,19);
  if(pChr) pChr->writeValue((uint8_t*)frame,sizeof(frame));
}

void JKBMS::bms_settings() {
  cell_voltage_undervoltage_protection = ((receivedBytes[13]<<24|receivedBytes[12]<<16|receivedBytes[11]<<8|receivedBytes[10])*0.001);
  cell_voltage_overvoltage_protection  = ((receivedBytes[21]<<24|receivedBytes[20]<<16|receivedBytes[19]<<8|receivedBytes[18])*0.001);
  max_charge_current                   = ((receivedBytes[53]<<24|receivedBytes[52]<<16|receivedBytes[51]<<8|receivedBytes[50])*0.001);
  max_discharge_current                = ((receivedBytes[65]<<24|receivedBytes[64]<<16|receivedBytes[63]<<8|receivedBytes[62])*0.001);
  total_battery_capacity               = ((receivedBytes[133]<<24|receivedBytes[132]<<16|receivedBytes[131]<<8|receivedBytes[130])*0.001);
  cell_count                           = (receivedBytes[117]<<24|receivedBytes[116]<<16|receivedBytes[115]<<8|receivedBytes[114]);
  balance_starting_voltage             = ((receivedBytes[141]<<24|receivedBytes[140]<<16|receivedBytes[139]<<8|receivedBytes[138])*0.001);
  DBG_PRINTF("Settings: %d cells, %.2fAh\n", cell_count, total_battery_capacity);
}

void JKBMS::parseDeviceInfo() {
  new_data=false;
  if(frame<134) return;
  Uptime=(receivedBytes[41]<<24)|(receivedBytes[40]<<16)|(receivedBytes[39]<<8)|receivedBytes[38];
  DBG_PRINTF("Device info received, uptime=%lu s\n", Uptime);
}

void JKBMS::parseData() {
  new_data=false; ignoreNotifyCount=10;
  for(int j=0,i=7;i<38&&j<16;j++,i+=2)
    cellVoltage[j]=(uint16_t)(receivedBytes[i]|(receivedBytes[i-1]<<8))*0.001;
  Average_Cell_Voltage  =((uint16_t)(receivedBytes[75]|(receivedBytes[74]<<8)))*0.001;
  Delta_Cell_Voltage    =((uint16_t)(receivedBytes[77]|(receivedBytes[76]<<8)))*0.001;
  for(int j=0,i=81;i<112&&j<16;j++,i+=2)
    wireResist[j]=((uint16_t)(receivedBytes[i]|(receivedBytes[i-1]<<8)))*0.001;
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
  Cycle_Capacity  =((uint32_t)receivedBytes[189]<<24|(uint32_t)receivedBytes[188]<<16|(uint32_t)receivedBytes[187]<<8|receivedBytes[186])*0.001;
  Uptime          =receivedBytes[196]<<16|receivedBytes[195]<<8|receivedBytes[194];
  sec=Uptime%60; Uptime/=60; mi=Uptime%60; Uptime/=60; hr=Uptime%24; days=Uptime/24;
  Charge  =receivedBytes[198]>0;
  Discharge=receivedBytes[199]>0;
  Balance =receivedBytes[201]>0;
  DBG_PRINTLN("--- BMS Data ---");
  DBG_PRINTF("V=%.2fV I=%.2fA SOC=%d%%\n", Battery_Voltage, Charge_Current, Percent_Remain);
  for(int j=0;j<cell_count&&j<16;j++) DBG_PRINTF("  C%d: %.3fV\n",j+1,cellVoltage[j]);
}

// ===================== Sync BLE → Display =====================
static void syncBMSData() {
  bms.cellCount      = jkBms.cell_count;
  for(int i=0;i<16;i++) { bms.cellVoltage[i]=jkBms.cellVoltage[i]; bms.wireResist[i]=jkBms.wireResist[i]; }
  bms.avgCellVoltage   = jkBms.Average_Cell_Voltage;
  bms.deltaCellVoltage = jkBms.Delta_Cell_Voltage;
  bms.battVoltage      = jkBms.Battery_Voltage;
  bms.battPower        = jkBms.Battery_Power;
  bms.chargeCurrent    = jkBms.Charge_Current;
  bms.balanceCurrent   = jkBms.Balance_Curr;
  bms.battT1           = jkBms.Battery_T1;
  bms.battT2           = jkBms.Battery_T2;
  bms.mosTemp          = jkBms.MOS_Temp;
  bms.percentRemain    = jkBms.Percent_Remain;
  bms.capacityRemain   = jkBms.Capacity_Remain;
  bms.nominalCapacity  = jkBms.Nominal_Capacity;
  bms.cycleCount       = jkBms.Cycle_Count;
  bms.cycleCapacity    = jkBms.Cycle_Capacity;
  bms.uptimeSec        = jkBms.Uptime;
  bms.uptimeDays       = jkBms.days;
  bms.uptimeHrs        = jkBms.hr;
  bms.uptimeMin        = jkBms.mi;
  bms.charge           = jkBms.Charge;
  bms.discharge        = jkBms.Discharge;
  bms.balance          = jkBms.Balance;
  bms.balancingAction  = jkBms.Balancing_Action;
  bms.newFrame         = jkBms.new_data;
}

// ===================== Display Drawing =====================
static void drawHeader() {
  tft.fillRect(0,0,SCREEN_W,HEADER_H,CLR_HEADER_BG);
  tft.setTextColor(CLR_WHITE,CLR_HEADER_BG);
  tft.drawString("JK-BMS Monitor",PAD,HEADER_H/2-6,2);
  tft.setTextSize(1);
  if(wifiConnected && bmsConnected) {
    tft.setTextColor(CLR_GREEN,CLR_HEADER_BG);
    tft.drawCentreString("CONNECTED",SCREEN_W/2,HEADER_H/2-6,2);
  } else if(wifiConnected) {
    tft.setTextColor(CLR_AMBER,CLR_HEADER_BG);
    tft.drawCentreString("SCANNING...",SCREEN_W/2,HEADER_H/2-6,2);
  } else {
    tft.setTextColor(CLR_RED,CLR_HEADER_BG);
    tft.drawCentreString("NO WIFI",SCREEN_W/2,HEADER_H/2-6,2);
  }
}

static void drawSOCBar() {
  int y=TOP_PAD, barW=SCREEN_W-PAD*2, barH=SOC_BAR_H-8, x=PAD+4;
  char lbl[16];
  snprintf(lbl,sizeof(lbl),"SOC %d%%",bms.percentRemain);
  tft.setTextColor(CLR_WHITE);
  tft.drawString(lbl,PAD,y,1);
  tft.fillRect(x,y+2,barW,barH,CLR_DARK_GRAY);
  int fw=(barW*bms.percentRemain)/100; fw=constrain(fw,0,barW);
  uint16_t sc=CLR_GREEN;
  if(bms.percentRemain<=10) sc=CLR_RED;
  else if(bms.percentRemain<=25) sc=CLR_YELLOW;
  tft.fillRect(x,y+2,fw,barH,sc);
  tft.drawRect(x,y+2,barW,barH,CLR_LIGHT_GRAY);
}

static uint16_t cellColor(float v) {
  if(v<3.0f) return CLR_RED;
  if(v<3.3f||v>3.9f) return CLR_YELLOW;
  return CLR_GREEN;
}

static void drawCellVoltages() {
  int x=PAD, y=TOP_PAD+SOC_BAR_H;
  tft.setTextColor(CLR_CYAN);
  tft.drawString("CELL VOLTAGES",x,y,1);
  y+=10;
  tft.drawLine(x,y,SCREEN_W-PAD,y,CLR_DARK_GRAY);
  y+=4;
  for(int i=0;i<bms.cellCount&&i<CELL_ROWS;i++) {
    char line[40];
    snprintf(line,sizeof(line),"  C%d: %.3fV",i+1,bms.cellVoltage[i]);
    tft.setTextColor(cellColor(bms.cellVoltage[i]));
    tft.drawString(line,x+4,y,2);
    snprintf(line,sizeof(line),"     R: %.3f ohm",bms.wireResist[i]);
    tft.setTextColor(CLR_GRAY);
    tft.drawString(line,x+4,y+12,1);
    y+=CELL_ROW_H;
  }
}

static void drawBatteryStats() {
  int x=SCREEN_W/2+4, y=TOP_PAD+SOC_BAR_H+10;
  tft.fillRect(x,y-6,SCREEN_W/2-8,STATS_BLOCK_H,CLR_CARD_BG);
  tft.drawRect(x,y-6,SCREEN_W/2-8,STATS_BLOCK_H,CLR_DARK_GRAY);
  tft.setTextColor(CLR_CYAN,CLR_CARD_BG);
  tft.drawString("BATTERY",x+6,y,1);
  char s[24];
  snprintf(s,sizeof(s),"%.1fV",bms.battVoltage);
  tft.setTextColor(CLR_WHITE,CLR_CARD_BG);
  tft.drawString("V: ",x+6,y+12,2);
  tft.drawString(s,x+30,y+12,2);
  snprintf(s,sizeof(s),"%.1fA",bms.chargeCurrent);
  tft.drawString("I: ",x+6,y+28,2);
  tft.drawString(s,x+30,y+28,2);
  snprintf(s,sizeof(s),"%.0fW",bms.battPower);
  tft.drawString("P: ",x+6,y+44,2);
  tft.drawString(s,x+30,y+44,2);
  snprintf(s,sizeof(s),"%.1f/%.1fAh",bms.capacityRemain,bms.nominalCapacity);
  tft.drawString("Cap:",x+6,y+56,1);
  tft.setTextColor(CLR_AMBER,CLR_CARD_BG);
  tft.drawString(s,x+30,y+56,1);
}

static void drawTemps() {
  int x=SCREEN_W/2+4, y=TOP_PAD+SOC_BAR_H+STATS_BLOCK_H+6;
  tft.setTextColor(CLR_CYAN);
  tft.drawString("TEMPERATURE",x,y,1);
  y+=10;
  char s[24];
  uint16_t tc=CLR_WHITE;
  if(bms.battT1>55.0f) tc=CLR_RED;
  else if(bms.battT1<0.0f) tc=CLR_YELLOW;
  snprintf(s,sizeof(s),"  T1: %.1f C",bms.battT1);
  tft.setTextColor(tc);
  tft.drawString(s,x,y,2);
  y+=20;
  snprintf(s,sizeof(s),"  T2: %.1f C",bms.battT2);
  tft.setTextColor(CLR_WHITE);
  tft.drawString(s,x,y,2);
  y+=20;
  snprintf(s,sizeof(s)," MOS: %.1f C",bms.mosTemp);
  tft.drawString(s,x,y,2);
}

static uint16_t btnColor(bool active, int type) {
  if(type==0) return active?0x07C0:0x2400;
  if(type==1) return active?0x001F:0x0008;
  return active?0xFA60:0x6328;
}

static void drawControls() {
  int bw=(SCREEN_W-PAD*3-20)/3, y=SCREEN_H-CTRL_BTN_H-2;
  // Charge
  int bx=PAD;
  tft.fillRect(bx,y,bw,CTRL_BTN_H,btnColor(bms.charge,0));
  tft.drawRect(bx,y,bw,CTRL_BTN_H,CLR_GRAY);
  tft.setTextColor(CLR_WHITE);
  tft.drawCentreString("CHARGE",bx+bw/2,y+CTRL_BTN_H/2-6,2);
  // Discharge
  bx=PAD+bw+10;
  tft.fillRect(bx,y,bw,CTRL_BTN_H,btnColor(bms.discharge,1));
  tft.drawRect(bx,y,bw,CTRL_BTN_H,CLR_GRAY);
  tft.drawCentreString("DISCHG",bx+bw/2,y+CTRL_BTN_H/2-6,2);
  // Balance
  bx=PAD*2+bw*2+10;
  tft.fillRect(bx,y,bw,CTRL_BTN_H,btnColor(bms.balance,2));
  tft.drawRect(bx,y,bw,CTRL_BTN_H,CLR_GRAY);
  tft.drawCentreString("BALANCE",bx+bw/2,y+CTRL_BTN_H/2-6,2);
}

static void drawStatusLine() {
  int x=PAD, y=SCREEN_H-CTRL_BTN_H-18;
  char s[64];
  snprintf(s,sizeof(s),"Bal: %d",bms.balancingAction);
  tft.setTextColor(CLR_AMBER);
  tft.drawString(s,x,y,1);
  snprintf(s,sizeof(s),"Uptime: %ud %uh %um",bms.uptimeDays,bms.uptimeHrs,bms.uptimeMin);
  tft.setTextColor(CLR_GRAY);
  tft.drawString(s,SCREEN_W/2,y,1);
}

static void drawScreen() {
  tft.fillScreen(CLR_BLACK);
  drawHeader(); drawSOCBar(); drawCellVoltages();
  drawBatteryStats(); drawTemps(); drawStatusLine(); drawControls();
}

static void drawUpdated() {
  drawScreen(); // Full redraw for now (safe, no flicker)
}

// ===================== Touch =====================
static void readTouch() {
  if(touchscreen.touched()) {
    TS_Point p = touchscreen.getPoint();
    touch.x = mapTx(p.z > 0 ? p.x : 0);
    touch.y = mapTy(p.z > 0 ? p.y : 0);
    touch.touched = p.z > 0;
  } else {
    touch.touched = false;
  }
}

static void handleTouch() {
  if(!touch.touched) return;
  if(millis()-touch.lastTime<TOUCH_DEBOUNCE_MS) return;
  touch.lastTime = millis();

  int tx=touch.x, ty=touch.y;
  int bw=(SCREEN_W-PAD*3-20)/3, btnY=SCREEN_H-CTRL_BTN_H-2;

  // Charge
  if(tx>=PAD && tx<=PAD+bw && ty>=btnY && ty<=btnY+CTRL_BTN_H) {
    DBG_PRINTLN("Touch: CHARGE");
    requestToggle(0); return;
  }
  // Discharge
  if(tx>=PAD+bw+10 && tx<=PAD+bw*2+10 && ty>=btnY && ty<=btnY+CTRL_BTN_H) {
    DBG_PRINTLN("Touch: DISCHARGE");
    requestToggle(1); return;
  }
  // Balance
  if(tx>=PAD*2+bw*2+10 && tx<=PAD*2+bw*2+10+bw && ty>=btnY && ty<=btnY+CTRL_BTN_H) {
    DBG_PRINTLN("Touch: BALANCE");
    requestToggle(2); return;
  }
}

// ===================== BLE Control =====================
static void requestToggle(int type) {
  if(ctrlPending.writing) return;
  ctrlPending.type=type;
  ctrlPending.pending=true;
  ctrlPending.writing=true;
  DBG_PRINTF("Toggle: type=%d\n",type);
}

static void processControl() {
  if(!ctrlPending.pending||!ctrlPending.writing||!bmsConnected) return;
  ctrlPending.pending=false;

  uint8_t addr;
  if(ctrlPending.type==0) addr=0x1D; // Charge
  else if(ctrlPending.type==1) addr=0x1E; // Discharge
  else addr=0x1F; // Balance

  bool cur=false;
  if(ctrlPending.type==0) cur=bms.charge;
  else if(ctrlPending.type==1) cur=bms.discharge;
  else cur=bms.balance;

  uint32_t val=cur?0:1;
  DBG_PRINTF("BLE write: 0x%02X = %lu\n",addr,val);
  jkBms.writeRegister(addr,val,0x04);
  delay(350);
  DBG_PRINTLN("BLE write done");
  ctrlPending.writing=false;
}

// ===================== Web Server =====================
static void handleRoot() {
  if(LittleFS.exists("/index.html")) {
    server.sendHeader("Location","/main");
    server.send(302);
  } else {
    server.send(200,"text/html",
      "<html><body style='text-align:center;margin-top:60px;font-family:sans-serif;background:#1a1a2e;color:#eee;'>"
      "<h2>JK-BMS Monitor</h2><p>Upload index.html via /fs</p>"
      "<a href='/fs' style='color:#0af;font-size:1.2em;'>File Manager</a>"
      "</body></html>");
  }
}

static void handleJSON() {
  DynamicJsonDocument doc(2048);
  doc["battery_voltage"]=bms.battVoltage;
  doc["battery_power"]=bms.battPower;
  doc["charge_current"]=bms.chargeCurrent;
  doc["percent_remain"]=bms.percentRemain;
  doc["capacity_remain"]=bms.capacityRemain;
  doc["nominal_capacity"]=bms.nominalCapacity;
  doc["cycle_count"]=bms.cycleCount;
  doc["battery_t1"]=bms.battT1;
  doc["battery_t2"]=bms.battT2;
  doc["mos_temp"]=bms.mosTemp;
  doc["charge"]=bms.charge;
  doc["discharge"]=bms.discharge;
  doc["balance"]=bms.balance;
  doc["balancing_action"]=bms.balancingAction;
  doc["balance_curr"]=bms.balanceCurrent;
  doc["avg_cell_voltage"]=bms.avgCellVoltage;
  doc["delta_cell_voltage"]=bms.deltaCellVoltage;
  doc["cell_count"]=bms.cellCount;
  JsonArray cells=doc.createNestedArray("cell_voltages");
  for(int i=0;i<bms.cellCount&&i<16;i++) cells.add(bms.cellVoltage[i]);
  JsonArray resist=doc.createNestedArray("wire_resist");
  for(int i=0;i<bms.cellCount&&i<16;i++) resist.add(bms.wireResist[i]);
  doc["uptime_seconds"]=bms.uptimeSec;
  doc["uptime_days"]=bms.uptimeDays;
  doc["uptime_hours"]=bms.uptimeHrs;
  doc["uptime_minutes"]=bms.uptimeMin;
  String json;
  serializeJson(doc,json);
  server.send(200,"application/json",json);
}

static void handleControl() {
  if(server.method()!=HTTP_POST){server.send(405);return;}
  DynamicJsonDocument doc(200);
  if(deserializeJson(doc,server.arg("plain"))) {server.send(400,"text/plain","Bad JSON");return;}
  String action=doc["action"];
  String state=doc["state"];
  if(state!="on"&&state!="off"){server.send(400);return;}
  uint8_t addr;
  if(action.startsWith("charging")||action=="charge") addr=0x1D;
  else if(action.startsWith("discharging")||action=="discharge") addr=0x1E;
  else addr=0x1F;
  uint32_t val=(state=="on")?1:0;
  DBG_PRINTF("Web control: 0x%02X=%lu\n",addr,val);
  jkBms.writeRegister(addr,val,0x04);
  delay(350);
  server.send(200,"text/plain","OK");
}

static void handleSketchInfo() {
  DynamicJsonDocument doc(200);
  doc["core_version"]=String(ESP_ARDUINO_VERSION_MAJOR)+"."+ESP_ARDUINO_VERSION_MINOR+"."+ESP_ARDUINO_VERSION_PATCH;
  doc["compile_date"]=__DATE__;
  doc["compile_time"]=__TIME__;
  String json;
  serializeJson(doc,json);
  server.send(200,"application/json",json);
}

static void handleFreeHeap() {
  DynamicJsonDocument doc(100);
  uint32_t f=ESP.getFreeHeap();
  char s[16];
  snprintf(s,sizeof(s),"%.1fKB",(float)f/1024);
  doc["free_heap"]=s;
  doc["free_heap_bytes"]=f;
  String json;
  serializeJson(doc,json);
  server.send(200,"application/json",json);
}

static void handleUptime() {
  DynamicJsonDocument doc(100);
  doc["uptime_seconds"]=totalUptime;
  char s[32];
  snprintf(s,sizeof(s),"%lud %02lu:%02lu:%02lu",
    totalUptime/86400,(totalUptime%86400)/3600,(totalUptime%3600)/60,totalUptime%60);
  doc["uptime_formatted"]=s;
  String json;
  serializeJson(doc,json);
  server.send(200,"application/json",json);
}

static void handleFileList() {
  String files;
  File root=LittleFS.open("/");
  File f=root.openNextFile();
  while(f){
    char sz[16];
    if(f.size()<1024) snprintf(sz,sizeof(sz),"%zuB",f.size());
    else if(f.size()<1048576) snprintf(sz,sizeof(sz),"%.1fKB",(float)f.size()/1024);
    else snprintf(sz,sizeof(sz),"%.1fMB",(float)f.size()/1048576);
    files+="<div style='background:#16213e;padding:10px;margin:5px 0;border-radius:4px;display:flex;justify-content:space-between;'>"
    "<span>"+String(f.name())+"</span><span style='color:#888;margin:0 10px'>"+String(sz)+"</span>"
    "<a href='/view?file="+String(f.name())+"' style='color:#0af;text-decoration:none'>view</a> "
    "<a href='/delete?file="+String(f.name())+"' style='color:#f44;text-decoration:none'>del</a></div>";
    f=root.openNextFile();
  }
  server.send(200,"text/html",
    "<html><head><style>body{font-family:sans-serif;background:#1a1a2e;color:#eee;padding:20px;}a{color:#0af;text-decoration:none;}</style></head><body>"
    "<h2>LittleFS Manager</h2>"
    "<form method='post' action='/upload' enctype='multipart/form-data'>"
    "<input type='file' name='upload[]' multiple><input type='submit' value='Upload'></form><br>"
    "<a href='/format' style='color:#f44'>Format LittleFS</a><br><br>"
    "<h3>Files</h3>"+files+"<br><a href='/'>Home</a></body></html>");
}

static void handleFileUpload() {
  if(server.uri()!="/upload")return;
  HTTPUpload& upload=server.upload();
  static File file;
  if(upload.status==UPLOAD_FILE_START){
    file=LittleFS.open("/"+upload.filename,"w");
  }else if(upload.status==UPLOAD_FILE_WRITE){
    if(file)file.write(upload.buf,upload.currentSize);
  }else if(upload.status==UPLOAD_FILE_END){
    if(file)file.close();
    server.sendHeader("Location","/fs");
    server.send(303);
  }
}

static void handleFileDelete(){
  String fn=server.arg("file");
  LittleFS.remove(fn);
  server.sendHeader("Location","/fs");
  server.send(303);
}

static void handleFileView(){
  String fn=server.arg("file");
  if(!fn.startsWith("/"))fn="/"+fn;
  File f=LittleFS.open(fn,"r");
  if(!f){server.send(404);return;}
  String c=f.readString();
  f.close();
  server.send(200,"text/html","<html><body style='background:#1a1a2e;color:#eee;font-family:monospace;padding:20px;'>"
    "<h2>"+fn+"</h2><pre>"+c+"</pre><br><a href='/fs' style='color:#0af'>Back</a></body></html>");
}

static void handleFormat(){LittleFS.format();server.sendHeader("Location","/fs");server.send(303);}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  DBG_PRINTLN("\n=== JK-BMS CYD Monitor ===");

  // TFT
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(CLR_BLACK);
  tft.setTextColor(CLR_CYAN);
  tft.setTextSize(2);
  tft.drawCentreString("JK-BMS",SCREEN_W/2,100,4);
  tft.setTextColor(CLR_WHITE);
  tft.setTextSize(1);
  tft.drawCentreString("CYD Monitor",SCREEN_W/2,160,2);
  tft.drawString("WiFi connecting...",PAD,230,2);

  // Touch
  touchscreen.begin();
  touchscreen.setRotation(1);

  // LittleFS
  if(!LittleFS.begin(true)){
    DBG_PRINTLN("LittleFS mount failed");
  } else {
    DBG_PRINTLN("LittleFS mounted");
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WiFi_SSID, WiFi_Password);
  DBG_PRINTLN("Connecting WiFi...");
  int att=0;
  while(WiFi.status()!=WL_CONNECTED && att<30){
    delay(500);Serial.print(".");att++;
    if(att%6==0){
      char s[40];
      snprintf(s,sizeof(s),"WiFi %d/30",att);
      tft.drawString(s,PAD,230,2);
    }
  }
  if(WiFi.status()==WL_CONNECTED){
    wifiConnected=true;
    DBG_PRINTLN("\nWiFi connected!");
    DBG_PRINTF("IP: %s\n",WiFi.localIP().toString().c_str());
    tft.drawString("WiFi OK",PAD,230,2);
    tft.setTextColor(CLR_GREEN);
    tft.drawString(WiFi.localIP().toString().c_str(),PAD,250,2);
  }else{
    DBG_PRINTLN("\nWiFi failed!");
    tft.setTextColor(CLR_RED);
    tft.drawString("WiFi FAILED",PAD,230,2);
  }

  // Web server
  server.serveStatic("/main",LittleFS,"/index.html");
  server.serveStatic("/style.css",LittleFS,"/style.css","text/css");
  server.on("/",HTTP_GET,handleRoot);
  server.on("/data",HTTP_GET,handleJSON);
  server.on("/control",HTTP_POST,handleControl);
  server.on("/sketchinfo",HTTP_GET,handleSketchInfo);
  server.on("/freeheap",HTTP_GET,handleFreeHeap);
  server.on("/uptime",HTTP_GET,handleUptime);
  server.on("/fs",HTTP_GET,handleFileList);
  server.on("/upload",HTTP_POST,handleFileUpload);
  server.on("/delete",HTTP_GET,handleFileDelete);
  server.on("/view",HTTP_GET,handleFileView);
  server.on("/format",HTTP_GET,handleFormat);
  server.begin();
  DBG_PRINTLN("Web server started");

  // NimBLE
  DBG_PRINTLN("Initializing NimBLE...");
  NimBLEDevice::init("JK-BMS-CYD");
  NimBLEDevice::setPower(3);
  pScan=NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacksInstance);
  pScan->setInterval(100);
  pScan->setWindow(100);
  pScan->setActiveScan(true);

  uptimeStart=millis();
  drawScreen();
}

// ===================== Loop =====================
void loop() {
  unsigned long now=millis();

  server.handleClient();

  // BLE connection
  if(jkBms.doConnect&&!jkBms.connected){
    if(jkBms.connectToServer()){
      bmsConnected=true;
      DBG_PRINTLN("BMS connected!");
    }
    jkBms.doConnect=false;
  }

  // Connection timeout
  if(jkBms.connected&&(now-jkBms.lastNotifyTime>20000)){
    DBG_PRINTLN("BMS timeout");
    bmsConnected=false;
    NimBLEClient* pc=NimBLEDevice::getClientByPeerAddress(jkBms.advDevice->getAddress());
    if(pc)pc->disconnect();
  }

  // Sync data
  if(bms.newFrame||bmsConnected!=(jkBms.connected)){
    syncBMSData();
  }

  // Process control
  processControl();

  // Touch
  readTouch();
  handleTouch();

  // Draw
  if(now-lastDrawTime>=DISPLAY_REFRESH_INTERVAL||bms.newFrame||ctrlPending.writing){
    drawUpdated();
    lastDrawTime=now;
    bms.newFrame=false;
  }

  // Rescan
  if(!bmsConnected&&(now-lastScanTime>=10000)){
    DBG_PRINTLN("Scanning for BMS...");
    pScan->start(5000,false,true);
    lastScanTime=now;
  }

  // Uptime
  totalUptime=(now-uptimeStart)/1000;

  delay(10);
}
