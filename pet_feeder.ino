#include <time.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoOTA.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>
#include "HX711.h"
#include <Wire.h>
#include "RTClib.h"

#include "RtcMem.h"
#include "PinCtrl.h"
#include "Logger.h"
#include "web_page.h"

// Required for LIGHT_SLEEP_T delay mode
extern "C" {
#include "user_interface.h"
}

#define DOG_FEEDER
#define VERTICAL_FEEDER

#define BACKWARD_ROTATE 0
#define SERVO_STOP 90

#ifdef VERTICAL_FEEDER
// Vertical feeder: with paddles
// Forward speed (range: 100-180 for 0% - 100%)
#define SERVO_MOVE_FW 140
// Backward speed (range: 0-80 for 100% - 0%)
#define SERVO_MOVE_BW 0
#ifdef DOG_FEEDER
#define GRT (60)
#elif CAT_FEEDER
// Not tested yet!
#define GRT (220)
#else
#define GRT (220)
#endif
#else
// Horizontal feeder: with auger
// Forward speed (range: 0 - 80 for 100% - 0%)
#define SERVO_MOVE_FW 0
// Backward speed (range: 100 - 180 for 0% - 100%)
#define SERVO_MOVE_BW 180
#ifdef DOG_FEEDER
#define GRT (45)
#elif CAT_FEEDER
#define GRT (220)
#else
#define GRT (220)
#endif
#endif

#define MAP_GRAMS(g) ((GRT + gData.calibration) * g)

#define HR_SEC        3600
#define HOURS(x)      ((x) * HR_SEC)
#define NTP_REFRESH (HOURS(1) * 1000)

#define BTOI(b) ((b & 0x80)?((UINT_MAX << 8) | (int)b):(int)b)

#define WIFI_TIMEOUT  30
#define SLEEP_SECS    HOURS(2)
//#define SLEEP_SECS    60 // for deep-sleep debugging
#define MODEM_SLEEP 1

bool haveScale = false;
bool haveRTC = false;
u16 wakeNum = 0; // number of wake-ups since power-on
u16 crashNum = 0; // number of crashes since power-on

  /* RTC Drift Calculation
   * NVRAM data (registers)
   * 0: drift control
   * 1: actual drift (in seconds)
   * 2: RTC calibration (in days)
   * 4-7: last RTC drift update
   * 
   * drift control structure:
   * 1-3: progress of monitored drifts
   *   0: drift enabled
   */
#define RTC_DRIFT_EN 1
#define RTC_DRIFT_CTL   0
#define RTC_DRIFT_REG   1
#define RTC_DRIFT_CAL   2
#define RTC_DRIFT_ST    4

typedef struct {
  bool enabled;
  u8 progress;
  byte value;
  u8 calibration;
  u32 last_update;
} TimeDrift;

u8 ntpFail = 0;  // number of failed NTP queries
bool timeUpdated = false; // indicator that the current time has been updated, either from RTC or NTP
u64 rtcMillis; // Elapsed millis from boot until we got the actual time
u64 rtcUptime; // Elapsed millis since we got time from NTP or RTC
time_t rtcTime; // Actual NTP/RTC time
TimeDrift rtcDrift; // Calculated RTC drift

/* Digital I/O */
const u8 I2C_SCL = D1;
const u8 I2C_SDA = D2;
const u8 pin_ctrl_stat1 = D3;
const u8 pin_ctrl_stat2 = D4;
const u8 pin_ctrl_servo = D5;
const u8 pin_led_status = D6;
const u8 pin_btn_control = D7;
const u8 pin_ctrl_mux = D8;
/* END Digital I/O */

/* Analog I/O */
const u8 pin_ctrl_adc = A0;
//#define BAT_DIV (100.0f / 300.0f) // voltage divider of 200k/100k at battery input
/*
 * The voltage divider is not actually 1/3 (0.33), because the more we increase the
 * resistor values, the more the voltage drops. So, just measure the input with a 
 * voltmeter and get the actual value to be used. Here, we got 0.2812
 */
#define BAT_DIV 0.2812f
#define ADC_READS 5
/* END Analog I/O */

ButtonManager *button;
bool buttonWake = false;
PinControl pinCtrl(1);
DeviceState state(&pinCtrl, pin_led_status);
Servo servo;

// HX711 circuit wiring
#define DEF_CALI (1905)
#define M300G_CALI (2041.5f)
HX711 scale;

RTC_DS1307 rtc;

/* WiFi configurations */
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
IPAddress staticIP(0, 0, 0, 0);
DNSServer dnsServer;
AsyncWebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

#define host_name "config.info"
#define MYSSID "Pet Feeder"

String *ssidNames; // place-holder for SSID scan results
u8 ssidNum; // number of scanned SSIDs

u8 dst = 0; //daylight saving time
/* END WiFi configurations */

typedef struct {
  u8 hh;
  u8 mm;
  u8 grams;
} job_t;

typedef enum {
    DEEP_SLEEP = 1,
    LIGHT_SLEEP,
    ALWAYS_ON
} SleepType;

typedef enum {
  BATTERY_UNKNOWN,
  BATTERY_ABSENT,
  BATTERY_GOOD,
  BATTERY_LOW,
  BATTERY_VERY_LOW,
} BatteryState;
BatteryState batState = BATTERY_UNKNOWN;
u64 lastBatRead = 0; // millis timestamp of last battery voltage reading

#define MAX_JOBS 6 // up to 6 feeds per day
/* Global data structure to hold what we need in flash memory */
typedef struct {
  size_t sz;
  bool dirty;
  bool corrupted;
  bool initialized;
  SleepType sleep_type;
  char wifi_ssid[32];
  char wifi_pass[32];
  s8 calibration;
  job_t jobs[MAX_JOBS];
  u32 id;
} data_t;
static data_t gData; // global data to be saved in flash
#define DATA_CHKSUM 0xFCECFCEC

// run-time information
typedef struct {
  u8 hh;
  u8 mm;
} run_time_t;
run_time_t lastRun;
time_t lastRunTime;
u64 lastRunMillis;
u64 lastStartMillis;
u64 lastTurnMillis;
u64 lastStallMillis;
u32 lastRunDuration;
u8 stallCheckNum;
u16 peakCurrent;
// Stall current in mili-amps (this will be smaller for smaller angular speeds)
u16 servoStallCurrent = 700;

// job trigger informatio
typedef enum {
  JOB_TRIGGER_NONE,
  JOB_TRIGGER_AUTO,
  JOB_TRIGGER_BUTTON,
} JobTrigger;
JobTrigger startTrigger;
JobTrigger stopTrigger;
u8 jobGrams;

/* I2C reset bit */
#define rtcmem_get_i2c_reset() magic_get_bit(5)
#define rtcmem_set_i2c_reset(rtc) magic_set_bit(rtc, 5)

/* Scale present bit */
#define rtcmem_get_scale() magic_get_bit(4)
#define rtcmem_set_scale(scale) magic_set_bit(scale, 4)

/* RTC present bit */
#define rtcmem_get_rtc() magic_get_bit(3)
#define rtcmem_set_rtc(rtc) magic_set_bit(rtc, 3)

/* Daylight Saving Time bit */
#define rtcmem_get_dst() magic_get_bit(2)
#define rtcmem_set_dst(dst) magic_set_bit(dst, 2)

/* Deep-sleep enabled bit */
#define rtcmem_get_sleep() magic_get_bit(1)
#define rtcmem_set_sleep(sleep) magic_set_bit(sleep, 1)

/* Initialized bit */
#define rtcmem_get_init() magic_get_bit(0)
#define rtcmem_set_init(ini) \
{\
  magic_set_bit(ini, 0);\
  if (gData.initialized != ini) {\
    gData.initialized = ini;\
    gData.dirty = true;\
  }\
}

// RUN_BLK
void rtcmem_get_lastRun(run_time_t &rt) {
  rt.hh = rtcmem_get_byte(RUN_BLK, 0);
  rt.mm = rtcmem_get_byte(RUN_BLK, 1);
}
void rtcmem_set_lastRun(const run_time_t &rt) {
  rtcmem_set_byte(RUN_BLK, 0, rt.hh);
  rtcmem_set_byte(RUN_BLK, 1, rt.mm);
}

//STATS_BLK
#define rtcmem_get_wakes() rtcmem_get_u16(STATS_BLK, 0)
#define rtcmem_set_wakes(v) rtcmem_set_u16(STATS_BLK, v, 0)
#define rtcmem_get_crashes() rtcmem_get_u16(STATS_BLK, 2)
#define rtcmem_set_crashes(v) rtcmem_set_u16(STATS_BLK, v, 2)

void rtc_drift_reset(u32 t = 0) {
  LOG(1, "Resetting RTC drift regs\n");
  u8 val = 0;
  rtc.writenvram(RTC_DRIFT_CTL, &val, 1);
  rtc.writenvram(RTC_DRIFT_REG, &val, 1);
  rtc.writenvram(RTC_DRIFT_CAL, &val, 1);
  rtc.writenvram(RTC_DRIFT_ST, (u8*)&t, 4);
}

void rtc_drift_update(DateTime rtc_time, DateTime ntp_time, byte cur_drift = 0) {
#if RTC_DRIFT_EN
  u8 ctl;

  rtc.readnvram((u8*)&rtcDrift.last_update, 4, RTC_DRIFT_ST);
  rtc.readnvram(&ctl, 1, RTC_DRIFT_CTL);
  rtc.readnvram(&(rtcDrift.value), 1, RTC_DRIFT_REG);
  rtc.readnvram(&(rtcDrift.calibration), 1, RTC_DRIFT_CAL);
  LOG(3, "RTC drift control: 0x%02X\n", ctl);
  LOG(3, "RTC drift value: 0x%02X\n", rtcDrift.value);
  LOG(3, "RTC drift calibration: 0x%02X\n", rtcDrift.calibration);

  rtcDrift.progress = REG_GET(ctl, 3, 1);
  rtcDrift.enabled = ctl & BIT(0);

  int diff = rtcTime - rtcDrift.last_update;
  if (rtcTime - rtcDrift.last_update < HOURS(rtcDrift.calibration * 24)) {
  //if (rtcTime - rtcDrift.last_update < 60) {
    LOG(3, "RTC drift: not enough hours passed to calculate RTC drift (actual seconds: %u)\n", diff);
    return;
  }

  // If drift is already established, update rtcTime
  if (rtcDrift.enabled) {
    TimeSpan ts(BTOI(rtcDrift.value));
    rtc_time = rtc_time - ts;
    rtc.adjust(rtc_time);
    rtcTime = rtc_time.unixtime();
    rtcDrift.last_update = (u32)rtcTime;
    rtc.writenvram(RTC_DRIFT_ST, (u8*)&rtcDrift.last_update, 4);
    LOG(1, "Updated RTC time with drift of: %ds: %02u/%02u/%u %02u:%02u:%02u\n", BTOI(rtcDrift.value),
        rtc_time.day(), rtc_time.month(), rtc_time.year(),
        rtc_time.hour(), rtc_time.minute(), rtc_time.second());
    return;
  }

  if (!cur_drift)
    return;
  
  if (!rtcDrift.calibration) {
    rtcDrift.calibration = 1;
    rtc.writenvram(RTC_DRIFT_CAL, &(rtcDrift.calibration), 1);
    LOG(1, "Updated RTC drift calibration period: %u days\n", rtcDrift.calibration);
  }
  
  if (rtcDrift.progress < 4) {
    DateTime last_tm(rtcDrift.last_update);   
    LOG(1, "Saved RTC drift: %ds (last check: %02u/%02u/%u %02u:%02u:%02u)\n", BTOI(rtcDrift.value),
        rtc_time.day(), rtc_time.month(), rtc_time.year(),
        rtc_time.hour(), rtc_time.minute(), rtc_time.second());
    
    rtcDrift.value += (cur_drift - rtcDrift.value) / (rtcDrift.progress + 1);
    rtcDrift.progress++;
    LOG(1, "Current RTC drift: %ds (progress: %u, updated drift: %ds)\n", BTOI(cur_drift), rtcDrift.progress, BTOI(rtcDrift.value));
    
    TimeSpan ts(BTOI(rtcDrift.value));
    rtc_time = rtc_time - ts;
    rtcTime = rtc_time.unixtime();
    rtc.adjust(rtc_time);
    rtcDrift.last_update = (u32)rtcTime;
    rtc.writenvram(RTC_DRIFT_ST, (u8*)&rtcDrift.last_update, 4);
    LOG(1, "RTC date with drift (%d secs, %u drift checks): %02u/%02u/%u %02u:%02u:%02u\n",
      ts.totalseconds(),
      rtcDrift.progress,
      rtc_time.day(), rtc_time.month(), rtc_time.year(),
      rtc_time.hour(), rtc_time.minute(), rtc_time.second());      

    ctl |= REG_PUT(rtcDrift.progress, 3, 1);
    if (rtcDrift.progress == 3) {
      ctl |= BIT(0);
      rtcDrift.calibration = 30;
      LOG(1, "Updating RTC calibration period: %u days\n", rtcDrift.calibration);
      rtc.writenvram(RTC_DRIFT_CAL, &(rtcDrift.calibration), 1);
      rtc.readnvram(&(rtcDrift.calibration), 1, RTC_DRIFT_CAL);
      LOG(1, "Updated RTC calibration period: %u days\n", rtcDrift.calibration);
    }
    rtc.writenvram(RTC_DRIFT_CTL, &ctl, 1);
    rtc.writenvram(RTC_DRIFT_REG, &(rtcDrift.value), 1);
  }
#endif
}

bool update_rtc(time_t ntpTime = 0) {
  u32 saved_time = 0;
  long long diff = 0;
  
  rtcTime = 0;
  
  if (!haveRTC)
    return false;

  if (rtcmem_get_i2c_reset()) {
    reset_i2c();
    if (!rtc.begin()) {
      LOG(1, "RTC probe failed in update!\n");
      return false;
    }
    rtcmem_set_i2c_reset(false);
  }

  if (!rtc.isrunning()) {
    DateTime t(F(__DATE__), F(__TIME__));
    saved_time = t.unixtime();
    rtc.adjust(t);
    // Reset drift and update saved time
    rtc_drift_reset(saved_time);
  }

  rtcMillis = millis();
  DateTime ntp_time((u32)ntpTime);
  DateTime rtc_time = rtc.now();
  rtcTime = rtc_time.unixtime();
  rtc.readnvram((u8*)&saved_time, 4, 4);
  diff = rtcTime - saved_time;
  DateTime last_time(saved_time);
  LOG(1, "RTC date: %02u/%02u/%u %02u:%02u:%02u (Last saved: %02u/%02u/%u %02u:%02u:%02u)\n",
      rtc_time.day(), rtc_time.month(), rtc_time.year(),
      rtc_time.hour(), rtc_time.minute(), rtc_time.second(),
      last_time.day(), last_time.month(), last_time.year(),
      last_time.hour(), last_time.minute(), last_time.second());
  if (saved_time && abs(diff) > HOURS(48)) {
    LOG(1, "Saved time mismatches current time with more than 48h. Requesting time from NTP!\n");
    rtcTime = 0;
    if (!ntpTime) {
      state.setState(GET_NTP_TIME);
      return false;
    }
  } else {
    rtc_drift_update(rtc_time, ntp_time);
  }

  if (ntpTime != 0) {
    LOG(1, "NTP timestamp: %llu, RTC timestamp: %llu\n", ntpTime, rtcTime);
    LOG(1, "NTP time (%02u/%02u/%u %02u:%02u:%02u) | RTC time (%02u/%02u/%u %02u:%02u:%02u)\n",
          ntp_time.day(), ntp_time.month(), ntp_time.year(), ntp_time.hour(), ntp_time.minute(), ntp_time.second(),
          rtc_time.day(), rtc_time.month(), rtc_time.year(), rtc_time.hour(), rtc_time.minute(), rtc_time.second());
    diff = rtcTime - ntpTime;
    if (abs(diff) > 30) {
      LOG(1, "NTP time differs from RTC time with more than 30s. Adjusting RTC!\n");
      rtc.adjust(ntp_time);
      rtcTime = ntpTime;
      saved_time = ntp_time.unixtime();
      rtc.writenvram(RTC_DRIFT_ST, (u8*)&saved_time, 4);
    } else {
      rtc_drift_update(rtc_time, ntp_time, (byte)diff);
    }
    
    // Even though we didn't adjust the RTC, update the rtc_updated time
    u32 rtc_updated;
    system_rtc_mem_read(RTCU_BLK, &rtc_updated, sizeof(rtc_updated));
    rtc_updated = (u32)ntpTime;
    system_rtc_mem_write(RTCU_BLK, &rtc_updated, sizeof(rtc_updated));
  }

  system_rtc_mem_write(RTC_BLK, &rtcTime, sizeof(rtcTime));
  rtcUptime = 0;
  system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
  
  timeUpdated = true;
  return true;
}

time_t get_time(bool useNtp = false) {
  time_t now = 0;

  if (useNtp) {
    timeClient.setTimeOffset(HOURS(2+dst));
    timeClient.forceUpdate();
    now = timeClient.getEpochTime();
    if (now > (2021 - 1970) * 365 * HOURS(24)) {
      LOG(1, "Current NTP time: %llu -> %s", now, ctime(&now));
      //check for DST
      struct tm *t = localtime(&now);
      if (!dst &&
          ((t->tm_mon > 2 && t->tm_mon < 9) ||
          (t->tm_mon == 2 && 31 - t->tm_mday < 7) ||
          (t->tm_mon == 9 && 31 - t->tm_mday >= 7))) {
        dst = 1;
        rtcmem_set_dst(true);
        now += 3600;
        LOG(1, "Updating DST to: %u -> %s", dst, ctime(&now));
      } else if (dst &&
          ((t->tm_mon < 2 && t->tm_mon > 9) ||
          (t->tm_mon == 2 && 31 - t->tm_mday >= 7) ||
          (t->tm_mon == 9 && 31 - t->tm_mday < 7))) {
        dst = 0;
        rtcmem_set_dst(false);
        now -= 3600;
        LOG(1, "Updating DST to: %u -> %s", dst, ctime(&now));
      }
      if (!haveRTC) {
        // don't have RTC, we rely on NTP
        system_rtc_mem_write(RTC_BLK, &now, sizeof(now));
        rtcUptime = 0;
        system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
      } else {
        update_rtc(now);
      }
      rtcMillis = millis();
      timeUpdated = true;
      rtcTime = now;
      return now;
    }
    return 0;
  }

  if (!rtcTime)
      system_rtc_mem_read(RTC_BLK, &rtcTime, sizeof(rtcTime));
  if (!rtcUptime)
    system_rtc_mem_read(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
  u64 uptime = rtcUptime + (millis() - rtcMillis);
  LOG(3, "Saved NTP/RTC time: %llu -> %s", rtcTime, ctime(&rtcTime));
  if (!rtcTime)
    return 0;
  now = rtcTime;
  if (uptime > 1000) {
    now += uptime / 1000;
    LOG(3, "Updated NTP/RTC time: %llu -> %s", now, ctime(&now));
  }
  
  return now;
}

String html_processor(const String& var) {
  String ret = "";
  char c[2];
  u32 ss = millis() / 1000;
  u32 mm = ss / 60;
  ss %= 60;
  u32 hh = mm / 60;
  mm %= 60;
  u32 dd = hh / 24;
  hh %= 24;

  if (var == "DATE") {
    time_t now  = get_time();
    if (now)
      ret += "<p class=\"time\">Date: " + String(ctime(&now)) + "</p>";
  } else if (var == "DD") {
    sprintf(c, "%u", dd);
    ret = String(c);
  } else if (var == "HH") {
    sprintf(c, "%02u", hh);
    ret = String(c);
  } else if (var == "MM") {
    sprintf(c, "%02u", mm);
    ret = String(c);
  } else if (var == "SS") {
    sprintf(c, "%02u", ss);
    ret = String(c);
  } else if (var == "CAL") {
    ret = String(gData.calibration);
  } else if (var == "HIDE_BTN") {
    if (strlen(gData.wifi_ssid))
      ret += "+";
    else
      ret += "-";
  } else if (var == "HIDE_WIFI_CFG") {
    if (strlen(gData.wifi_ssid))
      ret += " style=\"display:none\"";
    else
      ret += " style=\"display:block\"";
  } else if (var == "WIFI_PASS" && strlen(gData.wifi_pass)) {
    ret = String(gData.wifi_pass);
  } else if (var == "WIFI_SSIDS") {
    String ssid = String(gData.wifi_ssid);
    if (!ssidNum)
      ret += String("\t\t<option value=\"") + ssid + String("\">") + ssid + String("</option>\n");
    for (int i = 0; i < ssidNum; i++) {
      String sel = ">";
      if (ssid == ssidNames[i])
        sel = " selected>";
      ret += String("\t\t<option value=\"") + ssidNames[i] + String("\"") + sel + ssidNames[i] + String("</option>\n");
    }
  } else if (var.startsWith("SLEEP_")) {
    u8 slp = var[var.length() - 1] - '0';
    if (gData.sleep_type == slp)
        ret = String(" checked");
  } else if (var.startsWith("TD")) {
    u8 id = var[var.length() - 1] - '0';
    if (id < MAX_JOBS) {
      job_t *job = &gData.jobs[id];
      if (job->grams)
        ret = "block";
      else
        ret = "none";
    }
  } else if (var.startsWith("TM")) {
    u8 id = var[var.length() - 1] - '0';
    if (id < MAX_JOBS) {
      job_t *job = &gData.jobs[id];
      if (job->grams) {
        sprintf(c, "%02d", job->hh);
        ret = String(c) + ":";
        sprintf(c, "%02d", job->mm);
        ret += String(c);
      }
    }
  } else if (var.startsWith("GR")) {
    u8 id = var[var.length() - 1] - '0';
    if (id < MAX_JOBS) {
      job_t *job = &gData.jobs[id];
      if (job->grams)
        ret = String(job->grams);
    }
  }
  
  return ret;
}

void load_flash(bool first_boot = false) {
  EEPROM.begin(sizeof(gData));
  if (gData.corrupted)
    return;
  EEPROM.get(0, gData);
  LOG(1, "Data loaded from flash (saved size: %d, actual size:%d, id:0x%08X)\n", gData.sz, sizeof(gData), gData.id);
  if (gData.sz != sizeof(gData) || gData.id != DATA_CHKSUM) {
    LOG(1, "Flash data corrupted! Resetting data\n");
    memset(&gData, 0, sizeof(gData));
    gData.corrupted = true;
    EEPROM.put(0, gData);
    EEPROM.end();
    return;
  }
  gData.dirty = false;
  if (first_boot) {
    //wifi_to_rtc();
    rtcmem_set_init(gData.initialized);
    if (gData.sleep_type == DEEP_SLEEP)
      rtcmem_set_sleep(true);
  }
  if (!gData.sleep_type) {
    gData.sleep_type = DEEP_SLEEP;
    gData.dirty = true;
  }
}

void save_flash() {
  if (!gData.dirty)
    return;
    
  gData.sz = sizeof(gData);
  gData.corrupted = false;
  gData.dirty = false;
  gData.id = DATA_CHKSUM;
  
  EEPROM.put(0, gData);
  EEPROM.commit();
  LOG(1, "Data saved to flash!\n");
}

bool wifi_connect() {
  if (!strlen(gData.wifi_ssid) || !strlen(gData.wifi_pass))
    load_flash();
  if (!strlen(gData.wifi_ssid) || !strlen(gData.wifi_pass))
    return false;

  LOG(1, "WiFi connect: SSID=%s PASS=%s\n", gData.wifi_ssid, gData.wifi_pass);
  if (staticIP[0]) {
    IPAddress gateway(staticIP[0], staticIP[1], staticIP[2], 1);
    IPAddress subnet(255, 255, 255, 0);
    bool ret = WiFi.config(staticIP, gateway, subnet);
    LOG(1, "Trying to acquire staticIP: %s, using gateway: %s (ret=%u)\n",
      staticIP.toString().c_str(), gateway.toString().c_str(), ret);
  }
  
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(MYSSID);     
  WiFi.begin(gData.wifi_ssid, gData.wifi_pass);

  return true;
}

void wifi_disconnect() {
  if (WiFi.status() != WL_CONNECTED)
    return;
    
  LOG(1, "Disconnecting Wifi...\n");
  WiFi.disconnect();
  delay(1);
}

void handleNotFound(AsyncWebServerRequest *request) {
  String msg = "File Not Found\n\n";
  msg += "URI: ";
  msg += request->url();
  msg += "\nMethod: ";
  msg += (request->method() == HTTP_GET) ? "GET" : "POST";
  msg += "\nArguments: ";
  msg += request->args();
  msg += "\n";

  for (u8 i = 0; i < request->args(); i++) {
    msg += " " + request->argName(i) + ": " + request->arg(i) + "\n";
  }

  request->send(404, "text/plain", msg);
}

void handleConfig(AsyncWebServerRequest *request) {
  String msg = "";
  String html;
  String wifi_ssid, wifi_pass;
  u8 sleep_type;
  int lastJobId = -1;

  load_flash();
  for (u8 i = 0; i < request->args(); i++) {
    String argN = request->argName(i);
    String argV = request->arg(i);
    LOG(1, "argN=%s, argV=%s\n", argN, argV);
    if (argN == "ssid") {
      wifi_ssid = argV;
    } else if (argN == "pass") {
      wifi_pass = argV;
    } else if (argN == "sleep") {
      sleep_type = argV.toInt();
    } else if (argN == "cal") {
      s8 cal = argV.toInt();
      if (cal != gData.calibration) {
        if (cal < 0 && abs(cal) < (GRT / 2))
          cal = - (GRT / 2);
        gData.calibration = cal;
        gData.dirty = true;
        msg += "<br>Saved feeder calibration: " + argV;
      }
    } else if (argN.startsWith("tm")) {
      u8 id = argN[argN.length() - 1] - '0';
      if (!id && !argV.length()) {
        // erase all jobs
        LOG(1, "Erasing %u jobs: total size: %u\n", sizeof(gData.jobs) / sizeof(job_t), sizeof(gData.jobs));
        memset(&gData.jobs, 0, sizeof(gData.jobs));
        msg += "<br>Erased all previous jobs! ";
      } else if (id < MAX_JOBS && argV != "") {
        int c = argV.indexOf(':');
        if (c == -1) {
          msg += "<br>Invalid job time: " + argV;
        } else {
          lastJobId = id;
          u8 hh = argV.substring(0, c).toInt();
          u8 mm = argV.substring(c+1, argV.length()).toInt();
          if (hh > 23 || mm > 59) {
            msg += "<br>Invalid job time: " + argV;
          } else {
            gData.jobs[id].hh = hh;
            gData.jobs[id].mm = mm;
            gData.dirty = true;
            msg += "<br>Saved job time: " + argV;
            LOG(1, "Saving job[%u] time: %u:%u\n", id, gData.jobs[id].hh, gData.jobs[id].mm); 
          }
        }
      }
    } else if (argN.startsWith("gr")) {
      u8 id = argN[argN.length() - 1] - '0';
      if (id < MAX_JOBS && id == lastJobId) {
       int val = argV.toInt();
        if (val > 255)
          val = 255;
        gData.jobs[id].grams = val;
        gData.dirty = true;
        msg += ", grams: " + String(val);
        LOG(1, "Saving job[%u] grams: %u\n", id, gData.jobs[id].grams);
      }
    }
  }
  if (lastJobId != -1)
    for (int i = lastJobId + 1; i < MAX_JOBS; i++)
      gData.jobs[i] = {0, 0, 0};

  if ((wifi_ssid == "" || wifi_pass == "") && !strlen(gData.wifi_ssid)) {
    html = "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"></head><body>";
    html += "<h2 style=\"font-size: 1.4rem\">";
    html += "WiFi network name and/or pass not provided!";
    html += "</body></html>";
    request->send(406, "text/html", html);
    return;
  }

  if (wifi_ssid != String(gData.wifi_ssid) ||
      wifi_pass != String(gData.wifi_pass)) {
    memset(gData.wifi_ssid, 0, sizeof(gData.wifi_ssid));
    wifi_ssid.toCharArray(gData.wifi_ssid, wifi_ssid.length() + 1);
    memset(gData.wifi_pass, 0, sizeof(gData.wifi_pass));
    wifi_pass.toCharArray(gData.wifi_pass, wifi_pass.length() + 1);
    gData.dirty = true;
    msg += "WiFi network saved as: " + wifi_ssid + "<br>";
    state.setState(WIFI_TEST);
  }
  
  if (sleep_type != 0 && sleep_type != gData.sleep_type) {
    gData.dirty = true;
    msg += "<br>Sleep Type saved as: ";
    switch (sleep_type) {
      case 1:
        gData.sleep_type = DEEP_SLEEP;
        rtcmem_set_sleep(true);
        msg += "Deep Sleep";
        break;
      case 2:
        gData.sleep_type = LIGHT_SLEEP;
        msg += "Light Sleep";
        break;
      case 3:
        gData.sleep_type = ALWAYS_ON;        
        msg += "Always On";
        break;
    }
  }

  if (msg == "")
    msg = "No changes applied!";
  else if (state.getState() != WIFI_TEST)
    state.setState(CONFIGURED);
  html = "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"></head><body>";
  html += "<h2 style=\"font-size: 1.4rem\">";
  html += msg;
  html += "</body></html>";
  request->send(200, "text/html", html);

  save_flash();
}

void handleStatus(AsyncWebServerRequest *request) {
  String msg = "";
  String html;
  char tmp[64];
  time_t now;
  struct tm *t;
  char c[2];
  u32 ss = millis() / 1000;
  u32 mm = ss / 60;
  ss %= 60;
  u32 hh = mm / 60;
  mm %= 60;
  u32 dd = hh / 24;
  hh %= 24;
  
  now = get_time();
  t = localtime(&now);
  
  html = "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<style>code { display: block;  font-family: monospace;  white-space: pre;  margin: 1em 0;}</style></head><body>";
  html += "<h2 style=\"font-size: 1.2rem\">";
  html += "<p class=\"time\">Date: " + String(ctime(&now)) + "</p>";
  sprintf(tmp, "<p>Uptime: %udays %02u:%02u:%02u </p>", dd, hh, mm, ss);
  html += String(tmp);
  sprintf(tmp, "<p>Since last boot: Wake-ups: %u | Crashes: %u</p>", wakeNum, crashNum);
  html += String(tmp);
  sprintf(tmp, "<p>Last run: %02u:%02u</p>", lastRun.hh, lastRun.mm);
  html += String(tmp);
  
  if (startTrigger == JOB_TRIGGER_AUTO)
    html += "<p>Last Pump trigger: AUTO</p>";
  else if (stopTrigger == JOB_TRIGGER_BUTTON)
    html += "<p>Last Pump trigger: BUTTON</p>";
  else
    html += "<p>Last Stop trigger: UNDEFINED</p>";
    
  if (stopTrigger == JOB_TRIGGER_AUTO)
    html += "<p>Last Stop trigger: AUTO</p>";
  else if (stopTrigger == JOB_TRIGGER_BUTTON)
    html += "<p>Last Stop trigger: BUTTON</p>";
  else
    html += "<p>Last Stop trigger: UNDEFINED</p>";

  html += "<p>Last Run Time: " + String(ctime(&lastRunTime)) + "</p>";
  html += "<p>Last Run Duration: " + String(lastRunDuration / 1000) + " seconds</p>";
  html += "<p>DST: " + String(dst) + "</p>";
  html += "<p>Serial output:</p>";
  html += "</h2><h2 style=\"font-size: 0.8rem\"><pre><code>";
  html += Logger::getLog();
  html += "</code></pre></h2>";
  html += "</body></html>";
  request->send(200, "text/html", html);
}

bool wifi_config(int mode) {
  if (mode == WIFI_AP) {
    LOG(1, "Starting WiFi scan...\n");
    ssidNum = WiFi.scanNetworks(false, false);
    
    if (ssidNum == 0) {
      LOG(1, "No networks found!\n");
      return false;
    } else if (ssidNum > 0) {
      LOG(1, "%d networks found\n", ssidNum);
      ssidNames = new String[ssidNum];
      for (int8_t i = 0; i < ssidNum; i++) {
        ssidNames[i] = WiFi.SSID(i);
        ssidNames[i].trim();
        if (ssidNames[i].length() > 2)
          LOG(1, "%02d: SSID: %s\n", i, ssidNames[i]);
      }
   
    } else {
      LOG(1, "WiFi scan error %d\n", ssidNum);
      return false;
    }
    
    LOG(1, "Setting soft-AP %s ...\n", MYSSID);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    if(WiFi.softAP(MYSSID)) {
      IPAddress IP = WiFi.softAPIP();
      LOG(1, "AP Ready! IP address: ");
      Serial.println(IP);
    } else {
      LOG(1, "AP setup failed!");
    }
    dnsServer.setTTL(300);
    dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);
    dnsServer.start(DNS_PORT, host_name, apIP);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, html_processor);
  });
  server.on("/config", handleConfig);
  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();
  LOG(1, "HTTP server started for: http://%s\n", host_name);
  
  return true;
}

#define FEED_25G MAP_GRAMS(25)
bool check_for_jobs(u64 *timeTillNext = NULL) {   
  time_t now;
  struct tm *t;
  job_t *job = NULL;
  bool useNtp = (WiFi.status() == WL_CONNECTED);

  if (useNtp && (millis() - rtcMillis) < NTP_REFRESH) 
    useNtp = false;
    
  now = get_time(useNtp);
  if (!now) {
    state.setState(GET_NTP_TIME);
    return true;
  }

  if (timeTillNext)
    *timeTillNext = UINT_MAX;
  t = localtime(&now);
  run_time_t thisRun = { t->tm_hour, t->tm_min };
  LOG(4, "Last run: %02u:%02u, this run: %02u:%02u\n",
    lastRun.hh, lastRun.mm, thisRun.hh, thisRun.mm);
  for (u8 i = 0; i < MAX_JOBS; i++) {
    job_t *nextJob = &gData.jobs[i];
    // Check for done jobs
    if (lastRun.hh && t->tm_hour <= lastRun.hh && t->tm_min <= lastRun.mm + 2)
      continue;
    LOG(4, "Checking job: %02u:%02u, grams: %u\n",
        nextJob->hh, nextJob->mm, nextJob->grams);
    if (!nextJob->grams || i == MAX_JOBS - 1) {
      if (lastRun.hh || lastRun.mm) {
        lastRun = {0, 0};
        LOG(1, "Resetting lastRun: %02u:%02u\n", lastRun.hh, lastRun.mm);
        rtcmem_set_lastRun(lastRun);
      }
      break;
    } else if (t->tm_hour == nextJob->hh && t->tm_min >= nextJob->mm && t->tm_min - 2 <= nextJob->mm) {
      run_time_t thisRun = { t->tm_hour, t->tm_min };
      job = nextJob;
      LOG(2, "Preparing job: %02u:%02u, grams: %u\n",
          job->hh, job->mm, job->grams);
      break; 
    } else if (timeTillNext) {
      s32 secs = (nextJob->hh - t->tm_hour);
      if (secs < 0)
        secs += 24;
      secs *= 3600;
      secs += (nextJob->mm - t->tm_min) * 60;
      secs -= t->tm_sec;
      LOG(4, "Time till this job: %us\n", secs);
      if (!*timeTillNext || secs < *timeTillNext)
        *timeTillNext = secs;
    }
  }
  if (!job || !job->grams)
    return false;
  
  // We are good, let's start the Servo
  startTrigger = JOB_TRIGGER_AUTO;
  lastRun = thisRun;
  jobGrams = job->grams;
  rtcmem_set_lastRun(lastRun);
  state.setState({START_SERVO, MAP_GRAMS(job->grams)});
  return true;
}

void save_uptime(u64 sleep_time = 0) {
  u64 mil = millis();
  LOG(1, "Time elapsed since boot: %llu, elapsed time since we got NTP: %llu\n", mil, rtcMillis);
  //u64 uptime;
  //system_rtc_mem_read(UPTIME_BLK, &uptime, sizeof(uptime));
  LOG(1, "Saved uptime: %llu\n", rtcUptime);
  rtcUptime += mil + sleep_time / 1000;
  if (rtcMillis > 0)
    rtcUptime -= rtcMillis;
  system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
  LOG(1, "Updated uptime: %llu\n", rtcUptime);
}

void do_light_sleep() {
  LOG(1, "Entering light-sleep\n");
  #if MODEM_SLEEP
  WiFi.forceSleepBegin();
  delay(1);
  #endif
  state.setState(SLEEPING);
}

void enter_sleep() {
  u64 sleep_now;
  if (gData.sleep_type == LIGHT_SLEEP) {
    do_light_sleep();
    return;
  }

  if (gData.sleep_type != DEEP_SLEEP)
    return;

  sleep_now = SLEEP_SECS;
  u64 time_till_next;
  check_for_jobs(&time_till_next);
  LOG(1, "Time till next job: %llus, max_sleep: %llus\n", time_till_next, sleep_now);
  if (time_till_next != UINT_MAX && time_till_next < sleep_now) {
    if (time_till_next > 10)
      sleep_now = time_till_next - 10;
     else
      sleep_now = time_till_next;
  }

  if (sleep_now < 20) {
    if (!timeUpdated)
      state.setState(GET_NTP_TIME);
    else
      do_light_sleep();
    return;
  }

  sleep_now *= 1000000;
  // Connect D0 to RST to wake up
  pinMode(D0, WAKEUP_PULLUP);
  
  wifi_disconnect();
  unInstallButton(button);
  pinCtrl.stopLeds();

  save_flash();
  save_uptime(sleep_now);
  rtcmem_set_magic(MAGIC_ID);
  LOG(1, "Entering sleep for %llu seconds (%d)\n", (sleep_now / 1000000), wakeNum);
  delay(10);
  if (batState == BATTERY_LOW ||
      batState == BATTERY_VERY_LOW) {
    /* 
     * Prepare the system for deep-sleep for sleep_now microseconds, but with intermitent 
     * wakeups once every 3 seconds.
     */
    u16 sleep_secs = (sleep_now / 1000000);
    if (batState == BATTERY_VERY_LOW)
      sleep_secs |= 0x8000;
    rtcmem_set_lpsleep_id(MAGIC_ID);
    rtcmem_set_lpsleep_secs(sleep_secs);
    
    ESP.deepSleep((3000000 - 10000), WAKE_RF_DEFAULT);
  }
  ESP.deepSleep((sleep_now - 10000), WAKE_RF_DEFAULT);
}

/*
 * read_adc: reads the ADC pin value, assuming the ADC has an internal voltage divider 
 * of R1:220k + R2:100k
 * r: extra resitor (in Kohm) added to R1, in order to increase the voltage divider
 * returns: calculated voltage read by ADC
 */
u16 read_adc(u16 r = 0) {
  u16 adc_sum = 0, val, mV;
  
  for (int i = 0; i < ADC_READS; i++) {
    adc_sum += analogRead(pin_ctrl_adc);
    delay(1);
  }
  val = adc_sum / ADC_READS;
  
  mV = map(val, 0, 1024, 0, (320 + r) * 10);
  u64 msec = millis();
  u64 sec = msec / 1000;
  msec %= 1000;
  LOG(4, "@%llu.%llus ADC value: %u, mV: %u (maxV: %u)\n", sec, msec, val, mV, (320 + r) * 10);
  return mV;
}

void calibrate_load_sell() {
  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());
  
  scale.set_scale(DEF_CALI);
  scale.tare();
  Serial.println("Place weight on the scale...");
  for (int i = 5; i > 0; i--) {
    Serial.printf("Seconds remaining: %d\n", i);
    delay(1000);
  }
  float w = scale.get_units(10);
  Serial.printf("scale read: %f\n", w);
  if (w > 300) {
    scale.set_scale(M300G_CALI);
    w = scale.get_units(10);
    Serial.printf("Retry: scale read: %f\n", w);
  }
}

void check_battery() {
  // Read battery voltage every hour
  if (batState == BATTERY_ABSENT ||
      (lastBatRead && (millis() - lastBatRead < HOURS(1) * 1000)))
    return;
  
  // Reading battery voltage
  digitalWrite(pin_ctrl_stat1, LOW);
  digitalWrite(pin_ctrl_stat2, HIGH);
  lastBatRead = millis();
  u16 adc_val = read_adc();
  u16 bat_mV = (adc_val * 1.0f) / BAT_DIV;
  LOG(1, "Battery mV: %u\n", bat_mV);
  if (bat_mV >= 2500 && bat_mV < 4500) {
    // We have 1x18650
    if (bat_mV < 3000) {
      LOG(1, "1x battery present and very low (%umV)\n", bat_mV);
      batState = BATTERY_VERY_LOW;
    } else if (bat_mV < 3200) {
      LOG(1, "1x battery present and low (%umV)\n", bat_mV);
      batState = BATTERY_LOW;
    } else {
      LOG(1, "1x battery present and in good shape (%umV)\n", bat_mV);
      batState = BATTERY_GOOD;
    }
  } else if (bat_mV >= 5500) {
    // We have 2x18650
    if (bat_mV < 6200) {
      LOG(1, "2x battery present and very low (%umV)\n", bat_mV);
      batState = BATTERY_VERY_LOW;
    } else if (bat_mV < 6500) {
      LOG(1, "2x battery present and low (%umV)\n", bat_mV);
      batState = BATTERY_LOW;
    } else {
      LOG(1, "2x battery present and in good shape (%umV)\n", bat_mV);
      batState = BATTERY_GOOD;
    }
  } else {
    batState = BATTERY_ABSENT;
  }
  
  //LOG(1, "DEBUG: forcing battery to BATTERY_VERY_LOW");
  //batState = BATTERY_VERY_LOW; // For debugging only
  
  digitalWrite(pin_ctrl_stat1, HIGH);
}

void reset_i2c() {
  LOG(2, "Resetting i2c lines\n");
  /*
   * Since we have a non-i2c device (like the HX711 scale controller) on the i2c lines, 
   * we need to bring the i2c lines in a state where an actual i2c device (like the RTC)
   * can be accessed. 
   * Since ESP8266 doesn't have support for Wire::end(), we'll just do it here, manually.
   */
  Wire.endTransmission();
  digitalWrite(I2C_SDA, LOW);
  digitalWrite(I2C_SCL, LOW);
  delay(5);
  rtcmem_set_i2c_reset(true);
}

void do_lowbat_sleep(bool bat_very_low, u16 sleep_secs) {
  u8 num_blinks = 2;
  u8 del = 100;
  
  if (bat_very_low) {
    num_blinks = 4;
    del = 75;
  }
  for (u8 i = 0; i < num_blinks; i++) {
    digitalWrite(pin_led_status, HIGH);
    delay(del);
    digitalWrite(pin_led_status, LOW);
    delay(del);
  }
  if (sleep_secs > 3) {
    sleep_secs -= 3;
    rtcmem_set_lpsleep_id(MAGIC_ID);
    if (bat_very_low)
      sleep_secs |= 0x8000;
    rtcmem_set_lpsleep_secs(sleep_secs);
  } else {
    rtcmem_set_lpsleep_id(0);
  }
  
  ESP.deepSleep(3000000, WAKE_RF_DEFAULT);
}

void setup() {
  pinMode(pin_btn_control, INPUT);
  pinMode(pin_led_status, OUTPUT);
  bool btn_wake = (digitalRead(pin_btn_control) == LOW);

  if (!btn_wake && rtcmem_get_lpsleep_id() == MAGIC_ID) {
    u16 sleep_secs = rtcmem_get_lpsleep_secs();
    bool bat_very_low = false;
    if (sleep_secs & 0x8000) {
      bat_very_low = true;
      sleep_secs &= ~(0x8000);
    }
    if (sleep_secs > 0)
      do_lowbat_sleep(bat_very_low, sleep_secs);
  }

  u32 magic_id;
  bool first_boot = false;
  bool reset_ntp = false;
  bool rtc_init = false;

  pinMode(pin_ctrl_stat1, OUTPUT);
  pinMode(pin_ctrl_stat2, OUTPUT);
  pinMode(pin_ctrl_mux, OUTPUT);
  pinMode(pin_ctrl_servo, OUTPUT);

  /* 
   * Put the mux pin to H in order to disable the RST mux while awake.
   * This will prevent resetting us while pressing the button.
   */
  digitalWrite(pin_ctrl_mux, HIGH);
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  LOG(1, "\nSetup\n");
  LOG(1, "Button wake: %u\n", btn_wake);

  button = installButton(pin_btn_control, BTN_TYPE_MOMENTARY, BTN_PULLUP);
  if (!button) {
    LOG(1, "Could not install button!!!\n");  
    delay(5000);
    ESP.restart();
  }

  check_battery();

  pinCtrl.addPin(pin_led_status, LOW);

  rtcTime = 0;
  rtcUptime = 0;
  rtcDrift.calibration = 1;
  magic_id = rtcmem_get_magic();
  crashNum = rtcmem_get_crashes();
  
  if (magic_id != MAGIC_ID) {
    if (magic_id == RUN_MAGIC_ID) {
      crashNum = rtcmem_get_crashes();
      LOG(1, "Oooups! Just had a crash...\n");
      crashNum++;
      rtcmem_set_crashes(crashNum);
      reset_ntp = true;
    } else if (magic_id == OTA_MAGIC_ID) {
      LOG(1, "Restart after OTA\n");
    } else{
      // TODO: For debugging only, remove this delay
      //delay(10000);
      LOG(1, "First boot... clearing regs!\n");
      magic_id = 0;
      first_boot = true;
      system_rtc_mem_write(RTC_BLK, &rtcTime, sizeof(rtcTime));
      system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
      for (u8 blk = MAGIC_BLK; blk >= STATS_BLK; blk--)
        system_rtc_mem_write(blk, &magic_id, sizeof(magic_id));

      //Probe HX711 load_cell
      reset_i2c();
      scale.begin(I2C_SDA, I2C_SCL);
      if (scale.wait_ready_timeout(1000, 100)) {
        LOG(1, "Scale present!\n");
        rtcmem_set_scale(true);
        scale.power_down();
      } else {
        LOG(1, "Scale absent!\n");
      }

      //Probe RTC
      reset_i2c();
      if (rtc.begin()) {
        rtcmem_set_rtc(true);
        rtc_init = true;
        LOG(1, "RTC present!\n");
        rtc.writeSqwPinMode(DS1307_ON);
        Ds1307SqwPinMode sq_pin = rtc.readSqwPinMode();
        LOG(2, "RTC control register: 0x%02X\n", (u8)sq_pin);
        rtcmem_set_i2c_reset(false);
      } else {
        LOG(1, "RTC absent!\n");
      }
    }
  } else {
    system_rtc_mem_read(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
    system_rtc_mem_read(RTC_BLK, &rtcTime, sizeof(rtcTime));
  }

  dst = !!rtcmem_get_dst();
  load_flash(first_boot);
  wakeNum = rtcmem_get_wakes();
  if (magic_id == MAGIC_ID) {
    wakeNum++;
    rtcmem_set_wakes(wakeNum);
  }
  crashNum = rtcmem_get_crashes();
  rtcmem_get_lastRun(lastRun);
  haveRTC = rtcmem_get_rtc();
  haveScale = rtcmem_get_scale();
  rtcmem_set_magic(RUN_MAGIC_ID);

  if (reset_ntp && !haveRTC) {
    // Since we crashed, we need to start over with NTP time
    system_rtc_mem_write(RTC_BLK, &rtcTime, sizeof(rtcTime));
    system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
  }

  if (haveRTC && !rtc_init) {
    if (rtcmem_get_i2c_reset())
      reset_i2c();
    haveRTC = false;
    // Sometimes it seems to fail, so retry up to 5 times if it fails
    for (u8 i = 1; i <= 5; i++) {
      if (!rtc.begin()) {
          LOG(1, "RTC probe failed in wakeup: %u\n", i);
          delay(50);
      } else {
        haveRTC = true;
        break;
      }
    }
    rtcmem_set_i2c_reset(false);
  }
  state.setState(NO_STATE);
  update_rtc();

  // Only tested stall current for 0-40 / 140-180, so will use these by now
  if (SERVO_MOVE_FW > 100)
    servoStallCurrent = map(SERVO_MOVE_FW, 140, 180, 400, 700);
  else
    servoStallCurrent = map(SERVO_MOVE_FW, 0, 40, 700, 400);

  time_t now = get_time();
  LOG(1, "Wake-ups: %u\n", wakeNum);
  LOG(1, "Crashes: %u\n", crashNum);
  LOG(1, "Initialized: %d\n", gData.initialized);
  LOG(1, "Sleep type: %d\n", gData.sleep_type);
  LOG(1, "DST: %u\n", dst);
  LOG(1, "RTC present: %s\n", haveRTC?"TRUE":"FALSE");
  LOG(1, "Scale present: %s\n", haveScale?"TRUE":"FALSE");
  LOG(1, "NTP/RTC uptime: %llu\n", rtcUptime);
  LOG(1, "NTP/RTC time: %llu -> %s", rtcTime, ctime(&rtcTime));
  LOG(1, "Servo stall current: %u\n", servoStallCurrent);
  LOG(1, "Last Run: %02u:%02u\n", lastRun.hh, lastRun.mm);
  LOG(1, "WiFi config: SSID=%s PASS=%s\n", gData.wifi_ssid, gData.wifi_pass);
  for (int i = 0; i < MAX_JOBS; i++) {
    job_t *job = &gData.jobs[i];
    if (job->grams)
      LOG(1, "Active job: %02u:%02u, grams: %u\n", job->hh, job->mm, job->grams);
  }

  timeClient.begin();
  
  if (btn_wake) {
    LOG(1, "Button wake-up\n");
    state.setState({BTN_WAKE_UP, 500});
  } else if (state.getState() != NO_STATE) {
    state.setStateWithNext({state.getState(), state.getWaitTime()}, {RUN_IDLE, 0});
  } else {
    state.setState(RUN_IDLE);
  }

  if (gData.corrupted || !strlen(gData.wifi_ssid) || !strlen(gData.wifi_pass))
    state.setState(CONFIGURE);
}

void loop() {
  pinCtrl.run();

  switch (state.getState()) {
    case SLEEPING:
      wifi_set_sleep_type(NONE_SLEEP_T);
      check_battery();
      if (check_for_jobs())
        return;
      if (getAction(button, BTN_ACT_PRESSED)) {
          startTrigger = JOB_TRIGGER_BUTTON;
          jobGrams = 0;
          state.setState({START_SERVO, 0});;
          return;
      }
      if (getAction(button, BTN_ACT_SINGLE_TAP)) {
        #if MODEM_SLEEP
        WiFi.forceSleepWake();
        delay(10);
        #endif
        state.setState({WAKE_UP, 300000});
        return;
      }
      if (getAction(button, BTN_ACT_DOUBLE_TAP)) {
        state.setState(BEGIN_OTA);
        return;
      }
      digitalWrite(pin_led_status, HIGH);
      delay(100);
      digitalWrite(pin_led_status, LOW);
      wifi_set_sleep_type(LIGHT_SLEEP_T);
      delay(3000);
      return;
      
    case WAKE_UP:
      if (getAction(button, BTN_ACT_PRESSED)) {
        startTrigger = JOB_TRIGGER_BUTTON;
        state.setState({START_SERVO, 0});
        return;
      }
      if (getAction(button, BTN_ACT_SINGLE_TAP)) {
        state.setState(RUN_IDLE);
        return;
      }
      if (getAction(button, BTN_ACT_SINGLE_TAP_ON)) {
        state.setState({PUSH_BUTTON, 2000}, {RUN_IDLE, 0}, {CONFIGURE, 0});
        return;
      }
      if (getAction(button, BTN_ACT_DOUBLE_TAP)) {
        state.setState(BEGIN_OTA);
        return;
      }
      if (state.expired()) {
        state.setState(RUN_IDLE);
        return;
      }
      if (WiFi.status() != WL_CONNECTED) {
        wifi_connect();
        state.setState({WIFI_CONNECT, WIFI_TIMEOUT * 1000}, {WAKE_UP, state.getWaitTime()}, {RUN_IDLE, 0});
      }
      return;

    case BTN_WAKE_UP:
      if (state.expired()) {
        if (digitalRead(pin_btn_control) == LOW) {
          setButtonAction(button, BTN_ACT_PRESSED);
          startTrigger = JOB_TRIGGER_BUTTON;
          if (haveRTC)
            state.setState({START_SERVO, 0});
          else
            state.setStateWithNext({START_SERVO, 0}, {GET_NTP_TIME, 0});
        } else {
          /* 
           *  Since we are woken-up by the button, we might aswell check the NTP time,
           *  so comment out this branch for now
          if (haveRTC) 
            state.setState({WAKE_UP, 300000});
          else
          */
            state.setStateWithNext({GET_NTP_TIME, 0}, {WAKE_UP, 300000});
        }
      }
      return;
      
    case RUN_IDLE:
      if (haveRTC) {
        u32 rtc_updated = 0;
        system_rtc_mem_read(RTCU_BLK, &rtc_updated, sizeof(rtc_updated));
        u64 diff = rtcTime - rtc_updated;
        if (diff > HOURS(rtcDrift.calibration * 24)) {
          LOG(2, "RTC last updated %llus ago (RTC calibration: %u days). Updating NTP time to calibrate RTC.\n", diff, rtcDrift.calibration);
          state.setState(GET_NTP_TIME);
          return;
        }
      }
      if (check_for_jobs())
        return;
      if (getAction(button, BTN_ACT_PRESSED)) {
        startTrigger = JOB_TRIGGER_BUTTON;
        state.setState({START_SERVO, 0});
        return;
      }
      if (getAction(button, BTN_ACT_SINGLE_TAP_ON)) {
        state.setState({PUSH_BUTTON, 2000}, {RUN_IDLE, 0}, {CONFIGURE, 0});
        return;
      }
      if (getAction(button, BTN_ACT_DOUBLE_TAP)) {
        state.setState(BEGIN_OTA);
        return;
      }
      break;
      
    case START_SERVO:
      LOG(1, "Preparing servo\n");
      wifi_disconnect();
      if (haveScale) {
        LOG(1, "Powering scale\n");
        reset_i2c();
        scale.begin(I2C_SDA, I2C_SCL);
        scale.wait_ready_timeout(1000, 100);
        scale.set_scale(DEF_CALI);
        scale.tare();
      }
      
      // Prepare stat pins for stall detection
      digitalWrite(pin_ctrl_stat1, HIGH);
      digitalWrite(pin_ctrl_stat2, HIGH);
      
      LOG(1, "Starting servo\n");
      servo.attach(pin_ctrl_servo);
      lastTurnMillis = lastStallMillis = lastStartMillis = stallCheckNum = 0;
      peakCurrent = 0;
      lastRunTime = get_time();
      state.setState({RUN_SERVO, state.getWaitTime()});
      lastRunMillis = millis();
      servo.write(SERVO_MOVE_FW);
      return;
      
    case RUN_SERVO:
      {
        delay(10);
        u16 mA = read_adc();
        u64 currentMillis = millis();
        if (mA > peakCurrent)
          peakCurrent = mA;
        // We have a 1 ohm current sense resistor, so current = voltage
        // Also, allow the servo to start (100ms should do)
        if (mA > servoStallCurrent && currentMillis - lastStartMillis > 100)
          stallCheckNum++;
        else
          stallCheckNum = 0;
          
        if (stallCheckNum > 5) {
          stallCheckNum = 0;
          if (servo.read() == SERVO_MOVE_FW) {
            LOG(1, "Servo stall detected in FW direction!\n");
            lastStartMillis = currentMillis;
            servo.write(SERVO_MOVE_BW);
          } else {
            LOG(1, "Servo stall detected in BW direction!\n");
            lastStartMillis = currentMillis;
            servo.write(SERVO_MOVE_FW);
          }
          lastStallMillis = currentMillis;
        }
        
        if (lastStallMillis && currentMillis - lastStallMillis > 500) {
          LOG(1, "Resuming servo from stall\n");
          if (servo.read() == SERVO_MOVE_FW) {
            lastStartMillis = currentMillis;
            servo.write(SERVO_MOVE_BW);
          } else {
            state.extendWaitTime((currentMillis - lastStallMillis) * 12 / 10);
            lastStartMillis = currentMillis;
            servo.write(SERVO_MOVE_FW);
          }
          lastStallMillis = 0;
        }

#if BACKWARD_ROTATE
        if (currentMillis - lastTurnMillis > 300 && servo.read() == SERVO_MOVE_FW) {
          lastStartMillis = currentMillis;
          servo.write(SERVO_MOVE_BW);
          lastTurnMillis = currentMillis;
        } else if (currentMillis - lastTurnMillis > 100 && servo.read() == 180) {
          state.extendWaitTime((currentMillis - lastTurnMillis) * 12 / 10);
          lastStartMillis = currentMillis;
          servo.write(SERVO_MOVE_FW);
          lastTurnMillis = currentMillis;
        }
#endif
        
        stopTrigger = JOB_TRIGGER_NONE;
        if ((servo.read() == SERVO_MOVE_FW) && state.expired()) {
          lastRunDuration = (currentMillis - lastRunMillis) - state.getExtraTime();
          LOG(1, "Stopping servo (state expired): total run: %u ms, peak current: %u mA\n", lastRunDuration, peakCurrent);
          stopTrigger = JOB_TRIGGER_AUTO;
        } else if (!state.getWaitTime() &&
                   startTrigger == JOB_TRIGGER_BUTTON &&
                   (getAction(button, BTN_ACT_RELEASED) || !buttonPressed(button))) {
          lastRunDuration = (currentMillis - lastRunMillis) - state.getExtraTime();
          LOG(1, "Stopping servo (push button): total run: %u ms, peak current: %u mA\n", lastRunDuration, peakCurrent);
          stopTrigger = JOB_TRIGGER_BUTTON;
        }
        if (stopTrigger != JOB_TRIGGER_NONE) {
          servo.write(SERVO_STOP);
          servo.detach();

          u8 gr;
          if (haveScale) {
            // Wait a little bit for the grains to settle on the scale
            delay(200);
            float w = scale.get_units(10);
            LOG(1, "Scale read: %.2fg\n", w);
            scale.power_down();
            gr = round(w + 0.49);
          } else {
            jobGrams = 0;
          }
          
          if (jobGrams > 2 && (jobGrams - 2) > gr) {
          //if (false) {
            jobGrams -= gr;
            LOG(1, "Reschedulling for more grams: %d\n", jobGrams);
            startTrigger = JOB_TRIGGER_AUTO;
            state.setState({START_SERVO, MAP_GRAMS(jobGrams)});
          } else {
            if (!state.setNextState())
              state.setState(RUN_IDLE);
            jobGrams = 0;
          }
        }
      }
      return;

    case PUSH_BUTTON:
      if (state.expired())
        state.setFailed();
      else if (getAction(button, BTN_ACT_RELEASED))
        state.setSuccess();
      return;

    case CONFIGURE:
      if (wifi_config(WIFI_AP)) {
        state.setState(CONFIGURING);
        return;
      }
      LOG(1, "Failed to setup WiFi AP!!!\n");  
      break;
      
    case CONFIGURING:
      {
        if (getAction(button, BTN_ACT_DOUBLE_TAP)) {
          wifi_disconnect();
          state.setState(RUN_IDLE);
          return;
        }
        WiFiMode_t wifi_mode = WiFi.getMode();
        switch (wifi_mode) {
          case WIFI_AP:
            if (WiFi.softAPgetStationNum())
              dnsServer.processNextRequest();
            break;
          default:
            break;
        }
        return;
      }
      break;
      
    case CONFIGURED:
      LOG(1, "Testing WiFi params succeded!\n");
      save_flash();
      state.setState(GET_NTP_TIME);
      break;
      
    case WIFI_TEST:
      LOG(1, "Testing new WiFi params...\n");
      wifi_disconnect();
      if (wifi_connect()) {
        state.setState({WIFI_CONNECT, WIFI_TIMEOUT * 1000}, {CONFIGURED, 0}, {CONFIGURE, 0});
        return;
      }
      state.setState(CONFIGURE);
      return;
      
    case WIFI_CONNECT:
      if (WiFi.status() == WL_CONNECTED) {
        u32 elapsed = state.getElapsed();
        staticIP = WiFi.localIP();
        LOG(1, "Connected to: %s (after %.02f seconds)\n", WiFi.SSID(), (double)elapsed / 1000);
        LOG(1, "IP address: %s; Hostname: %s\n", staticIP.toString().c_str(), WiFi.hostname().c_str());

        if (ssidNum > 0) {
          delete[] ssidNames;
          ssidNum = 0;
        }
        // since we are connected to wifi, update rtcTime
        //time_t now = get_time(true);
        //LOG(1, "Current NTP time: %llu -> %s", now, ctime(&now));
        wifi_config(WIFI_STA);
        state.setSuccess();
      } else if (WiFi.status() == WL_WRONG_PASSWORD || WiFi.status() == WL_NO_SSID_AVAIL) { 
        LOG(1, "WiFi config is wrong! (invalid SSID or PASS)\n");
        // Re-configure failed, reload the flash contents
        load_flash();
        state.setFailed();
      } else if (state.expired()) {
        LOG(1, "WiFi connect timed-out!\n");
        // Re-configure failed, reload the flash contents
        load_flash();
        state.setFailed();
      }
      return;

    case GET_NTP_TIME:
      if (WiFi.status() != WL_CONNECTED) {
        time_t now = get_time();
        if (now && ntpFail++ > 2) {
          LOG(1, "NTP time sync failed, using estimate: %s\n", ctime(&now));
          if (!state.setNextState())
            state.setState(RUN_IDLE);
          return;
        }
        if (wifi_connect()) {
          state.setState({WIFI_CONNECT, WIFI_TIMEOUT * 1000}, {GET_NTP_TIME, 0}, {GET_NTP_TIME, 0});
          return;
        } else {
          state.setState(CONFIGURE);
          return;
        }
      }
      if (!get_time(true)) {
        time_t now = get_time();
        if (now && ntpFail++ > 2) {
          LOG(1, "NTP time sync failed, using estimate: %s\n", ctime(&now));
          if (!state.setNextState())
            state.setState(RUN_IDLE);
        } else {
          LOG(1, "Waiting for time...\n");
          state.setState({WAIT_NTP_TIME, 5000});
        }
        return;
      }
      ntpFail = 0;
      if (!state.setNextState())
        state.setState(RUN_IDLE);
      return;

    case WAIT_NTP_TIME:
      {
        time_t now = get_time(true);
        if (now > 0) {
          LOG(1, "Current NTP time: %llu -> %s", now, ctime(&now));
          if (!state.setNextState())
            state.setState(RUN_IDLE);
        } else if (state.expired()) {
          LOG(1, "Get NTP time timed-out!\n");
          state.setState(GET_NTP_TIME);
        }
        return;
      }

    case BEGIN_OTA:
      if (WiFi.status() != WL_CONNECTED) {
        wifi_connect();
        state.setState({WIFI_CONNECT, WIFI_TIMEOUT * 1000}, {BEGIN_OTA, 0}, {RUN_IDLE, 0});
        return;
      }
      ArduinoOTA.onStart([]() {
        pinCtrl.stopLeds();
      });
      ArduinoOTA.onEnd([]() {
        digitalWrite(pin_led_status, LOW);
        u8 state = LOW;
        for (int i = 0; i < 20; i++) {
          delay(200);
          if (state == HIGH)
            state = LOW;
          else
            state = HIGH;
          digitalWrite(pin_led_status, state);
        }
        save_flash();
        save_uptime();
        rtcmem_set_magic(OTA_MAGIC_ID);
      });
      ArduinoOTA.onError([](ota_error_t error) {
        digitalWrite(pin_led_status, HIGH);
        u8 state = HIGH;
        for (int i = 0; i < 6; i++) {
          delay(200);
          if (state == HIGH)
            state = LOW;
          else
            state = HIGH;
          digitalWrite(pin_led_status, state);
        }
        (void)error;
        ESP.restart();
      });
      ArduinoOTA.begin();
      state.setState(OTA_UPDATE);
      return;
      
    case OTA_UPDATE:
      ArduinoOTA.handle();
      if (getAction(button, BTN_ACT_DOUBLE_TAP)) {
        delay(1000);
        state.setState(RUN_IDLE);
      }
      return;
      
    default:
      break;
  }
 
  enter_sleep();
}
