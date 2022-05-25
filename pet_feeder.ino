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
RTC_DS1307 rtc;

#include "Bits.h"
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

#define MAP_GRAMS(g) ((GRT + g_data.calibration) * g)

#define FPM_SLEEP_MAX_TIME           0xFFFFFFF
#define MAX_UINT      0x7fffffff
#define HR_SEC        3600
#define HOURS(x)      ((x) * HR_SEC)
#define NTP_REFRESH (HOURS(1) * 1000)

#define WIFI_TIMEOUT  30
#define SLEEP_SECS    HOURS(2)
#define MODEM_SLEEP 1

/* RTC memory defines */
#define MAGIC_ID 0xfcec
#define RUN_MAGIC_ID 0xccec
#define OTA_MAGIC_ID 0xdcec
/*
 * Structure of MAGIC_BLK:
 * 31-16: MAGIC_ID
 * 15-3: Reserved
 *    2: Daylight Saving Time
 *    1: Deep Sleep Enabled
 *    0: Initialized bit
 */
/*
 * Structure of RUN_BLK:
 * 15-0: lastRun
 */
#define RTC_BLK 127 // 127-128 - time obtained from NTP/RTC
#define UPTIME_BLK 125 //125-126 - time elapsed since we got NTP/RTC
#define MAGIC_BLK 124
#define RTCU_BLK 123 // last updated RTC time
#define RUN_BLK 122
#define STATS_BLK 121
/* END RTC memory defines */

u16 wake_num = 0; // number of wake-ups since power-on
u16 crash_num = 0; // number of crashes since power-on
u8 ntpFail = 0;
bool first_boot = false;
bool time_updated = false;
bool haveScale = false;
bool haveRTC = false;
bool i2cReset = false;
u64 rtcMillis; // Elapsed millis from boot until we got the actual time
time_t rtcTime; // actual NTP/RTC time
u64 rtcUptime; // Elapsed millis since we got time from NTP or RTC

/* Digital I/O */
const u8 pin_ctrl_stat1 = D3;
const u8 pin_ctrl_stat2 = D4;
const u8 pin_ctrl_servo = D5;
const u8 pin_led_status = D6;
const u8 pin_btn_control = D7;
const u8 pin_ctrl_mux = D8;


//#define BAT_DIV (100.0f / 300.0f) // voltage divider of 200k/100k at battery input
/*
 * The voltage divider is not actually 1/3 (0.33), because the more we increase the
 * resistor values, the more the voltage drops. So, just measure the input with a 
 * voltmeter and get the actual value to be used. Here, we got 0.2812
 */
#define BAT_DIV 0.2812f
#define ADC_READS 5
const u8 pin_ctrl_adc = A0;

ButtonManager *button;
bool buttonWake = false;
// create pin controller with 1 pin (status led)
PinControl pinCtrl(1);
DeviceState state(&pinCtrl, pin_led_status);
Servo servo;

// HX711 circuit wiring
#define DEF_CALI (1905)
#define M300G_CALI (2041.5f)
const int LOADCELL_DOUT_PIN = D2;
const int LOADCELL_SCK_PIN = D1;
HX711 scale;

/* END Digital I/O */

/* WiFi configurations */
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
IPAddress staticIP(0, 0, 0, 0);
DNSServer dnsServer;
#define host_name "config.info"
#define MYSSID "Pet Feeder"

AsyncWebServer server(80); 

String *ssid_names; // place-holder for SSID scan results
u8 ssid_num; // number of scanned SSIDs

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
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
static data_t g_data; // global data to be saved in flash
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

bool updateRTC() {
  rtcTime = 0;
  
  if (!haveRTC)
    return false;

  if (i2cReset) {
    reset_i2c();
  
    if (!rtc.begin()) {
      LOG(1, "RTC probe failed in update!\n");
      return false;
    }
    i2cReset = false;
  }

  if (!rtc.isrunning())
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  DateTime rtc_time = rtc.now();
  rtcTime = rtc_time.unixtime();
  system_rtc_mem_write(RTC_BLK, &rtcTime, sizeof(rtcTime));
  rtcUptime = 0;
  system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));

  LOG(1, "RTC date: %02u/%02u/%u %02u:%02u:%02u\n",
        rtc_time.day(), rtc_time.month(), rtc_time.year(),
        rtc_time.hour(), rtc_time.minute(), rtc_time.second());
  
  return true;
}

bool sleepRTC() {
  if (!haveRTC)
    return false;

  if (i2cReset) {
    // Reset i2c lines
    digitalWrite(LOADCELL_DOUT_PIN, LOW);
    digitalWrite(LOADCELL_SCK_PIN, LOW);
  
    if (!rtc.begin()) {
      LOG(1, "RTC probe failed in sleep!\n");
      return false;
    }
  }

  rtc.writeSqwPinMode(DS1307_ON);

  return true;
}

time_t getCurrentTime(bool useNtp = false) {
  time_t now = 0;

  if (useNtp) {
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
        rtc_set_dst(true);
        now += 3600;
        LOG(1, "Updating DST to: %u -> %s", dst, ctime(&now));
      } else if (dst &&
          ((t->tm_mon < 2 && t->tm_mon > 9) ||
          (t->tm_mon == 2 && 31 - t->tm_mday >= 7) ||
          (t->tm_mon == 9 && 31 - t->tm_mday < 7))) {
        dst = 0;
        rtc_set_dst(false);
        now -= 3600;
        LOG(1, "Updating DST to: %u -> %s", dst, ctime(&now));
      }
      if (!haveRTC) {
        // don't have RTC, we rely on NTP
        system_rtc_mem_write(RTC_BLK, &now, sizeof(now));
        rtcUptime = 0;
        system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
      }
      rtcMillis = millis();
      time_updated = true;
      if (updateRTC()) {
        DateTime ntp_time((u32)now);
        DateTime rtc_time((u32)rtcTime);
        LOG(1, "NTP timestamp: %llu, RTC timestamp: %llu\n", now, rtcTime);
        LOG(1, "NTP time (%02u/%02u/%u %02u:%02u:%02u) | RTC time (%02u/%02u/%u %02u:%02u:%02u)\n",
              ntp_time.day(), ntp_time.month(), ntp_time.year(), ntp_time.hour(), ntp_time.minute(), ntp_time.second(),
              rtc_time.day(), rtc_time.month(), rtc_time.year(), rtc_time.hour(), rtc_time.minute(), rtc_time.second());
        long long diff = now - rtcTime;
        if (abs(diff) > 30) {
          LOG(1, "NTP time (%02u:%02u:%02u %02u/%02u/%u) differs from RTC time (%u:%u:%u %u/%u/%u) with more than 60s. Adjusting RTC!\n",
              ntp_time.hour(), ntp_time.minute(), ntp_time.second(), ntp_time.day(), ntp_time.month(), ntp_time.year(),
              rtc_time.hour(), rtc_time.minute(), rtc_time.second(), rtc_time.day(), rtc_time.month(), rtc_time.year());
          rtc.adjust(ntp_time);
          // Also, update the regs
          system_rtc_mem_write(RTC_BLK, &now, sizeof(now));
          rtcUptime = 0;
          system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
        }
      }
      u32 rtc_updated = (u32)now;
      system_rtc_mem_write(RTCU_BLK, &rtc_updated, sizeof(rtc_updated));
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

String processor(const String& var) {
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
    time_t now  = getCurrentTime();
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
    ret = String(g_data.calibration);
  } else if (var == "HIDE_BTN") {
    if (strlen(g_data.wifi_ssid))
      ret += "+";
    else
      ret += "-";
  } else if (var == "HIDE_WIFI_CFG") {
    if (strlen(g_data.wifi_ssid))
      ret += " style=\"display:none\"";
    else
      ret += " style=\"display:block\"";
  } else if (var == "WIFI_PASS" && strlen(g_data.wifi_pass)) {
    ret = String(g_data.wifi_pass);
  } else if (var == "WIFI_SSIDS") {
    String ssid = String(g_data.wifi_ssid);
    if (!ssid_num)
      ret += String("\t\t<option value=\"") + ssid + String("\">") + ssid + String("</option>\n");
    for (int i = 0; i < ssid_num; i++) {
      String sel = ">";
      if (ssid == ssid_names[i])
        sel = " selected>";
      ret += String("\t\t<option value=\"") + ssid_names[i] + String("\"") + sel + ssid_names[i] + String("</option>\n");
    }
  } else if (var.startsWith("SLEEP_")) {
    u8 slp = var[var.length() - 1] - '0';
    if (g_data.sleep_type == slp)
        ret = String(" checked");
  } else if (var.startsWith("TD")) {
    u8 id = var[var.length() - 1] - '0';
    if (id < MAX_JOBS) {
      job_t *job = &g_data.jobs[id];
      if (job->grams)
        ret = "block";
      else
        ret = "none";
    }
  } else if (var.startsWith("TM")) {
    u8 id = var[var.length() - 1] - '0';
    if (id < MAX_JOBS) {
      job_t *job = &g_data.jobs[id];
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
      job_t *job = &g_data.jobs[id];
      if (job->grams)
        ret = String(job->grams);
    }
  }
  
  return ret;
}

// MAGIC_BLK
#define rtc_get_magic() rtc_get_u16(MAGIC_BLK, 2)
#define rtc_set_magic(v) rtc_set_u16(MAGIC_BLK, v, 2)

bool rtc_get_sleep() {
  u32 w_data = 0;
  system_rtc_mem_read(MAGIC_BLK, &w_data, sizeof(w_data));
  LOG(2, "rtc_get_sleep -> w_data: 0x%02X\n", w_data);
   
  return MAGIC_GET_SLEEP(w_data);
}

void rtc_set_sleep(bool sleep) {
  u32 w_data = 0;
  system_rtc_mem_read(MAGIC_BLK, &w_data, sizeof(w_data));
  w_data = MAGIC_SET_SLEEP(w_data, !!sleep);
  LOG(2, "rtc_set_sleep(%d) -> w_data: 0x%02X\n", sleep, w_data);
  system_rtc_mem_write(MAGIC_BLK, &w_data, sizeof(w_data));
}

bool rtc_get_dst() {
  u32 w_data = 0;
  system_rtc_mem_read(MAGIC_BLK, &w_data, sizeof(w_data));
  LOG(2, "rtc_get_dst -> w_data: 0x%02X\n", w_data);
   
  return MAGIC_GET_DST(w_data);
}

void rtc_set_dst(bool dst) {
  u32 w_data = 0;
  system_rtc_mem_read(MAGIC_BLK, &w_data, sizeof(w_data));
  w_data = MAGIC_SET_DST(w_data, !!dst);
  LOG(2, "rtc_set_dst(%d) -> w_data: 0x%02X\n", dst, w_data);
  system_rtc_mem_write(MAGIC_BLK, &w_data, sizeof(w_data));
}

bool rtc_get_init() {
  u32 w_data = 0;
  bool ret = false;
  
  system_rtc_mem_read(MAGIC_BLK, &w_data, sizeof(w_data));
  LOG(2, "rtc_get_init -> w_data: 0x%02X\n", w_data);
  
  return MAGIC_GET_INIT(w_data);
}

void rtc_set_init(bool ini) {
  u32 w_data = 0;
  
  system_rtc_mem_read(MAGIC_BLK, &w_data, sizeof(w_data));
  w_data = MAGIC_SET_INIT(w_data, !!ini);
  LOG(2, "rtc_set_init -> w_data: 0x%02X\n", w_data);
  system_rtc_mem_write(MAGIC_BLK, &w_data, sizeof(w_data));

  if (g_data.initialized != ini) {
    g_data.initialized = ini;
    g_data.dirty = true;
  }
}

// RUN_BLK
void rtc_get_lastRun(run_time_t &rt) {
  rt.hh = rtc_get_byte(RUN_BLK, 0);
  rt.mm = rtc_get_byte(RUN_BLK, 1);
}
void rtc_set_lastRun(const run_time_t &rt) {
  rtc_set_byte(RUN_BLK, 0, rt.hh);
  rtc_set_byte(RUN_BLK, 1, rt.mm);
}

//STATS_BLK
#define rtc_get_wakes() rtc_get_u16(STATS_BLK, 0)
#define rtc_set_wakes(v) rtc_set_u16(STATS_BLK, v, 0)
#define rtc_get_crashes() rtc_get_u16(STATS_BLK, 2)
#define rtc_set_crashes(v) rtc_set_u16(STATS_BLK, v, 2)

void load_flash(bool first_boot = false) {
  EEPROM.begin(sizeof(g_data));
  if (g_data.corrupted)
    return;
  EEPROM.get(0, g_data);
  LOG(1, "Data loaded from flash (saved size: %d, actual size:%d, id:0x%08X)\n", g_data.sz, sizeof(g_data), g_data.id);
  if (g_data.sz != sizeof(g_data) || g_data.id != DATA_CHKSUM) {
    LOG(1, "Flash data corrupted! Resetting data\n");
    memset(&g_data, 0, sizeof(g_data));
    g_data.corrupted = true;
    EEPROM.put(0, g_data);
    EEPROM.end();
    return;
  }
  g_data.dirty = false;
  if (first_boot) {
    //wifi_to_rtc();
    rtc_set_init(g_data.initialized);
    if (g_data.sleep_type == DEEP_SLEEP)
      rtc_set_sleep(true);
  }
  if (!g_data.sleep_type) {
    g_data.sleep_type = DEEP_SLEEP;
    g_data.dirty = true;
  }
}

void save_flash() {
  if (!g_data.dirty)
    return;
    
  g_data.sz = sizeof(g_data);
  g_data.corrupted = false;
  g_data.dirty = false;
  g_data.id = DATA_CHKSUM;
  
  EEPROM.put(0, g_data);
  EEPROM.commit();
  LOG(1, "Data saved to flash!\n");
}

bool wifi_connect() {
  if (!strlen(g_data.wifi_ssid) || !strlen(g_data.wifi_pass))
    load_flash();
  if (!strlen(g_data.wifi_ssid) || !strlen(g_data.wifi_pass))
    return false;

  LOG(1, "WiFi connect: SSID=%s PASS=%s\n", g_data.wifi_ssid, g_data.wifi_pass);
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
  WiFi.begin(g_data.wifi_ssid, g_data.wifi_pass);

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
      if (cal != g_data.calibration) {
        if (cal < 0 && abs(cal) < (GRT / 2))
          cal = - (GRT / 2);
        g_data.calibration = cal;
        g_data.dirty = true;
        msg += "<br>Saved feeder calibration: " + argV;
      }
    } else if (argN.startsWith("tm")) {
      u8 id = argN[argN.length() - 1] - '0';
      if (!id && !argV.length()) {
        // erase all jobs
        LOG(1, "Erasing %u jobs: total size: %u\n", sizeof(g_data.jobs) / sizeof(job_t), sizeof(g_data.jobs));
        memset(&g_data.jobs, 0, sizeof(g_data.jobs));
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
            g_data.jobs[id].hh = hh;
            g_data.jobs[id].mm = mm;
            g_data.dirty = true;
            msg += "<br>Saved job time: " + argV;
            LOG(1, "Saving job[%u] time: %u:%u\n", id, g_data.jobs[id].hh, g_data.jobs[id].mm); 
          }
        }
      }
    } else if (argN.startsWith("gr")) {
      u8 id = argN[argN.length() - 1] - '0';
      if (id < MAX_JOBS && id == lastJobId) {
       int val = argV.toInt();
        if (val > 255)
          val = 255;
        g_data.jobs[id].grams = val;
        g_data.dirty = true;
        msg += ", grams: " + String(val);
        LOG(1, "Saving job[%u] grams: %u\n", id, g_data.jobs[id].grams);
      }
    }
  }
  if (lastJobId != -1)
    for (int i = lastJobId + 1; i < MAX_JOBS; i++)
      g_data.jobs[i] = {0, 0, 0};

  if ((wifi_ssid == "" || wifi_pass == "") && !strlen(g_data.wifi_ssid)) {
    html = "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"></head><body>";
    html += "<h2 style=\"font-size: 1.4rem\">";
    html += "WiFi network name and/or pass not provided!";
    html += "</body></html>";
    request->send(406, "text/html", html);
    return;
  }

  if (wifi_ssid != String(g_data.wifi_ssid) ||
      wifi_pass != String(g_data.wifi_pass)) {
    memset(g_data.wifi_ssid, 0, sizeof(g_data.wifi_ssid));
    wifi_ssid.toCharArray(g_data.wifi_ssid, wifi_ssid.length() + 1);
    memset(g_data.wifi_pass, 0, sizeof(g_data.wifi_pass));
    wifi_pass.toCharArray(g_data.wifi_pass, wifi_pass.length() + 1);
    g_data.dirty = true;
    msg += "WiFi network saved as: " + wifi_ssid + "<br>";
    state.setState(WIFI_TEST);
  }
  
  if (sleep_type != 0 && sleep_type != g_data.sleep_type) {
    g_data.dirty = true;
    msg += "<br>Sleep Type saved as: ";
    switch (sleep_type) {
      case 1:
        g_data.sleep_type = DEEP_SLEEP;
        rtc_set_sleep(true);
        msg += "Deep Sleep";
        break;
      case 2:
        g_data.sleep_type = LIGHT_SLEEP;
        msg += "Light Sleep";
        break;
      case 3:
        g_data.sleep_type = ALWAYS_ON;        
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
  
  now = getCurrentTime();
  t = localtime(&now);
  
  html = "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<style>code { display: block;  font-family: monospace;  white-space: pre;  margin: 1em 0;}</style></head><body>";
  html += "<h2 style=\"font-size: 1.2rem\">";
  html += "<p class=\"time\">Date: " + String(ctime(&now)) + "</p>";
  sprintf(tmp, "<p>Uptime: %udays %02u:%02u:%02u </p>", dd, hh, mm, ss);
  html += String(tmp);
  sprintf(tmp, "<p>Since last boot: Wake-ups: %u | Crashes: %u</p>", wake_num, crash_num);
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
    ssid_num = WiFi.scanNetworks(false, false);
    
    if (ssid_num == 0) {
      LOG(1, "No networks found!\n");
      return false;
    } else if (ssid_num > 0) {
      LOG(1, "%d networks found\n", ssid_num);
      ssid_names = new String[ssid_num];
      for (int8_t i = 0; i < ssid_num; i++) {
        ssid_names[i] = WiFi.SSID(i);
        ssid_names[i].trim();
        if (ssid_names[i].length() > 2)
          LOG(1, "%02d: SSID: %s\n", i, ssid_names[i]);
      }
   
    } else {
      LOG(1, "WiFi scan error %d\n", ssid_num);
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
    request->send_P(200, "text/html", index_html, processor);
  });
  server.on("/config", handleConfig);
  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();
  LOG(1, "HTTP server started for: http://%s\n", host_name);
  
  return true;
}

#define FEED_25G MAP_GRAMS(25)
bool checkForJobs(u64 *timeTillNext = NULL) {   
  time_t now;
  struct tm *t;
  job_t *job = NULL;
  bool useNtp = (WiFi.status() == WL_CONNECTED);

  if (useNtp && (millis() - rtcMillis) < NTP_REFRESH) 
    useNtp = false;
    
  now = getCurrentTime(useNtp);
  if (!now) {
    state.setState(GET_NTP_TIME);
    return true;
  }

  if (timeTillNext)
    *timeTillNext = MAX_UINT;
  t = localtime(&now);
  run_time_t thisRun = { t->tm_hour, t->tm_min };
  LOG(3, "Last run: %02u:%02u, this run: %02u:%02u\n",
    lastRun.hh, lastRun.mm, thisRun.hh, thisRun.mm);
  for (u8 i = 0; i < MAX_JOBS; i++) {
    job_t *nextJob = &g_data.jobs[i];
    // Check for done jobs
    if (lastRun.hh && t->tm_hour <= lastRun.hh && t->tm_min <= lastRun.mm + 2)
      continue;
    LOG(3, "Checking job: %02u:%02u, grams: %u\n",
        nextJob->hh, nextJob->mm, nextJob->grams);
    if (!nextJob->grams || i == MAX_JOBS - 1) {
      if (lastRun.hh || lastRun.mm) {
        lastRun = {0, 0};
        LOG(1, "Resetting lastRun: %02u:%02u\n", lastRun.hh, lastRun.mm);
        rtc_set_lastRun(lastRun);
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
      LOG(3, "Time till this job: %us\n", secs);
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
  rtc_set_lastRun(lastRun);
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
  u64 sleepNow;
  //state.setState(SLEEPING);
  //return;
  if (g_data.sleep_type == LIGHT_SLEEP ||
      batState == BATTERY_LOW ||
      batState == BATTERY_VERY_LOW) {
    do_light_sleep();
    return;
  }

  if (g_data.sleep_type != DEEP_SLEEP)
    return;

  //u32 clk_per = system_rtc_clock_cali_proc();
  //u64 max_sleep = (u64)(MAX_UINT >> 12) * (u64)clk_per;
  //LOG(1, "Preparing sleep: cali_proc=%u, max_sleep=%llu\n", clk_per, max_sleep);
  //sleepNow = max_sleep;
  sleepNow = SLEEP_SECS;
  u64 timeTillNext;
  checkForJobs(&timeTillNext);
  LOG(1, "Time till next job: %llus, max_sleep: %llus\n", timeTillNext, sleepNow);
  if (timeTillNext != MAX_UINT && timeTillNext < sleepNow) {
    if (timeTillNext > 10)
      sleepNow = timeTillNext - 10;
     else
      sleepNow = timeTillNext;
  }

  if (sleepNow < 20) {
    if (!time_updated)
      state.setState(GET_NTP_TIME);
    else
      do_light_sleep();
    return;
  }

  sleepNow *= 1000000;
  // Connect D0 to RST to wake up
  pinMode(D0, WAKEUP_PULLUP);
  sleepRTC();
  delay(1);
  
  wifi_disconnect();
  unInstallButton(button);
  pinCtrl.stopLeds();

  save_flash();
  save_uptime(sleepNow);
  rtc_set_magic(MAGIC_ID);
  LOG(1, "Entering sleep for %llu seconds (%d)\n", (sleepNow / 1000000), wake_num);
  delay(10);
  ESP.deepSleep((sleepNow - 10000), WAKE_RF_DEFAULT);
}

ICACHE_RAM_ATTR void checkPIR() {
  LOG(1, "PIR event\n");
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
  LOG(3, "@%llu.%llus ADC value: %u, mV: %u (maxV: %u)\n", sec, msec, val, mV, (320 + r) * 10);
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
  digitalWrite(pin_ctrl_stat1, HIGH);
}

void reset_i2c() {
  LOG(2, "Resetting i2c lines\n");
  // Reset i2c lines
  digitalWrite(LOADCELL_DOUT_PIN, LOW);
  digitalWrite(LOADCELL_SCK_PIN, LOW);
  delay(1);
  i2cReset = true;
}

void setup() {
  u32 magic_id;
  bool btn_wake = false;

  pinMode(pin_ctrl_stat1, OUTPUT);
  pinMode(pin_ctrl_stat2, OUTPUT);
  pinMode(pin_ctrl_mux, OUTPUT);
  pinMode(pin_ctrl_servo, OUTPUT);
  pinMode(pin_led_status, OUTPUT);
  pinMode(pin_btn_control, INPUT);
  
  digitalWrite(pin_ctrl_mux, HIGH);
  digitalWrite(pin_ctrl_servo, LOW);
  btn_wake = (digitalRead(pin_btn_control) == LOW);
  
  // TODO: For debugging only, remove this delay
  //delay(3000);
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  LOG(1, "\nSetup\n");
  LOG(1, "Button wake: %u\n", btn_wake);

  /*
  LOG(1, "Waiting for probing:\n");
  int max_wait = 5;
  for (int i = 0; i < max_wait; i++) {
    LOG(1, "Seconds remaining: %d\n", max_wait - i);
    delay(1000);
  }
  */

  reset_i2c();
  //Probe HX711 load_cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  haveScale = scale.wait_ready_timeout(1000, 100);
  if (haveScale) {
    LOG(1, "Scale present!\n");
    //calibrate_load_cell();
  } else {
    LOG(1, "Scale absent!\n");
  }
  scale.power_down();
  
  reset_i2c();
  //Probe RTC
  haveRTC = rtc.begin();
  if (haveRTC) {
    LOG(1, "RTC present!\n");
    Ds1307SqwPinMode sq_pin = rtc.readSqwPinMode();
    LOG(2, "RTC control register: 0x%02X\n", (u8)sq_pin);
    i2cReset = false;
  } else {
    LOG(1, "RTC absent!\n");
  }

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
  magic_id = rtc_get_magic();
  crash_num = rtc_get_crashes();
  
  if (magic_id != MAGIC_ID) {
    if (magic_id == RUN_MAGIC_ID) {
      crash_num = rtc_get_crashes();
      LOG(1, "Oooups! Just had a crash...\n");
      crash_num++;
      rtc_set_crashes(crash_num);
      // Since we crashed, we need to start over with NTP time
      // Reset time only if we rely on NTP
      if (!haveRTC) {
        system_rtc_mem_write(RTC_BLK, &rtcTime, sizeof(rtcTime));
        system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
      }
    } else if (magic_id == OTA_MAGIC_ID) {
      LOG(1, "Restart after OTA\n");
    } else{
      LOG(1, "First boot... clearing regs!\n");
      magic_id = 0;
      first_boot = true;
      system_rtc_mem_write(RTC_BLK, &rtcTime, sizeof(rtcTime));
      system_rtc_mem_write(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
      for (u8 blk = MAGIC_BLK; blk >= STATS_BLK; blk--)
        system_rtc_mem_write(blk, &magic_id, sizeof(magic_id));
    }
  } else {
    system_rtc_mem_read(UPTIME_BLK, &rtcUptime, sizeof(rtcUptime));
    system_rtc_mem_read(RTC_BLK, &rtcTime, sizeof(rtcTime));
  }

  updateRTC();

  dst = !!rtc_get_dst();
  load_flash(first_boot);
  wake_num = rtc_get_wakes();
  if (magic_id == MAGIC_ID) {
    wake_num++;
    rtc_set_wakes(wake_num);
  }
  crash_num = rtc_get_crashes();
  rtc_get_lastRun(lastRun);
  rtc_set_magic(RUN_MAGIC_ID);

  // Only tested stall current for 0-40 / 140-180, so will use these by now
  if (SERVO_MOVE_FW > 100)
    servoStallCurrent = map(SERVO_MOVE_FW, 140, 180, 400, 700);
  else
    servoStallCurrent = map(SERVO_MOVE_FW, 0, 40, 700, 400);

  time_t now = getCurrentTime();
  LOG(1, "Wake-ups: %u\n", wake_num);
  LOG(1, "Crashes: %u\n", crash_num);
  LOG(1, "Initialized: %d\n", g_data.initialized);
  LOG(1, "DST: %u\n", dst);
  LOG(1, "NTP/RTC uptime: %llu\n", rtcUptime);
  LOG(1, "NTP/RTC time: %llu -> %s", rtcTime, ctime(&rtcTime));
  LOG(1, "Current time: %llu -> %s", now, ctime(&now));
  LOG(1, "Sleep type: %d\n", g_data.sleep_type);
  LOG(1, "Servo stall current: %u\n", servoStallCurrent);
  LOG(1, "Last Run: %02u:%02u\n", lastRun.hh, lastRun.mm);
  LOG(1, "WiFi config: SSID=%s PASS=%s\n", g_data.wifi_ssid, g_data.wifi_pass);
  for (int i = 0; i < MAX_JOBS; i++) {
    job_t *job = &g_data.jobs[i];
    if (job->grams)
      LOG(1, "Active job: %02u:%02u, grams: %u\n", job->hh, job->mm, job->grams);
  }

  timeClient.begin();
  timeClient.setTimeOffset(HOURS(2+dst));
  if (btn_wake) {
    LOG(1, "Button wake-up\n");
    state.setState({BTN_WAKE_UP, 500});
  } else {
    state.setState(RUN_IDLE);
  }

  if (g_data.corrupted || !strlen(g_data.wifi_ssid) || !strlen(g_data.wifi_pass))
    state.setState(CONFIGURE);
}

void loop() {
  pinCtrl.run();

  switch (state.getState()) {
    case SLEEPING:
      wifi_set_sleep_type(NONE_SLEEP_T);
      check_battery();
      if (checkForJobs())
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
      {
        u8 num_blinks = 1;
        u8 del = 100;
        if (batState == BATTERY_LOW) {
          num_blinks = 2;
        } else if (batState == BATTERY_VERY_LOW) {
          num_blinks = 4;
          del = 75;
        }
        for (u8 i = 0; i < num_blinks; i++) {
            digitalWrite(pin_led_status, HIGH);
            delay(del);
            digitalWrite(pin_led_status, LOW);
            delay(del);
          }
      }
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
        if ((rtcTime - rtc_updated) > HOURS(720)) {
          LOG(2, "RTC last updated %llus ago\n", rtcTime - rtc_updated);
          state.setState(GET_NTP_TIME);
          return;
        }
      }
      if (checkForJobs())
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
        scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
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
      lastRunTime = getCurrentTime();
      state.setState({RUN_SERVO, state.getWaitTime()});
      lastRunMillis = millis();
      servo.write(SERVO_MOVE_FW);
      return;
      
    case RUN_SERVO:
      {
        delay(10);
        u16 mA = read_adc();
        u64 currentMillis = millis();
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
        bool jobDone = state.expired();
        /*
        if (haveScale) {
          jobDone = false;
          if (scale.is_ready()) {
            u8 w = round(scale.get_units(5) + 0.49);
            jobDone = (w >= jobGrams);
            if (jobDone)
              LOG(1, "Weight achieved: %u\n", w);
          }
        }
        */
        if ((servo.read() == SERVO_MOVE_FW) && jobDone) {
          lastRunDuration = (currentMillis - lastRunMillis) - state.getExtraTime();
          LOG(1, "Stopping servo (state expired): total run: %ums\n", lastRunDuration);
          stopTrigger = JOB_TRIGGER_AUTO;
        } else if (!state.getWaitTime() && startTrigger == JOB_TRIGGER_BUTTON && getAction(button, BTN_ACT_RELEASED)) {
          lastRunDuration = (currentMillis - lastRunMillis) - state.getExtraTime();
          LOG(1, "Stopping servo (push button): total run: %ums\n", lastRunDuration);
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

        if (ssid_num > 0) {
          delete[] ssid_names;
          ssid_num = 0;
        }
        // since we are connected to wifi, update rtcTime
        //time_t now = getCurrentTime(true);
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
        time_t now = getCurrentTime();
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
      if (!getCurrentTime(true)) {
        time_t now = getCurrentTime();
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
        time_t now = getCurrentTime(true);
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
        rtc_set_magic(OTA_MAGIC_ID);
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
