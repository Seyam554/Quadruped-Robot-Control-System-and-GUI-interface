/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║      QUADRUPED ROBOT SYSTEM — WiFi Hotspot / Access Point        ║
 * ║      Target : ESP32-S3 DevKit C1 v1.1                            ║
 * ║      UDP    : Port 5555  (matches robot_gui.py)                  ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  Network: 192.168.181.0/24                                       ║
 * ║  AP IP  : 192.168.181.1                                          ║
 * ║                                                                  ║
 * ║  Expected device IPs (set static IP on each board):              ║
 * ║    Laptop            — DHCP (any .2 – .254)                      ║
 * ║    XIAO ESP32-C6     — 192.168.181.146  (Robot)                  ║
 * ║    XIAO ESP32-C6     — 192.168.181.10   (Air Quality Monitor)    ║
 * ║    XIAO ESP32-S3     — 192.168.181.175  (Camera)                 ║
 * ║                                                                  ║
 * ║  HOW IT WORKS:                                                   ║
 * ║  1. This ESP32-S3 acts as a WiFi access point.                   ║
 * ║  2. All devices (laptop + robot boards) connect to this AP.      ║
 * ║  3. Inter-device communication is enabled so the laptop can      ║
 * ║     reach the robot/AQM/camera directly via UDP 5555.            ║
 * ║  4. Serial monitor shows connected devices, RSSI, and signal%.   ║
 * ║  5. No changes needed to robot_gui.py.                           ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"

// ── ESP-IDF version detection ─────────────────────────────────────────────
#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR >= 5
  #include "esp_netif.h"
  #include "esp_wifi_ap_get_sta_list.h" 
  #define IDF5
#else
  #include "tcpip_adapter.h"
#endif

// ═════════════════════════════════════════════════════════════════════════
//  CONFIGURATION  — edit these to match your setup
// ═════════════════════════════════════════════════════════════════════════
#define AP_SSID         "Error404"     // WiFi name your devices connect to
#define AP_PASS         "12345670"          // Min 8 chars (WPA2)
#define AP_CHANNEL      6                   // WiFi channel (1-13)
#define AP_MAX_CLIENTS  8                   // Max simultaneous connections

#define UDP_PORT        5555               // Must match ROBOT_PORT in robot_gui.py
#define SCAN_PERIOD_MS  3000               // How often to refresh station list

// The dashboard will print exactly once every 5 seconds now
#define PRINT_PERIOD_MS 5000               

// AP addressing — subnet must match robot_gui.py default IPs (192.168.181.x)
const IPAddress AP_IP     (192, 168, 181,   1);
const IPAddress AP_GATEWAY(192, 168, 181,   1);
const IPAddress AP_SUBNET (255, 255, 255,   0);

// ═════════════════════════════════════════════════════════════════════════
//  KNOWN DEVICE REGISTRY
// ═════════════════════════════════════════════════════════════════════════
struct KnownDevice {
  uint8_t     mac[6];
  const char* name;
};

// ▼▼▼  FILL IN YOUR DEVICE MAC ADDRESSES BELOW  ▼▼▼
KnownDevice KNOWN_DEVICES[] = {
  {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, "ROBOT-XIAO-C6"},    // XIAO ESP32-C6 robot
  {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, "AQM-XIAO-C6"},      // XIAO ESP32-C6 air quality
  {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, "CAM-XIAO-S3"},      // XIAO ESP32-S3 camera
  {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, "LAPTOP"},           // Control laptop
};
// ▲▲▲  MAC addresses above — 0x00 entries are ignored  ▲▲▲

#define KNOWN_COUNT ((int)(sizeof(KNOWN_DEVICES) / sizeof(KNOWN_DEVICES[0])))

// ═════════════════════════════════════════════════════════════════════════
//  STATION TRACKING TABLE
// ═════════════════════════════════════════════════════════════════════════
struct Station {
  bool     valid;
  uint8_t  mac[6];
  char     ip[16];        
  char     name[28];
  int8_t   rssi;          
  uint32_t connectedAt;   
  uint32_t lastUdpMs;     
};

#define MAX_STA 8
Station    stations[MAX_STA];
int        stationCount = 0;

// IP→MAC cache populated by DHCP-assign events (survives scan rebuilds)
struct IpCache { char ip[16]; uint8_t mac[6]; bool valid; };
IpCache ipCache[MAX_STA];

// ═════════════════════════════════════════════════════════════════════════
//  UDP SOCKET (monitors port 5555 traffic destined for AP IP)
// ═════════════════════════════════════════════════════════════════════════
WiFiUDP  udpSock;
uint8_t  udpBuf[512];

// ═════════════════════════════════════════════════════════════════════════
//  HELPER FUNCTIONS
// ═════════════════════════════════════════════════════════════════════════

static void macToStr(const uint8_t* m, char* out) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
          m[0], m[1], m[2], m[3], m[4], m[5]);
}

static bool macEqual(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

static bool macAllZero(const uint8_t* m) {
  for (int i = 0; i < 6; i++) if (m[i]) return false;
  return true;
}

static const char* lookupName(const uint8_t* mac) {
  for (int i = 0; i < KNOWN_COUNT; i++) {
    if (macAllZero(KNOWN_DEVICES[i].mac)) continue;  
    if (macEqual(KNOWN_DEVICES[i].mac, mac)) return KNOWN_DEVICES[i].name;
  }
  return nullptr;
}

static int rssiToQuality(int8_t rssi) {
  if (rssi >= -50) return 100;
  if (rssi <= -100) return 0;
  return (rssi + 100) * 2;
}

static const char* qualityBar(int pct) {
  if (pct >= 80) return "████  EXCELLENT";
  if (pct >= 60) return "███░  GOOD     ";
  if (pct >= 40) return "██░░  FAIR     ";
  if (pct >= 20) return "█░░░  WEAK     ";
  return              "░░░░  POOR     ";
}

static const char* cachedIP(const uint8_t* mac) {
  for (int i = 0; i < MAX_STA; i++) {
    if (ipCache[i].valid && macEqual(ipCache[i].mac, mac)) return ipCache[i].ip;
  }
  return nullptr;
}

static void cacheIP(const uint8_t* mac, const char* ip) {
  for (int i = 0; i < MAX_STA; i++) {
    if (ipCache[i].valid && macEqual(ipCache[i].mac, mac)) {
      strncpy(ipCache[i].ip, ip, 15); ipCache[i].ip[15] = '\0';
      return;
    }
  }
  for (int i = 0; i < MAX_STA; i++) {
    if (!ipCache[i].valid) {
      memcpy(ipCache[i].mac, mac, 6);
      strncpy(ipCache[i].ip, ip, 15); ipCache[i].ip[15] = '\0';
      ipCache[i].valid = true;
      return;
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════
//  STATION LIST SCAN (called every SCAN_PERIOD_MS)
// ═════════════════════════════════════════════════════════════════════════
static void scanStations() {
  wifi_sta_list_t wsl;
  memset(&wsl, 0, sizeof(wsl));
  if (esp_wifi_ap_get_sta_list(&wsl) != ESP_OK) return;

  char ipStrBuf[MAX_STA][16];
  bool haveIPs = false;

#ifdef IDF5
  wifi_sta_mac_ip_list_t mac_ip_list;
  memset(&mac_ip_list, 0, sizeof(mac_ip_list));
  
  if (esp_wifi_ap_get_sta_list_with_ip(&wsl, &mac_ip_list) == ESP_OK) {
    haveIPs = true;
    for (int i = 0; i < mac_ip_list.num && i < MAX_STA; i++) {
      uint32_t addr = mac_ip_list.sta[i].ip.addr;
      snprintf(ipStrBuf[i], 16, "%u.%u.%u.%u",
               addr & 0xFF, (addr >> 8) & 0xFF,
               (addr >> 16) & 0xFF, (addr >> 24) & 0xFF);
      cacheIP(mac_ip_list.sta[i].mac, ipStrBuf[i]);
    }
  }
#else
  tcpip_adapter_sta_list_t tsl;
  memset(&tsl, 0, sizeof(tsl));
  if (tcpip_adapter_get_sta_list(&wsl, &tsl) == ESP_OK) {
    haveIPs = true;
    for (int i = 0; i < tsl.num && i < MAX_STA; i++) {
      uint32_t addr = tsl.sta[i].ip.addr;
      snprintf(ipStrBuf[i], 16, "%u.%u.%u.%u",
               addr & 0xFF, (addr >> 8) & 0xFF,
               (addr >> 16) & 0xFF, (addr >> 24) & 0xFF);
      cacheIP(tsl.sta[i].mac, ipStrBuf[i]);
    }
  }
#endif

  uint8_t  oldMac[MAX_STA][6];
  uint32_t oldUdp[MAX_STA];
  uint32_t oldConn[MAX_STA];
  int      oldCount = stationCount;
  for (int i = 0; i < oldCount; i++) {
    memcpy(oldMac[i], stations[i].mac, 6);
    oldUdp[i]  = stations[i].lastUdpMs;
    oldConn[i] = stations[i].connectedAt;
  }

  memset(stations, 0, sizeof(stations));
  stationCount = 0;

  for (int i = 0; i < wsl.num && stationCount < MAX_STA; i++) {
    Station& s = stations[stationCount];
    s.valid = true;
    memcpy(s.mac, wsl.sta[i].mac, 6);
    s.rssi = wsl.sta[i].rssi;

    const char* cached = cachedIP(s.mac);
    if (haveIPs && i < wsl.num) {
      strncpy(s.ip, ipStrBuf[i], 15);
    } else if (cached) {
      strncpy(s.ip, cached, 15);
    } else {
      strcpy(s.ip, "—.—.—.—");
    }
    s.ip[15] = '\0';

    const char* known = lookupName(s.mac);
    if (known) {
      strncpy(s.name, known, sizeof(s.name) - 1);
    } else {
      snprintf(s.name, sizeof(s.name), "DEVICE-%d", stationCount + 1);
    }
    s.name[sizeof(s.name) - 1] = '\0';

    s.connectedAt = millis();  
    s.lastUdpMs   = 0;
    for (int j = 0; j < oldCount; j++) {
      if (macEqual(s.mac, oldMac[j])) {
        s.lastUdpMs   = oldUdp[j];
        s.connectedAt = oldConn[j];
        break;
      }
    }

    stationCount++;
  }
}

// ═════════════════════════════════════════════════════════════════════════
//  SERIAL DASHBOARD
// ═════════════════════════════════════════════════════════════════════════
static void printDashboard() {
  unsigned long now = millis();

  // Print a batch of blank lines to push the old text off the screen.
  // This simulates a "clear screen" in the Arduino Serial Monitor.
  for (int i = 0; i < 40; i++) {
    Serial.println();
  }

  Serial.println(F("╔══════════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║        QUADRUPED HOTSPOT — DEVICE DASHBOARD                          ║"));
  Serial.println(F("╠══════════════════════════════════════════════════════════════════════╣"));
  Serial.printf( "║  AP: %-15s  SSID: %-20s  CH: %2d       ║\n",
                  WiFi.softAPIP().toString().c_str(), AP_SSID, AP_CHANNEL);
  Serial.printf( "║  Stations: %d/%d  |  Uptime: %lu s%-37s║\n",
                  stationCount, AP_MAX_CLIENTS, now / 1000, "");
  Serial.println(F("╠════════════════════╦═════════════════╦═══════════════════╦═══════════╣"));
  Serial.println(F("║  NAME              ║  IP ADDRESS     ║  RSSI / SIGNAL    ║  ACTIVE   ║"));
  Serial.println(F("╠════════════════════╬═════════════════╬═══════════════════╬═══════════╣"));

  if (stationCount == 0) {
    Serial.println(F("║  (no devices connected — waiting...)                                 ║"));
  } else {
    for (int i = 0; i < stationCount; i++) {
      Station& s = stations[i];
      int  q    = rssiToQuality(s.rssi);
      char macStr[18];
      macToStr(s.mac, macStr);

      char activity[12];
      if (s.lastUdpMs == 0) {
        strcpy(activity, "  idle   ");
      } else {
        unsigned long age = (now - s.lastUdpMs) / 1000;
        if (age < 5)        strcpy(activity, "  ACTIVE ");
        else if (age < 30)  snprintf(activity, sizeof(activity), " %3lus ago ", age);
        else                strcpy(activity, "  quiet  ");
      }

      unsigned long connSec = (now - s.connectedAt) / 1000;
      char connStr[12];
      if (connSec < 60)          snprintf(connStr, sizeof(connStr), "%3lus", connSec);
      else if (connSec < 3600)   snprintf(connStr, sizeof(connStr), "%2lum%02lus", connSec/60, connSec%60);
      else                       snprintf(connStr, sizeof(connStr), "%luh%02lum", connSec/3600, (connSec%3600)/60);

      Serial.printf("║  %-18s║  %-15s║ %4ddBm  %s ║  %-7s║\n",
                    s.name, s.ip, s.rssi, qualityBar(q), activity);
      Serial.printf("║  MAC: %-14s                                               ║\n", macStr);

      if (i < stationCount - 1)
        Serial.println(F("╠════════════════════╬═════════════════╬═══════════════════╬═══════════╣"));
    }
  }
  Serial.println(F("╚════════════════════╩═════════════════╩═══════════════════╩═══════════╝"));
  Serial.printf("[TIP] Copy unknown MAC addresses above -> paste into KNOWN_DEVICES[] in the sketch.\n");
}

// ═════════════════════════════════════════════════════════════════════════
//  EVENT HANDLERS
// ═════════════════════════════════════════════════════════════════════════
void onStaConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  char macStr[18];
  macToStr(info.wifi_ap_staconnected.mac, macStr);
  int aid = info.wifi_ap_staconnected.aid;
  const char* name = lookupName(info.wifi_ap_staconnected.mac);

  Serial.println();
  Serial.println(F("  ┌── DEVICE CONNECTED ──────────────────────────────────┐"));
  Serial.printf ("  │  MAC : %s  AID: %d\n", macStr, aid);
  Serial.printf ("  │  Name: %s\n", name ? name : "(unknown — add to KNOWN_DEVICES[])");
  Serial.println(F("  └──────────────────────────────────────────────────────┘"));
}

void onStaDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  char macStr[18];
  macToStr(info.wifi_ap_stadisconnected.mac, macStr);
  const char* name = lookupName(info.wifi_ap_stadisconnected.mac);

  for (int i = 0; i < MAX_STA; i++) {
    if (ipCache[i].valid && macEqual(ipCache[i].mac, info.wifi_ap_stadisconnected.mac)) {
      ipCache[i].valid = false;
      break;
    }
  }

  Serial.println();
  Serial.println(F("  ┌── DEVICE DISCONNECTED ───────────────────────────────┐"));
  Serial.printf ("  │  MAC : %s\n", macStr);
  Serial.printf ("  │  Name: %s\n", name ? name : "(unknown)");
  Serial.println(F("  └──────────────────────────────────────────────────────┘"));
}

// ═════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println(F("\n"));
  Serial.println(F("  ╔════════════════════════════════════════════╗"));
  Serial.println(F("  ║   QUADRUPED HOTSPOT  —  Starting up...     ║"));
  Serial.println(F("  ╚════════════════════════════════════════════╝\n"));

  memset(stations, 0, sizeof(stations));
  memset(ipCache,  0, sizeof(ipCache));

  WiFi.onEvent(onStaConnected,    ARDUINO_EVENT_WIFI_AP_STACONNECTED);
  WiFi.onEvent(onStaDisconnected, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

  WiFi.mode(WIFI_AP);

  if (!WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET)) {
    Serial.println(F("[ERROR] softAPConfig failed — check IP settings."));
  }

  bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);
  if (!ok) {
    Serial.println(F("[ERROR] softAP failed to start!"));
  } else {
    Serial.println(F("[OK] Access Point started."));
  }

  delay(200);  

  Serial.println(F("[OK] Inter-STA communication: ON (handled by default in SoftAP)"));

  Serial.println();
  Serial.printf("  WiFi Name (SSID) : %s\n",   AP_SSID);
  Serial.printf("  WiFi Password    : %s\n",   AP_PASS);
  Serial.printf("  AP IP Address    : %s\n",   WiFi.softAPIP().toString().c_str());
  Serial.printf("  Subnet           : 255.255.255.0\n");
  Serial.printf("  Channel          : %d\n",   AP_CHANNEL);
  Serial.printf("  Max Clients      : %d\n\n", AP_MAX_CLIENTS);

  Serial.println(F("  ── Device IPs expected by robot_gui.py ─────────────────────"));
  Serial.println(F("     Robot  XIAO ESP32-C6 : 192.168.181.146  (set static in firmware)"));
  Serial.println(F("     AQM    XIAO ESP32-C6 : 192.168.181.10   (set static in firmware)"));
  Serial.println(F("     Camera XIAO ESP32-S3 : 192.168.181.175  (set static in firmware)"));
  Serial.println(F("     Laptop               : DHCP (auto)"));
  Serial.println(F("  ──────────────────────────────────────────────────────────────\n"));

  udpSock.begin(UDP_PORT);
  Serial.printf("[OK] UDP monitor open on port %d (AP IP traffic only)\n\n", UDP_PORT);

  Serial.println(F("Connect your devices to the WiFi network above, then"));
  Serial.println(F("launch robot_gui.py on the laptop.\n"));
  Serial.println(F("Dashboard updates every 5 seconds.\n"));
}

// ═════════════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════════════
static unsigned long lastScan  = 0;
static unsigned long lastPrint = 0;

void loop() {
  unsigned long now = millis();

  // ── UDP monitor
  int plen = udpSock.parsePacket();
  if (plen > 0) {
    IPAddress src = udpSock.remoteIP();
    int       rlen = udpSock.read(udpBuf, sizeof(udpBuf) - 1);
    if (rlen > 0) {
      udpBuf[rlen] = '\0';

      char srcStr[16];
      snprintf(srcStr, sizeof(srcStr), "%d.%d.%d.%d", src[0], src[1], src[2], src[3]);
      for (int i = 0; i < stationCount; i++) {
        if (strcmp(stations[i].ip, srcStr) == 0) {
          stations[i].lastUdpMs = now;
          break;
        }
      }
    }
  }

  // ── Periodic station scan
  // Fixed logic: check elapsed time since lastScan
  if (now - lastScan >= SCAN_PERIOD_MS || lastScan == 0) {
    lastScan = now;
    scanStations();
  }

  // ── Periodic dashboard print
  // Fixed logic: check elapsed time since lastPrint
  if (now - lastPrint >= PRINT_PERIOD_MS || lastPrint == 0) {
    lastPrint = now;
    printDashboard();
  }
}