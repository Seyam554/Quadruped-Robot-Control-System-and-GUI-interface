#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <DHT.h>

WiFiMulti  wifiMulti;
WiFiServer tcpServer(5555);
WiFiClient tcpClient;

// ===============================
// WIFI NETWORKS
// ===============================
struct WiFiNetwork { const char* ssid; const char* password; };

static const WiFiNetwork WIFI_NETWORKS[] = {
  { "Tazwar's HONOR X9b 5G",  "b3qivk5g_z37utr76"    },
  { "Error404",        "12345670"  },
  { "TP-Link_729E",     "27278178" },
  { "Seyam",        "b3qivk5g_z37utr76"     },
};
static const uint8_t WIFI_NETWORK_COUNT = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// ===============================
// INTERVALS (ms)
// ===============================
#define INTERVAL_GAS          200
#define INTERVAL_DHT         2000
#define INTERVAL_SERIAL      1000
#define INTERVAL_WIFI_CHECK 30000
#define WIFI_CONNECT_TIMEOUT_MS 15000

// ===============================
// GAS SENSOR
// ===============================
#define GAS_ANALOG  0
#define GAS_DIGITAL 3

// ===============================
// DHT11
// ===============================
#define DHTPIN  D2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ===============================
// LED (Active-Low on D6)
// +leg → 5V, -leg → D6
// D6 LOW  = LED ON  (voltage difference)
// D6 HIGH = LED OFF (no voltage difference)
// ===============================
#define LED_PIN   D6
static bool ledState = false;   // false = OFF (D6 HIGH), true = ON (D6 LOW)

static void setLED(bool on) {
  ledState = on;
  digitalWrite(LED_PIN, on ? LOW : HIGH);  // Active-low
}

static void toggleLED() {
  setLED(!ledState);
  Serial.printf("[LED]  %s\n", ledState ? "ON" : "OFF");
}

// ===============================
// TIMERS
// ===============================
static uint32_t tGas       = 0;
static uint32_t tDHT       = 0;
static uint32_t tSerial    = 0;
static uint32_t tWifiCheck = 0;

// ===============================
// CACHED VALUES
// ===============================
static int   gasRaw  = 0;
static float gasO2   = 20.9f, gasCO  = 0.0f,
             gasH2S  = 0.0f,  gasLEL = 0.0f;
static float dhtTempC = 0.0f, dhtTempF = 0.0f, dhtHum = 0.0f;
static bool  dhtOk        = false;
static bool  serverStarted = false;
static uint32_t tcpTxCount = 0;

// ===============================
// WIFI
// ===============================
static void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  Serial.println("[WiFi] Registering networks:");
  for (uint8_t i = 0; i < WIFI_NETWORK_COUNT; i++) {
    wifiMulti.addAP(WIFI_NETWORKS[i].ssid, WIFI_NETWORKS[i].password);
    Serial.printf("       [%d] %s\n", i + 1, WIFI_NETWORKS[i].ssid);
  }
}

static bool wifiConnect() {
  Serial.print("[WiFi] Connecting");
  uint32_t start = millis();
  while (millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    if (wifiMulti.run() == WL_CONNECTED) {
      Serial.println(" OK");
      Serial.printf("[WiFi] SSID : %s\n",     WiFi.SSID().c_str());
      Serial.printf("[WiFi] IP   : %s\n",     WiFi.localIP().toString().c_str());
      Serial.printf("[WiFi] RSSI : %d dBm\n", WiFi.RSSI());
      return true;
    }
    Serial.print(".");
    delay(300);
  }
  Serial.println(" FAILED");
  return false;
}

// ===============================
// TCP SERVER START
// ===============================
static void startTCPServer() {
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  serverStarted = true;
  Serial.printf("[TCP]  Listening on %s:5555\n",
    WiFi.localIP().toString().c_str());
}

// ===============================
// SENSOR TASKS
// ===============================
static void taskGas() {
  gasRaw  = analogRead(GAS_ANALOG);
  float m = constrain((float)gasRaw / 4095.0f, 0.0f, 1.0f);
  gasO2   = 20.9f - (m * 0.4f);
  gasCO   = m * 4.5f;
  gasH2S  = m * 0.012f;
  gasLEL  = m * 1.2f;
}

static void taskDHT() {
  dhtHum   = dht.readHumidity();
  dhtTempC = dht.readTemperature();
  dhtTempF = dht.readTemperature(true);
  dhtOk    = (!isnan(dhtHum) && !isnan(dhtTempC));
}

// ===============================
// JSON BUILDER
// ===============================
static void buildJSON(char* buf, size_t len) {
  snprintf(buf, len,
    "{"
      "\"uptime\":%lu,"
      "\"gas\":{"
        "\"raw\":%d,"
        "\"o2\":%.2f,"
        "\"co\":%.3f,"
        "\"h2s\":%.4f,"
        "\"lel\":%.3f"
      "},"
      "\"dht\":{"
        "\"ok\":%s,"
        "\"temp_c\":%.1f,"
        "\"temp_f\":%.1f,"
        "\"humidity\":%.1f"
      "},"
      "\"wifi\":{"
        "\"ssid\":\"%s\","
        "\"ip\":\"%s\","
        "\"rssi\":%d"
      "},"
      "\"led\":%s"
    "}\n",
    (unsigned long)(millis() / 1000),
    gasRaw, gasO2, gasCO, gasH2S, gasLEL,
    dhtOk ? "true" : "false",
    dhtOk ? dhtTempC : 0.0f,
    dhtOk ? dhtTempF : 0.0f,
    dhtOk ? dhtHum    : 0.0f,
    WiFi.SSID().c_str(),
    WiFi.localIP().toString().c_str(),
    WiFi.RSSI(),
    ledState ? "true" : "false"   // LED state included in JSON stream
  );
}

// ===============================
// TCP TASK
// ===============================
static void taskTCP() {
  if (!serverStarted) return;

  // Clean up dead client
  if (tcpClient && !tcpClient.connected()) {
    Serial.println("[TCP]  Client disconnected");
    tcpClient.stop();
  }

  // Accept new client
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient newClient = tcpServer.available();
    if (newClient) {
      tcpClient  = newClient;
      tcpTxCount = 0;
      Serial.printf("[TCP]  Client connected: %s\n",
        tcpClient.remoteIP().toString().c_str());
    }
  }

  // Read incoming commands from client ('L'/'l' = toggle LED)
  if (tcpClient && tcpClient.connected()) {
    while (tcpClient.available()) {
      char c = tcpClient.read();
      if (c == 'L' || c == 'l') toggleLED();
    }
  }

  // Send JSON
  if (tcpClient && tcpClient.connected()) {
    char json[512];
    buildJSON(json, sizeof(json));
    size_t written = tcpClient.print(json);
    if (written > 0) {
      tcpTxCount++;
    } else {
      Serial.println("[TCP]  Write failed — dropping client");
      tcpClient.stop();
    }
  }
}

// ===============================
// SERIAL — check for 'L' toggle
// ===============================
static void taskSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'L' || c == 'l') toggleLED();
  }
}

// ===============================
// SERIAL DASHBOARD
// ===============================
static void printSerial() {
  uint32_t s     = millis() / 1000;
  const char* dhtSt = dhtOk
    ? (dhtTempC > 30 ? "HOT" : dhtTempC < 18 ? "COLD" : "OK")
    : "ERR";
  const char* tcpSt = (tcpClient && tcpClient.connected())
    ? tcpClient.remoteIP().toString().c_str()
    : "none";

  Serial.println("+--------------------------------------------------+");
  Serial.printf( "| Uptime  : %02lu:%02lu:%02lu                               |\n",
    s/3600, (s%3600)/60, s%60);
  Serial.println("+--------------------------------------------------+");
  Serial.printf( "| GAS Raw : %-4d / 4095                            |\n", gasRaw);
  Serial.printf( "| O2      : %5.1f %%                                |\n", gasO2);
  Serial.printf( "| CO      : %7.3f ppm                             |\n", gasCO);
  Serial.printf( "| H2S     : %8.4f ppm                            |\n", gasH2S);
  Serial.printf( "| LEL     : %6.3f %%                               |\n", gasLEL);
  Serial.println("+--------------------------------------------------+");
  Serial.printf( "| Temp    : %5.1f C  /  %5.1f F   [%-4s]          |\n",
    dhtTempC, dhtTempF, dhtSt);
  Serial.printf( "| Humidity: %5.1f %%                                |\n", dhtHum);
  Serial.println("+--------------------------------------------------+");
  Serial.printf( "| SSID    : %-34s |\n", WiFi.SSID().c_str());
  Serial.printf( "| IP      : %-34s |\n", WiFi.localIP().toString().c_str());
  Serial.printf( "| RSSI    : %-4d dBm                               |\n", WiFi.RSSI());
  Serial.println("+--------------------------------------------------+");
  Serial.printf( "| TCP     : %-5s  port=5555  tx=%-6lu             |\n",
    serverStarted ? "UP" : "DOWN", tcpTxCount);
  Serial.printf( "| Client  : %-34s |\n", tcpSt);
  Serial.println("+--------------------------------------------------+");
  Serial.printf( "| LED     : D6 = %-3s  [Send 'L' to toggle]        |\n",
    ledState ? "ON " : "OFF");
  Serial.println("+--------------------------------------------------+\n");
}

// ===============================
// SETUP
// ===============================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========== ESP32-C6 BOOT ==========");

  // LED pin — start OFF (D6 HIGH = no voltage diff = LED OFF)
  pinMode(LED_PIN, OUTPUT);
  setLED(false);
  Serial.println("[LED]  D6 ready (active-low, OFF)");

  // Sensors
  dht.begin();
  Serial.println("[DHT]  DHT11 ready");
  pinMode(GAS_ANALOG,  INPUT);
  pinMode(GAS_DIGITAL, INPUT);
  taskDHT();
  taskGas();

  // WiFi → TCP
  wifiInit();
  if (wifiConnect()) {
    startTCPServer();
  } else {
    Serial.println("[TCP]  Skipped — no WiFi");
  }

  Serial.println("====================================");
  Serial.println("Send 'L' in Serial Monitor to toggle LED");
  Serial.println("====================================\n");

  uint32_t now = millis();
  tGas       = now;
  tDHT       = now + 10;
  tSerial    = now;
  tWifiCheck = now;
}

// ===============================
// LOOP
// ===============================
void loop() {
  uint32_t now = millis();

  taskSerial();               // Check for 'L' keypress — runs every loop

  if (now - tGas >= INTERVAL_GAS) {
    tGas = now;
    taskGas();
    taskTCP();
  }

  if (now - tDHT >= INTERVAL_DHT) {
    tDHT = now;
    taskDHT();
  }

  if (now - tSerial >= INTERVAL_SERIAL) {
    tSerial = now;
    printSerial();
  }

  if (now - tWifiCheck >= INTERVAL_WIFI_CHECK) {
    tWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Lost — reconnecting...");
      tcpClient.stop();
      serverStarted = false;
      if (wifiConnect()) startTCPServer();
    }
  }
}