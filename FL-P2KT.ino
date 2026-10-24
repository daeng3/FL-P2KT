/*
 * ================================================================
 * Lmag-S Electromagnetic Flowmeter Logger - ESP32D / ESP32-WROOM-32D
 * Modbus RTU RS485 (FC03) + MQTT + NTP + Web Dashboard + OTA
 *
 * Register/protocol source: Lmag-S Electromagnetic Flowmeter
 * MODBUS RTU Protocol V1.8.3.2 (body labels are V1.8.0).
 *
 * MAIN FEATURES
 *  - Lmag-S main data block: holding registers 0x1010..0x1022 (19 words)
 *  - Optional auxiliary data: 0x1023..0x1027 (temperature/pressure, battery code, temperature)
 *  - Net total calculated as forward total minus reverse total
 *  - Byte ordering based on examples in the Lmag-S protocol manual
 *  - RS485 turnaround fix: RX enabled immediately; 4-character RTU quiet time before TX
 *  - One-combination-per-step Modbus auto scan (preferred ID/baud first)
 *  - MQTT JSON publishing, retained online/offline status, command subscription
 *  - NTP timestamp (WIB / UTC+7)
 *  - Preferences/NVS configuration, editable from authenticated web page
 *  - Authenticated web dashboard, JSON endpoint, debug log, manual firmware upload
 *  - GitHub/HTTPS OTA and periodic version check via raw GitHub URLs
 *  - Battery voltage ADC measurement (calibration factor is hardware-dependent)
 *  - Task watchdog, reconnection handling, rescan after consecutive errors
 *
 * LIBRARIES
 *  - ModbusMaster
 *  - PubSubClient
 *  - ESP32 Arduino core (WiFi, WebServer, Update, Preferences, HTTPUpdate)
 *
 * ESP32-WROOM-32D DEFAULT WIRING
 *  - MAX485 DE and /RE tied together -> GPIO4 (RS485_DIR_PIN)
 *  - MAX485 RO -> GPIO16 (RS485_RX_PIN / RX2)
 *  - MAX485 DI <- GPIO17 (RS485_TX_PIN / TX2)
 *  - Battery divider output -> GPIO35 (optional; use suitable divider!)
 *  - On-board LED -> GPIO2 (may be active-low or unavailable on some boards)
 *
 * SAFETY NOTES
 *  - Change Wi-Fi and web credentials before deployment.
 *  - GitHub OTA expects FL-Turap.ino.bin and version.txt in the configured repository.
 *  - setInsecure() is used for convenient HTTPS setup. For production, use a
 *    trusted CA certificate or signed firmware verification.
 *  - Never connect a battery voltage directly to ESP32 GPIO35. Use a divider
 *    and calibrate BATTERY_CAL_FACTOR for the actual circuit.
 * ================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>

// ============================================================================
// 1. PROJECT SETTINGS - EDIT THESE VALUES
// ============================================================================

const uint8_t DEVICE_ID = 1;                 // Give every logger a unique ID
const char* DEVICE_NAME = "LmagS-Flowmeter-P2KT";
const uint16_t FIRMWARE_VERSION = 3;          // RS485 turnaround/inter-frame timing fix

// Network defaults. They can also be changed on the authenticated /config page.
const char* DEFAULT_WIFI_SSID = "hadahade";
const char* DEFAULT_WIFI_PASSWORD = "levion1234";
const char* DEFAULT_MQTT_HOST = "103.197.190.235";
const uint16_t DEFAULT_MQTT_PORT = 1883;
const char* DEFAULT_MQTT_USER = "";
const char* DEFAULT_MQTT_PASSWORD = "";

// Web authentication: change these before production deployment.
const char* WEB_USER = "admin";
const char* WEB_PASSWORD = "levion123";

// GitHub OTA: update these paths if the repository/filenames change.
// The supplied compile log shows the Arduino sketch is named FL-Turap.ino,
// therefore Arduino's compiled application binary should be FL-Turap.ino.bin.
// version.txt must contain the same version number as the published firmware.
const String repoRaw =
  "https://raw.githubusercontent.com/daeng3/FL-P2KT/refs/heads/main/";
const String firmwareURL = repoRaw + "FL-P2KT.ino.bin";
const String versionURL  = repoRaw + "version.txt";

// Time (WIB, UTC+7).
const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.nist.gov";
const long GMT_OFFSET_SEC = 7L * 3600L;
const int DAYLIGHT_OFFSET_SEC = 0;

// ============================================================================
// 2. ESP32D HARDWARE CONFIGURATION
// ============================================================================

#define RS485_DIR_PIN 4
#define RS485_RX_PIN  16       // UART2 RX (ESP32-WROOM-32D)
#define RS485_TX_PIN  17       // UART2 TX (ESP32-WROOM-32D)
#define BATTERY_PIN   35       // ADC1 input-only pin; optional voltage divider
#define LED_WIFI_PIN  2

const bool ENABLE_BATTERY_MONITOR = true;
// Example carried over from the reference logger; must be calibrated against
// a multimeter and the actual resistor divider before using these readings.
const float BATTERY_CAL_FACTOR = 25.91f;
const float ADC_REFERENCE_V = 3.30f;
const uint16_t ADC_MAX_COUNTS = 4095;

HardwareSerial RS485Serial(2);
ModbusMaster node;
WebServer server(80);
WiFiClient mqttNetworkClient;
PubSubClient mqttClient(mqttNetworkClient);
Preferences prefs;

// ============================================================================
// 3. Lmag-S MODBUS SETTINGS
// ============================================================================

const uint8_t DEFAULT_SLAVE_ID = 1;
const uint32_t DEFAULT_BAUD_RATE = 9600;
const bool AUTO_SCAN_ENABLED = false;
const uint8_t SCAN_MAX_ID = 10;               // Increase up to 247 if required
const uint8_t FAILS_BEFORE_RESCAN = 5;
const uint32_t MODBUS_READ_INTERVAL_MS = 2000;
const uint32_t MQTT_PUBLISH_INTERVAL_MS = 2000;
const uint32_t MQTT_RETRY_INTERVAL_MS = 5000;
const uint32_t WIFI_RETRY_INTERVAL_MS = 30000;
const uint32_t OTA_CHECK_INTERVAL_MS = 3600000UL;
const uint32_t SCAN_RETRY_COOLDOWN_MS = 30000UL;

// Register 4112 decimal = 0x1010. FC03 reads 19 registers through 0x1022.
const uint16_t REG_FLOW_BASE = 0x1010;
const uint16_t FLOW_REGISTER_COUNT = 19; // mandatory/core block only
const uint8_t IDX_FLOW_RATE = 0;
const uint8_t IDX_FORWARD_TOTAL_INT = 2;
const uint8_t IDX_FORWARD_TOTAL_FRAC = 4;
const uint8_t IDX_VELOCITY = 6;
const uint8_t IDX_FLOW_PERCENT = 8;
const uint8_t IDX_RESISTANCE = 10;
const uint8_t IDX_REVERSE_TOTAL_INT = 12;
const uint8_t IDX_REVERSE_TOTAL_FRAC = 14;
const uint8_t IDX_FLOW_UNIT = 16;
const uint8_t IDX_TOTAL_UNIT = 17;
const uint8_t IDX_ALARM = 18;

// Auxiliary values listed in the protocol register table after alarm status.
// Read separately so a device that does not expose optional values can still
// provide the main 19-register flow dataset.
const uint16_t REG_AUX_BASE = 0x1023;
const uint16_t AUX_REGISTER_COUNT = 5; // 0x1023..0x1027
const uint8_t IDX_AUX_TEMP_PRESSURE = 0; // float: registers 0x1023-0x1024
const uint8_t IDX_AUX_BATTERY_POWER = 2; // uchar: register 0x1025
const uint8_t IDX_AUX_TEMPERATURE = 3;   // float: registers 0x1026-0x1027

// Values documented for Lmag-S serial configuration register 0x35.
const uint32_t BAUD_OPTIONS[] = {9600, 19200, 4800, 2400, 1200, 14400, 28800, 38400};
const size_t BAUD_OPTION_COUNT = sizeof(BAUD_OPTIONS) / sizeof(BAUD_OPTIONS[0]);

// ============================================================================
// 4. APPLICATION CONFIGURATION AND STATE
// ============================================================================

struct AppConfig {
  String ssid;
  String wifiPassword;
  String mqttHost;
  uint16_t mqttPort;
  String mqttUser;
  String mqttPassword;
};
AppConfig cfg;

struct FlowData {
  float flowRate = 0.0f;
  float velocity = 0.0f;
  float flowPercent = 0.0f;
  float resistanceKOhm = 0.0f;
  double forwardTotal = 0.0;
  double reverseTotal = 0.0;
  double netTotal = 0.0;            // forwardTotal - reverseTotal
  float temperaturePressureRaw = NAN; // Protocol does not specify physical unit here
  uint8_t batteryPowerCode = 0;       // UCHAR/code as documented by Lmag-S
  float temperatureRaw = NAN;         // No unit inferred from the supplied manual
  bool auxDataValid = false;
  String auxError = "Belum dibaca";
  uint8_t flowUnit = 0;
  uint8_t totalUnit = 0;
  uint8_t alarm = 0;
  uint8_t slaveID = 0;
  uint32_t baudRate = DEFAULT_BAUD_RATE;
  bool valid = false;
  uint32_t lastUpdateMillis = 0;
  String lastError = "Belum ada pembacaan";
  uint32_t successCount = 0;
  uint32_t totalErrorCount = 0;
};
FlowData data;

struct AppStatus {
  bool wifiConnected = false;
  bool mqttConnected = false;
  bool modbusConnected = false;
  bool timeSynced = false;
  uint32_t mqttPublishCount = 0;
  uint32_t mqttErrorCount = 0;
  uint32_t lastMqttReconnect = 0;
  uint32_t lastWifiReconnect = 0;
  uint32_t lastMqttPublish = 0;
  String lastModbusError = "Belum ada";
};
AppStatus status;

String mqttTopicData;
String mqttTopicCommand;
String mqttTopicStatus;
String debugLog;

uint8_t activeSlaveID = DEFAULT_SLAVE_ID;
uint32_t activeBaud = DEFAULT_BAUD_RATE;
uint8_t consecutiveModbusFailures = 0;
bool flagRestart = false;
bool flagGithubUpdate = false;
bool flagRescan = false;
bool apStarted = false;

uint32_t lastModbusRead = 0;
uint32_t lastMqttPublishAttempt = 0;
uint32_t lastOtaCheck = 0;
uint32_t lastScanStepAt = 0;
uint32_t lastPublishedDataUpdate = UINT32_MAX;

// Auto-scan cursor. Each pass probes only one configuration to keep the loop
// alive and feed the watchdog; preferred ID/baud is tried first.
uint16_t scanCursor = 0;
uint32_t scanRetryAfter = 0;
uint32_t lastAuxWarningMillis = 0;

// ============================================================================
// 5. LOGGING AND SIMPLE UTILITIES
// ============================================================================

void addLog(const String& message) {
  String line = "[" + String(millis() / 1000UL) + "s] " + message;
  Serial.println(line);
  debugLog = line + "\n" + debugLog;
  if (debugLog.length() > 6000) debugLog.remove(6000);
}

String htmlEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); ++i) {
    char c = in[i];
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '\"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c; break;
    }
  }
  return out;
}

bool requireWebAuth() {
  if (server.authenticate(WEB_USER, WEB_PASSWORD)) return true;
  server.requestAuthentication();
  return false;
}

bool githubOTAConfigured() {
  return firmwareURL.length() > 0 && versionURL.length() > 0 &&
         firmwareURL.startsWith("https://") && versionURL.startsWith("https://") &&
         firmwareURL.indexOf("YOUR_") < 0 && versionURL.indexOf("YOUR_") < 0;
}

// ============================================================================
// 6. WATCHDOG - supports ESP-IDF 4.x and 5.x used by ESP32 Arduino packages
// ============================================================================

void setupWatchdog() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdtConfig = {};
  wdtConfig.timeout_ms = 30000;
  wdtConfig.idle_core_mask = (1U << portNUM_PROCESSORS) - 1U;
  wdtConfig.trigger_panic = true;
  esp_err_t initResult = esp_task_wdt_init(&wdtConfig);
#else
  esp_err_t initResult = esp_task_wdt_init(30, true);
#endif
  if (initResult != ESP_OK && initResult != ESP_ERR_INVALID_STATE) {
    addLog("Watchdog: init error=" + String((int)initResult));
  }
  esp_err_t addResult = esp_task_wdt_add(NULL);
  if (addResult != ESP_OK && addResult != ESP_ERR_INVALID_STATE) {
    addLog("Watchdog: task registration error=" + String((int)addResult));
  } else {
    addLog("Watchdog aktif (30 detik)");
  }
}

void feedWatchdog() {
  esp_task_wdt_reset();
}

// ============================================================================
// 7. NVS CONFIGURATION
// ============================================================================

void loadConfig() {
  prefs.begin("lmags", true);
  cfg.ssid = prefs.getString("ssid", DEFAULT_WIFI_SSID);
  cfg.wifiPassword = prefs.getString("wifiPass", DEFAULT_WIFI_PASSWORD);
  cfg.mqttHost = prefs.getString("mqttHost", DEFAULT_MQTT_HOST);
  cfg.mqttPort = (uint16_t)prefs.getUShort("mqttPort", DEFAULT_MQTT_PORT);
  cfg.mqttUser = prefs.getString("mqttUser", DEFAULT_MQTT_USER);
  cfg.mqttPassword = prefs.getString("mqttPass", DEFAULT_MQTT_PASSWORD);
  prefs.end();

  mqttTopicData = "sensor/lmag-s-" + String(DEVICE_ID);
  mqttTopicCommand = "flowmeter/cmd/" + String(DEVICE_ID);
  mqttTopicStatus = mqttTopicData + "/status";

  addLog("NVS: konfigurasi dimuat; broker=" + cfg.mqttHost + ":" + String(cfg.mqttPort));
}

void saveConfig() {
  prefs.begin("lmags", false);
  prefs.putString("ssid", cfg.ssid);
  prefs.putString("wifiPass", cfg.wifiPassword);
  prefs.putString("mqttHost", cfg.mqttHost);
  prefs.putUShort("mqttPort", cfg.mqttPort);
  prefs.putString("mqttUser", cfg.mqttUser);
  prefs.putString("mqttPass", cfg.mqttPassword);
  prefs.end();
  addLog("NVS: konfigurasi disimpan");
}

// ============================================================================
// 8. RS485 / MODBUS HELPERS
// ============================================================================

void preTransmission() {
  // Modbus RTU needs an idle interval between frames. Wait four 10-bit
  // character times before enabling the driver (safe for the active baud).
  const uint32_t baud = activeBaud ? activeBaud : DEFAULT_BAUD_RATE;
  const uint32_t quietUs = (40000000UL + baud - 1UL) / baud;
  delayMicroseconds(quietUs);

  // MAX485 DE and /RE are tied together: HIGH=TX, LOW=RX.
  digitalWrite(RS485_DIR_PIN, HIGH);
  delayMicroseconds(50);
}

void postTransmission() {
  // Return to receive mode immediately after ModbusMaster flushes UART TX.
  // Do not leave DE high during an extra delay: a fast slave response could
  // start while the ESP32 receiver is still disabled.
  digitalWrite(RS485_DIR_PIN, LOW);
  delayMicroseconds(50);
}

void beginModbus(uint32_t baud, uint8_t id) {
  RS485Serial.end();
  delay(20);
  RS485Serial.begin(baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  delay(20);
  node.begin(id, RS485Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  activeBaud = baud;
  activeSlaveID = id;
}

String modbusErrorText(uint8_t code) {
  switch (code) {
    case 0x00: return "OK";
    case 0x01: return "Illegal function";
    case 0x02: return "Illegal data address";
    case 0x03: return "Illegal data value";
    case 0x04: return "Slave device failure";
    case 0xE0: return "Invalid slave ID";
    case 0xE1: return "Invalid function response";
    case 0xE2: return "Response timeout";
    case 0xE3: return "Invalid CRC";
    default: return "Modbus error 0x" + String(code, HEX);
  }
}

/*
 * Lmag-S manual example (page 5): response data bytes 2C 52 1A 46 = 9876.54.
 * ModbusMaster yields r1=0x2C52 and r2=0x1A46. Required IEEE754 bits are
 * 0x461A522C on ESP32. Do not use a generic word-swap assumption here.
 */
float regToFloatInv(uint16_t r1, uint16_t r2) {
  uint32_t raw = ((uint32_t)(r2 & 0x00FF) << 24) |
                 ((uint32_t)((r2 >> 8) & 0x00FF) << 16) |
                 ((uint32_t)(r1 & 0x00FF) << 8) |
                 ((uint32_t)((r1 >> 8) & 0x00FF));
  float value = 0.0f;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

/* Manual example: bytes B1 68 DE 3A = 987654321; this function yields it. */
uint32_t regToLong(uint16_t r1, uint16_t r2) {
  return ((uint32_t)(r2 & 0x00FF) << 24) |
         ((uint32_t)((r2 >> 8) & 0x00FF) << 16) |
         ((uint32_t)(r1 & 0x00FF) << 8) |
         ((uint32_t)((r1 >> 8) & 0x00FF));
}

String flowUnitText(uint8_t value) {
  static const char* units[] = {
    "m3/s", "m3/min", "m3/h", "L/s", "L/min", "L/h",
    "USg/m", "USg/h", "ig/m", "ig/h", "t/s", "t/m", "t/h",
    "kg/s", "kg/min", "kg/h"
  };
  return value < 16 ? String(units[value]) : String("Unknown");
}

String totalUnitText(uint8_t value) {
  static const char* units[] = {"L", "m3", "USgal", "igal", "kg", "t"};
  return value < 6 ? String(units[value]) : String("Unknown");
}

String alarmText(uint8_t alarm) {
  if (alarm == 0) return "Normal";
  String out;
  if (alarm & 0x02) out += "Excitation alarm; ";
  if (alarm & 0x04) out += "Electrode alarm; ";
  if (alarm & 0x08) out += "Empty pipe alarm; ";
  if (alarm & 0x10) out += "High flow alarm; ";
  if (alarm & 0x20) out += "Low flow alarm; ";
  if (out.length() == 0) return "Unknown alarm bits";
  out.trim();
  if (out.endsWith(";")) out.remove(out.length() - 1);
  return out;
}

bool testSlave(uint8_t id, uint32_t baud) {
  beginModbus(baud, id);
  uint8_t result = node.readHoldingRegisters(REG_FLOW_BASE, 2);
  status.lastModbusError = modbusErrorText(result);
  if (result == node.ku8MBSuccess) {
    activeSlaveID = id;
    activeBaud = baud;
    status.modbusConnected = true;
    data.slaveID = id;
    data.baudRate = baud;
    consecutiveModbusFailures = 0;
    addLog("Modbus slave ditemukan: ID=" + String(id) + ", baud=" + String(baud));
    return true;
  }
  return false;
}

// One probe per call to avoid spending an entire watchdog window on a scan.
void scanOneCombination() {
  if (!AUTO_SCAN_ENABLED) {
    if (millis() >= scanRetryAfter) {
      scanRetryAfter = millis() + SCAN_RETRY_COOLDOWN_MS;
      if (!testSlave(DEFAULT_SLAVE_ID, DEFAULT_BAUD_RATE)) {
        addLog("Modbus: ID/baud manual belum merespons (" + status.lastModbusError + ")");
      }
    }
    return;
  }
  if (millis() < scanRetryAfter) return;

  // cursor 0: try the explicitly configured ID/baud first.
  if (scanCursor == 0) {
    scanCursor = 1;
    addLog("Modbus: mencoba konfigurasi utama ID=" + String(DEFAULT_SLAVE_ID) +
           ", baud=" + String(DEFAULT_BAUD_RATE));
    if (testSlave(DEFAULT_SLAVE_ID, DEFAULT_BAUD_RATE)) return;
    addLog("Modbus: konfigurasi utama gagal; melanjutkan auto-scan");
    return;
  }

  // Then enumerate the documented baud rates and addresses 1..SCAN_MAX_ID.
  const uint16_t totalCombos = (uint16_t)(BAUD_OPTION_COUNT * SCAN_MAX_ID);
  uint16_t index = scanCursor - 1;
  if (index >= totalCombos) {
    scanCursor = 0;
    scanRetryAfter = millis() + SCAN_RETRY_COOLDOWN_MS;
    addLog("Modbus: seluruh kombinasi scan belum berhasil; mencoba lagi setelah 30 detik");
    return;
  }
  scanCursor++;

  size_t baudIndex = index / SCAN_MAX_ID;
  uint8_t id = (uint8_t)((index % SCAN_MAX_ID) + 1);
  uint32_t baud = BAUD_OPTIONS[baudIndex];

  // Avoid repeating the preferred combination.
  if (baud == DEFAULT_BAUD_RATE && id == DEFAULT_SLAVE_ID) return;

  if (!testSlave(id, baud)) {
    Serial.printf("Scan: ID=%u baud=%lu -> %s\n", id, (unsigned long)baud,
                  status.lastModbusError.c_str());
  }
}

bool readFlowmeter() {
  // Main data block documented by Lmag-S: FC03, 19 registers, 0x1010..0x1022.
  const uint8_t result = node.readHoldingRegisters(REG_FLOW_BASE, FLOW_REGISTER_COUNT);
  if (result != node.ku8MBSuccess) {
    status.lastModbusError = modbusErrorText(result);
    data.lastError = status.lastModbusError;
    data.valid = false;
    return false;
  }

  // Decode all main values before issuing another Modbus request because the
  // ModbusMaster response buffer is reused by every request.
  const float flowRate = regToFloatInv(node.getResponseBuffer(IDX_FLOW_RATE),
                                       node.getResponseBuffer(IDX_FLOW_RATE + 1));
  const uint32_t forwardInt = regToLong(node.getResponseBuffer(IDX_FORWARD_TOTAL_INT),
                                        node.getResponseBuffer(IDX_FORWARD_TOTAL_INT + 1));
  const float forwardFrac = regToFloatInv(node.getResponseBuffer(IDX_FORWARD_TOTAL_FRAC),
                                          node.getResponseBuffer(IDX_FORWARD_TOTAL_FRAC + 1));
  const float velocity = regToFloatInv(node.getResponseBuffer(IDX_VELOCITY),
                                       node.getResponseBuffer(IDX_VELOCITY + 1));
  const float percent = regToFloatInv(node.getResponseBuffer(IDX_FLOW_PERCENT),
                                      node.getResponseBuffer(IDX_FLOW_PERCENT + 1));
  const float resistance = regToFloatInv(node.getResponseBuffer(IDX_RESISTANCE),
                                         node.getResponseBuffer(IDX_RESISTANCE + 1));
  const uint32_t reverseInt = regToLong(node.getResponseBuffer(IDX_REVERSE_TOTAL_INT),
                                        node.getResponseBuffer(IDX_REVERSE_TOTAL_INT + 1));
  const float reverseFrac = regToFloatInv(node.getResponseBuffer(IDX_REVERSE_TOTAL_FRAC),
                                          node.getResponseBuffer(IDX_REVERSE_TOTAL_FRAC + 1));
  const uint8_t flowUnit = (uint8_t)(node.getResponseBuffer(IDX_FLOW_UNIT) & 0x00FF);
  const uint8_t totalUnit = (uint8_t)(node.getResponseBuffer(IDX_TOTAL_UNIT) & 0x00FF);
  const uint8_t alarm = (uint8_t)(node.getResponseBuffer(IDX_ALARM) & 0x00FF);

  if (!isfinite(flowRate) || !isfinite(forwardFrac) || !isfinite(velocity) ||
      !isfinite(percent) || !isfinite(resistance) || !isfinite(reverseFrac)) {
    status.lastModbusError = "Main block contains NaN/Inf; check byte order and device response";
    data.lastError = status.lastModbusError;
    data.valid = false;
    return false;
  }

  // Optional/auxiliary data from the Lmag-S table:
  // 0x1023..0x1024 = Temperature/Pressure float;
  // 0x1025 = Battery Power UCHAR; 0x1026..0x1027 = Temperature float.
  // Failure here does not invalidate the main flow measurement.
  bool auxOk = false;
  float temperaturePressure = NAN;
  uint8_t batteryPower = 0;
  float temperature = NAN;
  String auxError;
  const uint8_t auxResult = node.readHoldingRegisters(REG_AUX_BASE, AUX_REGISTER_COUNT);
  if (auxResult == node.ku8MBSuccess) {
    temperaturePressure = regToFloatInv(
      node.getResponseBuffer(IDX_AUX_TEMP_PRESSURE),
      node.getResponseBuffer(IDX_AUX_TEMP_PRESSURE + 1));
    batteryPower = (uint8_t)(node.getResponseBuffer(IDX_AUX_BATTERY_POWER) & 0x00FF);
    temperature = regToFloatInv(
      node.getResponseBuffer(IDX_AUX_TEMPERATURE),
      node.getResponseBuffer(IDX_AUX_TEMPERATURE + 1));
    if (isfinite(temperaturePressure) && isfinite(temperature)) {
      auxOk = true;
    } else {
      auxError = "Auxiliary float is NaN/Inf";
    }
  } else {
    auxError = modbusErrorText(auxResult);
  }

  // Commit main data only after the entire required main block validates.
  data.flowRate = flowRate;
  data.forwardTotal = (double)forwardInt + (double)forwardFrac;
  data.reverseTotal = (double)reverseInt + (double)reverseFrac;
  data.netTotal = data.forwardTotal - data.reverseTotal;
  data.velocity = velocity;
  data.flowPercent = percent;
  data.resistanceKOhm = resistance;
  data.flowUnit = flowUnit;
  data.totalUnit = totalUnit;
  data.alarm = alarm;

  data.auxDataValid = auxOk;
  data.temperaturePressureRaw = auxOk ? temperaturePressure : NAN;
  data.batteryPowerCode = auxOk ? batteryPower : 0;
  data.temperatureRaw = auxOk ? temperature : NAN;
  data.auxError = auxOk ? String("") : auxError;
  if (!auxOk && (lastAuxWarningMillis == 0 ||
                 millis() - lastAuxWarningMillis >= 30000UL)) {
    lastAuxWarningMillis = millis();
    addLog("Data auxiliary tidak tersedia/invalid; data utama tetap valid: " + auxError);
  }

  data.slaveID = activeSlaveID;
  data.baudRate = activeBaud;
  data.lastUpdateMillis = millis();
  data.lastError = "";
  data.valid = true;
  data.successCount++;
  status.lastModbusError = "OK";
  status.modbusConnected = true;
  consecutiveModbusFailures = 0;
  return true;
}

// ============================================================================
// 9. BATTERY ADC
// ============================================================================

float readBatteryVoltage() {
  if (!ENABLE_BATTERY_MONITOR) return -1.0f;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 12; ++i) {
    sum += analogRead(BATTERY_PIN);
    delay(2);
  }
  const float avg = (float)sum / 12.0f;
  return (avg / (float)ADC_MAX_COUNTS) * ADC_REFERENCE_V * BATTERY_CAL_FACTOR;
}

// ============================================================================
// 10. WIFI, TIME, MQTT
// ============================================================================

void connectWiFi() {
  if (cfg.ssid.length() == 0 || cfg.ssid == "GANTI_SSID_WIFI") {
    addLog("Wi-Fi SSID belum dikonfigurasi; masuk mode AP setup");
    WiFi.mode(WIFI_AP_STA);
    if (!apStarted) {
      apStarted = WiFi.softAP("LMAG-S-SETUP", "LmagSetup123");
      addLog(String("AP setup: ") + (apStarted ? "aktif" : "gagal") + " (192.168.4.1)");
    }
    status.wifiConnected = false;
    return;
  }

  addLog("Wi-Fi: menyambung ke " + cfg.ssid);
  WiFi.mode(WIFI_AP_STA);

  // Nama perangkat yang tampil di router/mDNS. Hostname hanya boleh huruf,
  // angka, dan tanda hubung, jadi ganti spasi/underscore bila ada.
  String hostname = String(DEVICE_NAME) + "-" + String(DEVICE_ID);
  hostname.replace(' ', '-');
  hostname.replace('_', '-');
  WiFi.setHostname(hostname.c_str());

  WiFi.begin(cfg.ssid.c_str(), cfg.wifiPassword.c_str());
  uint8_t attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    feedWatchdog();
    delay(500);
    digitalWrite(LED_WIFI_PIN, !digitalRead(LED_WIFI_PIN));
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    status.wifiConnected = true;
    digitalWrite(LED_WIFI_PIN, HIGH);
    addLog("Wi-Fi terhubung; IP=" + WiFi.localIP().toString());
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);
    status.timeSynced = false;
  } else {
    status.wifiConnected = false;
    addLog("Wi-Fi gagal; mengaktifkan AP konfigurasi");
    if (!apStarted) {
      apStarted = WiFi.softAP("LMAG-S-SETUP", "LmagSetup123");
      addLog(String("AP setup: ") + (apStarted ? "aktif" : "gagal") + " (192.168.4.1)");
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  message.reserve(length + 1);
  for (unsigned int i = 0; i < length; ++i) message += (char)payload[i];
  message.trim();
  String normalized = message;
  normalized.toLowerCase();
  addLog("MQTT command [" + String(topic) + "]: " + normalized);

  if (normalized == "restart" || normalized == "reboot") {
    flagRestart = true;
  } else if (normalized == "update") {
    flagGithubUpdate = true;
  } else if (normalized == "scan" || normalized == "rescan") {
    flagRescan = true;
  } else if (normalized == "status") {
    // The next valid measurement will be published immediately by lowering the timer.
    lastMqttPublishAttempt = 0;
    lastPublishedDataUpdate = UINT32_MAX;
  }
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    status.mqttConnected = false;
    return;
  }

  mqttClient.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(5);
  mqttClient.setBufferSize(1024);

  String clientId = "LMAGS_" + String(DEVICE_ID) + "_" + String((uint32_t)ESP.getEfuseMac(), HEX);
  addLog("MQTT: menyambung ke " + cfg.mqttHost + ":" + String(cfg.mqttPort));

  bool connected = false;
  if (cfg.mqttUser.length() > 0) {
    connected = mqttClient.connect(clientId.c_str(), cfg.mqttUser.c_str(), cfg.mqttPassword.c_str(),
                                   mqttTopicStatus.c_str(), 1, true, "offline");
  } else {
    connected = mqttClient.connect(clientId.c_str(), mqttTopicStatus.c_str(), 1, true, "offline");
  }

  status.mqttConnected = connected;
  if (connected) {
    mqttClient.publish(mqttTopicStatus.c_str(), "online", true);
    mqttClient.subscribe(mqttTopicCommand.c_str(), 1);
    addLog("MQTT tersambung; data=" + mqttTopicData + ", command=" + mqttTopicCommand);
  } else {
    addLog("MQTT gagal, state=" + String(mqttClient.state()));
  }
}

bool publishFlowData() {
  if (!mqttClient.connected() || !data.valid) return false;

  const time_t epoch = time(nullptr);
  const long long timestamp = (epoch > 1000000000) ? (long long)epoch : 0LL;
  const float batteryV = readBatteryVoltage();
  const String fUnit = flowUnitText(data.flowUnit);
  const String tUnit = totalUnitText(data.totalUnit);
  const String alarm = alarmText(data.alarm);
  const String ip = WiFi.localIP().toString();

  char tempPressureJson[24];
  char batteryPowerJson[12];
  char temperatureJson[24];
  if (data.auxDataValid) {
    snprintf(tempPressureJson, sizeof(tempPressureJson), "%.6f", data.temperaturePressureRaw);
    snprintf(batteryPowerJson, sizeof(batteryPowerJson), "%u", data.batteryPowerCode);
    snprintf(temperatureJson, sizeof(temperatureJson), "%.6f", data.temperatureRaw);
  } else {
    strcpy(tempPressureJson, "null");
    strcpy(batteryPowerJson, "null");
    strcpy(temperatureJson, "null");
  }

  char payload[1024];
  snprintf(payload, sizeof(payload),
    "{\"device\":\"%s\",\"device_id\":%u,\"ts\":%lld,\"time_synced\":%s,"
    "\"uptime_ms\":%lu,\"flow_rate\":%.6f,\"flow_unit\":\"%s\","
    "\"velocity_ms\":%.6f,\"flow_percent\":%.3f,\"forward_total\":%.6f,"
    "\"reverse_total\":%.6f,\"net_total\":%.6f,\"total_unit\":\"%s\","
    "\"resistance_kohm\":%.3f,\"temperature_pressure_raw\":%s,"
    "\"battery_power_code\":%s,\"temperature_raw\":%s,\"aux_data_valid\":%s,"
    "\"alarm\":\"%s\",\"alarm_code\":%u,\"slave_id\":%u,\"baud\":%lu,"
    "\"battery_v\":%.3f,\"fw_version\":%u,\"ip\":\"%s\"}",
    DEVICE_NAME, DEVICE_ID, timestamp, (timestamp ? "true" : "false"),
    (unsigned long)millis(), data.flowRate, fUnit.c_str(), data.velocity,
    data.flowPercent, data.forwardTotal, data.reverseTotal, data.netTotal, tUnit.c_str(),
    data.resistanceKOhm, tempPressureJson, batteryPowerJson, temperatureJson,
    data.auxDataValid ? "true" : "false", alarm.c_str(), data.alarm, data.slaveID,
    (unsigned long)data.baudRate, batteryV, FIRMWARE_VERSION, ip.c_str());

  if (mqttClient.publish(mqttTopicData.c_str(), payload, false)) {
    status.mqttPublishCount++;
    status.lastMqttPublish = millis();
    digitalWrite(LED_WIFI_PIN, LOW);
    delay(40);
    digitalWrite(LED_WIFI_PIN, HIGH);
    Serial.printf("MQTT published: %s\n", payload);
    return true;
  }

  status.mqttErrorCount++;
  addLog("MQTT: publish gagal");
  return false;
}

// ============================================================================
// 11. OPTIONAL GITHUB OTA
// ============================================================================

void performGithubUpdate() {
  if (!githubOTAConfigured()) {
    addLog("GitHub OTA belum aktif: periksa firmwareURL dan versionURL di kode");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    addLog("GitHub OTA dibatalkan: Wi-Fi tidak tersambung");
    return;
  }

  addLog("GitHub OTA: mulai mengunduh firmware");
  digitalWrite(LED_WIFI_PIN, LOW);
  // The firmware download can take longer than the watchdog limit. Unsubscribe
  // this task during the blocking update; successful OTA reboots automatically.
  esp_task_wdt_delete(NULL);

  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  secureClient.setTimeout(15000);
  httpUpdate.rebootOnUpdate(true);
  const t_httpUpdate_return result = httpUpdate.update(
    secureClient, firmwareURL, String(FIRMWARE_VERSION));

  if (result == HTTP_UPDATE_FAILED) {
    addLog("GitHub OTA gagal: " + httpUpdate.getLastErrorString());
  } else if (result == HTTP_UPDATE_NO_UPDATES) {
    addLog("GitHub OTA: tidak ada update baru");
  }
  // On failure/no update the device continues running.
  esp_task_wdt_add(NULL);
  digitalWrite(LED_WIFI_PIN, HIGH);
}

void checkGithubVersion() {
  if (!githubOTAConfigured() || WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  secureClient.setTimeout(10000);
  HTTPClient http;
  http.setTimeout(10000);
  if (!http.begin(secureClient, versionURL)) {
    addLog("OTA check: tidak dapat membuka version URL");
    return;
  }
  http.addHeader("Cache-Control", "no-cache");
  const int httpCode = http.GET();
  bool shouldUpdate = false;
  if (httpCode == HTTP_CODE_OK) {
    String versionText = http.getString();
    versionText.trim();
    const long remoteVersion = versionText.toInt();
    addLog("OTA check: remote=" + String(remoteVersion) + ", local=" + String(FIRMWARE_VERSION));
    if (remoteVersion > FIRMWARE_VERSION) {
      addLog("OTA check: versi baru ditemukan");
      shouldUpdate = true;
    }
  } else {
    addLog("OTA check: HTTP " + String(httpCode));
  }
  http.end();
  if (shouldUpdate) performGithubUpdate();
}

// ============================================================================
// 12. WEB PAGES / API / MANUAL OTA
// ============================================================================

String getDashboardHTML() {
  String html;
  html.reserve(7000);
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='5'><title>Lmag-S Logger</title><style>";
  html += "body{font-family:Arial,sans-serif;background:#0f172a;color:#e2e8f0;padding:12px;margin:0}";
  html += "h2{color:#38bdf8}.card{background:#1e293b;padding:14px;margin:10px 0;border-radius:8px}";
  html += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}";
  html += ".v{font-size:20px;color:#38bdf8;font-weight:bold;overflow-wrap:anywhere}.l{font-size:12px;color:#94a3b8}";
  html += ".ok{color:#34d399}.bad{color:#f87171}.btn{display:block;background:#2563eb;color:white;padding:11px;margin:8px 0;text-decoration:none;border-radius:5px;text-align:center}";
  html += "input{padding:9px;max-width:100%}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}";
  html += "</style></head><body><h2>" + String(DEVICE_NAME) + "</h2>";
  html += "<div class='card'><b>Status</b><div class='grid'>";
  html += "<div class='l'>Wi-Fi<br><span class='" + String(status.wifiConnected ? "ok" : "bad") + "'>" + String(status.wifiConnected ? "CONNECTED" : "DISCONNECTED") + "</span></div>";
  html += "<div class='l'>MQTT<br><span class='" + String(status.mqttConnected ? "ok" : "bad") + "'>" + String(status.mqttConnected ? "CONNECTED" : "DISCONNECTED") + "</span></div>";
  html += "<div class='l'>Modbus<br><span class='" + String(status.modbusConnected ? "ok" : "bad") + "'>" + String(status.modbusConnected ? "CONNECTED" : "DISCONNECTED") + "</span></div>";
  html += "<div class='l'>NTP<br><span class='" + String(status.timeSynced ? "ok" : "bad") + "'>" + String(status.timeSynced ? "SYNCED" : "WAITING") + "</span></div>";
  html += "</div></div>";

  html += "<div class='card'><b>Data Flowmeter</b>";
  if (data.valid) {
    html += "<div class='grid'>";
    html += "<div><div class='l'>Flow rate</div><div class='v'>" + String(data.flowRate, 3) + " " + flowUnitText(data.flowUnit) + "</div></div>";
    html += "<div><div class='l'>Velocity</div><div class='v'>" + String(data.velocity, 3) + " m/s</div></div>";
    html += "<div><div class='l'>Flow percentage</div><div class='v'>" + String(data.flowPercent, 2) + " %</div></div>";
    html += "<div><div class='l'>Resistance</div><div class='v'>" + String(data.resistanceKOhm, 2) + " kOhm</div></div>";
    html += "<div><div class='l'>Forward total</div><div class='v'>" + String(data.forwardTotal, 3) + " " + totalUnitText(data.totalUnit) + "</div></div>";
    html += "<div><div class='l'>Reverse total</div><div class='v'>" + String(data.reverseTotal, 3) + " " + totalUnitText(data.totalUnit) + "</div></div>";
    html += "<div><div class='l'>Net total (forward - reverse)</div><div class='v'>" + String(data.netTotal, 3) + " " + totalUnitText(data.totalUnit) + "</div></div>";
    if (data.auxDataValid) {
      html += "<div><div class='l'>Temperature/Pressure (raw)</div><div class='v'>" + String(data.temperaturePressureRaw, 3) + "</div></div>";
      html += "<div><div class='l'>Battery power code</div><div class='v'>" + String(data.batteryPowerCode) + "</div></div>";
      html += "<div><div class='l'>Temperature (raw)</div><div class='v'>" + String(data.temperatureRaw, 3) + "</div></div>";
    } else {
      html += "<div><div class='l'>Auxiliary data</div><div class='v bad'>Unavailable</div></div>";
    }
    html += "</div><p>Alarm: <b class='" + String(data.alarm == 0 ? "ok" : "bad") + "'>" + alarmText(data.alarm) + " (0x" + String(data.alarm, HEX) + ")</b></p>";
    if (!data.auxDataValid && data.auxError.length()) html += "<p class='l'>Auxiliary read: " + htmlEscape(data.auxError) + "</p>";
  } else {
    html += "<p class='bad'>Menunggu data valid.</p>";
  }
  html += "</div>";

  html += "<div class='card'><b>Diagnostik</b><p class='l'>Slave ID: " + String(activeSlaveID) +
          " | Baud: " + String(activeBaud) + " | Serial: 8N1</p>";
  html += "<p class='l'>Read OK: " + String(data.successCount) + " | Error: " + String(data.totalErrorCount) +
          " | Consecutive: " + String(consecutiveModbusFailures) + "</p>";
  html += "<p class='l'>MQTT topic: " + mqttTopicData + "</p><p class='l'>Published: " +
          String(status.mqttPublishCount) + " | Publish errors: " + String(status.mqttErrorCount) + "</p>";
  html += "<p class='l'>Battery: " + String(ENABLE_BATTERY_MONITOR ? String(readBatteryVoltage(), 2) + " V (estimate)" : String("disabled")) +
          " | Uptime: " + String(millis() / 1000UL) + " s</p>";
  html += "<p class='l'>Last Modbus: " + htmlEscape(status.lastModbusError) + "</p>";
  if (data.lastError.length()) html += "<p class='bad'>Last data error: " + htmlEscape(data.lastError) + "</p>";
  html += "</div>";

  html += "<div class='card'><b>Manual OTA Upload (.bin)</b><form method='POST' action='/update' enctype='multipart/form-data'>";
  html += "<input type='file' name='update' accept='.bin' required><br><br><input type='submit' value='Upload Firmware'></form>";
  html += "<p class='l'>Gunakan file firmware .bin yang dikompilasi untuk ESP32 yang sama.</p></div>";
  html += "<a class='btn' href='/config'>Konfigurasi Wi-Fi / MQTT</a>";
  html += "<a class='btn' href='/api'>Lihat JSON API</a>";
  html += "<a class='btn' href='/debug'>Debug log</a>";
  html += "<p class='l'>Firmware v" + String(FIRMWARE_VERSION) + " | IP " + WiFi.localIP().toString() + "</p></body></html>";
  return html;
}

String getConfigHTML() {
  String html;
  html.reserve(4000);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Konfigurasi Lmag-S</title><style>body{font-family:Arial;background:#0f172a;color:#eee;padding:16px}form{max-width:560px;margin:auto}label{display:block;margin-top:12px}input{box-sizing:border-box;width:100%;padding:10px;margin-top:4px}button,a{display:block;padding:12px;margin-top:14px;background:#2563eb;color:white;border:0;text-align:center;text-decoration:none;border-radius:5px}.note{color:#94a3b8;font-size:12px}</style></head><body>";
  html += "<h2>Konfigurasi Lmag-S Logger</h2><p class='note'>Perubahan disimpan di NVS lalu ESP32 restart. Password Wi-Fi yang kosong tidak mengubah password tersimpan.</p>";
  html += "<form method='POST' action='/config'>";
  html += "<label>Wi-Fi SSID<input name='ssid' value='" + htmlEscape(cfg.ssid) + "' required></label>";
  html += "<label>Password Wi-Fi (kosong = tidak diubah)<input name='wifiPass' type='password' value=''></label>";
  html += "<label>MQTT host / IP<input name='mqttHost' value='" + htmlEscape(cfg.mqttHost) + "' required></label>";
  html += "<label>MQTT port<input name='mqttPort' type='number' min='1' max='65535' value='" + String(cfg.mqttPort) + "' required></label>";
  html += "<label>MQTT username (boleh kosong)<input name='mqttUser' value='" + htmlEscape(cfg.mqttUser) + "'></label>";
  html += "<label>MQTT password (kosong = kosongkan autentikasi)<input name='mqttPass' type='password' value=''></label>";
  html += "<button type='submit'>Simpan dan restart</button></form><a href='/'>Kembali</a></body></html>";
  return html;
}

String getDebugHTML() {
  String html;
  html.reserve(debugLog.length() + 1000);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='4'><title>Debug Lmag-S</title><style>body{font-family:monospace;background:#000;color:#0f0;padding:10px}pre{white-space:pre-wrap;overflow-wrap:anywhere}a{color:#5af}</style></head><body>";
  html += "<a href='/'>Kembali ke dashboard</a><h3>Debug log (refresh 4 detik)</h3><pre>" + htmlEscape(debugLog) + "</pre></body></html>";
  return html;
}

void handleRoot() {
  if (!requireWebAuth()) return;
  server.send(200, "text/html; charset=utf-8", getDashboardHTML());
}

void handleConfigPage() {
  if (!requireWebAuth()) return;
  server.send(200, "text/html; charset=utf-8", getConfigHTML());
}

void handleSaveConfig() {
  if (!requireWebAuth()) return;

  String newSSID = server.arg("ssid");
  String newWifiPass = server.arg("wifiPass");
  String newHost = server.arg("mqttHost");
  long newPort = server.arg("mqttPort").toInt();
  if (newSSID.length() == 0 || newHost.length() == 0 || newPort < 1 || newPort > 65535) {
    server.send(400, "text/plain", "Konfigurasi tidak valid. Periksa SSID, host, dan port.");
    return;
  }

  cfg.ssid = newSSID;
  if (newWifiPass.length() > 0) cfg.wifiPassword = newWifiPass;
  cfg.mqttHost = newHost;
  cfg.mqttPort = (uint16_t)newPort;
  cfg.mqttUser = server.arg("mqttUser");
  cfg.mqttPassword = server.arg("mqttPass");
  saveConfig();
  server.send(200, "text/html; charset=utf-8",
              "<html><body><h3>Konfigurasi disimpan.</h3><p>ESP32 akan restart dalam beberapa saat.</p></body></html>");
  delay(500);
  ESP.restart();
}

void handleApi() {
  if (!requireWebAuth()) return;
  const time_t epoch = time(nullptr);
  char tempPressureJson[24];
  char batteryPowerJson[12];
  char temperatureJson[24];
  if (data.auxDataValid) {
    snprintf(tempPressureJson, sizeof(tempPressureJson), "%.6f", data.temperaturePressureRaw);
    snprintf(batteryPowerJson, sizeof(batteryPowerJson), "%u", data.batteryPowerCode);
    snprintf(temperatureJson, sizeof(temperatureJson), "%.6f", data.temperatureRaw);
  } else {
    strcpy(tempPressureJson, "null");
    strcpy(batteryPowerJson, "null");
    strcpy(temperatureJson, "null");
  }

  char json[1150];
  snprintf(json, sizeof(json),
    "{\"device\":\"%s\",\"device_id\":%u,\"ts\":%lld,\"valid\":%s,"
    "\"flow_rate\":%.6f,\"flow_unit\":\"%s\",\"velocity_ms\":%.6f,"
    "\"flow_percent\":%.3f,\"forward_total\":%.6f,\"reverse_total\":%.6f,"
    "\"net_total\":%.6f,\"total_unit\":\"%s\",\"resistance_kohm\":%.3f,"
    "\"temperature_pressure_raw\":%s,\"battery_power_code\":%s,"
    "\"temperature_raw\":%s,\"aux_data_valid\":%s,\"alarm_code\":%u,"
    "\"alarm\":\"%s\",\"slave_id\":%u,\"baud\":%lu,\"battery_v\":%.3f,"
    "\"fw_version\":%u,\"modbus_error\":\"%s\",\"aux_error\":\"%s\"}",
    DEVICE_NAME, DEVICE_ID, (long long)(epoch > 1000000000 ? epoch : 0), data.valid ? "true" : "false",
    data.flowRate, flowUnitText(data.flowUnit).c_str(), data.velocity, data.flowPercent,
    data.forwardTotal, data.reverseTotal, data.netTotal, totalUnitText(data.totalUnit).c_str(),
    data.resistanceKOhm, tempPressureJson, batteryPowerJson, temperatureJson,
    data.auxDataValid ? "true" : "false", data.alarm, alarmText(data.alarm).c_str(), data.slaveID,
    (unsigned long)data.baudRate, ENABLE_BATTERY_MONITOR ? readBatteryVoltage() : -1.0f,
    FIRMWARE_VERSION, status.lastModbusError.c_str(), data.auxError.c_str());
  server.send(200, "application/json; charset=utf-8", json);
}

void handleDebug() {
  if (!requireWebAuth()) return;
  server.send(200, "text/html; charset=utf-8", getDebugHTML());
}

void handleManualUpdatePost() {
  if (!requireWebAuth()) return;
  server.sendHeader("Connection", "close");
  if (Update.hasError()) {
    server.send(500, "text/plain", "FAIL: firmware update error");
  } else {
    server.send(200, "text/plain", "OK: firmware diterima, perangkat akan restart");
    delay(500);
    ESP.restart();
  }
}

void handleManualUpdateUpload() {
  if (!server.authenticate(WEB_USER, WEB_PASSWORD)) return;
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    addLog("Manual OTA: upload dimulai: " + upload.filename);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    const size_t written = Update.write(upload.buf, upload.currentSize);
    if (written != upload.currentSize) Update.printError(Serial);
    feedWatchdog();
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("Manual OTA berhasil: %u bytes\n", upload.totalSize);
      addLog("Manual OTA: upload selesai");
    } else {
      Update.printError(Serial);
      addLog("Manual OTA: update gagal");
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    addLog("Manual OTA: upload dibatalkan");
  }
}

// ============================================================================
// 13. SETUP AND MAIN LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW);
  pinMode(LED_WIFI_PIN, OUTPUT);
  digitalWrite(LED_WIFI_PIN, LOW);
  if (ENABLE_BATTERY_MONITOR) pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);

  setupWatchdog();
  addLog("Boot: Lmag-S logger ESP32D");
  addLog("Device=" + String(DEVICE_NAME) + ", firmware=" + String(FIRMWARE_VERSION));
  addLog("RS485 pins DIR=" + String(RS485_DIR_PIN) + " RX=" + String(RS485_RX_PIN) + " TX=" + String(RS485_TX_PIN));
  loadConfig();

  beginModbus(DEFAULT_BAUD_RATE, DEFAULT_SLAVE_ID);
  connectWiFi();

  mqttClient.setBufferSize(1024);
  mqttClient.setCallback(mqttCallback);
  if (WiFi.status() == WL_CONNECTED) connectMQTT();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/config", HTTP_GET, handleConfigPage);
  server.on("/config", HTTP_POST, handleSaveConfig);
  server.on("/api", HTTP_GET, handleApi);
  server.on("/debug", HTTP_GET, handleDebug);
  server.on("/update", HTTP_POST, handleManualUpdatePost, handleManualUpdateUpload);
  server.begin();

  if (WiFi.status() == WL_CONNECTED) {
    addLog("Dashboard: http://" + WiFi.localIP().toString());
  }
  if (apStarted) addLog("Setup AP: LMAG-S-SETUP / LmagSetup123; buka http://192.168.4.1");
  addLog("Web server siap. MQTT command: restart, update, scan, status");
}

void loop() {
  feedWatchdog();
  server.handleClient();

  // Wi-Fi state and reconnect.
  status.wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (!status.wifiConnected && millis() - status.lastWifiReconnect >= WIFI_RETRY_INTERVAL_MS) {
    status.lastWifiReconnect = millis();
    addLog("Wi-Fi: mencoba koneksi ulang");
    connectWiFi();
  }

  // NTP synchronization state.
  if (!status.timeSynced && time(nullptr) > 1000000000) {
    status.timeSynced = true;
    addLog("NTP: waktu tersinkronisasi; WIB");
  }

  // MQTT reconnect loop.
  if (WiFi.status() == WL_CONNECTED) {
    if (mqttClient.connected()) {
      mqttClient.loop();
      status.mqttConnected = true;
    } else {
      status.mqttConnected = false;
      if (millis() - status.lastMqttReconnect >= MQTT_RETRY_INTERVAL_MS) {
        status.lastMqttReconnect = millis();
        connectMQTT();
      }
    }
  } else {
    status.mqttConnected = false;
  }

  // Commands are handled outside MQTT callback to avoid long blocking work there.
  if (flagRestart) {
    flagRestart = false;
    addLog("Perintah restart diterima");
    delay(250);
    ESP.restart();
  }
  if (flagRescan) {
    flagRescan = false;
    addLog("Perintah rescan diterima");
    status.modbusConnected = false;
    data.valid = false;
    scanCursor = 0;
    scanRetryAfter = 0;
  }
  if (flagGithubUpdate) {
    flagGithubUpdate = false;
    performGithubUpdate();
  }

  // Scheduled OTA version check (enabled when the raw GitHub URLs are valid).
  if (millis() - lastOtaCheck >= OTA_CHECK_INTERVAL_MS) {
    lastOtaCheck = millis();
    checkGithubVersion();
  }

  // Poll Modbus. A failed read does not reuse old data as if it were current.
  if (millis() - lastModbusRead >= MODBUS_READ_INTERVAL_MS) {
    lastModbusRead = millis();
    if (!status.modbusConnected) {
      scanOneCombination();
      if (!status.modbusConnected) {
        data.totalErrorCount++;
      }
    } else {
      if (readFlowmeter()) {
        // Success counter is incremented in readFlowmeter().
      } else {
        data.totalErrorCount++;
        consecutiveModbusFailures++;
        addLog("Modbus read gagal: " + status.lastModbusError +
               " (consecutive=" + String(consecutiveModbusFailures) + ")");
        if (consecutiveModbusFailures >= FAILS_BEFORE_RESCAN) {
          status.modbusConnected = false;
          data.valid = false;
          scanCursor = 0;
          scanRetryAfter = 0;
          addLog("Modbus: terlalu banyak kegagalan; memulai scan ulang");
        }
      }
    }
  }

  // Publish only fresh valid data; if MQTT was down, latest valid data can be sent on reconnect.
  if (mqttClient.connected() && data.valid &&
      data.lastUpdateMillis != lastPublishedDataUpdate &&
      millis() - lastMqttPublishAttempt >= MQTT_PUBLISH_INTERVAL_MS) {
    lastMqttPublishAttempt = millis();
    if (publishFlowData()) lastPublishedDataUpdate = data.lastUpdateMillis;
  }

  delay(5);
}
