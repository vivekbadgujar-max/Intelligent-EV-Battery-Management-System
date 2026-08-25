#define BLYNK_TEMPLATE_ID "TMPL36OnLHkD5"
#define BLYNK_TEMPLATE_NAME "EV BMS Task5"
#define BLYNK_AUTH_TOKEN "nKkqxnUij0v4qdk0oJ-niWzcGSOd8tmh"
#define CLOUD_TEST_BUTTON 25

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// =====================================================
// EV BMS - TASK 1 + TASK 2 + TASK 3 + TASK 4
//
// TASK 1 : Adaptive Multi-Cell Battery Intelligence
// TASK 2 : Event-Driven Safety Protection
// TASK 3 : Intelligent HMI & Diagnostic Interface
// TASK 4 : Fault-Tolerant Embedded Runtime
//
// Hardware:
// ESP32 DevKit V1
// 4 x Potentiometers
// 16x2 I2C LCD
// 4 x LEDs
// Relay
// Buzzer
// =====================================================

// =====================================================
// 1. PIN CONFIGURATION
// =====================================================

#define CELL1_PIN 34
#define CELL2_PIN 35
#define CELL3_PIN 32
#define CELL4_PIN 33

#define SDA_PIN 21
#define SCL_PIN 22

#define LED_GREEN  4
#define LED_YELLOW 2
#define LED_ORANGE 15
#define LED_RED    5

#define RELAY_PIN  26
#define BUZZER_PIN 27

// =====================================================
// 2. LCD
// =====================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
// 3. BATTERY CONFIGURATION
// =====================================================

#define NUM_CELLS 4

const int ADC_MAX = 4095;

const float ADC_MAX_VOLTAGE = 3.3;

const float CELL_MIN_VOLTAGE = 2.5;
const float CELL_MAX_VOLTAGE = 4.2;

// =====================================================
// 4. TASK 1 HEALTH THRESHOLDS
// =====================================================

const float HEALTHY_LIMIT = 3.0;
const float MINOR_LIMIT   = 10.0;
const float CRITICAL_LIMIT = 10.0;

// =====================================================
// 5. TASK 2 SAFETY THRESHOLDS
// =====================================================

const float UV_THRESHOLD = 2.8;
const float OV_THRESHOLD = 4.15;

const float RAPID_CHANGE_THRESHOLD = 0.10;

// =====================================================
// 6. TIMING
// =====================================================

const unsigned long SAMPLE_INTERVAL = 200;

const unsigned long LCD_PAGE_INTERVAL = 3000;

const unsigned long LCD_REFRESH_INTERVAL = 500;

const unsigned long SERIAL_INTERVAL = 2000;

const unsigned long FAULT_CONFIRM_TIME = 500;

const unsigned long SAFETY_RECOVERY_TIME = 5000;

const unsigned long RUNTIME_RECOVERY_TIME = 3000;

const unsigned long RELAY_LOCK_TIME = 3000;

const unsigned long BUZZER_INTERVAL = 300;

// =====================================================
// 7. TASK 4 DIAGNOSTIC SETTINGS
// =====================================================

// IMPORTANT:
//
// Keep all FALSE for NORMAL operation.
//
// Change ONE at a time for testing.
//
// -----------------------------------------------------

bool simulateFrozenADC = false;

bool simulateRelayMismatch = false;

bool simulateShutdown = false;

bool cloudTestMode = false;

// Frozen ADC threshold
const int FROZEN_LIMIT = 10;

// =====================================================
// 8. TIMERS
// =====================================================

unsigned long previousSampleTime = 0;

unsigned long previousLCDPageTime = 0;

unsigned long previousLCDRefreshTime = 0;

unsigned long previousSerialTime = 0;

unsigned long faultStartTime = 0;

unsigned long safetyRecoveryStart = 0;

unsigned long runtimeRecoveryStart = 0;

unsigned long lastRelayChangeTime = 0;

unsigned long previousBuzzerTime = 0;

// =====================================================
// 9. BATTERY DATA
// =====================================================

float cellVoltage[NUM_CELLS];

float previousCellVoltage[NUM_CELLS];

int cellADC[NUM_CELLS];

int previousADC[NUM_CELLS];

int frozenCount[NUM_CELLS];

bool cellSensorHealthy[NUM_CELLS];

float packVoltage = 0.0;

float averageVoltage = 0.0;

float maxVoltage = 0.0;

float minVoltage = 0.0;

float imbalanceVoltage = 0.0;

float imbalancePercent = 0.0;

int strongestCell = 0;

int weakestCell = 0;

String batteryHealth;

// =====================================================
// 10. TASK 2 SAFETY STATE
// =====================================================

enum SafetyState
{
  SAFETY_NORMAL,
  SAFETY_WARNING,
  SAFETY_TRIPPED,
  SAFETY_RECOVERY
};

SafetyState safetyState = SAFETY_NORMAL;

// =====================================================
// 11. TASK 2 FAULT FLAGS
// =====================================================

bool underVoltageFault = false;

bool overVoltageFault = false;

bool rapidChangeFault = false;

bool sensorFault = false;

bool faultDetected = false;

bool faultConfirmed = false;

// =====================================================
// 12. TASK 4 RUNTIME STATES
// =====================================================

enum RuntimeMode
{
  RUNTIME_NORMAL,
  RUNTIME_DEGRADED,
  RUNTIME_FAILSAFE,
  RUNTIME_SHUTDOWN
};

RuntimeMode runtimeMode = RUNTIME_NORMAL;

// =====================================================
// 13. TASK 4 FAULT FLAGS
// =====================================================

bool invalidSensorFault = false;

bool frozenADCFault = false;

bool relayMismatchFault = false;

bool multipleFailureFault = false;

// =====================================================
// 14. RELAY
// =====================================================

bool relayState = true;

// =====================================================
// 15. LCD HMI
// =====================================================

int currentPage = 0;

const int TOTAL_PAGES = 5;

bool faultPriorityActive = false;

RuntimeMode previousRuntimeMode = RUNTIME_NORMAL;

SafetyState previousSafetyState = SAFETY_NORMAL;

int previousPage = -1;

// =====================================================
// 16. FAULT LOG
// =====================================================

struct FaultLog
{
  unsigned long timestamp;

  String faultName;

  String mode;
};

#define MAX_FAULT_LOGS 10

FaultLog faultLogs[MAX_FAULT_LOGS];

int faultLogCount = 0;

// =====================================================
// 17. FAULT EVENT MEMORY
// =====================================================

bool previousInvalidSensorFault = false;

bool previousFrozenADCFault = false;

bool previousRelayMismatchFault = false;

bool previousUnderVoltageFault = false;

bool previousOverVoltageFault = false;

bool previousRapidChangeFault = false;


// =====================================================
// TASK 5 - INTELLIGENT CLOUD TELEMETRY
// =====================================================
// Blynk virtual datastream mapping - MATCHES THE USER'S DASHBOARD:
 // V0  Cell 1 Voltage
 // V1  Cell 2 Voltage
 // V2  Cell 3 Voltage
 // V3  Cell 4 Voltage
 // V4  Pack Voltage
 // V5  Average Voltage
 // V6  Imbalance %
 // V7  Battery Health
 // V8  Runtime Mode
 // V9  Safety State
 // V10 Relay State
 // V11 Fault / Event Message
 // V12 WiFi RSSI
//
// IMPORTANT: Create the same datastream VPINs in Blynk.
 // Event code used below: bms_fault
// =====================================================

#define ENABLE_BLYNK_EVENTS 1

const unsigned long WIFI_RECONNECT_INTERVAL = 5000;
const unsigned long BLYNK_RECONNECT_INTERVAL = 5000;
const float TELEMETRY_VOLTAGE_CHANGE = 0.05;
const float TELEMETRY_IMBALANCE_CHANGE = 0.5;

unsigned long previousWiFiReconnect = 0;
unsigned long previousBlynkReconnect = 0;

bool previousCloudConnected = false;
bool cloudSyncRequired = true;

// Last values actually transmitted to Blynk
float sentCellVoltage[NUM_CELLS] = {-100, -100, -100, -100};
float sentPackVoltage = -100;
float sentAverageVoltage = -100;
float sentImbalancePercent = -100;
String sentRuntimeMode = "";
String sentSafetyState = "";
String sentHealth = "";
String sentFaultText = "";
int sentRelayState = -1;
int sentRSSI = 999;

// Small event queue for faults/state changes while cloud is offline
#define CLOUD_EVENT_QUEUE_SIZE 10
String cloudEventQueue[CLOUD_EVENT_QUEUE_SIZE];
int cloudEventHead = 0;
int cloudEventTail = 0;
int cloudEventCount = 0;

void queueCloudEvent(const String &eventText)
{
  if (cloudEventCount < CLOUD_EVENT_QUEUE_SIZE)
  {
    cloudEventQueue[cloudEventTail] = eventText;
    cloudEventTail = (cloudEventTail + 1) % CLOUD_EVENT_QUEUE_SIZE;
    cloudEventCount++;
  }
  else
  {
    // Keep the newest event when the queue is full.
    cloudEventQueue[cloudEventHead] = eventText;
    cloudEventHead = (cloudEventHead + 1) % CLOUD_EVENT_QUEUE_SIZE;
    cloudEventTail = cloudEventHead;
  }
}

bool cloudEventAvailable()
{
  return cloudEventCount > 0;
}

String popCloudEvent()
{
  if (cloudEventCount == 0) return "";

  String e = cloudEventQueue[cloudEventHead];
  cloudEventHead = (cloudEventHead + 1) % CLOUD_EVENT_QUEUE_SIZE;
  cloudEventCount--;
  return e;
}

String getRuntimeModeText()
{
  switch (runtimeMode)
  {
    case RUNTIME_NORMAL:   return "NORMAL";
    case RUNTIME_DEGRADED: return "DEGRADED";
    case RUNTIME_FAILSAFE: return "FAILSAFE";
    default:               return "SHUTDOWN";
  }
}

String getSafetyStateText()
{
  switch (safetyState)
  {
    case SAFETY_NORMAL:   return "NORMAL";
    case SAFETY_WARNING:  return "WARNING";
    case SAFETY_TRIPPED:  return "TRIPPED";
    default:              return "RECOVERY";
  }
}

String getMainFaultText()
{
  if (overVoltageFault) return "OVER VOLTAGE";
  if (underVoltageFault) return "UNDER VOLTAGE";
  if (relayMismatchFault) return "RELAY MISMATCH";
  if (invalidSensorFault) return "INVALID SENSOR";
  if (frozenADCFault) return "ADC FROZEN";
  if (rapidChangeFault) return "RAPID VOLTAGE CHANGE";
  return "NO ACTIVE FAULT";
}

void sendBlynkEvent(const String &eventText)
{
#if ENABLE_BLYNK_EVENTS
  if (Blynk.connected())
  {
    Blynk.logEvent("bms_fault", eventText);
  }
#else
  (void)eventText;
#endif
}

void syncAllTelemetry()
{
  if (!Blynk.connected()) return;

  // Full synchronization is performed ONLY after a cloud connection/reconnection.
  Blynk.virtualWrite(V0, cellVoltage[0]);
  Blynk.virtualWrite(V1, cellVoltage[1]);
  Blynk.virtualWrite(V2, cellVoltage[2]);
  Blynk.virtualWrite(V3, cellVoltage[3]);
  Blynk.virtualWrite(V4, packVoltage);
  Blynk.virtualWrite(V5, averageVoltage);
  Blynk.virtualWrite(V6, imbalancePercent);
  Blynk.virtualWrite(V7, batteryHealth);
  Blynk.virtualWrite(V8, getRuntimeModeText());
  Blynk.virtualWrite(V9, getSafetyStateText());
  Blynk.virtualWrite(V10, relayState ? 1 : 0);
  Blynk.virtualWrite(V11, getMainFaultText());
  Blynk.virtualWrite(V12, WiFi.RSSI());

  for (int i = 0; i < NUM_CELLS; i++)
    sentCellVoltage[i] = cellVoltage[i];

  sentPackVoltage = packVoltage;
  sentAverageVoltage = averageVoltage;
  sentImbalancePercent = imbalancePercent;
  sentRuntimeMode = getRuntimeModeText();
  sentSafetyState = getSafetyStateText();
  sentRelayState = relayState ? 1 : 0;
  sentRSSI = WiFi.RSSI();
  sentHealth = batteryHealth;
  sentFaultText = getMainFaultText();

  cloudSyncRequired = false;
}

void flushCloudEvents()
{
  if (!Blynk.connected()) return;

  // Send queued events after reconnect, then remove them from the queue.
  while (cloudEventAvailable())
  {
    String eventText = popCloudEvent();
    Blynk.virtualWrite(V10, eventText);
    sendBlynkEvent(eventText);
  }
}

void publishChangedTelemetry()
{
  if (!Blynk.connected()) return;

  // V0-V3: cell voltage; send only after a meaningful change.
  for (int i = 0; i < NUM_CELLS; i++)
  {
    if (fabs(cellVoltage[i] - sentCellVoltage[i]) >= TELEMETRY_VOLTAGE_CHANGE)
    {
      Blynk.virtualWrite(V0 + i, cellVoltage[i]);
      sentCellVoltage[i] = cellVoltage[i];
    }
  }

  // V4: pack voltage.
  if (fabs(packVoltage - sentPackVoltage) >= TELEMETRY_VOLTAGE_CHANGE)
  {
    Blynk.virtualWrite(V4, packVoltage);
    sentPackVoltage = packVoltage;
  }

  // V5: average voltage.
  if (fabs(averageVoltage - sentAverageVoltage) >= TELEMETRY_VOLTAGE_CHANGE)
  {
    Blynk.virtualWrite(V5, averageVoltage);
    sentAverageVoltage = averageVoltage;
  }

  // V6: imbalance percentage.
  if (fabs(imbalancePercent - sentImbalancePercent) >= TELEMETRY_IMBALANCE_CHANGE)
  {
    Blynk.virtualWrite(V6, imbalancePercent);
    sentImbalancePercent = imbalancePercent;
  }

  // V7: battery health.
  if (batteryHealth != sentHealth)
  {
    Blynk.virtualWrite(V7, batteryHealth);
    sentHealth = batteryHealth;
  }

  // V8: runtime mode.
  String runtimeText = getRuntimeModeText();
  if (runtimeText != sentRuntimeMode)
  {
    Blynk.virtualWrite(V8, runtimeText);
    sentRuntimeMode = runtimeText;
  }

  // V9: safety state.
  String safetyText = getSafetyStateText();
  if (safetyText != sentSafetyState)
  {
    Blynk.virtualWrite(V9, safetyText);
    sentSafetyState = safetyText;
  }

  // V10: relay state.
  int relay = relayState ? 1 : 0;
  if (relay != sentRelayState)
  {
    Blynk.virtualWrite(V10, relay);
    sentRelayState = relay;
  }

  // V11: fault/event text.
  String faultText = getMainFaultText();
  if (faultText != sentFaultText)
  {
    Blynk.virtualWrite(V11, faultText);
    sentFaultText = faultText;
  }

  // V12: Wi-Fi RSSI.
  int rssi = WiFi.RSSI();
  if (abs(rssi - sentRSSI) >= 5)
  {
    Blynk.virtualWrite(V12, rssi);
    sentRSSI = rssi;
  }
}

void processCloudTelemetry()
{
  unsigned long now = millis();

  // WiFi reconnect is non-blocking: no delay() is used here.
  if (WiFi.status() != WL_CONNECTED)
  {
    if (now - previousWiFiReconnect >= WIFI_RECONNECT_INTERVAL)
    {
      previousWiFiReconnect = now;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    previousCloudConnected = false;
    return;
  }

  // Blynk reconnect is attempted periodically,
// unless cloud test mode intentionally disables it.
if (cloudTestMode)
{
  if (Blynk.connected())
  {
    Blynk.disconnect();
  }

  previousCloudConnected = false;
  return;
}

if (!Blynk.connected())
{
  if (now - previousBlynkReconnect >= BLYNK_RECONNECT_INTERVAL)
  {
    previousBlynkReconnect = now;
    Blynk.connect(1000);
  }

  previousCloudConnected = false;
  return;
}

  // Detect a new cloud connection.
  if (!previousCloudConnected)
  {
    previousCloudConnected = true;
    cloudSyncRequired = true;
    Serial.println(">>> BLYNK CONNECTED - CLOUD SYNC <<<");
  }

  if (cloudSyncRequired)
  {
    syncAllTelemetry();
    flushCloudEvents();
  }
  else
  {
    publishChangedTelemetry();
  }
}


void connectWiFi()
{
  Serial.println();
  Serial.println("Starting Wi-Fi connection...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void handleCloudTestButton()
{

  bool pressed = (digitalRead(CLOUD_TEST_BUTTON) == LOW);

  if (pressed && !cloudTestMode)
  {
    cloudTestMode = true;

    Serial.println();
    Serial.println("======================================");
    Serial.println(">>> CLOUD TEST MODE: OFFLINE <<<");
    Serial.println("Blynk communication intentionally disabled");
    Serial.println("BMS operation continues normally");
    Serial.println("======================================");
  }

  if (!pressed && cloudTestMode)
  {
    cloudTestMode = false;

    Serial.println();
    Serial.println("======================================");
    Serial.println(">>> CLOUD TEST MODE: ONLINE <<<");
    Serial.println("Blynk communication restored");
    Serial.println("Queued events will synchronize");
    Serial.println("======================================");
  }
}

void updateRiskDiagnosticMask()
{
  int riskMask = 0;

  // Bit 0 - Under Voltage
  if (underVoltageFault)
  {
    riskMask |= (1 << 0);
  }

  // Bit 1 - Over Voltage
  if (overVoltageFault)
  {
    riskMask |= (1 << 1);
  }

  // Bit 2 - Rapid Voltage Change
  if (rapidChangeFault)
  {
    riskMask |= (1 << 2);
  }

  // Bit 3 - Invalid Sensor
  if (invalidSensorFault)
  {
    riskMask |= (1 << 3);
  }

  if (Blynk.connected())
  {
    Blynk.virtualWrite(V13, riskMask);
  }
}

int lastPublishedFaultCount = 0;

void publishFaultHistory()
{
  if (!Blynk.connected())
  {
    return;
  }

  // Send only newly added fault events
  while (lastPublishedFaultCount < faultLogCount)
  {
    int i = lastPublishedFaultCount;

    String eventText = "";

    eventText += "#";
    eventText += String(i + 1);
    eventText += " ";
    eventText += String(faultLogs[i].timestamp);
    eventText += "ms ";
    eventText += faultLogs[i].faultName;
    eventText += " [";
    eventText += faultLogs[i].mode;
    eventText += "]";

    Blynk.virtualWrite(V14, eventText);

    lastPublishedFaultCount++;
  }
}

// ADD THIS HERE

void updateOperatorRecommendation()
{
  String recommendation;

  if (underVoltageFault)
  {
    recommendation = "UNDER VOLTAGE: Inspect affected cell and reduce load.";
  }
  else if (overVoltageFault)
  {
    recommendation = "OVER VOLTAGE: Stop charging and inspect cell.";
  }
  else if (rapidChangeFault)
  {
    recommendation = "RAPID CHANGE: Check cell and sensor.";
  }
  else if (invalidSensorFault)
  {
    recommendation = "INVALID SENSOR: Check sensor connection.";
  }
  else if (batteryHealth == "CRITICAL IMBALANCE")
  {
    recommendation = "CRITICAL IMBALANCE: Inspect and balance cells.";
  }
  else if (batteryHealth == "MINOR IMBALANCE")
  {
    recommendation = "MINOR IMBALANCE: Monitor weakest cell.";
  }
  else
  {
    recommendation = "NORMAL: Battery operating normally.";
  }

  if (Blynk.connected())
  {
    Blynk.virtualWrite(V15, recommendation);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  connectWiFi();

  Blynk.config(BLYNK_AUTH_TOKEN);

  Serial.println("Blynk configured; cloud connection will be handled in loop.");

  pinMode(CLOUD_TEST_BUTTON, INPUT_PULLUP);
  // =================================================
  // ADC
  // =================================================

  analogReadResolution(12);

  pinMode(CELL1_PIN, INPUT);
  pinMode(CELL2_PIN, INPUT);
  pinMode(CELL3_PIN, INPUT);
  pinMode(CELL4_PIN, INPUT);

  // =================================================
  // LEDs
  // =================================================

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_RED, LOW);

  // =================================================
  // RELAY
  // =================================================

  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);

  relayState = true;

  lastRelayChangeTime = millis();

  // =================================================
  // BUZZER
  // =================================================

  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  // =================================================
  // I2C
  // =================================================

  Wire.begin(SDA_PIN, SCL_PIN);

  // =================================================
  // LCD
  // =================================================

  lcd.init();

  lcd.backlight();

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("EV BMS TASK 4");

  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // =================================================
  // INITIAL READING
  // =================================================

  readCellVoltages();

  calculateBatteryParameters();

  identifyStrongestWeakest();

  calculateImbalance();

  classifyBatteryHealth();

  for (int i = 0; i < NUM_CELLS; i++)
  {
    previousCellVoltage[i] = cellVoltage[i];

    previousADC[i] = cellADC[i];

    frozenCount[i] = 0;

    cellSensorHealthy[i] = true;
  }

  updateLEDs();

  displayNormalLCD();

  Serial.println();
  Serial.println("==============================================");
  Serial.println("     EV BMS TASK 1 + 2 + 3 + 4 READY");
  Serial.println("==============================================");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{

  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED)
  {
    if (!cloudTestMode && Blynk.connected())
    {
      Blynk.run();
    }
  }

  // TASK 5: network/cloud manager.
  // Runs independently of battery protection logic.
  processCloudTelemetry();

  // ===================================================
  // BATTERY + SAFETY + RUNTIME
  // ===================================================

  if (now - previousSampleTime >= SAMPLE_INTERVAL)
  {
    previousSampleTime = now;

    // -------------------------------------------------
    // TASK 1
    // -------------------------------------------------

    readCellVoltages();

    calculateBatteryParameters();

    identifyStrongestWeakest();

    calculateImbalance();

    classifyBatteryHealth();

    // -------------------------------------------------
    // TASK 2
    // -------------------------------------------------

    detectSafetyConditions();

    updateSafetyState();

    updateRelay();

    updateBuzzer();

    // -------------------------------------------------
    // TASK 4
    // -------------------------------------------------

    checkSensorHealth();

    checkFrozenADC();

    checkRelayMismatch();

    updateRuntimeMode();

    updateLEDs();

    // -------------------------------------------------
    // FAULT EVENT LOGGING
    // -------------------------------------------------

    processFaultEvents();

    updateRiskDiagnosticMask();

    publishFaultHistory();

    updateOperatorRecommendation();

  }

  // ===================================================
  // TASK 3 - AUTOMATIC PAGE ROTATION
  // ===================================================

  if (now - previousLCDPageTime >= LCD_PAGE_INTERVAL)
  {
    previousLCDPageTime = now;

    // Do not rotate normal pages during critical faults

    if (!faultDetected &&
        runtimeMode == RUNTIME_NORMAL)
    {
      currentPage++;

      if (currentPage >= TOTAL_PAGES)
      {
        currentPage = 0;
      }

      previousPage = -1;
    }
  }

  // ===================================================
  // LCD REFRESH
  // ===================================================

  if (now - previousLCDRefreshTime >= LCD_REFRESH_INTERVAL)
  {
    previousLCDRefreshTime = now;

    updateLCD();
  }

  // ===================================================
  // SERIAL
  // ===================================================

  if (now - previousSerialTime >= SERIAL_INTERVAL)
  {
    previousSerialTime = now;

    displaySerialReport();
  }
}

// =====================================================
// TASK 1
// READ CELL VOLTAGES
// =====================================================

void readCellVoltages()
{
  cellADC[0] = analogRead(CELL1_PIN);

  cellADC[1] = analogRead(CELL2_PIN);

  cellADC[2] = analogRead(CELL3_PIN);

  cellADC[3] = analogRead(CELL4_PIN);

  for (int i = 0; i < NUM_CELLS; i++)
  {
    cellVoltage[i] =
      convertADCToCellVoltage(cellADC[i]);
  }
}

// =====================================================
// TASK 1
// ADC TO VOLTAGE
// =====================================================

float convertADCToCellVoltage(int rawADC)
{
  float adcVoltage =
    ((float)rawADC / ADC_MAX)
    * ADC_MAX_VOLTAGE;

  float voltage =
    CELL_MIN_VOLTAGE
    +
    (adcVoltage / ADC_MAX_VOLTAGE)
    *
    (CELL_MAX_VOLTAGE -
     CELL_MIN_VOLTAGE);

  return voltage;
}

// =====================================================
// TASK 1
// BATTERY PARAMETERS
// =====================================================

void calculateBatteryParameters()
{
  packVoltage = 0.0;

  for (int i = 0; i < NUM_CELLS; i++)
  {
    packVoltage += cellVoltage[i];
  }

  averageVoltage =
    packVoltage / NUM_CELLS;

  maxVoltage = cellVoltage[0];

  minVoltage = cellVoltage[0];

  for (int i = 1; i < NUM_CELLS; i++)
  {
    if (cellVoltage[i] > maxVoltage)
    {
      maxVoltage = cellVoltage[i];
    }

    if (cellVoltage[i] < minVoltage)
    {
      minVoltage = cellVoltage[i];
    }
  }
}

// =====================================================
// TASK 1
// STRONGEST / WEAKEST
// =====================================================

void identifyStrongestWeakest()
{
  strongestCell = 0;

  weakestCell = 0;

  for (int i = 1; i < NUM_CELLS; i++)
  {
    if (cellVoltage[i] >
        cellVoltage[strongestCell])
    {
      strongestCell = i;
    }

    if (cellVoltage[i] <
        cellVoltage[weakestCell])
    {
      weakestCell = i;
    }
  }
}

// =====================================================
// TASK 1
// IMBALANCE
// =====================================================

void calculateImbalance()
{
  imbalanceVoltage =
    maxVoltage - minVoltage;

  if (averageVoltage > 0)
  {
    imbalancePercent =
      (imbalanceVoltage /
       averageVoltage)
      * 100.0;
  }
  else
  {
    imbalancePercent = 0.0;
  }
}

// =====================================================
// TASK 1
// HEALTH CLASSIFICATION
// =====================================================

void classifyBatteryHealth()
{
  if (imbalancePercent < 3.0)
  {
    batteryHealth = "HEALTHY";
  }

  else if (imbalancePercent < 10.0)
  {
    batteryHealth = "MINOR IMBALANCE";
  }

  else
  {
    batteryHealth = "CRITICAL IMBALANCE";
  }
}

// =====================================================
// TASK 2
// SAFETY DETECTION
// =====================================================

void detectSafetyConditions()
{
  underVoltageFault = false;

  overVoltageFault = false;

  rapidChangeFault = false;

  sensorFault = false;

  for (int i = 0; i < NUM_CELLS; i++)
  {
    // -----------------------------------------------
    // Under Voltage
    // -----------------------------------------------

    if (cellVoltage[i] < UV_THRESHOLD)
    {
      underVoltageFault = true;
    }

    // -----------------------------------------------
    // Over Voltage
    // -----------------------------------------------

    if (cellVoltage[i] > OV_THRESHOLD)
    {
      overVoltageFault = true;
    }

    // -----------------------------------------------
    // Rapid Voltage Change
    // -----------------------------------------------

    float change =
      fabs(cellVoltage[i] -
           previousCellVoltage[i]);

    if (change > RAPID_CHANGE_THRESHOLD)
    {
      rapidChangeFault = true;
    }

    // -----------------------------------------------
    // Invalid sensor
    // -----------------------------------------------

    if (cellADC[i] <= 5 ||
        cellADC[i] >= 4090)
    {
      sensorFault = true;
    }
  }

  // Store previous voltages

  for (int i = 0; i < NUM_CELLS; i++)
  {
    previousCellVoltage[i] =
      cellVoltage[i];
  }

  faultDetected =
    underVoltageFault ||
    overVoltageFault ||
    rapidChangeFault ||
    sensorFault;
}

// =====================================================
// TASK 2
// SAFETY STATE MACHINE
// =====================================================

void updateSafetyState()
{
  unsigned long now = millis();

  switch (safetyState)
  {
    case SAFETY_NORMAL:

      if (faultDetected)
      {
        safetyState = SAFETY_WARNING;

        faultStartTime = now;
      }

      break;

    case SAFETY_WARNING:

      if (!faultDetected)
      {
        safetyState = SAFETY_NORMAL;

        faultConfirmed = false;

        break;
      }

      if (now - faultStartTime >=
          FAULT_CONFIRM_TIME)
      {
        faultConfirmed = true;

        safetyState = SAFETY_TRIPPED;
      }

      break;

    case SAFETY_TRIPPED:

      if (!faultDetected)
      {
        safetyRecoveryStart = now;

        safetyState = SAFETY_RECOVERY;
      }

      break;

    case SAFETY_RECOVERY:

      if (faultDetected)
      {
        safetyState = SAFETY_TRIPPED;

        break;
      }

      if (now - safetyRecoveryStart >=
          SAFETY_RECOVERY_TIME)
      {
        safetyState = SAFETY_NORMAL;

        faultConfirmed = false;
      }

      break;
  }
}

// =====================================================
// TASK 2
// RELAY CONTROL
// =====================================================

void updateRelay()
{
  unsigned long now = millis();

  // -----------------------------------------------
  // SAFETY TRIP
  // -----------------------------------------------

  if (safetyState == SAFETY_WARNING ||
      safetyState == SAFETY_TRIPPED)
  {
    if (relayState)
    {
      if (now - lastRelayChangeTime >=
          RELAY_LOCK_TIME)
      {
        digitalWrite(RELAY_PIN, LOW);

        relayState = false;

        lastRelayChangeTime = now;

        Serial.println(">>> RELAY CUT-OFF <<<");
      }
    }
  }

  // -----------------------------------------------
  // RECOVERY
  // -----------------------------------------------

  else if (safetyState ==
           SAFETY_NORMAL)
  {
    if (!relayState)
    {
      if (now - lastRelayChangeTime >=
          RELAY_LOCK_TIME)
      {
        digitalWrite(RELAY_PIN, HIGH);

        relayState = true;

        lastRelayChangeTime = now;

        Serial.println(">>> RELAY RESTORED <<<");
      }
    }
  }
}

// =====================================================
// TASK 2
// BUZZER
// =====================================================

void updateBuzzer()
{
  unsigned long now = millis();

  if (safetyState == SAFETY_WARNING ||
      safetyState == SAFETY_TRIPPED)
  {
    if (now - previousBuzzerTime >=
        BUZZER_INTERVAL)
    {
      previousBuzzerTime = now;

      digitalWrite(
        BUZZER_PIN,
        !digitalRead(BUZZER_PIN)
      );
    }
  }

  else
  {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// =====================================================
// TASK 4
// SENSOR HEALTH
// =====================================================

void checkSensorHealth()
{
  invalidSensorFault = false;

  for (int i = 0; i < NUM_CELLS; i++)
  {
    if (cellADC[i] <= 5 ||
        cellADC[i] >= 4090)
    {
      invalidSensorFault = true;

      cellSensorHealthy[i] = false;
    }
    else
    {
      cellSensorHealthy[i] = true;
    }
  }
}

// =====================================================
// TASK 4
// FROZEN ADC
//
// IMPORTANT:
// A stable battery voltage is NOT automatically
// considered an ADC fault.
//
// Frozen ADC is activated only when:
// simulateFrozenADC = true
// =====================================================

void checkFrozenADC()
{
  frozenADCFault = false;

  if (!simulateFrozenADC)
  {
    for (int i = 0; i < NUM_CELLS; i++)
    {
      frozenCount[i] = 0;

      previousADC[i] =
        cellADC[i];
    }

    return;
  }

  // Diagnostic simulation enabled

  for (int i = 0; i < NUM_CELLS; i++)
  {
    if (cellADC[i] ==
        previousADC[i])
    {
      frozenCount[i]++;
    }
    else
    {
      frozenCount[i] = 0;
    }

    if (frozenCount[i] >=
        FROZEN_LIMIT)
    {
      frozenADCFault = true;

      frozenCount[i] =
        FROZEN_LIMIT;
    }

    previousADC[i] =
      cellADC[i];
  }
}

// =====================================================
// TASK 4
// RELAY MISMATCH
// =====================================================

void checkRelayMismatch()
{
  relayMismatchFault =
    simulateRelayMismatch;
}

// =====================================================
// TASK 4
// RUNTIME STATE MACHINE
// =====================================================

void updateRuntimeMode()
{
  unsigned long now = millis();

  // =================================================
  // SHUTDOWN
  // =================================================

  if (simulateShutdown)
  {
    if (runtimeMode !=
        RUNTIME_SHUTDOWN)
    {
      logFault(
        "SYSTEM SHUTDOWN",
        "SHUTDOWN"
      );
    }

    runtimeMode =
      RUNTIME_SHUTDOWN;

    digitalWrite(RELAY_PIN, LOW);

    relayState = false;

    return;
  }

  // =================================================
  // FAILSAFE
  // =================================================

  if (relayMismatchFault ||
      overVoltageFault ||
      underVoltageFault)
  {
    if (runtimeMode !=
        RUNTIME_FAILSAFE)
    {
      logFault(
        "CRITICAL SAFETY FAULT",
        "FAILSAFE"
      );
    }

    runtimeMode =
      RUNTIME_FAILSAFE;

    runtimeRecoveryStart = 0;

    return;
  }

  // =================================================
  // DEGRADED
  // =================================================

  if (invalidSensorFault ||
      frozenADCFault ||
      rapidChangeFault)
  {
    if (runtimeMode ==
        RUNTIME_NORMAL)
    {
      runtimeRecoveryStart = 0;

      logFault(
        "SYSTEM DEGRADED",
        "DEGRADED"
      );
    }

    runtimeMode =
      RUNTIME_DEGRADED;

    return;
  }

  // =================================================
  // RECOVERY FROM FAILSAFE
  // =================================================

  if (runtimeMode ==
      RUNTIME_FAILSAFE)
  {
    if (!underVoltageFault &&
        !overVoltageFault &&
        !relayMismatchFault)
    {
      if (runtimeRecoveryStart == 0)
      {
        runtimeRecoveryStart = now;
      }

      if (now - runtimeRecoveryStart >=
          RUNTIME_RECOVERY_TIME)
      {
        runtimeMode =
          RUNTIME_NORMAL;

        runtimeRecoveryStart = 0;

        logFault(
          "FAILSAFE RECOVERED",
          "NORMAL"
        );
      }
    }
    else
    {
      runtimeRecoveryStart = 0;
    }

    return;
  }

  // =================================================
  // RECOVERY FROM DEGRADED
  // =================================================

  if (runtimeMode ==
      RUNTIME_DEGRADED)
  {
    if (!invalidSensorFault &&
        !frozenADCFault &&
        !rapidChangeFault)
    {
      if (runtimeRecoveryStart == 0)
      {
        runtimeRecoveryStart = now;
      }

      if (now - runtimeRecoveryStart >=
          RUNTIME_RECOVERY_TIME)
      {
        runtimeMode =
          RUNTIME_NORMAL;

        runtimeRecoveryStart = 0;

        logFault(
          "DEGRADED RECOVERED",
          "NORMAL"
        );
      }
    }
    else
    {
      runtimeRecoveryStart = 0;
    }

    return;
  }

  // =================================================
  // NORMAL
  // =================================================

  if (!invalidSensorFault &&
      !frozenADCFault &&
      !relayMismatchFault &&
      !underVoltageFault &&
      !overVoltageFault &&
      !rapidChangeFault)
  {
    runtimeMode =
      RUNTIME_NORMAL;
  }
}

// =====================================================
// TASK 4
// FAULT EVENT PROCESSING
// =====================================================

void processFaultEvents()
{
  // -----------------------------------------------
  // Invalid sensor event
  // -----------------------------------------------

  if (invalidSensorFault &&
      !previousInvalidSensorFault)
  {
    logFault(
      "INVALID SENSOR",
      "DEGRADED"
    );
  }

  // -----------------------------------------------
  // Frozen ADC event
  // -----------------------------------------------

  if (frozenADCFault &&
      !previousFrozenADCFault)
  {
    logFault(
      "ADC FROZEN",
      "DEGRADED"
    );
  }

  // -----------------------------------------------
  // Relay mismatch
  // -----------------------------------------------

  if (relayMismatchFault &&
      !previousRelayMismatchFault)
  {
    logFault(
      "RELAY MISMATCH",
      "FAILSAFE"
    );
  }

  // -----------------------------------------------
  // Under voltage
  // -----------------------------------------------

  if (underVoltageFault &&
      !previousUnderVoltageFault)
  {
    logFault(
      "UNDER VOLTAGE",
      "FAILSAFE"
    );
  }

  // -----------------------------------------------
  // Over voltage
  // -----------------------------------------------

  if (overVoltageFault &&
      !previousOverVoltageFault)
  {
    logFault(
      "OVER VOLTAGE",
      "FAILSAFE"
    );
  }

  // -----------------------------------------------
  // Rapid voltage change
  // -----------------------------------------------

  if (rapidChangeFault &&
      !previousRapidChangeFault)
  {
    logFault(
      "RAPID VOLTAGE CHANGE",
      "DEGRADED"
    );
  }

  // Save previous states

  previousInvalidSensorFault =
    invalidSensorFault;

  previousFrozenADCFault =
    frozenADCFault;

  previousRelayMismatchFault =
    relayMismatchFault;

  previousUnderVoltageFault =
    underVoltageFault;

  previousOverVoltageFault =
    overVoltageFault;

  previousRapidChangeFault =
    rapidChangeFault;
}

// =====================================================
// TASK 4
// FAULT LOG
// =====================================================

void logFault(String faultName,
              String mode)
{
  if (faultLogCount >=
      MAX_FAULT_LOGS)
  {
    return;
  }

  faultLogs[faultLogCount].timestamp =
    millis();

  faultLogs[faultLogCount].faultName =
    faultName;

  faultLogs[faultLogCount].mode =
    mode;

  // Task 5: preserve the fault event if cloud is offline.
  queueCloudEvent(faultName + " [" + mode + "]");

  faultLogCount++;

  Serial.println();
  Serial.println("******** FAULT EVENT ********");

  Serial.print("Time  : ");
  Serial.print(millis());
  Serial.println(" ms");

  Serial.print("Fault : ");
  Serial.println(faultName);

  Serial.print("Mode  : ");
  Serial.println(mode);

  Serial.println("******************************");
}

// =====================================================
// LED CONTROL
// =====================================================

void updateLEDs()
{
  digitalWrite(LED_GREEN, LOW);

  digitalWrite(LED_YELLOW, LOW);

  digitalWrite(LED_ORANGE, LOW);

  digitalWrite(LED_RED, LOW);

  // -----------------------------------------------
  // SHUTDOWN
  // -----------------------------------------------

  if (runtimeMode ==
      RUNTIME_SHUTDOWN)
  {
    digitalWrite(LED_RED, HIGH);

    return;
  }

  // -----------------------------------------------
  // FAILSAFE
  // -----------------------------------------------

  if (runtimeMode ==
      RUNTIME_FAILSAFE)
  {
    digitalWrite(LED_RED, HIGH);

    return;
  }

  // -----------------------------------------------
  // DEGRADED
  // -----------------------------------------------

  if (runtimeMode ==
      RUNTIME_DEGRADED)
  {
    digitalWrite(LED_YELLOW, HIGH);

    return;
  }

  // -----------------------------------------------
  // NORMAL
  // -----------------------------------------------

  if (batteryHealth ==
      "HEALTHY")
  {
    digitalWrite(LED_GREEN, HIGH);
  }

  else if (batteryHealth ==
           "MINOR IMBALANCE")
  {
    digitalWrite(LED_YELLOW, HIGH);
  }

  else
  {
    digitalWrite(LED_ORANGE, HIGH);
  }
}

// =====================================================
// TASK 3
// LCD MANAGER
// =====================================================

void updateLCD()
{
  // =================================================
  // SHUTDOWN PRIORITY
  // =================================================

  if (runtimeMode ==
      RUNTIME_SHUTDOWN)
  {
    displayShutdownLCD();

    return;
  }

  // =================================================
  // FAILSAFE PRIORITY
  // =================================================

  if (runtimeMode ==
      RUNTIME_FAILSAFE)
  {
    displayFailsafeLCD();

    return;
  }

  // =================================================
  // DEGRADED PRIORITY
  // =================================================

  if (runtimeMode ==
      RUNTIME_DEGRADED)
  {
    displayDegradedLCD();

    return;
  }

  // =================================================
  // SAFETY FAULT PRIORITY
  // =================================================

  if (faultDetected)
  {
    displayFaultLCD();

    return;
  }

  // =================================================
  // NORMAL HMI PAGES
  // =================================================

  if (currentPage == previousPage)
  {
    return;
  }

  previousPage =
    currentPage;

  lcd.clear();

  switch (currentPage)
  {
    case 0:
      displayCellPage();
      break;

    case 1:
      displayPackPage();
      break;

    case 2:
      displayHealthPage();
      break;

    case 3:
      displaySafetyPage();
      break;

    case 4:
      displayDiagnosticPage();
      break;
  }
}

// =====================================================
// LCD PAGE 1
// =====================================================

void displayCellPage()
{
  lcd.setCursor(0, 0);

  lcd.print("C1:");
  lcd.print(cellVoltage[0], 2);

  lcd.print(" C2:");
  lcd.print(cellVoltage[1], 2);

  lcd.setCursor(0, 1);

  lcd.print("C3:");
  lcd.print(cellVoltage[2], 2);

  lcd.print(" C4:");
  lcd.print(cellVoltage[3], 2);
}

// =====================================================
// LCD PAGE 2
// =====================================================

void displayPackPage()
{
  lcd.setCursor(0, 0);

  lcd.print("PACK:");
  lcd.print(packVoltage, 2);
  lcd.print("V");

  lcd.setCursor(0, 1);

  lcd.print("AVG:");
  lcd.print(averageVoltage, 2);
  lcd.print("V");
}

// =====================================================
// LCD PAGE 3
// =====================================================

void displayHealthPage()
{
  lcd.setCursor(0, 0);

  lcd.print("IMB:");
  lcd.print(imbalancePercent, 1);
  lcd.print("%");

  lcd.setCursor(0, 1);

  lcd.print("W:C");
  lcd.print(weakestCell + 1);

  lcd.print(" S:C");
  lcd.print(strongestCell + 1);
}

// =====================================================
// LCD PAGE 4
// =====================================================

void displaySafetyPage()
{
  lcd.setCursor(0, 0);

  lcd.print("SAFETY:");

  if (safetyState ==
      SAFETY_NORMAL)
  {
    lcd.print("NORMAL");
  }

  else if (safetyState ==
           SAFETY_WARNING)
  {
    lcd.print("WARNING");
  }

  else if (safetyState ==
           SAFETY_TRIPPED)
  {
    lcd.print("TRIPPED");
  }

  else
  {
    lcd.print("RECOVERY");
  }

  lcd.setCursor(0, 1);

  lcd.print("RELAY:");

  if (relayState)
  {
    lcd.print("ON");
  }
  else
  {
    lcd.print("OFF");
  }
}

// =====================================================
// LCD PAGE 5
// =====================================================

void displayDiagnosticPage()
{
  lcd.setCursor(0, 0);

  lcd.print("SYSTEM:");

  if (runtimeMode ==
      RUNTIME_NORMAL)
  {
    lcd.print("NORMAL");
  }

  else if (runtimeMode ==
           RUNTIME_DEGRADED)
  {
    lcd.print("DEGRADE");
  }

  else if (runtimeMode ==
           RUNTIME_FAILSAFE)
  {
    lcd.print("FAILSAFE");
  }

  else
  {
    lcd.print("SHUTDOWN");
  }

  lcd.setCursor(0, 1);

  lcd.print("NO ACTIVE FAULT");
}

// =====================================================
// LCD NORMAL
// =====================================================

void displayNormalLCD()
{
  lcd.clear();

  lcd.setCursor(0, 0);

  lcd.print("RUNTIME:NORMAL");

  lcd.setCursor(0, 1);

  lcd.print("SYSTEM OK");
}

// =====================================================
// LCD DEGRADED
// =====================================================

void displayDegradedLCD()
{
  lcd.clear();

  lcd.setCursor(0, 0);

  lcd.print("RUNTIME:DEGRADE");

  lcd.setCursor(0, 1);

  if (invalidSensorFault)
  {
    lcd.print("SENSOR FAULT");
  }

  else if (frozenADCFault)
  {
    lcd.print("ADC FROZEN");
  }

  else if (rapidChangeFault)
  {
    lcd.print("RAPID DV");
  }

  else
  {
    lcd.print("CHECK SYSTEM");
  }
}

// =====================================================
// LCD FAILSAFE
// =====================================================

void displayFailsafeLCD()
{
  lcd.clear();

  lcd.setCursor(0, 0);

  lcd.print("RUNTIME:FAILSAFE");

  lcd.setCursor(0, 1);

  lcd.print("RELAY:");

  if (relayState)
  {
    lcd.print("ON");
  }
  else
  {
    lcd.print("OFF");
  }
}

// =====================================================
// LCD SHUTDOWN
// =====================================================

void displayShutdownLCD()
{
  lcd.clear();

  lcd.setCursor(0, 0);

  lcd.print("SYSTEM SHUTDOWN");

  lcd.setCursor(0, 1);

  lcd.print("RELAY:OFF");
}

// =====================================================
// LCD FAULT
// =====================================================

void displayFaultLCD()
{
  lcd.clear();

  if (underVoltageFault)
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT:UNDER V");

    lcd.setCursor(0, 1);

    lcd.print("C");
    lcd.print(weakestCell + 1);

    lcd.print(":");
    lcd.print(cellVoltage[weakestCell], 2);
    lcd.print("V");
  }

  else if (overVoltageFault)
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT:OVER V");

    lcd.setCursor(0, 1);

    lcd.print("CHECK CELL");
  }

  else if (rapidChangeFault)
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT:RAPID DV");

    lcd.setCursor(0, 1);

    lcd.print("CHECK BATTERY");
  }

  else if (sensorFault)
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT:SENSOR");

    lcd.setCursor(0, 1);

    lcd.print("SAFE MODE");
  }

  else
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT DETECTED");

    lcd.setCursor(0, 1);

    lcd.print("CHECK SYSTEM");
  }
}

// =====================================================
// SERIAL REPORT
// =====================================================

void displaySerialReport()
{
  Serial.println();

  Serial.println(
    "================================================"
  );

  Serial.println(
    "       EV BMS TASK 1 + 2 + 3 + 4"
  );

  Serial.println(
    "================================================"
  );

  // =================================================
  // BATTERY
  // =================================================

  Serial.println("BATTERY PARAMETERS");

  Serial.println(
    "------------------------------------------------"
  );

  for (int i = 0; i < NUM_CELLS; i++)
  {
    Serial.print("Cell ");

    Serial.print(i + 1);

    Serial.print(" Voltage      ");

    Serial.print(cellVoltage[i], 2);

    Serial.println(" V");
  }

  Serial.print("Pack Voltage             ");

  Serial.print(packVoltage, 2);

  Serial.println(" V");

  Serial.print("Average Voltage          ");

  Serial.print(averageVoltage, 2);

  Serial.println(" V");

  Serial.print("Imbalance                ");

  Serial.print(imbalancePercent, 2);

  Serial.println(" %");

  Serial.print("Strongest Cell           C");

  Serial.println(strongestCell + 1);

  Serial.print("Weakest Cell             C");

  Serial.println(weakestCell + 1);

  Serial.print("Battery Health           ");

  Serial.println(batteryHealth);

  // =================================================
  // SAFETY
  // =================================================

  Serial.println();

  Serial.println("SAFETY STATUS");

  Serial.println(
    "------------------------------------------------"
  );

  Serial.print("Under Voltage            ");

  Serial.println(
    underVoltageFault ? "YES" : "NO"
  );

  Serial.print("Over Voltage             ");

  Serial.println(
    overVoltageFault ? "YES" : "NO"
  );

  Serial.print("Rapid Voltage Change     ");

  Serial.println(
    rapidChangeFault ? "YES" : "NO"
  );

  Serial.print("Safety State             ");

  printSafetyState();

  Serial.print("Relay                    ");

  Serial.println(
    relayState ? "ON" : "OFF"
  );

  // =================================================
  // TASK 4
  // =================================================

  Serial.println();

  Serial.println("FAULT-TOLERANT RUNTIME");

  Serial.println(
    "------------------------------------------------"
  );

  Serial.print("Runtime Mode             ");

  printRuntimeMode();

  Serial.print("Invalid Sensor           ");

  Serial.println(
    invalidSensorFault ? "YES" : "NO"
  );

  Serial.print("Frozen ADC               ");

  Serial.println(
    frozenADCFault ? "YES" : "NO"
  );

  Serial.print("Relay Mismatch           ");

  Serial.println(
    relayMismatchFault ? "YES" : "NO"
  );

  // =================================================
  // FAULT LOG
  // =================================================

  Serial.println();

  Serial.println("FAULT LOG");

  Serial.println(
    "------------------------------------------------"
  );

  if (faultLogCount == 0)
  {
    Serial.println("No faults logged");
  }
  else
  {
    for (int i = 0;
         i < faultLogCount;
         i++)
    {
      Serial.print("#");

      Serial.print(i + 1);

      Serial.print("  ");

      Serial.print(
        faultLogs[i].timestamp
      );

      Serial.print(" ms  ");

      Serial.print(
        faultLogs[i].faultName
      );

      Serial.print("  [");

      Serial.print(
        faultLogs[i].mode
      );

      Serial.println("]");
    }
  }

  Serial.println();
  Serial.println("CLOUD TELEMETRY");
  Serial.println("------------------------------------------------");
  Serial.print("WiFi Status              ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "OFFLINE");
  Serial.print("Blynk Status             ");
  Serial.println(Blynk.connected() ? "CONNECTED" : "OFFLINE");
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("WiFi RSSI                ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
  Serial.print("Queued Cloud Events      ");
  Serial.println(cloudEventCount);

  Serial.println(
    "================================================"
  );
}

// =====================================================
// PRINT SAFETY STATE
// =====================================================

void printSafetyState()
{
  if (safetyState ==
      SAFETY_NORMAL)
  {
    Serial.println("NORMAL");
  }

  else if (safetyState ==
           SAFETY_WARNING)
  {
    Serial.println("WARNING");
  }

  else if (safetyState ==
           SAFETY_TRIPPED)
  {
    Serial.println("TRIPPED");
  }

  else
  {
    Serial.println("RECOVERY");
  }
}

// =====================================================
// PRINT RUNTIME MODE
// =====================================================

void printRuntimeMode()
{
  if (runtimeMode ==
      RUNTIME_NORMAL)
  {
    Serial.println("NORMAL");
  }

  else if (runtimeMode ==
           RUNTIME_DEGRADED)
  {
    Serial.println("DEGRADED");
  }

  else if (runtimeMode ==
           RUNTIME_FAILSAFE)
  {
    Serial.println("FAILSAFE");
  }

  else
  {
    Serial.println("SHUTDOWN");
  }
}
