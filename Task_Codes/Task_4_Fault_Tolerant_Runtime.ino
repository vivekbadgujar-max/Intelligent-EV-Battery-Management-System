#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
// EV BATTERY MANAGEMENT SYSTEM
// TASK 1 + TASK 2 + TASK 3 + TASK 4
//
// TASK 1:
// Adaptive Multi-Cell Battery Intelligence Engine
//
// TASK 2:
// Event-Driven Safety Protection Kernel
//
// TASK 3:
// Intelligent Embedded HMI & Diagnostic Interface
//
// TASK 4:
// Fault-Tolerant Embedded Runtime System
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

const float HEALTHY_LIMIT = 2.0;
const float MINOR_LIMIT = 5.0;
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
const unsigned long LCD_INTERVAL = 3000;
const unsigned long SERIAL_INTERVAL = 2000;

const unsigned long FAULT_CONFIRM_TIME = 500;
const unsigned long RECOVERY_TIME = 5000;
const unsigned long RELAY_LOCK_TIME = 3000;
const unsigned long BUZZER_INTERVAL = 300;

// =====================================================
// TASK 4 TIMING
// =====================================================

// Same ADC value for this many samples
// = frozen ADC suspicion

const int FROZEN_LIMIT = 10;

// Time after which a degraded condition
// can return to NORMAL

const unsigned long DEGRADED_RECOVERY_TIME = 3000;

// =====================================================
// 7. TIMERS
// =====================================================

unsigned long previousSampleTime = 0;
unsigned long previousLCDTime = 0;
unsigned long previousSerialTime = 0;

unsigned long faultStartTime = 0;
unsigned long recoveryStartTime = 0;

unsigned long lastRelayChangeTime = 0;
unsigned long previousBuzzerTime = 0;

unsigned long degradedStartTime = 0;

// =====================================================
// 8. BATTERY DATA
// =====================================================

float cellVoltage[NUM_CELLS];

float previousCellVoltage[NUM_CELLS];

int cellADC[NUM_CELLS];

int previousADC[NUM_CELLS];

int frozenCount[NUM_CELLS];

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
// 9. TASK 2 SAFETY STATE
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
// 10. TASK 2 FAULT FLAGS
// =====================================================

bool underVoltageFault = false;
bool overVoltageFault = false;
bool rapidChangeFault = false;
bool sensorFault = false;

bool faultDetected = false;
bool faultConfirmed = false;

// =====================================================
// 11. TASK 4 RUNTIME MODES
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
// 12. TASK 4 FAULT TYPES
// =====================================================

bool invalidSensorFault = false;
bool frozenADCFault = false;
bool relayMismatchFault = false;

// =====================================================
// 13. RELAY
// =====================================================

bool relayState = true;

// =====================================================
// 14. TASK 4 SOFTWARE RELAY MISMATCH
// =====================================================

// FALSE = normal operation
// TRUE  = simulate relay mismatch

bool simulateRelayMismatch = false;

// =====================================================
// 15. TASK 3 HMI
// =====================================================

int currentPage = 0;

const int TOTAL_PAGES = 5;

int previousPage = -1;

bool faultScreenShown = false;

// =====================================================
// 16. TASK 4 FAULT LOG
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
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

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

  // =================================================
  // RELAY
  // =================================================

  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);

  relayState = true;

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
  lcd.print("Fault Runtime");

  // Startup display only
  delay(1500);

  // =================================================
  // INITIAL BATTERY READING
  // =================================================

  readCellVoltages();

  calculateBatteryParameters();

  identifyStrongestWeakest();

  calculateImbalance();

  classifyBatteryHealth();

  // Store initial ADC and voltage values

  for (int i = 0; i < NUM_CELLS; i++)
  {
    previousCellVoltage[i] = cellVoltage[i];

    previousADC[i] = cellADC[i];

    frozenCount[i] = 0;
  }

  updateLEDs();

  updateLCD();

  displaySerialReport();

  Serial.println();
  Serial.println("======================================");
  Serial.println("TASK 4 FAULT-TOLERANT RUNTIME READY");
  Serial.println("======================================");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  unsigned long currentTime = millis();

  // ===================================================
  // 1. BATTERY + SAFETY + RUNTIME
  // ===================================================

  if (currentTime - previousSampleTime >= SAMPLE_INTERVAL)
  {
    previousSampleTime = currentTime;

    // ---------------- TASK 1 ----------------

    readCellVoltages();

    calculateBatteryParameters();

    identifyStrongestWeakest();

    calculateImbalance();

    classifyBatteryHealth();

    // ---------------- TASK 2 ----------------

    detectSafetyConditions();

    updateSafetyState();

    updateRelay();

    updateBuzzer();

    // ---------------- TASK 4 ----------------

    checkSensorHealth();

    checkFrozenADC();

    checkRelayMismatch();

    updateRuntimeMode();

    updateLEDs();
  }

  // ===================================================
  // 2. HMI
  // ===================================================

  if (currentTime - previousLCDTime >= LCD_INTERVAL)
  {
    previousLCDTime = currentTime;

    if (!faultDetected &&
        runtimeMode == RUNTIME_NORMAL)
    {
      currentPage++;

      if (currentPage >= TOTAL_PAGES)
      {
        currentPage = 0;
      }
    }

    updateLCD();
  }

  // ===================================================
  // 3. SERIAL
  // ===================================================

  if (currentTime - previousSerialTime >= SERIAL_INTERVAL)
  {
    previousSerialTime = currentTime;

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
// ADC TO CELL VOLTAGE
// =====================================================

float convertADCToCellVoltage(int rawADC)
{
  float adcVoltage =
    ((float)rawADC / ADC_MAX)
    * ADC_MAX_VOLTAGE;

  float simulatedVoltage =
    CELL_MIN_VOLTAGE
    +
    (adcVoltage / ADC_MAX_VOLTAGE)
    *
    (CELL_MAX_VOLTAGE -
     CELL_MIN_VOLTAGE);

  return simulatedVoltage;
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
// STRONGEST / WEAKEST CELL
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
// HEALTH
// =====================================================

void classifyBatteryHealth()
{
  if (imbalancePercent < HEALTHY_LIMIT)
  {
    batteryHealth = "HEALTHY";
  }

  else if (imbalancePercent < MINOR_LIMIT)
  {
    batteryHealth = "MINOR IMBALANCE";
  }

  else if (imbalancePercent < CRITICAL_LIMIT)
  {
    batteryHealth = "CRITICAL IMBALANCE";
  }

  else
  {
    batteryHealth = "PACK FAILURE";
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
    if (cellVoltage[i] < CELL_MIN_VOLTAGE ||
        cellVoltage[i] > CELL_MAX_VOLTAGE)
    {
      sensorFault = true;
    }

    if (cellVoltage[i] < UV_THRESHOLD)
    {
      underVoltageFault = true;
    }

    if (cellVoltage[i] > OV_THRESHOLD)
    {
      overVoltageFault = true;
    }

    float voltageChange =
      fabs(cellVoltage[i] -
           previousCellVoltage[i]);

    if (voltageChange >
        RAPID_CHANGE_THRESHOLD)
    {
      rapidChangeFault = true;
    }
  }

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
        recoveryStartTime = now;

        safetyState = SAFETY_RECOVERY;
      }

      break;

    case SAFETY_RECOVERY:

      if (faultDetected)
      {
        safetyState = SAFETY_TRIPPED;

        break;
      }

      if (now - recoveryStartTime >=
          RECOVERY_TIME)
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

  if (safetyState == SAFETY_TRIPPED ||
      safetyState == SAFETY_WARNING)
  {
    if (relayState == true)
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

  else if (safetyState ==
           SAFETY_NORMAL)
  {
    if (relayState == false)
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

  if (safetyState ==
      SAFETY_WARNING ||
      safetyState ==
      SAFETY_TRIPPED)
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
    // Extreme ADC values simulate
    // disconnected/invalid sensor

    if (cellADC[i] <= 5 ||
        cellADC[i] >= 4090)
    {
      invalidSensorFault = true;

      String fault =
        "C" + String(i + 1)
        + " INVALID SENSOR";

      logFault(fault, "DEGRADED");
    }
  }
}

// =====================================================
// TASK 4
// FROZEN ADC DETECTION
// =====================================================

void checkFrozenADC()
{
  frozenADCFault = false;

  for (int i = 0; i < NUM_CELLS; i++)
  {
    if (cellADC[i] == previousADC[i])
    {
      frozenCount[i]++;
    }
    else
    {
      frozenCount[i] = 0;
    }

    if (frozenCount[i] >= FROZEN_LIMIT)
    {
      frozenADCFault = true;

      String fault =
        "C" + String(i + 1)
        + " ADC FROZEN";

      logFault(fault, "DEGRADED");

      // Prevent excessive counter growth
      frozenCount[i] = FROZEN_LIMIT;
    }

    previousADC[i] = cellADC[i];
  }
}

// =====================================================
// TASK 4
// RELAY MISMATCH
// =====================================================

void checkRelayMismatch()
{
  relayMismatchFault = false;

  // Software simulation only
  if (simulateRelayMismatch)
  {
    relayMismatchFault = true;

    logFault(
      "RELAY MISMATCH",
      "FAILSAFE"
    );
  }
}

// =====================================================
// TASK 4
// RUNTIME MODE
// =====================================================

void updateRuntimeMode()
{
  unsigned long now = millis();

  // =================================================
  // CRITICAL FAULT → FAILSAFE
  // =================================================

  if (relayMismatchFault ||
      overVoltageFault ||
      underVoltageFault)
  {
    if (runtimeMode != RUNTIME_FAILSAFE)
    {
      logFault(
        "CRITICAL SAFETY FAULT",
        "FAILSAFE"
      );
    }

    runtimeMode = RUNTIME_FAILSAFE;

    degradedStartTime = 0;

    return;
  }

  // =================================================
  // SHUTDOWN CONDITION
  // =================================================

  if (sensorFault &&
      invalidSensorFault &&
      frozenADCFault)
  {
    runtimeMode = RUNTIME_SHUTDOWN;

    logFault(
      "MULTIPLE SYSTEM FAILURE",
      "SHUTDOWN"
    );

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
      runtimeMode = RUNTIME_DEGRADED;

      degradedStartTime = now;

      logFault(
        "SYSTEM DEGRADED",
        "DEGRADED"
      );
    }

    return;
  }

  // =================================================
  // RECOVERY FROM FAILSAFE
  // =================================================

  if (runtimeMode ==
      RUNTIME_FAILSAFE)
  {
    if (!faultDetected &&
        !relayMismatchFault)
    {
      if (degradedStartTime == 0)
      {
        degradedStartTime = now;
      }

      if (now - degradedStartTime >=
          RECOVERY_TIME)
      {
        runtimeMode =
          RUNTIME_NORMAL;

        degradedStartTime = 0;

        logFault(
          "FAILSAFE RECOVERED",
          "NORMAL"
        );
      }
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
      if (degradedStartTime == 0)
      {
        degradedStartTime = now;
      }

      if (now - degradedStartTime >=
          DEGRADED_RECOVERY_TIME)
      {
        runtimeMode =
          RUNTIME_NORMAL;

        degradedStartTime = 0;

        logFault(
          "DEGRADED RECOVERED",
          "NORMAL"
        );
      }
    }

    return;
  }

  // =================================================
  // NORMAL
  // =================================================

  if (!faultDetected &&
      !invalidSensorFault &&
      !frozenADCFault &&
      !relayMismatchFault)
  {
    runtimeMode = RUNTIME_NORMAL;
  }
}

// =====================================================
// TASK 4
// FAULT LOGGING
// =====================================================

void logFault(String faultName,
              String mode)
{
  // Avoid filling the log repeatedly
  // with the same event every sample

  static String lastFault = "";

  if (faultName == lastFault)
  {
    return;
  }

  lastFault = faultName;

  if (faultLogCount < MAX_FAULT_LOGS)
  {
    faultLogs[faultLogCount].timestamp =
      millis();

    faultLogs[faultLogCount].faultName =
      faultName;

    faultLogs[faultLogCount].mode =
      mode;

    faultLogCount++;
  }

  Serial.println();
  Serial.println("******** FAULT LOG ********");

  Serial.print("Time  : ");
  Serial.print(millis());
  Serial.println(" ms");

  Serial.print("Fault : ");
  Serial.println(faultName);

  Serial.print("Mode  : ");
  Serial.println(mode);

  Serial.println("***************************");
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

  // =================================================
  // TASK 4 PRIORITY
  // =================================================

  if (runtimeMode ==
      RUNTIME_SHUTDOWN)
  {
    digitalWrite(LED_RED, HIGH);

    return;
  }

  if (runtimeMode ==
      RUNTIME_FAILSAFE)
  {
    digitalWrite(LED_RED, HIGH);

    return;
  }

  if (runtimeMode ==
      RUNTIME_DEGRADED)
  {
    digitalWrite(LED_YELLOW, HIGH);

    return;
  }

  // =================================================
  // TASK 2 SAFETY
  // =================================================

  if (safetyState ==
      SAFETY_WARNING ||
      safetyState ==
      SAFETY_TRIPPED)
  {
    digitalWrite(LED_RED, HIGH);

    return;
  }

  // =================================================
  // TASK 1 HEALTH
  // =================================================

  if (batteryHealth == "HEALTHY")
  {
    digitalWrite(LED_GREEN, HIGH);
  }

  else if (batteryHealth ==
           "MINOR IMBALANCE")
  {
    digitalWrite(LED_YELLOW, HIGH);
  }

  else if (batteryHealth ==
           "CRITICAL IMBALANCE")
  {
    digitalWrite(LED_ORANGE, HIGH);
  }

  else
  {
    digitalWrite(LED_RED, HIGH);
  }
}

// =====================================================
// TASK 3 + TASK 4
// LCD HMI
// =====================================================

void updateLCD()
{
  // =================================================
  // TASK 4 SHUTDOWN
  // =================================================

  if (runtimeMode ==
      RUNTIME_SHUTDOWN)
  {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("SYSTEM SHUTDOWN");

    lcd.setCursor(0, 1);
    lcd.print("CHECK FAULT");

    return;
  }

  // =================================================
  // TASK 4 FAILSAFE
  // =================================================

  if (runtimeMode ==
      RUNTIME_FAILSAFE)
  {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("RUNTIME:FAILSAFE");

    lcd.setCursor(0, 1);
    lcd.print("RELAY:OFF");

    return;
  }

  // =================================================
  // TASK 4 DEGRADED
  // =================================================

  if (runtimeMode ==
      RUNTIME_DEGRADED)
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

    else
    {
      lcd.print("CHECK SYSTEM");
    }

    return;
  }

  // =================================================
  // TASK 3 FAULT PRIORITY
  // =================================================

  if (faultDetected)
  {
    if (!faultScreenShown)
    {
      displayFaultPriority();

      faultScreenShown = true;
    }

    return;
  }

  // Fault cleared

  if (faultScreenShown)
  {
    lcd.clear();

    faultScreenShown = false;

    previousPage = -1;
  }

  // =================================================
  // NORMAL HMI
  // =================================================

  if (currentPage == previousPage)
  {
    return;
  }

  previousPage = currentPage;

  lcd.clear();

  // =================================================
  // PAGE 1
  // =================================================

  if (currentPage == 0)
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

  // =================================================
  // PAGE 2
  // =================================================

  else if (currentPage == 1)
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

  // =================================================
  // PAGE 3
  // =================================================

  else if (currentPage == 2)
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

  // =================================================
  // PAGE 4
  // =================================================

  else if (currentPage == 3)
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
      lcd.print("ON ");
    else
      lcd.print("OFF");
  }

  // =================================================
  // PAGE 5
  // =================================================

  else if (currentPage == 4)
  {
    lcd.setCursor(0, 0);

    lcd.print("DIAGNOSTICS");

    lcd.setCursor(0, 1);

    lcd.print("SYSTEM OK");
  }
}

// =====================================================
// TASK 3
// FAULT PRIORITY DISPLAY
// =====================================================

void displayFaultPriority()
{
  lcd.clear();

  if (underVoltageFault)
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT: UNDER V");

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

    lcd.print("FAULT: OVER V");

    lcd.setCursor(0, 1);

    lcd.print("CHECK CELL");
  }

  else if (rapidChangeFault)
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT: RAPID DV");

    lcd.setCursor(0, 1);

    lcd.print("CHECK BATTERY");
  }

  else if (sensorFault)
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT: SENSOR");

    lcd.setCursor(0, 1);

    lcd.print("SAFE MODE");
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
  // TASK 2
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
    for (int i = 0; i < faultLogCount; i++)
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

  Serial.println(
    "================================================"
  );
}
