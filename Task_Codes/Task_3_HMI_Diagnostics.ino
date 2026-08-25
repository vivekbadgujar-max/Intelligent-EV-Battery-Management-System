#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
// EV BATTERY MANAGEMENT SYSTEM
// TASK 1 + TASK 2 + TASK 3
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
// Hardware:
// ESP32 DevKit V1
// 4 x Potentiometers = simulated cells
// 16x2 I2C LCD
// 4 x Status LEDs
// Relay Module
// Buzzer
//
// Cell 1 -> GPIO34
// Cell 2 -> GPIO35
// Cell 3 -> GPIO32
// Cell 4 -> GPIO33
//
// LCD SDA -> GPIO21
// LCD SCL -> GPIO22
//
// Green LED  -> GPIO4
// Yellow LED -> GPIO2
// Orange LED -> GPIO15
// Red LED    -> GPIO5
//
// Relay -> GPIO26
// Buzzer -> GPIO27
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
// 2. LCD CONFIGURATION
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
// 7. TIMERS
// =====================================================

unsigned long previousSampleTime = 0;

unsigned long previousLCDTime = 0;

unsigned long previousSerialTime = 0;

unsigned long faultStartTime = 0;

unsigned long recoveryStartTime = 0;

unsigned long lastRelayChangeTime = 0;

unsigned long previousBuzzerTime = 0;

// =====================================================
// 8. BATTERY DATA
// =====================================================

float cellVoltage[NUM_CELLS];

float previousCellVoltage[NUM_CELLS];

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
// 9. SAFETY STATE MACHINE
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
// 10. FAULT TYPES
// =====================================================

bool underVoltageFault = false;

bool overVoltageFault = false;

bool rapidChangeFault = false;

bool sensorFault = false;

bool faultDetected = false;

bool faultConfirmed = false;

// =====================================================
// 11. RELAY STATE
// =====================================================

bool relayState = true;

// =====================================================
// 12. TASK 3 HMI
// =====================================================

int currentPage = 0;

const int TOTAL_PAGES = 5;

// Stores previous page
int previousPage = -1;

// Prevents continuous fault-screen redraw
bool faultScreenShown = false;

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // ---------------- ADC ----------------

  analogReadResolution(12);

  pinMode(CELL1_PIN, INPUT);
  pinMode(CELL2_PIN, INPUT);
  pinMode(CELL3_PIN, INPUT);
  pinMode(CELL4_PIN, INPUT);

  // ---------------- LEDs ----------------

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // ---------------- Relay ----------------

  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);

  relayState = true;

  // ---------------- Buzzer ----------------

  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  // ---------------- I2C ----------------

  Wire.begin(SDA_PIN, SCL_PIN);

  // ---------------- LCD ----------------

  lcd.init();

  lcd.backlight();

  lcd.clear();

  lcd.setCursor(0, 0);

  lcd.print("EV BMS TASK 3");

  lcd.setCursor(0, 1);

  lcd.print("Smart HMI");

  // Startup delay only
  // Not used in safety/HMI operation
  delay(1500);

  // ---------------- Initial reading ----------------

  readCellVoltages();

  calculateBatteryParameters();

  identifyStrongestWeakest();

  calculateImbalance();

  classifyBatteryHealth();

  // Store initial voltage

  for (int i = 0; i < NUM_CELLS; i++)
  {
    previousCellVoltage[i] = cellVoltage[i];
  }

  updateLEDs();

  updateLCD();

  displaySerialReport();

  Serial.println();

  Serial.println("======================================");

  Serial.println("TASK 3 HMI READY");

  Serial.println("======================================");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  unsigned long currentTime = millis();

  // ===================================================
  // 1. BATTERY + SAFETY PROCESSING
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

    updateLEDs();
  }

  // ===================================================
  // 2. TASK 3 HMI
  // ===================================================

  if (currentTime - previousLCDTime >= LCD_INTERVAL)
  {
    previousLCDTime = currentTime;

    // Rotate only when there is NO fault

    if (!faultDetected)
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
  // 3. SERIAL REPORT
  // ===================================================

  if (currentTime - previousSerialTime >= SERIAL_INTERVAL)
  {
    previousSerialTime = currentTime;

    displaySerialReport();
  }
}

// =====================================================
// TASK 1
// READ FOUR CELL VOLTAGES
// =====================================================

void readCellVoltages()
{
  int rawCell1 = analogRead(CELL1_PIN);

  int rawCell2 = analogRead(CELL2_PIN);

  int rawCell3 = analogRead(CELL3_PIN);

  int rawCell4 = analogRead(CELL4_PIN);

  cellVoltage[0] =
    convertADCToCellVoltage(rawCell1);

  cellVoltage[1] =
    convertADCToCellVoltage(rawCell2);

  cellVoltage[2] =
    convertADCToCellVoltage(rawCell3);

  cellVoltage[3] =
    convertADCToCellVoltage(rawCell4);
}

// =====================================================
// TASK 1
// ADC TO SIMULATED CELL VOLTAGE
// =====================================================

float convertADCToCellVoltage(int rawADC)
{
  float adcVoltage;

  adcVoltage =
    ((float)rawADC / ADC_MAX)
    * ADC_MAX_VOLTAGE;

  float simulatedVoltage;

  simulatedVoltage =
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
// CALCULATE BATTERY PARAMETERS
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
// IDENTIFY STRONGEST / WEAKEST CELL
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
// CALCULATE IMBALANCE
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
// DETECT SAFETY CONDITIONS
// =====================================================

void detectSafetyConditions()
{
  underVoltageFault = false;

  overVoltageFault = false;

  rapidChangeFault = false;

  sensorFault = false;

  for (int i = 0; i < NUM_CELLS; i++)
  {
    // Sensor anomaly

    if (cellVoltage[i] < CELL_MIN_VOLTAGE ||
        cellVoltage[i] > CELL_MAX_VOLTAGE)
    {
      sensorFault = true;
    }

    // Under-voltage

    if (cellVoltage[i] < UV_THRESHOLD)
    {
      underVoltageFault = true;
    }

    // Over-voltage

    if (cellVoltage[i] > OV_THRESHOLD)
    {
      overVoltageFault = true;
    }

    // Rapid voltage change

    float voltageChange =
      fabs(cellVoltage[i] -
           previousCellVoltage[i]);

    if (voltageChange >
        RAPID_CHANGE_THRESHOLD)
    {
      rapidChangeFault = true;
    }
  }

  // Store current readings

  for (int i = 0; i < NUM_CELLS; i++)
  {
    previousCellVoltage[i] =
      cellVoltage[i];
  }

  // Overall fault

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
    // =================================================
    // NORMAL
    // =================================================

    case SAFETY_NORMAL:

      if (faultDetected)
      {
        safetyState = SAFETY_WARNING;

        faultStartTime = now;
      }

      break;

    // =================================================
    // WARNING
    // =================================================

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

    // =================================================
    // TRIPPED
    // =================================================

    case SAFETY_TRIPPED:

      if (!faultDetected)
      {
        recoveryStartTime = now;

        safetyState = SAFETY_RECOVERY;
      }

      break;

    // =================================================
    // RECOVERY
    // =================================================

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

  // Relay OFF during warning/tripped

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

        Serial.println(
          ">>> RELAY CUT-OFF <<<"
        );
      }
    }
  }

  // Relay ON during normal

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

        Serial.println(
          ">>> RELAY RESTORED <<<"
        );
      }
    }
  }
}

// =====================================================
// TASK 2
// BUZZER CONTROL
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
// LED HEALTH INDICATION
// =====================================================

void updateLEDs()
{
  digitalWrite(LED_GREEN, LOW);

  digitalWrite(LED_YELLOW, LOW);

  digitalWrite(LED_ORANGE, LOW);

  digitalWrite(LED_RED, LOW);

  // Safety fault gets priority

  if (safetyState ==
      SAFETY_WARNING ||
      safetyState ==
      SAFETY_TRIPPED)
  {
    digitalWrite(LED_RED, HIGH);

    return;
  }

  // Battery health indication

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
// TASK 3
// LCD HMI MANAGER
// =====================================================

void updateLCD()
{
  // ===================================================
  // FAULT PRIORITY
  // ===================================================

  if (faultDetected)
  {
    if (!faultScreenShown)
    {
      displayFaultPriority();

      faultScreenShown = true;
    }

    return;
  }

  // ===================================================
  // FAULT CLEARED
  // ===================================================

  if (faultScreenShown)
  {
    lcd.clear();

    faultScreenShown = false;

    previousPage = -1;
  }

  // ===================================================
  // NORMAL HMI
  // ===================================================

  // Only redraw when page changes

  if (currentPage == previousPage)
  {
    return;
  }

  previousPage = currentPage;

  lcd.clear();

  // ===================================================
  // PAGE 1
  // LIVE CELL DATA
  // ===================================================

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

  // ===================================================
  // PAGE 2
  // PACK ANALYTICS
  // ===================================================

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

  // ===================================================
  // PAGE 3
  // CELL ANALYTICS
  // ===================================================

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

  // ===================================================
  // PAGE 4
  // PROTECTION STATUS
  // ===================================================

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

    lcd.print(" B:");

    if (safetyState ==
        SAFETY_WARNING ||
        safetyState ==
        SAFETY_TRIPPED)
    {
      lcd.print("ON");
    }
    else
    {
      lcd.print("OFF");
    }
  }

  // ===================================================
  // PAGE 5
  // DIAGNOSTICS
  // ===================================================

  else if (currentPage == 4)
  {
    lcd.setCursor(0, 0);

    lcd.print("DIAGNOSTICS");

    lcd.setCursor(0, 1);

    if (!faultDetected)
    {
      lcd.print("SYSTEM OK");
    }
    else
    {
      lcd.print("CHECK FAULT");
    }
  }
}

// =====================================================
// TASK 3
// FAULT PRIORITY DISPLAY
// =====================================================

void displayFaultPriority()
{
  lcd.clear();

  // ===================================================
  // UNDER VOLTAGE
  // ===================================================

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

  // ===================================================
  // OVER VOLTAGE
  // ===================================================

  else if (overVoltageFault)
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT: OVER V");

    lcd.setCursor(0, 1);

    lcd.print("CHECK CELL");
  }

  // ===================================================
  // RAPID VOLTAGE CHANGE
  // ===================================================

  else if (rapidChangeFault)
  {
    lcd.setCursor(0, 0);

    lcd.print("FAULT: RAPID DV");

    lcd.setCursor(0, 1);

    lcd.print("CHECK BATTERY");
  }

  // ===================================================
  // SENSOR FAULT
  // ===================================================

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
    "       EV BMS TASK 1 + 2 + 3"
  );

  Serial.println(
    "================================================"
  );

  Serial.println(
    "Parameter                 Value"
  );

  Serial.println(
    "------------------------------------------------"
  );

  // Cell voltages

  Serial.print("Cell 1 Voltage            ");

  Serial.print(cellVoltage[0], 2);

  Serial.println(" V");

  Serial.print("Cell 2 Voltage            ");

  Serial.print(cellVoltage[1], 2);

  Serial.println(" V");

  Serial.print("Cell 3 Voltage            ");

  Serial.print(cellVoltage[2], 2);

  Serial.println(" V");

  Serial.print("Cell 4 Voltage            ");

  Serial.print(cellVoltage[3], 2);

  Serial.println(" V");

  Serial.println(
    "------------------------------------------------"
  );

  // Battery parameters

  Serial.print("Pack Voltage              ");

  Serial.print(packVoltage, 2);

  Serial.println(" V");

  Serial.print("Average Voltage           ");

  Serial.print(averageVoltage, 2);

  Serial.println(" V");

  Serial.print("Maximum Voltage           ");

  Serial.print(maxVoltage, 2);

  Serial.println(" V");

  Serial.print("Minimum Voltage           ");

  Serial.print(minVoltage, 2);

  Serial.println(" V");

  Serial.print("Imbalance Voltage         ");

  Serial.print(imbalanceVoltage, 2);

  Serial.println(" V");

  Serial.print("Imbalance Percentage      ");

  Serial.print(imbalancePercent, 2);

  Serial.println(" %");

  Serial.print("Strongest Cell            Cell ");

  Serial.println(strongestCell + 1);

  Serial.print("Weakest Cell              Cell ");

  Serial.println(weakestCell + 1);

  Serial.print("Battery Health            ");

  Serial.println(batteryHealth);

  Serial.println(
    "------------------------------------------------"
  );

  // Safety information

  Serial.print("Under Voltage Fault      ");

  Serial.println(
    underVoltageFault ? "YES" : "NO"
  );

  Serial.print("Over Voltage Fault       ");

  Serial.println(
    overVoltageFault ? "YES" : "NO"
  );

  Serial.print("Rapid Change Fault       ");

  Serial.println(
    rapidChangeFault ? "YES" : "NO"
  );

  Serial.print("Sensor Fault             ");

  Serial.println(
    sensorFault ? "YES" : "NO"
  );

  Serial.print("Fault Confirmed          ");

  Serial.println(
    faultConfirmed ? "YES" : "NO"
  );

  Serial.print("Relay State              ");

  Serial.println(
    relayState ? "ON" : "OFF"
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

  Serial.println(
    "================================================"
  );
}
