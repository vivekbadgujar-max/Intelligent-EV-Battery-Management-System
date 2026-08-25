#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
// ADAPTIVE MULTI-CELL BATTERY INTELLIGENCE ENGINE
// TASK 1
//
// Hardware:
// ESP32 DevKit V1
// 4 x Potentiometers -> simulated 4 cells
// 16x2 I2C LCD
// 4 LEDs -> battery health indication
//
// ADC:
// Cell 1 -> GPIO34
// Cell 2 -> GPIO35
// Cell 3 -> GPIO32
// Cell 4 -> GPIO33
// =====================================================

// =====================================================
// 1. PIN CONFIGURATION
// =====================================================

// Cell ADC pins
#define CELL1_PIN 34
#define CELL2_PIN 35
#define CELL3_PIN 32
#define CELL4_PIN 33

// LCD I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// Health LEDs
#define LED_GREEN  4
#define LED_YELLOW 2
#define LED_ORANGE 15
#define LED_RED    5

// =====================================================
// 2. LCD CONFIGURATION
// =====================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
// 3. BATTERY CONFIGURATION
// =====================================================

#define NUM_CELLS 4

// ESP32 ADC
const int ADC_MAX = 4095;
const float ADC_MAX_VOLTAGE = 3.3;

// Simulated cell voltage range
const float CELL_MIN_VOLTAGE = 2.5;
const float CELL_MAX_VOLTAGE = 4.2;

// =====================================================
// 4. HEALTH THRESHOLDS
// =====================================================
//
// These are demonstration thresholds for the project.
// They are NOT universal lithium battery safety limits.
//

const float HEALTHY_LIMIT = 2.0;
const float MINOR_LIMIT = 5.0;
const float CRITICAL_LIMIT = 10.0;

// =====================================================
// 5. TIMING
// =====================================================

const unsigned long SAMPLE_INTERVAL = 1000;
const unsigned long LCD_INTERVAL = 3000;
const unsigned long SERIAL_INTERVAL = 2000;

unsigned long previousSampleTime = 0;
unsigned long previousLCDTime = 0;
unsigned long previousSerialTime = 0;

// =====================================================
// 6. LCD PAGES
// =====================================================

int currentPage = 0;

const int TOTAL_PAGES = 4;

// =====================================================
// 7. BATTERY DATA
// =====================================================

float cellVoltage[NUM_CELLS];

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
// 8. SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // ESP32 ADC resolution
  analogReadResolution(12);

  // ADC input pins
  pinMode(CELL1_PIN, INPUT);
  pinMode(CELL2_PIN, INPUT);
  pinMode(CELL3_PIN, INPUT);
  pinMode(CELL4_PIN, INPUT);

  // LED pins
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // Turn LEDs OFF initially
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_RED, LOW);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // LCD
  lcd.init();
  lcd.backlight();

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("EV BMS TASK 1");

  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  delay(1000);

  // First measurement
  readCellVoltages();
  calculateBatteryParameters();
  identifyStrongestWeakest();
  calculateImbalance();
  classifyBatteryHealth();
  updateLEDs();

  displayLCD();
  displaySerialTable();

  Serial.println();
  Serial.println("System started successfully.");
}

// =====================================================
// 9. MAIN LOOP
// =====================================================

void loop()
{
  unsigned long currentTime = millis();

  // ---------------------------------------------------
  // Battery measurement
  // ---------------------------------------------------

  if (currentTime - previousSampleTime >= SAMPLE_INTERVAL)
  {
    previousSampleTime = currentTime;

    readCellVoltages();

    calculateBatteryParameters();

    identifyStrongestWeakest();

    calculateImbalance();

    classifyBatteryHealth();

    updateLEDs();
  }

  // ---------------------------------------------------
  // LCD automatic screen rotation
  // ---------------------------------------------------

  if (currentTime - previousLCDTime >= LCD_INTERVAL)
  {
    previousLCDTime = currentTime;

    currentPage++;

    if (currentPage >= TOTAL_PAGES)
    {
      currentPage = 0;
    }

    displayLCD();
  }

  // ---------------------------------------------------
  // Serial table
  // ---------------------------------------------------

  if (currentTime - previousSerialTime >= SERIAL_INTERVAL)
  {
    previousSerialTime = currentTime;

    displaySerialTable();
  }
}

// =====================================================
// 10. READ FOUR CELL VOLTAGES
// =====================================================

void readCellVoltages()
{
  int rawCell1 = analogRead(CELL1_PIN);
  int rawCell2 = analogRead(CELL2_PIN);
  int rawCell3 = analogRead(CELL3_PIN);
  int rawCell4 = analogRead(CELL4_PIN);

  cellVoltage[0] = convertADCToCellVoltage(rawCell1);
  cellVoltage[1] = convertADCToCellVoltage(rawCell2);
  cellVoltage[2] = convertADCToCellVoltage(rawCell3);
  cellVoltage[3] = convertADCToCellVoltage(rawCell4);
}

// =====================================================
// 11. ADC TO SIMULATED CELL VOLTAGE
// =====================================================

float convertADCToCellVoltage(int rawADC)
{
  float adcVoltage;

  adcVoltage =
    ((float)rawADC / ADC_MAX) * ADC_MAX_VOLTAGE;

  float simulatedVoltage;

  simulatedVoltage =
    CELL_MIN_VOLTAGE +
    (adcVoltage / ADC_MAX_VOLTAGE) *
    (CELL_MAX_VOLTAGE - CELL_MIN_VOLTAGE);

  return simulatedVoltage;
}

// =====================================================
// 12. CALCULATE BATTERY PARAMETERS
// =====================================================

void calculateBatteryParameters()
{
  packVoltage = 0.0;

  // Calculate pack voltage
  for (int i = 0; i < NUM_CELLS; i++)
  {
    packVoltage += cellVoltage[i];
  }

  // Average cell voltage
  averageVoltage =
    packVoltage / NUM_CELLS;

  // Initial maximum and minimum
  maxVoltage = cellVoltage[0];
  minVoltage = cellVoltage[0];

  // Find maximum and minimum
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
// 13. IDENTIFY STRONGEST AND WEAKEST CELL
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
// 14. CALCULATE IMBALANCE
// =====================================================

void calculateImbalance()
{
  // Difference between highest and lowest cell
  imbalanceVoltage =
    maxVoltage - minVoltage;

  // Percentage imbalance
  if (averageVoltage > 0)
  {
    imbalancePercent =
      (imbalanceVoltage /
       averageVoltage) * 100.0;
  }
  else
  {
    imbalancePercent = 0.0;
  }
}

// =====================================================
// 15. BATTERY HEALTH CLASSIFICATION
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
// 16. UPDATE HEALTH LEDs
// =====================================================

void updateLEDs()
{
  // Turn everything OFF
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_RED, LOW);

  if (batteryHealth == "HEALTHY")
  {
    digitalWrite(LED_GREEN, HIGH);
  }

  else if (batteryHealth == "MINOR IMBALANCE")
  {
    digitalWrite(LED_YELLOW, HIGH);
  }

  else if (batteryHealth == "CRITICAL IMBALANCE")
  {
    digitalWrite(LED_ORANGE, HIGH);
  }

  else
  {
    digitalWrite(LED_RED, HIGH);
  }
}

// =====================================================
// 17. LCD DISPLAY
// =====================================================

void displayLCD()
{
  lcd.clear();

  // ---------------------------------------------------
  // PAGE 1: CELL VOLTAGES
  // ---------------------------------------------------

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

  // ---------------------------------------------------
  // PAGE 2: PACK PARAMETERS
  // ---------------------------------------------------

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

  // ---------------------------------------------------
  // PAGE 3: IMBALANCE
  // ---------------------------------------------------

  else if (currentPage == 2)
  {
    lcd.setCursor(0, 0);

    lcd.print("IMB:");
    lcd.print(imbalancePercent, 2);

    lcd.print("%");

    lcd.setCursor(0, 1);

    lcd.print("W:C");
    lcd.print(weakestCell + 1);

    lcd.print(" S:C");
    lcd.print(strongestCell + 1);
  }

  // ---------------------------------------------------
  // PAGE 4: HEALTH
  // ---------------------------------------------------

  else if (currentPage == 3)
  {
    lcd.setCursor(0, 0);

    if (batteryHealth == "HEALTHY")
    {
      lcd.print("STATUS: HEALTHY");
    }

    else if (batteryHealth == "MINOR IMBALANCE")
    {
      lcd.print("STATUS: MINOR");
    }

    else if (batteryHealth == "CRITICAL IMBALANCE")
    {
      lcd.print("STATUS: CRITICAL");
    }

    else
    {
      lcd.print("STATUS: FAILURE");
    }

    lcd.setCursor(0, 1);

    lcd.print("DV:");
    lcd.print(imbalanceVoltage, 2);

    lcd.print("V");
  }
}

// =====================================================
// 18. SERIAL MONITOR TABLE
// =====================================================

void displaySerialTable()
{
  Serial.println();
  Serial.println("================================================");
  Serial.println("       ADAPTIVE BATTERY INTELLIGENCE");
  Serial.println("================================================");

  Serial.println("Parameter                 Value");
  Serial.println("------------------------------------------------");

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

  Serial.println("------------------------------------------------");

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

  Serial.println("------------------------------------------------");
  Serial.println("              END OF SAMPLE");
  Serial.println("================================================");
}
