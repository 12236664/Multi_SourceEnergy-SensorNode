,/**
 * ============================================================
 * Multi-Source Energy-Harvesting Autonomous Sensor Node
 * The Copperbelt University — Group Seven
 * ============================================================
 * 
 * Description:
 *   Full firmware for an autonomous sensor node that harvests
 *   ambient energy from Solar, Magnetic, and RF sources.
 *   Stores energy in a supercapacitor, monitors storage levels4 
 *   via ADC, and adapts duty-cycling based on energy state.
 *   Transmits telemetry via USB-UART (CSV format) to a laptop.
 * 
 * Hardware:
 *   - MCU         : Arduino Nano / ATmega328P (3.3V or 5V)
 *   - Sensors     : DHT22 (Temperature + Humidity)
 *                   ZMPT101B (AC mains voltage sensing, 2mA/2mA module)
 *   - Storage     : 30F / 3.3V Supercapacitor
 *   - Voltage Div : R1=10kΩ, R2=10kΩ  (Vcap → A0, scaled to ADC)
 *   - USB-UART    : CP2102 / FT231X (built-in or external)
 *   - Energy Src  : Solar PV + DC-DC, Magnetic Coil + Rectifier,
 *                   RF Antenna + Rectifier → converge at VREG bus
 * 
 * Pin Mapping:
 *   A0  — Voltage divider output (supercapacitor voltage sense)
 *   A1  — ZMPT101B analog output (AC voltage sense, biased ~Vcc/2)
 *   D4  — DHT22 data pin
 *   D5  — Power enable for sensors (HIGH = sensors ON, gates DHT22 + ZMPT101B)
 *   TX  — Serial UART to USB-UART converter
 *   RX  — (unused in this design, reserved)
 * 
 * Telemetry Format (CSV):
 *   device_id, sequence, temperature, humidity, Vcap_mV, AC_Voltage_V, energy_mode
 *   e.g.: NODE01,1052,26.4,57.8,3185,231.4,NORMAL
 * 
 * Duty Cycle Modes:
 *   LOW    — 60s sample interval, 0.5s active  (Vcap < 1.8V)
 *   NORMAL — 10s sample interval, 1s   active  (1.8V ≤ Vcap < 2.7V)
 *   HIGH   — 2s  sample interval, 1s   active  (Vcap ≥ 2.7V)
 *   SLEEP  — System halted, deep sleep          (Vcap < 1.2V)
 * 
 * Authors : Group Seven, Dept. of Computer Engineering, CBU
 * Year    : 2026
 * Supervisor: Mr. Shumba
 * ============================================================
 */

// ─── Library Includes ────────────────────────────────────────
#include <DHT.h>        // DHT22 sensor library
#include <avr/sleep.h>  // AVR sleep modes
#include <avr/wdt.h>    // Watchdog timer for timed wakeup

// ─── Pin Definitions ─────────────────────────────────────────
#define PIN_VCAP_ADC A0      // Supercapacitor voltage sense
#define PIN_ZMPT_ADC A1      // ZMPT101B AC voltage sense (analog out)
#define PIN_DHT 4            // DHT22 data pin
#define PIN_SENSOR_ENABLE 5  // Sensor power gate (active HIGH)

// ─── DHT22 Setup ─────────────────────────────────────────────
#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);

// ─── ZMPT101B AC Voltage Sensor Setup ─────────────────────────
// ZMPT101B (2mA/2mA precision voltage transformer module) outputs
// a small AC signal centered on a DC bias (~Vcc/2) proportional to
// the sensed AC mains voltage. We sample many points across several
// mains cycles, compute the RMS of the AC component, then scale by
// a calibration constant determined against a known reference (e.g.
// a multimeter on the same mains line) to get true RMS volts.
#define ZMPT_SAMPLES 400          // ADC samples per RMS measurement (~ a few 50Hz cycles)
#define ZMPT_SAMPLE_INTERVAL_US 200  // Spacing between samples (microseconds)
#define ZMPT_ADC_MIDPOINT 511.5f  // Expected ADC bias point (10-bit ADC, mid-rail)
// TODO: Calibrate ZMPT_CALIBRATION against a known-good voltmeter reading.
//   ZMPT_CALIBRATION = (true_Vrms_from_multimeter) / (raw_ADC_Vrms_measured)
#define ZMPT_CALIBRATION 92.5f

// ─── Energy Thresholds (millivolts) ──────────────────────────
#define VCAP_SLEEP_MV 1200  // Below this: full sleep, no sensing
#define VCAP_LOW_MV 1800    // Below this: LOW mode
#define VCAP_HIGH_MV 2700   // Above this: HIGH mode
// Between LOW_MV and HIGH_MV = NORMAL mode

// ─── Voltage Divider Scaling ─────────────────────────────────
// R1 = 10kΩ (top), R2 = 10kΩ (bottom)
// Vout = Vcap × R2/(R1+R2) = Vcap × 0.5
// Vcap_mV = ADC_mV / 0.5 = ADC_mV × 2
// ADC reference = 5000mV (for 5V Nano) or 3300mV (for 3.3V)
#define ADC_REF_MV 5000     // Change to 3300 for 3.3V MCU
#define DIVIDER_RATIO 0.5f  // R2/(R1+R2)

// ─── Duty Cycle Timing (milliseconds) ────────────────────────
#define SLEEP_INTERVAL_LOW_MS 60000UL
#define SLEEP_INTERVAL_NORMAL_MS 10000UL
#define SLEEP_INTERVAL_HIGH_MS 2000UL
#define ACTIVE_DURATION_MS 1000UL  // Sensor active window

// ─── Device Identity ─────────────────────────────────────────
#define DEVICE_ID "NODE01"
#define UART_BAUD 9600

// ─── Energy Mode Enum ────────────────────────────────────────
typedef enum {
  MODE_DEEPSLEEP = 0,
  MODE_LOW = 1,
  MODE_NORMAL = 2,
  MODE_HIGH = 3
} EnergyMode;

// ─── Global State ────────────────────────────────────────────
static uint32_t g_sequence = 0;
static EnergyMode g_currentMode = MODE_NORMAL;
static uint32_t g_lastWakeTime = 0;

// ─── Function Prototypes ─────────────────────────────────────
uint16_t readVcapMillivolts(void);
float readACVoltage(void);
EnergyMode determineEnergyMode(uint16_t vcap_mv);
bool readSensorData(float *temperature, float *humidity);
void transmitTelemetry(float temp, float hum, uint16_t vcap_mv, float ac_voltage, EnergyMode mode);
void enterSleepMode(uint32_t sleep_ms);
void blinkStatusLED(EnergyMode mode);
void enableSensors(bool enable);
const char *modeToString(EnergyMode mode);
void printCSVHeader(void);

// ─── WDT ISR (Watchdog wakeup) ───────────────────────────────
ISR(WDT_vect) {
  // Watchdog wakeup — no action needed, just wake MCU
}


// ╔══════════════════════════════════════════════════════╗
// ║                    SETUP                            ║
// ╚══════════════════════════════════════════════════════╝
void setup() {
  // Initialise serial communication
  Serial.begin(UART_BAUD);
  while (!Serial) { ; }  // Wait for USB-UART on boards that need it

  // Configure GPIO
  pinMode(PIN_SENSOR_ENABLE, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_VCAP_ADC, INPUT);
  pinMode(PIN_ZMPT_ADC, INPUT);

  // Set ADC reference to internal 1.1V for better low-voltage resolution
  // (Optional: comment out if using external 5V/3.3V reference)
  // analogReference(INTERNAL);

  // Power down sensors initially
  enableSensors(false);
  digitalWrite(PIN_STATUS_LED, LOW);

  // Print CSV header for laptop logger
  Serial.println(F("=== Multi-Source Energy-Harvesting Autonomous Sensor Node ==="));
  Serial.println(F("=== The Copperbelt University  |  Group Seven  |  2026     ==="));
  Serial.println(F("pc_timestamp,device_id,sequence,temperature_C,humidity_pct,Vcap_mV,AC_Voltage_V,energy_mode"));

  // Short startup delay for power stabilisation
  delay(500);
}


// ╔══════════════════════════════════════════════════════╗
// ║                  MAIN LOOP                          ║
// ╚══════════════════════════════════════════════════════╝
void loop() {
  // ── Step 1: Read supercapacitor voltage ──────────────
  uint16_t vcap_mv = readVcapMillivolts();

  // ── Step 2: Determine energy mode ────────────────────
  g_currentMode = determineEnergyMode(vcap_mv);

  // ── Step 3: Deep-sleep if energy critically low ──────
  if (g_currentMode == MODE_DEEPSLEEP) {
    // Only report once before sleeping deeply
    Serial.print(F("SYSTEM,DEEP_SLEEP,Vcap="));
    Serial.print(vcap_mv);
    Serial.println(F("mV — waiting for energy recharge"));
    Serial.flush();

    digitalWrite(PIN_STATUS_LED, LOW);
    enableSensors(false);

    // Sleep for 60 seconds then re-check
    enterSleepMode(SLEEP_INTERVAL_LOW_MS);
    return;  // Re-check on next loop iteration
  }

  // ── Step 4: Blink LED to indicate mode ───────────────
  blinkStatusLED(g_currentMode);

  // ── Step 5: Enable sensors and read data ─────────────
  enableSensors(true);
  delay(ACTIVE_DURATION_MS);  // Sensor warm-up window

  float temperature = 0.0f;
  float humidity = 0.0f;
  bool sensorOK = readSensorData(&temperature, &humidity);

  // ZMPT101B AC voltage reading is independent of the DHT22, so it
  // is captured regardless of whether the temperature/humidity read
  // succeeded.
  float ac_voltage = readACVoltage();

  // ── Step 6: Transmit telemetry ────────────────────────
  if (sensorOK) {
    g_sequence++;
    transmitTelemetry(temperature, humidity, vcap_mv, ac_voltage, g_currentMode);
  } else {
    // Sensor read failed — transmit error record
    g_sequence++;
    Serial.print(F("SENSOR_ERR,"));
    Serial.print(DEVICE_ID);
    Serial.print(F(","));
    Serial.print(g_sequence);
    Serial.print(F(",NaN,NaN,"));
    Serial.print(vcap_mv);
    Serial.print(F(","));
    Serial.print(ac_voltage, 1);
    Serial.print(F(","));
    Serial.println(modeToString(g_currentMode));
  }
  Serial.flush();

  // ── Step 7: Power down sensors to conserve energy ────
  enableSensors(false);
  digitalWrite(PIN_STATUS_LED, LOW);

  // ── Step 8: Enter sleep for duty cycle interval ───────
  uint32_t sleepDuration;
  switch (g_currentMode) {
    case MODE_LOW: sleepDuration = SLEEP_INTERVAL_LOW_MS; break;
    case MODE_HIGH: sleepDuration = SLEEP_INTERVAL_HIGH_MS; break;
    case MODE_NORMAL:
    default: sleepDuration = SLEEP_INTERVAL_NORMAL_MS; break;
  }
  enterSleepMode(sleepDuration);
}


// ╔══════════════════════════════════════════════════════╗
// ║           FUNCTION IMPLEMENTATIONS                  ║
// ╚══════════════════════════════════════════════════════╝

/**
 * readVcapMillivolts()
 * Reads ADC pin A0 (voltage divider output) and converts
 * back to actual supercapacitor voltage in millivolts.
 * Takes 8 samples and averages for noise rejection.
 */
uint16_t readVcapMillivolts(void) {
  uint32_t adcSum = 0;
  const uint8_t NUM_SAMPLES = 8;

  for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
    adcSum += analogRead(PIN_VCAP_ADC);
    delay(2);
  }

  uint16_t adcAvg = (uint16_t)(adcSum / NUM_SAMPLES);

  // Convert ADC count to millivolts at the divider output
  // ADC_mV = (adcAvg / 1023.0) × ADC_REF_MV
  uint32_t adcMv = ((uint32_t)adcAvg * ADC_REF_MV) / 1023UL;

  // Scale back through divider: Vcap = ADC_mV / DIVIDER_RATIO
  uint16_t vcapMv = (uint16_t)((float)adcMv / DIVIDER_RATIO);

  return vcapMv;
}


/**
 * readACVoltage()
 * Samples the ZMPT101B analog output many times across several
 * mains cycles, computes the RMS value of the AC component about
 * its DC bias point, converts that to RMS volts at the ADC pin,
 * then applies ZMPT_CALIBRATION to report true AC mains voltage.
 */
float readACVoltage(void) {
  uint64_t sumSquares = 0;

  for (uint16_t i = 0; i < ZMPT_SAMPLES; i++) {
    int16_t raw = analogRead(PIN_ZMPT_ADC);
    float centered = (float)raw - ZMPT_ADC_MIDPOINT;
    sumSquares += (uint64_t)(centered * centered);
    delayMicroseconds(ZMPT_SAMPLE_INTERVAL_US);
  }

  float meanSquare = (float)sumSquares / (float)ZMPT_SAMPLES;
  float rmsCounts = sqrtf(meanSquare);

  // Convert RMS ADC counts to RMS volts at the ADC pin
  float rmsVoltsAtPin = (rmsCounts / 1023.0f) * ((float)ADC_REF_MV / 1000.0f);

  // Scale by calibration factor to get true AC mains RMS voltage
  float acVoltageRms = rmsVoltsAtPin * ZMPT_CALIBRATION;

  return acVoltageRms;
}


/**
 * determineEnergyMode()
 * Maps supercapacitor voltage to an energy operating mode.
 */
EnergyMode determineEnergyMode(uint16_t vcap_mv) {
  if (vcap_mv < VCAP_SLEEP_MV) return MODE_DEEPSLEEP;
  if (vcap_mv < VCAP_LOW_MV) return MODE_LOW;
  if (vcap_mv >= VCAP_HIGH_MV) return MODE_HIGH;
  return MODE_NORMAL;
}


/**
 * readSensorData()
 * Powers up DHT22 and reads temperature + humidity.
 * Returns true on success, false on sensor failure.
 */
bool readSensorData(float *temperature, float *humidity) {
  dht.begin();
  delay(2300);  // DHT22 minimum 2s inter-reading delay

  float h = dht.readHumidity();
  float t = dht.readTemperature();  // Celsius

  if (isnan(h) || isnan(t)) {
    return false;
  }

  *temperature = t;
  *humidity = h;
  return true;
}


/**
 * transmitTelemetry()
 * Formats and sends a single CSV line over USB-UART.
 * Format: device_id,sequence,temp,humidity,Vcap_mV,AC_Voltage_V,mode
 */
void transmitTelemetry(float temp, float hum, uint16_t vcap_mv, float ac_voltage, EnergyMode mode) {
  // Timestamp placeholder (laptop Python logger appends real timestamp)
  Serial.print(DEVICE_ID);
  Serial.print(F(","));
  Serial.print(g_sequence);
  Serial.print(F(","));
  Serial.print(temp, 1);
  Serial.print(F(","));
  Serial.print(hum, 1);
  Serial.print(F(","));
  Serial.print(vcap_mv);
  Serial.print(F(","));
  Serial.print(ac_voltage, 1);
  Serial.print(F(","));
  Serial.println(modeToString(mode));
}


/**
 * enableSensors()
 * Controls the power gate transistor for sensor peripherals.
 * Setting HIGH = power on sensors.
 */
void enableSensors(bool enable) {
  digitalWrite(PIN_SENSOR_ENABLE, enable ? HIGH : LOW);
}


/**
 * blinkStatusLED()
 * Visual feedback of current energy mode:
 *   LOW    = 1 slow blink
 *   NORMAL = 2 medium blinks
 *   HIGH   = 3 fast blinks
 */
void blinkStatusLED(EnergyMode mode) {
  uint8_t blinks;
  uint16_t blinkDelay;

  switch (mode) {
    case MODE_LOW:
      blinks = 1;
      blinkDelay = 400;
      break;
    case MODE_HIGH:
      blinks = 3;
      blinkDelay = 80;
      break;
    case MODE_NORMAL:
    default:
      blinks = 2;
      blinkDelay = 150;
      break;
  }

  for (uint8_t i = 0; i < blinks; i++) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(blinkDelay);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(blinkDelay);
  }
}


/**
 * enterSleepMode()
 * Uses the AVR watchdog timer to sleep the MCU for the
 * requested duration. Wakes up via WDT interrupt.
 * This drastically reduces current consumption during idle.
 * 
 * The WDT can sleep in 8-second chunks; this function
 * loops until the total duration is achieved.
 */
void enterSleepMode(uint32_t sleep_ms) {
  uint32_t elapsed = 0;

  while (elapsed < sleep_ms) {
    // Select watchdog timeout — use 8s for long sleeps, 1s for short
    uint32_t remaining = sleep_ms - elapsed;
    uint8_t wdtPrescaler;
    uint32_t chunkMs;

    if (remaining >= 8000) {
      wdtPrescaler = (1 << WDP3) | (1 << WDP0);  // 8 seconds
      chunkMs = 8000;
    } else if (remaining >= 4000) {
      wdtPrescaler = (1 << WDP3);  // 4 seconds
      chunkMs = 4000;
    } else if (remaining >= 2000) {
      wdtPrescaler = (1 << WDP2) | (1 << WDP1) | (1 << WDP0);  // 2 seconds
      chunkMs = 2000;
    } else if (remaining >= 1000) {
      wdtPrescaler = (1 << WDP2) | (1 << WDP1);  // 1 second
      chunkMs = 1000;
    } else {
      // Remainder too small for WDT — use delay
      delay(remaining);
      break;
    }

    // Configure WDT for interrupt (not reset)
    noInterrupts();
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDIE) | wdtPrescaler;
    interrupts();

    // Enter power-down sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();  // CPU halts here until WDT fires
    sleep_disable();

    elapsed += chunkMs;
  }

  // Disable WDT after sleeping
  wdt_disable();
}


/**
 * modeToString()
 * Returns a human-readable string for the current energy mode.
 */
const char *modeToString(EnergyMode mode) {
  switch (mode) {
    case MODE_DEEPSLEEP: return "DEEP_SLEEP";
    case MODE_LOW: return "LOW";
    case MODE_HIGH: return "HIGH";
    case MODE_NORMAL:
    default: return "NORMAL";
  }
}
