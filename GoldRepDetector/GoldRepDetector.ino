/*
 * Auto-Detecting Exercise Rep Counter + BLE Sender
 * Barbell side
 *
 * - Keeps original BLE Nordic UART behavior
 * - Uses new auto-detecting rep logic (bicep curl on Y, bench press on Z)
 */

#include "Wire.h"
#include <math.h>
#include <ArduinoBLE.h>
#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <SPI.h>

// ===== IMU =====
LSM6DS3 imu(I2C_MODE, 0x6A);


// ===== FILTER ===== 
const double FS = 100.0;   // Sampling frequency (Hz)
// low pass
const double FC = 6.0;     // Cutoff frequency (Hz)
const double FN = 2 * FC / FS;
// high pass
const double FC_HP = 0.08;
const double FN_HP = 2 * FC_HP / FS;
// low pass
auto aMagFilter = butter<4>(FN);
auto gMagFilter = butter<4>(FN);
// high pass
auto aHighPassBase = butter<4>(FN_HP);   // low-pass at 0.5 Hz for HPF trick
auto gHighPassBase = butter<4>(FN_HP);
// mwi
const int MWI_SIZE = 10;
// accel mwi
float aBuffer[MWI_SIZE] = {0};
float aSum = 0;
int aIndex = 0;
// gyro mwi
float gBuffer[MWI_SIZE] = {0};
float gSum = 0;
int gIndex = 0;


// // ===== TIMING =====
const uint32_t SAMPLE_INTERVAL_US = 5000;  // ~200 Hz sampling
uint32_t lastTick = 0;
int timestamp;
float ax, ay, az, gx, gy, gz;
float aMag_hp, gMag_hp, aMag_s, gMag_s;
float aMag, gMag;
float aMWI, gMWI;
float aSquare, gSquare;


// ===== THRESHOLD DETECTION ===== 
// --- ACCEL ---
float SPKI_a = 0, NPKI_a = 0, THRESHOLD_a = 0;
float prev2_a = 0, prev1_a = 0, current_a = 0;
// --- GYRO ---
float SPKI_g = 0, NPKI_g = 0, THRESHOLD_g = 0;
float prev2_g = 0, prev1_g = 0, current_g = 0;
// Timing
unsigned long lastRepTime = 0;
unsigned long startTime = 0;
// Rep count
int repCount = 0;
bool peakA;
bool peakG;
// Refractory
const int REFRACTORY_PERIOD = 100;
// --- ACCEL threshold state ---
float peakt_a = 0, peaki_a = 0;
// unsigned long lastQRS_a = 0;  // for cc equivalent (time-based reset)
size_t cc_a = 0;
// --- GYRO threshold state ---
float peakt_g = 0, peaki_g = 0;
// unsigned long lastQRS_g = 0;
size_t cc_g = 0;
#define LOOKBACK_N 10
float lookback_a[LOOKBACK_N] = {0};
float lookback_g[LOOKBACK_N] = {0};


// ===== INIT ===== 
bool initialized = false;
// ===== BLE (Nordic UART) =====
BLEService dataService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
                         BLERead | BLENotify,
                         64);

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  // Wait up to 2 seconds for Serial if it's there, but don't block forever
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000);

  initialize();
}

// ===== LOOP =====
void loop() {
  BLE.poll();

  uint32_t now = micros();
  if ((int32_t)(now - lastTick) < (int32_t)SAMPLE_INTERVAL_US) return;
  lastTick += SAMPLE_INTERVAL_US;

  handleDetection();
}
