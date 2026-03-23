/*
 * Auto-Detecting Exercise Rep Counter + BLE Sender
 * Barbell side
 *
 * - Keeps original BLE Nordic UART behavior
 * - Uses new auto-detecting rep logic (bicep curl on Y, bench press on Z)
 */

#include "Wire.h"
#include <math.h>
// #include <ArduinoBLE.h>
#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <SPI.h>
#include <SD.h>

// SD Card variables
File imuFile;
const int chipSelect = 10;  // digital pin

// Filter variables
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
const int MWI_SIZE = 85;
// accel mwi
float aBuffer[MWI_SIZE] = {0};
float aSum = 0;
int aIndex = 0;
// gyro mwi
float gBuffer[MWI_SIZE] = {0};
float gSum = 0;
int gIndex = 0;

// // ===== TIMING =====
const uint32_t SAMPLE_INTERVAL_US = 400;  // ~100 Hz sampling; Moved to 400 Hz for quick data download
uint32_t lastTick = 0;

int timestamp;
float ax, ay, az, gx, gy, gz;
float aMag_hp, gMag_hp, aMag_s, gMag_s;
float aMag, gMag;
float aMWI, gMWI;
float aSquare, gSquare;

// ===== EMA SMOOTHING =====
// const float ALPHA = 0.15f;

// // ===== SLOPE DETECTION PARAMETERS =====
// const float DESCENDING_SLOPE_THRESHOLD = -0.010f;
// const float ASCENDING_SLOPE_THRESHOLD  =  0.010f;
// const int MIN_SLOPE_SAMPLES = 10;
// const uint32_t MIN_PHASE_DURATION_MS = 300;

// ===== EXERCISE DETECTION =====
// enum ExerciseType { 
//   EXERCISE_DETECTING,     // In detection mode
//   EXERCISE_BICEP_CURL,    // Y-axis dominant
//   EXERCISE_BENCH_PRESS    // Z-axis dominant
// };

// ExerciseType currentExercise = EXERCISE_DETECTING;

// const uint32_t FLAT_RESET_DURATION_MS = 5000;  // 5 seconds
// uint32_t flatPhaseStartTime = 0;

// // ===== CIRCULAR BUFFER PARAMS =====
// const int BUFFER_SIZE = 32;
// const int PHASE_WINDOW_SLOPES = 5;

// // Separate buffers for each axis
// float aMag_history[BUFFER_SIZE];
// float gMag_history[BUFFER_SIZE];
// float aMag_slope_history[BUFFER_SIZE];
// float gMag_slope_history[BUFFER_SIZE];
// int historyIndex = 0;
// bool historyFilled = false;

// // Running sums for slope windows
// float aMag_phaseSlopeSum = 0.0f;
// float gMag_phaseSlopeSum = 0.0f;
// int phaseSlopeCount = 0;

// // ===== SLOPE STATE TRACKING =====
// enum InstantaneousSlopeDirection { FLAT, DESCENDING, ASCENDING };
// // ===== PHASE TRACKING =====
// enum ConfirmedMotionPhase { PHASE_UNKNOWN, PHASE_ASCENDING, PHASE_DESCENDING, PHASE_FLAT };

// // Accel slope tracking
// InstantaneousSlopeDirection aMag_currentSlopeState = FLAT;
// int aMag_slopeDirectionCount = 0;
// ConfirmedMotionPhase aMag_lastPhase = PHASE_FLAT;
// uint32_t aMag_phaseStartTime = 0;

// // Gyro slope tracking
// InstantaneousSlopeDirection gMag_currentSlopeState = FLAT;
// int gMag_slopeDirectionCount = 0;
// ConfirmedMotionPhase gMag_lastPhase = PHASE_FLAT;
// uint32_t gMag_phaseStartTime = 0;

// // Active measurement phase (the one we're counting reps on)
// ConfirmedMotionPhase activePhase = PHASE_FLAT;
// ConfirmedMotionPhase previousPhase = PHASE_FLAT;

// // ===== REP COUNTER =====
// int repCount = 0;

// // Bicep curl rep state
// enum BicepRepState { 
//   BICEP_WAITING,
//   BICEP_ASCENDING,
//   BICEP_TOP_FLAT,
//   BICEP_DESCENDING
// };
// BicepRepState bicepRepState = BICEP_WAITING;

// // Bench press valley state
// enum ValleyState { 
//   VALLEY_IDLE,
//   VALLEY_SAW_DESC,
//   VALLEY_SAW_FLAT1,
//   VALLEY_SAW_ASC
// };
// ValleyState valleyState = VALLEY_IDLE;


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

  uint32_t now = micros();
  if ((int32_t)(now - lastTick) < (int32_t)SAMPLE_INTERVAL_US) return;
  lastTick += SAMPLE_INTERVAL_US;

  handleDetection();
}
