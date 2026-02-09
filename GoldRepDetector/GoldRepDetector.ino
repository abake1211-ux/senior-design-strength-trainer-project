#include <ArduinoBLE.h>

/*
 * Auto-Detecting Exercise Rep Counter + BLE Sender
 * Barbell side
 *
 * - Keeps original BLE Nordic UART behavior
 * - Uses new auto-detecting rep logic (bicep curl on Y, bench press on Z)
 */

#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>
#include <ArduinoBLE.h>

// #include  <Filters.h>
// #include <Filters/Butterworth.hpp>

// const double f_c = 6; // Hz

// auto filter = butter<4>(f_c)

// ===== IMU =====
LSM6DS3 imu(I2C_MODE, 0x6A);

// ===== TIMING =====
const uint32_t SAMPLE_INTERVAL_US = 5000;  // ~200 Hz sampling
uint32_t lastTick = 0;

// ===== EMA SMOOTHING =====
const float ALPHA = 0.15f;

// ===== SLOPE DETECTION PARAMETERS =====
const float DESCENDING_SLOPE_THRESHOLD = -0.010f;
const float ASCENDING_SLOPE_THRESHOLD  =  0.010f;
const int MIN_SLOPE_SAMPLES = 10;
const uint32_t MIN_PHASE_DURATION_MS = 300;

// ===== SMOOTHED ACCEL (g) =====
float ax_s = 0.0f, ay_s = 0.0f, az_s = 0.0f;

// ===== EXERCISE DETECTION =====
enum ExerciseType { 
  EXERCISE_DETECTING,     // In detection mode
  EXERCISE_BICEP_CURL,    // Y-axis dominant
  EXERCISE_BENCH_PRESS    // Z-axis dominant
};

ExerciseType currentExercise = EXERCISE_DETECTING;

const uint32_t FLAT_RESET_DURATION_MS = 10000;  // 10 seconds
uint32_t flatPhaseStartTime = 0;

// ===== CIRCULAR BUFFER PARAMS =====
const int BUFFER_SIZE = 32;
const int PHASE_WINDOW_SLOPES = 5;

// Separate buffers for each axis
float ay_history[BUFFER_SIZE];
float az_history[BUFFER_SIZE];
float ay_slope_history[BUFFER_SIZE];
float az_slope_history[BUFFER_SIZE];
int historyIndex = 0;
bool historyFilled = false;

// Running sums for slope windows
float ay_phaseSlopeSum = 0.0f;
float az_phaseSlopeSum = 0.0f;
int phaseSlopeCount = 0;

// ===== SLOPE STATE TRACKING =====
enum InstantaneousSlopeDirection { FLAT, DESCENDING, ASCENDING };
// ===== PHASE TRACKING =====
enum ConfirmedMotionPhase { PHASE_UNKNOWN, PHASE_ASCENDING, PHASE_DESCENDING, PHASE_FLAT };

// Y-axis slope tracking
InstantaneousSlopeDirection ay_currentSlopeState = FLAT;
int ay_slopeDirectionCount = 0;
ConfirmedMotionPhase ay_lastPhase = PHASE_FLAT;
uint32_t ay_phaseStartTime = 0;

// Z-axis slope tracking
InstantaneousSlopeDirection az_currentSlopeState = FLAT;
int az_slopeDirectionCount = 0;
ConfirmedMotionPhase az_lastPhase = PHASE_FLAT;
uint32_t az_phaseStartTime = 0;

// Active axis phase (the one we're counting reps on)
ConfirmedMotionPhase activePhase = PHASE_FLAT;
ConfirmedMotionPhase previousPhase = PHASE_FLAT;

// ===== REP COUNTER =====
int repCount = 0;

// Bicep curl rep state
enum BicepRepState { 
  BICEP_WAITING,
  BICEP_ASCENDING,
  BICEP_TOP_FLAT,
  BICEP_DESCENDING
};
BicepRepState bicepRepState = BICEP_WAITING;

// Bench press valley state
enum ValleyState { 
  VALLEY_IDLE,
  VALLEY_SAW_DESC,
  VALLEY_SAW_FLAT1,
  VALLEY_SAW_ASC
};
ValleyState valleyState = VALLEY_IDLE;

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
  while (!Serial && millis() - start < 2000) {
    ;
  }

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
