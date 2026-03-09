// ===== DETECTION AND COUNTING FUNCTIONS =====

void updateBufferAndSlopes() {
  aMag_history[historyIndex] = aMag_s;
  gMag_history[historyIndex] = gMag_s;

  int prevIdx = (historyIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
  
  float aMag_newSlope = aMag_history[historyIndex] - aMag_history[prevIdx];
  float gMag_newSlope = gMag_history[historyIndex] - gMag_history[prevIdx];

  aMag_slope_history[historyIndex] = aMag_newSlope;
  gMag_slope_history[historyIndex] = gMag_newSlope;

  aMag_phaseSlopeSum += aMag_newSlope;
  gMag_phaseSlopeSum += gMag_newSlope;
  phaseSlopeCount++;

  if (phaseSlopeCount > PHASE_WINDOW_SLOPES) {
    int oldSlopeIdx = (historyIndex - PHASE_WINDOW_SLOPES + BUFFER_SIZE) % BUFFER_SIZE;
    aMag_phaseSlopeSum -= aMag_slope_history[oldSlopeIdx];
    gMag_phaseSlopeSum -= gMag_slope_history[oldSlopeIdx];
    phaseSlopeCount--;
  }

  historyIndex = (historyIndex + 1) % BUFFER_SIZE;
  if (historyIndex == 0) historyFilled = true;
}

float getPhaseSlope(bool useAccel) {
  if (phaseSlopeCount == 0) return 0.0f;
  
  if (useAccel) {
    return aMag_phaseSlopeSum / (float)phaseSlopeCount;
  } else {
    return gMag_phaseSlopeSum / (float)phaseSlopeCount;
  }
}

InstantaneousSlopeDirection determineSlopeState(float slope) {
  if (slope < DESCENDING_SLOPE_THRESHOLD) {
    return DESCENDING;
  } else if (slope > ASCENDING_SLOPE_THRESHOLD) {
    return ASCENDING;
  } else {
    return FLAT;
  }
}

void updateSlopeState(bool isYaxis, float currentSlope, InstantaneousSlopeDirection &slopeState, int &slopeCount) {
  (void)isYaxis; // not used, but kept for symmetry
  InstantaneousSlopeDirection newState = determineSlopeState(currentSlope);

  if (newState == slopeState) {
    slopeCount++;
  } else {
    slopeState = newState;
    slopeCount = 1;
  }
}

bool isSlopeConfirmed(InstantaneousSlopeDirection currentState, int count, InstantaneousSlopeDirection expectedState) {
  return (currentState == expectedState) && (count >= MIN_SLOPE_SAMPLES);
}

// Detect phase for a specific axis
ConfirmedMotionPhase detectMeasType(bool isYaxis, InstantaneousSlopeDirection &currentState, int &slopeCount, 
                           ConfirmedMotionPhase &lastPhase, uint32_t &phaseStartTime) {
  float currentSlope = getPhaseSlope(isYaxis);
  updateSlopeState(isYaxis, currentSlope, currentState, slopeCount);
  
  // Check if enough time has passed for transition
  if ((millis() - phaseStartTime) < MIN_PHASE_DURATION_MS) {
    return lastPhase;  // No change yet
  }
  
  ConfirmedMotionPhase newPhase = lastPhase;
  
  if (isSlopeConfirmed(currentState, slopeCount, ASCENDING)) {
    if (lastPhase != PHASE_ASCENDING) {
      newPhase = PHASE_ASCENDING;
    }
  } else if (isSlopeConfirmed(currentState, slopeCount, DESCENDING)) {
    if (lastPhase != PHASE_DESCENDING) {
      newPhase = PHASE_DESCENDING;
    }
  } else if (isSlopeConfirmed(currentState, slopeCount, FLAT)) {
    if (lastPhase != PHASE_FLAT) {
      newPhase = PHASE_FLAT;
    }
  }
  
  if (newPhase != lastPhase) {
    phaseStartTime = millis();
    lastPhase = newPhase;
  }
  
  return newPhase;
}

// ===== BICEP CURL REP COUNTER =====
void updateBicepCurlReps(ConfirmedMotionPhase newPhase) {
  // Bicep curl state machine: FLAT → ASC → FLAT(top) → DESC → FLAT(bottom)
  switch (bicepRepState) {
    case BICEP_WAITING:
      if (newPhase == PHASE_ASCENDING) {
        bicepRepState = BICEP_ASCENDING;
      }
      break;
      
    case BICEP_ASCENDING:
      if (newPhase == PHASE_FLAT) {
        bicepRepState = BICEP_TOP_FLAT;
      }
      break;
      
    case BICEP_TOP_FLAT:
      if (newPhase == PHASE_DESCENDING) {
        bicepRepState = BICEP_DESCENDING;
      } else if (newPhase == PHASE_ASCENDING) {
        bicepRepState = BICEP_ASCENDING;
      }
      break;
      
    case BICEP_DESCENDING:
      if (newPhase == PHASE_FLAT) {
        repCount++;
        Serial.print("REP_COUNTED,total=");
        Serial.println(repCount);
        bicepRepState = BICEP_WAITING;
      }
      break;
  }
}

// ===== BENCH PRESS REP COUNTER =====
void updateBenchPressReps(ConfirmedMotionPhase newPhase) {
  // Valley detection: DESC → FLAT → ASC → FLAT
  switch (valleyState) {
    case VALLEY_IDLE:
      if (newPhase == PHASE_DESCENDING) {
        valleyState = VALLEY_SAW_DESC;
      }
      break;
      
    case VALLEY_SAW_DESC:
      if (newPhase == PHASE_FLAT) {
        valleyState = VALLEY_SAW_FLAT1;
      } else if (newPhase == PHASE_ASCENDING) {
        valleyState = VALLEY_SAW_ASC;
      }
      break;
      
    case VALLEY_SAW_FLAT1:
      if (newPhase == PHASE_ASCENDING) {
        valleyState = VALLEY_SAW_ASC;
      } else if (newPhase == PHASE_DESCENDING) {
        valleyState = VALLEY_SAW_DESC;
      }
      break;
      
    case VALLEY_SAW_ASC:
      if (newPhase == PHASE_FLAT) {
        repCount++;
        Serial.print("REP_COUNTED,total=");
        Serial.println(repCount);
        valleyState = VALLEY_IDLE;
      } else if (newPhase == PHASE_DESCENDING) {
        valleyState = VALLEY_SAW_DESC;
      }
      break;
  }
}

// ===== MAIN DETECTION & COUNTING LOGIC =====
void processExercise() {
  // Always monitor both axes
  ConfirmedMotionPhase aMag_phase = detectMeasType(true,  aMag_currentSlopeState, aMag_slopeDirectionCount, 
                                        aMag_lastPhase, aMag_phaseStartTime);
  ConfirmedMotionPhase gMag_phase = detectMeasType(false, gMag_currentSlopeState, gMag_slopeDirectionCount, 
                                        gMag_lastPhase, gMag_phaseStartTime);
  
  // DETECTION MODE: waiting for first axis to move
  if (currentExercise == EXERCISE_DETECTING) {
    // Check if both are still flat
    if (aMag_phase == PHASE_FLAT && gMag_phase == PHASE_FLAT) {
      activePhase = PHASE_FLAT;
      return;
    }
    
    // Instruement moved! Determine if accel or gyro moved
    if (gMag_phase == PHASE_ASCENDING) {
      // Gyro responded → Bicep Curl
      currentExercise = EXERCISE_BICEP_CURL;
      activePhase = gMag_phase;
      Serial.println("EXERCISE_DETECTED: BICEP_CURL (Gyroscope active)");
    } else if (aMag_phase == PHASE_ASCENDING) {
      // Accel moved first → Bench Press
      currentExercise = EXERCISE_BENCH_PRESS;
      activePhase = aMag_phase;
      Serial.println("EXERCISE_DETECTED: BENCH_PRESS (Accelerometer active)");
    } else {
      // Both moved simultaneously - pick the one with larger slope magnitude
      // THIS SHOULD BE CHANGED TO COMPARE SOMETHING LIKE DURATION OR AREA UNDER CURVE
      float aMag_slope = fabs(getPhaseSlope(true));
      float gMag_slope = fabs(getPhaseSlope(false));
      
      if (aMag_slope < gMag_slope) {
        currentExercise = EXERCISE_BICEP_CURL;
        activePhase = gMag_phase;
        Serial.println("EXERCISE_DETECTED: BICEP_CURL (Gyro larger slope)");
      } else {
        currentExercise = EXERCISE_BENCH_PRESS;
        activePhase = gMag_phase;
        Serial.println("EXERCISE_DETECTED: BENCH_PRESS (Accel larger slope)");
      }
    }
    
    flatPhaseStartTime = millis();
    previousPhase = activePhase;
    return;
  }
  
  // ACTIVE MODE: count reps on dominant axis
  if (currentExercise == EXERCISE_BICEP_CURL) {
    activePhase = gMag_phase;
  } else {  // BENCH_PRESS
    activePhase = aMag_phase;
  }
  
  // Check for 10-second flat timeout → return to detection mode
  if (activePhase == PHASE_FLAT) {
    if (previousPhase != PHASE_FLAT) {
      // Just entered flat
      flatPhaseStartTime = millis();
    } else if ((millis() - flatPhaseStartTime) >= FLAT_RESET_DURATION_MS) {
      // Been flat for 10+ seconds
      resetToDetectionMode();
      return;
    }
  } else {
    // Not flat, update timer
    flatPhaseStartTime = millis();
  }
  
  // Update rep counter when phase changes
  if (activePhase != previousPhase) {
    if (currentExercise == EXERCISE_BICEP_CURL) {
      updateBicepCurlReps(activePhase);
    } else {  // BENCH_PRESS
      updateBenchPressReps(activePhase);
    }
  }
  
  previousPhase = activePhase;
}

void handleDetection() {
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();

  float aMag = sqrt(ax*ax + ay*ay + az*az);
  float gMag = sqrt(gx*gx + gy*gy + gz*gz);
  
  // Median filter
  float aMag_m = aMed(aMag);
  float gMag_m = gMed(gMag);

  aMag_s = aMagFilter(aMag);
  gMag_s = gMagFilter(gMag);

  updateBufferAndSlopes();
  processExercise();

// ===== PRINT STATEMENTS =====
  printToSerialMonitorAndBluetooth();

}
