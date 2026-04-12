    // ===== HELPER FUNCTIONS =====

// const char* exerciseTypeToString(ExerciseType e) {
//   switch (e) {
//     case EXERCISE_DETECTING: return "DETECTING";
//     case EXERCISE_BICEP_CURL: return "BICEP_CURL";
//     case EXERCISE_BENCH_PRESS: return "BENCH_PRESS";
//     default: return "UNKNOWN";
//   }
// }

// const char* phaseToString(ConfirmedMotionPhase p) {
//   switch (p) {
//     case PHASE_ASCENDING: return "PHASE_ASC";
//     case PHASE_DESCENDING: return "PHASE_DESC";
//     case PHASE_FLAT: return "PHASE_FLAT";
//     default: return "PHASE_?";
//   }
// }

// void resetToDetectionMode() {
//   currentExercise = EXERCISE_DETECTING;
//   repCount = 0;
//   bicepRepState = BICEP_WAITING;
//   valleyState = VALLEY_IDLE;
//   activePhase = PHASE_FLAT;
//   previousPhase = PHASE_FLAT;
//   flatPhaseStartTime = millis();
// }

bool isPeak(float prev2, float prev1, float current) {
    return (prev1 > prev2) && (prev1 > current);
}

// ===== Replay IMU from CSV =====
bool readNextIMU(int &timestamp, float &ax, float &ay, float &az,
                 float &gx, float &gy, float &gz) {

    if (imu.begin() != 0) return false;

    timestamp = startTime - millis();
    ax = imu.readFloatAccelX();
    ay = imu.readFloatAccelY();
    az = imu.readFloatAccelZ();
    gx = imu.readFloatGyroX();
    gy = imu.readFloatGyroY();
    gz = imu.readFloatGyroZ();

    return true;
}

float updateMWI(float newSample, float* buffer, float &sum, int &index) {
    // Remove oldest
    sum -= buffer[index];

    // Add new
    buffer[index] = newSample;
    sum += newSample;

    // Circular increment
    index = (index + 1) % MWI_SIZE;

    // Return average
    return sum / MWI_SIZE;
}

void printToSerialMonitorAndBluetooth() {
    // CSV line over Serial + BLE: ay,az,exercise,repCount,activePhase
  char line[80];
  // Create meaningful line (character limit only allows for 5 vars)
  snprintf(line, sizeof(line), "%d,%.5f,%.5f,%.5f,%d",
           timestamp, aMag, aMWI, THRESHOLD_a, repCount);
  // Print to Serial Monitor
  Serial.println(line);
  // Print to BLE device, if connected
  if (BLE.connected()) {
    txChar.writeValue((uint8_t*)line, strlen(line));
  }
}

