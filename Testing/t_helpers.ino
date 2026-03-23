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

// ===== Replay IMU from CSV =====
bool readNextIMU(int &timestamp, float &ax, float &ay, float &az,
                 float &gx, float &gy, float &gz) {

    if (!imuFile.available()) return false;

    String line = imuFile.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return false;

    int first = line.indexOf(',');
    int second = line.indexOf(',', first + 1);
    int third = line.indexOf(',', second + 1);
    int fourth = line.indexOf(',', third + 1);
    int fifth = line.indexOf(',', fourth + 1);
    int sixth = line.indexOf(',', fifth + 1);

    timestamp = line.substring(0, first).toInt();
    ax = line.substring(first + 1, second).toFloat();
    ay = line.substring(second + 1, third).toFloat();
    az = line.substring(third + 1, fourth).toFloat();
    gx = line.substring(fourth + 1, fifth).toFloat();
    gy = line.substring(fifth + 1, sixth).toFloat();
    gz = line.substring(sixth + 1).toFloat();

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
  snprintf(line, sizeof(line), "%d,%.5f,%.5f,%.5f,%.5f",
           timestamp, aMag, gMag, aMag_s, gMag_s);
  // Print meaningful line to Serial Monitor
  Serial.println(line);
  
  }

