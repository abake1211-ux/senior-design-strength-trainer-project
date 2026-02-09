// ===== HELPER FUNCTIONS =====

const char* exerciseTypeToString(ExerciseType e) {
  switch (e) {
    case EXERCISE_DETECTING: return "DETECTING";
    case EXERCISE_BICEP_CURL: return "BICEP_CURL";
    case EXERCISE_BENCH_PRESS: return "BENCH_PRESS";
    default: return "UNKNOWN";
  }
}

const char* phaseToString(ConfirmedMotionPhase p) {
  switch (p) {
    case PHASE_ASCENDING: return "PHASE_ASC";
    case PHASE_DESCENDING: return "PHASE_DESC";
    case PHASE_FLAT: return "PHASE_FLAT";
    default: return "PHASE_?";
  }
}

void resetToDetectionMode() {
  currentExercise = EXERCISE_DETECTING;
  repCount = 0;
  bicepRepState = BICEP_WAITING;
  valleyState = VALLEY_IDLE;
  activePhase = PHASE_FLAT;
  previousPhase = PHASE_FLAT;
  flatPhaseStartTime = millis();
}

void printToSerialMonitorAndBluetooth() {
    // CSV line over Serial + BLE: ay,az,exercise,repCount,activePhase
  char line[80];
  // Create meaningful line
  snprintf(line, sizeof(line), "%.4f,%.4f,%s,%d,%s",
           ay_s, az_s,
           exerciseTypeToString(currentExercise),
           repCount,
           phaseToString(activePhase));
  // Print meaningful line to Serial Monitor
  Serial.println(line);
  // Send meaningful line to BLE Receiver if connected
  if (BLE.connected()) {
  txChar.writeValue((uint8_t*)line, strlen(line));
  }
}
