// ===== INITIALIZATION FUNCTIONS =====

void initBuffers(float seedAy, float seedAz) {
  for (int i = 0; i < BUFFER_SIZE; ++i) {
    ay_history[i] = seedAy;
    az_history[i] = seedAz;
    ay_slope_history[i] = 0.0f;
    az_slope_history[i] = 0.0f;
  }
  historyIndex = 0;
  historyFilled = true;
  ay_phaseSlopeSum = 0.0f;
  az_phaseSlopeSum = 0.0f;
  phaseSlopeCount = 0;
}

void initBLE(){
  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1) {}
  }

  BLE.setLocalName("BarbellIMU");
  BLE.setDeviceName("BarbellIMU");
  BLE.setAdvertisedService(dataService);
  dataService.addCharacteristic(txChar);
  BLE.addService(dataService);
  BLE.advertise();

  Serial.println("BLE advertising as BarbellIMU...");
}

void initialize() {
  if (imu.begin() != 0) {
    Serial.println("IMU init fail");
  } else {
    ax_s = imu.readFloatAccelX();
    ay_s = imu.readFloatAccelY();
    az_s = imu.readFloatAccelZ();

    initBuffers(ay_s, az_s);

    Serial.println("=== AUTO-DETECTING EXERCISE TRACKER + BLE ===");
    Serial.println("Start exercising - first axis to move determines exercise type");
    Serial.println("ay_g,az_g,exercise,rep_count,active_phase");
  }

  lastTick = micros();
  ay_phaseStartTime = millis();
  az_phaseStartTime = millis();
  flatPhaseStartTime = millis();

  initBLE();
}

