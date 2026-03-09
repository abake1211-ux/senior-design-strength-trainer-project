// ===== INITIALIZATION FUNCTIONS =====

void initBuffers(float seedA, float seedG) {
  for (int i = 0; i < BUFFER_SIZE; ++i) {
    aMag_history[i] = seedA;
    gMag_history[i] = seedG;
    aMag_slope_history[i] = 0.0f;
    gMag_slope_history[i] = 0.0f;
  }
  historyIndex = 0;
  historyFilled = true;
  aMag_phaseSlopeSum = 0.0f;
  gMag_phaseSlopeSum = 0.0f;
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

    float ax0 = imu.readFloatAccelX();
    float ay0 = imu.readFloatAccelY();
    float az0 = imu.readFloatAccelZ();
    float gx0 = imu.readFloatGyroX();
    float gy0 = imu.readFloatGyroY();
    float gz0 = imu.readFloatGyroZ();

    float aMag0 = sqrt(ax0*ax0 + ay0*ay0 + az0*az0);
    float gMag0 = sqrt(gx0*gx0 + gy0*gy0 + gz0*gz0);

    for (int i = 0; i < 20; i++) {
      aMag_s = aMagFilter(aMag0);
      gMag_s = gMagFilter(gMag0);}

    initBuffers(aMag_s, gMag_s);

    Serial.println("=== AUTO-DETECTING EXERCISE TRACKER + BLE ===");
    Serial.println("Start exercising - first axis to move determines exercise type");
    Serial.println("aMag_g,gMag_deg/s,exercise,rep_count,active_phase");
  }

  lastTick = micros();
  aMag_phaseStartTime = millis();
  gMag_phaseStartTime = millis();
  flatPhaseStartTime = millis();

  initBLE();
}

