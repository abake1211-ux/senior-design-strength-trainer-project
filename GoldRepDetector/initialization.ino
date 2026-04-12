// ===== INITIALIZATION FUNCTIONS =====

void initializeThresholds(float aMWI, float gMWI) {
    // Accel
    if (aMWI > SPKI_a) {
      SPKI_a = aMWI;
    }
    NPKI_a = 0.5 * SPKI_a;
    THRESHOLD_a = 0.5 * SPKI_a;

    // Gyro
    if (gMWI > SPKI_g) {
      SPKI_g = gMWI;
    }
    NPKI_g = 0.5 * SPKI_g;
    THRESHOLD_g = 0.5 * SPKI_g;
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
    while(1) {}
  }

  Serial.println("IMU connection successful. Initializing BLE.");

  initBLE();

  Serial.println("BLE connection successful. Starting program.");

  // delay(10000);

  lastTick = micros();
  startTime = millis();
}




