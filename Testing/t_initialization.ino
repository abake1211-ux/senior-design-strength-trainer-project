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

void initialize() {

  if (!SD.begin(chipSelect)) {
        Serial.println("SD initialization failed!");
        while (1);
  }

  imuFile = SD.open("BENCH7.txt");
  if (!imuFile) {
      Serial.println("Failed to open file!");
      while (1);
  }

  Serial.println("Starting IMU replay...");

  // int timestamp0;
  // float ax0, ay0, az0, gx0, gy0, gz0;

  // if (readNextIMU(timestamp0, ax0, ay0, az0, gx0, gy0, gz0)) {
    
  //   float aMag0 = sqrt(ax0*ax0 + ay0*ay0 + az0*az0);
  //   float gMag0 = sqrt(gx0*gx0 + gy0*gy0 + gz0*gz0);

  //  // for (int i = 0; i < 20; i++) {
  //   //  aMag_s = aMagFilter(aMag0);
  //   //  gMag_s = gMagFilter(gMag0);
  //  // }

  // }

  delay(10000);

  lastTick = micros();
  startTime = millis();
}




