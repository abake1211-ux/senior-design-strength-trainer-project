// ===== INITIALIZATION FUNCTIONS =====

// void initBuffers(float seedA, float seedG) {
//   for (int i = 0; i < BUFFER_SIZE; ++i) {
//     aMag_history[i] = seedA;
//     gMag_history[i] = seedG;
//     aMag_slope_history[i] = 0.0f;
//     gMag_slope_history[i] = 0.0f;
//   }
//   historyIndex = 0;
//   historyFilled = true;
//   aMag_phaseSlopeSum = 0.0f;
//   gMag_phaseSlopeSum = 0.0f;
//   phaseSlopeCount = 0;
// }

void initialize() {

  if (!SD.begin(chipSelect)) {
        Serial.println("SD initialization failed!");
        while (1);
  }

  imuFile = SD.open("BENCH1.txt");
  if (!imuFile) {
      Serial.println("Failed to open file!");
      while (1);
  }

  Serial.println("Starting IMU replay...");

  int timestamp0;
  float ax0, ay0, az0, gx0, gy0, gz0;

  if (readNextIMU(timestamp0, ax0, ay0, az0, gx0, gy0, gz0)) {
    
    float aMag0 = sqrt(ax0*ax0 + ay0*ay0 + az0*az0);
    float gMag0 = sqrt(gx0*gx0 + gy0*gy0 + gz0*gz0);

    for (int i = 0; i < 20; i++) {
      aMag_s = aMagFilter(aMag0);
      gMag_s = gMagFilter(gMag0);
    }

  }

  delay(10000);

  // initBuffers(aMag_s, gMag_s);
  // aMag_phaseStartTime = millis();
  // gMag_phaseStartTime = millis();
  // flatPhaseStartTime = millis();
  lastTick = micros();
}




