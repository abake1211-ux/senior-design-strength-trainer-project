// ---------------------------------------------------------------------------------
// Adaptive threshold detector — Accel
// ---------------------------------------------------------------------------------
bool processPeak_accel(float current_a, float prev1_a, float prev2_a) {
    bool repDetected = false;
    unsigned long currentTime = millis();

    // Shift lookback buffer
    for (int i = LOOKBACK_N - 1; i > 0; i--) { lookback_a[i] = lookback_a[i-1]; }
    lookback_a[0] = current_a;
    float lookback = lookback_a[LOOKBACK_N - 1]; // oldest sample

    cc_a++;
    if (cc_a > 100000) {
        cc_a = 0;
        THRESHOLD_a = 0;
        SPKI_a = 0;
        NPKI_a = 0;
        peakt_a = 0;
        peaki_a = 0;
    }

    // Track local peak using longer lookback
    if (current_a > lookback && current_a > peakt_a) { peakt_a = current_a; }

    // Detect end of candidate peak using longer lookback
    if (current_a <= lookback && current_a < 0.75f * peakt_a) {
        peaki_a = peakt_a;

        if (peaki_a > THRESHOLD_a) {
            SPKI_a = 0.125f * peaki_a + 0.875f * SPKI_a;

            bool outsideRefractory = (currentTime - lastRepTime) > REFRACTORY_PERIOD;
            if (outsideRefractory) {
                lastRepTime = currentTime;
                cc_a = 0;
                repCount++;
                repDetected = true;
            }
        // Serial.print("peaki_a: "); Serial.print(peaki_a);
        // Serial.print(" | THRESHOLD_a: "); Serial.print(THRESHOLD_a);
        // Serial.print(" | SPKI_a: "); Serial.print(SPKI_a);
        // Serial.print(" | NPKI_a: "); Serial.println(NPKI_a);
        } else {
            NPKI_a = 0.125f * peaki_a + 0.875f * NPKI_a;
        }

        THRESHOLD_a = NPKI_a + 0.65f * (SPKI_a - NPKI_a);
        peakt_a = 0.0f;
    }

    return repDetected;
}

// ---------------------------------------------------------------------------------
// Adaptive threshold detector — Gyro
// ---------------------------------------------------------------------------------
// bool processPeak_gyro(float current_g, float prev1_g, float prev2_g) {
//     bool repDetected = false;
//     unsigned long currentTime = millis();

//     // Time-based stale signal reset
//     if ((currentTime - lastQRS_g) > 5000) {
//         THRESHOLD_g = 0;
//         SPKI_g = 0;
//         NPKI_g = 0;
//         peakt_g = 0;
//         peaki_g = 0;
//         lastQRS_g = currentTime;
//     }

//     // Track local peak
//     if (current_g > prev2_g && current_g > peakt_g) { peakt_g = current_g; }

//     // Detect end of candidate peak
//     if (current_g <= prev2_g && current_g < 0.5f * peakt_g) {
//         peaki_g = peakt_g;

//         if (peaki_g > THRESHOLD_g) {
//             SPKI_g = 0.125f * peaki_g + 0.875f * SPKI_g;

//             bool outsideRefractory = (currentTime - lastRepTime) > REFRACTORY_PERIOD;
//             if (outsideRefractory) {
//                 lastRepTime = currentTime;
//                 lastQRS_g = currentTime;
//                 repCount++;
//                 repDetected = true;
//             }
//         } else {
//             NPKI_g = 0.125f * peaki_g + 0.875f * NPKI_g;
//         }

//         THRESHOLD_g = NPKI_g + 0.25f * (SPKI_g - NPKI_g);
//         peakt_g = 0.0f;
//     }

//     return repDetected;
// }


void handleDetection() {


  if (readNextIMU(timestamp, ax, ay, az, gx, gy, gz)){
  
    // --- MAGNITUDE --- //
    aMag = sqrt(ax*ax + ay*ay + az*az);
    gMag = sqrt(gx*gx + gy*gy + gz*gz);

    // --- HIGH PASS --- //
    aMag_hp = aMag - aHighPassBase(aMag);
    gMag_hp = gMag - gHighPassBase(gMag);

    // --- LOW PASS --- //
    aMag_s = aMagFilter(aMag_hp);
    gMag_s = gMagFilter(gMag_hp);

    // --- SQUARING --- // 
    aSquare = aMag_s*aMag_s;
    gSquare = gMag_s*gMag_s;

    // --- MOVING WINDOW INTEGRATION --- //
    aMWI = updateMWI(aSquare, aBuffer, aSum, aIndex);
    gMWI = updateMWI(gSquare, gBuffer, gSum, gIndex);

    // Update current values
    current_a = aMWI;
    current_g = gMWI;

    if (!initialized) {
      initializeThresholds(current_a, current_g);
      initialized = true;
    }

// --- Peak detection and threshold (called every sample) ---
    if (initialized) {
        peakA = processPeak_accel(current_a, prev1_a, prev2_a);
        // peakG = processPeak_gyro(current_g, prev1_g, prev2_g);
    } 

    // --- Shift memory ---
    prev2_a = prev1_a;
    prev1_a = current_a;

    prev2_g = prev1_g;
    prev1_g = current_g;
    

  } else {

    Serial.println("End of file reached.");
    while (1); // stop after replay

  }

// ===== PRINT STATEMENTS =====
  printToSerialMonitorAndBluetooth();

}
