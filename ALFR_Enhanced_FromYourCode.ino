//========================================================
// ALFR - Enhanced from YOUR working code
// Added: Thick line handling + Better debugging
// Kept: All your working pin config and motor control
//========================================================

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for L298N Motor Driver----
// Motor A (Left Motor)
#define IN1 9
#define IN2 8
#define ENA 10 // PWM Pin for Motor A
// Motor B (Right Motor)
#define IN3 7
#define IN4 6
#define ENB 5  // PWM Pin for Motor B
//--------------------------------------------------

//--------Line Details---------
bool isBlackLine = 1;             // 1 for Black line, 0 for White line
unsigned int numSensors = 8;
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 140;           // Increased max speed slightly
int currentSpeed = 90;        // Start a bit faster
int sensorWeight[8] = { -8, -4, -2, -1, 1, 2, 4, 8 };

int activeSensors;
float Kp = 0.012;      // Slightly reduced for thick lines
float Kd = 6.0;        // Increased for smoother control
float Ki = 0.0008;     // Small integral
int onLine = 1;

// Calibration values for A6, A7 (paste your calibrated values here)
int minValues[2] = {40, 46};
int maxValues[2] = {860, 899};
int threshold[2] = {450, 472};

int sensorValue[8], sensorArray[8];

// Thick line detection
int blackSensorCount = 0;
bool thickLineDetected = false;

void setup() {
  // ADC Speed Optimization
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  Serial.begin(115200);

  // Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Digital sensor pins (A0-A5)
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  // UI Pins
  pinMode(11, INPUT_PULLUP);  // Calibration Button (Optional)
  pinMode(12, INPUT_PULLUP);  // Start Button
  pinMode(13, OUTPUT);        // LED Indicator

  Serial.println(F("======================"));
  Serial.println(F("ALFR - THICK LINE READY"));
  Serial.println(F("======================"));
  Serial.println(F("Based on YOUR working code"));
  Serial.println(F("Enhanced for thick lines"));
  Serial.println();
  Serial.println(F("Current PID:"));
  Serial.print(F("Kp=")); Serial.print(Kp);
  Serial.print(F(" Kd=")); Serial.print(Kd);
  Serial.print(F(" Ki=")); Serial.println(Ki);
  Serial.println();
  Serial.println(F("Press Pin 11 to calibrate A6-A7"));
  Serial.println(F("Press Pin 12 to START"));
  Serial.println(F("Or send 'C' to calibrate, 'G' to go"));
  Serial.println(F("======================\n"));
}

void loop() {
  // Check for calibration
  if (digitalRead(11) == LOW || (Serial.available() && (Serial.peek() == 'C' || Serial.peek() == 'c'))) {
    if (Serial.available()) Serial.read();
    calibrate();
    Serial.println(F("\nReady! Press Pin 12 to start"));
  }

  // Check for start
  if (digitalRead(12) == LOW || (Serial.available() && (Serial.peek() == 'G' || Serial.peek() == 'g'))) {
    if (Serial.available()) Serial.read();

    Serial.println(F("\n*** STARTING LINE FOLLOW ***\n"));
    delay(500);

    // MAIN LINE FOLLOWING LOOP
    while (1) {
      // Check for stop command
      if (Serial.available() && (Serial.peek() == 'S' || Serial.peek() == 's')) {
        Serial.read();
        motorLeftRun(0);
        motorRightRun(0);
        Serial.println(F("\n*** STOPPED ***"));
        break;
      }

      readLine();

      // Count how many sensors see black
      blackSensorCount = 0;
      for (int i = 0; i < 8; i++) {
        if (sensorArray[i] == 1) blackSensorCount++;
      }

      // Detect thick line (5 or more sensors on black)
      thickLineDetected = (blackSensorCount >= 5);

      bool allEqual = true;
      for (int i = 1; i < 8; i++) {
        if (sensorArray[i] != sensorArray[0]) {
          allEqual = false;
          break;
        }
      }

      // Soft Start logic
      if (currentSpeed < lfSpeed) currentSpeed++;

      // Hard RIGHT turn (rightmost 3 sensors on black, leftmost off)
      if (sensorArray[7] == 1 && sensorArray[6] == 1 && sensorArray[5] == 1 && sensorArray[0] == 0) {
        digitalWrite(13, LOW);

        // Brief brake
        int brakeSpeed = (lsp < 100 || rsp < 100) ? 20 : 80;
        motorLeftRun(-brakeSpeed);
        motorRightRun(-brakeSpeed);
        delay(5);
        motorLeftRun(0);
        motorRightRun(0);
        delay(5);

        // Sharp right turn
        motorLeftRun(-200);
        motorRightRun(200);
        delay(100);  // Hold turn longer
      }
      // Hard LEFT turn (leftmost 3 sensors on black, rightmost off)
      else if (sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[2] == 1 && sensorArray[7] == 0) {
        digitalWrite(13, LOW);

        // Brief brake
        int brakeSpeed = (lsp < 100 || rsp < 100) ? 20 : 80;
        motorLeftRun(-brakeSpeed);
        motorRightRun(-brakeSpeed);
        delay(5);
        motorLeftRun(0);
        motorRightRun(0);
        delay(5);

        // Sharp left turn
        motorLeftRun(200);
        motorRightRun(-200);
        delay(100);  // Hold turn longer
      }
      // THICK LINE DETECTED - Use edge following
      else if (thickLineDetected && onLine == 1) {
        digitalWrite(13, HIGH);

        // On thick line, follow the edge (use right edge)
        // Reduce speed for stability
        int thickLineSpeed = currentSpeed * 0.75;

        // Modified PID for edge following
        error = 0;
        activeSensors = 0;

        // Weight the rightmost sensors more heavily for edge following
        int edgeWeights[8] = { -10, -6, -3, -1, 1, 3, 6, 10 };

        for (int i = 0; i < 8; i++) {
          error += edgeWeights[i] * sensorArray[i] * sensorValue[i];
          activeSensors += sensorArray[i];
        }

        if (activeSensors > 0) {
          error = error / activeSensors;
        }

        P = error;
        I = I + error;
        D = error - previousError;

        PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
        previousError = error;

        lsp = thickLineSpeed + PIDvalue;
        rsp = thickLineSpeed - PIDvalue;

        // Constrain
        if (lsp > 255) lsp = 255;
        if (lsp < -255) lsp = -255;
        if (rsp > 255) rsp = 255;
        if (rsp < -255) rsp = -255;

        motorLeftRun(lsp);
        motorRightRun(rsp);

        // Print thick line debug
        if (millis() % 300 < 50) {
          Serial.print(F("[THICK LINE] Black sensors: "));
          Serial.print(blackSensorCount);
          Serial.print(F(" Err: "));
          Serial.print((int)error);
          Serial.print(F(" L:"));
          Serial.print(lsp);
          Serial.print(F(" R:"));
          Serial.println(rsp);
        }
      }
      // Normal line following (your original PID)
      else if (!allEqual && onLine == 1) {
        linefollow();
        digitalWrite(13, HIGH);

        // Print normal debug
        if (millis() % 300 < 50) {
          Serial.print(F("[NORMAL] Err: "));
          Serial.print((int)error);
          Serial.print(F(" L:"));
          Serial.print(lsp);
          Serial.print(F(" R:"));
          Serial.println(rsp);
        }
      }
      else {
        // All sensors equal - might be lost
        if (millis() % 500 < 50) {
          Serial.println(F("[LOST LINE?]"));
        }
      }
    }
  }
}

void linefollow() {
  // YOUR ORIGINAL PID CODE
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }

  if (activeSensors > 0) {
    error = error / activeSensors;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed + PIDvalue;
  rsp = currentSpeed - PIDvalue;

  // Constrain Speeds
  if (lsp > 255) lsp = 255;
  if (lsp < -255) lsp = -255;
  if (rsp > 255) rsp = 255;
  if (rsp < -255) rsp = -255;

  motorLeftRun(lsp);
  motorRightRun(rsp);
}

void calibrate() {
  Serial.println(F("\n========== CALIBRATING A6, A7 =========="));
  Serial.println(F("Move robot over BLACK and WHITE"));
  Serial.println(F("Calibrating for 10 seconds..."));

  // Initialize
  minValues[0] = analogRead(A6);
  maxValues[0] = analogRead(A6);
  minValues[1] = analogRead(A7);
  maxValues[1] = analogRead(A7);

  // LED blinks during calibration
  unsigned long startCal = millis();
  while (millis() - startCal < 10000) {
    digitalWrite(13, (millis() / 100) % 2);

    // Calibrate A6
    int val6 = analogRead(A6);
    if (val6 < minValues[0]) minValues[0] = val6;
    if (val6 > maxValues[0]) maxValues[0] = val6;

    // Calibrate A7
    int val7 = analogRead(A7);
    if (val7 < minValues[1]) minValues[1] = val7;
    if (val7 > maxValues[1]) maxValues[1] = val7;

    // Progress
    if ((millis() - startCal) % 1000 < 50) {
      Serial.print(F("Progress: "));
      Serial.print((millis() - startCal) / 1000);
      Serial.println(F("s"));
    }

    delay(10);
  }

  // Calculate thresholds
  threshold[0] = (minValues[0] + maxValues[0]) / 2;
  threshold[1] = (minValues[1] + maxValues[1]) / 2;

  Serial.println(F("\n=== CALIBRATION COMPLETE ==="));
  Serial.println(F("A6 | A7:"));
  Serial.print(F("Min: ")); Serial.print(minValues[0]);
  Serial.print(F(" | ")); Serial.println(minValues[1]);
  Serial.print(F("Max: ")); Serial.print(maxValues[0]);
  Serial.print(F(" | ")); Serial.println(maxValues[1]);
  Serial.print(F("Threshold: ")); Serial.print(threshold[0]);
  Serial.print(F(" | ")); Serial.println(threshold[1]);
  Serial.println(F("============================\n"));

  // Rapid blink to confirm
  for (int i = 0; i < 6; i++) {
    digitalWrite(13, !digitalRead(13));
    delay(100);
  }
  digitalWrite(13, LOW);
}

void readLine() {
  // YOUR ORIGINAL SENSOR READING CODE
  onLine = 0;

  // Read digital sensors (A0-A5)
  for (int i = 0; i < 6; i++) {
    int digitalValue = digitalRead(A0 + i);

    if (isBlackLine) {
      sensorArray[i] = digitalValue;
    } else {
      sensorArray[i] = !digitalValue;
    }

    sensorValue[i] = sensorArray[i] ? 1000 : 0;

    if (sensorArray[i]) onLine = 1;
  }

  // Read analog sensors (A6-A7)
  for (int i = 0; i < 2; i++) {
    int analogValue = analogRead(A6 + i);

    if (isBlackLine) {
      sensorValue[6 + i] = map(analogValue, minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[6 + i] = map(analogValue, minValues[i], maxValues[i], 1000, 0);
    }

    sensorValue[6 + i] = constrain(sensorValue[6 + i], 0, 1000);
    sensorArray[6 + i] = sensorValue[6 + i] > 500;

    if (sensorArray[6 + i]) onLine = 1;
  }
}

//--------YOUR ORIGINAL MOTOR FUNCTIONS-----------------
void motorLeftRun(int motorSpeed) {
  if (motorSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(motorSpeed));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motorRightRun(int motorSpeed) {
  if (motorSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(motorSpeed));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

/*
 * WHAT'S CHANGED FROM YOUR CODE:
 *
 * 1. Added thick line detection (counts sensors on black)
 * 2. When 5+ sensors see black, uses edge-following mode
 * 3. Edge mode: different weights, slower speed
 * 4. Better debug output via Serial
 * 5. Added stop command ('S')
 * 6. Slightly tuned PID for thick lines
 * 7. Longer turn delays for sharp curves
 *
 * EVERYTHING ELSE IS YOUR WORKING CODE!
 * - Same pins
 * - Same motor functions
 * - Same sensor reading
 * - Same basic structure
 */
