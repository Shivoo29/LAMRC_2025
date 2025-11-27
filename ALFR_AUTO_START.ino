//========================================================
// ALFR - AUTO START VERSION (No Buttons Required!)
// Just upload, disconnect USB, flip power switch = GO!
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

//--------Line Configuration---------
bool isBlackLine = 1;             // 1 for Black line, 0 for White line
unsigned int numSensors = 8;
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 130;           // Max speed
int currentSpeed = 85;        // Starting speed
int sensorWeight[8] = { -8, -4, -2, -1, 1, 2, 4, 8 };

int activeSensors;
float Kp = 0.012;     // Slightly reduced for thick lines
float Kd = 6.0;       // Increased for smooth control
float Ki = 0.0008;    // Small integral
int onLine = 1;

// *** PASTE YOUR CALIBRATED VALUES HERE (A6, A7 only) ***
int minValues[2] = {40, 46};      // A6, A7 min values
int maxValues[2] = {860, 899};    // A6, A7 max values
int threshold[2] = {450, 472};    // A6, A7 thresholds
// *** If you calibrated, update these numbers above ***

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

  // LED Indicator
  pinMode(13, OUTPUT);

  Serial.println(F("========================================"));
  Serial.println(F("ALFR - AUTO START (Thick Line Edition)"));
  Serial.println(F("========================================"));
  Serial.println(F("Using pre-saved calibration values"));
  Serial.println(F("A6 min/max: "));
  Serial.print(minValues[0]); Serial.print(F("/")); Serial.println(maxValues[0]);
  Serial.println(F("A7 min/max: "));
  Serial.print(minValues[1]); Serial.print(F("/")); Serial.println(maxValues[1]);
  Serial.println(F("\nStarting in 3 seconds..."));
  Serial.println(F("Place robot on line NOW!"));
  Serial.println(F("========================================\n"));

  // 3 second countdown with LED blinks
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    Serial.println(F("..."));
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
  }

  Serial.println(F("*** STARTING NOW! ***\n"));
  digitalWrite(13, HIGH);
  delay(200);
}

void loop() {
  // NO BUTTONS - Just start immediately!

  // MAIN LINE FOLLOWING LOOP (runs forever)
  while (1) {
    readLine();

    // Count sensors seeing black
    blackSensorCount = 0;
    for (int i = 0; i < 8; i++) {
      if (sensorArray[i] == 1) blackSensorCount++;
    }

    // Detect thick line (5+ sensors on black)
    thickLineDetected = (blackSensorCount >= 5);

    // Check if all sensors are equal
    bool allEqual = true;
    for (int i = 1; i < 8; i++) {
      if (sensorArray[i] != sensorArray[0]) {
        allEqual = false;
        break;
      }
    }

    // Soft speed ramp-up
    if (currentSpeed < lfSpeed) currentSpeed++;

    // ===== HARD RIGHT TURN =====
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
      motorLeftRun(-220);
      motorRightRun(220);
      delay(80);  // Hold turn
    }
    // ===== HARD LEFT TURN =====
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
      motorLeftRun(220);
      motorRightRun(-220);
      delay(80);  // Hold turn
    }
    // ===== THICK LINE DETECTED - Edge Following =====
    else if (thickLineDetected && onLine == 1) {
      digitalWrite(13, HIGH);

      // Reduce speed for thick line stability
      int thickSpeed = currentSpeed * 0.70;

      // Modified sensor weights for edge following
      error = 0;
      activeSensors = 0;
      int edgeWeights[8] = { -10, -6, -3, -1, 1, 3, 6, 10 };

      for (int i = 0; i < 8; i++) {
        error += edgeWeights[i] * sensorArray[i] * sensorValue[i];
        activeSensors += sensorArray[i];
      }

      if (activeSensors > 0) {
        error = error / activeSensors;
      }

      // PID calculation
      P = error;
      I = I + error;

      // Anti-windup
      if (I > 3000) I = 3000;
      if (I < -3000) I = -3000;

      D = error - previousError;

      PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
      previousError = error;

      lsp = thickSpeed + PIDvalue;
      rsp = thickSpeed - PIDvalue;

      // Constrain speeds
      if (lsp > 255) lsp = 255;
      if (lsp < -255) lsp = -255;
      if (rsp > 255) rsp = 255;
      if (rsp < -255) rsp = -255;

      motorLeftRun(lsp);
      motorRightRun(rsp);

      // Debug output (reduced frequency to avoid serial slowdown)
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 500) {
        Serial.print(F("[THICK] Black:"));
        Serial.print(blackSensorCount);
        Serial.print(F(" Err:"));
        Serial.print((int)error);
        Serial.print(F(" L:"));
        Serial.print(lsp);
        Serial.print(F(" R:"));
        Serial.println(rsp);
        lastPrint = millis();
      }
    }
    // ===== NORMAL LINE FOLLOWING (PID) =====
    else if (!allEqual && onLine == 1) {
      linefollow();
      digitalWrite(13, HIGH);

      // Debug output
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 500) {
        Serial.print(F("[NORMAL] Err:"));
        Serial.print((int)error);
        Serial.print(F(" L:"));
        Serial.print(lsp);
        Serial.print(F(" R:"));
        Serial.println(rsp);
        lastPrint = millis();
      }
    }
    // ===== LINE LOST =====
    else {
      // Keep last motor speeds or search
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 1000) {
        Serial.println(F("[SEARCHING...]"));
        lastPrint = millis();
      }
    }
  }
}

void linefollow() {
  // Standard PID line following
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

  // Anti-windup
  if (I > 3000) I = 3000;
  if (I < -3000) I = -3000;

  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed + PIDvalue;
  rsp = currentSpeed - PIDvalue;

  // Constrain speeds
  if (lsp > 255) lsp = 255;
  if (lsp < -255) lsp = -255;
  if (rsp > 255) rsp = 255;
  if (rsp < -255) rsp = -255;

  motorLeftRun(lsp);
  motorRightRun(rsp);
}

void readLine() {
  onLine = 0;

  // Read DIGITAL sensors (A0-A5)
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

  // Read ANALOG sensors (A6-A7)
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

//--------Motor Functions (Unchanged from your code)-----------------
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
 * ================================================================
 * HOW TO USE (NO BUTTONS NEEDED):
 * ================================================================
 *
 * 1. Update calibration values at top (minValues, maxValues)
 *    - If you already calibrated, paste those values
 *    - Or use default values (40, 46, 860, 899)
 *
 * 2. Upload this code to Arduino Nano via USB
 *
 * 3. Wait for upload to complete
 *
 * 4. Disconnect USB cable
 *
 * 5. Place robot on the black line
 *
 * 6. Turn ON the power switch (LiPo)
 *
 * 7. Robot will:
 *    - Wait 3 seconds (LED blinks)
 *    - Start following line automatically!
 *
 * NO BUTTONS NEEDED - Just power on and go!
 *
 * ================================================================
 * WHAT'S IMPROVED:
 * ================================================================
 *
 * - AUTO START on power up (no buttons!)
 * - Thick line detection (5+ sensors = thick line)
 * - Edge following mode for thick lines
 * - Modified sensor weights for edge tracking
 * - Reduced speed on thick lines for stability
 * - Better turn handling with braking
 * - Anti-windup on integral term
 * - Debug output to Serial (optional, won't affect performance)
 *
 * ================================================================
 */
