/*
 * ================================================================
 * ALFR - THICK LINE EDITION (FIXED VERSION)
 * ================================================================
 * Uses CORRECT pin configuration and sensor reading method
 * - A0-A5: DIGITAL sensors (digitalRead)
 * - A6-A7: ANALOG sensors (analogRead with calibration)
 * - Correct L298N motor pins
 * ================================================================
 */

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// ==================== PIN CONFIGURATION ====================
// L298N Motor Driver - LEFT MOTOR (Motor A)
#define IN1 9   // Left motor direction 1
#define IN2 8   // Left motor direction 2
#define ENA 10  // Left motor speed (PWM)

// L298N Motor Driver - RIGHT MOTOR (Motor B)
#define IN3 7   // Right motor direction 1
#define IN4 6   // Right motor direction 2
#define ENB 5   // Right motor speed (PWM)

// Buttons and LED
#define CALIBRATE_BUTTON 11  // Optional calibration button
#define START_BUTTON 12      // Start button
#define LED_PIN 13          // Status LED

// ==================== CONFIGURATION ====================
bool isBlackLine = true;     // true for black line on white background
unsigned int numSensors = 8;

// Speed Settings - INCREASED FOR BETTER MOVEMENT
int baseSpeed = 140;         // Base speed for line following
int maxSpeed = 200;          // Maximum speed on straights
int turnSpeed = 120;         // Speed for sharp turns
int currentSpeed = 120;      // Start at decent speed

// PID Tuning - OPTIMIZED FOR THICK LINES
float Kp = 0.015;    // Reduced for thick lines (less aggressive)
float Ki = 0.0005;   // Small integral
float Kd = 8.0;      // High derivative for smooth control

// Sensor weights for position calculation
// Negative on left, positive on right
int sensorWeight[8] = {-8, -4, -2, -1, 1, 2, 4, 8};

// ==================== GLOBAL VARIABLES ====================
int sensorValue[8];      // Raw sensor values (0-1000 scale)
int sensorBinary[8];     // Binary: 1 = on line, 0 = off line

// Calibration values for analog sensors (A6, A7 only)
int minValues[2] = {40, 46};    // Default mins for A6, A7
int maxValues[2] = {860, 899};  // Default maxs for A6, A7
int threshold[2] = {450, 472};  // Default thresholds

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float PIDvalue = 0;

int leftSpeed = 0;
int rightSpeed = 0;

// Line detection
bool onLine = false;
bool allSensorsBlack = false;
bool allSensorsWhite = false;

// Timing
unsigned long lastPrint = 0;
unsigned long startTime = 0;

// ==================== SETUP ====================
void setup() {
  // Optimize ADC for faster analog reads
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(115200);
  Serial.println(F("========================================"));
  Serial.println(F("ALFR - Thick Line Edition (FIXED)"));
  Serial.println(F("========================================"));

  // Motor pins
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
  // A6, A7 are analog-only, no pinMode needed

  // Button and LED pins
  pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Serial.println(F("\nSensor Configuration:"));
  Serial.println(F("  A0-A5: DIGITAL sensors"));
  Serial.println(F("  A6-A7: ANALOG sensors (with calibration)"));
  Serial.println(F("\nCurrent A6-A7 calibration:"));
  printCalibration();

  Serial.println(F("\n========================================"));
  Serial.println(F("OPTIONS:"));
  Serial.println(F("1. Press PIN 11 to CALIBRATE A6-A7"));
  Serial.println(F("2. Press PIN 12 to START with current values"));
  Serial.println(F("3. Send 'C' via Serial to calibrate"));
  Serial.println(F("4. Send 'G' via Serial to start"));
  Serial.println(F("========================================\n"));

  // Blink LED to show ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// ==================== MAIN LOOP ====================
void loop() {
  // Check for calibration button or serial command
  if (digitalRead(CALIBRATE_BUTTON) == LOW || Serial.available() > 0) {
    char cmd = Serial.read();
    if (digitalRead(CALIBRATE_BUTTON) == LOW || cmd == 'C' || cmd == 'c') {
      calibrateAnalogSensors();
      delay(500);
    }
  }

  // Check for start button or serial command
  if (digitalRead(START_BUTTON) == LOW || Serial.available() > 0) {
    char cmd = Serial.read();
    if (digitalRead(START_BUTTON) == LOW || cmd == 'G' || cmd == 'g') {
      startLineFollowing();
    }
  }
}

// ==================== LINE FOLLOWING ====================
void startLineFollowing() {
  Serial.println(F("\n========================================"));
  Serial.println(F("STARTING LINE FOLLOWING!"));
  Serial.println(F("Send 'S' to stop anytime"));
  Serial.println(F("========================================\n"));

  digitalWrite(LED_PIN, HIGH);
  delay(1000);

  startTime = millis();
  currentSpeed = baseSpeed - 20; // Start slow

  while (1) {
    // Check for stop command
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 'S' || cmd == 's') {
        stopMotors();
        Serial.println(F("\n*** STOPPED ***"));
        digitalWrite(LED_PIN, LOW);
        delay(500);
        return;
      }
    }

    // Read all sensors
    readAllSensors();

    // Soft speed ramp-up
    if (currentSpeed < baseSpeed) {
      currentSpeed++;
    }

    // Detect special cases
    checkSpecialCases();

    // Handle sharp turns (hard-coded for reliability)
    if (sensorBinary[0] && sensorBinary[1] && sensorBinary[2] && !sensorBinary[7]) {
      // Hard LEFT turn detected
      hardLeft();
    }
    else if (sensorBinary[7] && sensorBinary[6] && sensorBinary[5] && !sensorBinary[0]) {
      // Hard RIGHT turn detected
      hardRight();
    }
    else if (onLine) {
      // Normal PID line following
      calculatePID();
      applyMotorControl();
    }
    else if (allSensorsWhite) {
      // Lost line - search based on last error
      searchForLine();
    }
    else if (allSensorsBlack) {
      // On thick line - slow down and follow edge
      followThickLine();
    }

    // Debug output
    if (millis() - lastPrint > 200) {
      printDebug();
      lastPrint = millis();
    }
  }
}

// ==================== SENSOR READING ====================
void readAllSensors() {
  onLine = false;
  int blackCount = 0;
  int whiteCount = 0;

  // Read DIGITAL sensors (A0-A5)
  for (int i = 0; i < 6; i++) {
    int digitalValue = digitalRead(A0 + i);

    // For black line: HIGH (1) means black detected
    if (isBlackLine) {
      sensorBinary[i] = digitalValue;
    } else {
      sensorBinary[i] = !digitalValue;
    }

    // Set sensor value for PID (0 or 1000)
    sensorValue[i] = sensorBinary[i] ? 1000 : 0;

    if (sensorBinary[i]) {
      onLine = true;
      blackCount++;
    } else {
      whiteCount++;
    }
  }

  // Read ANALOG sensors (A6-A7) with calibration
  for (int i = 0; i < 2; i++) {
    int analogValue = analogRead(A6 + i);

    // Map to 0-1000 range
    if (isBlackLine) {
      sensorValue[6 + i] = map(analogValue, minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[6 + i] = map(analogValue, minValues[i], maxValues[i], 1000, 0);
    }

    // Constrain to valid range
    sensorValue[6 + i] = constrain(sensorValue[6 + i], 0, 1000);

    // Binary threshold at 500
    sensorBinary[6 + i] = (sensorValue[6 + i] > 500);

    if (sensorBinary[6 + i]) {
      onLine = true;
      blackCount++;
    } else {
      whiteCount++;
    }
  }

  // Check special conditions
  allSensorsBlack = (blackCount >= 6);  // Most sensors see black
  allSensorsWhite = (whiteCount >= 7);  // Almost all see white
}

void checkSpecialCases() {
  // You can add special detection logic here
  // For example: stop markers, intersections, etc.
}

// ==================== PID CONTROL ====================
void calculatePID() {
  // Calculate weighted position
  error = 0;
  int activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    if (sensorBinary[i]) {
      error += sensorWeight[i] * sensorValue[i];
      activeSensors++;
    }
  }

  if (activeSensors > 0) {
    error = error / activeSensors;
  } else {
    error = lastError;  // Use last known error if lost
  }

  // PID calculation
  integral += error;

  // Anti-windup
  if (integral > 5000) integral = 5000;
  if (integral < -5000) integral = -5000;

  derivative = error - lastError;

  PIDvalue = (Kp * error) + (Ki * integral) + (Kd * derivative);

  lastError = error;
}

void applyMotorControl() {
  // Calculate motor speeds
  leftSpeed = currentSpeed + PIDvalue;
  rightSpeed = currentSpeed - PIDvalue;

  // Constrain to valid range
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Apply speeds
  motorLeft(leftSpeed);
  motorRight(rightSpeed);
}

// ==================== THICK LINE HANDLING ====================
void followThickLine() {
  // When most sensors see black (thick line), slow down
  // and use edge-following logic

  Serial.println(F("[THICK LINE DETECTED]"));

  // Slow down
  int slowSpeed = baseSpeed * 0.6;

  // Try to follow the right edge
  if (sensorBinary[7] == 0) {
    // Right edge visible, steer towards it
    motorLeft(slowSpeed);
    motorRight(slowSpeed * 0.5);
  } else if (sensorBinary[0] == 0) {
    // Left edge visible, steer away
    motorLeft(slowSpeed * 0.5);
    motorRight(slowSpeed);
  } else {
    // All black, go straight slowly
    motorLeft(slowSpeed);
    motorRight(slowSpeed);
  }
}

// ==================== SPECIAL MANEUVERS ====================
void hardLeft() {
  Serial.println(F("[HARD LEFT]"));

  // Brief brake
  motorLeft(-50);
  motorRight(-50);
  delay(10);

  // Sharp left turn
  motorLeft(-turnSpeed);
  motorRight(turnSpeed);
  delay(50);
}

void hardRight() {
  Serial.println(F("[HARD RIGHT]"));

  // Brief brake
  motorLeft(-50);
  motorRight(-50);
  delay(10);

  // Sharp right turn
  motorLeft(turnSpeed);
  motorRight(-turnSpeed);
  delay(50);
}

void searchForLine() {
  // Lost line - search based on last error
  if (lastError < 0) {
    // Line was on left
    motorLeft(-baseSpeed * 0.5);
    motorRight(baseSpeed * 0.5);
  } else {
    // Line was on right
    motorLeft(baseSpeed * 0.5);
    motorRight(-baseSpeed * 0.5);
  }
}

// ==================== MOTOR CONTROL ====================
void motorLeft(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(speed));
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(speed));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motorRight(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, abs(speed));
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(speed));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  motorLeft(0);
  motorRight(0);
}

// ==================== CALIBRATION ====================
void calibrateAnalogSensors() {
  Serial.println(F("\n========================================"));
  Serial.println(F("CALIBRATING ANALOG SENSORS (A6, A7)"));
  Serial.println(F("========================================"));
  Serial.println(F("Move robot over BLACK and WHITE"));
  Serial.println(F("Calibrating for 10 seconds..."));

  // Initialize with first reading
  minValues[0] = analogRead(A6);
  maxValues[0] = analogRead(A6);
  minValues[1] = analogRead(A7);
  maxValues[1] = analogRead(A7);

  // Calibrate for 10 seconds
  unsigned long startCal = millis();
  while (millis() - startCal < 10000) {
    // Blink LED
    digitalWrite(LED_PIN, (millis() / 100) % 2);

    // Read A6
    int val6 = analogRead(A6);
    if (val6 < minValues[0]) minValues[0] = val6;
    if (val6 > maxValues[0]) maxValues[0] = val6;

    // Read A7
    int val7 = analogRead(A7);
    if (val7 < minValues[1]) minValues[1] = val7;
    if (val7 > maxValues[1]) maxValues[1] = val7;

    // Print progress
    if ((millis() - startCal) % 1000 == 0) {
      Serial.print(F("Progress: "));
      Serial.print((millis() - startCal) / 1000);
      Serial.println(F("s"));
    }

    delay(10);
  }

  // Calculate thresholds
  threshold[0] = (minValues[0] + maxValues[0]) / 2;
  threshold[1] = (minValues[1] + maxValues[1]) / 2;

  digitalWrite(LED_PIN, HIGH);

  Serial.println(F("\nCALIBRATION COMPLETE!"));
  printCalibration();

  // Blink rapidly to confirm
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
  digitalWrite(LED_PIN, LOW);
}

void printCalibration() {
  Serial.println(F("\nA6 | A7 Values:"));
  Serial.print(F("Min: "));
  Serial.print(minValues[0]);
  Serial.print(F(" | "));
  Serial.println(minValues[1]);

  Serial.print(F("Max: "));
  Serial.print(maxValues[0]);
  Serial.print(F(" | "));
  Serial.println(maxValues[1]);

  Serial.print(F("Threshold: "));
  Serial.print(threshold[0]);
  Serial.print(F(" | "));
  Serial.println(threshold[1]);
}

// ==================== DEBUG OUTPUT ====================
void printDebug() {
  // Print sensor array
  Serial.print(F("Sensors: ["));
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorBinary[i]);
    if (i < 7) Serial.print(F("|"));
  }
  Serial.print(F("] "));

  // Print error and PID
  Serial.print(F("Err:"));
  Serial.print((int)error);
  Serial.print(F(" PID:"));
  Serial.print((int)PIDvalue);
  Serial.print(F(" "));

  // Print motor speeds
  Serial.print(F("L:"));
  Serial.print(leftSpeed);
  Serial.print(F(" R:"));
  Serial.print(rightSpeed);
  Serial.print(F(" "));

  // Print status
  if (allSensorsBlack) Serial.print(F("[THICK]"));
  if (allSensorsWhite) Serial.print(F("[LOST]"));
  if (onLine) Serial.print(F("[OK]"));

  // Print time
  Serial.print(F(" T:"));
  Serial.print((millis() - startTime) / 1000);
  Serial.println(F("s"));
}

/*
 * ================================================================
 * TUNING GUIDE FOR THICK LINES:
 * ================================================================
 *
 * Current settings are CONSERVATIVE for thick lines.
 *
 * If robot is too slow:
 *   Increase baseSpeed (currently 100)
 *
 * If robot wobbles:
 *   Decrease Kp (currently 0.015)
 *   Increase Kd (currently 8.0)
 *
 * If robot doesn't respond to turns:
 *   Increase Kp slightly
 *
 * If robot loses line on curves:
 *   Decrease speed
 *   Increase turnSpeed delay in hardLeft/hardRight
 *
 * ================================================================
 */
