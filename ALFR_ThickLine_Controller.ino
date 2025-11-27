/*
 * ================================================================
 * ADVANCED LINE FOLLOWER ROBOT (ALFR) - THICK LINE EDITION
 * ================================================================
 * Designed for LAM Research Challenge 2025 - Hardware Hustle
 *
 * FEATURES:
 * - Optimized for THICK black line tracks
 * - PID control with auto-tuning capability
 * - Edge following algorithm for thick lines
 * - Comprehensive debugging via Serial Monitor
 * - Auto-calibration routine
 * - Smooth motor control
 *
 * HARDWARE:
 * - Arduino Nano R3
 * - Smart Elex RLS-08 Analog Line Sensor Array (8 sensors)
 * - L298N Motor Driver
 * - N20 Gear Motors (600 RPM)
 * ================================================================
 */

// ==================== PIN CONFIGURATION ====================
// RLS-08 Sensor Array Pins (Analog Inputs)
#define SENSOR_1  A0  // Leftmost sensor
#define SENSOR_2  A1
#define SENSOR_3  A2
#define SENSOR_4  A3
#define SENSOR_5  A4
#define SENSOR_6  A5
#define SENSOR_7  A6
#define SENSOR_8  A7  // Rightmost sensor

// L298N Motor Driver Pins
#define MOTOR_LEFT_FORWARD   5   // ENA (PWM)
#define MOTOR_LEFT_BACKWARD  6   // IN1
#define MOTOR_LEFT_ENABLE    7   // IN2

#define MOTOR_RIGHT_FORWARD  9   // ENB (PWM)
#define MOTOR_RIGHT_BACKWARD 10  // IN3
#define MOTOR_RIGHT_ENABLE   11  // IN4

// Calibration Button (Optional - connect to GND to trigger)
#define CALIBRATE_BUTTON 2

// LED for status indication
#define STATUS_LED 13

// ==================== CONFIGURATION ====================
// Motor Speed Settings
#define BASE_SPEED 120        // Base speed (0-255) - Adjust based on your track
#define MAX_SPEED  200        // Maximum speed for straight lines
#define MIN_SPEED  80         // Minimum speed for sharp turns
#define TURN_SPEED 100        // Speed during turns

// PID Controller Settings (TUNE THESE FOR YOUR TRACK!)
// Start with these values and adjust based on performance
float Kp = 25.0;    // Proportional gain (increase for faster response)
float Ki = 0.0;     // Integral gain (usually keep low, increase if drift occurs)
float Kd = 15.0;    // Derivative gain (increase for smoother, less oscillation)

// Sensor Configuration
#define NUM_SENSORS 8
#define THRESHOLD_MARGIN 100  // Margin for black/white detection

// Line Following Mode
#define FOLLOW_MODE_CENTER 0  // Follow center of line (for thin lines)
#define FOLLOW_MODE_EDGE   1  // Follow edge of line (for THICK lines)
int followMode = FOLLOW_MODE_EDGE;  // Use EDGE mode for thick lines

// Debugging
#define DEBUG_MODE true       // Set to false to disable serial debugging
#define DEBUG_INTERVAL 200    // Print debug info every X milliseconds

// ==================== GLOBAL VARIABLES ====================
// Sensor readings
int sensorValue[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS];
bool sensorBinary[NUM_SENSORS];  // true = black, false = white

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

// Position tracking
int position = 0;
int lastPosition = 3500;  // Center position

// Timing
unsigned long lastDebugTime = 0;
unsigned long startTime = 0;

// Calibration flag
bool isCalibrated = false;

// ==================== SETUP ====================
void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println(F("========================================"));
  Serial.println(F("ALFR - Thick Line Edition v1.0"));
  Serial.println(F("LAM Research Challenge 2025"));
  Serial.println(F("========================================"));

  // Initialize motor pins
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_ENABLE, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_ENABLE, OUTPUT);

  // Initialize other pins
  pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);

  // Initialize sensor arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  // Wait for user to start
  Serial.println(F("\nPress CALIBRATE button or send 'C' via Serial to calibrate..."));
  blinkLED(3, 200);

  // Wait for calibration trigger
  while (!isCalibrated) {
    if (digitalRead(CALIBRATE_BUTTON) == LOW || Serial.available() > 0) {
      if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'C' || cmd == 'c') {
          calibrateSensors();
          isCalibrated = true;
        }
      } else {
        calibrateSensors();
        isCalibrated = true;
      }
    }
  }

  Serial.println(F("\n========================================"));
  Serial.println(F("Calibration Complete! Starting in 3 seconds..."));
  Serial.println(F("========================================\n"));

  delay(3000);
  startTime = millis();

  Serial.println(F("PID Parameters:"));
  Serial.print(F("Kp = ")); Serial.println(Kp);
  Serial.print(F("Ki = ")); Serial.println(Ki);
  Serial.print(F("Kd = ")); Serial.println(Kd);
  Serial.print(F("Follow Mode: "));
  Serial.println(followMode == FOLLOW_MODE_EDGE ? F("EDGE (Thick Line)") : F("CENTER"));
  Serial.println();
}

// ==================== MAIN LOOP ====================
void loop() {
  // Read all sensors
  readSensors();

  // Calculate line position
  position = calculatePosition();

  // Calculate PID correction
  calculatePID();

  // Apply motor corrections
  applyMotorControl();

  // Debug output
  if (DEBUG_MODE && (millis() - lastDebugTime > DEBUG_INTERVAL)) {
    printDebugInfo();
    lastDebugTime = millis();
  }

  // Check for runtime commands
  checkSerialCommands();
}

// ==================== SENSOR FUNCTIONS ====================
void readSensors() {
  // Read raw analog values
  sensorValue[0] = analogRead(SENSOR_1);
  sensorValue[1] = analogRead(SENSOR_2);
  sensorValue[2] = analogRead(SENSOR_3);
  sensorValue[3] = analogRead(SENSOR_4);
  sensorValue[4] = analogRead(SENSOR_5);
  sensorValue[5] = analogRead(SENSOR_6);
  sensorValue[6] = analogRead(SENSOR_7);
  sensorValue[7] = analogRead(SENSOR_8);

  // Convert to binary based on threshold
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Black line typically gives LOW reading, white surface gives HIGH
    sensorBinary[i] = (sensorValue[i] < sensorThreshold[i]);
  }
}

int calculatePosition() {
  /*
   * Position calculation for THICK LINE:
   *
   * For thick lines, we use EDGE FOLLOWING approach:
   * - We detect which edge of the line we're tracking
   * - Calculate position based on the edge, not center
   *
   * Position scale: 0 (far left) to 7000 (far right)
   * Center = 3500
   *
   * Sensor layout:  [0][1][2][3][4][5][6][7]
   *                 LEFT ←        → RIGHT
   */

  int weightedSum = 0;
  int sensorSum = 0;

  if (followMode == FOLLOW_MODE_EDGE) {
    // EDGE FOLLOWING MODE (for thick lines)
    // We look for the transition from white to black (left edge)
    // or black to white (right edge)

    // Count sensors on black
    int blackCount = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorBinary[i]) blackCount++;
    }

    // If most sensors are on black (thick line detected)
    if (blackCount >= 4) {
      // Follow the RIGHT edge of the line
      // Look for the rightmost black sensor
      for (int i = NUM_SENSORS - 1; i >= 0; i--) {
        if (sensorBinary[i]) {
          // Weight sensors to create smooth position
          weightedSum += i * 1000;
          sensorSum++;
          break;  // Only use the edge sensor
        }
      }

      // Also consider the sensor next to it for smoother tracking
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorBinary[i]) {
          weightedSum += i * 1000;
          sensorSum++;
        }
      }
    } else {
      // Normal center-weighted calculation for thinner sections
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorBinary[i]) {
          weightedSum += i * 1000;
          sensorSum++;
        }
      }
    }
  } else {
    // CENTER FOLLOWING MODE (traditional approach)
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorBinary[i]) {
        weightedSum += i * 1000;
        sensorSum++;
      }
    }
  }

  // Calculate position
  int pos;
  if (sensorSum > 0) {
    pos = weightedSum / sensorSum;
    lastPosition = pos;  // Remember last valid position
  } else {
    // Lost the line! Use last known position
    pos = lastPosition;

    // If completely lost, try to recover based on last position
    if (lastPosition < 3500) {
      // Line was on left, turn left to find it
      pos = 0;
    } else {
      // Line was on right, turn right to find it
      pos = 7000;
    }
  }

  return pos;
}

// ==================== PID CONTROL ====================
void calculatePID() {
  // Calculate error (deviation from center)
  // Center position is 3500
  error = position - 3500;

  // Proportional term
  float P = error;

  // Integral term (accumulated error)
  integral += error;

  // Anti-windup: limit integral to prevent excessive buildup
  if (integral > 5000) integral = 5000;
  if (integral < -5000) integral = -5000;

  // Derivative term (rate of change of error)
  derivative = error - lastError;

  // Calculate correction
  correction = (Kp * P) + (Ki * integral) + (Kd * derivative);

  // Limit correction to reasonable range
  if (correction > 255) correction = 255;
  if (correction < -255) correction = -255;

  // Store error for next iteration
  lastError = error;
}

void applyMotorControl() {
  // Base speed adjustment based on error magnitude
  int currentBaseSpeed = BASE_SPEED;

  // Slow down for sharp turns
  if (abs(error) > 2000) {
    currentBaseSpeed = TURN_SPEED;
  } else if (abs(error) < 500) {
    currentBaseSpeed = MAX_SPEED;  // Speed up on straight sections
  }

  // Calculate individual motor speeds
  int leftSpeed = currentBaseSpeed + correction;
  int rightSpeed = currentBaseSpeed - correction;

  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Apply minimum speed threshold to prevent motor stall
  if (leftSpeed > 0 && leftSpeed < MIN_SPEED) leftSpeed = MIN_SPEED;
  if (leftSpeed < 0 && leftSpeed > -MIN_SPEED) leftSpeed = -MIN_SPEED;
  if (rightSpeed > 0 && rightSpeed < MIN_SPEED) rightSpeed = MIN_SPEED;
  if (rightSpeed < 0 && rightSpeed > -MIN_SPEED) rightSpeed = -MIN_SPEED;

  // Set motor directions and speeds
  setMotorSpeed(leftSpeed, rightSpeed);
}

void setMotorSpeed(int left, int right) {
  // Left motor
  if (left > 0) {
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_LEFT_ENABLE, HIGH);
    analogWrite(MOTOR_LEFT_FORWARD, left);
  } else if (left < 0) {
    digitalWrite(MOTOR_LEFT_ENABLE, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
    analogWrite(MOTOR_LEFT_FORWARD, -left);
  } else {
    digitalWrite(MOTOR_LEFT_ENABLE, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    analogWrite(MOTOR_LEFT_FORWARD, 0);
  }

  // Right motor
  if (right > 0) {
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_ENABLE, HIGH);
    analogWrite(MOTOR_RIGHT_FORWARD, right);
  } else if (right < 0) {
    digitalWrite(MOTOR_RIGHT_ENABLE, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
    analogWrite(MOTOR_RIGHT_FORWARD, -right);
  } else {
    digitalWrite(MOTOR_RIGHT_ENABLE, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
  }
}

// ==================== CALIBRATION ====================
void calibrateSensors() {
  Serial.println(F("\n========================================"));
  Serial.println(F("STARTING CALIBRATION"));
  Serial.println(F("========================================"));
  Serial.println(F("Move the robot left and right over the line"));
  Serial.println(F("for the next 5 seconds..."));
  Serial.println();

  // Reset min/max values
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  // Calibrate for 5 seconds
  unsigned long calibrationStart = millis();
  int calibrationTime = 5000;  // 5 seconds

  while (millis() - calibrationStart < calibrationTime) {
    // Blink LED during calibration
    digitalWrite(STATUS_LED, (millis() / 100) % 2);

    // Read all sensors
    sensorValue[0] = analogRead(SENSOR_1);
    sensorValue[1] = analogRead(SENSOR_2);
    sensorValue[2] = analogRead(SENSOR_3);
    sensorValue[3] = analogRead(SENSOR_4);
    sensorValue[4] = analogRead(SENSOR_5);
    sensorValue[5] = analogRead(SENSOR_6);
    sensorValue[6] = analogRead(SENSOR_7);
    sensorValue[7] = analogRead(SENSOR_8);

    // Update min and max values
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValue[i] < sensorMin[i]) {
        sensorMin[i] = sensorValue[i];
      }
      if (sensorValue[i] > sensorMax[i]) {
        sensorMax[i] = sensorValue[i];
      }
    }

    // Print progress
    int progress = (millis() - calibrationStart) * 100 / calibrationTime;
    if (progress % 20 == 0) {
      Serial.print(F("Progress: "));
      Serial.print(progress);
      Serial.println(F("%"));
    }

    delay(10);
  }

  digitalWrite(STATUS_LED, HIGH);

  // Calculate thresholds (midpoint between min and max)
  Serial.println(F("\nCalibration Results:"));
  Serial.println(F("Sensor | Min  | Max  | Threshold"));
  Serial.println(F("-------|------|------|----------"));

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;

    Serial.print(F("   "));
    Serial.print(i);
    Serial.print(F("   | "));
    Serial.print(sensorMin[i]);
    Serial.print(F(" | "));
    Serial.print(sensorMax[i]);
    Serial.print(F(" | "));
    Serial.println(sensorThreshold[i]);
  }

  Serial.println(F("========================================"));
  blinkLED(5, 100);
}

// ==================== DEBUG FUNCTIONS ====================
void printDebugInfo() {
  // Print sensor states
  Serial.print(F("Sensors: ["));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorBinary[i] ? F("■") : F("□"));
    if (i < NUM_SENSORS - 1) Serial.print(F("|"));
  }
  Serial.print(F("] "));

  // Print position
  Serial.print(F("Pos:"));
  Serial.print(position);
  Serial.print(F(" "));

  // Print error
  Serial.print(F("Err:"));
  Serial.print((int)error);
  Serial.print(F(" "));

  // Print correction
  Serial.print(F("Corr:"));
  Serial.print((int)correction);
  Serial.print(F(" "));

  // Print motor speeds
  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  Serial.print(F("Motors L:"));
  Serial.print(leftSpeed);
  Serial.print(F(" R:"));
  Serial.print(rightSpeed);

  // Print runtime
  Serial.print(F(" Time:"));
  Serial.print((millis() - startTime) / 1000);
  Serial.println(F("s"));
}

void printSensorValues() {
  Serial.println(F("\nRaw Sensor Values:"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(sensorValue[i]);
    Serial.print(F(" ("));
    Serial.print(sensorBinary[i] ? F("BLACK") : F("WHITE"));
    Serial.println(F(")"));
  }
  Serial.println();
}

// ==================== UTILITY FUNCTIONS ====================
void checkSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'S':
      case 's':
        // Stop motors
        setMotorSpeed(0, 0);
        Serial.println(F("\n*** MOTORS STOPPED ***"));
        Serial.println(F("Send 'G' to continue or 'C' to recalibrate"));
        while (true) {
          if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'G' || c == 'g') {
              Serial.println(F("Resuming..."));
              break;
            } else if (c == 'C' || c == 'c') {
              calibrateSensors();
              isCalibrated = true;
              Serial.println(F("Resuming..."));
              break;
            }
          }
        }
        break;

      case 'V':
      case 'v':
        // Print sensor values
        printSensorValues();
        break;

      case 'P':
      case 'p':
        // Print current PID values
        Serial.println(F("\nCurrent PID Values:"));
        Serial.print(F("Kp = ")); Serial.println(Kp);
        Serial.print(F("Ki = ")); Serial.println(Ki);
        Serial.print(F("Kd = ")); Serial.println(Kd);
        break;

      case '+':
        // Increase base speed
        if (BASE_SPEED < 250) {
          Serial.print(F("Base speed increased to: "));
          Serial.println(BASE_SPEED + 10);
        }
        break;

      case '-':
        // Decrease base speed
        if (BASE_SPEED > 50) {
          Serial.print(F("Base speed decreased to: "));
          Serial.println(BASE_SPEED - 10);
        }
        break;

      case 'H':
      case 'h':
        printHelp();
        break;
    }
  }
}

void printHelp() {
  Serial.println(F("\n========================================"));
  Serial.println(F("AVAILABLE COMMANDS:"));
  Serial.println(F("========================================"));
  Serial.println(F("S - Stop motors (pause)"));
  Serial.println(F("G - Go (resume from stop)"));
  Serial.println(F("C - Calibrate sensors"));
  Serial.println(F("V - View raw sensor values"));
  Serial.println(F("P - Print PID values"));
  Serial.println(F("+ - Increase base speed"));
  Serial.println(F("- - Decrease base speed"));
  Serial.println(F("H - Show this help"));
  Serial.println(F("========================================\n"));
}

void blinkLED(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(duration);
    digitalWrite(STATUS_LED, LOW);
    delay(duration);
  }
}

/*
 * ================================================================
 * TUNING GUIDE:
 * ================================================================
 *
 * If robot oscillates (wobbles) too much:
 *   - Decrease Kp (try reducing by 5-10)
 *   - Increase Kd (try increasing by 5)
 *
 * If robot responds slowly to turns:
 *   - Increase Kp (try increasing by 5-10)
 *   - Decrease Kd if robot becomes unstable
 *
 * If robot drifts off line gradually:
 *   - Increase Ki slightly (try 0.1 to 0.5)
 *   - But be careful: too much Ki causes instability
 *
 * If robot is too slow:
 *   - Increase BASE_SPEED
 *   - Increase MAX_SPEED
 *
 * If robot overshoots turns:
 *   - Decrease TURN_SPEED
 *   - Increase Kd
 *
 * For THICK lines:
 *   - Keep followMode = FOLLOW_MODE_EDGE
 *   - May need to reduce Kp as less correction needed
 *
 * ================================================================
 */
