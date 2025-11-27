/*
 * ================================================================
 * SIMPLE MOTOR TEST - NO SENSORS
 * ================================================================
 * This code will make the robot move forward immediately on power-up
 * Use this to verify your motors are wired correctly
 * ================================================================
 */

// L298N Motor Driver Pins - LEFT MOTOR (Motor A)
#define IN1 9   // Left motor direction 1
#define IN2 8   // Left motor direction 2
#define ENA 10  // Left motor speed (PWM)

// L298N Motor Driver Pins - RIGHT MOTOR (Motor B)
#define IN3 7   // Right motor direction 1
#define IN4 6   // Right motor direction 2
#define ENB 5   // Right motor speed (PWM)

#define LED_PIN 13

void setup() {
  Serial.begin(115200);

  // Setup motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  Serial.println(F("========================================"));
  Serial.println(F("MOTOR TEST - Starting in 2 seconds"));
  Serial.println(F("========================================"));
  Serial.println(F("The robot will:"));
  Serial.println(F("1. Move FORWARD for 2 seconds"));
  Serial.println(F("2. Move BACKWARD for 2 seconds"));
  Serial.println(F("3. Turn LEFT for 1 second"));
  Serial.println(F("4. Turn RIGHT for 1 second"));
  Serial.println(F("5. STOP"));
  Serial.println(F("========================================\n"));

  delay(2000);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);

  // TEST 1: Forward
  Serial.println(F("TEST 1: Moving FORWARD..."));
  motorLeft(200);   // Full speed forward
  motorRight(200);  // Full speed forward
  delay(2000);

  // Stop briefly
  Serial.println(F("Stopping..."));
  motorLeft(0);
  motorRight(0);
  delay(500);

  // TEST 2: Backward
  Serial.println(F("TEST 2: Moving BACKWARD..."));
  motorLeft(-200);  // Full speed backward
  motorRight(-200); // Full speed backward
  delay(2000);

  // Stop briefly
  Serial.println(F("Stopping..."));
  motorLeft(0);
  motorRight(0);
  delay(500);

  // TEST 3: Turn Left
  Serial.println(F("TEST 3: Turning LEFT..."));
  motorLeft(-150);  // Left backward
  motorRight(150);  // Right forward
  delay(1000);

  // Stop briefly
  Serial.println(F("Stopping..."));
  motorLeft(0);
  motorRight(0);
  delay(500);

  // TEST 4: Turn Right
  Serial.println(F("TEST 4: Turning RIGHT..."));
  motorLeft(150);   // Left forward
  motorRight(-150); // Right backward
  delay(1000);

  // TEST 5: Stop
  Serial.println(F("TEST 5: STOPPING"));
  motorLeft(0);
  motorRight(0);

  digitalWrite(LED_PIN, LOW);

  Serial.println(F("\n========================================"));
  Serial.println(F("MOTOR TEST COMPLETE!"));
  Serial.println(F("========================================"));
  Serial.println(F("Did the motors move?"));
  Serial.println(F("  YES: Motors are wired correctly!"));
  Serial.println(F("  NO: Check your wiring:"));
  Serial.println(F("    - L298N ENA -> Arduino Pin 10"));
  Serial.println(F("    - L298N IN1 -> Arduino Pin 9"));
  Serial.println(F("    - L298N IN2 -> Arduino Pin 8"));
  Serial.println(F("    - L298N ENB -> Arduino Pin 5"));
  Serial.println(F("    - L298N IN3 -> Arduino Pin 7"));
  Serial.println(F("    - L298N IN4 -> Arduino Pin 6"));
  Serial.println(F("    - Make sure L298N has power!"));
  Serial.println(F("    - Remove jumpers on ENA/ENB if present"));
  Serial.println(F("========================================\n"));

  // Wait 5 seconds before repeating
  Serial.println(F("Repeating test in 5 seconds...\n"));
  delay(5000);
}

// Motor control functions
void motorLeft(int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(speed));
  } else if (speed < 0) {
    // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(speed));
  } else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motorRight(int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, abs(speed));
  } else if (speed < 0) {
    // Backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(speed));
  } else {
    // Stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}
