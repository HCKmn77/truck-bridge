#include <Arduino.h>
#include <Servo.h>

Servo myServo;
const int servoPin = 3; // Servo signal pin
int angle = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial1.println("Serial1-DEBUG connected to servo control.");
  myServo.attach(servoPin);
  Serial.println("Servo control ready! Enter angle (0-180):");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any extra whitespace

    if (input.length() > 0) {
      int newAngle = input.toInt();

      // Check if input is valid (0-180)
      if (newAngle >= 0 && newAngle <= 180) {
        angle = newAngle;
        myServo.write(angle);
        Serial.print("Moved to: ");
        Serial.println(angle);
        Serial1.print("DEBUG: Moved to: ");
        Serial1.println(angle);
      } else {
        Serial.println("Invalid angle! Please enter 0-180.");
        Serial1.println("DEBUG: Invalid angle! Please enter 0-180.");
      }
    }
  }
}
