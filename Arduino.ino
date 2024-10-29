#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);  // 16x2 LCD


// Define pins for stepper motor and servo motor
const int stepPin = 3;
const int dirPin = 2;
const int servoPin = 4;

// Initialize Servo object
Servo tiltServo;

// Variables for motor control
int panAngle = 90;
int tiltAngle = 90;

// Previous positions for optimization
int prevPanAngle = 270;
int prevTiltAngle = 90;

// Define stepper motor states for half-stepping
const int stepperHalfStepSequence[8][2] = {
  {1, 0}, {1, 1}, {0, 1}, {0, 0},
  {0, 1}, {1, 1}, {1, 0}, {0, 0}
};

void setup() {
  Serial.begin(9600);
  // Initialize the LCD
  lcd.init();
  
  // Turn on the backlight
  lcd.backlight();
  
  // Print a message to the LCD.
  lcd.setCursor(0, 0);  // Start at the first column and first row
  lcd.print("Hello, World!");
  
  // Stepper motor pins setup
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  // Attach servo motor
  tiltServo.attach(servoPin);
  tiltServo.write(tiltAngle);  // Initialize tilt angle
}

void loop() {
  digitalWrite(6, HIGH);  // Microstepping control for DRV8825
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int xIndex = data.indexOf('X');
    int yIndex = data.indexOf('Y');
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Tareget Locked");
    
    if (xIndex != -1 && yIndex != -1) {
      panAngle = data.substring(xIndex + 1, yIndex).toInt();
      tiltAngle = data.substring(yIndex + 1).toInt();
      lcd.setCursor(0,1);
      lcd.print("X:");
      lcd.print(panAngle);
      lcd.print(" Y:");
      lcd.print(tiltAngle);
      delay(1);
      
      // Move stepper motor for pan control
      if (panAngle != prevPanAngle) {
        int steps = abs(panAngle - prevPanAngle); // Calculate the steps
        bool clockwise = panAngle > prevPanAngle;
        rotateStepper(steps * 8, clockwise); 
        prevPanAngle = panAngle;
      }

      // Move servo for tilt control
      if (tiltAngle != prevTiltAngle) {
        tiltServo.write(tiltAngle);
        prevTiltAngle = tiltAngle;
      }
    }
  }
  else{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Tareget NOT Locked");
  }
}

void rotateStepper(int steps, bool clockwise) {
  digitalWrite(dirPin, clockwise ? HIGH : LOW);
  
  // Set the delay between steps (in microseconds)
  int stepDelay = 125; // Decrease this value to increase speed (4 times faster than 500)

  // Perform the rotation
  for (int i = 0; i < steps; i++) {
    // Determine the current half-step index
    int stepIndex = (i % 8);
    
    // Set the stepper motor pins according to the half-step sequence
    digitalWrite(stepPin, stepperHalfStepSequence[stepIndex][0]);
    delayMicroseconds(stepDelay); // Delay for half step
    digitalWrite(stepPin, stepperHalfStepSequence[stepIndex][1]);
    delayMicroseconds(stepDelay); // Delay for half step
  }
}
