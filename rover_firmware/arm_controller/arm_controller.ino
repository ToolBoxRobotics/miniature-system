/*
 * ROVER ARM CONTROLLER - Arduino Mega #2
 * Hardware:
 * - 5x NEMA17 Steppers (A4988 Drivers)
 * - 1x Servo (Gripper)
 * - 5x Limit Switches (Calibration)
 */

#include <AccelStepper.h>
#include <Servo.h>

// --- Configuration ---
// Stepper Defines (Driver Type 1 = Step/Dir)
const int STEPS_PER_REV = 200; // Standard NEMA17

// Axis 1 (Base Rotation)
AccelStepper axis1(1, 22, 23); // Step=22, Dir=23
const int LIM_1 = 30;

// Axis 2 (Shoulder)
AccelStepper axis2(1, 24, 25);
const int LIM_2 = 31;

// Axis 3 (Elbow)
AccelStepper axis3(1, 26, 27);
const int LIM_3 = 32;

// Axis 4 (Wrist Pitch)
AccelStepper axis4(1, 28, 29);
const int LIM_4 = 33;

// Axis 5 (Wrist Roll)
AccelStepper axis5(1, 36, 37);
const int LIM_5 = 34;

// Gripper
Servo gripper;
const int PIN_GRIPPER = 9;

// Global System State
bool isCalibrated = false;

void setup() {
  Serial.begin(115200);

  // Setup Switches
  pinMode(LIM_1, INPUT_PULLUP);
  pinMode(LIM_2, INPUT_PULLUP);
  pinMode(LIM_3, INPUT_PULLUP);
  pinMode(LIM_4, INPUT_PULLUP);
  pinMode(LIM_5, INPUT_PULLUP);

  // Setup Gripper
  gripper.attach(PIN_GRIPPER);
  gripper.write(90); // Open state

  // Configure Steppers (Adjust speeds/accel for your mechanics)
  configureAxis(&axis1, 1000, 500);
  configureAxis(&axis2, 800, 400); // Heavy axes slower
  configureAxis(&axis3, 800, 400);
  configureAxis(&axis4, 1500, 800);
  configureAxis(&axis5, 1500, 800);

  Serial.println("RDY:Waiting for command or 'C' to calibrate");
}

void loop() {
  // Run steppers to target
  axis1.run();
  axis2.run();
  axis3.run();
  axis4.run();
  axis5.run();

  // Serial Command Handling
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // CALIBRATION COMMAND
    if (cmd == 'C') { 
      calibrateArm(); 
    }
    
    // MOVE COMMAND: Format "M,100,200,50,-100,0,90" (steps1...5, gripper)
    else if (cmd == 'M') {
      long t1 = Serial.parseInt();
      long t2 = Serial.parseInt();
      long t3 = Serial.parseInt();
      long t4 = Serial.parseInt();
      long t5 = Serial.parseInt();
      int grip = Serial.parseInt();

      if (isCalibrated) {
        axis1.moveTo(t1);
        axis2.moveTo(t2);
        axis3.moveTo(t3);
        axis4.moveTo(t4);
        axis5.moveTo(t5);
        gripper.write(grip);
        Serial.println("ACK:Moving");
      } else {
        Serial.println("ERR:NotCalibrated");
      }
    }
  }
}

void configureAxis(AccelStepper* axis, float maxSpeed, float accel) {
  axis->setMaxSpeed(maxSpeed);
  axis->setAcceleration(accel);
}

void calibrateAxis(AccelStepper* axis, int limitPin) {
  // Move "backwards" until switch hit
  axis->setSpeed(-300); 
  while (digitalRead(limitPin) == HIGH) { // Assuming LOW when pressed
    axis->runSpeed();
  }
  axis->stop();
  axis->setCurrentPosition(0); // Set Home
  // Move away slightly
  axis->runToNewPosition(50);
}

void calibrateArm() {
  Serial.println("STAT:Calibrating...");
  // Calibrate sequentially to avoid collisions
  // Order matters depending on your mechanical stowage!
  calibrateAxis(&axis5, LIM_5);
  calibrateAxis(&axis4, LIM_4);
  calibrateAxis(&axis3, LIM_3);
  calibrateAxis(&axis2, LIM_2);
  calibrateAxis(&axis1, LIM_1);
  
  isCalibrated = true;
  Serial.println("STAT:Calibrated");
}
