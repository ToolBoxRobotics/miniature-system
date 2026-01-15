/*
 * ROVER BASE CONTROLLER - Arduino Mega #1
 * Hardware:
 * - 6x DC Motors (via 3x DRI0002)
 * - 4x Servos (via PCA9685 Channel 0-3)
 * - INA226 (0x40), MPU6050 (via TCA9548A if needed, or direct)
 * - 2x Encoders (Middle Wheels)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA226.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Configuration ---
// DRI0002 Motor Driver Pins (Example mapping)
// Right Side (Front, Mid, Rear)
const int PIN_M_R_DIR[3] = {22, 24, 26}; 
const int PIN_M_R_PWM[3] = {2, 3, 4};    
// Left Side (Front, Mid, Rear)
const int PIN_M_L_DIR[3] = {23, 25, 27}; 
const int PIN_M_L_PWM[3] = {5, 6, 7};    

// Encoders (Middle Wheels Only)
const int ENC_L_A = 18; // Interrupt pin
const int ENC_L_B = 30;
const int ENC_R_A = 19; // Interrupt pin
const int ENC_R_B = 31;

// Objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Default I2C 0x40
Adafruit_INA226 ina226 = Adafruit_INA226();
Adafruit_MPU6050 mpu;

// Variables
volatile long enc_pos_l = 0;
volatile long enc_pos_r = 0;
unsigned long last_send = 0;

void setup() {
  Serial.begin(115200); // High speed for ROS
  Wire.begin();

  // Setup Motors
  for(int i=0; i<3; i++) {
    pinMode(PIN_M_R_DIR[i], OUTPUT); pinMode(PIN_M_R_PWM[i], OUTPUT);
    pinMode(PIN_M_L_DIR[i], OUTPUT); pinMode(PIN_M_L_PWM[i], OUTPUT);
  }

  // Setup Encoders
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncR, RISING);

  // Init Sensors
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); // 50Hz for Servos

  if (!ina226.begin()) Serial.println("ERR:INA226");
  
  // MPU6050 Init (Assuming TCA9548A is handled or MPU is on main bus)
  // If using TCA, send byte to 0x70 to select channel before init
  if (!mpu.begin()) Serial.println("ERR:MPU6050");
}

void loop() {
  // 1. Listen for Serial Commands from ROS 2
  // Format: <S1,S2,S3,S4,ML,MR> (Servo Angles 0-180, Motor Speeds -255 to 255)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseCommand(input);
  }

  // 2. Publish Sensor Data @ 10Hz
  if (millis() - last_send > 100) {
    sendTelemetry();
    last_send = millis();
  }
}

void parseCommand(String input) {
  // Basic parsing logic (implement robust strtok in production)
  // Expected: "s,90,90,90,90,100,100" -> s, FL_ang, FR_ang, RL_ang, RR_ang, L_speed, R_speed
  if (input.startsWith("s")) {
    int values[6]; 
    // ... Extract integers from CSV string ...
    // (Pseudocode for brevity)
    
    // Apply to Hardware
    // Steering Servos (Channels 0-3 on PCA9685)
    // map 0-180 degrees to pulse length (approx 150-600)
    // setServo(0, values[0]); ...
    
    // Drive Motors
    // setMotorGroup(0, values[4]); // Left
    // setMotorGroup(1, values[5]); // Right
  }
}

void setMotorGroup(int side, int speed) {
  int pwm_val = abs(speed);
  int dir = (speed > 0) ? HIGH : LOW;
  
  if (side == 0) { // Left
    for(int i=0; i<3; i++) {
       digitalWrite(PIN_M_L_DIR[i], dir);
       analogWrite(PIN_M_L_PWM[i], pwm_val);
    }
  } else { // Right
    for(int i=0; i<3; i++) {
       digitalWrite(PIN_M_R_DIR[i], dir);
       analogWrite(PIN_M_R_PWM[i], pwm_val);
    }
  }
}

void sendTelemetry() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float busVoltage = ina226.readBusVoltage();
  float current_mA = ina226.readCurrent();

  // JSON format for easy parsing in Python ROS node
  Serial.print("{\"EL\":"); Serial.print(enc_pos_l);
  Serial.print(",\"ER\":"); Serial.print(enc_pos_r);
  Serial.print(",\"V\":"); Serial.print(busVoltage);
  Serial.print(",\"A\":"); Serial.print(current_mA);
  Serial.print(",\"GZ\":"); Serial.println(g.gyro.z); // Yaw rate
  Serial.print("}");
  Serial.println();
}

void readEncL() {
  if (digitalRead(ENC_L_B) == HIGH) enc_pos_l++; else enc_pos_l--;
}
void readEncR() {
  if (digitalRead(ENC_R_B) == HIGH) enc_pos_r++; else enc_pos_r--;
}
