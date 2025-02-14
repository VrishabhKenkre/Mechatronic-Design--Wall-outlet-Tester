#include "Potentiometer.h"   // Your provided potentiometer code
#include <Servo.h>
#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ----- Pin Definitions & Constants -----
#define IN1 7
#define IN2 6
#define IN3 5
#define IN4 4
#define STEPS_PER_REV 2048

// ----- IMUSensor Class -----
class IMUSensor {
  private:
    Adafruit_MPU6050 mpu;
    float last_yaw;
    
  public:
    IMUSensor() : last_yaw(0.0) {}

    void start() {
      Serial.begin(9600);
      Wire.begin();
      
      Serial.println("Initializing MPU6050...");
      if (!mpu.begin()) {
        Serial.println("MPU6050 Connection Failed! Check wiring.");
        while (1);
      }
      Serial.println("MPU6050 Initialized Successfully!");
      
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      delay(2000); // Allow IMU to stabilize
    }

    float get_yaw_change() {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      
      // Calculate new yaw based on gyro z (converted to degrees/sec)
      float current_yaw = last_yaw + (g.gyro.z * 57.2958) * 0.01;
      float yaw_change = current_yaw - last_yaw;
      last_yaw = current_yaw;
      return yaw_change;
    }
    
    float getCurrentYaw() {
      return last_yaw;
    }
};

// ----- StepperMotor Class -----
class StepperMotor {
  private:
    Stepper stepperMotor;
    IMUSensor &imu;  // Reference to IMU object
    
  public:
    StepperMotor(IMUSensor &imuRef)
      : stepperMotor(STEPS_PER_REV, IN1, IN3, IN2, IN4), imu(imuRef) {
        stepperMotor.setSpeed(10);  // Set motor speed (RPM)
    }
    
    // Normal sensor-based update:
    void update() {
      float yaw_change = imu.get_yaw_change();
      int steps = (int)(yaw_change / (360.0 / STEPS_PER_REV));
      if (abs(steps) > 1) {
        stepperMotor.step(steps);
      }
    }
    
    // Control update: override with GUI-supplied IMU value (in degrees)
    void controlUpdate(float guiIMUVal) {
      int steps = (int)(guiIMUVal / (360.0 / STEPS_PER_REV));
      stepperMotor.step(steps);
    }
};

// ----- Dummy DCMotor Class -----
// (Replace with your actual DCMotor library code if available.)
class DCMotor {
  private:
    int motorPin;
    
  public:
    DCMotor(int pin, float dummy1, float dummy2, float dummy3, float dummy4) {
      motorPin = pin;
      pinMode(motorPin, OUTPUT);
    }
    
    // In sensor mode, update may use the potentiometer reading.
    void update() {
      // For example, do nothing or run a default routine.
    }
    
    // In control mode, update the motor output using the GUI pot value.
    void controlUpdate(int controlVal) {
      // Map the 0-1023 GUI value to a PWM output (0-255)
      int pwmVal = map(controlVal, 0, 1023, 0, 255);
      analogWrite(motorPin, pwmVal);
    }
};

// ----- Servo (using the Arduino Servo library) -----
Servo Servo1;

// ----- ServoMotor Structure (for hardware encoder control) -----
struct ServoMotor {
  int angle;   // Servo angle (0 to 180)
  int count;   // Rotary encoder count
  bool bool_CW; // true: clockwise, false: counter-clockwise
};

// ----- Global Objects & Variables -----
IMUSensor imu;
StepperMotor stepper(imu);
Potentiometer pot(A1, 5.0, 100.0);  // Uses your provided Potentiometer code
DCMotor dc_motor(A0, 5.0, 270.0, 3.0, 3000.0);
ServoMotor servo_motor;

int servoPin = 3;  // Servo control pin

// Encoder pins (for hardware mode)
int encoderPinA = 8;
int encoderPinB = 9;
int encoderBtn = 10;
int encoderPinA_prev;
int encoderPinA_value;
int servo_active = 1;
float servo_prev_time = 0;
float servo_debounce_time = 50;
// Variables for GUI control (set via serial commands)
bool guiControlEnabled = false;
int guiPotValue = 0;      // For DC Motor control (0-1023)
int guiEncoderValue = 0;  // For Servo control (0-180)
int guiIMUValue = 0;      // For Stepper control (-90 to +90)

void setup() {
  Serial.begin(9600);
  imu.start();
  
  // Initialize encoder pins:
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderBtn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderBtn), toggle_servo, RISING);
  encoderPinA_prev = digitalRead(encoderPinA);
  
  
  // Attach the servo:
  Servo1.attach(servoPin);
}

void toggle_servo() {
  if (millis() - servo_prev_time > servo_debounce_time) {
    if ( servo_active == 0 ) {
      servo_active = 1;
    } else {
      servo_active = 0;
    }
  }
}

void loop() {
  // --- Process Serial Commands from the GUI ---
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Command to disable GUI control
    if (command.startsWith("CONTROL:OFF")) {
      guiControlEnabled = false;
    }
    // Command to enable GUI control with values:
    // Expected format: "P:<pot_val>,E:<encoder_val>,I:<imu_val>"
    else if (command.startsWith("P:")) {
      guiControlEnabled = true;
      int comma1 = command.indexOf(',');
      int comma2 = command.lastIndexOf(',');
      
      String pStr = command.substring(2, comma1); // after "P:" up to first comma
      String eStr = command.substring(command.indexOf("E:") + 2, comma2);
      String iStr = command.substring(command.indexOf("I:") + 2);
      
      guiPotValue = pStr.toInt();
      guiEncoderValue = eStr.toInt();
      guiIMUValue = iStr.toInt();
    }
  }
  
  // --- Motor Control Logic ---
  if (guiControlEnabled) {
    // GUI control mode: override sensor-based behavior.
    
    // Use GUI IMU value to control the stepper motor.
    stepper.controlUpdate(guiIMUValue);
    
    // Use GUI encoder value to control the servo.
    Servo1.write(guiEncoderValue);
    
    // Use GUI potentiometer value to control the DC motor.
    dc_motor.controlUpdate(guiPotValue);
  }
  else {
    // Sensor (hardware) control mode.
    
    // Update potentiometer reading.
    pot.update();
    
    // Update stepper motor using sensor (IMU) data.
    stepper.update();
    
    // Process hardware encoder for servo control.
    encoderPinA_value = digitalRead(encoderPinA);
    if (encoderPinA_value != encoderPinA_prev) {
      if (digitalRead(encoderPinB) != encoderPinA_value) {
        servo_motor.count++;
      }
      else {
        servo_motor.count--;
      }
      if (servo_motor.count >= 0 && servo_motor.count <= 20) {
        servo_motor.angle = servo_motor.count * 9;
        Servo1.write(servo_motor.angle);
      }
    }
    encoderPinA_prev = encoderPinA_value;
    
    // DC Motor update based on sensor reading (using potentiometer value)
    dc_motor.update();
  }
  
  // --- Debug Output ---
  Serial.print("current_yaw: ");
  Serial.print(imu.getCurrentYaw());
  Serial.print(", currentValue: ");
  if (!guiControlEnabled) {
    // Here we print the raw potentiometer value.
    // If currentValue isn’t public, add a getter in your Potentiometer class.
    Serial.print(pot.currentValue);
  }
  else {
    Serial.print(guiPotValue);
  }
  Serial.print(", count: ");
  if (!guiControlEnabled) {
    Serial.println(servo_motor.angle);
  }
  else {
    Serial.println(guiEncoderValue);
  }
  
  delay(10);
}