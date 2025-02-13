#ifndef DCMOTOR_H
#define DCMOTOR_H

#include "Potentiometer.h"

class DCMotor : public Potentiometer {
public:
  float estimated_rpm;  // Estimated speed of the motor in RPM
  float start_voltage;  // Minimum voltage required to start the motor
  float max_rpm;        // Maximum speed of the motor at max voltage

  // Constructor
  DCMotor(int potPin, float inputVoltage, float range, float startV, float maxRPM);

  // Method to update motor values
  void update();

  // Function to map voltage to speed
  float mapSpeed(float voltage, float v_min, float v_max, float rpm_min, float rpm_max);

  // Method to print motor values
  void printValues();
};

#endif