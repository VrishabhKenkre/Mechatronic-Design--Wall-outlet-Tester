#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <Arduino.h>

class Potentiometer {
public:
  int pin;          // Analog pin connected to the potentiometer
  int minValue;     // Minimum value read during calibration
  int maxValue;     // Maximum value read during calibration
  float vin;        // Input voltage (e.g., 5V)
  float rotationRange; // Range of rotation in degrees
  int currentValue; // Current analog value
  float estimated_v_in;    // Current voltage
  float pot_input_angle;      // Current angle in degrees

  // Constructor
  Potentiometer(int potPin, float inputVoltage, float range);

  // Method to read and update potentiometer values
  void update();

  // Method to print the current potentiometer values
  void printValues();
};

#endif