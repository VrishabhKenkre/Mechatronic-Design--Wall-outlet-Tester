#include "DCMotor.h"

DCMotor::DCMotor(int potPin, float inputVoltage, float range, float startV, float maxRPM)
  : Potentiometer(potPin, inputVoltage, range), start_voltage(startV), max_rpm(maxRPM) {
  estimated_rpm = 0.0;
}

void DCMotor::update() {
  // Call the base class update method to get the potentiometer values
  Potentiometer::update();

  // Estimate the motor speed using the mapping function
  estimated_rpm = mapSpeed(estimated_v_in, start_voltage, vin, 0.0, max_rpm);
}

float DCMotor::mapSpeed(float voltage, float v_min, float v_max, float rpm_min, float rpm_max) {
  // Ensure voltage is within expected range
  if (voltage <= v_min) return rpm_min;
  if (voltage >= v_max) return rpm_max;

  // Linear mapping equation
  return rpm_min + (voltage - v_min) * (rpm_max - rpm_min) / (v_max - v_min);
}

void DCMotor::printValues() {
  // Serial.print("Potentiometer Value: ");
  Serial.print(currentValue);
  // Serial.print(" | Voltage: ");
  // Serial.print(estimated_v_in, 2);
  // Serial.print(" V");
  // Serial.print(" | Angle: ");
  // Serial.print(pot_input_angle, 2);
  // Serial.print(" degrees");
  // Serial.print(" | Estimated RPM: ");
  // Serial.print(estimated_rpm, 2);
  // Serial.println(" RPM");
}