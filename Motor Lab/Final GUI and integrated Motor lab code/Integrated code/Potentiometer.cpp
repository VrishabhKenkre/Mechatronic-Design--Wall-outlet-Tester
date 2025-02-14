#include "Potentiometer.h"

Potentiometer::Potentiometer(int potPin, float inputVoltage, float range) {
  pin = potPin;
  vin = inputVoltage;
  rotationRange = range;
  minValue = 0; // Default minimum ADC value
  maxValue = 1023; // Default maximum ADC value (10-bit ADC)
  currentValue = 0;
  estimated_v_in = 0.0;
  pot_input_angle = 0.0;
}

void Potentiometer::update() {
  currentValue = analogRead(pin);

  // Calculate voltage
  estimated_v_in = currentValue * (vin / maxValue);

  // Calculate angle
  pot_input_angle = map(currentValue, minValue, maxValue, 0, rotationRange);

  // Constrain pot_input_angle within bounds
  pot_input_angle = constrain(pot_input_angle, 0, rotationRange);
}

void Potentiometer::printValues() {
  // Serial.print("Potentiometer Value: ");
  Serial.print(currentValue);
  // Serial.print(" | estimated_v_in: ");
  // Serial.print(estimated_v_in, 2);
  // Serial.print(" V");
  // Serial.print(" | pot_input_angle: ");
  // Serial.print(pot_input_angle, 2);
  // Serial.println(" degrees");
}
