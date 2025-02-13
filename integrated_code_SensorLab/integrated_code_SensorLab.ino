#include <DHT.h>
#include <DHT_U.h>

// DHT Sensor Configuration
#define DHTPIN 2       // Data pin connected to D2
#define DHTTYPE DHT11  // DHT11 sensor type
DHT_Unified dht(DHTPIN, DHTTYPE); // Object for the DHT sensor

// Potentiometer Class
class Potentiometer {
public:
  int pin;          // Analog pin connected to the potentiometer
  int minValue;     // Minimum value read during calibration
  int maxValue;     // Maximum value read during calibration
  float vin;        // Input voltage (e.g., 5V)
  float rotationRange; // Range of rotation in degrees
  int currentValue; // Current analog value
  float voltage;    // Current voltage
  float angle;      // Current angle in degrees

  // Constructor
  Potentiometer(int potPin, float inputVoltage, float range) {
    pin = potPin;
    vin = inputVoltage;
    rotationRange = range;
    minValue = 0; // Default minimum ADC value
    maxValue = 1023; // Default maximum ADC value (10-bit ADC)
    currentValue = 0;
    voltage = 0.0;
    angle = 0.0;
  }

  // Method to read and update potentiometer values
  void update() {
    currentValue = analogRead(pin);

    // Calculate voltage
    voltage = currentValue * (vin / maxValue);

    // Calculate angle
    angle = map(currentValue, minValue, maxValue, 0, rotationRange);

    // Constrain angle within bounds
    angle = constrain(angle, 0, rotationRange);
  }

  // Method to print the current potentiometer values
  void printValues() {
    Serial.print("Potentiometer Value: ");
    Serial.print(currentValue);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 2);
    Serial.print(" V");
    Serial.print(" | Angle: ");
    Serial.print(angle, 2);
    Serial.println(" degrees");
  }
};

// Flex Sensor Configuration
int flexValue;

// Initialize Potentiometer and Flex Sensor
Potentiometer Pot_1(A0, 5.0, 270.0); // Pin A0, 5V input, 270° range
#define FLEXPIN A1 // Flex sensor connected to A1

// Function to Display DHT Sensor Properties
void properties() {
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.print("DHT Sensor Type: ");
  Serial.println(sensor.name);
  Serial.print("Min Temp: ");
  Serial.print(sensor.min_value);
  Serial.println(" °C");
  Serial.print("Max Temp: ");
  Serial.print(sensor.max_value);
  Serial.println(" °C");
  Serial.print("Resolution: ");
  Serial.print(sensor.resolution);
  Serial.println(" °C");
  Serial.println();

  dht.humidity().getSensor(&sensor);
  Serial.print("DHT Sensor Type: ");
  Serial.println(sensor.name);
  Serial.print("Min Humidity: ");
  Serial.print(sensor.min_value);
  Serial.println(" %");
  Serial.print("Max Humidity: ");
  Serial.print(sensor.max_value);
  Serial.println(" %");
  Serial.print("Resolution: ");
  Serial.print(sensor.resolution);
  Serial.println(" %");
  Serial.println();
}

void setup() {
  Serial.begin(9600); // Start serial communication

  // Initialize Sensors
  dht.begin();
  Serial.println("DHT11 Temperature and Humidity Sensor");
  properties();
}

void loop() {
  // Potentiometer Readings
  Pot_1.update();

  // Flex Sensor Readings
  flexValue = analogRead(FLEXPIN);

  // DHT Sensor Readings
  sensors_event_t event;
  float t, h;

  dht.temperature().getEvent(&event);
  t = event.temperature; // Temperature reading

  dht.humidity().getEvent(&event);
  h = event.relative_humidity; // Humidity reading

  // Unified printing of  all sensor readings
  Serial.println("----- Sensor Readings -----");
  Pot_1.printValues();
  Serial.print("Flex Sensor Value: ");
  Serial.println(flexValue);
  if (isnan(t)) {
    Serial.println("Temperature: Error reading temperature!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" °C");
  }
  if (isnan(h)) {
    Serial.println("Humidity: Error reading humidity!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.println(" %");
  }
  Serial.println("---------------------------");

  // Unified Delay
  delay(1000); // Using the longest delay here (because of DHT sensor's requirement)
}