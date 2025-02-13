#include <DHT.h>
#include <DHT_U.h>


#define DHTPIN 2       // Data pin connected to D2
#define DHTTYPE DHT11  // DHT11 sensor type

DHT_Unified dht(DHTPIN, DHTTYPE);

void properties(){
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.print("Sensor Type: ");Serial.println(sensor.name);
  Serial.print("Min Value: "); Serial.print(sensor.min_value); Serial.println(" °C");
  Serial.print("Max Value: "); Serial.print(sensor.max_value); Serial.println(" °C");
  Serial.print("Resolution: "); Serial.print(sensor.resolution); Serial.println(" °C");
  Serial.println();

  dht.humidity().getSensor(&sensor);
  Serial.print("Sensor Type: ");Serial.println(sensor.name);
  Serial.print("Min Value: "); Serial.print(sensor.min_value); Serial.println(" %");
  Serial.print("Max Value: "); Serial.print(sensor.max_value); Serial.println(" %");
  Serial.print("Resolution: "); Serial.print(sensor.resolution); Serial.println(" %");
  Serial.println();
}

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  dht.begin();         
  Serial.println("DHT11 Temperature and Humidity Sensor");
  properties();
  
}

void loop() {
  delay(2000); 

  sensors_event_t event;
  float t;
  float h;
  dht.temperature().getEvent(&event);
  t = event.temperature;
  

  if (isnan(t)) {
    Serial.println("Error reading temperature!");
  } else {
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" °C");
  }

  //Comment out the above and uncomment the below for humidity readings

  // dht.humidity().getEvent(&event);
  // h = event.relative_humidity;

  // if(isnan(h)){
  //   Serial.println("Error reading humidity!");
  // }else{
  // Serial.print("Humidity: ");
  // Serial.print(h);
  // Serial.println(" %\t");
  // }
}
