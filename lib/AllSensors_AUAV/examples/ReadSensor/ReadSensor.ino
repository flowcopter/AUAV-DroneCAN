#include <Wire.h>

#include <AllSensors_AUAV.h>

AllSensors_AUAV pressureSensor(&Wire, AllSensors_AUAV::SensorPressureRange::L10D);

void setup() {
  Serial.begin(115200);

  Wire.begin();

  pressureSensor.setPressureUnit(AllSensors_AUAV::PressureDifferentialUnit::PASCAL);
}

void loop() {
    pressureSensor.startMeasurement();
    pressureSensor.readData(true, AllSensors_AUAV::SensorType::DIFFERENTIAL);
    Serial.print(",");
    Serial.print(pressureSensor.pressure_d);
    Serial.print(",");
    Serial.print(pressureSensor.temperature_d);

    pressureSensor.readData(true, AllSensors_AUAV::SensorType::ABSOLUTE);
    Serial.print(",");
    Serial.print(pressureSensor.pressure_a);
    Serial.print(",");
    Serial.println(pressureSensor.temperature_a);

    delay(100);
}