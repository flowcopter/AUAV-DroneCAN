#ifndef ALLSENSORS_AUAV_H
#define ALLSENSORS_AUAV_H

#include <stdint.h>
#include <Wire.h>

class AllSensors_AUAV {
public:

  static const uint8_t I2C_ADDRESS_DIFF = 0x26;             // 38 in decimal
  static const uint8_t I2C_ADDRESS_ABS = 0x27;              // 39 in decimal
  
  enum SensorType {
    DIFFERENTIAL,
    ABSOLUTE,
  };

  enum MeasurementType {
    SINGLE    = 0xAA,
    AVERAGE2  = 0xAC,
    AVERAGE4  = 0xAD,
    AVERAGE8  = 0xAE,
    AVERAGE16 = 0xAF,
  };

  enum StatusFlags {
    RESERVED_7    = 1<<7,
    POWER_ON      = 1<<6,
    BUSY          = 1<<5,
    MODE          = 1<<4 | 1<<3,
    ERROR_MEMORY  = 1<<2,
    CONFIGURATION = 1<<1,
    ERROR_ALU     = 1<<0,
  };

  enum PressureUnit {
    IN_H2O = 'H',
    PASCAL = 'P',
  };

  enum TemperatureUnit {
    CELCIUS    = 'C',
    FAHRENHEIT = 'F',
    KELVIN     = 'K',
  };

  enum SensorPressureRange {
    L05D = 5,
    L10D = 10,
    L30D = 30,
  };

private:

  // The length of the status information in the I2C response.
  static const uint8_t READ_STATUS_LENGTH = 1;

  // The length of the pressure data in the I2C response.
  static const uint8_t READ_PRESSURE_LENGTH = 3;

  // The length of the temperature data in the I2C response.
  static const uint8_t READ_TEMPERATURE_LENGTH = 3;

  // The value of a full scale temperature or pressure, 2^24.
  static constexpr uint32_t FULL_SCALE_REF = (uint32_t) 1 << 24;

  // The value of a zero reference pressure, 2^23.
  static constexpr uint32_t PRESSURE_ZERO_REF = (uint32_t) 1 << 23;

  // The value of a zero reference pressure, 0.1 * 2^24.
  static constexpr float ABS_PRESSURE_ZERO_REF = FULL_SCALE_REF / 10.0f;

  TwoWire *bus;
  float pressure_range;
  PressureUnit pressure_diff_unit;
  PressureUnit pressure_abs_unit;
  TemperatureUnit temperature_unit;

  // Convert a raw digital differential pressure read from the sensor to a floating point value in inH2O.
  float transferDifferentialPressure(unsigned long raw_value) {
    // Based on the following formula in the datasheet:
    //     Pressure(inH2O) = 1.25 x ((P_out_dig - OS_dig) / 2^24) x FSS(inH2O)
    return 1.25f * (((float)raw_value - PRESSURE_ZERO_REF) / FULL_SCALE_REF) * pressure_range;
  }

    // Convert a raw digital absolute pressure read from the sensor to a floating point value in mbar.
  float transferAbsolutePressure(unsigned long raw_value) {
      // Based on the following formula in the datasheet:
      //     Pressure(inH2O) = 1.25 x ((P_out_dig - OS_dig) / 2^24) x FSS(inH2O)
      return 250 + 1250 * (((float) raw_value - ABS_PRESSURE_ZERO_REF) / FULL_SCALE_REF);
    }

  // Convert a raw digital temperature read from the sensor to a floating point value in Celcius.
  float transferTemperature(unsigned long raw_value) {
    // Based on the following formula in the datasheet:
    //     Temperature(degC) = ((T_out_dig * 155) / 2^24) - 45
    return (((float)raw_value * 155.0f) / FULL_SCALE_REF) - 45.0f;
  }

  // Convert the input in inH2O to the configured pressure output unit.
  float convertDiffPressure(float in_h2o) {
    switch(pressure_diff_unit) {
      case PASCAL:
        return 249.08f * in_h2o;
      case IN_H2O:
      default:
        return in_h2o;
    }
  }

  // Convert the absolute pressure in mbar to the configured pressure output unit.
  float convertAbsPressure(float mbar) {
    switch(pressure_abs_unit) {
      case PASCAL:
        return mbar * 100.0f; // Convert mbar to Pascal
      case IN_H2O:
      default:
        return mbar / 249.08f; // Convert mbar to inH2O
    }
  }

  // Convert the input in Celcius to the configured temperature output unit.
  float convertTemperature(float degree_c) {
    switch(temperature_unit) {
      case FAHRENHEIT:
        return degree_c * 1.8f + 32.0f;
      case KELVIN:
        return degree_c + 273.15f;
      case CELCIUS:
      default:
        return degree_c;
    }    
  }

public:

  uint8_t status;
  
  float pressure_d;
  float temperature_d;

  float pressure_a;
  float temperature_a;

  uint32_t raw_p = 0;
  uint32_t raw_t = 0;

  unsigned long last_message_abs = 0;
  unsigned long last_message_diff = 0;

  // Return true if the status byte input indicates that the sensor is currently busy.
  static bool isBusy(uint8_t status_arg) {
    return (status_arg & StatusFlags::BUSY) != 0;
  }

  // Return true if the status byte input indicates that the sensor experienced an error.
  static bool isError(uint8_t status_arg) {
    return (status_arg & (StatusFlags::ERROR_MEMORY | StatusFlags::ERROR_ALU)) != 0;
  }

  AllSensors_AUAV(TwoWire *bus);

  // Set the configured pressure unit for data output (the default is inH2O).
  void setPressureDiffUnit(PressureUnit pressure_diff_unit) {
    this->pressure_diff_unit = pressure_diff_unit;
  }

  // Set the configured pressure unit for data output (the default is inH2O).
  void setPressureAbsUnit(PressureUnit pressure_abs_unit) {
    this->pressure_abs_unit = pressure_abs_unit;
  } 

  // Set the configured temperature unit for data output (the default is Celcius).
  void setTemperatureUnit(TemperatureUnit temperature_unit) {
    this->temperature_unit = temperature_unit;
  }

  // Set the configured pressure range for the sensor (the default is 10 inH2O).
  void setPressureRange(float _pressure_range) {
    this->pressure_range = _pressure_range;
  } 

  // Request that the sensor start a measurement. Following this command, the sensor will begin a
  // measurement which takes between 2.8ms (Single 16-bit) and 61.9ms (Average16 18-bit) to complete.
  // In the interim, isBusy() or (readStatus() directly) can be polled until the sensor no longer
  // reports itself busy, at which time the data registers can be read to find the result.

  // If no argument is provided, a single measurement is taken. Otherwise, a MeasurementType can be
  // provided to allow multiple averaged measurements to be taken.
  void startMeasurement(SensorType type, MeasurementType measurement_type = MeasurementType::SINGLE);

  // Read the current status register from the sensor.
  uint8_t readStatus(SensorType type);

  // A simple wrapper around reading the current status register and returning true if it indicates
  // that the sensor is busy.
  bool isBusy(SensorType type) {
    return isBusy(readStatus(type));
  }

  // Read the data from the sensor. This function will read the data from the sensor and store it in
  // the pressure and temperature member. Returns True if a reading was successfully taken.
  // Return Value   0: Success, 1: AUAV Error, 2: Sensor is busy, 3: Invalid sensor type
  uint8_t readData(SensorType type);
};

#endif // ALLSENSORS_AUAV_H