#include "Calibrated16bitSensor.h"

// Better to provide a custom helper function to read the raw integer angle from your sensor, but this works too.
uint16_t Calibrated16bitSensor_DefaultReadHelper(Sensor *s) {
  return s->getMechanicalAngle()*(65536.0f/_2PI);
}

void Calibrated16bitSensor::init(int _lut_resolution, const uint16_t *_lut)
{
  lut_resolution = _lut_resolution;
  lut = _lut;
  this->Sensor::init();
  // Perform first reading to initialize spike filter variables
  lastRaw = beforeLastRaw = readSensor16bit(&wrapped);
  getSensorAngle();
}

// Perform full calibration. The table passed in here will be filled with data and its pointer retained.
// The table should have space for one more entry than _lut_resolution (simplifies interpolation code)
void Calibrated16bitSensor::init(FOCMotor *motor, int _lut_resolution, uint16_t *_lut)
{
  int i, j;
  uint16_t table[_lut_resolution*4];

  lut_resolution = _lut_resolution;
  lut = _lut;
  this->Sensor::init();

  Serial.println("Calibrated16bitSensor: Begin calibration movement");
  // Rotate motor forward and back, recording the sensor values at each step.

  // Move slightly negative, so first loop iteration moves forward like subsequent iterations
  motor->setPhaseVoltage(motor->voltage_sensor_align, 0, _electricalAngle(-1 * _2PI / lut_resolution, motor->pole_pairs));
  delay(2000/lut_resolution);
  for (j = 0; j < 2; j++)
    for (i = 0; i < lut_resolution; i++) {
      uint32_t time = 3000000 / lut_resolution, start_time = _micros();
      while(_micros() < start_time + time) {
        float a = (i - 1) + (float)(_micros() - start_time) / time;
        motor->setPhaseVoltage(motor->voltage_sensor_align, 0, _electricalAngle(a * _2PI / lut_resolution, motor->pole_pairs));
      }
      table[j*lut_resolution+i] = readSensor16bit(&wrapped);
    }
  // Complete previous revolution, so first loop iteration moves backward like subsequent iterations
  motor->setPhaseVoltage(motor->voltage_sensor_align, 0, 0);
  delay(2000/lut_resolution);
  for (j = 0; j < 2; j++)
    for (i = lut_resolution - 1; i >= 0; i--) {
      uint32_t time = 3000000 / lut_resolution, start_time = _micros();
      while(_micros() < start_time + time) {
        float a = (i + 1) - (float)(_micros() - start_time) / time;
        motor->setPhaseVoltage(motor->voltage_sensor_align, 0, _electricalAngle(a * _2PI / lut_resolution, motor->pole_pairs));
      }
      table[(2+j)*lut_resolution+i] = readSensor16bit(&wrapped);
    }

  // Average the samples by adding half the signed difference so it behaves correctly near wraparound
  for (i = 0; i < lut_resolution; i++) {
    uint16_t temp1 = table[lut_resolution*0+i] + (((int16_t)(table[lut_resolution*1+i] - table[lut_resolution*0+i])) >> 1);
    uint16_t temp2 = table[lut_resolution*2+i] + (((int16_t)(table[lut_resolution*3+i] - table[lut_resolution*2+i])) >> 1);
    table[i] = (uint16_t)(temp1 + (((int16_t)(temp2 - temp1)) >> 1));
  }
  table[lut_resolution] = table[0]; // Extra for interpolation

  //Serial.print("const uint16_t table[] = {"); delayMicroseconds(200);
  //printTable(table, lut_resolution);
  //Serial.print("\n};\n\n"); delayMicroseconds(200);

  for (i = 0; i < lut_resolution; i++) {
    uint16_t raw = (int16_t)(((int32_t)i << 16) / lut_resolution); // Equivalent sensor value for this table index
    for (j = 0; j < lut_resolution; j++) {
      int32_t a = table[j], b = table[j+1];
      // Figure out if raw angle is between a and b.
      // if a and b are on the same side of wraparound, the signs of the differences should be different.
      // If a and b are on opposite sides of wraparound, the signs of the differences should be equal.
      int32_t wrap = abs((int32_t)a-b)>0x7fff ? (1L<<31) : 0;
      if (((a-raw)^(b-raw)^wrap) & (1L<<31)) {
        int32_t cal = (int32_t)j << 16; // Integer portion of calibrated angle for this sensor value
        cal += (((int32_t)((int16_t)(raw - a))) << 16) / (int16_t)(b - a); // Fractional portion
        cal /= lut_resolution; // Range is now 0-65535 for one revolution
        _lut[i] = (uint16_t)cal;
      }
    }
  }

  // Set extra entry at end of table, used for interpolation
  _lut[lut_resolution] = _lut[0];

  // Perform first reading to initialize spike filter variables
  lastRaw = beforeLastRaw = readSensor16bit(&wrapped);
  getSensorAngle();

  Serial.println("Calibrated16bitSensor: Calibration complete");
  printCalibration();
}

void Calibrated16bitSensor::update() {
  if(readSensor16bit == Calibrated16bitSensor_DefaultReadHelper)
    wrapped.update(); // This reads the sensor angle, which is typically not necessary with a custom read helper
  Sensor::update();
}

float Calibrated16bitSensor::getSensorAngle() {
  uint16_t oldRaw = beforeLastRaw;
  beforeLastRaw = lastRaw;
  lastRaw = readSensor16bit(&wrapped);

  // If the new reading is significantly different from the last two, it's probably bad.
  if (abs((int16_t)(lastRaw - beforeLastRaw)) > 500 && abs((int16_t)(lastRaw - oldRaw)) > 500)
    return (float)lastCal * (_2PI/65536.0f);

  // Interpolated lookup
  int32_t temp = (int32_t)lastRaw * lut_resolution;
  const unsigned int idx = temp >> 16, frac = (uint16_t)temp;
  const uint16_t a = lut[idx], b = lut[idx + 1];
  const int32_t diff = (int16_t)(b - a);
  lastCal = (uint16_t)(((diff * frac) >> 16) + a);

  // Convert to radians
  return (float)lastCal * (_2PI/65536.0f);
}

// Print data to serial so it can be copy/pasted into the code to avoid re-calibrating every startup.
void Calibrated16bitSensor::printCalibration() const {
  Serial.print("const uint16_t lut["); delayMicroseconds(200);
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  for (int i = 0; i <= lut_resolution; i++) {
    if(!(i & 7)) { Serial.print("\n\t"); delayMicroseconds(200); }
    Serial.print(lut[i]); delayMicroseconds(200);
    if(i != lut_resolution) { Serial.print(", "); delayMicroseconds(200); }
  }
  Serial.print("\n};\n"); delayMicroseconds(200);
}
