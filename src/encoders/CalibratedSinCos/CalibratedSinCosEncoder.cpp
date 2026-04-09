#include "CalibratedSinCosEncoder.h"

// This function can be overridden with custom ADC code on platforms with poor analogRead performance.
__attribute__((weak)) void ReadSinCosEncoder(int pinA, int pinB, uint16_t *a, uint16_t *b)
{
  *a = analogRead(pinA);
  *b = analogRead(pinB);
}

void CalibratedSinCosEncoder::init(uint16_t _minA, uint16_t _maxA, uint16_t _minB, uint16_t _maxB,
  uint16_t _centerA, uint16_t _centerB, uint16_t _thirtyA, uint16_t _sixtyA, int _lut_resolution,
  const uint16_t *_lutAP, const uint16_t *_lutAN, const uint16_t *_lutBP, const uint16_t *_lutBN)
{
  minA = _minA; maxA = _maxA; minB = _minB; maxB = _maxB;
  centerA = _centerA; centerB = _centerB; thirtyA = _thirtyA; sixtyA = _sixtyA;
  lut_resolution = _lut_resolution;
  lutAP = _lutAP; lutAN = _lutAN; lutBP = _lutBP; lutBN = _lutBN;
}

// Perform full calibration. The tables passed in here will be filled with data and their pointers retained.
// Each table should have space for one more entry than _lut_resolution (simplifies interpolation code)
void CalibratedSinCosEncoder::init(FOCMotor *motor, int calibration_steps, int _lut_resolution,
  uint16_t *_lutAP, uint16_t *_lutAN, uint16_t *_lutBP, uint16_t *_lutBN)
{
  int i, j;
  uint16_t a, b;
  int32_t tableA[calibration_steps+1]={0}, tableB[calibration_steps+1]={0}; // Extra entry at the end to simplify interpolation code

  lut_resolution = _lut_resolution; lutAP = _lutAP; lutAN = _lutAN; lutBP = _lutBP; lutBN = _lutBN;

  Serial.println("CalibratedSinCosEncoder: Begin calibration movement");
  // Rotate motor forward and back, recording the sensor values at each step.

  // Move slightly negative, so first loop iteration moves forward like subsequent iterations
  motor->setPhaseVoltage(motor->voltage_sensor_align, 0, _electricalAngle(-1 * _2PI / calibration_steps, motor->pole_pairs));
  delay(2000 / calibration_steps);
  for (j = 0; j < 4; j++)
    for (i = 0; i < calibration_steps; i++) {
      uint32_t time = 2000000 / calibration_steps, start_time = _micros();
      while(_micros() < start_time + time) {
        float a = (i - 1) + (float)(_micros() - start_time) / time;
        motor->setPhaseVoltage(motor->voltage_sensor_align, 0, _electricalAngle(a * _2PI / calibration_steps, motor->pole_pairs));
      }
      ReadSinCosEncoder(pinA, pinB, &a, &b);
      tableA[i] += a;
      tableB[i] += b;
    }
  // Complete previous revolution, so first loop iteration moves backward like subsequent iterations
  motor->setPhaseVoltage(motor->voltage_sensor_align, 0, 0);
  delay(2000 / calibration_steps);
  for (j = 0; j < 4; j++)
    for (i = calibration_steps - 1; i >= 0; i--) {
      uint32_t time = 2000000 / calibration_steps, start_time = _micros();
      while(_micros() < start_time + time) {
        float a = (i + 1) - (float)(_micros() - start_time) / time;
        motor->setPhaseVoltage(motor->voltage_sensor_align, 0, _electricalAngle(a * _2PI / calibration_steps, motor->pole_pairs));
      }
      ReadSinCosEncoder(pinA, pinB, &a, &b);
      tableA[i] += a;
      tableB[i] += b;
    }
    
  for (i = 0; i < calibration_steps; i++)
    tableA[i] >>= 3, tableB[i] >>= 3;
  // Set extra entry at end of tables, used for interpolation
  tableA[calibration_steps] = tableA[0], tableB[calibration_steps] = tableB[0];

  // Find min/max values, and their locations in the table (=the associated shaft angle)
  int minA_idx, maxA_idx, minB_idx, maxB_idx;
  minA = 65535, maxA = 0, minB = 65535, maxB = 0;
  for (i = 0; i < calibration_steps; i++) {
    if(tableA[i] <= minA) minA = tableA[i], minA_idx = i;
    if(tableA[i] >= maxA) maxA = tableA[i], maxA_idx = i;
    if(tableB[i] <= minB) minB = tableB[i], minB_idx = i;
    if(tableB[i] >= maxB) maxB = tableB[i], maxB_idx = i;
  }

  // Center value is 90 degrees from min and max, not simply the average of min and max since 
  // axial magnet offset will cause one to be farther from center than the other.
  // Use average going both directions from min and max to improve accuracy even further.
  centerA = (uint16_t)(((uint32_t)
    tableA[(minA_idx + calibration_steps/4) % calibration_steps] +
    tableA[(maxA_idx + calibration_steps/4) % calibration_steps] +
    tableA[(minA_idx + calibration_steps*3/4) % calibration_steps] +
    tableA[(maxA_idx + calibration_steps*3/4) % calibration_steps]) / 4);
  centerB = (uint16_t)(((uint32_t)
    tableB[(minB_idx + calibration_steps/4) % calibration_steps] +
    tableB[(maxB_idx + calibration_steps/4) % calibration_steps] +
    tableB[(minB_idx + calibration_steps*3/4) % calibration_steps] +
    tableB[(maxB_idx + calibration_steps*3/4) % calibration_steps]) / 4);

  // The revolution is divided up into 8 sectors. In the four sectors where one sensor is near its sine wave peak, 
  // only the other sensor is used. In the other four sectors, the output angle is a weighted combination of the 
  // value from each sensor, so there's no discontinuity at the sector boundaries.
  // The blended sectors each span 30 degrees, while the single-sensor sectors each span 60 degrees.
  // Only sensor A is used to determine the sector boundary locations, which don't need to be exact. Else I would 
  // use a similar averaging process as the center values above.
  thirtyA = tableA[(minA_idx + calibration_steps*4/12) % calibration_steps];
  sixtyA = tableA[(minA_idx + calibration_steps*5/12) % calibration_steps];

  Serial.println("CalibratedSinCosEncoder: Generating lookup tables");

  // Generate lookup tables to convert raw ADC data to shaft angle.
  // Note: It's possible some entries near the peaks will not be filled, but that's ok since they'll never be used anyway.
  // It would be possible to save a bit of space by not storing those entries at all, but more trouble than it's worth.
  for (i = 0; i < lut_resolution; i++) {
    a = minA + (uint32_t)(maxA - minA) * i / lut_resolution;
    b = minB + (uint32_t)(maxB - minB) * i / lut_resolution;
    // Reverse interpolated lookup to find the shaft angles that go with these sensor values
    for (j = 0; j < calibration_steps; j++) {
      if((tableA[j] <= a && tableA[j + 1] >= a) || (tableA[j + 1] <= a && tableA[j] >= a)) {
        int32_t val = ((int32_t)j + calibration_steps - minA_idx) << 16; // Integer portion of shaft position for this sensor value
        val += (((int32_t)a - tableA[j]) << 16) / (tableA[j + 1] - tableA[j]); // Fractional portion
        val /= calibration_steps; // Range is now 0-65535 for one revolution
        (tableB[j] < centerB) ? (_lutAN[i] = (uint16_t)val) : (_lutAP[i] = (uint16_t)val);
      }

      if((tableB[j] <= b && tableB[j + 1] >= b) || (tableB[j + 1] <= b && tableB[j] >= b)) {
        int32_t val = ((int32_t)j + calibration_steps - minA_idx) << 16;
        val += (((int32_t)b - tableB[j]) << 16) / (tableB[j + 1] - tableB[j]);
        val /= calibration_steps;
        (tableA[j] < centerA) ? (_lutBN[i] = (uint16_t)val) : (_lutBP[i] = (uint16_t)val);
      }
    }
  }

  // Set extra entry at end, used for interpolation
  _lutAN[lut_resolution] = _lutAN[0];
  _lutAP[lut_resolution] = _lutAP[0];
  _lutBN[lut_resolution] = _lutBN[0];
  _lutBP[lut_resolution] = _lutBP[0];

  Serial.println("CalibratedSinCosEncoder: Calibration complete");
  printCalibration();
}

// Helper function to look up shaft angle corresponding to sensor value
uint16_t CalibratedSinCosEncoder::InterpolatedLookup(const uint16_t *lut, uint16_t val, uint16_t range) const {
  int idx = (uint32_t)val * lut_resolution / range;
  int frac = (uint32_t)val * lut_resolution % range;
  return (uint16_t)((int32_t)lut[idx] + ((int32_t)((int16_t)(lut[idx+1] - lut[idx])) * frac / range));
}

float CalibratedSinCosEncoder::getSensorAngle() {
  uint16_t a, b;
  ReadSinCosEncoder(pinA, pinB, &a, &b);
  lastA = a;
  lastB = b;

  if(a >= sixtyA || a <= centerA - (sixtyA - centerA)) { // A is near peak. Use B reading only.
    angleA = blended = 0;
    angleB = InterpolatedLookup(a < centerA ? lutBN : lutBP, b - minB, maxB - minB);
    return (float)angleB * (_2PI/65536.0f);
  }
  else if(a <= thirtyA && a >= centerA - (thirtyA - centerA)) { // A is near center, therefore B is near peak. Use A only.
    angleB = blended = 0;
    angleA = InterpolatedLookup(b < centerB ? lutAN : lutAP, a - minA, maxA - minA);
    return (float)angleA * (_2PI/65536.0f);
  }

  // Otherwise both sensors have good data. Fade from one to the other so there are no 
  // discontinuities at the sector boundaries.

  // Convert sensor readings to angle depending on quadrant. In a perfect world, both angles would be equal.
  angleA = InterpolatedLookup(b < centerB ? lutAN : lutAP, a - minA, maxA - minA);
  angleB = InterpolatedLookup(a < centerA ? lutBN : lutBP, b - minB, maxB - minB);

  // Alpha goes from 0 to 1<<8 as a goes from thirtyA to sixtyA, above or below centerA
  int32_t alpha = ((int32_t)(abs((int16_t)(a - centerA)) - (int16_t)(thirtyA - centerA)) << 8) / (sixtyA - thirtyA);
  blended = (uint16_t)(angleA + (alpha * (int16_t)(angleB - angleA) >> 8));
  return (float)blended * (_2PI/65536.0f);
}

void CalibratedSinCosEncoder::printTable(const uint16_t *table, int num) {
  for (int i = 0; i < num; i++) {
    if(!(i & 7)) { Serial.print("\n\t"); delayMicroseconds(200); }
    Serial.print(table[i]); delayMicroseconds(200);
    if(i != num - 1) { Serial.print(", "); delayMicroseconds(200); }
  }
}

// Print data to serial so it can be copy/pasted into the code to avoid re-calibrating every startup.
void CalibratedSinCosEncoder::printCalibration() const {
  Serial.print("\tsensor.init("); delayMicroseconds(200);
  Serial.print(minA); Serial.print(", "); delayMicroseconds(200);
  Serial.print(maxA); Serial.print(", "); delayMicroseconds(200);
  Serial.print(minB); Serial.print(", "); delayMicroseconds(200);
  Serial.print(maxB); Serial.print(",\n\t\t"); delayMicroseconds(200);
  Serial.print(centerA); Serial.print(", "); delayMicroseconds(200);
  Serial.print(centerB); Serial.print(", "); delayMicroseconds(200);
  Serial.print(thirtyA); Serial.print(", "); delayMicroseconds(200);
  Serial.print(sixtyA); Serial.print(", "); delayMicroseconds(200);
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print(",\n\t\tlutAP, lutAN, lutBP, lutBN);\n\nconst uint16_t lutAP[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutAP, lut_resolution);
  Serial.print("\n};\n\nconst uint16_t lutAN[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutAN, lut_resolution);
  Serial.print("\n};\n\nconst uint16_t lutBP[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutBP, lut_resolution);
  Serial.print("\n};\n\nconst uint16_t lutBN[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutBN, lut_resolution);
  Serial.print("\n};\n"); delayMicroseconds(200);
}
