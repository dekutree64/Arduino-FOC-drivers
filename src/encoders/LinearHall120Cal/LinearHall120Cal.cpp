#include "LinearHall120Cal.h"

// This function can be overridden with custom ADC code on platforms with poor analogRead performance.
__attribute__((weak)) void ReadLinearHalls120(int pinA, int pinB, int pinC, uint16_t *a, uint16_t *b, uint16_t *c)
{
  *a = analogRead(pinA);
  *b = analogRead(pinB);
  *c = analogRead(pinC);
}

void LinearHall120Cal::init(uint16_t _minA, uint16_t _maxA, uint16_t _minB, uint16_t _maxB, uint16_t _minC, uint16_t _maxC,
  uint16_t _centerA, uint16_t _centerB, uint16_t _centerC, int _lut_resolution,
  const uint16_t *_lutAP, const uint16_t *_lutAN,
  const uint16_t *_lutBP, const uint16_t *_lutBN,
  const uint16_t *_lutCP, const uint16_t *_lutCN)
{
  minA = _minA; maxA = _maxA; minB = _minB; maxB = _maxB; minC = _minC; maxC = _maxC;
  centerA = _centerA; centerB = _centerB; centerC = _centerC; lut_resolution = _lut_resolution;
  lutAP = _lutAP; lutAN = _lutAN; lutBP = _lutBP; lutBN = _lutBN; lutCP = _lutCP; lutCN = _lutCN;
}

// Perform full calibration. The tables passed in here will be filled with data and their pointers retained.
// Each table should have space for one more entry than _lut_resolution (simplifies interpolation code)
void LinearHall120Cal::init(FOCMotor *motor, int calibration_steps, int _lut_resolution,
  uint16_t *_lutAP, uint16_t *_lutAN, uint16_t *_lutBP, uint16_t *_lutBN, uint16_t *_lutCP, uint16_t *_lutCN)
{
  int i, j;
  uint16_t a, b, c;
  // Extra entry at the end to simplify interpolation code
  int32_t tableA[calibration_steps+1]={0}, tableB[calibration_steps+1]={0}, tableC[calibration_steps+1]={0};

  lut_resolution = _lut_resolution;
  lutAP = _lutAP; lutAN = _lutAN; lutBP = _lutBP; lutBN = _lutBN; lutCP = _lutCP; lutCN = _lutCN;

  Serial.println("LinearHall120Cal: Begin calibration movement");
  // Rotate motor forward and back, recording the sensor values at each step.

  // Move slightly negative, so first loop iteration moves forward like subsequent iterations
  motor->setPhaseVoltage(motor->voltage_sensor_align, 0, _electricalAngle(-1 * _2PI / calibration_steps, motor->pole_pairs));
  delay(2);
  for (j = 0; j < 4; j++)
    for (i = 0; i < calibration_steps; i++) {
      uint32_t time = 2000000 / calibration_steps, start_time = _micros();
      while(_micros() < start_time + time) {
        float a = (i - 1) + (float)(_micros() - start_time) / time;
        motor->setPhaseVoltage(motor->voltage_sensor_align, 0, _electricalAngle(a * _2PI / calibration_steps, motor->pole_pairs));
      }
      ReadLinearHalls120(pinA, pinB, pinC, &a, &b, &c);
      tableA[i] += a; tableB[i] += b; tableC[i] += c;
    }
  // Complete previous revolution, so first loop iteration moves backward like subsequent iterations
  motor->setPhaseVoltage(motor->voltage_sensor_align, 0, 0);
  delay(2);
  for (j = 0; j < 4; j++)
    for (i = calibration_steps - 1; i >= 0; i--) {
      uint32_t time = 2000000 / calibration_steps, start_time = _micros();
      while(_micros() < start_time + time) {
        float a = (i + 1) - (float)(_micros() - start_time) / time;
        motor->setPhaseVoltage(motor->voltage_sensor_align, 0, _electricalAngle(a * _2PI / calibration_steps, motor->pole_pairs));
      }
      ReadLinearHalls120(pinA, pinB, pinC, &a, &b, &c);
      tableA[i] += a; tableB[i] += b; tableC[i] += c;
    }
  
  for (i = 0; i < calibration_steps; i++)
    tableA[i] >>= 3, tableB[i] >>= 3, tableC[i] >>= 3;
  // Set extra entry at end of tables, used for interpolation
  tableA[calibration_steps] = tableA[0]; tableB[calibration_steps] = tableB[0]; tableC[calibration_steps] = tableC[0];

  //Serial.print("const uint16_t tableA[] = {");
  //printTable((uint16_t*)tableA, calibration_steps, true);
  //Serial.print("\n};\n\nconst uint16_t tableB[] = {");
  //printTable((uint16_t*)tableB, calibration_steps, true);
  //Serial.print("\n};\n\nconst uint16_t tableC[] = {");
  //printTable((uint16_t*)tableC, calibration_steps, true);
  //Serial.print("\n};\n\n");

  // Find min/max values, and their locations in the table (=the associated shaft angle)
  int minA_idx, maxA_idx, minB_idx, maxB_idx, minC_idx, maxC_idx;
  minA = 65535, maxA = 0, minB = 65535, maxB = 0, minC = 65535, maxC = 0;
  for (i = 0; i < calibration_steps; i++) {
    if(tableA[i] <= minA) minA = tableA[i], minA_idx = i;
    if(tableA[i] >= maxA) maxA = tableA[i], maxA_idx = i;
    if(tableB[i] <= minB) minB = tableB[i], minB_idx = i;
    if(tableB[i] >= maxB) maxB = tableB[i], maxB_idx = i;
    if(tableC[i] <= minC) minC = tableC[i], minC_idx = i;
    if(tableC[i] >= maxC) maxC = tableC[i], maxC_idx = i;
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
  centerC = (uint16_t)(((uint32_t)
    tableC[(minC_idx + calibration_steps/4) % calibration_steps] +
    tableC[(maxC_idx + calibration_steps/4) % calibration_steps] +
    tableC[(minC_idx + calibration_steps*3/4) % calibration_steps] +
    tableC[(maxC_idx + calibration_steps*3/4) % calibration_steps]) / 4);

  Serial.println("LinearHall120Cal: Generating lookup tables");
  
  // Generate lookup tables to convert raw ADC data to shaft angle.
  // Note: It's possible some entries near the peaks will not be filled, but that's ok since they'll never be used anyway.
  // It would be possible to save a bit of space by not storing those entries at all, but more trouble than it's worth.
  for (i = 0; i < lut_resolution; i++) {
    a = minA + (uint32_t)(maxA - minA) * i / lut_resolution;
    b = minB + (uint32_t)(maxB - minB) * i / lut_resolution;
    c = minC + (uint32_t)(maxC - minC) * i / lut_resolution;
    // Reverse interpolated lookup to find the shaft angles that go with these sensor values
    for (j = 0; j < calibration_steps; j++) {
      if((tableA[j] <= a && tableA[j + 1] >= a) || (tableA[j + 1] <= a && tableA[j] >= a)) {
        int32_t angle = ((int32_t)j + calibration_steps - minA_idx) << 16; // Integer portion of shaft position for this sensor value
        angle += (((int32_t)a - tableA[j]) << 16) / (tableA[j + 1] - tableA[j]); // Fractional portion
        angle /= calibration_steps; // Range is now 0-65535 for one revolution
        if ((int16_t)(tableB[j] - centerB) < (int16_t)(tableC[j] - centerC))
          _lutAN[i] = (uint16_t)angle; else _lutAP[i] = (uint16_t)angle;
      }

      if((tableB[j] <= b && tableB[j + 1] >= b) || (tableB[j + 1] <= b && tableB[j] >= b)) {
        int32_t angle = ((int32_t)j + calibration_steps - minA_idx) << 16;
        angle += (((int32_t)b - tableB[j]) << 16) / (tableB[j + 1] - tableB[j]);
        angle /= calibration_steps;
        if ((int16_t)(tableA[j] - centerA) < (int16_t)(tableC[j] - centerC))
          _lutBN[i] = (uint16_t)angle; else _lutBP[i] = (uint16_t)angle;
      }

      if((tableC[j] <= c && tableC[j + 1] >= c) || (tableC[j + 1] <= c && tableC[j] >= c)) {
        int32_t angle = ((int32_t)j + calibration_steps - minA_idx) << 16;
        angle += (((int32_t)c - tableC[j]) << 16) / (tableC[j + 1] - tableC[j]);
        angle /= calibration_steps;
        if ((int16_t)(tableA[j] - centerA) < (int16_t)(tableB[j] - centerB))
          _lutCN[i] = (uint16_t)angle; else _lutCP[i] = (uint16_t)angle;
      }
    }
  }

  // Set extra entry at end, used for interpolation
  _lutAN[lut_resolution] = _lutAN[0];
  _lutAP[lut_resolution] = _lutAP[0];
  _lutBN[lut_resolution] = _lutBN[0];
  _lutBP[lut_resolution] = _lutBP[0];
  _lutCN[lut_resolution] = _lutCN[0];
  _lutCP[lut_resolution] = _lutCP[0];

  Serial.println("LinearHall120Cal: Calibration complete");
  printCalibration();
}

// Helper function to look up shaft angle corresponding to sensor value
uint16_t LinearHall120Cal::InterpolatedLookup(const uint16_t *lut, uint16_t val, uint16_t range) const {
  int idx = (uint32_t)val * lut_resolution / range;
  int frac = (uint32_t)val * lut_resolution % range;
  return (uint16_t)((int32_t)lut[idx] + ((int32_t)((int16_t)(lut[idx+1] - lut[idx])) * frac / range));
}

float LinearHall120Cal::getSensorAngle() {
  uint16_t angle;
  uint16_t a, b, c;
  int16_t diffA, diffB, diffC;

  ReadLinearHalls120(pinA, pinB, pinC, &a, &b, &c);
  a = _constrain(a, minA, maxA); b = _constrain(b, minB, maxB); c = _constrain(c, minC, maxC);
  lastA = a; lastB = b; lastC = c;
  diffA = (int16_t)(a - centerA); diffB = (int16_t)(b - centerB); diffC = (int16_t)(c - centerC);

  // If one sensor is within +- 15 degrees of its center value, use its value alone.
  // sin(15) is 0.2588, so the total -1 to +1 range divided by 8 is close enough.
  if (abs(diffA) < ((maxA - minA) >> 3)) {
    angleB = angleC = 0;
    angle = angleA = InterpolatedLookup(diffB < diffC ? lutAN : lutAP, a - minA, maxA - minA);
  }
  else if (abs(diffB) < ((maxB - minB) >> 3)) {
    angleA = angleC = 0;
    angle = angleB = InterpolatedLookup(diffA < diffC ? lutBN : lutBP, b - minB, maxB - minB);
  }
  else if (abs(diffC) < ((maxC - minC) >> 3)) {
    angleA = angleB = 0;
    angle = angleC = InterpolatedLookup(diffA < diffB ? lutCN : lutCP, c - minC, maxC - minC);
  }
  // Average the results from whichever two sensors are closest to their center values.
  // Averaging is done by adding half the signed difference, which gives correct results near the wraparound point.
  else if(abs(diffA) > abs(diffB)) {
    if(abs(diffA) > abs(diffC)) {
      angleA = 0;
      angleB = InterpolatedLookup(diffA < diffC ? lutBN : lutBP, b - minB, maxB - minB);
      angleC = InterpolatedLookup(diffA < diffB ? lutCN : lutCP, c - minC, maxC - minC);
      angle = angleB + ((int16_t)(angleC - angleB) >> 1);
    }
    else {
      angleA = InterpolatedLookup(diffB < diffC ? lutAN : lutAP, a - minA, maxA - minA);
      angleB = InterpolatedLookup(diffA < diffC ? lutBN : lutBP, b - minB, maxB - minB);
      angleC = 0;
      angle = angleA + ((int16_t)(angleB - angleA) >> 1);
    }
  } else {
    if(abs(diffB) > abs(diffC)) {
      angleA = InterpolatedLookup(diffB < diffC ? lutAN : lutAP, a - minA, maxA - minA);
      angleB = 0;
      angleC = InterpolatedLookup(diffA < diffB ? lutCN : lutCP, c - minC, maxC - minC);
      angle = angleA + ((int16_t)(angleC - angleA) >> 1);
    }
    else {
      angleA = InterpolatedLookup(diffB < diffC ? lutAN : lutAP, a - minA, maxA - minA);
      angleB = InterpolatedLookup(diffA < diffC ? lutBN : lutBP, b - minB, maxB - minB);
      angleC = 0;
      angle = angleA + ((int16_t)(angleB - angleA) >> 1);
    }
  }
  return (float)angle * (_2PI/65536.0f);
}

// Helper function to print lookup tables to serial
void LinearHall120Cal::printTable(const uint16_t *table, int num) {
  for (int i = 0; i < num; i++) {
    if(!(i & 7)) { Serial.print("\n\t"); delayMicroseconds(200); }
    Serial.print(table[i]); delayMicroseconds(200);
    if(i != num - 1) { Serial.print(", "); delayMicroseconds(200); }
  }
}

// Print data to serial so it can be copy/pasted into the code to avoid re-calibrating every startup.
void LinearHall120Cal::printCalibration() const {
  Serial.print("\tsensor.init("); delayMicroseconds(200);
  Serial.print(minA); Serial.print(", "); delayMicroseconds(200);
  Serial.print(maxA); Serial.print(", "); delayMicroseconds(200);
  Serial.print(minB); Serial.print(", "); delayMicroseconds(200);
  Serial.print(maxB); Serial.print(", "); delayMicroseconds(200);
  Serial.print(minC); Serial.print(", "); delayMicroseconds(200);
  Serial.print(maxC); Serial.print(",\n\t\t"); delayMicroseconds(200);
  Serial.print(centerA); Serial.print(", "); delayMicroseconds(200);
  Serial.print(centerB); Serial.print(", "); delayMicroseconds(200);
  Serial.print(centerC); Serial.print(", "); delayMicroseconds(200);
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print(",\n\t\tlutAPg, lutAN, lutBP, lutBN, lutCP, lutCN);\n\nconst uint16_t lutAP[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutAP, lut_resolution+1);
  Serial.print("\n};\n\nconst uint16_t lutAN[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutAN, lut_resolution+1);
  Serial.print("\n};\n\nconst uint16_t lutBP[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutBP, lut_resolution+1);
  Serial.print("\n};\n\nconst uint16_t lutBN[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutBN, lut_resolution+1);
  Serial.print("\n};\n\nconst uint16_t lutCP[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutCP, lut_resolution+1);
  Serial.print("\n};\n\nconst uint16_t lutCN[");
  Serial.print(lut_resolution); delayMicroseconds(200);
  Serial.print("+1] = { // Extra entry at end for interpolation"); delayMicroseconds(200);
  printTable(lutCN, lut_resolution+1);
  Serial.print("\n};\n"); delayMicroseconds(200);
}
