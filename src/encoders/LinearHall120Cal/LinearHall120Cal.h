#ifndef LINEAR_HALL_120_CAL
#define LINEAR_HALL_120_CAL

#include <SimpleFOC.h>

// This function can be overridden with custom ADC code on platforms with poor analogRead performance.
void ReadLinearHalls120(int pinA, int pinB, int pinC, uint16_t *a, uint16_t *b, uint16_t *c);

class LinearHall120Cal: public Sensor {
  public:
    LinearHall120Cal(int _pinA, int _pinB, int _pinC) : pinA(_pinA), pinB(_pinB), pinC(_pinC) {}

    // The parameters for this are typically copy-pasted from the printout after calibrating
    void init(uint16_t _minA, uint16_t _maxA, uint16_t _minB, uint16_t _maxB, uint16_t _minC, uint16_t _maxC,
      uint16_t _centerA, uint16_t _centerB, uint16_t _centerC, int _lut_resolution,
      const uint16_t *_lutAP, const uint16_t *_lutAN,
      const uint16_t *_lutBP, const uint16_t *_lutBN,
      const uint16_t *_lutCP, const uint16_t *_lutCN);

    // Perform full calibration. The tables passed in here will be filled with data and their pointers retained.
    // Each table should have space for one more entry than _lut_resolution (simplifies interpolation code)
    void init(FOCMotor *motor, int calibration_steps, int _lut_resolution,
      uint16_t *_lutAP, uint16_t *_lutAN, uint16_t *_lutBP, uint16_t *_lutBN, uint16_t *_lutCP, uint16_t *_lutCN);

    // Helper function to look up shaft angle corresponding to sensor value
    uint16_t InterpolatedLookup(const uint16_t *lut, uint16_t val, uint16_t range) const;

    // Helper function to print lookup tables to serial
    static void printTable(const uint16_t *table, int num);

    // Print data to serial so it can be copy/pasted into the code to avoid re-calibrating every startup.
    void printCalibration() const;

protected:
    float getSensorAngle() override;

    int pinA, pinB, pinC; // CPU pin that each sensor is connected to
    uint16_t centerA=0, centerB=0, centerC=0; // Sensor reading when magnet is perpendicular to it
    uint16_t minA=0, maxA=0, minB=0, maxB=0, minC=0, maxC=0; // Minimum and maximum readings from each sensor during calibration
    const uint16_t *lutAP=NULL, *lutAN=NULL, *lutBP=NULL, *lutBN=NULL, *lutCP=NULL, *lutCN=NULL; // Lookup tables to convert sensor reading to shaft angle
    int lut_resolution=0; // Number of entries in each lookup table (excluding the extra entry to simplify interpolation)
    uint16_t lastA=0, lastB=0, lastC=0; // Last sensor readings, only retained for debugging purposes
    uint16_t angleA=0, angleB=0, angleC=0; // Last angle values calculated for each sensor, only retained for debugging purposes
};

#endif
