#ifndef CALIBRATED_16BIT_SENSOR_H
#define CALIBRATED_16BIT_SENSOR_H

#include <SimpleFOC.h>

// Better to provide a custom helper function to read the raw integer angle from your sensor, but this works too.
uint16_t Calibrated16bitSensor_DefaultReadHelper(Sensor *s);

class Calibrated16bitSensor: public Sensor{
  public:
    Calibrated16bitSensor(Sensor& _wrapped, uint16_t (*_readSensor16bit)(Sensor*) = Calibrated16bitSensor_DefaultReadHelper) :
      wrapped(_wrapped), readSensor16bit(_readSensor16bit) {}

    // After calibrating, the lookup table printed to serial can be copy-pasted into the program 
    // and passed in here to avoid re-calibrating every startup.
    void init(int _lut_resolution, const uint16_t *_lut);

    // Perform full calibration. The table passed in here will be filled with data and its pointer retained.
    // The table should have space for one more entry than _lut_resolution (simplifies interpolation code)
    void init(FOCMotor *motor, int _lut_resolution, uint16_t *_lut);

    void update();

    // Print data to serial so it can be copy/pasted into the code to avoid re-calibrating every startup.
    void printCalibration() const;

protected:
    float getSensorAngle() override;

    Sensor &wrapped;
    uint16_t (*readSensor16bit)(Sensor*); // Helper function allows reading raw integer angle from most sensors
    const uint16_t *lut=NULL; // Table has one extra entry on the end to simplify interpolation code
    int lut_resolution=0; // Number of entries in lookup table, excluding the extra for interpolation
    uint16_t lastRaw=0, beforeLastRaw=0, lastCal=0; // Last sensor reading
};

#endif
