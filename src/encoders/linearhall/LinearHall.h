#ifndef LINEAR_HALL_SENSOR_LIB_H
#define LINEAR_HALL_SENSOR_LIB_H

#include <SimpleFOC.h>

// This function can be overridden with custom ADC code on platforms with poor analogRead performance.
void ReadLinearHalls(int hallA, int hallB, int *a, int *b);

class LinearHall: public Sensor{
  public:
    // Note: With sensor_spacing_120 you may need to swap hallA and hallB (one way is effectively -240 degrees apart and won't work).
    LinearHall(int hallA, int hallB, int pp, bool sensor_spacing_120 = false);

    void init(int centerA, int centerB, float _amplitude_ratio = 1.0f); // Initialize without moving motor
    void init(class FOCMotor *motor); // Move motor to find center values

    int centerA;
    int centerB;
    int lastA, lastB;
    int electrical_rev;
    float amplitude_ratio; // Correction factor if one sensor is slightly farther from the magnets
    bool sensor_spacing_120; // false = sensors spaced 90 electrical degrees, true = 120 degrees

  protected:
    float readSensors();

    // Get current shaft angle from the sensor hardware, and
    // return it as a float in radians, in the range 0 to 2PI.
    //  - This method is pure virtual and must be implemented in subclasses.
    //    Calling this method directly does not update the base-class internal fields.
    //    Use update() when calling from outside code.
    float getSensorAngle() override;

    int pinA;
    int pinB;
    int pp;
    float prev_reading;
};

#endif
