#ifndef FIELD_WEAKENING_SENSOR_H
#define FIELD_WEAKENING_SENSOR_H

class FieldWeakeningSensor : public Sensor {
public:
  FieldWeakeningSensor(Sensor& _wrapped, FOCMotor& _motor, float _factor = 0) : wrapped(_wrapped), motor(_motor), factor(_factor) {}

  void update() { wrapped.update(); Sensor::update(); }
  
  Sensor& wrapped;
  FOCMotor& motor;
  float factor = 0;

protected:
  float getSensorAngle() { return _normalizeAngle(wrapped.getMechanicalAngle() + factor * motor.voltage.q * motor.sensor_direction); }
};

#endif
