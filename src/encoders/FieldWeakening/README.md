This is a wrapper-type sensor class used to perform field weakening for increased top speed, at the cost of some torque. It offsets the electrical angle proportionally to the applied voltage, so there is no discontinuity in response as you increase speed. However that does mean you lose torque sooner than strictly necessary.


Usage:
```c++
FieldWeakeningSensor weakening(sensor, motor);

void setup() {
  ...
  weakening.factor = 0.3/motor.voltage_limit/motor.pole_pairs;
  motor.linkSensor(&weakening);
}
```
That will offset the electrical angle by 0.3 radians (about 17 degrees) when you reach maximum voltage.
