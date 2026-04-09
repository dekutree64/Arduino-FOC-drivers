This class is for use with two linear hall sensors such as 49E spaced 90 degrees around a magnet mounted to the end of the motor shaft, and is faster and more accurate than running LinearHall through CalibratedSensor.
The maximum accuracy I achieved in testing was about ±0.5° with oversampling on STM32.

Usage:
```c++
#define N_LUT 400
uint16_t lutAN[N_LUT+1], lutAP[N_LUT+1];
uint16_t lutBN[N_LUT+1], lutBP[N_LUT+1];
SinCosEncoder sensor(PA1, PA2); // Change these to your sensor pins

void setup() {
  ...
  motor.init();
  sensor.init(&motor, 200, N_LUT, lutAP, lutAN, lutBP, lutBN);
  motor.linkSensor(&sensor);
  motor.initFOC();
}
```
