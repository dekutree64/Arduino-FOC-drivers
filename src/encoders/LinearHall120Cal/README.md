This class is for use with three linear hall sensors such as 49E spaced 120 degrees around a magnet mounted to the end of the motor shaft.
The maximum accuracy I achieved in testing was about ±0.5° with oversampling on STM32.

Example usage: 
```c++
#define N_LUT 400
uint16_t lutAN[N_LUT+1], lutAP[N_LUT+1];
uint16_t lutBN[N_LUT+1], lutBP[N_LUT+1];
uint16_t lutCN[N_LUT+1], lutCP[N_LUT+1];
LinearHall120Cal sensor(PA1, PA2, PA3); // Change these to your sensor pins

void setup() {
  ...
  motor.init();
  sensor.init(&motor, 200, N_LUT, lutAP, lutAN, lutBP, lutBN, lutCP, lutCN);
  motor.linkSensor(&sensor);
  motor.initFOC();
}
```
