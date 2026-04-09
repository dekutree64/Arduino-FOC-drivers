This class is somewhat redundant to the existing CalibratedSensor in the Arduino-FOC-drivers repository, but is faster, and CalibratedSensor seems to have a small problem at the wraparound point that I couldn't solve (floating point wrapping is far more confusing than the power-of-two integers used here)


Example usage, running a stepper motor with a 6PWM BLDC driver on STM32G431:
```c++
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <SimpleFOCExperimental.h>
#include <encoders/as5048a/MagneticSensorAS5048A.h>
#include <sensors/Calibrated16bitSensor.h>

HybridStepperMotor motor(50);
BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15),
uint16_t readAS5048A(Sensor *s) { return static_cast<MagneticSensorAS5048A*>(s)->readRawAngle() << 2; }
MagneticSensorAS5048A sensor(PA3);
Calibrated16bitSensor calibrated(sensor, readAS5048A);
#define N_LUT 200
uint16_t lut[N_LUT+1];

void setup() {
  Serial.setTx(PC10);
  Serial.setRx(PC11);
  Serial.begin(115200);

  SPI.setSCLK(PA5);
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.begin();
  
  driver.voltage_power_supply= 12;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 1.0f;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.init();
  sensor.init();
  calibrated[i].init(&motor, N_LUT, lut);
  motor.linkSensor(&calibrated);
  motor.initFOC();
}
```
