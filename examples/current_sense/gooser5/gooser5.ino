/** 
 * Gooser5 position motion control example with LinearHall encoder on ENC0 connector (pins E0B and E0C), 
 * and potentiometer on ENC1 connector (pin E1A) to control position.
 *
 */

// Compile for board "Generic G431CBUx"
#if !defined(STM32G4xx)

void setup() {}
void loop() {}

#else

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <current_sense/gooser5/Gooser5CurrentSense.h>
#include <encoders/linearhall/LinearHall.h>
#include <utilities/stm32math/STM32G4CORDICTrigFunctions.h>

// Motor instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_PHASE_AH, M0_PHASE_AL, M0_PHASE_BH, M0_PHASE_BL, M0_PHASE_CH, M0_PHASE_CL);
Gooser5CurrentSense currentSense = Gooser5CurrentSense(45.0f, 0); // 45mV/A for 31 amp version of ACS711KEXLT
LinearHall sensor(E0B_PIN, E0C_PIN, motor.pole_pairs);


void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  // Enable CORDIC for faster sin/cos
  SimpleFOC_CORDIC_Config();

  // turn on LED
  pinMode(PA12, OUTPUT);
  digitalWrite(PA12, 1);

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.dead_zone = 0.01;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  currentSense.linkDriver(&driver);

  // current sensing
  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration 
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  motor.velocity_limit = 4;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();

  // this is not essential, but improves current sense accuracy (moves the motor, needs to be after motor.init)
  currentSense.calibrateGain(&motor);

  // initialize and calibrate sensor hardware (moves the motor, needs to be after motor.init)
  sensor.init(&motor);

  // align encoder and start FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  _delay(1000);
}

void loop() {
  // this is where the potentiometer pin is read
  currentSense.startInjectedConversions();

  // main FOC algorithm function
  motor.loopFOC();

  // Set motor target to potentiometer reading. The first time this is called, it enables the channel,
  // calls startInjectedConversions, and waits for the result. Subsequent calls return the result immediately.
  motor.target = currentSense.getResultInjected(Gooser5CurrentSense::Channel::encoder0A)*_2PI/16384.0f;

  // Motion control function
  motor.move();

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();
}

#endif