#pragma once

#if defined(STM32G4xx)

#include <SimpleFOC.h>


#define CS_mVpA 293.333f // Current sense nominal millivolts per amp for 2.2k resistors

// Connections between CPU and driver
#define EN_DRV PB8 
#define FAULT_PIN PF0
#define HFS_PIN PB0

// Stepstick enable and home pins
#define EN_PIN PF1
#define HOME_PIN PB7

// Stepstick step and direction pins
#define STEP_PIN PA_12_ALT1 // TIM4_CH2
#define DIR_PIN PB6 // TIM4_CH1

// Encoder connector
#define SPI1_NSS PA4 // ADC2_IN17
#define SPI1_SCK PA5 // ADC2_IN13
#define SPI1_MISO PA6 // ADC2_IN3
#define SPI1_MOSI PA7 // ADC2_IN4

// Stepstick SPI pins
#define SPI3_NSS PA15
#define SPI3_SCK PB3
#define SPI3_MISO PB4
#define SPI3_MOSI PB5

// UART shares pins with SPI3
#define UART_TX SPI3_SCK
#define UART_RX SPI3_MISO

// I2C
#define I2C1_SDA HOME_PIN
#define I2C1_SCL SPI3_NSS

// Current sense
#define CS1_PIN PA3 // ADC1_IN4
#define CS2_PIN PA2 // ADC1_IN3
#define CS3_PIN PA1 // ADC12_IN2
#define CS4_PIN PA0 // ADC12_IN1


// Current sense class for MAX22213, with 4 unidirectional lowside channels
class StepstickCurrentSense: public CurrentSense {
  public:  
    StepstickCurrentSense(float mVpA, int pinA, int pinB, int pinC, int pinD);

    // CurrentSense interface implementing functions 
    int init() override;
    PhaseCurrent_s getPhaseCurrents() override;
    int driverAlign(float, bool) override { return skip_align; } // Auto-alignment not supported (fail if skip_align is false)
    void linkMotor(FOCMotor *_motor) { motor = _motor; }

    void calibrateOffsets();

    FOCMotor *motor = NULL;
    int pinD;
    float gain_d; //!< phase D gain
    float offset_id; //!< zero current D voltage value (center of the adc reading)
    float current_a, current_b, current_c, current_d; // Last current readings
    float volts_to_amps_ratio; //!< Volts to amps ratio
    float last_known_lag = 0; // Angle of current vector (atan2 of AB current) minus applied electrical angle, wrapped to -pi,+pi range
    float low_speed_threshold = _PI; // motor.shaft_velocity where last_known_lag should be set to 0 even if it's not measurable
    int raw_min = 10; // Minimum ADC reading to be considered valid (after subtracting zero offset)



  // --- ADC management section (can be used without a class instance) ---

    enum Channel { 
      nss = 0, // PA4/A2C17
      sck,     // PA5/A2C13
      miso,    // PA6/A2C3
      mosi,    // PA7/A2C4
      en_pin,  // PF1/A2C10

      num
    };

    static void initADC(); // Can be called at startup, or automatically by setChannelEnabled.

    // This reconfigures the ADC, so readings will be invalid for ~50µs while DMA buffer is refilled using the new channel order.
    // Returns false if flag already in the requested state, in which case the DMA buffer remains valid.
    // sample_time 2 works better for high resistance sources like potentiometers.
    static bool setChannelEnabled(Channel channel, bool enable, bool wait_for_buffer = false, int sample_time = 0);
    static void setChannelSampleTime(Channel channel, int setting); // Setting is 0-7, see page 691 of RM0440

    // This will automatically enable the requested channel and wait for result if not already enabled. Results are 0-16383 range.
    static int getResult(Channel channel);
    static float getTemperature(); // °C
    static Channel pinToChannel(int pin); // Takes pin name like PA4

    // These should only be modified internally, but can be read by user code
    static bool adc_initialized;
    static uint8_t oversamples_shift; // Number of samples to average for each channel (note: 4x hardware oversampling on top of this)
    static uint16_t enable_flags; // Bitmask indicating which ADC items will be sampled in regular mode
    static uint8_t enabled_num; // Number of channels enabled
    static uint8_t channel_idx[Channel::num]; // Each channels's location in the DMA buffer
    static volatile uint16_t dma_buffer[32]; // Regular channel results
};

#endif
