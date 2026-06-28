#pragma once

#if defined(STM32G4xx)

#include <SimpleFOC.h>


// Gooser5 pin definitions
#define M0_PHASE_AH PA8
#define M0_PHASE_AL PB13
#define M0_PHASE_BH PA9
#define M0_PHASE_BL PB14
#define M0_PHASE_CH PA10
#define M0_PHASE_CL PB15
#define M1_PHASE_AH PC6
#define M1_PHASE_AL PC10
#define M1_PHASE_BH PB8
#define M1_PHASE_BL PB0
#define M1_PHASE_CH PB9
#define M1_PHASE_CL PB1
#define UART2_TX PB3 // Programming connector
#define UART2_RX PB4
#define UART3_TX PB10 // Communication connector
#define UART3_RX PC11
#define I2C1_SDA PB7
#define I2C1_SCL PA15
#define E0A_PIN PA6 // TIM3_CH1
#define E0B_PIN PA7 // TIM3_CH2, LinearHall pinA
#define E0C_PIN PA2 // LinearHall pinB
#define E1A_PIN PA5 // TIM2_CH1
#define E1B_PIN PA1 // TIM2_CH2, LinearHall pinA
#define E1C_PIN PA4 // LinearHall pinB
#define SPI1_NSS E1C_PIN
#define SPI1_SCK E1A_PIN
#define SPI1_MISO E0A_PIN
#define SPI1_MOSI E0B_PIN
#define SPI3_SCK UART2_RX
#define SPI3_MISO UART2_TX
#define SPI3_MOSI PB5
#define SWDIO_PIN PA13
#define SWCLK_PIN PA14
#define VBUS_PIN PC4
#define LED_PIN PA12


class Gooser5CurrentSense: public CurrentSense {
  public:
    Gooser5CurrentSense(float mVpA, int _motor_id = 0) : motor_id(_motor_id), volts_to_amps_ratio(1000.0f/mVpA) {}

    // CurrentSense interface implementing functions
    int init(float gainA_mVpA, float gainB_mVpA);
    int init() override { return init(NOT_SET, NOT_SET); }
    PhaseCurrent_s getPhaseCurrents() override;

    // ACS711 sensors seem to each have a bit different gain, so call this to calibrate them individually. gain_a and gain_b will 
    // be printed to serial if SimpleFOCDebug is enabled, and can be passed to init to avoid needing to calibrate every startup.
    // The motor will jump to a few different positions using motor.voltage_sensor_align to measure positive and negative current 
    // on each sensor. Currently only works with BLDCMotor (not hybrid stepper). 
    void calibrateGain(FOCMotor *motor);
    void calibrateOffsets(); // This is called by init, no need to call from user code

    int motor_id; // Set to 0 or 1. 0 reads the S0A/PB11 and S0B/PB12 pins, 1 reads S1A/PF0 and S1B/PA0.
    float volts_to_amps_ratio; // Nominal gain in amps per volt



  // --- ADC management section (can be used without a class instance) ---

    enum Channel { 
      current0A = 0, // S0A/PB11/A2C14 
      current0B, // S0B/PB12/A1C11
      current1A, // S1A/PF0/A1C10
      current1B, // S1B/PA0/A2C1
      dummy1,    // PA3, not connected to anything. Used to prevent illegal simultaneous read of A1C3 and A2C3 (see note in readme)
      encoder0A, // PA6/A2C3 (CAUTION! Do not read this at the same time as A1C3)
      encoder0B, // PA7/A2C4 LinearHall pinA
      encoder0C, // PA2/A1C3 LinearHall pinB (CAUTION! Do not read this at the same time as A2C3)
      encoder1A, // PA5/A2C13
      encoder1B, // PA1/A1C2 LinearHall pinA
      encoder1C, // PA4/A2C17 LinearHall pinB
      vbus,      // PC4/A2C5 (best to read this with injected channel)

      num
    };

    static void initADC(); // Can be called at startup, or automatically by setChannelEnabled.
    static void startInjectedConversions(bool wait_for_result = false); // Typically called once per iteration of motor.move and loopFOC.

    // This reconfigures the ADC, so readings will be invalid for ~50µs while DMA buffer is refilled using the new channel order.
    // Returns false if flag already in the requested state or too many items are enabled, in which case the DMA buffer remains valid.
    static bool setChannelEnabledRegular(Channel channel, bool enable, bool wait_for_buffer = false, int sample_time = 0);
    static bool setChannelEnabledInjected(Channel channel, bool enable, bool sample_and_wait = false, int sample_time = 2);

    // These will automatically enable the requested channel and wait for result if not already enabled.
    // Will return 0 if too many channels enabled already.
    // Results are 0-16383 range.
    static int getResultRegular(Channel channel);
    static int getResultInjected(Channel channel);
    static float getBusVoltage() { return getResultInjected(Channel::vbus) * (39.1f/16384.0f); }

    // These should only be modified internally, but can be read by user code
    static bool adc_initialized;
    static uint8_t oversamples_shift[2]; // Number of samples to average for each channel (note: 4x hardware oversampling on top of this)
    static uint16_t regular_flags; // Bitmask indicating which ADC items will be sampled in regular mode
    static uint16_t injected_flags; // Bitmask indicating which ADC items will be sampled with injected channels
    static uint8_t regular_num[2]; // Number of regular channels enabled (max 8 per ADC, see eChannel enum for which ADC each channel uses)
    static uint8_t injected_num[2]; // Number of injected channels enabled (max 4 per ADC, see eChannel enum for which ADC each channel uses)
    static uint8_t regular_idx[Channel::num]; // Each channels's location in the DMA buffer
    static uint8_t injected_idx[Channel::num]; // Which injected data register holds each channel's result
    static volatile uint16_t dma_buffer[2][32]; // Regular channel results
};

#endif
