#include <SimpleFOC.h>
#include <current_sense\hardware_specific\stm32\stm32_mcu.h>
#include "StepstickCurrentSense.h"

#define ADC_VOLTAGE 3.0f

// Regular conversions are accumulated in a DMA buffer to give a running average over this time period.
// At 42.5MHz ADC clock, 1700 cycles is 40µs (one PWM cycle at standard 25KHz). However, the motor update 
// typically takes 50-60µs, and using a longer time window only gives a slight lowpass filtering effect, 
// so the value chosen is ~47µs which allows 64x oversampling on two linear halls for best quality if no 
// other regular channels are used.
#define MAX_TIME 2000 // ADC clock cycles


// --- Gooser5CurrentSense static variables ---

// Array to go from Channel enum to ADC2 channel number
static const uint8_t dChannel[StepstickCurrentSense::Channel::num] = {17, 13, 3, 4, 10};

// 12.5 cycle conversion time + sample time, multiplied by 4x hardware oversampling.
// Sample times from RM0440: 0=2.5, 1=6.5, 2=12.5, 3=24.5, 4=47.5, 5=92.5, 6=247.5, 7=640.5
static const uint16_t time_table[] = { 15*4, 19*4, 25*4, 37*4, 60*4, 105*4, 260*4, 653*4 };

bool StepstickCurrentSense::adc_initialized = false;
uint8_t StepstickCurrentSense::oversamples_shift = 0;
uint16_t StepstickCurrentSense::enable_flags = 0;
uint8_t StepstickCurrentSense::enabled_num = 0;
uint8_t StepstickCurrentSense::channel_idx[StepstickCurrentSense::Channel::num] = {0};
volatile uint16_t StepstickCurrentSense::dma_buffer[32];


// --- StepstickCurrentSense static functions ---

void StepstickCurrentSense::initADC() {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_DMA1EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
  RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1; // ADC use SYSCLK
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD;
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN;
  delayMicroseconds(20); // Regulator startup time from STM32G431 datasheet
  // Dual mode injected simultaneous (ADC1 and 2 operate independently for regular conversions), enable temperature sensor, clock 170MHz/4 = 42.5MHz
  ADC12_COMMON->CCR = 5 | ADC_CCR_VSENSESEL | ADC_CCR_PRESC_1;
  ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL); // Must be done before ADC_CR_ADEN
  ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL);
  ADC1->CR |= ADC_CR_ADEN; ADC2->CR |= ADC_CR_ADEN;
  while(!(ADC1->ISR & ADC_ISR_ADRDY)){} // Wait for ADC startup
  ADC1->CFGR = ADC_CFGR_CONT | ADC_CFGR_OVRMOD;
  ADC2->CFGR = ADC_CFGR_CONT | ADC_CFGR_OVRMOD | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
  ADC2->CFGR2 = ADC_CFGR2_ROVSE | ADC_CFGR2_OVSR_0; // 4x oversampling (regular channels only)
  ADC1->SQR1 = (16<<6); // Temperature sensor
  ADC1->JSQR = 1|(4<<9)|(3<<15)|ADC_JSQR_JEXTEN_0; // ADC1 converts ch4 then ch3 (phase current A, B), trigger on rising edge.
  ADC2->JSQR = 1|(2<<9)|(1<<15)|ADC_JSQR_JEXTEN_0; // ADC2 converts ch2 then ch1 (phase current C, D), trigger on rising edge.
  ADC1->SMPR1 = (2<<4*3)|(2<<3*3); // Sample time for current sense 12.5 cycles
  ADC2->SMPR1 = (2<<2*3)|(2<<1*3);
  ADC1->SMPR2 = (6<<(15-9)*3); // Sample time for temperature 248.5 cycles (5.8 microseconds)

  DMAMUX1_Channel0->CCR = 36; // ADC2, from Table 91 on page 420 of reference manual rm0440
  DMA1_Channel1->CPAR = (uint32_t)&ADC2->DR;
  DMA1_Channel1->CMAR = (uint32_t)dma_buffer;
  // 16-bit memory, 16-bit peripheral, circular mode
  DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC;

  ADC1->CR |= ADC_CR_ADSTART; // Start temperature monitoring
  adc_initialized = true;
}

bool StepstickCurrentSense::setChannelEnabled(Channel channel, bool enable, bool wait_for_buffer, int sample_time) {
  int flag = 1<<channel;
  if (((enable_flags & flag) && enable) || (!(enable_flags & flag) && !enable))
    return false; // Already in the requested state

  // Pause ADC
  if (!adc_initialized)
    initADC();
  else if (ADC2->CR & ADC_CR_ADSTART) {
    ADC2->CR |= ADC_CR_ADSTP;
    while(ADC2->CR & ADC_CR_ADSTP);
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
  }

  // Set enable flag and sample time
  if (!enable)
    enable_flags &= ~flag, channel_idx[channel] = 0;
  else {
    enable_flags |= flag;
    int ch = dChannel[channel];
    if(ch <= 9) ADC2->SMPR1 = (ADC2->SMPR1 & ~(7<<(ch*3))) | (sample_time<<(ch*3));
    else ch-=9, ADC2->SMPR2 = (ADC2->SMPR2 & ~(7<<(ch*3))) | (sample_time<<(ch*3));
  }

  // Rebuild sequence register and other variables
  uint32_t SQR1 = 0, total_time = 0;
  enabled_num = 0;
  for (int i = 0; (1<<i) <= enable_flags; i++) {
    if (enable_flags & (1<<i)) {
      int ch = dChannel[i];
      if(ch<=9) total_time += time_table[(ADC2->SMPR1 >> (ch*3)) & 7];
      else      total_time += time_table[(ADC2->SMPR2 >> ((ch-9)*3)) & 7];
      SQR1 |= ch<<(enabled_num+1)*6;
      channel_idx[i] = enabled_num++;
    }
  }

  // Reconfigure ADC
  if (enabled_num != 0) {
    oversamples_shift = 0;
    while(oversamples_shift < 5 && (total_time<<1) <= MAX_TIME)
      oversamples_shift++, total_time <<= 1; 
    DMA1_Channel1->CNDTR = enabled_num << oversamples_shift;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC2->SQR1 = SQR1|(enabled_num-1);
    ADC2->CR |= ADC_CR_ADSTART;
    if (wait_for_buffer)
      delayMicroseconds(50);
  }
  return true;
}

int StepstickCurrentSense::getResult(Channel channel) {
  if (!(enable_flags & (1<<channel)))
    setChannelEnabled(channel, true, true);
  int sum = 0;
  volatile uint16_t *buffer = dma_buffer + channel_idx[channel];
  for (int i = 0; i < (1<<oversamples_shift); i++)
    sum += buffer[i];
  return sum >> oversamples_shift;
}

float StepstickCurrentSense::getTemperature() {
  static const uint16_t CAL1_TEMP = 30, CAL2_TEMP = 130, TS_CAL1 = *(uint16_t*)0x1FFF75A8, TS_CAL2 = *(uint16_t*)0x1FFF75CA;
  static const float temp_factor = (float)(CAL2_TEMP - CAL1_TEMP) / (float)(TS_CAL2 - TS_CAL1);
  return (ADC1->DR - TS_CAL1) * temp_factor + CAL1_TEMP;
}

StepstickCurrentSense::Channel StepstickCurrentSense::pinToChannel(int pin) {
  switch(pin) {
  default:
  case SPI1_NSS:  return Channel::nss;
  case SPI1_SCK:  return Channel::sck;
  case SPI1_MISO: return Channel::miso;
  case SPI1_MOSI: return Channel::mosi;
  case EN_PIN:    return Channel::en_pin;
  }
}


// --- StepstickCurrentSense member functions ---

StepstickCurrentSense::StepstickCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC, int _pinD) {
  pinA = _pinA, pinB = _pinB, pinC = _pinC, pinD = _pinD;
  volts_to_amps_ratio = 1000.0f / _mVpA;
  gain_a = gain_b = gain_c = gain_d = volts_to_amps_ratio*(ADC_VOLTAGE/4096.0f);
}

int StepstickCurrentSense::init() {
  if (driver==nullptr) {
    SIMPLEFOC_DEBUG("CUR: Driver not linked!");
    return 0;
  }
  if (motor==nullptr) {
    SIMPLEFOC_DEBUG("CUR: Motor not linked!");
    return 0;
  }

  // This is similar to _driverSyncLowSide.
  // Initially the ADC would trigger on highside center, so jump past one timer update event to get it triggering on lowside.
  TIM1->CR1 &= ~TIM_CR1_CEN; // Pause timer
  TIM1->CNT = (TIM1->CR1 & TIM_CR1_DIR) ? 0 : TIM1->ARR; // Set counter to end value of whichever direction it's going, to skip udpate event
  MODIFY_REG(TIM1->CR2, TIM_CR2_MMS, LL_TIM_TRGO_UPDATE); // Trigger on update event
  ADC1->CR |= ADC_CR_JADSTART; // Enable injected conversions
  TIM1->CR1 |= TIM_CR1_CEN; // Restart timer

  calibrateOffsets();
  initialized = true;
  return 1; // Success
}

PhaseCurrent_s StepstickCurrentSense::getPhaseCurrents() {
  int raw_a = ADC1->JDR1, raw_b = ADC1->JDR2, raw_c = ADC2->JDR1, raw_d = ADC2->JDR2; // Fetch these as close to simultaneously as possible
  if((raw_a-=(int)offset_ia)<raw_min) current_a=0; else current_a=raw_a*gain_a;
  if((raw_b-=(int)offset_ib)<raw_min) current_b=0; else current_b=raw_b*gain_b;
  if((raw_c-=(int)offset_ic)<raw_min) current_c=0; else current_c=raw_c*gain_c;
  if((raw_d-=(int)offset_id)<raw_min) current_d=0; else current_d=raw_d*gain_d;

  switch(driver_type) {
  case DriverType::Stepper: {
    ABCurrent_s current;
    // Pin order is very confusing here.
    // Timer channels 1,2,3,4 and sense channels a,b,c,d correspond to stepstick pins 2B,2A,1A,1B.
    // c,d is first coil (alpha current), b,a is second coil (beta current).
    // StepperDriver4PWM outputs positive voltage on 1B if Ualpha is positive, and positive on 2B if Ubeta is positive.
    // At low speed (no phase lag), positive Ualpha gives valid current on sense channel c.
    if(current_c > current_d) current.alpha = current_c;
    else current.alpha = -current_d;
    if(current_b > current_a) current.beta = current_b;
    else current.beta = -current_a;
    return (PhaseCurrent_s){current.alpha,current.beta,0};
  }
  case DriverType::DC: {
    PhaseCurrent_s current = {0,0,0};
    // Sense channels A,B,C,D correspond to stepstick pins 2B,2A,1A,1B.
    // C,D is motor
    // DCDriver sets 1A positive if first motor's voltage is positive, and 2A positive if second motor's voltage is positive.
    current.a = (current_c > current_d) ? current_c : -current_d;
    return current;
  }
  case DriverType::BLDC: {
    PhaseCurrent_s current = {current_c, current_d, current_b};
    // If two phases are valid, full current is known
    if     (current.a&&current.b) current.c = -current.a-current.b;
    else if(current.a&&current.c) current.b = -current.a-current.c;
    else if(current.b&&current.c) current.a = -current.b-current.c;
    else { // Reconstruct all three currents from one known, on the assumption that they are all sinusoidal and spaced 120 degrees
      if(motor->shaft_velocity < low_speed_threshold) last_known_lag = 0;
      float magnitude, angle = _normalizeAngle(motor->electrical_angle + last_known_lag);
      const float sa = _sin(angle), sb = _sin(angle-_2PI/3), sc = _sin(angle-_2PI*2/3); // Each phase's sine wave at current angle
      if     (current.a) magnitude = current.a/sa;
      else if(current.b) magnitude = current.b/sb;
      else if(current.c) magnitude = current.c/sc;
      return (PhaseCurrent_s){sa*magnitude, sb*magnitude, sc*magnitude};
    }
    // Update current vector angle
    ABCurrent_s ab = getABCurrents(current);
    last_known_lag = fmod(_atan2(-ab.alpha, ab.beta) - motor->electrical_angle, _2PI);
    if(last_known_lag >= _PI) last_known_lag -= _2PI;
    else if(last_known_lag <= -_PI) last_known_lag += _2PI;
    return current; }
  default: return (PhaseCurrent_s){0,0,0};
  }
}

void StepstickCurrentSense::calibrateOffsets() {
  const int calibration_rounds = 1000;

  if (driver_type == DriverType::BLDC)
    static_cast<BLDCDriver*>(driver)->setPwm(driver->voltage_limit/2, driver->voltage_power_supply/2, driver->voltage_power_supply/2);

  _delay(10);
  // find adc offset = zero current voltage
  offset_ia = offset_ib = 0;
  // read the adc voltage 1000 times ( arbitrary number )
  for (int i = 0; i < calibration_rounds; i++) {
    offset_ia += ADC1->JDR1;
    offset_ib += ADC1->JDR2;
    offset_ic += ADC2->JDR1;
    offset_id += ADC2->JDR2;
    delayMicroseconds(40); // One 25kHz PWM cycle
  }
  // calculate the mean offsets
  offset_ia /= calibration_rounds; //SIMPLEFOC_DEBUG("CS: offset_ia ", offset_ia); _delay(40);
  offset_ib /= calibration_rounds; //SIMPLEFOC_DEBUG("CS: offset_ib ", offset_ib); _delay(40);
  offset_ic /= calibration_rounds; //SIMPLEFOC_DEBUG("CS: offset_ic ", offset_ic); _delay(40);
  offset_id /= calibration_rounds; //SIMPLEFOC_DEBUG("CS: offset_id ", offset_id); _delay(40);

  if (driver_type == DriverType::BLDC)
    static_cast<BLDCDriver*>(driver)->setPwm(0,0,0);
}
