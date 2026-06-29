#if defined(STM32G4xx)

#include <SimpleFOC.h>
#include <current_sense/gooser5/Gooser5CurrentSense.h>

#define ADC_VOLTAGE 3.3f


// --- Gooser5CurrentSense static variables ---

// [0] is which ADC to use (0=ADC1, 1=ADC2), [1] is channel number
static const uint8_t dChannel[Gooser5CurrentSense::Channel::num][2] = {
/*current0A*/ {1,14},
/*current0B*/ {0,11},
/*current1A*/ {0,10},
/*current1B*/ {1, 1},
/*dummy1   */ {0, 2},
/*encoder0A*/ {1, 3},
/*encoder0B*/ {1, 4},
/*encoder0C*/ {0, 3},
/*encoder1A*/ {1,13},
/*encoder1B*/ {0, 2},
/*encoder1C*/ {1,17},
/*vbus     */ {1, 5}
};

// This table is indexed by num_enabled-1. The more channels are enabled, the fewer samples each of them can take. For example
// a value of 3 means 1<<3=8 entries will be stored in the DMA buffer for each item. Each of those is already 4 samples 
// summed together by hardware oversampling, giving a total of 32 samples averaged together and scaled to 0-16383 range.
static const uint8_t dOversamplesShift[] = {5, 4, 3, 3, 2, 2, 2, 2};

volatile uint32_t *JDR[2] = {&ADC1->JDR1,&ADC2->JDR1};

bool Gooser5CurrentSense::adc_initialized = false;
uint8_t Gooser5CurrentSense::oversamples_shift[2] = {0};
uint16_t Gooser5CurrentSense::regular_flags = 0;
uint16_t Gooser5CurrentSense::injected_flags = 0;
uint8_t Gooser5CurrentSense::regular_num[2] = {0};
uint8_t Gooser5CurrentSense::injected_num[2] = {0};
uint8_t Gooser5CurrentSense::regular_idx[Gooser5CurrentSense::Channel::num] = {0};
uint8_t Gooser5CurrentSense::injected_idx[Gooser5CurrentSense::Channel::num] = {0};
volatile uint16_t Gooser5CurrentSense::dma_buffer[2][32];


// --- Override of LinearHall reading function ---

void ReadLinearHalls(int hallA, int hallB, int *a, int *b) {
  if(hallA == PA7 && hallB == PA2) {
    *a = Gooser5CurrentSense::getResultRegular(Gooser5CurrentSense::Channel::encoder0B),
    *b = Gooser5CurrentSense::getResultRegular(Gooser5CurrentSense::Channel::encoder0C);
  } else if(hallA == PA1 && hallB == PA4) {
    *a = Gooser5CurrentSense::getResultRegular(Gooser5CurrentSense::Channel::encoder1B),
    *b = Gooser5CurrentSense::getResultRegular(Gooser5CurrentSense::Channel::encoder1C);
  } else *a = *b = 0;
}


// --- Gooser5CurrentSense static functions ---

void Gooser5CurrentSense::initADC() {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_DMA1EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
  RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1; // ADC use SYSCLK
  ADC1->CR &= ~ADC_CR_DEEPPWD; ADC2->CR &= ~ADC_CR_DEEPPWD;
  ADC1->CR |= ADC_CR_ADVREGEN; ADC2->CR |= ADC_CR_ADVREGEN;
  delayMicroseconds(20); // Regulator startup time from STM32G431CB datasheet
  ADC12_COMMON->CCR = ADC_CCR_PRESC_1; // Clock 170MHz/4 = 42.5MHz
  ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL); // Must be done before ADC_CR_ADEN
  ADC2->CR |= ADC_CR_ADCAL; while(ADC2->CR & ADC_CR_ADCAL);
  ADC1->CR |= ADC_CR_ADEN; ADC2->CR |= ADC_CR_ADEN;
  while(!(ADC1->ISR & ADC_ISR_ADRDY)){} // Wait for ADC startup
  // Enable DMA, CMA circular, ignore overrun, convert continuously, disable injected queue
  ADC1->CFGR = ADC2->CFGR = ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_OVRMOD | ADC_CFGR_CONT | ADC_CFGR_JQDIS;
  ADC1->CFGR2 = ADC2->CFGR2 = ADC_CFGR2_ROVSE | ADC_CFGR2_JOVSE | ADC_CFGR2_OVSR_0; // 4x oversampling

  DMAMUX1_Channel0->CCR = 5; // ADC1, from Table 91 on page 420 of reference manual rm0440
  DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
  DMA1_Channel1->CMAR = (uint32_t)&dma_buffer[0];
  DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC; // 16-bit memory, 16-bit peripheral, circular mode

  DMAMUX1_Channel1->CCR = 36; // ADC2, from Table 91 on page 420 of reference manual rm0440
  DMA1_Channel2->CPAR = (uint32_t)&ADC2->DR;
  DMA1_Channel2->CMAR = (uint32_t)&dma_buffer[1];
  DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC; // 16-bit memory, 16-bit peripheral, circular mode

  adc_initialized = true;
}

void Gooser5CurrentSense::startInjectedConversions(bool wait_for_result) {
  if(ADC1->JSQR) ADC1->CR |= ADC_CR_JADSTART;
  if(ADC2->JSQR) ADC2->CR |= ADC_CR_JADSTART;
  if(wait_for_result) while((ADC1->CR | ADC2->CR) & ADC_CR_JADSTART);
}

bool Gooser5CurrentSense::setChannelEnabledRegular(Channel channel, bool enable, bool wait_for_buffer, int sample_time) {
  int flag = 1<<channel, adc = dChannel[channel][0];
  if (((regular_flags & flag) && enable) || (!(regular_flags & flag) && !enable) || (enable && regular_num[adc] >= 8))
    return false; // Already in the requested state, or too many enabled

  // Pause ADC
  ADC_TypeDef *ADC = adc ? ADC2 : ADC1;
  DMA_Channel_TypeDef *DMA = adc ? DMA1_Channel2 : DMA1_Channel1;
  if (!adc_initialized)
    initADC();
  else if (ADC->CR & ADC_CR_ADSTART) {
    ADC->CR |= ADC_CR_ADSTP;
    while(ADC->CR & ADC_CR_ADSTP);
    DMA->CCR &= ~DMA_CCR_EN;
  }

  // Set enable flag and sample time
  if (!enable)
    regular_flags &= ~flag, regular_idx[channel] = 0;
  else {
    regular_flags |= flag;
    int ch = dChannel[channel][1];
    if(ch <= 9) ADC->SMPR1 = (ADC->SMPR1 & ~(7<<(ch*3))) | (sample_time<<(ch*3));
    else ch-=9, ADC->SMPR2 = (ADC->SMPR2 & ~(7<<(ch*3))) | (sample_time<<(ch*3));
  }

  // Rebuild sequence registers and other variables
  uint32_t SQR1 = 0, SQR2 = 0, num = 0;
  for (int i = 0; (1<<i) <= regular_flags; i++) {
    if (regular_flags & (1<<i)) {
      regular_idx[i] = num;
      if(num<4) SQR1 |= dChannel[i][1]<<(num+1)*6;
      else      SQR2 |= dChannel[i][1]<<(num-4)*6;
      num++;
    }
  }

  // Reconfigure ADC
  if (num != 0) {
    regular_num[adc] = num;
    oversamples_shift[adc] = dOversamplesShift[num-1];
    DMA->CNDTR = num << oversamples_shift[adc];
    DMA->CCR |= DMA_CCR_EN;
    ADC->SQR1 = SQR1|(num-1), ADC->SQR2 = SQR2;
    ADC->CR |= ADC_CR_ADSTART;
    if (wait_for_buffer)
      delayMicroseconds(50);
  }
  return true;
}

bool Gooser5CurrentSense::setChannelEnabledInjected(Channel channel, bool enable, bool sample_and_wait, int sample_time) {
  int flag = 1<<channel, adc = dChannel[channel][0];
  if (((injected_flags & flag) && enable) || (!(injected_flags & flag) && !enable) || (enable && injected_num[adc] >= 8))
    return false; // Already in the requested state, or too many enabled

  // Pause ADC
  ADC_TypeDef *ADC = adc ? ADC2 : ADC1;
  if (!adc_initialized)
    initADC();
  else if (ADC->CR & ADC_CR_JADSTART) {
    ADC->CR |= ADC_CR_JADSTP;
    while(ADC->CR & ADC_CR_JADSTP);
  }

  // Set enable flag and sample time
  if (!enable)
    injected_flags &= ~flag, injected_idx[channel] = 0;
  else {
    injected_flags |= flag;
    int ch = dChannel[channel][1];
    if(ch <= 9) ADC->SMPR1 = (ADC->SMPR1 & ~(7<<(ch*3))) | (sample_time<<(ch*3));
    else ch-=9, ADC->SMPR2 = (ADC->SMPR2 & ~(7<<(ch*3))) | (sample_time<<(ch*3));
  }

  // Rebuild sequence register and other variables
  uint32_t JSQR = 0, num = 0;
  for (int i = 0; (1<<i) <= injected_flags; i++) {
    if (injected_flags & (1<<i)) {
      injected_idx[i] = num;
      JSQR |= dChannel[i][1]<<(9 + num*6);
      num++;
    }
  }

  // Reconfigure ADC
  if (num != 0) {
    injected_num[adc] = num;
    ADC->JSQR = JSQR|(num-1);
    if (sample_and_wait)
      startInjectedConversions(true);
  }
  return true;
}

int Gooser5CurrentSense::getResultRegular(Channel channel) {
  if (!(regular_flags & (1<<channel)) && !setChannelEnabledRegular(channel, true, true))
      return 0;
  int sum = 0, adc = dChannel[channel][0], ovss = oversamples_shift[adc];
  volatile uint16_t *buffer = dma_buffer[adc] + regular_idx[channel];
  for (int i = 0; i < (1<<ovss); i++)
    sum += buffer[i];
  return sum >> ovss;
}

int Gooser5CurrentSense::getResultInjected(Channel channel) {
  if (!(injected_flags & (1<<channel)) && !setChannelEnabledInjected(channel, true, true))
      return 0;
  return JDR[dChannel[channel][0]][injected_idx[channel]];
}


// --- Gooser5CurrentSense member functions ---

int Gooser5CurrentSense::init(float gainA_mVpA, float gainB_mVpA) {
  if (driver==nullptr) {
    SIMPLEFOC_DEBUG("CUR: Driver not linked!");
    return 0;
  }

  // Set gains to nominal value if no calibrated values provided
  gain_a = (gainA_mVpA == NOT_SET ? volts_to_amps_ratio : (1000.0f/gainA_mVpA)) * (ADC_VOLTAGE/16384.0f);  // amps per volt x volts per adc unit = amps per adc unit
  gain_b = (gainB_mVpA == NOT_SET ? volts_to_amps_ratio : (1000.0f/gainB_mVpA)) * (ADC_VOLTAGE/16384.0f);

  setChannelEnabledRegular((Channel)(Channel::current0A+(motor_id<<1)), true);
  setChannelEnabledRegular((Channel)(Channel::current0B+(motor_id<<1)), true);
  driver->enable(); // Make sure driver is enabled
  if(driver_type==DriverType::BLDC)
    static_cast<BLDCDriver*>(driver)->setPwm(driver->voltage_limit/2, driver->voltage_limit/2, driver->voltage_limit/2);
  calibrateOffsets();
  if(driver_type==DriverType::BLDC)
      static_cast<BLDCDriver*>(driver)->setPwm(0,0,0);
  initialized = true;
  return 1; // Success
}

PhaseCurrent_s Gooser5CurrentSense::getPhaseCurrents() {
  PhaseCurrent_s current;
  current.a = (getResultRegular((Channel)(Channel::current0A+(motor_id<<1))) - offset_ia) * gain_a;
  current.b = (getResultRegular((Channel)(Channel::current0B+(motor_id<<1))) - offset_ib) * gain_b;
  current.c = -current.a - current.b;
  return current;
}

void Gooser5CurrentSense::calibrateOffsets() {
  const int calibration_rounds = 1000;

  _delay(10);
  // find adc offset = zero current voltage
  offset_ia = offset_ib = 0;
  // read the adc voltage 1000 times ( arbitrary number )
  for (int i = 0; i < calibration_rounds; i++) {
    offset_ia += getResultRegular((Channel)(Channel::current0A+(motor_id<<1)));
    offset_ib += getResultRegular((Channel)(Channel::current0B+(motor_id<<1)));
    delayMicroseconds(50);
  }
  // calculate the mean offsets
  offset_ia /= calibration_rounds; //SIMPLEFOC_DEBUG("CS: offset_ia ", offset_ia); _delay(40);
  offset_ib /= calibration_rounds; //SIMPLEFOC_DEBUG("CS: offset_ib ", offset_ib); _delay(40);
}

void Gooser5CurrentSense::calibrateGain(FOCMotor *motor) {
  int i, phase;
  float p1[2] = {0}, p2[2] = {0}, result[4][51];
  float midV = driver->voltage_limit*0.5f;

  if (!initialized) {
    SIMPLEFOC_DEBUG("CUR: init needs to be called before calibrateGain");
    return;
  }
  if (driver_type != DriverType::BLDC) // Hybrid stepper not supported yet
    return;

  // Start with nominal gain
  gain_a = volts_to_amps_ratio * (ADC_VOLTAGE/16384.0f);
  gain_b = volts_to_amps_ratio * (ADC_VOLTAGE/16384.0f);

  // Measure positive and negative current on each phase.
  // Order is -a,+a,-b,+b
  for (phase = 0; phase < 4; phase++) {
    // Ramp voltage up to voltage_sensor_align and back down to 0.
    // Only downward ramp gives accurate current, since motor moves during upward ramp.
    for (i = 0; i <= 100; i++) {
      float v = motor->voltage_sensor_align * (i<50 ? i : 100-i) / 50.0f;
      switch(phase) {
      case 0: ((BLDCDriver*)driver)->setPwm(midV+v,midV-v,midV-v); break;
      case 1: ((BLDCDriver*)driver)->setPwm(midV-v,midV+v,midV+v); break;
      case 2: ((BLDCDriver*)driver)->setPwm(midV-v,midV+v,midV-v); break;
      case 3: ((BLDCDriver*)driver)->setPwm(midV+v,midV-v,midV+v); break;
      }
      _delay(i == 49 ? 100 : 2); // Long delay just before peak current, to let motor position stabilize
      if (i >= 50) { // Record samples on ramp-down, after motor position is stabilized
        PhaseCurrent_s current = getPhaseCurrents();
        float used = result[phase][100-i] = (phase>>1) ? current.b : current.a; // result array is only used for debug output
        if(phase&1) used = -used;
        if(i >= 50 && i <= 70) p2[phase>>1] += used / 21.0f; // p2 is the averaged readings from calibration_voltage x 1.0 to 0.6
        if(i >= 70 && i <= 90) p1[phase>>1] += used / 21.0f; // p1 is the averaged readings from calibration_voltage x 0.6 to 0.2
      }
    }
  }
  ((BLDCDriver*)driver)->setPwm(0,0,0);
  float average_amplitude = (p2[0] + p2[1] - p1[0] - p1[1]) / 2;
  gain_a = volts_to_amps_ratio * average_amplitude / (p2[0] - p1[0]) * (ADC_VOLTAGE/16384.0f);
  gain_b = volts_to_amps_ratio * average_amplitude / (p2[1] - p1[1]) * (ADC_VOLTAGE/16384.0f);
  SIMPLEFOC_DEBUG("CUR: gain_a(mVpA) ", 1000.0f/(gain_a/(ADC_VOLTAGE/16384.0f))); _delay(40);
  SIMPLEFOC_DEBUG("CUR: gain_b(mVpA) ", 1000.0f/(gain_b/(ADC_VOLTAGE/16384.0f))); _delay(40);
  
#if 0 // Print out full current ramps in mA, and corresponding voltage in mV
  char string[64];
  sprintf(string, "p1a %5i, p2a %5i, p1b%5i, p2b%5i,",
    (int)(p1[0]*1000), (int)(p2[0]*1000), (int)(p1[1]*1000), (int)(p2[1]*1000));
  SIMPLEFOC_DEBUG(string); _delay(40);
  for (i = 0; i <= 50; i++) {
    float v = motor->voltage_sensor_align*_1_SQRT3 * i / 50.0f;
    sprintf(string, "%5i,%5i,%5i,%5i,//%3i",
      (int)(result[0][i]*1000), (int)(result[1][i]*1000), (int)(result[2][i]*1000), (int)(result[3][i]*1000), (int)(v*1000));
    SIMPLEFOC_DEBUG(string); _delay(40);
  }
#endif
}

#endif