This class acts as both ADC manager via its static members, and motor CurrentSense class via regular members.
It cannot coexist with SimpleFOC's other current sense classes or the Arduino function analogRead.

The ADCs are set up in dual injected mode (regular independent) with 42.5MHz clock and 4x oversampling. Injected 
channels are reserved for current sense (two channels are sampled by each ADC). The only ADC1 regular channel 
used for anything is the temperature sensor, which requires 5µs sample time.

ADC2 regular channels are used for the encoder pins, plus the stepstick enable pin can be used for ADC2_IN10 if desired. Enabled channels are converted continuously into a DMA buffer for increased oversampling. The size of the DMA buffer is adjusted so it overwrites within 47µs (40µs would completely overwrite within one 25kHz PWM cycle, but this allows 64x oversamplng of linear halls for better quality). When a channel result is requested, all its entries in the buffer are averaged together and scaled to 0-16383 range. Compared to higher hardware oversampling, this method averages each value over the last 47µs period rather than being random chance whether you get the most recently sampled channel or one that's a full 47µs out of date just before its new result comes in.

By default, regular channels are set to 2.5 cycles sample time which works well for linear halls and other fast but noisy sources. 12.5 cycles work better for higher resistance sources like potentiometers.
Sample time options from RM0440: 0=2.5, 1=6.5, 2=12.5, 3=24.5, 4=47.5, 5=92.5, 6=247.5, 7=640.5