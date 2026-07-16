This class acts as both ADC manager via its static members, and motor CurrentSense class via regular members.
It cannot coexist with SimpleFOC's other current sense classes or the Arduino function analogRead.

The ADCs are set up in independent mode with 42.5MHz clock and 4x oversampling. Channels can be enabled for regular 
mode which converts continuously into a DMA buffer for increased oversampling (max 9 channels per ADC), or injected 
mode (max 4 channels per ADC) which converts each time startInjectedConversions is called (typically once per loopFOC).

Generally, regular mode should be used for noisy but low resistance sources like the current sensors and LinearHall,
and injected mode used for high resistance sources like the bus voltage divider and potentiometers which need
longer sample time and would significantly reduce the samples available for current sense if done in regular mode.
By default, regular channels are set to 2.5 cycles sample time, and injected channels are 12.5 cycles.
Sample time options from RM0440: 0=2.5, 1=6.5, 2=12.5, 3=24.5, 4=47.5, 5=92.5, 6=247.5, 7=640.5

The size of the DMA buffer for regular channels is adjusted so it overwrites within 47µs (40µs would completely 
overwrite within one 25kHz PWM cycle, but this allows up to 128x oversamplng of current sensors for better quality). 
When a channel result is requested, all its entries in the buffer are averaged together and scaled to 0-16383 range. 
Compared to higher hardware oversampling, this method averages each value over the last 47µs period rather than 
being random chance whether you get the most recently sampled channel or one that's a full 47µs out of date just 
before its new result comes in. Time taken by injected channels is not accounted for in the DMA buffer size 
calculation, so it can take longer than 47µs to fully cycle, but the effect is usually negligible.

Caution: ADC1 and ADC2 can't read the same channel number at the same time. Channels encoder0A and encoder0C are 
the only place this is a concern. If you must use both, enable Channel::dummy1 so it will read simultaneously with 
encoder0A. If using regular mode, ensure that both conversion sequences are the same length with the same sample 
times on each channel so they remain in sync.

Note: The internal temperature sensor is not enabled. It's not very useful due to distance from the mosfets, and 
needs >5µs sample time so it would need ADC1 configured for no oversampling.