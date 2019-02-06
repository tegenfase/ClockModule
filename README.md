# ClockModule

 - STM32 based clock divider and multiplier
   - EXTERNAL mode:
     - Input period is measured by TIM2's input capture on channel 1.
     - TODO: implement averaging/tap-tempo
     - TIM5's period is set equal to a subdivision of the input period.
     - Generates outputs in it's IRQ
   - INTERNAL mode:
     - TIM5's period is set manually
 - Proving ground for the timing code to be used in the ControlModule
 




