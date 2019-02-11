# ClockModule

 - STM32 based clock divider and multiplier
   - EXTERNAL mode:
     - Input period is measured by TIM2's input capture on channel 1. (PA0)
     - Several periods are averaged. If a period measurement is out of bounds the routine starts over
     - TIM5's period is set equal to a subdivision of the input period.
     - TIM5 generates subTicks and sets outputs accordingly in it's IRQ.
   - INTERNAL mode:
     - TIM5's period is set manually
     - TIM2 is used in OC mode on channel 2 for the masterTick();
 - Proving ground for the timing code to be used in the ControlModule
 
## TODO:

- Implement averaging/tap-tempo (DONE!)
- Have multiple subdivision outputs (MOSTLY WORKING)
- MIDI (over USB) would be really useful
- Design user interface
- Hardware
 - PCB
 - Use "blue pill" development board?
 - FRONT PANEL
- Convert to STM32F0/F1 (Currently developing on the STM32F4)



