# nRF24L01_full_init
nRF24L01 driver for optimized stm32 MCU's with full initialization code, which is crucial for these modules. Otherwise they may be pretty unstable.
Original driver is written for AVR MCU's and is's source code was taken from here "http://aterlux.ru/article/nrf24l01p". Huge thanks to it's author!!!

I print text first when board's init is complete and second when radio module's init is complete. If something is wrong with module init you will see just "Board init OK"

For stm32 devices there's no need in base module for nRF24L01. So you have to connecr nRF24L01 directly to the board. 
And no additional capacitors are needed if you use this init code.

Code is not complete there'slot's  work todo. For example with IRQ pin, it should be used through callback function.
