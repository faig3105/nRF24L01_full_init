# nRF24L01_full_init
nRF24L01 driver optimized for stm32 MCU's with FULL INITIALIZATION CODE, which is crucial for these radio modules. 
All existing registers (even if you don't need to change them) are being initialized on every startup and every time you push reset button on the microcontroller, otherwise they may be PRETTY unstable.

Original driver is written for AVR MCU's and is's source code was taken from here "http://aterlux.ru/article/nrf24l01p". Huge thanks to it's author!!!

I print confirmation text over UART first when board's init is complete and second when radio module's init is complete. If something is wrong with module init there will be just "Board init OK".

For stm32 devices there's no need in base module for nRF24L01. So you have to connecr nRF24L01 directly to the board. 
And no additional capacitors are needed if you use this init code.

Code is not complete there'slot's  work todo. For example with IRQ pin, it should be used through callback function.





/*b There's "void on_packet(uint8_t * buf, uint8_t size)" fuction in "main.c" which is called when data packet is received. Use it to parse the received message (add there your own code) or to print it for debugging./*b
