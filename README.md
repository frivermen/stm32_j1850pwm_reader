# stm32_j1850pwm_reader
STM32F030F6 reader for J1850 PWM protocol

For connect stm32 to obd2 use lm393:

 NC 1        8 3.3V
 
 NC 2        7 PB1
 
 NC 3        6 J1850-
 
GND 4        5 J1850+


Теоретически можно использовать только плюсовую линию для чтения, через резисторный делитель, но проверять я ленюсь, поэтому эта сноска на русском.
