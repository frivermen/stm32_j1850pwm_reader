# stm32_j1850pwm_reader
STM32F030F6 reader for J1850 PWM protocol

For connect stm32 to obd2 use lm393:

```
 NC 1        8 3.3V
 
 NC 2        7 PB1
 
 NC 3        6 J1850-
 
GND 4        5 J1850+
```

A similar result can be obtained if you use EML327:
```
    ATZ - reset OBD II UART
    ATL1 - set Line Feed Mode
    ATH1 - set message headers ON
    ATS1 - set spaces between bytes to ON
    ATAL - allow messages that are longer than 7 bytes

    ATSP1 - set protocol to SAE J1850 PWM (41.6 kbaud)

    ATMA - all messages
```

Теоретически можно использовать только плюсовую линию для чтения, через резисторный делитель, но проверять я ленюсь, поэтому эта сноска на русском.
