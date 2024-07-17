Allmost messages have current format:

```> PP TT SS DD .. DD CC  FF <```

Where:

PP - priority byte(see sae j1850 pwm)

TT - target address byte

SS - source address byte

DD .. DD - data bytes

CC - CRC8 byte(see ```crc8.c```)

FF - one IFR byte(i thing it use for response acknowledge)

Bellow some messages, which i collected from my ford focus mk1. First two ECU send every time, if engine is started and if it stopped(only if ignition turned on).
```
> 81 1B 10 25 00 00 00 00 00 81  60 <  // if engine off
> 81 1B 10 25 13 19 FF 3D 00 81  60 <  // if started
```
81 - priority

1B - target address(i think this is transmission)

10 - source address(ECU)

25 - ???

13,19 - RPM (0x1319 / 0x04 = 1222 rpm)

FF,3D - Short Fuel Trim (16 bit signed int, 0x0000 is 0, 0xFFFF is -1 etc.)

00 - Throttle position (n / 2 = real position in percent)

81 - CRC (how to calc it see in ```crc8.c```)

60 - IFR
```
> A1 29 10 02 0D 46 < // without IFR and crc8
```
A1 - header

29 - target(i don't know that is it)

10 - source(ECU)

02 - ???

0D,46 - VSS (0x0D46 / 0x80 = 71 km/h)

C2 - CRC
```
> A1 7B 10 02 00 < // without IFR and crc8
```
A1 - header

7B - target(i don't know that is it)

10 - source(ECU)

02 - ???

0D - some counter, that increment if VSS is changing(i think if speed up for 1km/h it increment for 1, but need to check this)

C2 - CRC


This data i collected throw Forscan connected via ELM327(i remove CRC and IFR bytes)
```
> C4 10 F5 22 11 41 < - request
> C4 F5 10 62 11 41 01 BF < - answer
```
C4 - priority

F5 - target(ELM327 Forscan)

10 - source(ECU)

22(62), 11, 41 - command for ignition pulse duration

01,BF - ign pulse in ms*10^2(0x1BF = 447 => 4.47ms)
