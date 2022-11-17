# Exploded_Mini17
**See the warning at the bottom of this readme!**

This is an exploded version of the [Mini17](https://github.com/M17-Project/Mini17) UHF handheld tranceiver, a [TR-9](https://github.com/M17-Project/TR-9) successor. Its purpose is to allow for experimentation with the [CC1200](https://www.ti.com/product/CC1200) chip and its M17 use. In other words, it's an STM32+CC1200 evaluation board.

<img src="https://raw.githubusercontent.com/M17-Project/Exploded_Mini17/main/render.png" width="500">

## Flashing
To enter the DFU mode, keep BTN2 pressed while powering the board up.

## Hardware
The Exploded Mini17 has all the peripherals a walkie-talkie development board should have:
* GD32/STM32F405RGT6 micro with most of the unused pins available at the 2.54mm pin headers,
* RF front end built around CC1200 chip,
* 90° SMA connector,
* set of 3 buttons,
* microphone and a 1W speaker, along with amplifiers,
* USB-C connector for programming and power supply.

### Connector list

| Designator   |  Function   | Remarks |
|--------------|-------------|---------|
|`CN1`|External power supply|6..15V|
|`CN2`|External Button 1|Low-active|
|`CN3`|External Button 2|Low-active|
|`CN4`|External interrupt|Max. 3.3V|
|`CN5`|External Button 3|Low-active|
|`CN6`|External speaker output|≥8ohms, set `U6`/`U7` jumpers accordingly|
|`CN7`|External microphone input|set `U10` jumper accordingly, bias provided by `U9`|
|`H1`|GPIO port, 3 I/O lines|Max. 3.3V|
|`H2`|SWD||
|`H3`|GPIO port, 3 I/O lines|Max. 3.3V|
|`H4`|GPIO port, 11 I/O lines|Max. 3.3V|
|`H5/ANALOG`|Analog I/O port|Max. 3.3V|
|`H6`|GPIO port, 6 I/O lines|Max. 3.3V|

### Test point list
| Designator   |  Function   | Remarks |
|--------------|-------------|---------|
|`TP1`|Speaker output signal, negative||
|`TP2`|Speaker output signal, positive||
|`TP3`|Audio DAC output signal||
|`TP4`|Speaker amplifier input signal||
|`TP5`|Microphone signal|DC-biased|
|`TP6`|Amplified microphone signal|DC-biased|

## Warning!
There's an error in the PCB, at the bottom layer, just below the `U3` CC1200 chip. `VCC` is shorted with `GND`. Some copper carving is needed to isolate this and make the board work (see image below).

<img src="https://raw.githubusercontent.com/M17-Project/Exploded_Mini17/main/carve.png" width=300>
