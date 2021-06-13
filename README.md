## Tiny Reflow Controller V3

The key part of the code in this repository is based on Rocket Scream's [TinyReflowController](https://github.com/rocketscream/TinyReflowController), however this is not a fork, it has been complete rewrite to suit my own build of a solder reflow hot plate. A few libraries used for the firmware are also different from what Rocket Scream used for various reasons.

I don't use the Rocket Scream' TinyReflow Controller hardware either, instead I built an Arduino shield which hosts a 1.3" SH-1106-based OLED (which is different from Rocket Scream's SSD1306-based OLED), with an external MAX31855 breakout board, and all the necessary control circuitry soldered on a 6x8cm prototyping board. An Arduino Leonardo was used as the MCU.

A [UYue 946-1010](https://github.com/e-tinkers/TinyReflowControllerV3/blob/master/resources/UYue_946-1010.jpg) Preheater Station is reconfigured by replacing preheater's original controller board with the self-build controller. The idea of hacking a UYue Preheater was inspired by [DigiCool Things' youtube videos](https://youtu.be/ZxsIIwjR5n8).

<!-- Further detail description on how to build the solder reflow hot plate can be found at [e-tinkers.com](). -->

## Dependencies
The firmware has the dependencies of several Arduino Libraries, if you are using PlatformIO, it has been setup under the `lib_deps` directive in the `platformio.ini`.

  - [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library)
  - [SSD1306Ascii Library](https://github.com/greiman/SSD1306Ascii)
  - [Button Arduino Library](https://github.com/e-tinkers/button)
  - [MAX31855 Arduino Library](https://github.com/e-tinkers/MAX31855)

If you are using Arduino IDE, you will have to download and install the libraries via Library Manager or using git clone, or directly download from the respective github repository.

## Schematic
The interconnection of various parts is outlined in [schematic](https://github.com/e-tinkers/TinyReflowControllerV3/blob/master/resources/TinyReflowControllerV3.pdf). The Solid State Relay and Thermocouple are re-used with the parts that came with the UYue Preheater.

## Disclaimer
Dealing with high voltage is a very dangerous act! Please make sure you know what you are dealing with and have proper knowledge before hand. Your use of any information or materials on this Tiny Reflow Controller is entirely at your own risk, for which we shall not be liable.

## Licences

This Tiny Reflow Controller hardware and firmware are released under the [Creative Commons Share Alike v3.0 license](http://creativecommons.org/licenses/by-sa/3.0/). You are free to take this piece of code, use it and modify it. All we ask is attribution including the supporting libraries used in this firmware.

## Others
This firmware has also been tested on an E-Tinkers' [ATtiny3217 Dev Board](https://github.com/e-tinkers/attiny3217) running on [megaTinyCore](  https://github.com/SpenceKonde/megaTinyCore).
