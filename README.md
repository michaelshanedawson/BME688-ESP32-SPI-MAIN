## BME688-ESP32-SPI-MAIN ##
This repository contains SPI drivers and example code for the Bosch Sensortec BME688 temperature, pressure, humidity and gas sensor unit.

As of 1/8/2024 it is a work in process. This is my first venture into writing a driver for any hardware on any platform. I am a hardware engineer and I am wanting to step into the unknown of coding. This repository has been written in VS Code with the ESP IDF. If you happen to pull this repository and have some feedback on how I can improve it, please feel free to let me know. You can contact me at : michaelshanedawson@gmail.com

### CHANGELOG ###
* v0.1 1/8/2024 - Initial workable version compiled and tested.

* v0.2 1/9/2024 - Added pressure, humidity and gas plate temperature measurement conversion functionality. Need to test further to validate resulting data, may require offsets.

* v0.3 1/30/2024 - Removed the bitread macro in favor of bitmasking instead. Updated to declare prototypes early in the code. Disabled the watchdog for now. Updated to read in a while function for easier testing and development. Some values still seem off, could require a burn in. Temperature data is the only one that seems close to being proper. May require a calibration offset to deal with it.
