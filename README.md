## BME688-ESP32-SPI-MAIN ##
This repository contains SPI drivers and example code for the Bosch Sensortec BME688 temperature, pressure, humidity and gas sensor unit. The datasheet is located here : https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme688/#documents

As of 1/8/2024 it is a work in process. This is my first venture into writing a driver for any hardware on any platform. I am a hardware engineer and I am wanting to step into the unknown of coding. This repository has been written in VS Code with the ESP IDF. If you happen to pull this repository and have some feedback on how I can improve it, please feel free to let me know. You can contact me at : michaelshanedawson@gmail.com

### CHANGELOG ###
* v0.1 1/8/2024 - Initial workable version compiled and tested.

* v0.2 1/9/2024 - Added pressure, humidity and gas plate temperature measurement conversion functionality. Need to test further to validate resulting data, may require offsets.

* v0.3 1/30/2024 - Removed the bitread macro in favor of bitmasking instead. Updated to declare prototypes early in the code. Disabled the watchdog for now. Updated to read in a while function for easier testing and development. Some values still seem off, could require a burn in. Temperature data is the only one that seems close to being proper. May require a calibration offset to deal with it.

* v0.4 1/31/2024 - Added a temperature offset calculation to the code.

* v0.5 2/7/2024 - Fixed an issue polling the wrong register for the MSB for the humidity ADC data, was 0x26 and is supposed to be 0x25. Made a few updates to code readability. Verified all calibration data is being polled from correct registers. Verified that each ADC values are being polled from correct registers. Verified that all calculations match datasheet. Still looking into the system to determine if there is a calibration routine, if a burn-in period is required or offset values are necessary.
