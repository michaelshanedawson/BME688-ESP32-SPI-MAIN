/*
* BME688-ESP32-SPI-MAIN
* main.c
* Michael Dawson
* michaelshanedawson@gmail.com
*
* see README for changelog
* v0.1 - 1/8/2024
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <time.h>

/*Pin Configurations for the SPI bus*/
#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 2 //MISO Pin
#define PIN_NUM_MOSI 7 //MOSI Pin
#define PIN_NUM_CS 10 //CS (SS) Pin
#define PIN_NUM_CLK 6 //SPI CLK Pin

#define generic_gpio_pin 3

//Bit extracting voodoo
#define bit_read(value, bit) (((value) >> (bit)) & 0x01)

//BME688 Configuration Variables
#define HEATER_TARGET_TEMP 300.00 //Target temperature for the gas sensor heater, usually between 200°C and 400°C. Floating point value.
#define AMBIENT_TEMP 22.0 //General ambient room temperature, used to set the heater resistance. 

/*Global variables*/
uint8_t registerData = 0x00; //Variable to hold register data read results

/*Temperature sensor calibration data*/
double parT1 = 0x00; //Two bytes at LSB 0xE9 and MSB 0xEA, page 0
double parT2 = 0x00; //Two bytes at LSB 0x8A and MSB 0x8B, page 0
double parT3 = 0x00; //Single byte at 0x8C, page 0

/*Gas sensor calibration data*/
double parG1 = 0x00; //Single byte at 0xED, page 0
double parG2 = 0x00; //Two bytes at LSB 0xEB and MSB 0xEC, page 0
double parG3 = 0x00; //Single byte at 0xEE, page 0
int resHeatValue = 0x00; //Single signed value at 0x00, page 0
uint8_t resHeatRange = 0x00; //Heater range at 0x02 <5:4>, page 0 

/*GPIO Pulse*/
void pulse(uint8_t pin)
{
    gpio_set_level(pin, 1);
    gpio_set_level(pin, 0);
}

//Custom delay function
int delay(int numMilliseconds)
{
    clock_t startTime = clock();
    numMilliseconds = numMilliseconds * 1000;

    while (clock() < startTime + numMilliseconds)
    ;
    return 0;
}

/*Specific function used to select the SPI memory page of the BME688*/
void select_memory_page(spi_device_handle_t spi, uint8_t page)
{
    uint8_t pageValue = page << 4;
    uint8_t regData = 0x73 | 0 << 7;

    esp_err_t ret;
    /*When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired*/
    spi_device_acquire_bus(spi, portMAX_DELAY);

    /*Perform the SPI transaction to send the data.*/
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Data Length in bits
    t.tx_buffer=&regData;               //The data is the calculated delta phase
    t.user=(void*)0;                //D/C needs to be set to 0
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE  ;   //Keep CS active after data transfer
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
   
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&pageValue;               //The data is the phase data
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    /*Release SPI bus*/
    spi_device_release_bus(spi);
}

/*
* This function will perform the reading of a specified register on the BME688
* Will perform this in two steps, the first is to notify the device of which memory page we need access to.
* The second step is to read the intended register.
*/
uint8_t spi_read(spi_device_handle_t spi, uint8_t reg, uint8_t page)
{        
    /*This tells the BME 688 which memory page we want to read from*/
    select_memory_page(spi, page);   

    
    /*When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired*/
    spi_device_acquire_bus(spi, portMAX_DELAY);

    /*Perform the SPI transaction to send the control byte data.*/
    uint8_t controlByte = reg | 1 << 7;
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Data Length in bits
    t.tx_buffer=&controlByte;               //The data is the calculated delta phase
    t.user=(void*)0;                //D/C needs to be set to 0
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE  ;   //Keep CS active after data transfer
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
   
    /*Reads the data from the register and returns it*/
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;
    ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );    
    spi_device_release_bus(spi); // Release bus
    return *(int*)t.rx_data; //Performs the return of the requested register data
}

void spi_write(spi_device_handle_t spi, uint8_t reg, uint8_t data, uint8_t page)
{
    /*This tells the BME 688 which memory page we want to write to*/
    select_memory_page(spi, page);

    // When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
    spi_device_acquire_bus(spi, portMAX_DELAY);

    esp_err_t ret;
    spi_transaction_t t;
    uint8_t byteData = reg | 0 << 7;
    //This sends the control byte which is R/W and the register location to access   
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&byteData;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    //This sends the data to be written to the selected register
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&data;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    // Release the SPI bus
    spi_device_release_bus(spi);
   
            
}

void take_reading(spi_device_handle_t spi)
{
    uint8_t operationByte = spi_read(spi, 0x74, 1); //Gets the current defined register
    operationByte = operationByte | 1 << 0; //Sets the mode bit to take a reading
    spi_write(spi, 0x74, operationByte, 1); //Writes the updated register value. the mode bits reset to 0 once the reading takes place
}

void debugging(spi_device_handle_t spi)
{
    //Get the chip ID for the BME688
    //registerData = spi_read(spi, 0x50, 0);
    //printf("The chip ID is: %#x \n", registerData);

    //Get the variant ID, should return 0x01 for the BME688
    //registerData = spi_read(spi, 0x70, 0);
    //printf("The variant ID is: %#x \n", registerData);

    //Testing area for specific registers
    registerData = spi_read(spi, 0x22, 1);
    printf("The ADC MSB data is: %#x \n", registerData);
    registerData = spi_read(spi, 0x23, 1);
    printf("The ADC LSB data is: %#x \n", registerData);
    registerData = spi_read(spi, 0x24, 1);
    printf("The ADC XLSB data is: %#x \n", registerData);

    //Print out the calibration data we get from the registers
    //printf("The converted data is: %f \n", parT3);
}

void app_main(void)
{
    /*Configure GPIO pins*/
    gpio_reset_pin(generic_gpio_pin);
    gpio_set_direction(generic_gpio_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(generic_gpio_pin, 0);

    /*Initializes the SPI driver*/
    esp_err_t ret;
    spi_device_handle_t spi;

    /*Configures the SPI driver*/
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0,
    };

    /*Device specific SPI settings*/
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=20*1000*1000,           //Clock out at 20 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    };

    /*Initialize the SPI bus*/
    ret=spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    /*Attach the SPI device to the SPI bus*/
    ret=spi_bus_add_device(SPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);    

    /*
    * Begin BME688 configuration

    * ---Forced Mode---
    * We will need calibration data to perform the value calculations, this is stored in registers.
    * 
    * 
    * Set humidity oversampling value
    * Set temperature oversampling value
    * Set pressure oversampling value
    * 
    * Set the gas wait value for heating up of the plate
    * Set the heater step for the heater resistance value
    * Set the nb_conv to select the prior defined heater settings
    * Set run_gas_h to 1 to enable gas measurements
    * 
    * Trigger a single reading
    */

   /*Get the calibration data first and store it in the global variables*/
   uint8_t tempData = 0x00; //Generic temporary data store

   /*Calibration data for the temperature sensor*/   
   parT1 = spi_read(spi, 0xEA, 0) << 8 | spi_read(spi, 0xE9, 0);
   parT2 = spi_read(spi, 0x8B, 0) << 8 | spi_read(spi, 0x8A, 0);
   parT3 = spi_read(spi, 0x8C, 0);

   /*Calibration data for the gas sensor heating and measurement*/
   parG1 = spi_read(spi, 0xED, 0);   
   parG2 = spi_read(spi, 0XEC, 0) << 8 | spi_read(spi, 0XEB, 0); 
   parG3 = spi_read(spi, 0xEE, 0);
   resHeatValue = spi_read(spi, 0x00, 0);
   tempData = spi_read(spi, 0x02, 0);
   resHeatRange = bit_read(tempData, 5) << 1 | bit_read(tempData, 4);

   /*Begin configuration of the BME688 registers*/
   tempData = 0x01;
   spi_write(spi, 0x72, tempData, 1); //Configure the humidity oversampling value
   tempData = 0x00 | 0x02 << 5 | 0x05 << 2;
   spi_write(spi, 0x74, tempData, 1); //Configure the temperature and pressure oversampling values
   tempData = 0x19 | 0b01 << 6;
   spi_write(spi, 0x64, tempData, 1); //Configures the gas heating wait time. Uses a base value and a multiplier value

   //Need to calculate the heater resistance to send to the BME688, this uses some of the calibration data stored.
   double var1 = (parG1 / 16.0) + 49.0;
   double var2 = ((parG2 / 32768.0) * 0.0005) + 0.00235;
   double var3 = (parG3 / 1024);
   double var4 = var1 * (1.0 + (var2 * (double)HEATER_TARGET_TEMP));
   double var5 = var4 + (var3 * (double)AMBIENT_TEMP);
   uint8_t resHeatX = (uint8_t)(3.4 * (( var5 * (4.0 / (4.0 + (double)resHeatRange)) * (1.0/(1.0 + ((double)resHeatValue * 0.002)))) -25));
   spi_write(spi, 0x5A, resHeatX, 1); //Sends the resistance value for the gas sensor heater

   tempData = 0x00 | 1 << 5;
   spi_write(spi, 0x71, tempData, 1); //Selects the previously defined heater settings for gas measurements

   /*Here is where we will take a reading from the unit and perform the necessary calculations on it to present the values*/
   take_reading(spi); 
   delay(2000); //Give the unit some time to actually perform the reading, there should be a bit flag we can look at later

   /*Perform register data pulling and mathematics to calculate the data*/

   //Temperature reading
   double tempADC = spi_read(spi, 0x22, 1) << 12 | spi_read(spi, 0x23, 1) << 4; //This gets the raw ADC value from the BME688

   /*Performs the temperature conversion calculations on the raw ADC value*/
   double temp1 = ((tempADC / 16384.0) - (parT1 / 1024)) * parT2;
   double temp2 = (((tempADC / 131072.0) - (parT1 / 8192.0)) * ((tempADC / 131072.0) - (parT1 / 8192))) * (parT3 * 16.0);
   double tFine = temp1 + temp2;
   double tempComp = tFine / 5120.0;
   printf("Current temperature is= %.1f", tempComp);
   printf("°C \n");


   //Small bit of debug code, remove later
   //debugging(spi);

}




