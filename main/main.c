/*
* BME688-ESP32-SPI-MAIN
* main.c
* Michael Dawson
* michaelshanedawson@gmail.com
*
* SPI Driver and example code for the Bosch Sensortec BME688 temperature, humidity, pressure and gas sensor unit. This example uses the forced mode for measurements.
* Datasheet can be found here : https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme688/#documents
*
* see README for changelog
* v0.3
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/*Pin Configurations for the SPI bus*/
#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 2 //MISO Pin
#define PIN_NUM_MOSI 7 //MOSI Pin
#define PIN_NUM_CS 10 //CS (SS) Pin
#define PIN_NUM_CLK 6 //SPI CLK Pin

#define generic_gpio_pin 3

//BME688 Configuration Variables
#define HEATER_TARGET_TEMP 300.00 //Target temperature for the gas sensor heater, usually between 200°C and 400°C. Floating point value.
#define AMBIENT_TEMP 22.0 //General ambient room temperature, used to set the heater resistance. 

/*Global variables*/
uint8_t registerData = 0x00; //Variable to hold register data read results
double currentTemperature = 0.0;

/*Declare prototypes*/
void pulse(uint8_t pin);
void debugging(spi_device_handle_t spi);
void select_memory_page(spi_device_handle_t spi, uint8_t page);
uint8_t spi_read(spi_device_handle_t spi, uint8_t reg, uint8_t page);
void spi_write(spi_device_handle_t spi, uint8_t reg, uint8_t data, uint8_t page);
void take_reading(spi_device_handle_t spi);
double get_temperature();

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

/*Pressure sensor calibration data*/
double parP1 = 0x00; //Two bytes at LSB 0x8E and MSB 0x8F, page 0
double parP2 = 0x00; //Two bytes at LSB 0x90 and MSB 0x91, page 0
double parP3 = 0x00; //Single byte at 0x92, page 0
double parP4 = 0x00; //Two bytes at LSB 0x94 and MSB 0x95, page 0
double parP5 = 0x00; //Two bytes at LSB 0x96 and MSB 0x97, page 0
double parP6 = 0x00; //Single byte at 0x99, page 0
double parP7 = 0x00; //Single byte at 0x98, page 0
double parP8 = 0x00; //Two bytes at LSB 0x9C and MSB 0x9D, page 0
double parP9 = 0x00; //Two bytes at LSB 0x9E and MSB 0x9F, page 0
double parP10 = 0x00; //Single byte at 0xA0, page 0

/*Humidity sensor calibration data*/
double parH1 = 0x00; //Two bytes at LSB 0xE2<3:0> and MSB 0xE3, page 0
double parH2 = 0x00; //Two bytes at LSB 0xE2<7:4> and MSB 0xE1, page 0
double parH3 = 0x00; //Single byte at 0xE4, page 0
double parH4 = 0x00; //Single byte at 0xE5, page 0
double parH5 = 0x00; //Single byte at 0xE6, page 0
double parH6 = 0x00; //Single byte at 0xE7, page 0
double parH7 = 0x00; //Single byte at 0xE8, page 0


/*GPIO Pulse*/
void pulse(uint8_t pin)
{
    gpio_set_level(pin, 1);
    gpio_set_level(pin, 0);
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
    operationByte = operationByte | 1 << 0; //Sets the mode bit to take a reading. We don't want to change anything else.
    spi_write(spi, 0x74, operationByte, 1); //Writes the updated register value. the mode bits reset to 0 once the reading takes place

    vTaskDelay(2000 / portTICK_PERIOD_MS);  //Delay the task for x number of milliseconds. 1000mS = 1 second. Give the unit some time to actually perform the reading, there should be a bit flag we can look at later

   /*Perform register data pulling and mathematics to calculate the data*/

   /*Temperature reading*/
   double tempADC = spi_read(spi, 0x22, 1) << 12 | spi_read(spi, 0x23, 1) << 4; //This gets the raw temperature ADC value from the BME688

   /*Performs the temperature conversion calculations on the raw ADC value for floating point result, result is in Celsius*/
   double temp1 = ((tempADC / 16384.0) - (parT1 / 1024)) * parT2;
   double temp2 = (((tempADC / 131072.0) - (parT1 / 8192.0)) * ((tempADC / 131072.0) - (parT1 / 8192))) * (parT3 * 16.0);
   double tFine = temp1 + temp2;
   double tempComp = tFine / 5120.0;
   printf("Current temperature is= %.1f", tempComp);
   printf("°C \n");

   /*Pressure reading*/
   double pressureADC = spi_read(spi, 0x1F, 1) << 12 | spi_read(spi, 0x20, 1) << 4; //This gets the raw pressure ADC value

   /*Performs the pressure conversion calculation on the raw ADC value for floating point result, result is in Pascal.
   * Please note, the datasheet for the BME688 has some of the formula variables wrong. There are entries for var1_p, var2_p, var3_p.
   * These should just be var1, var2 and var3, confirmed to past product datasheet such as the BME680 which uses the same formulas to calculate with.
   * To make things easier, added conversion to inHg prior to printing. May add an offset for calibration later, needs further testing to see if
   * sensor requires proper burn-in to function properly.
   */
   double pressure1 = (tFine / 2.0) - 64000.0;
   double pressure2 = pressure1 * pressure1 * (parP6 / 131072.0);
   pressure2 = pressure2 + (pressure1 * parP5 * 2.0);
   pressure2 = (pressure2 / 4.0) + (parP4 * 65536.0);
   pressure1 = (((parP3 * pressure1 * pressure1) / 16384.0) + (parP2 * pressure1)) / 524288.0;
   pressure1 = (1.0 + (pressure1 / 32768.0)) * parP1;
   double pressureComp = 1048576.0 - pressureADC;
   pressureComp = ((pressureComp - (pressure2 / 4096.0)) * 6250.0) / pressure1;
   pressure1 = (parP9 * pressureComp * pressureComp) / 2147483648.0;
   pressure2 = pressureComp * (parP8 / 32768.0);
   double pressure3 = (pressureComp / 256.0) * (pressureComp / 256.0) * (pressureComp / 256.0) * (parP10 / 131072.0);
   pressureComp = pressureComp + (pressure1 + pressure2 + pressure3 + (parP7 * 128.0)) / 16.0;

   pressureComp = pressureComp * 0.000295;
   printf("Current pressure is= %.1f", pressureComp);
   printf("inHg \n");

   /*Humidity reading*/
   double humidityADC = spi_read(spi, 0x26, 1) << 8 | spi_read(spi, 0x26, 1); //This gets the raw humidity ADC value

   /*Performs the humidity conversion calculation on the raw ADC value for the floating point result.*/
   double humidity1 = humidityADC - ((parH1 * 16.0) + ((parH3 / 2.0) * tempComp));
   double humidity2 = humidity1 * ((parH2 / 262144.0) * (1.0 + ((parH4 / 16384.0)* tempComp) + ((parH5 / 1048576.0) * tempComp * tempComp)));
   double humidity3 = parH6 / 16384.0;
   double humidity4 = parH7 / 2097152.0;
   double humidityComp = humidity2 + ((humidity3 + (humidity4 * tempComp)) * humidity2 * humidity2);

   printf("Current humidity is= %.1f", humidityComp);
   printf("%% \n");

   /*Gas resistance reading*/
   uint8_t tempData = spi_read(spi, 0x2D, 1);
   uint32_t gasRange = (tempData & 0x0F);
   int32_t gasADC = spi_read(spi, 0x2C, 1) << 2 | (tempData & 0xC0) >> 1;

   uint32_t gas1 = UINT32_C(262144) >> gasRange;
   int32_t gas2 = gasADC - INT32_C(512);
   gas2 *= INT32_C(3);
   gas2 = INT32_C(4096) + gas2;
   double gasResistance = 1000000.0f * (float)gas1 / (float)gas2;

   printf("Gas plate resistance is= %.1f", gasResistance);
   printf("Ω \n");
   printf("------------------------------------ \n");
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
    registerData = spi_read(spi, 0x2C, 1);
    printf("The 0x2C is: %#x \n", registerData);
    registerData = spi_read(spi, 0x2D, 1);
    printf("The 0x2D is: %#x \n", registerData);
    

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
   resHeatRange = (tempData & 0x3);

   /*Calibration data for the pressure sensor*/
   parP1 = spi_read(spi, 0x8F, 0) << 8 | spi_read(spi, 0x8E, 0);
   parP2 = spi_read(spi, 0x91, 0) << 8 | spi_read(spi, 0x90, 0);
   parP3 = spi_read(spi, 0x92, 0);
   parP4 = spi_read(spi, 0x95, 0) << 8 | spi_read(spi, 0x94, 0);
   parP5 = spi_read(spi, 0x97, 0) << 8 | spi_read(spi, 0x96, 0);
   parP6 = spi_read(spi, 0x99, 0);
   parP7 = spi_read(spi, 0x98, 0);
   parP8 = spi_read(spi, 0x9D, 0) << 8 | spi_read(spi, 0x9C, 0);
   parP9 = spi_read(spi, 0x9F, 0) << 8 | spi_read(spi, 0x9E, 0);
   parP10 = spi_read(spi, 0xA0, 0);

   /*Calibration data for the humidity sensor*/
   tempData = spi_read(spi, 0xE2, 0);
   parH1 = spi_read(spi, 0xE3, 0) << 4 | (tempData & 0x0F);
   parH2 = spi_read(spi, 0xE1, 0) << 4 | (tempData & 0xF0);
   parH3 = spi_read(spi, 0xE4, 0);
   parH4 = spi_read(spi, 0xE5, 0);
   parH5 = spi_read(spi, 0xE6, 0);
   parH6 = spi_read(spi, 0xE7, 0);
   parH7 = spi_read(spi, 0xE8, 0);

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

   //Small bit of debug code, remove later
   //debugging(spi);

   while(1)
   {
        /*Here is where we will take a reading from the unit and perform the necessary calculations on it to present the values*/
        take_reading(spi); 
   }
}




