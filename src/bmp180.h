#include "driver/gpio.h"



// Function Prototypes
esp_err_t bmp180_init(void);
float bmp180_readTemp(void);
float bmp180_readPress(void);

#define CHECK( cmd ) \
    if( cmd != ESP_OK) { UART_SEND("CHECK FAIL"); return ESP_FAIL; }

// I2C SETTINGS
#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT 1000
#endif

// BMP180 SETTINGS
#define BMP180_ADDRW 0xEE
#define BMP180_ADDRR 0xEF
#define BMP180_CALLDATA 0xAA // Address to read calibration data, return 22 bytes
#define BMP180_MEASURE 0xF4 // Address to write a measure command
#define BMP180_TEMP 0x2E // Measure command for Temperature
#define BMP180_READ 0xF6 // Address to read data from measurments

#define BMP180_OSS_1 // Here we can select oversampling mode for pressure measurement
#ifdef BMP180_OSS_0
#define BMP180_PRESS 0x34 // Measure command for Pressure, different for every OSS setting
#define BMP180_PRESS_WAIT 5 // Time in [ms] that uC need to wait before asking for reasult
#define BMP180_OSS 0 // It is needed for algorithm
#endif
#ifdef BMP180_OSS_1
#define BMP180_PRESS 0x74
#define BMP180_PRESS_WAIT 8
#define BMP180_OSS 1
#endif
#ifdef BMP180_OSS_2
#define BMP180_PRESS 0xB4
#define BMP180_PRESS_WAIT 14
#define BMP180_OSS 2
#endif
#ifdef BMP180_OSS_3
#define BMP180_PRESS 0xF4
#define BMP180_PRESS_WAIT 26
#define BMP180_OSS 3
#endif