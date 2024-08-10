#include "bmp180.h"
#include "main.h"
#include "string.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "math.h"
#include "esp_err.h"


// Variables
short int AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD; // Some variables to store calibration data
long int X1, X2, X3, B3, B5, B6; // More variables for calculations
unsigned long int B4, B7, UT, UP; // As above
uint8_t dataT[2];
uint8_t dataP[3];
uint8_t data_to_send[2];


// Public Functions
esp_err_t bmp180_init(void) {
    uint8_t callibrationData[22]; // Array to store callibration data
    //vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 10);

    data_to_send[0] = BMP180_CALLDATA; // Prepare data to send
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Create command line
    if(cmd == NULL) { return ESP_FAIL; }; // Check if cmd is created successfully

    // Create communication sequence and start it
    CHECK(i2c_master_start(cmd));
    CHECK(i2c_master_write_byte(cmd, BMP180_ADDRW, true));
    CHECK(i2c_master_write(cmd, data_to_send, 1, true));
    CHECK(i2c_master_start(cmd));
    CHECK(i2c_master_write_byte(cmd, BMP180_ADDRR, true));
    CHECK(i2c_master_read(cmd, callibrationData, 22, I2C_MASTER_ACK));
    CHECK(i2c_master_stop(cmd));
    esp_err_t is_okey = i2c_master_cmd_begin(0, cmd, 100 / portTICK_PERIOD_MS);
    if(is_okey == ESP_OK ) {
        UART_SEND("ESP_OK\n\r");
    } else if(is_okey == ESP_ERR_INVALID_STATE ) {
        UART_SEND("ESP_ERR_INVALID_STATE\n\r");
    } else if(is_okey == ESP_ERR_TIMEOUT ) {
        UART_SEND("ESP_ERR_TIMEOUT\n\r");
    } else if(is_okey == ESP_FAIL) {
        UART_SEND("ESP_FAIL");
    }
    i2c_cmd_link_delete(cmd); // Delete link

    AC1 = ((callibrationData[0] << 8) | callibrationData[1]);
    AC2 = ((callibrationData[2] << 8) | callibrationData[3]);
    AC3 = ((callibrationData[4] << 8) | callibrationData[5]);
    AC4 = ((callibrationData[6] << 8) | callibrationData[7]);
    AC5 = ((callibrationData[8] << 8) | callibrationData[9]);
    AC6 = ((callibrationData[10] << 8) | callibrationData[11]);
    B1 = ((callibrationData[12] << 8) | callibrationData[13]);
    B2 = ((callibrationData[14] << 8) | callibrationData[15]);
    MB = ((callibrationData[16] << 8) | callibrationData[17]);
    MC = ((callibrationData[18] << 8) | callibrationData[19]);
    MD = ((callibrationData[20] << 8) | callibrationData[21]);

    return ESP_OK;
}

float bmp180_readTemp(void)
{
    float bmpT = 0;

    // SEND command "BMP180_TEMP" to memory "BMP180_WRITE"
    // WAIT 5ms
    // SEND read command "BMP180_READ" and write response to dataT

    // Create and send command to start temperature measure
    data_to_send[0] = BMP180_MEASURE;
    data_to_send[1] = BMP180_TEMP;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(cmd == NULL) { UART_SEND("FAIL"); return ESP_FAIL; };
    CHECK(i2c_master_start(cmd));
    CHECK(i2c_master_write_byte(cmd, BMP180_ADDRW, true));
    CHECK(i2c_master_write(cmd, data_to_send, 2, true));
    CHECK(i2c_master_stop(cmd));
    CHECK(i2c_master_cmd_begin(0, cmd, 100 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    vTaskDelay( 10 / portTICK_PERIOD_MS); // Wait 5ms before asking for value as specified in datasheet

    // Create and send command to read temperature
    data_to_send[0] = BMP180_READ;
    cmd = i2c_cmd_link_create();
    if(cmd == NULL) { UART_SEND("FAIL"); return ESP_FAIL; };
    CHECK(i2c_master_start(cmd));
    CHECK(i2c_master_write_byte(cmd, BMP180_ADDRW, true));
    CHECK(i2c_master_write(cmd, data_to_send, 1, true));
    CHECK(i2c_master_start(cmd));
    CHECK(i2c_master_write_byte(cmd, BMP180_ADDRR, true));
    CHECK(i2c_master_read(cmd, dataT, 2, I2C_MASTER_ACK));
    CHECK(i2c_master_stop(cmd));
    i2c_master_cmd_begin(0, cmd, 100 / portTICK_PERIOD_MS); // Without CHECK, data from BMP180 are not comming with ACK so it will fail always
    i2c_cmd_link_delete(cmd);

	UT = (uint16_t) (dataT[0]<<8) + dataT[1];
	X1 = (UT - AC6) * (AC5 / pow(2, 15));
	X2 = MC * pow(2,11) / (X1 + MD);
	B5 = X1 + X2;
	bmpT = ((B5 + 8) / pow(2,4)) / 10.0;

	return bmpT;
}

float bmp180_readPress(void)
{
    float bmpP = 0;

    // Create and send command to start pressure measure
    data_to_send[0] = BMP180_MEASURE;
    data_to_send[1] = BMP180_PRESS;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(cmd == NULL) { UART_SEND("FAIL"); return ESP_FAIL; };
    CHECK(i2c_master_start(cmd));
    CHECK(i2c_master_write_byte(cmd, BMP180_ADDRW, true));
    CHECK(i2c_master_write(cmd, data_to_send, 2, true));
    CHECK(i2c_master_stop(cmd));
    CHECK(i2c_master_cmd_begin(0, cmd, 100 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    vTaskDelay( (BMP180_PRESS_WAIT + 10) / portTICK_PERIOD_MS); // Wait before asking for result

    // Create and send command to read pressure
    data_to_send[0] = BMP180_READ;
    cmd = i2c_cmd_link_create();
    if(cmd == NULL) { UART_SEND("FAIL"); return ESP_FAIL; };
    CHECK(i2c_master_start(cmd));
    CHECK(i2c_master_write_byte(cmd, BMP180_ADDRW, true));
    CHECK(i2c_master_write(cmd, data_to_send, 1, true));
    CHECK(i2c_master_start(cmd));
    CHECK(i2c_master_write_byte(cmd, BMP180_ADDRR, true));
    CHECK(i2c_master_read(cmd, dataP, 3, I2C_MASTER_ACK));
    CHECK(i2c_master_stop(cmd));
    i2c_master_cmd_begin(0, cmd, 100 / portTICK_PERIOD_MS); // Without CHECK, data from BMP180 are not comming with ACK so it will fail always
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    data_to_send[0] = 0xE0;
    if(cmd == NULL) { UART_SEND("FAIL"); return ESP_FAIL; };
    CHECK(i2c_master_start(cmd));
    CHECK(i2c_master_write_byte(cmd, BMP180_ADDRW, true));
    CHECK(i2c_master_write(cmd, data_to_send, 1, true));
    CHECK(i2c_master_stop(cmd));
    CHECK(i2c_master_cmd_begin(0, cmd, 100 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

	UP = ((dataP[0]<<16) + (dataP[1]<<8) + dataP[2])>>(8-BMP180_OSS);

	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6 / (pow(2,12)))) / (pow(2,11));
	X2 = AC2 * B6 / (pow(2,11));
	X3 = X1 + X2;
	B3 = (((AC1 * 4 + X3)<<BMP180_OSS) + 2) / 4; // 0?
	X1 = AC3 * B6 / pow(2,13);
	X2 = (B1 * (B6 * B6 / (pow(2,12)))) / (pow(2,16));
	X3 = ((X1 + X2) + 2) / pow(2,2);
	B4 = AC4 * (unsigned long)(X3 + 32768) / (pow(2,15));
	B7 = ((unsigned long) UP - B3) * (50000>>BMP180_OSS);
	if (B7<0x80000000)
		bmpP = ((B7*2)/B4);
	else
		bmpP = ((B7 / B4) * 2);
    X1 = (bmpP / (pow(2,8))) * (bmpP / (pow(2,8)));
    X1 = (X1 * 3038) / (pow(2,16));
    X2 = (-7357 * bmpP) / (pow(2,16));
    bmpP = bmpP + ((X1 + X2 + 3791) / (pow(2,4)));

    bmpP = (float) bmpP / 1000; // Display in hPa instead of Pa

	return bmpP;
}