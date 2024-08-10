#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "main.h"
#include "string.h"
#include "bmp180.h"

// Config for I2C BUS
i2c_config_t esp_i2c_config;

// UART VARIABLES
StreamBufferHandle_t uartMessageBuff;
void *streamTxData;
char streamRxData[UART_MESSAGE_SIZE_BUFF];
uart_config_t uart_settings;
QueueHandle_t uart_queue;

// UART Functions
esp_err_t uartInit() {
    //uart_config_t uart_settings;
    uart_settings.baud_rate = 9600;
    uart_settings.data_bits = UART_DATA_8_BITS; // TODO: Check
    uart_settings.parity = UART_PARITY_DISABLE;
    uart_settings.stop_bits = UART_STOP_BITS_1;
    uart_settings.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_settings.source_clk = UART_SCLK_DEFAULT;

    if(uart_param_config(UART_NUM, &uart_settings) != ESP_OK) { return ESP_FAIL; }
    if(uart_set_pin(UART_NUM, UART_TX, UART_RX, UART_RTS, UART_CTS) != ESP_OK) { return ESP_FAIL; }
    if(uart_driver_install(UART_NUM, UART_BUFFER_LENGTH, UART_BUFFER_LENGTH, 10, &uart_queue, 0) != ESP_OK) { return ESP_FAIL; }
    return ESP_OK;
}

void uartSend(void *uartTX)
{
    uart_write_bytes(UART_NUM, (char *)uartTX, strlen(uartTX));
    vTaskDelete(NULL);
}

// GPIO Functions
void hearthbeatLED(void *pvParameter) {
    uint16_t pinNumber = PIN_LED;
    gpio_config_t io_led_pin;
    io_led_pin.intr_type = GPIO_INTR_DISABLE;
    io_led_pin.mode = GPIO_MODE_OUTPUT;
    io_led_pin.pin_bit_mask = (1ULL<<pinNumber);
    io_led_pin.pull_down_en = 0;
    io_led_pin.pull_up_en = 0;
    gpio_config(&io_led_pin);
    while(1) {
        gpio_set_level(pinNumber, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(pinNumber, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


// Main
void app_main() {
    char bmp180_temp_char[32];
    char bmp180_press_char[32];
    float bmp180_temp;
    float bmp180_press;

    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // Just wait
    while(uartInit() == ESP_FAIL){} // UART Init, code will stope if can't init
    UART_SEND( "UART INIT OKEY\n\r" );

    // Start of heartbeat LED
    BaseType_t Hearthbeat = xTaskCreatePinnedToCore(&hearthbeatLED, "hearthbeatLED", 2048, NULL, tskIDLE_PRIORITY, NULL, APP_CPU_NUM);
    UART_SEND( "Hearthbeat LED Started\n\r" );

    //// I2C Setup
    esp_i2c_config.mode = I2C_MODE_MASTER;
    esp_i2c_config.sda_io_num = 23;
    esp_i2c_config.sda_pullup_en = GPIO_PULLUP_DISABLE;
    esp_i2c_config.scl_io_num = 19;
    esp_i2c_config.scl_pullup_en = GPIO_PULLUP_DISABLE;
    esp_i2c_config.master.clk_speed = 100000;
    esp_i2c_config.clk_flags = 0;
    // Loading config
    if(i2c_param_config(0, &esp_i2c_config) != ESP_OK) {
        UART_SEND("ERROR");
    }
    // Installing driver
    if(i2c_driver_install(0, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        UART_SEND("ERROR");
    }
    
    // Init BMP180
    if((esp_err_t) bmp180_init() != ESP_FAIL) {
        UART_SEND( "BMP180 INIT OKEY\n\r" );
    } else {
        UART_SEND( "BMP180 INIT FAIL\n\r" );
    }

    // Check and write temperature
    while(1) {
        vTaskDelay( 500 / portTICK_PERIOD_MS );
        bmp180_temp = bmp180_readTemp();
        sprintf(bmp180_temp_char, "Temp: %2.1f\n\r", bmp180_temp);
        UART_SEND( bmp180_temp_char );

        vTaskDelay( 500 / portTICK_PERIOD_MS );
        //bmp180_press = bmp180_readPress();
        //sprintf(bmp180_press_char, "Pressure: %4.1f\n\r", bmp180_press);
        //UART_SEND( bmp180_press_char );
    }
}