// Public Functions


// I2C SETTINGS
#define I2C_TIMEOUT 1000

// UART SETTINGS
#define UART_MESSAGE_SIZE_BUFF 64 // Size of buffer for xStream in RTOS
#define UART_NUM UART_NUM_0 // Set what UART port to use
#define UART_TX 1 // U2 TXD: 17, U0 TXD: 1
#define UART_RX 3 // U2 RXD: 16, U0 RXD: 3
#define UART_RTS UART_PIN_NO_CHANGE
#define UART_CTS UART_PIN_NO_CHANGE
#define UART_BUFFER_LENGTH (1024 * 2)
#define UART_SEND( message ) \
    uart_write_bytes(UART_NUM, (char *)message, strlen(message))


// NORMAL GPIO SETTINGS
#define PIN_LED 22 // GPIO22