# ESP32_BMP180_I2C_UART
This project is for ESP32 Lolin32 lite board or clone of it.

It is program to read data from BMP180 sensor via I2C and send it through UART.

Temperature is tested and seems to be working, but I have probably defective sensor so cant check pressure measurments.

Connections on ESP32:

19 - SCL + external 5.1kOhm pull-up

23 - SDA + external 5.1kOhm pull-up

You can read data via UART on PC. It is configured to send data via USB connector.
