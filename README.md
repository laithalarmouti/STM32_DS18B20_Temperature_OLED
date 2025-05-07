STM32 DS18B20 Temperature Monitor with OLED Display
This project demonstrates using an STM32F411RE microcontroller to read temperature data from a DS18B20 temperature sensor and display it on a SSD1306 OLED display (I2C interface).

Written in C using STM32CubeIDE HAL drivers
Uses 1-Wire protocol to communicate with the DS18B20
Temperature is displayed in °C on the OLED in real time
Custom GPIO configurations to handle DS18B20 data pin
--------------------------------
Features
Initialize I2C communication with SSD1306 OLED

Initialize GPIO for 1-Wire communication with DS18B20

Send Skip ROM (0xCC) and Read Scratchpad (0xBE) commands to DS18B20

Convert and display the temperature on OLED every 3 seconds

Uses TIM11 for accurate microsecond delays (for 1-Wire timing)

Tested On
STM32F411RE Nucleo Board
SSD1306 128x64 OLED Display (I2C)
DS18B20 temperature sensor connected to GPIOA Pin 1

How It Works
Initializes I2C1 peripheral and TIM11 timer

Sends a start pulse to DS18B20 to initialize communication

Writes 0xCC (Skip ROM) → addressing all devices

Writes 0x44 to start temperature conversion

After conversion delay, reads scratchpad (0xBE) to retrieve temperature data

Converts raw data to °C and displays on OLED

