#pragma once

//SPI0 (default for 18, 16, 19, 17)
#define PIN_IMU_SCK 6
#define PIN_IMU_TX 7
#define PIN_IMU_RX 4
#define PIN_IMU_CS 5

//27 can be used for other purposes if read from bms
#define PIN_ANALOG_BATTERY_VOLTAGE 27
#define PIN_ANALOG_CHARGE_VOLTAGE 26
//28 can be used for other purposes if read from bms
#define PIN_ANALOG_CHARGE_CURRENT 28

#define PIN_ESC_ENABLE 20
#define PIN_RASPI_POWER 21
#define PIN_ENABLE_CHARGE 22

#define PIN_EMERGENCY_1 18
#define PIN_EMERGENCY_2 19
#define PIN_EMERGENCY_3 3
#define PIN_EMERGENCY_4 2

#define PIN_MUX_IN 11
#define PIN_MUX_OUT 12
#define PIN_MUX_ADDRESS_0 13
#define PIN_MUX_ADDRESS_1 14
#define PIN_MUX_ADDRESS_2 15

// Pico pins used for communication
// PIN1, GP0 - UART0 TX, I2C0 SDA
// PIN2, GP1 - UART0 RX, I2C0 SCL/SCK
// PIN11, GP8 - UART1 TX, I2C0 SDA
// PIN12, GP9 - UART1 RX, I2C0 SCL/SCK
// PIN21, GP16 - UART0 TX, I2C0 SDA
// PIN22, GP17 - UART0 RX, I2C0 SCL/SCK
//
//default pins configured in ...\packages\framework-arduinopico\variants\rpipico\pins_arduino.h
//
//#define PIN_SERIAL1_TX (0u)
//#define PIN_SERIAL1_RX (1u)
//
//#define PIN_SERIAL2_TX (8u)
//#define PIN_SERIAL2_RX (9u)
//
//#define PIN_WIRE0_SDA  (4u)
//#define PIN_WIRE0_SCL  (5u)
//
//#define PIN_WIRE1_SDA  (26u)
//#define PIN_WIRE1_SCL  (27u)
//
//#define PIN_SPI0_MISO  (16u)
//#define PIN_SPI0_MOSI  (19u)
//#define PIN_SPI0_SCK   (18u)
//#define PIN_SPI0_SS    (17u)

//I2C0 (default for i2c0 4,5)
#define PIN_DISPLAY_SDA 16
#define PIN_DISPLAY_SCK 17

//UART0 (default for uart0 0,1)
#define PIN_UPLINK_TX 0
#define PIN_UPLINK_RX 1

//UART1 (default for uart1 8,9)
#define PIN_BMS_TX 8
#define PIN_BMS_RX 9

//currently unused. enable pi power?
#define PIN_UNUSED 10
//#define PIN_DISPLAY_RESET 10

