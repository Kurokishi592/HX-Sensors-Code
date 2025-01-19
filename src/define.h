#ifndef _DEFINE_H_
#define _DEFINE_H_
#endif

#define READ_DELAY 5
#define DELAY_BTWN_READ 5

// MAG/BMP SPI
#define CS_MAG 10
#define CS_BMP 36
#define MOSI_P 11
#define MISO_P 12
#define CLK 13

// MPU I2C
#define MPU_SDA 18
#define MPU_SCL 19
#define MPU6500_ADDR 0x68

// BAR30
#define BAR_SDA 25
#define BAR_SCL 24

// CAN Bus
#define CAN_RX 30
#define CAN_TX 31

// Distance Sensor UARTs
// Left
#define DL_RX 0
#define DL_TX 1
// Right
#define DR_RX 7
#define DR_TX 8
// Front
#define DF_RX 16
#define DF_TX 17
// Back
#define DB_RX 15
#define DB_TX 14

// UM7 (unused)
// #define UM_RX 21
// #define UM_TX 20

// Disable switches
// #define SW_RD 33
#define SW_PRINTLN 33   // disable_println_during_run
#define SW_LD 34
#define SW_BD 35
#define SW_FD 36
#define SW_BAR 37
#define SW_BMP 38
#define SW_MAG 39
#define SW_MPU 40

// Indicator LED
#define IND_1 21
#define IND_2 2 
#define FAST 200
#define SLOW 400

// INA219 Power Monitoring Board
#define INA_ADDR 0x40   // 1000000 in binary is 0x40 in hex
#define INA_SHUNT 0.1

// Software Reset Pin
#define RESET 41

// IMU Raw -> Roll/Pitch
#define sqr(x) x * x
#define hypotenuse(x, y) sqrt(sqr(x) + sqr(y))

// #define PRINT