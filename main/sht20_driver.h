/**
 * @file sht20_driver.htmlinclude [block] file-name
 * @author Bilgehan Paray (bilgehanparay@gmail.com)
 * @brief ESP32 driver code for SHT20 sensor
 * @version 0.1
 * @date 2023-03-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef SHT20_DRIVER_H_
#define SHT20_DRIVER_H_
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_types.h"

static const char *TAG = "i2c-simple-example";

// sensor configs
#define SHT20_ADDR                   0x40          /*!< Slave address of the SHT20 sensor */
#define SHT20_READ                   (SHT20_ADDR << 1) | 1
#define SHT20_WRITE                  (SHT20_ADDR << 1) | 0

// SHT20 registers
#define TRIGGER_T_MEASUREMENT_HOLD_MASTER   0xE3
#define TRIGGER_RH_MEASUREMENT_HOLD_MASTER  0xE5
#define TRIGGER_T_MEASUREMENT               0xF3
#define TRIGGER_RH_MEASUREMENT              0xF5
#define WRITE_USER_REGISTER                 0xE6
#define READ_USER_REGISTER                  0xE7
#define SOFT_RESET                          0xFE

// User byte bit definitions
#define OTP_BIT                             1
#define HEATER_BIT                          2
#define BATTERY_BIT                         6
#define RES_BIT_LOW                         0
#define RES_BIT_HIGH                        7

// useful macros
#define GET_RES(X) ((((X >> RES_BIT_HIGH) & 0x01) << 1) | (X >> RES_BIT_LOW & 0x01))
#define SET_BIT(X,Y) (X | (0x01 << Y))
#define CLEAR_BIT(X,Y) (X & ~(0x01 << Y))

typedef enum{
    DISABLE_OTP_RELOAD,
    ENABLE_OTP_RELOAD
}eOTP_t;

typedef enum{
    HEATER_ENABLED,
    HEATER_DISABLED
}eHeater_t;

typedef enum{
    BATTERY_OK,
    BATTERY_LOW
}eBatteryStatus_t;

typedef enum{
    RH12_T14,
    RH8_T12,
    RH10_T13,
    RH11_T11
}eResolution_t;

typedef struct{
    eResolution_t measurementResolution;
    eBatteryStatus_t batteryStatus;
    uint8_t reserved;
    eHeater_t heater;
    eOTP_t OTPReload;
}SHT20_User_Byte_st;

typedef struct{
    i2c_port_t i2c_num;
    int scl_pin;
    int sda_pin;
    uint32_t clock_freq;
    uint32_t timeout;
}SHT20_config_st;

// init sht20 communication
void init(SHT20_config_st *sht_config);

// set and get user settings byte
SHT20_User_Byte_st getUserByte(SHT20_config_st *sht_config);
void setUserByte(SHT20_config_st *sht_config, SHT20_User_Byte_st user_byte);
void printUserByte(SHT20_User_Byte_st user_byte);

// execute soft reset
void execSoftReset(SHT20_config_st *sht_config);

// Measurement functions uses hold master mode
float getTMeasurement(SHT20_config_st *sht_config);
float getRHMeasurement(SHT20_config_st *sht_config);

#endif