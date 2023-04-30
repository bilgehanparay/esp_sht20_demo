/*
* Demo app reading sensor info 
* from sht20 sensor via I2C
* based on esp-idf i2c_simple_main.c
*/
/**
 * Sensor Connections:
 * ---------------------
 * Sensor  |  ESP |
 * blue    |  19  | CLK
 * yellow  |  18  | SDA   
 * black   |  GND | GND
 * brown   |  3V3 | VCC 
 * ---------------------
*/
#include "esp_log.h"
#include "sht20_driver.h"

// I2C configs
#define SHT20_I2C_MASTER_SCL_IO           19                     /*!< GPIO number used for I2C master clock */
#define SHT20_I2C_MASTER_SDA_IO           18                     /*!< GPIO number used for I2C master data  */
#define SHT20_I2C_MASTER_NUM              I2C_NUM_0              /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define SHT20_I2C_MASTER_FREQ_HZ          100000                 /*!< I2C master clock frequency */
#define SHT20_I2C_MASTER_TX_BUF_DISABLE   0                      /*!< I2C master doesn't need buffer */
#define SHT20_I2C_MASTER_RX_BUF_DISABLE   0                      /*!< I2C master doesn't need buffer */
#define SHT20_I2C_MASTER_TIMEOUT_MS       1000

SHT20_config_st sht_config;

void task1(void *params){
    while(true){
        SHT20_User_Byte_st user_byte = getUserByte(&sht_config);
        printUserByte(user_byte);
        printf("\n");
        float t = getTMeasurement(&sht_config);
        //float rh = getRHMeasurement(&sht_config);
        printf("Temp: %f C\n", t);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    sht_config.i2c_num      = SHT20_I2C_MASTER_NUM;
    sht_config.scl_pin      = SHT20_I2C_MASTER_SCL_IO;
    sht_config.sda_pin      = SHT20_I2C_MASTER_SDA_IO;
    sht_config.clock_freq   = SHT20_I2C_MASTER_FREQ_HZ;
    sht_config.timeout      = SHT20_I2C_MASTER_TIMEOUT_MS;
    init(&sht_config);
    xTaskCreate(&task1, "user byte read", 2048, "task 1", 1, NULL);
}
//