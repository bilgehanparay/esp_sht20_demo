#include "sht20_driver.h"

void init(SHT20_config_st *sht_config){
    // i2c config
     i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sht_config->sda_pin,
        .scl_io_num = sht_config->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = sht_config->clock_freq
    };
    i2c_param_config(sht_config->i2c_num, &i2c_config);
    i2c_driver_install(sht_config->i2c_num, 
                       I2C_MODE_MASTER,
                       0,
                       0,
                       0);
}

SHT20_User_Byte_st getUserByte(SHT20_config_st *sht_config){
    
    uint8_t raw = 0;
    SHT20_User_Byte_st user_byte;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, SHT20_WRITE, true);
    i2c_master_write_byte(cmd_handle, READ_USER_REGISTER, true);

    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, SHT20_READ, true);
    i2c_master_read(cmd_handle, &raw, 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd_handle);
    
    i2c_master_cmd_begin(sht_config->i2c_num, cmd_handle, sht_config->timeout/ portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);

    // TODO: handle incorrect input
    user_byte.OTPReload         = ((raw >> OTP_BIT) & 0x01) == 1 ?DISABLE_OTP_RELOAD:ENABLE_OTP_RELOAD;
    user_byte.heater            = ((raw >> HEATER_BIT) & 0x01) == 1 ?HEATER_ENABLED:HEATER_DISABLED;
    user_byte.reserved          = (raw >> 3)  & 0x03;
    user_byte.batteryStatus     = ((raw >> BATTERY_BIT) & 0x01) == 1 ?BATTERY_LOW:BATTERY_OK;
    uint8_t res = GET_RES(raw);
    switch(res){
        case 0:
            user_byte.measurementResolution = RH12_T14;
        break;

        case 1:
            user_byte.measurementResolution = RH8_T12;
        break;

        case 2:
            user_byte.measurementResolution = RH10_T13;
        break;

        case 3:
            user_byte.measurementResolution = RH11_T11;
        break;
        default:
            printf("error\n");
        break;
    }
    return user_byte;
};

void setUserByte(SHT20_config_st *sht_config, SHT20_User_Byte_st user_byte){
    uint8_t raw = 0;
    // set otp disable bit
    if(user_byte.OTPReload == DISABLE_OTP_RELOAD){
        raw = SET_BIT(raw, OTP_BIT);
    }else if(user_byte.OTPReload == ENABLE_OTP_RELOAD){
        raw = CLEAR_BIT(raw, OTP_BIT);
    }
    // set heater bit
    if(user_byte.heater == HEATER_DISABLED){
        raw = CLEAR_BIT(raw, HEATER_BIT);
    }else if(user_byte.heater == HEATER_ENABLED){
        raw = SET_BIT(raw, HEATER_BIT);
    }
    // battery stat can't be set
    // set resolution bits
    switch(user_byte.measurementResolution){
        case RH12_T14:
            raw = CLEAR_BIT(raw, RES_BIT_LOW);
            raw = CLEAR_BIT(raw, RES_BIT_HIGH);
        break;

        case RH8_T12:
            raw = SET_BIT(raw, RES_BIT_LOW);
            raw = CLEAR_BIT(raw, RES_BIT_HIGH);
        break;

        case RH10_T13:
            raw = CLEAR_BIT(raw, RES_BIT_LOW);
            raw = SET_BIT(raw, RES_BIT_HIGH);
        break;

        case RH11_T11:
            raw = SET_BIT(raw, RES_BIT_LOW);
            raw = SET_BIT(raw, RES_BIT_HIGH);
        break;
        
        default:
            // set default res to RH12_T14
            raw = CLEAR_BIT(raw, RES_BIT_LOW);
            raw = CLEAR_BIT(raw, RES_BIT_HIGH);
        break;
    }
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, SHT20_WRITE, true);
    i2c_master_write_byte(cmd_handle, WRITE_USER_REGISTER, true);
    i2c_master_write_byte(cmd_handle, raw, true);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(sht_config->i2c_num, cmd_handle, sht_config->timeout/ portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
}

void printUserByte(SHT20_User_Byte_st user_byte){
    char *otp;
    switch(user_byte.OTPReload){
        case DISABLE_OTP_RELOAD:
            otp = "DISABLE_OTP_RELOAD";
        break;
        case ENABLE_OTP_RELOAD:
            otp = "ENABLE_OTP_RELOAD";
        break;
        default:
            otp = "UNKNOWN";
        break;
    }
    char *heat;
    switch(user_byte.heater){
        case HEATER_ENABLED:
            heat = "HEATER_ENABLED";
        break;

        case HEATER_DISABLED:
            heat = "HEATER_DISABLED";
        break;

        default:
            heat = "UNKNOWN";
        break;
    }
    char *battStat;
    switch(user_byte.batteryStatus){
        case BATTERY_OK:
            battStat = "BATTERY_OK";
        break;
        
        case BATTERY_LOW:
            battStat = "BATTERY_LOW";
        break;

        default:
            battStat = "UNKNOWN";
        break;
    }

    char *reso;
    switch(user_byte.measurementResolution){
        case RH12_T14:
            reso = "RH12_T14";
        break;
        case RH8_T12:
            reso = "RH8_T12";
        break;
        case RH10_T13:
            reso = "RH10_T13";
        break;
        case RH11_T11:
            reso = "RH11_T11";
        break;
        default:
            reso = "UNKNOWN";
        break;
    }
    printf("OTP: %s HEATER: %s BATT_STAT: %s RESOLUTION: %s\n", otp, heat, battStat, reso);
}

// execute soft reset
void execSoftReset(SHT20_config_st *sht_config){
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, SHT20_WRITE, true);
    i2c_master_write_byte(cmd_handle, SOFT_RESET, true);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(sht_config->i2c_num, 
                         cmd_handle, 
                         sht_config->timeout/ portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
}
// TODO: 
// Measurment functions wont work: check the link!
// https://esp32.com/viewtopic.php?f=2&t=8715
float getTMeasurement(SHT20_config_st *sht_config){
    float t = 0;
    uint8_t byte0 = 0;
    uint8_t byte1 = 0;
    uint8_t byte2 = 0;
    i2c_set_timeout(sht_config->i2c_num, 1048575);
    // Issue temp read command with no hold mode
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd_handle));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd_handle, SHT20_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd_handle, TRIGGER_T_MEASUREMENT, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd_handle));
    esp_err_t comm_ok = i2c_master_cmd_begin(sht_config->i2c_num, 
                                             cmd_handle, 
                                             sht_config->timeout/ portTICK_PERIOD_MS);
    // TODO: is this necessary?
    //i2c_cmd_link_delete(cmd_handle);
    if(comm_ok == ESP_OK){
        printf("t measurment, write OK\n");
    }else{
        printf("t measurement, command write fail, err: %s\n", esp_err_to_name(comm_ok));
    }
    // wait for 100 ms for the sensor to complete measurement
    vTaskDelay(100/portTICK_PERIOD_MS);
    // Poll the measurement
    int i = 0;
    do{
        //i2c_cmd_handle_t cmd_handle_ =  i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd_handle));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd_handle, SHT20_READ, true));
        ESP_ERROR_CHECK(i2c_master_read(cmd_handle, &byte0, 1, I2C_MASTER_ACK));
        ESP_ERROR_CHECK(i2c_master_read(cmd_handle, &byte1, 1, I2C_MASTER_ACK));
        ESP_ERROR_CHECK(i2c_master_read(cmd_handle, &byte2, 1, I2C_MASTER_NACK));
        (i2c_master_cmd_begin(sht_config->i2c_num, 
                                cmd_handle,
                                sht_config->timeout/portTICK_PERIOD_MS));
        //i2c_cmd_link_delete(cmd_handle_);
        vTaskDelay(10/portTICK_PERIOD_MS);
        if(comm_ok == ESP_OK){
            printf("read was OK, sensor returned ACK\n");
            break;
        }else if(comm_ok == ESP_FAIL || comm_ok == ESP_ERR_TIMEOUT){
            printf("sensor did not returned ACK\n");
            //cmd_handle_ = i2c_cmd_link_create();
            ESP_ERROR_CHECK(i2c_master_stop(cmd_handle));
            ESP_ERROR_CHECK(i2c_master_cmd_begin(sht_config->i2c_num, 
                                       cmd_handle,
                                       sht_config->timeout/portTICK_PERIOD_MS));
            //i2c_cmd_link_delete(cmd_handle_);
        }else{
            printf("temp read returned error %s\n", esp_err_to_name(comm_ok));
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }while(i++ < 10);
    i2c_cmd_link_delete(cmd_handle);

    // parse measurement
    // TODO: Check checksum
    /* This calculations are from Chapter 6 of SHT20 Datasheet*/
    // 1. Check stat bit is Temperature(zero)
    if(((byte1 >> 1) & 1) != 0){ //
    printf("stat error\n");
        //return 0; // error
    }
    // 2. Clear stat bit for temperature calculation
    byte1 = CLEAR_BIT(byte1, 1); 
    byte1 = CLEAR_BIT(byte1, 0); 
    printf("byte0: %d\n", byte0);
    printf("byte1: %d\n", byte1);
    // 3. get 16 bit number 
    uint16_t byte = ((uint16_t)byte0 << 8) | ((uint16_t)byte1);
    // invoke the formula
    t = -46.85 + 175.72*byte/(65536.0);
    return t;
}

float getRHMeasurement(SHT20_config_st *sht_config){
    float rh = 0;
    uint8_t byte0, byte1, byte2;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, SHT20_WRITE, true);
    i2c_master_write_byte(cmd_handle, TRIGGER_RH_MEASUREMENT_HOLD_MASTER, true);

    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, SHT20_READ, true);
    i2c_master_read(cmd_handle, &byte0, 1, I2C_MASTER_ACK);
    i2c_master_read(cmd_handle, &byte1, 1, I2C_MASTER_ACK);
    i2c_master_read(cmd_handle, &byte2, 1, I2C_MASTER_NACK); // checksum
    i2c_master_stop(cmd_handle);
    
    i2c_master_cmd_begin(sht_config->i2c_num, cmd_handle, sht_config->timeout/ portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);

    // parse measurement
    // TODO: check checksum
    /* This calculations are from Chapter 6 of SHT20 Datasheet*/
    // 1. Check stat bit is RH(one)
    if(((byte1 >> 1) & 1) != 1){ //
        return 0; // error
    }
    // 2. Clear stat bit for rh calculation
    byte1 = CLEAR_BIT(byte1, 1); 
    // 3. get 16 bit number 
    uint16_t byte = (byte0 << 8) + byte1;
    // invoke the formula
    // relative humidity above liquid water
    rh = -6.0 + 125.0*byte/(65536.0);
    printf("byte rh = %d\n", byte);
    return rh;
}