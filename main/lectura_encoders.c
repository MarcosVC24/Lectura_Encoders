#include <stdio.h>
#include <sdkconfig.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include <math.h>

#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_SDA_IO           7
#define I2C_MASTER_PORT             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define I2C_SLAVE_ADDR_LENGHT       I2C_ADDR_BIT_LEN_7
#define I2C_SLAVE_FREQ_HZ           400000
#define I2C_SLAVE_SENSOR_ADDR       0X5E

#define I2C_RESET                   0X00

#define RAD_TO_DEG                  57.296
//#define TLV493D_SENSOR_ADDR         0x5E

static const char *TAG = "Prueba";
uint8_t fast_config [4] = {0x00, 0x06, 0x00, 0x40};
uint8_t dataBuff [2];

static int8_t conversion_table [8] = {-128, 64, 32, 16, 8, 4, 2, 1};

void i2c_config(){

}

void app_main(void){
    // i2c_config();
    
    // Se configura el bus i2c
    i2c_master_bus_handle_t i2c_bus_handle = NULL;

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle)); // Se setea la configuración
    ESP_LOGI(TAG, "I2C configurado");

    i2c_master_dev_handle_t i2c_dev_handle = NULL;

    i2c_device_config_t tlv493d_config = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_SLAVE_SENSOR_ADDR,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_reset = NULL;

    i2c_device_config_t reset = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_RESET,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };
   
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &tlv493d_config, &i2c_dev_handle));
    ESP_LOGI(TAG, "Handle configurado");

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &reset, &i2c_reset));
    ESP_LOGI(TAG, "Reset handle configurado");

    vTaskDelay(pdMS_TO_TICKS(40));

    ESP_ERROR_CHECK(i2c_master_receive(i2c_reset, dataBuff, 1, -1));
    ESP_LOGI(TAG, "Encoder Reiniciado");

    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus_handle, I2C_SLAVE_SENSOR_ADDR, 10));
    ESP_LOGI(TAG, "Prueba realizada");

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, fast_config, 4, -1));
    ESP_LOGI(TAG, "Al parecer se ha configurado el sensor de manera exitosa");

    while(1){
        ESP_ERROR_CHECK(i2c_master_receive(i2c_dev_handle, dataBuff, 2, 10));
        float realValue;

        for (uint8_t i = 0; i < 8; i++){
            if (dataBuff[0] & (1 << (7 - i))){
                realValue += conversion_table[i];
            }
        }
        float valorCampo = realValue * 1.56;
        realValue = 0.0;
        for (uint8_t i = 0; i < 8; i++){
            if (dataBuff[1] & (1 << (7 - i))){
                realValue += conversion_table[i];
            }
        }

        float valorCampo2 = realValue * 1.56;
        float rotacion_rad = atan2(valorCampo, valorCampo2);
        float rotacion_deg = rotacion_rad * RAD_TO_DEG;
        rotacion_deg -= 148.17; // Offset para cero
        if (rotacion_deg < 0){
            rotacion_deg = 180 + (180 - fabs(rotacion_deg));
        }
        ESP_LOGI(TAG, "El valor crudo es el siguiente para Bx: %u y para By: %u", dataBuff[0], dataBuff[1]);
        ESP_LOGI(TAG, "El valor procesado es el siguiente para Bx: %.2fmT y para By: %.2fmt", valorCampo, valorCampo2);
        realValue = 0.0;
        ESP_LOGI(TAG, "Rotacion: %.2f", rotacion_deg);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}