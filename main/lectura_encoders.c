#include <stdio.h>
#include <sdkconfig.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include <math.h>

#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_SDA_IO           7
#define I2C_MASTER2_SCL_IO          10
#define I2C_MASTER2_SDA_IO          11
#define I2C_MASTER_PORT             I2C_NUM_0
#define I2C_MASTER_PORT2            I2C_NUM_1
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define I2C_SLAVE_ADDR_LENGHT       I2C_ADDR_BIT_LEN_7
#define I2C_SLAVE_FREQ_HZ           400000
#define I2C_SLAVE_SENSOR1_ADDR      0b1011110
#define I2C_SLAVE_SENSOR2_ADDR      0x4A
#define I2C_SLAVE_SENSOR3_ADDR      0xCA

#define I2C_RESET                   0X00

#define RAD_TO_DEG                  57.296
#define CONSTANT_8BITS              1.56
#define CONSTANT_12BITS             0.098

#define VDD_SENSOR                  4

static const char direcciones[5] = {0x5E, 0x56, 0x4E, 0x4A, 0xCA};

uint8_t adress[95-74];
/* for(uint8_t i = 74; i<95; i++){
    
} */

static const char *TAG = "Prueba";
uint8_t fast_config_dev1 [4] = {0b00000000, 0b00000110, 0b00000000, 0b00100000}; // Configuración del Encoder 1
uint8_t fast_config_dev2 [4] = {0b00000000, 0b01100110, 0b00000000, 0b00100000};
uint8_t fast_config_dev3 [4] = {0b00000000, 0b01000110, 0b00000000, 0b00100000};
uint8_t dataBuff [5];

//static int8_t conversion_table_8bits [8] = {-128, 64, 32, 16, 8, 4, 2, 1}; // Tabla de conversión para 8 bits
static int16_t conversion_table_12bits [12] = {-2048, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1}; // Tabla de conversión para 12 bits

void i2c_config(){
}

void app_main(void){
    // i2c_config();

    gpio_set_direction(VDD_SENSOR,GPIO_MODE_OUTPUT);
    gpio_set_level(VDD_SENSOR, 0);

    // Se configuran los buses i2c //
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
    ESP_LOGI(TAG, "I2C Channel 1 configurado");

    i2c_master_bus_handle_t i2c_bus2_handle = NULL;

    i2c_master_bus_config_t bus2_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT2,
        .scl_io_num = I2C_MASTER2_SCL_IO,
        .sda_io_num = I2C_MASTER2_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus2_config, &i2c_bus2_handle)); // Se setea la configuración
    ESP_LOGI(TAG, "I2C Channel 2 configurado");
    // Se termina la configuración de buses //

    // SE configuran los dispositivos del bus (los encoders) //
    i2c_master_dev_handle_t i2c_dev1_handle = NULL;

    i2c_device_config_t tlv493d_config = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_SLAVE_SENSOR1_ADDR,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_dev2_handle = NULL;

    i2c_device_config_t tlv493d_config2 = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_SLAVE_SENSOR2_ADDR,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_dev3_handle = NULL;

    i2c_device_config_t tlv493d_config3 = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_SLAVE_SENSOR1_ADDR,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_reset = NULL; // Se genera la dirección 0x00 para el reset de los dispositivos

    i2c_device_config_t reset = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_RESET,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_reset2 = NULL; // Se genera la dirección 0x00 para el reset de los dispositivos

    i2c_device_config_t reset2 = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_RESET,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };
    // Se terminan de configurar los encoders //


    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &tlv493d_config, &i2c_dev1_handle));
    ESP_LOGI(TAG, "Handle 1 configurado");

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &tlv493d_config2, &i2c_dev2_handle));
    ESP_LOGI(TAG, "Handle 2 configurado");

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus2_handle, &tlv493d_config3, &i2c_dev3_handle));
    ESP_LOGI(TAG, "Handle 3 configurado");

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &reset, &i2c_reset));
    ESP_LOGI(TAG, "Reset handle configurado");

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus2_handle, &reset2, &i2c_reset2));
    ESP_LOGI(TAG, "Reset2 handle configurado");

    vTaskDelay(pdMS_TO_TICKS(40));

    ESP_ERROR_CHECK(i2c_master_receive(i2c_reset, dataBuff, 1, -1));
    ESP_LOGI(TAG, "Encoder Reiniciado");

    ESP_ERROR_CHECK(i2c_master_receive(i2c_reset2, dataBuff, 1, -1));
    ESP_LOGI(TAG, "Encoder 3 Reiniciado");

    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus_handle, I2C_SLAVE_SENSOR1_ADDR, 10));
    ESP_LOGI(TAG, "Sensor 1 configurado con direccion predeterminada");

    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus2_handle, I2C_SLAVE_SENSOR1_ADDR, 10));
    ESP_LOGI(TAG, "Sensor 3 configurado con direccion predeterminada");

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev3_handle, fast_config_dev1, 4, 10));
    ESP_LOGI(TAG, "Se ha reconfigurado al sensor 3");

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev1_handle, fast_config_dev2, 4, 10));
    ESP_LOGI(TAG, "Se ha reconfigurado al sensor 1");
    
    esp_err_t ret = i2c_master_probe(i2c_bus_handle, 0x4A, 10);
    esp_err_t ret2 = i2c_master_probe(i2c_bus2_handle, 0x5E, 10);
    if(ret == ESP_OK && ret2 == ESP_OK){
        ESP_LOGI(TAG, "Está configurado al sensor 1 en la dirección 0x4A y al sensor 3 en la dirección 0x5E");
    }
    
    /* for(uint8_t i = 0; i<5; i++){
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, direcciones[i], 10);
        if (ret != ESP_OK){
            ESP_LOGI(TAG, "No es %x",direcciones[i]);
        }
        else{
            ESP_LOGI(TAG, "Este si es alv: %x",direcciones[i]);
        }
    } */

    gpio_set_level(VDD_SENSOR,1);

    //ESP_ERROR_CHECK(i2c_master_receive(i2c_dev3_handle, dataBuff, 1, 10));

    vTaskDelay(pdMS_TO_TICKS(1));

    /* ESP_ERROR_CHECK(i2c_master_receive(i2c_reset, dataBuff, 1, -1));
    ESP_LOGI(TAG, "Encoder Reiniciado"); */

    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev1_handle, fast_config_dev1, 4, 10));
    ESP_LOGI(TAG, "Se ha configurado al sensor 2");

    for(uint8_t i = 0; i<5; i++){
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, direcciones[i], 10);
        if (ret != ESP_OK){
            ESP_LOGI(TAG, "No es %x",direcciones[i]);
        }
        else{
            ESP_LOGI(TAG, "Este si es alv: %x",direcciones[i]);
        }
    }

    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus_handle, I2C_SLAVE_SENSOR1_ADDR, 10));
    ESP_LOGI(TAG, "Sensor 2 configurado con direccion predeterminada");

    /* for(uint8_t i = 60; i<150; i++){
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, direcciones[i], 10);
        if (ret != ESP_OK){
            ESP_LOGI(TAG, "No es %x",i);
        }
        else{
            ESP_LOGI(TAG, "Este si es alv: %x",i);
        }
    } */

    while(1){
        ESP_ERROR_CHECK(i2c_master_receive(i2c_dev1_handle, dataBuff, 5, 10));
        float realValue;

        uint16_t data_12Bits = dataBuff[0];
        data_12Bits = data_12Bits << 4;
        data_12Bits = data_12Bits | (dataBuff[4] >> 4);

        for (uint8_t i = 0; i < 12; i++){
            if (data_12Bits & (1 << (11 - i))){
                realValue += conversion_table_12bits[i];
            }
        }

        float valorCampo = realValue * CONSTANT_12BITS;
        realValue = 0.0;
        data_12Bits = 0;

        data_12Bits = dataBuff[1];
        data_12Bits = data_12Bits << 4;
        data_12Bits = data_12Bits | (dataBuff[4] & 0b00001111);

        for (uint8_t i = 0; i < 12; i++){
            if (data_12Bits & (1 << (11 - i))){
                realValue += conversion_table_12bits[i];
            }
        }
        float valorCampo2 = realValue * CONSTANT_12BITS;

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