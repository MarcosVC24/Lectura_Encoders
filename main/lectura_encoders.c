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

float realValue = 0;
float valorCampoX = 0;
float valorCampoY = 0;
float rotacion_rad = 0;
float rotacion_deg[3];

static const char direcciones[5] = {0x5E, 0x56, 0x4E, 0x4A, 0xCA}; // Direcciones para los encoders (solo funcionan 0x5E y 0x4A)

static const char *TAG = "Prueba";
uint8_t fast_config_dev1 [4] = {0b00000000, 0b00000110, 0b00000000, 0b00100000}; // Configuración fast config con dirección en 0x5E
uint8_t fast_config_dev2 [4] = {0b00000000, 0b01100110, 0b00000000, 0b00100000}; // Configuración fast config con dirección en 0x4A
uint8_t dataBuff[5] = {0};
uint8_t datas[3][5] = {0};

//static int8_t conversion_table_8bits [8] = {-128, 64, 32, 16, 8, 4, 2, 1}; // Tabla de conversión para 8 bits
static int16_t conversion_table_12bits [12] = {-2048, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1}; // Tabla de conversión para 12 bits

void i2c_config(){
}

void app_main(void){
    // i2c_config();

    gpio_set_direction(VDD_SENSOR,GPIO_MODE_OUTPUT); // Se configura al pin de alimentación del sensor 2 cómo salida
    gpio_set_level(VDD_SENSOR, 0); // Se configura en low

    // Se configuran los buses i2c //
    
    // BUS 1
    i2c_master_bus_handle_t i2c_bus_handle = NULL; // Handle a utilizar para esta unidad i2c

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

    // BUS 2
    i2c_master_bus_handle_t i2c_bus2_handle = NULL; // Handle a utilizar para esta unidad i2c

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
    i2c_master_dev_handle_t i2c_dev1_handle = NULL; // Handle para Encoder 1
    // Configuración Encoder 1
    i2c_device_config_t tlv493d_config = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_SLAVE_SENSOR1_ADDR,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_dev2_handle = NULL; // Handle para Encoder 2
    // Configuración Encoder 2
    i2c_device_config_t tlv493d_config2 = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_SLAVE_SENSOR2_ADDR,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_dev3_handle = NULL; // Handle para Encoder 3
    // Handle para Encoder 3
    i2c_device_config_t tlv493d_config3 = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_SLAVE_SENSOR1_ADDR,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_reset = NULL; // Se genera la dirección 0x00 para el reset de los dispositivos del bus i2c 1
    // Se configura el reset para el bus 1
    i2c_device_config_t reset = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_RESET,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_reset2 = NULL; // Se genera la dirección 0x00 para el reset de los dispositivos del bus i2c 2
    // Se configura el reset para el bus 2
    i2c_device_config_t reset2 = {
        .dev_addr_length = I2C_SLAVE_ADDR_LENGHT,
        .device_address = I2C_RESET,
        .scl_speed_hz = I2C_SLAVE_FREQ_HZ,
    };
    // Se terminan de configurar los encoders //

    // Se asignan los dispositivos a los buses //
    // Encoder 1 a bus 1
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &tlv493d_config, &i2c_dev1_handle));
    ESP_LOGI(TAG, "Handle 1 configurado");
    // Encoder 3 a bus 1
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &tlv493d_config2, &i2c_dev2_handle));
    ESP_LOGI(TAG, "Handle 2 configurado");
    // Encoder 2 a bus 2
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus2_handle, &tlv493d_config3, &i2c_dev3_handle));
    ESP_LOGI(TAG, "Handle 3 configurado");
    // Reset 1 a bus 1
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &reset, &i2c_reset));
    ESP_LOGI(TAG, "Reset handle configurado");
    // Reset 2 a bus 2
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus2_handle, &reset2, &i2c_reset2));
    ESP_LOGI(TAG, "Reset2 handle configurado");
    // Se terminan de asignar los dispositivos a los buses //

    vTaskDelay(pdMS_TO_TICKS(40)); // Delay antes de entablar comunicación con los dispositvos para dar tiempo de respuesta

    // Se configuran los Encoders 1 y 3 //
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
   
    // Se verifica que se han asignado correctamnte las direcciones a los encoders
    esp_err_t ret = i2c_master_probe(i2c_bus_handle, 0x4A, 10);
    esp_err_t ret2 = i2c_master_probe(i2c_bus2_handle, 0x5E, 10);
    if(ret == ESP_OK && ret2 == ESP_OK){
        ESP_LOGI(TAG, "Está configurado al sensor 1 en la dirección 0x4A y al sensor 3 en la dirección 0x5E");
    }
    // Termina la configuración de los encoders 1 y 3

    // Esto puede ser utilizado para escanear las posibles direcciones en dónde se deberían de encontrar los encoders reconfigurados
    /* for(uint8_t i = 0; i<5; i++){
        esp_err_t ret = i2c_master_probe(i2c_bus_handle, direcciones[i], 10);
        if (ret != ESP_OK){
            ESP_LOGI(TAG, "No es %x",direcciones[i]);
        }
        else{
            ESP_LOGI(TAG, "Este si es alv: %x",direcciones[i]);
        }
    } */

    // Se configura al Encoder 2 //
    gpio_set_level(VDD_SENSOR,1);

    vTaskDelay(pdMS_TO_TICKS(1));

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
    // Termina la configuración del sensor 2 //
    while(1){
        ESP_ERROR_CHECK(i2c_master_receive(i2c_dev1_handle, dataBuff, 5, 10)); // Se leen los registros necesarios para el Encoder 1
        uint16_t data_12Bits = dataBuff[0];
        data_12Bits = data_12Bits << 4;
        data_12Bits = data_12Bits | (dataBuff[4] >> 4);

        for (uint8_t i = 0; i < 12; i++){
            if (data_12Bits & (1 << (11 - i))){
                realValue += conversion_table_12bits[i];
            }
        }
        valorCampoX = realValue * CONSTANT_12BITS;
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
        valorCampoY = realValue * CONSTANT_12BITS;
        rotacion_rad = atan2(valorCampoX, valorCampoY);
        rotacion_deg[1] = rotacion_rad * RAD_TO_DEG;
        
        ESP_ERROR_CHECK(i2c_master_receive(i2c_dev2_handle, dataBuff, 5, 10)); // Se leen los registros necesarios para el Encoder 2
        data_12Bits = dataBuff[0];
        data_12Bits = data_12Bits << 4;
        data_12Bits = data_12Bits | (dataBuff[4] >> 4);

        for (uint8_t i = 0; i < 12; i++){
            if (data_12Bits & (1 << (11 - i))){
                realValue += conversion_table_12bits[i];
            }
        }
        
        valorCampoX = realValue * CONSTANT_12BITS;
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
        valorCampoY = realValue * CONSTANT_12BITS;
        rotacion_rad = atan2(valorCampoX, valorCampoY);
        rotacion_deg[2] = rotacion_rad * RAD_TO_DEG;
        
        ESP_ERROR_CHECK(i2c_master_receive(i2c_dev3_handle, dataBuff, 5, 10)); // Se leen los registros necesarios para el Encoder 3
        data_12Bits = dataBuff[0];
        data_12Bits = data_12Bits << 4;
        data_12Bits = data_12Bits | (dataBuff[4] >> 4);

        for (uint8_t i = 0; i < 12; i++){
            if (data_12Bits & (1 << (11 - i))){
                realValue += conversion_table_12bits[i];
            }
        }
        valorCampoX = realValue * CONSTANT_12BITS;
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
        valorCampoY = realValue * CONSTANT_12BITS;
        rotacion_rad = atan2(valorCampoX, valorCampoY);
        rotacion_deg[3] = rotacion_rad * RAD_TO_DEG;

        ESP_LOGI(TAG, "Rotación Encoder 1: %.2f Rotación Encoder 2: %.2f Rotación Encoder 3: %.2f", rotacion_deg[1],rotacion_deg[2],rotacion_deg[3]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}