#ifndef BME680_CUSTOM_H
#define BME680_CUSTOM_H

#include "bme680.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// Configuração do I2C e sensor
#define SDA_PIN_BME 4
#define SCL_PIN_BME 5
#define I2C_PORT_BME i2c0
#define BME680_ADDR BME680_I2C_ADDR_SECONDARY

// Parâmetros de calibração/filtro
#define DEADZONE_METROS 0.2F
#define NUM_CALIBRACAO 50
#define ALPHA 0.2f

// Funções de interface I2C
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);

// Funções de inicialização e calibração
void bme680_inicializar(struct bme680_dev *sensor, uint16_t *periodo);
float calibrar_pressao(struct bme680_dev *sensor, uint16_t periodo);

// Função de leitura processada (pressão e altitude filtrada)
bool bme680_ler_altitude(struct bme680_dev *sensor, uint16_t periodo,
                         float pressao_base, float *pressao, float *altitude);

#endif
