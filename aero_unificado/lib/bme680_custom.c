#include <stdio.h>
#include <math.h>
#include "bme680_custom.h"

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    for (int i = 0; i < len; i++) buf[i + 1] = data[i];
    
    int result = i2c_write_blocking(I2C_PORT_BME, dev_id, buf, len + 1, false);
    return result == (len + 1) ? BME680_OK : BME680_E_COM_FAIL;
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    int write_result = i2c_write_blocking(I2C_PORT_BME, dev_id, &reg_addr, 1, true);
    if (write_result != 1) return BME680_E_COM_FAIL;

    int read_result = i2c_read_blocking(I2C_PORT_BME, dev_id, data, len, false);
    return read_result == len ? BME680_OK : BME680_E_COM_FAIL;
}

void user_delay_ms(uint32_t period) {
    sleep_ms(period);
}

void bme680_inicializar(struct bme680_dev *sensor, uint16_t *periodo) {
    i2c_init(I2C_PORT_BME, 400 * 1000);  // 400 kHz - mais rápido
    gpio_set_function(SDA_PIN_BME, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN_BME, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN_BME);
    gpio_pull_up(SCL_PIN_BME);

    sensor->dev_id = BME680_ADDR;
    sensor->intf = BME680_I2C_INTF;
    sensor->read = user_i2c_read;
    sensor->write = user_i2c_write;
    sensor->delay_ms = user_delay_ms;
    sensor->amb_temp = 25; // Temperatura ambiente estimada

    sleep_ms(100);

    if (bme680_init(sensor) != BME680_OK) {
        printf("Erro ao iniciar sensor\n");
        while (1) tight_loop_contents();
    }

    // Configuração RÁPIDA - oversampling mínimo
    sensor->tph_sett.os_hum = BME680_OS_NONE;     // Sem umidade
    sensor->tph_sett.os_pres = BME680_OS_4X;      // 1x pressão (rápido)
    sensor->tph_sett.os_temp = BME680_OS_1X;      // 1x temperatura
    sensor->tph_sett.filter = BME680_FILTER_SIZE_3; // Sem filtro IIR

    // Desabilitar sensor de gás (muito lento)
    sensor->gas_sett.run_gas = BME680_DISABLE_GAS_MEAS;
    sensor->gas_sett.heatr_ctrl = BME680_DISABLE_HEATER;

    uint8_t sel = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | 
                  BME680_FILTER_SEL | BME680_GAS_MEAS_SEL | 
                  BME680_HCNTRL_SEL | BME680_RUN_GAS_SEL;
    
    bme680_set_sensor_settings(sel, sensor);
    bme680_get_profile_dur(periodo, sensor);
    
    printf("Tempo de medição BME680: %u ms\n", *periodo);
}

float calibrar_pressao(struct bme680_dev *sensor, uint16_t periodo) {
    printf("Calibrando pressão base (%d amostras)...\n", NUM_CALIBRACAO);
    float soma = 0.0f;
    int contador_valido = 0;

    for (int i = 0; i < NUM_CALIBRACAO && contador_valido < NUM_CALIBRACAO; i++) {
        sensor->power_mode = BME680_FORCED_MODE;
        bme680_set_sensor_mode(sensor);

        // Delay baseado no tempo de medição real + margem
        user_delay_ms(periodo + 10);

        struct bme680_field_data data;
        if (bme680_get_sensor_data(&data, sensor) == BME680_OK) {
            if (data.status & BME680_NEW_DATA_MSK) {
                float pressao_hpa = data.pressure / 100.0f;
                soma += pressao_hpa;
                contador_valido++;
                
                if (contador_valido % 10 == 0) {
                    printf("Calibração: %d/%d amostras\n", contador_valido, NUM_CALIBRACAO);
                }
            }
        }
    }
    
    if (contador_valido > 0) {
        float pressao_media = soma / contador_valido;
        printf("Calibração completa: %.2f hPa (%d leituras)\n", 
               pressao_media, contador_valido);
        return pressao_media;
    } else {
        printf("ERRO: Nenhuma leitura válida!\n");
        return -1.0f;
    }
}

bool bme680_ler_altitude(struct bme680_dev *sensor, uint16_t periodo,
                         float pressao_base, float *pressao, float *altitude) {
    struct bme680_field_data dados;
    
    sensor->power_mode = BME680_FORCED_MODE;
    bme680_set_sensor_mode(sensor);
    
    // Delay mínimo necessário
    user_delay_ms(periodo + 5);

    if (bme680_get_sensor_data(&dados, sensor) == BME680_OK &&
        (dados.status & BME680_NEW_DATA_MSK)) {
        *pressao = dados.pressure / 100.0f;
        float fator = powf(*pressao / pressao_base, 1.0f / 5.255f);
        *altitude = 44330.0f * (1.0f - fator);
        return true;
    }
    return false;
}