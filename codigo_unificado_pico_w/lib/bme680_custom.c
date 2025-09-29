#include <stdio.h>
#include <math.h>
#include "bme680_custom.h"

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    for (int i = 0; i < len; i++) buf[i + 1] = data[i];
    return i2c_write_blocking(I2C_PORT_BME, dev_id, buf, len + 1, false) == (len + 1) ? BME680_OK : BME680_E_COM_FAIL;
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (i2c_write_blocking(I2C_PORT_BME, dev_id, &reg_addr, 1, true) != 1) return BME680_E_COM_FAIL;
    return i2c_read_blocking(I2C_PORT_BME, dev_id, data, len, false) == len ? BME680_OK : BME680_E_COM_FAIL;
}

void user_delay_ms(uint32_t period) {
    sleep_ms(period);
}

void bme680_inicializar(struct bme680_dev *sensor, uint16_t *periodo) {
    i2c_init(I2C_PORT_BME, 100 * 1000);
    gpio_set_function(SDA_PIN_BME, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN_BME, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN_BME);
    gpio_pull_up(SCL_PIN_BME);

    sensor->dev_id = BME680_ADDR;
    sensor->intf = BME680_I2C_INTF;
    sensor->read = user_i2c_read;
    sensor->write = user_i2c_write;
    sensor->delay_ms = user_delay_ms;

    if (bme680_init(sensor) != BME680_OK) {
        printf("Erro ao iniciar sensor\n");
        while (1) tight_loop_contents();
    }

    sensor->tph_sett.os_hum = BME680_OS_NONE;
    sensor->tph_sett.os_pres = BME680_OS_4X;
    sensor->tph_sett.os_temp = BME680_OS_1X;
    sensor->tph_sett.filter = BME680_FILTER_SIZE_3;

    uint8_t sel = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL;
    bme680_set_sensor_settings(sel, sensor);
    bme680_get_profile_dur(periodo, sensor);
}

float calibrar_pressao(struct bme680_dev *sensor, uint16_t periodo) {
    float soma = 0.0f;
    struct bme680_field_data data;
    for (int i = 0; i < NUM_CALIBRACAO; i++) {
        sensor->power_mode = BME680_FORCED_MODE;
        bme680_set_sensor_mode(sensor);
        user_delay_ms(periodo + 10);
        if (bme680_get_sensor_data(&data, sensor) == BME680_OK && (data.status & BME680_NEW_DATA_MSK)) {
            soma += data.pressure / 100.0f;
        } else i--;
    }
    return soma / NUM_CALIBRACAO;
}

bool bme680_ler_altitude(struct bme680_dev *sensor, uint16_t periodo,
                         float pressao_base, float *pressao, float *altitude) {
    struct bme680_field_data dados;
    sensor->power_mode = BME680_FORCED_MODE;
    bme680_set_sensor_mode(sensor);
    user_delay_ms(periodo + 10);

    if (bme680_get_sensor_data(&dados, sensor) == BME680_OK && (dados.status & BME680_NEW_DATA_MSK)) {
        *pressao = dados.pressure / 100.0f;
        float fator = powf(*pressao / pressao_base, 1.0f / 5.255f);
        *altitude = 44330.0f * (1.0f - fator);
        return true;
    }
    return false;
}
