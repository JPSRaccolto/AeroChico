#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6500.h"
#include "bme680_custom.h"
#include "GPS_neo_6.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);

    // ---------- Inicialização I2C para MPU6500 ----------
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("Inicializando MPU-6500...\n");
    mpu6500_inicializar();

    uint8_t id, buf;
    i2c_write_blocking(I2C_PORT, MPU6500_ENDERECO, (uint8_t[]){0x75}, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6500_ENDERECO, &buf, 1, false);
    id = buf;
    if (id != 0x70 && id != 0x68) {
        printf("Falha ao detectar MPU-6500!\n");
        return 1;
    }

    calibra_giroscopio();
    calibra_aceleracao();

    float theta = 0.0, phi = 0.0;
    absolute_time_t t_anterior = get_absolute_time();

    // ---------- Inicialização BME680 ----------
    struct bme680_dev sensor;
    uint16_t periodo;
    printf("Inicializando BME680...\n");
    bme680_inicializar(&sensor, &periodo);
    float pressao_base = calibrar_pressao(&sensor, periodo);
    printf("Pressao base: %.2f hPa\n", pressao_base);

    // ---------- Inicialização GPS ----------
    gps_init();
    uint32_t last_display = 0;

    // ---------- Loop principal ----------
    while (true) {
        // ----- MPU6500 -----
        absolute_time_t t_atual = get_absolute_time();
        float dt = absolute_time_diff_us(t_anterior, t_atual) / 1e6f;
        t_anterior = t_atual;
        leitura(bias_giro, erro_aceleracao, &theta, &phi, dt);

        // ----- BME680 -----
        float pressao, altitude;
        if (bme680_ler_altitude(&sensor, periodo, pressao_base, &pressao, &altitude)) {
            printf("Pressao: %.2f hPa | Altitude: %.2f m\n", pressao, altitude);
        }

        // ----- GPS -----
        read_gps_data();
        if (is_gps_valid()) {
            uint32_t tempo = get_gps_time_seconds();
            double xgps = get_gps_x();
            double ygps = get_gps_y();
            double zgps = get_gps_z();
            double velocidade = get_gps_velocity();
            // Pode salvar em SD ou transmitir
        } else {
            printf("Aguardando GPS...\n");
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_display > 2000) {
            display_gps_data();
            last_display = now;
        }

        sleep_ms(30); // controla frequência de leitura (~10 Hz base)
    }
}
