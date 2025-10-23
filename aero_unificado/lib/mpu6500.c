#include <stdio.h>
#include <math.h>
#include "mpu6500.h"

// Variáveis globais definidas aqui
float bias_giro[3] = {0};
float erro_aceleracao[3] = {0};

// Funções auxiliares I2C
static void mpu6500_escrever(uint8_t reg, uint8_t dado) {
    uint8_t buf[] = {reg, dado};
    i2c_write_blocking(I2C_PORT, MPU6500_ENDERECO, buf, 2, false);
}

static void mpu6500_ler(uint8_t reg, uint8_t *buf, uint16_t tamanho) {
    i2c_write_blocking(I2C_PORT, MPU6500_ENDERECO, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6500_ENDERECO, buf, tamanho, false);
}

// Inicialização do sensor
void mpu6500_inicializar() {
    mpu6500_escrever(0x6B, 0x00);  // Sai do modo de suspensão
    sleep_ms(100);
    mpu6500_escrever(0x6B, 0x01);  // Usa giroscópio X como clock
    sleep_ms(100);

    mpu6500_escrever(0x1A, 0x03);  // DLPF giroscópio 41 Hz
    mpu6500_escrever(0x1B, 0x08);  // ±500 °/s
    mpu6500_escrever(0x1D, 0x03);  // DLPF acelerômetro 44.8 Hz
    mpu6500_escrever(0x1C, 0x08);  // ±4g
}

// Calibração giroscópio
void calibra_giroscopio(){
    printf("Calibrando giroscópio... mantenha o sensor parado.\n");
    int32_t soma_giro[3] = {0};

    for (int i = 0; i < NUM_AMOSTRAS; i++) {
        uint8_t buffer[6];
        int16_t leitura_bruta[3];

        mpu6500_ler(0x43, buffer, 6);
        for (int j = 0; j < 3; j++) {
            leitura_bruta[j] = (int16_t)((buffer[j * 2] << 8) | buffer[j * 2 + 1]);
            soma_giro[j] += leitura_bruta[j];
        }
        sleep_ms(5);
    }

    for (int j = 0; j < 3; j++) {
        bias_giro[j] = (soma_giro[j] / (float)NUM_AMOSTRAS) / SENSIBILIDADE_GIRO;
        printf("Bias giroscópio eixo %c: %.2f °/s\n", 'X' + j, bias_giro[j]);
    }
}

// Calibração acelerômetro
void calibra_aceleracao(){
    printf("Calibrando acelerômetro... mantenha o sensor parado.\n");
    int32_t soma_aceleracao[3] = {0};

    for(int i = 0; i < NUM_AMOSTRAS; i++) {
        uint8_t buffer[6];
        int16_t leitura_bruta[3];
        mpu6500_ler(0x3B, buffer, 6);

        for (int j = 0; j < 3; j++) {
            leitura_bruta[j] = (int16_t)((buffer[j * 2] << 8) | buffer[j * 2 + 1]);
            soma_aceleracao[j] += leitura_bruta[j];
        }
        sleep_ms(5);
    }

    for (int j = 0; j < 3; j++) {
        float media = (soma_aceleracao[j] / (float)NUM_AMOSTRAS) / SENSIBILIDADE_ACELERACAO;

        if (j == 2) {
            erro_aceleracao[j] = media - 1.0;  // Z ≈ +1g
        } else {
            erro_aceleracao[j] = media;
        }
        printf("Erro acelerômetro %c: %.2f g\n", 'X' + j, erro_aceleracao[j]);
    }
}

// Leitura com filtro complementar
void leitura(float bias_giro[3], float erro_aceleracao[3], float *theta, float *phi, float dt) {
    int16_t aceleracao_bruto[3], giro_bruto[3];
    float aceleracao[3], giro[3];
    uint8_t buffer[6];

    // Acelerômetro
    mpu6500_ler(0x3B, buffer, 6);
    for (int i = 0; i < 3; i++) {
        aceleracao_bruto[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
        float erro_corrigido = copysign(erro_aceleracao[i], aceleracao_bruto[i]);
        aceleracao[i] = ((aceleracao_bruto[i] / SENSIBILIDADE_ACELERACAO) - erro_corrigido) * GRAVIDADE;
    }

    // Giroscópio
    mpu6500_ler(0x43, buffer, 6);
    for (int i = 0; i < 3; i++) {
        giro_bruto[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
        giro[i] = (giro_bruto[i] / SENSIBILIDADE_GIRO) - bias_giro[i]; // °/s
    }

    // Ângulos via acelerômetro (graus)
    float theta_acc = atan2(aceleracao[0], sqrt(aceleracao[1]*aceleracao[1] + aceleracao[2]*aceleracao[2])) * (180.0 / M_PI);
    float phi_acc   = atan2(aceleracao[1], sqrt(aceleracao[0]*aceleracao[0] + aceleracao[2]*aceleracao[2])) * (180.0 / M_PI);

    // Integração giroscópio
    float theta_giro = *theta + giro[0] * dt;
    float phi_giro   = *phi   + giro[1] * dt;

    // Filtro complementar
    float alpha = 0.95; // mais responsivo
    *theta = alpha * theta_giro + (1 - alpha) * theta_acc;
    *phi   = alpha * phi_giro   + (1 - alpha) * phi_acc;

    //printf("Atitude (Pitch θ): %.2f° | Bank Angle (Roll φ): %.2f°\n", *theta, *phi);
}
