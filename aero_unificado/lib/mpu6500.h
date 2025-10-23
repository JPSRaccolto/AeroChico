#ifndef MPU6500_H
#define MPU6500_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Definições do sensor
#define I2C_PORT i2c1
#define SDA_PIN 2
#define SCL_PIN 3

#define MPU6500_ENDERECO 0x68
#define SENSIBILIDADE_GIRO 131.0        // ±250°/s
#define SENSIBILIDADE_ACELERACAO 8192.0 // ±4g
#define GRAVIDADE 9.81
#define NUM_AMOSTRAS 1000

// Variáveis globais de calibração
extern float bias_giro[3];
extern float erro_aceleracao[3];

// Inicialização e calibração
void mpu6500_inicializar();
void calibra_giroscopio();
void calibra_aceleracao();

// Leitura com filtro complementar
void leitura(float bias_giro[3], float erro_aceleracao[3], float *theta, float *phi, float dt);

#endif
