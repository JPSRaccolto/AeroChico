/**
 * GPS NEO-6
 * Biblioteca simples para leitura de dados GPS
 */

#ifndef GPS_NEO_6_H
#define GPS_NEO_6_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/time.h"

// Estrutura de dados GPS (apenas para uso interno)
typedef struct {
    double longitude;
    double latitude;
    double XGPS;         // metros, Leste desde origem
    double YGPS;         // metros, Norte desde origem  
    double ZGPS;         // altitude em metros
    char time[12];       // UTC
    char time_br[12];    // Brasília
    uint32_t time_seconds;
    double velocity;     // km/h
    bool valid_fix;
    char status;         // 'A' = Active, 'V' = Void
    char satellites[4];
    char date[8];
} gps_data_t;

// Funções públicas principais
void gps_init(void);
void read_gps_data(void);
void read_gps_data_debug(void);
void read_gps_data_zgps_debug(void);
void test_uart_raw(void);
void display_gps_data(void);

// Funções específicas para seus dados (mais simples)
bool is_gps_valid(void);
uint32_t get_gps_time_seconds(void);
double get_gps_x(void);
double get_gps_y(void);
double get_gps_z(void);
double get_gps_velocity(void);

int get_gps_satellites(void);
#endif // GPS_NEO_6_H