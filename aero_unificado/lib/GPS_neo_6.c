/**
 * GPS NEO-6 - Versão Completa com Diagnóstico
 */

#include "GPS_neo_6.h"

#define GPS_UART_ID uart0
#define GPS_BAUD_RATE 9600
#define GPS_TX_PIN 17
#define GPS_RX_PIN 16

#define NMEA_BUFFER_SIZE 256
static char nmea_buffer[NMEA_BUFFER_SIZE];
static int buffer_index = 0;

static gps_data_t gps_data = {0};

static double origin_lat = 0.0;
static double origin_lon = 0.0;
static bool origin_set = false;

// Contadores de diagnóstico
static uint32_t sentences_received = 0;
static uint32_t sentences_valid = 0;
static uint32_t sentences_gprmc = 0;
static uint32_t sentences_gpgga = 0;

#define EARTH_RADIUS 6371000.0
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
#define POSITION_THRESHOLD 0.5

static double zgps_anterior = 0.0;  // Guardar último ZGPS válido

static void latlon_to_xy(double latitude, double longitude, double lat0, double lon0, double* XGPS, double* YGPS) {
    double dLat = (latitude - lat0) * DEG_TO_RAD;
    double dLon = (longitude - lon0) * DEG_TO_RAD;
    double latRad = lat0 * DEG_TO_RAD;
    *XGPS = dLon * cos(latRad) * EARTH_RADIUS;
    *YGPS = dLat * EARTH_RADIUS;
}

static void convert_utc_to_brasilia(const char* utc_time, char* br_time, uint32_t* seconds) {
    if (strlen(utc_time) < 6) {
        strcpy(br_time, "00:00:00");
        *seconds = 0;
        return;
    }
    int hours = (utc_time[0] - '0') * 10 + (utc_time[1] - '0');
    int minutes = (utc_time[2] - '0') * 10 + (utc_time[3] - '0');
    int seconds_part = (utc_time[4] - '0') * 10 + (utc_time[5] - '0');
    hours -= 3;
    if (hours < 0) hours += 24;
    sprintf(br_time, "%02d:%02d:%02d", hours, minutes, seconds_part);
    *seconds = (uint32_t)(hours * 3600 + minutes * 60 + seconds_part);
}

static uint8_t calculate_nmea_checksum(const char* sentence, int start, int end) {
    uint8_t checksum = 0;
    for (int i = start; i < end; i++) checksum ^= (uint8_t)sentence[i];
    return checksum;
}

static bool validate_nmea_checksum(const char* sentence) {
    int len = strlen(sentence);
    if (len < 5) return false;
    int star_pos = -1;
    for (int i = len - 3; i >= 0; i--) {
        if (sentence[i] == '*') { star_pos = i; break; }
    }
    if (star_pos == -1) return false;
    uint8_t calculated = calculate_nmea_checksum(sentence, 1, star_pos);
    char checksum_str[3] = {0};
    strncpy(checksum_str, &sentence[star_pos + 1], 2);
    uint8_t received = (uint8_t)strtol(checksum_str, NULL, 16);
    return calculated == received;
}

static double nmea_to_decimal(const char* coord, char direction) {
    if (coord == NULL || strlen(coord) == 0) return 0.0;
    double value = atof(coord);
    int degrees = (int)(value / 100.0);
    double minutes = value - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);
    if (direction == 'S' || direction == 'W') decimal = -decimal;
    return decimal;
}

static void process_gprmc(const char* sentence) {
    sentences_gprmc++;
    
    char temp_sentence[NMEA_BUFFER_SIZE];
    strcpy(temp_sentence, sentence);
    char* token = strtok(temp_sentence, ",");
    int field = 0;

    char time_str[12] = {0};
    char status = 'V';
    char lat_str[16] = {0};
    char lat_dir = 0;
    char lon_str[16] = {0};
    char lon_dir = 0;
    char speed_str[16] = {0};

    while (token != NULL) {
        switch (field) {
            case 1:
                if (strlen(token) >= 6) strncpy(time_str, token, 11);
                break;
            case 2:
                if (strlen(token) > 0) status = token[0];
                break;
            case 3:
                if (strlen(token) > 0) strncpy(lat_str, token, 15);
                break;
            case 4:
                if (strlen(token) > 0) lat_dir = token[0];
                break;
            case 5:
                if (strlen(token) > 0) strncpy(lon_str, token, 15);
                break;
            case 6:
                if (strlen(token) > 0) lon_dir = token[0];
                break;
            case 7:
                if (strlen(token) > 0) strncpy(speed_str, token, 15);
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    if (strlen(time_str) >= 6) {
        strncpy(gps_data.time, time_str, 11);
        gps_data.time[11] = '\0';
        convert_utc_to_brasilia(gps_data.time, gps_data.time_br, &gps_data.time_seconds);
    }

    if (status == 'A') {
        gps_data.valid_fix = true;
        
        if (strlen(speed_str) > 0) {
            double speed_knots = atof(speed_str);
            gps_data.velocity = speed_knots * 1.852;
            if (gps_data.velocity < 0.5) gps_data.velocity = 0.0;
        }
        
        if (strlen(lat_str) > 0 && strlen(lon_str) > 0) {
            gps_data.latitude = nmea_to_decimal(lat_str, lat_dir);
            gps_data.longitude = nmea_to_decimal(lon_str, lon_dir);
            
            if (!origin_set) {
                origin_lat = gps_data.latitude;
                origin_lon = gps_data.longitude;
                origin_set = true;
                gps_data.XGPS = 0.0;
                gps_data.YGPS = 0.0;
            } else {
                double new_x, new_y;
                latlon_to_xy(gps_data.latitude, gps_data.longitude, origin_lat, origin_lon, &new_x, &new_y);
                
                double dx = new_x - gps_data.XGPS;
                double dy = new_y - gps_data.YGPS;
                double dist = sqrt(dx*dx + dy*dy);
                
                if (dist > POSITION_THRESHOLD || gps_data.velocity > 1.0) {
                    gps_data.XGPS = new_x;
                    gps_data.YGPS = new_y;
                }
            }
        }
    } else {
        gps_data.valid_fix = false;
    }
}

static void process_gpgga(const char* sentence) {
    char temp_sentence[NMEA_BUFFER_SIZE];
    strcpy(temp_sentence, sentence);
    char* token = strtok(temp_sentence, ",");
    int field = 0;

    char fix_quality = '0';
    char alt_str[16] = {0};
    char num_sat_str[8] = {0};

    while (token != NULL) {
        switch (field) {
            case 6: // Fix quality
                if (strlen(token) > 0) fix_quality = token[0];
                break;
            case 7: // Number of satellites
                if (strlen(token) > 0) {
                    strncpy(num_sat_str, token, sizeof(num_sat_str)-1);
                    num_sat_str[sizeof(num_sat_str)-1] = '\0';
                    strncpy(gps_data.satellites, num_sat_str, 3);
                    gps_data.satellites[3] = '\0';
                }
                break;
            case 9: // Altitude
                if (strlen(token) > 0 && fix_quality != '0') {
                    strncpy(alt_str, token, sizeof(alt_str)-1);
                    alt_str[sizeof(alt_str)-1] = '\0';
                    double zgps_novo = atof(alt_str);
                    
                    // PROTEÇÃO: só atualiza se for um valor razoável (> 0)
                    if (zgps_novo > 0) {
                        gps_data.ZGPS = zgps_novo;
                        zgps_anterior = zgps_novo;  // Guardar como válido
                    } else {
                        // Se receber 0 ou negativo, mantém o anterior
                        gps_data.ZGPS = zgps_anterior;
                    }
                } else {
                    // Se fix_quality = '0', mantém o valor anterior
                    gps_data.ZGPS = zgps_anterior;
                }
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }
}
static void process_nmea_sentence(const char* sentence) {
    sentences_received++;
    
    if (!validate_nmea_checksum(sentence)) return;
    
    sentences_valid++;
    
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        process_gprmc(sentence);
    } else if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        process_gpgga(sentence);
    }
}

void gps_init(void) {
    uart_init(GPS_UART_ID, GPS_BAUD_RATE);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART_ID, true);
}

void read_gps_data(void) {
    while (uart_is_readable(GPS_UART_ID)) {
        char c = uart_getc(GPS_UART_ID);
        
        if (c == '$') {
            // Se havia dados antes, processa antes de resetar
            if (buffer_index > 0) {
                nmea_buffer[buffer_index] = '\0';
                if (strlen(nmea_buffer) > 6) {
                    process_nmea_sentence(nmea_buffer);
                }
            }
            buffer_index = 0;
            nmea_buffer[buffer_index++] = c;
        } 
        else if (c == '\n' || c == '\r') {
            if (buffer_index > 0 && nmea_buffer[0] == '$') {
                nmea_buffer[buffer_index] = '\0';
                if (strlen(nmea_buffer) > 6) {
                    process_nmea_sentence(nmea_buffer);
                }
                buffer_index = 0;
            }
        } 
        else if (buffer_index < NMEA_BUFFER_SIZE - 1) {
            nmea_buffer[buffer_index++] = c;
        }
    }
}

// Adicione esta função ao GPS_neo_6.c para DEBUG apenas do ZGPS

void read_gps_data_zgps_debug(void) {
    while (uart_is_readable(GPS_UART_ID)) {
        char c = uart_getc(GPS_UART_ID);
        
        if (c == '$') {
            if (buffer_index > 0) {
                nmea_buffer[buffer_index] = '\0';
                if (strlen(nmea_buffer) > 6) {
                    // SÓ MOSTRA SE FOR GPGGA (que contém altitude)
                    if (strncmp(nmea_buffer, "$GPGGA", 6) == 0 || strncmp(nmea_buffer, "$GNGGA", 6) == 0) {
                        printf("DEBUG GPGGA ANTES: %s\n", nmea_buffer);
                        printf("  ZGPS ANTES: %.2f\n", gps_data.ZGPS);
                    }
                    process_nmea_sentence(nmea_buffer);
                    
                    if (strncmp(nmea_buffer, "$GPGGA", 6) == 0 || strncmp(nmea_buffer, "$GNGGA", 6) == 0) {
                        printf("  ZGPS DEPOIS: %.2f\n\n", gps_data.ZGPS);
                    }
                }
            }
            buffer_index = 0;
            nmea_buffer[buffer_index++] = c;
        } 
        else if (c == '\n' || c == '\r') {
            if (buffer_index > 0 && nmea_buffer[0] == '$') {
                nmea_buffer[buffer_index] = '\0';
                if (strlen(nmea_buffer) > 6) {
                    if (strncmp(nmea_buffer, "$GPGGA", 6) == 0 || strncmp(nmea_buffer, "$GNGGA", 6) == 0) {
                        printf("DEBUG GPGGA ANTES: %s\n", nmea_buffer);
                        printf("  ZGPS ANTES: %.2f\n", gps_data.ZGPS);
                    }
                    process_nmea_sentence(nmea_buffer);
                    
                    if (strncmp(nmea_buffer, "$GPGGA", 6) == 0 || strncmp(nmea_buffer, "$GNGGA", 6) == 0) {
                        printf("  ZGPS DEPOIS: %.2f\n\n", gps_data.ZGPS);
                    }
                }
                buffer_index = 0;
            }
        } 
        else if (buffer_index < NMEA_BUFFER_SIZE - 1) {
            nmea_buffer[buffer_index++] = c;
        }
    }
}
void read_gps_data_debug(void) {
    while (uart_is_readable(GPS_UART_ID)) {
        char c = uart_getc(GPS_UART_ID);
        if (c == '$') {
            buffer_index = 0;
            nmea_buffer[buffer_index++] = c;
        } else if (c == '\n' || c == '\r') {
            if (buffer_index > 0) {
                nmea_buffer[buffer_index] = '\0';
                if (strlen(nmea_buffer) > 6) {
                    // PRINT DEBUG - VER TUDO QUE CHEGA
                    printf("DEBUG GPS RX: %s\n", nmea_buffer);
                    
                    process_nmea_sentence(nmea_buffer);
                    
                    // PRINT STATUS PÓS PROCESSAMENTO
                    printf("  → STATUS: %c | VALID: %d | LAT: %.6f | LON: %.6f | ALT: %.2f | SATS: %s | TIME: %s\n",
                           gps_data.status, gps_data.valid_fix, gps_data.latitude, 
                           gps_data.longitude, gps_data.ZGPS, gps_data.satellites, gps_data.time_br);
                }
                buffer_index = 0;
            }
        } else if (buffer_index < NMEA_BUFFER_SIZE - 1) {
            nmea_buffer[buffer_index++] = c;
        } else {
            buffer_index = 0;
        }
    }
}

void test_uart_raw(void) {
    printf("\n=== TESTE DE UART BRUTO ===\n");
    printf("Aguardando dados por 10 segundos...\n");
    
    absolute_time_t start = get_absolute_time();
    uint32_t byte_count = 0;
    
    while (absolute_time_diff_us(start, get_absolute_time()) < 10000000) {  // 10 segundos
        if (uart_is_readable(GPS_UART_ID)) {
            char c = uart_getc(GPS_UART_ID);
            printf("%c", c);
            byte_count++;
        }
        sleep_ms(1);
    }
    
    printf("\n\n=== TESTE FINALIZADO ===\n");
    printf("Total de bytes recebidos: %u\n", byte_count);
    
    if (byte_count == 0) {
        printf("⚠️  NENHUM DADO RECEBIDO! Verificar:\n");
        printf("   - Conexão RX do GPS\n");
        printf("   - Baudrate (9600)\n");
        printf("   - Alimentação do GPS\n");
    } else {
        printf("✓ GPS está enviando dados\n");
    }
}

void display_gps_data(void) {
    printf("\n======= GPS DATA =======\n");
    
    if (gps_data.valid_fix) {
        printf("STATUS: GPS FIX VALIDO\n");
        printf("Posicao: X=%.2f Y=%.2f Z=%.2f m\n", 
               gps_data.XGPS, gps_data.YGPS, gps_data.ZGPS);
        printf("Tempo: %s (%u s)\n", gps_data.time_br, gps_data.time_seconds);
        printf("Velocidade: %.2f km/h\n", gps_data.velocity);
        printf("Satelites: %s\n", gps_data.satellites);
    } else {
        printf("STATUS: AGUARDANDO FIX GPS\n");
        printf("Satelites: %s\n", gps_data.satellites);
        printf("Tempo: %s\n", gps_data.time_br);
    }
    printf("========================\n");
}

bool is_gps_valid(void) {
    return gps_data.valid_fix;
}

uint32_t get_gps_time_seconds(void) {
    return gps_data.time_seconds;
}

double get_gps_x(void) {
    return gps_data.XGPS;
}

double get_gps_y(void) {
    return gps_data.YGPS;
}

double get_gps_z(void) {
    return gps_data.ZGPS;
}

double get_gps_velocity(void) {
    return gps_data.velocity;
}

int get_gps_satellites(void) {
    return atoi(gps_data.satellites);
}

// Função de diagnóstico
void gps_print_stats(void) {
    printf("[STATS] Total=%u Validas=%u RMC=%u GGA=%u Fix=%d Sats=%s\n",
           sentences_received, sentences_valid, sentences_gprmc, sentences_gpgga,
           gps_data.valid_fix, gps_data.satellites);
}