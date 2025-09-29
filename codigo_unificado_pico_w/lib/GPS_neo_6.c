/**
 * GPS NEO-6 Simple Library Implementation
 */

#include "GPS_neo_6.h"

// Configurações do GPS
#define GPS_UART_ID uart1
#define GPS_BAUD_RATE 9600
#define GPS_TX_PIN 8
#define GPS_RX_PIN 9

// Buffer NMEA
#define NMEA_BUFFER_SIZE 256
static char nmea_buffer[NMEA_BUFFER_SIZE];
static int buffer_index = 0;

// Dados GPS globais
static gps_data_t gps_data = {0};

// Origem para conversão XY
static double origin_lat = 0.0;
static double origin_lon = 0.0;
static bool origin_set = false;

// Constantes
#define EARTH_RADIUS 6371000.0
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)

// ==================== FUNÇÕES INTERNAS ====================

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

static void process_gpgga(const char* sentence) {
    char temp_sentence[NMEA_BUFFER_SIZE];
    strcpy(temp_sentence, sentence);
    char* token = strtok(temp_sentence, ",");
    int field = 0;

    char lat_str[16] = {0};
    char lat_dir = 0;
    char lon_str[16] = {0};
    char lon_dir = 0;
    char fix_quality = '0';
    char alt_str[16] = {0};
    char num_sat_str[8] = {0};

    while (token != NULL) {
        switch (field) {
            case 1: // Time
                if (strlen(token) >= 6) {
                    strncpy(gps_data.time, token, 11);
                    gps_data.time[11] = '\0';
                    convert_utc_to_brasilia(gps_data.time, gps_data.time_br, &gps_data.time_seconds);
                }
                break;
            case 2: // Latitude
                if (strlen(token) > 0) strncpy(lat_str, token, sizeof(lat_str)-1);
                break;
            case 3: // Latitude direction
                if (strlen(token) > 0) lat_dir = token[0];
                break;
            case 4: // Longitude
                if (strlen(token) > 0) strncpy(lon_str, token, sizeof(lon_str)-1);
                break;
            case 5: // Longitude direction
                if (strlen(token) > 0) lon_dir = token[0];
                break;
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
            case 9: // Altitude (m)
                if (strlen(token) > 0) {
                    strncpy(alt_str, token, sizeof(alt_str)-1);
                    alt_str[sizeof(alt_str)-1] = '\0';
                    gps_data.ZGPS = atof(alt_str);
                }
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    if (fix_quality != '0') {
        gps_data.latitude = nmea_to_decimal(lat_str, lat_dir);
        gps_data.longitude = nmea_to_decimal(lon_str, lon_dir);
        gps_data.valid_fix = true;

        if (!origin_set) {
            origin_lat = gps_data.latitude;
            origin_lon = gps_data.longitude;
            origin_set = true;
            gps_data.XGPS = 0.0;
            gps_data.YGPS = 0.0;
        } else {
            latlon_to_xy(gps_data.latitude, gps_data.longitude, origin_lat, origin_lon, &gps_data.XGPS, &gps_data.YGPS);
        }
    }
}

static void process_gprmc(const char* sentence) {
    char temp_sentence[NMEA_BUFFER_SIZE];
    strcpy(temp_sentence, sentence);
    char* token = strtok(temp_sentence, ",");
    int field = 0;

    char status = 'V';
    char speed_str[16] = {0};

    while (token != NULL) {
        switch (field) {
            case 1: // Time
                if (strlen(token) >= 6 && strlen(gps_data.time) == 0) {
                    strncpy(gps_data.time, token, 11);
                    gps_data.time[11] = '\0';
                    convert_utc_to_brasilia(gps_data.time, gps_data.time_br, &gps_data.time_seconds);
                }
                break;
            case 2: // Status
                if (strlen(token) > 0) status = token[0];
                break;
            case 7: // Speed over ground in knots
                if (strlen(token) > 0) strncpy(speed_str, token, sizeof(speed_str)-1);
                break;
            case 9: // Date
                if (strlen(token) >= 6) {
                    strncpy(gps_data.date, token, 7);
                    gps_data.date[7] = '\0';
                }
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    if (status == 'A' && strlen(speed_str) > 0) {
        double speed_knots = atof(speed_str);
        gps_data.velocity = speed_knots * 1.852;
        if (gps_data.velocity < 1.0) gps_data.velocity = 0.0;
    }
}

static void process_nmea_sentence(const char* sentence) {
    if (!validate_nmea_checksum(sentence)) return;
    
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        process_gpgga(sentence);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        process_gprmc(sentence);
    }
}

// ==================== FUNÇÕES PÚBLICAS ====================

void gps_init(void) {
    uart_init(GPS_UART_ID, GPS_BAUD_RATE);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART_ID, 8, 1, UART_PARITY_NONE);
}

void read_gps_data(void) {
    while (uart_is_readable(GPS_UART_ID)) {
        char c = uart_getc(GPS_UART_ID);
        if (c == '$') {
            buffer_index = 0;
            nmea_buffer[buffer_index++] = c;
        } else if (c == '\n' || c == '\r') {
            if (buffer_index > 0) {
                nmea_buffer[buffer_index] = '\0';
                if (strlen(nmea_buffer) > 6) process_nmea_sentence(nmea_buffer);
                buffer_index = 0;
            }
        } else if (buffer_index < NMEA_BUFFER_SIZE - 1) {
            nmea_buffer[buffer_index++] = c;
        } else {
            buffer_index = 0;
        }
    }
}

void display_gps_data(void) {
    char time_utc_formatted[12];
    char date_formatted[12];
    
    if (strlen(gps_data.time) >= 6) 
        sprintf(time_utc_formatted, "%.2s:%.2s:%.2s", gps_data.time, gps_data.time+2, gps_data.time+4);
    else 
        strcpy(time_utc_formatted, "00:00:00");
    
    if (strlen(gps_data.date) >= 6) 
        sprintf(date_formatted, "%.2s/%.2s/%.2s", gps_data.date, gps_data.date+2, gps_data.date+4);
    else 
        strcpy(date_formatted, "00/00/00");

    printf("\n======= GPS DATA =======\n");
    if (gps_data.valid_fix) {
        printf("STATUS: GPS FIX VÁLIDO\n");
        printf("Longitude (graus): %.8f\n", gps_data.longitude);
        printf("Latitude  (graus): %.8f\n", gps_data.latitude);
        printf("XGPS (m, Leste desde origem): %.2f m\n", gps_data.XGPS);
        printf("YGPS (m, Norte desde origem): %.2f m\n", gps_data.YGPS);
        printf("ZGPS (Altitude): %.2f m\n", gps_data.ZGPS);
        printf("Tempo UTC: %s\n", time_utc_formatted);
        printf("Tempo Brasília: %s\n", gps_data.time_br);
        printf("Segundos: %u s\n", gps_data.time_seconds);
        printf("Data: %s\n", date_formatted);
        printf("Velocidade: %.2f km/h\n", gps_data.velocity);
        printf("Satélites: %s\n", gps_data.satellites);

        printf("\nDADOS COMPACTOS:\n");
        printf("Lon=%.8f,Lat=%.8f,X=%.2f,Y=%.2f,Z=%.2f,T=%s,S=%u,V=%.2f\n",
               gps_data.longitude, gps_data.latitude, gps_data.XGPS, gps_data.YGPS, gps_data.ZGPS,
               gps_data.time_br, gps_data.time_seconds, gps_data.velocity);
    } else {
        printf("STATUS: AGUARDANDO FIX GPS...\n");
    }
    printf("========================\n\n");
}

// Funções específicas - mais simples e eficientes
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

