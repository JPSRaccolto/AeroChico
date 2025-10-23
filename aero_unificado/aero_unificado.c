#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6500.h"
#include "bme680_custom.h"
#include "GPS_neo_6.h"

#define GPS_FILTER_SIZE 5
#define GPS_MOVEMENT_THRESHOLD 0.5  // Ignorar movimentos menores que 50cm
#define G_ACCEL 9.81  // Aceleração gravitacional em m/s²

// Estados do planador
typedef enum {
    ATT = 0,  // Acoplado à nave mãe
    DPL = 1,  // Em voo
    LND = 2   // Em solo
} drone_status_t;

typedef struct {
    double x_buffer[GPS_FILTER_SIZE];
    double y_buffer[GPS_FILTER_SIZE];
    double z_buffer[GPS_FILTER_SIZE];
    int index;
    int count;
} gps_filter_t;

typedef struct {
    uint32_t gps_time;
    double latitude;
    double longitude;
    double altitude_gps;
    double altitude_bme;
    double velocity_cas;
    double accel_x;
    double accel_y;
    double accel_z;
    double theta;
    double phi;
    drone_status_t status;
    uint8_t gps_sats;
} hud_data_t;

gps_filter_t gps_filter = {0};
hud_data_t hud_data = {0};

void gps_filter_add(double x, double y, double z) {
    gps_filter.x_buffer[gps_filter.index] = x;
    gps_filter.y_buffer[gps_filter.index] = y;
    gps_filter.z_buffer[gps_filter.index] = z;
    
    gps_filter.index = (gps_filter.index + 1) % GPS_FILTER_SIZE;
    if (gps_filter.count < GPS_FILTER_SIZE) {
        gps_filter.count++;
    }
}

void gps_filter_get_average(double *x, double *y, double *z) {
    double sum_x = 0, sum_y = 0, sum_z = 0;
    
    for (int i = 0; i < gps_filter.count; i++) {
        sum_x += gps_filter.x_buffer[i];
        sum_y += gps_filter.y_buffer[i];
        sum_z += gps_filter.z_buffer[i];
    }
    
    *x = sum_x / gps_filter.count;
    *y = sum_y / gps_filter.count;
    *z = sum_z / gps_filter.count;
}

// Calcular CAS (Calibrated Airspeed) a partir de pressão dinâmica
double calcular_cas(float pressao_atual, float pressao_base) {
    // Pressão dinâmica = pressão_atual - pressao_base
    float pressao_dinamica = (pressao_atual - pressao_base) * 100.0f;  // Pa
    
    // CAS = sqrt(2 * Pressão_dinâmica / densidade_ar)
    // Densidade ar ao nível do mar ~1.225 kg/m³
    float densidade_ar = 1.225f;
    
    // Filtro de ruído: se pressão dinâmica < 0.5 Pa, considerar parado
    if (pressao_dinamica < 0.5f) pressao_dinamica = 0;
    if (pressao_dinamica < 0) pressao_dinamica = 0;
    
    float cas_ms = sqrt((2.0f * pressao_dinamica) / densidade_ar);
    float cas_kmh = cas_ms * 3.6f;
    
    return (double)cas_kmh;
}

// Variável para guardar o status anterior (para hysteresis)
static drone_status_t status_anterior = ATT;

// Determinar status do planador com parâmetro de tempo + hysteresis
drone_status_t determinar_status(double altitude, double velocidade, uint32_t tempo_voo) {
    // LND (Em Solo): altitude < 2m E velocidade < 0.5 km/h
    if (altitude < 2.0 && velocidade < 0.5) {
        status_anterior = LND;
        return LND;
    }
    
    // DPL (Em Voo): altitude > 5m E tempo > 60 segundos E velocidade > 0.5 km/h
    if (altitude > 5.0 && tempo_voo > 60 && velocidade > 0.5) {
        status_anterior = DPL;
        return DPL;
    }
    
    // ATT (Acoplado): zona intermediária com hysteresis
    // Se estava em LND, precisa subir para > 3m ou velocidade > 1 km/h para sair
    if (status_anterior == LND && (altitude > 3.0 || velocidade > 1.0)) {
        status_anterior = ATT;
        return ATT;
    }
    
    // Se estava em DPL, precisa descer para < 3m ou velocidade < 0.5 km/h para sair
    if (status_anterior == DPL && (altitude < 3.0 || velocidade < 0.5)) {
        status_anterior = ATT;
        return ATT;
    }
    
    // Manter o status anterior na zona cinzenta
    return status_anterior;
}

const char* status_to_string(drone_status_t status) {
    switch (status) {
        case ATT: return "ATT";
        case DPL: return "DPL";
        case LND: return "LND";
        default: return "UNK";
    }
}

// Enviar dados para HUD (sobreposição de vídeo)
void enviar_hud(hud_data_t *hud) {
    // Formato: HUD|TIME|ALT|CAS|G_Z|SAT
    // Exemplo: HUD|19:03:44|424.70|15.5|1.02|09
    
    uint32_t time_s = hud->gps_time;
    uint8_t hours = (time_s / 3600) % 24;
    uint8_t minutes = (time_s / 60) % 60;
    uint8_t seconds = time_s % 60;
    
    // Fator de carga em Z (em múltiplos de g)
    double g_z = hud->accel_z / G_ACCEL;
    
    printf("HUD|%02d:%02d:%02d|%.1f|%.1f|%.2f|%s\n",
           hours, minutes, seconds,
           hud->altitude_bme,
           hud->velocity_cas,
           g_z,
           status_to_string(hud->status));
}

// Salvar dados brutos em arquivo (para análise pós-voo)
void salvar_dados_arquivo(double xgps, double ygps, double zgps, float theta, float phi, uint32_t tempo_gps) {
    // Formato original: DATA,tempo_segundos,X,Y,Z,theta,phi
    printf("DATA,%u,%.2f,%.2f,%.2f,%.2f,%.2f\n",
           tempo_gps, xgps, ygps, zgps, theta, phi);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("Sistema iniciando...\n");

    // GPS
    printf("Inicializando GPS...\n");
    gps_init();
    sleep_ms(200);
    printf("GPS inicializado\n");

    // BME680
    printf("Inicializando BME680...\n");
    struct bme680_dev sensor;
    uint16_t periodo_bme;
    bme680_inicializar(&sensor, &periodo_bme);
    float pressao_base = calibrar_pressao(&sensor, periodo_bme);
    printf("BME680 pronto - Pressão base: %.2f hPa\n", pressao_base);

    // MPU6500
    printf("Inicializando MPU6500...\n");
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    mpu6500_inicializar();
    
    uint8_t id, buf;
    i2c_write_blocking(I2C_PORT, MPU6500_ENDERECO, (uint8_t[]){0x75}, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6500_ENDERECO, &buf, 1, false);
    id = buf;
    
    if (id != 0x70 && id != 0x68) {
        printf("ERRO: MPU6500 não detectado (ID: 0x%02X)\n", id);
        return 1;
    }
    printf("MPU6500 detectado (ID: 0x%02X)\n", id);

    printf("Calibrando giroscópio...\n");
    calibra_giroscopio();
    printf("Calibrando acelerômetro...\n");
    calibra_aceleracao();

    float theta = 0.0, phi = 0.0;
    float accel_x = 0, accel_y = 0, accel_z = 0;
    absolute_time_t t_anterior = get_absolute_time();

    printf("\n=== SISTEMA PRONTO ===\n");
    printf("Aguardando fix GPS...\n\n");

    uint32_t contador = 0;
    uint32_t contador_captura = 0;  // Contador de capturas GPS válidas
    float altitude_bme = 0.0;
    float altitude_bme_anterior = 0.0;
    float pressao_atual = 0.0;
    
    // Variáveis para tempo contínuo
    uint32_t gps_time_offset = 0;
    absolute_time_t tempo_inicio = {0};
    bool tempo_inicializado = false;

    while (true) {
        contador++;
        absolute_time_t t_atual = get_absolute_time();
        float dt = absolute_time_diff_us(t_anterior, t_atual) / 1e6f;
        t_anterior = t_atual;
        
        // PRIORIDADE 1: Leitura MPU6500
        leitura(bias_giro, erro_aceleracao, &theta, &phi, dt);
        
        // TODO: Ler aceleração bruta do MPU6500 para fator de carga
        // Por enquanto usar theta/phi como proxy
        accel_z = cos(phi * 3.14159 / 180.0) * G_ACCEL;
        
        // PRIORIDADE 2: Leitura GPS múltiplas vezes
        for (int i = 0; i < 10; i++) {
            read_gps_data();
        }
        
        // PRIORIDADE 3: Leitura BME680
        if (contador % 5 == 0) {
            float alt_temp = 0;
            bme680_ler_altitude(&sensor, periodo_bme, pressao_base, 
                               &pressao_atual, &alt_temp);
            
            // Proteção: guardar último valor válido
            if (alt_temp > 0.1f) {
                altitude_bme = alt_temp;
                altitude_bme_anterior = alt_temp;
            } else if (contador > 100) {
                altitude_bme = altitude_bme_anterior;
            }
        }
        
        // Detecção de parada (altitude < 20cm)
        if (altitude_bme < 0.2f) {
            printf("STOP\n");
        }

        // Processar GPS se válido
        if (is_gps_valid()) {
            double zgps_raw = get_gps_z();
            
            // Iniciar captura apenas quando ZGPS > 0
            if (zgps_raw > 0) {
                contador_captura++;
                if (contador_captura == 1) {
                    printf("Iniciar captura\n");
                }
            }
            
            // Inicializar tempo na primeira leitura válida
            if (!tempo_inicializado) {
                gps_time_offset = get_gps_time_seconds();
                tempo_inicio = get_absolute_time();
                tempo_inicializado = true;
            }
            
            // Calcular tempo contínuo: tempo_gps_inicial + segundos decorridos no Pico
            uint32_t tempo_pico_ms = absolute_time_diff_us(tempo_inicio, t_atual) / 1000;
            uint32_t tempo_total = gps_time_offset + (tempo_pico_ms / 1000);
            
            double xgps_raw = get_gps_x();
            double ygps_raw = get_gps_y();
            // zgps_raw já foi declarado acima, não redeclarar
            
            // Filtro de média móvel
            gps_filter_add(xgps_raw, ygps_raw, zgps_raw);
            double xgps = 0, ygps = 0, zgps = 0;
            gps_filter_get_average(&xgps, &ygps, &zgps);
            
            // Atualizar dados HUD
            hud_data.gps_time = tempo_total;
            hud_data.latitude = xgps;
            hud_data.longitude = ygps;
            hud_data.altitude_gps = zgps;
            hud_data.gps_sats = get_gps_satellites();
            hud_data.altitude_bme = altitude_bme;
            hud_data.velocity_cas = calcular_cas(pressao_atual, pressao_base);
            hud_data.accel_z = accel_z;
            hud_data.theta = theta;
            hud_data.phi = phi;
            
            // Tempo decorrido desde o início (em segundos)
            hud_data.status = determinar_status(altitude_bme, hud_data.velocity_cas, tempo_total);
            
            // SAÍDA 1: Dados para HUD (sobreposição vídeo)
            enviar_hud(&hud_data);
            
            // SAÍDA 2: Dados brutos (arquivo/análise)
            salvar_dados_arquivo(xgps, ygps, zgps, theta, phi, tempo_total);
        }
        
        sleep_ms(20);
    }

    return 0;
}
