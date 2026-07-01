/** * @file main.c 
 * @brief REHAB IMU — Calibración robusta, beta adaptativo, contadores independientes
 *
 * Cambios principales:
 * 1. Calibración IMU: Comprueba la varianza del giroscopio. Si detecta movimiento 
 * descarta muestras, espera 1 s y reintenta (hasta 5 veces). Fallback a últimas lecturas.
 * 2. Calibración FSR: Comprueba varianza y valor absoluto. Reintenta si hay presión (hasta 3 veces).
 * 3. Beta adaptativo de Madgwick: En reposo baja a 0.033 para reducir el jitter. 
 * Al manipular, sube a 0.1 para mantener reactividad.
 * 4. Contadores de notificación independientes para IMU y FSR.
 *
 * COMPATIBILIDAD: imu_packet_t pasa a 20 bytes (4 floats de quaternion + 1 float de movimiento lineal).
 * Paquete FSR sigue siendo 4 bytes (int32_t). UUIDs y nombre BLE idénticos.
 */ 

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <math.h>
#include <string.h>

/* --- Parámetros de Madgwick y Beta Adaptativo --- */
/* MADGWICK_BETA_REST: Filtra el ruido del acelerómetro en reposo. */
#define MADGWICK_BETA_REST       0.033f
/* MADGWICK_BETA_MOVING: Reactividad alta para seguir rotaciones rápidas. */
#define MADGWICK_BETA_MOVING     0.1f
/* GYRO_MOTION_THRESHOLD: Por debajo (~8.6°/s) se considera reposo. */
#define GYRO_MOTION_THRESHOLD    0.15f
#define BETA_LERP_RATE           0.05f
#define ACCEL_BASELINE_LERP_RATE 0.02f
#define ACCEL_MOTION_LERP_RATE   0.60f
#define ACCEL_MOTION_DECAY_RATE  0.25f

/* --- Umbrales de Calibración --- */
/* GYRO_VARIANCE_LIMIT: Límite para considerar la IMU en reposo (< 0.01 rad²/s²). */
#define GYRO_VARIANCE_LIMIT      0.01f
#define CALIB_MAX_ATTEMPTS_IMU   5
#define CALIB_MAX_ATTEMPTS_FSR   3
/* FSR_CALIB_MAX_VARIANCE: Varianza máxima permitida para calibrar el FSR. */
#define FSR_CALIB_MAX_VARIANCE   100
/* FSR_CALIB_MAX_MEAN: Si el valor medio supera 200, se asume presión intencional. */
#define FSR_CALIB_MAX_MEAN       200

/* --- Configuración de Hardware --- */
#define BATTERY_READ_INTERVAL_MS 10000
#define VBAT_ENABLE_GPIO_NODE    DT_NODELABEL(gpio0)
#define VBAT_ENABLE_PIN          14

typedef enum { STATE_IDLE, STATE_CONNECTED } system_state_t;
static system_state_t current_state = STATE_IDLE; 
static struct bt_conn *current_conn = NULL; 

/* --- Definición de UUIDs BLE --- */
#define BT_UUID_CUSTOM_SERVICE_VAL BT_UUID_128_ENCODE(0x12345678,0x1234,0x5678,0x1234,0x56789abcdef0) 
#define BT_UUID_CUSTOM_IMU_CHAR_VAL BT_UUID_128_ENCODE(0x12345678,0x1234,0x5678,0x1234,0x56789abcdef1) 
#define BT_UUID_CUSTOM_FSR_CHAR_VAL BT_UUID_128_ENCODE(0x12345678,0x1234,0x5678,0x1234,0x56789abcdef2) 

/* --- Configuración ADC --- */
#define ADC_DEVICE_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1_2 
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID_FSR 0
#define ADC_CHANNEL_ID_BAT 1 

#ifndef NRF_SAADC_INPUT_AIN1
#define NRF_SAADC_INPUT_AIN1 2
#endif
#ifndef NRF_SAADC_INPUT_AIN7
#define NRF_SAADC_INPUT_AIN7 8
#endif 
#ifndef BT_DATA_TX_POWER_LEVEL
#define BT_DATA_TX_POWER_LEVEL 0x0A
#endif 

/* --- Estructuras de Datos --- */
typedef struct __attribute__((packed)) { 
    float q0, q1, q2, q3;
    float linear_motion;
} imu_packet_t;

typedef struct { 
    float q0, q1, q2, q3; 
    float beta; 
} madgwick_filter_t; 

/* --- Variables Globales --- */
static const struct device *adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
static const struct device *lsm6dsl_dev;
static imu_packet_t current_packet; 
static int32_t current_fsr_value = 0;
static int32_t fsr_offset = 0;
static uint8_t current_battery_level = 100;
static float accel_offset[3] = {0.0f}, gyro_offset[3] = {0.0f};
static bool is_imu_notify_enabled = false;
static bool is_fsr_notify_enabled = false;
static bool is_bat_notify_enabled = false;
static madgwick_filter_t madgwick_state = { .q0 = 1.0f, .beta = MADGWICK_BETA_MOVING };
static float accel_motion_filtered = 0.0f;
static float accel_baseline[3] = {0.0f, 0.0f, 0.0f};
static bool accel_baseline_ready = false;
static int16_t adc_sample_buffer[1]; 

/* --- Secuencias ADC --- */
static const struct adc_channel_cfg fsr_cfg = { 
    .gain = ADC_GAIN, .reference = ADC_REFERENCE, .acquisition_time = ADC_ACQUISITION_TIME, 
    .channel_id = ADC_CHANNEL_ID_FSR, .input_positive = NRF_SAADC_INPUT_AIN1 
};
static struct adc_sequence fsr_seq = { 
    .channels = BIT(ADC_CHANNEL_ID_FSR), .buffer = adc_sample_buffer, 
    .buffer_size = sizeof(adc_sample_buffer), .resolution = ADC_RESOLUTION 
};
static const struct adc_channel_cfg bat_cfg = { 
    .gain = ADC_GAIN_1_6, .reference = ADC_REFERENCE, .acquisition_time = ADC_ACQUISITION_TIME, 
    .channel_id = ADC_CHANNEL_ID_BAT, .input_positive = NRF_SAADC_INPUT_AIN7 
};
static struct adc_sequence bat_seq = { 
    .channels = BIT(ADC_CHANNEL_ID_BAT), .buffer = adc_sample_buffer, 
    .buffer_size = sizeof(adc_sample_buffer), .resolution = ADC_RESOLUTION 
}; 

/* --- Funciones Auxiliares --- */

/**
 * @brief Configura el modo de energía de la IMU.
 */
static void set_imu_power_mode(bool active) { 
    struct sensor_value odr = { .val1 = active ? 104 : 0, .val2 = 0 }; 
    sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr); 
    sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr); 
} 

/**
 * @brief Lee el valor del FSR aplicando el offset de calibración.
 */
static int32_t read_fsr_real(void) { 
    if (adc_read(adc_dev, &fsr_seq) != 0) return 0; 
    int32_t val = (int32_t)adc_sample_buffer[0]; 
    val = (val - fsr_offset);
    if (val < 0) val = 0;
    return val; 
} 

/**
 * @brief Lee el voltaje de la batería y lo convierte a porcentaje (0-100%).
 */
static void read_battery_voltage(void) { 
    if (adc_read(adc_dev, &bat_seq) == 0) { 
        int32_t val = adc_sample_buffer[0]; if(val<0) val=0;
        float mv = ((float)val * 3.6f / 4095.0f) * 2.96f; 
        int lvl = (int)((mv - 3.3f) * 100.0f / (4.2f - 3.3f)); 
        current_battery_level = (uint8_t)(lvl > 100 ? 100 : (lvl < 0 ? 0 : lvl)); 
    } 
} 

/**
 * @brief Algoritmo de Madgwick para 6-DoF.
 */
static void madgwick_update_6dof(madgwick_filter_t *st, float gx, float gy, float gz, float ax, float ay, float az, float dt) { 
    float s0, s1, s2, s3, q0, q1, q2, q3, norm;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    q0 = st->q0; q1 = st->q1; q2 = st->q2; q3 = st->q3; 
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz); qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy); 
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx); qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx); 
    
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) { 
        norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az + 0.0001f); ax *= norm; ay *= norm; az *= norm; 
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3; _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2; _8q1 = 8.0f * q1; _8q2 = 8.0f * q2; 
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3; 
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay; 
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az; 
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az; 
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay; 
        norm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
        qDot1 -= st->beta * s0 * norm; qDot2 -= st->beta * s1 * norm; qDot3 -= st->beta * s2 * norm; qDot4 -= st->beta * s3 * norm; 
    } 
    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt; 
    norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); 
    st->q0 = q0 * norm; st->q1 = q1 * norm; st->q2 = q2 * norm; st->q3 = q3 * norm; 
} 

/**
 * @brief Calibración inicial robusta de la IMU.
 * Evalúa la varianza en reposo. Si se detecta movimiento (ruido excesivo), reintenta.
 * @return true si se calibró en reposo limpio, false si se usaron las últimas lecturas (fallback).
 */
static bool calibrate_sensors_at_rest(void) { 
    struct sensor_value r[3];
    float t_g[3], t_a[3], sq_g[3];

    set_imu_power_mode(true);
    k_sleep(K_SECONDS(2));

    for (int attempt = 0; attempt < CALIB_MAX_ATTEMPTS_IMU; attempt++) {
        memset(t_g,  0, sizeof(t_g));
        memset(t_a,  0, sizeof(t_a));
        memset(sq_g, 0, sizeof(sq_g));

        for (int i = 0; i < 200; i++) {
            sensor_sample_fetch(lsm6dsl_dev);

            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, r);
            for (int ax = 0; ax < 3; ax++) {
                float v = (float)sensor_value_to_double(&r[ax]);
                t_g[ax]  += v;
                sq_g[ax] += v * v;
            }

            sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, r);
            for (int ax = 0; ax < 3; ax++) {
                t_a[ax] += (float)sensor_value_to_double(&r[ax]);
            }

            k_sleep(K_MSEC(5));
        }

        /* Varianza total del giroscopio (suma de los 3 ejes) */
        float gyro_var_sum = 0.0f;
        for (int ax = 0; ax < 3; ax++) {
            float mean = t_g[ax] / 200.0f;
            gyro_var_sum += (sq_g[ax] / 200.0f) - (mean * mean);
        }

        if (gyro_var_sum < GYRO_VARIANCE_LIMIT) {
            /* Reposo confirmado -> aplicar offsets */
            for (int ax = 0; ax < 3; ax++) {
                gyro_offset[ax]  = t_g[ax] / 200.0f;
                accel_offset[ax] = t_a[ax] / 200.0f;
            }
            accel_offset[2] -= 9.80665f;
            return true;
        }

        /* Movimiento detectado -> esperar y reintentar */
        k_sleep(K_SECONDS(1));
    }

    /* Fallback: usar las últimas lecturas si se agotaron los intentos */
    for (int ax = 0; ax < 3; ax++) {
        gyro_offset[ax]  = t_g[ax] / 200.0f;
        accel_offset[ax] = t_a[ax] / 200.0f;
    }
    accel_offset[2] -= 9.80665f;
    return false;
}

/**
 * @brief Calibración de FSR. Evita offsets erróneos si hay presión aplicada al encender.
 */
static void calibrate_fsr(void) {
    long s = 0;

    for (int attempt = 0; attempt < CALIB_MAX_ATTEMPTS_FSR; attempt++) {
        s = 0;
        long s_sq = 0;

        for (int i = 0; i < 50; i++) {
            adc_read(adc_dev, &fsr_seq);
            int32_t v = (int32_t)adc_sample_buffer[0];
            s    += v;
            s_sq += (long)v * v;
            k_sleep(K_MSEC(10));
        }

        long mean = s / 50;
        long var  = (s_sq / 50) - (mean * mean);

        if (var < FSR_CALIB_MAX_VARIANCE && mean < FSR_CALIB_MAX_MEAN) {
            break; /* Offset válido, sin presión */
        }

        k_sleep(K_MSEC(500));
    }

    fsr_offset = (int32_t)(s / 50);
}

/* --- Callbacks GATT y Definición de Servicios BLE --- */
static ssize_t read_imu(struct bt_conn *c, const struct bt_gatt_attr *a, void *b, uint16_t l, uint16_t o) { return bt_gatt_attr_read(c, a, b, l, o, &current_packet, sizeof(current_packet)); } 
static void imu_ccc(const struct bt_gatt_attr *attr, uint16_t val) { is_imu_notify_enabled = (val == BT_GATT_CCC_NOTIFY); } 
static ssize_t read_fsr(struct bt_conn *c, const struct bt_gatt_attr *a, void *b, uint16_t l, uint16_t o) { return bt_gatt_attr_read(c, a, b, l, o, &current_fsr_value, sizeof(current_fsr_value)); } 
static void fsr_ccc(const struct bt_gatt_attr *attr, uint16_t val) { is_fsr_notify_enabled = (val == BT_GATT_CCC_NOTIFY); } 
static ssize_t read_bat(struct bt_conn *c, const struct bt_gatt_attr *a, void *b, uint16_t l, uint16_t o) { return bt_gatt_attr_read(c, a, b, l, o, &current_battery_level, sizeof(current_battery_level)); } 
static void bat_ccc(const struct bt_gatt_attr *attr, uint16_t val) { is_bat_notify_enabled = (val == BT_GATT_CCC_NOTIFY); } 

BT_GATT_SERVICE_DEFINE(custom_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE_VAL)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_CUSTOM_IMU_CHAR_VAL), BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_imu, NULL, &current_packet), BT_GATT_CCC(imu_ccc, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), 
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_CUSTOM_FSR_CHAR_VAL), BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_fsr, NULL, &current_fsr_value), BT_GATT_CCC(fsr_ccc, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE) 
); 
BT_GATT_SERVICE_DEFINE(batt_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS), BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_bat, NULL, &current_battery_level), BT_GATT_CCC(bat_ccc, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE) ); 

static const struct bt_data ad[] = { BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)), BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL), BT_DATA_BYTES(BT_DATA_TX_POWER_LEVEL, 0x00) }; 
static const struct bt_data sd[] = { BT_DATA(BT_DATA_NAME_COMPLETE, "RehabBlue", 10) }; 
static const struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL); 

static void connected(struct bt_conn *c, uint8_t e) { 
    if(!e) { current_state = STATE_CONNECTED; if(!current_conn) current_conn = bt_conn_ref(c); set_imu_power_mode(true); } 
} 
static void disconnected(struct bt_conn *c, uint8_t r) { 
    set_imu_power_mode(false); if(current_conn) { bt_conn_unref(current_conn); current_conn = NULL; } sys_reboot(SYS_REBOOT_COLD); 
} 
BT_CONN_CB_DEFINE(cb) = { .connected = connected, .disconnected = disconnected }; 

/* --- Loop Principal --- */
int main(void) { 
    /* Inicialización de periféricos */
    lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl); 
    const struct device *gpio_dev = DEVICE_DT_GET(VBAT_ENABLE_GPIO_NODE); 
    
    if (device_is_ready(gpio_dev)) {
        gpio_pin_configure(gpio_dev, VBAT_ENABLE_PIN, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_LOW);
        gpio_pin_set(gpio_dev, VBAT_ENABLE_PIN, 1);
    } 

    if (device_is_ready(adc_dev)) {
        adc_channel_setup(adc_dev, &fsr_cfg);
        adc_channel_setup(adc_dev, &bat_cfg);
        calibrate_fsr();
    } 

    if (device_is_ready(lsm6dsl_dev)) {
        calibrate_sensors_at_rest();
        set_imu_power_mode(false);
    } 

    /* Inicialización de Bluetooth */
    bt_enable(NULL);
    bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd)); 
    
    struct sensor_value a[3], g[3];
    int64_t last_bat = 0, last_time = k_uptime_get();

    /* Contadores para controlar la frecuencia de notificación independiente */
    int imu_notify_cnt = 0;
    int fsr_notify_cnt = 0;

    while (1) { 
        int64_t now = k_uptime_get();
        float dt = (float)(now - last_time) / 1000.0f;
        last_time = now; 
        
        if (dt <= 0) dt = 0.001f; 
        if (dt > 0.1f) dt = 0.1f; 
        
        if (current_state == STATE_CONNECTED) { 

            /* --- Monitor de Batería --- */
            if (now - last_bat > BATTERY_READ_INTERVAL_MS) {
                read_battery_voltage();
                if (is_bat_notify_enabled)
                    bt_gatt_notify(NULL, &batt_svc.attrs[1], &current_battery_level, 1);
                last_bat = now;
            }

            /* --- Lectura FSR --- */
            current_fsr_value = read_fsr_real(); 

            /* --- Lectura IMU y actualización Madgwick --- */
            if (sensor_sample_fetch(lsm6dsl_dev) == 0
                && sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, a) == 0
                && sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, g) == 0)
            { 
                float gx = (float)sensor_value_to_double(&g[0]) - gyro_offset[0];
                float gy = (float)sensor_value_to_double(&g[1]) - gyro_offset[1];
                float gz = (float)sensor_value_to_double(&g[2]) - gyro_offset[2]; 
                float ax = (float)sensor_value_to_double(&a[0]) - accel_offset[0];
                float ay = (float)sensor_value_to_double(&a[1]) - accel_offset[1];
                float az = (float)sensor_value_to_double(&a[2]) - accel_offset[2]; 

                /* Estimación de movimiento lineal (sin gravedad) */
                if (!accel_baseline_ready) {
                    accel_baseline[0] = ax;
                    accel_baseline[1] = ay;
                    accel_baseline[2] = az;
                    accel_baseline_ready = true;
                } else {
                    accel_baseline[0] += (ax - accel_baseline[0]) * ACCEL_BASELINE_LERP_RATE;
                    accel_baseline[1] += (ay - accel_baseline[1]) * ACCEL_BASELINE_LERP_RATE;
                    accel_baseline[2] += (az - accel_baseline[2]) * ACCEL_BASELINE_LERP_RATE;
                }

                float dax = ax - accel_baseline[0];
                float day = ay - accel_baseline[1];
                float daz = az - accel_baseline[2];
                float accel_motion = sqrtf(dax*dax + day*day + daz*daz);
                float lerp = (accel_motion > accel_motion_filtered) ? ACCEL_MOTION_LERP_RATE : ACCEL_MOTION_DECAY_RATE;
                accel_motion_filtered += (accel_motion - accel_motion_filtered) * lerp;
                current_packet.linear_motion = accel_motion_filtered;

                /* Ajuste de beta dinámico según magnitud de rotación */
                float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
                float target_beta = (gyro_mag < GYRO_MOTION_THRESHOLD)
                                  ? MADGWICK_BETA_REST
                                  : MADGWICK_BETA_MOVING;
                madgwick_state.beta += (target_beta - madgwick_state.beta) * BETA_LERP_RATE;

                madgwick_update_6dof(&madgwick_state, gx, gy, gz, ax, ay, az, dt); 

                /* Normalizar el cuaternión saliente para el paquete BLE */
                float qN = sqrtf(madgwick_state.q0*madgwick_state.q0
                               + madgwick_state.q1*madgwick_state.q1
                               + madgwick_state.q2*madgwick_state.q2
                               + madgwick_state.q3*madgwick_state.q3); 
                if (qN > 0.0f) { 
                    current_packet.q0 = madgwick_state.q0 / qN;
                    current_packet.q1 = madgwick_state.q1 / qN;
                    current_packet.q2 = madgwick_state.q2 / qN;
                    current_packet.q3 = madgwick_state.q3 / qN; 
                } 

                /* --- Despacho de notificaciones BLE --- */
                imu_notify_cnt++;
                fsr_notify_cnt++;

                /* Envía paquete IMU (~33 Hz con loop de 10ms) */
                if (is_imu_notify_enabled && (imu_notify_cnt % 3 == 0))
                    bt_gatt_notify(NULL, &custom_svc.attrs[2],
                                   &current_packet, sizeof(current_packet));

                /* Envía paquete FSR (~10 Hz con loop de 10ms) */
                if (is_fsr_notify_enabled && (fsr_notify_cnt % 10 == 0))
                    bt_gatt_notify(NULL, &custom_svc.attrs[5],
                                   &current_fsr_value, 4);
            }

            k_sleep(K_MSEC(10)); 
        } else {
            k_sleep(K_SECONDS(1));
        }
    }

    return 0; 
}