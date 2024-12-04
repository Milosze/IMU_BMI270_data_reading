// Fragmenty ponizszego kodu moga byc objete jakas licencja

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi2.h"
#include "bmi270.h"
#include "common.h"

// Okreslana w m/s^2
#define GRAVITY_EARTH (9.80665f)

// Do oznaczania czujnikow
#define ACCEL UINT8_C(0)
#define GYRO UINT8_C(1)

// Do warunkow
#define TRUE 1

// Struktura na przeliczone dane z czujnukow
typedef struct
{
    float x;
    float y;
    float z;
} vector3d;

// Funkcja konfigurujaca czujniki (zwraca status)
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

// Funkcja przeliczajaca wartosci bezposrednio z czujnikow na m/s^2
// Przyjmuje jako argumenty wartosc surowa przyspieszenia i rozdzielczosc akcelerometru w bitach
// Zwraca wartosc przyspieszenia w m/s^2
static float raw_to_mps2(int16_t val, uint8_t bit_width);

// Funkcja przeliczajaca wartosci bezposrednio z czujnikow na m/s^2
// Przyjmuje jako argumenty wartosc surowa predkosci katowej i rozdzielczosc zyroskopu w bitach
// Zwraca wartosc predkosci katowej w stopniach na sekunde
static float raw_to_dps(int16_t val, uint8_t bit_width);

// Funkcja inicjalizujaca czujniki
// UWAGA: TA FUNKCJA MOZE UNIERUCHOMIC WYKONYWANIE PROGRAMU w przypadku niepowodzenia inicjalizacji
static void initialize_and_enable_accel_gyro(struct bmi2_dev *bmi);

// ----> Ta wlasciwa funkcja posrednio realizujaca czytanie danych z czujnikow <----
// Przyjmuje jako argumenty strukture z konfiguracja bmi270 oraz dwie struktury, w ktorych zapisze dane z pomiaru.
// Zwraca status
static int8_t read_sensor_data(struct bmi2_dev *bmi, vector3d *acc, vector3d *gyr);

void app_main()
{
    // Status
    int8_t rslt;

    // Do konfiguracji bmi270
    struct bmi2_dev bmi;

    vector3d acc;
    vector3d gyr;

    printf("##Inicjalizacja##\n");
    initialize_and_enable_accel_gyro(&bmi);
    printf("##Inicjalizacja zakonczona sukcesem##\n\n");

    printf("##Czytanie danych##\n\n");
    while(TRUE)
    {
        rslt = read_sensor_data(&bmi, &acc, &gyr);
        bmi2_error_codes_print_result(rslt);

        // Sprawdzenie, czy udalo sie zczytac dane
        if(rslt == BMI2_OK)
        {
            printf("##Poczatek zestawu danych##\n>Dane akceleromrtu:\nx: %4.2f\ny: %4.2f\nz: %4.2f\n",
                    acc.x, acc.y, acc.z);

            printf(">Dane zyroskopu:\nx: %4.2f\ny: %4.2f\nz: %4.2f\n##Koniec zestawu danych##\n\n",
                    gyr.x, gyr.y, gyr.z);
        }
        else
        {
            printf("!!Blad pomiaru (kod %d)!!\n\n", rslt)
        }

        // Opoznienie, by pomiary byly wykonywane nie czesciej niz 50ms
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

static int8_t read_sensor_data(struct bmi2_dev *bmi, vector3d *acc, vector3d *gyr)
{
    // Status
    int8_t rslt;

    // Struktura do odbierania danych z czujnikow
    struct bmi2_sens_data sensor_data = { { 0 } };

    // Zczytanie danych z czujnikow
    rslt = bmi2_get_sensor_data(&sensor_data, bmi);
    bmi2_error_codes_print_result(rslt);

    // Sprawdzenie, czy udalo sie zczytac dane
    if((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) && (sensor_data.status & BMI2_DRDY_GYR))
    {
        // Konwersja z surowych danych na m/s^2
        acc->x = raw_to_mps2(sensor_data.acc.x, bmi->resolution);
        acc->y = raw_to_mps2(sensor_data.acc.y, bmi->resolution);
        acc->z = raw_to_mps2(sensor_data.acc.z, bmi->resolution);

        // Konwersja z surowych danych na dps
        gyr->x = raw_to_dps(sensor_data.gyr.x, bmi->resolution);
        gyr->y = raw_to_dps(sensor_data.gyr.y, bmi->resolution);
        gyr->z = raw_to_dps(sensor_data.gyr.z, bmi->resolution);
    }

    return rslt;
}

static void initialize_and_enable_accel_gyro(struct bmi2_dev *bmi)
{
    // Status
    int8_t rslt;

    // Potrzebne do inicjalizacji
    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    // Inicjalizacja interfejsu
    do
    {
    rslt = bmi2_interface_init(bmi, BMI2_SPI_INTF); // BMI2_I2C_INTF dla I2C
    bmi2_error_codes_print_result(rslt);
    } while(rslt != BMI2_OK);

    // Inicjalizacja bmi270
    do
    {
    rslt = bmi270_init(bmi);
    bmi2_error_codes_print_result(rslt);
    } while(rslt != BMI2_OK);
    
    // Konfiguracja ustawien czujnikow
    do
    {
    rslt = set_accel_gyro_config(bmi);
    bmi2_error_codes_print_result(rslt);
    } while(rslt != BMI2_OK);
    
    // Aktywacja czujnikow
    do
    {
    rslt = bmi2_sensor_enable(sensor_list, 2, bmi);
    bmi2_error_codes_print_result(rslt);
    } while(rslt != BMI2_OK);
}

static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    // Status
    int8_t rslt;

    // Struktura do inicjalizacji akcelerometru i zyroskopu
    struct bmi2_sens_config config[2];

    // Wybranie czujnikow do konfiguracji
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    // Pobranie domyslnych konfiguracji
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

    // Nie wiem co tu sie wlasciwie dzieje
    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        // W TYM BLOKU NALEZY WYBRAC KONFIGURACJE

        // Czestotliwosc wyjscia danych
        // Dobralem nalblizej jak sie dalo jednego pomiaru na 50ms (nie wiem, czy mialem to zrobic)
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_25HZ;

        // Zakres mierzonego przyspierszenia (+/- 2G, 4G, 8G, 16G)
        // Jak patrzylem na parametry rakiet kola, to 8G jest chyba ok
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_8G;

        // Ilosc probek, z ktorych srednia sklada sie na pomiar
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG8;

        // 
        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        // Czestotliwosc wyjscia danych (ta dla zyroskopu)
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;

        // Zakres mierzonej predkosci katowej
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        // Ustawienie wybranych konfiguracji
        rslt = bmi2_set_sensor_config(config, 2, bmi);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

static float raw_to_mps2(int16_t val, uint8_t bit_width)
{
    double powerBasis = 2;
    float g_range = 8;
    float half_scale = (float)((pow(powerBasis, (double)bit_width) / 2.0f));
    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

static float raw_to_dps(int16_t val, uint8_t bit_width)
{
    double powerBasis = 2;
    float dps = 2000;
    float half_scale = (float)((pow((double)powerBasis, (double)bit_width) / 2.0f));
    return (dps / (half_scale)) * (val);
}