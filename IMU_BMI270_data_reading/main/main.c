// Some parts of the following code might or might not be covered by a license

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi2.h"
#include "bmi270.h"
#include "common.h"

#define GRAVITY_EARTH (9.80665f)

#define ACCEL UINT8_C(0)
#define GYRO UINT8_C(1)

#define TRUE 1

typedef struct
{
    float x;
    float y;
    float z;
} vector3d;

static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

// Converts raw sensor data to meters per second squared
static float raw_to_mps2(int16_t val, uint8_t bit_width);

// Converts raw sensor data to degrees per second
static float raw_to_dps(int16_t val, uint8_t bit_width);

static int8_t initialize_and_enable_accel_gyro(struct bmi2_dev *bmi);

// ----> Sensor data reading function <----
// Accelerometer data stored in acc and gyroscope date stored in gyr
static int8_t read_sensor_data(struct bmi2_dev *bmi, vector3d *acc, vector3d *gyr);

void app_main()
{
    int8_t rslt;

    // Structure for bmi270 handling
    struct bmi2_dev bmi;

    vector3d accel_data;
    vector3d gyro_data;

    printf("##Inicjalizacja##\n");

    do
    {
        rslt = initialize_and_enable_accel_gyro(&bmi);
        if(rslt != BMI2_OK) bmi2_coines_deinit();
    } while(rslt != BMI2_OK);

    printf("##Inicjalizacja zakonczona sukcesem##\n\n");

    printf("##Czytanie danych##\n\n");

    while(TRUE)
    {
        rslt = read_sensor_data(&bmi, &accel_data, &gyro_data);
        bmi2_error_codes_print_result(rslt);

        if(rslt == BMI2_OK)
        {
            printf("##Poczatek zestawu danych##\n>Dane akceleromrtu:\nx: %4.2f\ny: %4.2f\nz: %4.2f\n",
                    accel_data.x, accel_data.y, accel_data.z);

            printf(">Dane zyroskopu:\nx: %4.2f\ny: %4.2f\nz: %4.2f\n##Koniec zestawu danych##\n\n",
                    gyro_data.x, gyro_data.y, gyro_data.z);
        }
        else
        {
            printf("!!Blad pomiaru (kod %d)!!\n\n", rslt)
        }

        // 50ms delay between data sets
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

static int8_t read_sensor_data(struct bmi2_dev *bmi, vector3d *acc, vector3d *gyr)
{
    int8_t rslt;

    // Structure for raw sensor data
    struct bmi2_sens_data sensor_data = { { 0 } };

    rslt = bmi2_get_sensor_data(&sensor_data, bmi);
    bmi2_error_codes_print_result(rslt);

    if((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) && (sensor_data.status & BMI2_DRDY_GYR))
    {
        acc->x = raw_to_mps2(sensor_data.acc.x, bmi->resolution);
        acc->y = raw_to_mps2(sensor_data.acc.y, bmi->resolution);
        acc->z = raw_to_mps2(sensor_data.acc.z, bmi->resolution);

        gyr->x = raw_to_dps(sensor_data.gyr.x, bmi->resolution);
        gyr->y = raw_to_dps(sensor_data.gyr.y, bmi->resolution);
        gyr->z = raw_to_dps(sensor_data.gyr.z, bmi->resolution);
    }

    return rslt;
}

static int8_t initialize_and_enable_accel_gyro(struct bmi2_dev *bmi)
{
    int8_t rslt;

    // List of sensors whitch will be used
    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    // SPI interface init
    rslt = bmi2_interface_init(bmi, BMI2_SPI_INTF); // BMI2_I2C_INTF dla I2C
    bmi2_error_codes_print_result(rslt);
    if(rslt != BMI2_OK) return rslt;

    // BMI270 init
    rslt = bmi270_init(bmi);
    bmi2_error_codes_print_result(rslt);
    if(rslt != BMI2_OK) return rslt;
    
    // Sensors configuration
    rslt = set_accel_gyro_config(bmi);
    bmi2_error_codes_print_result(rslt);
    if(rslt != BMI2_OK) return rslt;
    
    // Sensors enabling
    rslt = bmi2_sensor_enable(sensor_list, 2, bmi);
    bmi2_error_codes_print_result(rslt);
    if(rslt != BMI2_OK) return rslt;

    return BMI2_OK;
}

static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    int8_t rslt;

    // Structure for accelerometer and gyroscope configurations
    struct bmi2_sens_config config[2];

    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    // Storing default configurations in config
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        // DEFAULT CONFIGURATIONS MIGHT BE CHNGED HERE

        // Accelerometer data output rate
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_25HZ;

        // Acceleration range (+/- 2G, 4G, 8G, 16G)
        // I think 8G will be enough
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_8G;

        // Samples number for average
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG8;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        // Gyroscope data output rate
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;

        // Angular velocity range
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