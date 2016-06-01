/*
 * task_ahrs.c
 *
 *  Created on: Oct 5, 2015
 *      Author: pkaar
 */

#include "task_ahrs.h"

#include "gpio.h"
#include "hmc5883l.h"
#include "madgwick.h"
#include "mpu6050.h"
#include "quat.h"
#include "types.h"

#include "uart.h"

#include "FreeRTOS.h"
#include "task.h"

#define AHRS_SEND_QUATERNION        1

#if (AHRS_SEND_QUATERNION == 1)
#define MPU6050_SAMPLE_PERIOD_MS    4
#define HMC5883L_SAMPLE_PERIOD_MS   20
#define BLUETOOTH_SAMPLE_PERIOD_MS  20
#else
#define MPU6050_SAMPLE_PERIOD_MS    20
#define HMC5883L_SAMPLE_PERIOD_MS   20
#define BLUETOOTH_SAMPLE_PERIOD_MS  20
#endif

#define HMC5883L_DOWNSAMPLE_RATE    (HMC5883L_SAMPLE_PERIOD_MS / MPU6050_SAMPLE_PERIOD_MS)
#define BLUETOOTH_DOWNSAMPLE_RATE   (BLUETOOTH_SAMPLE_PERIOD_MS / MPU6050_SAMPLE_PERIOD_MS)

#define TASK_AHRS_PERIOD_MS         (MPU6050_SAMPLE_PERIOD_MS / portTICK_PERIOD_MS)
#define TASK_AHRS_FRQ_HZ            (1000 / MPU6050_SAMPLE_PERIOD_MS)

volatile uint8_t ahrs_state = TASK_AHRS_STATE_PAUSE;

static void task_ahrs(void* pvParameters);

error_t task_ahrs_init(void)
{
    /* Initialize GPIO pin PTD3 as floating input. */
    gpio_init(PTD3, GPIO_INPUT, GPIO_FLOAT);

    /* Initialize HMC5883L and MPU6050. */
    hmc5883l_init();
    mpu6050_init();

    /* Initialize Madgwick algorithm. */
    madgwick_init(TASK_AHRS_FRQ_HZ, 5.0f, 0.25f, 0.01f);

    //uart_init(115200);

    /* Create AHRS task with stack size of 256 bytes and maximum priority. */
    xTaskCreate(task_ahrs, "task_ahrs", 256, (void*) NULL,
                (configMAX_PRIORITIES - 1), (xTaskHandle*) NULL);

    return ERR_OK;
}


error_t task_ahrs_state(uint8_t state)
{
    if ((state != TASK_AHRS_STATE_START) && (state != TASK_AHRS_STATE_PAUSE))
    {
        return ERR_FAIL;
    }

    ahrs_state = state;

    return ERR_OK;
}


static void task_ahrs(void* pvParameters)
{
    uint8_t hmc5883l_downsample_cnt;
    uint8_t bluetooth_downsample_cnt;
    uint8_t pushbutton_level;

    vector_float_t acceleration;
    vector_float_t angular_velocity;
    vector_float_t magnetic_field;

    quat_t quat;
    quat_t quat_reference;

    TickType_t xLastWakeTime;

    hmc5883l_downsample_cnt = 0;
    bluetooth_downsample_cnt = 0;

    /* Set default reference position. */
    quat_reference.w = 1.0;
    quat_reference.x = 0.0;
    quat_reference.y = 0.0;
    quat_reference.z = 0.0;

    /* Initialize xLastWakeTime with the current system time. */
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        switch (ahrs_state)
        {
            case (TASK_AHRS_STATE_START):
                /* Read calibrated MPU6050 data. */
                mpu6050_data_read(&acceleration, & angular_velocity);

                /* Check if HMC5883L data needs to be read. */
                if (!hmc5883l_downsample_cnt)
                {
                    /* Read calibrated HMC5883L data. */
                    hmc5883l_data_read(&magnetic_field);
                }

                /* Increment downsample count. */
                hmc5883l_downsample_cnt = (hmc5883l_downsample_cnt + 1) %
                        HMC5883L_DOWNSAMPLE_RATE;

                /* Calculate orientation using acceleration, angular velocity
                 * and magnetic field data. */
                madgwick_update(acceleration, angular_velocity, magnetic_field,
                                &quat);

                /* Get pushbutton level. */
                gpio_get(PTD3, &pushbutton_level);

                /* Check pushbutton level. Orientation reference position has
                 * to be updated if pushbutton is pressed. */
                if (pushbutton_level == GPIO_HIGH)
                {
                    quat_reference = quat_conj(quat);
                }

                /* Calculate orientation according to reference position. */
                quat = quat_mult(quat, quat_reference);

                /* Check if x-AHRS data needs to be transmitted. */
                if (!bluetooth_downsample_cnt)
                {
#if (AHRS_SEND_QUATERNION == 1)
                    uart_transmit((uint8_t*) &quat, sizeof(quat));
#else
                    uart_transmit((uint8_t*) &acceleration, sizeof(acceleration));
                    uart_transmit((uint8_t*) &angular_velocity, sizeof(angular_velocity));
                    uart_transmit((uint8_t*) &magnetic_field, sizeof(magnetic_field));
#endif
                }

                /* Increment downsample count. */
                bluetooth_downsample_cnt = (bluetooth_downsample_cnt + 1) %
                        BLUETOOTH_DOWNSAMPLE_RATE;
                break;

            case (TASK_AHRS_STATE_PAUSE):
                /* Reset Madgwick AHRS algorithm and set orientation and
                 * algorithm gain are to default values. */
                madgwick_reset();

                /* Set default reference position. */
                quat_reference.w = 1.0;
                quat_reference.x = 0.0;
                quat_reference.y = 0.0;
                quat_reference.z = 0.0;
                break;
        }

        /* Suspend task until time delay has passed. This allows lower priority
         * tasks to be executed. */
        vTaskDelayUntil(&xLastWakeTime, TASK_AHRS_PERIOD_MS);
    }
}
