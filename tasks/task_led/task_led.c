/*******************************************************************************
 * task_led.c
 *
 * Description:
 *  This module initializes LED functionality to indicate device states. The
 *  module uses standard GPIOs to control LEDs. A low priority task performs
 *  LED control according the current task state.
 *
 * Functions:
 *  error_t task_led_init(void);
 *  error_t task_led_state(uint8_t state);
 *
 * Note:
 *  This module was intended to be used on Kinetis MKL25Z128 microcontrollers.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/

#include "task_led.h"

#include "gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#define LED_STATE       PTE24
#define LED_POWER       PTE25

#define LED_ON_OFF_PERIOD_MS    (125 / portTICK_PERIOD_MS)

volatile uint8_t led_state = TASK_LED_STATE_PAUSE;

static void task_led(void* pvParameters);

/*******************************************************************************
 * error_t task_led_init(void)
 *
 * Description:
 *  Initialize LED task. The LED task is executed with the lowest priority.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *
 * Example:
 *  error = task_led_init();
 *
 * Note:
 *  Lowest priority is defined as priority 1. The idle task is executed with a
 *  priority of 0. Therefore the LED task and the idle task are not supposed to
 *  share CPU time. The LED task is always executed before the idle task.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
error_t task_led_init(void)
{
    /* Initialize GPIO pin PTD5 and PTD7 as low outputs. */
    gpio_init(LED_STATE, GPIO_OUTPUT, GPIO_LOW);
    gpio_init(LED_POWER, GPIO_OUTPUT, GPIO_HIGH);

    /* Create LED task with lowest priority. */
    xTaskCreate(task_led, "task_led", configMINIMAL_STACK_SIZE, (void*) NULL,
                (tskIDLE_PRIORITY + 1), (xTaskHandle*) NULL);

    return ERR_OK;
}


/*******************************************************************************
 * error_t task_led_state(uint8_t state)
 *
 * Description:
 *  Set state of LED task. Possible errors are reported to the calling
 *  function.
 *
 * Parameters:
 *  state       LED task state
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - invalid LED task state
 *
 * Example:
 *  error = task_led_state(TASK_LED_STATE_BLINK);
 *
 * Note:
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
error_t task_led_state(uint8_t state)
{
    if ((state != TASK_LED_STATE_BLINK) && (state != TASK_LED_STATE_PAUSE))
    {
        return ERR_FAIL;
    }

    led_state = state;

    return ERR_OK;
}


/*******************************************************************************
 * void task_led(void* pvParameters)
 *
 * Description:
 *  This task manages LED control. LEDs are used to indicate the current state
 *  of the device. Two LEDs are used to indicate RUNNING, PAUSE and ERROR state
 *  of the device. RUNNING state is indicated by a blinking LED (LED_STATE),
 *  PAUSE is indicated by turned off LEDs and ERROR state is indicated by a
 *  turned on LED (LED_ERROR).
 *
 * Parameters:
 *  pvParameters    parameter for the task
 *
 * Return:
 *  none
 *
 * Example:
 *  This method is not supposed to be called but used as an argument when
 *  creating the actual task.
 *
 * Note:
 *  This method is not available outside this module.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
void task_led(void* pvParameters)
{
    TickType_t xLastWakeTime;

    /* Initialize xLastWakeTime with the current system time. */
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* Check LED task state. */
        switch (led_state)
        {
            case (TASK_LED_STATE_BLINK):
                gpio_set(LED_STATE, GPIO_HIGH);
                vTaskDelayUntil(&xLastWakeTime, LED_ON_OFF_PERIOD_MS);

                gpio_set(LED_STATE, GPIO_LOW);
                vTaskDelayUntil(&xLastWakeTime, LED_ON_OFF_PERIOD_MS);
                break;

            case (TASK_LED_STATE_PAUSE):
                gpio_set(LED_STATE, GPIO_LOW);
                vTaskDelayUntil(&xLastWakeTime, LED_ON_OFF_PERIOD_MS);
                break;
        }
    }
}
