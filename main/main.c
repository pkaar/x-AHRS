#include "cpu.h"
#include "clock.h"
#include "delay.h"
#include "error.h"
#include "gpio.h"
#include "mkl25z128xxx4.h"

#include "task_ahrs.h"
#include "task_led.h"
#include "task_serial.h"

#include "FreeRTOS.h"
#include "task.h"

int main(void)
{
    /* Initialize clock module. The controller is running at 48 MHz. */
    clock_init();

    /* Block program execution to make sure all hardware components are powered
     * up and running. */
    delay_ms(100);

    gpio_init(PTB2, GPIO_OUTPUT, GPIO_FLOAT);

    /* Initialize all FreeRTOS tasks. */
    task_ahrs_init();
    task_led_init();
    task_serial_init();

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    while (1)
    {
    }

    return 0;
}
