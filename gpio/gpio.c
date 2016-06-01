/*******************************************************************************
 * gpio.c
 *
 * Description:
 *  This module initializes Kinetis MKL25Z128 GPIO module. Each GPIO pin can be
 *  initialized as logical input or output. Logical pin levels can be set and
 *  read.
 *
 * Functions:
 *  error_t gpio_init(uint8_t pin_id, uint8_t mode, uint8_t level)
 *  error_t gpio_set(uint8_t pin_id, uint8_t level)
 *  error_t gpio_get(uint8_t pin_id, uint8_t* level)
 *
 * Note:
 *  This module is intended to be used on Freescales Kinetis MKL25Z128
 *  processores.
 *
 * History:
 *  pka, 31/JUL/2015, initial code
 ******************************************************************************/

#include "gpio.h"

#define PORTA       0
#define PORTB       1
#define PORTC       2
#define PORTD       3
#define PORTE       4

static uint32_t gpio[5];

/*******************************************************************************
 * error_t gpio_init(uint8_t pin_id, uint8_t mode, uint8_t level)
 *
 * Description:
 *  Initialize specified GPIO pin. The pin can be initialized to operate in
 *  input or output mode. When initialized as input, the pull mode has to be
 *  specified. Valid pull modes are pullup or pulldown. When initialized as
 *  output, the initial logical level has to be specified. Valid logical levels
 *  are logical low or logical high.
 *
 * Parameters:
 *  pin_id      GPIO pin to be initialized
 *  mode        operation mode
 *  level       pull mode when input, initial logical level when output
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_GPIO_PIN            - invalid GPIO pin
 *              ERR_GPIO_MODE           - invalid GPIO operation mode
 *              ERR_GPIO_PULLMODE       - invalid GPIO pull mode
 *              ERR_GPIO_LEVEL          - invalid GPIO logical level
 *
 * Example:
 *  error = gpio_init(PTA0, OUTPUT, LOW);
 *
 * Note:
 *  This module does not provide GPIO interrupt functionality.
 *
 * History:
 *  pka, 31/JUL/2015, initial code
 ******************************************************************************/
error_t gpio_init(uint8_t pin_id, uint8_t mode, uint8_t level)
{
    uint8_t port;                   /* port number */
    uint8_t pin;                    /* pin number */

    PORT_MemMapPtr port_base_ptr;   /* port peripheral instance base address */
    GPIO_MemMapPtr pt_base_ptr;     /* GPIO peripheral instance base address */

    /* Check for valid pin identifier. */
    if ((pin_id < 0x00) || (pin_id > 0x9F))
    {
        return ERR_GPIO_PIN;
    }

    /* Check if pin mode is GPIO_INPUT. */
    if (mode == GPIO_INPUT)
    {
        /* In input mode only GPIO_FLOAT and GPIO_PULLUP are valid. */
        if ((level != GPIO_FLOAT) && (level != GPIO_PULLUP))
        {
            return ERR_GPIO_PULLMODE;
        }
    }

    /* Check if pin mode is GPIO_OUTPUT. */
    else if (mode == GPIO_OUTPUT)
    {
        /* In output mode only GPIO_LOW and GPIO_HIGH are valid. */
        if ((level != GPIO_LOW) && (level != GPIO_HIGH))
        {
            return ERR_GPIO_LEVEL;
        }
    }

    /* Invalid pin mode. */
    else
    {
        return ERR_GPIO_MODE;
    }

    /* Get port number, which is stored in bits 7-5 of pin identifier. */
    port = pin_id >> 5;

    /* Get pin number, which is stored in bits 4-0 of pin identifier. */
    pin = pin_id & 0x1F;

    /* Clear initialization flag for selected pin. */
    gpio[port] &= ~(1 << pin);

    /* Check which port was selected. */
    switch (port)
    {
        case PORTA:
            /* Check if clock was already activated. */
            if (!(SIM_SCGC5 & SIM_SCGC5_PORTA_MASK))
            {
                /* Activate port clock. */
                SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
            }

            /* Set port base address pointer according to corresponding port. */
            port_base_ptr = PORTA_BASE_PTR;

            /* Set GPIO base address pointer according to corresponding port. */
            pt_base_ptr = PTA_BASE_PTR;
            break;

        case PORTB:
            if (!(SIM_SCGC5 & SIM_SCGC5_PORTB_MASK))
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
            }

            port_base_ptr = PORTB_BASE_PTR;

            pt_base_ptr = PTB_BASE_PTR;
            break;

        case PORTC:
            if (!(SIM_SCGC5 & SIM_SCGC5_PORTC_MASK))
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
            }

            port_base_ptr = PORTC_BASE_PTR;

            pt_base_ptr = PTC_BASE_PTR;
            break;

        case PORTD:
            if (!(SIM_SCGC5 & SIM_SCGC5_PORTD_MASK))
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
            }

            port_base_ptr = PORTD_BASE_PTR;

            pt_base_ptr = PTD_BASE_PTR;
            break;

        case PORTE:
            if (!(SIM_SCGC5 & SIM_SCGC5_PORTE_MASK))
            {
                SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
            }

            port_base_ptr = PORTE_BASE_PTR;

            pt_base_ptr = PTE_BASE_PTR;
            break;

        default:
            return ERR_GPIO_PIN;
    }

    /* Set pin to input. This corresponds to the reset value of each pin. */
    GPIO_PDDR_REG(pt_base_ptr) &= ~ GPIO_PIN(pin);

    /* Clear all relevant bits in PORTx_PCRn register before writing to them, in
     * order to avoid errors caused by incorrect bit values. */
    PORT_PCR_REG(port_base_ptr, pin) &= ~(PORT_PCR_MUX(0) |
            PORT_PCR_DSE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);

    /* Check which mode of operation was selected. */
    switch (mode)
    {
        case GPIO_INPUT:
            /* Check which pull mode was selected. */
            switch (level)
            {
                case GPIO_FLOAT:
                    /* Set pin multiplexing mode to GPIO and disable internal
                     * pull resistor. */
                    PORT_PCR_REG(port_base_ptr, pin) |= PORT_PCR_MUX(1);

                    /* Set pin to input. */
                    GPIO_PDDR_REG(pt_base_ptr) &= ~ GPIO_PIN(pin);
                    break;

                case GPIO_PULLUP:
                    /* Set pin multiplexing mode to GPIO and enable internal
                     * pullup resistor. */
                    PORT_PCR_REG(port_base_ptr, pin) |= PORT_PCR_MUX(1) |
                            PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

                    /* Set pin to input. */
                    GPIO_PDDR_REG(pt_base_ptr) &= ~ GPIO_PIN(pin);
                    break;

                default:
                    return ERR_GPIO_PULLMODE;
            }

            break;

        case GPIO_OUTPUT:
            /* Check which pin level was selected. */
            switch (level)
            {
                case GPIO_LOW:
                    /* Set pin multiplexing mode to GPIO and enable high drive
                     * strength. */
                    PORT_PCR_REG(port_base_ptr, pin) |= PORT_PCR_MUX(1) |
                            PORT_PCR_DSE_MASK;

                    /* Set pin level to low. */
                    GPIO_PDOR_REG(pt_base_ptr) &= ~GPIO_PIN(pin);

                    /* Set pin to output. */
                    GPIO_PDDR_REG(pt_base_ptr) |= GPIO_PIN(pin);
                    break;

                case GPIO_HIGH:
                    /* Set pin multiplexing mode to GPIO and enable high drive
                     * strength. */
                    PORT_PCR_REG(port_base_ptr, pin) |= PORT_PCR_MUX(1) |
                            PORT_PCR_DSE_MASK;

                    /* Set pin level to low. */
                    GPIO_PDOR_REG(pt_base_ptr) |= GPIO_PIN(pin);

                    /* Set pin to output. */
                    GPIO_PDDR_REG(pt_base_ptr) |= GPIO_PIN(pin);
                    break;

                default:
                    return ERR_GPIO_LEVEL;
            }

            break;

        default:
            return ERR_GPIO_MODE;
    }

    /* Set initialization flag for selected pin, indicating the initialization
     * procedure was successfully completed. */
    gpio[port] |= (1 << pin);

    return ERR_OK;
}


/*******************************************************************************
 * error_t gpio_set(uint8_t pin_id, uint8_t level)
 *
 * Description:
 *  Set logical level of specified pin. Setting logical levels is only possible
 *  for initialized input and output pins, whereby setting levels for input pins
 *  has no effect.
 *
 * Parameters:
 *  pin_id      GPIO pin
 *  level       logical level
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_GPIO_PIN            - invalid GPIO pin
 *              ERR_GPIO_INIT           - access to uninitialized GPIO pin
 *              ERR_GPIO_LEVEL          - invalid GPIO logical level
 *
 * Example:
 *  error = gpio_set(PTA0, HIGH);
 *
 * Note:
 *  This module does not provide GPIO interrupt functionality.
 *
 * History:
 *  pka, 31/JUL/2015, initial code
 ******************************************************************************/
error_t gpio_set(uint8_t pin_id, uint8_t level)
{
    uint8_t port;                   /* port number */
    uint8_t pin;                    /* pin number */

    GPIO_MemMapPtr pt_base_ptr;     /* GPIO peripheral instance base address */

    /* Check for valid pin identifier. */
    if ((pin_id < 0x00) || (pin_id > 0x9F))
    {
        return ERR_GPIO_PIN;
    }

    /* Get port number, which is stored in bits 7-5 of pin identifier. */
    port = pin_id >> 5;

    /* Get pin number, which is stored in bits 4-0 of pin identifier. */
    pin = pin_id & 0x1F;

    /* Check if pin is initialized. */
    if (!(gpio[port] & (1 << pin)))
    {
        return ERR_GPIO_INIT;
    }

    /* Check which port was selected. */
    switch (port)
    {
        case PORTA:
            /* Set GPIO base address pointer according to corresponding port. */
            pt_base_ptr = PTA_BASE_PTR;
            break;

        case PORTB:
            pt_base_ptr = PTB_BASE_PTR;
            break;

        case PORTC:
            pt_base_ptr = PTC_BASE_PTR;
            break;

        case PORTD:
            pt_base_ptr = PTD_BASE_PTR;
            break;

        case PORTE:
            pt_base_ptr = PTE_BASE_PTR;
            break;

        default:
            return ERR_GPIO_PIN;
    }

    /* Check which logical output level was selected. */
    switch (level)
    {
        case GPIO_LOW:
            GPIO_PCOR_REG(pt_base_ptr) |= GPIO_PIN(pin);
            break;

        case GPIO_HIGH:
            GPIO_PSOR_REG(pt_base_ptr) |= GPIO_PIN(pin);
            break;

        case GPIO_INVERT:
            GPIO_PTOR_REG(pt_base_ptr) |= GPIO_PIN(pin);
            break;

        default:
            return ERR_GPIO_LEVEL;
    }

    return ERR_OK;
}


/*******************************************************************************
 * error_t gpio_get(uint8_t pin_id, uint8_t* level)
 *
 * Description:
 *  Get logical level of specified pin. Getting logical levels is only possible
 *  for initialized input and output pins.
 *
 * Parameters:
 *  pin_id      GPIO pin
 *  level       current logical level
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_GPIO_PIN            - invalid GPIO pin
 *              ERR_GPIO_INIT           - access to uninitialized GPIO pin
 *
 * Example:
 *  error = gpio_get(PTA0, &pta0_level);
 *
 * Note:
 *  This module does not provide GPIO interrupt functionality.
 *
 * History:
 *  pka, 31/JUL/2015, initial code
 ******************************************************************************/
error_t gpio_get(uint8_t pin_id, uint8_t* level)
{
    uint8_t port;                   /* port number */
    uint8_t pin;                    /* pin number */

    GPIO_MemMapPtr pt_base_ptr;     /* GPIO peripheral instance base address */

    /* Check for valid pin identifier. */
    if ((pin_id < 0x00) || (pin_id > 0x9F))
    {
        return ERR_GPIO_PIN;
    }

    /* Get port number, which is stored in bits 7-5 of pin identifier. */
    port = pin_id >> 5;

    /* Get pin number, which is stored in bits 4-0 of pin identifier. */
    pin = pin_id & 0x1F;

    /* Check if pin is initialized. */
    if (!(gpio[port] & (1 << pin)))
    {
        return ERR_GPIO_INIT;
    }

    /* Check which port was selected. */
    switch (port)
    {
        case PORTA:
            /* Set GPIO base address pointer according to corresponding port. */
            pt_base_ptr = PTA_BASE_PTR;
            break;

        case PORTB:
            pt_base_ptr = PTB_BASE_PTR;
            break;

        case PORTC:
            pt_base_ptr = PTC_BASE_PTR;
            break;

        case PORTD:
            pt_base_ptr = PTD_BASE_PTR;
            break;

        case PORTE:
            pt_base_ptr = PTE_BASE_PTR;
            break;

        default:
            return ERR_GPIO_PIN;
    }

    /* Check if pin logic level in GPIOx_PDIR is logical high. */
    if (GPIO_PDIR_REG(pt_base_ptr) & GPIO_PDIR_PDI(GPIO_PIN(pin)))
    {
        *level = GPIO_HIGH;
    }
    else
    {
        *level = GPIO_LOW;
    }

    return ERR_OK;
}
