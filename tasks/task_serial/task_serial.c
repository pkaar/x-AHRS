/*******************************************************************************
 * task_serial.c
 *
 * Description:
 *  This module is used to initialize serial communication module. The module
 *  uses UART interface to transmit and receive data. To ensure data integrity
 *  the message is transmitted with a start and end byte, length information
 *  and a CRC checksum. A valid serial data stream has to follow the convention
 *  |STX|LENGTH|DATA|DATA| ... |DATA|CRC16|CRC16|ETX|.
 *  Data transmission is done calling the appropriate function. Receiving is
 *  implemented within a task which is executed when data is available. Data
 *  is interpreted within this task. Valid or invalid data is reported by
 *  transmitting ACK and NAK respectively.
 *
 * Functions:
 *  error_t task_serial_init(void);
 *  error_t serial_transmit(uint8_t* data, uint16_t size);
 *
 * Note:
 *  This module was intended to be used on Kinetis MKL25Z128 microcontrollers.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/

#include "task_serial.h"

#include "cpu.h"
#include "crc16.h"
#include "pit.h"
#include "uart.h"

#include "task_ahrs.h"
#include "task_led.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

#define SERIAL_BAUD                     115200UL

#define SERIAL_TIMEOUT_MS               100
#define SERIAL_TIMEOUT_HZ               (1000 / SERIAL_TIMEOUT_MS)

#define SERIAL_CMD_START_SAMPLING       0x00
#define SERIAL_CMD_PAUSE_SAMPLING       0x01

#define SERIAL_CMD_BLOCK_BYTE_CNT       2

#define SERIAL_CMD_ACK                  {TASK_SERIAL_MSG_CMD_BLOCK, 0x06}
#define SERIAL_CMD_NAK                  {TASK_SERIAL_MSG_CMD_BLOCK, 0x15}

#define SERIAL_STX                      0x02
#define SERIAL_ETX                      0x03

#define SERIAL_TX_DATA_BYTE_CNT         64
#define SERIAL_RX_DATA_BYTE_CNT         64

#define SERIAL_STX_BYTE_CNT             1
#define SERIAL_ETX_BYTE_CNT             1
#define SERIAL_LEN_BYTE_CNT             1
#define SERIAL_CRC_BYTE_CNT             2

#define SERIAL_FRAMING_BYTE_CNT         (SERIAL_STX_BYTE_CNT +                 \
                                         SERIAL_ETX_BYTE_CNT +                 \
                                         SERIAL_LEN_BYTE_CNT +                 \
                                         SERIAL_CRC_BYTE_CNT)

#define SERIAL_STX_PENDING              0x00
#define SERIAL_ETX_PENDING              0x01
#define SERIAL_LEN_PENDING              0x02
#define SERIAL_DAT_PENDING              0x03

#define SERIAL_RX_CALLBACK_UART         0x00
#define SERIAL_RX_CALLBACK_PIT          0x01

volatile uint8_t rx_data[SERIAL_RX_DATA_BYTE_CNT + SERIAL_FRAMING_BYTE_CNT];
volatile uint8_t rx_data_length = 0;

xSemaphoreHandle xSemaphoreRxComplete;

static void task_serial(void* pvParameters);

/*******************************************************************************
 * error_t task_serial_init(void)
 *
 * Description:
 *  Initialize serial module and serial receive task. The serial receive task
 *  is unlocked by a binary semaphore and therefore only executed when the
 *  semaphore is released.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to create semaphore
 *
 * Example:
 *  error = task_serial_init();
 *
 * Note:
 *  Interrupt priorities have to be adjusted in order to make calls of FreeRTOS
 *  interrupt safe functions.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
error_t task_serial_init(void)
{
    /* Initialize UART module. */
    uart_init(SERIAL_BAUD);

    /* Set UART interrupt priority. */
    set_int_priority(INT_UART0, 2);

    /* Initialize PIT module. */
    pit_init(PIT0, SERIAL_TIMEOUT_HZ, PIT_ENABLE_INT);

    /* Set PIT interrupt priority. */
    set_int_priority(INT_PIT, 2);

    /* Create binary semaphore to indicate complete messages. */
    vSemaphoreCreateBinary(xSemaphoreRxComplete);

    /* Check if semaphore was created successfully. */
    if (xSemaphoreRxComplete == NULL)
    {
        return ERR_FAIL;
    }

    /* Create serial receive task with stack size of 128 bytes and a priority
     * one level lower than AHRS task. */
    xTaskCreate(&task_serial, "task_serial", 128, (void*) NULL,
                (configMAX_PRIORITIES - 2), (xTaskHandle*) NULL);

    return ERR_OK;
}


/*******************************************************************************
 * error_t serial_transmit(uint8_t* data, uint16_t size)
 *
 * Description:
 *  Write data to serial interface. To ensure data integrity the message is
 *  transmitted with a start and end byte, length information and a CRC
 *  checksum. A valid serial data stream has to follow the convention
 *  |STX|LENGTH|DATA|DATA| ... |DATA|CRC16|CRC16|ETX|. A maximum data size of
 *  128 bytes can be transmitted.
 *
 * Parameters:
 *  data        memory address of data to be transmitted
 *  size        number of bytes to be transmitted
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to write data
 *
 * Example:
 *  error = serial_transmit(data, sizeof(data);
 *
 * Note:
 *  Serial module has to be initialized before calling this routine.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
error_t serial_transmit(uint8_t* data, uint16_t size)
{
    static uint8_t message[SERIAL_TX_DATA_BYTE_CNT + SERIAL_FRAMING_BYTE_CNT];

    uint16_t msg_idx;
    uint16_t crc_value;

    /* Check for valid data size. */
    if (size > 0x80)
    {
        return ERR_FAIL;
    }

    /* Clear message index. */
    msg_idx = 0;

    /* Calculate CRC checksum. */
    crc_value = crc16(data, size);

    /* Add start of text and increment message index. */
    message[msg_idx++] = SERIAL_STX;

    /* Add message length and increment message index. The message length is
     * calculated of the length of the data and the checksum length. */
    message[msg_idx++] = size + sizeof(crc_value);

    /* Add data and increment message index. */
    memcpy(&message[msg_idx], data, size);
    msg_idx = msg_idx + size;

    /* Add CRC value and increment message index. */
    memcpy(&message[msg_idx], &crc_value, sizeof(crc_value));
    msg_idx = msg_idx + sizeof(crc_value);

    /* Add end of text and increment message index. The message index
     * corresponds to the entire message length. */
    message[msg_idx++] = SERIAL_ETX;

    /* Transmit message. */
    if (uart_transmit(message, msg_idx) != ERR_OK)
    {
        return ERR_FAIL;
    }

    return ERR_OK;
}


/*******************************************************************************
 * error_t serial_receive(uint8_t* data, uint16_t* size)
 *
 * Description:
 *  Read data from serial interface. Only valid data is copied to the specified
 *  memory address. Data is valid if the CRC checksum value is correct and at
 *  least one byte of data was received. Possible errors are reported to the
 *  calling function. A valid serial data stream has to follow the convention
 *  |STX|LENGTH|DATA|DATA| ... |DATA|CRC16|CRC16|ETX|.
 *
 * Parameters:
 *  data        memory address of data to be received
 *  size        number of valid bytes received
 *
 * Return:
 *  error       ERR_OK                  - completed without error
 *              ERR_FAIL                - failed to read data
 *
 * Example:
 *  error = serial_receive(data, &size);
 *
 * Note:
 *  Check error values when calling this function in order to identify invalid
 *  data. Inform the transmitter by sending NACK to the serial interface.
 *  This method is not available outside this module.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
error_t serial_receive(uint8_t* data, uint16_t* size)
{
    uint16_t crc_value;

    /* Check for correct number of bytes in receive buffer. The minimum number
     * of bytes is the buffer is 1 data byte and the CRC value. */
    if (rx_data_length < (1 + sizeof(crc_value)))
    {
        return ERR_FAIL;
    }

    /* Calculate CRC checksum value. */
    crc_value = crc16((uint8_t*) rx_data, rx_data_length);

    /* Return an error to the calling function in case the CRC checksum is not
     * equal to 0. */
    if (crc_value != 0)
    {
        return ERR_FAIL;
    }

    /* Copy received data to specified memory address. Only data without the
     * CRC checksum is copied. */
    memcpy(data, (uint8_t*) rx_data, rx_data_length - sizeof(crc_value));

    /* Copy data size to specified memory address. */
    *size = rx_data_length - sizeof(crc_value);

    return ERR_OK;
}


/*******************************************************************************
 * void serial_rx_callback(uint8_t caller)
 *
 * Description:
 *  Read serial data from UART receive buffer. A valid serial data stream has
 *  to follow the convention |STX|LENGTH|DATA|DATA| ... |DATA|CRC16|CRC16|ETX|.
 *  Messages starting with bytes not equal to STX are ignored. Within a message
 *  two bytes have to be received within a defined time span. Otherwise a
 *  timeout is generated.
 *
 * Parameters:
 *  caller      calling interrupt service routine
 *
 * Return:
 *  none
 *
 * Example:
 *  serial_rx_callback(SERIAL_RX_CALLBACK_UART);
 *
 * Note:
 *  Serial module has to be initialized before calling this routine.
 *  For generating timeouts a PIT timer is used. This timer is started when
 *  STX is received. For every received byte the timer is reset. If ETX is
 *  detected, the timer is stopped. If the timer expires due to a delayed or
 *  missing byte, a timer interrupt is triggered.
 *  This method is not available outside this module.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
void serial_rx_callback(uint8_t caller)
{
    uint8_t data;

    static uint8_t state = SERIAL_STX_PENDING;
    static uint8_t length;

    static uint16_t cnt;

    portBASE_TYPE xTaskWoken = pdFALSE;

    /* Check if caller was UART interrupt. Processing of received data is
     * required. */
    if (caller == SERIAL_RX_CALLBACK_UART)
    {
        /* Get received data byte from UART. */
        uart_receive(&data, 1);

        /* Restart PIT timer. The PIT timer is restarted after each received
         * byte and acts as a timeout in case of communication problems. */
        pit_restart(PIT0);

        /* Check current protocol interpreter state. */
        switch (state)
        {
            /* Start byte pending. The protocol interpreter is waiting for STX
             * to be received. */
            case (SERIAL_STX_PENDING):
                /* Check if received byte is STX, indicating message start. */
                if (data == SERIAL_STX)
                {
                    /* Reset message length and byte count. */
                    length = 0;
                    cnt = 0;

                    /* Set state to SERIAL_LENGTH_PENDING. The next received
                     * byte specifies the message length. */
                    state = SERIAL_LEN_PENDING;
                }
                break;

            /* Message length pending. The protocol interpreter is waiting for
             * message length to be received. */
            case (SERIAL_LEN_PENDING):
                length = data;

                /* Check if length exceeds maximum message length. */
                if (length > SERIAL_RX_DATA_BYTE_CNT + SERIAL_CRC_BYTE_CNT)
                {
                    /* Set state to SERIAL_STX_PENDING. For a valid message
                     * the next received byte has to be STX. */
                    state = SERIAL_STX_PENDING;

                    /* Reset message length and byte count. */
                    length = 0;
                    cnt = 0;

                    /* Stop PIT timer since the entire message was received. */
                    pit_stop(PIT0);

                    /* Set binary semaphore to unlock message interpreter
                     * task. */
                    xSemaphoreGiveFromISR(xSemaphoreRxComplete, &xTaskWoken);

                    portYIELD_FROM_ISR(xTaskWoken);

                    break;
                }

                /* Set state to SERIAL_LENGTH_PENDING. The next received bytes
                 * contain the message information. */
                state = SERIAL_DAT_PENDING;
                break;

            /* Message data pending. The protocol interpreter is waiting for
             * the specified count of data bytes to be received. */
            case (SERIAL_DAT_PENDING):
                /* Write received data byte to memory. */
                rx_data[cnt++] = data;

                /* Check if number of bytes received is equal to the specified
                 * message length. */
                if (cnt == length)
                {
                    /* Set state to SERIAL_ETX_PENDING. For a valid message the
                     * next received byte has to be ETX. */
                    state = SERIAL_ETX_PENDING;
                }
                break;

            /* End byte pending. The protocol interpreter is waiting for ETX to
             * be received. */
            case (SERIAL_ETX_PENDING):
                /* Check if received byte is ETX, indicating message end. If
                 * the byte received is not equal to ETX, the entire message
                 * will be ignored. */
                if (data == SERIAL_ETX)
                {
                    /* Valid message was received. Set message according to
                     * received bytes. */
                    rx_data_length = length;
                }
                else
                {
                    /* Invalid message was received. Clear message length for
                     * further error handling. */
                    rx_data_length = 0;
                }

                /* Set state to SERIAL_STX_PENDING. For a valid message
                 * the next received byte has to be STX. */
                state = SERIAL_STX_PENDING;

                /* Reset message length and byte count. */
                length = 0;
                cnt = 0;

                /* Stop PIT timer since the entire message was received. */
                pit_stop(PIT0);

                /* Set binary semaphore to unlock message interpreter task. */
                xSemaphoreGiveFromISR(xSemaphoreRxComplete, &xTaskWoken);

                portYIELD_FROM_ISR(xTaskWoken);

                break;
        }
    }
    /* Check if caller was PIT interrupt. Resetting protocol interpreter due to
     * a communication timeout is required. */
    else if (caller == SERIAL_RX_CALLBACK_PIT)
    {
        /* Communication timeout occurred. Clear message length for further
         * error handling. */
        rx_data_length = 0;

        /* Set state to SERIAL_STX_PENDING. For a valid message the
         * next received byte has to be STX. */
        state = SERIAL_STX_PENDING;

        /* Reset message length and byte count. */
        length = 0;
        cnt = 0;

        /* Stop PIT timer since a timeout has already occurred. */
        pit_stop(PIT0);

        /* Set binary semaphore to unlock message interpreter task. */
        xSemaphoreGiveFromISR(xSemaphoreRxComplete, &xTaskWoken);

        portYIELD_FROM_ISR(xTaskWoken);
    }
}


/*******************************************************************************
 * void uart_rx_callback(void)
 *
 * Description:
 *  UART receive callback function. This function is called when bytes are
 *  received on the UART interface.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * Example:
 *  uart_rx_callback();
 *
 * Note:
 *  This method is not available outside this module.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
void uart_rx_callback(void)
{
    /* Call serial receive callback. */
    serial_rx_callback(SERIAL_RX_CALLBACK_UART);
}


/*******************************************************************************
 * void pit0_callback(void)
 *
 * Description:
 *  PIT callback function. This function is called when the timer reaches 0.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 * Example:
 *  pit0_callback();
 *
 * Note:
 *  This method is not available outside this module.
 *
 * History:
 *  pka, 17/NOV/2015, initial code
 ******************************************************************************/
void pit0_callback(void)
{
    /* Call serial receive callback. */
    serial_rx_callback(SERIAL_RX_CALLBACK_PIT);
}


/*******************************************************************************
 * void task_serial(void* pvParameters)
 *
 * Description:
 *  Serial receive task. This task is executed when data is received on serial
 *  interface. Valid and invalid messages trigger task execution. Valid
 *  messages are interpreted and the appropriate event is executed.
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
void task_serial(void* pvParameters)
{
    uint8_t data[SERIAL_RX_DATA_BYTE_CNT];
    uint16_t size;

    uint8_t cmd_ack[] = SERIAL_CMD_ACK;
    uint8_t cmd_nak[] = SERIAL_CMD_NAK;

    xSemaphoreTake(xSemaphoreRxComplete, 0);

    while (1)
    {
        /* Block on the semaphore to wait for an UART RX interrupt event. */
        xSemaphoreTake(xSemaphoreRxComplete, portMAX_DELAY);

        /* Check if received data is valid. */
        if (serial_receive(data, &size) == ERR_OK)
        {
            /* First byte in data stream represents message identifier. */
            switch (data[0])
            {
                /* Message identifier command block. */
                case (TASK_SERIAL_MSG_CMD_BLOCK):
                    /* Check message size. A valid command message consists of
                     * two bytes. */
                    if (size != SERIAL_CMD_BLOCK_BYTE_CNT)
                    {
                        /* Transmit NAK. */
                        serial_transmit(cmd_nak, sizeof(cmd_nak));

                        break;
                    }

                    /* Second byte in data stream represents command. */
                    switch (data[1])
                    {
                        /* Received command START SAMPLING. */
                        case (SERIAL_CMD_START_SAMPLING):
                            /* Set LED task state to blinking. */
                            task_led_state(TASK_LED_STATE_BLINK);

                            /* Set AHRS task state to start. */
                            task_ahrs_state(TASK_AHRS_STATE_START);

                            /* Transmit ACK. */
                            //serial_transmit(cmd_ack, sizeof(cmd_ack));
                            break;

                        /* Received command PAUSE SAMPLING. */
                        case (SERIAL_CMD_PAUSE_SAMPLING):
                            /* Set LED task state to pause. */
                            task_led_state(TASK_LED_STATE_PAUSE);

                            /* Set AHRS task state to pause. */
                            task_ahrs_state(TASK_AHRS_STATE_PAUSE);

                            /* Transmit ACK. */
                            //serial_transmit(cmd_ack, sizeof(cmd_ack));
                            break;

                        /* Unknown command. */
                        default:
                            /* Transmit NAK. */
                            serial_transmit(cmd_nak, sizeof(cmd_nak));
                    }
                    break;

                /* Unknown command block. */
                default:
                    /* Transmit NAK. */
                    serial_transmit(cmd_nak, sizeof(cmd_nak));
            }
        }

        /* Received data is invalid. */
        else
        {
            /* Transmit NAK. */
            serial_transmit(cmd_nak, sizeof(cmd_nak));
        }
    }
}
