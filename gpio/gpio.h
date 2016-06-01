/*******************************************************************************
 * gpio.h
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

#ifndef GPIO_H_
#define GPIO_H_

#include "error.h"
#include "mkl25z128xxx4.h"

#define GPIO_PIN(pin)       (1 << (pin & 0x1F))

#define PTA0                0x00
#define PTA1                0x01
#define PTA2                0x02
#define PTA3                0x03
#define PTA4                0x04
#define PTA5                0x05
#define PTA6                0x06
#define PTA7                0x07
#define PTA8                0x08
#define PTA9                0x09
#define PTA10               0x0A
#define PTA11               0x0B
#define PTA12               0x0C
#define PTA13               0x0D
#define PTA14               0x0E
#define PTA15               0x0F
#define PTA16               0x10
#define PTA17               0x11
#define PTA18               0x12
#define PTA19               0x13
#define PTA20               0x14
#define PTA21               0x15
#define PTA22               0x16
#define PTA23               0x17
#define PTA24               0x18
#define PTA25               0x19
#define PTA26               0x1A
#define PTA27               0x1B
#define PTA28               0x1C
#define PTA29               0x1D
#define PTA30               0x1E
#define PTA31               0x1F
#define PTB0                0x20
#define PTB1                0x21
#define PTB2                0x22
#define PTB3                0x23
#define PTB4                0x24
#define PTB5                0x25
#define PTB6                0x26
#define PTB7                0x27
#define PTB8                0x28
#define PTB9                0x29
#define PTB10               0x2A
#define PTB11               0x2B
#define PTB12               0x2C
#define PTB13               0x2D
#define PTB14               0x2E
#define PTB15               0x2F
#define PTB16               0x30
#define PTB17               0x31
#define PTB18               0x32
#define PTB19               0x33
#define PTB20               0x34
#define PTB21               0x35
#define PTB22               0x36
#define PTB23               0x37
#define PTB24               0x38
#define PTB25               0x39
#define PTB26               0x3A
#define PTB27               0x3B
#define PTB28               0x3C
#define PTB29               0x3D
#define PTB30               0x3E
#define PTB31               0x3F
#define PTC0                0x40
#define PTC1                0x41
#define PTC2                0x42
#define PTC3                0x43
#define PTC4                0x44
#define PTC5                0x45
#define PTC6                0x46
#define PTC7                0x47
#define PTC8                0x48
#define PTC9                0x49
#define PTC10               0x4A
#define PTC11               0x4B
#define PTC12               0x4C
#define PTC13               0x4D
#define PTC14               0x4E
#define PTC15               0x4F
#define PTC16               0x50
#define PTC17               0x51
#define PTC18               0x52
#define PTC19               0x53
#define PTC20               0x54
#define PTC21               0x55
#define PTC22               0x56
#define PTC23               0x57
#define PTC24               0x58
#define PTC25               0x59
#define PTC26               0x5A
#define PTC27               0x5B
#define PTC28               0x5C
#define PTC29               0x5D
#define PTC30               0x5E
#define PTC31               0x5F
#define PTD0                0x60
#define PTD1                0x61
#define PTD2                0x62
#define PTD3                0x63
#define PTD4                0x64
#define PTD5                0x65
#define PTD6                0x66
#define PTD7                0x67
#define PTD8                0x68
#define PTD9                0x69
#define PTD10               0x6A
#define PTD11               0x6B
#define PTD12               0x6C
#define PTD13               0x6D
#define PTD14               0x6E
#define PTD15               0x6F
#define PTD16               0x70
#define PTD17               0x71
#define PTD18               0x72
#define PTD19               0x73
#define PTD20               0x74
#define PTD21               0x75
#define PTD22               0x76
#define PTD23               0x77
#define PTD24               0x78
#define PTD25               0x79
#define PTD26               0x7A
#define PTD27               0x7B
#define PTD28               0x7C
#define PTD29               0x7D
#define PTD30               0x7E
#define PTD31               0x7F
#define PTE0                0x80
#define PTE1                0x81
#define PTE2                0x82
#define PTE3                0x83
#define PTE4                0x84
#define PTE5                0x85
#define PTE6                0x86
#define PTE7                0x87
#define PTE8                0x88
#define PTE9                0x89
#define PTE10               0x8A
#define PTE11               0x8B
#define PTE12               0x8C
#define PTE13               0x8D
#define PTE14               0x8E
#define PTE15               0x8F
#define PTE16               0x90
#define PTE17               0x91
#define PTE18               0x92
#define PTE19               0x93
#define PTE20               0x94
#define PTE21               0x95
#define PTE22               0x96
#define PTE23               0x97
#define PTE24               0x98
#define PTE25               0x99
#define PTE26               0x9A
#define PTE27               0x9B
#define PTE28               0x9C
#define PTE29               0x9D
#define PTE30               0x9E
#define PTE31               0x9F

#define GPIO_INPUT          0x00
#define GPIO_OUTPUT         0x01

#define GPIO_FLOAT              0x00
#define GPIO_PULLUP             0x01

#define GPIO_LOW                0x00
#define GPIO_HIGH               0x01
#define GPIO_INVERT             0x02

/* Provided functions. */
error_t gpio_init(uint8_t pin_id, uint8_t mode, uint8_t level);
error_t gpio_set(uint8_t pin_id, uint8_t level);
error_t gpio_get(uint8_t pin_id, uint8_t* level);

#endif /* GPIO_H_ */
