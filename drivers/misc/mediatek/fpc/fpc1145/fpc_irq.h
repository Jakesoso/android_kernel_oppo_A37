
#ifndef __FPC_IRQ_H__
#define __FPC_IRQ_H__
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint.h>

#define FPC_IRQ_GPIO        GPIO_FPC_EINT_PIN //GPIO14
#define FPC_RESET_GPIO      GPIO_FPC_RESET_PIN//GPIO145
#define CUST_EINT_EDGE_SENSITIVE 0
#define FPC_INT_IRQNO       CUST_EINT_FPC_NUM //EINT6

#endif

