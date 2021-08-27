/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/poll.h>
#include <linux/sort.h>
#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <mach/eint.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <linux/gpio.h>

#include "fpc_spi.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 touch sensor driver.");

#define FPC_DEBUG  printk("%s\n", __func__)
#define FPC1020_IRQ_GPIO        GPIO_FPC_EINT_PIN//GPIO14
#define FPC1020_RESET_GPIO      GPIO_FPC_RESET_PIN//GPIO145
#define FPC1020_CS_GPIO         GPIO_FPC_CS_PIN//GPIO169


/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */


static int fpc_spi_probe(struct spi_device *spi);

static int fpc_spi_remove(struct spi_device *spi);

static int fpc_spi_reset_init(fpc_spi_data_t *fpc1020);

static int fpc_spi_gpio_reset(fpc_spi_data_t *fpc1020);


static struct spi_driver fpc_spi_driver = {
	.driver = {
		.name	= FPC1020_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= fpc_spi_probe,
	.remove	= fpc_spi_remove
};

static struct spi_board_info spi_board_devs[] = {
	[0] = {
        .modalias="fpc",
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
	},
};




/* -------------------------------------------------------------------- */
static int fpc_spi_probe(struct spi_device *spi)
{
	int error = 0;
	fpc_spi_data_t *fpc1020 = NULL;

	fpc1020 = kzalloc(sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		dev_err(&spi->dev,
		"failed to allocate memory for struct fpc1020_data\n");

		return -ENOMEM;
	}

	fpc1020->reset_gpio = FPC1020_RESET_GPIO;
	fpc1020->irq_gpio   = FPC1020_IRQ_GPIO;
	
	
	error = fpc_spi_reset_init(fpc1020);
	if (error)
	{
		printk("fpc1020_reset_init--error=%d\n",error);	
	}

	error = fpc_spi_gpio_reset(fpc1020);
	if (error)
	{
		printk("fpc1020_reset--errori=%d\n",error);
		goto err;
	}

	return 0;

err:

	kfree(fpc1020);

	return error;
}


/* -------------------------------------------------------------------- */
static int fpc_spi_remove(struct spi_device *spi)
{

	return 0;
}


/* -------------------------------------------------------------------- */
static int fpc_spi_reset_init(fpc_spi_data_t *fpc1020)
{
	int error = 0;

	error = mt_set_gpio_mode(FPC1020_RESET_GPIO, GPIO_MODE_00);

	if (error != 0) {
		printk("[FPC]gpio_request (reset) failed.\n");
		return error;
	}

	error = mt_set_gpio_dir(FPC1020_RESET_GPIO, GPIO_DIR_OUT);

	return error;
}

/* -------------------------------------------------------------------- */
static int fpc_spi_gpio_reset(fpc_spi_data_t *fpc1020)
{
	int error = 0;
	int counter = FPC1020_RESET_RETRIES;

	while (counter) {
		counter--;

		mt_set_gpio_out(fpc1020->reset_gpio,GPIO_OUT_ONE);
		udelay(FPC1020_RESET_HIGH1_US);

		mt_set_gpio_out(fpc1020->reset_gpio,GPIO_OUT_ZERO);
		udelay(FPC1020_RESET_LOW_US);

		mt_set_gpio_out(fpc1020->reset_gpio,GPIO_OUT_ONE);
		udelay(FPC1020_RESET_HIGH2_US);

		error = mt_get_gpio_in(fpc1020->irq_gpio) ? 0 : -EIO;

		if (!error) {
			printk("%s OK !\n", __func__);
			counter = 0;
		} else {
			printk("%s timed out,retrying ...\n",
				__func__);

			udelay(1250);
		}
	}
	return error;
}

static int __init fpc_spi_init(void)
{

#ifdef DEBUG
    printk("%s\n", __func__);
#endif

    spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
    
	if (spi_register_driver(&fpc_spi_driver))
		return -EINVAL;
		
	return 0;

}

static void __exit fpc_spi_exit(void)
{

#ifdef DEBUG
	printk("%s\n", __func__);
#endif

	spi_unregister_driver(&fpc_spi_driver);
}

module_init(fpc_spi_init);
module_exit(fpc_spi_exit);
