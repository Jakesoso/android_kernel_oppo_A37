#ifndef __FPC_SPI_H__
#define __FPC_SPI_H__

#define FPC1020_DEV_NAME                        "fpc"

#define FPC1020_SPI_CLOCK_SPEED                 (5 * 1000000U)

#define FPC1020_RESET_RETRIES			2
#define FPC1020_RESET_LOW_US			1000
#define FPC1020_RESET_HIGH1_US			100
#define FPC1020_RESET_HIGH2_US			1250

typedef struct {
	struct device          *device;
	struct cdev            cdev;
	dev_t                  devno;
	u32                    reset_gpio;
	u32                    irq_gpio;
} fpc_spi_data_t;


#endif