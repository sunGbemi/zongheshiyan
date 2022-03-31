/*
 * NRF905_hw.h
 *
 *  Created on: 17.06.2018
 *      Author: Wojciech Domski
 */

#ifndef NRF905_HW_H_
#define NRF905_HW_H_

#include <stdint.h>

typedef struct {
	int pin;
	void *port;
} NRF905_hw_dio_t;

#define NRF905_HW_GPIO_TXEN			0
#define NRF905_HW_GPIO_TRX_EN		1
#define NRF905_HW_GPIO_PWR			2

#define NRF905_HW_GPIO_CD			3
#define NRF905_HW_GPIO_AM			4
#define NRF905_HW_GPIO_DR			5

#define NRF905_HW_GPIO_CS			6

typedef struct {
	NRF905_hw_dio_t gpio[7];
	void *spi;
	void *tim;
} NRF905_hw_t;

int NRF905_hw_gpio_get(NRF905_hw_t *hw, uint8_t gpio);

int NRF905_hw_gpio_set(NRF905_hw_t *hw, uint8_t gpio, uint8_t value);

void NRF905_hw_delay_ms(NRF905_hw_t *hw, uint32_t ms);

void NRF905_hw_delay_us(NRF905_hw_t *hw, uint16_t delay);

void NRF905_hw_enable_timer(NRF905_hw_t *hw);

int NRF905_hw_spi_transfer(NRF905_hw_t *hw, uint8_t data_tx, uint8_t *data_rx);

#define NRF905_HW_POWERED_UP(hw)		NRF905_hw_gpio_get(hw, NRF905_HW_GPIO_PWR)
#define NRF905_HW_POWER_DOWN(hw)		NRF905_hw_gpio_set(hw, NRF905_HW_GPIO_PWR, 0)
#define NRF905_HW_POWER_UP(hw)			NRF905_hw_gpio_set(hw, NRF905_HW_GPIO_PWR, 1)

#define NRF905_HW_STANDBY_ENTER(hw)		NRF905_hw_gpio_set(hw, NRF905_HW_GPIO_TRX_EN, 0)
#define NRF905_HW_STANDBY_LEAVE(hw)		NRF905_hw_gpio_set(hw, NRF905_HW_GPIO_TRX_EN, 1)

#define NRF905_HW_MODE_RX(hw)			NRF905_hw_gpio_set(hw, NRF905_HW_GPIO_TXEN, 0)
#define NRF905_HW_MODE_TX(hw)			NRF905_hw_gpio_set(hw, NRF905_HW_GPIO_TXEN, 1)

#define NRF905_HW_SPI_SELECT(hw)		NRF905_hw_gpio_set(hw, NRF905_HW_GPIO_CS, 0)
#define NRF905_HW_SPI_DESELECT(hw)		NRF905_hw_gpio_set(hw, NRF905_HW_GPIO_CS, 1)

#endif /* NRF905_HW_H_ */
