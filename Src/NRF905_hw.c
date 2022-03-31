/*
 * NRF905_hw.c
 *
 *  Created on: 17.06.2018
 *      Author: Wojciech Domski
 */

#include "NRF905_hw.h"

#include "gpio.h"
#include "tim.h"
#include "spi.h"

int NRF905_hw_gpio_get(NRF905_hw_t *hw, uint8_t gpio) {
	if (hw == NULL) {
		return -1;
	}

	if (gpio >= 0 && gpio <= 6) {
		if (hw->gpio[gpio].port) {
			return HAL_GPIO_ReadPin((GPIO_TypeDef*) hw->gpio[gpio].port,
					hw->gpio[gpio].pin);
		} else {
			return -2;
		}
	}

	return -3;
}

int NRF905_hw_gpio_set(NRF905_hw_t *hw, uint8_t gpio, uint8_t value) {
	if (hw == NULL) {
		return -1;
	}

	if (gpio >= 0 && gpio <= 6) {
		if (hw->gpio[gpio].port) {
			HAL_GPIO_WritePin((GPIO_TypeDef*) hw->gpio[gpio].port,
					hw->gpio[gpio].pin, value);
		} else {
			return -2;
		}
	}

	return 0;
}

void NRF905_hw_delay_ms(NRF905_hw_t *hw, uint32_t ms) {
	HAL_Delay(ms);
}

void NRF905_hw_delay_us(NRF905_hw_t *hw, uint16_t delay) {
	if (hw->tim != NULL) {
		volatile uint16_t current_time, stop_time;
		current_time = __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef* ) (hw->tim));
		stop_time = current_time + delay;

		//overflow
		if (stop_time < current_time) {
			do {
				current_time = __HAL_TIM_GET_COUNTER(
						(TIM_HandleTypeDef* ) (hw->tim));
			} while (current_time > stop_time);
		}

		do {
			current_time = __HAL_TIM_GET_COUNTER(
					(TIM_HandleTypeDef* ) (hw->tim));
		} while (current_time < stop_time);
	}
}

void NRF905_hw_enable_timer(NRF905_hw_t *hw) {
	if (hw->tim != NULL) {
		__HAL_TIM_SET_COUNTER((TIM_HandleTypeDef* )(hw->tim), 0);
		__HAL_TIM_ENABLE((TIM_HandleTypeDef* )hw->tim);
	}
}

int NRF905_hw_spi_transfer(NRF905_hw_t *hw, uint8_t data_tx, uint8_t *data_rx) {
	uint8_t dummy;
	int ret;

	if (hw == NULL) {
		return -1;
	}

	if (data_rx == NULL) {
		data_rx = &dummy;
	}

	ret = HAL_SPI_TransmitReceive((SPI_HandleTypeDef*) hw->spi, &data_tx,
			data_rx, 1, 100);

	return ret;
}
