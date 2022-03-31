/*
 * NRF905.c
 *
 *  Created on: 17.06.2018
 *      Author: Wojciech Domski
 *      WWW:    domski.pl
 *
 * Originally based on: https://github.com/zkemble/nRF905-arduino
 */

#include "NRF905.h"

#include <string.h>
#include <stdint.h>

static const uint8_t NRF905_default_config[] = {
NRF905_CHANNEL,
NRF905_AUTO_RETRAN | NRF905_LOW_RX | NRF905_PWR | NRF905_BAND
		| ((NRF905_CHANNEL >> 8) & 0x01), (NRF905_ADDR_SIZE << 4)
		| NRF905_ADDR_SIZE,
NRF905_PAYLOAD_SIZE, // RX payload size
		NRF905_PAYLOAD_SIZE, // TX payload size
		0xE7, 0xE7, 0xE7, 0xE7, // Default receive address
		NRF905_CRC | NRF905_CLK_FREQ | NRF905_OUTCLK };

uint8_t NRF905_read_config_register(NRF905_t *dev, uint8_t reg) {
	if (dev == NULL) {
		return -1;
	}
	uint8_t val = 0;
	NRF905_HW_SPI_SELECT(dev->hw);
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_R_CONFIG | reg, NULL);
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_NOP, &val);
	NRF905_HW_SPI_DESELECT(dev->hw);
	return val;
}

int NRF905_write_config_register(NRF905_t *dev, uint8_t reg, uint8_t val) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_HW_SPI_SELECT(dev->hw);
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_W_CONFIG | reg, NULL);
	NRF905_hw_spi_transfer(dev->hw, val, NULL);
	NRF905_HW_SPI_DESELECT(dev->hw);

	return 0;
}

int NRF905_set_config_reg1(NRF905_t *dev, uint8_t val, uint8_t mask, uint8_t reg) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_write_config_register(dev, reg,
			(NRF905_read_config_register(dev, NRF905_REG_CONFIG1) & mask)
					| val);

	return 0;
}

int NRF905_set_config_reg2(NRF905_t *dev, uint8_t val, uint8_t mask, uint8_t reg) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_write_config_register(dev, reg,
			(NRF905_read_config_register(dev, NRF905_REG_CONFIG2) & mask)
					| val);

	return 0;
}

int NRF905_setAddress(NRF905_t *dev, uint32_t address, uint8_t cmd) {
	if (dev == NULL) {
		return -1;
	}
	uint8_t i;
	NRF905_HW_SPI_SELECT(dev->hw);
	NRF905_hw_spi_transfer(dev->hw, cmd, NULL);
	for (i = 0; i < 4; i++) {
		NRF905_hw_spi_transfer(dev->hw, address >> (8 * i), NULL);
	}
	NRF905_HW_SPI_DESELECT(dev->hw);

	return 0;
}

uint8_t NRF905_read_status(NRF905_t *dev) {
	if (dev == NULL) {
		return -1;
	}
	uint8_t status;

	NRF905_HW_SPI_SELECT(dev->hw);
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_NOP, &status);
	NRF905_HW_SPI_DESELECT(dev->hw);

	return status;
}

// Hardware: Data ready pin high
// Software: Data ready status bit set

int NRF905_data_ready(NRF905_t *dev) {
	if (dev == NULL) {
		return -1;
	}

	//use software check for data ready
	if (dev->software_pin_configuration & NRF905_USE_SOFTWARE_DATA_READY) {
		return (NRF905_read_status(dev) & (1 << NRF905_STATUS_DR));
	}

	//use hardware data ready
	return NRF905_hw_gpio_get(dev->hw, NRF905_HW_GPIO_DR);
}

// Hardware: Address match pin high
// Software: Address match status bit set
int NRF905_address_matched(NRF905_t *dev) {
	if (dev == NULL) {
		return -1;
	}

	//use software check for data ready
	if (dev->software_pin_configuration & NRF905_USE_SOFTWARE_ADDRESS_MATCH) {
		return (NRF905_read_status(dev) & (1 << NRF905_STATUS_AM));
	}
	//use hardware address match
	return NRF905_hw_gpio_get(dev->hw, NRF905_HW_GPIO_AM);
}

int NRF905_set_channel(NRF905_t *dev, uint16_t channel) {
	if (dev == NULL) {
		return -1;
	}
	if (channel > 511) {
		channel = 511;
	}

	uint8_t reg = (NRF905_read_config_register(dev, NRF905_REG_CONFIG1)
			& NRF905_MASK_CHANNEL) | (channel >> 8);

	NRF905_HW_SPI_SELECT(dev->hw);
	NRF905_hw_spi_transfer(dev->hw,
	NRF905_CMD_W_CONFIG | NRF905_REG_CHANNEL, NULL);
	NRF905_hw_spi_transfer(dev->hw, channel, NULL);
	NRF905_hw_spi_transfer(dev->hw, reg, NULL);
	NRF905_HW_SPI_DESELECT(dev->hw);

	return 0;
}

int NRF905_set_band(NRF905_t *dev, NRF905_band_t band) {
	if (dev == NULL) {
		return -1;
	}

	uint8_t reg = (NRF905_read_config_register(dev, NRF905_REG_CONFIG1)
			& NRF905_MASK_BAND) | band;

	NRF905_HW_SPI_SELECT(dev->hw);
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_W_CONFIG | NRF905_REG_CONFIG1,
	NULL);
	NRF905_hw_spi_transfer(dev->hw, reg, NULL);
	NRF905_HW_SPI_DESELECT(dev->hw);

	return 0;
}

int NRF905_set_auto_retransmit(NRF905_t *dev, NRF905_auto_retran_t val) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_set_config_reg1(dev, val, NRF905_MASK_AUTO_RETRAN,
	NRF905_REG_AUTO_RETRAN);

	return 0;
}

int NRF905_set_low_rx_power(NRF905_t *dev, NRF905_low_rx_t val) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_set_config_reg1(dev, val, NRF905_MASK_LOW_RX, NRF905_REG_LOW_RX);

	return 0;
}

int NRF905_set_tx_power(NRF905_t *dev, NRF905_pwr_t val) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_set_config_reg1(dev, val, NRF905_MASK_PWR, NRF905_REG_PWR);

	return 0;
}

int NRF905_set_CRC(NRF905_t *dev, NRF905_crc_t val) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_set_config_reg2(dev, val, NRF905_MASK_CRC, NRF905_REG_CRC);

	return 0;
}

int NRF905_set_clk_out(NRF905_t *dev, NRF905_outclk_t val) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_set_config_reg2(dev, val, NRF905_MASK_OUTCLK, NRF905_REG_OUTCLK);

	return 0;
}

int NRF905_set_payload_size(NRF905_t *dev, uint8_t size) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_HW_SPI_SELECT(dev->hw);
	if (size > NRF905_MAX_PAYLOAD)
		size = NRF905_MAX_PAYLOAD;

	NRF905_hw_spi_transfer(dev->hw,
	NRF905_CMD_W_CONFIG | NRF905_REG_RX_PAYLOAD_SIZE, NULL);
	NRF905_hw_spi_transfer(dev->hw, size, NULL);
	NRF905_hw_spi_transfer(dev->hw, size, NULL);
	NRF905_HW_SPI_DESELECT(dev->hw);

	return 0;
}

int NRF905_set_address_size(NRF905_t *dev, NRF905_addr_size_t size) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_HW_SPI_SELECT(dev->hw);
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_W_CONFIG | NRF905_REG_ADDR_WIDTH,
	NULL);
	NRF905_hw_spi_transfer(dev->hw, (size << 4) | size, NULL);
	NRF905_HW_SPI_DESELECT(dev->hw);

	return 0;
}

uint8_t NRF905_receive_busy(NRF905_t *dev) {
	if (dev == NULL) {
		return -1;
	}
	return NRF905_address_matched(dev);
}

uint8_t NRF905_airway_busy(NRF905_t *dev) {
	if (dev == NULL) {
		return -1;
	}

	return NRF905_hw_gpio_get(dev->hw, NRF905_HW_GPIO_CD);
}

int NRF905_set_listen_address(NRF905_t *dev, uint32_t address) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_setAddress(dev, address,
	NRF905_CMD_W_CONFIG | NRF905_REG_RX_ADDRESS);
	return 0;
}

uint8_t NRF905_tx(NRF905_t *dev, uint32_t sendTo, void *data, uint8_t len,
		NRF905_nextmode_t nextMode) {
	// TODO check DR is low?

	if (dev == NULL) {
		return -1;
	}

	if (NRF905_airway_busy(dev))
		return 0;

	NRF905_setAddress(dev, sendTo, NRF905_CMD_W_TX_ADDRESS);

	// Load new payload
	if (data != NULL) {
		NRF905_HW_SPI_SELECT(dev->hw);
		NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_W_TX_PAYLOAD,
		NULL);
		for (uint8_t i = 0; i < len; i++) {
			NRF905_hw_spi_transfer(dev->hw, ((uint8_t*) data)[i],
			NULL);
		}
		NRF905_HW_SPI_DESELECT(dev->hw);
	}

	if (!NRF905_HW_POWERED_UP(dev->hw)) {
		NRF905_HW_STANDBY_ENTER(dev->hw);
		NRF905_HW_POWER_UP(dev->hw);
		NRF905_hw_delay_ms(dev->hw, 3);
	}

	if (NRF905_airway_busy(dev))
		return 0;

	// Put into transmit mode
	NRF905_HW_MODE_TX(dev->hw);

	// Pulse standby pin to start transmission
	NRF905_HW_STANDBY_LEAVE(dev->hw);

	if (nextMode == NRF905_NEXTMODE_RX) {
		// The datasheets says that the radio can switch straight to RX mode after
		// a transmission is complete by clearing TX_EN while transmitting, but
		// if this is done within ~700us the transmission seems to get corrupt.
		NRF905_hw_delay_us(dev->hw, 700);
		NRF905_HW_MODE_RX(dev->hw);
	} else if (nextMode == NRF905_NEXTMODE_STANDBY) {
		NRF905_hw_delay_us(dev->hw, 14);
		NRF905_HW_STANDBY_ENTER(dev->hw);
	}
	// else NRF905_NEXTMODE_TX

	return 1;
}

int NRF905_rx(NRF905_t *dev) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_HW_MODE_RX(dev->hw);
	NRF905_HW_STANDBY_LEAVE(dev->hw);
	NRF905_HW_POWER_UP(dev->hw);
	return 0;
}

int NRF905_read(NRF905_t *dev, void *data, uint8_t len) {
	if (dev == NULL) {
		return -1;
	}
	if (len > NRF905_MAX_PAYLOAD)
		len = NRF905_MAX_PAYLOAD;

	NRF905_HW_SPI_SELECT(dev->hw);
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_R_RX_PAYLOAD, NULL);

	// Get received payload
	for (uint8_t i = 0; i < len; i++) {
		NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_NOP, &((uint8_t*) data)[i]);
	}

	NRF905_HW_SPI_DESELECT(dev->hw);

	return 0;
}

int NRF905_power_down(NRF905_t *dev) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_HW_POWER_DOWN(dev->hw);
	return 0;
}

int NRF905_power_up(NRF905_t *dev) {
	if (dev == NULL) {
		return -1;
	}
	uint8_t powered_up = NRF905_HW_POWERED_UP(dev->hw);
	NRF905_HW_STANDBY_ENTER(dev->hw);
	NRF905_HW_POWER_UP(dev->hw);
	if (!powered_up) {
		NRF905_hw_delay_ms(dev->hw, 3);
	}
	return 0;
}

int NRF905_standby(NRF905_t *dev) {
	if (dev == NULL) {
		return -1;
	}
	NRF905_HW_STANDBY_ENTER(dev->hw);
	NRF905_HW_POWER_UP(dev->hw);
	return 0;
}

int NRF905_get_config(NRF905_t *dev, void *regs) {

	if (dev == NULL) {
		return -1;
	}
	NRF905_HW_SPI_SELECT(dev->hw);

	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_R_CONFIG, NULL);
	for (uint8_t i = 0; i < NRF905_REGISTER_COUNT; i++) {
		NRF905_hw_spi_transfer(dev->hw,
		NRF905_CMD_NOP, &((uint8_t*) regs)[i]);
	}

	NRF905_HW_SPI_DESELECT(dev->hw);

	return 0;
}

int NRF905_init(NRF905_t *dev, NRF905_hw_t *hw) {
	uint8_t i;

	if (dev == NULL) {
		return -1;
	}

	if (hw == NULL) {
		return -2;
	}

	dev->hw = hw;

	dev->software_pin_configuration = 0x00;

	NRF905_hw_enable_timer(hw);

	if (hw->gpio[NRF905_HW_GPIO_DR].port == NULL) {
		dev->software_pin_configuration |= NRF905_USE_SOFTWARE_DATA_READY;
	}
	if (hw->gpio[NRF905_HW_GPIO_AM].port == NULL) {
		dev->software_pin_configuration |= NRF905_USE_SOFTWARE_ADDRESS_MATCH;
	}

	NRF905_HW_SPI_DESELECT(dev->hw);

	NRF905_HW_POWER_DOWN(dev->hw);
	NRF905_HW_STANDBY_ENTER(hw);
	NRF905_HW_MODE_RX(hw);
	NRF905_hw_delay_ms(dev->hw, 3);

	// Set control registers
	NRF905_HW_SPI_SELECT(dev->hw);
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_W_CONFIG | 0x00, NULL);
	for (i = 0; i < sizeof(NRF905_default_config); i++) {
		NRF905_hw_spi_transfer(dev->hw, NRF905_default_config[i], NULL);
	}
	NRF905_HW_SPI_DESELECT(dev->hw);

	NRF905_hw_delay_us(dev->hw, 100);

	NRF905_HW_SPI_SELECT(dev->hw);
	// Default transmit address
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_W_TX_ADDRESS, NULL);
	for (i = 0; i < 4; i++) {
		NRF905_hw_spi_transfer(dev->hw, 0xE7, NULL);
	}
	NRF905_HW_SPI_DESELECT(dev->hw);

	NRF905_hw_delay_us(dev->hw, 100);

	NRF905_HW_SPI_SELECT(dev->hw);
	// Clear transmit payload
	NRF905_hw_spi_transfer(dev->hw, NRF905_CMD_W_TX_PAYLOAD, NULL);
	for (i = 0; i < NRF905_MAX_PAYLOAD; i++) {
		NRF905_hw_spi_transfer(dev->hw, 0x00, NULL);
	}
	NRF905_HW_SPI_DESELECT(dev->hw);

	return 0;
}
