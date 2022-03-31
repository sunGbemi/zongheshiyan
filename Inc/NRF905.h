/*
 * NRF905.c
 *
 *  Created on: 17.06.2018
 *      Author: Wojciech Domski
 *      WWW:    domski.pl
 *
 * Originally based on: https://github.com/zkemble/nRF905-arduino
 */


#ifndef NRF905_H_
#define NRF905_H_

#include "NRF905_hw.h"

// Instructions
#define NRF905_CMD_NOP			0xFF
#define NRF905_CMD_W_CONFIG		0x00
#define NRF905_CMD_R_CONFIG		0x10
#define NRF905_CMD_W_TX_PAYLOAD	0x20
#define NRF905_CMD_R_TX_PAYLOAD	0x21
#define NRF905_CMD_W_TX_ADDRESS	0x22
#define NRF905_CMD_R_TX_ADDRESS	0x23
#define NRF905_CMD_R_RX_PAYLOAD	0x24
#define NRF905_CMD_CHAN_CONFIG	0x80

// Registers
#define NRF905_REG_CHANNEL			0x00
#define NRF905_REG_CONFIG1			0x01
#define NRF905_REG_ADDR_WIDTH		0x02
#define NRF905_REG_RX_PAYLOAD_SIZE	0x03
#define NRF905_REG_TX_PAYLOAD_SIZE	0x04
#define NRF905_REG_RX_ADDRESS		0x05
#define NRF905_REG_CONFIG2			0x09

// TODO remove
#define NRF905_REG_AUTO_RETRAN	NRF905_REG_CONFIG1
#define NRF905_REG_LOW_RX		NRF905_REG_CONFIG1
#define NRF905_REG_PWR			NRF905_REG_CONFIG1
#define NRF905_REG_BAND			NRF905_REG_CONFIG1
#define NRF905_REG_CRC			NRF905_REG_CONFIG2
#define NRF905_REG_CLK			NRF905_REG_CONFIG2
#define NRF905_REG_OUTCLK		NRF905_REG_CONFIG2
#define NRF905_REG_OUTCLK_FREQ	NRF905_REG_CONFIG2

// Clock options
#define NRF905_CLK_4MHZ			0x00
#define NRF905_CLK_8MHZ			0x08
#define NRF905_CLK_12MHZ		0x10
#define NRF905_CLK_16MHZ		0x18
#define NRF905_CLK_20MHZ		0x20

// Register masks
#define NRF905_MASK_CHANNEL		0xFE
#define NRF905_MASK_AUTO_RETRAN	~(NRF905_AUTO_RETRAN_ENABLE | NRF905_AUTO_RETRAN_DISABLE) //0xDF
#define NRF905_MASK_LOW_RX		~(NRF905_LOW_RX_ENABLE | NRF905_LOW_RX_DISABLE) //0xEF
#define NRF905_MASK_PWR			~(NRF905_PWR_n10 | NRF905_PWR_n2 | NRF905_PWR_6 | NRF905_PWR_10) //0xF3
#define NRF905_MASK_BAND		~(NRF905_BAND_433 | NRF905_BAND_868 | NRF905_BAND_915) //0xFD
#define NRF905_MASK_CRC			(uint8_t)(~(NRF905_CRC_DISABLE | NRF905_CRC_8 | NRF905_CRC_16)) //0x3F // typecast to stop compiler moaning about large integer truncation
#define NRF905_MASK_CLK			~(NRF905_CLK_4MHZ | NRF905_CLK_8MHZ | NRF905_CLK_12MHZ | NRF905_CLK_16MHZ | NRF905_CLK_20MHZ) //0xC7
#define NRF905_MASK_OUTCLK		~(NRF905_OUTCLK_DISABLE | NRF905_OUTCLK_4MHZ | NRF905_OUTCLK_2MHZ | NRF905_OUTCLK_1MHZ | NRF905_OUTCLK_500KHZ) // 0xF8

// Bit positions
#define NRF905_STATUS_DR		5
#define NRF905_STATUS_AM		7

#define NRF905_USE_SOFTWARE_DATA_READY			0x01
#define NRF905_USE_SOFTWARE_ADDRESS_MATCH		0x02

typedef struct {
	NRF905_hw_t *hw;
	uint8_t frequency;
	uint8_t config[11];
	uint8_t software_pin_configuration;
} NRF905_t;

// Crystal frequency (the one the radio module is using)
// NRF905_CLK_4MHZ
// NRF905_CLK_8MHZ
// NRF905_CLK_12MHZ
// NRF905_CLK_16MHZ
// NRF905_CLK_20MHZ
#define NRF905_CLK_FREQ		NRF905_CLK_16MHZ

///////////////////
// Default radio settings
///////////////////

// Frequency
// Channel 0 is 422.4MHz for the 433MHz band, each channel increments the frequency by 100KHz, so channel 10 would be 423.4MHz
// Channel 0 is 844.8MHz for the 868/915MHz band, each channel increments the frequency by 200KHz, so channel 10 would be 846.8MHz
// Max channel is 511 (473.5MHz / 947.0MHz)
#define NRF905_CHANNEL			10

// Frequency band
// 868 and 915 are actually the same thing
// NRF905_BAND_433
// NRF905_BAND_868
// NRF905_BAND_915
#define NRF905_BAND			NRF905_BAND_433

// Output power
// n means negative, n10 = -10
// NRF905_PWR_n10 (-10dBm = 100uW)
// NRF905_PWR_n2 (-2dBm = 631uW)
// NRF905_PWR_6 (6dBm = 4mW)
// NRF905_PWR_10 (10dBm = 10mW)
#define NRF905_PWR			NRF905_PWR_10

// Save a few mA by reducing receive sensitivity
// NRF905_LOW_RX_DISABLE (Normal sensitivity)
// NRF905_LOW_RX_ENABLE (Lower sensitivity)
#define NRF905_LOW_RX		NRF905_LOW_RX_DISABLE

// Constantly retransmit payload while in transmit mode
// Can be useful in areas with lots of interference, but you'll need to make sure you can differentiate between re-transmitted packets and new packets (like an ID number).
// It will also block other transmissions if collision avoidance is enabled.
// NRF905_AUTO_RETRAN_DISABLE
// NRF905_AUTO_RETRAN_ENABLE
#define NRF905_AUTO_RETRAN	NRF905_AUTO_RETRAN_DISABLE

// Output a clock signal on pin 3 of IC
// NRF905_OUTCLK_DISABLE
// NRF905_OUTCLK_500KHZ
// NRF905_OUTCLK_1MHZ
// NRF905_OUTCLK_2MHZ
// NRF905_OUTCLK_4MHZ
#define NRF905_OUTCLK		NRF905_OUTCLK_DISABLE

// CRC checksum
// NRF905_CRC_DISABLE
// NRF905_CRC_8
// NRF905_CRC_16
#define NRF905_CRC			NRF905_CRC_16

// Address size
// The address is actually the SYNC part of the packet, just after the preamble and before the data
// NRF905_ADDR_SIZE_1 (not recommended, a lot of false invalid packets will be received)
// NRF905_ADDR_SIZE_4
#define NRF905_ADDR_SIZE	NRF905_ADDR_SIZE_4

// Payload size (1 - 32)
#define NRF905_PAYLOAD_SIZE	NRF905_MAX_PAYLOAD

/**
 * @brief Available modes after transmission complete.
 */
typedef enum {
	NRF905_NEXTMODE_STANDBY, ///< Standby mode
	NRF905_NEXTMODE_RX, ///< Receive mode
	NRF905_NEXTMODE_TX ///< Transmit mode (will auto-retransmit if ::NRF905_AUTO_RETRAN is ::NRF905_AUTO_RETRAN_ENABLE, otherwise will transmit a carrier wave with no data)
} NRF905_nextmode_t;

/**
 * @brief Frequency bands.
 */
typedef enum {
// NOTE:
// When using NRF905_BAND_868 and NRF905_BAND_915 for calculating channel (NRF905_CALC_CHANNEL(f, b)) they should be value 0x01,
// but when using them for setting registers their value should be 0x02.
// They're defined as 0x02 here so when used for calculating channel they're right shifted by 1

	NRF905_BAND_433 = 0x00,	///< 433MHz band
	NRF905_BAND_868 = 0x02,	///< 868/915MHz band
	NRF905_BAND_915 = 0x02	///< 868/915MHz band
} NRF905_band_t;

/**
 * @brief Output power (n means negative, n10 = -10).
 */
typedef enum {
	NRF905_PWR_n10 = 0x00,	///< -10dBm = 100uW
	NRF905_PWR_n2 = 0x04,	///< -2dBm = 631uW
	NRF905_PWR_6 = 0x08,	///< 6dBm = 4mW
	NRF905_PWR_10 = 0x0C	///< 10dBm = 10mW
} NRF905_pwr_t;

/**
 * @brief Save a few mA by reducing receive sensitivity.
 */
typedef enum {
	NRF905_LOW_RX_DISABLE = 0x00,	///< Disable low power receive
	NRF905_LOW_RX_ENABLE = 0x10		///< Enable low power receive
} NRF905_low_rx_t;

/**
 * @brief Auto re-transmit options.
 */
typedef enum {
	NRF905_AUTO_RETRAN_DISABLE = 0x00,	///< Disable auto re-transmit
	NRF905_AUTO_RETRAN_ENABLE = 0x20	///< Enable auto re-transmit
} NRF905_auto_retran_t;

/**
 * @brief Output a clock signal on pin 3 of IC.
 */
typedef enum {
	NRF905_OUTCLK_DISABLE = 0x00,	///< Disable output clock
	NRF905_OUTCLK_4MHZ = 0x04,		///< 4MHz clock
	NRF905_OUTCLK_2MHZ = 0x05,		///< 2MHz clock
	NRF905_OUTCLK_1MHZ = 0x06,		///< 1MHz clock
	NRF905_OUTCLK_500KHZ = 0x07,	///< 500KHz clock (default)
} NRF905_outclk_t;

/**
 * @brief CRC Checksum.
 *
 * The CRC is calculated across the address (SYNC word) and payload
 */
typedef enum {
	NRF905_CRC_DISABLE = 0x00,	///< Disable CRC
	NRF905_CRC_8 = 0x40,///< 8bit CRC (Don't know what algorithm is used for this one)
	NRF905_CRC_16 = 0xC0,		///< 16bit CRC (CRC16-CCITT-FALSE (0xFFFF))
} NRF905_crc_t;

/**
 * @brief Address size.
 *
 * This is actually used as the SYNC word
 */
typedef enum {
	NRF905_ADDR_SIZE_1 = 0x01,///< 1 byte (not recommended, a lot of false invalid packets will be received)
	NRF905_ADDR_SIZE_4 = 0x04,	///< 4 bytes
} NRF905_addr_size_t;

#define NRF905_MAX_PAYLOAD		36 ///< Maximum payload size
#define NRF905_REGISTER_COUNT	10 ///< Configuration register count

#define NRF905_CALC_CHANNEL(f, b)	((((f) / (1 + (b>>1))) - 422400000UL) / 100000UL) ///< Workout channel from frequency & band

uint8_t NRF905_read_config_register(NRF905_t *dev, uint8_t reg);

int NRF905_write_config_register(NRF905_t *dev, uint8_t reg, uint8_t val);

int NRF905_set_config_reg1(NRF905_t *dev, uint8_t val, uint8_t mask,
		uint8_t reg);

int NRF905_set_config_reg2(NRF905_t *dev, uint8_t val, uint8_t mask,
		uint8_t reg);

int NRF905_setAddress(NRF905_t *dev, uint32_t address, uint8_t cmd);

uint8_t NRF905_read_status(NRF905_t *dev);

int NRF905_data_ready(NRF905_t *dev);

int NRF905_address_matched(NRF905_t *dev);

int NRF905_set_channel(NRF905_t *dev, uint16_t channel);

int NRF905_set_band(NRF905_t *dev, NRF905_band_t band);

int NRF905_set_auto_retransmit(NRF905_t *dev, NRF905_auto_retran_t val);

int NRF905_set_low_rx_power(NRF905_t *dev, NRF905_low_rx_t val);

int NRF905_set_tx_power(NRF905_t *dev, NRF905_pwr_t val);

int NRF905_set_CRC(NRF905_t *dev, NRF905_crc_t val);

int NRF905_set_clk_out(NRF905_t *dev, NRF905_outclk_t val);

int NRF905_set_payload_size(NRF905_t *dev, uint8_t size);

int NRF905_set_address_size(NRF905_t *dev, NRF905_addr_size_t size);

uint8_t NRF905_receive_busy(NRF905_t *dev);

uint8_t NRF905_airway_busy(NRF905_t *dev);

int NRF905_set_listen_address(NRF905_t *dev, uint32_t address);

uint8_t NRF905_tx(NRF905_t *dev, uint32_t sendTo, void *data, uint8_t len,
		NRF905_nextmode_t nextMode);

int NRF905_rx(NRF905_t *dev);

int NRF905_read(NRF905_t *dev, void *data, uint8_t len);

int NRF905_power_down(NRF905_t *dev);

int NRF905_power_up(NRF905_t *dev);

int NRF905_standby(NRF905_t *dev);

int NRF905_get_config(NRF905_t *dev, void *regs);

int NRF905_init(NRF905_t *dev, NRF905_hw_t *hw);

#endif
