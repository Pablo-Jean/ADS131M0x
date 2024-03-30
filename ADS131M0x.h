/*
 * ADS131M0x.h
 *
 *  Created on: Mar 30, 2024
 *      Author: pablo-jean
 */

#ifndef DRIVER_ADS131M0X_ADS131M0X_H_
#define DRIVER_ADS131M0X_ADS131M0X_H_

/**
 * Includes
 */

#include <stdint.h>
#include <cmsis_compiler.h>

/**
 * Macros
 */

#define ADS131M0_DRIVER_INITILIZED	0xC4

#define ADS131_CMD_NULL				0x0000
#define ADS131_CMD_RESET			0x0011
#define ADS131_CMD_STANDBY			0x0022
#define ADS131_CMD_WAKEUP			0x0033
#define ADS131_CMD_LOCK				0x0555
#define ADS131_CMD_UNLOCK			0x0655
#define ADS131_CMD_RREG				0xA000
#define ADS131_CMD_WREG				0xB000

// Read-Only registers
#define ADS131_REG_ID				0x00
#define ADS131_REG_STATUS			0x01
// Globl Settings registers
#define ADS131_REG_MODE				0x02
#define ADS131_REG_CLOCK			0x03
#define ADS131_REG_GAIN1			0x04
#define ADS131_REG_GAIN2			0x05
#define ADS131_REG_CFG				0x06
#define ADS131_REG_THRSHLD_MSB		0x07
#define ADS131_REG_THRSHLD_LSB		0x08
// Channel 0 Specific Registers
#define ADS131_REG_CH0_CFG			0x09
#define ADS131_REG_CH0_OCAL_MSB		0x0A
#define ADS131_REG_CH0_OCAL_LSB		0x0B
#define ADS131_REG_CH0_GCAL_MSB		0x0C
#define ADS131_REG_CH0_GCAL_LSB		0x0D
// Channel 1 Specific Registers
#define ADS131_REG_CH1_CFG			0x0E
#define ADS131_REG_CH1_OCAL_MSB		0x0F
#define ADS131_REG_CH1_OCAL_LSB		0x10
#define ADS131_REG_CH1_GCAL_MSB		0x11
#define ADS131_REG_CH1_GCAL_LSB		0x12
// Register Map CRC and Reserved
#define ADS131_REG_REGMAP_CRC		0x3E
#define ADS131_REG_RESERVED			0x3F


/**
 * Enumerates
 */

typedef enum{
	ADS131_OK,
	ADS131_FAILED,
	ADS131_NOT_INITIALIZED,
	ADS131_CHANNEL_DOENST_EXISTS,

	ADS131_UNKNOWN = 0xFF
}ads131_err_e;


/**
 * Typedefs and Structs
 */

// Strucuts for the fields of each register of the
// ADS131M0x, this is to make more easy the handleing of
// the parameters
typedef __PACKED_STRUCT{
	uint8_t _r1 	: 8;
	uint8_t CHANCNT : 4;
	uint8_t _r2		: 4;
}ads131_reg_id_t;

typedef __PACKED_STRUCT{
	uint8_t DRDY0	: 1;
	uint8_t DRDY1	: 1;
	uint8_t _r1		: 6;
	uint8_t WLENGTH : 2;
	uint8_t RESET	: 1;
	uint8_t CRC_TYPE: 1;
	uint8_t CRC_ERR : 1;
	uint8_t REG_MAP : 1;
	uint8_t F_RESYNC: 1;
	uint8_t LOCK	: 1;
}ads131_reg_status_t;

typedef __PACKED_STRUCT{
	uint8_t DRDY_FMT  : 1;
	uint8_t DRDY_HiZ  : 1;
	uint8_t DRDY_SEL  : 2;
	uint8_t TIMEOUT   : 1;
	uint8_t _r1		  : 3;
	uint8_t WLENGTH   : 2;
	uint8_t RESET     : 1;
	uint8_t CRC_TYPE  : 1;
	uint8_t RX_CRC_EN : 1;
	uint8_t REG_CRC_EN: 1;
	uin8t_t _r2		  : 1;
}ads131_reg_mode_t;

typedef __PACKED_STRUCT{
	uint8_t PWR		: 2;
	uint8_t OSR		: 2;
	uint8_t TBM		: 1;
	uint8_t _r1		: 2;
	uint8_t CH0_EN	: 1;
	uint8_t CH1_EN	: 1;
	uint8_t _r2		: 6;
}ads131_reg_clock_t;

typedef __PACKED_STRUCT{
	uint8_t PGAGAIN0 	: 3;
	uint8_t _res1		: 1;
	uint8_t PGAGAIN1	: 3;
	uint8_t _res2		: 1;
	uint8_t PGAGAIN2	: 3;
	uint8_t _res3		: 1;
	uint8_t PGAGAIN3	: 3;
	uint8_t _res4		: 1;
}ads131_reg_gain1_t;

typedef __PACKED_STRUCT{
	uint8_t PGAGAIN4 	: 3;
	uint8_t _res1		: 1;
	uint8_t PGAGAIN5	: 3;
	uint8_t _res2		: 1;
	uint8_t PGAGAIN6	: 3;
	uint8_t _res3		: 1;
	uint8_t PGAGAIN7	: 3;
	uint8_t _res4		: 1;
}ads131_reg_gain2_t;

typedef __PACKED_STRUCT{
	uint8_t CD_EN		: 1;
	uint8_t CD_LEN		: 3;
	uint8_t CD_NUM		: 3;
	uint8_t CD_ALLCH	: 1;
	uint8_t GC_EN		: 1;
	uint8_t GC_DLY		: 4;
	uint8_t _res1		: 3;
}ads131_reg_cfg_t;

typedef __PACKED_STRUCT{
	uint16_t CD_TH_MSB;
}ads131_thrshld_lsb_t;

typedef __PACKED_STRUCT{
	uint8_t DCBLOCK		: 4;
	uint8_t _res1		: 4;
	uint8_t CD_TH_LSB	: 8;
}ads131_thrshld_lsb_t;

typedef __PACKED_STRUCT{
	uint8_t MUX			: 2;
	uint8_t DCBLK_DIS0	: 1;
	uint8_t _res1		: 3;
	uint16_t PHASE		: 10;
}ads131_chx_cfg_t;

typedef __PACKED_STRUCT{
	uint16_t OCAL_MSB;
}ads131_chx_ocal_msb_t;

typedef __PACKED_STRUCT{
	uint8_t _res1		: 8;
	uint8_t OCAL_LSB	: 8;
}ads131_chx_ocal_lsb_t;

typedef __PACKED_STRUCT{
	uint16_t GCAL_MSB;
}ads131_chx_gcal_msb_t;

typedef __PACKED_STRUCT{
	uint8_t _res1		: 8;
	uint8_t GCAL_LSB	: 8;
}ads131_chx_gcal_lsb_t;

typedef __PACKED_STRUCT{
	uint16_t REG_CRC;
}ads131_reg_regmap_crc_t;

// Definitions of external functions that must be provided from
// developer in the initilization

typedef void (*Lock_f)(void);
typedef void (*Unlock_f)(void);
typedef void (*CSPin_f)(uint8_t Sig);
typedef void (*SYNCPin_f)(uint8_t Sig);
typedef void (*SPITransfer_f)(uint8_t *Tx, uint8_t *Rx, uint32_t len);

/* Main Handle of the ADS131M0x Driver */
typedef struct{
	struct{
		Lock_f Lock;
		Unlock_f Unlock;
		CSPin_f CSPin;
		SYNCPin_f SYNCPin;
		SPITransfer_f SPITransfer;
	}fxn;
	uint8_t Initialized;
	uint8_t nChannels;
}ads131_t;

typedef struct{
	uint32_t Channel0;
	uint32_t Channel1;
	uint32_t Channel2;
	uint32_t Channel3;
	uint32_t Channel4;
	uint32_t Channel5;
	uint32_t Channel6;
	uint32_t Channel7;
}ads131_channels_val_t;

/*
 * Public Functions
 */

ads131_err_e ads131_init(ads131_t *Ads131);

ads131_err_e ads131_write_reg(ads131_t *Ads131, uint8_t reg, uint8_t val);

ads131_err_e ads131_read_reg(ads131_t *Ads131, uint8_t reg, uint8_t *val);

ads131_err_e ads131_read_one_channel(ads131_t *Ads131, uint32_t *val);

ads131_err_e ads131_read_all_channel(ads131_t *Ads131, ads131_channels_val_t *val);

// More configurations must be added later

#endif /* DRIVER_ADS131M0X_ADS131M0X_H_ */
