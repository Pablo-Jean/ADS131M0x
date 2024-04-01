/*
 * ADS131M0x.c
 *
 *  Created on: Mar 30, 2024
 *      Author: pablo-jean
 */

#include "ADS131M0x.h"

/**
 * Macros
 */

#define _SYNC_ON		0
#define _SYNC_OFF		1
#define _CS_ON			0
#define _CS_OFF			1

/*
 * Privates
 */

/* Basic routines to call external */
static void __write_spi(ads131_t *Ads131, uint8_t *tx, uint32_t len){
	Ads131->fxn.SPITransfer(tx, NULL, len);
}

static void __read_spi(ads131_t *Ads131, uint8_t *rx, uint32_t len){
	Ads131->fxn.SPITransfer(NULL, rx, len);
}

static void __transfer_spi(ads131_t *Ads131, uint8_t *tx, uint8_t *rx, uint32_t len){
	Ads131->fxn.SPITransfer(tx, rx, len);
}

static void __cs_pin(ads131_t *Ads131, uint8_t cs){
	Ads131->fxn.CSPin(cs);
}

static void __sync_reset_pin(ads131_t *Ads131, uint8_t sync_reset){
	if (Ads131->fxn.SYNCPin != NULL)
		Ads131->fxn.SYNCPin(sync_reset);
}

static __delay_ms(ads131_t *Ads131, uint32_t ms){
	Ads131->fxn.DelayMs(ms);
}

static void __lock_spi(ads131_t *Ads131){
	if (Ads131->fxn.Lock != NULL)
		Ads131->fxn.Lock();
}

static void __unlock_spi(ads131_t *Ads131){
	if (Ads131->fxn.Unlock != NULL)
		Ads131->fxn.Unlock();
}

/* Middle functions */

uint32_t _xfer_word(ads131_t *Ads131, uint32_t *TxWord){
	uint32_t RxWord[4] = {0};

	__lock_spi(Ads131);
	__cs_pin(Ads131, _CS_ON);
	__transfer_spi(Ads131, (uint8_t*)TxWord, (uint8_t*)RxWord, Ads131->WordSize*1);
	__cs_pin(Ads131, _CS_OFF);
	__unlock_spi(Ads131);

	return RxWord[0];
}

ads131_err_e _read_regs(ads131_t *Ads131, uint32_t reg, uint16_t *val){
	uint32_t frame[4] = {0};
	uint32_t rregCmd;
	uint32_t status;

	rregCmd = ADS131_CMD_RREG | ( (reg & 0x3F) << 7 );
	frame[0] = ( (rregCmd >> 8) & 0xFF) | ( (rregCmd & 0xFF) << 8);

	status = _xfer_word(Ads131, frame);

	status = _xfer_word(Ads131, ADS131_CMD_NULL);

	*val = (uint16_t)((status >> 8) & 0xFF) | ( (status & 0xFF) << 8);
}

ads131_err_e _write_regs(ads131_t *Ads131, uint32_t reg, uint16_t val){
//	uint8_t frame[12];
//	uint8_t frameLenin8b;
//	uint32_t *a = frame;
//
//	memset(frame, 0, sizeof(frame));
//
//	reg <<= 8;
//	memcpy(frame, &reg, 3);
//
//	__lock_spi(Ads131);
//	__cs_pin(Ads131, _CS_ON);
//	__write_spi(Ads131, frame, sizeof(frame));
//	__cs_pin(Ads131, _CS_OFF);
//
//	memset(frame, 0, sizeof(frame));
//	__cs_pin(Ads131, _CS_ON);
//	__read_spi(Ads131, frame, 3);
//	__cs_pin(Ads131, _CS_OFF);
//	__unlock_spi(Ads131);
//
//	*val = (uint16_t)(*a >> 8);
}

/*
 * Publics
 */

ads131_err_e ads131_init(ads131_t *Ads131){
	ads131_reg_status_t status;
	ads131_reg_id_t id;

	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->fxn.CSPin == NULL ||
		Ads131->fxn.SPITransfer == NULL ||
		Ads131->fxn.DelayMs == NULL){
		return ADS131_UNKNOWN;
	}

	Ads131->WordSize = 3;

	__sync_reset_pin(Ads131, _SYNC_ON);
	__delay_ms(Ads131, 5);
	__sync_reset_pin(Ads131, _SYNC_OFF);
	__delay_ms(Ads131, 5);

	_read_regs(Ads131, ADS131_REG_STATUS, &status._raw);
	_read_regs(Ads131, ADS131_REG_ID, &id._raw);
}

ads131_err_e ads131_write_reg(ads131_t *Ads131, uint32_t reg, uint16_t val){

}

ads131_err_e ads131_read_reg(ads131_t *Ads131, uint32_t reg, uint16_t *val){

}

ads131_err_e ads131_read_one_channel(ads131_t *Ads131, uint32_t *val){

}

ads131_err_e ads131_read_all_channel(ads131_t *Ads131, ads131_channels_val_t *val){

}
