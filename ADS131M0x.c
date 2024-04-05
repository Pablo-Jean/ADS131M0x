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

#define _SYNC_ON				0
#define _SYNC_OFF				1
#define _CS_ON					0
#define _CS_OFF					1

#define _24BITS_MAX			((16777216.0 - 1) / 2)

/*
 * Privates
 */

static uint8_t GainTable[] = {1, 2, 4, 8, 16, 32, 64, 128};
static uint32_t OsrTable[] = {128, 256, 512, 1024, 2048, 4096, 8192, 16384, 64};

/* Basic routines to call external */
//static void __write_spi(ads131_t *Ads131, uint8_t *tx, uint32_t len){
//	Ads131->fxn.SPITransfer(tx, NULL, len);
//}
//
//static void __read_spi(ads131_t *Ads131, uint8_t *rx, uint32_t len){
//	Ads131->fxn.SPITransfer(NULL, rx, len);
//}

static ads131_err_e __transfer_spi(ads131_t *Ads131, uint8_t *tx, uint8_t *rx, uint32_t len){
	return Ads131->fxn.SPITransfer(tx, rx, len);
}

static void __cs_pin(ads131_t *Ads131, uint8_t cs){
	Ads131->fxn.CSPin(cs);
}

static void __sync_reset_pin(ads131_t *Ads131, uint8_t sync_reset){
	if (Ads131->fxn.SYNCPin != NULL)
		Ads131->fxn.SYNCPin(sync_reset);
}

static void __delay_ms(ads131_t *Ads131, uint32_t ms){
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

uint32_t _xfer_word(ads131_t *Ads131, uint32_t TxWord){
	uint32_t RxWord = 0;

	__lock_spi(Ads131);
	__cs_pin(Ads131, _CS_ON);
	__transfer_spi(Ads131, (uint8_t*)&TxWord, (uint8_t*)&RxWord, Ads131->WordSize*1);
	__cs_pin(Ads131, _CS_OFF);
	__unlock_spi(Ads131);

	return RxWord;
}

ads131_err_e _xfer_words(ads131_t *Ads131, uint8_t *TxWord, uint8_t *RxWord, uint8_t Len){
	ads131_err_e err;

	__lock_spi(Ads131);
	__cs_pin(Ads131, _CS_ON);
	err = __transfer_spi(Ads131, (uint8_t*)TxWord, (uint8_t*)RxWord, Len);
	__cs_pin(Ads131, _CS_OFF);
	__unlock_spi(Ads131);

	return err;
}

uint32_t _xfer_cmd(ads131_t *Ads131, uint16_t TxCmd){
	uint32_t TxWord, RxWord;

	TxWord = ( (TxCmd >> 8) & 0xFF) | ( (TxCmd & 0xFF) << 8 ) << 8;
	RxWord = _xfer_word(Ads131, TxWord);

	return RxWord;
}

ads131_err_e _read_regs(ads131_t *Ads131, uint32_t reg, uint16_t *val){
	uint32_t frame = 0;
	uint32_t rregCmd;
	uint32_t status;

	rregCmd = ADS131_CMD_RREG | ( (reg & 0x3F) << 7 );
	frame = ( (rregCmd >> 8) & 0xFF) | ( (rregCmd & 0xFF) << 8 );

	status = _xfer_word(Ads131, frame);

	status = _xfer_word(Ads131, ADS131_CMD_NULL);

	*val = (uint16_t)((status >> 8) & 0xFF) | ( (status & 0xFF) << 8);
	return ADS131_OK;
}

ads131_err_e _read_n_words(ads131_t *Ads131, uint32_t *val, uint32_t Words){
	uint8_t *TxDummyFrame, *RxFrame, i, offset;
	ads131_err_e err;
	size_t nBytes;

	nBytes = Words*Ads131->WordSize;

	TxDummyFrame = (uint8_t*)malloc(nBytes);
	RxFrame = (uint8_t*)malloc(nBytes);
	if (RxFrame == NULL || TxDummyFrame == NULL){
		while(1);
	}
	memset(TxDummyFrame, 0, nBytes);

	__lock_spi(Ads131);
	__cs_pin(Ads131, _CS_ON);
	err = __transfer_spi(Ads131, TxDummyFrame, RxFrame, nBytes);
	__cs_pin(Ads131, _CS_OFF);
	__unlock_spi(Ads131);

	offset = 0;
	for (i=0 ; i<Words ; i++){
		val[i] = 0;
		val[i] |= (RxFrame[offset++] << 16);
		val[i] |= (RxFrame[offset++] << 8);
		val[i] |= (RxFrame[offset++]);
	}

	free(TxDummyFrame);
	free(RxFrame);

	return err;
}

ads131_err_e _write_regs(ads131_t *Ads131, uint32_t reg, uint16_t val){
	uint8_t frame[6] = {0};
	uint32_t wregCmd;

	wregCmd = ADS131_CMD_WREG | ( (reg & 0x3F) << 7 );

	frame[0] = ((wregCmd >> 8) & 0xFF);
	frame[1] = (wregCmd & 0xFF);
	frame[3] = ((val >> 8) & 0xFF);
	frame[4] = (val & 0xFF);

	return _xfer_words(Ads131, frame, NULL, sizeof(frame));
}

ads131_err_e _update_samplerate(ads131_t *Ads131){
	ads131_reg_clock_t Clock;
	uint8_t TbIdx;
	const uint32_t ULP_FMOD = 1024000;
	const uint32_t LP_FMOD = 2048000;
	const uint32_t HR_FMOD = 4096000;
	const uint32_t MAXSPS = 32000;

	_read_regs(Ads131, ADS131_REG_CLOCK, &Clock._raw);
	if (Clock.TBM == 1){
		TbIdx = ADS131_OSR_64;
	}
	else{
		TbIdx = Clock.OSR;
	}
	if (Clock.POWER == ADS131_PM_ULTRA_LOW_POWER){
		Ads131->_intern.kSamples = ULP_FMOD/OsrTable[TbIdx];
	}
	else if (Clock.POWER == ADS131_PM_ULTRA_LOW_POWER){
		Ads131->_intern.kSamples = LP_FMOD/OsrTable[TbIdx];
	}
	else{
		Ads131->_intern.kSamples = HR_FMOD/OsrTable[TbIdx];
	}

	if (Ads131->_intern.kSamples > MAXSPS){
		Ads131->_intern.kSamples = MAXSPS;
	}

	return ADS131_OK;
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
	Ads131->_intern.ReferenceVoltage = 1200.0;

	// Performa  Reset on the device
	__sync_reset_pin(Ads131, _SYNC_ON);
	__delay_ms(Ads131, 5);
	__sync_reset_pin(Ads131, _SYNC_OFF);
	__delay_ms(Ads131, 5);

	_read_regs(Ads131, ADS131_REG_STATUS, &status._raw);
	_read_regs(Ads131, ADS131_REG_ID, &id._raw);
	Ads131->nChannels = id.CHANCNT;
	Ads131->DeviceModel = (ads131_model_e)id.CHANCNT;

	if (status.LOCK == 1){
		_xfer_cmd(Ads131, ADS131_CMD_UNLOCK);
	}

	_update_samplerate(Ads131);

	Ads131->_intern.Initialized = ADS131M0_DRIVER_INITILIZED;

	return ADS131_OK;
}

ads131_err_e ads131_write_reg(ads131_t *Ads131, uint32_t reg, uint16_t val){
	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->_intern.Initialized != ADS131M0_DRIVER_INITILIZED){
		return ADS131_NOT_INITIALIZED;
	}

	return _write_regs(Ads131, reg, val);
}

ads131_err_e ads131_read_reg(ads131_t *Ads131, uint32_t reg, uint16_t *val){
	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->_intern.Initialized != ADS131M0_DRIVER_INITILIZED){
		return ADS131_NOT_INITIALIZED;
	}

	return _read_regs(Ads131, reg, val);
}

ads131_err_e ads131_set_gain(ads131_t *Ads131, ads131_channel_e Channel, ads131_gain_e GainLevel){
	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->_intern.Initialized != ADS131M0_DRIVER_INITILIZED){
		return ADS131_NOT_INITIALIZED;
	}

	// prevent any invalid Gain input;
	GainLevel &= 0x7;
	if (Channel < 4){
		ads131_reg_gain1_t Gain1;

		ads131_read_reg(Ads131, ADS131_REG_GAIN1, &Gain1._raw);
		// clear region containing the old Gain
		Gain1._raw &= ~(0x7 << Channel*4);
		// Apply the Gain Level
		Gain1._raw |= ((uint8_t)GainLevel << Channel*4);
		ads131_write_reg(Ads131, ADS131_REG_GAIN1, Gain1._raw);
		// Update Gain on handler
		Ads131->_intern.ChInfo[Channel].Gain = GainLevel;
	}
	else{
		ads131_reg_gain2_t Gain2;

		ads131_read_reg(Ads131, ADS131_REG_GAIN2, &Gain2._raw);
		// clear region containing the old Gain
		Gain2._raw &= ~(0x7 << (Channel-4)*4);
		// Apply the Gain Level
		Gain2._raw |= ((uint8_t)GainLevel << (Channel-4)*4);
		ads131_write_reg(Ads131, ADS131_REG_GAIN2, Gain2._raw);
		// Update Gain on handler
		Ads131->_intern.ChInfo[Channel].Gain = GainLevel;
	}

	return ADS131_OK;
}

ads131_err_e ads131_set_channel_enable(ads131_t *Ads131, ads131_channel_e Channel, ads131_enable_e Enable){
	ads131_reg_clock_t Clock;

	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->_intern.Initialized != ADS131M0_DRIVER_INITILIZED){
		return ADS131_NOT_INITIALIZED;
	}

	_read_regs(Ads131, ADS131_REG_CLOCK, &Clock._raw);
	if (Enable == ADS131_ENABLE){
		Clock._raw |= (1 << (8+Channel));
	}
	else{
		Clock._raw &= ~(1 << (8+Channel));
	}
	_write_regs(Ads131, ADS131_REG_CLOCK, Clock._raw);
	Ads131->_intern.ChInfo[Channel].enabled = Enable;

	return ADS131_OK;
}

ads131_err_e ads131_set_mux(ads131_t *Ads131, ads131_channel_e Channel, ads131_mux_e Mux){
	ads131_reg_chx_cfg_t ChCfg;
	uint16_t ChannelCfgAddr;

	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->_intern.Initialized != ADS131M0_DRIVER_INITILIZED){
		return ADS131_NOT_INITIALIZED;
	}

	Mux &= 0x3;
	ChannelCfgAddr = ADS131_REG_CH0_CFG + (Channel * 0x5);
	_read_regs(Ads131, ChannelCfgAddr, &ChCfg._raw);
	ChCfg.MUX = Mux;
	_write_regs(Ads131, ChannelCfgAddr, ChCfg._raw);

	return ADS131_OK;
}

ads131_err_e ads131_set_osr(ads131_t *Ads131, ads131_osr_value_e Osr){
	ads131_reg_clock_t Clock;

	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->_intern.Initialized != ADS131M0_DRIVER_INITILIZED){
		return ADS131_NOT_INITIALIZED;
	}

	_read_regs(Ads131, ADS131_REG_CLOCK, &Clock._raw);
	if (Osr == ADS131_OSR_64){
		// Turbo Mode
		Clock.TBM = 1;
	}
	else{
		Clock.TBM = 0;
		Clock.OSR = (uint8_t)Osr;
	}
	_write_regs(Ads131, ADS131_REG_CLOCK, Clock._raw);
	_update_samplerate(Ads131);

	return ADS131_OK;
}

ads131_err_e ads131_set_power_mode(ads131_t *Ads131, ads131_power_mode_e PowerMode){
	ads131_reg_clock_t Clock;

	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->_intern.Initialized != ADS131M0_DRIVER_INITILIZED){
		return ADS131_NOT_INITIALIZED;
	}

	_read_regs(Ads131, ADS131_REG_CLOCK, &Clock._raw);
	Clock.POWER = PowerMode;
	_write_regs(Ads131, ADS131_REG_CLOCK, Clock._raw);
	_update_samplerate(Ads131);

	return ADS131_OK;
}

ads131_err_e ads131_read_all_channel(ads131_t *Ads131, ads131_channels_val_t *val){
	uint32_t Response[ADS131_MAX_CHANNELS_CHIPSET + 1];
	uint8_t i, offset;
	const int32_t MODULO = 1 << 24;

	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->_intern.Initialized != ADS131M0_DRIVER_INITILIZED){
		return ADS131_NOT_INITIALIZED;
	}

	_read_n_words(Ads131, Response, ADS131_MAX_CHANNELS_CHIPSET + 1);

	offset = 1;
	for (i=0 ; i<Ads131->nChannels ; i++){
		val->ChannelRaw[i] = Response[offset++];
		if (val->ChannelRaw[i] & MODULO){
			val->ChannelRaw[i] = ~((int32_t*)(val))[i];
		}

		val->ChannelVoltageMv[i] = (val->ChannelRaw[i] * (Ads131->_intern.ReferenceVoltage/_24BITS_MAX));
		val->ChannelVoltageMv[i] /= (GainTable[Ads131->_intern.ChInfo[i].Gain]);
	}

	return ADS131_OK;
}

ads131_err_e ads131_read_one_channel(ads131_t *Ads131, ads131_channel_e Channel, uint32_t *raw, double *miliVolt){
	ads131_channels_val_t val;
	ads131_err_e err;

	if (Ads131 == NULL){
		return ADS131_UNKNOWN;
	}
	if (Ads131->_intern.Initialized != ADS131M0_DRIVER_INITILIZED){
		return ADS131_NOT_INITIALIZED;
	}

	if (Channel > Ads131->nChannels){
		return ADS131_CHANNEL_DOENST_EXISTS;
	}

	err = ads131_read_all_channel(Ads131, &val);
	if (err){
		return err;
	}
	if (raw != NULL){
		*raw = val.ChannelRaw[Channel];
	}
	if (miliVolt != NULL){
		*miliVolt = val.ChannelVoltageMv[Channel];
	}

	return ADS131_OK;
}
