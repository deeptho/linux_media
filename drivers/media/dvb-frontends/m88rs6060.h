//SPX-License-Identifier: GPL-2.0-or-later
/*
 * Montage Technology M88rs6060 demodulator and tuner drivers
 * some code form
 * Copyright (c) 2021 Davin zhang <Davin@tbsdtv.com> www.Turbosight.com
 *
*/

#ifndef _M88RS6060_H_
#define _M88RS6060_H_
#include <linux/dvb/frontend.h>

#define MT_FE_MCLK_KHZ   96000

/*
*
*/

enum m88rs6060_ts_mode {
	MtFeTsOutMode_Unknown = 0,
	MtFeTsOutMode_Serial,
	MtFeTsOutMode_Parallel,
	MtFeTsOutMode_Common,
};

enum MT_FE_CODE_RATE {
	MtFeCodeRate_Undef =-1,
	MtFeCodeRate_1_2 = FEC_1_2,
	MtFeCodeRate_1_4 = FEC_1_4,
	MtFeCodeRate_2_3 = FEC_2_3,
	MtFeCodeRate_3_4 = FEC_3_4,
	MtFeCodeRate_4_5 = FEC_4_5,
	MtFeCodeRate_5_6 = FEC_5_6,
	MtFeCodeRate_7_8 = FEC_7_8,
	MtFeCodeRate_8_9 = FEC_8_9,
	MtFeCodeRate_3_5 = FEC_3_5,
	MtFeCodeRate_9_10 = FEC_9_10,
	MtFeCodeRate_2_5 = FEC_2_5,
	MtFeCodeRate_1_3 = FEC_1_3,
	MtFeCodeRate_11_15 = FEC_11_15,
	MtFeCodeRate_5_9 = FEC_5_9,
	MtFeCodeRate_7_9 = FEC_7_9,
	MtFeCodeRate_4_15 = FEC_4_15,
	MtFeCodeRate_7_15 = FEC_7_15,
	MtFeCodeRate_8_15 = FEC_8_15,
	    //,MtFeCodeRate_1_2L
	    //,MtFeCodeRate_3_5L
	    //,MtFeCodeRate_2_3L
	    //,MtFeCodeRate_5_9L
	    //,MtFeCodeRate_8_15L
	    //,MtFeCodeRate_11_15L
	MtFeCodeRate_13_18 = FEC_13_18,
	MtFeCodeRate_9_20 = FEC_9_20,
	MtFeCodeRate_11_20 = FEC_11_20,
	MtFeCodeRate_23_36 = FEC_23_36,
	MtFeCodeRate_25_36 = FEC_25_36,
	MtFeCodeRate_29_45 = FEC_29_45,
	MtFeCodeRate_11_45 = FEC_11_45,
	MtFeCodeRate_13_45 = FEC_13_45,
	MtFeCodeRate_14_45 = FEC_14_45,
	MtFeCodeRate_26_45 = FEC_26_45,
	    //,MtFeCodeRate_26_45L
	MtFeCodeRate_28_45 = FEC_28_45,

	    //,MtFeCodeRate_29_45L
	MtFeCodeRate_31_45 = FEC_31_45,
	    //,MtFeCodeRate_31_45L
	MtFeCodeRate_32_45 = FEC_32_45,
	    //,MtFeCodeRate_32_45L
	MtFeCodeRate_77_90 = FEC_77_90
};

enum MT_FE_ROLL_OFF {
	MtFeRollOff_Undef = -1,
	MtFeRollOff_0p35 = ROLLOFF_35,
	MtFeRollOff_0p25 = ROLLOFF_25,
	MtFeRollOff_0p20 = ROLLOFF_20,
	MtFeRollOff_0p15 = ROLLOFF_15,
	MtFeRollOff_0p10 = ROLLOFF_10,
	MtFeRollOff_0p05 = ROLLOFF_5
};

enum MT_FE_SPECTRUM_MODE {
	MtFeSpectrum_Undef = 0, MtFeSpectrum_Normal, MtFeSpectrum_Inversion
};

enum MT_FE_TYPE {
	MtFeType_Undef = 0,
	MtFeType_DVBC = 0x10,
	MtFeType_DVBT = 0x20,
	MtFeType_DVBT2 = 0x21,
	MtFeType_DTMB = 0x30,
	MtFeType_DvbS = 0x40,
	MtFeType_DvbS2 = 0x41,
	MtFeType_DvbS2X = 0x42,
	MtFeType_ABS = 0x50,
	MtFeType_TMS = 0x51,
	MtFeType_DTV_Unknown = 0xFE,
	MtFeType_DTV_Checked = 0xFF
};

enum MT_FE_MOD_MODE {
	MtFeModMode_Undef = -1,
	MtFeModMode_Qpsk = QPSK,
	MtFeModMode_16Qam = QAM_16,
	MtFeModMode_32Qam = QAM_32,
	MtFeModMode_64Qam = QAM_64,
	MtFeModMode_128Qam = QAM_128,
	MtFeModMode_256Qam = QAM_256,
		MtFeModMode_Auto = QAM_AUTO,
	MtFeModMode_8psk = PSK_8,
	MtFeModMode_16Apsk = APSK_16,
	MtFeModMode_32Apsk = APSK_32,
	MtFeModMode_64Apsk = APSK_64,
	MtFeModMode_128Apsk = APSK_128,
	MtFeModMode_256Apsk = APSK_256,
	MtFeModMode_16Apsk_L = APSK_16L,
	MtFeModMode_8Apsk_L = APSK_8L,
	MtFeModMode_32Apsk_L = APSK_32L,
	MtFeModMode_64Apsk_L = APSK_64L,
	MtFeModMode_128Apsk_L = APSK_128L,
	MtFeModMode_256Apsk_L = APSK_256L,
	MtFeModMode_4Qam,
	MtFeModMode_4QamNr,
};
enum MT_FE_LOCK_STATE {
	MtFeLockState_Undef = 0,
	MtFeLockState_Unlocked,
	MtFeLockState_Locked,
	MtFeLockState_Waiting
};

struct m88rs6060_cfg {
	struct dvb_frontend** fe;
	u8 demod_adr;
	u8 tuner_adr;
	enum m88rs6060_ts_mode ts_mode;	// 1:serial 2: parallel 3:common
	/* select ts output pin order
	 * for serial ts mode, swap pin D0 &D7
	 * for parallel or CI mode ,swap the order of D0 ~D7
	 */
	bool ts_pinswitch;
	u32 clk;
	u16 i2c_wr_max;
	u8 envelope_mode;	//for diseqc   default 0
	//0x11 or 0x12 0x11 : there is only one i2c_STOP flag. 0x12 ther are two I2C_STOP flag.
	u8 repeater_value;

	u8 num; // for ci setting;
	bool HAS_CI; // for 6910se ci

	void (*SetSpeedstatus)(struct i2c_adapter * i2c, int tuner);
	void (*SetTimes)(struct i2c_adapter * i2c, int tuner,int times);
	int  (*GetSpeedstatus)(struct i2c_adapter * i2c, int tuner);
	int (*GetSpeed)(struct i2c_adapter * i2c, int tuner);

	void (*write_properties)(struct i2c_adapter * i2c, u8 reg, u32 buf);
	void (*read_properties)(struct i2c_adapter * i2c, u8 reg, u32 * buf);
};

extern struct i2c_client*  m88rs6060_attach(struct i2c_adapter *i2c, struct i2c_board_info* board_info, int adapterno);
extern void m88rs6060_detach(struct dvb_frontend* fe);

#endif
