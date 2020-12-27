/*
 * Driver for the ST STV091x DVB-S/S2 demodulator.
 *
 * Copyright (C) 2014-2015 Ralph Metzler <rjkm@metzlerbros.de>
 *                         Marcus Metzler <mocm@metzlerbros.de>
 *                         developed for Digital Devices GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _STV091X_H_
#define _STV091X_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include <media/dvb_frontend.h>

struct stv091x_cfg {
	u32 clk;
	u8  adr;
	u8  parallel;
	u8  rptlvl;
	u8  dual_tuner;

	/* Hook for Lock LED */
	void (*set_lock_led) (struct dvb_frontend *fe, int offon);

	//update the FW.
	void (*write_properties) (struct i2c_adapter *i2c,u8 reg, u32 buf);
	void (*read_properties) (struct i2c_adapter *i2c,u8 reg, u32 *buf);
	// EEPROM access
	void (*write_eeprom) (struct i2c_adapter *i2c,u8 reg, u8 buf);
	void (*read_eeprom) (struct i2c_adapter *i2c,u8 reg, u8 *buf);
};

#if IS_REACHABLE(CONFIG_DVB_STV091X)
extern struct dvb_frontend *stv091x_attach(struct i2c_adapter *i2c,
					   struct stv091x_cfg *cfg, int nr);
#else
static inline struct dvb_frontend *stv091x_attach(struct i2c_adapter *i2c,
						  struct stv091x_cfg *cfg,
						  int nr)
{
	pr_warn("%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}


#endif

struct stv_base {
	struct list_head     stvlist;

	u8                   adr;
	struct i2c_adapter  *i2c;
	struct mutex         i2c_lock;
	struct mutex         reg_lock;
	int                  count;

	u32                  extclk;
	u32                  mclk;

	u8                   dual_tuner;

	/* Hook for Lock LED */
	void (*set_lock_led) (struct dvb_frontend *fe, int offon);

	void (*write_properties) (struct i2c_adapter *i2c,u8 reg, u32 buf);
	void (*read_properties) (struct i2c_adapter *i2c,u8 reg, u32 *buf);

	void (*write_eeprom) (struct i2c_adapter *i2c,u8 reg, u8 buf);
	void (*read_eeprom) (struct i2c_adapter *i2c,u8 reg, u8 *buf);
};

enum ReceiveMode { Mode_None, Mode_DVBS, Mode_DVBS2, Mode_Auto };

enum DVBS2_FECType { DVBS2_64K, DVBS2_16K };

enum DVBS2_ModCod {
	DVBS2_DUMMY_PLF, DVBS2_QPSK_1_4, DVBS2_QPSK_1_3, DVBS2_QPSK_2_5,
	DVBS2_QPSK_1_2, DVBS2_QPSK_3_5, DVBS2_QPSK_2_3,	DVBS2_QPSK_3_4,
	DVBS2_QPSK_4_5,	DVBS2_QPSK_5_6,	DVBS2_QPSK_8_9,	DVBS2_QPSK_9_10,
	DVBS2_8PSK_3_5,	DVBS2_8PSK_2_3,	DVBS2_8PSK_3_4,	DVBS2_8PSK_5_6,
	DVBS2_8PSK_8_9,	DVBS2_8PSK_9_10, DVBS2_16APSK_2_3, DVBS2_16APSK_3_4,
	DVBS2_16APSK_4_5, DVBS2_16APSK_5_6, DVBS2_16APSK_8_9, DVBS2_16APSK_9_10,
	DVBS2_32APSK_3_4, DVBS2_32APSK_4_5, DVBS2_32APSK_5_6, DVBS2_32APSK_8_9,
	DVBS2_32APSK_9_10
};

enum FE_STV0910_ModCod {
	FE_DUMMY_PLF, FE_QPSK_14, FE_QPSK_13, FE_QPSK_25,
	FE_QPSK_12, FE_QPSK_35, FE_QPSK_23, FE_QPSK_34,
	FE_QPSK_45, FE_QPSK_56, FE_QPSK_89, FE_QPSK_910,
	FE_8PSK_35, FE_8PSK_23, FE_8PSK_34, FE_8PSK_56,
	FE_8PSK_89, FE_8PSK_910, FE_16APSK_23, FE_16APSK_34,
	FE_16APSK_45, FE_16APSK_56, FE_16APSK_89, FE_16APSK_910,
	FE_32APSK_34, FE_32APSK_45, FE_32APSK_56, FE_32APSK_89,
	FE_32APSK_910
};

enum FE_STV0910_RollOff { FE_SAT_35, FE_SAT_25, FE_SAT_20, FE_SAT_15 };

struct stv_spectrum_scan_state {
	bool spectrum_present;
	bool scan_in_progress;

	s32* freq;
	s32* spectrum;
	int spectrum_len;

};

struct stv_constellation_scan_state {
	bool constallation_present;
	bool in_progress;

	struct dtv_fe_constellation_sample* samples;
	int num_samples;
	int samples_len;
	int constel_select;
};


struct stv {
	struct stv_base     *base;
	struct dvb_frontend  fe;
	int                  nr;
	u16                  regoff;
	u8                   i2crpt;
	u8                   tscfgh;
	u8                   tsgeneral;
	unsigned long        tune_time;

	s32		tuner_bw;
	s32   search_range_hz;
	u32                  Started;
	u32                  DemodLockTime;
	enum ReceiveMode     ReceiveMode;
	u32                  DemodTimeout;
	s32                  FecTimeout;
	s32                  FirstTimeLock;
	u8                   DEMOD;
	u32                  symbol_rate;

	u8                      LastViterbiRate;
	enum fe_code_rate       PunctureRate;
	enum FE_STV0910_ModCod  ModCod;
	enum DVBS2_FECType      FECType;
	u32                     Pilots;
	enum FE_STV0910_RollOff FERollOff;

	u32   LastBERNumerator;
	u32   LastBERDenominator;
	u8    BERScale;
	bool timedout;
	bool fec_locked;
	bool demod_locked;
	bool satellite_scan;
	s32 scan_next_frequency;
	s32 scan_end_frequency;

	struct stv_spectrum_scan_state scan_state;
	struct stv_constellation_scan_state constellation_scan_state;
};

struct reg_field {
	u32 field_id;
	s32 val;
};

int  write_reg_fields_(struct stv* state, u16 addr, struct reg_field* fields, int num_fields);

#define write_reg_fields(state, addr, fields...)												\
	({struct reg_field temp[] = {fields};																	\
		write_reg_fields_(state, addr, &temp[0], sizeof(temp)/sizeof(temp[0]));})

int write_reg(struct stv *state, u16 reg, u8 val);
int read_regs(struct stv *state, u16 reg, u8 *val, int len);
u8 read_reg(struct stv *state, u16 reg);
int try_read_reg(struct stv *state, u16 reg, u8* val);
s32 read_reg_field(struct stv* state, u32 field_id);

int  write_reg_field(struct stv* state, u32 field_id, s32 val);

s32 STLog10(u32 value);

#endif
