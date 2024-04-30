/*
 * Driver for the STm STiD135 DVB-S/S2/S2X demodulator.
 *
 * Copyright (C) CrazyCat <crazycat69@narod.ru>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <asm/div64.h>

#include <media/dvb_frontend.h>
#include "stid135.h"
#include "i2c.h"
#include "stid135_drv.h"

LIST_HEAD(stvlist);

static int mode;
module_param(mode, int, 0444);
MODULE_PARM_DESC(mode,
		"Multi-switch mode: 0=quattro/quad 1=normal direct connection");

static unsigned int rfsource;
module_param(rfsource, int, 0644);
MODULE_PARM_DESC(rfsource, "RF source selection for direct connection mode (default:0 - auto)");

static unsigned int mc_auto;
module_param(mc_auto, int, 0644);
MODULE_PARM_DESC(mc_auto, "Enable auto modcode filtering depend from current C/N (default:0 - disabled)");

static unsigned int ts_nosync;
module_param(ts_nosync, int, 0644);
MODULE_PARM_DESC(ts_nosync, "TS FIFO Minimum latence mode (default:off)");

static unsigned int bbframe;
module_param(bbframe, int, 0644);
MODULE_PARM_DESC(bbframe, "BBFrame L3 encapsulation for GCS, GSE-HEM (default:off)");

struct stv_base {
	struct list_head     stvlist;

	u8                   adr;
	struct i2c_adapter  *i2c;
	struct mutex         status_lock;
	int                  count;
	u32                  extclk;
	u8                   ts_mode;

	int (*set_voltage)(struct i2c_adapter *i2c,
		enum fe_sec_voltage voltage, u8 rf_in);
	u8                   mode;

	fe_stid135_handle_t  handle;
	u8 tuner_use_count[4];
	STCHIP_Handle_t vglna_handle[4];

	void (*write_properties) (struct i2c_adapter *i2c,u8 reg, u32 buf);
	void (*read_properties) (struct i2c_adapter *i2c,u8 reg, u32 *buf);

	void (*write_eeprom) (struct i2c_adapter *i2c,u8 reg, u8 buf);
	void (*read_eeprom) (struct i2c_adapter *i2c,u8 reg, u8 *buf);

	//for tbs6912
	void (*set_TSsampling)(struct i2c_adapter *i2c,int tuner,int time);
	u32  (*set_TSparam)(struct i2c_adapter *i2c,int tuner,int time,bool flag);
	//end

	int vglna;

	bool control_22k; //for 6916
};

/*for debugging only: assumes only a single card is in use; otherwise wrong debug output
	will be printed, but apart from that behaviour will still be correct
*/
static atomic_t llr_rate_sum;

struct stv {
	struct stv_base     *base;
	struct dvb_frontend  fe;
	int                  nr;
	int                  rf_in;
	unsigned long        stats_time;
	int current_llr_rate;  //Remember the current reconfiguration to avoid calling hardware needlessly
	int current_max_llr_rate;  //Remember the current reconfiguration to avoid calling hardware needlessly
	struct fe_sat_signal_info signal_info;
    STCHIP_Handle_t vglna_handle;

	bool newTP; //for tbs6912
	u32  bit_rate; //for tbs6912;
	int loops ;//for tbs6912
};

static int stid135_compute_best_max_llr_rate(struct fe_sat_signal_info* signal_info, int* llr_rate)
{
	/*
		p. 23
		max 258 Bit/s per demod
		8psk: 45 (8 tuners)     3
		qpsk: 30     2
		16apsk: 22.5 4
		32apsk: 18   5
		64apsk: 20   6
	 */
	int bits_per_symbol = 3;
	switch(signal_info->modulation) {
	case FE_SAT_MOD_QPSK:
		bits_per_symbol = 2; break;
	case FE_SAT_MOD_8PSK:
	case FE_SAT_MOD_8PSK_L:
		bits_per_symbol = 3; break;
	case FE_SAT_MOD_16APSK:
	case FE_SAT_MOD_16APSK_L:
		bits_per_symbol = 4; break;
	case FE_SAT_MOD_32APSK:
	case FE_SAT_MOD_32APSK_L:
		bits_per_symbol = 5; break;
	case FE_SAT_MOD_VLSNR:
		bits_per_symbol = 3; break; //????
	case FE_SAT_MOD_64APSK:
	case FE_SAT_MOD_64APSK_L:
		bits_per_symbol = 6; break;
	case FE_SAT_MOD_128APSK:
		bits_per_symbol = 7; break;
	case FE_SAT_MOD_256APSK:
	case FE_SAT_MOD_256APSK_L:
		bits_per_symbol = 8; break;
	case FE_SAT_MOD_1024APSK:
		bits_per_symbol = 10; break;
	default:
		break;
	}

	/*
		We impose a limit as if this demodulator can comsume all available resources. However
		some limits also apply to all demodulators together. This limit is currently not imposed.
	 */
	*llr_rate = (signal_info->symbol_rate * bits_per_symbol + 0500000L)/1000000L;
	if(*llr_rate <= 90)
		return 90;
	if(*llr_rate <= 129)
		return 129;
	if(*llr_rate <= 180)
		return 180;
	return 256;
}



fe_lla_error_t set_maxllr_rate(int line, struct stv *state,	struct fe_sat_signal_info* info) {
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	int old=0;
	int tot;
	int llr_rate = 0;
	int max_llr_rate = info->symbol_rate ==0 ? 0:
		stid135_compute_best_max_llr_rate(info, &llr_rate);

	//	int delta = llr_rate - state->current_llr_rate;
	int delta = max_llr_rate - state->current_max_llr_rate;

	old = atomic_fetch_add(delta, &llr_rate_sum);
	tot = old + delta;

	if(tot> 720) {
		dev_warn(&state->base->i2c->dev, "line %d: demod %d: LDPC budget exceeded tot=%d\n",
						 line, state->nr,  tot);

	}
	if (state->current_max_llr_rate != max_llr_rate) {
		if(max_llr_rate == 0) {
			err |= fe_stid135_set_maxllr_rate(state->base->handle, state->nr +1, 90);
		} else
			if (max_llr_rate == 90) {
			if(state->current_max_llr_rate ==0) {
				//
			} else
				err |= fe_stid135_set_maxllr_rate(state->base->handle, state->nr +1, 90);
		} else {
			err |= fe_stid135_set_maxllr_rate(state->base->handle, state->nr +1, max_llr_rate);
		}
		dev_warn(&state->base->i2c->dev, "line %d: demod %d: set_maxllr_rate=%d (was %d tot=%d) "
						 "mod=%d symbol_rate=%d\n",
						 line, state->nr,
						 max_llr_rate,
						 state->current_max_llr_rate, tot,
						 info->modulation, info->symbol_rate);
	} else {
		dev_warn(&state->base->i2c->dev, "line %d: demod %d + tuner %d: set_maxllr_rate=%d (tot=%d) UNCHANGED "
						 "mod=%d symbol_rate=%d\n",
						 line, state->nr, state->rf_in,
						 max_llr_rate, tot,
						 info->modulation, info->symbol_rate);
	}
	state->current_max_llr_rate = max_llr_rate;
	state->current_llr_rate = llr_rate;
	return err;
}

fe_lla_error_t release_maxllr_rate(int line, struct stv *state) {
	struct fe_sat_signal_info info;
	info.symbol_rate = 0;
	return set_maxllr_rate(line, state,	&info);
}


I2C_RESULT I2cReadWrite(void *pI2CHost, I2C_MODE mode, u8 ChipAddress, u8 *Data, int NbData)
{
	struct stv_base     *base = (struct stv_base *)pI2CHost;
	struct i2c_msg msg = {.addr = ChipAddress>>1, .flags = 0,
			      .buf = Data, .len = NbData};
	int ret;


	if (!base) return I2C_ERR_HANDLE;

	if (mode == I2C_READ)
		msg.flags = I2C_M_RD;

	ret = i2c_transfer(base->i2c, &msg, 1);

	return (ret == 1) ? I2C_ERR_NONE : I2C_ERR_ACK;
}

static int stid135_probe(struct stv *state)
{
	struct fe_stid135_init_param init_params;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params;
	enum device_cut_id cut_id;
	int i;

	STCHIP_Info_t VGLNAChip;
	SAT_VGLNA_InitParams_t vglna_init_params;

	dev_warn(&state->base->i2c->dev, "%s\n", FE_STiD135_GetRevision());

	strcpy(init_params.demod_name,"STiD135");
	init_params.pI2CHost		=	state->base;
	init_params.demod_i2c_adr   	=	state->base->adr ? state->base->adr<<1 : 0xd0;
	init_params.demod_ref_clk  	= 	state->base->extclk ? state->base->extclk : 27;
	init_params.internal_dcdc	=	0;
	init_params.internal_ldo	=	1; // LDO supply is internal on Oxford valid board
	init_params.rf_input_type	=	0xF; // Single ended RF input on Oxford valid board rev2
	init_params.roll_off		=  	FE_SAT_35; // NYQUIST Filter value (used for DVBS1/DSS, DVBS2 is automatic)
	init_params.tuner_iq_inversion	=	FE_SAT_IQ_NORMAL;
	init_params.ts_nosync		=	ts_nosync;
	init_params.bbframe		=	bbframe;
	err = fe_stid135_init(&init_params,&state->base->handle);

	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_init error %d !\n", __func__, err);
		return -EINVAL;
	}

	p_params = state->base->handle;
	p_params->master_lock = &state->base->status_lock;

	err = fe_stid135_get_cut_id(state->base->handle,&cut_id);
	switch(cut_id)
	{
	case STID135_CUT1_0:
		dev_warn(&state->base->i2c->dev, "%s: cut 1.0\n", __func__);
		break;
	case STID135_CUT1_1:
		dev_warn(&state->base->i2c->dev, "%s: cut 1.1\n", __func__);
		break;
	case STID135_CUT1_X:
		dev_warn(&state->base->i2c->dev, "%s: cut 1.x\n", __func__);
		break;
	case STID135_CUT2_0:
		dev_warn(&state->base->i2c->dev, "%s: cut 2.0 \n", __func__);
		break;
	case STID135_CUT2_1:
		dev_warn(&state->base->i2c->dev, "%s: cut 2.1 \n", __func__);
		break;
	case STID135_CUT2_X_UNFUSED:
		dev_warn(&state->base->i2c->dev, "%s: cut 2.x \n", __func__);
		break;
	default:
		dev_warn(&state->base->i2c->dev, "%s: cut ? \n", __func__);
		return -EINVAL;
	}

	if (state->base->ts_mode == TS_STFE) { //DT: This code is called
		dev_warn(&state->base->i2c->dev, "%s: 8xTS to STFE mode init.\n", __func__);
		for(i=0;i<8;i++) {
			err |= fe_stid135_set_ts_parallel_serial(state->base->handle, i+1, FE_TS_PARALLEL_ON_TSOUT_0);
		}
		err |= fe_stid135_enable_stfe(state->base->handle,FE_STFE_OUTPUT0);
		err |= fe_stid135_set_stfe(state->base->handle, FE_STFE_TAGGING_MERGING_MODE, FE_STFE_INPUT1 |
						FE_STFE_INPUT2 |FE_STFE_INPUT3 |FE_STFE_INPUT4| FE_STFE_INPUT5 |
						FE_STFE_INPUT6 |FE_STFE_INPUT7 |FE_STFE_INPUT8 ,FE_STFE_OUTPUT0, 0xDE);
#if 0
		err |= fe_stid135_set_ts_parallel_serial(state->base->handle, FE_SAT_DEMOD_1, FE_TS_PARALLEL_ON_TSOUT_0);
#endif

	} else if (state->base->ts_mode == TS_8SER) { //DT: This code is not called
		dev_warn(&state->base->i2c->dev, "%s: 8xTS serial mode init.\n", __func__);
		for(i=0;i<8;i++) {
			err |= fe_stid135_set_ts_parallel_serial(state->base->handle, i+1, FE_TS_SERIAL_CONT_CLOCK);
		//	err |= fe_stid135_set_maxllr_rate(state->base->handle, i+1, 90);
		}
	} else { //DT: This code is not called
		dev_warn(&state->base->i2c->dev, "%s: 2xTS parallel mode init.\n", __func__);
		err |= fe_stid135_set_ts_parallel_serial(state->base->handle, FE_SAT_DEMOD_3, FE_TS_PARALLEL_PUNCT_CLOCK);
#if 1
		err |= fe_stid135_set_maxllr_rate(state->base->handle, FE_SAT_DEMOD_3, 180);
#endif
		err |= fe_stid135_set_ts_parallel_serial(state->base->handle, FE_SAT_DEMOD_1, FE_TS_PARALLEL_PUNCT_CLOCK);
#if 1
		err |= fe_stid135_set_maxllr_rate(state->base->handle, FE_SAT_DEMOD_1, 180);
#endif
	}

	if (state->base->mode == 0) {
		dev_warn(&state->base->i2c->dev, "%s: multiswitch mode init.\n", __func__);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER1);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER2);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER3);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER4);
		err |= fe_stid135_diseqc_init(state->base->handle,AFE_TUNER1, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(state->base->handle,AFE_TUNER3, FE_SAT_DISEQC_2_3_PWM);
		if(state->base->control_22k){ //202405 dtcheck; also needed for tbs6909x?
			err |= fe_stid135_diseqc_init(state->base->handle,AFE_TUNER2, FE_SAT_22KHZ_Continues);
			err |= fe_stid135_diseqc_init(state->base->handle,AFE_TUNER4, FE_SAT_22KHZ_Continues);
		}
		else{
			err |= fe_stid135_diseqc_init(state->base->handle,AFE_TUNER2, FE_SAT_DISEQC_2_3_PWM);
			err |= fe_stid135_diseqc_init(state->base->handle,AFE_TUNER4, FE_SAT_DISEQC_2_3_PWM);
		}
		if (state->base->set_voltage) {
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_13, 0);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_13, 1);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_18, 2);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_18, 3);
		}
	}

///////////////////*stvvglna*////////////////////////
	if (state->base->vglna) {
		dev_warn(&state->base->i2c->dev, "%s:Init STVVGLNA \n", __func__);

		vglna_init_params.Chip = &VGLNAChip;
		vglna_init_params.Chip->pI2CHost	  =	state->base;
		vglna_init_params.Chip->RepeaterHost = NULL;
		vglna_init_params.Chip->Repeater     = FALSE;
		vglna_init_params.Chip->I2cAddr      = 0xc8;
		vglna_init_params.NbDefVal = STVVGLNA_NBREGS;
		strcpy(vglna_init_params.Chip->Name, "STVVGLNA");
		stvvglna_init(&vglna_init_params, &state->base->vglna_handle[0]);
		stvvglna_set_standby(state->base->vglna_handle[0],0);
		dev_warn(&state->base->i2c->dev, "Initialized STVVGLNA 0 device\n");

        if (state->base->vglna > 1) {
			vglna_init_params.Chip->I2cAddr      = 0xce;
			stvvglna_init(&vglna_init_params, &state->base->vglna_handle[1]);
			stvvglna_set_standby(state->base->vglna_handle[1],0);
			dev_warn(&state->base->i2c->dev, "Initialized STVVGLNA 1 device\n");

			vglna_init_params.Chip->I2cAddr      = 0xcc;
			stvvglna_init(&vglna_init_params, &state->base->vglna_handle[2]);
			stvvglna_set_standby(state->base->vglna_handle[2],0);
			dev_warn(&state->base->i2c->dev, "Initialized STVVGLNA 2 device\n");
        }

		vglna_init_params.Chip->I2cAddr      = 0xca;
		stvvglna_init(&vglna_init_params, &state->base->vglna_handle[3]);
		stvvglna_set_standby(state->base->vglna_handle[3],0);
		dev_warn(&state->base->i2c->dev, "Initialized STVVGLNA 3 device\n");
	}

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: setup error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_init(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params = state->base->handle;

	if (state->base->mode == 0)
		return 0;

	dev_dbg(&state->base->i2c->dev, "%s: demod %d + tuner %d\n", __func__, state->nr, state->rf_in);

	mutex_lock(&state->base->status_lock);
	BUG_ON(state->rf_in>3);
	BUG_ON(state->base->tuner_use_count[state->rf_in]>1);
	if(state->base->tuner_use_count[state->rf_in]++ == 0) {
	err |= fe_stid135_tuner_enable(p_params->handle_demod, state->rf_in + 1);
	err |= fe_stid135_diseqc_init(state->base->handle, state->rf_in + 1, FE_SAT_DISEQC_2_3_PWM);
	}
	err |= fe_stid135_set_rfmux_path(p_params->handle_demod, state->nr + 1, state->rf_in + 1);
	mutex_unlock(&state->base->status_lock);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: enable tuner %d + demod %d error %d !\n", __func__, state->rf_in, state->nr, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static void stid135_release(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	int i;
	dev_dbg(&state->base->i2c->dev, "%s: demod %d\n", __func__, state->nr);

	state->base->count--;
	if (state->base->count == 0) {
		for(i = 0; i < 4; i++)
			if (state->base->vglna_handle[i])
				stvvglna_term(state->base->vglna_handle[i]);
		if (state->base->handle)
			FE_STiD135_Term (state->base->handle);
		list_del(&state->base->stvlist);
		kfree(state->base);
	}
	kfree(state);
}

static int stid135_set_parameters(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params = state->base->handle;
	struct fe_sat_search_params search_params;
	struct fe_sat_search_result search_results;
	u32 pls_mode, pls_code;
	s32 rf_power;
	BOOL lock_stat=0;
	struct fe_sat_signal_info signal_info;
#if 0
	s32 current_llr;
#endif
    u32 i, j, m;
    U8 vglna_status;
    S32 vglna_gain;

    dev_dbg(&state->base->i2c->dev,
			"delivery_system=%u modulation=%u frequency=%u symbol_rate=%u inversion=%u stream_id=%d\n",
			p->delivery_system, p->modulation, p->frequency,
					 p->symbol_rate, p->inversion, p->stream_id);
	release_maxllr_rate(__LINE__, state);
	mutex_lock(&state->base->status_lock);

	/* Search parameters */
#if 1
	search_params.search_algo 	= FE_SAT_WARM_START;
#else
	search_params.search_algo 	= FE_SAT_COLD_START;
#endif
	search_params.frequency 	= p->frequency*1000;
	search_params.symbol_rate 	= p->symbol_rate;
	search_params.modulation	= FE_SAT_MOD_UNKNOWN;
	search_params.modcode		= FE_SAT_DUMMY_PLF;
	search_params.search_range	= 10000000;
	search_params.puncture_rate	= FE_SAT_PR_UNKNOWN;
	switch (p->delivery_system)
	{
	  case SYS_DSS:
		search_params.standard		= FE_SAT_SEARCH_DSS;
		break;
	  case SYS_DVBS:
		search_params.standard		= FE_SAT_SEARCH_DVBS1;
		break;
	  case SYS_DVBS2:
		search_params.standard		= FE_SAT_SEARCH_DVBS2;
		break;
	  default:
		search_params.standard		= FE_SAT_AUTO_SEARCH;
	}
	search_params.iq_inversion	= FE_SAT_IQ_AUTO;
	search_params.tuner_index_jump	= 0; // ok with narrow band signal

	err = FE_STiD135_GetLoFreqHz(state->base->handle, &(search_params.lo_frequency));
	search_params.lo_frequency *= 1000000;

	err |= fe_stid135_unlock(state->base->handle, state->nr + 1);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_unlock error %d !\n", __func__, err);

	dev_dbg(&state->base->i2c->dev, "%s: demod %d + tuner %d\n", __func__, state->nr, state->rf_in);
	err |= fe_stid135_set_rfmux_path(p_params->handle_demod, state->nr + 1, state->rf_in + 1);

	if (p->stream_id == NO_STREAM_ID_FILTER) {
		pls_mode = 0;
		pls_code = 1;
	}
	else {
		pls_mode = (p->stream_id>>26) & 0x3;
		pls_code = (p->stream_id>>8) & 0x3FFFF;
		if (pls_mode == 0 && pls_code == 0)
			pls_code = 1;
	}

	if (p->scrambling_sequence_index) {
		pls_mode = 1;
		pls_code = p->scrambling_sequence_index;
#ifdef TOTEST
#else //Deep Thought: code must be moved inside of if test
		/* Set PLS before search */
		dev_dbg(&state->base->i2c->dev, "%s: set pls_mode %d, pls_code %d !\n", __func__, pls_mode, pls_code);
		err |= fe_stid135_set_pls(state->base->handle, state->nr + 1, pls_mode, pls_code);
#endif
	}
#ifdef TOTEST //Deep Thought: code must be moved outside of if test or multi-stream does not work
	/* Set PLS before search */
	dev_dbg(&state->base->i2c->dev, "%s: set pls_mode %d, pls_code %d !\n", __func__, pls_mode, pls_code);
	err |= fe_stid135_set_pls(state->base->handle, state->nr + 1, pls_mode, pls_code);
#endif


	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_pls error %d !\n", __func__, err);
#if 1 /*Deep Thought: keeping filters ensures that  a demod does not cause a storm of data when demodulation is
				failing (e.g., ran fade) This could cause other demods to fail as well as they share resources.
				filter_forbidden_modcodes may be better

				There could be reasons why  fe_stid135_reset_modcodes_filter is still needed, e.g., when  too strict filters
				are left from an earlier tune?
			*/
	err |= fe_stid135_reset_modcodes_filter(state->base->handle, state->nr + 1);
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_reset_modcodes_filter error %d !\n", __func__, err);

#endif
	err |= fe_stid135_search(state->base->handle, state->nr + 1, &search_params, &search_results, 0);
	err |= fe_stid135_get_lock_status(state->base->handle, state->nr + 1,&lock_stat );
	if (err != FE_LLA_NO_ERROR)
	{
		mutex_unlock(&state->base->status_lock);
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_search error %d !\n", __func__, err);
		return -1;
	}

	if (state->base->vglna_handle[state->rf_in]) {
		err |= stvvglna_get_status(state->base->vglna_handle[state->rf_in], &vglna_status);
		err |= stvvglna_get_gain(state->base->vglna_handle[state->rf_in], &vglna_gain);
		dev_dbg(&state->base->i2c->dev, "%s: VGLNA status %i, gain %d dB !\n", __func__, vglna_status, vglna_gain);
    }

	if (search_results.locked){
#if 1
		dev_dbg(&state->base->i2c->dev, "%s: locked !\n", __func__);
		fe_stid135_get_signal_info(state->base->handle, state->nr + 1, &signal_info,0);
		if(state->base->ts_mode == TS_STFE){
			//set maxllr,when the  demod locked ,allocation of resources
			/*Deep Thought: the value should be based on the symbo rate of the currently
				tuned mux
			*/
			/*DeepThought : without this code it is not possible to tune the italian multistreams on 5.0W

        11230H@51.5E has a symbolrate of 45MS/s (bitrate of 119MB/s) and requires a setting of at least 135;
				The next higher value = 180.
				12522V@5.0W has a symbolrate of 35.5MS and requires at least 3*35.5 = 100.5, but also does not work with
				129.
			*/
			err |= set_maxllr_rate(__LINE__, state,	&signal_info);
		}
#else
		dev_dbg(&state->base->i2c->dev, "%s: locked !\n", __func__);

		//set maxllr,when the  demod locked ,allocation of resources
		//err |= fe_stid135_set_maxllr_rate(state->base->handle, state->nr +1, 180);
		get_current_llr(state->base->handle, state->nr +1, &current_llr);

		//for tbs6912

#endif
		state->newTP = true;
		state->loops = 15;
		if(state->base->set_TSsampling)
			state->base->set_TSsampling(state->base->i2c,state->nr/2,4);   //for tbs6912
    }
	else {
		err |= fe_stid135_get_band_power_demod_not_locked(state->base->handle, state->nr + 1, &rf_power);
		dev_dbg(&state->base->i2c->dev, "%s: not locked, band rf_power %d dBm ! demod=%d tuner=%d\n",
						 __func__, rf_power / 1000, state->nr, state->rf_in);
	}

	/* Set modcode after search */
	if (p->modcode != MODCODE_ALL) {
        m = p->modcode;
        j = 0;
        dev_dbg(&state->base->i2c->dev, "%s: set Modcode mask %x!\n", __func__, p->modcode);
        m >>= 1;
        for (i=FE_SAT_QPSK_14; i < FE_SAT_MODCODE_UNKNOWN; i ++) {
            if (m & 1) {
                dev_dbg(&state->base->i2c->dev, "%s: Modcode %02x enabled!\n", __func__, i);
                state->modcode_mask[j].mod_code = i;
                state->modcode_mask[j].pilots = FE_SAT_PILOTS_OFF;
                state->modcode_mask[j].frame_length = FE_SAT_NORMAL_FRAME;
                state->modcode_mask[j+1].mod_code = i;
                state->modcode_mask[j+1].pilots = FE_SAT_PILOTS_ON;
                state->modcode_mask[j+1].frame_length = FE_SAT_NORMAL_FRAME;
                state->modcode_mask[j+2].mod_code = i;
                state->modcode_mask[j+2].pilots = FE_SAT_PILOTS_OFF;
                state->modcode_mask[j+2].frame_length = FE_SAT_SHORT_FRAME;
                state->modcode_mask[j+3].mod_code = i;
                state->modcode_mask[j+3].pilots = FE_SAT_PILOTS_ON;
                state->modcode_mask[j+3].frame_length = FE_SAT_SHORT_FRAME;
                j+=4;
            }
            m >>= 1;
        }
        err |= fe_stid135_set_modcodes_filter(state->base->handle, state->nr + 1, modcode_mask, j);
        if (err != FE_LLA_NO_ERROR)
            dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_modcodes_filter error %d !\n", __func__, err);
	}

	/* Set ISI after search */
	if (p->stream_id != NO_STREAM_ID_FILTER) {
		dev_warn(&state->base->i2c->dev, "%s: set ISI %d ! demod=%d tuner=%d\n", __func__, p->stream_id & 0xFF,
						 state->nr, state->rf_in);
		err |= fe_stid135_set_mis_filtering(state->base->handle, state->nr + 1, TRUE, p->stream_id & 0xFF, 0xFF);
	} else {
		dev_dbg(&state->base->i2c->dev, "%s: disable ISI filtering !\n", __func__);
		err |= fe_stid135_set_mis_filtering(state->base->handle, state->nr + 1, FALSE, 0, 0xFF);
	}
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_mis_filtering error %d !\n", __func__, err);

	mutex_unlock(&state->base->status_lock);
	return err != FE_LLA_NO_ERROR ? -1 : 0;

}

static int stid135_get_frontend(struct dvb_frontend *fe, struct dtv_frontend_properties *p)
{
	struct stv *state = fe->demodulator_priv;

	if (!state->signal_info.locked)
		return 0;

	switch (state->signal_info.standard) {
	case FE_SAT_DSS_STANDARD:
		p->delivery_system = SYS_DSS;
		break;
	case FE_SAT_DVBS2_STANDARD:
		p->delivery_system = SYS_DVBS2;
		break;
	case FE_SAT_DVBS1_STANDARD:
	default:
		p->delivery_system = SYS_DVBS;
	}

	switch (state->signal_info.modulation) {
	case FE_SAT_MOD_8PSK:
		p->modulation = PSK_8;
		break;
	case FE_SAT_MOD_16APSK:
		p->modulation = APSK_16;
		break;
	case FE_SAT_MOD_32APSK:
		p->modulation = APSK_32;
		break;
	case FE_SAT_MOD_64APSK:
		p->modulation = APSK_64;
		break;
	case FE_SAT_MOD_128APSK:
		p->modulation = APSK_128;
		break;
	case FE_SAT_MOD_256APSK:
		p->modulation = APSK_256;
		break;
	case FE_SAT_MOD_1024APSK:
		p->modulation = APSK_1024;
		break;
	case FE_SAT_MOD_8PSK_L:
		p->modulation = APSK_8L;
		break;
	case FE_SAT_MOD_16APSK_L:
		p->modulation = APSK_16L;
		break;
	case FE_SAT_MOD_32APSK_L:
		p->modulation = APSK_32L;
		break;
	case FE_SAT_MOD_64APSK_L:
		p->modulation = APSK_64L;
		break;
	case FE_SAT_MOD_256APSK_L:
		p->modulation = APSK_256L;
		break;
	case FE_SAT_MOD_QPSK:
	default:
		p->modulation = QPSK;
	}

	switch (state->signal_info.roll_off) {
	  case FE_SAT_05:
		p->rolloff = ROLLOFF_5;
		break;
	  case FE_SAT_10:
		p->rolloff = ROLLOFF_10;
		break;
	  case FE_SAT_15:
		p->rolloff = ROLLOFF_15;
		break;
	  case FE_SAT_20:
		p->rolloff = ROLLOFF_20;
		break;
	  case FE_SAT_25:
		p->rolloff = ROLLOFF_25;
		break;
	  case FE_SAT_35:
		p->rolloff = ROLLOFF_35;
		break;
	  default:
		p->rolloff = ROLLOFF_AUTO;
	}

	p->inversion = state->signal_info.spectrum == FE_SAT_IQ_SWAPPED ? INVERSION_ON : INVERSION_OFF;
	if (p->delivery_system == SYS_DVBS2) {
		enum fe_code_rate modcod2fec[0x20] = {
			FEC_NONE, FEC_1_4, FEC_1_3, FEC_2_5,
			FEC_1_2, FEC_3_5, FEC_2_3, FEC_3_4,
			FEC_4_5, FEC_5_6, FEC_8_9, FEC_9_10,
			FEC_3_5, FEC_2_3, FEC_3_4, FEC_5_6,
			FEC_8_9, FEC_9_10, FEC_2_3, FEC_3_4,
			FEC_4_5, FEC_5_6, FEC_8_9, FEC_9_10,
			FEC_3_4, FEC_4_5, FEC_5_6, FEC_8_9,
			FEC_9_10
		};
		if (state->signal_info.modcode < FE_SAT_MODCODE_UNKNOWN)
			p->fec_inner = modcod2fec[state->signal_info.modcode];
		else
			p->fec_inner = FEC_AUTO;
		p->pilot = state->signal_info.pilots == FE_SAT_PILOTS_ON ? PILOT_ON : PILOT_OFF;
	}
	else {
		switch (state->signal_info.puncture_rate) {
		case FE_SAT_PR_1_2:
			p->fec_inner = FEC_1_2;
			break;
		case FE_SAT_PR_2_3:
			p->fec_inner = FEC_2_3;
			break;
		case FE_SAT_PR_3_4:
			p->fec_inner = FEC_3_4;
			break;
		case FE_SAT_PR_5_6:
			p->fec_inner = FEC_5_6;
			break;
		case FE_SAT_PR_6_7:
			p->fec_inner = FEC_6_7;
			break;
		case FE_SAT_PR_7_8:
			p->fec_inner = FEC_7_8;
			break;
		default:
			p->fec_inner = FEC_NONE;
		}
	}

	return 0;
}

static int stid135_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	u32 speed;

	*status = 0;
	if (!mutex_trylock(&state->base->status_lock)) {
		if (state->signal_info.locked)
			*status |= FE_HAS_SIGNAL | FE_HAS_CARRIER
					| FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		return 0;
	}

	p->strength.len = 1;
	p->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->cnr.len = 1;
	p->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->pre_bit_error.len =1;
	p->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->pre_bit_count.len =1;
	p->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	err = fe_stid135_get_lock_status(state->base->handle, state->nr + 1, &state->signal_info.locked);
	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state->base->i2c->dev, "fe_stid135_get_lock_status error\n");
		mutex_unlock(&state->base->status_lock);
		return -EIO;
	}

	if (state->signal_info.locked) {
		/* demod has lock */
		*status |= FE_HAS_SIGNAL | FE_HAS_CARRIER
					| FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;


		if (err != FE_LLA_NO_ERROR) {
			dev_warn(&state->base->i2c->dev, "%s: fe_stid135_get_band_power_demod_not_locked error %d !\n", __func__, err);
            mutex_unlock(&state->base->status_lock);
			return 0;
		}

		err = fe_stid135_get_signal_quality(state->base->handle, state->nr + 1, &state->signal_info, mc_auto);
		mutex_unlock(&state->base->status_lock);
		if (err != FE_LLA_NO_ERROR) {
			dev_err(&state->base->i2c->dev, "fe_stid135_get_signal_quality error\n");
			return -EIO;
		}

	//ss	state->signal_info.power = 800;
	//	state->signal_info.C_N =909;

		p->strength.len = 2;
		p->strength.stat[0].scale = FE_SCALE_DECIBEL;
		p->strength.stat[0].svalue = state->signal_info.power;

		p->strength.stat[1].scale = FE_SCALE_RELATIVE;
		p->strength.stat[1].uvalue = (100 + state->signal_info.power/1000) * 656;

		p->cnr.len = 2;
		p->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		p->cnr.stat[0].svalue = state->signal_info.C_N * 100;

		p->cnr.stat[1].scale = FE_SCALE_RELATIVE;
		p->cnr.stat[1].uvalue = state->signal_info.C_N * 328;
		if (p->cnr.stat[1].uvalue > 0xffff)
			p->cnr.stat[1].uvalue = 0xffff;

		p->post_bit_error.len = 1;
		p->post_bit_error.stat[0].scale = FE_SCALE_COUNTER;
		p->post_bit_error.stat[0].uvalue = state->signal_info.ber;

		if (mc_auto && state->signal_info.standard == FE_SAT_DVBS2_STANDARD) {
			err = fe_stid135_filter_forbidden_modcodes(state->base->handle, state->nr + 1, state->signal_info.C_N * 10);

			if (err != FE_LLA_NO_ERROR)
				dev_warn(&state->base->i2c->dev, "%s: fe_stid135_filter_forbidden_modcodes error %d !\n", __func__, err);
		}
	} else {
		/* demod not locked */
		*status |= FE_HAS_SIGNAL;

		err = fe_stid135_get_band_power_demod_not_locked(state->base->handle, state->nr + 1, &state->signal_info.power);
		// if unlocked, set to lowest resource..
		mutex_unlock(&state->base->status_lock);
		if (err != FE_LLA_NO_ERROR) {
			dev_err(&state->base->i2c->dev, "fe_stid135_get_band_power_demod_not_locked error\n");
			return -EIO;
		}
		//for the tbs6912 ts setting
		if((state->base->set_TSparam)&&(state->newTP)) {
			speed = state->base->set_TSparam(state->base->i2c,state->nr/2,4,0);
			if(!state->bit_rate)
				state->bit_rate = speed;
			if((((speed-state->bit_rate)<160)&&((speed-state->bit_rate)>3))||(state->loops==0)) {
				state->base->set_TSparam(state->base->i2c,state->nr/2,4,1);
				state->newTP = false;
				state->bit_rate  = 0;
			} else {
				state->bit_rate = speed;
			state->loops--;
			}
		}
		p->strength.len = 2;
		p->strength.stat[0].scale = FE_SCALE_DECIBEL;
		p->strength.stat[0].svalue = state->signal_info.power;

		p->strength.stat[1].scale = FE_SCALE_RELATIVE;
		p->strength.stat[1].uvalue = (100 + state->signal_info.power/1000) * 656;
	}
	return 0;
}

static int stid135_tune(struct dvb_frontend *fe, bool re_tune,
		unsigned int mode_flags,
		unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	int r;

	if (re_tune) {
		r = stid135_set_parameters(fe);
		if (r)
			return r;
	}

	r = stid135_read_status(fe, status);
	if (r)
		return r;

	if (*status & FE_HAS_LOCK)
		return 0;

	*delay = HZ;

	return 0;
}


static enum dvbfe_algo stid135_get_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int stid135_set_voltage(struct dvb_frontend *fe, enum fe_sec_voltage voltage)
{
	struct stv *state = fe->demodulator_priv;
	if (state->base->mode == 0)
	{
		if (voltage == SEC_VOLTAGE_18)
			state->rf_in |= 2;
		else
			state->rf_in &= ~2;
		return 0;
	}

	if (state->base->set_voltage) {
		mutex_lock(&state->base->status_lock); //DeepThought: this performs multiple i2c calls and needs to be protected
		state->base->set_voltage(state->base->i2c, voltage, state->rf_in);
		mutex_unlock(&state->base->status_lock);
	}
	return 0;
}

static int stid135_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

	if (state->base->mode == 0)
	{
		if (tone == SEC_TONE_ON)
			state->rf_in |= 1;
		else
			state->rf_in &= ~1;

		return 0;
	}
	if(state->base->control_22k==false)   //for 6916 demod1 ,disable the 22k function.
		return 0;
	mutex_lock(&state->base->status_lock);
	err = fe_stid135_set_22khz_cont(state->base->handle,state->rf_in + 1, tone == SEC_TONE_ON);
	mutex_unlock(&state->base->status_lock);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_22khz_cont error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_send_master_cmd(struct dvb_frontend *fe,
			   struct dvb_diseqc_master_cmd *cmd)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

#if 0
	if (state->base->mode == 0)
		return 0;
#endif

	mutex_lock(&state->base->status_lock);
	err |= fe_stid135_diseqc_init(state->base->handle, state->rf_in + 1, FE_SAT_DISEQC_2_3_PWM);
	err |= fe_stid135_diseqc_send(state->base->handle, state->rf_in + 1, cmd->msg, cmd->msg_len);
	mutex_unlock(&state->base->status_lock);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_diseqc_send error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_recv_slave_reply(struct dvb_frontend *fe,
			    struct dvb_diseqc_slave_reply *reply)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

#if 0
	if (state->base->mode == 0)
		return 0;
#endif

	mutex_lock(&state->base->status_lock);
	err = fe_stid135_diseqc_receive(state->base->handle, reply->msg, &reply->msg_len);
	mutex_unlock(&state->base->status_lock);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_diseqc_receive error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_send_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

	if (state->base->mode == 0)
		return 0;

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_sleep(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params = state->base->handle;
	err |= release_maxllr_rate(__LINE__, state);

	if (state->base->mode == 0)
		return 0;

	dev_dbg(&state->base->i2c->dev, "%s: tuner %d\n", __func__, state->rf_in);

	mutex_lock(&state->base->status_lock);
	BUG_ON(state->rf_in>3);
	BUG_ON(state->base->tuner_use_count[state->rf_in]==0);
	if(state->base->tuner_use_count[state->rf_in]>0)
		state->base->tuner_use_count[state->rf_in]--;
	if(state->base->tuner_use_count[state->rf_in]==0)
		err = FE_STiD135_TunerStandby(p_params->handle_demod, state->rf_in + 1, 0);
	mutex_unlock(&state->base->status_lock);

	if (err != FE_LLA_NO_ERROR)
		dev_warn(&state->base->i2c->dev, "%s: STiD135 standby tuner %d failed!\n", __func__, state->rf_in);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*strength = 0;
	for (i=0; i < p->strength.len; i++) {
		if (p->strength.stat[i].scale == FE_SCALE_RELATIVE)
			*strength = (u16)p->strength.stat[i].uvalue;
		else if (p->strength.stat[i].scale == FE_SCALE_DECIBEL)
			*strength = ((100000 + (s32)p->strength.stat[i].svalue)/1000) * 656;
	}

	return 0;
}

static int stid135_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*snr = 0;
	for (i=0; i < p->cnr.len; i++)
		if (p->cnr.stat[i].scale == FE_SCALE_RELATIVE)
		  *snr = (u16)p->cnr.stat[i].uvalue;
	return 0;
}

static int stid135_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*ber = 1;
	for (i=0; i < p->post_bit_error.len; i++)
		if ( p->post_bit_error.stat[0].scale == FE_SCALE_COUNTER )
			*ber = (u32)p->post_bit_error.stat[0].uvalue;

	return 0;
}

static int stid135_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

static void spi_read(struct dvb_frontend *fe, struct ecp3_info *ecp3inf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;

	if (state->base->read_properties)
		state->base->read_properties(adapter,ecp3inf->reg, &(ecp3inf->data));
	return ;
}

static void spi_write(struct dvb_frontend *fe,struct ecp3_info *ecp3inf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;

	if (state->base->write_properties)
		state->base->write_properties(adapter,ecp3inf->reg, ecp3inf->data);
	return ;
}

static void eeprom_read(struct dvb_frontend *fe, struct eeprom_info *eepinf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;

	if (state->base->read_eeprom)
		state->base->read_eeprom(adapter,eepinf->reg, &(eepinf->data));
	return ;
}

static void eeprom_write(struct dvb_frontend *fe,struct eeprom_info *eepinf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;

	if (state->base->write_eeprom)
		state->base->write_eeprom(adapter,eepinf->reg, eepinf->data);
	return ;
}

static int stid135_read_temp(struct dvb_frontend *fe, s16 *temp)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

	mutex_lock(&state->base->status_lock);
	err = fe_stid135_get_soc_temperature(state->base->handle, temp);
	mutex_unlock(&state->base->status_lock);

	if (err != FE_LLA_NO_ERROR)
		dev_warn(&state->base->i2c->dev, "%s: fe_stid135_get_soc_temperature error %d !\n", __func__, err);
	return 0;
}


static struct dvb_frontend_ops stid135_ops = {
	.delsys = { SYS_DVBS, SYS_DVBS2, SYS_DSS },
	.info = {
		.name			= "STiD135 Multistandard",
		.frequency_min_hz	 = 950 * MHz,
		.frequency_max_hz 	= 2150 * MHz,
		.symbol_rate_min	= 100000,
		.symbol_rate_max	= 520000000,
		.caps			= FE_CAN_INVERSION_AUTO |
					  FE_CAN_FEC_AUTO       |
					  FE_CAN_QPSK           |
					  FE_CAN_2G_MODULATION  |
					  FE_CAN_MULTISTREAM
	},
	.init				= stid135_init,
	.sleep				= stid135_sleep,
	.release                        = stid135_release,
	.get_frontend_algo              = stid135_get_algo,
	.get_frontend                   = stid135_get_frontend,
	.tune                           = stid135_tune,
	.set_tone			= stid135_set_tone,
	.set_voltage			= stid135_set_voltage,

	.diseqc_send_master_cmd		= stid135_send_master_cmd,
	.diseqc_send_burst		= stid135_send_burst,
	.diseqc_recv_slave_reply	= stid135_recv_slave_reply,

	.read_status			= stid135_read_status,
	.read_signal_strength		= stid135_read_signal_strength,
	.read_snr			= stid135_read_snr,
	.read_ber			= stid135_read_ber,
	.read_ucblocks			= stid135_read_ucblocks,
	.spi_read			= spi_read,
	.spi_write			= spi_write,
	.eeprom_read			= eeprom_read,
	.eeprom_write			= eeprom_write,
    .read_temp			= stid135_read_temp,
};

static struct stv_base *match_base(struct i2c_adapter  *i2c, u8 adr)
{
	struct stv_base *p;

	list_for_each_entry(p, &stvlist, stvlist)
		if (p->i2c == i2c && p->adr == adr)
			return p;
	return NULL;
}

//DT: called with  nr=adapter=0...7 and rf_in = nr/2=0...3
struct dvb_frontend *stid135_attach(struct i2c_adapter *i2c,
				    struct stid135_cfg *cfg,
				    int nr, int rf_in)
{
	struct stv *state;
	struct stv_base *base;

	state = kzalloc(sizeof(struct stv), GFP_KERNEL);
	if (!state)
		return NULL;

	base = match_base(i2c, cfg->adr);
	if (base) {
		base->count++;
		state->base = base;
	} else {
		base = kzalloc(sizeof(struct stv_base), GFP_KERNEL);
		if (!base)
			goto fail;
		base->i2c = i2c;
		base->adr = cfg->adr;
		base->count = 1;
		base->extclk = cfg->clk;
		base->ts_mode = cfg->ts_mode;
		base->set_voltage = cfg->set_voltage;
		base->mode = cfg->set_voltage ? mode : 1;
		base->write_properties = cfg->write_properties;
		base->read_properties = cfg->read_properties;
		base->write_eeprom = cfg->write_eeprom;
		base->read_eeprom = cfg->read_eeprom;
		base->set_TSsampling = cfg->set_TSsampling;
		base->set_TSparam  = cfg->set_TSparam;
		base->vglna		=	cfg->vglna;    //for stvvglna 6909x v2 6903x v2
		base->control_22k	= cfg->control_22k;

		mutex_init(&base->status_lock);

		state->base = base;
		if (stid135_probe(state) < 0) {
			dev_warn(&i2c->dev, "No demod found at adr %02X on %s\n",
				 cfg->adr, dev_name(&i2c->dev));
			kfree(base);
			goto fail;
		}
		list_add(&base->stvlist, &stvlist);
	}
	state->fe.ops               = stid135_ops;
	state->fe.demodulator_priv  = state;
	state->nr = nr;
	state->newTP = false;
	state->bit_rate  = 0;
	state->loops = 15;
	state->current_max_llr_rate = 0;
	state->current_llr_rate = 0;
	if (rfsource > 0 && rfsource < 5)
		rf_in = rfsource - 1;
	state->rf_in = base->mode ? rf_in : 0;

	if (base->mode == 2)
		state->rf_in = 3;

	dev_info(&i2c->dev, "%s demod found at adr %02X on %s\n",
		 state->fe.ops.info.name, cfg->adr, dev_name(&i2c->dev));

	return &state->fe;

fail:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL_GPL(stid135_attach);

MODULE_DESCRIPTION("STiD135 driver");
MODULE_AUTHOR("CrazyCat");
MODULE_LICENSE("GPL");
