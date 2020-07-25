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
#include <linux/kthread.h>
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
#include "stid135-fft.h"
#define MAX_FFT_SIZE 8192
#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

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


/*for debugging only: assumes only a single card is in use; otherwise wrong debug output
	will be printed, but apart from that behaviour will still be correct
*/
static atomic_t llr_rate_sum;


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
			err |= fe_stid135_set_maxllr_rate(state, 90);
		} else
			if (max_llr_rate == 90) {
			if(state->current_max_llr_rate ==0) {
				//
			} else
				err |= fe_stid135_set_maxllr_rate(state, 90);
		} else {
			err |= fe_stid135_set_maxllr_rate(state, max_llr_rate);
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

//called once per chip
static int stid135_probe(struct stv *state_demod1)
{
	struct fe_stid135_init_param init_params;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params;
	enum device_cut_id cut_id;
	int i;
	char *VglnaIdString = NULL;
	STCHIP_Info_t VGLNAChip;
	STCHIP_Info_t* vglna_handle;
	STCHIP_Info_t* vglna_handle1;
	STCHIP_Info_t* vglna_handle2;
	STCHIP_Info_t* vglna_handle3;
	SAT_VGLNA_InitParams_t pVGLNAInit;
	SAT_VGLNA_InitParams_t pVGLNAInit1;
	SAT_VGLNA_InitParams_t pVGLNAInit2;
	SAT_VGLNA_InitParams_t pVGLNAInit3;
	dprintk("HERE state_demod1=%p\n", state_demod1);
	dprintk("HERE state->base=%p\n", state_demod1->base);
	dprintk("HERE state->base=%p\n", state_demod1->base->i2c);
	p_params = &state_demod1->base->ip;
	dev_warn(&state_demod1->base->i2c->dev, "%s\n", FE_STiD135_GetRevision());
	strcpy(init_params.demod_name,"STiD135");
	init_params.pI2CHost		=	state_demod1->base;
	init_params.demod_i2c_adr   	=	state_demod1->base->adr ? state_demod1->base->adr<<1 : 0xd0;
	init_params.demod_ref_clk  	= 	state_demod1->base->extclk ? state_demod1->base->extclk : 27;
	init_params.internal_dcdc	=	FALSE;
	init_params.internal_ldo	=	TRUE; // LDO supply is internal on Oxford valid board
	init_params.rf_input_type	=	0xF; // Single ended RF input on Oxford valid board rev2
	init_params.roll_off		=  	FE_SAT_35; // NYQUIST Filter value (used for DVBS1/DSS, DVBS2 is automatic)
	init_params.tuner_iq_inversion	=	FE_SAT_IQ_NORMAL;
#if 0
	err |= fe_stid135_apply_custom_qef_for_modcod_filter(state_demod1, NULL);
#endif
	err = fe_stid135_init(&init_params, &state_demod1->base->ip);
	dprintk("here err=%d\n", err);
	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state_demod1->base->i2c->dev, "%s: fe_stid135_init error %d !\n", __func__, err);
		return -EINVAL;
	}

	//p_params = state_demod1->base->handle;
	p_params->master_lock = &state_demod1->base->status_lock;
	dprintk("here state_demod1=%p\n", state_demod1);
	dprintk("here state_demod1->base=%p\n", state_demod1->base);
	dprintk("here state_demod1->base=%p\n", state_demod1->base);
	err = fe_stid135_get_cut_id(&state_demod1->base->ip,&cut_id);
	switch(cut_id)
	{
	case STID135_CUT1_0:
		dev_warn(&state_demod1->base->i2c->dev, "%s: cut 1.0\n", __func__);
		break;
	case STID135_CUT1_1:
		dev_warn(&state_demod1->base->i2c->dev, "%s: cut 1.1\n", __func__);
		break;
	case STID135_CUT1_X:
		dev_warn(&state_demod1->base->i2c->dev, "%s: cut 1.x\n", __func__);
		break;
	case STID135_CUT2_0:
		dev_warn(&state_demod1->base->i2c->dev, "%s: cut 2.0 \n", __func__);
		break;
	case STID135_CUT2_1:
		dev_warn(&state_demod1->base->i2c->dev, "%s: cut 2.1 \n", __func__);
		break;
	case STID135_CUT2_X_UNFUSED:
		dev_warn(&state_demod1->base->i2c->dev, "%s: cut 2.x \n", __func__);
		break;
	default:
		dev_warn(&state_demod1->base->i2c->dev, "%s: cut ? \n", __func__);
		return -EINVAL;
	}
	if (state_demod1->base->ts_mode == TS_STFE) { //DT: This code is called
		dev_warn(&state_demod1->base->i2c->dev, "%s: 8xTS to STFE mode init.\n", __func__);
#if 1
		for(i=0;i<8;i++) {
			err |= fe_stid135_set_ts_parallel_serial(&state_demod1->base->ip, i+1, FE_TS_PARALLEL_ON_TSOUT_0);
		}
#else // this is code from main driver
		//	for(i=0;i<8;i++) {
		err |= fe_stid135_set_ts_parallel_serial(state_demod1, FE_SAT_DEMOD_1, FE_TS_PARALLEL_ON_TSOUT_0);
		//	err |= fe_stid135_set_maxllr_rate(state_demod1->base->handle, i+1, 260);
		//	}
#endif
		err |= fe_stid135_enable_stfe(&state_demod1->base->ip,FE_STFE_OUTPUT0);
		err |= fe_stid135_set_stfe(&state_demod1->base->ip, FE_STFE_TAGGING_MERGING_MODE, FE_STFE_INPUT1 |
						FE_STFE_INPUT2 |FE_STFE_INPUT3 |FE_STFE_INPUT4| FE_STFE_INPUT5 |
						FE_STFE_INPUT6 |FE_STFE_INPUT7 |FE_STFE_INPUT8 ,FE_STFE_OUTPUT0, 0xDE);
	} else if (state_demod1->base->ts_mode == TS_8SER) { //DT: This code is not called
		dev_warn(&state_demod1->base->i2c->dev, "%s: 8xTS serial mode init.\n", __func__);
		for(i=0;i<8;i++) {
			err |= fe_stid135_set_ts_parallel_serial(&state_demod1->base->ip, i+1, FE_TS_SERIAL_CONT_CLOCK);
			//err |= fe_stid135_set_maxllr_rate(state_demod1, i+1, 90);
		}
	} else { //DT: This code is not called
		dev_warn(&state_demod1->base->i2c->dev, "%s: 2xTS parallel mode init.\n", __func__);
		err |= fe_stid135_set_ts_parallel_serial(&state_demod1->base->ip, FE_SAT_DEMOD_3, FE_TS_PARALLEL_PUNCT_CLOCK);
#if 1
		//err |= fe_stid135_set_maxllr_rate(state_demod1, FE_SAT_DEMOD_3, 180);
#endif
		//err |= fe_stid135_set_ts_parallel_serial(state_demod1, FE_SAT_DEMOD_1, FE_TS_PARALLEL_PUNCT_CLOCK);
#if 1
		//err |= fe_stid135_set_maxllr_rate(state_demod1, FE_SAT_DEMOD_1, 180);
#endif
	}
	if (state_demod1->base->mode == 0) {
		dev_warn(&state_demod1->base->i2c->dev, "%s: multiswitch mode init.\n", __func__);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER1);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER2);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER3);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER4);
		err |= fe_stid135_diseqc_init(&state_demod1->base->ip,AFE_TUNER1, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state_demod1->base->ip,AFE_TUNER2, FE_SAT_22KHZ_Continues);
		err |= fe_stid135_diseqc_init(&state_demod1->base->ip,AFE_TUNER3, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state_demod1->base->ip,AFE_TUNER4, FE_SAT_22KHZ_Continues);
		if (state_demod1->base->set_voltage) {
			state_demod1->base->set_voltage(state_demod1->base->i2c, SEC_VOLTAGE_13, 0);
			state_demod1->base->set_voltage(state_demod1->base->i2c, SEC_VOLTAGE_13, 1);
			state_demod1->base->set_voltage(state_demod1->base->i2c, SEC_VOLTAGE_18, 2);
			state_demod1->base->set_voltage(state_demod1->base->i2c, SEC_VOLTAGE_18, 3);
		}
	}
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state_demod1->base->i2c->dev, "%s: setup error %d !\n", __func__, err);


///////////////////init stvvglna/////////////////////////////////////
	if(state_demod1->base->vglna){ //for 909x v2 version
			dev_warn(&state_demod1->base->i2c->dev, "%s:Init STVVGLNA \n", __func__);
	VglnaIdString = "STVVGLNA";
		/* Init the VGLNA */
	pVGLNAInit.Chip = &VGLNAChip;

	pVGLNAInit.Chip->pI2CHost	  =	state_demod1->base;
	pVGLNAInit.Chip->RepeaterHost = NULL;
	pVGLNAInit.Chip->Repeater     = FALSE;
	pVGLNAInit.Chip->I2cAddr      = 0xc8;
	pVGLNAInit.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit, &vglna_handle);
	printk("Initialized STVVGLNA  0 device\n");
	stvvglna_set_standby(vglna_handle,1);
	dev_warn(&state_demod1->base->i2c->dev, "Initialized STVVGLNA 0 device\n");

	VglnaIdString = "STVVGLNA1";
	pVGLNAInit1.Chip = &VGLNAChip;

	pVGLNAInit1.Chip->pI2CHost	  =	state_demod1->base;
	pVGLNAInit1.Chip->RepeaterHost = NULL;
	pVGLNAInit1.Chip->Repeater     = FALSE;
	pVGLNAInit1.Chip->I2cAddr      = 0xce;
	pVGLNAInit1.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit1.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit1, &vglna_handle1);
	stvvglna_set_standby(vglna_handle1,1);
	dev_warn(&state_demod1->base->i2c->dev, "Initialized STVVGLNA 1 device\n");

	VglnaIdString = "STVVGLNA2";
	pVGLNAInit2.Chip = &VGLNAChip;
	pVGLNAInit2.Chip->pI2CHost	  =	state_demod1->base;
	pVGLNAInit2.Chip->RepeaterHost = NULL;
	pVGLNAInit2.Chip->Repeater     = FALSE;
	pVGLNAInit2.Chip->I2cAddr      = 0xcc;
	pVGLNAInit2.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit2.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit2, &vglna_handle2);
	stvvglna_set_standby(vglna_handle2,1);
	dev_warn(&state_demod1->base->i2c->dev, "Initialized STVVGLNA 2 device\n");

	VglnaIdString = "STVVGLNA3";
	pVGLNAInit3.Chip = &VGLNAChip;
	pVGLNAInit3.Chip->pI2CHost	  =	state_demod1->base;
	pVGLNAInit3.Chip->RepeaterHost = NULL;
	pVGLNAInit3.Chip->Repeater     = FALSE;
	pVGLNAInit3.Chip->I2cAddr      = 0xca;
	pVGLNAInit3.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit3.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit3, &vglna_handle3);
	stvvglna_set_standby(vglna_handle3,1);
	dev_warn(&state_demod1->base->i2c->dev, "Initialized STVVGLNA 3 device\n");
	}
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state_demod1->base->i2c->dev, "%s: setup error %d !\n", __func__, err);
	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_init(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params = &state->base->ip;

	if (state->base->mode == 0)
		return 0;

	dev_dbg(&state->base->i2c->dev, "%s: demod %d + tuner %d\n", __func__, state->nr, state->rf_in);

	mutex_lock(&state->base->status_lock);
	BUG_ON(state->rf_in>3);
	BUG_ON(state->base->tuner_use_count[state->rf_in]>1);
	if(state->base->tuner_use_count[state->rf_in]++ == 0) {
	err |= fe_stid135_tuner_enable(p_params->handle_demod, state->rf_in + 1);
	err |= fe_stid135_diseqc_init(&state->base->ip, state->rf_in + 1, FE_SAT_DISEQC_2_3_PWM);
	}
	err |= fe_stid135_set_rfmux_path(state, state->rf_in + 1);
	mutex_unlock(&state->base->status_lock);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: enable tuner %d + demod %d error %d !\n", __func__, state->rf_in, state->nr, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static void stid135_release(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	dev_dbg(&state->base->i2c->dev, "%s: demod %d\n", __func__, state->nr);

	state->base->count--;
	if (state->base->count == 0) {
		if (&state->base->ip)
			FE_STiD135_Term (&state->base->ip);
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
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *p_params = &state->base->ip;
	struct fe_sat_search_params search_params;
	struct fe_sat_search_result search_results;
	u32 pls_mode, pls_code;
	s32 rf_power;
	BOOL satellite_scan =0;
	//BOOL lock_stat=0;
	struct fe_sat_signal_info* signal_info = &state->signal_info;

	dev_dbg(&state->base->i2c->dev,
			"delivery_system=%u modulation=%u frequency=%u symbol_rate=%u inversion=%u stream_id=%d\n",
			p->delivery_system, p->modulation, p->frequency,
					 p->symbol_rate, p->inversion, p->stream_id);
	release_maxllr_rate(__LINE__, state);
	mutex_lock(&state->base->status_lock);

	/* Search parameters */
#if 1
	switch(p->algorithm) {
	case ALGORITHM_WARM:
		search_params.search_algo		= FE_SAT_WARM_START;
		satellite_scan = 0;
		break;

	case ALGORITHM_COLD:
	case ALGORITHM_COLD_BEST_GUESS:
		search_params.search_algo		= FE_SAT_COLD_START;
		satellite_scan = 0;
		break;

	case ALGORITHM_BLIND:
	case ALGORITHM_BLIND_BEST_GUESS:
	case ALGORITHM_BANDWIDTH:
		search_params.search_algo		= FE_SAT_BLIND_SEARCH;
		search_params.standard = FE_SAT_AUTO_SEARCH;
		search_params.puncture_rate = FE_SAT_PR_UNKNOWN;
		satellite_scan = 0;
		break;
	case ALGORITHM_SEARCH_NEXT:
		search_params.search_algo		= FE_SAT_NEXT;
		search_params.standard = FE_SAT_AUTO_SEARCH;
		search_params.puncture_rate = FE_SAT_PR_UNKNOWN;
		satellite_scan = 0;
		break;
	case ALGORITHM_SEARCH:
		//todo
		break;
	}

#else
	search_params.search_algo		= FE_SAT_WARM_START;
#endif
	search_params.frequency		=  p->frequency*1000;
	search_params.symbol_rate		=		(search_params.search_algo == FE_SAT_BLIND_SEARCH ||
																	 search_params.search_algo == FE_SAT_NEXT
																	 ) ?    30000000/*to prevent error detected*/ : p->symbol_rate;
	search_params.modulation	= FE_SAT_MOD_UNKNOWN;
	search_params.modcode		= FE_SAT_DUMMY_PLF;
	search_params.search_range_hz	= p->search_range > 0 ? p->search_range : 10000000;
	//dprintk("Search range set to %d\n", search_params.search_range_hz);

	search_params.puncture_rate	= FE_SAT_PR_UNKNOWN;
	if(	search_params.standard != FE_SAT_AUTO_SEARCH)
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
	//dprintk("search standard = %d\n", search_params.standard);
	search_params.iq_inversion	= FE_SAT_IQ_AUTO;
	search_params.tuner_index_jump	= 0; // ok with narrow band signal

	err = FE_STiD135_GetLoFreqHz(&state->base->ip, &(search_params.lo_frequency));
	//dprintk("lo_frequency = %d\n", search_params.lo_frequency);
	search_params.lo_frequency *= 1000000;
	if(search_params.search_algo == FE_SAT_BLIND_SEARCH ||
		 search_params.search_algo == FE_SAT_NEXT) {
		//search_params.frequency =		950000000 ;
		printk("BLIND: set freq=%d lo=%d\n",	search_params.frequency,	search_params.lo_frequency  );
	}

	dev_dbg(&state->base->i2c->dev, "%s: demod %d + tuner %d\n", __func__, state->nr, state->rf_in);
	err |= fe_stid135_set_rfmux_path(state, state->rf_in + 1);

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
	signal_info-> pls_code =pls_code;
	signal_info-> pls_mode =pls_mode;

	if (p->scrambling_sequence_index) {
		pls_mode = 1;
		pls_code = p->scrambling_sequence_index;
#if 1 //Deep Thought: code must be moved outside of if test or multi-stream does not work
		/* Set PLS before search */
		dev_dbg(&state->base->i2c->dev, "%s: set pls_mode %d, pls_code %d !\n", __func__, pls_mode, pls_code);
		err |= fe_stid135_set_pls(state, pls_mode, pls_code);
#endif
	}
#if 1 //Deep Thought: code must be moved outside of if test or multi-stream does not work
	/* Set PLS before search */
	dev_dbg(&state->base->i2c->dev, "%s: set pls_mode %d, pls_code %d !\n", __func__, pls_mode, pls_code);
	err |= fe_stid135_set_pls(state, pls_mode, pls_code);
#endif


	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_pls error %d !\n", __func__, err);
#if 1 /*Deep Thought: keeping filters ensures that  a demod does not cause a storm of data when demodulation is
				failing (e.g., ran fade) This could cause other demods to fail as well as they share resources.
				filter_forbidden_modcodes may be better

				There could be reasons why  fe_stid135_reset_modcodes_filter is still needed, e.g., when  too strict filters
				are left from an earlier tune?
			*/
	err |= fe_stid135_reset_modcodes_filter(state);
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_reset_modcodes_filter error %d !\n", __func__, err);
#endif
	err |= (error1=fe_stid135_search(state, &search_params, &search_results, satellite_scan));
	if(error1!=0)
		dprintk("fe_stid135_search returned error=%d\n", error1);
	if (err != FE_LLA_NO_ERROR)
	{
		mutex_unlock(&state->base->status_lock);
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_search error %d !\n", __func__, err);
		return -1;
	}

	dprintk("set_parameters: error=%d locked=%d\n", err, search_results.locked);
	if (search_results.locked){
		state->signal_info.locked = 1;
		state->signal_info.timedout = 0;
		state->signal_info.tuned = 1;
		dprintk("set_parameters: error=%d locked=%d\n", err, state->signal_info.locked);
#if 1
		dev_dbg(&state->base->i2c->dev, "%s: locked !\n", __func__);
		//set maxllr,when the  demod locked ,allocation of resources
		err |= fe_stid135_set_maxllr_rate(state, 180);
		//for tbs6912
#else
		dev_warn(&state->base->i2c->dev, "%s: locked ! demod=%d tuner=%d\n", __func__,  state->nr, state->rf_in);
#endif
		state->newTP = true;
		state->loops = 15;
		if(state->base->set_TSsampling)
			state->base->set_TSsampling(state->base->i2c,state->nr/2,4);   //for tbs6912
		}
	else {
		state->signal_info.tuned = 1;
		state->signal_info.timedout = 1;
		state->signal_info.locked = 0;
		err |= fe_stid135_get_band_power_demod_not_locked(state, &rf_power);
		dev_dbg(&state->base->i2c->dev, "%s: not locked, band rf_power %d dBm ! demod=%d tuner=%d\n",
						 __func__, rf_power / 1000, state->nr, state->rf_in);
	}
	dprintk("set_parameters: error=%d locked=%d\n", err, state->signal_info.locked);
	/* Set ISI before search */
	if (p->stream_id != NO_STREAM_ID_FILTER) {
		dev_warn(&state->base->i2c->dev, "%s: set ISI %d ! demod=%d tuner=%d\n", __func__, p->stream_id & 0xFF,
						 state->nr, state->rf_in);
		err |= fe_stid135_set_mis_filtering(state,  TRUE, p->stream_id & 0xFF, 0xFF);
		signal_info->isi = p->stream_id &0xff;
	} else {
		dev_dbg(&state->base->i2c->dev, "%s: disable ISI filtering !\n", __func__);
		err |= fe_stid135_set_mis_filtering(state,  FALSE, 0, 0xFF);
		signal_info->isi = 0;
	}
	dprintk("set_parameters: error=%d locked=%d\n", err, state->signal_info.locked);
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_mis_filtering error %d !\n", __func__, err);

	mutex_unlock(&state->base->status_lock);
	return err != FE_LLA_NO_ERROR ? -1 : 0;

}

static int stid135_get_frontend(struct dvb_frontend *fe, struct dtv_frontend_properties *p)
{
	struct stv *state = fe->demodulator_priv;
	//struct fe_stid135_internal_param *p_params = &state->base->ip;
	if (!state->signal_info.locked)
		return 0;
	if(state->demod_search_algo == FE_SAT_BLIND_SEARCH ||
		 state->demod_search_algo == FE_SAT_NEXT) {
		int max_isi_len= sizeof(p-> isi)/sizeof(p->isi[0]);
		fe_stid135_get_signal_info(state,  &state->signal_info, 0);
		//dprintk("MIS2: num=%d\n", state->signal_info.isi_list.nb_isi);
		p-> isi_list_len = state->signal_info.isi_list.nb_isi;
		if(p->isi_list_len>  max_isi_len)
			p->isi_list_len = max_isi_len;
		memcpy(p->isi, &state->signal_info.isi_list.isi[0], p->isi_list_len);
		p->frequency = state->signal_info.frequency;
		p->symbol_rate = state->signal_info.symbol_rate;
	}
	switch (state->signal_info.standard) {
	case FE_SAT_DSS_STANDARD:
		p->delivery_system = SYS_DSS;
		break;
	case FE_SAT_DVBS2_STANDARD:
		p->delivery_system = SYS_DVBS2;
		break;
	case FE_SAT_DVBS1_STANDARD:
	default:
		//dprintk("XXXX delivery_system set to DVBS\n");
		p->delivery_system = SYS_DVBS;
			}

	switch (state->signal_info.modulation) {
	case FE_SAT_MOD_8PSK:
		p->modulation = PSK_8;
		break;
	case FE_SAT_MOD_16APSK:
		p->modulation = APSK_16;
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
	p->stream_id = state->signal_info.isi;
	//printk("XXXXX stream_id=%d\n",p->stream_id);
#if 0
	p->pls_mode = state->signal_info.pls_mode;
	p->pls_code = state->signal_info.pls_code;
#endif
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
		dprintk("read status=%d\n", *status);
		return 0;
	}
	//dprintk("read_status locked=%d status=%d\n", state->signal_info.locked, *status);
	p->strength.len = 1;
	p->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->cnr.len = 1;
	p->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->pre_bit_error.len =1;
	p->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->pre_bit_count.len =1;
	p->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	err = fe_stid135_get_lock_status(state,  &state->signal_info.locked, &state->signal_info.data_present);
	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state->base->i2c->dev, "fe_stid135_get_lock_status error\n");
		mutex_unlock(&state->base->status_lock);
		dprintk("read status=%d\n", *status);
		return -EIO;
	}

	if(state->signal_info.tuned)
		*status |= FE_HAS_SIGNAL;
	if(state->signal_info.timedout)
		*status |= FE_TIMEDOUT;
	if(state->signal_info.locked)
		*status |= FE_HAS_CARRIER | FE_HAS_VITERBI;
	if(state->signal_info.locked)
		*status |= FE_HAS_CARRIER | FE_HAS_VITERBI;
	if(state->signal_info.data_present)
		*status |= FE_HAS_SYNC | FE_HAS_LOCK;

	if (!state->signal_info.locked) {
		/* demod not locked */
		*status |= FE_HAS_SIGNAL;
		dprintk("HAS_SIGNAL AND TUNED locked=%d status=%d\n", state->signal_info.locked, *status);
		err = fe_stid135_get_band_power_demod_not_locked(state,  &state->signal_info.power);
		// if unlocked, set to lowest resource..
		mutex_unlock(&state->base->status_lock);
		if (err != FE_LLA_NO_ERROR) {
			dev_err(&state->base->i2c->dev, "fe_stid135_get_band_power_demod_not_locked error\n");
			dprintk("read status=%d\n", *status);
			return -EIO;
		}

		p->strength.len = 2;
		p->strength.stat[0].scale = FE_SCALE_DECIBEL;
		p->strength.stat[0].svalue = state->signal_info.power;

		p->strength.stat[1].scale = FE_SCALE_RELATIVE;
		p->strength.stat[1].uvalue = (100 + state->signal_info.power/1000) * 656;
		return 0;
	} else {
		/* demod has lock */

		err = fe_stid135_get_signal_quality(state, &state->signal_info, mc_auto);
		mutex_unlock(&state->base->status_lock);
		if (err != FE_LLA_NO_ERROR) {
			dev_err(&state->base->i2c->dev, "fe_stid135_get_signal_quality error\n");
			dprintk("read status=%d\n", *status);
			return -EIO;
		}

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

			if (err != FE_LLA_NO_ERROR)
				dev_warn(&state->base->i2c->dev, "%s: fe_stid135_filter_forbidden_modcodes error %d !\n", __func__, err);
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
		}
		else {
			state->bit_rate = speed;
			state->loops--;
		}
	}


	return 0;
}

//extern void dttst(STCHIP_Handle_t *hChipHandle);

static int stid135_tune(struct dvb_frontend *fe, bool re_tune,
		unsigned int mode_flags,
		unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	int r;

	if (re_tune) {
		r = stid135_set_parameters(fe);
		//dprintk("stid135_set_parameters returned %d locked=%d\n", r, state->signal_info.locked);
		if (r)
			return r;
		state->tune_time = jiffies;
#if 0
		{
			fe_lla_error_t error;
			struct fe_stid135_internal_param *p_params = &state->base->ip;
			error=fe_stid135_set_vtm(p_params, state->nr+1,
															 state->signal_info.frequency, state->signal_info.symbol_rate,
															 state->signal_info.roll_off);
			dprintk("VTM: error=%d\n", error);
		}
#endif

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

	mutex_lock(&state->base->status_lock);
	err = fe_stid135_set_22khz_cont(&state->base->ip,state->rf_in + 1, tone == SEC_TONE_ON);
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
	err |= fe_stid135_diseqc_init(&state->base->ip, state->rf_in + 1, FE_SAT_DISEQC_2_3_PWM);
	err |= fe_stid135_diseqc_send(&state->base->ip, state->rf_in + 1, cmd->msg, cmd->msg_len);
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
	err = fe_stid135_diseqc_receive(&state->base->ip, reply->msg, &reply->msg_len);
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
	struct fe_stid135_internal_param *p_params = &state->base->ip;
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



static int stid135_get_spectrum_scan_fft_one_band(struct stv *state,
																									struct dtv_fe_spectrum* s,
																									s32 center_freq, u32 range,
																									u32* freq,
																									s32* rf_level,
																									s32* rf_band,
																									int fft_size, int mode, s32 pbandx1000, bool double_correction)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	s32 Pbandx1000;
	u32 nb_acquisition = 255; //average 1 samples
	s32 lo_frequency;
	u32 begin =0;
	int i;
	int step = range/1000;
	int a=3;
	s32 delta;
	dprintk("Start of spectrum scan\n");
	error = FE_STiD135_GetLoFreqHz(&state->base->ip, &lo_frequency);
	if(error) {
		dprintk("FE_STiD135_GetLoFreqHz FAILED: error=%d\n", error);
		return error;
	}
	lo_frequency *=  1000000; //now in Hz


	dprintk("center_freq=%dkHz lo=%dkHz range=%dkHz mode=%d fft_size=%d\n",
					center_freq, lo_frequency/1000, range/1000, mode, fft_size);

	error = fe_stid135_fft(state, state->nr+1, mode, nb_acquisition,
												 center_freq*1000 - lo_frequency, range, rf_level, &begin, MAX_FFT_SIZE);

	pbandx1000 += mode*6020; //compensate for FFT number of bins: 20*log10(2)
	delta = 10*(STLog10(range) - STLog10(fft_size)); //this is a power spectral density range/fft_size is the bin width in Hz
	//dprintk("Pbandx1000=%d error=%d\n", Pbandx1000, error);
	if(fft_size> MAX_FFT_SIZE)
		dprintk("BUG!!!! fft_size=%d\n", fft_size);

	for(i=0; i< fft_size; ++i) {
		s32 f = ((i-(signed)fft_size/2)*step)/(signed)fft_size +center_freq; //in kHz
		s32 correction1000 = (s32)(f/1000  - 1550);

		freq[i]= f;

		correction1000 = (s32)(correction1000*1000/600);
		correction1000 = (s32) (correction1000 * correction1000/1000);
		if(double_correction)
			correction1000 += correction1000;

		rf_level[i] += pbandx1000 - delta +correction1000;
		rf_band[i] = pbandx1000 +correction1000;
	}

	if(fft_size/2-a<0 || fft_size/2+a>= MAX_FFT_SIZE)
		dprintk("BUG!!!! a=%d fft_size=%d\n", a, fft_size);
	if(begin==1)
		rf_level[0] = rf_level[1];
	for(i=-a+1; i<a; ++i)
		rf_level[fft_size/2+i] = (rf_level[fft_size/2-a] + rf_level[fft_size/2+a])/2;
	return error;
}

static int stid135_get_spectrum_scan_fft(struct dvb_frontend *fe,
																				 struct dtv_fe_spectrum* s,
																				 unsigned int *delay,  enum fe_status *status
																				 )
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct stv *state = fe->demodulator_priv;
	s32 Reg[60];
	//	s32 center_freq; //in kHz
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;
	u32 idx=0;
	int i;
	s32 discont, left=0, right=0;
	u32* temp_freq = NULL;
	s32* temp_rf_level = NULL;
	s32* temp_rf_band = NULL;
	u32 table_size = 8192;
	s32 fft_size =  (p->scan_fft_size>0) ? p->scan_fft_size : 256;
	s32 max_range= 96800;
	s32 start_frequency = p->scan_start_frequency;
	s32 end_frequency = p->scan_end_frequency;
	//uint32_t resolution =  p->scan_resolution>0 ? p->scan_resolution : 100; //in kHz
	/*resolution is used by the ioctl call which sets the parameter; it cannot be changed
		here without affecting the buffer size s->num_freq, which determines how much memory
		we can fill.
	*/
	s32 sample_step = (end_frequency - start_frequency +s->num_freq-1)/s->num_freq;
	u32 range = sample_step * fft_size; //in kHz
	s32 useable_samples2 = ((fft_size*6)/10+1)/2;
	s32 lost_samples2 = fft_size/2 - useable_samples2;
	int subsample_factor =1;
	int mode;
	s32 start_idx;
	s32 pbandx1000;
	bool double_correction;
	for(mode=1;mode<=5; ++mode) {
		table_size >>=1;
		if(table_size <= fft_size)
			break;
	}
	if(fft_size != table_size)
		dprintk("Error: fft_size should be power of 2\n");
	fft_size = table_size;

	if(range >= max_range) {
		subsample_factor = (range+ max_range-1)/max_range;
		dprintk("specified resolution (%dkHz) must be lower for this fft size (%d) Will subsample by %d\n",
						p->scan_resolution/1000, fft_size, subsample_factor);
		//error= -EINVAL;
		//goto _end;
	}

	dprintk("demod: %d: tuner:%d range=[%d,%d]kHz resolution=%dkHz num_freq=%d range=%dkHz lost=%d useable=%d\n",
					state->nr, state->rf_in,
					start_frequency, end_frequency, sample_step,
					s->num_freq, range, lost_samples2, useable_samples2);
	s->scale = FE_SCALE_RELATIVE;

	temp_freq = kzalloc(MAX_FFT_SIZE * (sizeof(temp_freq[0])), GFP_KERNEL);
	dprintk("BUFFER1 freq=%p-%p\n", temp_freq , temp_freq+8192 * (sizeof(temp_freq[0])));

	temp_rf_level = kzalloc(8192 * (sizeof(temp_rf_level[0])), GFP_KERNEL);
	dprintk("BUFFER2 rf_level-%p-%p\n", temp_rf_level, temp_rf_level+8192 * (sizeof(temp_rf_level[0])));
	if (!temp_rf_level || !temp_freq) {
		error = -ENOMEM;
		goto _end;
	}

	temp_rf_band = kzalloc(8192 * (sizeof(temp_rf_band[0])), GFP_KERNEL);
	dprintk("BUFFER3 rf_band-%p-%p\n", temp_rf_band, temp_rf_band+8192 * (sizeof(temp_rf_band[0])));
	if (!temp_rf_band ) {
		error = -ENOMEM;
		goto _end;
	}


	error = fe_stid135_init_fft(&state->base->ip, state->nr+1, state->rf_in+1, mode, Reg);

	error |= estimate_band_power_demod_for_fft(state, state->rf_in+1, &pbandx1000, &double_correction);

	if(error) {
		dprintk("fe_stid135_init_fft FAILED: error=%d\n", error);
		return error;
	}

	for(idx=0; idx < s->num_freq;) {
		s32 center_freq = start_frequency + (idx + useable_samples2)* sample_step;
		if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
			//@todo:  should this be moved into  stid135_get_spectrum_scan_fft_one_band?
			dprintk("exiting on should stop\n");
			break;
		}

		error = stid135_get_spectrum_scan_fft_one_band(state, s, center_freq, range*1000,
																									 temp_freq, temp_rf_level, temp_rf_band,
																									 fft_size, mode, pbandx1000, double_correction);
		if(error) {
			dprintk("Error=%d\n", error);
			goto _end;
		}
#if 1
		i=lost_samples2;
		left = temp_rf_level[i];
		discont = (left-right);

		//implement triangular weight for window overlap
		if(idx>= useable_samples2)
			for(i=1; i< useable_samples2; i+=subsample_factor ) {
				s32 correction = ((discont* (useable_samples2+1-i))/(2*useable_samples2));
				s->rf_level[idx-i]+= correction;
				//if(s->rf_band)
				//	s->rf_band[idx]+= correction;
			}
#endif
		for(i= lost_samples2; i<fft_size-lost_samples2 && idx<s->num_freq; ++idx, i+=subsample_factor ) {
			s32 correction = (i<lost_samples2+useable_samples2 &&right!=0) ?-(discont*(useable_samples2-i+lost_samples2))/(2*useable_samples2) :0;
			s->freq[idx]= temp_freq[i];
			s->rf_level[idx] += temp_rf_level[i]+correction;
			if(s->rf_band)
				s->rf_band[idx]= temp_rf_band[i];
		}
		i=fft_size-lost_samples2;
		right = temp_rf_level[i-1];

	}

 _end:
	dprintk("ending\n");
	if(temp_freq)
		kfree(temp_freq);
	if(temp_rf_level)
		kfree(temp_rf_level);
	if(temp_rf_band)
		kfree(temp_rf_band);
	error |= (error1=fe_stid135_term_fft(&state->base->ip, state->nr+1, state->rf_in+1, Reg));
	if(error) {
		dprintk("fe_stid135_term_fft FAILED: error=%d\n", error);
	}
	*status =  FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	return 0;
	dprintk("encountered error\n");
	*status =  FE_TIMEDOUT|FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	return error;
}




static int stid135_get_spectrum_scan_regular(struct dvb_frontend *fe,
																						 struct dtv_fe_spectrum* s,
																						 unsigned int *delay,  enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct fe_stid135_internal_param * pParams = (struct fe_stid135_internal_param *) &state->base->ip;
	s32 lo_frequency;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;
	s32 pband_rf;
	s32 pch_rf;
	int i = 0;
	u32 start_frequency = p->scan_start_frequency;
	u32 end_frequency = p->scan_end_frequency;
	//u32 bandwidth = end_frequency-start_frequency; //in kHz
	u32 num_freq =	s->num_freq;
	uint32_t frequency;
	uint32_t resolution =  (p->scan_resolution>0) ? p->scan_resolution : p->symbol_rate/1000; //in kHz
	uint32_t bandwidth =  (p->symbol_rate>0) ? p->symbol_rate : p->scan_resolution*1000; //in Hz
#ifdef TODO
	state->tuner_bw = stv091x_bandwidth(ROLLOFF_AUTO, bandwidth);
#endif
	s->scale =  FE_SCALE_DECIBEL; //in units of 0.001dB
	dprintk("demod: %d: tuner:%d range=[%d,%d]kHz num_freq=%d resolution=%dkHz bw=%dkHz clock=%d\n", state->nr,
					state->rf_in,
					start_frequency, end_frequency,
					num_freq, resolution, bandwidth/1000,
					pParams->master_clock);

	error |= (error1=FE_STiD135_GetLoFreqHz(pParams, &lo_frequency));
	if(error1) {
		dprintk("Failed: err=%d\n", error1);
		goto __onerror;
	}
	lo_frequency *=  1000000; //now in Hz



	//warm start
	error |= (error1=ChipSetOneRegister(pParams->handle_demod,
																			(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(state->nr+1), 0x18));

	if(error1) {
		dprintk("Failed: err=%d\n", error1);
		goto __onerror;
	}

	//stop demod
	error |= (error1=ChipSetOneRegister(pParams->handle_demod,
																			(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(state->nr+1), 0x5C));
	if(error1) {
		dprintk("Failed: err=%d\n", error1);
		goto __onerror;
	}

	error |=(error1 = fe_stid135_set_symbol_rate(state, resolution*1000));
	if(error1) {
		dprintk("Failed: err=%d\n", error1);
		goto __onerror;
	}
	//stop demod
	error |= (error1=ChipSetOneRegister(pParams->handle_demod,
																			(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(state->nr+1), 0x5C));
	if(error1) {
		dprintk("Failed: err=%d\n", error1);
		goto __onerror;
	}
	if(error1) {
		dprintk("Failed: err=%d\n", error1);
		goto __onerror;
	}

	/*Set CFRUPLOW_USEMODE=0 => CFRUP/LOW are not used ; citroen 2 phase detection for QPKS; carrier 1 derotor on
		(@todo: is the rotator needed?)*/
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(state->nr+1), 0x06);


	for (i = 0; i < num_freq; i++) {
		if ((i% 20==19) &&  (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			dprintk("exiting on should stop\n");
			break;
		}
		//stop demod
		error |= (error1=ChipSetOneRegister(pParams->handle_demod,
																				(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(state->nr+1), 0x1C));
		if(error1) {
			dprintk("Failed: err=%d\n", error1);
			goto __onerror;
		}

		//warm start @todo: needed?
		error |= (error1=ChipSetOneRegister(pParams->handle_demod,
																				(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(state->nr+1), 0x18));
		if(error1) {
			dprintk("Failed: err=%d\n", error1);
			goto __onerror;
		}
		s->freq[i]= start_frequency +i*resolution;
		frequency = s->freq[i];

		//Set frequency with minimal register changes
		{
			s32 frequency_hz =  (s32)frequency*1000 -  lo_frequency;
			s32 si_register;
			int demod = state->nr +1;
			const u16 cfr_factor = 6711; // 6711 = 2^20/(10^6/2^6)*100

			/* Search range definition */
			si_register = ((frequency_hz/PLL_FVCO_FREQUENCY)*cfr_factor)/100;
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT2_CFR_INIT(demod),
																 ((u8)(si_register >> 16)));
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT1_CFR_INIT(demod),
																 ((u8)(si_register >> 8)));
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT0_CFR_INIT(demod),
																 ((u8)(si_register)));
			error |= ChipSetRegisters(pParams->handle_demod,REG_RC8CODEW_DVBSX_DEMOD_CFRINIT2(demod),3);
			if(error)
				goto __onerror;
			//msleep(10);

		}


		error |= (error1=FE_STiD135_GetRFLevel(state, &pch_rf, &pband_rf));
		s->rf_level[i] = pch_rf;
		if(error1) {
			dprintk("Failed: err=%d\n", error1);
			goto __onerror;
		}
		if(error)
			goto __onerror;

		usleep_range(12000, 13000);

	}
	*status =  FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	return 0;
 __onerror:
	dprintk("encountered error at %d/%d\n", i, num_freq);
	*status =  FE_TIMEDOUT|FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	return 0;
}


static int stid135_get_spectrum_scan(struct dvb_frontend *fe,
																		 struct dtv_fe_spectrum* s,
																		 unsigned int *delay,  enum fe_status *status)
{
	switch(s->spectrum_method) {
	case SPECTRUM_METHOD_SWEEP:
	default:
		return stid135_get_spectrum_scan_regular(fe, s, delay,  status);
		break;
	case SPECTRUM_METHOD_FFT:
		return stid135_get_spectrum_scan_fft(fe, s, delay,  status);
		break;
	}
	return -1;
}

#if 0
/*
	init = 1: start the scan at the search range specified by the user
	init = 0: start the scan just beyond the last found frequency
 */
static int stid135_sat_scan(struct dvb_frontend *fe, bool init,
														unsigned int *delay,  enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct fe_stid135_internal_param * pParams = &state->base->ip;
	int asperity;
	bool locked = false;
	//state->Started = 0;
	//state->satellite_scan = true;
	//todo: 	MinSymbolRate
	state->search_range =  500000;
	state->symbol_rate =  1000000;							/*	minimum symbol rate for scan = 1Msps	*/
	dprintk("set symbol_rate=%d\n", state->symbol_rate);

	if(init) {
		state->scan_next_frequency = p->scan_start_frequency;
		state->scan_end_frequency = p->scan_end_frequency;
	}

	while(state->scan_next_frequency < state->scan_end_frequency) {
		*status = 0;
		if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
			dprintk("exiting on should stop\n");
			*status =0;
			return 0;
		}
		p->frequency = state->scan_next_frequency;
		dprintk("start=%d end=%d init=%d\n",
						state->scan_next_frequency, state->scan_end_frequency,  init);

		if(p->frequency >= state->scan_end_frequency) {
			dprintk("Invalid ranged");
			return -1;
		}

		p->algorithm = ALGORITHM_SEARCH_NEXT;

		//TODO: satellitte_scan
		asperity = scan_within_tuner_bw(fe, &locked);
		dprintk("asperity=%d locked=%d freq=%d start=%d end=%d\n", asperity, locked,
						p->frequency, state->scan_next_frequency, state->scan_end_frequency);
		if(locked) {
			//state->scan_end_frequency = p->frequency;
			//stv091x_set_frequency_symbol_rate_bandwidth(state);
			uint32_t step = 135*((state->symbol_rate/1000) / (2*100));
			uint32_t next = p->frequency + step;
			dprintk("symbol_rate=%d step=%d next-%d\n", state->symbol_rate, step, next);
			if(next < state->scan_next_frequency) {
				next += state->search_range/1000;
				dprintk ("Adjusting next to prevent going backwards: new next=%d\n", next);
			}
			state->tune_time = jiffies;

			state->scan_next_frequency = next;

			*status = FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
			*delay = HZ;

			return 0;
		} else {
			*status = 0;
		}
	}
	if(state->scan_next_frequency >= state->scan_end_frequency) {
		dprintk("Signaling DONE\n");
		*status |= FE_TIMEDOUT;
	}
	return 0;
}
#endif

static struct dvb_frontend_ops stid135_ops = {
	.delsys = { SYS_DVBS, SYS_DVBS2, SYS_DSS },
	.info = {
		.name			= "STiD135 Multistandard",
		.frequency_min_hz	 = 950 * MHz,
		.frequency_max_hz		= 2150 * MHz,
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
	//	.scan =  stid135_sat_scan,
	.get_spectrum		= stid135_get_spectrum_scan,
	.extended_info = {
		.extended_caps          = FE_CAN_SPECTRUMSCAN	|
		FE_CAN_IQ
	}
};

static struct stv_base *match_base(struct i2c_adapter  *i2c, u8 adr)
{
	struct stv_base *p;

	list_for_each_entry(p, &stvlist, stvlist)
		if (p->i2c == i2c && p->adr == adr)
			return p;
	return NULL;
}

/*DT: called with  nr=adapter=0...7 and rf_in = nr/2=0...3
	state->base is created exactly once and is shared
	between the 8 demods; provides access to the i2c hardware and such
*/
struct dvb_frontend *stid135_attach(struct i2c_adapter *i2c,
						struct stid135_cfg *cfg,
						int nr, int rf_in)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	int i;
	struct stv *state;
	struct stv_base *base=NULL;

	//struct fe_stid135_internal_param * pParams;
	state = kzalloc(sizeof(struct stv), GFP_KERNEL);
	if (!state)
		return NULL;


	dprintk("ATTACH addr=0x%x\n", cfg->adr);
	base = match_base(i2c, cfg->adr);
	dprintk("ATTACH here\n");


	if (base) {
		base->count++;
		state->base = base;
		dprintk("ATTACH DUP nr=%d rf_in=%d base=%p count=%d\n", nr, rf_in, base, base->count);
	} else {
		base = kzalloc(sizeof(struct stv_base), GFP_KERNEL);
		if (!base)
			goto fail;
		dprintk("ATTACH NEW nr=%d rf_in=%d base=%p count=%d\n", nr, rf_in, base, base->count);
		base->i2c = i2c;
		base->adr = cfg->adr;
		base->count = 1;
		base->extclk = cfg->clk;
		base->ts_mode = cfg->ts_mode;
		base->set_voltage = cfg->set_voltage;
		base->mode = cfg->set_voltage ? mode : 1;
		base->write_properties = cfg->write_properties;
		base->read_properties = cfg->read_properties;
		base->set_TSsampling = cfg->set_TSsampling;
		base->set_TSparam  = cfg->set_TSparam;
		base->vglna		=	cfg->vglna;    //for stvvglna 6909x v2 6903x v2

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
	dprintk("ATTACH here\n");

	//pParams = &state->base->ip;
	/* Init for PID filtering feature */

	state->pid_flt.first_disable_all_command = TRUE;
	/* Init for GSE filtering feature */

	state->gse_flt.first_disable_all_protocol_command = TRUE;
	state->gse_flt.first_disable_all_label_command = TRUE;

	/* Init for MODCOD filtering feature */
#if 0
	//does not belong here: global for chip
	fe_stid135_modcod_flt_reg_init();
#endif
	for(i=0;i<NB_SAT_MODCOD;i++) {
		state->mc_flt[i].forbidden = FALSE;
	}
	error = fe_stid135_apply_custom_qef_for_modcod_filter(state, NULL);
	if(error)
		dprintk("fe_stid135_apply_custom_qef_for_modcod_filter error=%d\n", error);

	state->fe.ops               = stid135_ops;
	state->fe.demodulator_priv  = state;
	state->nr = nr;
	state->newTP = false;
	state->bit_rate  = 0;
	state->loops = 15;
	state->current_max_llr_rate = 0;
	state->current_llr_rate = 0;

	error |= fe_stid135_set_maxllr_rate(state, 90);
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
