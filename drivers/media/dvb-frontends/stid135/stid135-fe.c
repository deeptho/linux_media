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
#include "stid135-fft.h"

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
	void (*write_properties) (struct i2c_adapter *i2c,u8 reg, u32 buf);
	void (*read_properties) (struct i2c_adapter *i2c,u8 reg, u32 *buf);

	//for tbs6912
	void (*set_TSsampling)(struct i2c_adapter *i2c,int tuner,int time);  
	u32  (*set_TSparam)(struct i2c_adapter *i2c,int tuner,int time,bool flag);
	//end

	//for stvvglna
	bool vglna;
};

/*for debugging only: assumes only a single card is in use; otherwise wrong debug output 
	will be printed, but apart from that behaviour will still be correct
*/
static atomic_t llr_rate_sum;

struct stv {
	struct stv_base     *base;
	struct dvb_frontend  fe;
	int                  nr;     //DT: adapter aka demod: 0-7
	int                  rf_in;  //DT  tuner frontend: 0-3
	unsigned long        tune_time;
	int current_llr_rate;  //Remember the current reconfiguration to avoid calling hardware needlessly
	int current_max_llr_rate;  //Remember the current reconfiguration to avoid calling hardware needlessly
	struct fe_sat_signal_info signal_info;

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
	char *VglnaIdString = NULL;
	STCHIP_Info_t VGLNAChip;
	STCHIP_Handle_t vglna_handle;
	STCHIP_Handle_t vglna_handle1;
	STCHIP_Handle_t vglna_handle2;
	STCHIP_Handle_t vglna_handle3;
	SAT_VGLNA_InitParams_t pVGLNAInit;
	SAT_VGLNA_InitParams_t pVGLNAInit1;
	SAT_VGLNA_InitParams_t pVGLNAInit2;
	SAT_VGLNA_InitParams_t pVGLNAInit3;

	dev_warn(&state->base->i2c->dev, "%s\n", FE_STiD135_GetRevision());

	strcpy(init_params.demod_name,"STiD135");
	init_params.pI2CHost		=	state->base;
	init_params.demod_i2c_adr   	=	state->base->adr ? state->base->adr<<1 : 0xd0;
	init_params.demod_ref_clk  	= 	state->base->extclk ? state->base->extclk : 27;
	init_params.internal_dcdc	=	0;
	init_params.internal_ldo	=	1; // LDO supply is internal on Oxford valid board
#if 1 //main driver has value 0xf
	init_params.rf_input_type	=	0x0; // Single ended RF input on Oxford valid board rev2
#endif
	init_params.roll_off		=  	FE_SAT_35; // NYQUIST Filter value (used for DVBS1/DSS, DVBS2 is automatic)
	init_params.tuner_iq_inversion	=	FE_SAT_IQ_NORMAL;
	
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
#if 1 
		for(i=0;i<8;i++) {
			err |= fe_stid135_set_ts_parallel_serial(state->base->handle, i+1, FE_TS_PARALLEL_ON_TSOUT_0);
		}
#else // this is code from mai n driver
		//	for(i=0;i<8;i++) {
		err |= fe_stid135_set_ts_parallel_serial(state->base->handle, FE_SAT_DEMOD_1, FE_TS_PARALLEL_ON_TSOUT_0);
		//	err |= fe_stid135_set_maxllr_rate(state->base->handle, i+1, 260);
		//	}
#endif
		err |= fe_stid135_enable_stfe(state->base->handle,FE_STFE_OUTPUT0);
		err |= fe_stid135_set_stfe(state->base->handle, FE_STFE_TAGGING_MERGING_MODE, FE_STFE_INPUT1 |
						FE_STFE_INPUT2 |FE_STFE_INPUT3 |FE_STFE_INPUT4| FE_STFE_INPUT5 |
						FE_STFE_INPUT6 |FE_STFE_INPUT7 |FE_STFE_INPUT8 ,FE_STFE_OUTPUT0, 0xDE);
	} else if (state->base->ts_mode == TS_8SER) { //DT: This code is not called
		dev_warn(&state->base->i2c->dev, "%s: 8xTS serial mode init.\n", __func__);
		for(i=0;i<8;i++) {
			err |= fe_stid135_set_ts_parallel_serial(state->base->handle, i+1, FE_TS_SERIAL_CONT_CLOCK);
			err |= fe_stid135_set_maxllr_rate(state->base->handle, i+1, 90);
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
		err |= fe_stid135_diseqc_init(state->base->handle,AFE_TUNER2, FE_SAT_22KHZ_Continues);
		err |= fe_stid135_diseqc_init(state->base->handle,AFE_TUNER3, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(state->base->handle,AFE_TUNER4, FE_SAT_22KHZ_Continues);
		if (state->base->set_voltage) {		  
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_13, 0);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_13, 1);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_18, 2);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_18, 3);
		}
	}

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: setup error %d !\n", __func__, err);


/*///////////////////init stvvglna/////////////////////////////////////*/
	if(state->base->vglna){
	printk("__Initialized STVVGLNA__\n");
	VglnaIdString = "STVVGLNA";
		/* Init the VGLNA */
	pVGLNAInit.Chip = &VGLNAChip;
	
	pVGLNAInit.Chip->pI2CHost	  =	state->base;
	pVGLNAInit.Chip->RepeaterHost = NULL;
	pVGLNAInit.Chip->Repeater     = FALSE;
	pVGLNAInit.Chip->I2cAddr      = 0xc8;
	pVGLNAInit.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit, &vglna_handle);
	printk("Initialized STVVGLNA  0 device\n");	
	stvvglna_set_standby(vglna_handle,1);

	
	VglnaIdString = "STVVGLNA1";
	pVGLNAInit1.Chip = &VGLNAChip;
	
	pVGLNAInit1.Chip->pI2CHost	  =	state->base;
	pVGLNAInit1.Chip->RepeaterHost = NULL;
	pVGLNAInit1.Chip->Repeater     = FALSE;
	pVGLNAInit1.Chip->I2cAddr      = 0xce;
	pVGLNAInit1.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit1.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit1, &vglna_handle1);
	printk("Initialized STVVGLNA  1 device\n");	
	stvvglna_set_standby(vglna_handle1,1);
	
	VglnaIdString = "STVVGLNA2";
	pVGLNAInit2.Chip = &VGLNAChip;
	
	pVGLNAInit2.Chip->pI2CHost	  =	state->base;
	pVGLNAInit2.Chip->RepeaterHost = NULL;
	pVGLNAInit2.Chip->Repeater     = FALSE;
	pVGLNAInit2.Chip->I2cAddr      = 0xcc;
	pVGLNAInit2.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit2.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit2, &vglna_handle2);
	printk("Initialized STVVGLNA  2 device\n");	
	stvvglna_set_standby(vglna_handle2,1);
	
	VglnaIdString = "STVVGLNA3";
	pVGLNAInit3.Chip = &VGLNAChip;
	pVGLNAInit3.Chip->pI2CHost	  =	state->base;
	pVGLNAInit3.Chip->RepeaterHost = NULL;
	pVGLNAInit3.Chip->Repeater     = FALSE;
	pVGLNAInit3.Chip->I2cAddr      = 0xca;
	pVGLNAInit3.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit3.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit3, &vglna_handle3);
	printk("Initialized STVVGLNA  3 device\n");	
	stvvglna_set_standby(vglna_handle3,1);
	}
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
	dev_dbg(&state->base->i2c->dev, "%s: demod %d\n", __func__, state->nr);

	state->base->count--;
	if (state->base->count == 0) {
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
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params = state->base->handle;
	struct fe_sat_search_params search_params;
	struct fe_sat_search_result search_results;
	u32 pls_mode, pls_code;
	s32 rf_power;
	BOOL satellitte_scan =0;
	//BOOL lock_stat=0;
	struct fe_sat_signal_info* signal_info = &state->signal_info;

	
	dev_warn(&state->base->i2c->dev,
			"delivery_system=%u modulation=%u frequency=%u symbol_rate=%u inversion=%u stream_id=%d\n",
			p->delivery_system, p->modulation, p->frequency,
					 p->symbol_rate, p->inversion, p->stream_id);
	release_maxllr_rate(__LINE__, state);
	mutex_lock(&state->base->status_lock);

	/* Search parameters */
#if 1
	switch(p->algorithm) {
	case ALGORITHM_WARM: 
		search_params.search_algo 	= FE_SAT_WARM_START;
		satellitte_scan = 0;
		break;
		
	case ALGORITHM_COLD: 
	case ALGORITHM_COLD_BEST_GUESS: 
		search_params.search_algo 	= FE_SAT_COLD_START;
		satellitte_scan = 0;
		break;
		
	case ALGORITHM_BLIND: 
	case ALGORITHM_BLIND_BEST_GUESS:
	case ALGORITHM_BANDWIDTH:
		search_params.search_algo 	= FE_SAT_BLIND_SEARCH;
		search_params.standard = FE_SAT_AUTO_SEARCH;
		search_params.puncture_rate = FE_SAT_PR_UNKNOWN;
		satellitte_scan = 0;
		break;
	case ALGORITHM_NEXT: 
		search_params.search_algo 	= FE_SAT_NEXT;
		search_params.standard = FE_SAT_AUTO_SEARCH;
		search_params.puncture_rate = FE_SAT_PR_UNKNOWN;
		satellitte_scan = 0;
		break;
	}
	
#else
	search_params.search_algo 	= FE_SAT_WARM_START;
#endif
	search_params.frequency 	=  p->frequency*1000;
	search_params.symbol_rate 	= 	(search_params.search_algo == FE_SAT_BLIND_SEARCH ||
																	 search_params.search_algo == FE_SAT_NEXT
																	 ) ?    30000000/*to prevent error detected*/ : p->symbol_rate;
	search_params.modulation	= FE_SAT_MOD_UNKNOWN;
	search_params.modcode		= FE_SAT_DUMMY_PLF;
	search_params.search_range	= p->search_range > 0 ? p->search_range : 10000000;
	//dprintk("Search range set to %d\n", search_params.search_range);

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

	err = FE_STiD135_GetLoFreqHz(state->base->handle, &(search_params.lo_frequency));
	//dprintk("lo_frequency = %d\n", search_params.lo_frequency);
	search_params.lo_frequency *= 1000000;
	if(search_params.search_algo == FE_SAT_BLIND_SEARCH ||
		 search_params.search_algo == FE_SAT_NEXT) {
		//search_params.frequency = 	950000000 ;
		printk("BLIND: set freq=%d lo=%d\n", 	search_params.frequency,	search_params.lo_frequency  );
	}
	
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
	signal_info-> pls_code =pls_code;
	signal_info-> pls_mode =pls_mode;
	
	if (p->scrambling_sequence_index) {
		pls_mode = 1;
		pls_code = p->scrambling_sequence_index;
#if 1 //Deep Thought: code must be moved outside of if test or multi-stream does not work
		/* Set PLS before search */
		dev_dbg(&state->base->i2c->dev, "%s: set pls_mode %d, pls_code %d !\n", __func__, pls_mode, pls_code);
		err |= fe_stid135_set_pls(state->base->handle, state->nr + 1, pls_mode, pls_code);
#endif
	}
#if 1 //Deep Thought: code must be moved outside of if test or multi-stream does not work
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
	err |= (error1=fe_stid135_search(state->base->handle, state->nr + 1, &search_params, &search_results, satellitte_scan));
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
		err |= fe_stid135_set_maxllr_rate(state->base->handle, state->nr +1, 180);
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
		err |= fe_stid135_get_band_power_demod_not_locked(state->base->handle, state->nr + 1, &rf_power);
		dev_dbg(&state->base->i2c->dev, "%s: not locked, band rf_power %d dBm ! demod=%d tuner=%d\n",
						 __func__, rf_power / 1000, state->nr, state->rf_in);
	}
	dprintk("set_parameters: error=%d locked=%d\n", err, state->signal_info.locked);
	/* Set ISI before search */
	if (p->stream_id != NO_STREAM_ID_FILTER) {
		dev_warn(&state->base->i2c->dev, "%s: set ISI %d ! demod=%d tuner=%d\n", __func__, p->stream_id & 0xFF,
						 state->nr, state->rf_in);
		err |= fe_stid135_set_mis_filtering(state->base->handle, state->nr + 1, TRUE, p->stream_id & 0xFF, 0xFF);
		signal_info->isi = p->stream_id &0xff;
	} else {
		dev_dbg(&state->base->i2c->dev, "%s: disable ISI filtering !\n", __func__);
		err |= fe_stid135_set_mis_filtering(state->base->handle, state->nr + 1, FALSE, 0, 0xFF);
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
	struct fe_stid135_internal_param *p_params = state->base->handle;
	if (!state->signal_info.locked)
		return 0;
	if(p_params->demod_search_algo[state->nr] == FE_SAT_BLIND_SEARCH ||
		 p_params->demod_search_algo[state->nr] == FE_SAT_NEXT) {
		int max_isi_len= sizeof(p-> isi)/sizeof(p->isi[0]);
		fe_stid135_get_signal_info(p_params, state->nr + 1, &state->signal_info, 0);
		dprintk("MIS2: num=%d\n", state->signal_info.isi_list.nb_isi);
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
	printk("XXXXX stream_id=%d\n",p->stream_id);
	p->pls_mode = state->signal_info.pls_mode;
	p->pls_code = state->signal_info.pls_code;
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

	err = fe_stid135_get_lock_status(state->base->handle, state->nr + 1, &state->signal_info.locked, &state->signal_info.data_present);
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
	if(state->signal_info.data_present)
		*status |= FE_HAS_SYNC | FE_HAS_LOCK;

	if (!state->signal_info.locked) {
		/* demod not locked */
		*status |= FE_HAS_SIGNAL;
		dprintk("HAS_SIGNAL AND TUNED locked=%d status=%d\n", state->signal_info.locked, *status);
		err = fe_stid135_get_band_power_demod_not_locked(state->base->handle, state->nr + 1, &state->signal_info.power);
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
		
		err = fe_stid135_get_signal_quality(state->base->handle, state->nr + 1, &state->signal_info, mc_auto);
		mutex_unlock(&state->base->status_lock);
		if (err != FE_LLA_NO_ERROR) {
			dev_err(&state->base->i2c->dev, "fe_stid135_get_signal_quality error\n");
			dprintk("read status=%d\n", *status);
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

	}
	//dprintk("read status=%d\n", *status);

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


static int stid135_get_spectrum_scan(struct dvb_frontend *fe, struct dvb_fe_spectrum_scan *s)
{
	s32 Reg[60];
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct stv *state = fe->demodulator_priv;

	s32 center_freq=   p->frequency; //in kHz
	u32 range =    p->symbol_rate; //in Hz
	u32 mode = 1; //table of 4096 samples
	//u32 mode = 5; //table of 256 samples
	
	u32 nb_acquisition = 255; //average 1 samples
	u32 table_size = 8192;
	s32 lo_frequency;
	u32* freqs = s->freq;
	s32* rf_level = s->rf_level;
	u32 begin =0;
	int i;
	int step;
	for(mode=1;mode<=5; ++mode) {
		table_size >>=1;
		if(table_size <= s->num_freq)
			break;
	}
	dprintk("Start of spectrum scan\n");
	error = FE_STiD135_GetLoFreqHz(state->base->handle, &lo_frequency);
	if(error) {
		dprintk("FE_STiD135_GetLoFreqHz FAILED: error=%d\n", error);
		return error;
	}
	lo_frequency *=  1000000; //now in Hz
 
		
	error = fe_stid135_init_fft(state->base->handle, state->nr+1, state->rf_in+1, Reg);
	if(error) {
		dprintk("fe_stid135_init_fft FAILED: error=%d\n", error);
		return error;
	}
	dprintk("freq=%d lo=%d range=%d mode=%d table_size=%d\n", center_freq, lo_frequency, range, mode, table_size);
	
	error = fe_stid135_fft(state->base->handle, state->nr+1, mode, nb_acquisition,
												 center_freq*1000 - lo_frequency, range, rf_level, &begin);
	step = range/1000;
	if(begin==1)
		rf_level[0] = rf_level[1];
	for(i=0; i< table_size; ++i) {
		freqs[i]= center_freq  + ((i-(signed)table_size/2)*step)/(signed)table_size;
	}
	rf_level[table_size/2] = (rf_level[table_size/2-1] + rf_level[table_size/2+1])/2;
	error |= fe_stid135_term_fft(state->base->handle, state->nr+1, state->rf_in+1, Reg);
	if(error) {
		dprintk("fe_stid135_term_fft FAILED: error=%d\n", error);
	}
	return error;
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
	.get_spectrum_scan		= stid135_get_spectrum_scan,
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
		base->set_TSsampling = cfg->set_TSsampling;
		base->set_TSparam  = cfg->set_TSparam;
		base->vglna = cfg->vglna;

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
