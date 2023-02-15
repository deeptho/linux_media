/*
 * Driver for the STm STiD135 DVB-S/S2/S2X demodulator.
 *
 * Copyright (C) CrazyCat <crazycat69@narod.ru>
 * Copyright (C) Deep Thought <deeptho@gmail.com> - blindscan, spectrum and constellation scan
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
#define MAX_FFT_SIZE 8192
LIST_HEAD(stvlist);

static int mode;
module_param(mode, int, 0444);
MODULE_PARM_DESC(mode,
		"Multi-switch mode: 0=quattro/quad 1=normal direct connection");

static int vglna_mode=0;
module_param(vglna_mode, int, 0444);
MODULE_PARM_DESC(vglna_mode,
		"vlglna on/off");

bool fft_mode32=0;
module_param(fft_mode32, bool, 0644);
MODULE_PARM_DESC(fft_mode32,
		"0: 16 bit fft mode on; else 32 bit");

static unsigned int rfsource;
module_param(rfsource, int, 0644);
MODULE_PARM_DESC(rfsource, "RF source selection for direct connection mode (default:0 - auto)");

int stid135_verbose=0;
module_param(stid135_verbose, int, 0644);
MODULE_PARM_DESC(stid135_verbose, "verbose debugging");

static unsigned int mc_auto;
module_param(mc_auto, int, 0644);
MODULE_PARM_DESC(mc_auto, "Enable auto modcode filtering depend from current C/N (default:0 - disabled)");

static int blindscan_always=0; //always use blindscan
module_param(blindscan_always, int, 0644);
MODULE_PARM_DESC(blidscan_always, "Always tune using blindscan (default:0 - disabled)");

static unsigned int ts_nosync;
module_param(ts_nosync, int, 0644);
MODULE_PARM_DESC(ts_nosync, "TS FIFO Minimum latence mode (default:off)");

static unsigned int mis;
module_param(mis,int,0644);
MODULE_PARM_DESC(mis,"someone search the multi-stream signal lose packets,please turn on it(default:off)");

#define vprintk(fmt, arg...)																					\
	if(stid135_verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)



static inline enum fe_rolloff dvb_rolloff(struct stv*state) {
	switch (state->signal_info.roll_off) {
	case FE_SAT_05:
		return ROLLOFF_5;
		break;
	case FE_SAT_10:
		return ROLLOFF_10;
		break;
	case FE_SAT_15:
		return ROLLOFF_15;
		break;
	case FE_SAT_20:
		return ROLLOFF_20;
		break;
	case FE_SAT_25:
		return ROLLOFF_25;
		break;
	case FE_SAT_35:
		return ROLLOFF_35;
		break;
	case FE_SAT_LOW:
		return ROLLOFF_LOW;
	default:
		break;
	}
	return ROLLOFF_AUTO;
}

static inline enum fe_modulation dvb_modulation(struct stv*state) {
	switch (state->signal_info.modulation) {
	case FE_SAT_MOD_8PSK:
		return PSK_8;
		break;
	case FE_SAT_MOD_16APSK:
		return APSK_16;
		break;
	case FE_SAT_MOD_32APSK:
		return APSK_32;
		break;
	case FE_SAT_MOD_64APSK:
		return APSK_64;
		break;
	case FE_SAT_MOD_128APSK:
		return APSK_128;
		break;
	case FE_SAT_MOD_256APSK:
		return APSK_256;
		break;
	case FE_SAT_MOD_1024APSK:
		return APSK_1024;
		break;
	case FE_SAT_MOD_8PSK_L:
		return APSK_8L;
		break;
	case FE_SAT_MOD_16APSK_L:
		return APSK_16L;
		break;
	case FE_SAT_MOD_32APSK_L:
		return APSK_32L;
		break;
	case FE_SAT_MOD_64APSK_L:
		return APSK_64L;
		break;
	case FE_SAT_MOD_256APSK_L:
		return APSK_256L;
		break;
	case FE_SAT_MOD_DUMMY_PLF:
		return DUMMY_PLF;
	case FE_SAT_MOD_QPSK:
	default:
		return QPSK;
	}
}



static inline enum fe_delivery_system dvb_fec(struct stv* state, enum fe_delivery_system delsys) {
	if (delsys == SYS_DVBS2) {
		static enum fe_code_rate modcod2fec[0x82] = {
			FEC_NONE, FEC_1_4, FEC_1_3, FEC_2_5,
			FEC_1_2, FEC_3_5, FEC_2_3, FEC_3_4,
			FEC_4_5, FEC_5_6, FEC_8_9, FEC_9_10,
			FEC_3_5, FEC_2_3, FEC_3_4, FEC_5_6,
			FEC_8_9, FEC_9_10, FEC_2_3, FEC_3_4,
			FEC_4_5, FEC_5_6, FEC_8_9, FEC_9_10,
			FEC_3_4, FEC_4_5, FEC_5_6, FEC_8_9,
			FEC_9_10, FEC_NONE, FEC_NONE, FEC_NONE,
			FEC_1_2, FEC_2_3, FEC_3_4, FEC_5_6,
			FEC_6_7, FEC_7_8,   FEC_NONE, FEC_NONE,
			FEC_NONE, FEC_NONE, FEC_NONE, FEC_NONE,
			FEC_NONE, FEC_NONE, FEC_NONE, FEC_NONE,
			FEC_NONE, FEC_NONE, FEC_NONE, FEC_NONE,
			FEC_NONE, FEC_NONE, FEC_NONE, FEC_NONE,
			FEC_NONE, FEC_NONE, FEC_NONE, FEC_NONE,
			FEC_NONE, FEC_NONE, FEC_NONE, FEC_NONE,
			FEC_NONE, FEC_NONE, FEC_13_45, FEC_9_20,
			FEC_11_20, FEC_5_9, FEC_26_45, FEC_23_36,
			FEC_25_36, FEC_13_18, FEC_1_2, FEC_8_15,
			FEC_5_9, FEC_26_45, FEC_3_5, FEC_3_5,
			FEC_28_45, FEC_23_36, FEC_2_3, FEC_25_36,
			FEC_13_18, FEC_7_9, FEC_77_90, FEC_2_3,
			FEC_NONE, FEC_32_45, FEC_11_15, FEC_7_9,
			FEC_32_45, FEC_11_15, FEC_NONE, FEC_7_9,
			FEC_NONE, FEC_4_5, FEC_NONE, FEC_5_6,
			FEC_3_4, FEC_7_9, FEC_29_45, FEC_2_3,
			FEC_31_45, FEC_32_45, FEC_11_15, FEC_3_4,
			FEC_11_45, FEC_4_15, FEC_14_45, FEC_7_15,
			FEC_8_15, FEC_32_45, FEC_7_15, FEC_8_15,
			FEC_26_45, FEC_32_45, FEC_7_15, FEC_8_15,
			FEC_26_45, FEC_3_5, FEC_32_45, FEC_2_3,
			FEC_32_45, FEC_NONE, FEC_NONE, FEC_NONE,
			FEC_NONE, FEC_NONE
		};

		if (state->signal_info.modcode < sizeof(modcod2fec)) {
			return modcod2fec[state->signal_info.modcode];
		} else {
			return FEC_AUTO;
			dprintk("Unknown modcode=%d\n", state->signal_info.modcode);
		}
	} else {
		switch (state->signal_info.puncture_rate) {
		case FE_SAT_PR_1_2:
			return  FEC_1_2;
			break;
		case FE_SAT_PR_2_3:
			return  FEC_2_3;
			break;
		case FE_SAT_PR_3_4:
			return  FEC_3_4;
			break;
		case FE_SAT_PR_5_6:
			return  FEC_5_6;
			break;
		case FE_SAT_PR_6_7:
			return  FEC_6_7;
			break;
		case FE_SAT_PR_7_8:
			return  FEC_7_8;
			break;
		default:
			return  FEC_NONE;
		}
	}
}

static inline enum fe_delivery_system dvb_standard(struct stv* state) {
	switch (state->signal_info.standard) {
	case FE_SAT_DSS_STANDARD:
		return SYS_DSS;
		break;
	case FE_SAT_DVBS2_STANDARD:
		return SYS_DVBS2;
		break;
	case FE_SAT_DVBS1_STANDARD:
	default:
		return SYS_DVBS;
	}
}


void print_signal_info_(struct stv* state, const char* func, int line)
{
	#if 1
	struct fe_sat_signal_info* i= &state->signal_info;
	printk(KERN_DEBUG
				 pr_fmt("%s:%d "
								"demod=%d: fec=%d dmd=%d signal=%d carr=%d vit=%d sync=%d timedout=%d lock=%d\n"),
				 func, line,
				 state->nr,
				 i->fec_locked,
				 i->demod_locked,
				 i->has_signal,
				 i->has_carrier,
				 i->has_viterbi,
				 i->has_sync,
				 i->has_timedout,
				 i->has_lock
				 );
	#endif
}

static int stid135_stop_task(struct dvb_frontend* fe);


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
static int stid135_probe(struct stv *state)
{
	struct fe_stid135_init_param init_params;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params;
	enum device_cut_id cut_id;
	int i;

	// for vglna
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
	//end
	dev_warn(&state->base->i2c->dev, "%s\n", FE_STiD135_GetRevision());

	strcpy(init_params.demod_name,"STiD135");
	init_params.pI2CHost		=	state->base;
	init_params.demod_i2c_adr   	=	state->base->adr ? state->base->adr<<1 : 0xd0;
	init_params.demod_ref_clk  	= 	state->base->extclk ? state->base->extclk : 27;
	init_params.internal_dcdc	=	FALSE;
	init_params.internal_ldo	=	TRUE; // LDO supply is internal on Oxford valid board
	init_params.rf_input_type	=	0xF; // Single ended RF input on Oxford valid board rev2
	init_params.roll_off		=  	FE_SAT_35; // NYQUIST Filter value (used for DVBS1/DSS, DVBS2 is automatic)
	init_params.tuner_iq_inversion	=	FE_SAT_IQ_NORMAL;
	err = fe_stid135_init(&init_params, &state->base->ip);
#ifdef PROC_REGISTERS
	chip_init_proc(state, state->base->ip.handle_demod, "stid135");
	chip_init_proc(state, state->base->ip.handle_soc, "soc");
#endif

	init_params.ts_nosync		=	ts_nosync;
	init_params.mis_fix		= mis;

	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_init error %d !\n", __func__, err);
		return -EINVAL;
	}

	p_params = &state->base->ip;
	vprintk("here state=%p\n", state);
	vprintk("here state->base=%p\n", state->base);
	vprintk("here state->base=%p\n", state->base);
	err = fe_stid135_get_cut_id(&state->base->ip, &cut_id);
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
		// note that  FE_SAT_DEMOD_1 is not used in the code in the following call for this ts_mode
		err |= fe_stid135_set_ts_parallel_serial(&state->base->ip, FE_SAT_DEMOD_1, FE_TS_PARALLEL_ON_TSOUT_0);
		err |= fe_stid135_enable_stfe(&state->base->ip,FE_STFE_OUTPUT0);
		err |= fe_stid135_set_stfe(&state->base->ip, FE_STFE_TAGGING_MERGING_MODE, FE_STFE_INPUT1 |
						FE_STFE_INPUT2 |FE_STFE_INPUT3 |FE_STFE_INPUT4| FE_STFE_INPUT5 |
						FE_STFE_INPUT6 |FE_STFE_INPUT7 |FE_STFE_INPUT8 ,FE_STFE_OUTPUT0, 0xDE);
	} else if (state->base->ts_mode == TS_8SER) { //DT: This code is not called
		dev_warn(&state->base->i2c->dev, "%s: 8xTS serial mode init.\n", __func__);
		for(i=0;i<8;i++) {
			err |= fe_stid135_set_ts_parallel_serial(&state->base->ip, i+1, FE_TS_SERIAL_CONT_CLOCK);
			//err |= fe_stid135_set_maxllr_rate(state, i+1, 90);
		}
	} else { //DT: This code is called on TBS6903X?
		dev_warn(&state->base->i2c->dev, "%s: 2xTS parallel mode init.\n", __func__);
		err |= fe_stid135_set_ts_parallel_serial(&state->base->ip, FE_SAT_DEMOD_3, FE_TS_PARALLEL_PUNCT_CLOCK);
		err |= fe_stid135_set_ts_parallel_serial(&state->base->ip, FE_SAT_DEMOD_1, FE_TS_PARALLEL_PUNCT_CLOCK);
	}
	if (state->base->mode == 0) {
		dev_warn(&state->base->i2c->dev, "%s: multiswitch mode init.\n", __func__);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER1);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER2);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER3);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER4);
		err |= fe_stid135_diseqc_init(&state->base->ip,AFE_TUNER1, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state->base->ip,AFE_TUNER2, FE_SAT_22KHZ_Continues);
		err |= fe_stid135_diseqc_init(&state->base->ip,AFE_TUNER3, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state->base->ip,AFE_TUNER4, FE_SAT_22KHZ_Continues);
		if (state->base->set_voltage) {
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_13, 0);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_13, 1);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_18, 2);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_18, 3);
		}
	} else {
		err |= fe_stid135_diseqc_init(&state->base->ip,AFE_TUNER1, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state->base->ip,AFE_TUNER2, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state->base->ip,AFE_TUNER3, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state->base->ip,AFE_TUNER4, FE_SAT_DISEQC_2_3_PWM);
		if (state->base->set_voltage) {
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_OFF, 0);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_OFF, 1);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_OFF, 2);
			state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_OFF, 3);
		}

	}
///////////////////*stvvglna*////////////////////////
	if(state->base->vglna) { //for 6909x v2 version
		dev_warn(&state->base->i2c->dev, "%s:Init STVVGLNA \n", __func__);
	VglnaIdString = "STVVGLNA";
	pVGLNAInit.Chip = &VGLNAChip;

	pVGLNAInit.Chip->pI2CHost	  =	state->base;
	pVGLNAInit.Chip->RepeaterHost = NULL;
	pVGLNAInit.Chip->Repeater     = FALSE;
	pVGLNAInit.Chip->I2cAddr      = 0xc8;
	pVGLNAInit.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit, &vglna_handle);
#ifdef PROC_REGISTERS
	chip_init_proc(state, vglna_handle, "vglna0");
#endif

	stvvglna_set_standby(vglna_handle, vglna_mode);
	dev_warn(&state->base->i2c->dev, "Initialized STVVGLNA 0 device\n");

	VglnaIdString = "STVVGLNA1";
	pVGLNAInit1.Chip = &VGLNAChip;

	pVGLNAInit1.Chip->pI2CHost	  =	state->base;
	pVGLNAInit1.Chip->RepeaterHost = NULL;
	pVGLNAInit1.Chip->Repeater     = FALSE;
	pVGLNAInit1.Chip->I2cAddr      = 0xce;
	pVGLNAInit1.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit1.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit1, &vglna_handle1);
#ifdef PROC_REGISTERS
	chip_init_proc(state, vglna_handle, "vglna1");
#endif
	stvvglna_set_standby(vglna_handle1, vglna_mode);
	dev_warn(&state->base->i2c->dev, "Initialized STVVGLNA 1 device\n");

	VglnaIdString = "STVVGLNA2";
	pVGLNAInit2.Chip = &VGLNAChip;
	pVGLNAInit2.Chip->pI2CHost	  =	state->base;
	pVGLNAInit2.Chip->RepeaterHost = NULL;
	pVGLNAInit2.Chip->Repeater     = FALSE;
	pVGLNAInit2.Chip->I2cAddr      = 0xcc;
	pVGLNAInit2.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit2.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit2, &vglna_handle2);
#ifdef PROC_REGISTERS
	chip_init_proc(state, vglna_handle, "vglna2");
#endif

	stvvglna_set_standby(vglna_handle2, vglna_mode);
	dev_warn(&state->base->i2c->dev, "Initialized STVVGLNA 2 device\n");

	VglnaIdString = "STVVGLNA3";
	pVGLNAInit3.Chip = &VGLNAChip;
	pVGLNAInit3.Chip->pI2CHost	  =	state->base;
	pVGLNAInit3.Chip->RepeaterHost = NULL;
	pVGLNAInit3.Chip->Repeater     = FALSE;
	pVGLNAInit3.Chip->I2cAddr      = 0xca;
	pVGLNAInit3.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit3.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit3, &vglna_handle3);
#ifdef PROC_REGISTERS
	chip_init_proc(state, vglna_handle, "vglna3");
#endif

	stvvglna_set_standby(vglna_handle3, vglna_mode);
	dev_warn(&state->base->i2c->dev, "Initialized STVVGLNA 3 device\n");
	}
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: setup error %d !\n", __func__, err);
	return err != FE_LLA_NO_ERROR ? -1 : 0;
}


static int stid135_select_rf_in_(struct stv* state, int rf_in)
{
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	dprintk("demod=%d rf_in=%d old=%d rf_in_selected=%d; use counts: %d old=%d\n", state->nr, rf_in, state->rf_in,
					state->rf_in_selected, state->base->tuner_use_count[rf_in], state->base->tuner_use_count[state->rf_in]);
	BUG_ON((state->rf_in < 0 || state->rf_in >= 4));
	BUG_ON(rf_in < 0);
	if(rf_in >= 4) {
		dprintk("rf_in=%d out of range\n", rf_in);
		return -EINVAL;
	}
	if(rf_in == state->rf_in && state->rf_in_selected)
		return 0;  //already active

	if(rf_in != state->rf_in && state->rf_in_selected) {
		//change in rf_in
		BUG_ON(state->base->tuner_use_count[state->rf_in] < 0 || state->base->tuner_use_count[state->rf_in] > 4 );
		BUG_ON(state->base->tuner_use_count[rf_in] < 0 || state->base->tuner_use_count[rf_in] > 3 );

		state->base->tuner_use_count[state->rf_in]--; //decrease use count for old tuner

		if(state->base->tuner_owner[state->rf_in] == state->nr) {
			state->base->tuner_owner[state->rf_in] = -1; //release ownership of old tuner if we had it
			dprintk("demod=%d: released tuner ownership of rf_in=%d; use_count=%d\n", state->nr, state->rf_in,
							state->base->tuner_use_count[state->rf_in]);
		}

		//if no demod uses the old tuner anymore, put it to sleep
		if(state->base->tuner_use_count[state->rf_in] == 0) {
			dprintk("demod=%d: Calling TunerStandby 0 state->base=%p\n", state->nr, state->base);
			err = fe_stid135_set_22khz_cont(&state->base->ip, state->rf_in + 1, false);
			dprintk("demod=%d: Calling TunerStandby 0 done: state->base->i2c=%p\n", state->nr, state->base->i2c);
			if(state->base->set_voltage) {
				state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_OFF, state->rf_in);
			}
			dprintk("demod=%d: Set voltage OFF done\n", state->nr);
			err |= FE_STiD135_TunerStandby(state->base->ip.handle_demod, state->rf_in + 1, 0);
			dprintk("demod=%d: Tuner standby done done\n", state->nr);
		}
	}

	//increase use count of new tuner; do not take ownership until the next operation (voltage/tone/diseqc)
	if(state->base->tuner_use_count[rf_in]++ == 0) {
		dprintk("Enabling tuner demod=%d rf_in=%d\n", state->nr, rf_in);
		err |= fe_stid135_tuner_enable(state->base->ip.handle_demod, rf_in + 1);
		err |= fe_stid135_diseqc_init(&state->base->ip, rf_in + 1, FE_SAT_DISEQC_2_3_PWM);
	}

	dprintk("Setting rf_mux_path demod=%d rf_in=%d\n", state->nr, rf_in);
	err |= fe_stid135_set_rfmux_path(state, rf_in + 1);
	state->rf_in = rf_in;
	state->rf_in_selected = true;
	return err;
}

static int stid135_init(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

	if (state->base->mode == 0)
		return 0;

	dev_dbg(&state->base->i2c->dev, "%s: demod %d + tuner %d\n", __func__, state->nr, state->rf_in);
	if(state->base->mode == 0) {
		base_lock(state);
		err |= stid135_select_rf_in_(state, state->rf_in);
		base_unlock(state);
	} else {

		dprintk("state->rf_in_selected=%d adapter=%d state->rf_in=%d\n", state->rf_in_selected, state->nr, state->rf_in);
	}
	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static void stid135_release(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	dev_dbg(&state->base->i2c->dev, "%s: demod %d\n", __func__, state->nr);

	state->base->count--;
	if (state->base->count == 0) {
		base_lock(state);
#ifdef PROC_REGISTERS //todo: rewrite this code
		chip_close_proc(state, "stid135");
		chip_close_proc(state, "soc");
		if(state->base->vglna) { //for 6909x v2 version
			chip_close_proc(state, "vglna0");
			chip_close_proc(state, "vglna1");
			chip_close_proc(state, "vglna2");
			chip_close_proc(state, "vglna3");
		}
#endif
		FE_STiD135_Term (&state->base->ip);
		base_unlock(state);
		list_del(&state->base->stvlist);
		kfree(state->base);
	}
	kfree(state);
}

static bool pls_search_list(struct dvb_frontend* fe)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct stv *state = fe->demodulator_priv;
	struct fe_sat_signal_info* signal_info = &state->signal_info;
	int i = 0;
	int locked = 0;
	u8 matype_info;
	u8 isi;
	for(i=0; i<p->pls_search_codes_len;++i) {
		u32 pls_code = p->pls_search_codes[i];
		s32 pktdelin;
		u8 timeout = pls_code & 0xff;
		BUG_ON(i <0 || i>= sizeof(p->pls_search_codes)/sizeof(p->pls_search_codes[0]));
		vprintk("Trying scrambling mode=%d code %d stream_id=%d timeout=%d\n", (pls_code>>26) & 0x3,
						(pls_code>>8) & 0x3FFFF, p->stream_id, timeout);
		set_pls_mode_code(state, (pls_code>>26) & 0x3, (pls_code>>8) & 0x3FFFF);
#if 0
		if(p->stream_id !=  NO_STREAM_ID_FILTER)
			fe_stid135_set_mis_filtering(state,  TRUE, p->stream_id & 0xFF, 0xFF);
		else
			fe_stid135_set_mis_filtering(state,  TRUE, state->signal_info.isi & 0xFF, 0xFF);
		//fe_stid135_set_mis_filtering(state,  FALSE, 0, 0xFF);
#endif
		//write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x15);
		//write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x18);
		msleep(timeout? timeout: 100); //0 means: use default
		error =ChipGetField(state->base->ip.handle_demod,
												FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_LOCK(state->nr+1), &pktdelin);
		if(error)
			dprintk("FAILED; error=%d\n", error);


		if (pktdelin/*packet delineator locked*/) {
			locked=1;
			vprintk("PLS LOCKED\n");
		} else {
			vprintk("PLS NOT LOCKED\n");
		}
		//locked = wait_for_dmdlock(fe, 1 /*require_data*/);
		//dprintk("RESULT=%d\n", locked);
			if(locked) {
				error = fe_stid135_read_hw_matype(state, &matype_info, &isi);
				state->mis_mode= !fe_stid135_check_sis_or_mis(matype_info);
				dprintk("demod=%d: ISI mis_mode set to %d\n", state->nr, state->mis_mode);
				dprintk("demod=%d: selecting stream_id=%d\n", state->nr, isi);
				signal_info->isi = isi;
				//p->matype = matype_info;
				p->stream_id = 	(state->mis_mode? (isi&0xff):0xff) | (pls_code & ~0xff);
				dprintk("mis_mode=%d isi=0x%x pls_code=0x%x stream_id=0x%x",
								state->mis_mode, isi, pls_code, p->stream_id);
				dprintk("SET stream_id=0x%x isi=0x%x\n",p->stream_id, isi);
				break;
			}

			if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
				dprintk("exiting on should stop\n");
			break;
		}

	}

	if(locked) {
		FE_STiD135_GetFECLock(state, 200, &locked);
		dprintk("FEC LOCK=%d\n", locked);
	}
	return locked;
}


static bool pls_search_range(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_sat_signal_info* signal_info = &state->signal_info;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	u32 pls_code = 0;
	int locked = 0;
	u8 timeout = p->pls_search_range_start & 0xff;
	int count=0;
	u8 matype_info;
	u8 isi;
	if(p->pls_search_range_end == 0)
		return false;
	if(p->pls_search_range_start >= p->pls_search_range_end) {
		dprintk("Invalid PLS range: %d - %d", p->pls_search_range_start, p->pls_search_range_end);
		return false;
	}
	if(timeout==0)
		timeout = 25;
	dprintk("pls search range: %d to %d timeout=%d\n",
					(p->pls_search_range_start) & 0x3FFFF,
					(p->pls_search_range_end) & 0x3FFFF, timeout);

	atomic_set(&fe->algo_state.cur_index, 0);
	atomic_set(&fe->algo_state.max_index, (p->pls_search_range_end - p->pls_search_range_start)>>8);
	for(pls_code=p->pls_search_range_start; pls_code<p->pls_search_range_end; pls_code += 0x100, count++) {
		s32 pktdelin;
		if((count+= timeout)>=1000) {
			vprintk("Trying scrambling mode=%d code %d timeout=%d ...\n",
							(pls_code>>26) & 0x3, (pls_code>>8) & 0x3FFFF, timeout);
			atomic_add(count, &fe->algo_state.cur_index);
			count=0;
			//wake_up_interruptible(&fe->algo_state.wait_queue);
		}
		set_pls_mode_code(state, (pls_code>>26) & 0x3, (pls_code>>8) & 0x3FFFF);
		//write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x15);
		//write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x18);
		msleep(timeout? timeout: 25); //0 means: use default

		error =ChipGetField(state->base->ip.handle_demod,
												FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_LOCK(state->nr+1), &pktdelin);
		if(error)
			dprintk("FAILED; error=%d\n", error);

		if (pktdelin /*packet delineator locked*/)
			locked=1;
		//locked = wait_for_dmdlock(fe, 0 /*require_data*/);
		if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
			dprintk("exiting on should stop\n");
			break;
		}
		vprintk("PLS RESULT=%d\n", locked);
		if(locked) {
			error = fe_stid135_read_hw_matype(state, &matype_info, &isi);
			state->mis_mode= !fe_stid135_check_sis_or_mis(matype_info);
			dprintk("demod=%d: ISI mis_mode set to %d; selecting stream_id=%d\n", state->nr, state->mis_mode, isi);
			signal_info->isi = isi;
			p->stream_id = 	(state->mis_mode? (isi&0xff):0xff) | (pls_code & ~0xff);
			dprintk("demod=%d: ISI mis_mode=%d isi=0x%x pls_code=0x%x stream_id=0x%x", state->nr,
								state->mis_mode, isi, pls_code, p->stream_id);

		  //p->matype = matype_info;
			dprintk("PLS SET stream_id=0x%x isi=0x%x\n",p->stream_id, isi);
				break;
		}
	}
	dprintk("PLS search ended\n");
	return locked;
}


static int stid135_set_rf_input(struct dvb_frontend* fe, s32 rf_in)
{
	int ret;
	struct stv *state = fe->demodulator_priv;
	base_lock(state);
	ret = stid135_select_rf_in_(state, rf_in);
	base_unlock(state);
	return ret;
}


static int stid135_set_parameters(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;
	struct fe_sat_search_params search_params;
	s32 rf_power;
	//BOOL lock_stat=0;
	struct fe_sat_signal_info* signal_info = &state->signal_info;
	s32 current_llr=0;
	state->tune_time = ktime_get_coarse();
	memset(signal_info, 0, sizeof(*signal_info));
	vprintk(
					"[%d] delivery_system=%u modulation=%u frequency=%u symbol_rate=%u inversion=%u stream_id=%d\n",
					state->nr+1,
					p->delivery_system, p->modulation, p->frequency,
					p->symbol_rate, p->inversion, p->stream_id);
	dev_dbg(&state->base->i2c->dev,
			"delivery_system=%u modulation=%u frequency=%u symbol_rate=%u inversion=%u stream_id=%d\n",
			p->delivery_system, p->modulation, p->frequency,
					p->symbol_rate, p->inversion, p->stream_id);
	dprintk("demod=%d: user set stream_id=0x%x", state->nr, p->stream_id);

	if(blindscan_always) {
		p->algorithm = ALGORITHM_WARM;
		p->delivery_system = SYS_AUTO;
		p->symbol_rate = p->symbol_rate ==0 ? 22000000 : p->symbol_rate;
	}

#if 1 // official driver always uses FE_SAT_COLD_START
	/* Search parameters */
	switch(p->algorithm) {
	case ALGORITHM_WARM:
		search_params.search_algo	= FE_SAT_WARM_START;
		search_params.symbol_rate = p->symbol_rate;
		break;

	case ALGORITHM_COLD:
	case ALGORITHM_COLD_BEST_GUESS:
		search_params.search_algo		= FE_SAT_COLD_START;
		search_params.symbol_rate = p->symbol_rate ;
		break;

	case ALGORITHM_BLIND:
	case ALGORITHM_BLIND_BEST_GUESS:
#if 0
	case ALGORITHM_BANDWIDTH:
#endif
		search_params.search_algo		= FE_SAT_BLIND_SEARCH;
		search_params.standard = FE_SAT_AUTO_SEARCH;
		search_params.puncture_rate = FE_SAT_PR_UNKNOWN;
		if(p->symbol_rate<=0)
			search_params.symbol_rate = 22000000;
		else
			search_params.symbol_rate = p->symbol_rate;
		break;
#if 0
	case ALGORITHM_SEARCH_NEXT:
		search_params.search_algo		= FE_SAT_NEXT;
		search_params.standard = FE_SAT_AUTO_SEARCH;
		search_params.puncture_rate = FE_SAT_PR_UNKNOWN;
		if(p->symbol_rate<=0)
			search_params.symbol_rate = 22000000;
		else
			search_params.symbol_rate = p->symbol_rate;
		break;
	case ALGORITHM_SEARCH:
		//todo
		break;
#endif
	}
	if(p->algorithm != ALGORITHM_WARM)
		vprintk("[%d] Algorithm is not  WARM: %d\n", state->nr+1, p->algorithm);
#else
	search_params.search_algo 	= FE_SAT_COLD_START;
#endif

	search_params.stream_id = p->stream_id;

	search_params.frequency		=  p->frequency*1000;
	//search_params.symbol_rate		=		p->symbol_rate;
	vprintk("[%d] symbol_rate=%dkS/s\n", state->nr+1, p->symbol_rate/1000);
	search_params.modulation	= FE_SAT_MOD_UNKNOWN;
	search_params.modcode		= FE_SAT_DUMMY_PLF;
	search_params.search_range_hz	= p->search_range > 0 ? p->search_range : 10000000; //TODO => check during blindscan
	vprintk("[%d] SEARCH range set to %d (p=%d)\n",
					state->nr+1,
					search_params.search_range_hz, p->search_range );

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

	search_params.iq_inversion	= FE_SAT_IQ_AUTO;
	search_params.tuner_index_jump	= 0; // ok with narrow band signal
	//the following is in Mhz despite what the name suggests
	err = FE_STiD135_GetLoFreqHz(&state->base->ip, &(search_params.lo_frequency));
	vprintk("[%d] lo_frequency = %d\n", state->nr+1, search_params.lo_frequency); //1 550 000kHz
	search_params.lo_frequency *= 1000000; ///in Hz
	if(search_params.search_algo == FE_SAT_BLIND_SEARCH ||
		 search_params.search_algo == FE_SAT_NEXT) {
		//search_params.frequency =		950000000 ;
		vprintk("[%d] BLIND: set freq=%d lo=%d\n",
						state->nr+1,
						search_params.frequency,	search_params.lo_frequency  );
	}

#if 0
	if(reserve_llr_for_symbolrate(state, p->symbol_rate) == FE_LLA_OUT_OF_LLR)
		return FE_LLA_OUT_OF_LLR;
#endif
	dev_dbg(&state->base->i2c->dev, "%s: demod %d + tuner %d\n", __func__, state->nr, state->rf_in);
	vprintk("[%d] demod %d + tuner %d\n",
					state->nr+1,state->nr, state->rf_in);
	if(p->rf_in < 0  || !p->rf_in_valid)  {
		p->rf_in = state->rf_in_selected ? state->rf_in : state->fe.ops.info.default_rf_input;
		p->rf_in_valid = true;
		dprintk("demod=%d: Set rf_in to %d; rf_in_selected=%d  state->rf_in=%d default_rf_in=%d\n",
						state->nr, p->rf_in, state->rf_in_selected, state->rf_in, state->fe.ops.info.default_rf_input);
	}
	dprintk("demod=%d: Setting rf_in: %d\n", state->nr, p->rf_in);
	err |= stid135_select_rf_in_(state, p->rf_in);
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_rfmux_math error %d !\n", __func__, err);
	if(state->modcode_filter){

		/*Deep Thought: keeping filters ensures that  a demod does not cause a storm of data when demodulation is
			failing (e.g., ran fade) This could cause other demods to fail as well as they share resources.
			filter_forbidden_modcodes may be better

				There could be reasons why  fe_stid135_reset_modcodes_filter is still needed, e.g., when  too strict filters
				are left from an earlier tune?
		*/
		err |= fe_stid135_reset_modcodes_filter(state);
		if (err != FE_LLA_NO_ERROR) {
			dev_err(&state->base->i2c->dev, "%s: fe_stid135_reset_modcodes_filter error %d !\n", __func__, err);
			vprintk("[%d]: fe_stid135_reset_modcodes_filter error %d !\n", state->nr+1, err);
		}
	}

	err |= (error1=fe_stid135_search(state, &search_params, 0));
	if(error1!=0)
		dprintk("demod=%d: fe_stid135_search returned error=%d\n", state->nr, error1);
	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_search error %d !\n", __func__, err);
		dprintk("demod=%d: fe_stid135_search error %d !\n", state->nr, err);
		return -1;
	}
#if 1 //missing in official driver, but only called during blindscan
	if(!state->signal_info.has_viterbi && p->algorithm != ALGORITHM_WARM && p->algorithm != ALGORITHM_COLD) {
		bool locked=false;
		//print_signal_info(state, "(before trying pls)");
		vprintk("demod=%d: Trying pls: p->stream_id=%d state->signal_info.isi=0x%x\n", state->nr,
						p->stream_id, state->signal_info.isi);
		locked = pls_search_list(fe);
		if(!locked)
			locked = pls_search_range(fe);
		if(locked) {
			dprintk("demod=%d: PLS locked=%d\n", state->nr, locked);
		} else {
			set_pls_mode_code(state, 0, 1);
		}
		vprintk("After Trying pls: p->stream_id=%d locked=%d\n", p->stream_id, locked);
	} else {
		vprintk("now stream_id=0x%x\n", p->stream_id);
		if(p->stream_id != NO_STREAM_ID_FILTER) {
			vprintk("calling set_stream_index");
			set_stream_index(state, p->stream_id);
		}
	}

	vprintk("[%d] setting timedout=%d\n", state->nr+1, !state->signal_info.has_lock);
	/*
		has_viterbi: correctly tuned
		has_sync: received packets without errors
	 */

	vprintk("[%d] set_parameters: error=%d locked=%d vit=%d sync=%d timeout=%d\n",
					state->nr +1,
					err, state->signal_info.has_lock,
					state->signal_info.has_viterbi,					state->signal_info.has_sync,
					state->signal_info.has_timedout);
#endif

	if (state->signal_info.has_carrier){ //official driver may require has_sync

		vprintk("[%d] set_parameters: error=%d locked=%d\n", state->nr+1, err, state->signal_info.has_lock);

		dev_dbg(&state->base->i2c->dev, "%s: locked !\n", __func__);
		//set maxllr,when the  demod locked ,allocation of resources
		//err |= fe_stid135_set_maxllr_rate(state, 180);
		if(get_current_llr(state, &current_llr) == FE_LLA_OUT_OF_LLR) {
			dprintk("setting state->signal_info.out_of_llr = true");
			state->signal_info.out_of_llr = true;
			return -1;
		} else {
			dprintk("setting state->signal_info.out_of_llr = false");
			state->signal_info.out_of_llr = false;
		}

		//for tbs6912
		state->newTP = true;
		state->loops = 15;
		if(state->base->set_TSsampling)
			state->base->set_TSsampling(state->base->i2c,state->nr/2,4);   //for tbs6912
	} else {
		//state->signal_info.has_signal = 1;
		//state->signal_info.has_lock = 0;
		err |= fe_stid135_get_band_power_demod_not_locked(state, &rf_power);
		dev_dbg(&state->base->i2c->dev, "%s: not locked, band rf_power %d dBm ! demod=%d tuner=%d\n",
						 __func__, rf_power / 1000, state->nr, state->rf_in);
	}
	vprintk("[%d] set_parameters: error=%d locked=%d\n", state->nr+1, err, state->signal_info.has_lock);

	/* Set modcode after search */
	if (p->modcode != MODCODE_ALL) {
        u32 m = p->modcode;
        u32 j = 0;
				u32 i;
				struct fe_sat_dvbs2_mode_t modcode_mask[FE_SAT_MODCODE_UNKNOWN*4];
        dev_dbg(&state->base->i2c->dev, "%s: set Modcode mask %x!\n", __func__, p->modcode);
        m >>= 1;
        for (i=FE_SAT_QPSK_14; i < FE_SAT_MODCODE_UNKNOWN; i ++) {
            if (m & 1) {
                dev_dbg(&state->base->i2c->dev, "%s: Modcode %02x enabled!\n", __func__, i);
                modcode_mask[j].mod_code = i;
                modcode_mask[j].pilots = FE_SAT_PILOTS_OFF;
                modcode_mask[j].frame_length = FE_SAT_NORMAL_FRAME;
                modcode_mask[j+1].mod_code = i;
                modcode_mask[j+1].pilots = FE_SAT_PILOTS_ON;
                modcode_mask[j+1].frame_length = FE_SAT_NORMAL_FRAME;
                modcode_mask[j+2].mod_code = i;
                modcode_mask[j+2].pilots = FE_SAT_PILOTS_OFF;
                modcode_mask[j+2].frame_length = FE_SAT_SHORT_FRAME;
                modcode_mask[j+3].mod_code = i;
                modcode_mask[j+3].pilots = FE_SAT_PILOTS_ON;
                modcode_mask[j+3].frame_length = FE_SAT_SHORT_FRAME;
                j+=4;
            }
            m >>= 1;
        }
				state->modcode_filter = true;
        err |= fe_stid135_set_modcodes_filter(state, modcode_mask, j);
        if (err != FE_LLA_NO_ERROR)
            dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_modcodes_filter error %d !\n", __func__, err);
	}

	/* Set ISI after search */
	if (p->stream_id != NO_STREAM_ID_FILTER) {
		dev_warn(&state->base->i2c->dev, "%s: set ISI %d ! demod=%d tuner=%d\n", __func__, p->stream_id & 0xFF,
						 state->nr, state->rf_in);
		err |= fe_stid135_set_mis_filtering(state, TRUE, p->stream_id & 0xFF, 0xFF);
	} else {
		dev_dbg(&state->base->i2c->dev, "%s: disable ISI filtering !\n", __func__);
		err |= fe_stid135_set_mis_filtering(state, FALSE, 0, 0xFF);
	}
	if(p->stream_id == NO_STREAM_ID_FILTER) {
		state->signal_info.pls_mode = 0x00; //ROOT
		state->signal_info.pls_code = 1;
	} else {
		state->signal_info.pls_mode = ((p->stream_id >>26) & 0x3);
		state->signal_info.pls_code = ((p->stream_id >> 8)  & 0x3FFFF);
	}

	dprintk("set state->signal_info pls_mode=0x%x pls_code=0x%x stream_id=0%x",
					state->signal_info.pls_mode, state->signal_info.pls_code, p->stream_id);

	if(	state->signal_info.pls_code ==0)
			state->signal_info.pls_code = 1;
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_mis_filtering error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

#if 0
static int stid135_get_frontend(struct dvb_frontend* fe, struct dtv_frontend_properties *p)
{
	struct stv *state = fe->demodulator_priv;

	if (!state->signal_info.has_viterbi) {//official driver would test for has_sync
		vprintk("no viterbi\n");
		return 0;
	}

	if(true || state->demod_search_algo == FE_SAT_BLIND_SEARCH ||
							state->demod_search_algo == FE_SAT_NEXT) {
		memcpy(p->isi_bitset, state->signal_info.isi_list.isi_bitset, sizeof(p->isi_bitset));
		memcpy(p->matypes, state->signal_info.isi_list.matypes,
					 state->signal_info.isi_list.num_matypes*sizeof(p->matypes[0]));
		p->num_matypes = state->signal_info.isi_list.num_matypes;
		p->matype_val = state->signal_info.matype;
		p->matype_valid = true;
		p->frequency = state->signal_info.frequency;
		p->symbol_rate = state->signal_info.symbol_rate;
	}
	p->delivery_system = dvb_standard(state);
	p->modulation = dvb_modulation(state);
	p->rolloff = dvb_rolloff(state);

	p->inversion = state->signal_info.spectrum == FE_SAT_IQ_SWAPPED ? INVERSION_ON : INVERSION_OFF;
	p->modcode = state->signal_info.modcode;

	p->pilot = state->signal_info.pilots == FE_SAT_PILOTS_ON ? PILOT_ON : PILOT_OFF;
	p->fec_inner = dvb_fec(state, p->delivery_system);

	p->stream_id = ((state->mis_mode ? (state->signal_info.isi &0xff) :0xff) |
									(state->signal_info.pls_mode << 26) |
									((state->signal_info.pls_code &0x3FFFF)<<8)
									);
	vprintk("read stream_id mis=%d pls_mode=0x%x pls_code=0x%x stream_id=0%x",
					state->mis_mode,
					state->signal_info.pls_mode, state->signal_info.pls_code, p->stream_id);

	vprintk("READ stream_id=0x%x isi=0x%x\n",p->stream_id, state->signal_info.isi);
	return 0;
}
#endif

static int stid135_read_status_(struct dvb_frontend* fe, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	u32 speed;
	*status = 0;
	if(state->signal_info.has_error) {
		*status = FE_TIMEDOUT;
		return 0;
	}

	p->locktime = ktime_to_ns(state->signal_info.lock_time)/1000000;
	p->strength.len = 1;
	p->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->cnr.len = 1;
	p->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->pre_bit_error.len =1;
	p->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->pre_bit_count.len =1;
	p->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	err = fe_stid135_get_lock_status(state, NULL, NULL, NULL);
	if(state->signal_info.has_carrier)
		*status |= (FE_HAS_SIGNAL|FE_HAS_CARRIER);
	if(state->signal_info.has_viterbi)
		*status |= FE_HAS_VITERBI|FE_HAS_LOCK;
	if(state->signal_info.has_sync)
		*status |= FE_HAS_SYNC|FE_HAS_LOCK;
	if(state->signal_info.has_lock)
		*status |= FE_HAS_LOCK;
	if(state->signal_info.has_timing_lock)
		*status |= FE_HAS_TIMING_LOCK;
	if(state->signal_info.has_timedout)
		*status |= FE_TIMEDOUT;
	if(state->signal_info.out_of_llr) {
		dprintk("setting FE_OUT_OF_RESOURCES\n");
		*status |= FE_OUT_OF_RESOURCES;
	}
	//dprintk("set *status=0x%x\n", *status);
	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state->base->i2c->dev, "fe_stid135_get_lock_status error\n");
		return -EIO;
	}

	if (!state->signal_info.has_carrier) {
		/* demod not locked */
		*status |= FE_HAS_SIGNAL;
		vprintk("HAS_SIGNAL AND TUNED/no viterbi sync=%d status=%d\n", state->signal_info.has_sync, *status);
		err = fe_stid135_get_band_power_demod_not_locked(state,  &state->signal_info.power);
		// if unlocked, set to lowest resource..
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
		p->cnr.len=0; //may be a will confuse user space
		return 0;
	}

	get_raw_bit_rate(state, &p->bit_rate);
	vprintk("NOW: raw_bit_rate=%d\n", p->bit_rate);

	/* demod has lock */

	err = fe_stid135_get_signal_quality(state, &state->signal_info, mc_auto);

	if (err != FE_LLA_NO_ERROR) {
		dprintk("fe_stid135_get_signal_quality err=%d\n", err); //7 means ST_ERROR_INVALID_HANDLE FE_LLA_INVALID_HANDLE
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

	p->pre_bit_error.len = 1;
	p->pre_bit_error.stat[0].scale = FE_SCALE_COUNTER;
	p->pre_bit_error.stat[0].uvalue = state->signal_info.ber;
	p->pre_bit_count.stat[0].scale = FE_SCALE_COUNTER;
	p->pre_bit_count.stat[0].uvalue = 1e7;

	if (err != FE_LLA_NO_ERROR)
		dev_warn(&state->base->i2c->dev, "%s: fe_stid135_filter_forbidden_modcodes error %d !\n", __func__, err);

	//update isi list
	if(state->mis_mode) {
		vprintk("ISI calling isi_scan\n");
		err = fe_stid135_isi_scan(state, &state->signal_info.isi_list);
	}
	memcpy(p->isi_bitset, state->signal_info.isi_list.isi_bitset, sizeof(p->isi_bitset));
	memcpy(p->matypes, state->signal_info.isi_list.matypes, sizeof(p->matypes));
	p->num_matypes = state->signal_info.isi_list.num_matypes;
	vprintk("p->num_matypes=%d %d\n", p->num_matypes, state->signal_info.isi_list.num_matypes);

	{
		u8 matype;
		u8 isi_read;
		fe_stid135_read_hw_matype(state, &matype, &isi_read);
		if ( !!((matype &0x3) == 0x3) != !!((state->signal_info.matype &0x3) == 0x3)) {
			state->signal_info.low_roll_off_detected = true;
		}

		if(!((matype &0x3) == 0x3))
			state->signal_info.matype = matype;
		switch(state->signal_info.matype) {
		case 0:
			state->signal_info.roll_off = state->signal_info.low_roll_off_detected ? FE_SAT_15: FE_SAT_35;
			break;
		case 1:
			state->signal_info.roll_off = state->signal_info.low_roll_off_detected ? FE_SAT_10: FE_SAT_25;
			break;
		case 2:
			state->signal_info.roll_off = state->signal_info.low_roll_off_detected ? FE_SAT_05: FE_SAT_20;
			break;
		case 3:
			state->signal_info.roll_off = FE_SAT_LOW;
			break;
		}
	}

	p->matype_val = state->signal_info.matype;
	p->matype_valid = true;

	p->frequency = state->signal_info.frequency;
	p->symbol_rate = state->signal_info.symbol_rate;


	p->delivery_system = dvb_standard(state);
	p->modulation = dvb_modulation(state);
	p->rolloff = dvb_rolloff(state);

	p->inversion = state->signal_info.spectrum == FE_SAT_IQ_SWAPPED ? INVERSION_ON : INVERSION_OFF;
	p->modcode = state->signal_info.modcode;

	p->pilot = state->signal_info.pilots == FE_SAT_PILOTS_ON ? PILOT_ON : PILOT_OFF;
	p->fec_inner = dvb_fec(state, p->delivery_system);

	p->stream_id = ((state->mis_mode ? (state->signal_info.isi &0xff) :0xff) |
									(state->signal_info.pls_mode << 26) |
									((state->signal_info.pls_code &0x3FFFF)<<8)
									);
	vprintk("read stream_id mis=%d pls_mode=0x%x pls_code=0x%x stream_id=0%x",
					state->mis_mode,
					state->signal_info.pls_mode, state->signal_info.pls_code, p->stream_id);

	vprintk("READ stream_id=0x%x isi=0x%x\n",p->stream_id, state->signal_info.isi);

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


static int stid135_read_status(struct dvb_frontend* fe, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	int ret = 0;
	if (!base_trylock(state)) {
		if (state->signal_info.has_viterbi) {//XX: official driver tests for has_sync?
			*status |= FE_HAS_SIGNAL | FE_HAS_CARRIER
				| FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		}
		return 0;
	}
	ret = stid135_read_status_(fe, status);
	base_unlock(state);
	return ret;
}


static int stid135_tune_(struct dvb_frontend* fe, bool re_tune,
		unsigned int mode_flags,
		unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	int r;
	dprintk("demod=%d re_tune=%d\n", state->nr, re_tune);
	if (re_tune) {
		state->signal_info.out_of_llr = false;
		r = stid135_set_parameters(fe);
		if (r) {
			state->signal_info.has_error = true;
			if(state->signal_info.out_of_llr) {
				dprintk("demod=%d setting FE_OUT_OF_RESOURCES\n", state->nr);
			}
			*status = state->signal_info.out_of_llr ? FE_OUT_OF_RESOURCES : FE_TIMEDOUT;
			return r;
		}
		state->tune_time = jiffies;
	}

	if(re_tune) {
		vprintk("[%d] RETUNE: GET SIGNAL\n", state->nr+1);
		/*
			 retrieve information about modulation, frequency, symbol_rate
			 and CNR, BER
		*/
		memset(&state->signal_info.isi_list, 0, sizeof(state->signal_info.isi_list));
		fe_stid135_get_signal_info(state);
	}
	/*
		get lock status
		get rf level, CNR, BER
	 */
	r = stid135_read_status_(fe, status);
	dprintk("demod=%d LOCK TIME %lldms locked=%d\n", state->nr,
					ktime_to_ns(state->signal_info.lock_time)/1000000, state->signal_info.has_lock);
	vprintk("demod=%d setting timedout=%d\n", state->nr, !state->signal_info.has_lock);
	if(state->signal_info.has_timedout) {
		*status |= FE_TIMEDOUT;
		*status &= ~FE_HAS_LOCK;
	}

	if (r)
		return r;

	if (*status & FE_HAS_LOCK)
		return 0;

	*delay = HZ;

	return 0;
}

static int stid135_constellation_start_(struct dvb_frontend* fe, struct dtv_fe_constellation* user, int max_num_samples);

static int stid135_tune(struct dvb_frontend* fe, bool re_tune,
		unsigned int mode_flags,
		unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int r;
	base_lock(state);
	r = stid135_tune_(fe, re_tune, mode_flags, delay, status);
	if(/*state->signal_info.has_carrier &&*/ p->constellation.num_samples>0) {
		int max_num_samples = state->signal_info.symbol_rate /5 ; //we spend max 500 ms on this
		if(max_num_samples > 1024)
			max_num_samples = 1024; //also set an upper limit which should be fast enough
		r = stid135_constellation_start_(fe, &p->constellation, max_num_samples);
	}
	base_unlock(state);
	return r;
}


static enum dvbfe_algo stid135_get_algo(struct dvb_frontend* fe)
{
	return DVBFE_ALGO_HW;
}

static int stid135_set_voltage(struct dvb_frontend* fe, enum fe_sec_voltage voltage)
{
	struct stv *state = fe->demodulator_priv;
	dprintk("demod=%d rf=%d mode=%d voltage=%d", state->nr, state->rf_in,  state->base->mode, voltage);
	//dump_stack();
	if (state->base->mode == 0) //legacy: 1 band per input @todo: fix this
	{
		if (voltage == SEC_VOLTAGE_18)
			state->rf_in |= 2;
		else
			state->rf_in &= ~2;
		return 0;
	}

	if(state->rf_in <0) {
		dprintk("rf_in not set yet\n");
		return -EPERM;
	}

	/*
		when multiple adapters use the same tuner, only one can chnage voltage, send tones, diseqc etc
		This is the tuner_owner. tuner_owner=-1 means no owner.
		We ignore any request to change voltage unless tuner_owner== state->nr or
	 */
	if(state->base->tuner_owner[state->rf_in] == -1) {
		//take ownership
		state->base->tuner_owner[state->rf_in] = state->nr;
		dprintk("demod=%d: took tuner ownership of rf_in=%d; use_count=%d\n", state->nr, state->rf_in,
						state->base->tuner_use_count[state->rf_in]);
	}


	if(state->base->tuner_owner[state->rf_in] != state->nr) {
		dprintk("demod=%d rf_in=%d DISALLOW voltage=%d tuner_use_count=%d owner=%d\n",
						state->nr, state->rf_in, voltage,
						state->base->tuner_use_count[state->rf_in],
						state->base->tuner_owner[state->rf_in]);
		return -EPERM;
	}
	if(state->base->tuner_use_count[state->rf_in] > 1 && voltage == SEC_VOLTAGE_OFF) {
		dprintk("demod=%d rf_in=%d IGNORE voltage=OFF tuner_use_count=%d owner=%d\n",
						state->nr, state->rf_in,
						state->base->tuner_use_count[state->rf_in],
						state->base->tuner_owner[state->rf_in]);
		return 0;
	}

	if (state->base->set_voltage) {//official driver does not use mutex
		base_lock(state); //DeepThought: this performs multiple i2c calls and needs to be protected
		dprintk("demod=%d rf_in=%d calling state->base->set_voltage i2c=%p voltage=%d tune_use_count=%d owner=%d\n",
						state->nr, state->rf_in,
						state->base->i2c, voltage,
						state->base->tuner_use_count[state->rf_in],
						state->base->tuner_owner[state->rf_in]);
		stid135_select_rf_in_(state, state->rf_in);
		state->base->set_voltage(state->base->i2c, voltage, state->rf_in);
		base_unlock(state);
		dprintk("demod=%d: set_voltage done: rf_in=%d; use_count=%d\n", state->nr, state->rf_in,
						state->base->tuner_use_count[state->rf_in]);

	}
	return 0;
}

static int stid135_set_tone(struct dvb_frontend* fe, enum fe_sec_tone_mode tone)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	dprintk("demod=%d rf=%d mode=%d tone=%d", state->nr, state->rf_in,  state->base->mode, tone);
	if(state->base->mode == 0) //mode always 1
	{
		if (tone == SEC_TONE_ON)
			state->rf_in |= 1;
		else
			state->rf_in &= ~1;

		return 0;
	}

	if(state->rf_in <0) {
		dprintk("dmod=d%d: rf_in not set yet\n", state->nr);
		return -EPERM;
	}

	/*
		when multiple adapters use the same tuner, only one can chnage voltage, send tones, diseqc etc
		This is the tuner_owner. tuner_owner=-1 means no owner.
		We ignore any request to change voltage unless tuner_owner== state->nr or
	 */
	if(state->base->tuner_owner[state->rf_in] == -1) {
		//take ownership
		state->base->tuner_owner[state->rf_in] = state->nr;
		dprintk("demod=%d: took tuner ownership of rf_in=%d; use_count=%d\n", state->nr, state->rf_in,
						state->base->tuner_use_count[state->rf_in]);
	}

	if(state->base->tuner_owner[state->rf_in] != state->nr ) {
		dprintk("demod=%d rf_in=%d DISALLOW tone=%d tuner_use_count=%d owner=%d\n",
						state->nr, state->rf_in, tone,
						state->base->tuner_use_count[state->rf_in],
						state->base->tuner_owner[state->rf_in]);
		return -EPERM;
	} else {
		base_lock(state);
		err |= stid135_select_rf_in_(state, state->rf_in);
		err = fe_stid135_set_22khz_cont(&state->base->ip, state->rf_in + 1, tone == SEC_TONE_ON);
		base_unlock(state);

		dprintk("demod=%d DISEQC[%d] tone=%d error=%d err=%d abort=%d", state->nr, state->rf_in+1,  tone, err,
						state->base->ip.handle_demod->Error,
						state->base->ip.handle_demod->Abort);

		if (err != FE_LLA_NO_ERROR)
			dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_22khz_cont error %d !\n", __func__, err);

		return err != FE_LLA_NO_ERROR ? -1 : 0;
	}
}

static int stid135_send_master_cmd(struct dvb_frontend* fe,
				 struct dvb_diseqc_master_cmd *cmd)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

#if 0
	if (state->base->mode == 0)
		return 0;
#endif
	dprintk("diseqc rf_in=%d\n", state->rf_in);

	if(state->rf_in <0) {
		dprintk("rf_in not set yet\n");
		return -EPERM;
	}


	/*
		when multiple adapters use the same tuner, only one can chnage voltage, send tones, diseqc etc
		This is the tuner_owner. tuner_owner=-1 means no owner.
		We ignore any request to change voltage unless tuner_owner== state->nr or
	 */
	if(state->base->tuner_owner[state->rf_in] == -1) {
		//take ownership
		state->base->tuner_owner[state->rf_in] = state->nr;
		dprintk("demod=%d: took tuner ownership of rf_in=%d; use_count=%d\n", state->nr, state->rf_in,
						state->base->tuner_use_count[state->rf_in]);
	}

	/*
		when two adapters use the same tuner, we ignore any request to change tone
		as long as multiple frontends use the same tuner
	 */

	if(state->base->tuner_owner[state->rf_in] != state->nr ) {
		dprintk("demod=%d rf_in=%d DISALLOW diseqc tuner_use_count=%d owner=%d\n",
						state->nr, state->rf_in,
						state->base->tuner_use_count[state->rf_in],
						state->base->tuner_owner[state->rf_in]);
		return -EPERM;
	} else {
		base_lock(state);
		err |= stid135_select_rf_in_(state, state->rf_in);
		err |= fe_stid135_diseqc_init(&state->base->ip, state->rf_in + 1, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_send(state, state->rf_in + 1, cmd->msg, cmd->msg_len);
		base_unlock(state);
		dprintk("demod=%d: diseqc sent: rf_in=%d; use_count=%d\n", state->nr, state->rf_in,
						state->base->tuner_use_count[state->rf_in]);

		if (err != FE_LLA_NO_ERROR)
			dev_err(&state->base->i2c->dev, "%s: fe_stid135_diseqc_send error %d !\n", __func__, err);

		return err != FE_LLA_NO_ERROR ? -1 : 0;
	}
}

static int stid135_recv_slave_reply(struct dvb_frontend* fe,
					struct dvb_diseqc_slave_reply *reply)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

#if 0
	if (state->base->mode == 0)
		return 0;
#endif

	base_lock(state);
	err = fe_stid135_diseqc_receive(&state->base->ip, reply->msg, &reply->msg_len);
	base_unlock(state);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_diseqc_receive error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_send_burst(struct dvb_frontend* fe, enum fe_sec_mini_cmd burst)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

	if (state->base->mode == 0)
		return 0;
	stid135_select_rf_in_(state, state->rf_in);
	dprintk("Not implemented!\n");
	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

/*Called when the only user which has opened the frontend in read-write mode
	exits*/
static int stid135_sleep(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params = &state->base->ip;

	if (state->base->mode == 0) //official driver never calls code below
		return 0;

	dev_dbg(&state->base->i2c->dev, "%s: tuner %d\n", __func__, state->rf_in);
	dprintk("sleep called");
	base_lock(state);
	BUG_ON((state->rf_in<0 || state->rf_in >= 4));
	BUG_ON(state->base->tuner_use_count[state->rf_in] < 0);

	if(state->rf_in >= 0 && state->rf_in_selected) {
		if(state->base->tuner_owner[state->rf_in] == state->nr) {
			state->base->tuner_owner[state->rf_in] = -1; //release ownership
			dprintk("demod=%d: released tuner ownership of rf_in=%d; use_count=%d\n", state->nr, state->rf_in,
							state->base->tuner_use_count[state->rf_in]);
		} else {
			dprintk("demod=%d: decreased tuner use_count of rf_in=%d; use_count=%d\n", state->nr, state->rf_in,
							state->base->tuner_use_count[state->rf_in]);
		}

		if(state->base->tuner_use_count[state->rf_in] > 0)
			state->base->tuner_use_count[state->rf_in]--;
		if(state->base->tuner_use_count[state->rf_in]==0) {
			dprintk("Calling TunerStandby 0\n");
			err = fe_stid135_set_22khz_cont(&state->base->ip, state->rf_in + 1, false);
			if(state->base->set_voltage) {
				state->base->set_voltage(state->base->i2c, SEC_VOLTAGE_OFF, state->rf_in);
			}
			err |= FE_STiD135_TunerStandby(p_params->handle_demod, state->rf_in + 1, 0);
		}
	}
	state->rf_in_selected = false;
	base_unlock(state);
	if (err != FE_LLA_NO_ERROR)
		dev_warn(&state->base->i2c->dev, "%s: STiD135 standby tuner %d failed!\n", __func__, state->rf_in);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_read_signal_strength(struct dvb_frontend* fe, u16 *strength)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*strength = 0;
	for (i=0; i < p->strength.len; i++) {
		BUG_ON(i <0 || i>= sizeof(p->strength.stat)/sizeof(p->strength.stat[0])); //triggered
		if (p->strength.stat[i].scale == FE_SCALE_RELATIVE)
			*strength = (u16)p->strength.stat[i].uvalue;
		else if (p->strength.stat[i].scale == FE_SCALE_DECIBEL)
			*strength = ((100000 + (s32)p->strength.stat[i].svalue)/1000) * 656;
	}

	return 0;
}

static int stid135_read_snr(struct dvb_frontend* fe, u16 *snr)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*snr = 0;
	for (i=0; i < p->cnr.len; i++) {
		BUG_ON(i <0 || i>= sizeof(p->cnr.stat)/sizeof(p->cnr.stat[0]));
		if (p->cnr.stat[i].scale == FE_SCALE_RELATIVE)
			*snr = (u16)p->cnr.stat[i].uvalue;
	}
	return 0;
}

static int stid135_read_ber(struct dvb_frontend* fe, u32 *ber)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*ber = 1;
	for (i=0; i < p->post_bit_error.len; i++) {
		BUG_ON(i <0 || i>= sizeof(p->post_bit_error.stat)/sizeof(p->post_bit_error.stat[0]));
		if ( p->post_bit_error.stat[0].scale == FE_SCALE_COUNTER )
			*ber = (u32)p->post_bit_error.stat[0].uvalue;
	}
	return 0;
}

static int stid135_read_ucblocks(struct dvb_frontend* fe, u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

static void spi_read(struct dvb_frontend* fe, struct ecp3_info *ecp3inf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;

	if (state->base->read_properties)
		state->base->read_properties(adapter,ecp3inf->reg, &(ecp3inf->data));
	return ;
}

static void spi_write(struct dvb_frontend* fe,struct ecp3_info *ecp3inf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;

	if (state->base->write_properties)
		state->base->write_properties(adapter,ecp3inf->reg, ecp3inf->data);
	return ;
}

static void eeprom_read(struct dvb_frontend* fe, struct eeprom_info *eepinf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;

	if (state->base->read_eeprom)
		state->base->read_eeprom(adapter,eepinf->reg, &(eepinf->data));
	return ;
}

static void eeprom_write(struct dvb_frontend* fe,struct eeprom_info *eepinf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;

	if (state->base->write_eeprom)
		state->base->write_eeprom(adapter,eepinf->reg, eepinf->data);
	return ;
}

static int stid135_read_temp(struct dvb_frontend* fe, s16 *temp)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

	base_lock(state);
	err = fe_stid135_get_soc_temperature(&state->base->ip, temp);
	base_unlock(state);

	if (err != FE_LLA_NO_ERROR)
		dev_warn(&state->base->i2c->dev, "%s: fe_stid135_get_soc_temperature error %d !\n", __func__, err);
	return 0;
}

//to be called with base locked
static int stid135_get_spectrum_scan_fft(struct dvb_frontend* fe, unsigned int *delay,  enum fe_status *status)
{
	int error = stid135_spectral_scan_start(fe);
	if(!error) {
		*status =  FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
		return 0;
	} else {
		dprintk("encountered error\n");
		*status =  FE_TIMEDOUT|FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
		return error;
	}

}


static int stid135_get_spectrum_scan_sweep(struct dvb_frontend* fe,
																						 unsigned int *delay,  enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct spectrum_scan_state* ss = &state->spectrum_scan_state;
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

	uint32_t frequency;
	uint32_t resolution =  (p->scan_resolution>0) ? p->scan_resolution : p->symbol_rate/1000; //in kHz
	uint32_t bandwidth =  (p->symbol_rate>0) ? p->symbol_rate : p->scan_resolution*1000; //in Hz
	ss->spectrum_len = (end_frequency - start_frequency + resolution-1)/resolution;
	ss->freq = kzalloc(ss->spectrum_len * (sizeof(ss->freq[0])), GFP_KERNEL);
	ss->spectrum = kzalloc(ss->spectrum_len * (sizeof(ss->spectrum[0])), GFP_KERNEL);
	if (!ss->freq || !ss->spectrum) {
		return  -ENOMEM;
	}
	ss->spectrum_present = true;

#ifdef TODO
	state->tuner_bw = stv091x_bandwidth(ROLLOFF_AUTO, bandwidth);
#endif
	dprintk("demod: %d: tuner:%d range=[%d,%d]kHz num_freq=%d resolution=%dkHz bw=%dkHz clock=%d\n", state->nr,
					state->rf_in,
					start_frequency, end_frequency,
					ss->spectrum_len, resolution, bandwidth/1000,
					pParams->master_clock);

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


	for (i = 0; i < ss->spectrum_len; i++) {
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
		ss->freq[i]= start_frequency +i*resolution;
		frequency = ss->freq[i];

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
		ss->spectrum[i] = pch_rf;
		if(error1) {
			dprintk("Failed: err=%d\n", error1);
			goto __onerror;
		}
		if(error)
			goto __onerror;

		state_sleep(state, 12);

	}

	*status =  FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	return 0;
 __onerror:

	dprintk("encountered error at %d/%d\n", i, ss->spectrum_len);
	*status =  FE_TIMEDOUT|FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	return 0;
}

/*
	stops the current frontend task
 */
static int stid135_stop_task(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	struct spectrum_scan_state* ss = &state->spectrum_scan_state;
	struct constellation_scan_state* cs = &state->constellation_scan_state;

	if(ss->freq)
		kfree(ss->freq);
	if(ss->candidates)
		kfree(ss->candidates);
	if(ss->spectrum)
		kfree(ss->spectrum);
	if(cs->samples)
		kfree(cs->samples);
	memset(ss, 0, sizeof(*ss));
	memset(cs, 0, sizeof(*cs));

	if(state && state->rf_in >=0 && state->rf_in_selected) {
		if(state->base->tuner_owner[state->rf_in] == state->nr) {
			state->base->tuner_owner[state->rf_in] = -1; //release ownership
			dprintk("demod=%d: released tuner ownership of rf_in=%d; use_count=%d\n", state->nr, state->rf_in,
							state->base->tuner_use_count[state->rf_in]);
		}
	}

	//dprintk("Freed memory\n");
	return 0;
}

static int stid135_spectrum_start(struct dvb_frontend* fe,
																	struct dtv_fe_spectrum* s,
																	unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct spectrum_scan_state* ss = &state->spectrum_scan_state;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int ret=0;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	stid135_stop_task(fe);


	if(p->rf_in < 0 || !p->rf_in_valid)  {
		p->rf_in = state->rf_in_selected ? state->rf_in : state->fe.ops.info.default_rf_input;
		p->rf_in_valid = true;
		dprintk("Set rf_in to %d; rf_in_selected=%d  state->rf_in=%d default_rf_in=%d\n",
						p->rf_in, state->rf_in_selected, state->rf_in, state->fe.ops.info.default_rf_input);
	}

	if(err) {
		dprintk("Could not set rfpath error=%d\n", err);
	}
	s->scale =  FE_SCALE_DECIBEL; //in units of 0.001dB

	base_lock(state);

	switch(s->spectrum_method) {
	case SPECTRUM_METHOD_SWEEP:
	default:
		ret = stid135_get_spectrum_scan_sweep(fe,  delay,  status);
		s->num_freq = ss->spectrum_len;
		break;
	case SPECTRUM_METHOD_FFT:
		ret = stid135_get_spectrum_scan_fft(fe, delay, status);
		s->num_freq = ss->spectrum_len;
		break;
	}

	base_unlock(state);
	return -1;
}

int stid135_spectrum_get(struct dvb_frontend* fe, struct dtv_fe_spectrum* user)
{
	struct stv *state = fe->demodulator_priv;
	int error=0;
	base_lock(state);
	if (user->num_freq> state->spectrum_scan_state.spectrum_len)
		user->num_freq = state->spectrum_scan_state.spectrum_len;
	if (user->num_candidates > state->spectrum_scan_state.num_candidates)
		user->num_candidates = state->spectrum_scan_state.num_candidates;
	if(state->spectrum_scan_state.freq && state->spectrum_scan_state.spectrum) {
		if(user->freq && user->num_freq > 0 &&
			 copy_to_user((void __user*) user->freq, state->spectrum_scan_state.freq, user->num_freq * sizeof(__u32))) {
			error = -EFAULT;
		}
		if(user->rf_level && user->num_freq > 0 &&
			 copy_to_user((void __user*) user->rf_level, state->spectrum_scan_state.spectrum, user->num_freq * sizeof(__s32))) {
			error = -EFAULT;
		}
		if(user->candidates && user->num_candidates >0 &&
			 copy_to_user((void __user*) user->candidates,
										 state->spectrum_scan_state.candidates,
										 user->num_candidates * sizeof(struct spectral_peak_t))) {
			error = -EFAULT;
		}
	} else
		error = -EFAULT;
	base_unlock(state);
	//stid135_stop_task(fe);
	return error;
}


fe_lla_error_t FE_STiD135_GetCarrierFrequency(struct stv* state, u32 MasterClock, s32* carrierFrequency_p);
/*
	init = 1: start the scan at the search range specified by the user
	init = 0: start the scan just beyond the last found frequency
 */

static int stid135_scan_sat(struct dvb_frontend* fe, bool init,
														unsigned int *delay,  enum fe_status *status)
{
 	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	//struct fe_stid135_internal_param * pParams = &state->base->ip;
	//int asperity;
	//s32 carrier_frequency;
	bool retune=true;
	enum fe_status old;
	unsigned int mode_flags=0; //not used
	int ret=0;
	bool found=false;
	s32 minfreq=0;

	if(init) {
		if(state->spectrum_scan_state.scan_in_progress) {
			stid135_stop_task(fe); //cleanup older scan
		}

		if(p->rf_in < 0 || !p->rf_in_valid)  {
			p->rf_in = state->rf_in_selected ? state->rf_in : state->fe.ops.info.default_rf_input;
			p->rf_in_valid = true;
			dprintk("Set rf_in to %d; rf_in_selected=%d  state->rf_in=%d default_rf_in=%d\n",
							p->rf_in, state->rf_in_selected, state->rf_in, state->fe.ops.info.default_rf_input);
		}

		base_lock(state);
		ret = stid135_spectral_scan_start(fe);
		base_unlock(state);

		if(ret<0) {
			dprintk("Could not start spectral scan\n");
			return -1; //
		}
	} else {
		if(!state->spectrum_scan_state.scan_in_progress) {
			dprintk("Error: Called with init==false, but scan was  not yet started\n");

			base_lock(state);
			ret = stid135_spectral_scan_start(fe);
			base_unlock(state);

			if(ret<0) {
				dprintk("Could not start spectral scan\n");
				return -1; //
			}
		}

		minfreq= state->spectrum_scan_state.next_frequency;
		dprintk("SCAN SAT next_freq=%dkHz\n", state->spectrum_scan_state.next_frequency);
	}
	dprintk("SCAN SAT delsys=%d\n", p->delivery_system);
	while(!found) {
		if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
			dprintk("exiting on should stop\n");
			break;
		}

		*status = 0;
		base_lock(state);
		ret=stid135_spectral_scan_next(fe,  &p->frequency, &p->symbol_rate);
		base_unlock(state);

		if(ret<0) {
			dprintk("reached end of scan range\n");
			*status =  FE_TIMEDOUT;
			return error;
		}


		if (p->frequency < p->scan_start_frequency) {
			dprintk("implementation error: %d < %d\n",p->frequency, p->scan_start_frequency);
		}
		if (p->frequency > p->scan_end_frequency) {
			dprintk("implementation error: %d < %d\n",p->frequency, p->scan_end_frequency);
		}
		if(p->frequency < minfreq) {
			dprintk("Next freq %dkHz still in current tp (ends at %dkHz)\n", p->frequency, minfreq);
			continue;
		}

		p->algorithm = ALGORITHM_BLIND; //ALGORITHM_SEARCH_NEXT;

		p->scan_fft_size = p->scan_fft_size==0 ? 512: p->scan_fft_size;
		/*
			A large search range seems to be ok in all cases
		*/
		p->search_range = p->search_range ==0 ? (p->scan_fft_size * p->scan_resolution*1000): p->search_range;

		/*Although undocumented, a low symbol rate must be set to detect signals with SR below 1 MS/s
			and a low value does not seem to be essential for high SR transponders
		*/
#if 1
		p->symbol_rate = p->symbol_rate==0? 1000000: p->symbol_rate;
		p->stream_id = NO_STREAM_ID_FILTER;
#else
		p->symbol_rate = 1000000; //otherwise it may be set too low based on last transponder
		p->stream_id = NO_STREAM_ID_FILTER;
#endif
		dprintk("FREQ=%d search_range=%dkHz fft=%d res=%dkHz srate=%dkS/s\n",
						p->frequency, p->search_range/1000,
						p->scan_fft_size, p->scan_resolution, p->symbol_rate/1000);

		base_lock(state);
		ret = stid135_tune_(fe, retune, mode_flags, delay, status);
		base_unlock(state);

		old = *status;
		{
			state->base->ip.handle_demod->Error = FE_LLA_NO_ERROR;

			base_lock(state);
			fe_stid135_get_signal_info(state);
			base_unlock(state);

			memcpy(p->isi_bitset, state->signal_info.isi_list.isi_bitset, sizeof(p->isi_bitset));
			memcpy(p->matypes, state->signal_info.isi_list.matypes, sizeof(p->matypes));
			p->num_matypes = state->signal_info.isi_list.num_matypes;
			p->matype_val =  state->signal_info.matype;
			p->matype_valid = true;
			p->frequency = state->signal_info.frequency;
			p->symbol_rate = state->signal_info.symbol_rate;
		}
		found = !(*status & FE_TIMEDOUT) && (*status &FE_HAS_LOCK);
		if(found) {
			state->spectrum_scan_state.next_frequency = p->frequency + (p->symbol_rate*135)/200000;
			dprintk("BLINDSCAN: GOOD freq=%dkHz SR=%d kS/s returned status=%d next=%dkHz\n", p->frequency, p->symbol_rate/1000, *status,
							state->spectrum_scan_state.next_frequency /1000);
			state->spectrum_scan_state.num_good++;
		}
		else {
			dprintk("BLINDSCAN: BAD freq=%dkHz SR=%d kS/s returned status=%d\n", p->frequency, p->symbol_rate/1000, *status);
			state->spectrum_scan_state.num_bad++;
		}
	}

	return ret;
}


static int stid135_constellation_start_(struct dvb_frontend* fe, struct dtv_fe_constellation* user, int max_num_samples)
{
	struct stv *state = fe->demodulator_priv;
	struct constellation_scan_state* cs = &state->constellation_scan_state;
	struct fe_stid135_internal_param * pParams = &state->base->ip;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	int num_samples = user->num_samples;
	if(num_samples > max_num_samples)
		num_samples = max_num_samples;
	vprintk("demod: %d: constellation samples=%d/%d constel_select=%d\n", state->nr, user->num_samples,
					max_num_samples, (int)user->constel_select);
	if(num_samples ==0) {
		return -EINVAL;
	}
	if(cs->samples_len != num_samples) {
		if(cs->samples)
			kfree(cs->samples);
		cs->samples_len = num_samples;
		cs->samples = kzalloc(cs->samples_len * (sizeof(cs->samples[0])), GFP_KERNEL);
		if (!cs->samples) {
			return  -ENOMEM;
		}
	}

	cs->constel_select =  user->constel_select;
	cs->num_samples = 0;

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_IQCONST_CONSTEL_SELECT(state->nr+1),
												cs->constel_select);
	for (cs->num_samples = 0; cs->num_samples < cs->samples_len; ++cs->num_samples) {
		if ((cs->num_samples% 20==19) &&  (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			dprintk("exiting on should stop\n");
			break;
		}
		error |= ChipGetRegisters(pParams->handle_demod, REG_RC8CODEW_DVBSX_DEMOD_ISYMB(state->nr+1), 2);
		cs->samples[cs->num_samples].imag = ChipGetFieldImage(pParams->handle_demod,
																													FLD_FC8CODEW_DVBSX_DEMOD_ISYMB_I_SYMBOL(state->nr+1));
		cs->samples[cs->num_samples].real = ChipGetFieldImage(pParams->handle_demod,
																													FLD_FC8CODEW_DVBSX_DEMOD_QSYMB_Q_SYMBOL(state->nr+1));
	}
	vprintk("demod: %d: constellation retrieved samples=%d\n", state->nr, cs->num_samples);


	return 0;
}

static int stid135_constellation_get(struct dvb_frontend* fe, struct dtv_fe_constellation* user)
{
	struct stv *state = fe->demodulator_priv;
	struct constellation_scan_state* cs = &state->constellation_scan_state;
	int error = 0;
	base_lock(state);
	if(user->num_samples > cs->num_samples)
		user->num_samples = cs->num_samples;
	if(cs->samples) {
		if (copy_to_user((void __user*) user->samples, cs->samples,
										 user->num_samples * sizeof(cs->samples[0]))) {
			error = -EFAULT;
		}
	}
	else
		error = -EFAULT;
	base_unlock(state);
	return error;
}


static struct dvb_frontend_ops stid135_ops = {
	.delsys = { SYS_DVBS, SYS_DVBS2, SYS_DVBS2X, SYS_DSS, SYS_AUTO },
	.info = {
		.name			= "STiD135 Multistandard",
		.frequency_min_hz	 = 850 * MHz,
		.frequency_max_hz		= 2150 * MHz,
		.symbol_rate_min	= 100000,
		.symbol_rate_max	= 520000000,
		.caps			= FE_CAN_INVERSION_AUTO |
						FE_CAN_FEC_AUTO       |
						FE_CAN_QPSK           |
						FE_CAN_2G_MODULATION  |
		        FE_CAN_MULTISTREAM,
		.extended_caps          = FE_CAN_SPECTRUM_SWEEP	|FE_CAN_SPECTRUM_FFT |
		FE_CAN_IQ | FE_CAN_BLINDSEARCH,
		.supports_neumo = true,
		.default_rf_input = -1, //means: use adapter_no
		.num_rf_inputs = 0,
		.rf_inputs = { 0}
	},
	.init				= stid135_init,
	.sleep				= stid135_sleep,
	.release                        = stid135_release,
	.get_frontend_algo              = stid135_get_algo,
	//.get_frontend                   = stid135_get_frontend,
	.tune                           = stid135_tune,
	.set_rf_input			= stid135_set_rf_input,
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
	.stop_task =  stid135_stop_task,
	.scan =  stid135_scan_sat,
	.spectrum_start		= stid135_spectrum_start,
	.spectrum_get		= stid135_spectrum_get,
#if 0
	.constellation_start	= stid135_constellation_start,
#endif
	.constellation_get	= stid135_constellation_get,

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

static inline int num_cards(void)
{
	struct stv_base *p;
	int ret=0;
	list_for_each_entry(p, &stvlist, stvlist)
		ret++;
	return ret;
}

/*DT: called with  nr=adapter=0...7 and rf_in = nr/2=0...3
	state->base is created exactly once and is shared
	between the 8 demods; provides access to the i2c hardware and such
*/
struct dvb_frontend* stid135_attach(struct i2c_adapter *i2c,
																		struct stid135_cfg *cfg,
																		int nr, int rf_in)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	int i;
	struct stv *state;
	struct stv_base *base=NULL;
	extern struct stv_base *proc_base;
	state = kzalloc(sizeof(struct stv), GFP_KERNEL);

	if (!state)
		return NULL;

	base = match_base(i2c, cfg->adr);
	if (base) {
		base->count++;
		state->base = base;
		dprintk("ATTACH DUP nr=%d rf_in=%d base=%p count=%d i2c_addr=0x%x i2c=%s\n", nr, rf_in, base, base->count,
						cfg->adr,  dev_name(&i2c->dev));
	} else {
		base = kzalloc(sizeof(struct stv_base), GFP_KERNEL);
		if (!base)
			goto fail;

		for(i=0; i < sizeof(base->tuner_owner)/sizeof(base->tuner_owner[0]); ++i)
			base->tuner_owner[i] = -1;
		base->card_no = num_cards();
		base->i2c = i2c;
		base->adr = cfg->adr;
		base->count = 1;
		dprintk("ATTACH NEW nr=%d rf_in=%d base=%p count=%d i2c_addr=0x%x i2c=%s\n", nr, rf_in, base, base->count, cfg->adr,  dev_name(&i2c->dev));
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

		mutex_init(&base->lock.m);
		proc_base = base;
		state->base = base;
		if (stid135_probe(state) < 0) {
			dev_warn(&i2c->dev, "No demod found at adr %02X on %s\n",
				 cfg->adr, dev_name(&i2c->dev));
			kfree(base);
			goto fail;
		}

		list_add(&base->stvlist, &stvlist);
	}
#if 1 //official driver has none of this (it is done elsewhere)
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
	BUG_ON(sizeof(state->mc_flt)/sizeof(state->mc_flt[0])!=NB_SAT_MODCOD);
	for(i=0;i<NB_SAT_MODCOD;i++) {
		state->mc_flt[i].forbidden = FALSE;
	}
	error = fe_stid135_apply_custom_qef_for_modcod_filter(state, NULL);
	if(error)
		dprintk("fe_stid135_apply_custom_qef_for_modcod_filter error=%d\n", error);
#endif
	state->fe.ops               = stid135_ops;
	state->fe.ops.info.num_rf_inputs = cfg->num_rf_inputs;
	memcpy(&state->fe.ops.info.rf_inputs[0], &cfg->rf_inputs[0], state->fe.ops.info.num_rf_inputs);
	if (nr >=4)
		state->fe.ops.info.extended_caps &= ~FE_CAN_SPECTRUM_FFT;
	state->fe.demodulator_priv  = state;
	state->nr = nr;
	//state->nr = nr < 4 ? 2*nr  : 2*(nr-4)+1;
	state->newTP = false;
	state->bit_rate  = 0;
	state->loops = 15;

	state->modcode_filter = false;

	if (rfsource > 0 && rfsource < 5)
		rf_in = rfsource - 1;
	state->rf_in = base->mode ? rf_in : 0;

	if (base->mode == 2)
		state->rf_in = 3;
	state->fe.ops.info.default_rf_input = rf_in;
#if 0
	if(cfg->ts_mode == TS_2PAR) {
		//hack to detect tbs6303x
		state->fe.ops.info.rf_inputs[0] = 0;
		state->fe.ops.info.rf_inputs[1] = 3;
		state->fe.ops.info.num_rf_inputs = 2;
	}
#endif
	dev_info(&i2c->dev, "%s demod found at adr %02X on %s\n",
					 state->fe.ops.info.name, cfg->adr, dev_name(&i2c->dev));

	return &state->fe;

fail:
	kfree(state);
	return NULL;
}

static int stid135_module_init(void)
{
	fe_stid135_modcod_flt_reg_init();
	return 0;
}

static void stid135_module_exit(void)
{
	dprintk("module exit\n");
	return;
}


EXPORT_SYMBOL_GPL(stid135_attach);

module_init(stid135_module_init);
module_exit(stid135_module_exit);
MODULE_DESCRIPTION("STiD135 driver");
MODULE_AUTHOR("Crazycat (modified by DeepThought)");
MODULE_LICENSE("GPL");
