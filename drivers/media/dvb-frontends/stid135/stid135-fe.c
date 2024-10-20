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
LIST_HEAD(stv_demod_list);
LIST_HEAD(stv_chip_list);
LIST_HEAD(stv_card_list);

static int mode = 1;
module_param(mode, int, 0444);
MODULE_PARM_DESC(mode,
		"Multi-switch mode: 0=quattro/quad 1=normal direct connection; 2=unicable/jess");

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

char* reservation_mode_str(enum fe_reservation_mode mode) {
	switch(mode) {
	case FE_RESERVATION_MODE_MASTER_OR_SLAVE:
		return "MASTER/SLAVE";
	case FE_RESERVATION_MODE_MASTER:
		return "MASTER";
		break;
	case FE_RESERVATION_MODE_SLAVE:
	default:
		return "SLAVE";
	}
}

char* reservation_result_str(enum fe_ioctl_result result) {
	switch(result) {
	case FE_RESERVATION_MASTER:
		return "MASTER";
	case FE_RESERVATION_SLAVE:
		return "SLAVE";
		break;
	case FE_RESERVATION_RETRY:
		return "RETRY";
		break;
	case FE_RESERVATION_RELEASED:
		return "RELEASED";
		break;
	case FE_RESERVATION_FAILED:
	default:
		return "FAILED";
	}
}

char* tone_str(enum fe_sec_tone_mode tone) {
	switch(tone) {
	case SEC_TONE_ON:
		return "on";
	case SEC_TONE_OFF:
	default:
		return "off";
	}
}

char* voltage_str(enum fe_sec_voltage voltage) {
	switch(voltage) {
	case SEC_VOLTAGE_OFF:
	default:
		return "off";
	case SEC_VOLTAGE_13:
		return "13V";
	case SEC_VOLTAGE_18:
		return "18V";
	}
}

int active_rf_in_no(struct stv* state) {
	return state && 	state->active_tuner && 	state->active_tuner->active_rf_in
		? state->active_tuner->active_rf_in->rf_in_no : -1;
}

struct stv_rf_in_t* active_rf_in(struct stv* state) {
	return state && 	state->active_tuner && 	state->active_tuner->active_rf_in
		? state->active_tuner->active_rf_in : NULL;
}

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

static inline enum fe_code_rate dvb_fec(struct stv* state, enum fe_delivery_system delsys) {
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
			return FEC_1_2;
			break;
		case FE_SAT_PR_2_3:
			return FEC_2_3;
			break;
		case FE_SAT_PR_3_4:
			return FEC_3_4;
			break;
		case FE_SAT_PR_5_6:
			return FEC_5_6;
			break;
		case FE_SAT_PR_6_7:
			return FEC_6_7;
			break;
		case FE_SAT_PR_7_8:
			return FEC_7_8;
			break;
		default:
			return FEC_NONE;
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
	struct stv_chip_t     *base = (struct stv_chip_t *)pI2CHost;
	struct i2c_msg msg = {.addr = ChipAddress>>1, .flags = 0,
						.buf = Data, .len = NbData};
	int ret;


	if (!base) return I2C_ERR_HANDLE;

	if (mode == I2C_READ)
		msg.flags = I2C_M_RD;

	ret = i2c_transfer(base->i2c, &msg, 1);
	if(ret<0) {
		dprintk("BUG: i2c_transfer returned %d\n", ret);
	}
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
	SAT_VGLNA_InitParams_t pVGLNAInit;
	SAT_VGLNA_InitParams_t pVGLNAInit1;
	SAT_VGLNA_InitParams_t pVGLNAInit2;
	SAT_VGLNA_InitParams_t pVGLNAInit3;
	//end
	dev_warn(&state->chip->i2c->dev, "%s\n", FE_STiD135_GetRevision());

	strcpy(init_params.demod_name,"STiD135");
	init_params.pI2CHost		=	state->chip;
	init_params.demod_i2c_adr   	=	state->chip->adr ? state->chip->adr<<1 : 0xd0;
	init_params.demod_ref_clk  	= 	state->chip->extclk ? state->chip->extclk : 27;
	init_params.internal_dcdc	=	FALSE;
	init_params.internal_ldo	=	TRUE; // LDO supply is internal on Oxford valid board
	init_params.rf_input_type	=	0xF; // Single ended RF input on Oxford valid board rev2
	init_params.roll_off		=  	FE_SAT_35; // NYQUIST Filter value (used for DVBS1/DSS, DVBS2 is automatic)
	init_params.tuner_iq_inversion	=	FE_SAT_IQ_NORMAL;
	err = fe_stid135_init(&init_params, &state->chip->ip);

	init_params.ts_nosync		=	ts_nosync;
	init_params.mis_fix		= mis;

	if (err != FE_LLA_NO_ERROR) {
		dprintk("fe_stid135_init error %d !\n", err);
		return -EINVAL;
	}

	p_params = &state->chip->ip;
	vprintk("here state=%p\n", state);
	vprintk("here state->chip=%p\n", state->chip);
	vprintk("here state->chip=%p\n", state->chip);
	err = fe_stid135_get_cut_id(&state->chip->ip, &cut_id);
	switch(cut_id)
	{
	case STID135_CUT1_0:
		dev_warn(&state->chip->i2c->dev, "%s: cut 1.0\n", __func__);
		break;
	case STID135_CUT1_1:
		dev_warn(&state->chip->i2c->dev, "%s: cut 1.1\n", __func__);
		break;
	case STID135_CUT1_X:
		dev_warn(&state->chip->i2c->dev, "%s: cut 1.x\n", __func__);
		break;
	case STID135_CUT2_0:
		dev_warn(&state->chip->i2c->dev, "%s: cut 2.0 \n", __func__);
		break;
	case STID135_CUT2_1:
		dev_warn(&state->chip->i2c->dev, "%s: cut 2.1 \n", __func__);
		break;
	case STID135_CUT2_X_UNFUSED:
		dev_warn(&state->chip->i2c->dev, "%s: cut 2.x \n", __func__);
		break;
	default:
		dev_warn(&state->chip->i2c->dev, "%s: cut ? \n", __func__);
		return -EINVAL;
	}
	if (state->chip->ts_mode == TS_STFE) { //DT: This code is called
		dev_warn(&state->chip->i2c->dev, "%s: 8xTS to STFE mode init.\n", __func__);
		// note that  FE_SAT_DEMOD_1 is not used in the code in the following call for this ts_mode
		err |= fe_stid135_set_ts_parallel_serial(&state->chip->ip, FE_SAT_DEMOD_1, FE_TS_PARALLEL_ON_TSOUT_0);
		err |= fe_stid135_enable_stfe(&state->chip->ip,FE_STFE_OUTPUT0);
		err |= fe_stid135_set_stfe(&state->chip->ip, FE_STFE_TAGGING_MERGING_MODE, FE_STFE_INPUT1 |
						FE_STFE_INPUT2 |FE_STFE_INPUT3 |FE_STFE_INPUT4| FE_STFE_INPUT5 |
						FE_STFE_INPUT6 |FE_STFE_INPUT7 |FE_STFE_INPUT8 ,FE_STFE_OUTPUT0, 0xDE);
	} else if (state->chip->ts_mode == TS_8SER) { //DT: This code is not called
		dev_warn(&state->chip->i2c->dev, "%s: 8xTS serial mode init.\n", __func__);
		for(i=0;i<8;i++) {
			err |= fe_stid135_set_ts_parallel_serial(&state->chip->ip, i+1, FE_TS_SERIAL_CONT_CLOCK);
			//err |= fe_stid135_set_maxllr_rate(state, i+1, 90);
		}
	} else { //DT: This code is called on TBS6903X?
		dev_warn(&state->chip->i2c->dev, "%s: 2xTS parallel mode init.\n", __func__);
		err |= fe_stid135_set_ts_parallel_serial(&state->chip->ip, FE_SAT_DEMOD_3, FE_TS_PARALLEL_PUNCT_CLOCK);
		err |= fe_stid135_set_ts_parallel_serial(&state->chip->ip, FE_SAT_DEMOD_1, FE_TS_PARALLEL_PUNCT_CLOCK);
	}

	if (state->chip->multiswitch_mode == 0) {
		dev_warn(&state->chip->i2c->dev, "%s: multiswitch mode init.\n", __func__);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER1);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER2);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER3);
		err |= fe_stid135_tuner_enable(p_params->handle_demod, AFE_TUNER4);
		err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER1, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER3, FE_SAT_DISEQC_2_3_PWM);
		if(state->chip->control_22k){ //202405 dtcheck; also needed for tbs6909x?
			err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER2, FE_SAT_22KHZ_Continues);
			err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER4, FE_SAT_22KHZ_Continues);
		}
		else{
			err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER2, FE_SAT_DISEQC_2_3_PWM);
			err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER4, FE_SAT_DISEQC_2_3_PWM);
		}
	} else {
		err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER1, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER2, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER3, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_init(&state->chip->ip,AFE_TUNER4, FE_SAT_DISEQC_2_3_PWM);
	}
///////////////////*stvvglna*////////////////////////
	if(state->chip->vglna) { //for 6909x v2 version
		dev_warn(&state->chip->i2c->dev, "%s:Init STVVGLNA \n", __func__);
	VglnaIdString = "STVVGLNA";
	pVGLNAInit.Chip = &VGLNAChip;

	pVGLNAInit.Chip->pI2CHost	  =	state->chip;
	pVGLNAInit.Chip->RepeaterHost = NULL;
	pVGLNAInit.Chip->Repeater     = FALSE;
	pVGLNAInit.Chip->I2cAddr      = 0xc8;
	pVGLNAInit.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit, &state->chip->ip.vglna_handles[0]);

	stvvglna_set_standby(state->chip->ip.vglna_handles[0], vglna_mode);
	dev_warn(&state->chip->i2c->dev, "Initialized STVVGLNA 0 device\n");

	VglnaIdString = "STVVGLNA1";
	pVGLNAInit1.Chip = &VGLNAChip;

	pVGLNAInit1.Chip->pI2CHost	  =	state->chip;
	pVGLNAInit1.Chip->RepeaterHost = NULL;
	pVGLNAInit1.Chip->Repeater     = FALSE;
	pVGLNAInit1.Chip->I2cAddr      = 0xce;
	pVGLNAInit1.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit1.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit1, &state->chip->ip.vglna_handles[1]);
	stvvglna_set_standby(state->chip->ip.vglna_handles[1], vglna_mode);
	dev_warn(&state->chip->i2c->dev, "Initialized STVVGLNA 1 device\n");

	VglnaIdString = "STVVGLNA2";
	pVGLNAInit2.Chip = &VGLNAChip;
	pVGLNAInit2.Chip->pI2CHost	  =	state->chip;
	pVGLNAInit2.Chip->RepeaterHost = NULL;
	pVGLNAInit2.Chip->Repeater     = FALSE;
	pVGLNAInit2.Chip->I2cAddr      = 0xcc;
	pVGLNAInit2.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit2.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit2, &state->chip->ip.vglna_handles[2]);
	stvvglna_set_standby(state->chip->ip.vglna_handles[2], vglna_mode);
	dev_warn(&state->chip->i2c->dev, "Initialized STVVGLNA 2 device\n");

	VglnaIdString = "STVVGLNA3";
	pVGLNAInit3.Chip = &VGLNAChip;
	pVGLNAInit3.Chip->pI2CHost	  =	state->chip;
	pVGLNAInit3.Chip->RepeaterHost = NULL;
	pVGLNAInit3.Chip->Repeater     = FALSE;
	pVGLNAInit3.Chip->I2cAddr      = 0xca;
	pVGLNAInit3.NbDefVal = STVVGLNA_NBREGS;
	strcpy((char *)pVGLNAInit3.Chip->Name, VglnaIdString);
	stvvglna_init(&pVGLNAInit3, &state->chip->ip.vglna_handles[3]);

	stvvglna_set_standby(state->chip->ip.vglna_handles[3], vglna_mode);
	dev_warn(&state->chip->i2c->dev, "Initialized STVVGLNA 3 device\n");
	}
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->chip->i2c->dev, "%s: setup error %d !\n", __func__, err);
	return err != FE_LLA_NO_ERROR ? -1 : 0;
}
/*
	Try to reserve a specific rf_in and tuner_combination, while releasing
	or updating existing reserved resources

	possible state at input for this frontend
	-nothing reserved yet
	-keep tuner and rf_in but change voltage/tone/diseqc
	-keep tuner but switch to different rf_in
	-change both tuner and rf_in
 */

/*
	Reserves rf_in and tuner and release any existing rf_in and tuner reservation
	by this particular demod. The result is one of
	-reservation succeeded and this demod is now a master and should power on tuner and configure lnb
	-reservation succeeded and this demod is now using the tuner and rf_in in slave mode, and both are ready for use
	-reservation failed temporarily because some other demod is still configuring the tuner/rf_in, but
	 the configuration in progress is otherwise correct; the caller should retry the reservation later
	-reservation failed because of resource conflicts; retrying is useless

	Returns -1 in case of error
	bool will_be_master = true;
	bool fatal_error=false;
	bool temporary_error =false;

	In case ic=NULL, this call is used to unreserve a resource

 */
static enum fe_ioctl_result reserve_tuner_and_rf_in_(struct stv* state, struct fe_rf_input_control* ic) {
	struct stv_chip_t* chip = state->chip;
	struct stv_card_t* card = chip->card;
	int chip_no = chip->chip_no;
	int tuner_no = ic ? ic->rf_in : -1;
	int rf_in = ic ? ic->rf_in : -1;
	if(ic && (tuner_no <0 || tuner_no >= sizeof(chip->tuners)/sizeof(chip->tuners[0]))) {
		state_dprintk("BUG: invalid call: tune_no=%d\n", tuner_no);
		return FE_RESERVATION_FAILED;
	}
	if(!card_is_locked_by_state(state)) {
		state_dprintk("BUG: called without locking card\n");
		dump_stack();
	}
	//rf_in to reserve (could be on other chip)

	struct stv_tuner_t* new_tuner = ic? &chip->tuners[tuner_no] : NULL;
	struct stv_tuner_t* old_tuner = state->active_tuner;

	struct stv_rf_in_t* new_rf_in = (rf_in >=0 && rf_in < 4) ? &card->rf_ins[rf_in] : NULL;
	struct stv_rf_in_t* old_rf_in = (old_tuner && old_tuner->active_rf_in) ? old_tuner->active_rf_in: NULL;

	//output decisions
	bool unicable_mode = ic && ic->unicable_mode;
	bool must_be_master = ic && (ic->mode == FE_RESERVATION_MODE_MASTER);
	bool will_be_master = ic && (ic->mode == FE_RESERVATION_MODE_MASTER ||
															 ic->mode == FE_RESERVATION_MODE_MASTER_OR_SLAVE);
	bool same_tuner = new_tuner == old_tuner;
	bool same_rf_in = new_rf_in == old_rf_in;
	//Check for bugs
	if(old_tuner && ic && (old_tuner->reservation.owner != ic->owner)) {
		state_dprintk("BUG: we should own old_tuner: %d !=%d\n", old_tuner->reservation.owner, ic->owner);
		goto fatal_;
	}

	if(old_rf_in && ic && (old_rf_in->reservation.owner != ic->owner)) {
		state_dprintk("BUG: we should own old_tuner: %d !=%d\n", old_tuner->reservation.owner, ic->owner);
		goto fatal_;
	}

	//Check that no other application is using the resources that we need
	if(new_tuner && new_tuner->reservation.use_count>0 && ic && new_tuner->reservation.owner != ic->owner) {
		state_dprintk("tuner in use by other owner=%d use_count=%d\n", new_tuner->reservation.owner,
									new_tuner->reservation.use_count);
		goto fatal_;
	}

	if(new_rf_in && new_rf_in->reservation.use_count>0 && ic && new_rf_in->reservation.owner != ic->owner) {
		state_dprintk("rf_in in use by other owner %d\n", new_tuner->reservation.owner);
		goto fatal_;
	}

	if(new_tuner) {
		//check if the tuner that we need can be modified to what we need
		if(new_tuner->reservation.use_count == 1 && same_tuner) {
			//We have already reserved the tuner
		} else  if(new_tuner->reservation.use_count > 0) {
			//Our application owns the tuner; we can use it, but not control it
			will_be_master = false;
			if(must_be_master) {
				if(ic->config_id > new_tuner->reservation.config_id) {
					state_dprintk("can not yet become master because of other users: config_id=%d -> %d; use_count=%d\n",
												new_tuner->reservation.config_id, ic->config_id,
												new_tuner->reservation.use_count);
					goto tempfail_;
				} else {
					state_dprintk("can NEVER become master because of other users: config_id=%d -> %d; use_count=%d\n",
												new_tuner->reservation.config_id, ic->config_id,
												new_tuner->reservation.use_count);
				}
				goto fatal_;
			} else {
				if(ic->config_id >  new_tuner->reservation.config_id) {
					state_dprintk("cannot yet become slave: waiting for master config_id=%d -> %d; use_count=%d\n",
												new_tuner->reservation.config_id, ic->config_id,
												new_tuner->reservation.use_count);
					goto tempfail_;
				} else if(ic->config_id <  new_tuner->reservation.config_id)  {
					state_dprintk("can NEVER become slave because of other users: config_id=%d -> %d; use_count=%d\n",
												new_tuner->reservation.config_id, ic->config_id,
												new_tuner->reservation.use_count);
					goto fatal_;
				}
			}
		}
	}

	if(new_rf_in) {
		//check if the rf_in that we need can be modified to what we need
		if(new_rf_in->reservation.use_count == 1 && same_rf_in) {
			//We have already reserved the rf_in
		} else  if(new_tuner->reservation.use_count > 0) {
			//Our application owns the tuner; we can use it, but not control it
			will_be_master = false;
			if(must_be_master) {
				state_dprintk("cannot become master because of other users config_id=%d -> %d; use_count=%d\n",
											new_rf_in->reservation.config_id, ic->config_id,
										new_rf_in->reservation.use_count);
				goto fatal_;
			} else {
				if(ic->config_id >  new_tuner->reservation.config_id) {
					state_dprintk("cannot yet become slave: waiting for master config_id=%d -> %d; use_count=%d\n",
												new_rf_in->reservation.config_id, ic->config_id,
												new_rf_in->reservation.use_count);
					goto tempfail_;
				} else if(ic->config_id <  new_rf_in->reservation.config_id) {
					if(!state->legacy_rf_in) {
						state_dprintk("can NEVER become slave because of other users: config_id=%d -> %d; use_count=%d\n",
													new_rf_in->reservation.config_id, ic->config_id,
													new_rf_in->reservation.use_count);
						goto fatal_;
					}
				}
			}
			if(new_rf_in->reservation.config_id != ic->config_id) {
				state_dprintk("slave rf_in is not yet ready %d!=%d\n", new_rf_in->reservation.config_id, ic->config_id);
				goto tempfail_;
			}
		}
	}

	if(new_tuner && !will_be_master && !state->legacy_rf_in && !new_rf_in->sec_configured) {
		state_dprintk("cannot yet become slave because secondary equipment is not yet configured "
									"rf_in=%p config_id=%d -> %d; rf_in_use_count=%d\n",
									new_rf_in,
									new_rf_in->reservation.config_id, ic->config_id,
									new_rf_in->reservation.use_count);
		goto tempfail_;

	}

	if(new_tuner && !will_be_master && unicable_mode && !new_rf_in->unicable_mode) {
		state_dprintk("slave wants unicable but master disallows "
									"config_id=%d -> %d; rf_in_use_count=%d\n",
									new_rf_in->reservation.config_id, ic->config_id,
									new_rf_in->reservation.use_count);
		goto fatal_;
	}

	//we can make the reservation, but need to release old resources

	if(old_rf_in) {
		if (!same_rf_in) {
			state_dprintk("decrementing rf_in[%d].use_count=%d\n", old_rf_in->rf_in_no, old_rf_in->reservation.use_count);
			--old_rf_in->reservation.use_count;
		}
		if(old_rf_in->reservation.use_count == 0) {
			state_dprintk("releasing rf_in %d\n", old_rf_in->rf_in_no);
			old_rf_in->reservation.owner = -1;
			old_rf_in->reservation.config_id = -1;
		}
		if (old_rf_in->reservation.use_count <0) {
			state_dprintk("BUG rf_in[%d].use_count=%d < 0\n", old_rf_in->rf_in_no, old_rf_in->reservation.use_count);
		}
	}

	if(old_tuner) {
		if(!same_tuner) {
			state_dprintk("decrementing tuner[%d].use_count=%d\n", old_tuner->tuner_no, old_tuner->reservation.use_count);
			--old_tuner->reservation.use_count;
		}
		if(old_tuner->reservation.use_count == 0) {
			state_dprintk("releasing tuner[%d]\n", old_tuner->tuner_no);
			old_tuner->reservation.owner = -1;
			old_tuner->reservation.config_id = -1;
		}
		if (old_tuner->reservation.use_count <0) {
			state_dprintk("BUG use_count=%d < 0\n", old_tuner->reservation.use_count);
		}
	}
	if(!new_rf_in)
		return FE_RESERVATION_RELEASED;

	//make new rf_in reservation
	if(new_rf_in && new_rf_in->reservation.owner>=0 && new_rf_in->reservation.owner != ic->owner) {
		state_dprintk("BUG: unexpected owner %d %d\n", new_rf_in->reservation.owner, ic->owner);
	}
	if(will_be_master) {
		new_rf_in->sec_configured = false; //prepare for configuration
		state_dprintk("reset sec_configured rf_in=%p", new_rf_in);
		if(new_rf_in->reservation.config_id >= 0 && new_rf_in->reservation.use_count > same_rf_in &&
			 new_rf_in->reservation.config_id != ic->config_id) {
			state_dprintk("BUG: unexpected config_id %d %d use)count=%d\n", new_rf_in->reservation.config_id, ic->config_id, new_rf_in->reservation.use_count);
		}
		new_rf_in->reservation.config_id = ic->config_id;
	}
	if(!same_rf_in)
		new_rf_in->reservation.use_count++;
	if (new_rf_in && new_rf_in->reservation.owner == -1) {
		new_rf_in->reservation.owner = ic->owner;
		new_rf_in->reservation.config_id = ic->config_id;
	} else {
		if (new_rf_in && new_rf_in->reservation.owner != ic->owner) {
			state_dprintk("BUG: we are not owner\n");
		}
	}
	state_dprintk("reserved rf_in=%d config_id=%d rf_in[%d].use_count=%d\n", rf_in, ic->config_id,
								new_rf_in->rf_in_no,
								new_rf_in->reservation.use_count);

	//make tuner reservation
	if(new_tuner->reservation.owner>=0 && new_tuner->reservation.owner != ic->owner) {
		state_dprintk("BUG: unexpected owner %d %d\n", new_tuner->reservation.owner, ic->owner);
	}
	if(will_be_master) {
		if(new_tuner->reservation.config_id>=0 && new_tuner->reservation.use_count > same_tuner &&
			 new_tuner->reservation.config_id != ic->config_id) {
			state_dprintk("BUG: unexpected config_id %d %d use_count=%d\n", new_tuner->reservation.config_id, ic->config_id, new_tuner->reservation.use_count);
		}
		new_tuner->reservation.config_id = ic->config_id;
	}
	if(!same_tuner)
		new_tuner->reservation.use_count++;
	if(new_tuner->reservation.owner <0) {
		new_tuner->reservation.owner = ic->owner;
		new_tuner->reservation.config_id = ic->config_id;
	} else {
		if(new_tuner->reservation.owner !=ic->owner) {
			state_dprintk("BUG: we are not owner\n");
		}
	}
	state_dprintk("reserved tuner=%d config_id=%d tuner[%d].use_count=%d\n", tuner_no, ic->config_id,
								new_tuner->tuner_no, new_tuner->reservation.use_count);

	if(will_be_master)
		state_dprintk("Reservation MASTER chip_no=%d tuner_no=%d rf_in=%d\n",
									chip_no, tuner_no, rf_in);
	else
		state_dprintk("Reservation SLAVE chip_no=%d tuner_no=%d rf_in=%d\n",
									chip_no, tuner_no, rf_in);
	return will_be_master ? FE_RESERVATION_MASTER : FE_RESERVATION_SLAVE;
 tempfail_:
	state_dprintk("Reservation SOFT failed chip_no=%d tuner_no=%d rf_in=%d\n",
								chip_no, tuner_no, rf_in);
	return FE_RESERVATION_RETRY;
 fatal_:
	state_dprintk("Reservation HARD failed chip_no=%d tuner_no=%d rf_in=%d\n",
								chip_no, tuner_no, rf_in);
	return FE_RESERVATION_FAILED;
}

static enum fe_ioctl_result stid135_select_rf_in_(struct stv* state, struct fe_rf_input_control* ic)
{
	int err = FE_LLA_NO_ERROR;
	struct stv_chip_t* chip = state->chip;
	struct stv_card_t* card = chip->card;
	struct stv_tuner_t* old_tuner = state->active_tuner;
	struct stv_rf_in_t* old_rf_in = (old_tuner && old_tuner->active_rf_in) ? old_tuner->active_rf_in: NULL;
	int new_rf_in_no = ic ? ic->rf_in : -1;
	int old_rf_in_no = old_rf_in ? old_rf_in->rf_in_no : -1;
	int old_config_id = old_tuner ? old_tuner->reservation.config_id : -1;
	int new_tuner_no = new_rf_in_no;
	int old_tuner_no = old_tuner ? old_tuner->tuner_no : -1;
	struct stv_tuner_t* new_tuner = ic ? &state->chip->tuners[new_tuner_no] : NULL;
	struct stv_rf_in_t* new_rf_in = (ic && new_rf_in_no >=0 && new_rf_in_no < 4) ? &card->rf_ins[new_rf_in_no] : NULL;
	//for debugging
	int new_tuner_use_count_before = new_tuner ? new_tuner->reservation.use_count : 0;
	int new_rf_in_use_count_before = new_rf_in ? new_rf_in->reservation.use_count : 0;
	if(new_rf_in_no == old_rf_in_no && new_tuner_no == old_tuner_no && ic && old_config_id == ic->config_id) {
		state_dprintk("BUG: rf_in=%d AND tuner_no=%d unchanged\n", new_rf_in_no, new_tuner_no);
		return FE_RESERVATION_UNCHANGED;
	}

	enum fe_ioctl_result result = reserve_tuner_and_rf_in_(state, ic);
	if(new_tuner) {
		state_dprintk("owner=%d  config_id=%d old_rf_in=%d new_rf_in=%d; result=%s\n",
									ic->owner, ic->config_id, old_rf_in_no, new_rf_in_no, reservation_result_str(result));

		if(result != FE_RESERVATION_MASTER && result != FE_RESERVATION_SLAVE)
			return result;
	} else {

		state_dprintk("old_rf_in=%d RELEASED; result=%s\n", old_rf_in_no, reservation_result_str(result));

		if(result != FE_RESERVATION_RELEASED)
			state_dprintk("BUG: result=%s\n", reservation_result_str(result));

	}

	if(old_rf_in && old_rf_in->reservation.use_count==0) {
		struct stv_chip_t* old_chip = old_rf_in->controlling_chip;
		state_dprintk("old rf_in no longer in use: rf_in=%d; TONE OFF\n", old_rf_in_no);
		if(!old_chip || old_rf_in_no <0) {
			state_dprintk("BUG: chip=%p old_chip=%p old_rf_in_no=%d\n", state->chip, old_chip, old_rf_in_no);
		} else {
			bool must_lock = old_chip != state->chip;
			if(must_lock)
				chip_chip_lock(old_chip);
			err = fe_stid135_set_22khz_cont(&old_chip->ip, old_rf_in_no + 1, false);
			if(must_lock)
				chip_chip_unlock(old_chip);
			if(old_chip->set_voltage) {
				state_dprintk("old rf_in no longer in use: rf_in=%d; VOLTAGE OFF\n", old_rf_in_no);
				old_chip->set_voltage(old_chip->i2c, SEC_VOLTAGE_OFF, old_rf_in_no);
			}
			old_rf_in->voltage = SEC_VOLTAGE_OFF;
			old_rf_in->tone = SEC_TONE_OFF;
			old_rf_in->controlling_chip = NULL;
			old_rf_in->sec_configured = false;
			state_dprintk("reset sec_configured rf_in=%p", old_rf_in);
		}
	}
	if(old_tuner && old_tuner->reservation.use_count==0) {
		state_dprintk("old tuner no longer in use: rf_in=%d; TUNER STANDBY\n", old_tuner->tuner_no);
		err |= FE_STiD135_TunerStandby(state->chip->ip.handle_demod, old_rf_in_no + 1, 0);
		old_tuner->powered_on = false;
		old_tuner->active_rf_in = NULL;
		state->active_tuner = NULL;
	}
	if(!new_tuner) {
		state->active_tuner = new_tuner;
		state->is_master = false;
		state->quattro_rf_in_mask = 0;
		state->quattro_rf_in = 0;
		state->legacy_rf_in = false;
		return FE_RESERVATION_FAILED;
	}

	state_dprintk("use_counts: tuner=%d/%d rf_in=%d/%d\n", new_tuner_use_count_before,
								new_tuner->reservation.use_count,
								new_rf_in_use_count_before, new_rf_in->reservation.use_count);

	if(!new_tuner->powered_on)  {
		if (result == FE_RESERVATION_SLAVE)
			state_dprintk("slave reservation but tuner not yet powered on; powering on\n");
		if(result == FE_RESERVATION_MASTER)
			state_dprintk("master reservation but tuner not yet powered on; powering on\n");
		state_dprintk("Enabling tuner %d\n", new_rf_in_no);
		err |= fe_stid135_tuner_enable(state->chip->ip.handle_demod, new_rf_in_no + 1);
#if 0
		err |= fe_stid135_diseqc_init(&state->chip->ip, new_rf_in_no + 1, FE_SAT_DISEQC_2_3_PWM);
#endif

		new_tuner->powered_on = true;
		if(new_tuner->tuner_no != new_rf_in_no) {
			state_dprintk("BUG: tuner_no=%d != rf_in=%d\n", new_tuner->tuner_no, new_rf_in_no);
		}
		if(new_tuner->active_rf_in) {
			state_dprintk("unexpected: already have active_rf_in=%p\n", active_rf_in);
		} else {
			new_tuner->active_rf_in = new_rf_in;
			state_dprintk("set active_rf_in=%p\n", new_tuner->active_rf_in);
		}
	}

	state->active_tuner = new_tuner;
	if(!new_tuner->active_rf_in) {
		if(result != FE_RESERVATION_MASTER) {
			state_dprintk("BUG: tuner has no active_rf_in and we are not master\n");
		}
		new_tuner->active_rf_in = new_rf_in;
	} else if (new_tuner->active_rf_in != new_rf_in) {
		state_dprintk("BUG: active_rf_in=%d new_rf_in=%d\n", state->active_tuner->active_rf_in->rf_in_no, new_rf_in->rf_in_no);
	}
	state_dprintk("Setting rf_mux_path rf_in=%d (was %d) result=%d err=%d\n", new_rf_in_no, old_rf_in_no, result, err);

	if(!new_rf_in->controlling_chip) {
		int err1;
		new_rf_in->controlling_chip = state->chip;
		err |= (err1=fe_stid135_diseqc_init(&state->chip->ip, new_rf_in_no + 1, FE_SAT_DISEQC_2_3_PWM));
		state_dprintk("Set controlling chip and initing diseqc err=%d\n", err1);
		new_rf_in->tone = SEC_TONE_OFF;
		new_rf_in->voltage = SEC_VOLTAGE_OFF;
	}
	err |= fe_stid135_set_rfmux_path(state, new_rf_in_no + 1);
	if (err) {
		state_dprintk("Returning failed: err=%d\n", err);
		return FE_RESERVATION_FAILED;
	}
	state->is_master = (result == FE_RESERVATION_MASTER);
	if(state->is_master)
		new_rf_in->unicable_mode = ic->unicable_mode;
	return result;
}

/*
	select an rf_input and tuner if driver user did not call select_rf_input
	needed to support legacy drivers that do not call stid135_select_rf_in_
 */
static int stid135_select_rf_in_legacy_(struct stv* state)
{
	int rf_in_no = state->fe.ops.info.default_rf_input;
	enum fe_ioctl_result result = FE_RESERVATION_FAILED;
	if(!state->active_tuner || !state->active_tuner->active_rf_in) {
		state_dprintk("starting active_tuner=%p active_rf_in=%p\n", state->active_tuner,
									state->active_tuner ? state->active_tuner->active_rf_in : NULL);
		state->legacy_rf_in = true;
		if(state->chip->multiswitch_mode== FE_MULTISWITCH_MODE_QUATTRO) {
			if(state->quattro_rf_in_mask !=3) {
				state_dprintk("Quattro mode: no rf_in yet rf_in_no=%d rf_in_no_mask=%d\n",
											state->quattro_rf_in,state->quattro_rf_in_mask);
				return 0;
			} else
				rf_in_no = state->quattro_rf_in;
		} else if (state->chip->multiswitch_mode== FE_MULTISWITCH_MODE_UNICABLE) {
			rf_in_no = 3;
		}
		state_dprintk("rf_in_no=%d active_tuner=%p active_rf_in=%p\n", rf_in_no, state->active_tuner,
									state->active_tuner ? state->active_tuner->active_rf_in : NULL);

		struct fe_rf_input_control ic;
		ic.owner = (pid_t)0xffffffff;
		ic.config_id = 1;
		ic.rf_in = rf_in_no;
		ic.unicable_mode = true;
		if(!state_chip_is_locked_by_state(state)) {
			state_dprintk("Attempting select_rf_in_ without chp lock");
			dump_stack();
		}
		result = stid135_select_rf_in_(state, &ic);
		state_dprintk("selected legacy rf_in=%d result=%d\n", rf_in_no, result);
		return (result ==FE_RESERVATION_FAILED) ? -1 :0; /*Legacy is not clever enough for slave/master */
	}
	return 0;
}

static int stid135_init(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	state_dprintk("init\n");
	return 0;
}

static void stid135_release(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	dev_dbg(&state->chip->i2c->dev, "%s: demod %d\n", __func__, state->nr);

	state->chip->use_count--;
	if (state->chip->use_count == 0) {
		state_chip_lock(state);
		card_lock(state);
		FE_STiD135_Term (&state->chip->ip);
		state->chip->card->use_count--;
		if(state->chip->card->use_count ==0) {
			kobject_put(state->chip->card->sysfs_kobject);
			list_del(&state->chip->card->stv_card_list);
			kfree(state->chip->card);
		}
		card_unlock(state);
		state_chip_unlock(state);
		stv_chip_release_sysfs(state->chip);
		list_del(&state->chip->stv_chip_list);
		kfree(state->chip);
	}
	kobject_put(state->sysfs_kobject);
	list_del(&state->stv_demod_list);
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
		error = (fe_lla_error_t) ChipGetField(state->chip->ip.handle_demod,
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

		error = (fe_lla_error_t) ChipGetField(state->chip->ip.handle_demod,
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


static int stid135_set_rf_input(struct dvb_frontend* fe, struct fe_rf_input_control* ic)
{
	struct stv *state = fe->demodulator_priv;
	enum fe_ioctl_result result = FE_RESERVATION_FAILED;
	state_chip_lock(state);
	card_lock(state);
	state->legacy_rf_in = ic->config_id <0;
	result = stid135_select_rf_in_(state, ic);
	card_unlock(state);
	state_chip_unlock(state);
	return result;
}


static int stid135_set_parameters(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	int rf_in = active_rf_in_no(state);
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
	dev_dbg(&state->chip->i2c->dev,
			"delivery_system=%u modulation=%u frequency=%u symbol_rate=%u inversion=%u stream_id=%d\n",
			p->delivery_system, p->modulation, p->frequency,
					p->symbol_rate, p->inversion, p->stream_id);
	dprintk("demod=%d: user set stream_id=0x%x", state->nr, p->stream_id);

	if(state->chip->card->blindscan_always) {
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
	err = FE_STiD135_GetLoFreqHz(&state->chip->ip, &(search_params.lo_frequency));
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
	dev_dbg(&state->chip->i2c->dev, "%s: demod %d + tuner %d\n", __func__, state->nr, rf_in);
	vprintk("[%d] demod %d + tuner %d\n",
					state->nr+1,state->nr, rf_in);
	err |= stid135_select_rf_in_legacy_(state);

	if(state->modcode_filter){

		/*Deep Thought: keeping filters ensures that  a demod does not cause a storm of data when demodulation is
			failing (e.g., rain fade) This could cause other demods to fail as well as they share resources.
			filter_forbidden_modcodes may be better

				There could be reasons why  fe_stid135_reset_modcodes_filter is still needed, e.g., when  too strict filters
				are left from an earlier tune?
		*/
		err |= fe_stid135_reset_modcodes_filter(state);
		if (err != FE_LLA_NO_ERROR) {
			dev_err(&state->chip->i2c->dev, "%s: fe_stid135_reset_modcodes_filter error %d !\n", __func__, err);
			vprintk("[%d]: fe_stid135_reset_modcodes_filter error %d !\n", state->nr+1, err);
		}
	}

	err |= (error1=fe_stid135_search(state, &search_params, 0));
	if(error1!=0)
		dprintk("demod=%d: fe_stid135_search returned error=%d\n", state->nr, error1);
	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state->chip->i2c->dev, "%s: fe_stid135_search error %d !\n", __func__, err);
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

		dev_dbg(&state->chip->i2c->dev, "%s: locked !\n", __func__);
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
		if(state->chip->set_TSsampling)
			state->chip->set_TSsampling(state->chip->i2c,state->nr/2,4);   //for tbs6912
	} else {
		//state->signal_info.has_signal = 1;
		//state->signal_info.has_lock = 0;
		err |= fe_stid135_get_band_power_demod_not_locked(state, &rf_power);
		dev_dbg(&state->chip->i2c->dev, "%s: not locked, band rf_power %d dBm ! demod=%d tuner=%d\n",
						 __func__, rf_power / 1000, state->nr, rf_in);
	}
	vprintk("[%d] set_parameters: error=%d locked=%d\n", state->nr+1, err, state->signal_info.has_lock);

	/* Set modcode after search */
	if (p->modcode != MODCODE_ALL) {
        u32 m = p->modcode;
        u32 j = 0;
				u32 i;
				struct fe_sat_dvbs2_mode_t modcode_mask[FE_SAT_MODCODE_UNKNOWN*4];
        dev_dbg(&state->chip->i2c->dev, "%s: set Modcode mask %x!\n", __func__, p->modcode);
        m >>= 1;
        for (i=FE_SAT_QPSK_14; i < FE_SAT_MODCODE_UNKNOWN; i ++) {
            if (m & 1) {
                dev_dbg(&state->chip->i2c->dev, "%s: Modcode %02x enabled!\n", __func__, i);
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
            dev_err(&state->chip->i2c->dev, "%s: fe_stid135_set_modcodes_filter error %d !\n", __func__, err);
	}

	/* Set ISI after search */
	if (p->stream_id != NO_STREAM_ID_FILTER) {
		dev_warn(&state->chip->i2c->dev, "%s: set ISI %d ! demod=%d tuner=%d\n", __func__, p->stream_id & 0xFF,
						 state->nr, rf_in);
		err |= fe_stid135_set_mis_filtering(state, TRUE, p->stream_id & 0xFF, 0xFF);
	} else {
		dev_dbg(&state->chip->i2c->dev, "%s: disable ISI filtering !\n", __func__);
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
		dev_err(&state->chip->i2c->dev, "%s: fe_stid135_set_mis_filtering error %d !\n", __func__, err);

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
	vprintk("read stream_id mis=%d pls_mode=0x%x pls_code=0x%x stream_id=0%x fec=%d",
					state->mis_mode,
					state->signal_info.pls_mode, state->signal_info.pls_code, p->stream_id, p->fec_inner);

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
		dev_err(&state->chip->i2c->dev, "fe_stid135_get_lock_status error\n");
		return -EIO;
	}

	if (!state->signal_info.has_carrier) {
		/* demod not locked */
		*status |= FE_HAS_SIGNAL;
		vprintk("HAS_SIGNAL AND TUNED/no viterbi sync=%d status=%d\n", state->signal_info.has_sync, *status);
		err = fe_stid135_get_band_power_demod_not_locked(state,  &state->signal_info.power);
		// if unlocked, set to lowest resource..
		if (err != FE_LLA_NO_ERROR) {
			dev_err(&state->chip->i2c->dev, "fe_stid135_get_band_power_demod_not_locked error\n");
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
		dev_err(&state->chip->i2c->dev, "fe_stid135_get_signal_quality error\n");
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
		dev_warn(&state->chip->i2c->dev, "%s: fe_stid135_filter_forbidden_modcodes error %d !\n", __func__, err);

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
	vprintk("read stream_id mis=%d pls_mode=0x%x pls_code=0x%x stream_id=0%x fec=%d",
					state->mis_mode,
					state->signal_info.pls_mode, state->signal_info.pls_code, p->stream_id, p->fec_inner);

	vprintk("READ stream_id=0x%x isi=0x%x\n",p->stream_id, state->signal_info.isi);

	//for the tbs6912 ts setting
	if((state->chip->set_TSparam)&&(state->newTP)) {
		speed = state->chip->set_TSparam(state->chip->i2c,state->nr/2,4,0);
		if(!state->bit_rate)
			state->bit_rate = speed;
		if((((speed-state->bit_rate)<160)&&((speed-state->bit_rate)>3))||(state->loops==0)) {
			state->chip->set_TSparam(state->chip->i2c,state->nr/2,4,1);
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
	if (!state_chip_trylock(state)) {
		if (state->signal_info.has_viterbi) {//XX: official driver tests for has_sync?
			*status |= FE_HAS_SIGNAL | FE_HAS_CARRIER
				| FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		}
		return 0;
	}
	ret = stid135_read_status_(fe, status);
	state_chip_unlock(state);
	return ret;
}

static int stid135_set_sec_ready_(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	//	int err=0;
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	if(!rf_in) {
		state_dprintk("BUG active_rf_in==NULL\n");
		return -1;
	}
	state_dprintk("Marking rf_in as configured active_rf_in=%p old=%d\n", rf_in, rf_in->sec_configured);
	card_lock(state);
	rf_in->sec_configured = true; //we assume that caller has waited long enough
	card_unlock(state);
	return 0;
}

static int stid135_set_sec_ready(struct dvb_frontend* fe)
{
	int ret=0;
	struct stv *state = fe->demodulator_priv;
	state_chip_lock(state);
	ret = stid135_set_sec_ready_(fe);
	state_chip_unlock(state);
	return ret;
}

static int stid135_tune_(struct dvb_frontend* fe, bool re_tune,
		unsigned int mode_flags,
		unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	int r;
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	if(!rf_in) {
		state_dprintk("BUG active_rf_in==NULL\n");
		return -1;
	}
	dprintk("demod=%d re_tune=%d\n", state->nr, re_tune);
	if (re_tune) {
		stid135_set_sec_ready_(fe);
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
	int err=0;
	enum fe_ioctl_result result;
	state_chip_lock(state);
	if(!state->active_tuner || !state->active_tuner->active_rf_in) {
		card_lock(state);
		result = stid135_select_rf_in_legacy_(state);
		card_unlock(state);
	}
	if(result <0) {
		state_dprintk("rf_in=NULL result=%d active_tuner=%p active_rf_in=%p\n", result, state->active_tuner,
									state->active_tuner? state->active_tuner->active_rf_in : NULL);
		state_chip_unlock(state);
		return -1;
	}

	err = stid135_tune_(fe, re_tune, mode_flags, delay, status);
	if(/*state->signal_info.has_carrier &&*/ p->constellation.num_samples>0) {
		int max_num_samples = state->signal_info.symbol_rate /5 ; //we spend max 500 ms on this
		if(max_num_samples > 1024)
			max_num_samples = 1024; //also set an upper limit which should be fast enough
		err|= stid135_constellation_start_(fe, &p->constellation, max_num_samples);
	}
	state_chip_unlock(state);
	return err ?  -1 :0;
}


static enum dvbfe_algo stid135_get_algo(struct dvb_frontend* fe)
{
	return DVBFE_ALGO_HW;
}

static int stid135_set_voltage(struct dvb_frontend* fe, enum fe_sec_voltage voltage)
{

	struct stv *state = fe->demodulator_priv;
	int err=0;
	state_dprintk("mode=%d voltage=%s\n", state->chip->multiswitch_mode, voltage_str(voltage));
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	struct stv_tuner_t* tuner = state->active_tuner;
	bool last_rf_in_user=false;

	if(state->chip->multiswitch_mode== FE_MULTISWITCH_MODE_QUATTRO) {
		if(!(state->quattro_rf_in_mask &2)) {
			state->quattro_rf_in_mask |= 2;
			if (voltage == SEC_VOLTAGE_18) {
				state->quattro_rf_in |= 2;
			}
			else
				state->quattro_rf_in &= ~2;
		}
		state_dprintk("Quattro mode: voltage=%s mask=%d\n", voltage_str(voltage), state->quattro_rf_in_mask);
	}

	if (state->chip->set_voltage && (!tuner || ! rf_in)) {//@todo: locking maybe not needed
		//for older applications, which do not call FE_SET_RF_INPUT
		state_dprintk("before lock\n");
		state_chip_lock(state); //DeepThought: needed as this may call select_rf_in_ which needs chip access
		card_lock(state);
		state_dprintk("after lock\n");
		err |= stid135_select_rf_in_legacy_(state);
		state_dprintk("before unlock err=%d\n", err);
		card_unlock(state);
		state_chip_unlock(state);
		state_dprintk("after unlock\n");
		rf_in = active_rf_in(state);
		tuner = state->active_tuner;
	}

	if(!rf_in) {
		state_dprintk("BUG No active_rf_in set\n");
		return -EPERM;
	}

	if(!tuner) {
		state_dprintk("BUG No active_tuner set\n");
		return -EPERM;
	}
	last_rf_in_user = rf_in->reservation.use_count == 1;
	/*
		when multiple adapters use the same tuner, only one can change voltage, send tones, diseqc etc
		This is the tuner_owner. tuner_owner=-1 means no owner.
		We ignore any request to change voltage for all but one adapter, i.e., the master adapter
		Exceptions:
		 - we allow a slave to power down the LNB if the slave is the last user
		 - we also allow slaves to change voltages for unicable signalling

	*/
	if(!state->is_master && ! ( last_rf_in_user && voltage!= SEC_VOLTAGE_OFF) &&
		 ! (rf_in->unicable_mode && voltage !=  SEC_VOLTAGE_OFF)) {
		if(!rf_in) {
			state_dprintk("Skipping set_voltage=%s; no active rf_in\n", voltage_str(voltage));
			return -EPERM;
		}
		if(rf_in->voltage != voltage) {
			state_dprintk("Skipping set_voltage=%s; not master and current voltage=%s\n", voltage_str(voltage),
										voltage_str(rf_in->voltage));
			return voltage == SEC_VOLTAGE_OFF ? 0: -1;
		}
		state_dprintk("Skipping set_voltage=%s; not master, but voltage ok\n", voltage_str(voltage));
		return 0;
	}

	if(!state->is_master && ( last_rf_in_user && voltage== SEC_VOLTAGE_OFF))
		 state_dprintk("Allowing last slave to power down LNB");

	if(!state->is_master && (rf_in->unicable_mode && voltage !=  SEC_VOLTAGE_OFF))
		 state_dprintk("Allowing sklave to change voltage for unicable signalling");
	if(state->chip->set_voltage) {
		int old_voltage = rf_in->voltage;
		bool voltage_was_on = (old_voltage != SEC_VOLTAGE_OFF);
		bool voltage_is_on = (voltage != SEC_VOLTAGE_OFF);

		if(rf_in->unicable_mode && old_voltage == SEC_VOLTAGE_18 && voltage == SEC_VOLTAGE_18) {
			state_dprintk("Unicable command in progress; retry later");
			return FE_UNICABLE_DISEQC_RETRY;
		}
		card_lock(state); //DeepThought: maybe not needed (as it does not use stid135 chips)
		if(rf_in->reservation.use_count >1 && ! state->legacy_rf_in) {
			card_unlock(state);
			state_dprintk("SKIPPING set_voltage voltage=%s on=%d/%d "
										"tuner_use_count=%d rf_in_use_count=%d owner=%d\n", voltage_str(voltage),
										voltage_was_on, voltage_is_on,
										tuner->reservation.use_count, rf_in->reservation.use_count,
										tuner->reservation.owner);
		} else {
			rf_in->voltage = voltage;
			card_unlock(state);
			state_dprintk("calling set_voltage voltage=%s on=%d/%d "
										"tuner_use_count=%d rf_in_use_count=%d owner=%d\n", voltage_str(voltage),
										voltage_was_on, voltage_is_on, tuner->reservation.use_count, rf_in->reservation.use_count,
										tuner->reservation.owner);
			state->chip->set_voltage(state->chip->i2c, voltage, rf_in->rf_in_no);
		}
		state_dprintk("set_voltage done: rf_in=%d\n", rf_in->rf_in_no);
	}
	return 0;
}

static int stid135_set_tone(struct dvb_frontend* fe, enum fe_sec_tone_mode tone)
{

	struct stv *state = fe->demodulator_priv;
	int err=0;
	state_dprintk("mode=%d tone=%d", state->chip->multiswitch_mode, tone);
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	struct stv_tuner_t* tuner = state->active_tuner;

	if(state->chip->multiswitch_mode== FE_MULTISWITCH_MODE_QUATTRO) {
		if(!(state->quattro_rf_in_mask &1)) {
			state->quattro_rf_in_mask |= 1;
			if (tone == SEC_TONE_ON) {
				state->quattro_rf_in |= 1;
			}
			else
				state->quattro_rf_in &= ~1;
		}
		state_dprintk("Quattro mode: tone=%s mask=%d\n", tone_str(tone), state->quattro_rf_in_mask);
	}

	if ((!tuner || ! rf_in)) {//@todo: locking maybe not needed
		//for older applications, which do not call FE_SET_RF_INPUT
		state_chip_lock(state);
		card_lock(state);
		err |= stid135_select_rf_in_legacy_(state);
		card_unlock(state);
		state_chip_unlock(state);
		rf_in = active_rf_in(state);
		tuner = state->active_tuner;
	}

	if(!rf_in) {
		state_dprintk("BUG No active_rf_in set\n");
		return -EPERM;
	}

	if(!tuner) {
		state_dprintk("BUG No active_tuner set\n");
		return -EPERM;
	}

	/*
		when multiple adapters use the same tuner, only one can change voltage, send tones, diseqc etc
		This is the tuner_owner. tuner_owner=-1 means no owner.
		We ignore any request to change voltage for all but one adapter, i.e., the master adapter
	*/
	if(!state->is_master) {
		if(!rf_in) {
			state_dprintk("Skipping set_tone=%d; no active rf_in\n", tone);
			return -EPERM;
		}
		if(rf_in->tone != tone) {
			state_dprintk("Skipping set_tone=%d; not master and current tone=%d\n", tone, rf_in->tone);
			return -1;
		}
		state_dprintk("Skipping set_tone=%d; not master, but tone ok\n", tone);
		return 0;
	}

	state_chip_lock(state);
	if(!rf_in->controlling_chip) {
		state_dprintk("BUG: controlling_chip=NULL != chip=%p\n", state->chip);
		return -1;
	}
	if(rf_in->controlling_chip != state->chip)
		state_dprintk("info: controlling_chip=%p != chip=%p\n", rf_in->controlling_chip, state->chip);

	err = fe_stid135_set_22khz_cont(&rf_in->controlling_chip->ip, rf_in->rf_in_no + 1, tone == SEC_TONE_ON);
	rf_in->tone = tone;
	state_chip_unlock(state);
	state_dprintk("set tone=%d old_tone=%d error=%d err=%d abort=%d rf_in_use_count=%d", tone, rf_in->tone, err,
								state->chip->ip.handle_demod->Error,
								state->chip->ip.handle_demod->Abort, rf_in->reservation.use_count);
	rf_in->tone = tone;
	return err != FE_LLA_NO_ERROR ? -1 : 0;
	return 0;
}

static int stid135_send_long_master_cmd(struct dvb_frontend* fe,
																				 struct dvb_diseqc_long_master_cmd *cmd)
{

	struct stv *state = fe->demodulator_priv;
	int err=0;
	state_dprintk("diseqc");
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	struct stv_tuner_t* tuner = state->active_tuner;

	if (state->chip->multiswitch_mode == FE_MULTISWITCH_MODE_QUATTRO) {
		state_dprintk("Cannot send diseqc in quattro mode\n");
		return -EPERM;
	}

	if ((!tuner || ! rf_in)) {//@todo: locking maybe not needed
		//for older applications, which do not call FE_SET_RF_INPUT
		card_lock(state); //DeepThought: maybe not needed (as it does not use stid135 chips)
		err |= stid135_select_rf_in_legacy_(state);
		card_unlock(state);
		rf_in = active_rf_in(state);
		tuner = state->active_tuner;
	}

	if(!rf_in) {
		state_dprintk("BUG No active_rf_in set\n");
		return -EPERM;
	}

	if(!tuner) {
		state_dprintk("BUG No active_tuner set\n");
		return -EPERM;
	}

	/*
		when multiple adapters use the same tuner, only one can change voltage, send tones, diseqc etc
		This is the tuner_owner. tuner_owner=-1 means no owner.
		We ignore any request to change voltage for all but one adapter, i.e., the master adapter
		Exceptions: allow unicable signalling
	*/
	if(!state->is_master) {
		if(!rf_in) {
			state_dprintk("Skipping send_master_cmd; no active rf_in\n");
			return -EPERM;
		}
		if(!rf_in->unicable_mode) //unicable slaves are allowed to send diseqc messages
			return 0;
	}
	if(!rf_in->controlling_chip) {
		state_dprintk("BUG: rf_in[%d]->controlling_chip=NULL; tuner[%d].use_count=%d rf_in[%d].use_count=%d\n", rf_in->rf_in_no,
									tuner->tuner_no,
									tuner->reservation.use_count, rf_in->rf_in_no, rf_in->reservation.use_count);
		return -1;
	}
	state_chip_lock(state);
	card_lock(state);
	if(rf_in->sec_configured && ! state->legacy_rf_in && ! state->is_master && ! rf_in->unicable_mode) {
		card_unlock(state);
		state_dprintk("SKIPPING diseqc: rf_in=%d; tuner[%d].use_count=%d rf_in[%d].use_count=%d legacy=%d\n",
									rf_in->rf_in_no,
									tuner->tuner_no,
									tuner->reservation.use_count, rf_in->rf_in_no, rf_in->reservation.use_count, state->legacy_rf_in);
		state_chip_unlock(state);
	} else {
		card_unlock(state); //we do not need lock anymore, and it prevents chip_sleep
		err |= fe_stid135_diseqc_init(&rf_in->controlling_chip->ip, rf_in->rf_in_no + 1, FE_SAT_DISEQC_2_3_PWM);
		err |= fe_stid135_diseqc_send(state, rf_in->rf_in_no + 1, cmd->msg, cmd->msg_len);
		state_chip_unlock(state);
		state_dprintk("diseqc sent: rf_in=%d; tuner[%d].use_count=%d rf_in[%d].use_count=%d\n", rf_in->rf_in_no,
								tuner->tuner_no,
									tuner->reservation.use_count, rf_in->rf_in_no, rf_in->reservation.use_count);

		return err != 0 ? -1 : 0;
	}
	return 0;
}

static int stid135_recv_slave_reply(struct dvb_frontend* fe,
					struct dvb_diseqc_slave_reply *reply)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

	state_dprintk("diseqc_recv_slave_reply");
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	struct stv_tuner_t* tuner = state->active_tuner;
#if 1
	if (state->chip->multiswitch_mode == 0)
		return 0;
#endif
	if ((!tuner || ! rf_in)) {//@todo: locking maybe not needed
		//for older applications, which do not call FE_SET_RF_INPUT
		card_lock(state); //DeepThought: maybe not needed (as it does not use stid135 chips)
		err |= stid135_select_rf_in_legacy_(state);
		card_unlock(state);
		rf_in = active_rf_in(state);
		tuner = state->active_tuner;
	}

	if(!rf_in) {
		state_dprintk("BUG No active_rf_in set\n");
		return -EPERM;
	}

	if(!tuner) {
		state_dprintk("BUG No active_tuner set\n");
		return -EPERM;
	}

	state_chip_lock(state);
	card_lock(state);
	if(rf_in->sec_configured && ! state->legacy_rf_in) {
		card_unlock(state);
		state_dprintk("SKIPPING diseqc: rf_in=%d; tuner[%d].use_count=%d rf_in[%d].use_count=%d legacy=%d\n",
									rf_in->rf_in_no,
									tuner->tuner_no,
									tuner->reservation.use_count, rf_in->rf_in_no, rf_in->reservation.use_count, state->legacy_rf_in);
		state_chip_unlock(state);
	} else {
		card_unlock(state); //we do not need lock anymore, and it prevents chip_sleep
		err = fe_stid135_diseqc_receive(&state->chip->ip, reply->msg, &reply->msg_len);
		state_dprintk("received %d bytes:", reply->msg_len);
		state_chip_unlock(state);
		state_dprintk("diseqc read reply: rf_in=%d; tuner[%d].use_count=%d rf_in[%d].use_count=%d\n", rf_in->rf_in_no,
								tuner->tuner_no,
									tuner->reservation.use_count, rf_in->rf_in_no, rf_in->reservation.use_count);

		return err != 0 ? -1 : 0;
	}

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->chip->i2c->dev, "%s: fe_stid135_diseqc_receive error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_send_burst(struct dvb_frontend* fe, enum fe_sec_mini_cmd burst)
{
	struct stv *state = fe->demodulator_priv;
	int err=0;
	state_dprintk("diseqc");
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	struct stv_tuner_t* tuner = state->active_tuner;

	if (state->chip->multiswitch_mode == FE_MULTISWITCH_MODE_QUATTRO) {
		state_dprintk("Cannot send diseqc in quattro mode\n");
		return -EPERM;
	}

	if ((!tuner || ! rf_in)) {//@todo: locking maybe not needed
		//for older applications, which do not call FE_SET_RF_INPUT
		state_chip_lock(state); //DeepThought: maybe not needed (as it does not use stid135 chips)
		card_lock(state); //DeepThought: maybe not needed (as it does not use stid135 chips)
		err |= stid135_select_rf_in_legacy_(state);
		rf_in = active_rf_in(state);
		tuner = state->active_tuner;
		card_unlock(state);
		state_chip_unlock(state);
	}

	if(!rf_in) {
		state_dprintk("BUG No active_rf_in set\n");
		return -EPERM;
	}

	if(!tuner) {
		state_dprintk("BUG No active_tuner set\n");
		return -EPERM;
	}

	/*
		when multiple adapters use the same tuner, only one can change voltage, send tones, diseqc etc
		This is the tuner_owner. tuner_owner=-1 means no owner.
		We ignore any request to change voltage for all but one adapter, i.e., the master adapter
	*/
	if(!state->is_master) {
		if(!rf_in) {
			state_dprintk("Skipping send_burst; no active rf_in\n");
			return -EPERM;
		}
		return 0;
	}

	dprintk("Not implemented!\n");
	return err != 0 ? -1 : 0;
}

/*Called when the only user which has opened the frontend in read-write mode
	exits*/
static int stid135_sleep(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	int err=0;
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	struct stv_tuner_t* tuner = state->active_tuner;
	state_dprintk("ENTERING sleep\n");
	enum fe_ioctl_result result;
	if(!tuner || !rf_in) {
		state_dprintk("sleep; adapter did not use tuner; active_tuner=%p active_rf_in=%p\n",
									tuner, rf_in);
	} else {
		state_chip_lock(state);
		card_lock(state);
		state_dprintk("sleep tuner[%d].use_count=%d rf_in[%d].use_count=%d\n", tuner->tuner_no,
									tuner->reservation.use_count, rf_in->rf_in_no,
									rf_in->reservation.use_count);
		if(tuner->reservation.use_count<=0)
			state_dprintk("BUG: sleep tuner_use_count=%d <=0 \n", tuner->reservation.use_count);
		if(rf_in->reservation.use_count<=0)
			state_dprintk("BUG: sleep rf_in_use_count=%d <=0 \n", rf_in->reservation.use_count);
		result = stid135_select_rf_in_(state, NULL);
		state_dprintk("Before unlock\n");
		card_unlock(state);
		state_chip_unlock(state);
		state_dprintk("After unlock\n");
	}
	return err != 0 ? -1 : 0;
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
	struct i2c_adapter *adapter = state->chip->i2c;

	if (state->chip->read_properties)
		state->chip->read_properties(adapter,ecp3inf->reg, &(ecp3inf->data));
	return ;
}

static void spi_write(struct dvb_frontend* fe,struct ecp3_info *ecp3inf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->chip->i2c;

	if (state->chip->write_properties)
		state->chip->write_properties(adapter,ecp3inf->reg, ecp3inf->data);
	return ;
}

static void eeprom_read(struct dvb_frontend* fe, struct eeprom_info *eepinf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->chip->i2c;

	if (state->chip->read_eeprom)
		state->chip->read_eeprom(adapter,eepinf->reg, &(eepinf->data));
	return ;
}

static void eeprom_write(struct dvb_frontend* fe,struct eeprom_info *eepinf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->chip->i2c;

	if (state->chip->write_eeprom)
		state->chip->write_eeprom(adapter,eepinf->reg, eepinf->data);
	return ;
}

static int stid135_read_temp(struct dvb_frontend* fe, s16 *temp)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

	state_chip_lock(state);
	err = fe_stid135_get_soc_temperature(&state->chip->ip, temp);
	state_chip_unlock(state);

	if (err != FE_LLA_NO_ERROR)
		dev_warn(&state->chip->i2c->dev, "%s: fe_stid135_get_soc_temperature error %d !\n", __func__, err);
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
	//int rf_in = active_rf_in(state);
	struct fe_stid135_internal_param * pParams = (struct fe_stid135_internal_param *) &state->chip->ip;
	s32 lo_frequency;
	int error = FE_LLA_NO_ERROR;
	int error1 = FE_LLA_NO_ERROR;
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
	state_dprintk("range=[%d,%d]kHz num_freq=%d resolution=%dkHz bw=%dkHz clock=%d\n",
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

		state_chip_sleep(state, 12);

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
	//int rf_in_no = active_rf_in_no(state);
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	int old_rf_in_no = rf_in ? rf_in->rf_in_no : -1;
	int ret=0;
	fe_lla_error_t err = FE_LLA_NO_ERROR;

	stid135_stop_task(fe);


	if(p->rf_in < 0 || !p->rf_in_valid)  {
		p->rf_in = rf_in ? rf_in->rf_in_no : state->fe.ops.info.default_rf_input;
		p->rf_in_valid = true;
		state_dprintk("Set rf_in to %d; rf_in=%d  default_rf_in=%d\n",
						p->rf_in, old_rf_in_no, state->fe.ops.info.default_rf_input);
	}

	if(err) {
		state_dprintk("Could not set rfpath error=%d\n", err);
	}
	s->scale =  FE_SCALE_DECIBEL; //in units of 0.001dB

	state_chip_lock(state);
	state_dprintk("Marking rf_in as configured old=%d\n", rf_in->sec_configured);
	rf_in->sec_configured = true; //we assume that caller has waited long enough

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

	state_chip_unlock(state);
	return -1;
}

static int stid135_spectrum_get(struct dvb_frontend* fe, struct dtv_fe_spectrum* user)
{
	struct stv *state = fe->demodulator_priv;
	int error=0;
	state_chip_lock(state);
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
	state_chip_unlock(state);
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
	int rf_in = active_rf_in_no(state);
	//struct fe_stid135_internal_param * pParams = &state->chip->ip;
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
			p->rf_in = rf_in>=0 ?  rf_in : state->fe.ops.info.default_rf_input;
			p->rf_in_valid = true;
			state_dprintk("Set rf_in to %d; default_rf_in=%d\n",
										rf_in, state->fe.ops.info.default_rf_input);
		}

		state_chip_lock(state);
		ret = stid135_spectral_scan_start(fe);
		state_chip_unlock(state);

		if(ret<0) {
			dprintk("Could not start spectral scan\n");
			return -1; //
		}
	} else {
		if(!state->spectrum_scan_state.scan_in_progress) {
			dprintk("Error: Called with init==false, but scan was  not yet started\n");

			state_chip_lock(state);
			ret = stid135_spectral_scan_start(fe);
			state_chip_unlock(state);

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
		state_chip_lock(state);
		ret=stid135_spectral_scan_next(fe,  &p->frequency, &p->symbol_rate);
		state_chip_unlock(state);

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

		state_chip_lock(state);
		ret = stid135_tune_(fe, retune, mode_flags, delay, status);
		state_chip_unlock(state);

		old = *status;
		{
			state->chip->ip.handle_demod->Error = FE_LLA_NO_ERROR;

			state_chip_lock(state);
			fe_stid135_get_signal_info(state);
			state_chip_unlock(state);

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
	struct fe_stid135_internal_param * pParams = &state->chip->ip;
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
	struct stv_rf_in_t* rf_in = active_rf_in(state);
	state_chip_lock(state);
	if(!rf_in->sec_configured) {
		state_dprintk("Marking rf_in as configured rf_in=%p old=%d\n", rf_in, rf_in->sec_configured);
		rf_in->sec_configured = true; //we assume that caller has waited long enough

	}
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
	state_chip_unlock(state);
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
	.set_sec_ready                  = stid135_set_sec_ready,
	.set_rf_input			= stid135_set_rf_input,
	.set_tone			= stid135_set_tone,
	.set_voltage			= stid135_set_voltage,

	.diseqc_send_master_cmd		= NULL,
	.diseqc_send_long_master_cmd		= stid135_send_long_master_cmd,
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

static struct stv_chip_t* match_base(struct i2c_adapter* i2c, u8 adr)
{
	struct stv_chip_t *p;
	list_for_each_entry(p, &stv_chip_list, stv_chip_list)
		if (p->i2c == i2c && p->adr == adr)
			return p;
	return NULL;
}

static struct stv_card_t* match_card(struct tbsecp3_dev* dev)
{
	struct stv_card_t *p;
	list_for_each_entry(p, &stv_card_list, stv_card_list)
		if (p->dev == dev)
			return p;
	return NULL;
}

static inline int num_chips(void)
{
	struct stv_chip_t *p;
	int ret=0;
	list_for_each_entry(p, &stv_chip_list, stv_chip_list)
		ret++;
	return ret;
}

static inline int num_cards(void)
{
	struct stv_card_t* p;
	int ret=0;
	list_for_each_entry(p, &stv_card_list, stv_card_list)
		ret++;
	return ret;
}

static void init_stv_reservation(struct stv_reservation_t* res)
{
	res->owner = -1;
	res->config_id =-1;
}

static void init_rf_in(struct stv_rf_in_t* rf_in, int rf_in_no) {
	rf_in->rf_in_no = rf_in_no;
	rf_in->voltage = SEC_VOLTAGE_OFF;
	rf_in->tone = SEC_TONE_OFF;
	init_stv_reservation(&rf_in->reservation);
}

static void init_tuner(struct stv_tuner_t* tuner, int tuner_no) {
	tuner->tuner_no = tuner_no;
	init_stv_reservation(&tuner->reservation);
}

static void init_stv_card(struct stv_card_t* card, struct stv_chip_t* first_chip, struct tbsecp3_dev* tbsecp3_dev)
{
	int i;
	card->dev = tbsecp3_dev;
	mutex_init(&card->lock.mutex);
	card->card_no = num_cards();
	card->use_count = 1;
	card->num_chips = 1;
	card->chips[0] = first_chip;

	for(i=0; i < sizeof(card->rf_ins)/sizeof(card->rf_ins[0]); ++i) {
		init_rf_in(&card->rf_ins[i], i);
	}
	card->blindscan_always = blindscan_always;
}

static void init_stv_chip(struct stv_chip_t* chip, struct stv_card_t* card, int mode,
													struct stid135_cfg *cfg, struct i2c_adapter *i2c)
{
	mutex_init(&chip->lock.mutex);
	chip->global_chip_no = num_chips();
	chip->chip_no = card->num_chips -1;
	chip->adr = cfg->adr;
	chip->i2c = i2c;
	chip->card = card;
	chip->use_count = 1;
	chip->extclk = cfg->clk;
	chip->ts_mode = cfg->ts_mode;
	if(mode==1 && !cfg->set_voltage)
		chip->multiswitch_mode = 0;
	else
		chip->multiswitch_mode = mode;
	chip->num_tuners =4;
	chip->vglna		=	cfg->vglna;    //for stvvglna 6909x v2 6903x v2
	chip->control_22k	= cfg->control_22k;
	for(int i=0; i < sizeof(chip->tuners)/sizeof(chip->tuners[0]); ++i) {
		init_tuner(&chip->tuners[i], i);
	}
	chip->set_voltage = cfg->set_voltage;
	chip->write_properties = cfg->write_properties;
	chip->read_properties = cfg->read_properties;
	chip->write_eeprom = cfg->write_eeprom;
	chip->read_eeprom = cfg->read_eeprom;
	chip->set_TSsampling = cfg->set_TSsampling;
	chip->set_TSparam  = cfg->set_TSparam;
#ifdef NEWXXX
		mutex_init(&chip->lock.m);
#endif
}

/*DT: called with  nr=adapter=0...7 and rf_in = nr/2=0...3
	state->chip is created exactly once and is shared
	between the 8 demods; provides access to the i2c hardware and such
*/
struct dvb_frontend* stid135_attach(struct tbsecp3_dev* tbsecp3_dev, struct i2c_adapter *i2c,
																		struct stid135_cfg *cfg,
																		int nr, int rf_in)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	int i;
	struct stv *state;
	struct stv_chip_t* base=NULL;
	struct stv_card_t* card =NULL;
	state = kzalloc(sizeof(struct stv), GFP_KERNEL);

	if (!state)
		return NULL;

	base = match_base(i2c, cfg->adr);
	if (base) {
		base->use_count++;
		state->chip = base;
		dprintk("ATTACH DUP nr=%d rf_in=%d base=%p count=%d dev=%p i2c_addr=0x%x i2c=%s\n", nr, rf_in, base, base->use_count,
						tbsecp3_dev, cfg->adr,  dev_name(&i2c->dev));
	} else {
		base = kzalloc(sizeof(struct stv_chip_t), GFP_KERNEL);
		if (!base)
			goto fail;
		card = match_card(tbsecp3_dev);
		if(!card) {
			card = kzalloc(sizeof(struct stv_card_t), GFP_KERNEL);
			init_stv_card(card, base, tbsecp3_dev);
			list_add(&card->stv_card_list, &stv_card_list);
			stv_card_make_sysfs(card);
		} else {
			card->use_count++;
			card->chips[card->num_chips] = base;
			card->num_chips++;
		}
		init_stv_chip(base, card, mode, cfg, i2c);
		dprintk("ATTACH NEWx nr=%d rf_in=%d base=%p count=%d dev=%p i2c_addr=0x%x i2c=%s\n", nr, rf_in,
						base, base->use_count,
						tbsecp3_dev, cfg->adr,  dev_name(&i2c->dev));
		state->chip = base;
		if (stid135_probe(state) < 0) {
			dprintk("No demod found at adr %02X on %s\n", cfg->adr, dev_name(&i2c->dev));
			kfree(base);
			goto fail;
		}
		stv_chip_make_sysfs(base);
		list_add(&base->stv_chip_list, &stv_chip_list);
	}
#if 1 //official driver has none of this (it is done elsewhere)
	//pParams = &state->chip->ip;
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
	state->fe.ops.info.default_rf_input = rf_in;
	dev_info(&i2c->dev, "%s demod found at adr %02X on %s\n",
					 state->fe.ops.info.name, cfg->adr, dev_name(&i2c->dev));
	stv_demod_make_sysfs(state);
	list_add(&state->stv_demod_list, &stv_demod_list);
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
