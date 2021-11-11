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

static int vglna_mode=0;
module_param(vglna_mode, int, 0444);
MODULE_PARM_DESC(mode,
		"vlglna on/off");

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


#define vprintk(fmt, arg...)																					\
	if(stid135_verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)


void print_signal_info(const char* prefix, struct fe_sat_signal_info* i)
{
	#if 1
	vprintk("LOCK %s: fec=%d dmd=%d signal=%d carr=%d vit=%d sync=%d timedout=%d lock=%d\n",
				 prefix,
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
static int stid135_probe(struct stv *state_demod1)
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

	err = fe_stid135_init(&init_params, &state_demod1->base->ip);

	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state_demod1->base->i2c->dev, "%s: fe_stid135_init error %d !\n", __func__, err);
		return -EINVAL;
	}

	p_params = &state_demod1->base->ip;
	p_params->master_lock = &state_demod1->base->status_lock;
	vprintk("here state_demod1=%p\n", state_demod1);
	vprintk("here state_demod1->base=%p\n", state_demod1->base);
	vprintk("here state_demod1->base=%p\n", state_demod1->base);
	err = fe_stid135_get_cut_id(&state_demod1->base->ip, &cut_id);
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
		// note that  FE_SAT_DEMOD_1 is not used in the code in the following call for this ts_mode
		err |= fe_stid135_set_ts_parallel_serial(&state_demod1->base->ip, FE_SAT_DEMOD_1, FE_TS_PARALLEL_ON_TSOUT_0);
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
		err |= fe_stid135_set_ts_parallel_serial(&state_demod1->base->ip, FE_SAT_DEMOD_1, FE_TS_PARALLEL_PUNCT_CLOCK);
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
	stvvglna_set_standby(vglna_handle, vglna_mode);
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
	stvvglna_set_standby(vglna_handle1, vglna_mode);
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
	stvvglna_set_standby(vglna_handle2, vglna_mode);
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
	stvvglna_set_standby(vglna_handle3, vglna_mode);
	dev_warn(&state_demod1->base->i2c->dev, "Initialized STVVGLNA 3 device\n");
	}
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state_demod1->base->i2c->dev, "%s: setup error %d !\n", __func__, err);
	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_init(struct dvb_frontend* fe)
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
	BUG_ON((state->rf_in<0 || state->rf_in>=4));
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

static void stid135_release(struct dvb_frontend* fe)
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
		vprintk("Trying scrambling mode=%d code %d timeout=%d\n", (pls_code>>26) & 0x3, (pls_code>>8) & 0x3FFFF, timeout);
		set_pls_mode_code(state, (pls_code>>26) & 0x3, (pls_code>>8) & 0x3FFFF);
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
				dprintk("selecting stream_id=%d\n", isi);
				signal_info->isi = isi;
				//p->matype = matype_info;
				p->stream_id = 	(state->mis_mode? (isi&0xff):0xff) | (pls_code & ~0xff);
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
			dprintk("selecting stream_id=%d\n", isi);
			signal_info->isi = isi;
			p->stream_id = 	(state->mis_mode? (isi&0xff):0xff) | (pls_code & ~0xff);
		  //p->matype = matype_info;
			dprintk("PLS SET stream_id=0x%x isi=0x%x\n",p->stream_id, isi);
				break;
		}
	}
	dprintk("PLS search ended\n");
	return locked;
}


static int stid135_set_parameters(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;
	struct fe_sat_search_params search_params;
	struct fe_sat_search_result search_results;
	s32 rf_power;
	//BOOL lock_stat=0;
	struct fe_sat_signal_info* signal_info = &state->signal_info;
	s32 current_llr=0;
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
	if(blindscan_always) {
		p->algorithm = ALGORITHM_WARM;
		p->delivery_system = SYS_AUTO;
		p->symbol_rate = p->symbol_rate ==0 ? 22000000 : p->symbol_rate;
	}

#if 1 // XXX official driver always uses FE_SAT_COLD_START
	/* Search parameters */
	switch(p->algorithm) {
	case ALGORITHM_WARM:
		search_params.search_algo	= FE_SAT_WARM_START;
		search_params.symbol_rate = p->symbol_rate;
		break;

	case ALGORITHM_COLD:
	case ALGORITHM_COLD_BEST_GUESS:
		search_params.search_algo		= FE_SAT_COLD_START;
		search_params.symbol_rate = p->symbol_rate == 0;
		break;

	case ALGORITHM_BLIND:
	case ALGORITHM_BLIND_BEST_GUESS:
	case ALGORITHM_BANDWIDTH:
		search_params.search_algo		= FE_SAT_BLIND_SEARCH;
		search_params.standard = FE_SAT_AUTO_SEARCH;
		search_params.puncture_rate = FE_SAT_PR_UNKNOWN;
		if(p->symbol_rate<=0)
			search_params.symbol_rate = 22000000;
		else
			search_params.symbol_rate = p->symbol_rate;
		break;
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
	vprintk("[%d] lo_frequency = %d\n", state->nr+1, search_params.lo_frequency);
	search_params.lo_frequency *= 1000000;
	if(search_params.search_algo == FE_SAT_BLIND_SEARCH ||
		 search_params.search_algo == FE_SAT_NEXT) {
		//search_params.frequency =		950000000 ;
		vprintk("[%d[ BLIND: set freq=%d lo=%d\n",
						state->nr+1,
						search_params.frequency,	search_params.lo_frequency  );
	}

	dev_dbg(&state->base->i2c->dev, "%s: demod %d + tuner %d\n", __func__, state->nr, state->rf_in);
	vprintk("[%d] demod %d + tuner %d\n",
					state->nr+1,state->nr, state->rf_in);
	err |= fe_stid135_set_rfmux_path(state, state->rf_in + 1);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_rfmux_math error %d !\n", __func__, err);
#if 1 /*Deep Thought: keeping filters ensures that  a demod does not cause a storm of data when demodulation is
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
#endif

	err |= (error1=fe_stid135_search(state, &search_params, &search_results, 0));
#if 0  //XXX official driver calls this
	err |= fe_stid135_get_lock_status(state, 0, 0, 0 );
#endif
	if(error1!=0)
		dprintk("[%d] fe_stid135_search returned error=%d\n", state->nr+1, error1);
	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_search error %d !\n", __func__, err);
		dprintk("[%d[ fe_stid135_search error %d !\n", state->nr+1, err);
		return -1;
	}
#if 1 //missing in official driver, but only called during blindscan
	if(!state->signal_info.has_viterbi && p->algorithm != ALGORITHM_WARM && p->algorithm != ALGORITHM_COLD) {
		bool locked=false;
		print_signal_info("(before trying pls)", &state->signal_info);
		locked = pls_search_list(fe);
		if(!locked)
			locked = pls_search_range(fe);
		if(locked) {
			state->signal_info.has_lock=true;
			dprintk("PLS locked=%d\n", locked);
			print_signal_info("(PLS)", &state->signal_info);
		}
		if(locked) {
			/*The following can cause delock*/
			//set_stream_index(state, p->stream_id);
			//print_signal_info("(PLS2)", &state->signal_info);
		}
	} else {
		state->signal_info.has_lock=true;
#if 1
	set_stream_index(state, p->stream_id);
#endif
	}

	//state->DemodLockTime += TUNING_DELAY;
	vprintk("[%d] setting timedout=%d\n", state->nr+1, !state->signal_info.has_viterbi);
	/*
		has_viterbi: correctly tuned
		has_sync: received packets without errors
	 */
	state->signal_info.has_timedout = !state->signal_info.has_viterbi;


	vprintk("[%d] set_parameters: error=%d locked=%d vit=%d sync=%d timeout=%d\n",
					state->nr +1,
					err, state->signal_info.has_lock,
					state->signal_info.has_viterbi,					state->signal_info.has_sync,
					state->signal_info.has_timedout);
#endif

	if (state->signal_info.has_carrier){ //XXX official driver may require has_sync

		vprintk("[%d] set_parameters: error=%d locked=%d\n", state->nr+1, err, state->signal_info.has_lock);

		dev_dbg(&state->base->i2c->dev, "%s: locked !\n", __func__);
		//set maxllr,when the  demod locked ,allocation of resources
		//err |= fe_stid135_set_maxllr_rate(state, 180);
		get_current_llr(state, &current_llr);

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
	state->signal_info.pls_mode = (p->stream_id & 0x3);
	state->signal_info.pls_code = ((p->stream_id >> 8)  & 0x3FFFF);
	if(	state->signal_info.pls_code ==0)
			state->signal_info.pls_code = 1;
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_mis_filtering error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

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
		p->matype = state->signal_info.matype;
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
		if (state->signal_info.modcode < FE_SAT_MODCODE_UNKNOWN) {
			BUG_ON(state->signal_info.modcode <0 || state->signal_info.modcode>=0x20);
			p->fec_inner = modcod2fec[state->signal_info.modcode];
		} else
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
	p->stream_id = ((state->mis_mode ? (state->signal_info.isi &0xff) :0xff) |
									(state->signal_info.pls_mode << 26) |
									((state->signal_info.pls_code &0x3FFFF)<<8)
									);
	dprintk("SET stream_id=0x%x isi=0x%x\n",p->stream_id, state->signal_info.isi);
	return 0;
}

static int stid135_read_status_(struct dvb_frontend* fe, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	u32 speed;

	*status = 0;

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
	//dprintk("set *status=0x%x\n", *status);
	if (err != FE_LLA_NO_ERROR) {
		dev_err(&state->base->i2c->dev, "fe_stid135_get_lock_status error\n");
		return -EIO;
	}

	if (!state->signal_info.has_viterbi) { //XXX should perhaps be has_sync instead
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
		p->cnr.len=0; //XXX may be a will confuse user space
		return 0;
	}


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

	p->post_bit_error.len = 1;
	p->post_bit_error.stat[0].scale = FE_SCALE_COUNTER;
	p->post_bit_error.stat[0].uvalue = state->signal_info.ber;

	if (err != FE_LLA_NO_ERROR)
		dev_warn(&state->base->i2c->dev, "%s: fe_stid135_filter_forbidden_modcodes error %d !\n", __func__, err);


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
	if (!mutex_trylock(&state->base->status_lock)) {
		if (state->signal_info.has_viterbi) {//XX: official driver tests for has_sync?
			*status |= FE_HAS_SIGNAL | FE_HAS_CARRIER
				| FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		}
		return 0;
	}
	ret = stid135_read_status_(fe, status);
	mutex_unlock(&state->base->status_lock);
	return ret;
}


static int stid135_tune_(struct dvb_frontend* fe, bool re_tune,
		unsigned int mode_flags,
		unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	int r;
	if (re_tune) {
		r = stid135_set_parameters(fe);
		if (r) {
			return r;
		}
		state->tune_time = jiffies;
	}

	if(re_tune) {
		vprintk("[%d] RETUNE: GET SIGNAL\n", state->nr+1);
		fe_stid135_get_signal_info(state,  &state->signal_info, 0);
		//dprintk("MIS2: num=%d\n", state->signal_info.isi_list.nb_isi);
	}

	r = stid135_read_status_(fe, status);

	vprintk("[%d] setting timedout=%d\n", state->nr+1, !state->signal_info.has_viterbi);
	state->signal_info.has_timedout = !state->signal_info.has_viterbi;
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
	mutex_lock(&state->base->status_lock);
	r = stid135_tune_(fe, re_tune, mode_flags, delay, status);
	if(r>=0 && p->constellation.num_samples>0) {
		int max_num_samples = state->signal_info.symbol_rate /5 ; //we spend max 500 ms on this
		if(max_num_samples > 1024)
			max_num_samples = 1024; //also set an upper limit which should be fast enough
		r = stid135_constellation_start_(fe, &p->constellation, max_num_samples);
	}
	mutex_unlock(&state->base->status_lock);
	return r;
}


static enum dvbfe_algo stid135_get_algo(struct dvb_frontend* fe)
{
	return DVBFE_ALGO_HW;
}

static int stid135_set_voltage(struct dvb_frontend* fe, enum fe_sec_voltage voltage)
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

	if (state->base->set_voltage) {//XXX official driver does not use mutex
		mutex_lock(&state->base->status_lock); //DeepThought: this performs multiple i2c calls and needs to be protected
		state->base->set_voltage(state->base->i2c, voltage, state->rf_in);
		mutex_unlock(&state->base->status_lock);
	}
	return 0;
}

static int stid135_set_tone(struct dvb_frontend* fe, enum fe_sec_tone_mode tone)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	vprintk("DISEQC[%d] mode=%d tone=%d", state->rf_in+1,  state->base->mode, tone);
	if(state->base->mode == 0) //mode always 1
	{
		if (tone == SEC_TONE_ON)
			state->rf_in |= 1;
		else
			state->rf_in &= ~1;

		return 0;
	}

	mutex_lock(&state->base->status_lock);
	err = fe_stid135_set_22khz_cont(&state->base->ip, state->rf_in + 1, tone == SEC_TONE_ON);
	mutex_unlock(&state->base->status_lock);

	vprintk("DISEQC[%d] tone=%d error=%d err=%d abort=%d", state->rf_in+1,  tone, err,
					state->base->ip.handle_demod->Error,
					state->base->ip.handle_demod->Abort);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_22khz_cont error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
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

	mutex_lock(&state->base->status_lock);
	err |= fe_stid135_diseqc_init(&state->base->ip, state->rf_in + 1, FE_SAT_DISEQC_2_3_PWM);
	err |= fe_stid135_diseqc_send(&state->base->ip, state->rf_in + 1, cmd->msg, cmd->msg_len);
	mutex_unlock(&state->base->status_lock);

	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_diseqc_send error %d !\n", __func__, err);

	return err != FE_LLA_NO_ERROR ? -1 : 0;
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

	mutex_lock(&state->base->status_lock);
	err = fe_stid135_diseqc_receive(&state->base->ip, reply->msg, &reply->msg_len);
	mutex_unlock(&state->base->status_lock);

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

	return err != FE_LLA_NO_ERROR ? -1 : 0;
}

static int stid135_sleep(struct dvb_frontend* fe)
{
	struct stv *state = fe->demodulator_priv;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *p_params = &state->base->ip;

	if (state->base->mode == 0) //XXX official driver never calls code below
		return 0;

	dev_dbg(&state->base->i2c->dev, "%s: tuner %d\n", __func__, state->rf_in);
	dprintk("Starting to sleep");
	mutex_lock(&state->base->status_lock);
	BUG_ON((state->rf_in<0 || state->rf_in>=4));
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

	mutex_lock(&state->base->status_lock);
	err = fe_stid135_get_soc_temperature(&state->base->ip, temp);
	mutex_unlock(&state->base->status_lock);

	if (err != FE_LLA_NO_ERROR)
		dev_warn(&state->base->i2c->dev, "%s: fe_stid135_get_soc_temperature error %d !\n", __func__, err);
	return 0;
}


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
	struct spectrum_scan_state_t* ss = &state->scan_state;
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

		usleep_range(12000, 13000);

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
	struct spectrum_scan_state_t* ss = &state->scan_state;
	struct constellation_scan_state* cs = &state->constellation_scan_state;
	mutex_lock(&state->base->status_lock);
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
	mutex_unlock(&state->base->status_lock);
	//dprintk("Freed memory\n");
	return 0;
}

static int stid135_spectrum_start(struct dvb_frontend* fe,
																	struct dtv_fe_spectrum* s,
																	unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct spectrum_scan_state_t* ss = &state->scan_state;
	int ret=0;
	fe_lla_error_t err = FE_LLA_NO_ERROR;
	stid135_stop_task(fe);

	mutex_lock(&state->base->status_lock);
	err = fe_stid135_set_rfmux_path(state, state->rf_in + 1);
	if(err) {
		dprintk("Could not set rfpath error=%d\n", err);
	}
	s->scale =  FE_SCALE_DECIBEL; //in units of 0.001dB
	switch(s->spectrum_method) {
	case SPECTRUM_METHOD_SWEEP:
	default:
		ret=stid135_get_spectrum_scan_sweep(fe,  delay,  status);
		s->num_freq = ss->spectrum_len;
		break;
	case SPECTRUM_METHOD_FFT:
		ret=stid135_get_spectrum_scan_fft(fe, delay, status);
		s->num_freq = ss->spectrum_len;
		break;
	}
	mutex_unlock(&state->base->status_lock);
	return -1;
}

int stid135_spectrum_get(struct dvb_frontend* fe, struct dtv_fe_spectrum* user)
{
	struct stv *state = fe->demodulator_priv;
	int error=0;
	mutex_lock(&state->base->status_lock);
	if (user->num_freq> state->scan_state.spectrum_len)
		user->num_freq = state->scan_state.spectrum_len;
	if (user->num_candidates > state->scan_state.num_candidates)
		user->num_candidates = state->scan_state.num_candidates;
	if(state->scan_state.freq && state->scan_state.spectrum) {
		if(user->freq && user->num_freq > 0 &&
			 copy_to_user((void __user*) user->freq, state->scan_state.freq, user->num_freq * sizeof(__u32))) {
			error = -EFAULT;
		}
		if(user->rf_level && user->num_freq > 0 &&
			 copy_to_user((void __user*) user->rf_level, state->scan_state.spectrum, user->num_freq * sizeof(__s32))) {
			error = -EFAULT;
		}
		if(user->candidates && user->num_candidates >0 &&
			 copy_to_user((void __user*) user->candidates,
										 state->scan_state.candidates,
										 user->num_candidates * sizeof(struct spectral_peak_t))) {
			error = -EFAULT;
		}
	} else
		error = -EFAULT;
	mutex_unlock(&state->base->status_lock);
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
		if(state->scan_state.scan_in_progress) {
			stid135_stop_task(fe); //cleanup older scan
		}
		mutex_lock(&state->base->status_lock);
		ret = stid135_spectral_scan_start(fe);
		if(ret<0) {
			dprintk("Could not start spectral scan\n");
			mutex_unlock(&state->base->status_lock);
			return -1; //
		}
	} else {
		if(!state->scan_state.scan_in_progress) {
			dprintk("Error: Called with init==false, but scan was  not yet started\n");
			mutex_lock(&state->base->status_lock);
			ret = stid135_spectral_scan_start(fe);
			if(ret<0) {
				dprintk("Could not start spectral scan\n");
				return -1; //
			}
		}
		minfreq= state->scan_state.next_frequency;
		dprintk("SCAN SAT next_freq=%dkHz\n", state->scan_state.next_frequency);
	}
	dprintk("SCAN SAT delsys=%d\n", p->delivery_system);
	while(!found) {
		if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
			dprintk("exiting on should stop\n");
			break;
		}
		*status = 0;
		ret=stid135_spectral_scan_next(fe,  &p->frequency, &p->symbol_rate);
		if(ret<0) {
			dprintk("reached end of scan range\n");
			*status =  FE_TIMEDOUT;
			mutex_unlock(&state->base->status_lock);
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
		p->stream_id = -1;
#else
		p->symbol_rate = 1000000; //otherwise it may be set too low based on last transponder
		p->stream_id = -1;
#endif
		dprintk("FREQ=%d search_range=%dkHz fft=%d res=%dkHz srate=%dkS/s\n",
						p->frequency, p->search_range/1000,
						p->scan_fft_size, p->scan_resolution, p->symbol_rate/1000);
		ret = stid135_tune_(fe, retune, mode_flags, delay, status);
		old = *status;
		{
			state->base->ip.handle_demod->Error = FE_LLA_NO_ERROR;
			fe_stid135_get_signal_info(state,  &state->signal_info, 0);
			memcpy(p->isi_bitset, state->signal_info.isi_list.isi_bitset, sizeof(p->isi_bitset));
			p->matype =  state->signal_info.matype;
			p->frequency = state->signal_info.frequency;
			p->symbol_rate = state->signal_info.symbol_rate;
		}
		found = !(*status & FE_TIMEDOUT) && (*status &FE_HAS_LOCK);
		if(found) {
			state->scan_state.next_frequency = p->frequency + (p->symbol_rate*135)/200000;
			dprintk("BLINDSCAN: GOOD freq=%dkHz SR=%d kS/s returned status=%d next=%dkHz\n", p->frequency, p->symbol_rate/1000, *status,
							state->scan_state.next_frequency /1000);
			state->scan_state.num_good++;
		}
		else {
			dprintk("BLINDSCAN: BAD freq=%dkHz SR=%d kS/s returned status=%d\n", p->frequency, p->symbol_rate/1000, *status);
			state->scan_state.num_bad++;
		}
	}
	mutex_unlock(&state->base->status_lock);
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

#if 0
int stid135_constellation_start(struct dvb_frontend* fe,
																			 struct dtv_fe_constellation* user,
																			 unsigned int *delay, enum fe_status *status)
{
	int ret;
	struct stv* state = fe->demodulator_priv;
	stid135_stop_task(fe);
	mutex_lock(&state->base->status_lock);
	ret = stid135_constellation_start_(fe, user);
	*status =  FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	mutex_unlock(&state->base->status_lock);
	return 0;
}
#endif

static int stid135_constellation_get(struct dvb_frontend* fe, struct dtv_fe_constellation* user)
{
	struct stv *state = fe->demodulator_priv;
	struct constellation_scan_state* cs = &state->constellation_scan_state;
	int error = 0;
	mutex_lock(&state->base->status_lock);
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
	mutex_unlock(&state->base->status_lock);
	return error;
}






static struct dvb_frontend_ops stid135_ops = {
	.delsys = { SYS_DVBS, SYS_DVBS2, SYS_DVBS2X, SYS_DSS, SYS_AUTO },
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
		        FE_CAN_MULTISTREAM,
		.extended_caps          = FE_CAN_SPECTRUMSCAN	|FE_CAN_HR_SPECTRUMSCAN |
		FE_CAN_IQ | FE_CAN_BLINDSEARCH
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

	state = kzalloc(sizeof(struct stv), GFP_KERNEL);
	if (!state)
		return NULL;

	base = match_base(i2c, cfg->adr);
	if (base) {
		base->count++;
		state->base = base;
		dprintk("ATTACH DUP nr=%d rf_in=%d base=%p count=%d\n", nr, rf_in, base, base->count);
	} else {
		base = kzalloc(sizeof(struct stv_base), GFP_KERNEL);
		if (!base)
			goto fail;
		base->i2c = i2c;
		base->adr = cfg->adr;
		base->count = 1;
		dprintk("ATTACH NEW nr=%d rf_in=%d base=%p count=%d\n", nr, rf_in, base, base->count);
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
#if 1 //XXX official driver has none of this (it is done elsewhere)
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
	state->fe.demodulator_priv  = state;
	state->nr = nr;
	state->newTP = false;
	state->bit_rate  = 0;
	state->loops = 15;
	//state->current_max_llr_rate = 0;
	//state->current_llr_rate = 0;
	//error |= fe_stid135_set_maxllr_rate(state, 90);

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

static int stid135_module_init(void)
{
	fe_stid135_modcod_flt_reg_init();
	return 0;
}

static void stid135_module_exit(void)
{
	return;
}


EXPORT_SYMBOL_GPL(stid135_attach);

module_init(stid135_module_init);
module_exit(stid135_module_exit);
MODULE_DESCRIPTION("STiD135 driver");
MODULE_AUTHOR("CrazyCat");
MODULE_LICENSE("GPL");
