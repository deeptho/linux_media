/*
 * This file is part of STiD135 OXFORD LLA
 *
 * Copyright (c) <2020>-<2021>, Deep Thought <deeptho@gmail.com> Make this code actually work in the linux drivers
 * Copyright (c) <2014>-<2018>, STMicroelectronics - All Rights Reserved
 * Author(s): Mathias Hilaire (mathias.hilaire@st.com), Thierry Delahaye (thierry.delahaye@st.com) for STMicroelectronics.
 *
 * License terms: BSD 3-clause "New" or "Revised" License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/kernel.h>
#include <linux/kthread.h>
#include "stid135_drv.h"
#include "stid135_init.h"
#include "stid135-fft.h"
#include "stid135_initLLA_cut2.h"
#include "c8codew_addr_map.h"
#include "stid135_addr_map.h"


#define DmdLock_TIMEOUT_LIMIT      5500  // Fixed issue BZ#86598
//#define BLIND_SEARCH_AGC2BANDWIDTH  40
#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

extern int stid135_verbose;
#define vprintk(fmt, arg...)																						\
	if(stid135_verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

/*****************************************************
--FUNCTION	::	fe_stid135_fft_save_registers
--ACTION	::	save registers prior to callling FFT
--PARAMS IN	::	Handle -> Front End Handle
			Path -> number of the demod
			Reg[] -> empty array of data
--PARAMS OUT	::	Reg[] -> table of stored registers
--RETURN	::	Error (if any)
--***************************************************/
static fe_lla_error_t fe_stid135_fft_save_registers(struct stv* state, FE_OXFORD_TunerPath_t tuner_nb, s32 Reg[60])
{
	u32 i=0, reg_value;
	s32 fld_value;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	enum fe_stid135_demod path = state->nr+1;

	dprintk("Starting to store params path=%d tuner_nb=%d\n", path, tuner_nb);
	/* store param */
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_FIFO2_ULTRABS(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetOneRegister(pParams->handle_demod, REG_RC8CODEW_DVBSX_DEMOD_CARCFG(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDISTATE_I2C_DEMOD_MODE(path), &fld_value) ;
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(path), &fld_value);
	Reg[i++] = fld_value;

	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1ADJ_MANUAL(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1_ADJUSTED(path), &reg_value);
	Reg[i++] = reg_value;

#if 0
	error |= ChipGetField(pParams->handle_demod,  FLD_FSTID135_AFE_AFE_AGC2_CTRL_AGC2_BB_CTRL+	DVBSX_DEMOD[path-1], &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetField(pParams->handle_demod,  FLD_FSTID135_AFE_AFE_AGC2_CTRL_AGC2_RF_CTRL+	DVBSX_DEMOD[path-1], &reg_value);
	Reg[i++] = reg_value;
#endif


	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2ADJ_MANUAL(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2REF_XXDB(path), &reg_value);
	Reg[i++] = reg_value;


	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGM(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGL(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GCTRL(path), &reg_value);
	Reg[i++] = reg_value;


	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_UPDCONT_UPD_CONTINUOUS(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FFTCTRL(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GAINCONT(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_THRESHOLD(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEBUG1(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2O(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I0(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_AGCRF_AGCRFCFG_AGCRF_BETA(tuner_nb), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_AMM_FROZEN(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_QUAD_FROZEN(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFREF(tuner_nb), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CN_AGCIQ_BETA(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1AMM(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1QUAD(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGCKS(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCNLK(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCLK(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCLK(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1BCFG(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B2(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B1(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B0(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGCFG(path), &reg_value);
	Reg[i++] = reg_value;
#if 1
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCNLK(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCLK(path), &reg_value);
	Reg[i++] = reg_value;
#endif
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG1(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG0(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(path), &reg_value);
	Reg[i++] = reg_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR22(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR21(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR20(path), &reg_value);
	Reg[i++] = reg_value;


	dprintk("Saved %d registers\n", i);
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_init_fft
--ACTION	::	Init demod for FFT and store registers states before modification
--PARAMS IN	::	Handle -> Front End Handle
			Path -> number of the demod
			Reg[] -> empty array of data
--PARAMS OUT	::	Reg[] -> table of stored registers
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_init_fft(struct stv*state, int fft_mode, s32 Reg[60])
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	enum fe_stid135_demod path = state->nr+1;

	error |= fe_stid135_fft_save_registers(state, state->rf_in+1, Reg);
	/* ADJUSTEMENT REGISTERS BEFORE FFT  */

	/* Choix du mode de fonctionnement de la cellule FFT */
	//allow ultra blind scan\ to use FIFO2 RAM
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_FIFO2_ULTRABS(path),  0x01);

	//select narrow band setting, i.e, up to  Mclk/2, which is about 60Msamples/second
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(path), 0x01);

	//disable frequency scanning (essential!)
	error |= ChipSetOneRegister(pParams->handle_demod, REG_RC8CODEW_DVBSX_DEMOD_CARCFG(path), 0xc6);

	// UFBS block only connected to narrow-band demod, not wideband, so HDEBITCFG2/MODE_HAUTDEBIT = 01
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDISTATE_I2C_DEMOD_MODE(path), 0x02); /* set PEA to UFBS mode  */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(path), 0x00);

	/* Agc blocked */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1ADJ_MANUAL(path), 1);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2ADJ_MANUAL(path), 1);
#if 0
	error |= ChipSetField(pParams->handle_demod,  FLD_FSTID135_AFE_AFE_AGC2_CTRL_AGC2_BB_CTRL +	DVBSX_DEMOD[path-1], 1); //max gain
	error |= ChipSetField(pParams->handle_demod,  FLD_FSTID135_AFE_AFE_AGC2_CTRL_AGC2_RF_CTRL +	DVBSX_DEMOD[path-1], 1); //max gain
#endif
	//error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2O_AGC2_COEF(path), 0);0

	//configure decimation filters
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGM_FILTRE1_MANUEL(path), 1);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGM_FILTRE2_MANUEL(path), 1);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGM_FILTRE3_MANUEL(path), 1);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGM(path),1);

	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGL_FILTRE1_AMPL(path), 0);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGL_FILTRE2_AMPL(path), 0);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGL_FILTRE3_AMPL(path), 0);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGL(path),1);

	/* fft reading registers adjustment */
	//request reading symbols/intersymbols
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_INPUT_MODE(path), 0);
	//Low rate permits acquisition during computation, it is disabled;
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_LOW_RATE(path), 0);
	//store the result
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GCTRL(path), 1);

#if 0
	//select continuous update of spectrum after 2 scans
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_UPDCONT_UPD_CONTINUOUS(path), 2);
#else
		//select continuous update of spectrum after 2 scans
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_UPDCONT_UPD_CONTINUOUS(path), 0);
#endif
	// Request 256 samples; max would be 4096 samples: note: the actual call will later change this value
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_MODE(path), fft_mode);

	//negative frequencies are stored at lowest index (matlab fftshift)
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_SHIFT(path), 1);
	//disable CTE mode
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_CTE_MODE(path), 0);  	// Off
	//do not use "intersymbols", which limits fft size to 4096
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_MODE_INTERSYMBOL(path), 0);  // Off
	//Store the result
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FFTCTRL(path),1);

	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GAINCONT_GAIN_CONTINUOUS(path), 2);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GAINCONT_MODE_CONTINUOUS(path), 0);	// Off
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GAINCONT(path),1);

	//set a threshold to detect "peak found"
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_THRESHOLD_MAX_THRESHOLD(path), 4);
	//FFT continuous if a good peak was found, the alternative setting would be good for blindscan
#if 0
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_THRESHOLD_NO_STOP(path), 0);		// Off
#else
		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_THRESHOLD_NO_STOP(path), 1);		// Off
#endif
	//Store the result
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_THRESHOLD(path),1);

#if 1
	//do not treat overflow specially
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DISABLE_RESCALE(path), 0);
	//do not remove mean value to disable DC coefficient
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DISABLE_AVERAGE(path), 0);
#else
		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DISABLE_RESCALE(path), 1);
	//do not remove mean value to disable DC coefficient
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DISABLE_AVERAGE(path), 1);
	#endif

	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DEBUG_INTERSYMBOL(path), 0);	// Off
#if 1
	//spectrum will be in log format: (PSD=val*3/64 dB) (unsigned)
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_DB(path), 1);
#else
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_DB(path), 0);
#endif
	//read power spectral density instead of fft result
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_SEL_MEM(path), 0);

#if 1
	//format will be 32 bit, not 16 bit
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(path), 1);
#else
		//format will be 16 bit, not 32 bit
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(path), 0);
#endif
	//do not discards the values for which CTEHT=0. (unsigned)
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_CFO_FILT(path), 0);
	//store the result
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEBUG1(path),1);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1_ADJUSTED(path), 88);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2REF_XXDB(path), 56);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2O(path), 0x0);

	if((path == FE_SAT_DEMOD_1) || (path == FE_SAT_DEMOD_3))
		error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(path), 0x40);
	else
		error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(path), 0x68);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I0(path), 0x42);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_AGCRF_AGCRFCFG_AGCRF_BETA(state->rf_in+1), 0);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_AMM_FROZEN(path), 1);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_QUAD_FROZEN(path), 1);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFREF(state->rf_in+1), 0x58);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CN_AGCIQ_BETA(path), 0);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1AMM(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1QUAD(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGCKS(path), 0x02);


	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCNLK(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCLK(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCLK(path), 0x00);

	//loop carrier 1 coefficients
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(path), 0x00);

	//stop derotator
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1BCFG(path), 0x00);

	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFR1B2_CFR1B_VALUE(path), 0x00);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFR1B1_CFR1B_VALUE(path), 0x00);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFR1B0_CFR1B_VALUE(path), 0x00);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B2(path), 3);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGCFG(path), 0xc3);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCNLK(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCLK(path), 0x00);

	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TMGREG2_TMGREG(path), 0x00);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TMGREG1_TMGREG(path), 0x00);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TMGREG0_TMGREG(path), 0x00);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(path), 3);

	//carrier offset from register1 to register2 disabled
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(path), 0x00);

	//set carrier frequency offset to 0
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFR22_CAR2_FREQ(path), 0x00);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFR21_CAR2_FREQ(path), 0x00);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFR20_CAR2_FREQ(path), 0x00);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR22(path), 3);
 	WAIT_N_MS(200);		  /* Registers modification period */
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_term_fft
--ACTION	::	Reset demod as it was before fft
--PARAMS IN	::	Handle -> Front End Handle
			Path -> number of the demod
			Reg[] -> table of stored registers
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_term_fft(struct stv* state, s32 Reg[60])
{
	u32 i=0;

	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	enum fe_stid135_demod path = state->nr+1;

	/* Restore params */
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_FIFO2_ULTRABS(path), Reg[i++]);
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(path), Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, REG_RC8CODEW_DVBSX_DEMOD_CARCFG(path), Reg[i++]);

	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDISTATE_I2C_DEMOD_MODE(path), Reg[i++]);
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(path), Reg[i++]);

	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1ADJ_MANUAL(path), Reg[i++]);
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2ADJ_MANUAL(path), Reg[i++]);
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1_ADJUSTED(path), Reg[i++]);

#if 0
	error |= ChipSetField(pParams->handle_demod,  FLD_FSTID135_AFE_AFE_AGC2_CTRL_AGC2_BB_CTRL+	DVBSX_DEMOD[path-1], Reg[i++]);
	error |= ChipSetField(pParams->handle_demod,  FLD_FSTID135_AFE_AFE_AGC2_CTRL_AGC2_RF_CTRL+	DVBSX_DEMOD[path-1], Reg[i++]);
#endif

	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2REF_XXDB(path), Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGM(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGL(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GCTRL(path), (u32)Reg[i++]);
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_UPDCONT_UPD_CONTINUOUS(path), Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FFTCTRL(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GAINCONT(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_THRESHOLD(path), (u32)Reg[i++]);
 	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEBUG1(path), (u32)Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2O(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I0(path), (u32)Reg[i++]);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_AGCRF_AGCRFCFG_AGCRF_BETA(state->rf_in+1), (u32)Reg[i++]);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_AMM_FROZEN(path), (u32)Reg[i++]);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_QUAD_FROZEN(path), (u32)Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFREF(state->rf_in+1), (u32)Reg[i++]);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CN_AGCIQ_BETA(path), (u32)Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1AMM(path), (u32) Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1QUAD(path), (u32) Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGCKS(path), (u32) Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCNLK(path), (u32) Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCLK(path), (u32) Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCLK(path), (u32) Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1BCFG(path), (u32)Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B2(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B1(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B0(path), (u32)Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGCFG(path), (u32)Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCNLK(path), (u32)Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCLK(path), (u32)Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG1(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG0(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(path), (u32)Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR22(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR21(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR20(path), (u32)Reg[i++]);


	dprintk("Restored %d registers\n", i);
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_fft
--ACTION	::	Do an FFT with the hardware cell of the chip
--PARAMS IN	::	Handle -> Front End Handle
			Path -> number of the demod
			Mode -> 1/2/3/4/5 (modified the number of points of the fft)
			Nb_acquisition -> number of acquisition for an fft ( modified the precision of the fft)
			Frq -> frequency in Hz
			Range (in Hz)
			Tab[] -> empty table of data
			Begin -> pointer on an empty u32
--PARAMS OUT	::	Tab[] -> table of fft return data
			Begin -> first value on table of data
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_fft(struct stv* state, u32 mode, u32 nb_acquisition, s32 freq,
															u32 range, u32 *tab, u32* begin, s32 buffer_size)
{
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	enum fe_stid135_demod path = state->nr+1;

	u32 nb_words =0;
	s32 val[4]= { 0 }, val_max;
	u32 guard=12345;
	u32 jjj=0, iii=0, bin=0;
	s32 f;
	s32 memstat=0, contmode =0;
	u32 nbr_pts =0;
	u32 exp;
	static u32 max_exponent[4] = {0,0,0,0};
	s32 fld_value;
	u8 timeout = 0;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	const u16 cfr_factor = 6711; // 6711 = 2^20/(10^6/2^6)*100
	dprintk("FFT start path=%d\n", path);
	if(path > FE_SAT_DEMOD_4)
		return(FE_LLA_BAD_PARAMETER);
	//pParams = (struct fe_stid135_internal_param *) handle;
	// Set the number of fft bins (4096 or lower)
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_MODE(path), (s32)mode);

	//set number of fft acquisitions to accumulate
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTACC_NB_ACC_FFT(path), (s32)nb_acquisition);

	// Configure demodulator in ultra-fast blind scan
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_ENABLE(path), 1);

	/* Set frequency, i.e., initial value for carrier offset;
		 cfr_init is in MHz = Mclk * cfr_init / 2^24 (signed)

		 Hypothesis: master_clock=130MHz, frequency_hz=-550MHz..+650MHz
		 (-550=950-1500 and 650=2150-1500)
		 FVCO/4/12=6.2GHz/4/12=130MHz


		 So:
		 frequency set in MHz
		 = Mclk * ((freq/PLL_FVCO_FREQUENCY)*cfr_factor)/100 /2^24
		 =129.16 * ((freq/6200)*2^20/(10^6/2^6)*100)/100/2^24
		 =129.16 * ((freq/6200)/(10^6/2^6))/2^4
		 =129.16 * ((freq/6200)/(10^6/2^2))
		 =129.16 * ((freq/6200)/(10^6/2^2))

		 Does not makes sense: computatation yields freq/12....
	*/
	f = ((freq/PLL_FVCO_FREQUENCY)*cfr_factor)/100;

	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT2_CFR_INIT(path),
														 ((u8)(f >> 16)));
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT1_CFR_INIT(path),
														 ((u8)(f >> 8)));
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT0_CFR_INIT(path),
														 ((u8)f));
	error |= ChipSetRegisters(pParams->handle_demod, REG_RC8CODEW_DVBSX_DEMOD_CFRINIT2(path),3);

	// Set bandwidth (symbol rate)
	error |= fe_stid135_set_symbol_rate(state, range);


	// start acquisition
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_RESTART(path), 1);

	WAIT_N_MS(5);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_RESTART(path), 0);

	// wait for process done if not in continous mode
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GAINCONT_MODE_CONTINUOUS(path), &fld_value);
	if (!fld_value) {
		error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GSTAT_PSD_DONE(path), &contmode);
		while((contmode != TRUE) && (timeout < 40)){
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GSTAT_PSD_DONE(path), &contmode);
			timeout = (u8)(timeout + 1);
			ChipWaitOrAbort(pParams->handle_demod, 1);
		}
	}
	//dprintk("FFT calculate memory readout range\n");
	// calculate memory readout range
	nbr_pts = (u32)(8192 / XtoPowerY(2, (u32) mode));

	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(path), &fld_value);
	if(fld_value == 1) { // 32-bit mode
		nb_words = ((u32)1<<(10-mode+1));     // memory size N/8 for memory readout
	} else if(fld_value == 0) { // 16-bit mode
		nb_words = ((u32)1<<(10-mode));     // memory size N/8 for memory readout
	}
	//nb_words = ((u32)1<<(10-mode));     // memory size N/8 for memory readout
	//dprintk("mode=%d buffer_size=%d nbr_pts=%d nb_words=%d 32bit=%d\n", mode, buffer_size, nbr_pts, nb_words, fld_value == 1);
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(path), &fld_value);
	dprintk("error=%d\n", error);
	exp = (u32)fld_value;
	switch(path) {
	case FE_SAT_DEMOD_1:
		if(exp > max_exponent[0])
			max_exponent[0] = exp;
		break;
	case FE_SAT_DEMOD_2:
		if(exp > max_exponent[1])
			max_exponent[1] = exp;
		break;
	case FE_SAT_DEMOD_3:
		if(exp > max_exponent[2])
			max_exponent[2] = exp;
		break;
	case FE_SAT_DEMOD_4:
		if(exp > max_exponent[3])
			max_exponent[3] = exp;
		break;
	default:
		//Only first 4 demods supported by hardware
		break;
	}

	for (iii=0; iii<nb_words; iii++)	{   // set the FFT memory address in multiples of words
#if 0
		WAIT_N_MS(5);
#else
		//WAIT_N_MS(1);
#endif

		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(path), (iii>>8) & 0x03);
		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(path), (iii & 0xff));
		error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(path), 2);
		//dprintk("FFT %d/%d\n", i, nb_words);
		// wait for end of transfer
		error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(path), &memstat);
		while ((!memstat) && (timeout < 50)) {
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(path), &memstat);
			timeout = (u8)(timeout + 1);
			ChipWaitOrAbort(pParams->handle_demod, 1);
		}
		if(timeout == 50)
			return(FE_LLA_NODATA);
		//dprintk("FFT %d/%d end of transfer\n", i, nb_words);
		// read & store data to create an fft list

		error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(path), &fld_value);
		if(fld_value==0)
			dprintk("BUG!!!! 16 bit mode guard=%d\n", guard);
		if(fld_value == 1) { // 32-bit mode
			for (jjj=0; jjj<4; jjj++) {
				error |= ChipGetRegisters(pParams->handle_demod, (u16)(REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(path) + (4 * jjj)), 4);
				val[3] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(path) + ((4 * jjj) << 16)); // 16 = address bus width for demod
				val[2] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(path) + ((4 * jjj) << 16));
				val[1] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA11_MEM_VAL1(path) + ((4 * jjj) << 16));
				val[0] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA10_MEM_VAL1(path) + ((4 * jjj) << 16));
#if 0
				val_max = STLog10(((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0])
					+3010* exp;
#else
				//PSD=val*3/64 dB
				val_max = ((((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * 3000*XtoPowerY(2, exp))/64;
#endif
				bin = (u32)(iii*4+jjj);
				if(nbr_pts-bin-1 < 0 || nbr_pts-bin-1 >= buffer_size)
					dprintk("BUG!!!! nbr_pts-bin-1=%d max=%d i=%d j=%d guard=%d\n", nbr_pts-bin-1, buffer_size, iii,jjj,
									guard);
				tab[nbr_pts-bin-1] = (u32)val_max; 	// fill the table back to front
			}
		} else if(fld_value == 0) { // 16-bit mode
			// read temporary memory
			for (jjj=0; jjj<8; jjj++) {
				error |= ChipGetRegisters(pParams->handle_demod, (u16)(REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(path) + (2 * jjj)), 2);
				val[3] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(path) + ((2 * jjj) << 16)); // 16 = address bus width for demod
				val[2] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(path) + ((2 * jjj) << 16));
#if 0
				val_max = STLog10((val[3] << 8) + val[2]) + 3010*exp;
#else
					val_max = (((val[3] << 8) + val[2]) * 3000*XtoPowerY(2, exp))/64;
#endif
				bin = (u32)(iii*8+jjj);
				if(nbr_pts - bin-1 < 0 || bin >= buffer_size)
					dprintk("BUG!!!! bin=%d max=%d\n", bin, buffer_size);
				tab[nbr_pts-bin-1] = (u32)val_max; 	// fil the table back to front; unit is 0.001dB
			}
		}
	}
	//dprintk("FFT empty hardware fft\n");
	// Empty hardware fft
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(path), 0x00);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(path), 0x00);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(path), 2);
	//dprintk("FFT sleep down fft\n");
	// Sleep down hardware fft
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_RESTART(path), 1);

	// Calculate beginning of data table
	*begin = (u32)(nbr_pts-1-bin);
	return(error);
}


/*
	center_freq and range in kHz
 */
static int stid135_get_spectrum_scan_fft_one_band(struct stv *state,
																									s32 center_freq, s32 range,
																									s32 lo_frequency_hz,
																									u32* freq,
																									s32* rf_level,
																									int fft_size, int mode, s32 pbandx1000, bool double_correction)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	s32 Pbandx1000;
	u32 nb_acquisition = 255; //average 1 samples
	u32 begin =0;
	int i;
	int a=3;
	s32 delta;
#if 1
	vprintk("spectrum scan: center_freq=%dkHz lo=%dkHz range=%dkHz mode=%d fft_size=%d freq=%p rf_level=%p\n",
					center_freq, lo_frequency_hz/1000, range, mode, fft_size, freq, rf_level);
#endif
	error = fe_stid135_fft(state, mode, nb_acquisition,
												 center_freq*1000 - lo_frequency_hz, range*1000, rf_level, &begin, fft_size);

	pbandx1000 += mode*6020; //compensate for FFT number of bins: 20*log10(2)
	delta = 10*(3000 + STLog10(range) - STLog10(fft_size)); //this is a power spectral density range*1000/fft_size is the bin width in Hz
	//dprintk("Pbandx1000=%d error=%d\n", Pbandx1000, error);

	for(i=0; i< fft_size; ++i) {
		s32 f = ((i-(signed)fft_size/2)*range)/(signed)fft_size +center_freq; //in kHz
		s32 correction1000 = (s32)(f/1000  - 1550);

		freq[i]= f;

		correction1000 = (s32)(correction1000*1000/600);
		correction1000 = (s32) (correction1000 * correction1000/1000);
		if(double_correction)
			correction1000 += correction1000;

		rf_level[i] += pbandx1000 - delta +correction1000;
	}

	if(fft_size/2-a<0 || fft_size/2+a>= fft_size)
		dprintk("BUG!!!! a=%d fft_size=%d\n", a, fft_size);
	if(begin==1)
		rf_level[0] = rf_level[1];
	for(i=-a+1; i<a; ++i)
		rf_level[fft_size/2+i] = (rf_level[fft_size/2-a] + rf_level[fft_size/2+a])/2;
	return error;
}


int get_spectrum_scan_fft(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct stv *state = fe->demodulator_priv;
	struct spectrum_scan_state* ss = &state->spectrum_scan_state;

	u32 table_size = 8192;
	s32 max_range= 60000;//96800;

	u32* temp_freq = NULL;
	s32* temp_rf_level = NULL;
	s32 Reg[60];
	s32 last_avg =0, current_avg =0, correction =0;

	fe_lla_error_t error = FE_LLA_NO_ERROR;
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;

	u32 idx=0;
	int i;
	int mode;
	s32 start_frequency = p->scan_start_frequency;
	s32 end_frequency = p->scan_end_frequency;
	//s32 start_idx;
	s32 pbandx1000;
	bool double_correction;
	s32 useable_samples2;
	s32 lost_samples2;
	s32 min_correction[2];
	s32 max_correction[2];
	s32 sum_len;
	s32 sum_correction;

	ss->spectrum_present = false;
	if(p->scan_end_frequency < p->scan_start_frequency) {
		vprintk("FULLY DONE\n");
		return -1; //fully done
	}
	ss->start_frequency = start_frequency;
	ss->end_frequency = end_frequency;

	ss->frequency_step = p->scan_resolution==0 ? 50 : p->scan_resolution; //in kHz

	ss->fft_size =  (p->scan_fft_size>0) ? p->scan_fft_size : 512;
	if(ss->fft_size >= 4096)
		ss->fft_size = 4096;
	for(mode=1;mode<=5; ++mode) {
		table_size >>=1;
		if(table_size <= ss->fft_size)
			break;
	}
	if(ss->fft_size != table_size) {
		dprintk("Error: fft_size should be power of 2\n");
		return -1;
	}
	ss->fft_size = table_size;

	error = FE_STiD135_GetLoFreqHz(&state->base->ip, &ss->lo_frequency_hz);
	if(error) {
		dprintk("FE_STiD135_GetLoFreqHz FAILED: error=%d\n", error);
		return error;
	}
	ss->lo_frequency_hz *=  1000000; //now in Hz


 	ss->range = ss->frequency_step * ss->fft_size; //in kHz

	if(ss->range >= max_range) {
		ss->frequency_step = max_range/ss->fft_size;
		dprintk("specified resolution (%dkHz) is too large; decreased to %dkHz\n",
						p->scan_resolution, ss->frequency_step);
		ss->range = ss->frequency_step * ss->fft_size; //in kHz
	}

	ss->spectrum_len = (end_frequency-start_frequency + ss->frequency_step-1)/ss->frequency_step;
	useable_samples2 = ((ss->fft_size*6)/10+1)/2;
	lost_samples2 = ss->fft_size/2 - useable_samples2;



	dprintk("demod: %d: tuner:%d range=[%d,%d]kHz resolution=%dkHz fft_size=%d num_freq=%d range=%dkHz lost=%d useable=%d\n",
					state->nr, state->rf_in,
					start_frequency, end_frequency, ss->frequency_step, ss->fft_size,
					ss->spectrum_len, ss->range, lost_samples2*2, useable_samples2*2);

	ss->freq = kzalloc(ss->spectrum_len * (sizeof(ss->freq[0])), GFP_KERNEL);
	//ss->candidates = kzalloc(ss->spectrum_len * (sizeof(ss->candidates[0])), GFP_KERNEL); //much too big
	ss->spectrum = kzalloc(ss->spectrum_len * (sizeof(ss->spectrum[0])), GFP_KERNEL);
	temp_freq = kzalloc(8192 * (sizeof(temp_freq[0])), GFP_KERNEL);
	temp_rf_level = kzalloc(8192 * (sizeof(temp_rf_level[0])), GFP_KERNEL);
	if (!temp_rf_level || !temp_freq || !ss->freq || !ss->spectrum) {
		dprintk("ERROR spectrum_len=%d\n", ss->spectrum_len);
		error = -ENOMEM;
		goto _end;
	}
	print_spectrum_scan_state(&state->spectrum_scan_state);
	error = fe_stid135_init_fft(state, mode, Reg);

	error |= estimate_band_power_demod_for_fft(state, state->rf_in+1, &pbandx1000, &double_correction);

	if(error) {
		dprintk("fe_stid135_init_fft FAILED: error=%d\n", error);
		return error;
	}

	min_correction[0] = 0x7fffffff;
	min_correction[1] = 0x7fffffff;
	max_correction[0] = 0x80000000;
	max_correction[1] = 0x80000000;
	sum_correction =0;
	sum_len =0;

	for(idx=0; idx < ss->spectrum_len;) {
		//the offset useable_samples2 ensures that the first usable index is aligned with start_frequency
		s32 center_freq = start_frequency + (idx + useable_samples2)* ss->frequency_step;
		if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
			//@todo:  should this be moved into  stid135_get_spectrum_scan_fft_one_band?
			dprintk("exiting on should stop\n");
			break;
		}
		error = stid135_get_spectrum_scan_fft_one_band(state, center_freq, ss->range,
																									 ss->lo_frequency_hz,
																									 temp_freq, temp_rf_level,
																									 ss->fft_size, mode, pbandx1000, double_correction);
		if(error) {
			dprintk("Error=%d\n", error);
			goto _end;
		}

		current_avg =0;
		for(i=lost_samples2-5 ; i < lost_samples2+5 ; ++i)
			current_avg += temp_rf_level[i];
		current_avg /= 10;
		correction = (idx==0) ? 0  : -(current_avg - last_avg);
		for(i= lost_samples2; i< ss->fft_size-lost_samples2 && idx < ss->spectrum_len; ++idx, i++ ) {
			ss->freq[idx]= temp_freq[i];
			ss->spectrum[idx] = temp_rf_level[i] + correction;
		}

		if (correction < min_correction[0]) {
			min_correction[1] = min_correction[0];
			min_correction[0] = correction;
		} else if (correction < min_correction[1]) {
			min_correction[1] = correction;
		}


		if (correction > max_correction[0]) {
			max_correction[1] = max_correction[0];
			max_correction[0] = correction;
		} else  if (correction > max_correction[1]) {
			max_correction[1] = correction;
		}
		sum_correction += correction;
		sum_len ++;

		last_avg =0;

		for(i=ss->fft_size-lost_samples2-5 ; i < ss->fft_size-lost_samples2+5 ; ++i)
			last_avg += temp_rf_level[i] + correction;
		last_avg /= 10;

	}

	if(sum_len > 4) {
		sum_correction  -= min_correction[0] + min_correction[1] + max_correction[0] + max_correction[1];

		correction = sum_correction / (sum_len-4);
		dprintk("Applying correction %d\n", correction);
		for(i=0; i < ss->spectrum_len; ++i) {
			ss->spectrum[i] -= correction;
		}
	}

	ss->spectrum_present = true;
 _end:
	dprintk("ending error=%d\n", error);
	if(temp_freq)
		kfree(temp_freq);
	if(temp_rf_level)
		kfree(temp_rf_level);
	dprintk("Freed temp variables\n");
	error |= (error1=fe_stid135_term_fft(state, Reg));
	dprintk("Terminated fft\n");
	if(error) {
		dprintk("fe_stid135_term_fft FAILED: error=%d\n", error1);
	}
	return error;
}





/*
	returns -1 when done or when error
	0 on success
 */

int stid135_spectral_scan_start(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	struct spectrum_scan_state* ss = &state->spectrum_scan_state;
	//	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	ss->scan_in_progress =true;
	ss->current_idx = 0;
	error = get_spectrum_scan_fft(fe);
	if(error)
		return error;

	if (!ss->spectrum) {
		dprintk("No spectrum\n");
		return -ENOMEM;
	}
	dprintk("Calling neumo_scan_spectrum\n");
	return neumo_scan_spectrum(ss);
}


int stid135_spectral_scan_next(struct dvb_frontend *fe,
															 s32 *frequency_ret, s32* symbolrate_ret)
{
	struct stv *state = fe->demodulator_priv;
	struct spectrum_scan_state* ss = &state->spectrum_scan_state;

	if(ss->current_idx < ss->num_candidates) {
		struct spectral_peak_t * peak =
			&ss->candidates[ss->current_idx++];
		*frequency_ret = peak->freq;
		*symbolrate_ret = peak->symbol_rate;
		dprintk("Next frequency to scan: %d/%d %dkHz\n", ss->current_idx -1, ss->num_candidates, *frequency_ret);
		return 0;
	} else {
		dprintk("Current subband fully scanned %d frequencies\n", ss->num_candidates);
	}
	return -1;
}



void print_spectrum_scan_state_(struct spectrum_scan_state* ss,
																const char* func, int line)
{
	printk(KERN_DEBUG
				 "%s:%d\n------------------------------------\n"
				 "in_progress=%d  idx_end=%d  spectrum_len=%d  current_idx=%d\n"
				 "mode=%d fft_size=%d threshold=%d\n"
				 "lo_frequency_hz=%dkHz  range=%dkHz frequency_step=%dkHz\n"
				 "-------------------------------\n",
				 func, line,
				 ss->scan_in_progress, ss->spectrum_len, ss->spectrum_len, ss->current_idx,
				 ss->mode, ss->fft_size, ss->threshold,
				 ss->lo_frequency_hz/1000, ss->range, ss->frequency_step);
};
