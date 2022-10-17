/*
 * This file is part of STiD135 OXFORD LLA
 *
 * Copyright (c) <2020>-<2022>, Deep Thought <deeptho@gmail.com>:
 *     Make this code actually work in the linux drivers
 *     Combine results in overall spectrum
 *     Interface with dvbapi code, and making it interruptible
 *     Optimize for speed
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
	//do not treat overflow specially. Rescaling will apply
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DISABLE_RESCALE(path), 0);
	//do not remove mean value to disable DC coefficient (bad idea?)
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
	//read power spectral density instead of fft result; value 1 would read complex samples?
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_SEL_MEM(path), 0);


	//format will be 32 bit, not 16 bit
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(path), !!fft_mode32);
	dprintk("Selecting %d bit fft mode\n", !!fft_mode32 ? 32 : 16);

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


/*
	Read a spectrum from the internal ram, by copying it into registers and then reading those
 */

static fe_lla_error_t fe_stid135_read_psd_mem(struct stv* state, s32* rf_level,
																			 s32 start_idx, s32 end_idx, bool mode_32bit, s32 fft_size)
{
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	enum fe_stid135_demod path = state->nr+1;
	s32 idx, line;
	s32 val[4]= { 0 }, val_max;
	s32 memstat=0;
	u32 exp;
	s32 fld_value;
	int timeout = 0;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	int log_samples_per_line;
	s32 start_line;
	s32 end_line ;
#ifdef	DEBUG_TIME
	ktime_t start_time = ktime_get();
#endif

	if(path > FE_SAT_DEMOD_4)
		return(FE_LLA_BAD_PARAMETER);

	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(path), &fld_value);
	dprintk("FFT read_psd_mem path=%d mode_32bit=%d fft_size=%d exp=%d\n", path, mode_32bit, fft_size, exp);

	exp = (u32)fld_value;
#ifdef DEBUG_TIME
	{
		ktime_t now = ktime_get();
		ktime_t delta = ktime_sub(now, start_time);
		start_time = now;
		//dprintk("init took %lldms\n", delta/1000000);
		//init took 0ms
	}
#endif
	log_samples_per_line = (mode_32bit ? 2: 3);
	start_line = (fft_size -1 - (end_idx -1)) >> log_samples_per_line;
	end_line = (fft_size -1 - (start_idx - 1) + (1 << log_samples_per_line) -1 ) >> log_samples_per_line;

	/*8*start_line and 8*end_line index a line of internal memory containing 16 bytes of data
		all lines from start_line to end_line-1 must be read

		Note that spectrum data is stored in reverse order in memory
		The loop over lines below starts at the end of the spectrum and ends at the beginning

		idx is the index of a sample in internal memory  (stored as 4 byte or 2 byte integer)
		fft_size -1 - idx is the corresponding index in the output array (4 byte per sample)
		line points to memory line in which idx is located
	 */
	for(idx = fft_size -1 - (end_idx -1), line=start_line; line < end_line; ++line) {
		s32 start_register = idx - (line << log_samples_per_line);
		s32 end_register;
		s32 num_registers = (1 << log_samples_per_line) - start_register;
		s32 reg;
		if (num_registers  > end_idx - idx)
			num_registers = end_idx - idx; //can happen at end of range
		end_register = start_register + num_registers;

		if (idx  > fft_size -1 - start_idx) {
			dprintk("idx out of range: idx=%d\n", idx);
			break;
		}
#if 0
		dprintk("reading line=%d idx=%d o_idx=%d reg=%d-%d\n", line, idx, fft_size -1 -idx, start_register, end_register);
		/*Request read of 16 bytes from PSD memory into 8 2-byte registers (4 samples of 4 bytes each in 32 bit mode;
			8 samples of 2 bytes in 126 mode
		)
		*/
#endif
		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(path),
															 (line>>8) & 0x03);
		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(path),
															 (line & 0xff));
		error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(path), 2);

		// wait for end of transfer
		error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(path), &memstat);
		for (timeout = 0; (!memstat) && (timeout < 20); ++timeout) {
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(path), &memstat);
		 mutex_unlock(&state->base->status_lock);
			ChipWaitOrAbort(pParams->handle_demod, 1);
			mutex_lock(&state->base->status_lock);
		}

		if(timeout == 20)
			return FE_LLA_NODATA;

		error |= ChipGetRegisters(pParams->handle_demod, (u16)(REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(path) + (4 * 0)), 16);
		//Reads VA01, VA0i, VA11, VA10; accessed from cache below for jjj=0
		//Reads VA21, VA21, VA31, VA30 accessed from caDXche below for jjj=2
		//Reads VA31, VA41, VA51, VA50 accessed from cache below for jjj=3
		//Reads VA61, VA61, VA71, VA70 accessed from cache below for jjj=4
		for(reg = start_register; reg < end_register; ++reg, idx++) {
			//read one sample
			//Read 2 16-bit registers at a time, corresponding to 1 32-bit sample
			if ( mode_32bit) {
				val[3] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(path)
																	 + ((4 * reg) << 16));
				val[2] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(path)
																	 + ((4 * reg) << 16));
				val[1] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA11_MEM_VAL1(path)
																	 + ((4 * reg) << 16));
				val[0] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA10_MEM_VAL1(path)
																	 + ((4 * reg) << 16));
				//PSD=val*3/64 dB
				val_max = ((((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * 3000*XtoPowerY(2, exp))/64;
			} else { // 16-bit mode
				val[3] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(path)
																	 + ((2 * reg) << 16)); // 16 = address bus width for demod
				val[2] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(path)
																	 + ((2 * reg) << 16));
				val_max = (((val[3] << 8) + (val[2])) * 3000*XtoPowerY(2, exp))/64;
			}
			if (idx <0 || idx > fft_size -1) {
				dprintk("index out of range: idx=%d\n", idx);
			} else {
				rf_level[fft_size -1 - idx ] = val_max; 	// fill the table back to front
			}
		}
	}
	// Calculate beginning of data table
	return error;
}



/*****************************************************
--FUNCTION	::	fe_stid135_fft
--ACTION	::	Do an FFT with the hardware cell of the chip
--PARAMS IN	::	Handle -> Front End Handle
			Path -> number of the demod
			Mode -> 1/2/3/4/5 (modified the number of points of the fft)
			Nb_acquisition -> number of acquisition for an fft ( modified the precision of the fft)
			Frq -> Center frequency in Hz
			Range -> Bandwith (difference between highest and lowest frequency capture by fft) in Hz
			Tab[] -> empty table of data
	only data in [start_idx, end_idx[ will actually be retrieved

--PARAMS OUT	::	Tab[] -> table of fft return data
			Begin -> first value on table of data
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_fft(struct stv* state, u32 mode, u32 nb_acquisition, s32 freq,
															u32 range, s32* rf_level, s32 buffer_size,
															s32 start_idx, s32 end_idx)
{
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	enum fe_stid135_demod path = state->nr+1;
	s32 f;
	s32 contmode =0;
	s32 fld_value;
	s32 fft_size;
	int timeout = 0;
	bool mode_32bit = false;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	const u16 cfr_factor = 6711; // 6711 = 2^20/(10^6/2^6)*100
#ifdef	DEBUG_TIME
	ktime_t start_time = ktime_get();
#endif
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
	base_unlock(state);
#ifdef	DEBUG_TIME
	{
	ktime_t now = ktime_get();
	ktime_t delta = ktime_sub(now, start_time);
	start_time = now;
	dprintk("preparing took %lldus\n", delta/1000);
	//preparing took 2514us
	}
#endif
	WAIT_N_MS(5);
	base_lock(state);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_RESTART(path), 0);

	// wait for fft to finish if not in continuous mode
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GAINCONT_MODE_CONTINUOUS(path), &fld_value);
	if (!fld_value) {
		error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GSTAT_PSD_DONE(path), &contmode);
		for(timeout=0; !contmode && (timeout < 40); ++timeout){
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GSTAT_PSD_DONE(path), &contmode);
			mutex_unlock(&state->base->status_lock);
			ChipWaitOrAbort(pParams->handle_demod, 1);
			mutex_lock(&state->base->status_lock);
		}
	}
#ifdef	DEBUG_TIME
	{
		ktime_t now = ktime_get();
		ktime_t delta = ktime_sub(now, start_time);
		start_time = now;
		dprintk("waiting for fft to finish took %lldms\n", delta/1000000);
		//waiting for fft to finish took 31ms (including the initial 5ms sleep)
	}
#endif
	fft_size = 1<< (13-mode);
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(path), &fld_value);
	if(fld_value == 1) {
		//ask to read memory in terms if 32bit words
		mode_32bit = true;
	} else if(fld_value == 0) { // 16-bit mode
		//ask to read memory in terms if 16bit words
		mode_32bit = false;
	}
	error |= fe_stid135_read_psd_mem(state, rf_level,  start_idx, end_idx, mode_32bit, fft_size);
#ifdef	DEBUG_TIME
	{
		ktime_t now = ktime_get();
		ktime_t delta = ktime_sub(now, start_time);
		start_time = now;
		dprintk("read_psd_mem took %lldms\n", delta/1000000);
		//waiting for fft to finish took 31ms (including the initial 5ms sleep)
	}
#endif
	// Empty hardware fft
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(path), 0x00);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(path), 0x00);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(path), 2);
	//dprintk("FFT sleep down fft\n");
	// Sleep down hardware fft
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_RESTART(path), 1);
	return error;
}


/*
	Input:
	  center_freq in kHz
	  range in kHz: bandwith of the fft scan (bin index 0 corresponds to center_freq -range/2,
	                bin_index fft_size -1 corresponds to center_freq + range/2)
    fft_size: length of fft; power of 2 (typically 512)
    mode: integer such that fft_size == 1 << (13-mode)
	  start_idx, end_idx
	     only data for frequencies with bin index in [start_idx, end_idx[ will actually be retrieved
		   other returned values will be untouched. Frequency components are ordered w.r.t. increasing frequency
    lo_frequency_hz in Hz: frequency of the localk oscillator 1550000kHz
       the chip itself refers tuning frequencies to this value. A physical frequency of 1.55Ghz corresponds
		   to a chip frequency of 0
	  pbandx1000: average band power deduced from RF AGC
	Output:
	  freq[fft_size]: frequencies of spectral components
	  rf_level[fft_size]: strenngth of spectral components in millidB
 */
static int stid135_get_spectrum_scan_fft_one_band(struct stv *state,
																									s32 center_freq, s32 range,
																									s32 lo_frequency_hz,
																									int fft_size, int mode, s32 pbandx1000,
																									s32 start_idx, s32 end_idx,
																									u32* freq,
																									s32* rf_level
																									)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	s32 Pbandx1000;
	u32 nb_acquisition = 255; //average 1 samples
	int i;
	int a=3;
	s32 delta;
#if 1
	vprintk("spectrum scan: center_freq=%dkHz lo=%dkHz range=%dkHz mode=%d fft_size=%d freq=%p rf_level=%p\n",
					center_freq, lo_frequency_hz/1000, range, mode, fft_size, freq, rf_level);
#endif
	base_lock(state);
	error = fe_stid135_fft(state, mode, nb_acquisition,
												 center_freq*1000 - lo_frequency_hz, range*1000, rf_level, fft_size,
												 start_idx, end_idx);
	base_unlock(state);
	pbandx1000 += mode*6020; //compensate for FFT number of bins: 20*log10(2)
	delta = 10*(3000 + STLog10(range) - STLog10(fft_size)); //this is a power spectral density range*1000/fft_size is the bin width in Hz

	for(i=0; i< fft_size; ++i) {
		s32 f = ((i-(signed)fft_size/2)*range)/(signed)fft_size +center_freq; //in kHz
		s32 correction1000 = (s32)(f/1000  - 1550);

		freq[i]= f;

		correction1000 = (s32)(correction1000*1000/600);
		correction1000 = (s32) (correction1000 * correction1000/1000);
		rf_level[i] += pbandx1000 - delta +correction1000;
	}

	if(fft_size/2-a<0 || fft_size/2+a>= fft_size)
		dprintk("BUG!!!! a=%d fft_size=%d\n", a, fft_size);
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
	s32 useable_samples2;
	s32 lost_samples2;
	s32 min_correction[2];
	s32 max_correction[2];
	s32 sum_len;
	s32 sum_correction;
#ifdef	DEBUG_TIME
	ktime_t start_time;
#endif
	s32 start_idx, end_idx;
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
	/*compute mode such that fft_size <= 1 << (13-mode)
		So 1 << (13-mode) is the smallest power of 2 larger than or equal to fft_size
	*/
	for(mode=1;mode<=5; ++mode) {
		table_size >>=1; // 1<<(13-mode)
		if(table_size <= ss->fft_size)
			break;
	}
	if(ss->fft_size != table_size) {
		dprintk("Error: fft_size should be power of 2: changing from %d to %d\n", table_size, ss->fft_size);
	}
	ss->fft_size = table_size;

	base_lock(state);
	error = FE_STiD135_GetLoFreqHz(&state->base->ip, &ss->lo_frequency_hz);
	base_unlock(state);
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
	base_lock(state);
	print_spectrum_scan_state(&state->spectrum_scan_state);
#ifdef	DEBUG_TIME
	start_time = ktime_get();
#endif
	error = fe_stid135_init_fft(state, mode, Reg);
#ifdef DEBUG_TIME
	{
	ktime_t now = ktime_get();
	ktime_t delta = ktime_sub(now, start_time);
	start_time = now;
	dprintk("stid135_init took %lldus\n", delta/1000);
	//preparing took 2514us
	}
#endif

	error |= estimate_band_power_demod_for_fft(state, state->rf_in+1, &pbandx1000);
	base_unlock(state);

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
			goto _end;
			break;
		}
		start_idx = lost_samples2 -5;
		end_idx = ss->fft_size-lost_samples2 +5;
		error = stid135_get_spectrum_scan_fft_one_band(state, center_freq, ss->range,
																									 ss->lo_frequency_hz,
																									 ss->fft_size, mode, pbandx1000,
																									 start_idx, end_idx, temp_freq, temp_rf_level);
		if(error) {
			dprintk("Error=%d\n", error);
			goto _end;
		}
		/* correct spectrum to compensate for discontinuities from band to band, which may arise
			 due to differences in AGC and knowing AGC onlt with limited precision.
			 The current band is corrected such that the average spectral value in its low frequency region
			 equals the average spectral value of the high frequency range of the last (lower in frequency) band
		*/
		current_avg =0;
		for(i=lost_samples2-5 ; i < lost_samples2+5 ; ++i)
			current_avg += temp_rf_level[i];
		current_avg /= 10;
		correction = (idx==0) ? 0  : -(current_avg - last_avg);
		for(i= lost_samples2; i< ss->fft_size-lost_samples2 && idx < ss->spectrum_len; ++idx, i++ ) {
			ss->freq[idx]= temp_freq[i];
			ss->spectrum[idx] = temp_rf_level[i] + correction;
		}

		/* compute the mimimal correction applied over all bands, and also the second to lowest correction*/
		if (correction < min_correction[0]) {
			min_correction[1] = min_correction[0];
			min_correction[0] = correction;
		} else if (correction < min_correction[1]) {
			min_correction[1] = correction;
		}

		/* compute the maximal correction applied over all bands, and also the second to highest correction*/
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
		/* compute the average correction applied, but exclude the two highest and the two lowest
			 corrections, to reject outliers (most relevant on m88rs6060 based tbs cards where such
			 outliers actually occur
		*/
		sum_correction  -= min_correction[0] + min_correction[1] + max_correction[0] + max_correction[1];
		correction = sum_correction / (sum_len-4);

		/*
			subtract the average correction so that the overall correction is unbiased.
		 */
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
	base_lock(state);
	error |= (error1=fe_stid135_term_fft(state, Reg));
	base_unlock(state);
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

	if (!ss->spectrum || ! ss->spectrum_present) {
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
