/*
* This file is part of STiD135 OXFORD LLA 
 *
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

#include "stid135_drv.h"
#include "stid135_init.h"
#include "stid135-fft.h"
#include "stid135_initLLA_cut2.h"
#include "c8codew_addr_map.h"
#include "stid135_addr_map.h"


#define DmdLock_TIMEOUT_LIMIT      5500  // Fixed issue BZ#86598
//#define BLIND_SEARCH_AGC2BANDWIDTH  40
#ifndef HOST_PC
	#define DEMOD_IQPOWER_THRESHOLD     10
#endif
#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)
 
/*****************************************************
--FUNCTION	::	fe_stid135_init_fft
--ACTION	::	Init demod for FFT and store registers states before modification
--PARAMS IN	::	Handle -> Front End Handle
			Path -> number of the demod
			Reg[] -> empty array of data
--PARAMS OUT	::	Reg[] -> table of stored registers	
--RETURN	::	Error (if any) 
--***************************************************/
fe_lla_error_t fe_stid135_init_fft(fe_stid135_handle_t handle, enum fe_stid135_demod path, FE_OXFORD_TunerPath_t tuner_nb, s32 Reg[60])
{
	u32 i=0, reg_value;
	s32 fld_value;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams;
	pParams = (struct fe_stid135_internal_param *) handle;

	/* store param */
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_ENABLE(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_FIFO2_ULTRABS(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDISTATE_I2C_DEMOD_MODE(path), &fld_value) ;
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(path), &fld_value);
	Reg[i++] = fld_value;

	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1ADJ_MANUAL(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2ADJ_MANUAL(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2O_AGC2_COEF(path), &fld_value);
	Reg[i++] = fld_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGM(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGL(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I0(path), &reg_value);
	Reg[i++] = reg_value;
	
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_INPUT_MODE(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_UPDCONT_UPD_CONTINUOUS(path), &fld_value);
	Reg[i++] = fld_value;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_LOW_RATE(path), &fld_value);
	Reg[i++] = fld_value;

	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FFTCTRL(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GAINCONT(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_THRESHOLD(path), &reg_value);
	Reg[i++] = reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEBUG1(path), &reg_value);
	Reg[i++] = reg_value;

	/* ADJUSTEMENT REGISTERS BEFORE FFT  */
	
	/* Choix du mode de fonctionnement de la cellule FFT */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_FIFO2_ULTRABS(path),  0x01);  /* Priority to UFBS scan */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(path), 0x01);  /* Narrowband setting */
	// UFBS block only connected to narrow-band demod, not wideband, so HDEBITCFG2/MODE_HAUTDEBIT = 01
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDISTATE_I2C_DEMOD_MODE(path), 0x02); /* set PEA to UFBS mode  */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(path), 0x00);

	/* Agc blocked */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1ADJ_MANUAL(path), 1);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2ADJ_MANUAL(path), 1);
	//error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2O_AGC2_COEF(path), 0);
	
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGM_FILTRE1_MANUEL(path), 1);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGM_FILTRE2_MANUEL(path), 1);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGM_FILTRE3_MANUEL(path), 1);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGM(path),1);
	
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGL_FILTRE1_AMPL(path), 0); 
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGL_FILTRE2_AMPL(path), 0); 
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FILTCFGL_FILTRE3_AMPL(path), 0); 
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGL(path),1); 

	/* fft reading registers adjustment */
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_INPUT_MODE(path), 0);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_LOW_RATE(path), 0);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GCTRL(path), 1);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_UPDCONT_UPD_CONTINUOUS(path), 2);
	
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_MODE(path), 5);		// 256 samples
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_SHIFT(path), 1);		// On
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_CTE_MODE(path), 0);  	// Off 
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_MODE_INTERSYMBOL(path), 0);  // Off
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FFTCTRL(path),1); 
	
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GAINCONT_GAIN_CONTINUOUS(path), 2);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GAINCONT_MODE_CONTINUOUS(path), 0);	// Off
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GAINCONT(path),1); 	
	

	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_THRESHOLD_MAX_THRESHOLD(path), 4);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_THRESHOLD_NO_STOP(path), 0);		// Off
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_THRESHOLD(path),1); 
	
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DISABLE_RESCALE(path), 0);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DISABLE_AVERAGE(path), 0);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DEBUG_INTERSYMBOL(path), 0);	// Off
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_DB(path), 1);		// On
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_SEL_MEM(path), 0);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(path), 1);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_CFO_FILT(path), 0);	
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEBUG1(path),1);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1_ADJUSTED(path), 88);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2REF_XXDB(path), 56);
	
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2O(path), 0x0);
	
	if((path == FE_SAT_DEMOD_1) || (path == FE_SAT_DEMOD_3))
		error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(path), 0x40);
	else	
		error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(path), 0x68);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I0(path), 0x42);
	
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_AGCRF_AGCRFCFG_AGCRF_BETA(tuner_nb), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_AMM_FROZEN(path), 1);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_QUAD_FROZEN(path), 1);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFREF(tuner_nb), 0x58);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CN_AGCIQ_BETA(path), 0);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1AMM(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1QUAD(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGCKS(path), 0x02);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCNLK(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCLK(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCLK(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(path), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(path), 0x00);
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
	
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(path), 0x00);
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
fe_lla_error_t fe_stid135_term_fft(fe_stid135_handle_t handle, enum fe_stid135_demod path, s32 Reg[60])
{ 
	u32 i=0;

	fe_lla_error_t error = FE_LLA_NO_ERROR; 
	struct fe_stid135_internal_param *pParams; 
	pParams = (struct fe_stid135_internal_param *) handle; 

	/* Restore params */
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_ENABLE(path), Reg[i++]);
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_FIFO2_ULTRABS(path), Reg[i++]); 
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(path), Reg[i++]); 
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDISTATE_I2C_DEMOD_MODE(path), Reg[i++]);
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(path), Reg[i++]);

	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1ADJ_AGC1ADJ_MANUAL(path), Reg[i++]); 
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2ADJ_AGC2ADJ_MANUAL(path), Reg[i++]); 
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2O_AGC2_COEF(path), Reg[i++]);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGM(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGL(path), (u32)Reg[i++]);
	
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I0(path), (u32)Reg[i++]);

	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_INPUT_MODE(path), Reg[i++]); 
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_UPDCONT_UPD_CONTINUOUS(path), Reg[i++]);
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_LOW_RATE(path), Reg[i++]);	
	
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FFTCTRL(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GAINCONT(path), (u32)Reg[i++]);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_THRESHOLD(path), (u32)Reg[i++]);
 	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEBUG1(path), (u32)Reg[i++]);

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
fe_lla_error_t fe_stid135_fft(fe_stid135_handle_t handle, enum fe_stid135_demod path, u32 mode, u32 nb_acquisition, s32 frq, u32 range, u32 *tab, u32* begin)
{																		 
	u32 nb_words =0;
	s32 val[4]= { 0 }, val_max;
	u32 j=0, i=0, bin=0;
	s32 f;
	s32 memstat=0, contmode =0;
	u32 nbr_pts =0; 
	u32 exp;
	static u32 max_exponent[4] = {0,0,0,0};
	s32 fld_value;
	u8 timeout = 0;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams; 
	const u16 cfr_factor = 6711; // 6711 = 2^20/(10^6/2^6)*100

	if(path > FE_SAT_DEMOD_4)
		return(FE_LLA_BAD_PARAMETER);
	
	pParams = (struct fe_stid135_internal_param *) handle;
	// Set the precision of the fft
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_MODE(path), (s32)mode);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTACC_NB_ACC_FFT(path), (s32)nb_acquisition);

	// Configure demodulator in ultra-fast blind scan 
	error |= ChipSetField (pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_ENABLE(path), 1);
	
	// Set frequency
	f = ((frq/PLL_FVCO_FREQUENCY)*cfr_factor)/100;

	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT2_CFR_INIT(path),
	((u8)(f >> 16)));
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT1_CFR_INIT(path),
	((u8)(f >> 8)));
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT0_CFR_INIT(path),
	((u8)f));
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT2(path),3);

	// Set range
	error |= fe_stid135_set_symbol_rate(pParams->handle_demod, pParams->master_clock, range, path);


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

	// calculate memory readout range 
	nbr_pts = (u32)(8192 / XtoPowerY(2, (u32) mode));
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(path), &fld_value);
	if(fld_value == 1) { // 32-bit mode
		nb_words = ((u32)1<<(10-mode+1));     // memory size N/8 for memory readout 
	} else if(fld_value == 0) { // 16-bit mode
		nb_words = ((u32)1<<(10-mode));     // memory size N/8 for memory readout 
	}
	//nb_words = ((u32)1<<(10-mode));     // memory size N/8 for memory readout 
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(path), &fld_value);
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
	
	for (i=0; i<nb_words; i++)	{   // set the FFT memory address in multiples of words

		WAIT_N_MS(5);

		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(path), (i>>8) & 0x03);
		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(path), (i & 0xff));
		error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(path), 2);

		// wait for end of transfer
		error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(path), &memstat);
		while ((!memstat) && (timeout < 50)) {
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(path), &memstat);
			timeout = (u8)(timeout + 1);
			ChipWaitOrAbort(pParams->handle_demod, 1);
		}
		if(timeout == 50)
			return(FE_LLA_NODATA);
	
		// read & store data to create an fft list

		error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(path), &fld_value);
		if(fld_value == 1) { // 32-bit mode
			for (j=0; j<4; j++) {
				error |= ChipGetRegisters(pParams->handle_demod, (u16)(REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(path) + (4 * j)), 4);
				val[3] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(path) + ((4 * j) << 16)); // 16 = address bus width for demod
				val[2] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(path) + ((4 * j) << 16));
				val[1] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA11_MEM_VAL1(path) + ((4 * j) << 16));
				val[0] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA10_MEM_VAL1(path) + ((4 * j) << 16));
				val_max = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * XtoPowerY(2, exp);
				bin = (u32)(i*4+j);
				tab[nbr_pts-bin] = (u32)val_max; 	// fil the table back to front
			}
		} else if(fld_value == 0) { // 16-bit mode	
			// read temporary memory 
			for (j=0; j<8; j++) {
				error |= ChipGetRegisters(pParams->handle_demod, (u16)(REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(path) + (2 * j)), 2);
				val[3] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(path) + ((2 * j) << 16)); // 16 = address bus width for demod
				val[2] = ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(path) + ((2 * j) << 16));
				val_max = ((val[3] << 8) + val[2]) * XtoPowerY(2, exp);
				bin = (u32)(i*8+j);
				tab[nbr_pts-bin] = (u32)val_max; 	// fil the table back to front
			} 
		}
	}
 
	// Empty hardware fft
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(path), 0x00);
	error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(path), 0x00);
	error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(path), 2);

	// Sleep down hardware fft 
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_RESTART(path), 1); 

	// Calculate beginning of data table
	*begin = (u32)(nbr_pts-bin);
	
	return(error);
}
