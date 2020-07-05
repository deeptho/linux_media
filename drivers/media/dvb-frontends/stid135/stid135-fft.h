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

#ifndef STID135_FFT_H
#define STID135_FFT_H

#ifdef __cplusplus
    extern "C" {
#endif

			fe_lla_error_t fe_stid135_init_fft(fe_stid135_handle_t handle, enum fe_stid135_demod path,
																				 FE_OXFORD_TunerPath_t tuner_nb, s32 Reg[60]);
			
			fe_lla_error_t fe_stid135_term_fft(fe_stid135_handle_t handle, enum fe_stid135_demod path, FE_OXFORD_TunerPath_t tuner_nb, s32 Reg[60]);
			fe_lla_error_t fe_stid135_fft(fe_stid135_handle_t handle, enum fe_stid135_demod path, u32 mode,
																		u32 nb_acquisition, s32 frq, u32 range, u32 *tab, u32* begin);
				

			
#ifdef __cplusplus
    }
#endif

#endif  /* ndef STID135_FFT_H */
