/*
 * blindscan, spectrum, constellation code
 * Copyright (C) 2022 Deep Thought <deeptho@gmail.com>
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 */

#include <media/dvb_math.h>
#include <linux/kthread.h>
#include "m88rs6060_priv.h"
#include "si5351_priv.h"


#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt), __func__, __LINE__, ##arg)

#define vprintk(fmt, arg...)																					\
	if(verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)


/*blind scan register defines*/
#define    FFT_N		512


#define    PSD_OVERLAP      96
#define    AVE_TIMES        256
#define    DATA_LENGTH      512

#define    PSD_SCALE        0
#define    TUNER_SLIP_STEP          11
#define    FLAT_THR_ONE_LOOP                28
#define    BLINDSCAN_SYMRATE_KSS     45000//40000

int m88rs6060_init_fft(struct m88rs6060_state* state, s32 center_freq)
{
	u32 tmp;
	u8 sm_buf;
	sm_buf = 2;

	regmap_read(state->demod_regmap, 0x76, &tmp);  //set adaptive equalizer to bypassed
	tmp |= 0x80;
	regmap_write(state->demod_regmap, 0x76, tmp); //set adaptive equalizer to bypassed


	//first part of a soft reset
	regmap_write(state->demod_regmap, 0xB2, 0x01); //reset microcontroller
	regmap_write(state->demod_regmap, 0x00, 0x01); //chip id

	regmap_write(state->demod_regmap, 0x4A, 0x00);

	//This register is updated again below (clearing spectral inversion bit!!!
	regmap_read(state->demod_regmap, 0x4D, &tmp); //set bit 0, but 4 and bit 7 (bit 1: spectral inversion)
	//value has large - distorting - effect on spectrum
	tmp |= 0x91;
	regmap_write(state->demod_regmap, 0x4D, tmp);

	/* set parameters for internal blindsearch?
		 values which produce similar results: 0x70, 0x73, 0xf0, 0xff in spectrum scan
	 */
	regmap_read(state->demod_regmap, 0x90, &tmp);
	tmp |= ((AVE_TIMES/32 - 1)*16 + (DATA_LENGTH/128 - 1));  // = ((256/32 - 1)*16 + (512/128 - 1)) = ((7 * 16 + 3)) = 0x73
	/*or with 0x73 So only bit 0,3,7 are preserved?
		probably upper nibble relates to averaging time AVE_TIMES/32 - 1 = 7 means AVE_TIMES=32*8
		lower nibble relates to fft size? NO!! (0x03 woul means len=512)
	*/
	regmap_write(state->demod_regmap, 0x90, tmp);

	tmp = 0x42;
	regmap_write(state->demod_regmap, 0x91, tmp);

	tmp = (u8)((PSD_SCALE*32)  + sm_buf - 1); //PSD_SCALE=0 sm_buf=2 => tmp=1
	tmp &= ~0x80;
	regmap_write(state->demod_regmap, 0x92, tmp);


	tmp = TUNER_SLIP_STEP * 16 + 0xf;
	//Blindscan related ?
	regmap_write(state->demod_regmap, 0x97, tmp);

	//Blindscan related?
	tmp = FLAT_THR_ONE_LOOP; //28
	regmap_write(state->demod_regmap, 0x99, tmp);

	regmap_write(state->demod_regmap, 0x4B, 0x04);
	regmap_write(state->demod_regmap, 0x56, 0x01);
	regmap_write(state->demod_regmap, 0xA0, 0x44); // see rs6060_select_mclk: clock set to 96Mhz
	regmap_write(state->demod_regmap, 0x08, 0x83); /* set blindscan mode; allow access to all i2c registers,
																							and set reserved bit 1*/
	regmap_write(state->demod_regmap, 0x70, 0x00);

	regmap_read(state->demod_regmap, 0x4d, &tmp); //disable spectral inversion
	tmp &= ~0x02;
	regmap_write(state->demod_regmap, 0x4d, tmp); //disable spectral inversion

	//reset reg[0x30] to default 0x88
	//value has little or no effect on spectrum (sporadic drop on one tp)
	regmap_write(state->demod_regmap, 0x30, 0x88);
	//last part of soft reset
	regmap_write(state->demod_regmap, 0x00, 0x00); //chip id??
	regmap_write(state->demod_regmap, 0xB2, 0x00); //start microcontroller

	return 0;
}

static int m88rs6060_get_spectrum_scan_fft_one_band(struct m88rs6060_state* state,
																						 s32 center_freq, s32 range,
																						 u32* freq, s32* rf_level, int fft_size)
{
	bool	agc_locked, fft_done;
	s32	tmp, tmp1, tmp2, tmp3, tmp4;
	u8 cnt;
	u16	totaltpnum;
	u32	nSize = (FFT_N - 16 * 2) * 2;		// sm_buf = 2, overlap = 16, nSize = 968
	int i = 0;
	u32	nCount = 0;
	s32 strength;

	fft_done		= false;
	agc_locked = m88rs6060_wait_for_analog_agc_lock(state);
	if(!agc_locked) {
		dprintk("reg: No analogue lock\n");
		return -1; //no agc lock achieved
	}
	msleep(100);


	regmap_write(state->demod_regmap, 0x9a, 0x80);

	//wait for fft to finish
	cnt = 100;
	for(i=0; i<2; ++i) {
		for(cnt=0; cnt < 200; ++cnt) {
			regmap_read(state->demod_regmap, 0x9a, &tmp);
			fft_done = ((tmp & 0x80) == 0x00);
			if(fft_done)
				break;
			if(kthread_should_stop() || dvb_frontend_task_should_stop(&state->fe))
				break;
			msleep(10);
		}

		dprintk("fft_done=%d agc_locked=%d tmp=%d\n", fft_done, agc_locked, tmp);
	if(!(fft_done && agc_locked)) {
		if(i==1)
			msleep(100); //retry once
		else
			return -1;
	 } else
		break;
	}

	m88rs6060_get_gain(state, center_freq / 1000, &strength);

	regmap_read(state->demod_regmap, 0x9a, &tmp);
	totaltpnum = (u16)(tmp & 0x1F);
	dprintk("totaltpnum = %d nSize=%d\n", totaltpnum, nSize);
	nCount = 1;

	regmap_write(state->demod_regmap, 0x9A, 0x40);
	for(i = 0; i < 32; ++i) {
		rf_level[i]=0;
	}
	for(i = 32; i < fft_size; ++i) {
		unsigned buff[2];
		int x;
		freq[i] = ((i-(signed)fft_size/2)*range)/(signed)fft_size +center_freq; //in kHz
		regmap_read(state->demod_regmap, 0x9B, &buff[0]);
		regmap_read(state->demod_regmap, 0x9B, &buff[1]);
		x = (buff[1] << 8) | buff[0];
		x = ((20000 * ((long long) intlog10(x)))>> 24) -10 * strength - 108370 +7000;
		rf_level[i] =x;
	}

	for(; i < fft_size; ++i) {
		rf_level[i]=0;
	}

	msleep(50);

	regmap_write(state->demod_regmap, 0x9A, 0x20);

	regmap_read(state->demod_regmap, 0x9B, &tmp1);
	regmap_read(state->demod_regmap, 0x9B, &tmp2);
	regmap_read(state->demod_regmap, 0x9B, &tmp3);
	regmap_read(state->demod_regmap, 0x9B, &tmp4);

	return 0;
}


int m88rs6060_get_spectrum_scan_fft(struct dvb_frontend *fe,
																	unsigned int *delay, enum fe_status *status)
{

	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct m88rs6060_state* state = fe->demodulator_priv;
	struct spectrum_scan_state* ss = &state->spectrum_scan_state;
	int error=0;
	u32 table_size = FFT_N;
	u32* temp_freq = NULL;
	s32* temp_rf_level = NULL;
	s32 last_avg =0, current_avg =0, correction = 0;
	u32 idx=0;
	int i;
	s32 start_frequency = p->scan_start_frequency;
	s32 end_frequency = p->scan_end_frequency;
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
	ss->range = 96000;
	ss->fft_size = FFT_N;
	ss->frequency_step = ss->range/ss->fft_size; //187kHz
	dprintk("frequency_step=%d\n", ss->frequency_step);

	if(ss->fft_size != table_size) {
		dprintk("Error: fft_size should be power of 2\n");
		return -1;
	}
	ss->fft_size = table_size;

 	ss->range = ss->frequency_step * ss->fft_size; //in kHz

	ss->spectrum_len = (end_frequency - start_frequency + ss->frequency_step-1) /ss->frequency_step;
	useable_samples2 = ((ss->fft_size*6)/10+1)/2;
	lost_samples2 = ss->fft_size/2 - useable_samples2;

	dprintk("demod: %d: range=[%d,%d]kHz resolution=%dkHz fft_size=%d num_freq=%d range=%dkHz lost=%d useable=%d\n",
					state->nr,
					start_frequency, end_frequency, ss->frequency_step, ss->fft_size,
					ss->spectrum_len, ss->range, lost_samples2*2, useable_samples2*2);

	ss->freq = kzalloc(ss->spectrum_len * (sizeof(ss->freq[0])), GFP_KERNEL);
	ss->spectrum = kzalloc(ss->spectrum_len * (sizeof(ss->spectrum[0])), GFP_KERNEL);
	temp_freq = kzalloc(8192 * (sizeof(temp_freq[0])), GFP_KERNEL);
	temp_rf_level = kzalloc(8192 * (sizeof(temp_rf_level[0])), GFP_KERNEL);
	if (!temp_rf_level || !temp_freq || !ss->freq || !ss->spectrum) {
		dprintk("ERROR spectrum_len=%d\n", ss->spectrum_len);
		error = -1;
		goto _end;
	}

	m88rs6060_set_default_mclk(state);

	min_correction[0] = 0x7fffffff;
	min_correction[1] = 0x7fffffff;
	max_correction[0] = 0x80000000;
	max_correction[1] = 0x80000000;
	sum_correction =0;
	sum_len =0;

	for(idx=0; idx < ss->spectrum_len;) {
		s32 iFreqKHz;
		//the offset useable_samples2 ensures that the first usable index is aligned with start_frequency
		s32 center_freq = start_frequency + (idx + useable_samples2)* ss->frequency_step;
		if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
			dprintk("exiting on should stop\n");
			break;
		}
		dprintk("SPEC: center_freq=%d\n", center_freq);
		m88rs6060_init_fft(state, center_freq);
		iFreqKHz = (center_freq + 500) / 1000 * 1000;

		msleep(10);

		m88rs6060_set_tuner(state, iFreqKHz / 1000, BLINDSCAN_SYMRATE_KSS, 0);

		m88rs6060_set_carrier_offset(state, iFreqKHz - (iFreqKHz/ 1000)*1000);

		error = m88rs6060_get_spectrum_scan_fft_one_band(state,	iFreqKHz, ss->range,
																										 temp_freq, temp_rf_level, ss->fft_size);


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

	if(!error) {
		*status =  FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
		return 0;
	} else {
		dprintk("encountered error\n");
		*status =  FE_TIMEDOUT|FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
		return error;
	}
}
