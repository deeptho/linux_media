/*
 * (c) deeptho@gmail.com 2019-2025
 *
 * Copyright notice:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#pragma once
#include "neumo-scan.h"

#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

#define vprintk(fmt, arg...)																					\
	if(verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)


typedef int32_t s32;
typedef uint32_t u32;
typedef uint8_t u8;

struct spectrum_peak_t {
	u32 freq;				 // frequency of current peak
	s32 symbol_rate; // estimated symbolrate of current peak
	s32 mean_snr;
	//s32 min_snr;
	s32 mean_level;
	//s32 min_level;
};

struct spectrum_peak_internal_t {
	s32 idx;			// center index at which we last found a peak
	s32 lowest_left_idx; //minimum left of peak
	s32 lowest_right_idx; //minimum right of peak
	s32 rise_idx; // location of rising part of peal (3dB)
	s32 fall_idx; // location of falling part of peak (3dB)
	s32 freq;			// central frequency of current peak
	s32 bw;				// 3dB bandwidth of current peak
	s32 lowest_left_level;
	s32 lowest_right_level;
	s32 mean_snr; // SNR of central peak w.r.t. to what we think is noise level
	s32 mean_level;  //level of central plateau
	s32 fluctuation;  //estimate of mean absolute signal fluctuation

	s32 dip_level; // amplude of lowest dip in central
	s32 dip_snr; //snr of this dip w.r.t. what we think is the noise level
};


struct scan_internal_t {
	s32* rs;						// running sum
	s32* noise;						// noise values
	s32 start_idx;			// analysis starts at spectrum[start_idx]
	s32 end_idx;				// analysis end at spectrum[end_idx]-1
	s32 next_frequency; // If we found a transponder last time, this is the frequency just above the transponder bandwidth

	struct spectrum_peak_internal_t last_peak;

#ifdef TEST
	s32 last_rise_idx; // location of last processed rising peak

	s32 last_fall_idx; // location of last processed falling peak
#endif

	u8* peak_marks;
	struct spectrum_peak_internal_t* peaks;
	int num_peaks;
	int max_num_peaks;
	int w; // window_size to look for peaks
#ifdef USERSPACE
	void check();
#endif
};

enum slope_t
{
	NONE = 0,
	FALLING = 1,
	RISING = 2
};

int stid135_spectral_scan_init(spectrum_scan_state_t* ss, struct scan_internal_t* si, s32* spectrum,
															 u32* freq, int len);

void stid135_spectral_init_level(spectrum_scan_state_t* ss, struct scan_internal_t* si,
																 s32* falling_response_ret, s32* rising_response_ret);


inline s32 dtabs (s32 x) {
	return x < 0 ? - x: x;
}

inline s32 dtmin (s32 x, s32 y) {
	return x < y ? x: y;
}

inline s32 dtmax (s32 x, s32 y) {
	return x < y ? y: x;
}
