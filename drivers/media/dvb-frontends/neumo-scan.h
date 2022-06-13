/*
 *
 * Copyright (c) 2020-2022, Deep Thought <deepto@gmail.com> - All Rights Reserved
 *
 *
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
#ifndef NEUMODVB_SCAN_H
#define NEUMODVB_SCAN_H


/*
	state for spectrum scan
*/

struct constellation_scan_state {
	bool constallation_present;
	bool in_progress;

	struct dtv_fe_constellation_sample* samples;
	int num_samples;
	int samples_len;
	int constel_select;
};

struct spectrum_scan_state {
	//user specified input parameters
	int fft_size; //for fft
	s32 start_frequency;
	s32 end_frequency;
	s32 frequency_step; //bandwidth of one spectral bin in kHz
	int snr_w; //window_size to look for snr peaks
	int threshold; //minimum peak amplitude required
	int threshold2; //minimum peak amplitude required
	int mincount; //minimum number of above threshold detections to count as rise/fall


	s32 lo_frequency_hz;
	s32 range; //bandwidth of current fft in kHz (covers the whole spectrum, not just the useable part)
	int mode; //for fft


	//outputs
	bool spectrum_present;
	s32* freq;
	s32* spectrum;
	struct spectral_peak_t* candidates;
	int spectrum_len;
	int num_candidates;
	int num_good;
	int num_bad;

	//state
	bool scan_in_progress;
	int current_idx; //position at which we last stopped processing
	s32 next_frequency; // If we found a transponder last time, this is the frequency just above the transponder bandwidth


};

int neumo_scan_spectrum(struct spectrum_scan_state* ss);

#endif  /* ifndef NEUMODVB_SCAN_H */
