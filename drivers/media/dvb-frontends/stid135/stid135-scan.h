/*
 *
 * Copyright (c) 2020-2021, Deep Thought <deepto@gmail.com> - All Rights Reserved
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


#ifndef STID135_SCAN_H
#define STID135_SCAN_H


/*
	state for spectrum scan
*/

struct spectrum_scan_state {
	//user specified input parameters
	int fft_size; //for fft
	s32 start_frequency;
	s32 end_frequency;
	s32 sample_step; //bandwidth of one spectral bin in kHz
	s32 lo_frequency_hz;
	s32 range; //bandwidth of current fft in kHz (covers the whole spectrum, not just the useable part)
	int mode; //for fft
	int w; //window_size
	int threshold; //minimum peak amplitude required
	int mincount; //minimum number of above threshold detections to count as rise/fall


	//outputs
	bool spectrum_present;
	s32* freq;
	s32* spectrum;
	s32* candidate_frequencies;
	int spectrum_len;
	int num_candidates;
	int num_good;
	int num_bad;

	//state
	bool scan_in_progress;



	//To remove
	//int idx_start ;//analysis starts at spectrum[idx_start]
	//int idx_end ;//analysis end at spectrum[idx_end]-1
	int current_idx; //position at which we last stopped processing
	s32 next_frequency; // If we found a transponder last time, this is the frequency just above the transponder bandwidth

	int last_peak_idx; //index at which we last found a peak
	s32 last_peak_freq; //frequency of current peak
	s32 last_peak_bw; //bandwidth of current peak

	int last_rise_idx; //location of last processed rising peak
	int last_fall_idx; //location of last processed falling peak




	//u8* peak_marks;

};

int stid135_scan_spectrum(struct spectrum_scan_state* ss);

#endif  /* ifndef STID135_SCAN_H */
