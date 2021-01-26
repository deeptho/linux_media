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

#include "stid135-scan.h"


#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

#define vprintk(fmt, arg...)																					\
	if(stid135_verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)




enum slope_t {
	FALLING = 1,
	RISING = 2
};

#define MAXW 31

static s32 max_(s32*a, int n)
{
	int i;
	s32 ret=a[0];
	for(i=0; i<n;++i)
		if(a[i]> ret)
			ret=a[i];
	return ret;
}




/*
	candidate right edges of transponder
	w is window size
	n is size of signal
*/
static void falling(u8* pres, s32* psig, int n, int w, int thresh, int mincount)
{
	int count=0;
	bool peak_found=false;
	int i;
	s32 temp[MAXW];
	if(w >MAXW)
		w=MAXW;
	for(i=0;i<w;++i)
		temp[i]=psig[n-1];
	for(i=0; i< n; ++i) {
		s32 s = psig[i];
		s32 left_max;
		temp[i%w] = s;
		left_max = max_(temp, w);
		if (left_max-s > thresh)
			count++;
		else
			count =0;
		if(count >= mincount) {
			//mark complete peak if not already on a peak
			if(!peak_found) {
				//dprintk("SET FALL\n");
				pres[i] |=FALLING;
			}
			peak_found = true;
		} else {
			peak_found =false;
		}
	}
}

/*
	candidate left edges of transponder
	w is window size
	n is size of signal
*/
static void rising(u8* pres, s32* psig, int n, int w, int thresh, int mincount)
{
	int count=0;
	bool peak_found=false;
	int i;
	s32 temp[MAXW];
	if(w>MAXW)
		w=MAXW;
	for(i=0;i<w;++i)
		temp[i]=psig[n-1];
	for(i=n-1; i>=0; --i) {
		s32 s = psig[i];
		s32 right_max;
		temp[i%w] = s;
		right_max = max_(temp, w);
		if (right_max-s > thresh)
			count++;
		else
			count =0;
		if(count>=mincount) {
			//mark complete peak if not already on a peak
			if(!peak_found) {
				//dprintk("SET RISE\n");
				pres[i] |= RISING;
			}
			peak_found = true;
		} else {
			peak_found =false;
		}
	}
}


/*
	Fix some cases in which there is no rising edge between two falling edges,
	or vice versa
 */
static void fix(u8* pres, s32* psig, int n, int w, int thresh, int mincount)
{

	//bool peak_found=false;
	int i;
	//s32 temp[MAXW];
	int last=0;
	if(w>MAXW)
		w=MAXW;


	for(i=0; i<n; ++i) {
		if(i>0) {
			if(pres[i]!=0) {
				if((pres[i] & RISING ) && (pres[last] &RISING)) {
					falling(pres+last, psig+last, i-last+1, w, thresh, mincount);
				}
				if((pres[i] & FALLING ) && (pres[last] & FALLING)) {
					rising(pres+last, psig+last, i-last+1, w, thresh, mincount);
				}
				last = i;
			}
		}
	}
}





//returns index of a peak in the spectrum
static void find_candidate_tps(struct spectrum_scan_state* ss, u8* peak_marks)
{
	int idx=0;
	s32 last_peak_freq;
	s32 last_peak_bw;
	for(; idx < ss->spectrum_len; ++idx) {
		if(peak_marks[idx] & FALLING) {
			dprintk("FALLING FOUND at %d last_rise=%d last fall =%d\n",
							idx, ss->last_rise_idx, ss->last_fall_idx);
			if(ss->last_rise_idx > ss->last_fall_idx && ss->last_rise_idx>=0) {
				//candidate found; peak is between last_rise and current idx
				ss->last_peak_idx = (ss->last_rise_idx + idx)/2;
				last_peak_freq = ss->freq[ss->last_peak_idx]; //in kHz
				last_peak_bw = ss->freq[idx] - ss->freq[ss->last_rise_idx]; //in kHz
				dprintk("CANDIDATE: %d %dkHz BW=%dkHz\n", ss->last_peak_idx, last_peak_freq, last_peak_bw);
				ss->last_fall_idx = idx;
				if(peak_marks[idx]& RISING)
					ss->last_rise_idx = idx;
				idx++;
				if(ss->num_candidates >= ss->spectrum_len /*allocted size*/) {
					dprintk("Array size exceeded\n");
					break;
				}
				ss->candidate_frequencies[ss->num_candidates++] = last_peak_freq;
				continue;
			} else
				ss->last_fall_idx = idx;
		}

		if(peak_marks[idx]& RISING)
			ss->last_rise_idx = idx;
	}
}

/*
	spectrum is stored in ss on input
 */
int stid135_scan_spectrum(struct spectrum_scan_state* ss)
{
	u8* peak_marks;
	if (!ss->spectrum || ss->spectrum_len<=0) {
		return -ENOMEM;
	}


	//ss->idx_end = ss->spectrum_len;
	ss->current_idx = 0;
	ss->last_peak_idx = -1;
	ss->last_rise_idx = -1;
	ss->last_fall_idx = -1;

	ss->w =17;
	ss->threshold = 2000;
	ss->mincount = 3;
	peak_marks = kzalloc(ss->spectrum_len * (sizeof(peak_marks[0])), GFP_KERNEL);
	if(!peak_marks) {
		dprintk("Failed to allocate memory\n");
		return -1;
	}

	falling(peak_marks, ss->spectrum, ss->spectrum_len, ss->w, ss->threshold, ss->mincount);
	rising(peak_marks, ss->spectrum, ss->spectrum_len, ss->w, ss->threshold, ss->mincount);
	fix(peak_marks, ss->spectrum, ss->spectrum_len, ss->w, ss->threshold, ss->mincount);
	find_candidate_tps(ss, peak_marks);
	kzfree(peak_marks);

	//dump_data("marks", ss->peak_marks, ss->spectrum_len);
	return 0;
}
