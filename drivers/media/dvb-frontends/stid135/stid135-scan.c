/*
 * (c) deeptho@gmail.com 2019-2022
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
#include <linux/sort.h>
#include <media/dvb_frontend.h>
#include "stid135-scan.h"

extern int stid135_verbose;
#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

#define vprintk(fmt, arg...)																					\
	if(stid135_verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)




struct spectrum_peak_internal_t {
	s32 idx; //index at which we last found a peak
	s32 freq; //frequency of current peak
	s32 bw; //bandwidth of current peak
	s32 rise_idx; //location of last processed rising peak
	s32 fall_idx; //location of last processed falling peak
	s32 snr;
	s32 level;
};


struct scan_internal_t {
	s32* rs; //running sum
	s32 start_idx ;//analysis starts at spectrum[start_idx]
	s32 end_idx ;//analysis end at spectrum[end_idx]-1
	s32 current_idx; //position at which we last stopped processing
	s32 window_idx ; //index of current window size
	s32 next_frequency; // If we found a transponder last time, this is the frequency just above the transponder bandwidth

	struct spectrum_peak_internal_t last_peak;

	s32 last_rise_idx; //location of last processed rising peak
	s32 last_fall_idx; //location of last processed falling peak
	s32 last_peak_snr; //snr of last detected candidate

	u8* peak_marks;
	struct spectrum_peak_internal_t* peaks;
	int num_peaks;
	int max_num_peaks;
	int w; //window_size to look for peaks
};

enum slope_t {
	NONE = 0,
	FALLING=1,
	RISING=2
};


static s32 max_(s32*a, int n)
{
	int i;
	s32 ret=a[0];
	for(i=0; i<n;++i)
		if(a[i]> ret)
			ret=a[i];
	return ret;
}

static void clean_(s32*psig, s32*pres, int n)
{
	int i;
	s32 mean;
	s32 last = psig[0];
	int j;
	for (i=0; i< n; ++i) {
		if(pres[i] == pres[last]) {
			mean += psig[i];
		} else {

			if(i>last)
				mean/= (i-last);
			for(j=last; j<i;++j)
				psig[i] = mean;
			mean =psig[i];
			last=i;
		}
	}
}

static void running_sum(s32* pout, s32* psig, int n)
{
	int i;
	int accu=psig[0];
	for(i=0;i<n;++i) {
		accu += psig[i];
		pout[i]=accu;
	}
}


static s32 windows[] = {2,	 4,		6,
	8,	10,	 12,	14,
	16,	18, 20,	 22, 24, 26, 28, 30,
	32,	36, 40,	 44, 48,	52, 56, 60,
	64, 72,	 80,	88,	 96,	104, 112, 120,
	128, 144, 160, 176, 192, 208, 224, 240,
	256,	 288, 320, 352, 384, 416, 448, 480,
	512,  576, 640, 704, 768, 832, 896, 960,
	1024, 1152, 1280, 1408, 1536, 1664, 1792, 1920,
	2048};

static int check_candidate_tp(struct spectrum_scan_state_t* ss,
															struct scan_internal_t* si)
{
	int i;
	struct spectrum_peak_internal_t *cand = &si->last_peak;
	if(cand->snr < ss->threshold2) {
		vprintk("Rejecting too weak candidate\n");
		return -1;
	}
	for(i=0; i< si->num_peaks; ++i) {
		struct spectrum_peak_internal_t *old = &si->peaks[i];
		//older contained/overlapping transponder with smaller bandwidth
		if(cand->bw >= old->bw &&
			 (
				(old->rise_idx >= cand->rise_idx &&  old->rise_idx <= cand->fall_idx) ||
				(old->fall_idx >= cand->rise_idx &&  old->fall_idx <= cand->fall_idx)
				 )) {
			if( old->level - cand->level >= ss->threshold2) {
				//vprintk("Rejecting peak because it contains other peaks\n");
				return -1;
			} else {
#if 0
				vprintk("Overwriting peak because it contains other peaks\n");
				*old = *cand;
				return -1;
#else
				memmove(&si->peaks[i] , &si->peaks[i+1], sizeof(si->peaks[0]) * si->num_peaks-i-1);
				--i;
				si->num_peaks--;
				continue;
#endif
			}
		}

		//older contained/overlapping transponder with larger bandwidth
		if((cand->bw < old->bw && cand->rise_idx >= old->rise_idx &&  cand->rise_idx <= old->fall_idx) ||
			 (cand->fall_idx >= old->rise_idx &&  cand->fall_idx <= old->fall_idx)
			) {
			if( cand->level - old->level >= ss->threshold2) {
#if 0
				vprintk("Rejecting peak because it contains other peaks\n");
				*old = *cand;
				return -1;
#else
				memmove(&si->peaks[i] , &si->peaks[i+1], sizeof(si->peaks[0]) * si->num_peaks-i-1);
				si->num_peaks--;
				--i;
				continue;
#endif
			} else {
				//vprintk("Overwriting peak because it contains other peaks\n");
				return -1;
			}
		}
	}
	return 0;
}

/*
	candidate right edges of transponder
	w is window size
	n is size of signal

  --------------\
                 \
                  --
*/
static void falling_kernel(struct spectrum_scan_state_t* ss,
													 struct scan_internal_t* si)
{
	int count=0;
	bool peak_found=false;
	s32 delta = (si->w*16)/200;
	s32 w2= si->w/2;
	int n = ss->spectrum_len;
	int i;
	if(delta==0)
		delta=1;
	for(i=w2; i < n-delta; ++i) {
		s32 power = (si->rs[i]-si->rs[i -w2])/w2; //average strength
		s32 right = ss->spectrum[i + delta];
		if (power - right > ss->threshold)
			count++;
		else
			count =0;
		if(count >= ss->mincount) {
			//mark complete peak if not already on a peak
			if(!peak_found) {
				si->peak_marks[i] |=FALLING;
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
       /--------------
      /
-----/  |

*/
static void rising_kernel(struct spectrum_scan_state_t* ss,
													struct scan_internal_t* si)
{

	int count=0;
	bool peak_found=false;
	s32 delta = (si->w*16)/200; //rise interval
	s32 w2= si->w/2; //plateau interval
	int n = ss->spectrum_len;
	int i;
	if(delta==0)
		delta =1;
	for(i = n - w2 - 1; i >=delta; --i) {
		s32 power = (si->rs[i + w2]- si->rs[i])/w2;
		s32 left = ss->spectrum[i - delta];
		if (power - left > ss->threshold)
			count++;
		else
			count =0;
		if(count >= ss->mincount) {
			//mark complete peak if not already on a peak
			if(!peak_found) {
					si->peak_marks[i] |= RISING;
			}
			peak_found = true;
		} else {
			peak_found =false;
		}
	}
}


static void stid135_spectral_init_level(struct spectrum_scan_state_t* ss,
																				struct scan_internal_t* si)
{
	si->start_idx =  (si->w*16)/200;
	if (si->start_idx==0)
		si->start_idx++;
	si->end_idx = ss->spectrum_len - (si->w * 16)/200;
	if (si->end_idx== ss->spectrum_len)
		si->end_idx--;
	si->current_idx = si->start_idx;
	si->last_peak.idx = -1;
	si->last_rise_idx = -1;
	si->last_fall_idx = -1;
	memset(si->peak_marks, 0, sizeof(si->peak_marks[0]) * ss->spectrum_len);
	running_sum(si->rs, ss->spectrum, ss->spectrum_len);
	falling_kernel(ss, si);
	rising_kernel(ss, si);
	//fix_kernel(si->peak_marks, ss->spectrum, ss->spectrum_len, ss->w, ss->threshold, ss->mincount);
	}

static int stid135_spectral_scan_start(struct spectrum_scan_state_t* ss,
																			 struct scan_internal_t* si)
{
	int i;
	dprintk("Starting spectrum scan\n");
	si->max_num_peaks = 1024;
	ss->scan_in_progress =true;

	si->window_idx = 0;
	si->w = windows[si->window_idx];

	//ss->w =17;
	ss->snr_w = 35; ////percentage
	//ss->threshold = 2000;
	//ss->mincount = 3;
	si->peak_marks = (u8*)kzalloc(ss->spectrum_len * sizeof(si->peak_marks[0]),
																GFP_KERNEL);
	si->peaks =(struct spectrum_peak_internal_t*)
		kzalloc(si->max_num_peaks * sizeof(si->peaks[0]),
																GFP_KERNEL);
#if 0
	ss->freq = (s32*)kzalloc(ss->spectrum_len * sizeof(ss->freq[0]),
													 GFP_KERNEL);
#endif
	si->rs = (s32*)kzalloc(ss->spectrum_len * sizeof(si->rs[0]),
												 GFP_KERNEL);
	si->num_peaks = 0;
#if 0
	for(i=0; i <ss->spectrum_len; ++i)
		ss->freq[i]=i*ss->frequency_step;
#endif
	if (!ss->spectrum) {
		BUG_ON(!ss->spectrum);
		dprintk("error - no spectrum in input\n");
		return -ENOMEM;
	}
	BUG_ON(ss->candidates); //this will be allocated later
	stid135_spectral_init_level(ss, si);
	return 0;
}

static int stid135_spectral_scan_end(struct spectrum_scan_state_t* ss,
																			 struct scan_internal_t* si)
{
	int i;
	if(si->peak_marks)
		kfree(si->peak_marks);
	si->peak_marks = NULL;
	if(si->rs)
		kfree(si->rs);
	si->rs = NULL;
#if 0
	for(i=0; i <ss->spectrum_len; ++i)
		ss->freq[i]=i*ss->frequency_step;
#endif
	dprintk("num_peaks=%d\n", si->num_peaks);
	BUG_ON(ss->candidates != NULL);
	ss->candidates =
		(struct spectral_peak_t*)
		kzalloc(si->num_peaks * sizeof(struct spectral_peak_t), GFP_KERNEL);
	if(!ss->candidates)
		return -ENOMEM;
	ss->num_candidates = si->num_peaks;
	for(i=0; i < si->num_peaks; ++i) {
		struct spectrum_peak_internal_t* pi = &si->peaks[i];
		struct spectral_peak_t* p = &ss->candidates[i];
		p->freq = pi->freq;
		p->symbol_rate = pi->bw*1250;
		p->snr = pi->snr;
		p->level = pi->level;
	}
	if(si->peaks)
		kfree(si->peaks);
	si->peaks = NULL;
	si->num_peaks = 0;
	return 0;
}



static s32 peak_snr(struct spectrum_scan_state_t* ss,
										struct scan_internal_t* si)
{
	s32 mean=0;
	s32 min1=0;
	s32 min2=0;
	int i;
	s32 w = (ss->snr_w * (si->last_fall_idx - si->last_rise_idx))/100;
	if( si->last_fall_idx<=  si->last_rise_idx)
		return -99000;
	for(i=si->last_rise_idx; i< si->last_fall_idx; ++i)
		mean += ss->spectrum[i];
	mean /=(si->last_fall_idx - si->last_rise_idx);

	i= si->last_rise_idx - w;
	if(i<0)
		i=0;
	min1 = ss->spectrum[si->last_rise_idx];
	for(; i < si->last_rise_idx; ++i)
		if (ss->spectrum[i] < min1)
			min1 = ss->spectrum[i];

	i= si->last_fall_idx + w;
	if(i> ss->spectrum_len)
		i = ss->spectrum_len;
	min2 = ss->spectrum[si->last_fall_idx];
	for(; i > si->last_fall_idx; --i)
		if (ss->spectrum[i] < min2)
			min2 = ss->spectrum[i];

	if (min2<min1)
		min1= min2;
	return mean - min1;
}


//returns index of a peak in the spectrum
static int next_candidate_this_level(struct spectrum_scan_state_t* ss,
																		 struct scan_internal_t* si)
{
	s32 snr;
	for(; si->current_idx < si->end_idx; ++si->current_idx) {
		if(si->peak_marks[si->current_idx] & FALLING) {
			if(si->last_rise_idx > si->last_fall_idx && si->last_rise_idx>=0 &&
				 si->current_idx - si->last_rise_idx <= si->w && //maximum windows size
				 si->current_idx - si->last_rise_idx >= (si->w*2)/3 //minimum windows size
				) {

				//candidate found; peak is between last_rise and current idx
				si->last_peak.idx = (si->last_rise_idx + si->current_idx)/2;
				si->last_peak.freq =
					ss->freq[si->last_peak.idx]; //in kHz
				BUG_ON(si->current_idx-si->last_rise_idx > si->w);
				si->last_peak.bw = ss->freq[si->current_idx] - ss->freq[si->last_rise_idx]; //in kHz
				dprintk("CANDIDATE: %d %dkHz BW=%dkHz snr=%ddB\n", si->last_peak.idx, si->last_peak.freq,
								si->last_peak.bw, snr);
				si->last_fall_idx = si->current_idx;
#if 0
				si->last_peak_snr = peak_snr(ss);
#else
				BUG_ON(si->last_fall_idx != si->current_idx);
				si->last_peak.level = (si->rs[si->last_fall_idx] - si->rs[si->last_rise_idx]) /
					(si->last_fall_idx-si->last_rise_idx);
				{
					s32 delta = (si->w*16)/200;
					s32 left =  si->last_rise_idx - delta;
					s32 right =  si->last_fall_idx + delta;
					BUG_ON(left<0);
					BUG_ON(right >= ss->spectrum_len);
					left = ss->spectrum[left];
					right = ss->spectrum[right];
					if(left < right)
						right = left;
					si->last_peak_snr = si->last_peak.level - right;
					si->last_peak.snr = 	si->last_peak_snr;
				}
#endif
				si->last_peak.rise_idx = si->last_rise_idx;
				si->last_peak.fall_idx = si->current_idx;

				//assert(	ss->freq[si->last_peak.fall_idx] - ss->freq[si->last_peak.rise_idx] == ss->bw);


				if(si->peak_marks[si->current_idx]& RISING)
					si->last_rise_idx = si->current_idx;
				si->current_idx++;

				return si->last_peak.idx;
			}

			si->last_fall_idx = si->current_idx;
		}

		if(si->peak_marks[si->current_idx]& RISING)
			si->last_rise_idx = si->current_idx;
	}
	return -1;
}


static int next_candidate_tp(struct spectrum_scan_state_t* ss,
														 struct scan_internal_t* si)
{
	int ret;
	while(si->window_idx <  sizeof(windows)/sizeof(windows[0])) {
		if (si->current_idx >= si->end_idx) { //we reached end of a window
			if(++si->window_idx >=  sizeof(windows)/sizeof(windows[0]))
				return -1; //all windows done
			si->w = windows[si->window_idx]; //switch to next window size
			stid135_spectral_init_level(ss, si);
		}
		ret= next_candidate_this_level(ss, si);
		if(ret<0)
			continue;
#if 1
		dprintk("CANDIDATE: %d %dkHz BW=%dkHz snr=%ddB\n", si->last_peak.idx, si->last_peak.freq,
						si->last_peak.bw, si->last_peak_snr);
#endif
		return ret;
	}
	return -1;
}




static int stid135_spectral_scan_next(struct spectrum_scan_state_t* ss,
																			struct scan_internal_t* si,
																			s32 *frequency_ret, s32* snr_ret)
{
	int ret=0;
	while(ret>=0) {
		ret = next_candidate_tp(ss, si);
		if(ret>=0) {
			if(check_candidate_tp(ss, si)>=0) {
				dprintk("Next frequency to scan: [%d] %dkHz SNR=%d BW=%d\n", ret, si->last_peak.freq,
								si->last_peak_snr, si->last_peak.bw);
				*frequency_ret =  si->last_peak.freq;
				*snr_ret =  si->last_peak_snr;
				return  si->last_peak.idx;
			}
		} else {
			dprintk("Current subband fully scanned: current_idx=%d end_idx=%d\n", si->current_idx, si->end_idx);
		}
	}
	return -1;
}


static int cmp_fn(const void *pa, const void *pb) {
	struct spectrum_peak_internal_t* a = (struct spectrum_peak_internal_t*)pa;
	struct spectrum_peak_internal_t* b = (struct spectrum_peak_internal_t*)pb;
	return a->freq - b->freq;
}



/*
	spectrum is stored in ss on input
 */
int stid135_scan_spectrum(struct spectrum_scan_state_t* ss)
{
	int ret= 0;
	s32 frequency;
	s32 snr;
	struct scan_internal_t si;
	dprintk("Starting scan on spectrum %p with length %d\n", ss->spectrum, ss->spectrum_len);
	if (!ss->spectrum || ss->spectrum_len<=0) {
		return -ENOMEM;
	}
	ss->mincount = 1;
	ss->threshold = 3000;
	ss->threshold2 = 3000;
	ret = stid135_spectral_scan_start(ss, &si);
	if(ret<0) {
		dprintk("Error starting spectrum scan\n");
		return ret;
	}
	while(ret>=0) {
		ret = stid135_spectral_scan_next (ss, &si, &frequency, &snr);
		dprintk("FREQ=%d BW=%d SNR=%ddB\n", frequency, si.last_peak.bw, snr);
		if(ret>=0) {
			si.peaks[si.num_peaks++] = si.last_peak;
			vprintk("NP=%d\n", si.num_peaks);
		}
	}
	sort(&si.peaks[0], si.num_peaks, sizeof(si.peaks[0]), cmp_fn, NULL);
	return stid135_spectral_scan_end(ss, &si);
}
