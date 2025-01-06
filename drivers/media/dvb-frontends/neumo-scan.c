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
#include <linux/sort.h>
#include <media/dvb_frontend.h>
#include "./neumo-scan-private.h"


static void running_sum(s32* pout, s32* psig, int n) {
	int i;
	int accu = psig[0];
	for (i = 0; i < n; ++i) {
		accu += psig[i];
		pout[i] = accu;
	}
}

static void noise_est(s32* pout, s32* psig, int n) {
	int i;
	int accu = dtabs(2*psig[0] - psig[1]);
	for (i = 1; i < n-1; ++i) {
		s32 temp = dtabs(psig[i]*2 - (psig[i-1] +psig[i+1]));
		accu += temp;
		pout[i] = accu;
	}
	accu += dtabs(2*psig[i] - psig[i-1]);
	pout[i] = accu;
}

static s32 windows[] = {
	2,    4,    6,    8,   10,   12,   14,   16,   18,   20,   22,
	24,   26,   28,   30,   32,   34,   36,   38,   40,   42,   44,
	46,   48,   50,   52,   54,   56,   58,   60,   62,   64,   66,
	68,   72,   74,   76,   80,   82,   86,   88,   92,   94,   98,
	102,  106,  108,  112,  116,  120,  126,  130,  134,  140,  144,
	150,  154,  160,  166,  172,  178,  184,  190,  198,  204,  212,
	220,  228,  236,  244,  252,  262,  270,  280,  290,  300,  312,
	322,  334,  346,  358,  370,  384,  398,  412,  426,  442,  456,
	474,  490,  508,  526,  544,  564,  584,  604,  626,  648,  670,
	694,  720,  744,  772,  798,  826,  856,  886,  918,  950,  984,
	1020, 1056, 1094, 1132, 1172, 1214, 1256, 1302, 1348, 1396, 1444,
	1496, 1548, 1604, 1660, 1720, 1780, 1844, 1910, 1976, 2048,
};


static bool is_part_of(struct spectrum_peak_internal_t* a, struct spectrum_peak_internal_t* b) {
	bool a_rise_in_b = (a->rise_idx >= b->rise_idx && a->rise_idx <= b->fall_idx);
	bool a_fall_in_b =	(a->fall_idx >= b->rise_idx && a->fall_idx <= b->fall_idx);
	bool a_in_b = a_rise_in_b && a_fall_in_b;
	return a_in_b;
	if (a_in_b && //an older peak is contained in the candidate
			b->dip_snr >= a->mean_snr) //and cand has a dip due to it
		return false;
};

	/*returns 1 if a is better than a and should be kept,
		return -1 if b is better than a and should be kept
		returns 0 if both do not conflict
	*/
static int is_better_than(struct spectrum_peak_internal_t* a, struct spectrum_peak_internal_t* b) {
		//implicit preference for a, which has (often) lower bandwidth
		int a_in_b_count = 0;
		int b_in_a_count = 0;
		if(a->lowest_left_idx > b->rise_idx && a->lowest_left_idx < b->fall_idx ) {
			a_in_b_count++;
			if(b->mean_level - a->lowest_left_level >= b->fluctuation*8/* &&b_w>=512*/)
				return 1;
		}

		if(a->lowest_right_idx > b->rise_idx && a->lowest_right_idx < b->fall_idx) {
			a_in_b_count++;
			if(b->mean_level - a->lowest_right_level >= b->fluctuation*8 /*&& b_w>=512*/)
				return 1;
		}

		if(is_part_of(a,b))
			return -1;

		if(b->lowest_left_idx > a->rise_idx && b->lowest_left_idx < a->fall_idx) {
			b_in_a_count++;
			if(a->mean_level - b->lowest_left_level >= a->fluctuation*8  /*&& a_w>=512*/)
				return -1;
		}

		if(b->lowest_right_idx > a->rise_idx && b->lowest_right_idx < a->fall_idx) {
			b_in_a_count++;
			if(a->mean_level - b->lowest_right_level >= a->fluctuation*8 /*&& a_w>=512*/)
				return -1;
		}

		if(is_part_of(b,a))
			return 1;

		if(a_in_b_count == 0 && b_in_a_count ==0)
			return 0; // no overlap

		return (a->mean_snr >= b->mean_snr ) ? 1 : -1; //prefer the peak with the lowest dip

};

static int check_candidate_tp(spectrum_scan_state_t* ss, struct scan_internal_t* si)
{
	int i;
	struct spectrum_peak_internal_t* cand = &si->last_peak;
#ifdef DEBUGXXX
	auto match = [&](auto*cand) {
		return (cand->freq/1000 >=11175) && (cand->freq/1000 <= 11180);
	};
#endif
	if (cand->mean_snr < ss->threshold2) {
#ifdef DEBUGXXX
		if(match(cand))
			dprintk("Rejecting too weak candidate: %dkHz BW=%dkHz snr=%ddB/%ddB level=%ddB\n", cand->freq, cand->bw,
							cand->mean_snr, cand->dip_snr, cand->dip_level);
#endif
		return -1;
	}
	if(cand->dip_snr <= ss->threshold) {
#ifdef DEBUGXXX
		if(match(cand))
			dprintk("Rejecting too weak candidate: %dkHz BW=%dkHz snr=%ddB/%ddB level=%ddB\n", cand->freq, cand->bw,
							cand->mean_snr, cand->dip_snr, cand->dip_level);
#endif
		return -1;
	}


	for (i = 0; i < si->num_peaks; ++i) {
		struct spectrum_peak_internal_t* old = &si->peaks[i];
#ifdef DEBUGXXX
		bool print =  match(old) | match(cand);
#endif
		int old_is_better = is_better_than(old, cand);
		if(old_is_better == 0)
			continue; //no overlap
		if(old_is_better > 0) { //old is better
#ifdef DEBUGXXX
			if(print) {
				dprintk("Rejecting peak: new: %dkHz BW=%d.%03dMHz "
								"lvl=%d snr=%d fluct=%d dip =%d"
								" old: %dkHz BW=%d.%03dMHz lvl=%d snr=%d fluct=%d dip=%d w=%d\n",
								cand->freq, cand->bw/1000, cand->bw%1000, cand->mean_level,
								cand->mean_snr, cand->fluctuation, cand->dip_snr,
								old->freq, old->bw/1000, old->bw%1000, old->mean_level,
								old->mean_snr, old->fluctuation, old->dip_snr,
								si->w);
				is_better_than(old, cand);
			}
#endif
				return -1;
		} else if(old_is_better<0) { // cand is better
#ifdef DEBUGXXX
			if(print) {
				dprintk("Overwriting peak: new: %dkHz BW=%d.%03dMHz "
								"lvl=%d snr=%d fluct=%d dip =%d"
								" old: %dkHz BW=%d.%03dMHz lvl=%d snr=%d fluct=%d dip=%d w=%d\n",
								cand->freq, cand->bw/1000, cand->bw%1000, cand->mean_level,
								cand->mean_snr, cand->fluctuation, cand->dip_snr,
								old->freq, old->bw/1000, old->bw%1000, old->mean_level,
								old->mean_snr, old->fluctuation, old->dip_snr,
								si->w);
				is_better_than(old, cand);
			}
#endif //TODO: handle equal tps

			memmove(&si->peaks[i], &si->peaks[i + 1], sizeof(si->peaks[0]) * (si->num_peaks - i - 1));
			--i;
			si->num_peaks--;
#ifdef DEBUGXXX
			si->check();
#endif
			continue;
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

static void falling_kernel(spectrum_scan_state_t* ss, struct scan_internal_t* si,
													 s32* response_ret) {
	s32 delta = (si->w * 16) / 200;
	s32 w = si->w;
	int n = ss->spectrum_len;
	int i;
	int besti = -1;
	s32 best = 0;
	if(delta <= 2 || 2*delta >= si->w)
		delta = 1;
	if (delta == 0)
		delta = 1;
	for (i = n - delta -1; i >= w; --i) {
		s32 power = (si->rs[i] - si->rs[i - w]) / w;
		s32 right = ss->spectrum[i + delta];
		s32 response = power - right;
		if (response > ss->threshold) {
			// mark complete peak if not already on a peak
			if(besti >=0 && besti - i >= si->w/4)
				besti = -1;
			if(besti >=0 && best <= response /* && response > 2 * ss->threshold*/) {
				si->peak_marks[besti] &= ~FALLING;
				if (response_ret)
					response_ret[besti] = 0;
				besti = -1;
			}

			if(besti >= 0 && best  > response)
				continue; //current peak is weak and overlaps with previous one

			si->peak_marks[i] |= FALLING;
			if (response_ret)
				response_ret[i] = response;
			if(response > best || besti<0) {
				besti = i;
				best = response;
			}
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
static void rising_kernel(spectrum_scan_state_t* ss, struct scan_internal_t* si,
													s32* response_ret) {
	s32 delta = (si->w * 16) / 200; // rise interval
	s32 w = si->w;							// plateau interval
	int n = ss->spectrum_len;
	int i;
	int besti = -1;
	s32 best = 0;
	if(delta <= 2 || 2*delta >= si->w)
		delta = 1;
	if (delta == 0)
		delta = 1;
	for (i = delta; i <= n - w -1; ++i) {
		s32 power = (si->rs[i + w] - si->rs[i]) / w;
		s32 left = ss->spectrum[i - delta];
		s32 response = power - left;
		if (response > ss->threshold) {
			// erase small overlapping peaks
			if(besti >=0 && i - besti >= si->w/4)
				besti = -1;
			if(besti >=0 && best <= response /* && response > 2 * ss->threshold*/) {
				si->peak_marks[besti] &= ~RISING;
				if (response_ret)
					response_ret[besti] = 0;
				besti = -1;
			}

			if(besti >= 0 && best  > response)
				continue; //current peak is weak and overlaps with previous one

			si->peak_marks[i] |= RISING;
			if (response_ret)
				response_ret[i] = response;
			if(response > best || besti<0) {
				besti = i;
				best = response;
			}
		}
	}
}


void spectral_init_level(spectrum_scan_state_t* ss,
																 struct scan_internal_t* si,
																 s32* falling_response_ret,
																 s32* rising_response_ret) {
	si->start_idx = (si->w * 16) / 200;
	if (si->start_idx == 0)
		si->start_idx++;
	si->end_idx = ss->spectrum_len - (si->w * 16) / 200;
	if (si->end_idx == ss->spectrum_len)
		si->end_idx--;
	si->last_peak.idx = -1;
	memset(si->peak_marks, 0, sizeof(si->peak_marks[0]) * ss->spectrum_len);
	falling_kernel(ss, si, falling_response_ret);
	rising_kernel(ss, si, rising_response_ret);
}

static inline int find_falling(struct scan_internal_t* si, int start, int end) {
	int i;
	for(i = start; i < end; ++i) {
		if (si->peak_marks[i] & FALLING) {
			return i;
		}
	}
	return -1;
}

static inline int find_rising(struct scan_internal_t* si, int start, int end) {
	int i;
	for(i = end; i > start; --i) {
		if (si->peak_marks[i] & RISING) {
			return i;
		}
	}
	return -1;
}


static int process_candidate(spectrum_scan_state_t* ss, struct scan_internal_t* si,
														 int rise_idx, int fall_idx) {
	s32 delta, left, right, lowest_left, lowest_right;
	int i, lowest_left_i, lowest_right_i, left_3db_i, right_3db_i;
	s32 thresh;
	int count;
	int left_top_i, right_top_i;
	delta = (si->w * 16) / 100; //15% one sided extension
	if(delta <= 2 || 2*delta >= si->w)
		delta = 1;
	if (delta == 0)
		delta = 1;
	//compute average height of peak
	si->last_peak.mean_level =
		(si->rs[fall_idx] - si->rs[rise_idx]) / (fall_idx - rise_idx);
	//the factor 25/102 approximates 1/sqrt(6)
	si->last_peak.fluctuation = (25*(si->noise[fall_idx] - si->noise[rise_idx])) /(102*(fall_idx - rise_idx));
	//compute threshold for bandwidth computation
	thresh = si->last_peak.mean_level -3000;

	//comute the deepest values near the peak
	left = rise_idx - delta;
	if(left < 0)
		left = 0;
	right = fall_idx + delta;
	if(right > ss->spectrum_len)
		right = ss->spectrum_len;

	lowest_left = ss->spectrum[left];
	lowest_left_i = left;
	lowest_right = ss->spectrum[right-1];
	lowest_right_i = right - 1;

	for(i=left ; i < rise_idx ; ++i) {
		if(lowest_left > ss->spectrum[i]) {
			lowest_left = ss->spectrum[i];
			lowest_left_i = i;
		}
	}
	for(i=fall_idx+1; i < right ; ++i) {
		if(lowest_right > ss->spectrum[i]) {
			lowest_right = ss->spectrum[i];
			lowest_right_i = i;
		}
	}

	//compute the 3dB limits of the peak
	for(i=lowest_left_i, count=0; i < lowest_right_i; ++i) {
		if(ss->spectrum[i] >= thresh) {
			if(count==0)
				left_3db_i = i;
			if(++count >= si->w/8) //require at least si->w //8 values above threshold
				break;
		} else
			count=0;
	}
	if(count==0)
		left_3db_i = lowest_left_i;

	for(i=lowest_right_i, count=0; i >= lowest_left_i; --i) {
		if(ss->spectrum[i] >= thresh) {
			if(count==0)
				right_3db_i = i;
			if(++count >= si->w/8) //require at least si->w //8 values above threshold
				break;
		} else
			count = 0;
	}
	if(count==0)
		right_3db_i = lowest_right_i;

#if 1
	//compute bounds of top plateau
	for(i=left_3db_i; i <= right_3db_i; ++i) {
		if(ss->spectrum[i] >= si->last_peak.mean_level)
			break;
	}

	left_top_i = i;
	for(i=right_3db_i; i >= left_top_i; --i) {
		if(ss->spectrum[i] >= si->last_peak.mean_level)
			break;
	}
	right_top_i = i;
#endif
	if(right_top_i > left_top_i) {
		//recompute average height of peak
		si->last_peak.mean_level =
			(si->rs[right_top_i] - si->rs[left_top_i]) / (right_top_i - left_top_i);
	}


	//recompute the 3dB limits of the peak
	thresh = si->last_peak.mean_level -3000;
	for(i=lowest_left_i, count=0; i < lowest_right_i; ++i) {
		if(ss->spectrum[i] >= thresh) {
			if(count==0)
				left_3db_i = i;
			if(++count >= si->w/8) //require at least si->w //8 values above threshold
				break;
		} else
			count=0;
	}
	if(count==0)
		left_3db_i = lowest_left_i;

	for(i=lowest_right_i, count=0; i >= lowest_left_i; --i) {
		if(ss->spectrum[i] >= thresh) {
			if(count==0)
				right_3db_i = i;
			if(++count >= si->w/8) //require at least si->w //8 values above threshold
				break;
		} else
			count = 0;
	}
	if(count==0)
		right_3db_i = lowest_right_i;

#if 0
	//compute the lowest dip level in central peak
	si->last_peak.dip_level = ss->spectrum[left_3db_i];
	for(i=left_3db_i; i <= right_3db_i; ++i) {
		if(si->last_peak.dip_level > ss->spectrum[i])
			si->last_peak.dip_level = ss->spectrum[i];
	}
#else
	//compute the lowest dip level in central peak
	si->last_peak.dip_level = ss->spectrum[left_top_i];
	for(i=left_top_i; i <= right_top_i; ++i) {
		if(si->last_peak.dip_level > ss->spectrum[i])
			si->last_peak.dip_level = ss->spectrum[i];
	}
#endif
	si->last_peak.idx = (right_3db_i + left_3db_i) / 2;
	si->last_peak.dip_snr = si->last_peak.dip_level -  dtmin(lowest_right, lowest_left);
	si->last_peak.freq = ss->freq[si->last_peak.idx]; // in kHz
	si->last_peak.bw = ss->freq[right_3db_i] - ss->freq[left_3db_i]; // in kHz
	si->last_peak.fall_idx = right_3db_i;
	si->last_peak.rise_idx = left_3db_i;
	si->last_peak.lowest_left_idx = lowest_left_i;
	si->last_peak.lowest_right_idx = lowest_right_i;
	si->last_peak.lowest_left_level = lowest_left;
	si->last_peak.lowest_right_level = lowest_right;
	si->last_peak.mean_snr = si->last_peak.mean_level - dtmin(lowest_right, lowest_left);
#ifdef DEBUGXXX
	dprintk("CANDIDATE1: %d %dkHz [%d - %d] "
					"BW=%dkHz snr=%dmdB "
					"level=[%d %d %d]mdB w=%d\n",
					si->last_peak.idx, si->last_peak.freq, ss->freq[lowest_left_i], ss->freq[lowest_right_i],
					si->last_peak.bw, si->last_peak.mean_snr,
					ss->spectrum[lowest_left_i], si->last_peak.mean_level, ss->spectrum[lowest_right_i],
					si->w);
#endif

	return 0;
}

static int spectral_scan_init(spectrum_scan_state_t* ss, struct scan_internal_t* si) {
	si->max_num_peaks = 1024*4;

	ss->scan_in_progress = true;

	// ss->w =17;
	ss->snr_w = 35; //percentage
	// ss->threshold = 2000;
	// ss->mincount = 3;
	dprintk("ALLOCATING: spectrum_len=%d max_num_peaks=%d\n", ss->spectrum_len, si->max_num_peaks);
	if (!ss->spectrum || ss->spectrum_len ==0) {
		return -ENOMEM;
	}
	si->rs = kzalloc(ss->spectrum_len * (sizeof(si->rs[0])), GFP_KERNEL);
	si->noise = kzalloc(ss->spectrum_len * (sizeof(si->noise[0])), GFP_KERNEL);
	si->peak_marks = kzalloc(ss->spectrum_len * (sizeof(si->peak_marks[0])), GFP_KERNEL);
	si->peaks = (struct spectrum_peak_internal_t*) kzalloc(si->max_num_peaks * (sizeof(si->peaks[0])), GFP_KERNEL);

	running_sum(si->rs, ss->spectrum, ss->spectrum_len);
	noise_est(si->noise, ss->spectrum, ss->spectrum_len);
	si->num_peaks = 0;
	return 0;
}

static void spectral_scan_end(struct scan_internal_t* si) {
	if(si->peak_marks)
		kfree(si->peak_marks);
	si->peak_marks = NULL;
	if(si->peaks)
		kfree(si->peaks);
	si->peaks = NULL;
	if(si->rs)
		kfree(si->rs);
	si->rs = NULL;
	if(si->noise)
		kfree(si->noise);
	si->noise =NULL;
}

static int scan_level(spectrum_scan_state_t *ss, struct scan_internal_t *si,
											s32* spectrum, u32* freq, int spectrum_len, int w) {
	int fall_idx;
	int rise_idx;
	si->w = w;
	spectral_init_level(ss, si, NULL, NULL);
	fall_idx = find_falling(si, si->start_idx, si->end_idx);
	for(; fall_idx < si->end_idx && fall_idx >=0;
			fall_idx = find_falling(si, fall_idx+1, si->end_idx)) {
		//find  a pair of rising and falling peaks, separated by 1 to 1.5 times si.w
		int ll = fall_idx - (si->w * 150)/100;
		if (ll < si->start_idx)
			ll  = si->start_idx;
		rise_idx = find_rising(si, ll, fall_idx - si->w);
		for(; rise_idx >= ll && rise_idx>=0; rise_idx = find_rising(si, ll, rise_idx-1)) {
			//compute parameters of the candidate peak
			process_candidate(ss, si, rise_idx, fall_idx);
			if (check_candidate_tp(ss, si) >= 0) {
				si->peaks[si->num_peaks++] = si->last_peak;
				if (si->num_peaks >= si->max_num_peaks)
					return -1;
			}
		}
	}
	return 0;
}

static int scan_all(spectrum_scan_state_t *ss, struct scan_internal_t *si,
										 s32* spectrum, u32* freq, int spectrum_len) {
	int window_idx=0;
	spectral_scan_init(ss, si);
	for(window_idx=0; window_idx < (int)(sizeof(windows) / sizeof(windows[0])); ++window_idx) {
		int w = windows[window_idx];
		scan_level(ss, si, spectrum, freq, spectrum_len, w);
	}
	return 0;
}

static int cmp_fn(const void* pa, const void* pb) {
	struct spectrum_peak_t* a = (struct spectrum_peak_t*)pa;
	struct spectrum_peak_t* b = (struct spectrum_peak_t*)pb;
	return a->freq - b->freq;
}

/*
	spectrum is stored in ss on input
 */
int neumo_scan_spectrum(spectrum_scan_state_t* ss)
{
	struct scan_internal_t si;
	int j = 0;
	ss->threshold = 3000;
	ss->threshold2 = 3000;
	ss->mincount = 1;
	memset(&si, 0, sizeof(si));
	scan_all(ss, &si, ss->spectrum, ss->freq, ss->spectrum_len);
	dprintk("scan_all done\n");

	if(ss->candidates)
		kfree(ss->candidates);
	ss->num_candidates = si.num_peaks;
	ss->candidates = kzalloc(ss->num_candidates * (sizeof(ss->candidates[0])), GFP_KERNEL);
	dprintk("allocation done num_candidates=%d\n", ss->num_candidates);
	for (j = 0; j < si.num_peaks; ++j) {
		struct spectral_peak_t* p = & ss->candidates[j];
		p->freq= si.peaks[j].freq;
		p->symbol_rate = si.peaks[j].bw*1000; //top plateau approx SR. 10% underestimation
		p->snr = si.peaks[j].mean_snr;
		p->level = si.peaks[j].mean_level;
	}
	dprintk("starting to sort\n");
	sort(&ss->candidates[0], ss->num_candidates, sizeof(ss->candidates[0]), cmp_fn, NULL);
	dprintk("sort done\n");
	spectral_scan_end(&si);
	dprintk("returning\n");
	return 0;
}
