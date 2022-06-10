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
#include <linux/kthread.h>
#include "stid135_drv.h"
#include "stid135_init.h"
#include "stid135_initLLA_cut2.h"
#include "c8codew_addr_map.h"
#include "stid135_addr_map.h"
extern unsigned int stid135_verbose;
extern void chip_init_proc(STCHIP_Info_t* hChipHandle_, const char*name);
void chip_close_proc(const char* name);


#define DmdLock_TIMEOUT_LIMIT      5500  // Fixed issue BZ#86598
//#define BLIND_SEARCH_AGC2BANDWIDTH  40
#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)
#define vprintk(fmt, arg...)																					\
	if(stid135_verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

#define LNF_IP3_SWITCH_LOW  0x3C00
#define LNF_IP3_SWITCH_HIGH 0x8000
#define MODE_LNF 1
#define MODE_IP3 0

#define SNR_MIN_THRESHOLD 9
#define SNR_MAX_THRESHOLD 11

// Code useful for RF level estimation
#define POWER_IREF 95 //105

static u8  mc_mask_bitfield[FE_SAT_MODCODE_MAX];
static u16 mc_reg_addr[FE_SAT_MODCODE_MAX][8]; //TODO

void tst(struct fe_stid135_internal_param* pParams, u32 reg_value, const char* func, int line) {
	if (reg_value & ~0x11111) {
		int error;
		printk(KERN_DEBUG pr_fmt("%s:%d " "CONFIG1000  corrupt: 0x%x (rereading)\n"),  func, line, reg_value);
		error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, &reg_value);
		printk(KERN_DEBUG pr_fmt("%s:%d " "CONFIG1000  reread: re-read=0x%x error=%d\n"),  func, line, reg_value, error);
		reg_value= 0x11111;
		error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, reg_value);
		if(error) {
			printk(KERN_DEBUG pr_fmt("%s:%d " "CONFIG1000  corrupt: correction failed: error=%d\n"),  func, line, error);
		}
	}
}

static struct mc_array_customer st_mc_flt[FE_SAT_MODCODE_MAX] = {
	// MODCODE, SNRx100
	{FE_SATX_QPSK_11_45, -230},
	{FE_SAT_QPSK_14, -230},
	{FE_SATX_QPSK_13_45, -190},
	{FE_SATX_QPSK_4_15, -170},
	{FE_SAT_QPSK_13, -130},
	{FE_SATX_QPSK_14_45, -110},
	{FE_SAT_QPSK_25, -30},
	{FE_SATX_QPSK_9_20, 30},
	{FE_SAT_QPSK_12, 70},
	{FE_SATX_QPSK_7_15, 80},
	{FE_SATX_QPSK_11_20, 160},
	{FE_SATX_QPSK_8_15, 160},
	{FE_SAT_QPSK_35, 230},
	{FE_SAT_QPSK_23, 310},
	{FE_SATX_QPSK_32_45, 390},
	{FE_SAT_QPSK_34, 400},
	{FE_SATX_8PSK_7_15, 420},
	{FE_SAT_QPSK_45, 470},
	{FE_SATX_8PSK_8_15, 500},
	{FE_SAT_QPSK_56, 520},
	{FE_SATX_8APSK_5_9_L, 520},
	{FE_SATX_8APSK_26_45_L, 560},
	{FE_SAT_8PSK_35, 570},
	{FE_SATX_8PSK_26_45, 590},
	{FE_SATX_16APSK_1_2_L, 610},
	{FE_SAT_QPSK_89, 620},
	{FE_SATX_8PSK_23_36, 620},
	{FE_SAT_QPSK_910, 640},
	{FE_SAT_8PSK_23, 660},
	{FE_SATX_16APSK_7_15, 660},
	{FE_SATX_16APSK_8_15_L, 670},
	{FE_SATX_16APSK_5_9_L, 700},
	{FE_SATX_8PSK_25_36, 710},
	{FE_SATX_16APSK_8_15, 750},
	{FE_SATX_8PSK_13_18, 760},
	{FE_SATX_16APSK_26_45, 770},
	{FE_SATX_16APSK_3_5_L, 770},
	{FE_SAT_8PSK_34, 780},
	{FE_SATX_8PSK_32_45, 780},
	{FE_SATX_16APSK_26_45_S, 800},
	{FE_SATX_16APSK_3_5, 810},
	{FE_SATX_16APSK_28_45, 840},
	{FE_SATX_16APSK_23_36, 850},
	{FE_SATX_16APSK_3_5_S, 960},
	{FE_SATX_16APSK_2_3_L, 870},
	{FE_SAT_16APSK_23, 900},
	{FE_SAT_8PSK_56, 930},
	{FE_SATX_16APSK_25_36, 940},
	{FE_SATX_16APSK_13_18, 990},
	{FE_SATX_16APSK_32_45, 1030},
	{FE_SAT_16APSK_34, 1040},
	{FE_SAT_8PSK_89, 1050},
	{FE_SATX_16APSK_7_9, 1070},
	{FE_SAT_8PSK_910, 1080},
	{FE_SAT_16APSK_45, 1110},
	{FE_SAT_16APSK_56, 11600},
	{FE_SATX_32APSK_2_3_L, 11700},
	{FE_SATX_16APSK_77_90, 1200},
	{FE_SATX_32APSK_2_3, 1210},
	{FE_SATX_32APSK_32_45, 1250},
	{FE_SAT_16APSK_89, 1290},
	{FE_SATX_32APSK_11_15, 1290},
	{FE_SATX_32APSK_32_45_S, 1300},
	{FE_SAT_16APSK_910, 1320},
	{FE_SAT_32APSK_34, 1340},
	{FE_SATX_32APSK_7_9, 1370},
	{FE_SAT_32APSK_45, 1430},
	{FE_SAT_32APSK_56, 1490},
	{FE_SATX_64APSK_32_45_L, 1500},
	{FE_SATX_64APSK_11_15, 1600},
	{FE_SAT_32APSK_89, 1650},
	{FE_SAT_32APSK_910, 1690},
	{FE_SATX_64APSK_7_9, 1710},
	{FE_SATX_64APSK_4_5, 1740},
	{FE_SATX_64APSK_5_6, 1820},
	{FE_SATX_256APSK_29_45_L, 1890},
	{FE_SATX_128APSK_3_4, 1920},
	{FE_SATX_256APSK_32_45, 1920},
	{FE_SATX_256APSK_2_3_L, 1950},
	{FE_SATX_128APSK_7_9, 1970},
	{FE_SATX_256APSK_31_45_L, 2020},
	{FE_SATX_256APSK_11_15_L, 2030},
	{FE_SATX_256APSK_3_4, 2060}
};


static u32 LutGvanaIntegerTuner[256]={
	93487, 93305, 93201, 93274, 93202, 93316, 93319, 93252, 92752, 93298,
	93264, 93296, 93328, 93315, 93056, 93264, 93320, 93250, 93238, 92295,
	92289, 92317, 92248, 92247, 92160, 92294, 92177, 92266, 92236, 91284,
	91281, 91241, 91177, 91266, 91101, 90906, 90269, 90212, 90161, 90156,
	89737, 89249, 89187, 89141, 88726, 88180, 88153, 88077, 87403, 87179,
		87009, 86343, 86159, 86084, 85236, 85142, 84892, 84190, 84163, 83905,
		83191, 83170, 82753, 82192, 82237, 82221, 82206, 82191, 82175, 82160,
		82244, 81610, 81766, 81257, 81173, 81240, 80608, 80474, 80310, 80152,
		79763, 79315, 79216, 78844, 78596, 78246, 78041, 77663, 77503, 77188,
		76988, 76658, 76308, 76118, 75884, 75654, 75301, 75059, 74921, 74654,
		74196, 73898, 73761, 73749, 73269, 72920, 73004, 72454, 72332, 72078,
		71956, 71653, 71278, 71091, 71099, 70689, 70412, 70243, 70029, 70030,
		69584, 69207, 69175, 68916, 68920, 68416, 68209, 67959, 68076, 67758,
		67259, 67237, 67039, 67074, 66832, 66211, 66347, 65984, 65998, 65751,
		65170, 65436, 65035, 65086, 64862, 64365, 64238, 64245, 64089, 63970,
		63432, 63126, 63377, 62986, 62915, 62640, 62203, 62421, 61991, 62235,
		61703, 61332, 61175, 61178, 61029, 60691, 60597, 60271, 60118, 60234,
		59916, 59760, 59323, 59133, 59342, 59025, 58894, 58476, 58328, 57986,
		58407, 57945, 57666, 57391, 57235, 57735, 57070, 56676, 57088, 56301,
		56330, 56239, 55825, 55515, 55243, 55211, 54608, 54257, 54153, 53583,
		53286, 53372, 52810, 52243, 52259, 51731, 51267, 51354, 50613, 50294,
		50259, 49644, 49342, 49245, 48834, 48324, 48241, 47986, 47377, 47363,
		47234, 46803, 46722, 46185, 46315, 46414, 45640, 45702, 45219, 45333,
		45179, 45699, 44951, 44933, 44525, 44199, 44706, 44172, 44203, 44689,
		43869, 45183, 44692, 43704, 43194, 43548, 43181, 43179, 43177, 43198,
		43220, 43207, 43195, 43194, 43282, 43370
};

// End of code useful for RF level estimation

//==============================================================================
// Types

struct fe_sat_car_loop_vs_mode_code {
	enum fe_sat_modcode ModCode;
	u8 CarLoopPilotsOn_2;
	u8 CarLoopPilotsOff_2;
	u8 CarLoopPilotsOn_5;
	u8 CarLoopPilotsOff_5;
	u8 CarLoopPilotsOn_10;
	u8 CarLoopPilotsOff_10;
	u8 CarLoopPilotsOn_20;
	u8 CarLoopPilotsOff_20;
	u8 CarLoopPilotsOn_30;
	u8 CarLoopPilotsOff_30;
	u8 CarLoopPilotsOn_62_5;
	u8 CarLoopPilotsOff_62_5;
	u8 CarLoopPilotsOn_125;
	u8 CarLoopPilotsOff_125;
	u8 CarLoopPilotsOn_300;
	u8 CarLoopPilotsOff_300;
	u8 CarLoopPilotsOn_400;
	u8 CarLoopPilotsOff_400;
	u8 CarLoopPilotsOn_500;
	u8 CarLoopPilotsOff_500;
};

/*****************************************************
--STRUCTURE	::	FE_STiD135_S2_CN_LookUp
--DESCRIPTION	::	DVBS2 C/N Look-Up table.
	Lookup table used to convert register
	to C/N value for DVBS2.
--***************************************************/
static fe_lla_lookup_t  FE_STiD135_S2_CN_LookUp = {
51,
	{
		/* SFE(dB), Noise DVBS2 */

		{-32, 13393 }, { -22, 12561 }, {-17, 12200 },
		{-12, 11767 }, {-2, 11030 }, {8, 10208 },
		{18, 9441 }, {28, 8695 }, {38, 7944 },
		{48, 7275 }, {58, 6607 }, {68, 5992 },
		{78, 5384 }, {88, 4851 }, {98, 4357 },
		{108, 3936 }, {118, 3488 }, {128, 3134 },
		{138, 2795 }, {148, 2484 }, {158, 2214 },
		{168, 1997 }, {178, 1767 }, {188, 1585 },
		{198, 1401 }, {208, 1264 }, {218, 1118 },
		{228, 996 }, {238, 896 }, {248, 798 },
		{258, 713 }, {268, 645 }, {278, 596 },
		{288, 591 }, {298, 558 }, {308, 493 },
		{318, 459 }, {328, 430 }, {338, 415 },
		{348, 400 }, {358, 385 }, {368, 382 },
		{378, 380 }, {388, 371 }, {398, 339 },
		{498, 280 }, {598, 230 }, {698, 205 },
		{798, 185 }, {898, 170 }, {998, 160 }
	}
};


/*****************************************************
--STRUCTURE	::	FE_STiD135_S1_CN_LookUp
--DESCRIPTION	::	DVBS1 C/N Look-Up table.
	Lookup table used to convert register
	to C/N value for DVBS1/DSS.
--***************************************************/
static fe_lla_lookup_t  FE_STiD135_S1_CN_LookUp = {
34,
	{
		/* SFE(dB) Noise DVBS1  */

		{  -2, 9200 }, {   8, 8800 }, {  18, 8350 }, {  28, 7920 },
		{  38, 7492 }, {  48, 7010 }, {  58, 6475 }, {  68, 5920 },
		{  78, 5378 }, {  88, 4848 }, {  98, 4370 }, { 108, 3909 },
		{ 118, 3500 }, { 128, 3125 }, { 138, 2788 }, { 148, 2485 },
		{ 158, 2226 }, { 168, 1975 }, { 178, 1759 }, { 188, 1573 },
		{ 198, 1400 }, { 208, 1247 }, { 218, 1112 }, { 228, 1000 },
		{ 238,  890 }, { 248,  800 }, { 258,  710 }, { 268,  636 },
		{ 278,  581 }, { 288,  524 }, { 298,  478 }, { 308,  440 },
		{ 318,  394 }, { 328,  355 }
	}
};

/*****************************************************
--STRUCTURE	::	fe_stid135_avs_look_up
--DESCRIPTION	::	AVS Look-Up table.
	Lookup table used to convert process code
	to PWM value (AVS voltage)
--***************************************************/
static fe_lla_lookup_t fe_stid135_avs_look_up = {
16,
	{
		// On Oxford valid board :
		// register = 0x00 => Vavs=1.297V
		// register = 0xFF => Vavs=0.789V
		{ 0x30, 0 }, // 1.2V
		{ 0x30, 1 }, // 1.2V
		{ 0x3a, 2 }, // 1.18V
		{ 0x44, 3 }, // 1.16V
		{ 0x4e, 4 }, // 1.14V
		{ 0x56, 5 }, // 1.125V
		{ 0x5d, 6 }, // 1.11V
		{ 0x62, 7 }, // 1.1V
		{ 0x67, 8 }, // 1.09V
		{ 0x6e, 9 }, // 1.077V
		{ 0x75, 10 }, // 1.063V
		{ 0x7b, 11 }, // 1.05V
		{ 0x83, 12 }, // 1.035V
		{ 0x8b, 13 }, // 1.02V
		{ 0x90, 14 }, // 1.01V
		{ 0x95, 15 } // 1.0V
	}
};


/*****************************************************************
Tracking carrier loop carrier QPSK 1/4   to QPSK 2/5    Normal Frame
Tracking carrier loop carrier QPSK 1/2   to 8PSK 9/10   Normal Frame
Tracking carrier loop carrier 16APSK 2/3 to 32APSK 9/10 Normal Frame
******************************************************************/
static struct fe_sat_car_loop_vs_mode_code FE_STiD135_S2CarLoop[NB_SAT_MODCOD] = {
/*Modcod		2MPon		2MPoff	5MPon		5MPoff	10MPon	10MPoff	20MPon	20MPoff	30MPon	30MPoff	62.5MPon	62.5MPoff	125MPon	125MPoff	300MPon	300MPoff	400MPon	400MPoff	500MPon	500MPoff*/
/* Low CR QPSK */
	{FE_SAT_QPSK_14,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},
	{FE_SAT_QPSK_13,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},
	{FE_SAT_QPSK_25,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},
/* QSPSK */
	{FE_SAT_QPSK_12,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},
	{FE_SAT_QPSK_35,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},
	{FE_SAT_QPSK_23,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},
	{FE_SAT_QPSK_34,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SAT_QPSK_45,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SAT_QPSK_56,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SAT_QPSK_89,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SAT_QPSK_910,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
/* QPSK S2X NORMAL FRAME */
	{FE_SATX_QPSK_13_45,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},
	{FE_SATX_QPSK_9_20,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},
	{FE_SATX_QPSK_11_20,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},
/* QPSK S2X SHORT FRAME */
	{FE_SATX_QPSK_11_45,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},
	{FE_SATX_QPSK_4_15,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},
	{FE_SATX_QPSK_14_45,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},
	{FE_SATX_QPSK_7_15,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},
	{FE_SATX_QPSK_8_15,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},
	{FE_SATX_QPSK_32_45,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
/* 8SPSK */
	{FE_SAT_8PSK_35,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},
	{FE_SAT_8PSK_23,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SAT_8PSK_34,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SAT_8PSK_56,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SAT_8PSK_89,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SAT_8PSK_910,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
/* 8PSK S2X NORMAL FRAME */
	{FE_SATX_8PSK_23_36,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},
	{FE_SATX_8PSK_25_36,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SATX_8PSK_13_18,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
/* 8PSK S2X SHORT FRAME */
	{FE_SATX_8PSK_7_15,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26},
	{FE_SATX_8PSK_8_15,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26},
	{FE_SATX_8PSK_26_45,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},
	{FE_SATX_8PSK_32_45,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
/* 8PSK_L S2X NORMAL FRAME */
	{FE_SATX_8APSK_5_9_L,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26},
	{FE_SATX_8APSK_26_45_L,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},
/* 16ASPSK */
	{FE_SAT_16APSK_23,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SAT_16APSK_34,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SAT_16APSK_45,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SAT_16APSK_56,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SAT_16APSK_89,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SAT_16APSK_910,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
/* 16APSK S2X NORMAL FRAME */
	{FE_SATX_16APSK_26_45,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_3_5,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_28_45,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_23_36,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_25_36,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_13_18,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_7_9,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_77_90,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
/* 16APSK S2X SHORT FRAME */
	{FE_SATX_16APSK_7_15,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_8_15,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_26_45_S,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_3_5_S,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_32_45,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
/* 16APSK_L S2X NORMAL FRAME */
	{FE_SATX_16APSK_1_2_L,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_8_15_L,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_5_9_L, 	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_3_5_L,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SATX_16APSK_2_3_L, 	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
/* 32ASPSK */
	{FE_SAT_32APSK_34,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
	{FE_SAT_32APSK_45,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SAT_32APSK_56,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SAT_32APSK_89,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SAT_32APSK_910,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
/* 32APSK S2X NORMAL FRAME */
	{FE_SATX_32APSK_32_45,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SATX_32APSK_11_15,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SATX_32APSK_7_9,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
/* 32APSK S2X SHORT FRAME */
	{FE_SATX_32APSK_2_3,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
	{FE_SATX_32APSK_32_45_S,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},
/* 32APSK_L S2X NORMAL FRAME */
	{FE_SATX_32APSK_2_3_L,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},
/* 64APSK S2X NORMAL FRAME */
	{FE_SATX_64APSK_11_15,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SATX_64APSK_7_9,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SATX_64APSK_4_5,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SATX_64APSK_5_6,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
/* 64APSK_L S2X NORMAL FRAME */
	{FE_SATX_64APSK_32_45_L,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
/* 128APSK S2X NORMAL FRAME */
	{FE_SATX_128APSK_3_4,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SATX_128APSK_7_9,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
/* 256APSK S2X NORMAL FRAME */
	{FE_SATX_256APSK_32_45,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SATX_256APSK_3_4,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SATX_256APSK_29_45_L,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SATX_256APSK_2_3_L,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SATX_256APSK_31_45_L,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},
	{FE_SATX_256APSK_11_15_L,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09}
};

#define NB_QPSK_COEFF 20
#define NB_8PSK_COEFF 15
#define NB_16APSK_COEFF 24
#define NB_32APSK_COEFF 11
#define NB_64APSK_COEFF 5
#define NB_128APSK_COEFF 2
#define NB_256APSK_COEFF 6

static struct fe_sat_car_loop_vs_cnr fe_stid135_qpsk_car_loop[NB_QPSK_COEFF] =
{/* QPSK */
	/* CNRx10	0.5Msps	10Msps	30Msps	62.5Msps	125Msps	300Msps	400Msps	500Msps */
	{-23,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},	/* FE_SATX_QPSK_11_45 */
	{-22,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},	/* FE_SAT_QPSK_14 */
	{-18,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},	/* FE_SATX_QPSK_13_45 */
	{-16,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},	/* FE_SATX_QPSK_4_15 */
	{-12,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},	/* FE_SAT_QPSK_13 */
	{-10,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},	/* FE_SATX_QPSK_14_45 */
	{-2,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},	/* FE_SAT_QPSK_25 */
	{4,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},	/* FE_SATX_QPSK_9_20 */
	{9,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},	/* FE_SATX_QPSK_7_15 */
	{11,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},	/* FE_SAT_QPSK_12 */
	{17,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},	/* FE_SATX_QPSK_11_20 */
	{18,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},	/* FE_SATX_QPSK_8_15 */
	{24,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},	/* FE_SAT_QPSK_35 */
	{32,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},	/* FE_SAT_QPSK_23 */
	{40,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SATX_QPSK_32_45 */
	{41,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SAT_QPSK_34 */
	{48,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SAT_QPSK_45 */
	{53,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SAT_QPSK_56 */
	{63,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SAT_QPSK_89 */
	{65,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08}	/* FE_SAT_QPSK_910 */
};

static struct fe_sat_car_loop_vs_cnr fe_stid135_8psk_car_loop[NB_8PSK_COEFF] =
{/* 8PSK */
	/* CNRx10	0.5Msps	10Msps	30Msps	62.5Msps	125Msps	300Msps	400Msps	500Msps */
	{43,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26},	/* FE_SATX_8PSK_7_15 */
	{51,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26},	/* FE_SATX_8PSK_8_15 */
	{53,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26,	0x26},	/* FE_SATX_8APSK_5_9_L */
	{57,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},	/* FE_SATX_8APSK_26_45_L */
	{58,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07,	0x07},	/* FE_SAT_8PSK_35 */
	{60,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},	/* FE_SATX_8PSK_26_45 */
	{63,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27,	0x27},	/* FE_SATX_8PSK_23_36 */
	{67,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SAT_8PSK_23 */
	{72,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SATX_8PSK_25_36 */
	{77,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SATX_8PSK_13_18 */
	{79,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SATX_8PSK_32_45 */
	{80,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SAT_8PSK_34 */
	{94,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SAT_8PSK_56 */
	{106,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SAT_8PSK_89 */
	{109,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08}	/* FE_SAT_8PSK_910 */
};

static struct fe_sat_car_loop_vs_cnr fe_stid135_16apsk_car_loop[NB_16APSK_COEFF] =
{/* 16APSK */
	/* CNRx10	0.5Msps	10Msps	30Msps	62.5Msps	125Msps	300Msps	400Msps	500Msps */
	{62,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_1_2_L */
	{67,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_7_15 */
	{68,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_8_15_L */
	{71,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_5_9_L */
	{76,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_8_15 */
	{78,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_3_5_L */
	{79,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_26_45 */
	{81,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_26_45_S */
	{82,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_3_5 */
	{85,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_28_45 */
	{86,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_23_36 */
	{87,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_3_5_S */
	{88,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_2_3_L */
	{91,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SAT_16APSK_23 */
	{95,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_25_36 */
	{100,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_13_18 */
	{104,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_32_45 */
	{105,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SAT_16APSK_34 */
	{108,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_7_9 */
	{112,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SAT_16APSK_45 */
	{117,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SAT_16APSK_56 */
	{121,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_16APSK_77_90 */
	{130,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SAT_16APSK_89 */
	{133,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28}	/* FE_SAT_16APSK_910 */
};

static struct fe_sat_car_loop_vs_cnr fe_stid135_32apsk_car_loop[NB_32APSK_COEFF] =
{/* 32APSK */
		/* CNRx10	0.5Msps	10Msps	30Msps	62.5Msps	125Msps	300Msps	400Msps	500Msps */
	{118,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SATX_32APSK_2_3_L */
	{122,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SATX_32APSK_2_3 */
	{126,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SATX_32APSK_32_45 */
	{130,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08,	0x08},	/* FE_SATX_32APSK_11_15 */
	{131,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_32APSK_32_45_S */
	{135,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SAT_32APSK_34 */
	{138,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28,	0x28},	/* FE_SATX_32APSK_7_9 */
	{144,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SAT_32APSK_45 */
	{150,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SAT_32APSK_56 */
	{166,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SAT_32APSK_89 */
	{170,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09}	/* FE_SAT_32APSK_910 */
};

static struct fe_sat_car_loop_vs_cnr fe_stid135_64apsk_car_loop[NB_64APSK_COEFF] =
{/* 64APSK */
	/* CNRx10	0.5Msps	10Msps	30Msps	62.5Msps	125Msps	300Msps	400Msps	500Msps */
	{151,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_64APSK_32_45_L */
	{161,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_64APSK_11_15 */
	{172,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_64APSK_7_9 */
	{175,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_64APSK_4_5 */
	{183,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09}	/* FE_SATX_64APSK_5_6 */
};

static struct fe_sat_car_loop_vs_cnr fe_stid135_128apsk_car_loop[NB_128APSK_COEFF] =
{/* 128APSK */
	/* CNRx10	0.5Msps	10Msps	30Msps	62.5Msps	125Msps	300Msps	400Msps	500Msps */
	{193,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_128APSK_3_4 */
	{198,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09}	/* FE_SATX_128APSK_7_9 */
};

static struct fe_sat_car_loop_vs_cnr fe_stid135_256apsk_car_loop[NB_256APSK_COEFF] =
{/* 256APSK */
	/* CNRx10	0.5Msps	10Msps	30Msps	62.5Msps	125Msps	300Msps	400Msps	500Msps */
	{190,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_256APSK_29_45_L */
	{193,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_256APSK_32_45 */
	{196,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_256APSK_2_3_L */
	{203,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_256APSK_31_45_L */
	{204,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09},	/* FE_SATX_256APSK_11_15_L */
	{207,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09,	0x09}	/* FE_SATX_256APSK_3_4 */
};


static  u16 dvbs2_modcode_for_llr_x100[131] = {
	0,	//FE_SAT_DUMMY_PLF,
	400,	//FE_SAT_QPSK_14,
	300,	//FE_SAT_QPSK_13,
	250,	//FE_SAT_QPSK_25,
	200,	//FE_SAT_QPSK_12,
	167,	//FE_SAT_QPSK_35,
	150,	//FE_SAT_QPSK_23,
	133,	//FE_SAT_QPSK_34,
	125,	//FE_SAT_QPSK_45,
	120,	//FE_SAT_QPSK_56,
	113,	//FE_SAT_QPSK_89,
	111,	//FE_SAT_QPSK_910,
	167,	//FE_SAT_8PSK_35,
	150,	//FE_SAT_8PSK_23,
	133,	//FE_SAT_8PSK_34,
	120,	//FE_SAT_8PSK_56,
	113,	//FE_SAT_8PSK_89,
	111,	//FE_SAT_8PSK_910,
	150,	//FE_SAT_16APSK_23,
	133,	//FE_SAT_16APSK_34,
	125,	//FE_SAT_16APSK_45,
	120,	//FE_SAT_16APSK_56,
	113,	//FE_SAT_16APSK_89,
	111,	//FE_SAT_16APSK_910,
	133,	//FE_SAT_32APSK_34,
	125,	//FE_SAT_32APSK_45,
	120,	//FE_SAT_32APSK_56,
	113,	//FE_SAT_32APSK_89,
	111,	//FE_SAT_32APSK_910,
	0,		//FE_SAT_MODCODE_UNKNOWN,
	/* below modcodes are part of DVBS2x */

	0,	//FE_SAT_MODCODE_UNKNOWN_1E = 0x1E,
	0,	//FE_SAT_MODCODE_UNKNOWN_1F  = 0x1F,
	200,	//FE_SAT_DVBS1_QPSK_12 = 0x20,
	150,	//FE_SAT_DVBS1_QPSK_23 = 0x21,
	133,	//FE_SAT_DVBS1_QPSK_34 = 0x22,
	120,	//FE_SAT_DVBS1_QPSK_56 = 0x23,
	117,	//FE_SAT_DVBS1_QPSK_67 = 0x24,
	114,	//FE_SAT_DVBS1_QPSK_78 = 0x25,
	0,	//FE_SAT_MODCODE_UNKNOWN_26 = 0x26,
	0,	//FE_SAT_MODCODE_UNKNOWN_27 = 0x27,
	0,	//FE_SAT_MODCODE_UNKNOWN_28 = 0x28,
	0,	//FE_SAT_MODCODE_UNKNOWN_29 = 0x29,
	0,	//FE_SAT_MODCODE_UNKNOWN_2A = 0x2A,
	0,	//FE_SAT_MODCODE_UNKNOWN_2B = 0x2B,
	0,	//FE_SAT_MODCODE_UNKNOWN_2C = 0x2C,
	0,	//FE_SAT_MODCODE_UNKNOWN_2D = 0x2D,
	0,	//FE_SAT_MODCODE_UNKNOWN_2E = 0x2E,
	0,	//FE_SAT_MODCODE_UNKNOWN_2F = 0x2F,
	0,	//FE_SAT_MODCODE_UNKNOWN_30 = 0x30,
	0,	//FE_SAT_MODCODE_UNKNOWN_31 = 0x31,
	0,	//FE_SAT_MODCODE_UNKNOWN_32 = 0x32,
	0,	//FE_SAT_MODCODE_UNKNOWN_33 = 0x33,
	0,	//FE_SAT_MODCODE_UNKNOWN_34 = 0x34,
	0,	//FE_SAT_MODCODE_UNKNOWN_35 = 0x35,
	0,	//FE_SAT_MODCODE_UNKNOWN_36 = 0x36,
	0,	//FE_SAT_MODCODE_UNKNOWN_37 = 0x37,
	0,	//FE_SAT_MODCODE_UNKNOWN_38 = 0x38,
	0,	//FE_SAT_MODCODE_UNKNOWN_39 = 0x39,
	0,	//FE_SAT_MODCODE_UNKNOWN_3A = 0x3A,
	0,	//FE_SAT_MODCODE_UNKNOWN_3B = 0x3B,
	0,	//FE_SAT_MODCODE_UNKNOWN_3C = 0x3C,
	0,	//FE_SAT_MODCODE_UNKNOWN_3D = 0x3D,
	0,	//FE_SAT_MODCODE_UNKNOWN_3E = 0x3E,
	0,	//FE_SAT_MODCODE_UNKNOWN_3F = 0x3F,
	0,	//FE_SATX_VLSNR1		  = 0x40,
	0,	//FE_SATX_VLSNR2		= 0x41,
	346,	//FE_SATX_QPSK_13_45	  = 0x42,
	222,	//FE_SATX_QPSK_9_20 	= 0x43,
	182,	//FE_SATX_QPSK_11_20	  = 0x44,
	180,	//FE_SATX_8APSK_5_9_L	= 0x45,
	173,	//FE_SATX_8APSK_26_45_L   = 0x46,
	157,	//FE_SATX_8PSK_23_36	= 0x47,
	144,	//FE_SATX_8PSK_25_36	  = 0x48,
	138,	//FE_SATX_8PSK_13_18	= 0x49,
	200,	//FE_SATX_16APSK_1_2_L	  = 0x4A,
	188,	//FE_SATX_16APSK_8_15_L = 0x4B,
	180,	//FE_SATX_16APSK_5_9_L	  = 0x4C,
	173,	//FE_SATX_16APSK_26_45	= 0x4D,
	167,	//FE_SATX_16APSK_3_5	  = 0x4E,
	167,	//FE_SATX_16APSK_3_5_L	= 0x4F,
	161,	//FE_SATX_16APSK_28_45	  = 0x50,
	157,	//FE_SATX_16APSK_23_36	= 0x51,
	150,	//FE_SATX_16APSK_2_3_L	  = 0x52,
	144,	//FE_SATX_16APSK_25_36	= 0x53,
	138,	//FE_SATX_16APSK_13_18	  = 0x54,
	129,	//FE_SATX_16APSK_7_9	= 0x55,
	117,	//FE_SATX_16APSK_77_90	  = 0x56,
	150,	//FE_SATX_32APSK_2_3_L	= 0x57,
	0,	//FE_SATX_32APSK_R_58	  = 0x58,
	141,	//FE_SATX_32APSK_32_45	= 0x59,
	136,	//FE_SATX_32APSK_11_15	  = 0x5A,
	129,	//FE_SATX_32APSK_7_9	= 0x5B,
	141,	//FE_SATX_64APSK_32_45_L  = 0x5C,
	136,	//FE_SATX_64APSK_11_15	= 0x5D,
	0,	//FE_SATX_64APSK_R_5E	  = 0x5E,
	129,	//FE_SATX_64APSK_7_9	= 0x5F,
	0,	//FE_SATX_64APSK_R_60	  = 0x60,
	125,	//FE_SATX_64APSK_4_5	= 0x61,
	0,	//FE_SATX_64APSK_R_62	  = 0x62,
	120,	//FE_SATX_64APSK_5_6	= 0x63,
	133,	//FE_SATX_128APSK_3_4	  = 0x64,
	129,	//FE_SATX_128APSK_7_9	= 0x65,
	155,	//FE_SATX_256APSK_29_45_L = 0x66,
	150,	//FE_SATX_256APSK_2_3_L = 0x67,
	145,	//FE_SATX_256APSK_31_45_L = 0x68,
	141,	//FE_SATX_256APSK_32_45 = 0x69,
	136,	//FE_SATX_256APSK_11_15_L = 0x6A,
	133,	//FE_SATX_256APSK_3_4	= 0x6B,
	409,	//FE_SATX_QPSK_11_45_S	  = 0x6C,
	375,	//FE_SATX_QPSK_4_15_S	= 0x6D,
	321,	//FE_SATX_QPSK_14_45_S	  = 0x6E,
	214,	//FE_SATX_QPSK_7_15_S	= 0x6F,
	188,	//FE_SATX_QPSK_8_15_S	  = 0x70,
	141,	//FE_SATX_QPSK_32_45_S	= 0x71,
	214,	//FE_SATX_8PSK_7_15_S	  = 0x72,
	188,	//FE_SATX_8PSK_8_15_S	= 0x73,
	173,	//FE_SATX_8PSK_26_45_S	  = 0x74,
	141,	//FE_SATX_8PSK_32_45_S	= 0x75,
	214,	//FE_SATX_16APSK_7_15_S   = 0x76,
	188,	//FE_SATX_16APSK_8_15_S = 0x77,
	173,	//FE_SATX_16APSK_26_45_S  = 0x78,
	167,	//FE_SATX_16APSK_3_5_S	= 0x79,
	141,	//FE_SATX_16APSK_32_45_S  = 0x7A,
	150,	//FE_SATX_32APSK_2_3_S	= 0x7B,
	141,	//FE_SATX_32APSK_32_45_S  = 0x7C,
	0,	//FE_SATX_8PSK			= 0x7D,
	0,	//FE_SATX_32APSK		= 0x7E,
	0,	//FE_SATX_256APSK		= 0x7F,  /* POFF Modes */
	0,	//FE_SATX_16APSK	= 0x80,
	0,	//FE_SATX_64APSK	= 0x81,
	0,	//FE_SATX_1024APSK		= 0x82,

};	/*ModCod for DVBS2*/



 static u8 dvbs1_modcode_for_llr_x100[8] = {
	200,	//FE_SAT_PR_1_2 =0,
	150,	//FE_SAT_PR_2_3,
	133,	//FE_SAT_PR_3_4,
	125,	//FE_SAT_PR_4_5,	/* for turbo code only */
	120,	//FE_SAT_PR_5_6,
	117,	//FE_SAT_PR_6_7,	/* for DSS only */
	114,	//FE_SAT_PR_7_8,
	113 //FE_SAT_PR_8_9,	/* for turbo code only */
	//FE_SAT_PR_UNKNOWN
};	/*ModCod for DVBS1*/

/*******************
Current LLA revision
********************/
static const ST_Revision_t Revision  = "STiD135-LLA_REL_1.4.0-December_2018";

/*==============================================================================*/
/* Function Declarations  */

static fe_lla_error_t FE_STiD135_GetDemodLock    (struct stv* state, u32 TimeOut, BOOL *Lock);
static fe_lla_error_t FE_STiD135_StartSearch     (struct stv* state);
static fe_lla_error_t FE_STiD135_GetSignalParams(
		struct stv* state,
		BOOL satellite_scan, enum fe_sat_signal_type *range_p);

static fe_lla_error_t FE_STiD135_TrackingOptimization(struct stv* state);
static fe_lla_error_t FE_STiD135_BlindSearchAlgo(struct stv* state,
		u32 demodTimeout, BOOL satellite_scan, BOOL* lock_p);
static fe_lla_error_t fe_stid135_get_agcrf_path_(STCHIP_Info_t* hChip,
			enum fe_stid135_demod demod, s32* agcrf_path_p);
static fe_lla_error_t fe_stid135_get_agcrf_path(struct stv* state, s32* agcrf_path_p);
fe_lla_error_t FE_STiD135_GetFECLock (struct stv* state,
	u32 TimeOut, BOOL* lock_bool_p);
fe_lla_error_t fe_stid135_manage_matype_info(struct stv* state);
static fe_lla_error_t fe_stid135_manage_matype_info_raw_bbframe(struct stv* state);
static fe_lla_error_t fe_stid135_flexclkgen_init(struct fe_stid135_internal_param* pParams);
static fe_lla_error_t fe_stid135_set_reg_init_values(struct stv* state);
static fe_lla_error_t fe_stid135_set_reg_values_wb(struct stv* state);
#ifdef USER2
static fe_lla_error_t fe_stid135_manage_manual_rolloff(fe_stid135_handle_t handle, enum fe_stid135_demod demod);
#endif
bool fe_stid135_check_sis_or_mis(u8 matype);
static fe_lla_error_t fe_stid135_get_mode_code(struct stv* state,
					enum fe_sat_modcode *modcode,
					enum fe_sat_frame *frame,
					enum fe_sat_pilots *pilots);

fe_lla_error_t FE_STiD135_TunerStandby(STCHIP_Info_t* TunerHandle,
							 FE_OXFORD_TunerPath_t TunerNb, u8 Enable);

fe_lla_error_t fe_stid135_set_vcore_supply(struct fe_stid135_internal_param* pParams);
static fe_lla_error_t fe_stid135_select_min_isi(struct stv* state);
void fe_stid135_modcod_flt_reg_init(void);

//==============================================================================

/*****************************************************
**FUNCTION	:: FE_STiD135_GetMclkFreq
**ACTION	:: Returns the master clock frequency
**PARAMS IN	:: Handle ==> handle to the chip
**PARAMS OUT	:: Synthesizer frequency (Hz)
**RETURN	:: error
*****************************************************/
fe_lla_error_t FE_STiD135_GetMclkFreq(struct fe_stid135_internal_param* pParams, u32* MclkFreq_p)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	/*  Validation Board V1.0  */
	error |= FE_STiD135_GetLoFreqHz(pParams, MclkFreq_p);
	*MclkFreq_p =    (*MclkFreq_p * 1000000) / 12;

	return error;
}

/*****************************************************
**FUNCTION	:: FE_STiD135_GetMclkFreq
**ACTION	:: This function returns the LO frequency.
**PARAMS IN	:: Handle ==> Front End Handle
**PARAMS OUT	:: LO frequency ()
**RETURN	:: error
*****************************************************/
fe_lla_error_t FE_STiD135_GetLoFreqHz(struct fe_stid135_internal_param* pParams, u32* LoFreq_p)
{
	//struct fe_stid135_internal_param *pParams;
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	//pParams = (struct fe_stid135_internal_param *) Handle;

	*LoFreq_p = Oxford_GetFvco_MHz(pParams->handle_demod, pParams->quartz);
	*LoFreq_p /=4;

	return error;
}

/*****************************************************
--FUNCTION	::	GetSymbolRate
--ACTION	::	Get the current symbol rate
--PARAMS IN	::	hChip		->	handle to the chip
			MasterClock	->	Masterclock frequency (Hz)
			demod		->	current demod 1 .. 8
--PARAMS OUT	::	Symbol rate in Symbol/s
--RETURN	::	error
*****************************************************/
fe_lla_error_t FE_STiD135_GetSymbolRate_(STCHIP_Info_t* hChip, 	enum fe_stid135_demod Demod,
																				 u32 MasterClock, u32* symbolRate_p)
{
	u32 sfrField3,
	sfrField2,
	sfrField1,
	sfrField0,
	rem1,
	rem2;

	u32 intval1,
	intval2;
	u16 sfrReg;
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	sfrField3 = FLD_FC8CODEW_DVBSX_DEMOD_SFR3_SYMB_FREQ(Demod);	/*Found SR, byte 3*/
	sfrField2 = FLD_FC8CODEW_DVBSX_DEMOD_SFR2_SYMB_FREQ(Demod);	/*Found SR, byte 2*/
	sfrField1 = FLD_FC8CODEW_DVBSX_DEMOD_SFR1_SYMB_FREQ(Demod);	/*Found SR, byte 1*/
	sfrField0 = FLD_FC8CODEW_DVBSX_DEMOD_SFR0_SYMB_FREQ(Demod);	/*Found SR, byte 0*/
	sfrReg = (u16)REG_RC8CODEW_DVBSX_DEMOD_SFR3(Demod);


	error |= ChipGetRegisters(hChip, sfrReg, 4);
	*symbolRate_p = (u32)((ChipGetFieldImage(hChip, sfrField3) << 24)
		+ (ChipGetFieldImage(hChip, sfrField2) << 16)
		+ (ChipGetFieldImage(hChip, sfrField1) << 8)
		+ (ChipGetFieldImage(hChip, sfrField0)));


	MasterClock = MasterClock * 12;

	intval1 = MasterClock >> 16;
	intval2 = *symbolRate_p  >> 16;

	rem1 = (MasterClock) % 0x10000;
	rem2 = (*symbolRate_p) % 0x10000;

	*symbolRate_p =(intval1 * intval2) +
			((intval1 * rem2) >> 16) +
				((intval2 * rem1) >> 16);
	/* only for integer calculation */

	return error;

}

fe_lla_error_t FE_STiD135_GetSymbolRate(struct stv*state, u32 MasterClock, u32* symbolRate_p)
{
	enum fe_stid135_demod Demod = state->nr+1;
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	return FE_STiD135_GetSymbolRate_(hChip, Demod, MasterClock, symbolRate_p);
}



fe_lla_error_t fe_stid135_set_symbol_rate_(struct fe_stid135_internal_param * pParams,
																					 enum fe_stid135_demod Demod,
																					 u32 symbol_rate)
{
	STCHIP_Info_t* hchip = pParams->handle_demod;
	u32 reg_field2, reg_field1, reg_field0;
	u32 reg32;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	reg_field2 = FLD_FC8CODEW_DVBSX_DEMOD_SFRINIT2_SFR_INIT(Demod);
	reg_field1 = FLD_FC8CODEW_DVBSX_DEMOD_SFRINIT1_SFR_INIT(Demod);
	reg_field0 = FLD_FC8CODEW_DVBSX_DEMOD_SFRINIT0_SFR_INIT(Demod);

	/* Hypothesis: master_clock=130MHz, symbol_rate=0.5MSymb/s..500MSymb/s
	FVCO/4/12=6.2GHz/4/12=130MHz*/

	reg32 = (1<<27) / (pParams->master_clock/10000);
	reg32 = reg32 * (symbol_rate/10000);
	reg32 = reg32 / (3*(1<<5));

	error |= ChipSetFieldImage(hchip, reg_field2,
			((int)reg32 >> 16) & 0xFF);
	error |= ChipSetFieldImage(hchip, reg_field1,
			((int)reg32 >> 8) & 0xFF);
	error |= ChipSetFieldImage(hchip, reg_field0,
			((int)reg32) & 0xFF);

	error |= ChipSetRegisters(hchip, (u16)REG_RC8CODEW_DVBSX_DEMOD_SFRINIT2(Demod), 3);


	return error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_set_symbol_rate
--ACTION	::	Set the Symbol rate
--PARAMS IN	::	hChip		->	handle to the chip
			master_clock	->	Masterclock frequency (Hz)
			symbol_rate	->	Symbol Rate (Symbol/s)
			demod		->	current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	error
*****************************************************/
fe_lla_error_t fe_stid135_set_symbol_rate(struct stv* state,  u32 symbol_rate)
{
	struct fe_stid135_internal_param * pParams = &state->base->ip;
	u32 reg_field2, reg_field1, reg_field0;
	u32 reg32;
	STCHIP_Info_t* hchip = state->base->ip.handle_demod;
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	reg_field2 = FLD_FC8CODEW_DVBSX_DEMOD_SFRINIT2_SFR_INIT(state->nr+1);
	reg_field1 = FLD_FC8CODEW_DVBSX_DEMOD_SFRINIT1_SFR_INIT(state->nr+1);
	reg_field0 = FLD_FC8CODEW_DVBSX_DEMOD_SFRINIT0_SFR_INIT(state->nr+1);

	/* Hypothesis: master_clock=130MHz, symbol_rate=0.5MSymb/s..500MSymb/s
	FVCO/4/12=6.2GHz/4/12=130MHz*/

	reg32 = (1<<27) / (pParams->master_clock/10000);
	reg32 = reg32 * (symbol_rate/10000);
	reg32 = reg32 / (3*(1<<5));

	error |= ChipSetFieldImage(hchip, reg_field2,
			((int)reg32 >> 16) & 0xFF);
	error |= ChipSetFieldImage(hchip, reg_field1,
			((int)reg32 >> 8) & 0xFF);
	error |= ChipSetFieldImage(hchip, reg_field0,
			((int)reg32) & 0xFF);

	error |= ChipSetRegisters(hchip, (u16)REG_RC8CODEW_DVBSX_DEMOD_SFRINIT2(state->nr+1), 3);


	return error;
}


/*****************************************************
--FUNCTION	::	GetCarrierFrequency
--ACTION	::	Return the carrier frequency offset
--PARAMS IN	::	hChip -> handle to the chip
			MasterClock -> Masterclock frequency (Hz)
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	Frequency offset in Hz
--RETURN	::	error
--***************************************************/
fe_lla_error_t FE_STiD135_GetCarrierFrequencyOffset_(STCHIP_Info_t* hChip, enum fe_stid135_demod Demod,  u32 MasterClock,
				s32* carrierFrequency_p)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	u32 cfrField2,
		cfrField1,
		cfrField0,
		rem1;
	s32 derot,
		rem2,
		intval1,
		intval2;
	u16 cfrReg;

	BOOL sign=1;
	if(!carrierFrequency_p) {
		dprintk("Called with carrierFrequency_p=nullptr\n");
		return  FE_LLA_NO_ERROR;
	}
	if((int)Demod <=0 || (int) Demod >8) {
		dprintk("Called with Demod=%d\n", Demod);
		//*carrierFrequency_p = 0;
		return  FE_LLA_NO_ERROR;
	}
	cfrField2 = FLD_FC8CODEW_DVBSX_DEMOD_CFR12_CAR_FREQ(Demod); /* carrier frequency Byte 2 */
	cfrField1 = FLD_FC8CODEW_DVBSX_DEMOD_CFR11_CAR_FREQ(Demod); /* carrier frequency Byte 1 */
	cfrField0 = FLD_FC8CODEW_DVBSX_DEMOD_CFR10_CAR_FREQ(Demod); /* carrier frequency Byte 0 */
	cfrReg    = (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR12(Demod);

	/*	Read the carrier frequency regs value	*/
	error |= ChipGetRegisters(hChip, cfrReg, 3);

	derot = (ChipGetFieldImage(hChip, cfrField2) << 16) +
	(ChipGetFieldImage(hChip, cfrField1) << 8) +
	(ChipGetFieldImage(hChip, cfrField0));
	/*	compute the signed value	*/
	derot = Get2Comp(derot,24);

	if (derot < 0) {
		sign = 0;
		derot = -derot; /* Use positive values only to avoid
				negative value truncation */
	}

	/*
		Formula:  carrier_frequency = MasterClock * 12 * Reg / 2^24
	*/
	MasterClock = MasterClock * 12;

	intval1 = (s32)(MasterClock >> 12);
	intval2 = derot       >> 12;

	rem1 = MasterClock % 0x1000;
	rem2 = derot       % 0x1000;

	derot = (intval1 * intval2) +
			((intval1 * rem2) >> 12) +
			((intval2 * (s32)rem1) >> 12);  /*only for integer calculation */

	if (sign == 1) {
		*carrierFrequency_p=derot;  /* positive offset */
	} else {
		*carrierFrequency_p=-derot; /* negative offset */
	}
	return error;
}

fe_lla_error_t FE_STiD135_GetCarrierFrequencyOffset(struct stv* state, u32 MasterClock, s32* carrierFrequency_p)
{
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	return FE_STiD135_GetCarrierFrequencyOffset_(hChip, state->nr +1, MasterClock, carrierFrequency_p);
}

void report_(struct stv*state,  const char* func, int line)
{
	s32 freq;
	u32 srate;
	u32 MasterClock = state->base->ip.master_clock;
	FE_STiD135_GetCarrierFrequencyOffset(state, MasterClock, &freq);
	FE_STiD135_GetSymbolRate(state, MasterClock, &srate);
	printk(KERN_DEBUG pr_fmt("%s:%d freq=%dkHz srate=%dkHz"), func, line, freq/1000, srate/1000);
}
#define report(state) report_(state,  __func__, __LINE__)


fe_lla_error_t fe_stid135_set_carrier_frequency_init_(struct fe_stid135_internal_param * pParams,
																											enum fe_stid135_demod Demod,
																											s32 frequency_hz, s32 demod_search_range_hz)
{

	STCHIP_Info_t* hChip = pParams->handle_demod;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	s32 si_register;
	s32 freq_up, freq_low;
	s32 reg_up, reg_low;
	const u16 cfr_factor = 6711; // 6711 = 2^20/(10^6/2^6)*100

	/* First, set carrier freq boundaries */
	// CFR_UP = wanted_freq - LOF + search_range/2
	// CFR_DOWN = wanted_freq - LOF - search_range/2
	freq_up = frequency_hz + demod_search_range_hz / 2;
	freq_low = frequency_hz - demod_search_range_hz / 2;

	reg_up = (freq_up / 1000) * (1 << 11);
	reg_up = reg_up / ((pParams->master_clock / 1000) * 3);
	reg_up = reg_up * (1 << 11);

	reg_low = (freq_low / 1000) * (1 << 11);
	reg_low = reg_low / ((pParams->master_clock / 1000) * 3);
	reg_low = reg_low * (1 << 11);
	if(false /* todo: state->demod_search_algo == FE_SAT_NEXT*/) {
		/* Search range definition */
		/*citroen2, rotator on,  CFRUP/LOW computed from  DMDCODSFR2..0 (or TNRSTEPS/tuner_bwoffset if not available)
			in Blind Search; alternative: 0x06 -> unbounded search */
		error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(Demod), 0xc6);
	} else  {
		vprintk("[%d] ACTIVATE SEARCH RANGE %dkHz\n",  Demod, demod_search_range_hz/1000);
		/* Search range definition */
		//citroen2, rotator on, manual CFRUP/LOW
		error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(Demod), 0x46);

		/*
			0x46: bit[7:6]=01 manual programming of cfr up/low
			bit[2]=1 derotator on
					bi[1:0]=00 costas
		*/
		/*  manual cfrup/low setting, Carrier derotator on, phasedetect algo */
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRUP2_CFR_UP(Demod),
															 MMSB(reg_up));
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRUP1_CFR_UP(Demod),
															 MSB(reg_up));
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRUP0_CFR_UP(Demod),
															 LSB(reg_up));
		error |= ChipSetRegisters(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRUP2(Demod),3);

		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRLOW2_CFR_LOW(Demod),
															 MMSB(reg_low));
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRLOW1_CFR_LOW(Demod),
															 MSB(reg_low));
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRLOW0_CFR_LOW(Demod),
															 LSB(reg_low));
		error |= ChipSetRegisters(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRLOW2(Demod),3);
	}
	/*
		Formula:
							cfr_init =  cfr_init(MHz) / ckadc  * 2^24 (signed)
							ckadc = 12 * Mclk(MHz)
	*/

	/* Hypothesis: master_clock=130MHz, frequency_hz=-550MHz..+650MHz
	(-550=950-1500 and 650=2150-1500)
	FVCO/4/12=6.2GHz/4/12=130MHz */

		si_register = ((frequency_hz/PLL_FVCO_FREQUENCY)*cfr_factor)/100;
		dprintk("CFR_INIT set to %d\n", si_register );
		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT2_CFR_INIT(Demod),
															 ((u8)(si_register >> 16)));
		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT1_CFR_INIT(Demod),
															 ((u8)(si_register >> 8)));
		error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_CFRINIT0_CFR_INIT(Demod),
															 ((u8)si_register));
		error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT2(Demod),3);

		//	s32 freq;
		//	FE_STiD135_GetCarrierFrequencyOffset(hChip,master_clock,state->nr+1,&freq);
	return error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_set_carrier_frequency_init
--ACTION	::	Sets the CFR InitReg with the given frequency
--PARAMS IN	::	Handle		->	Front End Handle
			master_clock	->	Masterclock frequency (Hz)
			frequency_hz	->	InitFrequency (Hz)
			demod		->	current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	error
*****************************************************/
fe_lla_error_t fe_stid135_set_carrier_frequency_init(struct stv* state, s32 frequency_hz)
{
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	s32 si_register;
	s32 freq_up, freq_low;
	s32 reg_up, reg_low;
	const u16 cfr_factor = 6711; // 6711 = 2^20/(10^6/2^6)*100
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	//pParams = (struct fe_stid135_internal_param *) Handle;
	/* First, set carrier freq boundaries */
	// CFR_UP = wanted_freq - LOF + search_range/2
	// CFR_DOWN = wanted_freq - LOF - search_range/2
	if(state->demod_search_algo == FE_SAT_NEXT) {
		freq_up = (s32)(state->tuner_frequency)+ (s32)(state->demod_search_range_hz);
		freq_low = (s32)(state->tuner_frequency);
	} else {
		freq_up = (s32)(state->tuner_frequency) + (s32)(state->demod_search_range_hz / 2);
		freq_low = (s32)(state->tuner_frequency) - (s32)(state->demod_search_range_hz / 2);
	}
	reg_up = (freq_up / 1000) * (1 << 11);
	reg_up = reg_up / (((s32)pParams->master_clock / 1000) * 3);
	reg_up = reg_up * (1 << 11);

	reg_low = (freq_low / 1000) * (1 << 11);
	reg_low = reg_low / (((s32)pParams->master_clock / 1000) * 3);
	reg_low = reg_low * (1 << 11);
#if 1
	vprintk("[%d] Initial Freq: blind=%d next=%d freq=%d up=%d low=%d range=%d\n",
					state->nr+1,
				 state->demod_search_algo==FE_SAT_BLIND_SEARCH,
				 state->demod_search_algo==FE_SAT_NEXT,
				 state->tuner_frequency,
				 freq_up, freq_low, (s32)(state->demod_search_range_hz / 2));
#endif

	if(state->demod_search_algo == FE_SAT_NEXT) {
		/* Search range definition */
		/*citroen2, rotator on,  CFRUP/LOW computed from  DMDCODSFR2..0 (or TNRSTEPS/tuner_bwoffset if not available)
			in Blind Search; alternative: 0x06 -> unbounded search */
		//error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(state->nr+1), 0xc6);
		error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(state->nr+1), 0xc6);
		vprintk("[%d] setting unlimited search 0x%x\n", state->nr+1, 0xc6);

	} else  {
		//TODO: use asymmetric search range when scanning side of frequency peak
		//dprintk("ACTIVATE SEARCH RANGE %dkHz\n",  state->demod_search_range_hz/1000);
		/* Search range definition */
		error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(state->nr+1), 0x46);
		/*
			0x46: bit[7:6]=01 manual programming of cfr up/low
			bit[2]=1 derotator on
			bi[1:0]=00 costas
		*/
		/*  manual cfrup/low setting, Carrier derotator on, phasedetect algo */
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRUP2_CFR_UP(state->nr+1),
															 MMSB(reg_up));
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRUP1_CFR_UP(state->nr+1),
															 MSB(reg_up));
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRUP0_CFR_UP(state->nr+1),
															 LSB(reg_up));
		error |= ChipSetRegisters(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRUP2(state->nr+1),3);

		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRLOW2_CFR_LOW(state->nr+1),
															 MMSB(reg_low));
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRLOW1_CFR_LOW(state->nr+1),
															 MSB(reg_low));
		error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRLOW0_CFR_LOW(state->nr+1),
															 LSB(reg_low));
		error |= ChipSetRegisters(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRLOW2(state->nr+1),3);
	}
	/*
		Formula:
		cfr_init =  cfr_init(MHz) / ckadc  * 2^24 (signed)
							ckadc = 12 * Mclk(MHz)
	*/

	/* Hypothesis: master_clock=130MHz, frequency_hz=-550MHz..+650MHz
		 (-550=950-1500 and 650=2150-1500)
		 FVCO/4/12=6.2GHz/4/12=130MHz */
	vprintk("[%d] setting frequency=%dkHz\n", state->nr+1, frequency_hz/1000);
	si_register = ((frequency_hz/PLL_FVCO_FREQUENCY)*cfr_factor)/100;
	dprintk("CFR_INIT set to %d\n", si_register );
	error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT2(state->nr+1),
															((u8)(si_register >> 16)));
	error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT1(state->nr+1),
															((u8)(si_register >> 8)));
	error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT0(state->nr+1),
															((u8)(si_register)));
	//	s32 freq;
	//	FE_STiD135_GetCarrierFrequencyOffset(hChip,master_clock,state->nr+1,&freq);

	return error;
}


fe_lla_error_t tst1(struct stv* state, s32 frequency_hz)
{
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	s32 si_register;
	s32 freq_up, freq_low;
	s32 reg_up, reg_low;
	const u16 cfr_factor = 6711; // 6711 = 2^20/(10^6/2^6)*100
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	//pParams = (struct fe_stid135_internal_param *) Handle;
	/* First, set carrier freq boundaries */
	// CFR_UP = wanted_freq - LOF + search_range/2
	// CFR_DOWN = wanted_freq - LOF - search_range/2
	freq_up = (s32)(state->tuner_frequency) + (s32)(state->demod_search_range_hz / 2);
	freq_low = (s32)(state->tuner_frequency) - (s32)(state->demod_search_range_hz / 2);
	reg_up = (freq_up / 1000) * (1 << 11);
	reg_up = reg_up / (((s32)pParams->master_clock / 1000) * 3);
	reg_up = reg_up * (1 << 11);

	reg_low = (freq_low / 1000) * (1 << 11);
	reg_low = reg_low / (((s32)pParams->master_clock / 1000) * 3);
	reg_low = reg_low * (1 << 11);
	return error;
		//TODO: use asymmetric search range when scanning side of frequency peak
		//dprintk("ACTIVATE SEARCH RANGE %dkHz\n",  state->demod_search_range_hz/1000);
		/* Search range definition */
	error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(state->nr+1), 0x46);
	/*
		0x46: bit[7:6]=01 manual programming of cfr up/low
		bit[2]=1 derotator on
		bi[1:0]=00 costas
	*/



		/*  manual cfrup/low setting, Carrier derotator on, phasedetect algo */
	error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRUP2_CFR_UP(state->nr+1),
														 MMSB(reg_up));
	error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRUP1_CFR_UP(state->nr+1),
														 MSB(reg_up));
	error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRUP0_CFR_UP(state->nr+1),
														 LSB(reg_up));
	error |= ChipSetRegisters(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRUP2(state->nr+1),3);

	error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRLOW2_CFR_LOW(state->nr+1),
														 MMSB(reg_low));
	error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRLOW1_CFR_LOW(state->nr+1),
														 MSB(reg_low));
	error |= ChipSetFieldImage(hChip, FLD_FC8CODEW_DVBSX_DEMOD_CFRLOW0_CFR_LOW(state->nr+1),
														 LSB(reg_low));
	error |= ChipSetRegisters(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRLOW2(state->nr+1),3);

	/*
		Formula:
		cfr_init =  cfr_init(MHz) / ckadc  * 2^24 (signed)
		ckadc = 12 * Mclk(MHz)
	*/

	/* Hypothesis: master_clock=130MHz, frequency_hz=-550MHz..+650MHz
		 (-550=950-1500 and 650=2150-1500)
		 FVCO/4/12=6.2GHz/4/12=130MHz */
	dprintk("setting frequency=%dkHz\n", frequency_hz/1000);
	si_register = ((frequency_hz/PLL_FVCO_FREQUENCY)*cfr_factor)/100;

	error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT2(state->nr+1),
															((u8)(si_register >> 16)));
	error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT1(state->nr+1),
															((u8)(si_register >> 8)));
	error |= ChipSetOneRegister(hChip, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT0(state->nr+1),
															((u8)(si_register)));
	//	s32 freq;
	//	FE_STiD135_GetCarrierFrequencyOffset(hChip,master_clock,state->nr+1,&freq);

	return error;
}



/*****************************************************
--FUNCTION	::	FE_STiD135_TimingGetOffset
--ACTION	::	Returns the timing offset
--PARAMS IN	::	hChip -> handle to the chip
			SymbolRate -> Masterclock frequency (Hz)
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	timingoffset_p -> timing offset
--RETURN	::	error
*****************************************************/
static fe_lla_error_t FE_STiD135_TimingGetOffset_(STCHIP_Info_t* hChip, 	enum fe_stid135_demod Demod,
																								 u32 SymbolRate, s32* timingoffset_p)
{
	u32 tmgField2,
			tmgField1,
			tmgField0;

	u16 tmgreg;
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	tmgreg = (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(Demod);	/*Found timing offset reg HSB*/
	tmgField2 = FLD_FC8CODEW_DVBSX_DEMOD_TMGREG2_TMGREG(Demod);
	tmgField1 = FLD_FC8CODEW_DVBSX_DEMOD_TMGREG1_TMGREG(Demod);
	tmgField0 = FLD_FC8CODEW_DVBSX_DEMOD_TMGREG0_TMGREG(Demod);

	/* Formula :
	 tmgreg in MHz = symb_freq in MHz * (tmgreg * 2^-(24+5)) (unsigned)

	 SR_Offset = TMGRREG * SR /2^29
	 TMGREG is 3 bytes registers value
	 SR is the current symbol rate
	*/
	error |= ChipGetRegisters(hChip, tmgreg, 3);

	*timingoffset_p = (ChipGetFieldImage(hChip, tmgField2) << 16)+
		(ChipGetFieldImage(hChip, tmgField1) << 8)+
		(ChipGetFieldImage(hChip, tmgField0));

	*timingoffset_p = Get2Comp(*timingoffset_p,24);


	if (*timingoffset_p == 0) {
		*timingoffset_p = 1;
	}
	*timingoffset_p = ((s32)SymbolRate * 10) / ((s32)0x1000000 / (*timingoffset_p));
	*timingoffset_p = (*timingoffset_p) / 320;

	return error;
}

fe_lla_error_t FE_STiD135_TimingGetOffset(struct stv*state, u32 SymbolRate, s32* timingoffset_p)
{
	enum fe_stid135_demod Demod = state->nr+1;
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	return FE_STiD135_TimingGetOffset_(hChip, 	Demod, SymbolRate, timingoffset_p);
}


/*****************************************************
--FUNCTION	::	FE_STiD135_GetViterbiPunctureRate
--ACTION	::	Computes the puncture rate
--PARAMS IN	::	hChip -> handle to the chip
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	punctureRate_p -> puncture rate
--RETURN	::	error
*****************************************************/
static  fe_lla_error_t FE_STiD135_GetViterbiPunctureRate(struct stv* state, enum fe_sat_rate *punctureRate_p)
{
	s32 rateField;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod Demod = state->nr+1;
	error |= ChipGetField(hChip, FLD_FC8CODEW_DVBSX_VITERBI_VITCURPUN_VIT_CURPUN(Demod), &rateField);

	switch(rateField) {
	case 13:
		*punctureRate_p = FE_SAT_PR_1_2;
	break;

	case 18:
		*punctureRate_p = FE_SAT_PR_2_3;
	break;

	case 21:
		*punctureRate_p = FE_SAT_PR_3_4;
	break;

	case 24:
		*punctureRate_p = FE_SAT_PR_5_6;
	break;

	case 25:
		*punctureRate_p = FE_SAT_PR_6_7;
	break;

	case 26:
		*punctureRate_p = FE_SAT_PR_7_8;
	break;

	default:
		*punctureRate_p = FE_SAT_PR_UNKNOWN;
	break;
	}

	return	error;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_GetBer
--ACTION	::	Returns the Viterbi BER if DVBS1/DSS or the PER if DVBS2
--PARAMS IN	::	hChip -> handle to the chip
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	ber_p -> BER
--RETURN	::	error
*****************************************************/
static fe_lla_error_t FE_STiD135_GetBer_(STCHIP_Info_t* hChip, enum fe_stid135_demod Demod, u32* ber_p)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u32 ber, i;
	u32 demdStateReg,
			prVitField,
			pdellockField;
	s32 demodState;
	s32 fld_value;

	*ber_p = 10000000;

	demdStateReg  = FLD_FC8CODEW_DVBSX_DEMOD_DMDSTATE_HEADER_MODE(Demod);
	prVitField    = FLD_FC8CODEW_DVBSX_VITERBI_VSTATUSVIT_PRFVIT(Demod);
	pdellockField = FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_LOCK(Demod);

	error |= ChipGetField(hChip, demdStateReg, &demodState);

	switch(demodState) {
	case FE_SAT_SEARCH:
	case FE_SAT_PLH_DETECTED:
	default:
		*ber_p=10000000;		/*demod Not locked ber = 1*/
	break;

	case FE_SAT_DVBS_FOUND:

		*ber_p=0;
		/* Average 5 ber values */
		for (i = 0; i < 5; i++) {
			ChipWaitOrAbort(hChip, 5);
			error |= FE_STiD135_GetErrorCount(hChip,
					FE_STiD_COUNTER1, Demod, &ber);
			*ber_p += ber;
		}

		*ber_p /= 5;

		 /* Check for carrier */
		 error |= ChipGetField(hChip, prVitField, &fld_value);
		 if(fld_value)
		 {
			*ber_p *= 9766;		/* ber = ber * 10^7/2^10 */
			*ber_p = *ber_p >> 13; /* theses two lines =>
					ber = ber * 10^7/2^20 */
		}
	break;

	case FE_SAT_DVBS2_FOUND:

		*ber_p = 0;
		/* Average 5 ber values */
		for(i = 0; i < 5; i++) {
			ChipWaitOrAbort(hChip, 5);
			error |= FE_STiD135_GetErrorCount(hChip,
					FE_STiD_COUNTER1, Demod, &ber);
			*ber_p += ber;
		}
		*ber_p /= 5;

		/* Check for S2 FEC Lock */
		error |= ChipGetField(hChip, pdellockField, &fld_value) ;
		if(fld_value) {
			*ber_p *= 9766; /*	ber = ber * 10^7/2^10 */
			*ber_p = *ber_p >> 13; /*  this two lines =>
					PER = ber * 10^7/2^23 */
		}

	break;
	}

	return error;		 //ber/per scaled to 1e7
}


static fe_lla_error_t FE_STiD135_GetBer(struct stv* state, u32* ber_p)
{
	enum fe_stid135_demod Demod = state->nr+1;
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	return FE_STiD135_GetBer_(hChip, Demod, ber_p);
}



/*****************************************************
--FUNCTION	::	FE_STiD135_SetViterbiStandard
--ACTION	::	Sets the standard
--PARAMS IN	::	hChip -> handle to the chip
			Demod -> current demod 1 .. 8
			Standard -> standard
			puncture_rate -> PR
--PARAMS OUT	::	ber_p -> BER
--RETURN	::	error
*****************************************************/
static fe_lla_error_t FE_STiD135_SetViterbiStandard(STCHIP_Info_t* hChip,
					enum fe_sat_search_standard Standard,
					enum fe_sat_rate puncture_rate,
					enum fe_stid135_demod Demod)
{

	u16 fecmReg,
		prvitReg;
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	fecmReg  = (u16)REG_RC8CODEW_DVBSX_VITERBI_FECM(Demod);
	prvitReg = (u16)REG_RC8CODEW_DVBSX_VITERBI_PRVIT(Demod);

	switch (Standard) {
	case FE_SAT_AUTO_SEARCH:

		/* Disable DSS in auto mode search for DVBS1 and DVBS2
		only , DSS search is on demande */
		error |= ChipSetOneRegister(hChip, fecmReg, 0x00);
		/* Enable All PR exept 6/7 */
		error |= ChipSetOneRegister(hChip, prvitReg, 0x2F);

	break;

	case FE_SAT_SEARCH_DVBS1:
		/* Disable DSS */
		error |= ChipSetOneRegister(hChip, fecmReg, 0x00);
		switch (puncture_rate) {
		case FE_SAT_PR_UNKNOWN:
		case FE_SAT_PR_4_5:
		case FE_SAT_PR_6_7:
		case FE_SAT_PR_8_9:
			/* Enable All PR exept 6/7 */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x2F);
		break;

		case FE_SAT_PR_1_2:
			/* Enable 1/2 PR only */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x01);
		break;

		case FE_SAT_PR_2_3:
			/* Enable 2/3 PR only */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x02);
		break;

		case FE_SAT_PR_3_4:
			/* Enable 3/4 PR only */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x04);
		break;

		case FE_SAT_PR_5_6:
			/* Enable 5/6 PR only */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x08);
		break;

		case FE_SAT_PR_7_8:
			/* Enable 7/8 PR only */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x20);
		break;
		}

	break;

	case FE_SAT_SEARCH_DSS:

		error |= ChipSetOneRegister(hChip, fecmReg, 0x80);
		switch (puncture_rate) {
		case FE_SAT_PR_UNKNOWN:
		case FE_SAT_PR_3_4:
		case FE_SAT_PR_4_5:
		case FE_SAT_PR_5_6:
		case FE_SAT_PR_7_8:
		case FE_SAT_PR_8_9:
			/* Enable 1/2, 2/3 and 6/7 PR */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x13);
		break;

		case FE_SAT_PR_1_2:
			/* Enable 1/2 PR only */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x01);
		break;

		case FE_SAT_PR_2_3:
			/* Enable 2/3 PR only */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x02);
		break;

		case FE_SAT_PR_6_7:
			/* Enable All 7/8 PR only */
			error |= ChipSetOneRegister(hChip, prvitReg, 0x10);
		break;
		}

	break;

	case FE_SAT_SEARCH_DVBS2:
	case FE_SAT_SEARCH_TURBOCODE:
	break;
	}

	return error;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_GetRevision
--ACTION	::	Return current LLA version
--PARAMS IN	::	NONE
--PARAMS OUT	::	NONE
--RETURN	::	Revision ==> Text string containing LLA version
--***************************************************/
ST_Revision_t FE_STiD135_GetRevision(void)
{
	return (Revision);
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_cut_id
--ACTION	::	Return chip cut ID
--PARAMS IN	::	Handle	==> Front End Handle
--PARAMS OUT	::	cut_id => cut ID with major and minor ID
--RETURN	::	error
--***************************************************/
fe_lla_error_t fe_stid135_get_cut_id(struct fe_stid135_internal_param* pParams, enum device_cut_id *cut_id)
{
	u32 reg_value;
	u8 major_cut_id, minor_cut_id;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams;

	//	pParams = (struct fe_stid135_internal_param *) Handle;
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_STATUS97, &reg_value);
		major_cut_id = (u8)((reg_value >> 28) & 0x1);
		if(major_cut_id == 0) {
			*cut_id = STID135_CUT1_X;
		} else if(major_cut_id == 1) {
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_STATUS98, &reg_value);
			minor_cut_id = (u8)reg_value;
			if(minor_cut_id == 8) {
				*cut_id = STID135_CUT2_0;
			} else if(minor_cut_id == 9) {
				*cut_id = STID135_CUT2_1;
			} else {
				*cut_id = STID135_CUT2_X_UNFUSED;
			}
		} else {
			*cut_id = STID135_UNKNOWN_CUT;
		}
	return(error);
}

/*****************************************************
--FUNCTION	::	FE_STiD135_GetSignalInfoLite
--ACTION	::	Return C/N only
--PARAMS IN	::	Handle	==> Front End Handle
			Demod	==> Current demod 1 or 2
--PARAMS OUT	::	pInfo	==> Informations (C/N)
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t	FE_STiD135_GetSignalInfoLite(struct stv* state, struct fe_sat_signal_info *pInfo)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams;

	//pParams = (struct fe_stid135_internal_param *) Handle;

		if ((state->base->ip.handle_demod->Error) /*|| (pParams->h_tuner->Error)*/)
			error |= FE_LLA_I2C_ERROR;
		else {
			error |= FE_STiD135_CarrierGetQuality(state->base->ip.handle_demod, state->nr+1,
																						&(pInfo->C_N), &(pInfo->standard));
			if (state->base->ip.handle_demod->Error)
				error = FE_LLA_I2C_ERROR;
		}

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_lock_status
--ACTION	::	Return locked status
--PARAMS IN	::	Handle -> Front End Handle
			Demod -> Current demod 1 .. 8
--PARAMS OUT	::	carrier_lock -> Bool (locked or not)
			data_present -> Bool (data found or not)
--RETURN	::	error
--***************************************************/
fe_lla_error_t fe_stid135_get_lock_status(struct stv* state, bool*carrier_lock, bool*has_viterbi, bool* has_sync)
{

	struct fe_stid135_internal_param *pParams;
	enum fe_sat_search_state demodState;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	s32 fld_value[5];
	int error1;
	if(carrier_lock)
		*carrier_lock = false;
	if(has_sync)
		*has_sync = false;
	if(has_viterbi)
		*has_viterbi = false;
	//state->signal_info.has_sync is the same as *Locked_p in official driver

	pParams = &state->base->ip;

		error = error1 =  ChipGetField(state->base->ip.handle_demod,
																	 FLD_FC8CODEW_DVBSX_DEMOD_DMDSTATE_HEADER_MODE(state->nr+1), &(fld_value[0]));
		demodState = (enum fe_sat_search_state)(fld_value[0]);

		if(error1)
			dprintk("demodstate=%d error=%d\n", demodState, error1);

		switch (demodState) {
		case FE_SAT_SEARCH:
		case FE_SAT_PLH_DETECTED :
			//dprintk("FIRST PLHEADER DETECTED\n");

			state->signal_info.has_carrier = false;
			state->signal_info.has_viterbi = false;
			state->signal_info.has_sync = false;
		break;
		case FE_SAT_DVBS2_FOUND:
			state->signal_info.has_carrier = 	true;

			error |= (error1=ChipGetField(state->base->ip.handle_demod,
																		FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_LOCK_DEFINITIF(state->nr+1), &(fld_value[0])));
			state->signal_info.has_lock = fld_value[0];

			error |= (error1=ChipGetField(state->base->ip.handle_demod,
																		FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_LOCK(state->nr+1), &(fld_value[1])));
			state->signal_info.has_viterbi = fld_value[0] & fld_value[1];

			//TODO: stv091x does not check TSFIFO_LINEOK
			//fld_value[2]==0 means that packets with errors have been received
			error |= (error1=ChipGetField(state->base->ip.handle_demod,
																		FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS_TSFIFO_LINEOK(state->nr+1), &(fld_value[2])));

			state->signal_info.has_sync = fld_value[0] & fld_value[1] & fld_value[2];
#if 0			//only for dvb-s1?
			error |= ChipGetField(state->base->ip.handle_demod,
													FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_CAR_LOCK(state->nr+1),
														&(fld_value[3]));
#endif
			error |= ChipGetField(state->base->ip.handle_demod,
													FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_TMGLOCK_QUALITY(state->nr+1),
													&(fld_value[4]));
			state->signal_info.has_timing_lock =  fld_value[4]&2;

			if(carrier_lock)
				*carrier_lock =  state->signal_info.has_carrier;
			if(has_viterbi)
				*has_viterbi  = fld_value[0] & fld_value[1];
			if(has_sync) {
				*has_sync  = fld_value[0] & fld_value[1] & fld_value[2];
				if(!state->signal_info.has_sync)
					dprintk("[%d] DVBS2 NO SYNC: %d %d %d\n", state->nr+1,  fld_value[0] , fld_value[1] , fld_value[2]);
			}
				if(!state->signal_info.has_sync)
					vprintk("[%d] DVBS2 NO SYNC: %d %d %d\n", state->nr+1,  fld_value[0] , fld_value[1] , fld_value[2]);
		break;

		case FE_SAT_DVBS_FOUND:
			state->signal_info.has_carrier = 	true;
			error |= (error1=ChipGetField(state->base->ip.handle_demod,
																		FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_LOCK_DEFINITIF(state->nr+1), &(fld_value[0])));
			if(error1)
				dprintk("error=%d\n", error1);
			state->signal_info.has_lock = fld_value[0];

			error |= (error1=ChipGetField(state->base->ip.handle_demod,
																		FLD_FC8CODEW_DVBSX_VITERBI_VSTATUSVIT_LOCKEDVIT(state->nr+1), &(fld_value[1])));
			if(error1)
				dprintk("error=%d\n", error1);

			state->signal_info.has_viterbi = fld_value[0] & fld_value[1];


			//TODO: stv091x does not check TSFIFO_LINEOK
			//fld_value[2]==0 means that packets with errors have been received
			error |= (error1=ChipGetField(state->base->ip.handle_demod,
																		FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS_TSFIFO_LINEOK(state->nr+1), &(fld_value[2])));
			if(error1)
				dprintk("error=%d\n", error1);

			state->signal_info.has_sync = fld_value[0] & fld_value[1] & fld_value[2];

#if 0
			ChipGetField(state->base->ip.handle_demod,
									 FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_CAR_LOCK(state->nr+1), &(fld_value[3]));
			state->signal_info.has_carrier = fld_value[3];//dvbs only carrier lock
#endif

			error |= ChipGetField(state->base->ip.handle_demod,
													FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_TMGLOCK_QUALITY(state->nr+1),
													&(fld_value[4]));
			state->signal_info.has_timing_lock =  fld_value[4]&2;

			if(carrier_lock)
				*carrier_lock = state->signal_info.has_carrier;
			if(has_viterbi)
				*has_viterbi  = fld_value[0] & fld_value[1];
			if(has_sync) {
				*has_sync  = fld_value[0] & fld_value[1] & fld_value[2];
				if(!state->signal_info.has_sync)
					dprintk("NO SYNC: %d %d\n",  fld_value[0] , fld_value[1]);
			}
			break;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_data_status
--ACTION	::	Return data status
--PARAMS IN	::	Handle -> Front End Handle
			Demod -> Current demod 1 .. 8
--PARAMS OUT	::	Locked_p -> Bool (data working or not)
--RETURN	::	error
--***************************************************/
fe_lla_error_t fe_stid135_get_data_status(struct stv* state, BOOL* Locked_p)
{

	bool has_carrier=false;
	bool has_sync=false;
	bool has_viterbi=false;
	int err = fe_stid135_get_lock_status(state, &has_carrier, &has_viterbi, &has_sync);
	if(err)
		dprintk("ERROR in fe_stid135_get_lock_status: err=%d\n", err);
	dprintk("Called get_data_status: fe_stid135_get_lock_status: carrier=%d vit=%d sync=%d\n",
					has_carrier, has_viterbi, has_sync);
	*Locked_p = has_carrier && has_viterbi && has_sync;
	return err;
}


/*****************************************************
**FUNCTION	::	FE_STiD135_GetErrorCount
**ACTION	::	return the number of errors from a given counter
**PARAMS IN	::	hChip -> handle to the chip
			Counter -> used counter 1 or 2.
			Demod -> current demod 1 .. 2.
**PARAMS OUT	::	ber_p -> BER
**RETURN	::	error
*****************************************************/
fe_lla_error_t FE_STiD135_GetErrorCount(STCHIP_Info_t* hChip,
		enum fe_stid135_error_counter Counter, enum fe_stid135_demod Demod, u32* ber_p)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u32 lsb=0,msb=0,hsb=0;

	u32
		err1Field_hsb = 0,
		err1Field_msb = 0,
		err1Field_lsb = 0,
		err2Field_hsb = 0,
		err2Field_msb = 0,
		err2Field_lsb = 0;
	u16 err1Reg, err2Reg;

	/*Define Error fields */
	err1Field_hsb = FLD_FC8CODEW_DVBSX_HWARE_ERRCNT12_ERR_CNT1(Demod);
	err1Field_msb = FLD_FC8CODEW_DVBSX_HWARE_ERRCNT11_ERR_CNT1(Demod);
	err1Field_lsb = FLD_FC8CODEW_DVBSX_HWARE_ERRCNT10_ERR_CNT1(Demod);
	err1Reg = (u16)REG_RC8CODEW_DVBSX_HWARE_ERRCNT12(Demod);

	err2Field_hsb = FLD_FC8CODEW_DVBSX_HWARE_ERRCNT22_ERR_CNT2(Demod);
	err2Field_msb = FLD_FC8CODEW_DVBSX_HWARE_ERRCNT21_ERR_CNT2(Demod);
	err2Field_lsb = FLD_FC8CODEW_DVBSX_HWARE_ERRCNT20_ERR_CNT2(Demod);
	err2Reg = (u16)REG_RC8CODEW_DVBSX_HWARE_ERRCNT22(Demod);

	/*Read the Error value*/
	switch (Counter) {
	case FE_STiD_COUNTER1:
		error |= ChipGetRegisters(hChip, err1Reg, 3);
		hsb = (u32)ChipGetFieldImage(hChip, err1Field_hsb);
		msb = (u32)ChipGetFieldImage(hChip, err1Field_msb);
		lsb = (u32)ChipGetFieldImage(hChip, err1Field_lsb);
	break;

	case FE_STiD_COUNTER2:
		error |= ChipGetRegisters(hChip, err2Reg, 3);
		hsb = (u32)ChipGetFieldImage(hChip, err2Field_hsb);
		msb = (u32)ChipGetFieldImage(hChip, err2Field_msb);
		lsb = (u32)ChipGetFieldImage(hChip, err2Field_lsb);
	break;
	}

	/*Compute the Error value 3 bytes (HSB,MSB,LSB)*/
	*ber_p =(hsb<<16)+(msb<<8)+(lsb);

	return (error);
}


/*****************************************************
--FUNCTION	::	FE_STiD135_WaitForLock
--ACTION	::	Wait until demod + FEC locked or timout
--PARAMS IN	::	Handle -> Frontend handle
			Demod -> current demod 1 ... 8
			DemodTimeOut -> Time out in ms for demod
			FecTimeOut -> Time out in ms for FEC
			satellite_scan -> scan or acquisition
--PARAMS OUT	::	lock_p -> Lock status true or false
--RETURN	::	error
--***************************************************/
fe_lla_error_t FE_STiD135_WaitForLock(struct stv* state,
																			u32 DemodTimeOut,u32 FecTimeOut,BOOL satellite_scan, s32* lock_p, BOOL* fec_lock_p)
{
	fe_lla_error_t error               = FE_LLA_NO_ERROR;
	u32 strMergerLockField;
	struct fe_stid135_internal_param *pParams;
	BOOL lock;
	vprintk("[%d] entering  FecTimeOut=%d\n", state->nr+1,  FecTimeOut);
	pParams = &state->base->ip;

	/* Stream Merger lock status field) */
	strMergerLockField = FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS_TSFIFO_LINEOK(state->nr+1);
	error |= FE_STiD135_GetDemodLock(state, DemodTimeOut, lock_p);

	if (*lock_p) {
		error |= FE_STiD135_GetFECLock(state, FecTimeOut, &lock);
		if(fec_lock_p) {
			*fec_lock_p = lock;

		}
		*lock_p = *lock_p &&  lock;
	}



	/* LINEOK check is not performed during Satellite Scan */
	if (satellite_scan == FALSE) {
#if 0
		if (*lock_p) {
			*lock_p = 0;

			while ((timer < FecTimeOut) && (*lock_p == 0))
			{
				/*Check the stream merger Lock (good packet at
				the output)*/
				error |= ChipGetField(state->base->ip.handle_demod, strMergerLockField, lock_p);
				ChipWaitOrAbort(state->base->ip.handle_demod, 1);
				timer++;
			}
		}
#endif
	}
	state->signal_info.fec_locked = *lock_p;
	state->signal_info.has_sync = *lock_p;
	{ s32 fld_value;

		error |= ChipGetField(state->base->ip.handle_demod,
													FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS_TSFIFO_LINEOK(state->nr+1), &fld_value);

		vprintk("[%d] SYNC NOW %d line_ok=%d\n", state->nr+1, *lock_p, fld_value);
	}
	if (*lock_p)
		*lock_p = TRUE;
	else
		*lock_p = FALSE;

	return error;

}

static fe_lla_error_t CarrierLoopAutoAdjust(struct stv* state, s32 cnr)
{
	struct fe_stid135_internal_param *pParams;
	u32 symbol_rate=0;
	s32 offset;
	u8 aclcQPSK = 0x08, aclc8PSK = 0x08, aclc16APSK = 0x08, aclc32APSK = 0x08; /* DVBS2 */
	u8 aclc64APSK = 0x08, aclc128APSK = 0x08, aclc256APSK = 0x08; /* DVBS2X */
	u32 i=0;
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	pParams = &state->base->ip;
	/* Read the Symbol rate */
	error |= FE_STiD135_GetSymbolRate(state, pParams->master_clock,  &symbol_rate);
	error |= FE_STiD135_TimingGetOffset(state, symbol_rate, &offset);
	if(offset < 0) {
		offset *= (-1);
		symbol_rate -= (u32)(offset);
	} else
		symbol_rate += (u32)(offset);

	/* QPSK */
	while((i<NB_QPSK_COEFF)&& (cnr>fe_stid135_qpsk_car_loop[i].cnrthres))i++;
	if (i!=0)
		i--;

	if(symbol_rate<=2000000)
		aclcQPSK=fe_stid135_qpsk_car_loop[i].CarLoop_2;
	else if(symbol_rate<=10000000)
		aclcQPSK=fe_stid135_qpsk_car_loop[i].CarLoop_10;
	else if(symbol_rate<=30000000)
		aclcQPSK=fe_stid135_qpsk_car_loop[i].CarLoop_30;
	else if(symbol_rate<=62500000)
		aclcQPSK=fe_stid135_qpsk_car_loop[i].CarLoop_62_5;
	else if(symbol_rate<=125000000)
					aclcQPSK=fe_stid135_qpsk_car_loop[i].CarLoop_125;
	else if(symbol_rate<=300000000)
					aclcQPSK=fe_stid135_qpsk_car_loop[i].CarLoop_300;
	else if(symbol_rate<=400000000)
					aclcQPSK=fe_stid135_qpsk_car_loop[i].CarLoop_400;
	else
					aclcQPSK=fe_stid135_qpsk_car_loop[i].CarLoop_500;


	 /* 8PSK */
	i=0;
	while((i<NB_8PSK_COEFF)&& (cnr>fe_stid135_8psk_car_loop[i].cnrthres))i++;
	if (i!=0)
		i--;

	if(symbol_rate<=2000000)
		aclc8PSK=fe_stid135_8psk_car_loop[i].CarLoop_2;
	else if(symbol_rate<=10000000)
		aclc8PSK=fe_stid135_8psk_car_loop[i].CarLoop_10;
	else if(symbol_rate<=30000000)
		aclc8PSK=fe_stid135_8psk_car_loop[i].CarLoop_30;
	else if(symbol_rate<=62500000)
		aclc8PSK=fe_stid135_8psk_car_loop[i].CarLoop_62_5;
	else if(symbol_rate<=125000000)
					aclc8PSK=fe_stid135_8psk_car_loop[i].CarLoop_125;
	else if(symbol_rate<=300000000)
					aclc8PSK=fe_stid135_8psk_car_loop[i].CarLoop_300;
	else if(symbol_rate<=400000000)
					aclc8PSK=fe_stid135_8psk_car_loop[i].CarLoop_400;
	else
					aclc8PSK=fe_stid135_8psk_car_loop[i].CarLoop_500;

	/* 16APSK */
	i=0;
	while((i<NB_16APSK_COEFF)&& (cnr>fe_stid135_16apsk_car_loop[i].cnrthres))i++;
	if (i!=0)
		i--;

	if(symbol_rate<=2000000)
		aclc16APSK=fe_stid135_16apsk_car_loop[i].CarLoop_2;
	else if(symbol_rate<=10000000)
		aclc16APSK=fe_stid135_16apsk_car_loop[i].CarLoop_10;
	else if(symbol_rate<=30000000)
		aclc16APSK=fe_stid135_16apsk_car_loop[i].CarLoop_30;
	else if(symbol_rate<=62500000)
		aclc16APSK=fe_stid135_16apsk_car_loop[i].CarLoop_62_5;
	else if(symbol_rate<=125000000)
					aclc16APSK=fe_stid135_16apsk_car_loop[i].CarLoop_125;
	else if(symbol_rate<=300000000)
					aclc16APSK=fe_stid135_16apsk_car_loop[i].CarLoop_300;
	else if(symbol_rate<=400000000)
					aclc16APSK=fe_stid135_16apsk_car_loop[i].CarLoop_400;
	else
					aclc16APSK=fe_stid135_16apsk_car_loop[i].CarLoop_500;

	/* 32APSK */
	i=0;
	while((i<NB_32APSK_COEFF)&& (cnr>fe_stid135_32apsk_car_loop[i].cnrthres))i++;
	if (i!=0)
		i--;

	if(symbol_rate<=2000000)
		aclc32APSK=fe_stid135_32apsk_car_loop[i].CarLoop_2;
	else if(symbol_rate<=10000000)
		aclc32APSK=fe_stid135_32apsk_car_loop[i].CarLoop_10;
	else if(symbol_rate<=30000000)
		aclc32APSK=fe_stid135_32apsk_car_loop[i].CarLoop_30;
	else if(symbol_rate<=62500000)
		aclc32APSK=fe_stid135_32apsk_car_loop[i].CarLoop_62_5;
	else if(symbol_rate<=125000000)
					aclc32APSK=fe_stid135_32apsk_car_loop[i].CarLoop_125;
	else if(symbol_rate<=300000000)
					aclc32APSK=fe_stid135_32apsk_car_loop[i].CarLoop_300;
	else if(symbol_rate<=400000000)
					aclc32APSK=fe_stid135_32apsk_car_loop[i].CarLoop_400;
	else
					aclc32APSK=fe_stid135_32apsk_car_loop[i].CarLoop_500;

	/* 64APSK */
	i=0;
	while((i<NB_64APSK_COEFF)&& (cnr>fe_stid135_64apsk_car_loop[i].cnrthres))i++;
	if (i!=0)
		i--;

	if(symbol_rate<=2000000)
		aclc64APSK=fe_stid135_64apsk_car_loop[i].CarLoop_2;
	else if(symbol_rate<=10000000)
		aclc64APSK=fe_stid135_64apsk_car_loop[i].CarLoop_10;
	else if(symbol_rate<=30000000)
		aclc64APSK=fe_stid135_64apsk_car_loop[i].CarLoop_30;
	else if(symbol_rate<=62500000)
		aclc64APSK=fe_stid135_64apsk_car_loop[i].CarLoop_62_5;
	else if(symbol_rate<=125000000)
					aclc64APSK=fe_stid135_64apsk_car_loop[i].CarLoop_125;
	else if(symbol_rate<=300000000)
					aclc64APSK=fe_stid135_64apsk_car_loop[i].CarLoop_300;
	else if(symbol_rate<=400000000)
					aclc64APSK=fe_stid135_64apsk_car_loop[i].CarLoop_400;
	else
					aclc64APSK=fe_stid135_64apsk_car_loop[i].CarLoop_500;

	/* 128APSK */
	i=0;
	while((i<NB_128APSK_COEFF)&& (cnr>fe_stid135_128apsk_car_loop[i].cnrthres))i++;
	if (i!=0)
		i--;

	if(symbol_rate<=2000000)
		aclc128APSK=fe_stid135_128apsk_car_loop[i].CarLoop_2;
	else if(symbol_rate<=10000000)
		aclc128APSK=fe_stid135_128apsk_car_loop[i].CarLoop_10;
	else if(symbol_rate<=30000000)
		aclc128APSK=fe_stid135_128apsk_car_loop[i].CarLoop_30;
	else if(symbol_rate<=62500000)
		aclc128APSK=fe_stid135_128apsk_car_loop[i].CarLoop_62_5;
	else if(symbol_rate<=125000000)
					aclc128APSK=fe_stid135_128apsk_car_loop[i].CarLoop_125;
	else if(symbol_rate<=300000000)
					aclc128APSK=fe_stid135_128apsk_car_loop[i].CarLoop_300;
	else if(symbol_rate<=400000000)
					aclc128APSK=fe_stid135_128apsk_car_loop[i].CarLoop_400;
	else
					aclc128APSK=fe_stid135_128apsk_car_loop[i].CarLoop_500;

	/* 256APSK */
	i=0;
	while((i<NB_256APSK_COEFF)&& (cnr>fe_stid135_256apsk_car_loop[i].cnrthres))i++;
	if (i!=0)
		i--;

	if(symbol_rate<=2000000)
		aclc256APSK=fe_stid135_256apsk_car_loop[i].CarLoop_2;
	else if(symbol_rate<=10000000)
		aclc256APSK=fe_stid135_256apsk_car_loop[i].CarLoop_10;
	else if(symbol_rate<=30000000)
		aclc256APSK=fe_stid135_256apsk_car_loop[i].CarLoop_30;
	else if(symbol_rate<=62500000)
		aclc256APSK=fe_stid135_256apsk_car_loop[i].CarLoop_62_5;
	else if(symbol_rate<=125000000)
					aclc256APSK=fe_stid135_256apsk_car_loop[i].CarLoop_125;
	else if(symbol_rate<=300000000)
					aclc256APSK=fe_stid135_256apsk_car_loop[i].CarLoop_300;
	else if(symbol_rate<=400000000)
					aclc256APSK=fe_stid135_256apsk_car_loop[i].CarLoop_400;
	else
					aclc256APSK=fe_stid135_256apsk_car_loop[i].CarLoop_500;

	//printf( "0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n",aclcQPSK,aclc8PSK,aclc16APSK,aclc32APSK, aclc64APSK, aclc128APSK, aclc256APSK);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(state->nr+1), aclcQPSK);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S28(state->nr+1), aclc8PSK);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S216A(state->nr+1), aclc16APSK);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S232A(state->nr+1), aclc32APSK);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S264A(state->nr+1), aclc64APSK);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2128A(state->nr+1), aclc128APSK);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2256A(state->nr+1), aclc256APSK);

	return(error);
}

/*****************************************************
--FUNCTION	::	FE_STiD135_GetOptimCarrierLoop
--ACTION	::	Computes optimized alpha coefficient value for tracking
--PARAMS IN	::	SymbolRate -> symbol rate of RF signal
			ModCode -> mode code of RF signal
			pilots -> pilots of RF signal
--PARAMS OUT	::	NONE
--RETURN	::	alpha coefficient
--***************************************************/
u8 FE_STiD135_GetOptimCarrierLoop(u32 SymbolRate, enum fe_sat_modcode ModCode,
				s32 pilots)
{
	u8 aclcValue = 0x29;
	u32 i = 0;

	/* Find the index parame\ters for the Modulation */
	while ((i < NB_SAT_MODCOD) && (ModCode != FE_STiD135_S2CarLoop[i].ModCode))
		i++;

	if (i < NB_SAT_MODCOD) {
		if (pilots) {
			if (SymbolRate <= 3000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_2;
			else if (SymbolRate <= 7000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_5;
			else if (SymbolRate <= 15000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_10;
			else if (SymbolRate <= 25000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_20;
			else if (SymbolRate <= 46250000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_30;
			else if (SymbolRate <= 93750000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_62_5;
			else if (SymbolRate <= 212500000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_125;
			else if (SymbolRate <= 350000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_300;
			else if (SymbolRate <= 450000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_400;
			else
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOn_500;
		} else {
			if (SymbolRate <= 3000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_2;
			else if (SymbolRate <= 7000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_5;
			else if (SymbolRate <= 15000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_10;
			else if (SymbolRate <= 25000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_20;
			else if (SymbolRate <= 46250000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_30;
			else if (SymbolRate <= 93750000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_62_5;
			else if (SymbolRate <= 212500000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_125;
			else if (SymbolRate <= 350000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_300;
			else if (SymbolRate <= 450000000)
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_400;
			else
				aclcValue =
				FE_STiD135_S2CarLoop[i].CarLoopPilotsOff_500;
		}
	} else {
		/* Modulation Unknown */
	}


	return aclcValue;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_GetStandard
--ACTION	::	Returns the current standard (DVBS1,DSS or DVBS2
--PARAMS IN	::	hChip -> handle to the chip
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	foundStandard_p -> standard (DVBS1, DVBS2 or DSS)
--RETURN	::	error
*****************************************************/
fe_lla_error_t FE_STiD135_GetStandard(STCHIP_Info_t* hChip,
				enum fe_stid135_demod Demod, enum fe_sat_tracking_standard *foundStandard_p)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u32 stateField, dssdvbField;
	s32 state, fld_value = 0;
	u8 i;

	/* state machine search status field */
	stateField  = FLD_FC8CODEW_DVBSX_DEMOD_DMDSTATE_HEADER_MODE(Demod);
	/* Viterbi standard field */
	dssdvbField = FLD_FC8CODEW_DVBSX_VITERBI_FECM_DSS_DVB(Demod);

	error |= ChipGetField(hChip, stateField, &state);

	/* Fixed NCR issue raised in BZ#98564 */
	/*  When FLYWHEEL_CPT counter becomes 0xF the demodulator is defined as locked (DVBS2 only) */
	for(i=0;i<5;i++) {
		error |= ChipGetField(hChip, FLD_FC8CODEW_DVBSX_DEMOD_DMDFLYW_FLYWHEEL_CPT(Demod), &fld_value);
		error |= ChipGetField(hChip, stateField, &state);
		if((fld_value == 0xF) && (state == 2))
			break;
	}
	if((fld_value == 0xF) && (state == 2)) {
		*foundStandard_p = FE_SAT_DVBS2_STANDARD; /* Demod found DVBS2 */
	}

	else if (state == 3) {			/* Demod found DVBS1/DSS */
		/* Viterbi found DSS */
		error |= ChipGetField(hChip, dssdvbField, &fld_value);
		if (fld_value == 1)
			*foundStandard_p = FE_SAT_DSS_STANDARD;
		else
			*foundStandard_p = FE_SAT_DVBS1_STANDARD;
			/* Viterbi found DVBS1 */
	} else {
		*foundStandard_p = FE_SAT_UNKNOWN_STANDARD;
		/* Demod found nothing, standard unknown */
	}

	return error;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_CarrierGetQuality
--ACTION	::	Returns the carrier to noise of the current carrier
--PARAMS IN	::	hChip -> handle to the chip
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	c_n_p -> C/N of the carrier, 0 if no carrier
--RETURN	::	error
--***************************************************/
fe_lla_error_t FE_STiD135_CarrierGetQuality(STCHIP_Info_t* hChip, enum fe_stid135_demod Demod, s32* c_n_p,
																						enum fe_sat_tracking_standard* foundStandard)
{
	u32 lockFlagField, noiseField1, noiseField0;

	s32 regval,
	Imin,
	Imax,
	i;
	u16 noiseReg;
	fe_lla_lookup_t *lookup = NULL;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	*c_n_p = -100,

	lockFlagField =	FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_LOCK_DEFINITIF(Demod);
	error |= FE_STiD135_GetStandard(hChip,Demod, foundStandard);
	//dprintk("demod=%d foundStandard=%d\n", Demod, *foundStandard);
	if (*foundStandard == FE_SAT_DVBS2_STANDARD) {
		lookup = &FE_STiD135_S2_CN_LookUp;
		/*If DVBS2 use PLH normalized noise indicators*/
		noiseField1 = FLD_FC8CODEW_DVBSX_DEMOD_NNOSPLHT1_NOSPLHT_NORMED(Demod);
		noiseField0 = FLD_FC8CODEW_DVBSX_DEMOD_NNOSPLHT0_NOSPLHT_NORMED(Demod);
		noiseReg = (u16)REG_RC8CODEW_DVBSX_DEMOD_NNOSPLHT1(Demod);
	} else {
			lookup = &FE_STiD135_S1_CN_LookUp;
		/*if not DVBS2 use symbol normalized noise indicators*/
		noiseField1 = FLD_FC8CODEW_DVBSX_DEMOD_NNOSDATAT1_NOSDATAT_NORMED(Demod);
		noiseField0 = FLD_FC8CODEW_DVBSX_DEMOD_NNOSDATAT0_NOSDATAT_NORMED(Demod);
		noiseReg = (u16)REG_RC8CODEW_DVBSX_DEMOD_NNOSDATAT1(Demod);
	}
	error |= ChipGetField(hChip, lockFlagField, &regval);
	if (regval) {
		if ((lookup != NULL) && lookup->size) {
			regval = 0;
			/* ChipWaitOrAbort(hChip,5);*/
			for(i = 0; i<8; i++)
			{
				error |= ChipGetRegisters(hChip, noiseReg, 2);
				regval += MAKEWORD16(ChipGetFieldImage(hChip, noiseField1),
								 ChipGetFieldImage(hChip, noiseField0));

				ChipWaitOrAbort(hChip, 1);
			}
			regval /=8;

			Imin = 0;
			Imax = lookup->size-1;
			BUG_ON(Imin<0 || Imin>=sizeof(lookup->table)/sizeof(lookup->table[0]));
			BUG_ON(Imax<0 || Imax>=sizeof(lookup->table)/sizeof(lookup->table[0]));
			if (INRANGE(lookup->table[Imin].regval, regval,
									lookup->table[Imax].regval))
				{
				while ((Imax - Imin) > 1) {
					i = (Imax + Imin) >> 1;
					BUG_ON(Imin<0 || Imin>=sizeof(lookup->table)/sizeof(lookup->table[0]));
					if (INRANGE(lookup->table[Imin].regval,
						regval,lookup->table[i].regval))
						Imax = i;
					else
						Imin = i;
				}
				BUG_ON(Imin<0 || Imin>=sizeof(lookup->table)/sizeof(lookup->table[0]));
				BUG_ON(Imax<0 || Imax>=sizeof(lookup->table)/sizeof(lookup->table[0]));
				*c_n_p = ((regval - lookup->table[Imin].regval)
					* (lookup->table[Imax].realval
					- lookup->table[Imin].realval)
					/ (lookup->table[Imax].regval
					- lookup->table[Imin].regval))
					+ lookup->table[Imin].realval;
			} else if (regval < lookup->table[Imin].regval)
				*c_n_p = 1000;
		}
	}

	return error;
}


/*****************************************************
--FUNCTION	::	FE_STiD135_GetDemodLock
--ACTION	::	Returns the demod lock status
--PARAMS IN	::	handle -> Frontend handle
			TimeOut -> Demod timeout
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	Lock_p -> lock status, boolean
--RETURN	::	error
--***************************************************/
static fe_lla_error_t FE_STiD135_GetDemodLock (struct stv* state, u32 TimeOut, BOOL *Lock_p)
{
	u32 headerField, lockField, symbFreq1, symbFreq2;
	s32 lock = 0;
	u16 symbFreqRegister;
	//u32 symbolRate ;
	u32 TimeOut_SymbRate, SRate_1MSymb_Sec;
	u32 timer = 0;
	s32 fld_value, slc_min, slc_max, slc_sel;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u32 MclkFreq = 0;
	u8 timeout = 0;

	enum fe_sat_search_state demodState;
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	/* state machine search status field*/
	headerField = FLD_FC8CODEW_DVBSX_DEMOD_DMDSTATE_HEADER_MODE(state->nr+1);
	/* Demod lock status field*/
	lockField   = FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_LOCK_DEFINITIF(state->nr+1);

	symbFreqRegister = (u16)REG_RC8CODEW_DVBSX_DEMOD_SFR2(state->nr+1);

	symbFreq1 = FLD_FC8CODEW_DVBSX_DEMOD_SFR1_SYMB_FREQ(state->nr+1);
	symbFreq2 = FLD_FC8CODEW_DVBSX_DEMOD_SFR2_SYMB_FREQ(state->nr+1);

	TimeOut_SymbRate = TimeOut;
	vprintk("[%d] Timeout=%d\n", state->nr+1, TimeOut_SymbRate);
	 /* SR_register = 2^16 / (12 * MCLK) */
	//SRate_1MSymb_Sec = 0x1e5;
	MclkFreq = pParams->master_clock;

	SRate_1MSymb_Sec = (1<<16) * 1 / (12*MclkFreq/1000000);
	vprintk("[%d] using srate dependent timeout\n", state->nr+1);

	while ((timer < TimeOut_SymbRate) && (lock == 0)) {
		int old;
		int symbolRate;
		if (kthread_should_stop() || dvb_frontend_task_should_stop(&state->fe)) {
			dprintk("exiting on should stop\n");
			timer = TimeOut_SymbRate;
			break;
		}

		old = TimeOut_SymbRate;
		error |= ChipGetRegisters(state->base->ip.handle_demod, symbFreqRegister, 2);
		symbolRate = (u32)
			((ChipGetFieldImage(state->base->ip.handle_demod, symbFreq2) << 8)+
			 (ChipGetFieldImage(state->base->ip.handle_demod, symbFreq1)));
		if (TimeOut < DmdLock_TIMEOUT_LIMIT) {
			TimeOut_SymbRate = TimeOut;
		} else {
			/* no division by 0 */
			if (symbolRate < SRate_1MSymb_Sec)
				symbolRate = SRate_1MSymb_Sec;
			else if (symbolRate > (5 * SRate_1MSymb_Sec))
				symbolRate = 5 * SRate_1MSymb_Sec;

			TimeOut_SymbRate = TimeOut / (symbolRate / SRate_1MSymb_Sec);

			/* no weird results */
			if (TimeOut_SymbRate < DmdLock_TIMEOUT_LIMIT)
				TimeOut_SymbRate = DmdLock_TIMEOUT_LIMIT ;
			else if (TimeOut_SymbRate > TimeOut)
				TimeOut_SymbRate = TimeOut ;

			/* The new timeout is between 200 ms and original
				 TimeOut */
		}
		if(old != TimeOut_SymbRate ) {
			vprintk("[%d] timeout changed to %d\n", state->nr+1,  TimeOut_SymbRate);
		}


	error |= ChipGetField(state->base->ip.handle_demod, headerField, &fld_value);
	demodState = (enum fe_sat_search_state)fld_value;
	switch (demodState) {
	case FE_SAT_SEARCH: //value 00
	case FE_SAT_PLH_DETECTED :	//01 1st DVB-S2 PLHeader detected, searching for residual offset symbol.
		lock = 0;
		state->signal_info.has_carrier = false;
		if(state->signal_info.has_viterbi)
			dprintk("[%d] FE_SAT_PLH_DETECTED: viterbi=%d\n", state->nr+1, state->signal_info.has_viterbi);
		break;

	case FE_SAT_DVBS2_FOUND: /* value10: found a DVBS2 signal */
	case FE_SAT_DVBS_FOUND: //value 11  DVB-S1/Legacy DTV mode
		error |= ChipGetField(state->base->ip.handle_demod, lockField, &lock);
		vprintk("[%d] DVBS%d detected lock=%d\n", state->nr+1, (demodState==FE_SAT_DVBS_FOUND)?1:2, lock);
		break;
	}
	state->signal_info.has_carrier = lock;
#if 0
	if(state->signal_info.has_viterbi)
		dprintk("!!!!!  set has_carrier=0\n");
#endif
	if(lock == 0) {
#if 0
		state->signal_info.has_viterbi = false;
			state->signal_info.has_sync = false;
#endif
			mutex_unlock(pParams->master_lock);
			ChipWaitOrAbort(state->base->ip.handle_demod, 10);	/* wait 10ms */
			mutex_lock(pParams->master_lock);
		}
		//report(state);
		timer += 10;

	}
	if(lock == TRUE) {
		vprintk("[%d] LOCK_DEFINITIF achieved timout=%d/%d\n", state->nr+1, timer, TimeOut_SymbRate);
		state->signal_info.demod_locked = true;
		/* We have to wait for demod locked before reading ANNEXEM field (cut 1 only) */
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS6_SIGNAL_ANNEXEM(state->nr+1), &fld_value);
		/* Dummy write to reset slicemin (if DVBS1 test followed by DVBS2 test) */
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICEMIN_DEMODFLT_SLICEMIN(state->nr+1), 0);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICEMAX_DEMODFLT_SLICEMAX(state->nr+1), 0);
		if((fld_value == 0) || (fld_value == 1)) {
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICESEL_DEMODFLT_SLICESEL(state->nr+1), 0);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSDIVN_BYTE_OVERSAMPLING(state->nr+1), 0);
		} else { /* sliced mode */
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICEMIN_DEMODFLT_SLICEMIN(state->nr+1), &slc_min);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICEMAX_DEMODFLT_SLICEMAX(state->nr+1), &slc_max);
			while((slc_min == 255) && (slc_max == 0) && (timeout < 40)) { // 255 is not valid value for slicemin, 0 is not valid for slicemax
				ChipWaitOrAbort(state->base->ip.handle_demod,5);
				timeout = (u8)(timeout + 5);
				error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICEMIN_DEMODFLT_SLICEMIN(state->nr+1), &slc_min);
				error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICEMAX_DEMODFLT_SLICEMAX(state->nr+1), &slc_max);
			}
			if(timeout >= 40) {// we force to 1 slice number
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICESEL_DEMODFLT_SLICESEL(state->nr+1), slc_min);
				#ifdef USER2
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICESEL_DEMODFLT_SLICESEL(state->nr+1), 4);
				#endif
			} else {
				error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICESEL_DEMODFLT_SLICESEL(state->nr+1), &slc_sel);
				if((slc_sel < slc_min) || (slc_sel > slc_max)) {
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICESEL_DEMODFLT_SLICESEL(state->nr+1), slc_min);
				}
			}
		}
	} else {
		vprintk("[%d] timedout %d/%d\n", state->nr+1, timer, TimeOut_SymbRate);
	}
	*Lock_p = (BOOL) lock;
	state->signal_info.demod_locked = lock;
	return error;
}


/*****************************************************
--FUNCTION	::	FE_STiD135_GetFECLock
--ACTION	::	Returns the FEC lock status
--PARAMS IN	::	hChip -> handle of the chip
			TimeOut -> FEC timeout
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	lock_bool_p -> lock status, boolean
--RETURN	::	error
--***************************************************/
fe_lla_error_t FE_STiD135_GetFECLock(struct stv* state,
				u32 TimeOut, BOOL* lock_bool_p)
{
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod Demod = state->nr+1;
	u32 headerField, pktdelinField, lockVitField, timer = 0;
	s32 lock = 0;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	s32 fld_value;
	enum fe_sat_search_state demodState;

	/* P1_DMDSTATE state machine search status field */
	headerField   = FLD_FC8CODEW_DVBSX_DEMOD_DMDSTATE_HEADER_MODE(Demod);
	/* P1_PDELSTATUS1 packet delin (DVBS 2) lock status field */
	pktdelinField = FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_LOCK(Demod);
	/* P1_LOCKEDVIT Viterbi (DVBS1/DSS) lock status field */
	lockVitField  = FLD_FC8CODEW_DVBSX_VITERBI_VSTATUSVIT_LOCKEDVIT(Demod);

	error |= ChipGetField(hChip, headerField, &fld_value);
	demodState = (enum fe_sat_search_state)fld_value;

	while ((timer < TimeOut) && (lock == 0)) {

		switch (demodState) {
		case FE_SAT_SEARCH:
		case FE_SAT_PLH_DETECTED : /* no signal*/
			state->signal_info.has_carrier = false;
			state->signal_info.has_viterbi = false;
			//state->signal_info.has_sync = false;

			lock = 0;
		break;

		case FE_SAT_DVBS2_FOUND: /* found a DVBS2 signal */
			error |= ChipGetField(hChip, pktdelinField, &lock);
			state->signal_info.has_carrier = true;
			state->signal_info.has_viterbi = lock;
		break;

		case FE_SAT_DVBS_FOUND:
			error |= ChipGetField(hChip,lockVitField, &lock);
			state->signal_info.has_carrier = true;
			state->signal_info.has_viterbi = lock;
		break;
		}
		state->signal_info.fec_locked =lock;
		if (lock == 0)
		{
			ChipWaitOrAbort(hChip, 10);
			timer += 10;
		}
	}
	*lock_bool_p = (BOOL)lock;

	return error;
}



/*****************************************************
--FUNCTION	::	FE_STiD135_CarrierWidth
--ACTION	::	Computes the carrier width from symbol rate
			and roll off
--PARAMS IN	::	SymbolRate -> Symbol rate of the carrier
			(Kbauds or Mbauds)
			roll_off -> Rolloff * 100
--PARAMS OUT	::	NONE
--RETURN	::	Width of the carrier (KHz or MHz)
--***************************************************/
u32 FE_STiD135_CarrierWidth(u32 SymbolRate, enum fe_sat_rolloff roll_off)
{
	u32 rolloff = 0;

	switch (roll_off) {
	case FE_SAT_05:  rolloff = 05;  break;
	case FE_SAT_10:  rolloff = 10;  break;
	case FE_SAT_15:  rolloff = 15;  break;
	case FE_SAT_20:  rolloff = 20;  break;
	case FE_SAT_25:  rolloff = 25;  break;
	case FE_SAT_35:
			rolloff = 35;
	break;
	}
	return (SymbolRate + (SymbolRate * rolloff) / 100);
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_reg_init_values
--ACTION	::	Sets initial values of registers to be clean
			before an acquisition
--PARAMS IN	::	handle -> Frontend handle
			demod -> current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	error
--***************************************************/
static fe_lla_error_t fe_stid135_set_reg_init_values(struct stv* state)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	//p. 316 measuring point=demod iq out; normal mode, observe the selection iqsymb_sel;
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEMOD(state->nr+1), 0x00);

		//DVBS2 enable; no autoscan; no dvb s1  (default setting)
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(state->nr+1), 0xc8);

	//set loop carrier1 coefficients
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(state->nr+1), 0x45); // to make demod lock in DVBS1 (cold start only)

	//derotator 1b
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1BCFG(state->nr+1), 0x67);

	//p. 444 : authorize dvbs2 (default value)
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_PLSMSCRAMBB(state->nr+1), 0x70);

	//latency regulation
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSDLYSET2(state->nr+1), 0x31);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSDLYSET1(state->nr+1), 0x2B);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSDLYSET0(state->nr+1), 0x00);

	//differential correlator limit
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELMANT(state->nr+1), 0x78);

	/* New Setting by SG for NB low C/N */
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELABS(state->nr+1), 0x70);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELCOEF(state->nr+1), 0x20);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELFREQ(state->nr+1), 0x70);

	/* Reset matype forcing mode */
	error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_BBHCTRL2_FORCE_MATYPEMSB(state->nr+1), 0);
	error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL0_HEMMODE_SELECT(state->nr+1), 0);

	/* Remove MPEG packetization mode */
	error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_EMBINDVB(state->nr+1), 0);

	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(state->nr+1), 0x08);

	/* Go back to initial value (may be adapted in tracking function) */
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DFECFG(state->nr+1), 0xC1);
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_XFECFG2(state->nr+1), 0x03);

	/* Go back to default value in NB (lot of dummy PL frames in WB) */
	error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(state->nr+1), 0x25);

	/* Go back to initial value for MIS+ISSYI use-case */
	error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPCRPID1_SOFFIFO_PCRADJ(state->nr+1), 0);

	return error;
}

void tstxxx(struct stv* state)
{
	fe_lla_error_t error;
	fe_lla_error_t error1;
	u32 reg1,reg2,reg3;
	int Demod = state->nr+1;
	error = ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(Demod), 0xc6);
	return;
	if(state->nr !=0)
		return;
	//fe_stid135_set_reg_init_values(state);
	//ChipGetField(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELCFG(state->nr+1), &reg1);
	error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELCFG(state->nr+1), 0x83));
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(state->nr+1), &reg3);

	ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_DEMODFLT_XXXMODE(state->nr+1), &reg2);//0x14
	error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_DEMODFLT_XXXMODE(state->nr+1), 0x15));
	error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod), 0));
	error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x5C));
	error |= (error1=fe_stid135_set_symbol_rate(state, 27500000));
	printk("DEMOD_CARCFG was 0x%x\n", reg3);
	tst1(state, 27500000);
	return;
	error |= (error1=fe_stid135_set_carrier_frequency_init(state,1916200));
	error |= (error1=ChipSetFieldImage(state->base->ip.handle_demod,
																		 FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_SPECINV_CONTROL(Demod),
																		 FE_SAT_IQ_AUTO));   /* Set IQ inversion always in auto */

	error |= (error1=ChipSetRegisters(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEMOD(Demod), 1));

	printk("DEMOD_CORRELCFG was 0x%x\n", reg1);
	printk("DEMODFLT_XXXMODE was 0x%x\n", reg2);
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_reg_values_wb
--ACTION	::	Sets initial values of registers to be clean
			before an acquisition only in wideband usecase
--PARAMS IN	::	handle -> Frontend handle
			demod -> current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	error
--***************************************************/
static fe_lla_error_t fe_stid135_set_reg_values_wb(struct stv* state)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	enum fe_stid135_demod demod = state->nr+1;
		if (state->demod_search_range_hz > 24000000)
			state->demod_search_range_hz = 24000000;

		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(demod), 0x80);
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(demod), 0x00);
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1BCFG(demod), 0xc5);
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_PLSMSCRAMBB(demod), 0x40);

		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSDLYSET2(demod), 0xE2);
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSDLYSET1(demod), 0x0A);
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSDLYSET0(demod), 0x00);

		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELMANT(demod), 0x6A);
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELCOEF(demod), 0x20);
		/* Reset new Setting by SG for NB low C/N */
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELABS(demod), 0x8c);
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELFREQ(demod), 0x30);

		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(demod), 0x18);

		/* On a signal with lot of dummy PL frames in WB, then need to adjust CFR2CFR1 */
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(demod), 0x23);

	return error;
}
#ifdef USER2
/*****************************************************
--FUNCTION	::	fe_stid135_manage_manual_rolloff
--ACTION	::	Manages rolloff value if user knows its value
--PARAMS IN	::	handle -> Frontend handle
			demod -> current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	error
--***************************************************/
static fe_lla_error_t fe_stid135_manage_manual_rolloff(struct fe_stid135_internal_param* pParams, enum fe_stid135_demod demod)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams = NULL;

	if(handle != NULL) {
		pParams = (struct fe_stid135_internal_param *) handle;


#ifdef TOTEST //DeepThought This may not be needed 20201226
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALS2_ROLLOFF(demod), 1);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALSX_ROLLOFF(demod), 1);
#endif
		switch(pParams->roll_off[demod-1]) {
			case FE_SAT_05:
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(demod), 4);
			break;
			case FE_SAT_10:
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(demod), 3);
			break;
			case FE_SAT_15:
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(demod), 5);
			break;
			case FE_SAT_20:
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(demod), 2);
			break;
			case FE_SAT_25:
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(demod), 1);
			break;
			case FE_SAT_35:
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(demod), 0);
			break;
			/*TD*/
			default: /* by default automatic rolloff */
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALS2_ROLLOFF(demod), 0);
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALSX_ROLLOFF(demod), 0);

			break;
		}
	}
	else
		error |= FE_LLA_INVALID_HANDLE;

	return error;
}
#endif
/*****************************************************
--FUNCTION	::	fe_stid135_search
--ACTION	::	Search for a valid transponder
--PARAMS IN	::	handle -> Front End Handle
			pSearch -> Search parameters
			pResult -> Result of the search
			demod -> current demod 1..8
			satellite_scan ==> scan scope (0, 1 2)
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t	fe_stid135_search(struct stv* state,
	struct fe_sat_search_params *pSearch,  BOOL satellite_scan)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;

	enum fe_sat_signal_type signalType = FE_SAT_NOCARRIER;
	s32 fld_value;
	struct fe_stid135_internal_param *pParams = &state->base->ip;

	if ((!(INRANGE(100000, pSearch->symbol_rate,  520000000)))) {
		dprintk("[%d] error=FE_LLA_BAD_PARAMETER: symbol_rate=%d\n", state->nr+1,  pSearch->symbol_rate);
		return FE_LLA_BAD_PARAMETER;
	}

	if ((!(INRANGE(100000, pSearch->search_range_hz, 70000000)))) {
		dprintk("[%d] error=FE_LLA_BAD_PARAMETER: search_range=%d\n", state->nr+1,  pSearch->search_range_hz);
		return FE_LLA_BAD_PARAMETER;
	}

	if(state->base->ip.handle_demod->Error) {
		dprintk("[%d] CALLED with error set to %d (correcting)\n", state->nr+1, state->base->ip.handle_demod->Error);
		state->base->ip.handle_demod->Error = FE_LLA_NO_ERROR;
	}
	if (state->base->ip.handle_demod->Error) {
		dprintk("[%d] error=%d\n", state->nr+1, FE_LLA_I2C_ERROR);
		return FE_LLA_I2C_ERROR;
	}

	pParams->lo_frequency = pSearch->lo_frequency;

	state->tuner_frequency = (s32)(
		pSearch->frequency - pSearch->lo_frequency);
	state->demod_search_stream_id = pSearch->stream_id;
	state->demod_search_standard = pSearch->standard;
	state->demod_symbol_rate = pSearch->symbol_rate;
	state->demod_search_range_hz = pSearch->search_range_hz;
	state->demod_search_algo = pSearch->search_algo;
	state->demod_search_iq_inv = pSearch->iq_inversion;
	state->mis_mode = FALSE; /* Disable memorisation of MIS mode */

	/* Set default register values to start a clean search */
	error |= (error1=fe_stid135_set_reg_init_values(state)); //XXOK
	if(error1)
		dprintk("[%d] error=%d\n", state->nr+1, error1);

#ifdef USER2
	if(((pSearch->search_algo == FE_SAT_COLD_START) || (pSearch->search_algo == FE_SAT_WARM_START))
		&& (pSearch->symbol_rate >= pParams->master_clock >> 1)) { /* if SR >= MST_CLK / 2 */
		error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_MATCST1(demod), 0x70));
		if(error1)
			dprintk("[%d] error=%d\n", state->nr+1, error1);

			error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICESEL_DEMODFLT_SLICESEL(demod), 4));
			if(error1)
				dprintk("[%d] error=%d\n", state->nr+1, error1);

			/* no else part as it is done by set_reg_init_values() function */
	}
#endif
#ifdef ATB
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(demod), 2);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALS2_ROLLOFF(demod), 1);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALSX_ROLLOFF(demod), 1);
	#endif
#ifdef USER2
		pParams->roll_off[demod-1] = pSearch->roll_off;
		if(pSearch->man_rolloff == TRUE) {
			error |= fe_stid135_manage_manual_rolloff(handle, demod);
		}
#endif
#ifdef TOTEST //deepthough 20201226 In the main driver this code is disabled
#else
		if((pSearch->symbol_rate >= pParams->master_clock) &&
			 (state->demod_search_algo == FE_SAT_BLIND_SEARCH ||
				state->demod_search_algo == FE_SAT_NEXT)
			 ) /* if SR >= MST_CLK  & Blind search algo : usecase forbidden */ {
			dprintk("[%d] error=FE_LLA_NOT_SUPPORTED srate=%d\n", state->nr+1, pSearch->symbol_rate);
			return(FE_LLA_NOT_SUPPORTED);
		}

	/* Check if user wants to lock demod on a chip where High Symbol Rate feature is forbidden on this chip */
	error |= (error1=ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDSELOBS_NOSHDB_SEL(demod), &fld_value));
	if(error1)
		dprintk("[%d] error=%d\n", state->nr+1, error1);

	if((pSearch->symbol_rate >= pParams->master_clock) && fld_value == 3) {
		error1=FE_LLA_NOT_SUPPORTED;
		dprintk("[%d] error=%d\n", state->nr+1, error1);
		return(FE_LLA_NOT_SUPPORTED);
	}

	if(pSearch->symbol_rate >= pParams->master_clock) { /* if SR >= MST_CLK */
		vprintk("[%d] Wideband code called; warm start forced\n", state->nr+1);
		state->demod_search_algo = FE_SAT_WARM_START; // we force warm algo if SR > MST_CLK
		error |= (error1=fe_stid135_set_reg_values_wb(state));
		if(error1)
			dprintk("[%d] error=%d\n", state->nr+1, error1);

		error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELCFG(demod), 0x83));
		if(error1)
			dprintk("[%d] error=%d\n", state->nr+1, error1);
		error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_DEMODFLT_XXXMODE(demod), 0x15));
		vprintk("[%d] XXXMODE set t0 0x15\n", state->nr+1);
		if(error1)
			dprintk("[%d] error=%d\n", state->nr+1, error1);
	} else if((pSearch->symbol_rate >= pParams->master_clock >> 1) && (pSearch->symbol_rate < pParams->master_clock)) { /* if SR >= MST_CLK / 2 */
		vprintk("[%d] Force Medium symbol rate\n", state->nr+1);
		error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELCFG(demod), 0x83));
		if(error1)
			dprintk("[%d] error=%d\n", state->nr+1, error1);
		error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_DEMODFLT_XXXMODE(demod), 0x15));
		vprintk("[%d] XXXMODE set t0 0x15\n", state->nr+1);
		if(error1)
			dprintk("[%d] error=%d\n", state->nr+1, error1);
	} else {
		vprintk("[%d] other symbol rate\n", state->nr+1);
		error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CORRELCFG(demod), 0x01));
		if(error1)
			dprintk("[%d] error=%d\n", state->nr+1, error1);
	}
#endif //TOTEST
	state->demod_puncture_rate = pSearch->puncture_rate;
	state->demod_modulation = pSearch->modulation;
	state->demod_modcode = pSearch->modcode;
	state->tuner_index_jump = pSearch->tuner_index_jump;
	//dprintk("CHECK pr=%d mod=%d modcode=%d jump =%d\n", pSearch->puncture_rate,  pSearch->modulation, pSearch->modcode, pSearch->tuner_index_jump);
	if (error != FE_LLA_NO_ERROR) {
		error1=FE_LLA_NO_ERROR;
		dprintk("[%d] error=%d\n", state->nr+1, error1);
		return FE_LLA_BAD_PARAMETER;
	}

	if (demod == FE_SAT_DEMOD_2) {
		if(pSearch->symbol_rate > pParams->master_clock >> 1) { /* if SR >= MST_CLK / 2 */
			error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(demod), 3));
			if(error1)
				vprintk("[%d] error=%d\n", state->nr+1, error1);
			WAIT_N_MS(10);
			error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_PEGALCFG(demod), 0x1C));
			if(error1)
				dprintk("[%d] error=%d\n", state->nr+1, error1);
			} else {
			error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(demod), 2));
			if(error1)
				dprintk("[%d] error=%d\n", state->nr+1, error1);
			error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_PEGALCFG(demod), 0x00));
				if(error1)
					dprintk("[%d] error=%d\n", state->nr+1, error1);
			}
	}
	if (demod == FE_SAT_DEMOD_4) {
		if(pSearch->symbol_rate > pParams->master_clock >> 1) { /* if SR >= MST_CLK / 2 */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(demod), 3);
				WAIT_N_MS(10);
				error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_PEGALCFG(demod), 0x1C));
				if(error1)
					dprintk("[%d] error=%d\n", state->nr+1, error1);
			} else {
			error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(demod), 2));
			if(error1)
				dprintk("[%d] error=%d\n", state->nr+1, error1);
			error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_PEGALCFG(demod), 0x00));
			if(error1)
				dprintk("[%d]error=%d\n", state->nr+1, error1);
			}
	}

	error |= (error1=FE_STiD135_Algo(state, satellite_scan, &signalType));
	if(error1)
		dprintk("[%d] error=%d signalType=%d\n", state->nr+1, error1, signalType);

	pSearch->tuner_index_jump = state->tuner_index_jump;
	BUG_ON(state->base->ip.handle_demod->Error);
	if (signalType == FE_SAT_TUNER_JUMP) {
		error = FE_LLA_TUNER_JUMP;
		vprintk("[%d] error=%d\n", state->nr+1, error);

	} else if (signalType == FE_SAT_TUNER_NOSIGNAL) {
		/* half of the tuner bandwith jump */
		error = FE_LLA_TUNER_NOSIGNAL ;
		dprintk("[%d]error=%d\n", state->nr+1, error);
	}
	else if (((signalType == FE_SAT_RANGEOK)
						 || ((satellite_scan > 0)
						 && (signalType == FE_SAT_NODATA)))
						&& (state->base->ip.handle_demod->Error == CHIPERR_NO_ERROR))
	{
		if ((satellite_scan > 0) && (signalType == FE_SAT_NODATA)) {
			/* TPs with demod lock only are logged as well */
			error = FE_LLA_NODATA;
			dprintk("[%d]error=%d\n", state->nr+1, error);
		} else {
			error = FE_LLA_NO_ERROR;
		}
		BUG_ON(state->base->ip.handle_demod->Error);
		if (state->base->ip.handle_demod->Error) {
			error = FE_LLA_I2C_ERROR;
			dprintk("[%d] error=%d\n", state->nr+1, error);
		}

		/* Optimization setting for tracking */
		error |= (error1=FE_STiD135_TrackingOptimization(state));
		if(error1)
			dprintk("[%d] error=%d\n", state->nr+1, error1);


		/* Reset obs registers */
			error |= (error1=fe_stid135_reset_obs_registers(state));
			if(error1)
				dprintk("[%d] error=%d\n", state->nr+1, error1);

	} else {
		state->signal_info.has_lock = false;
#if 0
		state->signal_info.has_carrier = false;
		if(state->signal_info.has_viterbi)
			dprintk("!!!!!  set has_carrier=0\n");
#endif

		switch (state->demod_error) {
			/*I2C error*/
		case FE_LLA_I2C_ERROR: {
			error = FE_LLA_I2C_ERROR;
			dprintk("[%d] error=%d\n", state->nr+1, error);
		}
				break;
			case FE_LLA_INVALID_HANDLE:
			case FE_LLA_ALLOCATION:
			case FE_LLA_BAD_PARAMETER:
			case FE_LLA_SEARCH_FAILED:
			case FE_LLA_TRACKING_FAILED:
			case FE_LLA_NODATA:
			case FE_LLA_TUNER_NOSIGNAL:
			case FE_LLA_TUNER_JUMP:
			case FE_LLA_TUNER_4_STEP:
			case FE_LLA_TUNER_8_STEP:
			case FE_LLA_TUNER_16_STEP:
			case FE_LLA_TERM_FAILED:
			case FE_LLA_DISEQC_FAILED:
			case FE_LLA_NOT_SUPPORTED:
				error = FE_LLA_SEARCH_FAILED;
				dprintk("[%d] error=%d\n", state->nr+1, error);
				break;
			case FE_LLA_NO_ERROR:
				error = FE_LLA_NO_ERROR;
				dprintk("[%d] error=%d\n", state->nr+1, error);
				break;
		}
		BUG_ON(state->base->ip.handle_demod->Error);
		if (state->base->ip.handle_demod->Error) {
			error = FE_LLA_I2C_ERROR;
			dprintk("[%d] error=%d\n", state->nr+1, error);
		}
	}

	return error;
}



/*****************************************************
--FUNCTION	::	FE_STiD135_GetLockTimeout
--ACTION	::	Returns the demod and fec lock timeout in function
			of the symbol rate
--PARAMS IN	::	SymbolRate -> Symbol rate
			Algo -> algorithm used
--PARAMS OUT	::	DemodTimeout -> computed timeout for demod
			FecTimeout -> computed timeout for FEC
--RETURN	::	NONE
--***************************************************/
static void FE_STiD135_GetLockTimeout(u32 *DemodTimeout, u32 *FecTimeout,
		u32 SymbolRate, enum fe_sat_search_algo Algo)
{
	switch (Algo) {
	case FE_SAT_BLIND_SEARCH:
			(*DemodTimeout) = 3000;
			(*FecTimeout) = 3000;
			if (SymbolRate <= 1000000) {       /*SR <=1Msps*/
				(*DemodTimeout) *= 4;
				(*FecTimeout) *= 4;
			}
	break;
	case FE_SAT_NEXT:
		dprintk("Setting long timeouts\n");
			(*DemodTimeout) = 8000; // Fixed issue BZ#86598
			(*FecTimeout) = 8000;   // Fixed issue BZ#86598

		break;
	case FE_SAT_COLD_START:
	case FE_SAT_WARM_START:
		if (SymbolRate <= 1000000) {       /*SR <=1Msps*/
			(*DemodTimeout) = 3000;
			(*FecTimeout) = 2000;  /*1700 */
		} else if (SymbolRate <= 2000000) { /*1Msps < SR <=2Msps*/
			(*DemodTimeout) = 2500;
			(*FecTimeout) = 1300; /*1100 */
		} else if (SymbolRate <= 5000000) { /*2Msps< SR <=5Msps*/
			(*DemodTimeout) = 1000;
			(*FecTimeout) = 650; /* 550 */
		} else if (SymbolRate <= 10000000) { /*5Msps< SR <=10Msps*/
			(*DemodTimeout) = 700;
			(*FecTimeout) = 350; /*250 */
		} else if (SymbolRate <= 20000000) { /*10Msps< SR <=20Msps*/
			(*DemodTimeout) = 400;
			(*FecTimeout) = 200; /* 130 */
		} else {  /*SR >20Msps*/
			(*DemodTimeout)=300;
			(*FecTimeout)=200; /* 150 */
		}
	break;

	}
	if (Algo == FE_SAT_WARM_START) {
		/*if warm start
		demod timeout = coldtimeout/3
		fec timeout = same as cold*/
		(*DemodTimeout) /= 2;
	}

}


/**********************************************************************************
FUNCTION   : Estimate_Power_Int
ACTION     : basic function to verify channel power estimation (PchRF) and
						 band  power estimation (Pband) in order to verify designers
						 formulas principle inside ATB
						 Config is hardwired on TUNER1/DEMOD1
						 This function handle integer values for LLA

PARAMS IN  : Handle
             TunerNb[1;4]
             DemodNb [0;7]
             PowerIref should be set to 105
PARAMS OUT :  *AGCRFIN1,
							 *AGCRFIN0,
							 *Agc1,
							 *AGC1POWERI,
							 *AGC1POWERQ,
							 *AGC1REF,
							 *AGC2REF,
							 *AGC2I1,
							 *AGC2I0,
							 *mant,
							 *exp,
							 *Agc2,
							 *GvDig,
							 *InterpolatedGvana,
							 *PchRF,
							 *Pband
RETURN     : error
**********************************************************************************/
static fe_lla_error_t Estimate_Power_Int(struct stv* state,
					u8 TunerNb,
					u8 *AGCRFIN1,
					u8 *AGCRFIN0,
					u32 *Agc1,
					u8 *AGC1POWERI,
					u8 *AGC1POWERQ,
					u8 *AGC1REF,
					u8 *AGC2REF,
					u8 *AGC2I1,
					u8 *AGC2I0,
					u16 *mant,
					u8 *exp,
					u32 *Agc2x1000,
					u32*Gvanax1000,
					s32*PchRFx1000, /*estimated narrow band input power measured at the rf input */
					s32*Pbandx1000) /*wide band input signal power measured at the rf input*/
{
	enum fe_stid135_demod Demod = state->nr+1;
	s32 exp_abs_s32=0, exp_s32=0;
	u32 agc2x1000=0;
	u32 gain_analogx1000=0;
	u8 VGLNAgainMode = 0;
	u32 reg_value;
	s32 fld_value;
	u8 agcrfin1, agcrfin0, agc1poweri, agc1powerq,agc1ref, agc2ref, agc2i1, agc2i0,agciqin1, agciqin0/*, exponant=0*/;
	s8 exponant = 0; // RF level fix BZ#107344
	u16 mantisse=0;
	u8 NOSFR, DemodLock;
	u32 agc1=0;
	u32 agciqin;
	u8 agcrfpwm=0;
	s32 correction1000=0;
	u32 f;
	s32 x,y;
	u32 mult;
	struct fe_stid135_internal_param *pParams;
	fe_lla_error_t error=FE_LLA_NO_ERROR;
	pParams = &state->base->ip;

	/* Read all needed registers before calculation*/
	error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TAGC2_AGC2EXP_NOSFR(Demod), &fld_value);
	NOSFR= (u8)fld_value;
	error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDSTATE_DEMOD_LOCKED(Demod), &fld_value);
	DemodLock = (u8)fld_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(Demod), &reg_value);
	agc2i1 = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I0(Demod), &reg_value);
	agc2i0 = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFIN1(TunerNb), &reg_value);
	agcrfin1 = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFIN0(TunerNb), &reg_value);
	agcrfin0 = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2REF(Demod), &reg_value);
	agc2ref = (u8)reg_value;
	ChipWaitOrAbort(state->base->ip.handle_demod,1);
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1REF(Demod), &reg_value);
	agc1ref = (u8)reg_value;
	ChipWaitOrAbort(state->base->ip.handle_demod,1);
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1POWERI(Demod), &reg_value);
	agc1poweri = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1POWERQ(Demod), &reg_value);
	agc1powerq = (u8)reg_value;

	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1IQIN1(Demod), &reg_value);
	agciqin1 = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1IQIN0(Demod), &reg_value);
	agciqin0 = (u8)reg_value;

	/* check I2c error and demod lock for data coherency                */
	/* Bit NOSFR must be =0 and POWER_IREF should not be null  */
	if ((NOSFR==0) && /*(DemodLock==1) &&*/ (error==FE_LLA_NO_ERROR) && (POWER_IREF!=0) ) {
		/*************** calculate AGC2 ********************/
		/*  Agc2= (AGC2I1*4 +AGC2I1) *2^XtoPowerY (exp-9)  */
		/* exp min=5  max=15                               */
		/* so (exp-9) min=-4 max=6                         */
		/***************************************************/
		mantisse = (u16)((agc2i1 * 4) + ((agc2i0 >> 6) & 0x3));
		//exponant = (u8)(agc2i0 & 0x3f);
		/* Fix of BZ#107344
		AGC2 accumulator exponent is signed with a sign bit (bit#5 of AGC2I0 register) */
		if(((agc2i0 & 0x20) >> 5) == 0)
			exponant = (s8)(agc2i0 & 0x3f);
		else if(((agc2i0 & 0x20) >> 5) == 1)
			exponant = (s8)((agc2i0 & 0x1f) - 32);

		/*evaluate exp-9 */
		exp_s32 = (s32)(exponant - 9);
		if (exp_s32<= -32) {
			agc2x1000 = 0;
		}else if(exp_s32<0){
			/* if exp_s32<0 divide the mantissa  by 2^abs(exp_s32)*/
			exp_abs_s32= XtoPowerY(2,(u32)(- exp_s32));
			agc2x1000 = (u32)((1000 * mantisse) / exp_abs_s32);
		} else {
			/*if exp_s32> 0 multiply the mantissa  by 2^(exp_s32)*/
			exp_abs_s32= XtoPowerY(2,(u32)(exp_s32));
			agc2x1000 = (u32)((1000 * mantisse) * exp_abs_s32);
		}
		/******** interpolate Gvana ************/
		/* interpolate Gvana from LUT LutGvana */
		//agc1 = (u32)((256 * agcrfin1) + agcrfin0);
		BUG_ON(agcrfin1<0 || agcrfin1>=sizeof(LutGvanaIntegerTuner)/sizeof(LutGvanaIntegerTuner[0]));
		switch(TunerNb) {
			case AFE_TUNER1 :
				gain_analogx1000 = LutGvanaIntegerTuner[agcrfin1];
				error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC1_RF_PWM, &reg_value);
				agcrfpwm = (u8)reg_value;
			break;
			case AFE_TUNER2 :
				gain_analogx1000 = LutGvanaIntegerTuner[agcrfin1];
				error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC2_RF_PWM, &reg_value);
				agcrfpwm = (u8)reg_value;
			break;
			case AFE_TUNER3 :
				gain_analogx1000 = LutGvanaIntegerTuner[agcrfin1];
				error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC3_RF_PWM, &reg_value);
				agcrfpwm = (u8)reg_value;
			break;
			case AFE_TUNER4 :
			default:
				gain_analogx1000 = LutGvanaIntegerTuner[agcrfin1];
				error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC4_RF_PWM, &reg_value);
				agcrfpwm = (u8)reg_value;
			break;
		}
		/* if VGLNA is inIP3 substract  6db gain*/
		VGLNAgainMode = Oxford_GetVGLNAgainMode(state->base->ip.handle_demod, TunerNb);
		if (VGLNAgainMode != 1){
			/* InterpolatedGvan always >0*/
			gain_analogx1000 = gain_analogx1000 - 6000;
		}
		/************ Power Channel ************/
		/* PchAGC2 = agc2ref^2 */
		/* Pch(dBm) = 10xlog( PchAGC2 / GvDig^2) - GainAnalog */
		/* GvDig= ((agc2/165.8) * (1550/12)); for demod 1,3 */
		/* GvDig= ((agc2/268) * (1550/12));   for demod 2,4,5,6,7,8 */
		/* GvDig^2 = agc2x1000^2 * 607 for demod 1,3 */
		/* GvDig^2 = agc2x1000^2 * 232 for demod 2,4,5,6,7,8 */
		/* log(a/b) = log(a) - log(b) */
		/* 10xlog( PchAGC2 / GvDig^2) =  10xlog( PchAGC2)-10xlog(GvDig^2) */

		if ((Demod == FE_SAT_DEMOD_1) || (Demod == FE_SAT_DEMOD_3))
			mult = 607;
		else
			mult = 232;
#if 0
		x = (s32)STLog10((u32)(2 * agc2ref * agc2ref));
		if (agc2x1000 > 18809) // Avoid (2 x agc2x1000^2 x 607) > Integer size
			y = (s32)STLog10(2 * agc2x1000 * (agc2x1000/1000) * mult) - 6000;
		else if (agc2x1000 > 5947)
			y = (s32)STLog10(2 * agc2x1000 * (agc2x1000/100) * mult) - 7000;
		else if (agc2x1000 > 1880)
			y = (s32)STLog10(2 * agc2x1000 * (agc2x1000/10) * mult) - 8000;
		else
			y = (s32)STLog10(2 * agc2x1000 * agc2x1000 * mult) - 9000 ;
#else
		//The old code was wrong an suffered from overflow errors!
		x = 2*(s32)STLog10((u32)(agc2ref));
		y = (s32)STLog10(mult) + 2*(s32)STLog10(agc2x1000) - 9000;
#endif
		*PchRFx1000 = (s32)(10 * (x - y) - (s32)gain_analogx1000 + 3000); // + 3000 agc2x1000
		/************ Power Band ***************/
		/* PBand =  10log ((AGC1REF/POWERIREF)^2 * AGC1IQIN * 3) - Gvana and log(a/b) = log(a) - log(b)*/
		/* And AGCIQIN * 3 almost = AGCPOWERI^2 + AGCPOWERQ^2 */
		agciqin = (u32)((256 * agciqin1) + agciqin0);
#if 0
		*Pbandx1000 = (s32)(10 * (STLog10((u32)(3 * agciqin)) + STLog10((u32)(agc1ref * agc1ref)) -  STLog10((u32)(POWER_IREF * POWER_IREF))) - gain_analogx1000);
#else
		*Pbandx1000 = (s32)(10 * (STLog10((u32)(3 * agciqin))
															+ 2*STLog10((u32)(agc1ref))
															-  STLog10((u32)(POWER_IREF * POWER_IREF))
															)
												- gain_analogx1000);
#endif
		/************ Correction ***************/
		agc1 = (u32)((256 * agcrfin1) + agcrfin0);
		f = (u32) state->signal_info.frequency;
		correction1000 = (s32)(f/1000  - 1550);
		correction1000 = (s32)(correction1000*1000/600);
		correction1000 = (s32) (correction1000 * correction1000/1000);
		if ((agc1 > 0) && (agcrfpwm >= 128)) {
			correction1000 += correction1000;
		}
		*PchRFx1000 += correction1000;
		*Pbandx1000 += correction1000;
	} else {
		error |=FE_LLA_INVALID_HANDLE;
	}

	if ( error== FE_LLA_NO_ERROR) {
		*AGCRFIN1	= agcrfin1;
		*AGCRFIN0	= agcrfin0;
		*Agc1		= agc1;
		*AGC1POWERI	= agc1poweri;
		*AGC1POWERQ	= agc1powerq;
		*AGC1REF	= agc1ref;
		*AGC2REF	= agc2ref;
		*AGC2I1		= agc2i1;
		*AGC2I0		= agc2i0;
		*mant		= mantisse;
		*exp		= (u8)exponant;
		*Agc2x1000	= agc2x1000;
		*Gvanax1000	= gain_analogx1000;
	}
	return error;
}


/*****************************************************
--FUNCTION	::	FE_STiD135_GetRFLevel
--ACTION	::	Returns the RF level of signal (in the band currently tuned to)
--PARAMS IN	::	handle -> Frontend handle
			Demod -> current demod 1..8
--PARAMS OUT	::	pch_rf -> computed channel power
			pband_rf -> computed band power
--RETURN	::	error
--***************************************************/
fe_lla_error_t FE_STiD135_GetRFLevel(struct stv* state, s32 *pch_rf, s32 *pband_rf)

{
	u8 AGCRFIN1;
	u8 AGCRFIN0;
	u32 Agc1;
	u8 AGC1POWERI;
	u8 AGC1POWERQ;
	u8 AGC1REF;
	u8 AGC2REF;
	u8 AGC2I1;
	u8 AGC2I0;
	u16 mant;
	u8 exp;
	u32 Agc2x1000;
	u32 InterpolatedGvanax1000;
	s32 PchRFx1000 = 0;
	s32 Pbandx1000 = 0;
	s32 agcrf_path;
	//struct fe_stid135_internal_param *pParams = &state->base->ip;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	error |= fe_stid135_get_agcrf_path(state, &agcrf_path );

	error |= Estimate_Power_Int(state, (u8)agcrf_path, &AGCRFIN1, &AGCRFIN0, &Agc1, &AGC1POWERI, &AGC1POWERQ, &AGC1REF, &AGC2REF, &AGC2I1, &AGC2I0, &mant, &exp, &Agc2x1000, &InterpolatedGvanax1000, &PchRFx1000, &Pbandx1000);
	*pch_rf = PchRFx1000;
	*pband_rf = Pbandx1000;

	return(error);
}

/**********************************************************************************
FUNCTION	::	estimate_band_power_demod_not_locked
ACTION		::	Basic function to verify band power estimation (Pband) when
			demod is not locked
PARAMS IN		::	Handle
			Demod [0;7]
			TunerNb[1;4]
			PowerIref should be set to 105
PARAMS OUT	::	*AGCRFIN1,
			*AGCRFIN0,
			*AGC1REF,
			*Pbandx1000
RETURN		::	error
**********************************************************************************/
static fe_lla_error_t estimate_band_power_demod_not_locked(struct stv* state,
					u8 TunerNb,
					u8 *AGCRFIN1,
					u8 *AGCRFIN0,
					u8 *AGC1REF,
					s32 *Pbandx1000)
{
	enum fe_stid135_demod Demod = state->nr+1;
	u32 gain_analogx1000=0;
	u8 VGLNAgainMode = 0;
	u32 reg_value;
	u8 agcrfin1, agcrfin0, agc1ref, agciqin1, agciqin0;
	u32 agciqin;
	u8 savedmd = 0;

	struct fe_stid135_internal_param *pParams;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	pParams = &state->base->ip;
	/*Fig. p. 87
		AGC_RF = input to analogue amplifier -> controls vglna

		BB_AGC = analogue base band gain -> also has chebychef antialiasing filter
		The gain is applied at the input (before chebychef)

		p. 89
		VGLNA gain set by AFE_RF_CFG.VGLNAn_SELGAIN
		VGLNA has two gain/pin curves depending on low/high power signals
		VGLNA_SELGAIN=1 best for low signals

		p. 91 AGC1IQ? AGC_IQRF controls AGC_RF?; target is to get IQ power close to AGCIQ_REF which
		is the amplitude (sqrt of sum of squares). This is controlled by AGC1 which is part of RF digital
		processing, i.e,, it is the wide band digital AGC

		It seems that AGC1IQIN1 and GC1IQIN0 are undocumented registers, which contain the measured  wide band amplitude
		The computed gain can be read from  AGCRFINx

		p. 97: AGC2 is part of the demodulator and aims to set IQ power equal to AGCIQ_REF.
		This gain is read fromm AGC2_INTEGRATOR

	*/
	/* Read all needed registers before calculation*/

	//AGC_RF = computed gain for analogue amplifier
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFIN1(TunerNb), &reg_value);
	agcrfin1 = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFIN0(TunerNb), &reg_value);

	//reference amplitude value for analogue amplifier
	agcrfin0 = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1REF(Demod), &reg_value);
	agc1ref = (u8)reg_value;

	//measured wide band amplititude (power at output of analogue amplifier)
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1IQIN1(Demod), &reg_value);
	agciqin1 = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1IQIN0(Demod), &reg_value);
	agciqin0 = (u8)reg_value;
	BUG_ON(agcrfin1<0 || agcrfin1>=sizeof(LutGvanaIntegerTuner)/sizeof(LutGvanaIntegerTuner[0]));
	switch(TunerNb) {
		case AFE_TUNER1 :
			gain_analogx1000 = LutGvanaIntegerTuner[agcrfin1];
		break;
		case AFE_TUNER2 :
			gain_analogx1000 = LutGvanaIntegerTuner[agcrfin1];
		break;
		case AFE_TUNER3 :
			gain_analogx1000 = LutGvanaIntegerTuner[agcrfin1];
		break;
		case AFE_TUNER4 :
		default:
			gain_analogx1000 = LutGvanaIntegerTuner[agcrfin1];
		break;
	}
	/* if VGLNA is inIP3 substract  6db gain (lowe power versus high power input configuration)*/
	VGLNAgainMode = Oxford_GetVGLNAgainMode(state->base->ip.handle_demod, TunerNb);
	if (VGLNAgainMode != 1){
		/* InterpolatedGvan always >0*/
		gain_analogx1000 = gain_analogx1000 - 6000;
	}


	/* Read DMDISTATE to check if demod is stopped or reset, then put demod in search state for band power accuracy */
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), &reg_value);
	savedmd = (u8)reg_value;
	if ((savedmd == 0x5C) || (savedmd == 0x1C)) {
		dprintk("XXX set to 0\n");
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x00);
		WAIT_N_MS(100);
	}

	//reread AGC1IQINx
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1IQIN1(Demod), &reg_value);
	agciqin1 = (u8)reg_value;
	error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1IQIN0(Demod), &reg_value);
	agciqin0 = (u8)reg_value;
	agciqin = (u32)((256 * agciqin1) + agciqin0);

	//
#if 0
	*Pbandx1000 = (s32)(10 * (
														STLog10((u32)(3 * agciqin)) /*measured input power to digital frontend
																													AGCIQIN * 3 almost = AGCPOWERI^2 + AGCPOWERQ^2
																												*/
														+ STLog10((u32)(agc1ref * agc1ref)) /* POWER_IREF/agc1ref is an extra
																																	 amplification somwhere
																																	 and should be subtracted
																																*/
														-  STLog10((u32)(POWER_IREF * POWER_IREF))
														)
											- gain_analogx1000);
#else
	//avoid overflow!
	*Pbandx1000 = (s32)(10 * (
														STLog10((u32)(3 * agciqin)) /*measured input power to digital frontend
																													AGCIQIN * 3 almost = AGCPOWERI^2 + AGCPOWERQ^2
																												*/
														+ 2*STLog10((u32)(agc1ref)) /* POWER_IREF/agc1ref is an extra
																																	 amplification somwhere
																																	 and should be subtracted
																																*/
														-  STLog10((u32)(POWER_IREF * POWER_IREF))
													)
											- gain_analogx1000);
#endif

	/* Restore value of DMDISTATE */
	if (savedmd != 0) {
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), savedmd);
	}

	if ( error== FE_LLA_NO_ERROR) {
		*AGCRFIN1	= agcrfin1;
		*AGCRFIN0	= agcrfin0;
		*AGC1REF	= agc1ref;
	}
	return error;
}



/*****************************************************
--FUNCTION	::	fe_stid135_get_band_power_demod_not_locked
--ACTION	::	Returns the band pwoer when demod is not locked
--PARAMS IN	::	handle -> Frontend handle
			Demod -> current demod 1..8
--PARAMS OUT	::	pch_rf -> computed channel power
			pband_rf -> computed band power
--RETURN	::	error
--***************************************************/
fe_lla_error_t fe_stid135_get_band_power_demod_not_locked(struct stv* state, s32 *pband_rf)

{
	u8 AGCRFIN1;
	u8 AGCRFIN0;
	u8 AGC1REF;
	s32 Pbandx1000;

	s32 agcrf_path;
	struct fe_stid135_internal_param *pParams;
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	pParams = &state->base->ip;
	error |= fe_stid135_get_agcrf_path(state, &agcrf_path);
	error |= estimate_band_power_demod_not_locked(state, (u8)agcrf_path, &AGCRFIN1, &AGCRFIN0, &AGC1REF, &Pbandx1000);
	*pband_rf = Pbandx1000;

	return(error);
}

fe_lla_error_t estimate_band_power_demod_for_fft(struct stv* state,
																								 u8 TunerNb,
																								 s32 *Pbandx1000, bool* double_correction)
{
	u8 agcrfin1;
	u8 agcrfin0;
	u8 AGC1REF;
	s32 reg_value=0;
	//s32 correction1000=0;
	u8 agcrfpwm=0;
	u32 agc1;

	//struct fe_stid135_internal_param* pParams = &state->base->ip;

	fe_lla_error_t err= estimate_band_power_demod_not_locked(state, TunerNb,
																													 &agcrfin1, &agcrfin0, &AGC1REF, Pbandx1000);


	switch(TunerNb) {
	case AFE_TUNER1 :
		err |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC1_RF_PWM, &reg_value);
		agcrfpwm = (u8)reg_value;
		break;
	case AFE_TUNER2 :
		err |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC2_RF_PWM, &reg_value);
		agcrfpwm = (u8)reg_value;
		break;
	case AFE_TUNER3 :
		err |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC3_RF_PWM, &reg_value);
		agcrfpwm = (u8)reg_value;
		break;
	case AFE_TUNER4 :
	default:
		err |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC4_RF_PWM, &reg_value);
		agcrfpwm = (u8)reg_value;
		break;
	}
	agc1 = (u32)((256 * agcrfin1) + agcrfin0);

	*Pbandx1000 -= 49000; //???
	return err;
}

fe_lla_error_t estimate_band_power_demod_correctfor_fft(struct stv* state,
																								 u8 TunerNb,
																								 s32 *Pbandx1000, s32 frequency)
{
	u8 agcrfin1;
	u8 agcrfin0;
	u8 AGC1REF;
	s32 reg_value=0;
	s32 correction1000=0;
	u8 agcrfpwm=0;
	u32 agc1;

	//struct fe_stid135_internal_param* pParams = &state->base->ip;

	fe_lla_error_t err= estimate_band_power_demod_not_locked(state, TunerNb,
																													 &agcrfin1, &agcrfin0, &AGC1REF, Pbandx1000);


	switch(TunerNb) {
	case AFE_TUNER1 :
		err |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC1_RF_PWM, &reg_value);
		agcrfpwm = (u8)reg_value;
		break;
	case AFE_TUNER2 :
		err |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC2_RF_PWM, &reg_value);
		agcrfpwm = (u8)reg_value;
		break;
	case AFE_TUNER3 :
		err |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC3_RF_PWM, &reg_value);
		agcrfpwm = (u8)reg_value;
		break;
	case AFE_TUNER4 :
	default:
		err |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC4_RF_PWM, &reg_value);
		agcrfpwm = (u8)reg_value;
		break;
	}
	agc1 = (u32)((256 * agcrfin1) + agcrfin0);
	correction1000 = (s32)(frequency/1000  - 1550);
	correction1000 = (s32)(correction1000*1000/600);
	correction1000 = (s32) (correction1000 * correction1000/1000);
	if ((agc1 > 0) && (agcrfpwm >= 128)) {
			correction1000 += correction1000;
		}
	//dprintk("correction1000=%d\n", correction1000);
	*Pbandx1000 += correction1000;
	*Pbandx1000 -= 49000; //???
	return err;
}


fe_lla_error_t fe_stid135_get_signal_quality(struct stv* state,
					struct fe_sat_signal_info *pInfo,
					int mc_auto)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams;
	s32 pch_rf, pband_rf;

	pParams = &state->base->ip;

	error |= FE_STiD135_GetBer(state, &(pInfo->ber));
	if(error)
		dprintk("A demod=%p nr+1=%d", state->base->ip.handle_demod, state->nr+1);
	error |= FE_STiD135_GetRFLevel(state, &pch_rf, &pband_rf);
	if(error)
		dprintk("B demod=%p nr+1=%d", state->base->ip.handle_demod, state->nr+1);
	pInfo->power = pch_rf;
	error |= FE_STiD135_CarrierGetQuality(state->base->ip.handle_demod, state->nr+1, &(pInfo->C_N), &(pInfo->standard));
	if(error)
		dprintk("C demod=%p nr+1=%d", state->base->ip.handle_demod, state->nr+1);
	if (pInfo->standard == FE_SAT_DVBS2_STANDARD) {
		if (mc_auto) {
			error |= fe_stid135_filter_forbidden_modcodes(state, pInfo->C_N * 10);
		}
	if(error)
		dprintk("D demod=%p nr+1=%d", state->base->ip.handle_demod, state->nr+1);
	} else {
	}

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_signal_info
--ACTION	::	Return informations on the locked transponder
--PARAMS IN	::	Handle -> Front End Handle
			Demod -> Current demod 1 .. 8
			satellite_scan -> scan or acquisition context
--PARAMS OUT	::	pInfo -> Informations (BER,C/N,power ...)
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_get_signal_info(struct stv* state,
					struct fe_sat_signal_info *pInfo,
					u32 satellite_scan)
{
	enum fe_stid135_demod Demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams;
	s32 pch_rf, pband_rf;
	s32 fld_value[3];
	//u32 reg_value = 0;
	s32 symbolRateOffset = 0, carrier_frequency = 0;

	pParams = &state->base->ip;
	//	dprintk("signal_info: start\n");
	if ( state->base->ip.handle_demod->Error ) {
				error = FE_LLA_I2C_ERROR;
			 dprintk("here\n");
	} else {
		{
			bool has_carrier, has_sync, has_viterbi;
			error |= fe_stid135_get_lock_status(state, &has_carrier, &has_viterbi, &has_sync);
			dprintk("[%d] Called fe_stid135_get_lock_status: carrier=%d sync=%d\n", state->nr+1, has_carrier, has_sync);
			//pInfo->has_sync = has_sync;
			//pInfo->has_carrier = has_carrier;
		}
		/* transponder_frequency = tuner +  demod carrier
			 frequency */
		pInfo->frequency = pParams->lo_frequency / 1000; //allways a dummy value (1.5Ghz)?
		dprintk("[%d] freq=%d\n", state->nr+1, pInfo->frequency);
		/* On auxiliary demod, frequency found is not true, we have to pick it on master demod  */
		/* On auxiliary demod, SR found is not true, we have to pick it on master demod  */
		if(Demod == FE_SAT_DEMOD_2) {
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(Demod), &(fld_value[0]));
			if(fld_value[0] == 3) {
				error |= FE_STiD135_GetCarrierFrequencyOffset_(state->base->ip.handle_demod, FE_SAT_DEMOD_1, pParams->master_clock, &carrier_frequency);
				carrier_frequency /= 1000;
				error |= FE_STiD135_GetSymbolRate_(state->base->ip.handle_demod, FE_SAT_DEMOD_1, pParams->master_clock, &(pInfo->symbol_rate));
				error |= FE_STiD135_TimingGetOffset_(state->base->ip.handle_demod, FE_SAT_DEMOD_1, pInfo->symbol_rate,  &symbolRateOffset);
			}
			}
			if(Demod == FE_SAT_DEMOD_4) {
				error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(Demod), &(fld_value[0]));
				if(fld_value[0] == 3) {
					error |= FE_STiD135_GetCarrierFrequencyOffset_(state->base->ip.handle_demod, FE_SAT_DEMOD_3, pParams->master_clock,  &carrier_frequency);
					carrier_frequency /= 1000;
					error |= FE_STiD135_GetSymbolRate_(state->base->ip.handle_demod, FE_SAT_DEMOD_3, pParams->master_clock, &(pInfo->symbol_rate));
					error |= FE_STiD135_TimingGetOffset_(state->base->ip.handle_demod, FE_SAT_DEMOD_3, pInfo->symbol_rate, &symbolRateOffset);
				}
			}
			if(((Demod != FE_SAT_DEMOD_2) && (Demod != FE_SAT_DEMOD_4)) ||
					((Demod == FE_SAT_DEMOD_2) && (fld_value[0] != 3)) ||
					((Demod == FE_SAT_DEMOD_4) && (fld_value[0] != 3))) {
				error |= FE_STiD135_GetCarrierFrequencyOffset(state, pParams->master_clock, &carrier_frequency);
				carrier_frequency /= 1000;
				error |= FE_STiD135_GetSymbolRate(state, pParams->master_clock, &(pInfo->symbol_rate));
				error |= FE_STiD135_TimingGetOffset(state, pInfo->symbol_rate, &symbolRateOffset);
			}

			if(carrier_frequency < 0) {
				carrier_frequency *= (-1);
				pInfo->frequency -= (u32)carrier_frequency;
			}
			else
				pInfo->frequency += (u32)carrier_frequency;


			/* Get timing loop offset */
			if(symbolRateOffset < 0) {
				symbolRateOffset *= (-1);
				pInfo->symbol_rate -= (u32)symbolRateOffset;
			}
			else
				pInfo->symbol_rate += (u32)symbolRateOffset;

			error |= FE_STiD135_GetStandard(
				state->base->ip.handle_demod, Demod, &(pInfo->standard));

			 error |= FE_STiD135_GetViterbiPunctureRate(state, &(pInfo->puncture_rate));

			error |= fe_stid135_get_mode_code(state,
						&pInfo->modcode,
						&pInfo->frame_length,
						&pInfo->pilots);
			vprintk("[%d] GOT MODCODE %d\n", state->nr+1, pInfo->modcode);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TMGOBS_ROLLOFF_STATUS(Demod), &(fld_value[0]));
			pInfo->roll_off = (enum fe_sat_rolloff)(fld_value[0]);

			if (satellite_scan == FALSE) {
				error |= FE_STiD135_GetBer(state, &(pInfo->ber));
				error |= FE_STiD135_GetRFLevel(state, &pch_rf, &pband_rf);
				pInfo->power = pch_rf;
				pInfo->powerdBmx10 = pch_rf * 10;
				pInfo->band_power = pband_rf;
				error |= FE_STiD135_CarrierGetQuality(state->base->ip.handle_demod, Demod, &(pInfo->C_N), &(pInfo->standard));
			} else { /* no BER, Power and CNR measurement during scan */
				pInfo->ber = 0;
				pInfo->power = 0;
				pInfo->C_N = 100;
			}

			vprintk("[%d] here 3", state->nr+1);
			if(pInfo->standard == FE_SAT_DVBS2_STANDARD) {
				vprintk("[%d] here 4", state->nr+1);
				error |= ChipGetField(state->base->ip.handle_demod,FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS6_SPECINV_DEMOD(Demod), &(fld_value[0]));
				pInfo->spectrum = (enum fe_sat_iq_inversion)(fld_value[0]);

				if (((pInfo->modcode >= FE_SAT_QPSK_14) && (pInfo->modcode <= FE_SAT_QPSK_910))
					|| ((pInfo->modcode >= FE_SAT_DVBS1_QPSK_12) && (pInfo->modcode <= FE_SAT_DVBS1_QPSK_78))
					|| ((pInfo->modcode >= FE_SATX_QPSK_13_45) && (pInfo->modcode <= FE_SATX_QPSK_11_20))
					|| ((pInfo->modcode >= FE_SATX_QPSK_11_45) && (pInfo->modcode <= FE_SATX_QPSK_32_45)))
					pInfo->modulation = FE_SAT_MOD_QPSK;

				else if (((pInfo->modcode >= FE_SAT_8PSK_35) && (pInfo->modcode <= FE_SAT_8PSK_910))
					|| ((pInfo->modcode >= FE_SATX_8PSK_23_36) && (pInfo->modcode <= FE_SATX_8PSK_13_18))
					|| ((pInfo->modcode >= FE_SATX_8PSK_7_15) && (pInfo->modcode <= FE_SATX_8PSK_32_45))
					|| (pInfo->modcode == FE_SATX_8PSK))
					pInfo->modulation=FE_SAT_MOD_8PSK;

				else if (((pInfo->modcode >= FE_SAT_16APSK_23) && (pInfo->modcode <= FE_SAT_16APSK_910))
					|| ((pInfo->modcode >= FE_SATX_16APSK_26_45) && (pInfo->modcode <= FE_SATX_16APSK_23_36))
					|| ((pInfo->modcode >= FE_SATX_16APSK_25_36) && (pInfo->modcode <= FE_SATX_16APSK_77_90))
					|| ((pInfo->modcode >= FE_SATX_16APSK_7_15) && (pInfo->modcode <= FE_SATX_16APSK_32_45))
					|| (pInfo->modcode == FE_SATX_16APSK))
					pInfo->modulation=FE_SAT_MOD_16APSK;

				else if (((pInfo->modcode >= FE_SAT_32APSK_34) && (pInfo->modcode <= FE_SAT_32APSK_910))
					|| ((pInfo->modcode >= FE_SATX_32APSK_R_58) && (pInfo->modcode <= FE_SATX_32APSK_7_9))
					|| ((pInfo->modcode >= FE_SATX_32APSK_2_3) && (pInfo->modcode <= FE_SATX_32APSK_32_45_S))
					|| (pInfo->modcode == FE_SATX_32APSK))
					pInfo->modulation = FE_SAT_MOD_32APSK;

				else if ((pInfo->modcode == FE_SATX_VLSNR1) || (pInfo->modcode == FE_SATX_VLSNR2))
					pInfo->modulation = FE_SAT_MOD_VLSNR;

				else if (((pInfo->modcode >= FE_SATX_64APSK_11_15) && (pInfo->modcode <= FE_SATX_64APSK_5_6))
					|| (pInfo->modcode == FE_SATX_64APSK))
					pInfo->modulation = FE_SAT_MOD_64APSK;

				else if ((pInfo->modcode == FE_SATX_128APSK_3_4) || (pInfo->modcode == FE_SATX_128APSK_7_9))
					pInfo->modulation = FE_SAT_MOD_128APSK;

				else if ((pInfo->modcode == FE_SATX_256APSK_32_45) || (pInfo->modcode == FE_SATX_256APSK_3_4)
					|| (pInfo->modcode == FE_SATX_256APSK))
					pInfo->modulation = FE_SAT_MOD_256APSK;

				else if ((pInfo->modcode == FE_SATX_8APSK_5_9_L) || (pInfo->modcode == FE_SATX_8APSK_26_45_L))
					pInfo->modulation = FE_SAT_MOD_8PSK_L;

				else if (((pInfo->modcode >= FE_SATX_16APSK_1_2_L) && (pInfo->modcode <= FE_SATX_16APSK_5_9_L))
					|| (pInfo->modcode == FE_SATX_16APSK_3_5_L)
					|| (pInfo->modcode == FE_SATX_16APSK_2_3_L))
					pInfo->modulation = FE_SAT_MOD_16APSK_L;

				else if (pInfo->modcode == FE_SATX_32APSK_2_3_L)
					pInfo->modulation = FE_SAT_MOD_32APSK_L;

				else if (pInfo->modcode == FE_SATX_64APSK_32_45_L)
					pInfo->modulation = FE_SAT_MOD_64APSK_L;

				else if (((pInfo->modcode >= FE_SATX_256APSK_29_45_L) && (pInfo->modcode <= FE_SATX_256APSK_31_45_L))
					|| (pInfo->modcode == FE_SATX_256APSK_11_15_L))
					pInfo->modulation = FE_SAT_MOD_256APSK_L;

				else if (pInfo->modcode == FE_SATX_1024APSK)
					pInfo->modulation = FE_SAT_MOD_1024APSK;

				else
					pInfo->modulation = FE_SAT_MOD_UNKNOWN;

				/*reset the error counter to PER*/
				error |= ChipSetOneRegister(state->base->ip.handle_demod,
																		(u16)REG_RC8CODEW_DVBSX_HWARE_ERRCTRL1(Demod), 0x67);
				vprintk("[%d] MIS: mis_mode=%d\n", state->nr+1, state->mis_mode);
				if(state->mis_mode /* &&
					 (pParams->demod_search_algo[Demod-1] == FE_SAT_BLIND_SEARCH ||
					 pParams->demod_search_algo[Demod-1] == FE_SAT_NEXT)*/) {
					fe_lla_error_t error1 = FE_LLA_NO_ERROR;
					error1 =fe_stid135_isi_scan(state, &pInfo->isi_list);
					dprintk("MIS DETECTION: error=%d\n", error1);
				} else {
					u8 isi_read;
					fe_stid135_read_hw_matype(state, &pInfo->matype, &isi_read);
				}

			} else { /*DVBS1/DSS*/
				vprintk("[%d] DVBS1\n", state->nr+1);
				error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_VITERBI_FECM_IQINV(Demod), &(fld_value[0]));
				pInfo->spectrum = (enum fe_sat_iq_inversion)(fld_value[0]);
				pInfo->modulation = FE_SAT_MOD_QPSK;
				pInfo->matype = 0;
			}

			if (state->base->ip.handle_demod->Error)
				error |= FE_LLA_I2C_ERROR;

		}


	return error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_init
--ACTION	::	Initialisation of the Demod chip
--PARAMS IN	::	pInit -> Front End Init parameters
--PARAMS OUT	::	handle -> Handle to circuit
--RETURN	::	error
--***************************************************/
fe_lla_error_t fe_stid135_init(struct fe_sat_init_params *pInit,
															 struct fe_stid135_internal_param* pParams)
{
	char chr_tmp[50] = "";
	u8 i;

	/* Demodulator chip initialisation parameters */
	Demod_InitParams_t DemodInitParams;

	/* SOC chip initialisation parameters */
	Demod_InitParams_t SocInitParams;

	fe_lla_error_t error = FE_LLA_NO_ERROR;
	fe_lla_error_t flex_clk_gen_error = FE_LLA_NO_ERROR;

	STCHIP_Error_t demod_init_error = CHIPERR_NO_ERROR;
	STCHIP_Error_t SocError = CHIPERR_NO_ERROR;
	TUNER_Error_t TunerError = TUNER_NO_ERR;
	STCHIP_Info_t SocChip;

	/* Internal params structure allocation */

	STCHIP_Info_t DemodChip;

	//dprintk("here\n");
		/* Chip initialisation */

		/* Demodulator Init */
		#ifdef CHIP_STAPI
			DemodInitParams.Chip = (pParams->handle_demod);
		#else
			DemodInitParams.Chip = &DemodChip;
		#endif

		DemodInitParams.Chip->pI2CHost = pInit->pI2CHost;
		DemodInitParams.NbDefVal = STiD135_NBREGS;
		DemodInitParams.Chip->RepeaterHost = NULL;
		DemodInitParams.Chip->RepeaterFn = NULL;
		DemodInitParams.Chip->Repeater = FALSE;
		DemodInitParams.Chip->I2cAddr = pInit->demod_i2c_adr;
		DemodInitParams.Chip->Error = CHIPERR_NO_ERROR;
		strcpy((char *)DemodInitParams.Chip->Name,
			pInit->demod_name);
		demod_init_error = STiD135_Init(&DemodInitParams,
					&(pParams->handle_demod));
		/* Frequency clock XTAL */
		pParams->quartz = pInit->demod_ref_clk;
		/* Board depending datas */
		pParams->internal_dcdc = pInit->internal_dcdc;
		pParams->internal_ldo = pInit->internal_ldo;
		pParams->rf_input_type = pInit->rf_input_type;

		//DANGER: &(pParams->handle_demod) or (pParams->handle_demod)
		//	error |= fe_stid135_apply_custom_qef_for_modcod_filter(pParams, NULL);
		//fe_stid135_modcod_flt_reg_init(); XXX now done in module init

		TunerError = FE_STiD135_TunerInit(pParams);

		SocInitParams.Chip = &SocChip;
		SocInitParams.Chip->pI2CHost = pInit->pI2CHost;
		SocInitParams.Chip->RepeaterHost = NULL;
		SocInitParams.Chip->RepeaterFn   = NULL;
		SocInitParams.Chip->Repeater     = FALSE;
		SocInitParams.Chip->I2cAddr      = pInit->demod_i2c_adr;
		strcat(chr_tmp, pInit->demod_name);
		strcat(chr_tmp, "_SOC");
		strcpy((char *)SocInitParams.Chip->Name, chr_tmp);
		SocError = STiD135_SOC_Init(&SocInitParams, &(pParams->handle_soc));

		/* Adjust Vcore voltage regarding process (AVS) */
		error |= fe_stid135_set_vcore_supply(pParams);

		/* Initialization of Flex Clock Gen SOC block */
		flex_clk_gen_error = fe_stid135_flexclkgen_init(pParams);

		if(pParams->handle_demod != NULL) {
			if((demod_init_error == CHIPERR_NO_ERROR) && (TunerError == TUNER_NO_ERR) && (SocError == CHIPERR_NO_ERROR) && (flex_clk_gen_error == FE_LLA_NO_ERROR))
			{
#if 0 //not in main driver
				/* settings for STTUNER or non-Gui applications
				or Auto test*/
				pParams->quartz = pInit->demod_ref_clk;
				/* Ext clock in Hz */
				error =(fe_lla_error_t)ChipSetField(pParams->handle_demod,
					FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(PATH1),
					pInit->tuner_iq_inversion);
				error =(fe_lla_error_t)ChipSetField(pParams->handle_demod,
					FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(PATH2),
					pInit->tuner_iq_inversion);
				error =(fe_lla_error_t)ChipSetField(pParams->handle_demod,
					FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(PATH3),
					pInit->tuner_iq_inversion);
				/* switch to the PLL */
#endif

				/*Read the current Mclk*/
				error |= FE_STiD135_GetMclkFreq(pParams, &(pParams->master_clock));
				dprintk("MASTER CLOCK freq==%d\n", pParams->master_clock);
				if(pParams->master_clock == 0)
					return FE_LLA_INVALID_HANDLE;

				/* BB filter calibration on all RF channels */
				for(i=AFE_TUNER1;i<=AFE_TUNER4;i++) {
					error |= fe_stid135_set_rfmux_path_(pParams->handle_demod,
																							FE_SAT_DEMOD_1, (FE_OXFORD_TunerPath_t)i);
					error |= fe_stid135_init_before_bb_flt_calib_(pParams, FE_SAT_DEMOD_1, (FE_OXFORD_TunerPath_t)i, TRUE);
					error |= fe_stid135_bb_flt_calib_(pParams, FE_SAT_DEMOD_1, (FE_OXFORD_TunerPath_t)i);
					error |= fe_stid135_uninit_after_bb_flt_calib_(pParams, FE_SAT_DEMOD_1,
																												 (FE_OXFORD_TunerPath_t)i);
					error |= Oxford_TunerDisable(pParams->handle_demod, (FE_OXFORD_TunerPath_t)i);
				}
				error |= fe_stid135_set_rfmux_path_(pParams->handle_demod, FE_SAT_DEMOD_1, AFE_TUNER1);

				/* Check the error at the end of the init */
				if ((pParams->handle_demod->Error) || (pParams->handle_soc->Error))
					error = FE_LLA_I2C_ERROR;

			} else
				error = FE_LLA_I2C_ERROR;
		} else {
			error = FE_LLA_INVALID_HANDLE;
		}

	return error;
}




/*****************************************************
--FUNCTION	::	fe_stid135_manage_LNF_IP3
--ACTION	::	Select LNF or IP3 mode
--PARAMS IN	::	demod_handle -> Frontend handle
			tuner_nb -> tuner path to configure
									agc1Power -> Power at AGC1
--PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
static fe_lla_error_t fe_stid135_manage_LNF_IP3 (STCHIP_Info_t* demod_handle, FE_OXFORD_TunerPath_t tuner_nb, u32 agc1Power)
{
	STCHIP_Error_t error = CHIPERR_NO_ERROR;
				u8 modeLNF=0;

	modeLNF = Oxford_GetVGLNAgainMode(demod_handle, tuner_nb);

	if (modeLNF) {
		if (agc1Power > LNF_IP3_SWITCH_HIGH) {
			error |= Oxford_SetVGLNAgainMode(demod_handle, tuner_nb, MODE_IP3);
								}
				} else	if (agc1Power < LNF_IP3_SWITCH_LOW) {
								error |= Oxford_SetVGLNAgainMode(demod_handle, tuner_nb, MODE_LNF);
	}
				return (fe_lla_error_t)error;
}


/*****************************************************
--FUNCTION	::	FE_STiD135_Algo
--ACTION	::	Start a search for a valid DVBS1/DVBS2 or DSS
			transponder
--PARAMS IN	::	pParams -> Pointer to fe_stid135_internal_param structure
			Demod -> current demod 1 .. 8
			satellite_scan -> scan scope
--PARAMS OUT	::	signalType_p -> result of algo computation
--RETURN	::	error
--***************************************************/
fe_lla_error_t FE_STiD135_Algo(struct stv* state, BOOL satellite_scan, enum fe_sat_signal_type *signalType_p)
{
	enum fe_stid135_demod Demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;
	u32 demodTimeout = 4500;
	u32 fecTimeout = 450;
	s32	iqPower,
		agc1Power,
		i,
		fld_value;

	u32 streamMergerField;
	u16 pdel_status_timeout = 0;
	s32 AgcrfPath;

	BOOL lock = FALSE;

	s32 lockstatus;

	*signalType_p = FE_SAT_NOCARRIER;
	/*release reset DVBS2 packet delin*/
	error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod), 0));
	if(error1)
		dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

	streamMergerField = FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_RST_HWARE(Demod);

	/* Stop the stream merger before stopping the demod */
	/* stream merger Stop*/
	vprintk("[%d] XXX set to 0x5c\n", state->nr+1);
	error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x5C));
		if(error1)
			dprintk("[%d] ERROR=%d\n", state->nr+1, error1);


	/*Get the demod and FEC timeout recommended value depending on the
	symbol rate and the search algo*/
	FE_STiD135_GetLockTimeout(&demodTimeout, &fecTimeout,
														state->demod_symbol_rate, state->demod_search_algo);
	vprintk("[%d] FE_STiD135_GetLockTimeout: demodTimeout=%d fecTimeout=%d symrate=%d algo=%d\n",
					state->nr+1,
					demodTimeout, fecTimeout, state->demod_symbol_rate, state->demod_search_algo
					);
#if 1
#ifdef USER2
	demodTimeout = demodTimeout * 16; // Low CNR
#else
	demodTimeout = demodTimeout * 4;
#endif
	fecTimeout   = fecTimeout   * 4;
#endif

	error |= (error1=fe_stid135_set_symbol_rate(state, state->demod_symbol_rate));
	if(error1)
		dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

	error |= (error1=fe_stid135_set_carrier_frequency_init(state,
																												 state->tuner_frequency));
#if 0 //not in main driver
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALS2_ROLLOFF(Demod), 0);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALSX_ROLLOFF(Demod), 0);
#endif
		/* NO signal Detection */
		/* Read PowerI and PowerQ To check the signal Presence */

		error |= (error1=fe_stid135_get_agcrf_path(state, &AgcrfPath));
	if(error1)
		dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

	ChipWaitOrAbort(state->base->ip.handle_demod, 10);

	error |= (error1=ChipGetRegisters(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFIN1(AgcrfPath), 2));
	if(error1)
		dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

	agc1Power=MAKEWORD16(ChipGetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_AGCRF_AGCRFIN1_AGCRF_VALUE(AgcrfPath)),
		ChipGetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_AGCRF_AGCRFIN0_AGCRF_VALUE(AgcrfPath)));

	error |= (error1=fe_stid135_manage_LNF_IP3(state->base->ip.handle_demod, AgcrfPath, (u32)agc1Power));
	if(error1)
		dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

	iqPower = 0;

	if (agc1Power == 0) {
		/* if AGC1 integrator value is 0 then read POWERI,
		POWERQ registers */
		/* Read the IQ power value */
		for (i = 0; i < 5; i++) {
			error |= (error1=ChipGetRegisters(state->base->ip.handle_demod,
																				(u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRF_POWERI(AgcrfPath), 2));
			if(error1)
				dprintk("[%d] ERROR=%d i=%d\n", state->nr+1, error1, i);

			iqPower = iqPower + (ChipGetFieldImage(
			state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_AGCRF_AGCRF_POWERI_AGCRF_POWER_I(AgcrfPath))
					+  ChipGetFieldImage(
			state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_AGCRF_AGCRF_POWERQ_AGCRF_POWER_Q(AgcrfPath))) / 2;
		}
		iqPower /= 5;
	}


#if 0
		// In lab conditions (1 channel), for sensitivity autotest, we have to avoid
		// this condition otherwise we are not able to lock on low RF levels
		// We keep this if statement on embedded side
		if ((agc1Power == 0) && (iqPower < powerThreshold)) {
			/*If (AGC1=0 and iqPower<IQThreshold)  then no signal  */
			state->signal_info.locked = FALSE;
			state->signal_info.has_carrier = FALSE;
			state->signal_info.has_signal = FALSE;
			state->signal_info.has_sync = FALSE;
			state->signal_info.has_viterbi = FALSE;
			*signalType_p = FE_SAT_TUNER_NOSIGNAL ;
		} else
#endif
			vprintk("[%d] XXX blind=%d satellite_scan=%d\n",
							state->nr+1,
							state->demod_search_algo == FE_SAT_BLIND_SEARCH,
							satellite_scan);
		if ((state->demod_search_algo == FE_SAT_BLIND_SEARCH ||
				 state->demod_search_algo == FE_SAT_NEXT)
				&& (satellite_scan == TRUE)) {
			iqPower = 0;
			agc1Power = 0;
			state->signal_info.has_lock=false; /*if AGC1 integrator ==0
																						 and iqPower < Threshold then NO signal*/
			state->signal_info.has_carrier = false; /*if AGC1 integrator ==0
																								and iqPower < Threshold then NO signal*/
			if(state->signal_info.has_viterbi)
				dprintk("[%d] !!!!!  set has_carrier\n", state->nr+1);


			/* No edge detected, jump tuner bandwidth */
			*signalType_p = FE_SAT_TUNER_NOSIGNAL;

		}  else { /* falling edge detected or direct blind to be done */
			/* Set the IQ inversion search mode */

			error |= (error1=ChipSetFieldImage(state->base->ip.handle_demod,
																				 FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_SPECINV_CONTROL(Demod),
																				 FE_SAT_IQ_AUTO));   /* Set IQ inversion always in auto */
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

			error |= (error1=ChipSetRegisters(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEMOD(Demod), 1));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

			error |= (error1=FE_STiD135_SetSearchStandard(state));
#if 1
			/* Set ISI before search */
			vprintk("[%d] STREAM_ID=%d\n", state->nr+1,  state->demod_search_stream_id);
			set_stream_index(state, state->demod_search_stream_id);
#endif
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

		if (state->demod_search_algo != FE_SAT_BLIND_SEARCH
				&& state->demod_search_algo != FE_SAT_NEXT)
			error |= FE_STiD135_StartSearch(state);

		if (state->demod_search_algo == FE_SAT_BLIND_SEARCH ||
				state->demod_search_algo == FE_SAT_NEXT) {
			vprintk("[%d] calling FE_STiD135_BlindSearchAlgo: demodTimeout=%d\n",  state->nr+1, demodTimeout);
			error |= (error1=FE_STiD135_BlindSearchAlgo(state, demodTimeout,
																									satellite_scan, &lock));
			vprintk("[%d] end call FE_STiD135_BlindSearchAlgo: lock=%d\n",  state->nr+1, lock);
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

			vprintk("[%d] BLIND SEARCH: timeout=%d lock=%d\n", state->nr+1, demodTimeout, lock);
		} else {
			/* case warm or cold start wait for demod lock */
			error |= (error1=FE_STiD135_GetDemodLock(state, demodTimeout, &lock));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);
			vprintk("[%d] called FE_STiD135_GetDemodLock: lock=%d\n", 	state->nr+1, lock);
		}

		if (lock == TRUE) { /*lock = state->demod_lock: demod has seen dvbs1/2 but packet delin
													lock (DVBS2) or viterbi status lock (DVBS1)  has not been determined yet
												*/
			/* Read signal caracteristics and check the lock
				 range; this does some redundant register reads (registers have been read in getdemodlock)
				 finds: current standard, carrier frequency offset, symbol_rate, symbol_rate_offset, puncture rate,
				 modcode, rolloff, modulation, spectral inversion

*/
			error |= (error1=FE_STiD135_GetSignalParams(state,
																									satellite_scan, signalType_p));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);
			vprintk("[%d] here standard=%d\n", state->nr+1, state->signal_info.standard);
			/* Manage Matype Information if DVBS2 signal */
			if (state->signal_info.standard == FE_SAT_DVBS2_STANDARD) {
				/* Before reading MATYPE value, we need to wait for packet delin locked */
#if 0
				//stv09x1 checks bit 1 (PKTDELIN_LOCK)  whereas this code checks bit 0 ( FIRST_LOCK)
				error |= (error1=ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_FIRST_LOCK(Demod), &fld_value));
#else
				//stv09x1 checks bit 1 (PKTDELIN_LOCK)  whereas this code checks bit 0 ( FIRST_LOCK)
				error |= (error1=ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_LOCK(Demod), &fld_value));
#endif
				if(error1)
					dprintk("[%d] ERROR=%d\n", state->nr+1, error1);
				vprintk("[%d] here first_lock=%d\n", state->nr+1, fld_value);

				while ((fld_value != TRUE) && (pdel_status_timeout < 220)) {
					ChipWaitOrAbort(state->base->ip.handle_demod, 5);
					pdel_status_timeout = (u8)(pdel_status_timeout + 5);
#if 0
					//stv09x1 checks bit 1 (PKTDELIN_LOCK)  whereas this code checks bit 0 ( FIRST_LOCK)
					error |= (error1=ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_FIRST_LOCK(Demod), &fld_value));
#else
					//stv09x1 checks bit 1 (PKTDELIN_LOCK)  whereas this code checks bit 0 ( FIRST_LOCK)
					error |= (error1=ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_LOCK(Demod), &fld_value));
#endif
					if(error1)
						dprintk("[%d] ERROR=%d\n", state->nr+1, error1);
				}
				state->signal_info.has_viterbi = fld_value;
				if(fld_value) {
					state->signal_info.fec_locked=1;
				}
				vprintk("[%d] here2 first_lock=%d fec_locked=%d pdel_status_timeout=%d\n", state->nr+1, fld_value, state->signal_info.fec_locked,
								pdel_status_timeout);
				//at this point the packet delineator is locked
				//state->signal_info.has_viterbi = true;
				if (pdel_status_timeout < 220) {
					// Either we deal with common data: TS-MPEG, GSE, and so on...
#if 1
					error |= (error1=fe_stid135_manage_matype_info(state));
					if(error1)
						dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

					// ...or we deal with raw BB frames
#else
					error |= fe_stid135_manage_matype_info_raw_bbframe(pParams, Demod);
#endif
				} else
					return(FE_LLA_SEARCH_FAILED);
			}
		}
		}

		//TODO: in stv091x getsignalparameters and tracking optimisation are done as part of read_status

		if ((lock == TRUE) && (*signalType_p==FE_SAT_RANGEOK)) {
			BOOL fec_lock;
		/*The tracking optimization and the FEC lock check are
		perfomed only if:
			demod is locked and signal type is RANGEOK i.e
			a TP found  within the given search range
		*/

		// - new management of NCR
		/* Release stream merger reset */
			error |= (error1=ChipSetField(state->base->ip.handle_demod, streamMergerField, 0));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);
			ChipWaitOrAbort(state->base->ip.handle_demod, 3);
			/* Stream merger reset */
			error |= (error1=ChipSetField(state->base->ip.handle_demod, streamMergerField, 1));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

			/* Release stream merger reset */
			error |= (error1=ChipSetField(state->base->ip.handle_demod, streamMergerField, 0));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

			// - end of new management of NCR
			//TODO: at this point we should already have achieved fec lock, so why check again?
		error |= (error1=FE_STiD135_WaitForLock(state, demodTimeout, fecTimeout, satellite_scan, &lockstatus, &fec_lock));
		if(error1)
			dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

		state->signal_info.fec_locked = fec_lock;
		if (lockstatus == TRUE) {
			lock = TRUE;

			state->signal_info.has_lock = TRUE;

			if (state->signal_info.standard == FE_SAT_DVBS2_STANDARD) {
				/*reset DVBS2 packet delinator error
				counter */
				error |= (error1=ChipSetFieldImage(state->base->ip.handle_demod,
																					 FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL2_RESET_UPKO_COUNT(Demod), 1));
				if(error1)
					dprintk("[%d] ERROR=%d\n", state->nr+1, error1);
				error |= (error1=ChipSetRegisters(state->base->ip.handle_demod,
																					(u16)REG_RC8CODEW_DVBSX_PKTDELIN_PDELCTRL2(Demod), 1));
				if(error1)
					dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

				/*reset DVBS2 packet delinator error
				counter */
				error |= (error1=ChipSetFieldImage(state->base->ip.handle_demod,
																					 FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL2_RESET_UPKO_COUNT(Demod), 0));
				if(error1)
					dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

				error |= (error1=ChipSetRegisters(state->base->ip.handle_demod,
																					(u16)REG_RC8CODEW_DVBSX_PKTDELIN_PDELCTRL2(Demod), 1));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

				/* reset the error counter to  DVBS2
				packet error rate */
			error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod,
																					(u16)REG_RC8CODEW_DVBSX_HWARE_ERRCTRL1(Demod), 0x67));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

			} else {
				/* reset the viterbi bit error rate */
				error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod,
																						(u16)REG_RC8CODEW_DVBSX_HWARE_ERRCTRL1(Demod), 0x75));
			}
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

			/*Reset the Total packet counter */
			error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod,
																					(u16)REG_RC8CODEW_DVBSX_FECSPY_FBERCPT4(Demod), 0));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

			/*Reset the packet Error counter2 (and Set it to
			infinit error count mode )*/
			error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod,
																					(u16)REG_RC8CODEW_DVBSX_HWARE_ERRCTRL2(Demod), 0xc1));
			if(error1)
				dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

			if(state->demod_symbol_rate >= state->base->ip.master_clock) {
				// Need to perform a switch of latency regulation OFF/ON to make work WB NCR tests (low datarate)
				WAIT_N_MS(200);
				error |= (error1=ChipGetField(state->base->ip.handle_demod,
																			FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS3_PCRCALC_NCRREADY(Demod), &fld_value));
				if(error1)
					dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

				if(fld_value != 0) {
					WAIT_N_MS(100);
					error |= (error1=ChipGetField(state->base->ip.handle_demod,
																				FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS3_PCRCALC_ERROR(Demod), &fld_value));
					if(error1)
						dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

					if(fld_value == 1) {
						error |= (error1=ChipSetField(state->base->ip.handle_demod,
																					FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(Demod), 1));
						if(error1)
							dprintk("[%d] ERROR=%d\n", state->nr+1, error1);

						WAIT_N_MS(10);
						error |= (error1=ChipSetField(state->base->ip.handle_demod,
																					FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(Demod), 0));
						if(error1)
							dprintk("[%d] ERROR=%d\n", state->nr+1, error1);
					}
				}
			}
		} else {
			lock = FALSE;
			/*if the demod is locked and not the FEC signal
			type is no DATA*/
			*signalType_p = FE_SAT_NODATA;
			state->signal_info.has_lock = FALSE;
		}

		}
		vprintk("[%d] ALGO t exit: lock=%d\n", state->nr+1, lock);
	return error;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_SetSearchStandard
--ACTION	::	Set the Search standard (Auto, DVBS2 only or
			DVBS1/DSS only)
--PARAMS IN	::	pParams -> Pointer to fe_stid135_internal_param structure
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	error
--***************************************************/
fe_lla_error_t FE_STiD135_SetSearchStandard(struct stv* state)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	enum fe_stid135_demod Demod = state->nr+1;

	switch (state->demod_search_standard) {
	case FE_SAT_SEARCH_DVBS1:
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS1_ENABLE(Demod), 1);
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS2_ENABLE(Demod), 0);

		error |= ChipSetRegisters(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 1);

		error |= FE_STiD135_SetViterbiStandard(state->base->ip.handle_demod,
			state->demod_search_standard,
			state->demod_puncture_rate, Demod);

		error |= ChipSetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_HWARE_SFDLYSET2(Demod), 0); /* Enable Super FEC */

	break;

	case FE_SAT_SEARCH_DSS:
		/*If DVBS1/DSS only disable DVBS2 search*/
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS1_ENABLE(Demod), 1);
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS2_ENABLE(Demod), 0);
		error |= ChipSetRegisters(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 1);

		error |= FE_STiD135_SetViterbiStandard(state->base->ip.handle_demod,
				state->demod_search_standard,
				state->demod_puncture_rate, Demod);

		/* Stop Super FEC */
		 error |= ChipSetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_HWARE_SFDLYSET2(Demod), 2);

	break;

	case FE_SAT_SEARCH_DVBS2:
		/*If DVBS2 only activate the DVBS2 search and stop
		the VITERBI*/
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS1_ENABLE(Demod), 0);
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS2_ENABLE(Demod), 1);

		error |= ChipSetRegisters(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 1);
	break;

	case FE_SAT_AUTO_SEARCH:
		/*If automatic enable both DVBS1/DSS and DVBS2 search*/
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS1_ENABLE(Demod), 1);
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS2_ENABLE(Demod), 1);

#if 1 //XXX different
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
															 FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_CFR_AUTOSCAN(Demod), 1);

		error |= ChipSetFieldImage(state->base->ip.handle_demod,
															 FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_SCAN_ENABLE(Demod), 1);
#endif
		error |= ChipSetRegisters(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 1);

		error |= FE_STiD135_SetViterbiStandard(state->base->ip.handle_demod,
				state->demod_search_standard,
				state->demod_puncture_rate,Demod);
#if 0 // XXX different
		/* Enable Super FEC */
		error |= ChipSetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_HWARE_SFDLYSET2(Demod), 0);
#endif
	break;

	case FE_SAT_SEARCH_TURBOCODE:
	break;
	}
	return error;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_StartSearch
--ACTION	::	Programs the PEA to lauch a hardware search algo.
--PARAMS IN	::	pParams -> Pointer to fe_stid135_internal_param structure
			Demod -> demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	error
--***************************************************/
static fe_lla_error_t FE_STiD135_StartSearch(struct stv* state)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	enum fe_stid135_demod Demod = state->nr+1;
	/* Reset the Demod */
	if(state->demod_search_algo!= FE_SAT_NEXT) {
		//reset demod
		vprintk("[%d] XXX set to 0x1f\n", state->nr+1);
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x1F);
	}

#if 0 //test
		error |= (fe_lla_error_t)ChipSetFieldImage(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TMGTHRISE_TMGLOCK_THRISE(Demod), 0x18);
		error |= (fe_lla_error_t)ChipSetFieldImage(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TMGTHFALL_TMGLOCK_THFALL(Demod), 0x08);
#endif
	switch (state->demod_search_algo) {
	case FE_SAT_WARM_START:
		vprintk("[%d] WARM START\n", state->nr+1);
		/*The symbol rate and the exact
		carrier frequency are known */
		/*Trig an acquisition (start the search)*/

		/*better for low symbol rate - only choice which allows tuning to symbolrate 667
			Note that tuner bandwith still takes into account known symbol rate
		*/
		vprintk("[%d] XXX set to 0x01\n", state->nr+1);
		error |= ChipSetOneRegister(state->base->ip.handle_demod,
																(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x01);

		break;

	case FE_SAT_COLD_START:
		vprintk("[%d] COLD START\n", state->nr+1);
		/*The symbol rate is known*/
		/*Trig an acquisition (start the search)*/
		vprintk("[%d] XXX set to 0x15\n", state->nr+1);
		error |= ChipSetOneRegister(state->base->ip.handle_demod,
																(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x15);
		break;

	case FE_SAT_BLIND_SEARCH:
		vprintk("[%d] BLIND_SEARCH\n", state->nr+1);
#if 0
		{
		//the following always return AUTO_GLOW=1 and AUTO_GUP=1 before blindscan
		u32 tst1=0;
		u32 tst2=0;
		int error1 =ChipGetField(state->base->ip.handle_demod,
												 FLD_FC8CODEW_DVBSX_DEMOD_TMGCFG3_AUTO_GUP(Demod), &tst1);
		error1 |=ChipGetField(state->base->ip.handle_demod,
												 FLD_FC8CODEW_DVBSX_DEMOD_TMGCFG3_AUTO_GLOW(Demod), &tst2);
		dprintk("BEFORE AUTO_GLOW=%d AUTO_GUP=%d error=%d\n", tst1, tst2, error1);
		}
#endif
		vprintk("[%d] XXX set to 0x0\n", state->nr+1);
		/*Trigger an acquisition (start the search)*/
		error |= ChipSetOneRegister(state->base->ip.handle_demod,
																(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x0); //WAS 0x1
		/*
			1: use frequency found by last blindscan as a start
			0: use 0 frequency as a start
		 */
		break;
	case FE_SAT_NEXT:
		/*Trig an acquisition (start the search)*/
		//dprintk("SAT NEXT\n");
#if 0
		{
		//the following always return AUTO_GLOW=1 and AUTO_GUP=1 before blindscan
		u32 tst1=0;
		u32 tst2=0;
		int error1 =ChipGetField(state->base->ip.handle_demod,
												 FLD_FC8CODEW_DVBSX_DEMOD_TMGCFG3_AUTO_GUP(Demod), &tst1);
		error1 |=ChipGetField(state->base->ip.handle_demod,
												 FLD_FC8CODEW_DVBSX_DEMOD_TMGCFG3_AUTO_GLOW(Demod), &tst2);
		dprintk("BEFORE AUTO_GLOW=%d AUTO_GUP=%d error=%d\n", tst1, tst2, error1);
		}
#endif
		vprintk("[%d] XXX set to 0x14\n", state->nr+1);
		error |= ChipSetOneRegister(state->base->ip.handle_demod,
																(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x14);

		break;
	}
	return error;
}


/*****************************************************
--FUNCTION	::	FE_STiD135_BlindSearchAlgo
--ACTION	::	programs the PEA to lauch a hardware search algo.
--PARAMS IN	::	pParams -> Pointer to fe_stid135_internal_param structure
			Demod -> demod 1 .. 8
			demodTimeout -> timeout of the demod part
			satellite_scan -> scan or acquisition context
--PARAMS OUT	::	lock_p -> demod lock status
--RETURN	::	error
--***************************************************/
static fe_lla_error_t FE_STiD135_BlindSearchAlgo(struct stv* state,
	u32 demodTimeout, BOOL satellite_scan, BOOL* lock_p)
{
	enum fe_stid135_demod Demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;
	*lock_p = TRUE;
	if (satellite_scan == FALSE) {
	} else {
		state->demod_search_range_hz = (u32)(24000000 +
			state->tuner_index_jump * 1000);
		if (state->demod_search_range_hz > 40000000) {
			state->demod_search_range_hz = 40000000;
		}
	}
	dprintk("Starting blindsearch satellite_scan=%d jump=%d freq=%d+%d\n", satellite_scan,
					state->tuner_index_jump,		state->tuner_frequency,
					state->demod_search_range_hz);
	dprintk("XXX set to 0x5c\n");
	error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod,
																			(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x5C)); /* Demod Stop*/
	if(error1)
		dprintk("error=%d\n", error1);
#if 0
	error |= (error1=fe_stid135_set_symbol_rate(state,state->base->ip.master_clock,
																							state->demod_symbol_rate, Demod));
	if(error1)
		dprintk("error=%d\n", error1);
#endif
	error |= (error1=FE_STiD135_StartSearch(state));
	if(error1)
		dprintk("error=%d\n", error1);

	/*DeepThought: arbirary fixed timeout: 4000*/
	error |= (error1=FE_STiD135_GetDemodLock(state, demodTimeout, lock_p));
	if(error1)
		dprintk("error=%d\n", error1);
	vprintk("called FE_STiD135_GetDemodLock: lock=%d Timeout=%d\n", *lock_p, demodTimeout);
	return error;
}


/*****************************************************
--FUNCTION	::	GetSignalParams
--ACTION	::	Read signal caracteristics
--PARAMS IN	::	pParams -> Pointer to fe_stid135_internal_param structure
			Demod -> current demod 1 .. 8
			satellite_scan -> scan scope
--PARAMS OUT	::	range_p -> RANGE Ok or not
--RETURN	::	error
--***************************************************/
static  fe_lla_error_t FE_STiD135_GetSignalParams(
	struct stv* state,
	BOOL satellite_scan, enum fe_sat_signal_type *range_p)

{
	enum fe_stid135_demod Demod = state->nr+1;
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u32 reg_value = 0;
	s32 fld_value;
	s32 offsetFreq = 0,
		i = 0,
		symbolRateOffset = 0;
	u32 lo_frequency;

	u8 timing;

	*range_p=FE_SAT_OUTOFRANGE;

	ChipWaitOrAbort(state->base->ip.handle_demod, 5);

	if(state->demod_search_algo == FE_SAT_BLIND_SEARCH ||
		 state->demod_search_algo == FE_SAT_NEXT) {
		/*	if Blind search wait for symbol rate offset information
			transfer from the timing loop to the
			demodulator symbol rate
		*/
		error |= ChipGetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(Demod), &reg_value);
		timing =  (u8)reg_value;
		i = 0;
		while ((i <= 50) && (timing != 0) && (timing != 0xFF)) {
			error |= ChipGetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(Demod), &reg_value);
			timing =  (u8)reg_value;
			ChipWaitOrAbort(state->base->ip.handle_demod, 5);
			i += 5;
		}
	}
	error |= FE_STiD135_GetStandard(state->base->ip.handle_demod, Demod, &(state->signal_info.standard));

	/* On auxiliary demod, frequency found is not true, we have to pick it on master demod  */
	if(Demod == FE_SAT_DEMOD_2) {
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(Demod), &fld_value);
		if(fld_value == 3) {
			error |= FE_STiD135_GetCarrierFrequencyOffset_(pParams->handle_demod, FE_SAT_DEMOD_1, state->base->ip.master_clock, &offsetFreq);
			error |= FE_STiD135_GetSymbolRate_(pParams->handle_demod, FE_SAT_DEMOD_1,
																				 state->base->ip.master_clock, &(state->signal_info.symbol_rate));
			error |= FE_STiD135_TimingGetOffset_(pParams->handle_demod, FE_SAT_DEMOD_1,
																					state->signal_info.symbol_rate, &symbolRateOffset);
		}
	}
	if(Demod == FE_SAT_DEMOD_4) {
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(Demod), &fld_value);
		if(fld_value == 3) {
			error |= FE_STiD135_GetCarrierFrequencyOffset_(pParams->handle_demod, FE_SAT_DEMOD_3,
																							 state->base->ip.master_clock, &offsetFreq);
			error |= FE_STiD135_GetSymbolRate_(pParams->handle_demod, FE_SAT_DEMOD_3,
																				 state->base->ip.master_clock, &(state->signal_info.symbol_rate));
			error |= FE_STiD135_TimingGetOffset_(pParams->handle_demod, FE_SAT_DEMOD_3,
																					 state->signal_info.symbol_rate, &symbolRateOffset);
		}
	}
	if(((Demod != FE_SAT_DEMOD_2) && (Demod != FE_SAT_DEMOD_4)) ||
		((Demod == FE_SAT_DEMOD_2) && (fld_value != 3)) ||
		((Demod == FE_SAT_DEMOD_4) && (fld_value != 3))) {
		error |= FE_STiD135_GetCarrierFrequencyOffset(state, state->base->ip.master_clock, &offsetFreq);
		error |= FE_STiD135_GetSymbolRate(state, state->base->ip.master_clock, &state->signal_info.symbol_rate);
		error |= FE_STiD135_TimingGetOffset(state, state->signal_info.symbol_rate,  &symbolRateOffset);
	}
	/* Get timing loop offset */
	if(symbolRateOffset < 0) {
		symbolRateOffset *= (-1);
		state->signal_info.symbol_rate -= (u32)symbolRateOffset;
	} else
		state->signal_info.symbol_rate += (u32)symbolRateOffset;

	lo_frequency = state->base->ip.lo_frequency / 1000;
	state->signal_info.frequency = lo_frequency;
	if(offsetFreq < 0) {
		offsetFreq *= (-1);
		state->signal_info.frequency -= (u32)offsetFreq/1000;
	} else
		state->signal_info.frequency += (u32)offsetFreq/1000;

	error |=
		FE_STiD135_GetViterbiPunctureRate(state, &state->signal_info.puncture_rate);

	error = error | fe_stid135_get_mode_code(state,
				&state->signal_info.modcode,
				&state->signal_info.frame_length,
				&state->signal_info.pilots);

	error |= ChipGetField(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_TMGOBS_ROLLOFF_STATUS(Demod), &fld_value);
	state->signal_info.roll_off = (enum fe_sat_rolloff)fld_value;

	switch (state->signal_info.standard) {
	case FE_SAT_DVBS2_STANDARD:
		error |= ChipGetField(
		state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS6_SPECINV_DEMOD(Demod), &fld_value);
		state->signal_info.spectrum = (enum fe_sat_iq_inversion)fld_value;

		if (((state->signal_info.modcode >= FE_SAT_QPSK_14) && (state->signal_info.modcode <= FE_SAT_QPSK_910))
			|| ((state->signal_info.modcode >= FE_SAT_DVBS1_QPSK_12) && (state->signal_info.modcode <= FE_SAT_DVBS1_QPSK_78))
			|| ((state->signal_info.modcode >= FE_SATX_QPSK_13_45) && (state->signal_info.modcode <= FE_SATX_QPSK_11_20))
			|| ((state->signal_info.modcode >= FE_SATX_QPSK_11_45) && (state->signal_info.modcode <= FE_SATX_QPSK_32_45)))
			state->signal_info.modulation = FE_SAT_MOD_QPSK;

		else if (((state->signal_info.modcode >= FE_SAT_8PSK_35) && (state->signal_info.modcode <= FE_SAT_8PSK_910))
			|| ((state->signal_info.modcode >= FE_SATX_8PSK_23_36) && (state->signal_info.modcode <= FE_SATX_8PSK_13_18))
			|| ((state->signal_info.modcode >= FE_SATX_8PSK_7_15) && (state->signal_info.modcode <= FE_SATX_8PSK_32_45))
			|| (state->signal_info.modcode == FE_SATX_8PSK))
			state->signal_info.modulation = FE_SAT_MOD_8PSK;

		else if (((state->signal_info.modcode >= FE_SAT_16APSK_23) && (state->signal_info.modcode <= FE_SAT_16APSK_910))
			|| ((state->signal_info.modcode >= FE_SATX_16APSK_26_45) && (state->signal_info.modcode <= FE_SATX_16APSK_23_36))
			|| ((state->signal_info.modcode >= FE_SATX_16APSK_25_36) && (state->signal_info.modcode <= FE_SATX_16APSK_77_90))
			|| ((state->signal_info.modcode >= FE_SATX_16APSK_7_15) && (state->signal_info.modcode <= FE_SATX_16APSK_32_45))
			|| (state->signal_info.modcode == FE_SATX_16APSK))
			state->signal_info.modulation = FE_SAT_MOD_16APSK;

		else if (((state->signal_info.modcode >= FE_SAT_32APSK_34) && (state->signal_info.modcode <= FE_SAT_32APSK_910))
			|| ((state->signal_info.modcode >= FE_SATX_32APSK_R_58) && (state->signal_info.modcode <= FE_SATX_32APSK_7_9))
			|| ((state->signal_info.modcode >= FE_SATX_32APSK_2_3) && (state->signal_info.modcode <= FE_SATX_32APSK_32_45_S))
			|| (state->signal_info.modcode == FE_SATX_32APSK))
			state->signal_info.modulation = FE_SAT_MOD_32APSK;

		else if ((state->signal_info.modcode == FE_SATX_VLSNR1) || (state->signal_info.modcode == FE_SATX_VLSNR2))
			state->signal_info.modulation = FE_SAT_MOD_VLSNR;

		else if (((state->signal_info.modcode >= FE_SATX_64APSK_11_15) && (state->signal_info.modcode <= FE_SATX_64APSK_5_6))
			|| (state->signal_info.modcode == FE_SATX_64APSK))
			state->signal_info.modulation = FE_SAT_MOD_64APSK;

		else if ((state->signal_info.modcode == FE_SATX_128APSK_3_4) || (state->signal_info.modcode == FE_SATX_128APSK_7_9))
			state->signal_info.modulation = FE_SAT_MOD_128APSK;

		else if ((state->signal_info.modcode == FE_SATX_256APSK_32_45) || (state->signal_info.modcode == FE_SATX_256APSK_3_4)
			|| (state->signal_info.modcode == FE_SATX_256APSK))
			state->signal_info.modulation = FE_SAT_MOD_256APSK;

		else if ((state->signal_info.modcode == FE_SATX_8APSK_5_9_L) || (state->signal_info.modcode == FE_SATX_8APSK_26_45_L))
			state->signal_info.modulation = FE_SAT_MOD_8PSK_L;

		else if (((state->signal_info.modcode >= FE_SATX_16APSK_1_2_L) && (state->signal_info.modcode <= FE_SATX_16APSK_5_9_L))
			|| (state->signal_info.modcode == FE_SATX_16APSK_3_5_L)
			|| (state->signal_info.modcode == FE_SATX_16APSK_2_3_L))
			state->signal_info.modulation = FE_SAT_MOD_16APSK_L;

		else if (state->signal_info.modcode == FE_SATX_32APSK_2_3_L)
			state->signal_info.modulation = FE_SAT_MOD_32APSK_L;

		else if (state->signal_info.modcode == FE_SATX_64APSK_32_45_L)
			state->signal_info.modulation = FE_SAT_MOD_64APSK_L;

		else if (((state->signal_info.modcode >= FE_SATX_256APSK_29_45_L) && (state->signal_info.modcode <= FE_SATX_256APSK_31_45_L))
			|| (state->signal_info.modcode == FE_SATX_256APSK_11_15_L))
			state->signal_info.modulation = FE_SAT_MOD_256APSK_L;

		else if (state->signal_info.modcode == FE_SATX_1024APSK)
			state->signal_info.modulation = FE_SAT_MOD_1024APSK;

		else
			state->signal_info.modulation = FE_SAT_MOD_UNKNOWN;

	break;

	case FE_SAT_DVBS1_STANDARD:
	case FE_SAT_DSS_STANDARD:
		error |= ChipGetField(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_VITERBI_FECM_IQINV(Demod), &fld_value);
		state->signal_info.spectrum = (enum fe_sat_iq_inversion)fld_value;
		state->signal_info.modulation = FE_SAT_MOD_QPSK;
	break;

	case FE_SAT_TURBOCODE_STANDARD:
	case FE_SAT_UNKNOWN_STANDARD:
	break;
	}

	offsetFreq = (s32)state->signal_info.frequency * 1000
			- (s32)lo_frequency * 1000 - state->tuner_frequency;

	if((state->demod_search_algo == FE_SAT_BLIND_SEARCH ||
			state->demod_search_algo == FE_SAT_NEXT)
	|| (state->demod_symbol_rate < (u32)10000000)) {
		if ((state->demod_search_algo == FE_SAT_BLIND_SEARCH ||
				 state->demod_search_algo == FE_SAT_NEXT)) {
			*range_p = FE_SAT_RANGEOK;
		} else if ((u32)(ABS(offsetFreq)) <= ((
			state->demod_search_range_hz / 2)))
			*range_p = FE_SAT_RANGEOK;

		else if ((u32)(ABS(offsetFreq)) <= (FE_STiD135_CarrierWidth(
				state->signal_info.symbol_rate,
					state->signal_info.roll_off) / 2))
			*range_p = FE_SAT_RANGEOK;

		else
			*range_p = FE_SAT_OUTOFRANGE;
	} else {
		if ((u32)(ABS(offsetFreq)) <= ((state->demod_search_range_hz / 2)))
			*range_p = FE_SAT_RANGEOK;
		else
			*range_p = FE_SAT_OUTOFRANGE;
	}
	if(*range_p ==  FE_SAT_OUTOFRANGE) {
		dprintk("RETURNING OUT OF RANGE: freq=%dkHz search_range/2=%dkHz cw/2=%d\n",
						offsetFreq/1000, state->demod_search_range_hz/2000,
						FE_STiD135_CarrierWidth(
																		state->signal_info.symbol_rate,
																		state->signal_info.roll_off) / 2000);
	}
	return error;
}


/*****************************************************
--FUNCTION	::	FE_STiD135_TrackingOptimization
--ACTION	::	Set Optimized parameters for tracking
--PARAMS IN	::	pParams -> Pointer to fe_stid135_internal_param structure
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	error
--***************************************************/
static fe_lla_error_t FE_STiD135_TrackingOptimization(struct stv* state)
{
	enum fe_stid135_demod Demod = state->nr+1;
	u32 symbolRate;
	s32 pilots, timing_offset;
	u8 aclc;
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	enum fe_sat_modcode foundModcod;


	/* Read the Symbol rate */
	error |= FE_STiD135_GetSymbolRate(state, state->base->ip.master_clock, &symbolRate);
	error |= FE_STiD135_TimingGetOffset(state, symbolRate, &timing_offset);
	if(timing_offset < 0) {
		timing_offset *= (-1);
		symbolRate -= (u32)timing_offset;
	} else
		symbolRate += (u32)timing_offset;

	switch (state->signal_info.standard) {
	case FE_SAT_DVBS1_STANDARD:
		if (state->demod_search_standard == FE_SAT_AUTO_SEARCH) {
			error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS1_ENABLE(Demod), 1);
			error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS2_ENABLE(Demod), 0);
			error |= ChipSetRegisters(state->base->ip.handle_demod,
			(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 1);
		}
		/*Set the rolloff to the manual value (given at the
		initialization)*/
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(Demod), state->roll_off);
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALSX_ROLLOFF(Demod), 1);
		error |= ChipSetRegisters(state->base->ip.handle_demod,
			(u16)REG_RC8CODEW_DVBSX_DEMOD_DEMOD(Demod), 1);

		/* force to viterbi bit error */
		error |= ChipSetOneRegister(state->base->ip.handle_demod,
			(u16)REG_RC8CODEW_DVBSX_HWARE_ERRCTRL1(Demod), 0x75);
	break;

	case FE_SAT_DSS_STANDARD:
		if( state->demod_search_standard == FE_SAT_AUTO_SEARCH) {
			error |= ChipSetFieldImage(state->base->ip.handle_demod,
				FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS1_ENABLE(Demod), 1);
			error |= ChipSetFieldImage(state->base->ip.handle_demod,
				FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS2_ENABLE(Demod), 0);
			error |= ChipSetRegisters(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 1);
		}
		/*Set the rolloff to the manual value
		(given at the initialization)*/
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(Demod),state->roll_off);
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALSX_ROLLOFF(Demod), 1);
		error |= ChipSetRegisters(state->base->ip.handle_demod,
			(u16)REG_RC8CODEW_DVBSX_DEMOD_DEMOD(Demod), 1);

		/* force to viterbi bit error */
		error |= ChipSetOneRegister(state->base->ip.handle_demod,
			(u16)REG_RC8CODEW_DVBSX_HWARE_ERRCTRL1(Demod), 0x75);
	break;

	case FE_SAT_DVBS2_STANDARD:
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS1_ENABLE(Demod), 0);
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS2_ENABLE(Demod), 1);
		error |= ChipSetRegisters(state->base->ip.handle_demod,
			(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 1);

		/* force to DVBS2 PER  */
		error |= ChipSetOneRegister(state->base->ip.handle_demod,
			(u16)REG_RC8CODEW_DVBSX_HWARE_ERRCTRL1(Demod), 0x67);


		foundModcod = state->signal_info.modcode;
		pilots = state->signal_info.pilots;

		aclc = FE_STiD135_GetOptimCarrierLoop(symbolRate,
				foundModcod, pilots);

		if (((foundModcod >= FE_SAT_QPSK_14) && (foundModcod <= FE_SAT_QPSK_910))
			|| ((foundModcod >= FE_SAT_DVBS1_QPSK_12) && (foundModcod <= FE_SAT_DVBS1_QPSK_78))
			|| ((foundModcod >= FE_SATX_QPSK_13_45) && (foundModcod <= FE_SATX_QPSK_11_20))
			|| ((foundModcod >= FE_SATX_QPSK_11_45) && (foundModcod <= FE_SATX_QPSK_32_45))) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), aclc);
		} else if (((foundModcod >= FE_SAT_8PSK_35) && (foundModcod <= FE_SAT_8PSK_910))
			|| ((foundModcod >= FE_SATX_8PSK_23_36) && (foundModcod <= FE_SATX_8PSK_13_18))
			|| ((foundModcod >= FE_SATX_8PSK_7_15) && (foundModcod <= FE_SATX_8PSK_32_45))
			|| (foundModcod == FE_SATX_8PSK)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S28(Demod), aclc);
		} else if (((foundModcod >= FE_SAT_16APSK_23) && (foundModcod <= FE_SAT_16APSK_910))
			|| ((foundModcod >= FE_SATX_16APSK_26_45) && (foundModcod <= FE_SATX_16APSK_23_36))
			|| ((foundModcod >= FE_SATX_16APSK_25_36) && (foundModcod <= FE_SATX_16APSK_77_90))
			|| ((foundModcod >= FE_SATX_16APSK_7_15) && (foundModcod <= FE_SATX_16APSK_32_45))
			|| (foundModcod == FE_SATX_16APSK)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S216A(Demod), aclc);
		} else if (((foundModcod >= FE_SAT_32APSK_34) && (foundModcod <= FE_SAT_32APSK_910))
			|| ((foundModcod >= FE_SATX_32APSK_R_58) && (foundModcod <= FE_SATX_32APSK_7_9))
			|| ((foundModcod >= FE_SATX_32APSK_2_3) && (foundModcod <= FE_SATX_32APSK_32_45_S))
			|| (foundModcod == FE_SATX_32APSK)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S232A(Demod), aclc);
		} else if ((foundModcod == FE_SATX_VLSNR1) || (foundModcod == FE_SATX_VLSNR2)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			/*error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2(Demod), aclc);*/
		} else if (((foundModcod >= FE_SATX_64APSK_11_15) && (foundModcod <= FE_SATX_64APSK_5_6))
			|| (foundModcod == FE_SATX_64APSK)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S264A(Demod), aclc);
		} else if ((foundModcod == FE_SATX_128APSK_3_4) || (foundModcod == FE_SATX_128APSK_7_9)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2128A(Demod), aclc);
		} else if ((foundModcod == FE_SATX_256APSK_32_45) || (foundModcod == FE_SATX_256APSK_3_4)
			|| (foundModcod == FE_SATX_256APSK)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2256A(Demod), aclc);
		} else if ((foundModcod == FE_SATX_8APSK_5_9_L) || (foundModcod == FE_SATX_8APSK_26_45_L)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S28(Demod), aclc);
		} else if (((foundModcod >= FE_SATX_16APSK_1_2_L) && (foundModcod <= FE_SATX_16APSK_5_9_L))
			|| (foundModcod == FE_SATX_16APSK_3_5_L)
			|| (foundModcod == FE_SATX_16APSK_2_3_L)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S216A(Demod), aclc);
		} else if (foundModcod == FE_SATX_32APSK_2_3_L) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S232A(Demod), aclc);
		} else if (foundModcod == FE_SATX_64APSK_32_45_L) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S264A(Demod), aclc);
		} else if (((foundModcod >= FE_SATX_256APSK_29_45_L) && (foundModcod <= FE_SATX_256APSK_31_45_L))
			|| (foundModcod == FE_SATX_256APSK_11_15_L)) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2256A(Demod), aclc);
		} else if (foundModcod == FE_SATX_1024APSK) {
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_ACLC2S2Q(Demod), 0x48);
		}

		//else
	break;

	case FE_SAT_UNKNOWN_STANDARD:
	case FE_SAT_TURBOCODE_STANDARD:
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS1_ENABLE(Demod), 1);
		error |= ChipSetFieldImage(state->base->ip.handle_demod,
			FLD_FC8CODEW_DVBSX_DEMOD_DMDCFGMD_DVBS2_ENABLE(Demod), 1);
		error |= ChipSetRegisters (state->base->ip.handle_demod,
			(u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 1);
	break;
	}
	return error;
}


/******************************************************************************/
/*
		WideBand Section, specific Slice functionality
		This routines assume a valid demod lock on a HBWchannel
		and parallel symbol capability (WB Demod)
		ADC+AGC setup is not covered here
*/

/*****************************************************
--FUNCTION	::	fe_stid135_get_slice_list
--ACTION	::	Returns list of available slices
			Cardboard ram bug workaround on cut1
--PARAMS IN	::	Handle -> Front End Handle
			Demod -> current demod 1 .. 8
--PARAMS OUT	::	SlcList -> Slice list pointer
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_get_slice_list(struct stv* state,
		struct fe_stid135_slice_list *SlcList)
{
	enum fe_stid135_demod Demod = state->nr+1;
	u8 FEC_Lock_TimeOut = 100;
	u32 reg_value;
	BOOL lock = FALSE ;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	int Demodflt_Rammode;
	u8 SliceSel, MaxSliceNumber, MinSliceNumber;
	u16 i;
	struct fe_stid135_internal_param *pParams;

	pParams = &state->base->ip;

	SlcList->max_index = 0;

		/* check if the demod can handle it */
		if(Demod > FE_SAT_DEMOD_4)
			return FE_LLA_INVALID_HANDLE;

		/* store temporarily */
		error |= ChipGetField(state->base->ip.handle_demod,
		FLD_FC8CODEW_DVBSX_DEMOD_SLICECFG_DEMODFLT_RAMMODE(Demod), &Demodflt_Rammode);

		/* Switch off rammode */
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICECFG_DEMODFLT_RAMMODE(Demod), 0);
		/* fetch the SliceMax index */
		error |= ChipGetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_SLICEMAX(Demod), &reg_value);
		MaxSliceNumber = (u8)reg_value;

		/* no slice, no result */
		// Fixed issue, no slice detected on a sliced channel with only 1 slice
		if (MaxSliceNumber < 1) return error;

		/* fetch the SliceMin index */
		error |= ChipGetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_SLICEMIN(Demod), &reg_value);
		MinSliceNumber = (u8)reg_value;
		error |= ChipGetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_SLICESEL(Demod), &reg_value);
		SliceSel = (u8)reg_value;
		//rather use Getregisters,2 !

		for (i = MinSliceNumber; i <= MaxSliceNumber; i++) {
			/* tbd, try fetching it from SLC itself, also the anchor
			frame settings */
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_SLICESEL(Demod), i);

			// Delineator reset, to be sure
			error |= ChipSetField(state->base->ip.handle_demod,
				FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod),1);
			error |= ChipSetField(state->base->ip.handle_demod,
				FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod),0);

			error |= FE_STiD135_GetFECLock(state, FEC_Lock_TimeOut, &lock);
			if (lock) {
				SlcList->t_slice[SlcList->max_index].slice_id = (u8)i;
				SlcList->max_index = (u16)(SlcList->max_index + 1);
			} else {
				SlcList->t_slice[SlcList->max_index].slice_id = 0;
			}
		}
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICECFG_DEMODFLT_RAMMODE(Demod),
					Demodflt_Rammode);
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_SLICESEL(Demod),
			SliceSel);

		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod),1);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod),0);

	return error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_set_slice
--ACTION	::	Outputs desired slice
--PARAMS IN	::	Handle	==> Front End Handle
			Demod ==> current demod 1..8
			SliceID ==> slice id
--PARAMS OUT	::	None
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_slice(struct stv* state, u8 SliceID)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	enum fe_stid135_demod Demod = state->nr+1;
	error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_DEMODFLT_XXXMODE(Demod), 0x14);
	error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICESEL_DEMODFLT_SLICESEL(Demod),
												SliceID);


	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_slice_info
--ACTION	::	Returns Info on a selected slice
			Cardboard ram bug workaround on cut1
--PARAMS IN	::	Handle -> Front End Handle
			Demod -> current demod 1..8
			SliceId -> slice id
--PARAMS OUT	::	pSliceInfo -> pointer to slice data structure
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_get_slice_info(struct stv* state, u8 SliceId, struct fe_sat_slice *pSliceInfo)
{
	u8 FEC_Lock_TimeOut = 100;
	enum fe_stid135_demod Demod = state->nr+1;
	u32 reg_value;
	BOOL lock = FALSE;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 SliceSel, MaxSliceNumber, MinSliceNumber;
	struct fe_stid135_internal_param *pParams;

	pParams = &state->base->ip;

		/* check if the demod can handle it */
		if(Demod > FE_SAT_DEMOD_4)
			return FE_LLA_INVALID_HANDLE;

		/* fetch the SliceMax index */
		error |= ChipGetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_SLICEMAX(Demod), &reg_value);
		MaxSliceNumber = (u8)reg_value;
		/* fetch the SliceMin index */
		error |= ChipGetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_SLICEMIN(Demod), &reg_value);
		MinSliceNumber = (u8)reg_value;
		/* Save temporary */
		error |= ChipGetOneRegister(state->base->ip.handle_demod,
		(u16)REG_RC8CODEW_DVBSX_DEMOD_SLICESEL(Demod), &reg_value);
		SliceSel = (u8)reg_value;

		if((SliceId >= MinSliceNumber) && (SliceId <= MaxSliceNumber)) {
			/* tbd, try fetching it from SLC itself, also the anchor
			frame settings */
			error |= ChipSetOneRegister(state->base->ip.handle_demod,
				(u16)REG_RC8CODEW_DVBSX_DEMOD_SLICESEL(Demod), SliceId);

			// Delineator reset, to be sure
			error |= ChipSetField(state->base->ip.handle_demod,
				FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod),1);
			error |= ChipSetField(state->base->ip.handle_demod,
				FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod),0);

			error |= FE_STiD135_GetFECLock(state, FEC_Lock_TimeOut, &lock);
			if (lock) {
				// fetch the rest of the parameters
				pSliceInfo->slice_id = SliceId;
				error |= ChipGetOneRegister(state->base->ip.handle_demod,
						(u16)REG_RC8CODEW_DVBSX_PKTDELIN_PDELSTATUS2(Demod), &reg_value);
				pSliceInfo->modcode = ChipGetFieldImage(state->base->ip.handle_demod,
						FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS2_FRAME_MODCOD(Demod));
				pSliceInfo->frame_length = ChipGetFieldImage(state->base->ip.handle_demod,
						FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS2_FRAME_TYPE(Demod)) >> 1;
				pSliceInfo->pilots = ChipGetFieldImage(state->base->ip.handle_demod,
						FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS2_FRAME_TYPE(Demod)) & 0x01;
			} else {
				pSliceInfo->slice_id = SliceId;
				pSliceInfo->modcode = FE_SAT_MODCODE_UNKNOWN;
				pSliceInfo->frame_length = FE_SAT_UNKNOWN_FRAME;
				pSliceInfo->pilots = FE_SAT_UNKNOWN_PILOTS;
			}
		}
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_SLICESEL(Demod), SliceSel);

		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod),1);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(Demod),0);

	return error;
}


/*****************************************************
--FUNCTION	::	FE_STiD135_get_mid
--ACTION	::	returns Chip ID
--PARAMS IN	::	Handle -> Front End Handle
--PARAMS OUT	::	chip ID
--RETURN	::	error
--***************************************************/
fe_lla_error_t FE_STiD135_get_mid(struct fe_stid135_internal_param* pParams, u8* mid_p)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u32 reg_value;
	error |= ChipGetOneRegister(pParams->handle_demod, RC8CODEW_C8CODEW_TOP_CTRL_MID, &reg_value);
	*mid_p = (u8)reg_value;

	return error;
}


/*****************************************************
--FUNCTION	::	FE_STiD135_GetStatus
--ACTION	::	Get almost all status
--PARAMS IN	::	Handle -> Front End Handle
			Demod -> current demod 1..8
--PARAMS OUT	::	status_p -> bitfield structure reporting all status
--RETURN	::	error
--***************************************************/
fe_lla_error_t FE_STiD135_GetStatus(struct stv* state, struct status_bit_fields* status_p)
{
	enum fe_stid135_demod Demod = state->nr+1;
	s32 AgcrfPath = 0;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams;
	s32 val1, val2, val3;
	u32 val4;

	pParams = &state->base->ip;
	if(pParams != NULL) {
		error |= fe_stid135_get_agcrf_path(state,  &AgcrfPath);

		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_LOCK_DEFINITIF(Demod), &val1);
		status_p->lock_definitif = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_SLICESTAT_DEMODFLT_PDELSYNC(Demod), &val1);
		status_p->pdelsync = (u8)(val1 & 0x3);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_ASYMSTATUS_ASYMCTRL_ON(Demod), &val1);
		status_p->asymctrl_on = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GSTAT_PSD_DONE(Demod), &val1);
		status_p->gstat_psd_done = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISTXSTATUS_TX_END(AgcrfPath), &val1);
		status_p->distxstatus_tx_end = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISRXSTAT1_RXEND(AgcrfPath), &val1);
		status_p->disrxstat1_rxend = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_FECSPY_FSTATUS_FOUND_SIGNAL(Demod), &val1);
		status_p->fstatus_found_signal = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS_TSFIFO_LINEOK(Demod), &val1);
		status_p->tsfifo_lineok = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_SFSTATUS_SFEC_LINEOK(Demod), &val1);
		status_p->sfec_lineok = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_BCH_ERROR_FLAG(Demod), &val1);
		status_p->bch = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_LOCK(Demod), &val1);
		status_p->pktdelin_lock = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_SUPERFEC_SFECSTATUS_LOCKEDSFEC(Demod), &val1);
		status_p->lockedsfec = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_VITERBI_VSTATUSVIT_LOCKEDVIT(Demod), &val1);
		status_p->lockedvit = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDSTATE_DEMOD_MODE(Demod), &val1);
		status_p->demod_mode = (u8)(val1 & 0x1F);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_AGCRF_AGCRFCN_AGCRF_LOCKED(AgcrfPath), &val1 );
		status_p->agcrf_locked = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FSTID135_AFE_AFE_STATUS_PLL_LOCK, &val1);
		status_p->pll_lock = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_CAR_LOCK(Demod), &val1);
		status_p->car_lock = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS_TIMING_IS_LOCKED(Demod), &val1);
		status_p->timing_is_locked = (u8)(val1 & 0x1);
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_SYS_C8SECTPFE_SYS_OTHER_ERR_STATUS, &val4);
		val4 = val4 >> 1;
		status_p->tsdmaerror = (u8)(val4 & 0x1);
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_STATUS, &val4);
		val4 = val4 >> 1;
		status_p->dest0_status_dma_overflow = (u8)(val4 & 0x1);
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_STATUS, &val4);
		val4 = val4 >> 1;
		status_p->dest1_status_dma_overflow = (u8)(val4 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS5_SHDB_OVERFLOW(Demod), &val1);
		status_p->shdb_overflow = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS5_SHDB_ERROR(Demod), &val1);
		status_p->shdb_error = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS2_DEMOD_DELOCK(Demod), &val1);
		status_p->demod_delock = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_DELOCK(Demod), &val1);
		status_p->pktdelin_delock = (u8)(val1 & 0x1);
		/* Freeze the BBFCRCKO and UPCRCKO counters. Allow to read them in coherency. */
		//ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL2_RESET_UPKO_COUNT(Demod), 1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_BBFCRCKO1_BBHCRC_KOCNT(Demod), &val1) ;
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_BBFCRCKO0_BBHCRC_KOCNT(Demod) , &val2);
		status_p->bbfcrcko = (u16)((val1 << 8)+ (val2));

		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_UPCRCKO1_PKTCRC_KOCNT(Demod), &val1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_UPCRCKO0_PKTCRC_KOCNT(Demod), &val2);
		status_p->upcrcko = (u16)((val1<< 8) + val2);
		/* Reset and Restart BBFCRCKO and UPCRCKO counters. */
		//ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL2_RESET_UPKO_COUNT(Demod), 0);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS2_TSFIFO_LINENOK(Demod), &val1) ;
		status_p->tsfifo_linenok = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS2_DIL_NOSYNC(Demod), &val1);
		status_p->dil_nosync = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS_TSFIFO_ERROR(Demod), &val1);
		status_p->tsfifo_error = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSDEBUG0_TSFIFO_ERROR_EVNT(Demod), &val1);
		status_p->tsfifo_error_evnt = (u8)(val1 & 0x1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_ERRCNT12_ERR_CNT1(Demod), &val1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_ERRCNT11_ERR_CNT1(Demod), &val2);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_ERRCNT10_ERR_CNT1(Demod), &val3);
		status_p->errcnt1 = (u32)(((val1 << 16) + (val2 << 8) + val3) & 0xFFFFFF);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_ERRCNT22_ERR_CNT2(Demod), &val1);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_ERRCNT21_ERR_CNT2(Demod), &val2);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_ERRCNT20_ERR_CNT2(Demod), &val3);
		status_p->errcnt2 = (u32)(((val1 << 16) + (val2 << 8) + val3) & 0xFFFFFF);
	}

	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_reset_obs_registers
--ACTION	::	Resets memorized status
--PARAMS IN	::	Handle -> Front End Handle
			Demod -> current demod 1..8
--PARAMS OUT	::	NONE
--RETURN	::	error
--***************************************************/
fe_lla_error_t fe_stid135_reset_obs_registers(struct stv* state)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	enum fe_stid135_demod Demod = state->nr+1;
	struct fe_stid135_internal_param *pParams;
	s32 fld_value;
	pParams = &state->base->ip;
	if(pParams != NULL) {
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS5_SHDB_OVERFLOW(Demod), &fld_value);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS5_SHDB_ERROR(Demod), &fld_value);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS2_DEMOD_DELOCK(Demod), 0);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_PKTDELIN_DELOCK(Demod), 0);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL2_RESET_UPKO_COUNT(Demod), 1);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL2_RESET_UPKO_COUNT(Demod), 0);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS2_TSFIFO_LINENOK(Demod), 0);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS2_DIL_NOSYNC(Demod), 0);
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS_TSFIFO_ERROR(Demod), &fld_value);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSDEBUG0_TSFIFO_ERROR_EVNT(Demod), 0);
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELSTATUS1_BCH_ERROR_FLAG(Demod), 0);
		/* Reset counter of killed packet number */
		error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_ERRCNT20(Demod), 0x00);
	}

	return error;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_Term
--ACTION	::	Terminates chip connection
--PARAMS IN	::	Handle -> Front End Handle
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t FE_STiD135_Term(struct fe_stid135_internal_param* pParams)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;


	chip_close_proc("stid135");
	ChipClose(pParams->handle_demod);
	chip_close_proc("soc");
	ChipClose(pParams->handle_soc);


	return error;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_ReadHWMatype
--ACTION	::	Returns the Matype, need to Read 16 bits (2x8 bits
			linked registers)
--PARAMS IN	::	hChip -> Handle to the chip
			Demod -> current demod 1..8
--PARAMS OUT	::	isi_read -> ISI read
			matype -> Matype read
--RETURN	::	error
--***************************************************/
static fe_lla_error_t fe_stid135_read_hw_matype_(STCHIP_Info_t* hChip,
	enum fe_stid135_demod Demod, u8 *matype, u8 *isi_read)
{
	/*
		MATYPE (2 bytes) describes the input stream format, the type of mode adaptation and the
		transmission roll-off factor. All the stream information fields are stored in registers
		MATSTRx, UPLSTRx, DFLSTRx, SYNCSTR and SYNCDSTRx.
		SIS/MIS (1 bit) describes whether there is a single input stream or multiple input streams. If
		SIS/MIS = multiple input stream, then the second byte is the input stream identifier (ISI),
		otherwise the second byte is reserved.
		MATSTR0+1: matype of the current frame

	 */

	fe_lla_error_t error = FE_LLA_NO_ERROR;

	// Read at the same time both registers (specific registers)
	error |= ChipGetRegisters(hChip, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_MATSTR1(Demod), 2);
	*matype = (u8)(ChipGetFieldImage(hChip, FLD_FC8CODEW_DVBSX_PKTDELIN_MATSTR1_MATYPE_ROLLOFF(Demod))
		| ChipGetFieldImage(hChip, FLD_FC8CODEW_DVBSX_PKTDELIN_MATSTR1_MATYPE_NPD(Demod)) << 2
		| ChipGetFieldImage(hChip, FLD_FC8CODEW_DVBSX_PKTDELIN_MATSTR1_MATYPE_ISSYI(Demod)) << 3
		| ChipGetFieldImage(hChip, FLD_FC8CODEW_DVBSX_PKTDELIN_MATSTR1_MATYPE_CCMACM(Demod)) << 4
		| ChipGetFieldImage(hChip, FLD_FC8CODEW_DVBSX_PKTDELIN_MATSTR1_MATYPE_SISMIS(Demod)) << 5
		| ChipGetFieldImage(hChip, FLD_FC8CODEW_DVBSX_PKTDELIN_MATSTR1_MATYPE_TSGS(Demod)) << 6);
	*isi_read = (u8)ChipGetFieldImage(hChip, FLD_FC8CODEW_DVBSX_PKTDELIN_MATSTR0_MATYPEISI_CURRENT(Demod));

	if (hChip->Error) /*Check the error at the end of the function*/
		error = FE_LLA_I2C_ERROR;

	return error;

}

 fe_lla_error_t fe_stid135_read_hw_matype(struct stv* state, u8 *matype, u8 *isi_read)
{
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod Demod = state->nr+1;
	return fe_stid135_read_hw_matype_(hChip, Demod, matype, isi_read);
}


/*****************************************************
--FUNCTION	::	fe_stid135_manage_matype_info
--ACTION	::	Manage all action depending on Matype Information
--PARAMS IN	::	handle -> Front End Handle
			Demod -> current demod 1..8
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_manage_matype_info(struct stv* state)
{
	enum fe_stid135_demod Demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 matype_info, isi, genuine_matype;
	s32 fld_value;
	BOOL mis = FALSE;
	struct fe_stid135_internal_param *pParams = &state->base->ip;

	if(pParams == NULL)
		error=FE_LLA_INVALID_HANDLE;
	else {
		BUG_ON(state->base->ip.handle_demod->Error);
		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_TPKTDELIN_TESTBUS_SELECT(Demod), 8);
			/* Free MATYPE forced mode */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_BBHCTRL2_FORCE_MATYPEMSB(Demod), 0);
			/* Read Matype */
			error = fe_stid135_read_hw_matype(state, &matype_info, &isi);
			genuine_matype = matype_info;
			state->signal_info.matype = genuine_matype;
			/* Check if MIS stream (Multi Input Stream). If yes then set the MIS Filter to get the Min ISI */
			if (!fe_stid135_check_sis_or_mis(matype_info)) {
#if 0
				if (state->demod_search_algo == FE_SAT_BLIND_SEARCH ||
						state->demod_search_algo == FE_SAT_NEXT) {
					fe_lla_error_t error1 = FE_LLA_NO_ERROR;
					struct fe_sat_isi_struct_t isi;
					int i;
					error1 |=fe_stid135_isi_scan(handle, Demod, &isi);

					dprintk("MIS DETECTION: error=%d nb_isi=%d\n", error1, isi.nb_isi);
					BUG_ON(isi.nb_isi<0 || isi.nb_isi>sizeof(isi.isi(isi.isi[0]));
					for(i=0;i< isi.nb_isi;++i) {
						dprintk("MIS DETECTION: isi[%d]=%d\n", i, isi.isi[i]);
					}
				} else {
					dprintk("MIS DETECTED algo=%d\n", state->demod_search_algo);
				}
#endif
				mis = TRUE;
				state->mis_mode = TRUE;
				/* Get Min ISI and activate the MIS Filter */
				error |= fe_stid135_select_min_isi(state);
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BITSPEED(Demod), 0);
				/* ISSYI use-case : minimize jitter below 100ns and provides a smooth TS output rate */
				if((genuine_matype >> 3) & 0x1) {
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPCRPID1_SOFFIFO_PCRADJ(Demod), 1);
				}
			}
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_TPKTDELIN_TESTBUS_SELECT(Demod), 0);

			/* If TS/GS = 11 (MPEG TS), reset matype force bit and do NOT load frames in MPEG packets */
			if(((genuine_matype>>6) & 0x3) == 0x3) {
#if 0 //present in main driver; removed in tbs commit de809bd2e2ec572b5dbf6cba0e93fca192844939
					if((genuine_matype >> 3) & 0x3) {
					/* CCM or ISSYI used */
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(Demod), 0);
					//error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSYNC_TSFIFO_SYNCMODE(Demod), 0);
				} else {
					/* ACM and ISSYI not used */
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(Demod), 1);
					//error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSYNC_TSFIFO_SYNCMODE(Demod), 2);
					}
#endif
				/* Unforce HEM mode */
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL0_HEMMODE_SELECT(Demod), 0);
				/* Go back to reset value settings */
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG1_TSFIFO_MANSPEED(Demod), 0);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0xFF);
				/* To avoid reset of stream merger in annexM, ACM or if PID filter is enabled, set automatic computation of TS bit rate */
				error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DSTATUS6_SIGNAL_ANNEXEM(Demod), &fld_value);
				if((fld_value == 0) || (fld_value == 1) || (mis)) {
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BITSPEED(Demod), 0);
				} else { /* sliced mode */
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BITSPEED(Demod), 1);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSBITRATE1(Demod), 0x80);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSBITRATE0(Demod), 0x00);
				}
			}
			/* If TS/GS = 10 (GSE-HEM High Efficiency Mode) reset matype force bit, load frames in MPEG packets and disable latency regulation */
			else if(((genuine_matype>>6) & 0x3) == 0x2){
#ifdef USER1
				/* Force HEM mode */
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL0_HEMMODE_SELECT(Demod), 3);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG1_TSFIFO_MANSPEED(Demod), 3);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_EMBINDVB(Demod), 1); // - new management of NCR
					/* Adjust TSSPEED according to channel maximum bit rate for the demodulator */
					error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_SDEMAP_MAXLLRRATE(Demod), &fld_value);
					if(fld_value == 0x00) {
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0x10); // - new management of NCR
					} else {
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0x16); // - new management of NCR
					}
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(Demod), 0);
					/* To avoid reset of stream merger in annexM, ACM or if PID filter is enabled, set pragmatic smoothing mode for computation of TS bit rate */
					matype_info &= 0x0F;
					/* Set bit 5 to ignore ISI/MIS bit because not compatible with NCR feature (latency regulation) */
					matype_info |= 0xB0;
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_MATCST1(Demod), matype_info);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_BBHCTRL2_FORCE_MATYPEMSB(Demod), 1);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BITSPEED(Demod), 1);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSBITRATE1(Demod), 0x80);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSBITRATE0(Demod), 0x00);
#else
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_EMBINDVB(Demod), 1);
					/* Go back to reset value settings */
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG1_TSFIFO_MANSPEED(Demod), 0);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0xFF);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(Demod), 0);
					/* Unforce HEM mode */
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL0_HEMMODE_SELECT(Demod), 0);
#endif
			}
			/* If TS/GS = 00 (Generic packetized) or 01 (Generic continuous) force matype/tsgs = 10 and load frames in MPEG packets */
			else if((((genuine_matype>>6) & 0x3) == 0x0) || ((genuine_matype>>6) & 0x3) == 0x1) {
#ifdef USER1
					/* Force HEM mode */
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL0_HEMMODE_SELECT(Demod), 3);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG1_TSFIFO_MANSPEED(Demod), 3);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_EMBINDVB(Demod), 1); // - new management of NCR
					/* Adjust TSSPEED according to channel maximum bit rate for the demodulator */
					error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_SDEMAP_MAXLLRRATE(Demod), &fld_value);
					if(fld_value == 0x00) {
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0x10); // - new management of NCR
					} else {
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0x16); // - new management of NCR
					}
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(Demod), 0);
					/* To avoid reset of stream merger in annexM, ACM or if PID filter is enabled, set pragmatic smoothing mode for computation of TS bit rate */
					matype_info &= 0x0F;
					/* Set bit 5 to ignore ISI/MIS bit because not compatible with NCR feature (latency regulation) */
					matype_info |= 0xB0;
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_MATCST1(Demod), matype_info);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_BBHCTRL2_FORCE_MATYPEMSB(Demod), 1);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BITSPEED(Demod), 1);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSBITRATE1(Demod), 0x80);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSBITRATE0(Demod), 0x00);
#else
					matype_info &= 0x0F;
					/* Set bit 5 to ignore ISI/MIS bit because not compatible with NCR feature (latency regulation) */
					matype_info |= 0xB0;
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_MATCST1(Demod), matype_info);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_BBHCTRL2_FORCE_MATYPEMSB(Demod), 1);
					/* Force HEM mode */
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL0_HEMMODE_SELECT(Demod), 3);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_EMBINDVB(Demod), 1);
					/* Switch to manual CLKOUT frequency processing */
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG1_TSFIFO_MANSPEED(Demod), 3);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0x18);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(Demod), 0);
					/* To avoid reset of stream merger in annexM, ACM or if PID filter is enabled, set pragmatic smoothing mode for computation of TS bit rate */
#endif
			}
			/* WB, and MIS mode */
			if ((state->demod_symbol_rate > state->base->ip.master_clock) && mis) {
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG1_TSFIFO_MANSPEED(Demod), 3);
				error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_SDEMAP_MAXLLRRATE(Demod), &fld_value);
				if(fld_value == 0x00) {
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0x10); // - new management of NCR
				} else {
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0x16); // - new management of NCR
				}
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BITSPEED(Demod), 1);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSBITRATE1(Demod), 0x80);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSBITRATE0(Demod), 0x00);
			}
		}
	}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_manage_matype_info_raw_bbframe
--ACTION	::	Manage all action depending on Matype Information
			(raw BB frame handling)
--PARAMS IN	::	handle -> Front End Handle
			Demod -> current demod 1..8
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
static fe_lla_error_t fe_stid135_manage_matype_info_raw_bbframe(struct stv* state)
{
	//STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod Demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 matype_info, isi, genuine_matype;
	//struct fe_stid135_internal_param *pParams = &state->base->ip;

		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			/* Read Matype */
			error = fe_stid135_read_hw_matype(state, &matype_info, &isi);
			genuine_matype = matype_info;

			/* Check if MIS stream (Multi Input Stream). If yes then set the MIS Filter to get the Min ISI */
			if (!fe_stid135_check_sis_or_mis(matype_info)) {
				state->mis_mode = TRUE;
				/* Get Min ISI and activate the MIS Filter */
				error |= fe_stid135_select_min_isi(state);
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BITSPEED(Demod), 0);
				/* ISSYI use-case : minimize jitter below 100ns and provides a smooth TS output rate */
				if((genuine_matype >> 3) & 0x1) {
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPCRPID1_SOFFIFO_PCRADJ(Demod), 1);
				}
			}

			/* If TS/GS = 01 (Generic continuous) => raw BB frame */
			if(((genuine_matype>>6) & 0x3) == 0x1) {
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL2_FORCE_CONTINUOUS(Demod), 1);
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL2_FRAME_MODE(Demod), 1);
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSYNC_TSFIFO_SYNCMODE(Demod), 2);

				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_EMBINDVB(Demod), 1);
				/* Switch to manual CLKOUT frequency processing */
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG1_TSFIFO_MANSPEED(Demod), 3);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(Demod), 0x18);
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(Demod), 0);
				/* To avoid reset of stream merger in annexM, ACM or if PID filter is enabled, set pragmatic smoothing mode for computation of TS bit rate */
			}
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_matype_infos
--ACTION	::	Returns the MATYPE (byte 1) info
--PARAMS IN	::	Handle -> Front End Handle
			Demod -> current demod 1..8
--PARAMS OUT	::	matype_infos -> pointer to MatypeInfo struct to fill
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_get_matype_infos(struct stv* state, struct fe_sat_dvbs2_matype_info_t *matype_infos)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)Handle;
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod Demod = state->nr+1;

	u8 matype_val,isi;
	s32 fld_value;

	if (state->base->ip.handle_demod->Error )
		error=FE_LLA_I2C_ERROR;
	else {
			error |= fe_stid135_read_hw_matype(state,  &matype_val, &isi);
			/* Also available in Px_TSSTATEM:Px_TSFRAME_MODE */
			matype_infos->StreamType = (matype_val>>6)&0x3;
			matype_infos->InputStream = (matype_val>>5)&0x1;
			/* Also available in Px_TSSTATEM:Px_TSACM_MODE */
			matype_infos->CodingModulation = (matype_val>>4)&0x1;
			/* Also available in Px_TSSTATEL:Px_TSISSYI_ON */
			matype_infos->InputStreamSynchro = (matype_val>>3)&0x1;
			/* Also available in Px_TSSTATEL:Px_NPD_ON */
			matype_infos->NullPacketdeletion = (matype_val>>2)&0x1;
			/* Also available in Px_TMGOBS:Px_ROLLOFF_STATUS */
			error |= ChipGetField(hChip, FLD_FC8CODEW_DVBSX_DEMOD_TMGOBS_ROLLOFF_STATUS(Demod), &fld_value);
			matype_infos->roll_off = (enum fe_sat_rolloff)(fld_value);

			/*Check the error at the end of the function*/
			if (state->base->ip.handle_demod->Error )
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_TunerInit
--ACTION	::	Performs tuner init
--PARAMS IN	::	pParams -> Pointer to fe_stid135_internal_param structure
-PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
TUNER_Error_t FE_STiD135_TunerInit(struct fe_stid135_internal_param *pParams)
{
	TUNER_Error_t error = TUNER_NO_ERR;
	int i;

	/* First, put in sleep mode all tuners if they were previously enabled
	The aim is to lower down power consumption */
	error |= FE_STiD135_TunerStandby(pParams->handle_demod ,AFE_TUNER1, FALSE);
	error |= FE_STiD135_TunerStandby(pParams->handle_demod ,AFE_TUNER2, FALSE);
	error |= FE_STiD135_TunerStandby(pParams->handle_demod ,AFE_TUNER3, FALSE);
	error |= FE_STiD135_TunerStandby(pParams->handle_demod ,AFE_TUNER4, FALSE);

	error |= (TUNER_Error_t)Oxford_AfeInit(pParams->handle_demod, pParams->quartz, pParams->internal_dcdc, pParams->internal_ldo);
	if(error != TUNER_NO_ERR)
		return(error);
	WAIT_N_MS(100);
	error |= Oxford_CalVCOfilter(pParams->handle_demod);
	/* Set RF inputs either single ended or differential */
	error |= Oxford_SetVglnaInputs(pParams->handle_demod, pParams->rf_input_type);

	/* Select calibration code source: use AFE_BBxI_CAL and AFE_BBxQ_CAL */
	error |= Oxford_CalCodeSource(pParams->handle_demod, AFE_TUNER_ALL, TRUE);

	/* At startup, set NF mode of VGLNA gain */
	for(i=AFE_TUNER1;i<=AFE_TUNER4;i++) {
		error |= fe_stid135_manage_LNF_IP3(pParams->handle_demod, (FE_OXFORD_TunerPath_t)i, 0);
	}

	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_tuner_enable
--ACTION	::	Wakes up tuner
--PARAMS IN	::	tuner_handle -> Handle to the chip
			tuner_nb -> tuner path to wake up
--PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_tuner_enable(STCHIP_Info_t* tuner_handle, FE_OXFORD_TunerPath_t tuner_nb)
{
	STCHIP_Error_t error = CHIPERR_NO_ERROR;

	switch(tuner_nb) {
		case AFE_TUNER1:
			error |= Oxford_EnableLO(tuner_handle, AFE_TUNER1);
			error |= Oxford_TunerStartUp(tuner_handle, AFE_TUNER1);
			error |= (STCHIP_Error_t)Oxford_AdcStartUp(tuner_handle, AFE_TUNER1);
			error |= Oxford_SetVGLNAgainMode(tuner_handle, AFE_TUNER1, 0);
		break;
		case AFE_TUNER2:
			error |= Oxford_EnableLO(tuner_handle, AFE_TUNER2);
			error |= Oxford_TunerStartUp(tuner_handle, AFE_TUNER2);
			error |= (STCHIP_Error_t)Oxford_AdcStartUp(tuner_handle, AFE_TUNER2);
			error |= Oxford_SetVGLNAgainMode(tuner_handle, AFE_TUNER2, 0);
		break;
		case AFE_TUNER3:
			error |= Oxford_EnableLO(tuner_handle, AFE_TUNER3);
			error |= Oxford_TunerStartUp(tuner_handle, AFE_TUNER3);
			error |= (STCHIP_Error_t)Oxford_AdcStartUp(tuner_handle, AFE_TUNER3);
			error |= Oxford_SetVGLNAgainMode(tuner_handle, AFE_TUNER3, 0);
		break;
		case AFE_TUNER4:
			error |= Oxford_EnableLO(tuner_handle, AFE_TUNER4);
			error |= Oxford_TunerStartUp(tuner_handle, AFE_TUNER4);
			error |= (STCHIP_Error_t)Oxford_AdcStartUp(tuner_handle, AFE_TUNER4);
			error |= Oxford_SetVGLNAgainMode(tuner_handle, AFE_TUNER4, 0);
		break;
		case AFE_TUNER_ALL:
			error |= CHIPERR_INVALID_HANDLE;
		break;
	}
	/* Fix of BZ#116882 (FIFO reset required to make sure that read/write pointers are at a clean location */
	error |= ChipSetField(tuner_handle, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TOP_STOPCLK_STOP_CKTUNER, (0x1)<<(tuner_nb-1));
	WAIT_N_MS(10);
	error |= ChipSetField(tuner_handle, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TOP_STOPCLK_STOP_CKTUNER, (0x0)<<(tuner_nb-1));
	WAIT_N_MS(100);
	return (fe_lla_error_t)error;
}

/*****************************************************
--FUNCTION	::	FE_STiD135_TunerStandby
--ACTION	::	Puts in sleep or wake-up tuner
--PARAMS IN	::	TunerHandle -> Handle to the chip
			TunerNb -> tuner path
			Enable -> boolean variable
-PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t FE_STiD135_TunerStandby(STCHIP_Info_t* TunerHandle, FE_OXFORD_TunerPath_t TunerNb, u8 Enable)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;


	if(Enable) {
		error |= (fe_lla_error_t)Oxford_EnableLO(TunerHandle, TunerNb);
		error |= (fe_lla_error_t)Oxford_TunerStartUp(TunerHandle, TunerNb);
		error |= (fe_lla_error_t)Oxford_AdcStartUp(TunerHandle, TunerNb);
	} else { /* Enable = FALSE */
		error |= (fe_lla_error_t)Oxford_AdcStop(TunerHandle, TunerNb); //DEEPTHO
		error |= (fe_lla_error_t)Oxford_TunerDisable(TunerHandle, TunerNb);
	}

	return (fe_lla_error_t)error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_get_agcrf_path
--ACTION	::	Gets tuner path connected to current demod
--PARAMS IN	::	hChip -> Handle to the chip
			demod -> Current demod 1..8
-PARAMS OUT	::	agcrf_path_p -> tuner path number
--RETURN	::	error
--***************************************************/
static fe_lla_error_t fe_stid135_get_agcrf_path_(STCHIP_Info_t* hChip,
			enum fe_stid135_demod demod, s32* agcrf_path_p)
{

	fe_lla_error_t error = FE_LLA_NO_ERROR;

	switch(demod) {
		case 1:
			error |= ChipGetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX0_RFMUX_DEMOD_SEL_1, agcrf_path_p) ;
		break;

		case 2:
			error |= ChipGetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX1_RFMUX_DEMOD_SEL_2, agcrf_path_p) ;
		break;

		case 3:
			error |= ChipGetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX2_RFMUX_DEMOD_SEL_3, agcrf_path_p) ;
		break;

		case 4:
			error |= ChipGetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX3_RFMUX_DEMOD_SEL_4, agcrf_path_p) ;
		break;

		case 5:
			error |= ChipGetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX4_RFMUX_DEMOD_SEL_5, agcrf_path_p) ;
		break;

		case 6:
			error |= ChipGetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX5_RFMUX_DEMOD_SEL_6, agcrf_path_p) ;
		break;

		case 7:
			error |= ChipGetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX6_RFMUX_DEMOD_SEL_7, agcrf_path_p) ;
		break;

		case 8:
			error |= ChipGetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX7_RFMUX_DEMOD_SEL_8, agcrf_path_p) ;
		break;
	}
	if(error)
		dprintk("hChip=%p agcrf_path_p=%p *agcrf_path_p=%d", hChip, agcrf_path_p, *agcrf_path_p);
	*agcrf_path_p +=1;
	return(error);
}

static fe_lla_error_t fe_stid135_get_agcrf_path(struct stv* state, s32* agcrf_path_p)
{
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod Demod = state->nr+1;
	return fe_stid135_get_agcrf_path_(hChip, Demod, agcrf_path_p);
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_rfmux_path
--ACTION	::	Sets RF mux
--PARAMS IN	::	hChip -> Handle to the chip
			demod -> Current demod 1..8
			tuner -> Current tuner 1..4
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_rfmux_path_(STCHIP_Info_t* hChip, 	enum fe_stid135_demod Demod,
																					FE_OXFORD_TunerPath_t tuner)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	switch(Demod) {
		case 1:
			error |= ChipSetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX0_RFMUX_DEMOD_SEL_1, (s32)(tuner - 1));
		break;

		case 2:
			error |= ChipSetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX1_RFMUX_DEMOD_SEL_2, (s32)(tuner - 1));
		break;

		case 3:
			error |= ChipSetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX2_RFMUX_DEMOD_SEL_3, (s32)(tuner - 1));
		break;

		case 4:
			error |= ChipSetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX3_RFMUX_DEMOD_SEL_4, (s32)(tuner - 1));
		break;

		case 5:
			error |= ChipSetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX4_RFMUX_DEMOD_SEL_5, (s32)(tuner - 1));
		break;

		case 6:
			error |= ChipSetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX5_RFMUX_DEMOD_SEL_6, (s32)(tuner - 1));
		break;

		case 7:
			error |= ChipSetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX6_RFMUX_DEMOD_SEL_7, (s32)(tuner - 1));
		break;

		case 8:
			error |= ChipSetField(hChip, FLD_FC8CODEW_C8CODEW_RFMUX_RFMUX7_RFMUX_DEMOD_SEL_8, (s32)(tuner - 1));
		break;
	}

	if (hChip->Error) /*Check the error at the end of the function*/
		error = FE_LLA_I2C_ERROR;

	return(error);
}

fe_lla_error_t fe_stid135_set_rfmux_path(struct stv* state, FE_OXFORD_TunerPath_t tuner)
{
	enum fe_stid135_demod Demod = state->nr+1;
	STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	return fe_stid135_set_rfmux_path_(hChip, Demod, tuner);
}

/*
fe_lla_error_t fe_stid135_fskmodulatorctrl (struct stv* state,
			 bool fskmodulatorctrl)
{
}
fe_lla_error_t fe_stid135_configurefsk (struct fe_stid135_internal_param* pParams,
	enum fe_stid135_demod	demod, fe_stid135_modeparams_t fskparams)
{
}*/

/*****************************************************
--FUNCTION	::	fe_stid135_diseqc_init
--ACTION	::	Set the diseqC Tx mode
--PARAMS IN	::	handle -> Front End Handle
		::	tuner_nb -> Current tuner 1 .. 4
		::	diseqc_mode -> continues tone, 2/3 PWM, 3/3 PWM,
			2/3 envelop or 3/3 envelop.
--PARAMS OUT	::	None
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_diseqc_init(struct fe_stid135_internal_param* pParams,
		 FE_OXFORD_TunerPath_t tuner_nb,
		 enum fe_sat_diseqc_txmode diseqc_mode)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	struct fe_stid135_internal_param *pParams = &state->base->ip;
	s32 mode_fld, envelop_fld;
	u32 reg_value;

	if (pParams == NULL)
		return FE_LLA_INVALID_HANDLE;

	if (pParams->handle_demod->Error)
		return FE_LLA_I2C_ERROR;

	error |= Oxford_Disec_Lnb_FskStartup(pParams->handle_demod);

	switch(diseqc_mode) {
	case FE_SAT_22KHZ_Continues:
	case FE_SAT_DISEQC_2_3_PWM:
	case FE_SAT_DISEQC_3_3_PWM:
		mode_fld = diseqc_mode;
		envelop_fld = 0;
		break;
	case FE_SAT_22KHZ_Continues_ENVELOP:
		mode_fld = 0;
		envelop_fld = 1;
		break;
	case FE_SAT_DISEQC_2_3_ENVELOP:
		mode_fld = 2;
		envelop_fld = 1;
		break;
	case FE_SAT_DISEQC_3_3_ENVELOP:
		mode_fld = 3;
		envelop_fld = 1;
		break;
	default:
		return FE_LLA_BAD_PARAMETER;
		break;
	}
	vprintk("DISEQC[%d] error=%d mode=%d mode_fld=%d envelope_fld=%d", tuner_nb, error, diseqc_mode,
					mode_fld, envelop_fld);
	/* Set alternate function #1 */
	//p. 650 SYS_N_CONFIG1000 Alternative function output control for PIO0; only applies to tuners 2,3, 4
	switch(tuner_nb) {
	case AFE_TUNER2:
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, &reg_value);
		tst(pParams, reg_value, __func__, __LINE__);
		vprintk("DISEQC[%d] reg_value=0x%x error=%d", tuner_nb, reg_value, error);
		if(error) {
			dprintk("DISEQC[%d] ERROR reg_value=0x%x error=%d", tuner_nb, reg_value, error);
		} else {
			reg_value |= 0x00000001;
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, reg_value);
		}
	break;
	case AFE_TUNER3:
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, &reg_value);
		tst(pParams, reg_value, __func__, __LINE__);
		vprintk("DISEQC[%d] reg_value=%d error=%d", tuner_nb, reg_value, error);
		if(error) {
			dprintk("DISEQC[%d] ERROR reg_value=0x%x error=%d", tuner_nb, reg_value, error);
		} else {
			reg_value |= 0x00000010;
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, reg_value);
		}
	break;
	case AFE_TUNER4:
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, &reg_value);
		tst(pParams, reg_value, __func__, __LINE__);
		vprintk("DISEQC[%d] reg_value=%d error=%d", tuner_nb, reg_value, error);
		if(error) {
			dprintk("DISEQC[%d] ERROR reg_value=0x%x error=%d", tuner_nb, reg_value, error);
		} else {
			reg_value |= 0x00000100;
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, reg_value);
		}
		break;
	default:
		break;
	}
	vprintk("DISEQC[%d] error=%d", tuner_nb, error);
	error |= ChipSetField(pParams->handle_demod,
				FLD_FC8CODEW_DVBSX_LNBCTRL_LNB_VCTRL_VLNB_ON_EN, 1);
	error |= ChipSetField(pParams->handle_demod,
				FLD_FC8CODEW_DVBSX_DISEQC_DISTXCFG_DISEQC_MODE(tuner_nb),
				mode_fld);
	error |= ChipSetField(pParams->handle_demod,
				FLD_FC8CODEW_DVBSX_DISEQC_DISTXCFG_ENVELOP(tuner_nb),
				envelop_fld);
	error |=  ChipSetField(pParams->handle_demod,
				FLD_FC8CODEW_DVBSX_DISEQC_DISRXCFG_PINSELECT(AFE_TUNER1), 6);
	error |= ChipSetField(pParams->handle_demod,
				FLD_FC8CODEW_DVBSX_DISEQC_DISRXCFG_DISRX_ON(AFE_TUNER1), 1);
	error |= ChipSetField(pParams->handle_demod,
				FLD_FC8CODEW_DVBSX_DISEQC_DISTXCFG_DISTX_RESET(tuner_nb), 1);
	error |= ChipSetField(pParams->handle_demod,
				FLD_FC8CODEW_DVBSX_DISEQC_DISTXCFG_DISTX_RESET(tuner_nb), 0);
	vprintk("DISEQC[%d] error=%d", tuner_nb, error);

	/* Check the error at the end of the function */
	if(pParams->handle_demod->Error)
		error = FE_LLA_I2C_ERROR;

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_22khz_cont
--ACTION	::	Set 22KHz continues tone.
--PARAMS IN	::	handle -> Front End Handle
::	tuner_nb -> Current tuner 1 .. 4
::	tone -> continues tone on/off
--PARAMS OUT	::	None
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_22khz_cont(struct fe_stid135_internal_param* pParams,
						FE_OXFORD_TunerPath_t tuner_nb,
						BOOL tone)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = NULL;

	//pParams = (struct fe_stid135_internal_param*) handle;
	vprintk("DISEQC[%d] errror=%d abort=%d", tuner_nb, pParams->handle_demod->Error,
					pParams->handle_demod->Abort);
	if (pParams->handle_demod->Error) {
		dprintk("BUG!!!!!! ignoring error!");
		pParams->handle_demod->Error = false;
	}

	if (pParams->handle_demod->Abort) {
		dprintk("BUG!!!!!! ignoring abort!");
		pParams->handle_demod->Abort = false;
	}

	error |= ChipSetField(pParams->handle_demod,
		FLD_FC8CODEW_DVBSX_DISEQC_DISTXCFG_DISEQC_MODE(tuner_nb), tone ? FE_SAT_22KHZ_Continues : FE_SAT_DISEQC_2_3_PWM);
	vprintk("DISEQC[%d] tone=%d errror=%d", tuner_nb, tone, pParams->handle_demod->Error);
	/* Check the error at the end of the function */
	vprintk("DISEQC[%d] error=%d %d", tuner_nb, pParams->handle_demod->Error, error);
	if(pParams->handle_demod->Error)
		error = FE_LLA_I2C_ERROR;
	return error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_diseqc_send
--ACTION	::	Send bytes to DiseqC FIFO
--PARAMS IN	::	handle -> Front End Handle
			tuner_nb -> Current tuner 1 .. 4
		::	data -> Table of bytes to send.
		::	nbdata -> Number of bytes to send.
--PARAMS OUT	::	None
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_diseqc_send(struct fe_stid135_internal_param* pParams,
				FE_OXFORD_TunerPath_t tuner_nb, u8 *data, u8 nbdata)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = NULL;
	u32 i=0;
	s32 fld_value;
	vprintk("[%d] nbdata=%d pParams=%p",  tuner_nb, nbdata, pParams);
	//pParams = (struct fe_stid135_internal_param*) handle;
	if(pParams != NULL)
	{
		if(pParams->handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else
		{
			// We forbid receive part of tuner#1 when we send a Diseqc command (only RF#1 is diseqc-receive compliant)
			if(tuner_nb == AFE_TUNER1) {
				error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISRXCFG_DISRX_ON(AFE_TUNER1), 0);
			}

			error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISTXCFG_DIS_PRECHARGE(tuner_nb), 1);
			while(i < nbdata) {
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISTXSTATUS_TX_FIFO_FULL(tuner_nb), &fld_value);
				//vprintk("[%d] fld[%d]=%x",  tuner_nb, i, fld_value);
				while(fld_value) {
					error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISTXSTATUS_TX_FIFO_FULL(tuner_nb), &fld_value);
					//vprintk("[%d] fld[%d]=%x",  tuner_nb, i, fld_value);
				}
				BUG_ON(i<0 || i>=nbdata);
				//vprintk("[%d] data[%d]=%x",  tuner_nb, i, data[i]);
				error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DISEQC_DISTXFIFO(tuner_nb), data[i]);	/* send byte to FIFO :: WARNING don't use set field	!! */
				i++;
			}
			error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISTXCFG_DIS_PRECHARGE(tuner_nb), 0);

			i=0;
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISTXSTATUS_TX_IDLE(tuner_nb), &fld_value);
			//vprintk("[%d] lfd[%d]=%x",  tuner_nb, i, fld_value);
			while((fld_value != 1) && (i < 10)) {
				/*wait until the end of diseqc send operation */
				ChipWaitOrAbort(pParams->handle_demod, 10);
				i++;
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISTXSTATUS_TX_IDLE(tuner_nb), &fld_value);
				//vprintk("[%d] lfd[%d]=%x",  tuner_nb, i, fld_value);
			}

			if(tuner_nb == AFE_TUNER1) {
				error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISRXCFG_DISRX_ON(AFE_TUNER1), 1);
			}

			if(pParams->handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}
	}
	else
		error = FE_LLA_INVALID_HANDLE;
	//vprintk("[%d] error=%d",  tuner_nb, error);
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_diseqc_receive
--ACTION	::	Read received bytes from DiseqC FIFO
--PARAMS IN	::	handle -> Front End Handle
--PARAMS OUT	::	Data -> Table of received bytes.
		::	NbData -> Number of received bytes.
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_diseqc_receive(struct fe_stid135_internal_param* pParams,
	u8 *data, u8 *nbdata)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = NULL;
	u32 i=0;
	u32 reg_value;
	s32 fld_value;

	//pParams = (struct fe_stid135_internal_param*) handle;
		if(pParams->handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else
		{
			*nbdata=0;
			// Only the DiSEqC1 block has an Rx interface allowing full DiSEqC 2.x operation on this
			// interface, the three other blocks do not have receiver interfaces implemented.
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISRXSTAT1_RXEND(AFE_TUNER1), &fld_value);
			if(fld_value) {
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISRXBYTES_RXFIFO_BYTES(AFE_TUNER1), &fld_value);
				*nbdata = (u8)fld_value;
				for(i = 0;i < (*nbdata);i++)
				{
					error |= ChipGetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DISEQC_DISRXFIFO(AFE_TUNER1), &reg_value);
					data[i] = (u8)reg_value;
				}
			}
			if (pParams->handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_diseqc_reset
--ACTION	::	Resets both Tx and Rx blocks
--PARAMS IN	::	handle -> Front End Handle
--PARAMS OUT	::	tuner_nb -> Current tuner 1 .. 4
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_diseqc_reset(struct fe_stid135_internal_param* pParams, FE_OXFORD_TunerPath_t tuner_nb)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = NULL;

	//pParams = (struct fe_stid135_internal_param*) handle;
	if(pParams != NULL)
	{
		if(pParams->handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else
		{
			error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISTXCFG_DISTX_RESET(tuner_nb), 1);
			WAIT_N_MS(100);
			error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISTXCFG_DISTX_RESET(tuner_nb), 0);
			error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISRXCFG_DISRX_ON(AFE_TUNER1), 0);
			WAIT_N_MS(100);
			error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DISEQC_DISRXCFG_DISRX_ON(AFE_TUNER1), 1);
			if (pParams->handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}
	}
	else
		error = FE_LLA_INVALID_HANDLE;
	return error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_enable_pid
--ACTION	::	Enables given PID
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
			pid_number -> PID number to enable
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_enable_pid (struct stv* state, u32 pid_number)
{

	//STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod demod = state->nr+1;

	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, ram_index = 0;
	BOOL pid_already_output = FALSE;

		if (state->base->ip.handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			/* Check if pid already output */
			for(i=0;i<RAM_SIZE;i++) {
				if((state->pid_flt.ram_table[i].pid_number == pid_number) && (state->pid_flt.ram_table[i].command != NONE))
					pid_already_output = TRUE;
			}

			if(pid_already_output == FALSE) {
				/* Go to first free RAM location */
				for(i=0;i<RAM_SIZE;i++)
					if(state->pid_flt.ram_table[i].command == NONE) {
						ram_index = i;
						break;
					}
				/* Check if there is enough memory space */
				if(ram_index < RAM_SIZE - 2) {
					state->pid_flt.ram_table[ram_index].pid_number = pid_number;
					state->pid_flt.ram_table[ram_index + 1].pid_number = pid_number;
					/* Enable PID command takes exactly 2 bytes */
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->pid_flt.ram_table[ram_index].address);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), (state->pid_flt.ram_table[ram_index].pid_number | 0x8000) >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 1));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), (state->pid_flt.ram_table[ram_index].pid_number | 0x8000) & 0xFF);
					/* PID is no more dummy, but a real PID */
					state->pid_flt.ram_table[ram_index].command = ENABLE_PID;
					state->pid_flt.ram_table[ram_index + 1].command = ENABLE_PID;
				}
			}


			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error=FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_disable_pid
--ACTION	::	Disables given PID
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
			pid_number -> pid number to disable
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_disable_pid (struct stv* state, u32 pid_number)
{
	//STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod demod = state->nr+1;

	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, ram_index = 0;
	BOOL pid_already_output = FALSE;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			/* Check if pid already output */
			for(i=0;i<RAM_SIZE;i++) {
				if((state->pid_flt.ram_table[i].pid_number == pid_number)  && (state->pid_flt.ram_table[i].command != NONE))
				{
					pid_already_output = TRUE;
					ram_index = i;
					break;
				}
			}

			if(pid_already_output == TRUE) {
				/* Change pid already output to dummy PID */
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->pid_flt.ram_table[ram_index].address);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 1));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF & 0xFF);
				/* PID is no more a real PID, but a dummy PID */
				state->pid_flt.ram_table[ram_index].command = NONE;
				state->pid_flt.ram_table[ram_index].pid_number = 0;
				state->pid_flt.ram_table[ram_index + 1].command = NONE;
				state->pid_flt.ram_table[ram_index + 1].pid_number = 0;
			}

			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error=FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_disable_rangepid
--ACTION	::	Disables given range PID
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
			pid_start_range -> starting PID number to disable
			pid_stop_range -> ending PID number to disable
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_disable_rangepid (struct stv* state, u32 pid_start_range, u32 pid_stop_range)
{
	//STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, ram_index = 0;
	BOOL range_already_output = FALSE;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			/* Check if pid already output */
			for(i=0;i<RAM_SIZE-2;i++) {
				if((state->pid_flt.ram_table[i].command == SELECT_RANGE) && (state->pid_flt.ram_table[i].pid_number == pid_start_range) && (state->pid_flt.ram_table[i+2].pid_number == pid_stop_range))
				{
					range_already_output = TRUE;
					ram_index = i;
					break;
				}
			}
			if(range_already_output == TRUE) {
				/* Change pid already output to dummy PID */
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->pid_flt.ram_table[ram_index].address);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 1));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF & 0xFF);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 2));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 3));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF & 0xFF);
				/* PID is no more a real PID, but a dummy PID */
				state->pid_flt.ram_table[ram_index].command = NONE;
				state->pid_flt.ram_table[ram_index].pid_number = 0;
				state->pid_flt.ram_table[ram_index + 1].command = NONE;
				state->pid_flt.ram_table[ram_index + 1].pid_number = 0;
				state->pid_flt.ram_table[ram_index + 2].command = NONE;
				state->pid_flt.ram_table[ram_index + 2].pid_number = 0;
				state->pid_flt.ram_table[ram_index + 3].command = NONE;
				state->pid_flt.ram_table[ram_index + 3].pid_number = 0;
			}
			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error=FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_select_rangepid
--ACTION	::	Enables given range PID
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
			pid_start_range > starting PID number to enable
			pid_stop_range -> ending PID number to enable
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_select_rangepid (struct stv* state, u32 pid_start_range, u32 pid_stop_range)
{
	//STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, ram_index = 0;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
		if (state->base->ip.handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			/* Go to first free RAM location */
			for(i=0;i<RAM_SIZE;i++)
				if(state->pid_flt.ram_table[i].command == NONE) {
					ram_index = i;
					break;
				}
			/* Check if there is enough memory space */
			if(ram_index < RAM_SIZE - 4) {
				state->pid_flt.ram_table[ram_index].pid_number = pid_start_range;
				state->pid_flt.ram_table[ram_index + 1].pid_number = pid_start_range;
				state->pid_flt.ram_table[ram_index + 2].pid_number = pid_stop_range;
				state->pid_flt.ram_table[ram_index + 3].pid_number = pid_stop_range;
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->pid_flt.ram_table[ram_index].address);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), ((pid_stop_range | 0xC000) + 1) >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 1));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), ((pid_stop_range | 0xC000) + 1) & 0xFF);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 2));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), pid_start_range >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 3));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), pid_start_range & 0xFF);

				/* PID is no more dummy, but a real PID */
				state->pid_flt.ram_table[ram_index].command = SELECT_RANGE;
				state->pid_flt.ram_table[ram_index + 1].command = SELECT_RANGE;
				state->pid_flt.ram_table[ram_index + 2].command = SELECT_RANGE;
				state->pid_flt.ram_table[ram_index + 3].command = SELECT_RANGE;
			}

			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error=FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_enable_allpid
--ACTION	::	Enables all PIDs
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_enable_allpid (struct stv* state)
{
	//STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			/* All memory cells in RAM are reset, so update of table is necessary */
			for(i=0;i<RAM_SIZE>>1;i++) {
				state->pid_flt.ram_table[i].command = ENABLE_ALL_PID;
				state->pid_flt.ram_table[i].pid_number = 0;
			}
			// The filter RAM is erased to load a new program. All the packets are output waiting to go to 0x01 or 0x06
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x02);

			/* PASSALL command */
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0xA000 >> 8);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0xA000 & 0xFF);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0000 >> 8);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0000 & 0xFF);

			/* Activation with DELALL by default (0x01: H/W bug, ClearQuest opened) */
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x06);

			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error=FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_disable_allpid
--ACTION	::	Disables all PIDs
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_disable_allpid (struct stv* state)
{
	//STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			/* If this is the 1st time after power-up we perform this function or if a enable_all_pid has been performed, then we write into all memory cells in RAM */
			if((state->pid_flt.first_disable_all_command == TRUE) || (state->pid_flt.ram_table[0].command == ENABLE_ALL_PID)) {
				// The filter RAM is erased to load a new program. All the packets are output waiting to go to 0x01 or 0x06
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x03);

				for(i=0;i<RAM_SIZE;i++) {
					/* All PID are considered dummy PID */
					state->pid_flt.ram_table[i].command = NONE;
					state->pid_flt.ram_table[i].address = (u8)(i + 0x10);
					state->pid_flt.ram_table[i].pid_number = 0;
				}
				for(i=0;i<RAM_SIZE-1;i = (u8)(i + 2)) {
					/* Change all pid to dummy PID */
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->pid_flt.ram_table[i].address);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->pid_flt.ram_table[i+1].address);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF & 0xFF);
				}

				// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x06);

				state->pid_flt.first_disable_all_command = FALSE;
			} else {
			/* If this is NOT the 1st time we perform this function, then we write only into memory cells in RAM that do not output dummy PID */
			/* Aim is to avoid too much I2C traffic */
				for(i=0;i<RAM_SIZE - 1;i++) {
					if(state->pid_flt.ram_table[i].command != NONE) {
						/* All PID are considered dummy PID */
						state->pid_flt.ram_table[i].command = NONE;
						state->pid_flt.ram_table[i + 1].command = NONE;
						state->pid_flt.ram_table[i].pid_number = 0;
						state->pid_flt.ram_table[i + 1].pid_number = 0;
						/* Change all pid to dummy PID */
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->pid_flt.ram_table[i].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[i].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x9FFF & 0xFF);
						/* we have dealed with 2 memory cells, so we go further */
						i++;
					}
				}
			}

			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error=FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_changepid_number
--ACTION	::	Changes PID number from pid_inputnumber to pid_outputnumber
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
			pid_inputnumber -> PID number to modify
			pid_outputnumber -> new PID number
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_changepid_number (struct stv* state, u32 pid_inputnumber,u32 pid_outputnumber)
{
	//STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, ram_index = 0;
	u16 pid_offset;
	BOOL pid_already_output = FALSE;

	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			/* Check if pid already output */
			for(i=0;i<RAM_SIZE;i++) {
				if(state->pid_flt.ram_table[i].pid_number == pid_inputnumber)
				{
					pid_already_output = TRUE;
					ram_index = i;
					break;
				}
			}
			if(pid_already_output == TRUE) {
				/* Compute PID offset */
				if((s32)pid_outputnumber - (s32)pid_inputnumber < 0)
					pid_offset = (pid_outputnumber - pid_inputnumber) & 0x1FFF;
				else
					pid_offset = (u16)(pid_outputnumber - pid_inputnumber);
				/* Change PID number of current PID */
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->pid_flt.ram_table[ram_index].address);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), pid_inputnumber >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 1));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), pid_inputnumber & 0xFF);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 2));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), pid_offset >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->pid_flt.ram_table[ram_index].address + 3));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), pid_offset & 0xFF);

				state->pid_flt.ram_table[ram_index].command = CHANGE_PID;
			}

			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error=FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_enable_gselabel
--ACTION	::	Enables either 3-byte or 6-byte label
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
			gselabel_type -> 3-byte or 6-byte label
			gselabel -> value of label to enable
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_enable_gselabel(struct stv* state, enum label_type gselabel_type, u8 gselabel[6])
{
	//STCHIP_Info_t* hChip = state->base->ip.handle_demod;
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, j, k, ram_index;
	BOOL label_already_output = FALSE;
	BOOL ram_full;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			/* Check if label already output */
			if(gselabel_type == THREE_BYTE_LABEL) {
				for(i=(RAM_SIZE>>2);i < (RAM_SIZE>>1);i++) {
					if((state->gse_flt.ram_table[i].label[0] == gselabel[0])
					&& (state->gse_flt.ram_table[i].label[1] == gselabel[1])
					&& (state->gse_flt.ram_table[i].label[2] == gselabel[2]))
						label_already_output = TRUE;
				}
			}
			if(gselabel_type == SIX_BYTE_LABEL) {
				for(i=(RAM_SIZE>>1);i < RAM_SIZE;i++) {
					if((state->gse_flt.ram_table[i].label[0] == gselabel[0])
					&& (state->gse_flt.ram_table[i].label[1] == gselabel[1])
					&& (state->gse_flt.ram_table[i].label[2] == gselabel[2])
					&& (state->gse_flt.ram_table[i].label[3] == gselabel[3])
					&& (state->gse_flt.ram_table[i].label[4] == gselabel[4])
					&& (state->gse_flt.ram_table[i].label[5] == gselabel[5]))
						label_already_output = TRUE;
				}
			}
			/* if label not already output */
			if(label_already_output == FALSE) {
				/* Check if DISABLE_3BYTE_LABEL command is in second quarter of RAM */
				for(i=(RAM_SIZE>>2);i < (RAM_SIZE>>1);i++) {
					if(state->gse_flt.ram_table[i].command == DISABLE_3BYTE_LABEL)
					{
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 & 0xFF);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						/* Reflect GSE structure consequently - 6=number of modified bytes in RAM by this function */
						for(j=0;j<6;j++) {
							for(k=0;k<6;k++)
								state->gse_flt.ram_table[i+j].label[k] = 0;
							state->gse_flt.ram_table[i+j].command = NO_CMD;
						}
						/* Go to 6 address locations forward to avoid rewrite of memory cells */
						i = (u8)(i + 5); /* not 6 because of presence of i++ in loop */
					}
				}
				/* Check if DISABLE_6BYTE_LABEL command is in second half of RAM */
				for(i=(RAM_SIZE>>1);i < RAM_SIZE;i++) {
					if(state->gse_flt.ram_table[i].command == DISABLE_6BYTE_LABEL) {
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 & 0xFF);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 6));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 7));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						/* Reflect GSE structure consequently - 6=number of modified bytes in RAM by this function */
						for(j=0;j<8;j++) {
							for(k=0;k<6;k++)
								state->gse_flt.ram_table[i+j].label[k] = 0;
							state->gse_flt.ram_table[i+j].command = NO_CMD;
						}
						/* Go to 8 address locations forward to avoid rewrite of memory cells */
						i = (u8)(i + 7); /* not 8 because of presence of i++ in loop */
					}
				}
				if(gselabel_type == THREE_BYTE_LABEL) {
					/* Go to first free RAM location */
					ram_index = 0;
					for(i=(RAM_SIZE>>2);i <= (RAM_SIZE>>1) - 6;i++) { // check in second quarter of RAM
						if((state->gse_flt.ram_table[i].command == NO_CMD) || (state->gse_flt.ram_table[i].command == ENABLE_ALL_3BYTE_LABEL)){
							ram_index = i;
							break;
						}
					}
					if((ram_index == 0) && (i == (RAM_SIZE>>1) - 6 + 1))
						ram_full = TRUE;
					else
						ram_full = FALSE;
					/* Check if there is enough memory space */
					if((ram_index <= (RAM_SIZE>>1) - 6) && (ram_full == FALSE)){ // check in second quarter of RAM & if RAM not full
						/* Enable protocol command takes exactly 6 bytes for 3-byte label */
						/* We output 3-byte label */
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[ram_index].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 & 0xFF);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 2));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), gselabel[1]);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 3));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), gselabel[0]);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 4));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), gselabel[2]);
						// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x01);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTC(demod), 0x08);
						/* Reflect GSE structure consequently */
						for(i=0;i<6;i++) {
							for(j=0;j<3;j++)
								state->gse_flt.ram_table[ram_index+i].label[j] = gselabel[j];
							state->gse_flt.ram_table[ram_index+i].command = ENABLE_3BYTE_LABEL;
						}
					}
				}

				if(gselabel_type == SIX_BYTE_LABEL) {
					/* Go to first free RAM location */
					ram_index = 0;
					for(i=(RAM_SIZE>>1);i <= RAM_SIZE - 8;i++) { // check in second half of RAM
						if((state->gse_flt.ram_table[i].command == NO_CMD) || (state->gse_flt.ram_table[i].command == ENABLE_ALL_6BYTE_LABEL)){
							ram_index = i;
							break;
						}
					}
					if((ram_index == 0) && (i == RAM_SIZE - 8 + 1))
						ram_full = TRUE;
					else
						ram_full = FALSE;
					/* Check if there is enough memory space */
					if((ram_index <= RAM_SIZE - 8) && (ram_full == FALSE)){ // check in second half of RAM & if RAM not full
						/* Enable protocol command takes exactly 8 bytes for 6-byte label */
						/* We output 6-byte label */
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[ram_index].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 & 0xFF);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 2));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), gselabel[1]);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 3));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), gselabel[0]);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 4));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), gselabel[3]);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), gselabel[2]);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 6));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), gselabel[5]);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 7));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), gselabel[4]);
						// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x01);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTC(demod), 0x08);
						/* Reflect GSE structure consequently */
						for(i=0;i<8;i++) {
							for(j=0;j<6;j++)
								state->gse_flt.ram_table[ram_index+i].label[j] = gselabel[j];
							state->gse_flt.ram_table[ram_index+i].command = ENABLE_6BYTE_LABEL;
						}
					}
				}
			}
			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error)
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_enable_protocoltype
--ACTION	::	Enables protocol type GSE frame
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
			protocoltype -> value of protocol to enable
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_enable_protocoltype(struct stv* state, u16 protocoltype)
{
	enum fe_stid135_demod demod = state->nr+1;

	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, j, ram_index = 0, ram_full;
	BOOL protocol_already_output = FALSE;

		if (state->base->ip.handle_demod->Error )
			error = FE_LLA_I2C_ERROR;
		else {
			/* Check if protocol already output */
			for(i=0;i < (RAM_SIZE>>1);i++) {
				if((state->gse_flt.ram_table[i].protocol_number == protocoltype) && (state->gse_flt.ram_table[i].command != NO_CMD))
					protocol_already_output = TRUE;
			}
			/* if protocol not already output */
			if(protocol_already_output == FALSE) {
				/* Check if DISABLE_ALL_PROTOCOL command is in RAM */
				for(i=0;i < (RAM_SIZE>>2);i++) { // we check if protocol is already in first quarter of RAM
					if(state->gse_flt.ram_table[i].command == DISABLE_ALL_PROTOCOL)
					{
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 & 0xFF);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						/* Reflect GSE structure consequently - 6=number of modified bytes in RAM by this function */
						for(j=0;j<6;j++) {
							state->gse_flt.ram_table[i+j].protocol_number = 0;
							state->gse_flt.ram_table[i+j].command = NO_CMD;
						}
						/* Go to 6 address locations forward to avoid rewrite of memory cells */
						i = (u8)(i + 5); /* not 6 because of presence of i++ in loop */
					}
				}
				/* Go to first free RAM location */
				for(i=0;i < (RAM_SIZE>>2) - 6;i++) { // check in first quarter of RAM
					if((state->gse_flt.ram_table[i].command == NO_CMD) || (state->gse_flt.ram_table[i].command == ENABLE_ALL_PROTOCOL)) {
						ram_index = i;
						break;
					}
				}
				if((ram_index == 0) && (i == (RAM_SIZE>>2) - 6))
					ram_full = TRUE;
				else
					ram_full = FALSE;

				/* Check if there is enough memory space */
				//if(ram_index <= (RAM_SIZE>>1) - 12) {
				if((ram_index <= (RAM_SIZE>>2) - 12) && (ram_full == FALSE)){ // check in first quarter of RAM & if RAM not full
					/* Enable protocol command takes exactly 6 bytes for 3/6-byte label protocol and 6 bytes for broadcast protocol */
					/* We output 3/6-byte label protocol */
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[ram_index].address);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 1));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 2));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), protocoltype >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 3));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), protocoltype & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 4));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 5));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					/* We output broadcast protocol */
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 6));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0540 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 7));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0540 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 8));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), protocoltype >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 9));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), protocoltype & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 10));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 11));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x01);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTC(demod), 0x08);
					/* Reflect GSE structure consequently */
					for(i=0;i<12;i++) {
						state->gse_flt.ram_table[ram_index+i].protocol_number = protocoltype;
						state->gse_flt.ram_table[ram_index+i].command = ENABLE_PROTOCOL;
					}
				}
			}
			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error)
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_disable_gselabel
--ACTION	::	Disables either 3-byte or 6-byte label
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
			gselabel -> value of label to enable
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_disable_gselabel(struct stv* state, u8 gselabel[6])
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, j, ram_index = 0;
	enum label_type gselabel_type = THREE_BYTE_LABEL;
	BOOL label_already_output = FALSE;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		/* Check if label already output */
		for(i=(RAM_SIZE>>2);i < (RAM_SIZE>>1);i++) {
			if((state->gse_flt.ram_table[i].label[0] == gselabel[0])
			&& (state->gse_flt.ram_table[i].label[1] == gselabel[1])
			&& (state->gse_flt.ram_table[i].label[2] == gselabel[2])) {
				label_already_output = TRUE;
				gselabel_type = THREE_BYTE_LABEL;
				ram_index = i;
				break;
			}
		}
		for(i=(RAM_SIZE>>1);i < RAM_SIZE;i++) {
			if((state->gse_flt.ram_table[i].label[0] == gselabel[0])
			&& (state->gse_flt.ram_table[i].label[1] == gselabel[1])
			&& (state->gse_flt.ram_table[i].label[2] == gselabel[2])
			&& (state->gse_flt.ram_table[i].label[3] == gselabel[3])
			&& (state->gse_flt.ram_table[i].label[4] == gselabel[4])
			&& (state->gse_flt.ram_table[i].label[5] == gselabel[5])) {
				label_already_output = TRUE;
				gselabel_type = SIX_BYTE_LABEL;
				ram_index = i;
				break;
			}
		}
		/* if label already output */
		if(label_already_output == TRUE) {
			if(gselabel_type == THREE_BYTE_LABEL) {
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[ram_index].address);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 1));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 & 0xFF);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 2));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 3));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 4));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 5));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				/* Reflect GSE structure consequently */
				for(i=0;i<6;i++) {
					for(j=0;j<3;j++)
						state->gse_flt.ram_table[ram_index+i].label[j] = 0;
					state->gse_flt.ram_table[ram_index+i].command = NO_CMD;
				}
			}
			if(gselabel_type == SIX_BYTE_LABEL) {
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[ram_index].address);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 1));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 & 0xFF);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 2));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 3));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 4));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 5));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 6));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 7));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				/* Reflect GSE structure consequently */
				for(i=0;i<8;i++) {
					for(j=0;j<6;j++)
						state->gse_flt.ram_table[ram_index+i].label[j] = 0;
					state->gse_flt.ram_table[ram_index+i].command = NO_CMD;
				}
			}

			// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x06);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTC(demod), 0x08);

			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_disable_protocoltype
--ACTION	::	Disables protocol type GSE frame
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
			protocoltype -> value of protocol to enable
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_disable_protocoltype(struct stv* state, u16 protocoltype)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, j, ram_index = 0;
	BOOL protocol_already_output = FALSE;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

	if (state->base->ip.handle_demod->Error )
			error = FE_LLA_I2C_ERROR;
		else {
			/* Check if protocol already output */
			for(i=0;i < (RAM_SIZE>>1);i++) {
				if((state->gse_flt.ram_table[i].protocol_number == protocoltype) && (state->gse_flt.ram_table[i].command != NO_CMD))
				{
					protocol_already_output = TRUE;
					ram_index = i;
					break;
				}
			}

			if(protocol_already_output == TRUE) {
				/* Check if DISABLE_ALL_PROTOCOL command is in RAM : REALLY MANDATORY ??? */
				for(i=0;i < (RAM_SIZE>>2);i++) { // check in first quarter of RAM
					if(state->gse_flt.ram_table[i].command == DISABLE_ALL_PROTOCOL)
					{
						/* Change from disable all protocols to room booking */
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x6500 >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x6500 & 0xFF);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						/* Reflect GSE structure consequently */
						for(j=0;j<6;j++) {
							state->gse_flt.ram_table[i+j].protocol_number = 0;
							state->gse_flt.ram_table[i+j].command = NO_CMD;
						}
					}
				}
				/* Change protocol already output to room booking */
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[ram_index].address);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 1));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 & 0xFF);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 2));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 3));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 4));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 5));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);

				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 6));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0540 >> 8);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 7));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0540 & 0xFF);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 8));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 9));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 10));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[ram_index].address + 11));
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
				// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x06);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTC(demod), 0x08);
				/* Reflect GSE structure consequently */
				for(i=0;i<12;i++) {
					state->gse_flt.ram_table[ram_index+i].protocol_number = 0;
					state->gse_flt.ram_table[ram_index+i].command = NO_CMD;
				}
}

			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error)
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_enable_allgselabel
--ACTION	::	Enables all label GSE frame
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_enable_allgselabel(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, j, k;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error )
			error = FE_LLA_I2C_ERROR;
		else {
			/* Search all 3-byte label already output */
			for(i=(RAM_SIZE>>2);i < (RAM_SIZE>>1) - 6;i++) { // check in second quarter of RAM
				if(state->gse_flt.ram_table[i].command == ENABLE_3BYTE_LABEL) {
					/* Change 3-byte label already output to dummy label */
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);

					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 6));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 7));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 8));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 9));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 10));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 11));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					/* Reflect GSE structure consequently */
					for(j=0;j<12;j++) {
						for(k=0;k<6;k++)
							state->gse_flt.ram_table[i+j].label[k] = 0;
						state->gse_flt.ram_table[i+j].command = NO_CMD;
					}
					/* Go to 12 address locations forward to avoid rewrite of memory cells*/
					i = (u8)(i + 11); /* not 12 because of presence of i++ in loop */
				}
}
			/* Search all 6-byte label already output */
			for(i=(RAM_SIZE>>1);i < RAM_SIZE - 6;i++) { // check in second half of RAM
				if(state->gse_flt.ram_table[i].command == ENABLE_6BYTE_LABEL) {
					/* Change 3-byte label already output to dummy label */
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 6));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 7));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);

					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 8));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 9));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 10));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 11));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 12));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 13));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 14));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 15));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					/* Reflect GSE structure consequently */
					for(j=0;j<16;j++) {
						for(k=0;k<6;k++)
							state->gse_flt.ram_table[i+j].label[k] = 0;
						state->gse_flt.ram_table[i+j].command = NO_CMD;
					}
					/* Go to 16 address locations forward to avoid rewrite of memory cells*/
					i = (u8)(i + 15); /* not 16 because of presence of i++ in loop */
				}
			}
			/* Output labels different from zero ie output all 3-byte label */
			/* Set RAM pointer to first free RAM cell of second quarter of RAM */
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x10+(RAM_SIZE>>2));

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x1320 >> 8);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x1320 & 0xFF);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);

			/* Output labels different from zero ie output all 6-byte label */
			/* Set RAM pointer to first free RAM cell of second hald of RAM */
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x10+(RAM_SIZE>>1));

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x1720 >> 8);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x1720 & 0xFF);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);

			for(i=(RAM_SIZE>>2);i<6+(RAM_SIZE>>2);i++) {
				for(k=0;k<6;k++)
					state->gse_flt.ram_table[i].label[k] = 0;
				state->gse_flt.ram_table[i].command = ENABLE_ALL_3BYTE_LABEL;
			}
			for(i=(RAM_SIZE>>1);i<8+(RAM_SIZE>>1);i++) {
				for(k=0;k<6;k++)
					state->gse_flt.ram_table[i].label[k] = 0;
				state->gse_flt.ram_table[i].command = ENABLE_ALL_6BYTE_LABEL;
			}
			// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x06);

			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error = FE_LLA_I2C_ERROR;
		}
	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_disable_allgselabel
--ACTION	::	Disables all label GSE frame
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_disable_allgselabel (struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, j, k;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error )
			error = FE_LLA_I2C_ERROR;
		else {
			/* If this is the 1st time after power-up we perform this function, then we reset members of gse structure */
			if(state->gse_flt.first_disable_all_label_command == TRUE) {
				for(i=0;i < RAM_SIZE;i++) {
					state->gse_flt.ram_table[i].address = (u8)(i + 0x10);
				}
				for(i=(RAM_SIZE>>2); i <= (RAM_SIZE>>1);i++) { // fill-in second quarter of RAM
					state->gse_flt.ram_table[i].command = NO_CMD;
					for(j=0;j<6;j++)
						state->gse_flt.ram_table[i].label[j] = 0;
				}
				for(i=(RAM_SIZE>>1); i < RAM_SIZE;i++) { // fill-in second quarter of RAM
					state->gse_flt.ram_table[i].command = NO_CMD;
					for(j=0;j<6;j++)
						state->gse_flt.ram_table[i].label[j] = 0;
				}
				state->gse_flt.first_disable_all_label_command = FALSE;
				/* 192bytes/4/6 because 3-byte labels are only on second quarter and command on 6 bytes */
				for(i=(RAM_SIZE>>2);i <= (RAM_SIZE>>1) - 6;i++) {
					/* Set RAM pointer to the beginning of the second quarter */
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(0x10+i));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					/* Go to 6 address locations forward to avoid rewrite of memory cells */
					i = (u8)(i + 5); /* not 6 because of presence of i++ in loop */
				}
				/* 192bytes/2/6 because 6-byte labels are only on second half and command on 8 bytes */
				for(i=(RAM_SIZE>>1);i <= (11*RAM_SIZE/12) - 8;i++) {
					/* Set RAM pointer to the beginning of the second half */
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(0x10+i));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					/* Go to 8 address locations forward to avoid rewrite of memory cells */
					i = (u8)(i + 7); /* not 8 because of presence of i++ in loop */
				}
			} else {
				/* Search all 3-byte label already output */
				for(i=(RAM_SIZE>>2);i <= (RAM_SIZE>>1) - 6;i++) { // check in second quarter of RAM
					if((state->gse_flt.ram_table[i].command == ENABLE_3BYTE_LABEL) || (state->gse_flt.ram_table[i].command == ENABLE_ALL_3BYTE_LABEL)) {
						/* Output dummy 3-byte label */
						/* Output 3-byte label 0x000000 instead */
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0720 & 0xFF);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						/* Reflect GSE structure consequently */
						for(j=0;j<6;j++) {
							for(k=0;k<3;k++)
								state->gse_flt.ram_table[i+j].label[k] = 0;
							state->gse_flt.ram_table[i+j].command = NO_CMD;
						}
						/* Go to 6 address locations forward to avoid rewrite of memory cells */
						i = (u8)(i + 5); /* not 6 because of presence of i++ in loop */
					}
				}
				/* Search all 6-byte label already output */
				for(i=(RAM_SIZE>>1);i <= (11*RAM_SIZE/12) - 8;i++) { // check in second half of RAM
					if((state->gse_flt.ram_table[i].command == ENABLE_6BYTE_LABEL) || (state->gse_flt.ram_table[i].command == ENABLE_ALL_6BYTE_LABEL)) {
						/* Output dummy 6-byte label */
						/* Output 6-byte label 0x000000000000 instead */
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0320 & 0xFF);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						/* Reflect GSE structure consequently */
						for(j=0;j<8;j++) {
							for(k=0;k<3;k++)
								state->gse_flt.ram_table[i+j].label[k] = 0;
							state->gse_flt.ram_table[i+j].command = NO_CMD;
						}
						/* Go to 6 address locations forward to avoid rewrite of memory cells */
						i = (u8)(i + 7); /* not 8 because of presence of i++ in loop */
					}
				}
			}
			// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x06);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTC(demod), 0x08);
		}
		//Check the error at the end of the function
		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_enable_allprotocoltype
--ACTION	::	Enables all protocol GSE frame
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_enable_allprotocoltype(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, j/*, ram_index = 0*/;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
		if (state->base->ip.handle_demod->Error )
			error = FE_LLA_I2C_ERROR;
		else {
			/* Search all protocol already output */
			for(i=0;i < (RAM_SIZE>>2) - 6;i++) { // check in first quarter of RAM
				if(state->gse_flt.ram_table[i].command == ENABLE_PROTOCOL) {
					/* Change protocol already output to room booking */
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);

					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 6));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 7));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 8));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 9));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 10));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 11));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					/* Reflect GSE structure consequently */
					for(j=0;j<12;j++) {
						state->gse_flt.ram_table[i+j].protocol_number = 0;
						state->gse_flt.ram_table[i+j].command = NO_CMD;
					}
					/* Go to 6 address locations forward to avoid rewrite of memory cells*/
					i = (u8)(i + 11); /* not 12 because of presence of i++ in loop */
				}
			}
			/* Output protocols different from zero ie output all 3/6-byte label protocols */
			/* Set RAM pointer to first free RAM cell */
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x10);
			/* Output protocols different from zero ie output all 3/6-byte label protocols */
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x1520 >> 8);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x1520 & 0xFF);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			/* Output protocols different from zero ie output all broadcast protocols */
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x1540 >> 8);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x1540 & 0xFF);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			for(i=0;i<12;i++) {
				state->gse_flt.ram_table[i].protocol_number = 0;
				state->gse_flt.ram_table[i].command = ENABLE_ALL_PROTOCOL;
			}
			// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x06);
			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_disable_allprotocoltype
--ACTION	::	Disables all protocol GSE frame
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_disable_allprotocoltype(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, j;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error )
			error = FE_LLA_I2C_ERROR;
		else {
			/* If this is the 1st time after power-up we perform this function, then we reset members of gse structure */
			if(state->gse_flt.first_disable_all_protocol_command == TRUE) {
				for(i=0;i < (RAM_SIZE>>2);i++) { // check in first quarter of RAM
					state->gse_flt.ram_table[i].command = NO_CMD;
					state->gse_flt.ram_table[i].address = (u8)(i + 0x10);
					state->gse_flt.ram_table[i].protocol_number = 0;
				}
				for(i=(RAM_SIZE>>2);i < RAM_SIZE;i++) {
					state->gse_flt.ram_table[i].address = (u8)(i + 0x10);
				}
				state->gse_flt.first_disable_all_protocol_command = FALSE;
				/* RAM start from 0x10, not from 0x00 */
				/* Set RAM pointer to the beginning */
				/* 192bytes/4/6 because protocols are only on first quarter and command on 6 bytes */
				for(i=0;i < (RAM_SIZE>>2);i++) {
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(0x10 + i));
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 >> 8);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 & 0xFF);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
					/* Go to 6 address locations forward to avoid rewrite of memory cells */
					i = (u8)(i + 5); /* not 6 because of presence of i++ in loop */
				}
			} else {
				/* Search all protocol already output */
				for(i=0;i <= (RAM_SIZE>>2) - 6;i++) { // check in first quarter of RAM
				if((state->gse_flt.ram_table[i].command == ENABLE_PROTOCOL) || (state->gse_flt.ram_table[i].command == ENABLE_ALL_PROTOCOL)) {
						/* Change protocol already output to room booking */
						/* not room booking, but output protocol 0x0000 instead */
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), state->gse_flt.ram_table[i].address);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 >> 8);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 1));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x0520 & 0xFF);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 2));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 3));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 4));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), (u32)(state->gse_flt.ram_table[i].address + 5));
						error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
						/* Reflect GSE structure consequently */
						for(j=0;j<6;j++) {
							state->gse_flt.ram_table[i+j].protocol_number = 0;
							state->gse_flt.ram_table[i+j].command = NO_CMD;
						}
						/* Go to 6 address locations forward to avoid rewrite of memory cells */
						i = (u8)(i + 5); /* not 6 because of presence of i++ in loop */
					}
				}
			}
			// Activation with DELALL by default (0x01: H/W bug, ClearQuest opened)
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x06);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTC(demod), 0x08);

			//Check the error at the end of the function
			if (state->base->ip.handle_demod->Error )
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_select_ncr_source
--ACTION	::	Outputs NCR on TSout0
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_select_ncr_source(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
#ifdef USER1
			/* Select which TS has to be copied on TSout0 (TSmux) */
			error |= ChipSetField(pParams->handle_soc, FLD_FSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG160_TSMUX_SEL, (s32)(demod - 1));
			/* RCS feature association to selected demod - new management of NCR */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSRCSSEL_TSRCS_SEL, (s32)(demod - 1));
#else
			/* Set stream output frequency */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG1_TSFIFO_MANSPEED(demod), 0);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSSPEED(demod), 0x04);
			/* Make SUBNCR0 signal longer */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSRCSCFG_NCROUT_CLKLEN(demod), 2);
			/* Added a sync byte to trigg on when sniffing packets */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSINSDELH_TSINSDEL_SYNCBYTE(demod), 1);
			/* Do not embed in DVB NCR packets! */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_EMBINDVB(demod), 0);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATE1_TSOUT_NOSYNC(demod), 1);
#endif
		}
	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_unselect_ncr
--ACTION	::	Do not output anymore NCR on TSout0
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_unselect_ncr(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
		if (state->base->ip.handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			/* Unselect which TS has to be copied on TSout0 (TSmux) */
			error |= ChipSetField(state->base->ip.handle_soc, FLD_FSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG160_TSMUX_SEL, 0);

			/* Remove RCS feature association to demod - new management of NCR */
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_C8CODEW_TOP_CTRL_TSRCSSEL, 0);

			/* Set default value for SUBNCR0 signal */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSRCSCFG_NCROUT_CLKLEN(demod), 0);
		}
	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_ncr_lock_status
--ACTION	::	Gets status of NCR
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NCR status
--RETURN	::	error
--***************************************************/
fe_lla_error_t fe_stid135_get_ncr_lock_status(struct stv* state, BOOL* ncr_lock_status_p)
{
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	enum fe_stid135_demod demod = state->nr+1;
	s32 fld_value[2];
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	*ncr_lock_status_p = FALSE;

		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS3_PCRCALC_NCRREADY(demod), &(fld_value[0]));
		error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSSTATUS3_PCRCALC_ERROR(demod), &(fld_value[1])) ;
		if((fld_value[0] == 2) && (fld_value[1] == 0x0))
			*ncr_lock_status_p = TRUE;
		else
			*ncr_lock_status_p = FALSE;
	return(error);
}

/*
fe_lla_error_t fe_stid135_enable_irq (STCHIP_Info_t* handle,
			fe_stid135_irq_t irq_list)
{
}
fe_lla_error_t fe_stid135_disable_irq (STCHIP_Info_t* handle,
				fe_stid135_irq_t irq_list)
{
}
*/

/*****************************************************
--FUNCTION	::	fe_stid135_set_ts_parallel_serial
--ACTION	::	Sets PIOs according to wanted TS mode
--PARAMS IN	::	handle -> Pointer to the chip structure
			demod -> Current demod 1..8
			ts_mode -> TS mode for instance serial, parallel...
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_ts_parallel_serial(struct fe_stid135_internal_param *pParams, enum fe_stid135_demod demod, enum fe_ts_output_mode ts_mode)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	u32 reg_value;

		if (pParams->handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			switch(ts_mode) {
				case FE_TS_SERIAL_PUNCT_CLOCK:
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_SERIAL(demod), 1); /* Serial mode */
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(demod), 0);  /* punctured clock */
					error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSCFG2(demod), 1);
					if(demod == FE_SAT_DEMOD_1) {
						// Setting alternate function 2 from PIO4_0 to PIO4_2
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, &reg_value);
						reg_value |= 0x00000222;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, reg_value);

						// Setting output enable from PIO4_0 to PIO4_2
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x00000700;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

						// Setting clk not data for PIO4_0
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1108, &reg_value);
						reg_value |= 0x00000004;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1108, reg_value);
					}
					else if(demod == FE_SAT_DEMOD_2) {
						// Setting alternate function 2 from PIO4_3 to PIO4_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, &reg_value);
						reg_value |= 0x00222000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, reg_value);

						// Setting output enable from PIO4_3 to PIO4_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x00003800;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

						// Setting clk not data for PIO4_3
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1111, &reg_value);
						reg_value |= 0x00000004;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1111, reg_value);
					}
					else if(demod == FE_SAT_DEMOD_3) {
						// Setting alternate function 2 from PIO5_0 to PIO5_2
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, &reg_value);
						reg_value |= 0x00000222;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, reg_value);

						// Setting output enable from PIO5_0 to PIO5_2
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x00070000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

						// Setting clk not data for PIO5_0
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1116, &reg_value);
						reg_value |= 0x00000004;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1116, reg_value);
					}
					else if(demod == FE_SAT_DEMOD_4) {
						// Setting alternate function 2 from PIO5_3 to PIO5_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, &reg_value);
						reg_value |= 0x00222000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, reg_value);

						// Setting output enable from PIO5_3 to PIO5_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x00380000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

						// Setting clk not data for PIO5_3
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1119, &reg_value);
						reg_value |= 0x00000004;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1119, reg_value);
					}
					else if(demod == FE_SAT_DEMOD_5) {
						// Setting alternate function 2 from PIO6_0 to PIO6_2
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, &reg_value);
						reg_value |= 0x00000222;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, reg_value);

						// Setting output enable from PIO6_0 to PIO6_2
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x07000000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

						// Setting clk not data for PIO6_0
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1124, &reg_value);
						reg_value |= 0x00000004;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1124, reg_value);
					}
					else if(demod == FE_SAT_DEMOD_6) {
						// Setting alternate function 2 from PIO6_3 to PIO6_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, &reg_value);
						reg_value |= 0x00222000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, reg_value);

						// Setting output enable from PIO6_3 to PIO6_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x38000000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

						// Setting clk not data for PIO6_3
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1127, &reg_value);
						reg_value |= 0x00000004;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1127, reg_value);
					}
					else if(demod == FE_SAT_DEMOD_7) {
						// Setting alternate function 2 from PIO7_0 to PIO7_2
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, &reg_value);
						reg_value |= 0x00000222;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, reg_value);

						// Setting output enable from PIO7_0 to PIO7_2
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x07000000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, reg_value);

						// Setting clk not data for PIO7_0
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1124, &reg_value);
						reg_value |= 0x00000004;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1124, reg_value);
					}
					else if(demod == FE_SAT_DEMOD_8) {
						// Setting alternate function 2 from PIO7_3 to PIO7_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, &reg_value);
						reg_value |= 0x00222000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, reg_value);

						// Setting output enable from PIO7_3 to PIO7_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x38000000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, reg_value);

						// Setting clk not data for PIO7_3
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1127, &reg_value);
						reg_value |= 0x00000004;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1127, reg_value);
					}
					// Disable leaky packet mechanism, the bandwidth is free to be as low as possible
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPIDFLTC_PIDFLT_LEAKPKT(demod), 0);
					// Remove bug fix: workaround of BZ#98230: bad sampling of data[3] of TS bus
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(demod), 0);
				break;

				case FE_TS_SERIAL_CONT_CLOCK:
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_SERIAL(demod), 1); /* Serial mode */
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(demod), 1);  /* continues clock */
					error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSCFG2(demod), 1);

#if 1
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1000, &reg_value);
					reg_value |= 0x00001111;
					error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1000, reg_value);

					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, &reg_value);
					if(reg_value!=0x00111111){
						reg_value |= 0x00111111;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, reg_value);
					}

					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, &reg_value);
					if(reg_value!=0x00111111){
						reg_value |= 0x00111111;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, reg_value);
					}

					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, &reg_value);
					if(reg_value!=0x00111111){
						reg_value |= 0x00111111;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, reg_value);
					}

					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, &reg_value);
					if(reg_value!=0x00111111){
						reg_value |= 0x00111111;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, reg_value);
					}

					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
					if(reg_value!=0x3F3F3F00){
						reg_value |= 0x3F3F3F00;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);
					}
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, &reg_value);
					if(reg_value!=0x3F000000){
						reg_value |= 0x3F000000;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, reg_value);
					}

					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1050, &reg_value);
					if(reg_value!=0xFF000000){
						reg_value |= 0xFF000000;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1050, reg_value);
					}

					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1108, &reg_value);
					if(reg_value!=0x00000004){
						reg_value |= 0x00000004;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1108, reg_value);
					}

					// Setting clk not data for PIO4_3
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1111, &reg_value);
					if(reg_value!=0x00000004){
						reg_value |= 0x00000004;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1111, reg_value);
					}
					// Setting clk not data for PIO5_0
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1116, &reg_value);
					if(reg_value!=0x00000004){
						reg_value |= 0x00000004;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1116, reg_value);
					}
					// Setting clk not data for PIO5_3
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1119, &reg_value);
					if(reg_value!=0x00000004){
						reg_value |= 0x00000004;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1119, reg_value);
					}
					// Setting clk not data for PIO5_3
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1119, &reg_value);
					if(reg_value!=0x00000004){
						reg_value |= 0x00000004;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1119, reg_value);
					}
					// Setting clk not data for PIO6_0
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1124, &reg_value);
					if(reg_value!=0x00000004){
						reg_value |= 0x00000004;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1124, reg_value);
					}
					// Setting clk not data for PIO6_3
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1127, &reg_value);
					if(reg_value!=0x00000004){
						reg_value |= 0x00000004;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1127, reg_value);
					}
					// Setting clk not data for PIO7_0
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1124, &reg_value);
					if(reg_value!=0x00000004){
						reg_value |= 0x00000004;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1124, reg_value);
					}
					// Setting clk not data for PIO7_3
					error = ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1127, &reg_value);
					if(reg_value!=0x00000004){
						reg_value |= 0x00000004;
						error = ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1127, reg_value);
					}
#endif

					// Disable leaky packet mechanism, the bandwidth is free to be as low as possible
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPIDFLTC_PIDFLT_LEAKPKT(demod), 0);
					// Remove bug fix: workaround of BZ#98230: bad sampling of data[3] of TS bus
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(demod), 0);
				break;

				case FE_TS_PARALLEL_PUNCT_CLOCK:
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_SERIAL(demod), 0); /* Parallel mode */
					//error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(demod), 0);  /* punctured clock */
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(demod), 1);  /* contiunous clock */
					error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSCFG2(demod), 1);
					/* Only demod#1 and demod#3 are able to output on SOC PIOs
					It is a FPGA choice/limitation
					One option to output any demod to SOC PIOs is to use mux
					between demod and packet delineator */
					if(demod == FE_SAT_DEMOD_1) {
						// Setting alternate function 4 from PIO4_0 to PIO4_5
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, 0x00444444);
						// Setting alternate function 4 from PIO5_0 to PIO5_5
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, 0x00444444);

						// Setting output enable from PIO4_0 to PIO4_5 and from PIO5_0 to PIO5_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x003F3F00;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

						// Setting clk not data for PIO4_0
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1108, 0x00000004);

						// Setting data not clk for PIO4_3, PIO5_0 and PIO5_3
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1111, 0x00000000);
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1116, 0x00000000);
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1119, 0x00000000);
					}
					if(demod == FE_SAT_DEMOD_3) {
						// Setting alternate function 4 from PIO6_0 to PIO6_5
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, 0x00444444);
						// Setting alternate function 4 from PIO7_0 to PIO7_5
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, 0x00444444);

						// Setting output enable from PIO6_0 to PIO6_5
						error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
						reg_value |= 0x3F000000;
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);
						// Setting output enable from PIO7_0 to PIO7_5
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, 0x3F000000);

						// Setting clk not data for PIO6_0 (normally 0x4, but bug, workaround = 0x6)
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1124, 0x00000006);

						// Setting data not clk for PIO6_3, PIO7_0 and PIO7_3
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1127, 0x00000000);
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1123, 0x00000000);
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1127, 0x00000000);
					}
					// Disable leaky packet mechanism, the bandwidth is free to be as low as possible
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPIDFLTC_PIDFLT_LEAKPKT(demod), 0);
					// Bug fix: workaround of BZ#98230: bad sampling of data[3] of TS bus
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(demod), 3);
				break;

				case FE_TS_DVBCI_CLOCK:
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_SERIAL(demod), 0); /* Parallel mode */
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(demod), 1);  /* continues clock */
					error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSCFG2(demod), 1);
					// Disable leaky packet mechanism, the bandwidth is free to be as low as possible
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPIDFLTC_PIDFLT_LEAKPKT(demod), 0);
					// Remove bug fix: workaround of BZ#98230: bad sampling of data[3] of TS bus
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(demod), 0);
				break;

				case FE_TS_PARALLEL_ON_TSOUT_0:
					// Setting alternate function 3 from PIO4_0 to PIO4_5
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, 0x00333333);
					// Setting alternate function 3 from PIO5_0 to PIO5_5
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, 0x00333333);

					// Setting output enable from PIO4_0 to PIO4_5 and from PIO5_0 to PIO5_5
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
					reg_value |= 0x003F3F00;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, /*0x003F3F00*/reg_value);

					// Setting clk not data for PIO4_0 and clock inversion (BZ#75008)
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1108, 0x00000205);

					// Setting data not clk for pio from PIO4_1 to PIO4_5 and for pio from PIO5_0 to PIO5_4, and choose STFE clock as a clock source for retime block
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1109, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1110, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1111, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1112, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1113, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1116, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1117, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1118, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1119, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1120, 0x00000401);
				break;
				case FE_TS_PARALLEL_ON_TSOUT_1:
					// Setting alternate function 3 from PIO6_0 to PIO6_5
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, 0x00333333);
					// Setting alternate function 3 from PIO7_0 to PIO7_5
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, 0x00333333);

					// Setting output enable from PIO6_0 to PIO6_5
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
					reg_value |= 0x3F000000;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);
					// Setting output enable from PIO7_0 to PIO7_5
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, 0x3F000000);

					// Setting clk not data for PIO6_0 and clock inversion (BZ#75008)
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1124, 0x00000205);

					// Setting data not clk for pio from PIO6_0 to PIO6_5 and for pio from PIO7_0 to PIO7_4, and choose STFE clock as a clock source for retime block
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1125, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1126, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1127, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1128, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1129, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1124, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1125, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1126, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1127, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1128, 0x00000401);

					// Enable leaky packet mechanism, the bandwidth is sustained to a minimum value
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPIDFLTC_PIDFLT_LEAKPKT(demod), 1);
					// Remove bug fix: workaround of BZ#98230: bad sampling of data[3] of TS bus
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(demod), 0);
				break;

				case FE_TS_SERIAL_ON_TSOUT_0:
					// Setting alternate function 3 from PIO4_0 to PIO4_2
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, 0x00000333);

					// Setting output enable from PIO4_0 to PIO4_2
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
					reg_value |= 0x00000700;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

					// Setting clk not data for PIO4_0 and clock inversion (BZ#75008)
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1108, 0x00000205);

					// Setting data not clk for pio from PIO4_1 to PIO4_2 and choose STFE clock as a clock source for retime block
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1109, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1110, 0x00000401);
				break;

				case FE_TS_SERIAL_ON_TSOUT_1:
					// Setting alternate function 3 from PIO6_0 to PIO6_2
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, 0x00000333);

					// Setting output enable from PIO6_0 to PIO6_2
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
					reg_value |= 0x07000000;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

					// Setting clk not data for PIO6_0 and clock inversion (BZ#75008)
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1124, 0x00000205);

					// Setting data not clk for pio from PIO6_1 to PIO6_2 and choose STFE clock as a clock source for retime block
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1125, 0x00000401);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1126, 0x00000401);
				break;

				case FE_TS_ASI:
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_SERIAL(demod), 1); /* Serial mode */
					/* DVBCI setting: only for cut 1 */
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(demod), 1);  /* continuous clock only for BNC connector (J40) */
					error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSCFG2(demod), 1); /* reset of merger + TS_FIFO serial */

					// Setting alternate function 2 from PIO3_5 to PIO3_7
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1002, 0x22220000); // only for J40 connector

					// Setting output enable from PIO3_4 to PIO3_7
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
					reg_value |= 0x00F00000;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, reg_value);

					// Setting clk not data for PIO3_5
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1121, 0x00000004);

					// Setting clk not data for PIO3_4
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1120, 0x00000004);

					// Remove bug fix: workaround of BZ#98230: bad sampling of data[3] of TS bus
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(demod), 0);
				break;

				case FE_TS_NCR:
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_SERIAL(demod), 1); /* Serial mode */
					/* DVBCI setting: only for cut 1 */
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(demod), 1);  /*  CLKOUT pulse with each data for HE10 connector (J34) */
					error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSCFG2(demod), 1); /* reset of merger + TS_FIFO serial */

					// Setting alternate function 2 from PIO3_5 to PIO3_7
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1002, 0x11110000); // only for J34 connector

					// Setting output enable from PIO3_4 to PIO3_7
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, &reg_value);
					reg_value |= 0x00F00000;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, reg_value);

					// Setting clk not data for PIO3_5
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1121, 0x00000004);

					// Setting clk not data for PIO3_4
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1120, 0x00000004);

					// Remove bug fix: workaround of BZ#98230: bad sampling of data[3] of TS bus
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(demod), 0);
				break;

				case FE_TS_NCR_TEST_BUS: // new management of NCR
					// FE_GPIO[18]=ts_extncr_out[1]=PIO1[1] (I2C repeater bus, SDA)
					error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_TOP_CTRL_GPIO18CFG, 0x3C);

					// FE_GPIO[19]=ts_extncr_out[0]=PIO1[0] (I2C repeater bus, SCL)
					error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_TOP_CTRL_GPIO19CFG, 0x3E);

					// Select AVS-PWM0, EXTNCR
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1000, &reg_value);
					reg_value |= 0x00001011;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1000, reg_value);

					// Enable alternate function #1 for PIO2[0], PIO2[1], PIO2[2], PIO2[3] and PIO2[4]
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1001, &reg_value);
					reg_value |= 0x00011111;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1001, reg_value);

					// Select PIO3[4] as FE-GPIO[3], PIO3[5] as FE-GPIO[2], PIO3[6] as FE-GPIO[1] and PIO3[7] as FE-GPIO[0]
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1002, &reg_value);
					reg_value |= /*0x33331111*/0x33310000; // 0x33330000 enough ??
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1002, reg_value);

					// Enable PIO3[5] and PIO3[7] as output, enable PIO3[4] and PIO3[6] as input
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, &reg_value);
					reg_value |= 0x3f501f1b/* Minimal value = 0x00A00000, but we set all PIOs as output */;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, reg_value);
				break;

				/* Careful, in mux stream mode, demod#1 and demod#2 work in pair, as in wideband mode
				If demod#1 is not locked, then it is not possible to lock demod#2 */
				case FE_TS12_MUX_STREAM_MODE:
					// Set MUX stream, not DVBCI
					error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_GEN_TSGENERAL(1), 0x36);
					// Punctured clock
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(1), 0);
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(2), 0);
					// Program PIO4[5:0] in alternate 4
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001, 0x00444444);
					// Program PIO5[5:0] in alternate 4
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, 0x00444444);
					// Output enable, PIO pad enabled of PIO4[5:0] and PIO5[5:0]
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
					reg_value |= 0x003F3F00;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);
					// Setting clk not data for PIO4_0
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1108, 0x00000004);
					// Setting clk not data for PIO5_5
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1121, 0x00000006);
					// Disable leaky packet mechanism, the bandwidth is free to be as low as possible
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPIDFLTC_PIDFLT_LEAKPKT(1), 0);
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPIDFLTC_PIDFLT_LEAKPKT(2), 0);
					// Remove bug fix: workaround of BZ#98230: bad sampling of data[3] of TS bus
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(1), 0);
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(2), 0);
				break;

				/* Careful, in mux stream mode, demod#3 and demod#4 work in pair, as in wideband mode
				If demod#3 is not locked, then it is not possible to lock demod#4 */
				case FE_TS34_MUX_STREAM_MODE:
					// Set MUX stream, not DVBCI
					error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_GEN_TSGENERAL(2), 0x36);
					// Punctured clock
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(3), 0);
					error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG2_TSFIFO_DVBCI(4), 0);
					// Program PIO6[5:0] in alternate 4
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, 0x00444444);
					// Program PIO7[5:0] in alternate 4
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, 0x00444444);
					// Output enable, PIO pad enabled of PIO6[5:0]
					error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
					reg_value |= 0x3F000000;
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);
					// Output enable, PIO pad enabled of PIO7[5:0]
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, 0x3F000000);

					// Setting clk not data for PIO6_0
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1124, 0x00000006);
					// Setting clk not data for PIO7_5
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1129, 0x00000006);
					// Disable leaky packet mechanism, the bandwidth is free to be as low as possible
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPIDFLTC_PIDFLT_LEAKPKT(1), 0);
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSPIDFLTC_PIDFLT_LEAKPKT(2), 0);
					// Remove bug fix: workaround of BZ#98230: bad sampling of data[3] of TS bus
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(1), 0);
					error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSCFG0_TSFIFO_BCLKDEL1CK(2), 0);
				break;

				case FE_TS_OUTPUTMODE_DEFAULT:
				break;
			}
			// 130MHz Oxford-Mclk CK_F1
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_TOP_CTRL_GPIO14CFG, 0x7E);
			// Select Oxford-Mclk, IRQ, SGNL1[1]
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, 0x00011000);
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_enable_stfe
--ACTION	::	Enables STFE block (inputs and outputs)
--PARAMS IN	::	handle -> Front End Handle
			stfe_output -> either 1st output, or 2nd output or both
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_enable_stfe(struct fe_stid135_internal_param* pParams, enum fe_stid135_stfe_output stfe_output)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (pParams->handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			/* Enable IB0-IB7 */
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG128, 0x00000000);
			switch(stfe_output) {
			case FE_STFE_OUTPUT0:
				/* REQ = 1, enable TSout0 */
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG129, 0x00000002);
				/* REQ = 1, disable TSout1 */
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG130, 0x00000003);
			break;
			case FE_STFE_OUTPUT1:
				/* REQ = 1, disable TSout0 */
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG129, 0x00000003);
				/* REQ = 1, enable TSout1 */
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG130, 0x00000002);
			break;
			case FE_STFE_BOTH_OUTPUT:
				/* REQ = 1, enable TSout0 */
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG129, 0x00000002);
				/* REQ = 1, enable TSout1 */
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG130, 0x00000002);
			break;
			}
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_stfe
--ACTION	::	Sets PIOs according to wanted TS mode
--PARAMS IN	::	handle -> Front End Handle
			mode -> mode of STFE (bypass...)
			stfe_input_path -> see below description
			stfe_output_path -> see below description
			tag_header -> tag value in STFE tagging mode
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
/* u8 stfe_output_path : coding of output path: FE_STFE_OUTPUT0 => TSout0
									FE_STFE_OUTPUT1 => TSout1
									FE_STFE_BOTH_OUTPUT => TSout0 and TSout1

	In the context of FE_STFE_BYPASS_MODE :
	u8 stfe_input_path : coding of input path: PATH0 => IB0
							 PATH1 => IB1
							 PATH2 => IB2
							 PATH3 => IB3
							 PATH4 => IB4
							 PATH5 => IB5
							 PATH6 => IB6
							 PATH7 => IB7

	In the context of FE_STFE_TAGGING_MODE or FE_STFE_TAGGING_MERGING_MODE mode :
	u8 stfe_input_path : coding of input path: 0x1 => IB0
							 0x2 => IB1
							 0x4 => IB2
							 0x8 => IB3
							 0x10 => IB4
							 0x20 => IB5
							 0x40 => IB6
							 0x80 => IB7
							 .. but all combinations are possible, for instance :
							 0x74 => IB2 & IB4 & IB5 & IB6
							 0x94 => IB0 & IB2 & IB4 & IB7
*/
fe_lla_error_t fe_stid135_set_stfe(struct fe_stid135_internal_param* pParams, enum fe_stid135_stfe_mode mode, u8 stfe_input_path, enum fe_stid135_stfe_output stfe_output_path, u8 tag_header)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i, path[8] = {0};
	u32 reg_value;
	//u32 dest0_route_reg, dest1_route_reg;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
		if (pParams->handle_demod->Error )
			error=FE_LLA_I2C_ERROR;
		else {
			switch(mode) {
			case FE_STFE_BYPASS_MODE:
				/* Enables bypass mode, performs reset and enables processing on this stream */
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_SYSTEM(stfe_input_path+1), 0x00000007);

				switch(stfe_output_path) {
				case FE_STFE_OUTPUT0:
					// Enables bypass mode on TSout0
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_BYPASS, 0x80000000 | stfe_input_path);
				break;
				case FE_STFE_OUTPUT1:
					// Enables bypass mode on TSout1
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_BYPASS, 0x80000000 | stfe_input_path);
				break;
				case FE_STFE_BOTH_OUTPUT:
					// Enables bypass mode on both TSout0 and TSout1
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_BYPASS, 0x80000000 | stfe_input_path);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_BYPASS, 0x80000000 | stfe_input_path);
				break;
				}
			break;

			case FE_STFE_TAGGING_MODE:
				error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_BYPASS, &reg_value);
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_BYPASS, reg_value & 0x7FFFFFFF);
				error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_BYPASS, &reg_value);
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_BYPASS, reg_value & 0x7FFFFFFF);
				for(i=0;i<8;i++)
					path[i] = (stfe_input_path>>i) & 0x1;
				if(path[0]) {
					/* Top address of stream buffer in STFE internal RAM */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM0_TOP, 0x400*(0+1)-1);
					/* Base address of stream buffer in STFE internal RAM */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM0_BASE, 0x400*(0));
					/* Transport packet size, includes tagging bytes */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM0_PKTSIZE, 0x000000C4);
					/* Source-Destination routing : choose which inputs are sent to which outputs */
				}
				if(path[1]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM1_TOP, 0x400*(1+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM1_BASE, 0x400*(1));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM1_PKTSIZE, 0x000000C4);
				}
				if(path[2]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM2_TOP, 0x400*(2+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM2_BASE, 0x400*(2));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM2_PKTSIZE, 0x000000C4);
				}
				 if(path[3]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM3_TOP, 0x400*(3+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM3_BASE, 0x400*(3));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM3_PKTSIZE, 0x000000C4);
				 }
				if(path[4]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM4_TOP, 0x400*(4+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM4_BASE, 0x400*(4));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM4_PKTSIZE, 0x000000C4);
				}
				if(path[5]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM5_TOP, 0x400*(5+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM5_BASE, 0x400*(5));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM5_PKTSIZE, 0x000000C4);
				}
				if(path[6]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM6_TOP, 0x400*(6+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM6_BASE, 0x400*(6));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM6_PKTSIZE, 0x000000C4);
				}
				if(path[7]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM7_TOP, 0x400*(7+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM7_BASE, 0x400*(7));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM7_PKTSIZE, 0x000000C4);
				}

				if(stfe_output_path == FE_STFE_OUTPUT0) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_ROUTE, stfe_input_path);
				} else if (stfe_output_path == FE_STFE_OUTPUT1) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_ROUTE, stfe_input_path);
				} else if(stfe_output_path == FE_STFE_BOTH_OUTPUT) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_ROUTE, stfe_input_path);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_ROUTE, (u8)(~stfe_input_path));
				}

				/* Output channel FIFO trigger level */
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_FIFOTRIG, 0x00000019);
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_FIFOTRIG, 0x00000019);

				/* Setting STFE for merging the streams */
				for(i=0;i<8;i++) {
					if(path[i] == 1) {
						/* Control tag bytes insertion */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_TAG_BYTES(i+1), (u32)((tag_header<<24) | ((i+1)<<16) | 0x1));
						/* End address of buffer in internal RAM */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_BUFFER_END(i+1), (u32)(0x400*(i+1)-1));
						/* Start address of buffer in internal RAM */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_BUFFER_START(i+1), (u32)(0x400*i));
						/* Interrupt mask register */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_MASK(i+1), 0x0000001F);
						/* Clear FIFO, buffers and enables processing on this stream */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_SYSTEM(i+1), 0x00000003);
					}
				}
			break;

			case FE_STFE_TAGGING_MERGING_MODE:
				error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_BYPASS, &reg_value);
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_BYPASS, reg_value & 0x7FFFFFFF);
				error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_BYPASS, &reg_value);
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_BYPASS, reg_value & 0x7FFFFFFF);
				for(i=0;i<8;i++)
					path[i] = (stfe_input_path>>i) & 0x1;
				if(path[0]) {
					/* Top address of stream buffer in STFE internal RAM */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM0_TOP, 0x400*(0+1)-1);
					/* Base address of stream buffer in STFE internal RAM */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM0_BASE, 0x400*(0));
					/* Transport packet size, includes tagging bytes */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM0_PKTSIZE, 0x000000C4);
					/* Source-Destination routing : choose which inputs are sent to which outputs */
				}

				if(path[1]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM1_TOP, 0x400*(1+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM1_BASE, 0x400*(1));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM1_PKTSIZE, 0x000000C4);
				}

				if(path[2]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM2_TOP, 0x400*(2+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM2_BASE, 0x400*(2));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM2_PKTSIZE, 0x000000C4);
				}

				if(path[3]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM3_TOP, 0x400*(3+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM3_BASE, 0x400*(3));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM3_PKTSIZE, 0x000000C4);
				}

				if(path[4]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM4_TOP, 0x400*(4+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM4_BASE, 0x400*(4));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM4_PKTSIZE, 0x000000C4);
				}

				if(path[5]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM5_TOP, 0x400*(5+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM5_BASE, 0x400*(5));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM5_PKTSIZE, 0x000000C4);
				}

				if(path[6]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM6_TOP, 0x400*(6+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM6_BASE, 0x400*(6));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM6_PKTSIZE, 0x000000C4);
				}

				if(path[7]) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM7_TOP, 0x400*(7+1)-1);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM7_BASE, 0x400*(7));
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_STRM7_PKTSIZE, 0x000000C4);
				}

				/*overwritting output destination out0 register as ChipGetOneRegister is not working */
				if(stfe_output_path == FE_STFE_OUTPUT0) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_ROUTE, stfe_input_path);
				} else if (stfe_output_path == FE_STFE_OUTPUT1) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_ROUTE, stfe_input_path);
				} else if(stfe_output_path == FE_STFE_BOTH_OUTPUT) {
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_ROUTE, stfe_input_path);
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_ROUTE, (u8)~stfe_input_path);
				}

				/* Output channel FIFO trigger level */
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_FIFOTRIG, 0x00000019);
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_FIFOTRIG, 0x00000019);

				/* Setting STFE for merging the streams */
				for(i=0;i<8;i++) {
					if(path[i] == 1) {
						/* Control tag bytes insertion */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_TAG_BYTES(i+1), (u32)((tag_header<<24) | ((i+1)<<16) | 0x1));
						/* End address of buffer in internal RAM */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_BUFFER_END(i+1), (u32)(0x400*(i+1)-1));
						/* Start address of buffer in internal RAM */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_BUFFER_START(i+1), (u32)(0x400*i));
						/* Interrupt mask register */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_MASK(i+1), 0x0000001F);
						/* Clear FIFO, buffers and enables processing on this stream */
						error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_IB_C8SECTPFE_IB_SYSTEM(i+1), 0x00000003);
					}
				}
			break;
			}
		}
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_FORMAT, &reg_value);
		reg_value |= FSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_FORMAT_WAIT_WHOLE_PACKET__MASK;
		error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST0_FORMAT, reg_value);
#ifdef TOTEST //DeepThought This may not be needed 20201226, but present in latest official driver
#else
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_FORMAT, &reg_value);
		reg_value |= FSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_FORMAT_WAIT_WHOLE_PACKET__MASK;
		error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C8SECTPFE_TSDMA_C8SECTPFE_TSDMA_DEST1_FORMAT, reg_value);
#endif
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_flexclkgen_init
--ACTION	::	Sets Flex Clock Gen block
--PARAMS IN	::	handle -> Front End Handle
-PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
static fe_lla_error_t fe_stid135_flexclkgen_init(struct fe_stid135_internal_param* pParams)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	u32 system_clock, reg_value;

		if (pParams->handle_demod->Error)
			error=FE_LLA_I2C_ERROR;
		else {
			/* if MODE_PIN0 = 1, then the system clock is equal to the crystal clock divided by 2 */
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_STATUS48, &reg_value);
			if((reg_value & 0x1) == 1)
				system_clock = pParams->quartz >> 1;
			else
				system_clock = pParams->quartz;

			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG0, 0x10000000);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG1, 0x00030000);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG5, 0x002C25ED);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG6, 0x002C25ED);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG7, 0x002C25ED);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG8, 0x002C25ED);

			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG3, 0x00000000);

			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG89, 0x00000040);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG90, 0x00000040);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG91, 0x00000040);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG92, 0x00000040);

			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG98, 0x00000040);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG99, 0x00000040);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG6,  0x45454545);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG7,  0x45454545);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG8,  0x45454545);

			/* 0x1F0F is reset value, therefore maybe be ommited */
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG0, 0x00001F0F);

			switch(system_clock) {
				case 25:
					/* Control the PLL feedback divider settings */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG1, 0x00050000);
					/* Control output clock channel 0 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG5, 0x014371C7);
					/* Control output clock channel 1 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG6, 0x014371C7);
					/* Control output clock channel 2 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG7, 0x000ec4ec);
					/* Control output clock channel 3 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG8, 0x011c0000);
				break;
				case 27:
					/* Control the PLL feedback divider settings */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG1, 0x00040000);
					/* Control output clock channel 0 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG5, 0x01440000);
					/* Control output clock channel 1 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG6, 0x01440000);
					/* Control output clock channel 2 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG7, 0x000fa5fa);
					/* Control output clock channel 3 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG8, 0x011ccccc);
				break;
				case 30:
					/* Control the PLL feedback divider settings */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG1, 0x00020000);
					/* Control output clock channel 0 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG5, 0x01440000);
					/* Control output clock channel 1 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG6, 0x01440000);
					/* Control output clock channel 2 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG7, 0x000fa5fa);
					/* Control output clock channel 3 */
					error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG8, 0x011ccccc);
				break;
			}

			WAIT_N_MS(20);
			/* Power up just the PLL by clearing bit 12 */
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG0, 0x00000F0F);
			/* Wait for 20000ns to assert lock or poll bit 24 of gfgx_config0 till its 1 */
			WAIT_N_MS(10);
			/* Let the outputs come out of reset */
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG0, &reg_value);
			reg_value &= ~(u32)FSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG0_GFG0_RESET__MASK;
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG0, reg_value);
			/* Wait for a small time for the reset to come off and let the logic sort itself out. In sim doing the next bit too early
			the fast glitches still appear on the outputs */
			WAIT_N_MS(10);
			/* Power up the outputs which will go high for a while and then start generating the required clock */
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG0, &reg_value);
			reg_value &= ~(u32)FSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG0_GFG0_STANDBY__MASK;
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG0, reg_value);
			WAIT_N_MS(10);
			/* Program the required channels EN_PRG to 1 */
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG3, &reg_value);
			reg_value |= FSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG3_GFG0_STROBE__MASK;
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG3, reg_value);

			/* Define division ratio for final divider 0 - STBUS south clock */
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG89, 0x43);
			/* Define division ratio for final divider 1 - STBUS north clock */
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG90, 0x40);
			/* Define division ratio for final divider 2 - RAM clock for STFE */
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG91, 0x41);
			/* Define division ratio for final divider 3 - STFE system clock */
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG92, 0x42);
			/* Define division ratio for final divider 9 - TSOUT0 output clock : TS parallel */
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG98, 0x40);
			/* Define division ratio for final divider 10 - TSOUT1 output clock : TS parallel */
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG99, 0x40);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG6, 0x40404440);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG7, 0x42414141);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG8, 0x45434342);
		}
	return error;
}

/*****************************************************
--FUNCTION	::	get_clk_source
--ACTION	::	Useful function to compute clock frequency
			for each block
--PARAMS IN	::	Handle -> Front End Handle
			clkout_number ->
-PARAMS OUT	::	crossbar_p ->
--RETURN	::	error (if any)
--***************************************************/
static fe_lla_error_t get_clk_source(struct fe_stid135_internal_param* pParams, u8 clkout_number, u8* crossbar_p)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u32 reg_value;

	*crossbar_p = 0;

	switch(clkout_number) {
		case 0:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG6, &reg_value);
			*crossbar_p = (u8) reg_value;
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 1:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG6, &reg_value);
			*crossbar_p = (u8)(reg_value >> 8);
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 2:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG6, &reg_value);
			*crossbar_p = (u8)(reg_value >> 16);
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 3:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG6, &reg_value);
			*crossbar_p = (u8)(reg_value >> 24);
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 4:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG7, &reg_value);
			*crossbar_p = (u8)reg_value ;
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 5:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG7, &reg_value);
			*crossbar_p = (u8)(reg_value >> 8);
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 6:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG7, &reg_value);
			*crossbar_p = (u8)(reg_value >> 16);
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 7:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG7, &reg_value);
			*crossbar_p = (u8)(reg_value >> 24);
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 8:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG8, &reg_value);
			*crossbar_p = (u8)reg_value;
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 9:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG8, &reg_value);
			*crossbar_p = (u8)(reg_value >> 8);
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 10:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG8, &reg_value);
			*crossbar_p = (u8)(reg_value >> 16);
			*crossbar_p = (*crossbar_p)&0x7;
		break;
		case 11:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG8, &reg_value);
			*crossbar_p = (u8)(reg_value >> 24);
			*crossbar_p = (*crossbar_p)&0x7;
		break;
	}
	return(error);
}

/*****************************************************
--FUNCTION	::	compute_final_divider
--ACTION	::	Another useful function to compute clock frequency
			for each block
--PARAMS IN	::	Handle -> Front End Handle
			clkin ->
-PARAMS OUT	::	clkout_p ->
--RETURN	::	error (if any)
--***************************************************/
static fe_lla_error_t compute_final_divider(struct fe_stid135_internal_param *pParams,
																						u32 clkin, u8 clkout_number, u32* clkout_p)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	u8 crossbar = 0;
	s32 field_val;

	*clkout_p = pParams->quartz;

	error |= get_clk_source(pParams, clkout_number, &crossbar);
	switch(clkout_number) {
		case 0:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG89_FINDIV0DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 1:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG90_FINDIV1DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 2:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG91_FINDIV2DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 3:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG92_FINDIV3DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 4:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG93_FINDIV4DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 5:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG94_FINDIV5DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 6:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG95_FINDIV6DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 7:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG96_FINDIV7DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 8:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG97_FINDIV8DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 9:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG98_FINDIV9DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 10:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG99_FINDIV10DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
		case 11:
			error |= ChipGetField(pParams->handle_soc, FLD_FSTID135_OXFORD_FLEXCLKGEN_A_FCG_CONFIG100_FINDIV11DIVBY, &field_val);
			*clkout_p = clkin*100 / (u32)(field_val + 1);
		break;
	}
	return(error);
}

/*****************************************************
--FUNCTION	::	get_soc_block_freq
--ACTION	::	Compute clock frequency for each block
--PARAMS IN	::	Handle -> Front End Handle
			clkout_number ->
-PARAMS OUT	::	clkout_p ->
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t get_soc_block_freq(struct fe_stid135_internal_param *pParams, u8 clkout_number, u32* clkout_p)
{
	u8 crossbar;
	u32 clkin = 0;
	u32 Num, Den1, Den2, Den3, Den4, system_clock, reg_value;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//u32 reg_val;


	/* if MODE_PIN0 = 1, then the system clock is equal to the crystal clock divided by 2 */
	error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_STATUS48, &reg_value);
	if((reg_value & 0x1) == 1)
		system_clock = pParams->quartz >> 1;
	else
		system_clock = pParams->quartz;

	*clkout_p = system_clock * 100;

	error |= get_clk_source(pParams, clkout_number, &crossbar);

	switch(crossbar) {
		case 0:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG1, &reg_value);
			clkin = system_clock*(((reg_value>>16) & 0x7) + 16);
			error |= compute_final_divider(pParams, clkin, clkout_number, clkout_p);
		break;
		case 1:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG1, &reg_value);
			Num = system_clock*(((reg_value>>16) & 0x7) + 16);
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG5, &Den1);
			Den1 = (Den1>>24)&0x1;
			if(Den1==0)
				Den1 = 3;
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG5, &Den2);
			Den2 = (u32)XtoPowerY(2, ((Den2>>20)&0xF));
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG5, &Den3);
			Den3 = (Den3>>15)&0x1F;
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG5, &Den4);
			Den4 = (Den4)&0x7FFF;
			*clkout_p = (Num*(1<<20))/(Den1*Den2*((1<<20)+Den3*(1<<15)+Den4));
			error |= compute_final_divider(pParams, *clkout_p, clkout_number, clkout_p);
		break;
		case 2:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG1, &Num);
			Num = system_clock*(((Num>>16) & 0x7) + 16);
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG6, &Den1);
			Den1 = (Den1>>24)&0x1;
			if(Den1==0)
				Den1 = 3;
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG6, &Den2);
			Den2 = (u32)XtoPowerY(2, ((Den2>>20)&0xF));
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG6, &Den3);
			Den3 = (Den3>>15)&0x1F;
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG6, &Den4);
			Den4 = (Den4)&0x7FFF;
			*clkout_p = (Num*(1<<20))/(Den1*Den2*((1<<20)+Den3*(1<<15)+Den4));
			error |= compute_final_divider(pParams, *clkout_p, clkout_number, clkout_p);
		break;
		case 3:
			error |=ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG1, &Num);
			Num = system_clock*(((Num>>16) & 0x7) + 16);
			error |=ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG7, &Den1);
			Den1 = (Den1>>24)&0x1;
			if(Den1==0)
				Den1 = 3;
			error |=ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG7, &Den2);
			Den2 = (u32)XtoPowerY(2, ((Den2>>20)&0xF));
			error |=ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG7, &Den3);
			Den3 = (Den3>>15)&0x1F;
			error |=ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG7, &Den4);
			Den4 = (Den4)&0x7FFF;
			*clkout_p = (Num*(1<<20))/(Den1*Den2*((1<<20)+Den3*(1<<15)+Den4));
			error |= compute_final_divider(pParams, *clkout_p, clkout_number, clkout_p);
		break;
		case 4:
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG1, &Num);
			Num = system_clock*(((Num>>16) & 0x7) + 16);
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG8, &Den1);
			Den1 = (Den1>>24)&0x1;
			if(Den1==0)
				Den1 = 3;
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG8, &Den2);
			Den2 = (u32)XtoPowerY(2, ((Den2>>20)&0xF));
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG8, &Den3);
			Den3 = (Den3>>15)&0x1F;
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_OXFORD_FLEXCLKGEN_A_GFG0_CONFIG8, &Den4);
			Den4 = (Den4)&0x7FFF;
			*clkout_p = (Num*(1<<20))/(Den1*Den2*((1<<20)+Den3*(1<<15)+Den4));
			error |= compute_final_divider(pParams, *clkout_p, clkout_number, clkout_p);
		break;
		case 5:
			*clkout_p = system_clock * 100;
		break;
	}
	return(error);
}


/*****************************************************
--FUNCTION	::	fe_stid135_get_mode_code
--ACTION	::	Gets MODCOD, frame length and pilot presence
--PARAMS IN	::	pParams -> Pointer to fe_stid135_internal_param structure
			demod -> Current demod 1..8
-PARAMS OUT	::	modcode -> MODCOD
			frame -> frame length
			pilots -> pilot presence
--RETURN	::	error (if any)
--***************************************************/
static fe_lla_error_t fe_stid135_get_mode_code(struct stv* state,
					enum fe_sat_modcode *modcode,
					enum fe_sat_frame *frame,
					enum fe_sat_pilots *pilots)
{
	enum fe_stid135_demod demod = state->nr+1;
	u32 mcode = 0;
	s32 dmdmodcode = 0;
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	if(state->base->ip.handle_demod == NULL)
		error |= FE_LLA_INVALID_HANDLE;
	else {
		error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDMODCOD(demod), &mcode);

		if(state->signal_info.standard == FE_SAT_DVBS2_STANDARD) {

			if(((mcode >> 7) & 0x01) == 1) {
				dmdmodcode  =  (mcode >> 1) & 0x7F; // DVBS2X
			} else {
				dmdmodcode = (mcode >> 2) & 0x1F; // DVBS2 legacy
			}

			*pilots		=  (enum fe_sat_pilots)(mcode & 0x01);
			*frame		=  (enum fe_sat_frame)((mcode & 0x02) >> 1);

			if (dmdmodcode <= FE_SAT_MAX_LEGACY_MODCODE) { /* DVBS2 Legacy */
				*modcode    =  (enum fe_sat_modcode)dmdmodcode;
				*pilots     =  (enum fe_sat_pilots)((mcode & 0x01));
			}

			if ((dmdmodcode >=	FE_SAT_MIN_DVBS2X_MODCODE )  && /* DVBS2X */
				(dmdmodcode <=	FE_SAT_MAX_DVBS2X_MODCODE )) {
				*modcode  =  (enum fe_sat_modcode)(dmdmodcode);
				*frame  =  FE_SAT_NORMAL_FRAME;
			}

			if ((dmdmodcode >=	FE_SAT_MIN_SHORT_MODCODE  )  && /* DVBS2X short */
				(dmdmodcode <=	FE_SAT_MAX_SHORT_MODCODE )) {
				*frame  =  FE_SAT_SHORT_FRAME;
			}

			if ((dmdmodcode >=	FE_SAT_MIN_POFF_MODCODE )  && /* DVBS2X ponpoff */
				(dmdmodcode <=	FE_SAT_MAX_POFF_MODCODE )) {
				if (*pilots == FE_SAT_PILOTS_ON)
					dmdmodcode = dmdmodcode + (FE_SAT_MIN_PON_MODCODE - FE_SAT_MIN_POFF_MODCODE);
				*pilots = FE_SAT_PILOTS_ON;
			}
		}
		if(state->signal_info.standard == FE_SAT_DVBS1_STANDARD) {
			dmdmodcode = (mcode >> 2) & 0x3F; // DVBS1
			*pilots     =  FE_SAT_UNKNOWN_PILOTS;
			*frame      =  FE_SAT_UNKNOWN_FRAME;
			if ((dmdmodcode >=	FE_SAT_MIN_DVBS1_MODCODE )  && /* DVBS1 */
				(dmdmodcode <=	FE_SAT_MAX_DVBS1_MODCODE )) {
				*modcode    =  (enum fe_sat_modcode)(dmdmodcode);
			}
		}
	}

	if (state->base->ip.handle_demod->Error )
		error = FE_LLA_I2C_ERROR;

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_modcodes_filter
--ACTION	::	Set a MODCODES mask in the demodulators,
			only MODCODS of the given list are authorized at the input of the FEC
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 .. 8
			MODCODES_MASK -> array :list of the MODCODS to be enabled
			NbModes -> Number of modes to enable (size of MODCODES_MASK table)
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_modcodes_filter(struct stv* state,
						struct fe_sat_dvbs2_mode_t *MODCODES_MASK,
						s32 NbModes)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

	s32 regflist;
	u32 modcodlst1, reg_value;

	u16 regIndex = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTF(demod),
		fieldIndex,
		i;

	u8	regMask = 0xFF,
		pilots=0,
		frameLength=0,
		frameType;

		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			modcodlst1 = FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST1_SYMBRATE_FILTER(demod);
			regflist= (s32)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST2(demod);

			error |= ChipSetField(state->base->ip.handle_demod, modcodlst1, 0); /* disable automatic filter vs. SR */

			/*First disable all modcodes */
			/* Set to 1 modcod fields of MODCODLST1 register */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST1_DIS_32PSK_9_10(demod), 0x3);
			/* Set to 1 modcod fields of MODCODLST2 to MODCODLST3 registers */
			for(i=0;i<2;i++)
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)(regflist + i), 0xFF);
			/* Set to 1 modcod fields of MODCODLST4 register */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST4_DIS_16PSK_8_9(demod), 0xF);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST4_DIS_16PSK_9_10(demod), 0x3);
			/* Set to 1 modcod fields of MODCODLST5 to MODCODLST6 registers */
			for(i=3;i<5;i++)
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)(regflist + i), 0xFF);
			/* Set to 1 modcod fields of MODCODLST7 register */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST7_DIS_8P_8_9(demod), 0xF);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST7_DIS_8PSK_9_10(demod), 0x3);
			/* Set to 1 modcod fields of MODCODLST8 to MODCODLST9 registers */
			for(i=6;i<8;i++)
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)(regflist + i), 0xFF);
			/* Set to 1 modcod fields of MODCODLSTA register */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTA_DIS_QP_8_9(demod), 0xF);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTA_DIS_QPSK_9_10(demod), 0x3);
			/* Set to 1 modcod fields of MODCODLSTB to MODCODLSTE registers */
			for(i=9;i<13;i++)
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)(regflist + i), 0xFF);

			/* Set to 1 modcod field of MODCODLSTF register */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTF_DIS_QPSK_1_4(demod), 0xF);

			/* Set to 1 modcod fields from MODCODLS10 to MODCODLS1F registers - DVBS2X */
			for(i=14;i<30;i++) {
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)(regflist + i), 0xFF);
			}

			for(i=0;i<NbModes;i++) {
				if(INRANGE (FE_SAT_QPSK_14, MODCODES_MASK[i].mod_code, FE_SAT_32APSK_910)) {
					regIndex = (u16)(REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTF(demod) - MODCODES_MASK[i].mod_code/2);
					fieldIndex = MODCODES_MASK[i].mod_code % 2;
					pilots = MODCODES_MASK[i].pilots;
					frameLength = MODCODES_MASK[i].frame_length;

					error |= ChipGetOneRegister(state->base->ip.handle_demod,regIndex, &reg_value);
					regMask = (u8)reg_value;
					frameType= (u8)((frameLength << 1)|(pilots));
					if(fieldIndex ==0)
						regMask = (u8)(regMask & (~(0x01 << frameType)));
					else
						regMask = (u8)(regMask & (~(0x10 << frameType)));
					error |= ChipSetOneRegister(state->base->ip.handle_demod,regIndex,regMask);
				} else if(INRANGE (FE_SATX_QPSK_13_45, MODCODES_MASK[i].mod_code, FE_SATX_1024APSK)){
					regIndex = (u16)(REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1F(demod) - MODCODES_MASK[i].mod_code/2);
				}
				/*fieldIndex = MODCODES_MASK[i].mod_code % 2;
				pilots = MODCODES_MASK[i].pilots;
				frameLength = MODCODES_MASK[i].frame_length;

				error |= ChipGetOneRegister(state->base->ip.handle_demod,regIndex, &reg_value);
				regMask = (u8)reg_value;
				frameType= (u8)((frameLength << 1)|(pilots));
				if(fieldIndex ==0)
					regMask = (u8)(regMask & (~(0x01 << frameType)));
				else
					regMask = (u8)(regMask & (~(0x10 << frameType)));
				error |= ChipSetOneRegister(state->base->ip.handle_demod,regIndex,regMask);   */
			}
		}

		if (state->base->ip.handle_demod->Error ) /*Check the error at the end of the function*/
			error=FE_LLA_I2C_ERROR;
	return error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_reset_modcodes_filter
--ACTION	::	Unset all MODCODE mask in the demodulator
--PARAMS IN	::	Handle -> Front End Handle
			demod -> Current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_reset_modcodes_filter(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	fe_lla_error_t error1 = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	u8 i, row_max;
	u32 reg_value;
	if(state->base->ip.handle_demod->Error) {
		dprintk("CALLED with error=%d (correcting)\n", state->base->ip.handle_demod->Error);
		error = FE_LLA_NO_ERROR;
	}
	error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST0(demod), 0xFF));
	if(error1)
		dprintk("Error setting REG_RC8CODEW_DVBSX_DEMOD_MODCODLST0(%d)\n", demod);
	error |= (error1=ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST1_NRESET_MODCODLST(demod), 1));
	if(error1)
		dprintk("Error setting FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST1_NRESET_MODCODLST(%d)\n", demod);
	/*First disable all modcodes */
	row_max = sizeof(state->mc_flt) / sizeof(state->mc_flt[0]) - 1;
	for(i = 0;i <= row_max;i++) {
		state->mc_flt[i].forbidden = FALSE;
		error |= (error1=ChipGetOneRegister(state->base->ip.handle_demod, state->mc_flt[i].register_address, &reg_value));
		if(error1)
			dprintk("Error getting reg %d demod=%d\n", i, demod);
		reg_value &= ~(state->mc_flt[i].mask_value);
		error |= (error1=ChipSetOneRegister(state->base->ip.handle_demod, state->mc_flt[i].register_address, reg_value));
		if(error1)
			dprintk("Error getting reg %d demod=%d\n", i, demod);
		//dprintk("here error=%d\n", error);
	}

	if (state->base->ip.handle_demod->Error ) //Check the error at the end of the function
		error = FE_LLA_I2C_ERROR;
	return error;
}

/*
Below is a list containing ALL the MODCODs implemented or reserved by the
STiD135.
Please note that MODCOD performance is a function of the channel and MODCODs
behave differently in the presence of the various degradations and will impact
the monotonicity of the list.
The implementer must conduct their own characterization to ensure the list is
homogeneous in CNR knowing that the following factors intervene:
o	The phase, group-delay and frequency response of the transponder
o	The back off or operating point (linearity)
o	Cumulative phase noise of the components of the system
o	Fine setting of the transmit/receive filters, roll-off, pilots,
	adjacent channels, co-channel etc.
Furthermore the DVB-S2 spec is not coherent within itself (S2 in PER, S2X in FER).
Here below ST has provided a list as seen in the order of theoretical CNR as
printed in the DVB-S2 specifications EN 302307 parts 1 and 2 without correction
for PER/FER or any degradation.

The list is in order of CNR for QEF for each MODCOD starting with the lowest
		active CNR.
It is up to the implementer to provide and maintain this list.
Forbidden MODCODs (even though viable) should simply be given a very high value
	of CNR to ensure they are always disabled.
The MODCOD list management algorithm will first obtain lock with all MODCODs
enabled.  Once demodulator lock has been achieved the CNR will be established
and all the MODCODS superior to the current CNR will be disabled.
*/
/*****************************************************
--FUNCTION	::	fe_stid135_apply_custom_qef_for_modcod_filter
--ACTION	::	Fills-in modcod filtering array
--PARAMS IN	::	handle -> Front End Handle
		::	qef_table_from_customer -> pointer to a table of structure with all
			custom SNR for each MODCOD (MODCODs sorted from lowest SNR to biggest SNR)
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_apply_custom_qef_for_modcod_filter(struct stv* state, struct mc_array_customer *qef_table_from_customer)
{
	u8 i;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct mc_array_customer* cust_mc_flt =  qef_table_from_customer;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	if(state->base->ip.handle_demod->Error) {
		error = FE_LLA_I2C_ERROR;
	} else {
			if(cust_mc_flt == NULL) {
				for(i = 0;i < NB_SAT_MODCOD;i++) {
					state->mc_flt[i].register_address = mc_reg_addr[st_mc_flt[i].modcod_id][state->nr+1-1];
					state->mc_flt[i].mask_value = mc_mask_bitfield[st_mc_flt[i].modcod_id];
					state->mc_flt[i].qef = st_mc_flt[i].snr;
				}
			} else {
				for(i = 0;i < NB_SAT_MODCOD;i++) {
					state->mc_flt[i].register_address = mc_reg_addr[cust_mc_flt[i].modcod_id][state->nr+1-1];
					state->mc_flt[i].mask_value = mc_mask_bitfield[cust_mc_flt[i].modcod_id];
					state->mc_flt[i].qef = cust_mc_flt[i].snr;
				}
			}
	}
	if (state->base->ip.handle_demod->Error ) //Check the error at the end of the function
		error = FE_LLA_I2C_ERROR;
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_modcod_flt_reg_init
--ACTION	::	Fills-in modcod filtering array
--PARAMS IN	::	pParams -> Pointer to fe_stid135_internal_param structure
				demod -> Current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
void fe_stid135_modcod_flt_reg_init(void)
{
	u8 i;

	mc_mask_bitfield[FE_SAT_QPSK_14] = FC8CODEW_DVBSX_DEMOD_MODCODLSTF_DIS_QPSK_1_4__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_13] = FC8CODEW_DVBSX_DEMOD_MODCODLSTE_DIS_QPSK_1_3__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_25] = FC8CODEW_DVBSX_DEMOD_MODCODLSTE_DIS_QPSK_2_5__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_12] = FC8CODEW_DVBSX_DEMOD_MODCODLSTD_DIS_QPSK_1_2__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_35] = FC8CODEW_DVBSX_DEMOD_MODCODLSTD_DIS_QPSK_3_5__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_23] = FC8CODEW_DVBSX_DEMOD_MODCODLSTC_DIS_QP_2_3__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_34] = FC8CODEW_DVBSX_DEMOD_MODCODLSTC_DIS_QP_3_4__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_45] = FC8CODEW_DVBSX_DEMOD_MODCODLSTB_DIS_QP_4_5__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_56] = FC8CODEW_DVBSX_DEMOD_MODCODLSTB_DIS_QP_5_6__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_89] = FC8CODEW_DVBSX_DEMOD_MODCODLSTA_DIS_QP_8_9__MASK;
	mc_mask_bitfield[FE_SAT_QPSK_910] = FC8CODEW_DVBSX_DEMOD_MODCODLSTA_DIS_QPSK_9_10__MASK;
	mc_mask_bitfield[FE_SAT_8PSK_35] = FC8CODEW_DVBSX_DEMOD_MODCODLST9_DIS_8P_3_5__MASK;
	mc_mask_bitfield[FE_SAT_8PSK_23] = FC8CODEW_DVBSX_DEMOD_MODCODLST9_DIS_8P_2_3__MASK;
	mc_mask_bitfield[FE_SAT_8PSK_34] = FC8CODEW_DVBSX_DEMOD_MODCODLST8_DIS_8P_3_4__MASK;
	mc_mask_bitfield[FE_SAT_8PSK_56] = FC8CODEW_DVBSX_DEMOD_MODCODLST8_DIS_8P_5_6__MASK;
	mc_mask_bitfield[FE_SAT_8PSK_89] = FC8CODEW_DVBSX_DEMOD_MODCODLST7_DIS_8P_8_9__MASK;
	mc_mask_bitfield[FE_SAT_8PSK_910] = FC8CODEW_DVBSX_DEMOD_MODCODLST7_DIS_8PSK_9_10__MASK;
	mc_mask_bitfield[FE_SAT_16APSK_23] = FC8CODEW_DVBSX_DEMOD_MODCODLST6_DIS_16PSK_2_3__MASK;
	mc_mask_bitfield[FE_SAT_16APSK_34] = FC8CODEW_DVBSX_DEMOD_MODCODLST6_DIS_16PSK_3_4__MASK;
	mc_mask_bitfield[FE_SAT_16APSK_45] = FC8CODEW_DVBSX_DEMOD_MODCODLST5_DIS_16PSK_4_5__MASK;
	mc_mask_bitfield[FE_SAT_16APSK_56] = FC8CODEW_DVBSX_DEMOD_MODCODLST5_DIS_16PSK_5_6__MASK;
	mc_mask_bitfield[FE_SAT_16APSK_89] = FC8CODEW_DVBSX_DEMOD_MODCODLST4_DIS_16PSK_8_9__MASK;
	mc_mask_bitfield[FE_SAT_16APSK_910] = FC8CODEW_DVBSX_DEMOD_MODCODLST4_DIS_16PSK_9_10__MASK;
	mc_mask_bitfield[FE_SAT_32APSK_34] = FC8CODEW_DVBSX_DEMOD_MODCODLST3_DIS_32PSK_3_4__MASK;
	mc_mask_bitfield[FE_SAT_32APSK_45] = FC8CODEW_DVBSX_DEMOD_MODCODLST3_DIS_32PSK_4_5__MASK;
	mc_mask_bitfield[FE_SAT_32APSK_56] = FC8CODEW_DVBSX_DEMOD_MODCODLST2_DIS_32PSK_5_6__MASK;
	mc_mask_bitfield[FE_SAT_32APSK_89] = FC8CODEW_DVBSX_DEMOD_MODCODLST2_DIS_32PSK_8_9__MASK;
	mc_mask_bitfield[FE_SAT_32APSK_910] = FC8CODEW_DVBSX_DEMOD_MODCODLST1_DIS_32PSK_9_10__MASK;
	mc_mask_bitfield[FE_SATX_QPSK_13_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS1F_DIS_QPSK_13_45_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1F_DIS_QPSK_13_45_PON__MASK;
	mc_mask_bitfield[FE_SATX_QPSK_9_20] = FC8CODEW_DVBSX_DEMOD_MODCODLS1F_DIS_QPSK_9_20_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1F_DIS_QPSK_9_20_PON__MASK;
	mc_mask_bitfield[FE_SATX_QPSK_11_20] = FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_QPSK_11_20_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_QPSK_11_20_PON__MASK;
	mc_mask_bitfield[FE_SATX_8APSK_5_9_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8APSK_5_9_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8APSK_5_9_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_8APSK_26_45_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8APSK_26_45_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8APSK_26_45_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_8PSK_23_36] = FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8PSK_23_36_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8PSK_23_36_PON__MASK;
	mc_mask_bitfield[FE_SATX_8PSK_25_36] = FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_8PSK_25_36_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_8PSK_25_36_PON__MASK;
	mc_mask_bitfield[FE_SATX_8PSK_13_18] = FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_8PSK_13_18_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_8PSK_13_18_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_1_2_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_16APSK_1_2_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_16APSK_1_2_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_8_15_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_16APSK_8_15_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_16APSK_8_15_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_5_9_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_5_9_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_5_9_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_26_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_26_45_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_26_45_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_3_5] = FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_3_5_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_3_5_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_3_5_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_3_5_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_3_5_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_28_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_28_45_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_28_45_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_23_36] = FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_23_36_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_23_36_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_2_3_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_2_3_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_2_3_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_25_36] = FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_25_36_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_25_36_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_13_18] = FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_13_18_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_13_18_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_7_9] = FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_7_9_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_7_9_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_77_90] = FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_77_90_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_77_90_PON__MASK;
	mc_mask_bitfield[FE_SATX_32APSK_2_3_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_32APSK_2_3_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_32APSK_2_3_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_32APSK_32_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_32_45_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_32_45_PON__MASK;
	mc_mask_bitfield[FE_SATX_32APSK_11_15] = FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_11_15_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_11_15_PON__MASK;
	mc_mask_bitfield[FE_SATX_32APSK_7_9] = FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_7_9_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_7_9_PON__MASK;
	mc_mask_bitfield[FE_SATX_64APSK_32_45_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_32_45_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_32_45_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_64APSK_11_15] = FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_11_15_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_11_15_PON__MASK;
	mc_mask_bitfield[FE_SATX_64APSK_7_9] = FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_7_9_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_7_9_PON__MASK;
	mc_mask_bitfield[FE_SATX_64APSK_4_5] = FC8CODEW_DVBSX_DEMOD_MODCODLS17_DIS_64APSK_4_5_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS17_DIS_64APSK_4_5_PON__MASK;
	mc_mask_bitfield[FE_SATX_64APSK_5_6] = FC8CODEW_DVBSX_DEMOD_MODCODLS17_DIS_64APSK_5_6_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS17_DIS_64APSK_5_6_PON__MASK;
	mc_mask_bitfield[FE_SATX_128APSK_3_4] = FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_128APSK_3_4_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_128APSK_3_4_PON__MASK;
	mc_mask_bitfield[FE_SATX_128APSK_7_9] = FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_128APSK_7_9_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_128APSK_7_9_PON__MASK;
	mc_mask_bitfield[FE_SATX_256APSK_29_45_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_256APSK_29_45_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_256APSK_29_45_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_256APSK_2_3_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_256APSK_2_3_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_256APSK_2_3_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_256APSK_31_45_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_31_45_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_31_45_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_256APSK_32_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_32_45_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_32_45_PON__MASK;
	mc_mask_bitfield[FE_SATX_256APSK_11_15_L] = FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_11_15_L_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_11_15_L_PON__MASK;
	mc_mask_bitfield[FE_SATX_256APSK_3_4] = FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_3_4_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_3_4_PON__MASK;
	mc_mask_bitfield[FE_SATX_QPSK_11_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_11_45_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_11_45_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_QPSK_4_15] = FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_4_15_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_4_15_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_QPSK_14_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_14_45_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_14_45_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_QPSK_7_15] = FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_7_15_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_7_15_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_QPSK_8_15] = FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_QPSK_8_15_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_QPSK_8_15_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_QPSK_32_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_QPSK_32_45_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_QPSK_32_45_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_8PSK_7_15] = FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_8PSK_7_15_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_8PSK_7_15_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_8PSK_8_15] = FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_8PSK_8_15_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_8PSK_8_15_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_8PSK_26_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_8PSK_26_45_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_8PSK_26_45_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_8PSK_32_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_8PSK_32_45_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_8PSK_32_45_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_7_15] = FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_16APSK_7_15_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_16APSK_7_15_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_8_15] = FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_16APSK_8_15_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_16APSK_8_15_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_26_45_S] = FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_26_45_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_26_45_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_3_5_S] = FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_3_5_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_3_5_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_16APSK_32_45] = FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_32_45_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_32_45_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_32APSK_2_3] = FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_32APSK_2_3_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_32APSK_2_3_SH_PON__MASK;
	mc_mask_bitfield[FE_SATX_32APSK_32_45_S] = FC8CODEW_DVBSX_DEMOD_MODCODLS10_DIS_32APSK_32_45_SH_POFF__MASK + FC8CODEW_DVBSX_DEMOD_MODCODLS10_DIS_32APSK_32_45_SH_PON__MASK;

	for(i=FE_SAT_DEMOD_1;i <= FE_SAT_DEMOD_8;i++) {
		mc_reg_addr[FE_SAT_QPSK_14][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTF(i);
		mc_reg_addr[FE_SAT_QPSK_13][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTE(i);
		mc_reg_addr[FE_SAT_QPSK_25][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTE(i);
		mc_reg_addr[FE_SAT_QPSK_12][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTD(i);
		mc_reg_addr[FE_SAT_QPSK_35][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTD(i);
		mc_reg_addr[FE_SAT_QPSK_23][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTC(i);
		mc_reg_addr[FE_SAT_QPSK_34][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTC(i);
		mc_reg_addr[FE_SAT_QPSK_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTB(i);
		mc_reg_addr[FE_SAT_QPSK_56][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTB(i);
		mc_reg_addr[FE_SAT_QPSK_89][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTA(i);
		mc_reg_addr[FE_SAT_QPSK_910][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTA(i);
		mc_reg_addr[FE_SAT_8PSK_35][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST9(i);
		mc_reg_addr[FE_SAT_8PSK_23][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST9(i);
		mc_reg_addr[FE_SAT_8PSK_34][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST8(i);
		mc_reg_addr[FE_SAT_8PSK_56][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST8(i);
		mc_reg_addr[FE_SAT_8PSK_89][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST7(i);
		mc_reg_addr[FE_SAT_8PSK_910][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST7(i);
		mc_reg_addr[FE_SAT_16APSK_23][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST6(i);
		mc_reg_addr[FE_SAT_16APSK_34][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST6(i);
		mc_reg_addr[FE_SAT_16APSK_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST5(i);
		mc_reg_addr[FE_SAT_16APSK_56][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST5(i);
		mc_reg_addr[FE_SAT_16APSK_89][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST4(i);
		mc_reg_addr[FE_SAT_16APSK_910][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST4(i);
		mc_reg_addr[FE_SAT_32APSK_34][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST3(i);
		mc_reg_addr[FE_SAT_32APSK_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST3(i);
		mc_reg_addr[FE_SAT_32APSK_56][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST2(i);
		mc_reg_addr[FE_SAT_32APSK_89][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST2(i);
		mc_reg_addr[FE_SAT_32APSK_910][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLST1(i);
		mc_reg_addr[FE_SATX_QPSK_13_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1F(i);
		mc_reg_addr[FE_SATX_QPSK_9_20][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1F(i);
		mc_reg_addr[FE_SATX_QPSK_11_20][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1E(i);
		mc_reg_addr[FE_SATX_8APSK_5_9_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1E(i);
		mc_reg_addr[FE_SATX_8APSK_26_45_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1E(i);
		mc_reg_addr[FE_SATX_8PSK_23_36][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1E(i);
		mc_reg_addr[FE_SATX_8PSK_25_36][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1D(i);
		mc_reg_addr[FE_SATX_8PSK_13_18][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1D(i);
		mc_reg_addr[FE_SATX_16APSK_1_2_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1D(i);
		mc_reg_addr[FE_SATX_16APSK_8_15_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1D(i);
		mc_reg_addr[FE_SATX_16APSK_5_9_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1C(i);
		mc_reg_addr[FE_SATX_16APSK_26_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1C(i);
		mc_reg_addr[FE_SATX_16APSK_3_5][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1C(i);
		mc_reg_addr[FE_SATX_16APSK_3_5_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1C(i);
		mc_reg_addr[FE_SATX_16APSK_28_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1B(i);
		mc_reg_addr[FE_SATX_16APSK_23_36][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1B(i);
		mc_reg_addr[FE_SATX_16APSK_2_3_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1B(i);
		mc_reg_addr[FE_SATX_16APSK_25_36][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1B(i);
		mc_reg_addr[FE_SATX_16APSK_13_18][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1A(i);
		mc_reg_addr[FE_SATX_16APSK_7_9][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1A(i);
		mc_reg_addr[FE_SATX_16APSK_77_90][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1A(i);
		mc_reg_addr[FE_SATX_32APSK_2_3_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS1A(i);
		mc_reg_addr[FE_SATX_32APSK_32_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS19(i);
		mc_reg_addr[FE_SATX_32APSK_11_15][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS19(i);
		mc_reg_addr[FE_SATX_32APSK_7_9][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS19(i);
		mc_reg_addr[FE_SATX_64APSK_32_45_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS18(i);
		mc_reg_addr[FE_SATX_64APSK_11_15][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS18(i);
		mc_reg_addr[FE_SATX_64APSK_7_9][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS18(i);
		mc_reg_addr[FE_SATX_64APSK_4_5][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS17(i);
		mc_reg_addr[FE_SATX_64APSK_5_6][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS17(i);
		mc_reg_addr[FE_SATX_128APSK_3_4][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS16(i);
		mc_reg_addr[FE_SATX_128APSK_7_9][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS16(i);
		mc_reg_addr[FE_SATX_256APSK_29_45_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS16(i);
		mc_reg_addr[FE_SATX_256APSK_2_3_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS16(i);
		mc_reg_addr[FE_SATX_256APSK_31_45_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS15(i);
		mc_reg_addr[FE_SATX_256APSK_32_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS15(i);
		mc_reg_addr[FE_SATX_256APSK_11_15_L][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS15(i);
		mc_reg_addr[FE_SATX_256APSK_3_4][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS15(i);
		mc_reg_addr[FE_SATX_QPSK_11_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS14(i);
		mc_reg_addr[FE_SATX_QPSK_4_15][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS14(i);
		mc_reg_addr[FE_SATX_QPSK_14_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS14(i);
		mc_reg_addr[FE_SATX_QPSK_7_15][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS14(i);
		mc_reg_addr[FE_SATX_QPSK_8_15][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS13(i);
		mc_reg_addr[FE_SATX_QPSK_32_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS13(i);
		mc_reg_addr[FE_SATX_8PSK_7_15][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS13(i);
		mc_reg_addr[FE_SATX_8PSK_8_15][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS13(i);
		mc_reg_addr[FE_SATX_8PSK_26_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS12(i);
		mc_reg_addr[FE_SATX_8PSK_32_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS12(i);
		mc_reg_addr[FE_SATX_16APSK_7_15][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS12(i);
		mc_reg_addr[FE_SATX_16APSK_8_15][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS12(i);
		mc_reg_addr[FE_SATX_16APSK_26_45_S][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS11(i);
		mc_reg_addr[FE_SATX_16APSK_3_5_S][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS11(i);
		mc_reg_addr[FE_SATX_16APSK_32_45][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS11(i);
		mc_reg_addr[FE_SATX_32APSK_2_3][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS11(i);
		mc_reg_addr[FE_SATX_32APSK_32_45_S][i-1] = (u16)REG_RC8CODEW_DVBSX_DEMOD_MODCODLS10(i);
	}
}

/*****************************************************
--FUNCTION	::	fe_stid135_filter_forbidden_modcodes
--ACTION	::	Kills MODCODS at the output of demod vs CNR
--PARAMS IN	::	handle -> Frontend handle
			demod -> Current demod 1 .. 8
			cnr -> Carrier to noise ratio
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_filter_forbidden_modcodes(struct stv* state, s32 cnr)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	u8 row_min, row_max, found_row, i;
	u32 reg_value;

	if(state->base->ip.handle_demod->Error)
		error = FE_LLA_I2C_ERROR;
	else {
		row_min = 0;
		row_max = sizeof(state->mc_flt) / sizeof(state->mc_flt[0]) - 1;
		if(INRANGE(-300, cnr, 10000)) {
			while((row_max - row_min) > 1) {
				found_row = (u8)((row_max + row_min) >> 1);
				if(INRANGE(state->mc_flt[row_min].qef, cnr, state->mc_flt[found_row].qef))
					row_max = found_row;
				else
					row_min = found_row;
			}
		}
		/* Disable automatic filter vs. SR */
		error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST1_SYMBRATE_FILTER(demod), 0);

		/* Disable all forbidden modcodes */
		row_max = sizeof(state->mc_flt) / sizeof(state->mc_flt[0]) - 1;
		for(i = 0;i <= row_min;i++) {
			if(state->mc_flt[i].forbidden == TRUE) {
				state->mc_flt[i].forbidden = FALSE;
				error |= ChipGetOneRegister(state->base->ip.handle_demod, state->mc_flt[i].register_address, &reg_value);
				reg_value &= ~(state->mc_flt[i].mask_value);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, state->mc_flt[i].register_address, reg_value);
			}
		}
		for(i = (u8)(row_min + 1);i <= row_max;i++) {
			if(state->mc_flt[i].forbidden == FALSE) {
				state->mc_flt[i].forbidden = TRUE;
				error |= ChipGetOneRegister(state->base->ip.handle_demod, state->mc_flt[i].register_address, &reg_value);
				reg_value |= state->mc_flt[i].mask_value;
				error |= ChipSetOneRegister(state->base->ip.handle_demod, state->mc_flt[i].register_address, reg_value);
			}
		}
	}

	if (state->base->ip.handle_demod->Error ) //Check the error at the end of the function
		error = FE_LLA_I2C_ERROR;
	return error;
}

static u8 check_modcodes_mask_4bits(u8 *inventory, s32 fld_value, u8 modcode_offset, u8 index)
{
	switch(fld_value) {
		case 1:
			inventory[index++] = modcode_offset;
		break;
		case 2:
			inventory[index++] = (u8)(modcode_offset + 1);
		break;
		case 4:
			inventory[index++] = (u8)(modcode_offset + 2);
		break;
		case 8:
			inventory[index++] = (u8)(modcode_offset + 3);
		break;
	}
	return(index);
}

static u8 check_modcodes_mask_2bits(u8 *inventory, s32 fld_value, u8 modcode_offset, u8 index)
{
	switch(fld_value) {
		case 1:
			inventory[index++] = modcode_offset;
		break;
		case 2:
			inventory[index++] = (u8)(modcode_offset + 1);
		break;
	}
	return(index);
}

/*****************************************************
--FUNCTION	::	fe_stid135_modcodes_inventory
--ACTION	::	List during n ms MODCODS present in an ACM/VCM signal
--PARAMS IN	::	handle -> Front End handle
			demod -> Current demod 1 .. 8
			Duration -> function duration (ms)
--PARAMS OUT	::	MODES_Inventory -> array :list of present MODCODS
			NbModes -> Number of detected mode codes
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_modcodes_inventory(struct stv* state,
						u8 *MODES_Inventory,
						u8 *NbModes,
						u16 Duration)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	u32 modcodlst0,modcodlst1,modcodlste,modcodlstf,nresetmodcod;
	u8 Index = 0;
	s32 fld_value;
	u32 reg_value;

		if (state->base->ip.handle_demod->Error)
			error=FE_LLA_I2C_ERROR;
		else {

			modcodlst0 = FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST0_NACCES_MODCODCH(demod);
			modcodlst1 = FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST1_SYMBRATE_FILTER(demod);
			modcodlste = REG_RC8CODEW_DVBSX_DEMOD_MODCODLSTE(demod);
			modcodlstf = FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTF_MODCOD_NSTOCK(demod);
			nresetmodcod = FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST1_NRESET_MODCODLST(demod);

			error |= ChipSetField(state->base->ip.handle_demod, modcodlst1, 0);		/* disable automatic filter vs. SR */
			error |= ChipSetField(state->base->ip.handle_demod, nresetmodcod, 0);		/* Set all values to 0 */
			error |= ChipSetField(state->base->ip.handle_demod, modcodlstf, 0);		/* Set inventory mode */

			ChipWaitOrAbort(state->base->ip.handle_demod, Duration);			/* Wait for modecode occurence */

			// QPSK
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTF_DIS_QPSK_1_4(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x4, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTE_DIS_QPSK_1_3(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x8, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTE_DIS_QPSK_2_5(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0xc, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTD_DIS_QPSK_1_2(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x10, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTD_DIS_QPSK_3_5(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x14, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTC_DIS_QP_2_3(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x18, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTC_DIS_QP_3_4(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x1c, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTB_DIS_QP_4_5(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x20, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTB_DIS_QP_5_6(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x24, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTA_DIS_QP_8_9(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x28, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLSTA_DIS_QPSK_9_10(demod), &fld_value);
			Index = check_modcodes_mask_2bits(MODES_Inventory, fld_value, 0x2c, Index);
			// 8PSK
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST9_DIS_8P_3_5(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x30, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST9_DIS_8P_2_3(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x34, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST8_DIS_8P_3_4(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x38, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST8_DIS_8P_5_6(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x3c, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST7_DIS_8P_8_9(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x40, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST7_DIS_8PSK_9_10(demod), &fld_value);
			Index = check_modcodes_mask_2bits(MODES_Inventory, fld_value, 0x44, Index);
			// 16APSK
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST6_DIS_16PSK_2_3(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x48, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST6_DIS_16PSK_3_4(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x4c, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST5_DIS_16PSK_4_5(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x50, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST5_DIS_16PSK_5_6(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x54, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST4_DIS_16PSK_8_9(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x58, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST4_DIS_16PSK_9_10(demod), &fld_value);
			Index = check_modcodes_mask_2bits(MODES_Inventory, fld_value, 0x5c, Index);
			// 32APSK
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST3_DIS_32PSK_3_4(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x60, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST3_DIS_32PSK_4_5(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x64, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST2_DIS_32PSK_5_6(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x68, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST2_DIS_32PSK_8_9(demod), &fld_value);
			Index = check_modcodes_mask_4bits(MODES_Inventory, fld_value, 0x6c, Index);
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLST1_DIS_32PSK_9_10(demod), &fld_value);
			Index = check_modcodes_mask_2bits(MODES_Inventory, fld_value, 0x70, Index);
			// QPSK NF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1F_DIS_QPSK_13_45_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_QPSK_13_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1F_DIS_QPSK_13_45_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_QPSK_13_45 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1F_DIS_QPSK_9_20_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_QPSK_9_20 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1F_DIS_QPSK_9_20_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_QPSK_9_20 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_QPSK_11_20_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_QPSK_11_20 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_QPSK_11_20_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_QPSK_11_20 << 1) + 1;

			// 8PSK NF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8APSK_5_9_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8APSK_5_9_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8APSK_5_9_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_8APSK_5_9_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8APSK_26_45_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8APSK_26_45_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8APSK_26_45_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_8APSK_26_45_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8PSK_23_36_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8PSK_23_36 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1E_DIS_8PSK_23_36_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_8PSK_23_36 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_8PSK_25_36_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8PSK_25_36 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_8PSK_25_36_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_8PSK_25_36 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_8PSK_13_18_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8PSK_13_18 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_8PSK_13_18_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_8PSK_13_18 << 1) + 1;

			// 16APSK NF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_16APSK_1_2_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8PSK_13_18 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_16APSK_1_2_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_1_2_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_16APSK_8_15_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_8_15_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1D_DIS_16APSK_8_15_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_8_15_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_5_9_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_5_9_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_5_9_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_5_9_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_26_45_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_26_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_26_45_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_26_45 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_3_5_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_3_5 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_3_5_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_3_5 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_3_5_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_3_5_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1C_DIS_16APSK_3_5_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_3_5_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_28_45_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_28_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_28_45_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_28_45 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_23_36_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_23_36 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_23_36_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_23_36 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_2_3_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_2_3_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_2_3_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_2_3_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_25_36_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_25_36 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1B_DIS_16APSK_25_36_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_25_36 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_13_18_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_13_18 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_13_18_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_13_18 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_7_9_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_7_9 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_7_9_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_7_9 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_77_90_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_77_90 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_16APSK_77_90_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_77_90 << 1) + 1;

			// 32APSK NF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_32APSK_2_3_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_32APSK_2_3_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS1A_DIS_32APSK_2_3_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_32APSK_2_3_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_32_45_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_32APSK_32_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_32_45_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_32APSK_32_45 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_11_15_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_32APSK_11_15 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_11_15_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_32APSK_11_15 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_7_9_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_32APSK_7_9 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS19_DIS_32APSK_7_9_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_32APSK_7_9 << 1) + 1;

			// 64APSK NF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_32_45_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_64APSK_32_45_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_32_45_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_64APSK_32_45_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_11_15_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_64APSK_11_15 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_11_15_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_64APSK_11_15 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_7_9_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_64APSK_7_9 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS18_DIS_64APSK_7_9_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_64APSK_7_9 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS17_DIS_64APSK_4_5_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_64APSK_4_5 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS17_DIS_64APSK_4_5_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_64APSK_4_5 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS17_DIS_64APSK_5_6_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_64APSK_5_6 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS17_DIS_64APSK_5_6_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_64APSK_5_6 << 1) + 1;

			// 128APSK NF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_128APSK_3_4_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_128APSK_3_4 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_128APSK_3_4_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_128APSK_3_4 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_128APSK_7_9_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_128APSK_7_9 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_128APSK_7_9_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_128APSK_7_9 << 1) + 1;

			// 256APSK DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_256APSK_29_45_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_256APSK_29_45_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_256APSK_29_45_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_256APSK_29_45_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_256APSK_2_3_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_256APSK_2_3_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS16_DIS_256APSK_2_3_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_256APSK_2_3_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_31_45_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_256APSK_31_45_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_31_45_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_256APSK_31_45_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_32_45_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_256APSK_32_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_32_45_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_256APSK_32_45 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_11_15_L_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_256APSK_11_15_L << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_11_15_L_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_256APSK_11_15_L << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_3_4_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_256APSK_3_4 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS15_DIS_256APSK_3_4_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_256APSK_3_4 << 1) + 1;

			// QPSK SF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_11_45_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_QPSK_11_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_11_45_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_QPSK_11_45 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_4_15_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_QPSK_4_15 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_4_15_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_QPSK_4_15 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_14_45_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_QPSK_14_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_14_45_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_QPSK_14_45 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_7_15_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_QPSK_7_15 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS14_DIS_QPSK_7_15_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_QPSK_7_15 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_QPSK_8_15_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_QPSK_8_15 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_QPSK_8_15_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_QPSK_8_15 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_QPSK_32_45_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_QPSK_32_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_QPSK_32_45_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_QPSK_32_45 << 1) + 1;

			// 8PSK SF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_8PSK_7_15_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8PSK_7_15 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_8PSK_7_15_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_8PSK_7_15 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_8PSK_8_15_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8PSK_8_15 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS13_DIS_8PSK_8_15_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_8PSK_8_15 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_8PSK_26_45_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8PSK_26_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_8PSK_26_45_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_8PSK_26_45 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_8PSK_32_45_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_8PSK_32_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_8PSK_32_45_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_8PSK_32_45 << 1) + 1;

			// 16APSK SF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_16APSK_7_15_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_7_15 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_16APSK_7_15_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_7_15 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_16APSK_8_15_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_8_15 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS12_DIS_16APSK_8_15_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_8_15 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_26_45_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_26_45_S << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_26_45_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_26_45_S << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_3_5_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_3_5_S << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_3_5_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_3_5_S << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_32_45_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_16APSK_32_45 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_16APSK_32_45_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_16APSK_32_45 << 1) + 1;

			// 32APSK SF DVBS2X
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_32APSK_2_3_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_32APSK_2_3 << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS11_DIS_32APSK_2_3_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_32APSK_2_3 << 1) + 1;

			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS10_DIS_32APSK_32_45_SH_POFF(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = FE_SATX_32APSK_32_45_S << 1;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MODCODLS10_DIS_32APSK_32_45_SH_PON(demod), &fld_value);
			if (fld_value == 1)
				MODES_Inventory[Index++] = (FE_SATX_32APSK_32_45_S << 1) + 1;

			*NbModes = (u8)(Index + 1);

			/*set numeral mode */
			error |= ChipSetField(state->base->ip.handle_demod,modcodlst0,0);
			/* read mode code number sent to LDPC */
			error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)modcodlste, &reg_value);
			MODES_Inventory[Index] = (u8)reg_value;
			if (MODES_Inventory[Index]==0)
				*NbModes = (u8)(*NbModes - 1); /* Most probably the signal is not locked or not a DVB-S2 */

			error |= ChipSetField(state->base->ip.handle_demod, modcodlst0, 1);	/* back to mode code mask mode */
			error |= ChipSetField(state->base->ip.handle_demod, modcodlstf, 1);	/* back to mode code filter mode */
			error |= ChipSetField(state->base->ip.handle_demod, modcodlst1, 1);	/* enable automatic filter vs. SR */

			if (state->base->ip.handle_demod->Error ) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_soc_temperature
--ACTION	::	Gets SOC temperature in deg C
--PARAMS IN	::	handle -> Front End handle
--PARAMS OUT	::	soc_temp -> temperature
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_get_soc_temperature(struct fe_stid135_internal_param* pParams, s16 *soc_temp)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u32 reg_value;
	s32 soc_temperature;
	u8 dcorrect, data_ready;
	u16 timeout = 0;
	const u16 soc_temp_timeout = 500;

	// Reset thermal sensor block
	error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C9THS_GLUE_TOP_TSENSOR_CONFIG, 0x0);

	error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_STATUS126, &reg_value);
	dcorrect = reg_value & 0x1F;
	// If thermal sensor is not calibrated, program dcorrect with a default value
	if(dcorrect == 0) {
		error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C9THS_GLUE_TOP_TSENSOR_CONFIG, 0x10 << 5);
	} else {
		error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C9THS_GLUE_TOP_TSENSOR_CONFIG, (u32)(dcorrect << 5));
	}
	// Release reset and PDN at the same time
	error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C9THS_GLUE_TOP_TSENSOR_CONFIG, &reg_value);
	reg_value |= 0x410;
	error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C9THS_GLUE_TOP_TSENSOR_CONFIG, reg_value);
	reg_value = 0;
	error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C9THS_GLUE_TOP_TSENSOR_STATUS, &reg_value);
	data_ready = (reg_value>>10) & 0x1;
	while((data_ready != 1) && (timeout < soc_temp_timeout)) {
		timeout = (u16)(timeout + 5);
		error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_C9THS_GLUE_TOP_TSENSOR_STATUS, &reg_value);
		data_ready = (reg_value>>10) & 0x1;
	}
	if(timeout < soc_temp_timeout) {
		soc_temperature = (((s32)reg_value >> 11) & 0xFF) - 103;
		*soc_temp = (s8)soc_temperature;
	} else
		error = FE_LLA_NODATA;

	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_check_sis_or_mis
--ACTION	::	Return the MIS or SIS stream info
--PARAMS IN	::	matype -> MATYPE value
--PARAMS OUT	::	NONE
--RETURN	::	MIS or SIS TRUE : SIS, FALSE MIS
--***************************************************/
bool fe_stid135_check_sis_or_mis(u8 matype)
{

	u8 sis_or_mis;

	sis_or_mis = (matype >> 5) & 0x1;

	return (sis_or_mis);
}

/*****************************************************
--FUNCTION	::	fe_stid135_select_min_isi
--ACTION	::	Returns the MIS or SIS stream info
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 .. 8
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
static fe_lla_error_t fe_stid135_select_min_isi(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 min_isi, matype;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			/* Setup HW to store Min ISI */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL0_ISIOBS_MODE(demod), 1);

			/* Get Min ISI */
			ChipWaitOrAbort(state->base->ip.handle_demod, 30);

			error |= fe_stid135_read_hw_matype(state, &matype, &min_isi);

			/*Enable the MIS filtering*/
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_FILTER_EN(demod), 1);

			/*Set the MIS filter*/
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_ISIENTRY(demod), (u32)min_isi);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_ISIBITENA(demod), (u32)0xFF);

			/* Reset the packet delineator */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(demod), 1);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(demod), 0);

			/* Setup HW to store Current ISI */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL0_ISIOBS_MODE(demod), 0);
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_isi
--ACTION	::	Returns the ISI selected and output after filter
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demd 1 .. 8
--PARAMS OUT	::	ISI selected
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_get_isi(struct stv* state, u8 *p_isi_current)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u32 reg_value;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error)
			error=FE_LLA_I2C_ERROR;
		else {
			error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_ISIENTRY(demod), &reg_value);
			*p_isi_current = (u8)reg_value;
		}
	return error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_isi_scan
--ACTION	::	return the number and list of ISI
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 .. 8
--PARAMS OUT	::	p_isi_struct -> Struct ISI Nb of ISI and ISI numbers
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_isi_scan(struct stv* state, struct fe_sat_isi_struct_t *p_isi_struct)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 CurrentISI,matype;
	u8 i;
	u32 j=0;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error)
			error=FE_LLA_I2C_ERROR;
		else {
			// Test mode to be able to read all ISIs in MATSTR0 register,
			// otherwise only selected ISI is in MATSTR0 register
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_TPKTDELIN_TESTBUS_SELECT(demod), 8);
			/* Setup HW to store Current ISI */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL0_ISIOBS_MODE(demod), 0);
			ChipWaitOrAbort(state->base->ip.handle_demod,100);

			memset(&p_isi_struct->isi_bitset[0], 0, sizeof(p_isi_struct->isi_bitset));

			/* Get Current ISI and store in struct */
			for (i=0; i < 40; i++) {
				uint32_t mask;

				error |= fe_stid135_read_hw_matype(state, &matype, &CurrentISI);
				j = CurrentISI/32;
				mask = ((uint32_t)1)<< (CurrentISI%32);
				p_isi_struct->isi_bitset[j] |= mask;
#if 1
				if(CurrentISI == state->signal_info.isi) {
				state->signal_info.matype = matype;
				}
#endif
				ChipWaitOrAbort(state->base->ip.handle_demod,10);
			}
			// Go back to previous value of test mode
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_TPKTDELIN_TESTBUS_SELECT(demod), 0);
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_list_isi
--ACTION		::	Returns the Max and Min ISI
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 .. 8
--PARAMS OUT	::	p_min_isi -> ISI Min
			p_max_isi -> ISI Max
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_get_list_isi(struct stv* state, u8 *p_min_isi, u8 *p_max_isi)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	#if 0
		u8 matype, i=0;
	#endif
	s32 fld_value;

		if (state->base->ip.handle_demod->Error)
			error=FE_LLA_I2C_ERROR;
		else {
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_ISIMINSTR_MATYPE_ISIMIN(demod), &fld_value);
			*p_min_isi = (u8)fld_value;
			error |= ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_ISIMAXSTR_MATYPE_ISIMAX(demod), &fld_value);
			*p_max_isi = (u8)fld_value;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_select_isi
--ACTION	::	Selects an ISI for a MIS stream
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 .. 8
			isi_wanted -> Desired ISI nb
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_select_isi(struct stv* state, u8 isi_wanted)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error)
			error=FE_LLA_I2C_ERROR;
		else {
			/* Force MIS mode if MIS signal previously detected */
			if(state->mis_mode == TRUE) {
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_MATCST1_MATYPECST_SISMIS(demod), 0);
			}

			/*Enable the MIS filtering*/
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_FILTER_EN(demod), 1);

			/*Set the MIS filter*/
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_ISIBITENA(demod), 0xFF);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_ISIENTRY(demod), isi_wanted);
		}
	return error;
}



/*****************************************************
--FUNCTION	::	fe_stid135_set_mis_filtering
--ACTION	::	Set The multiple input stream filter and mask
--PARAMS IN	::	Handle -> Front End Handle
			demod -> Current demod 1 .. 8
			EnableFiltering -> enable disable the MIS filter
			mis_filter  -> MIS Filter
			mis_mask -> MIS Mask
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_mis_filtering(struct stv* state, BOOL EnableFiltering, u8 mis_filter, u8 mis_mask)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)Handle;

		if (state->base->ip.handle_demod->Error)
			error=FE_LLA_I2C_ERROR;
		else {
			if(EnableFiltering == TRUE) {
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_FILTER_EN(demod), 1); /*Enable the MIS filtering*/
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_ISIENTRY(demod), mis_filter);    /*Set the MIS filter*/
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_PKTDELIN_ISIBITENA(demod), mis_mask);			/*Set the MIS Mask*/
			} else {
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_FILTER_EN(demod), 0);	/*Disable the MIS filtering*/
			}
#ifdef LATEST
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(demod), 1);	/*reset the packet delin*/
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_PKTDELIN_PDELCTRL1_ALGOSWRST(demod), 0);	/*reset the packet delin*/
#endif
			if (state->base->ip.handle_demod->Error) /*Check the error at the end of the function*/
				error=FE_LLA_I2C_ERROR;
		}
	return error;
}


fe_lla_error_t  set_pls_mode_code(struct stv *state, u8 pls_mode, u32 pls_code)
{
	if (pls_mode == 0 && pls_code == 0)
		pls_code = 1;
	pls_mode &= 0x03;
	pls_code &= 0x3FFFF;
	state->signal_info.pls_mode= pls_mode;
	state->signal_info.pls_code= pls_code;
	//vprintk("[%d] SETTING PLS_MODE_CODE = %d %d\n", state->nr+1, pls_mode, pls_code);
	return fe_stid135_set_pls(state, pls_mode, pls_code);
}



fe_lla_error_t  set_stream_index(struct stv *state, int mis)
{
	fe_lla_error_t  err = FE_LLA_NO_ERROR;

	if (mis == NO_STREAM_ID_FILTER) {
		//dev_dbg(&state->base->i2c->dev, "%s: disable ISI filtering !\n", __func__);
		set_pls_mode_code(state, 0, 1);
		err |= fe_stid135_set_mis_filtering(state,  FALSE, 0, 0xFF);
		dprintk("SET stream_id=%d mis=0x%x",  mis &0xff, mis);
		state->signal_info.isi = 0xff;
	} else  {
		dev_warn(&state->base->i2c->dev, "%s: set ISI %d ! demod=%d tuner=%d\n", __func__, mis & 0xFF,
						 state->nr, state->rf_in);
		set_pls_mode_code(state, (mis>>26) & 0x3, (mis>>8) & 0x3FFFF);

		err |= fe_stid135_set_mis_filtering(state,  TRUE, mis & 0xFF, 0xFF);
		dprintk("SET stream_id=0x%x mis=0x%x",  mis &0xff, mis);
		state->signal_info.isi = mis &0xff;

	}
	vprintk("[%d] error=%d locked=%d\n", state->nr+1, err, state->signal_info.has_lock);
	if (err != FE_LLA_NO_ERROR)
		dev_err(&state->base->i2c->dev, "%s: fe_stid135_set_mis_filtering error %d !\n", __func__, err);
	return err;
}



/*****************************************************
--FUNCTION	::	fe_stid135_unlock
--ACTION	::	Unlock the demodulator , set the demod to idle state
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1..8
-PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_unlock(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if(state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			dprintk("XXX set to 0x1c\n");
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(demod), 0x1C); /* Demod Stop*/

			if(state->base->ip.handle_demod->Error)  /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}
	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_abort_flag
--ACTION	::	Set Abort flag On/Off
--PARAMS IN	::	handle -> Front End Handle
			abort -> boolean value
-PARAMS OUT	::	NONE
--RETURN	::	Error (if any)

--***************************************************/
fe_lla_error_t fe_stid135_set_abort_flag(struct fe_stid135_internal_param* pParams, BOOL abort)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if(pParams->handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			ChipAbort(pParams->handle_demod, abort);

			if(pParams->handle_demod->Error) /*Check the error at the end of the function*/
			error = FE_LLA_I2C_ERROR;
		}
	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_standby
--ACTION	::	Set demod STANDBY mode On/Off by accessing to the field
			STANDBY (bit[7] of the register SYNTCTRL @F1B6).
			In standby mode only the I2C clock is functional
			in order to wake-up the device
--PARAMS IN	::	handle -> Front End Handle
			standby_on -> standby ON/OFF switch
-PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_standby(struct fe_stid135_internal_param* pParams, u8 standby_on)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;
	enum fe_stid135_demod demod;
		if (pParams->handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			/* TODO : put also in standby FSK, DISEQC, AFE */

			/*Stop all Path Clocks*/
			for(demod = FE_SAT_DEMOD_1;demod <= FE_SAT_DEMOD_8;demod++) {
				error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_STOPCLK_STOP_CKDEMOD(demod), standby_on);   /*Stop Sampling clock*/
				error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_STOPCLK_STOP_CKDVBS1FEC(demod), standby_on);   /*Stop DVBS1 FEC clock*/
				error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_STOPCLK_STOP_CKDVBS2FEC(demod), standby_on);   /*Stop DVBS2 FEC clock*/
			}

			error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_SYNTCTRL_STANDBY, standby_on); /* general power ON*/

			if (pParams->handle_demod->Error) /*Check the error at the end of the function*/
				error=FE_LLA_I2C_ERROR;
		}


	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_clear_filter_ram
--ACTION	::	Clears RAM used to filter PIDs/GSE labels and protocols
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1.. 8
-PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_clear_filter_ram(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	u8 i;
	//struct fe_stid135_internal_param *pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			/* Init for PID filtering feature */
			state->pid_flt.first_disable_all_command = TRUE;
			/* Init for GSE filtering feature */
			state->gse_flt.first_disable_all_protocol_command = TRUE;
			state->gse_flt.first_disable_all_label_command = TRUE;

			/* Reset shared H/W RAM */
			for(i=0;i < RAM_SIZE;i++) {
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0x10 + (u32)i);
				error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTL(demod), 0x00);
			}
			/* Disable filter so that output everything */
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_TSPIDFLTM(demod), 0);
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_get_packet_error_rate
--ACTION	::	Returns the number of error packet and the window packet count
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 ..8
--PARAMS OUT	::	packets_error_count -> Number of packet error, max is 2^23 packet.
				total_packets_count -> total window packets , max is 2^24 packet.
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_get_packet_error_rate(struct stv* state, u32 *packets_error_count, u32 *total_packets_count)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams;
	u32 packets_count4 = 0,
		 packets_count3 = 0,
		 packets_count2 = 0,
		 packets_count1 = 0,
		 packets_count0 = 0;
	BOOL demod_locked = FALSE;


	//pParams = (struct fe_stid135_internal_param *)handle;

	if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			fe_stid135_get_data_status(state, &demod_locked);
			if(demod_locked == FALSE) { /*if Demod+FEC not locked */
				*packets_error_count = 1 << 23;		 /*Packet error count is set to the maximum value*/
				*total_packets_count = 1 << 24;		 /*Total Packet count is set to the maximum value*/
			} else {
				/*Read the error counter 2 (23 bits) */
				error |= FE_STiD135_GetErrorCount(state->base->ip.handle_demod, FE_STiD_COUNTER2, demod, packets_error_count);
				*packets_error_count &= 0x7FFFFF;

				/* Read the total packet counter 40 bits, read 5 bytes is mondatory */
				error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_FECSPY_FBERCPT4(demod), &packets_count4);    /*Read the Total packet counter byte 5*/
				error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_FECSPY_FBERCPT3(demod), &packets_count3);    /*Read the Total packet counter byte 4*/
				error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_FECSPY_FBERCPT2(demod), &packets_count2);    /*Read the Total packet counter byte 3*/
				error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_FECSPY_FBERCPT1(demod), &packets_count1);    /*Read the Total packet counter byte 2*/
				error |= ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_FECSPY_FBERCPT0(demod), &packets_count0);    /*Read the Total packet counter byte 1*/

				if( (packets_count4 == 0) && (packets_count3 == 0)) { /*Use the counter for a maximum of 2^24 packets*/
					*total_packets_count = ((packets_count2 & 0xFF) << 16) + ((packets_count1 & 0xFF) << 8) + (packets_count0 & 0xFF);
				} else {
					*total_packets_count = 1 << 24;
				}

				if(*total_packets_count == 0) {
					/* if the packets count doesn't start yet the packet error = 1 and total packets = 1 */
					/* if the packet counter doesn't start there is a FEC error */
					*total_packets_count = 1;
					*packets_error_count = 1;
				}
			}

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_FECSPY_FBERCPT4(demod), 0);    /*Reset the Total packet counter */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_FECSPY_FECSPY_BERMETER_RESET(demod), 1);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_FECSPY_FECSPY_BERMETER_RESET(demod), 0);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_HWARE_ERRCTRL2(demod), 0xc1); /*Reset the packet Error counter2 (and Set it to infinit error count mode )*/

			if (state->base->ip.handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_vcore_supply
--ACTION	::	Sets Vcore supply voltage
--PARAMS IN	::	handle -> Front End Handle
			pwm_value ->
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_vcore_supply(struct fe_stid135_internal_param* pParams)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams;
	u32 reg_value, pwm_value = 0x80, minor_cut;
	u8 avs_code;
	fe_lla_lookup_t *lookup = NULL;

	//pParams = (struct fe_stid135_internal_param *)handle;

		if (pParams->handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			lookup = &fe_stid135_avs_look_up;

			if ((lookup != NULL) && (lookup->size)) {
				// read process value in SAFMEM
				error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_STATUS100, &reg_value);
				avs_code = (u8)(reg_value & 0x1F); // process_tracking
				// Check if sample is fused. If it is fused, then reject 0 and 1 process code
				error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_STATUS98, &minor_cut);
				if(minor_cut != 0) {
					if((avs_code == 0) || (avs_code == 1)) {
						return(FE_LLA_NOT_SUPPORTED);
					}
				}
				pwm_value = (u32)lookup->table[avs_code].realval; // PWM value, linear vs AVS voltage
			}
			// Program PWM_OUT
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_PWMTIMER_PWM_VAL0, pwm_value);
			// Enable PWM
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_PWMTIMER_PWM_CTRL, 0x200);
			// Vref reference: 1.02V if internal DCDC1V8, or 1.1V if external DCDC1V8
			if(pParams->internal_dcdc == 1) {
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG36, 0x4);
			} else if(pParams->internal_dcdc == 0) {
				error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG36, 0x0);
			}
			// Program PIO1[2] in alternate function 1
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1000, &reg_value);
			reg_value |= 0x00000100;
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1000, reg_value);
			// Program PIO1[2] in output
			error |= ChipGetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, &reg_value);
			reg_value |= 0x00000004;
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, reg_value);

			if (pParams->handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_maxllr_rate
--ACTION	::	Sets the max LLR rate
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 .. 8
			maxllr_rate -> 90/129/180/258 MLLR/s
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t	fe_stid135_set_maxllr_rate(struct stv* state, u32 maxllr_rate)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams;

	//pParams = (struct fe_stid135_internal_param *)handle;
	printk("fe_stid135_set_maxllr_rate rate %d\n",maxllr_rate);
		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			if (maxllr_rate == 90) // 90 = 720Mcb/s / 8
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_SDEMAP_MAXLLRRATE(demod), 3);
			else if (maxllr_rate == 129) // 129.1667 = demod master clock
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_SDEMAP_MAXLLRRATE(demod), 2);
			else if (maxllr_rate == 180) // 180 = 720Mcb/s / 4
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_SDEMAP_MAXLLRRATE(demod), 1);
			else // 258.3333 = twice demod master clock
				error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_SDEMAP_MAXLLRRATE(demod), 0);

			if (state->base->ip.handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}

	return error;

}

/*****************************************************
--FUNCTION	::	fe_stid135_multi_normal_short_tuning
--ACTION	::	Provides an alternative set of LLR gains
			for short frames only
--PARAMS IN	::	handle -> Front End Handle
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_multi_normal_short_tuning(struct fe_stid135_internal_param* pParams)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams;

	//pParams = &state->base->ip;

		if (pParams->handle_demod->Error)
				error = FE_LLA_I2C_ERROR;

		else {
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF14, 0x28);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF15, 0x30);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF16, 0x50);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF19, 0x30);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF20, 0x40);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF21, 0x4a);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF22, 0x58);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF24, 0x80);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF25, 0x80);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF26, 0xb8);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF27, 0xe0);

			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF56_1, 0x30);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF57_0, 0x14);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF58_1, 0x28);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF61_1, 0x3d);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_C8CODEW_DVBS2FEC_GAINLLR_SF62_0, 0x58);
		}

		if (pParams->handle_demod->Error)
			error |= FE_LLA_I2C_ERROR;


	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_init_before_bb_flt_calib
--ACTION	::	Set chip ready for BB filter calibration
--PARAMS IN	::	Handle -> Front End Handle
			tuner_nb -> Current tuner 1 .. 4
			squarewave_generation -> boolean variable to
			enable/disable squarewave signal generation
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_init_before_bb_flt_calib_(struct fe_stid135_internal_param *pParams,
																										enum fe_stid135_demod Demod,
																									 FE_OXFORD_TunerPath_t tuner_nb, BOOL squarewave_generation)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;


	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGCKS(Demod), 0x02);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCNLK(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCNLK(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCLK(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCLK(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1BCFG(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B2(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B1(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1B0(Demod), 0x00);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGCFG(Demod), 0xc3);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCNLK(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCLK(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG1(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG0(Demod), 0x00);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR22(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR21(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR20(Demod), 0x00);



	/* ------------- ------------------- --------------------------------- */

	/* set bbfilt input to clock freq of (FVCO/2)/13 ~ 238.5MHz
		(programmable division by 13 in AFE) (for default FVCO=6200MHz !!!). */
	if(squarewave_generation == TRUE) {
		Oxford_StartBBcal(pParams->handle_demod, tuner_nb, 1);
	} else {
		Oxford_StopBBcal(pParams->handle_demod, tuner_nb);
	}

	/* Todo: Better to switch off the Tuner but leave the ADC working */

	/* ------------- ------------------- --------------------------------- */

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGM(Demod), 0x07);    /* switch off hd filters, set manual  */
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGL(Demod), 0x00);    /* switch off hd filters, no amplification  */


	/* ------------- AGC1 configuration --------------------------------- */
	/*  AGC1CFG: IQ mismatch control,  */

	/*  AMM_FROZEN:  compensation freeze (unsigned)
			AMM_CORRECT: compensation authorization
	*/
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_AMM_FROZEN(Demod),  1);

	/*	[3]  QUAD_FROZEN: compensation freeze (unsigned)
		[2]  QUAD_CORRECT: compensation authorization
		00:  */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CFG_QUAD_FROZEN(Demod),  1);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1AMM(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1QUAD(Demod), 0x00);

	/*	AGC1CN AGC1 control
		[2:0]  AGCIQ_BETA: AGC1 loop speed: 000: stop				*/
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1CN_AGCIQ_BETA(Demod), 0);


	/*	AGC1ADJ AGC1 loop set point	 ????  */

	/* ------------- AGC2 configuration --------------------------------- */

	/* AGC2O AGC2 configuration   */
	/* [2:0]  AGC2_COEF: AGC2 loop speed:  000: open loop, AGC2IM/L in manual */
	// gAPISetField(DEMOD, pathId, FDEMOD_P1_AGC2_COEF, 0);
	//ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC2O_AGC2_COEF(Demod), 0);

	/* AGC2IM/L							 */
	/* AGC2I1,AGC2I0 AGC2 accumulator  */

	/* AGC2ADJ Adjusted value of agc2_ref		*/
	/* AGC2ADJ_MANUAL: switch to manual mode 1: the automatic calculation is stopped, a value can be written in agc2ref_adjusted */



	/* ------------- Carrier Loop --------------------------------- */
	/* CARCFG Carrier 1 configuration								*/
	/* [2]  ROTAON: 1: carrier 1 derotator in action     0: carrier 1 loop open (unsigned)*/

	/* ------------- Timing  Loop --------------------------------- */



	/* ------------- internal settings ---------------------------- */
	switch(tuner_nb) {
		case AFE_TUNER1:
			error |= ChipSetField(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC1_CTRL_AGC1_BB_CTRL, 3);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC1_BB_PWM, 0xCF);	/* set AGC1 manually */
		break;
		case AFE_TUNER2:
			error |= ChipSetField(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC2_CTRL_AGC2_BB_CTRL, 3);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC2_BB_PWM, 0xCF);	/* set AGC1 manually */
		break;
		case AFE_TUNER3:
			error |= ChipSetField(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC3_CTRL_AGC3_BB_CTRL, 3);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC3_BB_PWM, 0xCF);	/* set AGC1 manually */
		break;
		case AFE_TUNER4:
			error |= ChipSetField(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC4_CTRL_AGC4_BB_CTRL, 3);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC4_BB_PWM, 0xCF);	/* set AGC1 manually */
		break;
		case AFE_TUNER_ALL:
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC1_CTRL_AGC1_BB_CTRL, 3);		 /* set AGC1 manually */
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC2_CTRL_AGC2_BB_CTRL, 3);		 /* set AGC1 manually */
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC3_CTRL_AGC3_BB_CTRL, 3);		 /* set AGC1 manually */
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC4_CTRL_AGC4_BB_CTRL, 3);		 /* set AGC1 manually */
			error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC1_CTRL, 4);

			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC1_BB_PWM_AGC1_BB_PWM, 0xCF);
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC2_BB_PWM_AGC2_BB_PWM, 0xCF);
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC3_BB_PWM_AGC3_BB_PWM, 0xCF);
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FSTID135_AFE_AFE_AGC4_BB_PWM_AGC4_BB_PWM, 0xCF);
			error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC1_BB_PWM, 4);
		break;
	}

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 0x40);		//idee PG 20/04/2017
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFG4(Demod), 0x80);		//idee PG 20/04/2017
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x00);		//idee PG ""

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2O(Demod), 0x00);			//idee PG ""
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(Demod), 0x40);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I0(Demod), 0x00);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_LOW_RATE(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_INPUT_MODE(Demod), 0);
	/* 0 = symbols/intersymbols */
	/* 1 = CTE */
	/* 2 = Z4 */
	/* 3 = filter calibration */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_CTE_MODE(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_SHIFT(Demod), /*1*/0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_MODE(Demod), 5);
	/* 0 = 8192 points, 1 = 4096 points */
	/* 2 = 2048 points, 3 = 1024 points */
	/* 4 = 512 points,  5 = 256 points */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTACC_NB_ACC_FFT(Demod), 255);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_THRESHOLD_NO_STOP(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_CFO_FILT(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_SEL_MEM(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_THRESHOLD_MAX_THRESHOLD(Demod), 4);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GAINCONT_MODE_CONTINUOUS(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GAINCONT_GAIN_CONTINUOUS(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_UPDCONT_UPD_CONTINUOUS(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DISABLE_RESCALE(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DEBUG_INTERSYMBOL(Demod), 1);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_DISABLE_AVERAGE(Demod), 1);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_MODE_INTERSYMBOL(Demod), 1);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(Demod), 1);
	/* Switch off PSD (Power Spectral Density) Log format*/
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_DB(Demod), 0);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_ENABLE(Demod), 1);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_AGCRF_AGCRFCFG_AGCRF_BETA(tuner_nb), 0);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFIN1(tuner_nb), 0x40);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFIN0(tuner_nb), 0x00);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_FIFO2_ULTRABS(Demod), 1); /* Priority to UFBS scan */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(Demod), 1); /* Narrowband setting */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDISTATE_I2C_DEMOD_MODE(Demod), 2); /* set PEA to UFBS mode  */
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_TNRCFG2_TUN_IQSWAP(Demod), 0); // DO NOT set to 1 !!!!

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_AGC1SHDB_ADCIN_IQOFF(Demod), 0);	/* set I input active */

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFREF(tuner_nb), 0x58);

	return(error);
}

 fe_lla_error_t fe_stid135_init_before_bb_flt_calib(struct stv* state,
																									 FE_OXFORD_TunerPath_t tuner_nb, BOOL squarewave_generation)
 {
	 	enum fe_stid135_demod Demod = state->nr+1;
		return fe_stid135_init_before_bb_flt_calib_(&state->base->ip, Demod, tuner_nb, squarewave_generation);
 }

/*****************************************************
--FUNCTION	::	fe_stid135_uninit_after_bb_flt_calib
--ACTION	::	Unset chip from BB filter calibration mode and
			switch back to normal mode
--PARAMS IN	::	Handle -> Front End Handle
			tuner_nb -> Current tuner 1 .. 4
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_uninit_after_bb_flt_calib_(struct fe_stid135_internal_param* pParams,
																										 enum fe_stid135_demod Demod,
																										FE_OXFORD_TunerPath_t tuner_nb)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGCKS(Demod), 0x00);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCNLK(Demod), 0x18);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCNLK(Demod), 0x28);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCLK(Demod), 0x18);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCLK(Demod), 0x28);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(Demod), 0x79);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(Demod), 0x18);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1BCFG(Demod), 0x67);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGCFG(Demod), 0xd3);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCNLK(Demod), 0x56);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCLK(Demod), 0x56);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG1(Demod), 0x06);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG0(Demod), 0xec);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(Demod), 0x25);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR22(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR21(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR20(Demod), 0x1c);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FILTCFGM(Demod), 0x30);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1CFG(Demod),  0x54);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1AMM(Demod), 0xff);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1QUAD(Demod), 0xfe);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1CN(Demod), 0x99);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2O(Demod), 0x5b);

	switch(tuner_nb) {
		case AFE_TUNER1:
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC1_CTRL, 0x00);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC1_BB_PWM, 0x00);
		break;
		case AFE_TUNER2:
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC2_CTRL, 0x00);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC2_BB_PWM, 0x00);
		break;
		case AFE_TUNER3:
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC3_CTRL, 0x00);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC3_BB_PWM, 0x00);
		break;
		case AFE_TUNER4:
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC4_CTRL, 0x00);
			error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RSTID135_AFE_AFE_AGC4_BB_PWM, 0x00);
			break;
	default:
		break;
	}
	error |= Oxford_StopBBcal(pParams->handle_demod, tuner_nb);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(Demod), 0xc8);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFG4(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(Demod), 0x1c);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I1(Demod), 0x6c);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2I0(Demod), 0x40);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GCTRL(Demod), 0x06);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FFTCTRL(Demod), 0xc8);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_FFTACC(Demod), 0x0a);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_THRESHOLD(Demod), 0x04);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DEBUG1(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_GAINCONT(Demod), 0x00);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_UPDCONT(Demod), 0x00);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_AGCRF_AGCRFCFG_AGCRF_BETA(tuner_nb), 1);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFIN1(tuner_nb), 0xca);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFIN0(tuner_nb), 0x10);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_HDEBITCFG0(Demod), 0x01);
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG2_MODE_HAUTDEBIT(Demod), 2);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TNRCFG2(Demod), 0x02);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC1SHDB(Demod), 0x00);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_AGCRF_AGCRFREF(tuner_nb), 0x60);

	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRUP2(Demod), 0xCE);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRUP1(Demod), 0xC9);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRUP0(Demod), 0xCC);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRLOW2(Demod), 0xCE);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRLOW1(Demod), 0x21);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRLOW0(Demod), 0x50);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT2(Demod), 0xCE);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT1(Demod), 0x75);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT0(Demod), 0x8E);
	error |= ChipSetOneRegister(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(Demod), 0xC6);
	return (fe_lla_error_t)error;
}

fe_lla_error_t fe_stid135_uninit_after_bb_flt_calib(struct stv* state,
																										FE_OXFORD_TunerPath_t tuner_nb)
{
	 	enum fe_stid135_demod Demod = state->nr+1;
		struct fe_stid135_internal_param *pParams = &state->base->ip;
	return fe_stid135_uninit_after_bb_flt_calib_(pParams, Demod, tuner_nb);

}

/*****************************************************
--FUNCTION	::	fe_stid135_read_bin_from_psd_mem
--ACTION	::	Reads PSD memory knowing bin max
--PARAMS IN	::	Handle -> Front End Handle
			bin_max -> bin number where max
			amplitude has been found
--PARAMS OUT	::	NONE
--RETURN	::	magnitude of 5 bins
--***************************************************/
fe_lla_error_t fe_stid135_read_bin_from_psd_mem(struct fe_stid135_internal_param* pParams, u32 bin_max)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	s32 fld_value;
	u32 exp, val[5]= { 0 }, memstat, nb_samples, den;
	u32 val_max_m2, val_max_m1, val_max, val_max_p1, val_max_p2;
	//struct fe_stid135_internal_param *pParams;

	// Carefull! In PSD memory, datas are 32-bits wide coded split in 2 fields:
	// (31:27); exp_max
	// (26:0): psd
	// with fft length of 256 bins, we get exp_max=0, but with higher fft lengths, exp_max is no more equal to 0
	//pParams = &state->base->ip;
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_MODE(FE_SAT_DEMOD_1), &fld_value);
	den = (u32)(XtoPowerY(2, (u32)fld_value));
	if(den != 0)
		nb_samples = (u32)(8192 / den);
	else
		return(FE_LLA_BAD_PARAMETER);
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEBUG1_MODE_FULL(FE_SAT_DEMOD_1), &fld_value);
	if(fld_value == 1) { // 32-bit mode
		if((bin_max%4) == 0) { // case bin_max divisable by 4 => 2 memory access needed
			if(bin_max == 0) {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), (((nb_samples-1)/4) >> 8) & 0x03);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), ((nb_samples-1)/4) & 0xFF);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			} else {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), ((bin_max/4-1) >> 8) & 0x03);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), (bin_max/4-1) & 0xFF);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
			memstat = (u32)fld_value;
			while (!memstat) {
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
				memstat = (u32)fld_value;
			}

			/* start computing val_max_m2 */
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(FE_SAT_DEMOD_1), &fld_value);
			exp = (u32)fld_value;
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA41(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA41_MEM_VAL4(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA40_MEM_VAL4(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA51_MEM_VAL5(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA50_MEM_VAL5(FE_SAT_DEMOD_1));
			val_max_m2 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_m2 */

			/* start computing val_max_m1 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA61(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA61_MEM_VAL6(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA60_MEM_VAL6(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA71_MEM_VAL7(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA70_MEM_VAL7(FE_SAT_DEMOD_1));
			val_max_m1 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_m1 */

			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), ((bin_max/4) >> 8) & 0x03);
			error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), (bin_max/4) & 0xFF);
			error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);

			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
			memstat = (u32)fld_value;
			while (!memstat) {
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
				memstat = (u32)fld_value;
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(FE_SAT_DEMOD_1), &fld_value);
			exp = (u32)fld_value;
			/* start computing val_max */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA11_MEM_VAL1(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA10_MEM_VAL1(FE_SAT_DEMOD_1));
			val_max = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max */

			if(bin_max == nb_samples-1) {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), 0x00);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), 0x00);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
			memstat = (u32)fld_value;
			while (!memstat) {
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
				memstat = (u32)fld_value;
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(FE_SAT_DEMOD_1), &fld_value);
			exp = (u32)fld_value;

			/* start computing val_max_p1 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA21(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA21_MEM_VAL2(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA20_MEM_VAL2(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA31_MEM_VAL3(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA30_MEM_VAL3(FE_SAT_DEMOD_1));
			val_max_p1 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_p1 */

			/* start computing val_max_p2 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA41(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA41_MEM_VAL4(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA40_MEM_VAL4(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA51_MEM_VAL5(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA50_MEM_VAL5(FE_SAT_DEMOD_1));
			val_max_p2 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_p2 */

		} else if((bin_max%4) == 1) { // case bin_max = 4*N+1
			if(bin_max == 0) {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), (((nb_samples-1)/4) >> 8) & 0x03);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), ((nb_samples-1)/4) & 0xFF);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			} else {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), ((bin_max/4) >> 8) & 0x03);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), (bin_max/4) & 0xFF);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
			memstat = (u32)fld_value;
			while (!memstat) {
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
				memstat = (u32)fld_value;
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(FE_SAT_DEMOD_1), &fld_value);
			exp = (u32)fld_value;
			/* start computing val_max_m2 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA61(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA61_MEM_VAL6(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA60_MEM_VAL6(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA71_MEM_VAL7(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA70_MEM_VAL7(FE_SAT_DEMOD_1));
			val_max_m2 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_m2 */

			/* start computing val_max_m1 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA11_MEM_VAL1(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA10_MEM_VAL1(FE_SAT_DEMOD_1));
			val_max_m1 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_m1 */

			/* start computing val_max */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA21(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA21_MEM_VAL2(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA20_MEM_VAL2(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA31_MEM_VAL3(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA30_MEM_VAL3(FE_SAT_DEMOD_1));
			val_max = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max */

			if(bin_max == nb_samples-1) {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), 0x00);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), 0x00);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
				memstat = (u32)fld_value;
				while (!memstat) {
					error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
					memstat = (u32)fld_value;
				}
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(FE_SAT_DEMOD_1), &fld_value);
				exp = (u32)fld_value;
			}
			/* start computing val_max_p1 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA41(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA41_MEM_VAL4(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA40_MEM_VAL4(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA51_MEM_VAL5(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA50_MEM_VAL5(FE_SAT_DEMOD_1));
			val_max_p1 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_p1 */

			/* start computing val_max_p2 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA61(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA61_MEM_VAL6(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA60_MEM_VAL6(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA71_MEM_VAL7(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA70_MEM_VAL7(FE_SAT_DEMOD_1));
			val_max_p2 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_p2 */

		} else if((bin_max%4) == 2) { // case bin_max = 4*N+2
			if(bin_max == 0) {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), (((nb_samples-1)/4) >> 8) & 0x03);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), ((nb_samples-1)/4) & 0xFF);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			} else {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), ((bin_max/4) >> 8) & 0x03);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), (bin_max/4) & 0xFF);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
			memstat = (u32)fld_value;
			while (!memstat) {
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
				memstat = (u32)fld_value;
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(FE_SAT_DEMOD_1), &fld_value);
			exp = (u32)fld_value;
			/* start computing val_max_m2 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA11_MEM_VAL1(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA10_MEM_VAL1(FE_SAT_DEMOD_1));
			val_max_m2 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_m2 */

			/* start computing val_max_m1 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA21(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA21_MEM_VAL2(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA20_MEM_VAL2(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA31_MEM_VAL3(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA30_MEM_VAL3(FE_SAT_DEMOD_1));
			val_max_m1 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_m1 */

			/* start computing val_max */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA41(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA41_MEM_VAL4(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA40_MEM_VAL4(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA51_MEM_VAL5(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA50_MEM_VAL5(FE_SAT_DEMOD_1));
			val_max = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max */

			if(bin_max == nb_samples-1) {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), 0x00);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), 0x00);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
				memstat = (u32)fld_value;
				while (!memstat) {
					error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
					memstat = (u32)fld_value;
				}
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(FE_SAT_DEMOD_1), &fld_value);
				exp = (u32)fld_value;
			}
			/* start computing val_max_p1 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA61(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA61_MEM_VAL6(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA60_MEM_VAL6(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA71_MEM_VAL7(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA70_MEM_VAL7(FE_SAT_DEMOD_1));
			val_max_p1 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_p1 */

			/* start computing val_max_p2 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA11_MEM_VAL1(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA10_MEM_VAL1(FE_SAT_DEMOD_1));
			val_max_p2 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_p2 */

		} else { // case bin_max = 4*N+3 => 2 memory access needed
			if(bin_max == 0) {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), (((nb_samples-1)/4) >> 8) & 0x03);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), ((nb_samples-1)/4) & 0xFF);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			} else {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), ((bin_max/4) >> 8) & 0x03);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), (bin_max/4) & 0xFF);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
			memstat = (u32)fld_value;
			while (!memstat) {
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
				memstat = (u32)fld_value;
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(FE_SAT_DEMOD_1), &fld_value);
			exp = (u32)fld_value;
			/* start computing val_max_m2 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA21(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA21_MEM_VAL2(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA20_MEM_VAL2(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA31_MEM_VAL3(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA30_MEM_VAL3(FE_SAT_DEMOD_1));
			val_max_m2 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_m2 */

			/* start computing val_max_m1 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA41(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA41_MEM_VAL4(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA40_MEM_VAL4(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA51_MEM_VAL5(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA50_MEM_VAL5(FE_SAT_DEMOD_1));
			val_max_m1 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_m1 */

			/* start computing val_max */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA61(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA61_MEM_VAL6(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA60_MEM_VAL6(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA71_MEM_VAL7(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA70_MEM_VAL7(FE_SAT_DEMOD_1));
			val_max = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max */

			if(bin_max == nb_samples-1) {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), 0x00);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), 0x00);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			} else {
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR1_MEM_ADDR(FE_SAT_DEMOD_1), ((bin_max/4+1) >> 8) & 0x03);
				error |= ChipSetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMADDR0_MEM_ADDR(FE_SAT_DEMOD_1), (bin_max/4+1) & 0xFF);
				error |= ChipSetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMADDR1(FE_SAT_DEMOD_1), 2);
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
			memstat = (u32)fld_value;
			while (!memstat) {
				error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMSTAT_MEM_STAT(FE_SAT_DEMOD_1), &fld_value);
				memstat = (u32)fld_value;
			}
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_EXPMAX_EXP_MAX(FE_SAT_DEMOD_1), &fld_value);
			exp = (u32)fld_value;
			/* start computing val_max_p1 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA01(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA01_MEM_VAL0(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA00_MEM_VAL0(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA11_MEM_VAL1(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA10_MEM_VAL1(FE_SAT_DEMOD_1));
			val_max_p1 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_p1 */

			/* start computing val_max_p2 */
			error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_MEMVA21(FE_SAT_DEMOD_1), 4);
			val[3] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA21_MEM_VAL2(FE_SAT_DEMOD_1));
			val[2] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA20_MEM_VAL2(FE_SAT_DEMOD_1));
			val[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA31_MEM_VAL3(FE_SAT_DEMOD_1));
			val[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_MEMVA30_MEM_VAL3(FE_SAT_DEMOD_1));
			val_max_p2 = (((val[3] & 0x7) << 24) + (val[2] << 16) + (val[1] << 8) + val[0]) * (u32)(XtoPowerY(2, exp));
			/* end computing val_max_p2 */
		}
		return(val_max_m2+val_max_m1+val_max+val_max_p1+val_max_p2);
	} else { // to be implemented for 16-bit
	}
	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_measure_harmonic
--ACTION	::	Performs a fft on a specified frequency/harmonic
--PARAMS IN	::	Handle -> Front End Handle
			desired_frequency -> frequency at which fft has to be performed (Hz)
			harmonic -> number of harmonic (usually 1 or 3)
			amplitude has been found
--PARAMS OUT	::	Magnitude based on 5 bins
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_measure_harmonic(struct fe_stid135_internal_param *pParams,
																					 enum fe_stid135_demod Demod, u32 desired_frequency,
																					 u8 harmonic, u32 *val_5bin)
{
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	s32 fld_value = 0, contmode;
	u32 timeout = 0;
	u32 binmax[2], peak_bin;
	s32 tuner_frequency;
	u32 demod_symbol_rate;
	u32 demod_search_range_hz;
	/* --------- measure harmonic -------------*/
	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_RESTART(Demod), 1);

	if(pParams->lo_frequency == 0) /* LO frequency not yet computed */
		error |= FE_STiD135_GetLoFreqHz(pParams, &(pParams->lo_frequency));
	tuner_frequency = (s32)(desired_frequency * harmonic - pParams->lo_frequency * 1000000 + 4000000); /* offset of 4MHz to avoid "battement" issue */
	demod_search_range_hz = 30000000;
	error |= fe_stid135_set_carrier_frequency_init_(pParams, Demod, tuner_frequency, demod_search_range_hz);

	demod_symbol_rate = 1*20*1000000;
	error |= fe_stid135_set_symbol_rate_(pParams, Demod, demod_symbol_rate);

	error |= ChipSetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GCTRL_UFBS_RESTART(Demod), 0); /* run acquisition 1h and wait until done */
	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GAINCONT_MODE_CONTINUOUS(Demod), &fld_value);
	if (!fld_value) {
		error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GSTAT_PSD_DONE(Demod), &contmode);
		while((contmode != TRUE) && (timeout < 40)){
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GSTAT_PSD_DONE(Demod), &contmode);
			timeout = (u8)(timeout + 1);
			ChipWaitOrAbort(pParams->handle_demod, 1);
		}
	}

	error |= ChipGetRegisters(pParams->handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BINMAX1(Demod), 2);
	binmax[0] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_BINMAX0_BIN_MAX(Demod));
	binmax[1] = (u32)ChipGetFieldImage(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_BINMAX1_BIN_MAX(Demod));
	peak_bin = ((binmax[1] & 0x1F) << 8) + binmax[0];
	*val_5bin = fe_stid135_read_bin_from_psd_mem(pParams, peak_bin);
	return(error);
}

/*****************************************************
--FUNCTION	::	fe_stid135_bb_flt_calib
--ACTION	::	Calibration algorithm which should be performed
			at power-up for each RF input
--PARAMS IN	::	Handle -> Front End Handle
			tuner_nb -> Current tuner 1 .. 4
--PARAMS OUT	::	NONE
--RETURN	::	error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_bb_flt_calib_(struct fe_stid135_internal_param *pParams,
																				enum fe_stid135_demod Demod, FE_OXFORD_TunerPath_t tuner_nb)
{
	//STCHIP_Info_t* hChip = pParams->handle_demod;
	s32 fld_value = 0;
	u32 measure_h1, measure_h3;
	u32 ratio = 0, ratio_threshold = 174; /* 174 because 10xlog(174/10)=12.4dB */
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	BOOL calib_value_found = FALSE;
	s32 epsilon = 5;
	s32 start_index = 0;
	s32 end_index;
	s32 middle_index = 0;

	s32 best_i_cal = 0x00, best_q_cal = 0x00;

	error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_FFTCTRL_FFT_MODE(FE_SAT_DEMOD_1), &fld_value);
	end_index = (s32)(8192 / XtoPowerY(2, (u32)fld_value)) - 1;
	/* Search loop (dichotomy method) */
	while(!calib_value_found && ((end_index - start_index) > 1)) {

		middle_index = (start_index + end_index)/2;  // we set middle index for ical

		error |= Oxford_StoreBBcalCode(pParams->handle_demod, tuner_nb, (u8)middle_index, (u8)(middle_index>>6));
		error |= fe_stid135_measure_harmonic(pParams, FE_SAT_DEMOD_1, SQUAREWAVE_FREQUENCY, 1, &measure_h1);
		error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GSTAT_PEAK_FOUND(FE_SAT_DEMOD_1), &fld_value);
		if(fld_value == 1) {
			error |= fe_stid135_measure_harmonic(pParams, FE_SAT_DEMOD_1, SQUAREWAVE_FREQUENCY, 3, &measure_h3);
			error |= ChipGetField(pParams->handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_GSTAT_PEAK_FOUND(FE_SAT_DEMOD_1), &fld_value);
			if(fld_value == 1) {
				ratio = (measure_h1 * 10) / measure_h3;
			}
		}

		calib_value_found = (ABS((s32)(ratio - ratio_threshold)) <= epsilon);

		if(ratio < ratio_threshold) end_index = middle_index;  // if value which is at ical = im is
		// greater than desired one, end index becomes middle index, therefore search range
		// is narrower on next turn
		else start_index = middle_index;  // otherwise start index becomes middle index
		//	and range is also narrower
	}
	if(ABS((s32)(ratio - ratio_threshold)) <= epsilon) { // if we find good value, we stop loop
		best_i_cal = middle_index;
		best_q_cal = middle_index >> 6;
	} else { // otherwise we set default value for ical
		best_i_cal = 0xC0;
		best_q_cal = 0x3;
	}

	Oxford_StoreBBcalCode(pParams->handle_demod, tuner_nb, (u8)best_i_cal, (u8)best_q_cal);
	return(error);
}

 fe_lla_error_t fe_stid135_bb_flt_calib(struct stv* state, FE_OXFORD_TunerPath_t tuner_nb)
 {
	 enum fe_stid135_demod Demod = state->nr+1;
	 struct fe_stid135_internal_param *pParams = &state->base->ip;
	 return fe_stid135_bb_flt_calib_(pParams, Demod, tuner_nb);
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_gold_code
--ACTION	::	Management of PLS scrambling
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 ..8
			goldcode -> DVB-S2 gold code
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_gold_code(struct stv* state, u32 goldcode)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = &state->base->ip;
		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			/* plscramb_root is the DVBS2 gold code. The root of PRBS X is computed internally */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_MODE(demod), 1);
			/* DVBS2 Gold Code */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_ROOT(demod), (goldcode >> 16) & 0x3);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT1_PLSCRAMB_ROOT(demod), (goldcode >> 8) & 0xFF);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT0_PLSCRAMB_ROOT(demod), goldcode & 0xFF);

			if (state->base->ip.handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}
	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_goldcold_root
--ACTION	::	Management of PLS scrambling
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 ..8
			goldcode_root -> root of goldcode
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_goldcold_root(struct stv* state, u32 goldcode_root)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = &state->base->ip;

		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			/* plscramb_root is the root of PRBS X */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_MODE(demod), 0);
			/* Root of PRBS X */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_ROOT(demod), (goldcode_root >> 16) & 0x3);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT1_PLSCRAMB_ROOT(demod), (goldcode_root >> 8) & 0xFF);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT0_PLSCRAMB_ROOT(demod), goldcode_root & 0xFF);

			if (state->base->ip.handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_disable_goldcold
--ACTION	::	Disable management of PLS scrambling
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 ..8
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_disable_goldcold(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams = &state->base->ip;

	if(pParams != NULL) {
		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			/* When writing 1 in PLROOT field, we disable goldcode feature */
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_MODE(demod), 0);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_ROOT(demod), 0x00);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT1_PLSCRAMB_ROOT(demod), 0x00);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT0_PLSCRAMB_ROOT(demod), 0x01);

			if (state->base->ip.handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}
	} else
		error = FE_LLA_INVALID_HANDLE;

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_pls
--ACTION	::	Management of PLS scrambling
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 ..8
			pls_mode -> scrambling mode
			pls_code -> scrambling code
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_pls(struct stv* state, u8 mode, u32 code)
{
	enum fe_stid135_demod demod = state->nr+1;
	unsigned error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = &state->base->ip;

	if (state->base->ip.handle_demod->Error) {
		error = FE_LLA_I2C_ERROR;
		dprintk("PLS NOT SET\n");
	}else {
			ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_MODE(demod), (mode) & 0x3);
			ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_ROOT(demod), (code >> 16) & 0x3);
			ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT1_PLSCRAMB_ROOT(demod), (code >> 8) & 0xFF);
			ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT0_PLSCRAMB_ROOT(demod), code & 0xFF);

			if (state->base->ip.handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}

	return (fe_lla_error_t)error;
}

fe_lla_error_t fe_stid135_get_pls(struct stv* state, u8 *p_mode, u32 *p_code )
{
	enum fe_stid135_demod demod = state->nr+1;
	unsigned error = FE_LLA_NO_ERROR;
	//struct fe_stid135_internal_param *pParams = &state->base->ip;
		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			error = ChipGetRegisters(state->base->ip.handle_demod, REG_RC8CODEW_DVBSX_DEMOD_PLROOT2(demod),3);
			*p_mode = ChipGetFieldImage(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_MODE(demod));
			*p_code = (ChipGetFieldImage(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT2_PLSCRAMB_ROOT(demod)) << 16) |
					(ChipGetFieldImage(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT1_PLSCRAMB_ROOT(demod)) << 8) |
					(ChipGetFieldImage(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_PLROOT0_PLSCRAMB_ROOT(demod)));
			if (state->base->ip.handle_demod->Error) /*Check the error at the end of the function*/
				error = FE_LLA_I2C_ERROR;
		}

	return (fe_lla_error_t)error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_enable_constel_display
--ACTION	::	Set constellation display
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 ..8
			measuring_point -> Selection of measuring point
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_enable_constel_display(struct stv* state, u8 measuring_point)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams = &state->base->ip;

	//pParams = (struct fe_stid135_internal_param *)handle;


		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			// Output constellation display samples at symbol rate
			// Test bus out and constellation point
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TEST_OUT, 1);
			switch(demod) {
				case FE_SAT_DEMOD_1:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(1), 0);
					//which demodulator (master demod?)
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
				case FE_SAT_DEMOD_2:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(1), 1);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
				case FE_SAT_DEMOD_3:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(2), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 1);
				break;
				case FE_SAT_DEMOD_4:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(2), 1);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 1);
				break;
				case FE_SAT_DEMOD_5:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(3), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 2);
				break;
				case FE_SAT_DEMOD_6:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(3), 1);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 2);
				break;
				case FE_SAT_DEMOD_7:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(4), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 3);
				break;
				case FE_SAT_DEMOD_8:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(4), 1);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 3);
				break;
			}

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TCTL1(demod), measuring_point);
			// MLCK on FE_GPIO14
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_C8CODEW_TOP_CTRL_GPIO14CFG, 0x7E);
			// I/Q constellation output (Alternate 5)
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, 0x00555555);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, 0x00555555);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, 0x00555555);
			// MLCK output: SoC alternate1
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, 0x00001000);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1103, 0x00000004);
			// Output enable: all for test
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, 0x3FF01F1B);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, 0x3F3F3F0F);
			// Enable 270MHz clock which is mandatory in DVBS
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDCFG4_DIS_CLKENABLE(demod), 1);

			if (state->base->ip.handle_demod->Error) /* Check the error at the end of the function */
				error = FE_LLA_I2C_ERROR;
		}

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_disable_constel_display
--ACTION	::	Unset constellation display
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 ..8
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_disable_constel_display(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	//	pParams = (struct fe_stid135_internal_param *)handle;
		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			// Output constellation display samples at symbol rate
			// Test bus out and constellation point
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TEST_OUT, 0);
			switch(demod) {
				case FE_SAT_DEMOD_1:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(1), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
				case FE_SAT_DEMOD_2:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(1), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
				case FE_SAT_DEMOD_3:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(2), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
				case FE_SAT_DEMOD_4:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(2), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
				case FE_SAT_DEMOD_5:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(3), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
				case FE_SAT_DEMOD_6:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(3), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
				case FE_SAT_DEMOD_7:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(4), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
				case FE_SAT_DEMOD_8:
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_PATH_CTRL_GEN_TBUSSEL_TBUS_SELECT(4), 0);
					error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_C8CODEW_TOP_CTRL_TSTOUT_TS, 0);
				break;
			}

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TCTL1(demod), 0x00);
			// MLCK on FE_GPIO14
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_C8CODEW_TOP_CTRL_GPIO14CFG, 0x82);
			// I/Q constellation output (Alternate 5)
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002, 0x00000000);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003, 0x00000000);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1003, 0x00000000);
			// MLCK output: SoC alternate1
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1000, 0x00000000);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1103, 0x00000000);
			// Output enable: all for test
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_NORTH_SYSTEM_CONFIG1040, 0x00000000);
			error |= ChipSetOneRegister(pParams->handle_soc, (u16)REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040, 0x00000000);
			// Enable 270MHz clock which is mandatory in DVBS
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DMDCFG4_DIS_CLKENABLE(demod), 0);

			if (state->base->ip.handle_demod->Error) /* Check the error at the end of the function */
				error = FE_LLA_I2C_ERROR;
		}

	return error;
}

/*****************************************************
--FUNCTION	::	fe_stid135_set_vtm
--ACTION	::	Management of Versatile Tuner Mode. This versatile
tuner mode down converts the carrier from a given frequency and outputs I/Q
samples at a given sample rate and therefore bandwidth. This allows any
frequency within the input band (950 to 2150 MHz) to be demodulated, filtered
and output as samples or symbols
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 ..8
			frequency_hz -> RF frequency (Hz)
			symbol_rate -> Symbol Rate (Symbol/s)
			roll_off -> Roll-Off
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_set_vtm(struct stv* state,
	u32 frequency_hz, u32 symbol_rate, enum fe_sat_rolloff roll_off)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	struct fe_stid135_internal_param *pParams = &state->base->ip;
	//	struct fe_stid135_internal_param *pParams;

	//pParams = (struct fe_stid135_internal_param *)handle;

		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALS2_ROLLOFF(demod), 1); // manual roll-off
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(demod), roll_off); // roll-off selection
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(demod), 0x40); // DVB-S only
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFG3(demod), 0x0A);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFG4(demod), 0x04);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_HDEBITCFG2(demod), 0x54); // Simple wide band
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_HDEBITCFG0(demod), 0x08);

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2O(demod), 0x03); // AGC set-up: AGC2 remains active
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGCKS(demod), 0x02);

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(demod), 0x06); // Carrier loop frozen
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCNLK(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCNLK(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCLK(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCLK(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1BCFG(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINC2(demod), 0x80);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINC1(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINC0(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(demod), 0x00);

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCNLK(demod), 0x00); // Timing loop frozen
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCLK(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGCFG3(demod), 0x16);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG1(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG0(demod), 0x00);

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_VITERBI_VITSCALE(demod), 0x04); // Prevents Viterbi block from locking
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_VITERBI_VTH78(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_VITERBI_PRVIT(demod), 0x60);
			//select intersymol output
			error |= fe_stid135_enable_constel_display(state, 5);

			// Program carrier offset
			error |= FE_STiD135_GetLoFreqHz(pParams, &(state->base->ip.lo_frequency));
			state->tuner_frequency = (s32)(frequency_hz - state->base->ip.lo_frequency*1000000);
			state->demod_search_range_hz = 60000000;
			error |= fe_stid135_set_carrier_frequency_init(state, (s32)frequency_hz);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(demod), 0x06); // Carrier loop frozen, one more time because fe_stid135_set_carrier_frequency_init() function writes 0x46 in it
			// Program SR
			error |= fe_stid135_set_symbol_rate(state, symbol_rate);
			// Launch a warm start (AEP = 0x18)
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDISTATE(demod), 0x18);
			// OPTION (should be automatic): Wait 100000 symbols and copy P1_CFRINIT2/1/0 values in P1_CFR2/1/0
			/*WAIT_N_MS(100); // to be adapted
			ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT2(demod), &reg_value);
			ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR12(demod), reg_value);
			ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT1(demod), &reg_value);
			ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR11(demod), reg_value);
			ChipGetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINIT0(demod), &reg_value);
			ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR10(demod), reg_value);*/

			if (state->base->ip.handle_demod->Error) /* Check the error at the end of the function */
				error = FE_LLA_I2C_ERROR;
		}

	return error;
}


/*****************************************************
--FUNCTION	::	fe_stid135_unset_vtm
--ACTION	::	Management of Versatile Tuner Mode. Restores the
context of demod register settings
--PARAMS IN	::	handle -> Front End Handle
			demod -> Current demod 1 ..8
--PARAMS OUT	::	NONE
--RETURN	::	Error (if any)
--***************************************************/
fe_lla_error_t fe_stid135_unset_vtm(struct stv* state)
{
	enum fe_stid135_demod demod = state->nr+1;
	fe_lla_error_t error = FE_LLA_NO_ERROR;
	//	struct fe_stid135_internal_param *pParams = &state->base->ip;


		if (state->base->ip.handle_demod->Error)
			error = FE_LLA_I2C_ERROR;
		else {
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_MANUALS2_ROLLOFF(demod), 0);
			error |= ChipSetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_DEMOD_ROLLOFF_CONTROL(demod), 0);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFGMD(demod), 0xC8);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFG3(demod), 0x08);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_DMDCFG4(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_HDEBITCFG2(demod), 0x94);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_HDEBITCFG0(demod), 0x01);

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGC2O(demod), 0x5B);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_AGCKS(demod), 0x00);

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARCFG(demod), 0xC6);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCNLK(demod), 0x28);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCNLK(demod), 0x18);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_ACLCLK(demod), 0x28);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_BCLCLK(demod), 0x18);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARFREQ(demod), 0x79);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CARHDR(demod), 0x18);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR1BCFG(demod), 0x67);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINC2(demod), 0x05);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINC1(demod), 0x29);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFRINC0(demod), 0x58);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_CFR2CFR1(demod), 0x25);

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCNLK(demod), 0x56);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_RTCLK(demod), 0x56);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGCFG3(demod), 0x06);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG2(demod), 0x40);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG1(demod), 0x06);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_DEMOD_TMGREG0(demod), 0xEC);

			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_VITERBI_VITSCALE(demod), 0x00);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_VITERBI_VTH78(demod), 0x28);
			error |= ChipSetOneRegister(state->base->ip.handle_demod, (u16)REG_RC8CODEW_DVBSX_VITERBI_PRVIT(demod), 0xBF);

			if (state->base->ip.handle_demod->Error) /* Check the error at the end of the function */
				error = FE_LLA_I2C_ERROR;
		}

	return error;
}

STCHIP_Error_t stvvglna_init(SAT_VGLNA_Params_t *InitParams, STCHIP_Info_t* *hChipHandle)
{
	STCHIP_Info_t* hChip = NULL;
	STCHIP_Error_t error = CHIPERR_NO_ERROR;
	u32 reg_value;
	u8 i;

	STCHIP_Register_t DefSTVVGLNAVal[STVVGLNA_NBREGS]=
	{
		{ RSTVVGLNA_REG0, 0x20 },
		{ RSTVVGLNA_REG1, 0x0F },
		{ RSTVVGLNA_REG2, 0x50 },
		{ RSTVVGLNA_REG3, 0x20 }
	};


		/* fill elements of external chip data structure */
		InitParams->Chip->NbInsts  = DEMOD_NBINSTANCES;
		InitParams->Chip->NbRegs   = STVVGLNA_NBREGS;
		InitParams->Chip->NbFields = STVVGLNA_NBFIELDS;
		InitParams->Chip->ChipMode = STCHIP_MODE_I2C2STBUS;
		InitParams->Chip->pData = NULL;

		InitParams->Chip->WrStart  = RSTVVGLNA_REG0;	// Default mode for STBus interface A2D1S1 : many issues such as: bad accuracy of SR and freq, not possible to lock demods#5 to #8 if blind search and SR>20MS/s
		InitParams->Chip->WrSize  = STVVGLNA_NBREGS;
		InitParams->Chip->RdStart = RSTVVGLNA_REG0;
		InitParams->Chip->RdSize = STVVGLNA_NBREGS;	//	ChipSetMapRegisterSize(STCHIP_REGSIZE_8);


		(*hChipHandle) = ChipOpen(InitParams->Chip);
		chip_init_proc(*hChipHandle, "vglna");
		hChip=(*hChipHandle);

		if(hChip != NULL)
		{
			/*******************************
			**   CHIP REGISTER MAP IMAGE INITIALIZATION
			**     ----------------------
			********************************/
			ChipUpdateDefaultValues(hChip,DefSTVVGLNAVal);
			BUG_ON(STVVGLNA_NBREGS > hChip->NbRegs);
			for(i=0;i<STVVGLNA_NBREGS;i++)
				hChip->pRegMapImage[i].Size = STCHIP_REGSIZE_8;

			error = ChipGetOneRegister(hChip,RSTVVGLNA_REG0, &reg_value);
			hChip->ChipID = (u8)reg_value;
			printk("VGLNA Chip id =0x%x\n",hChip->ChipID );
			error = hChip->Error;
		}


	return error;

}


STCHIP_Error_t stvvglna_set_standby(STCHIP_Info_t* hChip, U8 StandbyOn)
{
	STCHIP_Error_t error = CHIPERR_NO_ERROR;

	printk("stvvglan standby\n");
	if(hChip!=NULL)
	{
		if(StandbyOn)
		{
			error = ChipSetField(hChip,FSTVVGLNA_LNAGCPWD,1);
		}
		else
		{
			error = ChipSetField(hChip,FSTVVGLNA_LNAGCPWD,0);
		}

	}
	else
		error = CHIPERR_INVALID_HANDLE;

	return error;
}

STCHIP_Error_t stvvglna_get_status(STCHIP_Info_t* hChip, U8 *Status)
{
	STCHIP_Error_t error = CHIPERR_NO_ERROR;
	s32 flagAgcLow,flagAgcHigh;

	if(hChip!=NULL)
	{
		error = ChipGetField(hChip, FSTVVGLNA_RFAGCHIGH, &flagAgcHigh);
		error = ChipGetField(hChip,FSTVVGLNA_RFAGCLOW, &flagAgcLow);

		if(flagAgcHigh)
		{
			(*Status)=VGLNA_RFAGC_HIGH;
		}
		else if(flagAgcLow)
		{
			(*Status)=(U8)VGLNA_RFAGC_LOW;
		}
		else
			(*Status)=(U8)VGLNA_RFAGC_NORMAL;

		error = ChipGetError(hChip);
	}
	else
		error = CHIPERR_INVALID_HANDLE;


	return error;
}


STCHIP_Error_t stvvglna_get_gain(STCHIP_Info_t* hChip,S32 *Gain)
{

	S32 VGO, swlnaGain;
	STCHIP_Error_t error = CHIPERR_NO_ERROR;

	if(hChip != NULL)
	{
		error = ChipGetField(hChip, FSTVVGLNA_VGO, &VGO);
		error = ChipGetError(hChip);

		/*Trig to read the VGO and SWLNAGAIN val*/
		error = ChipSetFieldImage(hChip,FSTVVGLNA_GETAGC,1);
		error = ChipSetRegisters(hChip,RSTVVGLNA_REG1,1);
		WAIT_N_MS(5);

		/*(31-VGO[4:0]) * 13/31 + (3-SWLNAGAIN[1:0])*6 */
		error = ChipGetField(hChip,FSTVVGLNA_VGO, &VGO);
		error = ChipGetField(hChip,FSTVVGLNA_SWLNAGAIN, &swlnaGain);
		(*Gain)=(31-VGO)*13;
		(*Gain)/=31;
		(*Gain)+=(6*(3-swlnaGain));
	}
	else
		error = CHIPERR_INVALID_HANDLE;

	return error;
}


STCHIP_Error_t stvvglna_term(STCHIP_Info_t* hChip)
{
	STCHIP_Error_t error = CHIPERR_NO_ERROR;

	if(hChip)
	{
#ifndef ST_OSLINUX
		if(hChip->pData)
			free(hChip->pData);
		chip_close_proc("vglna");
		ChipClose(hChip);
#endif
	}

	return error;
}


fe_lla_error_t get_raw_bit_rate(struct stv* state, s32 *raw_bit_rate)
{
	s32 fld_value[2];
	struct fe_stid135_internal_param * pParams = &state->base->ip;
	// Bit rate = Mclk * tsfifo_bitrate / 16384

	ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSBITRATE1_TSFIFO_BITRATE(state->nr+1), &(fld_value[0]));
	ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSBITRATE0_TSFIFO_BITRATE(state->nr+1), &(fld_value[1]));

	*raw_bit_rate = ((fld_value[0]) << 8) + (fld_value[1]);
	FE_STiD135_GetMclkFreq(pParams, (u32*)&(fld_value[0]));
	*raw_bit_rate = (s32)(((fld_value[0])/16384)) * *raw_bit_rate;
	vprintk("[%d] Bit rate = %d Mbits/s\n", state->nr+1, *raw_bit_rate/1000000);

	return FE_LLA_NO_ERROR;
}

static int bits_per_sample(int modcode) {
	if (modcode >= FE_SAT_DVBS1_QPSK_12 ) {
		if(modcode <= FE_SAT_DVBS1_QPSK_78)
			return 2; //qpsk
	} else if(modcode <=  FE_SAT_32APSK_910 ) {
		if(modcode <= FE_SAT_QPSK_910)
			return 2; //qpsk
		if(modcode <= FE_SAT_8PSK_910 )
			return 3; //psk8
		if(modcode <=  FE_SAT_16APSK_910)
			return 4; //16APSK
		if(modcode <=  FE_SAT_32APSK_910)
			return 5; //32APSK
	} else {
		return 3; //todo: dvbsx codes
	}
	return 3; //default
}

fe_lla_error_t get_current_llr(struct stv* state, s32 *current_llr)
{
	s32 max_llr_allowed, raw_bit_rate;
	s32 fld_value[2];
	struct fe_stid135_internal_param * pParams = &state->base->ip;
	// Bit rate = Mclk * tsfifo_bitrate / 16384
	ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSBITRATE1_TSFIFO_BITRATE(state->nr+1), &(fld_value[0]));
	ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_HWARE_TSBITRATE0_TSFIFO_BITRATE(state->nr+1), &(fld_value[1]));

	raw_bit_rate = ((fld_value[0]) << 8) + (fld_value[1]);
	FE_STiD135_GetMclkFreq(pParams, (u32*)&(fld_value[0]));
	raw_bit_rate = (s32)(((fld_value[0])/16384)) * raw_bit_rate;
	vprintk("[%d] Bit rate = %d Mbits/s\n", state->nr+1, raw_bit_rate/1000000);
	*current_llr = 0;


	// LLR = TS bitrate * 1 / PR

	if(state->signal_info.modcode != 0) {
		if(state->signal_info.standard == FE_SAT_DVBS2_STANDARD) {
			bool vcm = !((state->signal_info.matype >> 4) & 1);
			int b = bits_per_sample(state->signal_info.modcode);
			int tmp = (state->signal_info.symbol_rate * b);
			dprintk("LLR sr=%d vcm=%d bps=%d raw_bit_rate=%d/%d \n", state->signal_info.symbol_rate, vcm, b, raw_bit_rate/1000, tmp/1000);
			if(state->signal_info.modcode < sizeof(dvbs2_modcode_for_llr_x100)/sizeof(dvbs2_modcode_for_llr_x100[0])
				 && (state->signal_info.modcode>=0) //just in case...
				 )
				*current_llr = (raw_bit_rate / 100) * dvbs2_modcode_for_llr_x100[state->signal_info.modcode];
			else {
				dprintk("[%d] Encountered unknown modcode for DVBS2: %d", state->nr+1, state->signal_info.modcode);
			}
			if (*current_llr < tmp) {
				//prevent freezes on RAI multistreams. The approach works but is not sufficient as modulation may vary
				dprintk("Increasing LLR from %d to %d\n", *current_llr, tmp);
				*current_llr = tmp;
			}
		}
		if(state->signal_info.standard == FE_SAT_DVBS1_STANDARD) {
			if(state->signal_info.puncture_rate < sizeof(dvbs1_modcode_for_llr_x100)
				 && (state->signal_info.modcode>=0) //just in case...
				 )  {
					 *current_llr = (raw_bit_rate / 100) * dvbs1_modcode_for_llr_x100[state->signal_info.puncture_rate];
			} else {
				dprintk("[%d] Encountered unknown puncture rate for DVBS1: %d", state->nr+1, state->signal_info.puncture_rate);
			}
		}
		if(*current_llr != 0)
			dprintk("[%d] Current LLR  = %d MLLR/s raw_bit_rate=%d modcode=%d dvbs1=%d\n", state->nr+1, *current_llr/1000000,
							raw_bit_rate,  state->signal_info.modcode, state->signal_info.standard == FE_SAT_DVBS1_STANDARD );
		else
			dprintk("[%d] LLR unknown\n", state->nr+1);

		bool vcm = !((state->signal_info.matype >> 4) & 1);
		if(false && vcm) //todo
			fe_stid135_set_maxllr_rate(state, 250); //disable limits; test above (rai multistreams) could still be wrong in case multiple modcodes
		else if((*current_llr/1000)<80000)
			fe_stid135_set_maxllr_rate(state, 90);
		else if(((*current_llr/1000)>80000)&&((*current_llr/1000)<113000))
			fe_stid135_set_maxllr_rate(state, 129);
		else if(((*current_llr/1000)>113000)&&((*current_llr/1000)<162000))
			fe_stid135_set_maxllr_rate(state, 180);
		else
			fe_stid135_set_maxllr_rate(state, 250);



	} else dprintk("[%d] LLR cannot be computed because dummy PLF!!\n", state->nr+1);

	ChipGetField(state->base->ip.handle_demod, FLD_FC8CODEW_DVBSX_DEMOD_HDEBITCFG0_SDEMAP_MAXLLRRATE(state->nr+1), &max_llr_allowed);

	switch(max_llr_allowed) {

		case 0:
			vprintk("[%d] Absolute maximum rate allowed by the LLR clock\n", state->nr+1);
		break;
		case 1:
			vprintk("[%d] Max LLR allowed = 180 MLLR/s\n", state->nr+1);
			if(*current_llr > 162000000)
				dprintk("[%d] Careful! LLR may reach max allowed LLR!\n", state->nr+1);
			break;
		case 2:
			FE_STiD135_GetMclkFreq(pParams, &(fld_value[0])) ;

			vprintk("[%d]Max LLR allowed = %d MLLR/s\n", state->nr+1, fld_value[0]/10*9000000);

			if(*current_llr >fld_value[0]/10*9)
				vprintk("[%d] Careful! LLR may reach max allowed LLR!\n", state->nr+1);

		break;
		case 3:
			printk("[%d] Max LLR allowed = 90 MLLR/s\n", state->nr+1);
			if(*current_llr >  81000000 )
				dprintk("[%d] Careful! LLR may reach max allowed LLR!\n", state->nr+1);

		break;
		default:
			dprintk("[%d] Unknown max LLR\n", state->nr+1);
		break;
	}
	return FE_LLA_NO_ERROR;

}


/*
	tests

	bool        has_signal; Has found something above the noise level.
                          always set to 1 after tuning; should depend on signal level
	bool        has_carrier; Has found a dvb signal.
                 DMDSTATE_HEADER_MODE indicates DVBS1 or DVBS2  and DSTATUS_LOCK_DEFINITIF is set (fe_stid135_get_lock_status,
								 FE_STiD135_GetDemodLock)
								 DMDSTATE_HEADER_MODE indicates DVBS1 or DVBS2 irrespective of  DSTATUS_LOCK_DEFINITIF (FE_STiD135_GetFECLock),
                 but presumably this assumes  DSTATUS_LOCK_DEFINITIF is still set

	bool        has_viterbi; 	FEC inner coding (Viterbi, LDPC or other inner code) is stable.
	               PDELSTATUS1_PKTDELIN_LOCK or VSTATUSVIT_LOCKEDVIT is set and has_carrier (fe_stid135_get_lock_status, FE_STiD135_GetFECLock)

	bool        has_sync; 	Synchronization bytes was found (mpeg stream)
	              TSSTATUS_TSFIFO_LINEOK=set (no stream errors) (fe_stid135_get_lock_status)

	bool        has_lock;  All is fine (should be and of the above? In code it is NOT like that:)
	                  FE_STiD135_GetDemodLock(state and    FE_STiD135_GetFECLock both return locked, which means that
										has_carrier==1  and  has_viterbi==1 (FE_STiD135_WaitForLock)
                    so some data is available but it is not necessarily valid (not has_sync). Seems superfluous

	bool        has_timedout = ! has_viterbi AT INITIAL TUNE
	bool       fec_locked = has_sync (GE_STiD135_WaitForLock)
                        =has_viterbi (FE_STiD135_GetFECLock)
                        =has_viterbi (FE_STiD135_Algo)
                        =has_sync (FE_STiD135_Algo)

	bool        demod_locked;
	             DSTATUS_LOCK_DEFINITIF is set (FE_STiD135_GetDemodLock)
               Could be associated with has_signal?


The transport streams are available on the banks of pins PIO3, PIO4, PIO5, PIO6 and PIO7
using the Alternative functions 1, 2, 3 or 4. However, the bank of PIO3 is also shared with
the FSK UART bus when using Alternative function 1.

 while running:
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1000                  = 0x00000100
PIO1_3: general purpose
PIO1_2: alternative function 1
PIO1_1: general purpose
PIO1_0: general purpose


736

REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1001                  = 0x00333333
PIO4_5:  Alternative function 3
PIO4_4:  Alternative function 3
PIO4_3:  Alternative function 3
PIO4_2:  Alternative function 3
PIO4_1:  Alternative function 3
PIO4_0:  Alternative function 3

cannot be set to alternative function 5!


REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1002                  = 0x00333333
PIO5_5:  Alternative function 3
PIO5_4:  Alternative function 3
PIO5_3:  Alternative function 3
PIO5_2:  Alternative function 3
PIO5_1:  Alternative function 3
PIO5_0:  Alternative function 3
can be set to alternative function 5!


REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1003                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1040                  = 0x003f3f04
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1050                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1055                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1060                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1100                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1101                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1102                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1103                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1104                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1105                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1106                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1107                  = 0x00000000
REG_RSTID135_SYSCFG_SOUTH_SYSTEM_CONFIG1108                  = 0x00000205
REG_


 */
