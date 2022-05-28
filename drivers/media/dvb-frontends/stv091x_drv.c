/*
 * Driver for the ST STV091x DVB-S/S2 demodulator.
 *
 * Copyright (C) 2014-2015 Ralph Metzler <rjkm@metzlerbros.de>
 * Marcus Metzler <mocm@metzlerbros.de>
 * developed for Digital Devices GmbH
 * Copyright (C) 2020 Deep Thought <deeptho@gmail.com>: blindscan, spectrum, constellation code
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <asm/div64.h>

#include <media/dvb_frontend.h>
#include "stv091x.h"
#include "stv091x_regs.h"

#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt), __func__, __LINE__, ##arg)

#define TUNING_DELAY 200
#define BER_SRC_S 0x20
#define BER_SRC_S2 0x20

static unsigned int single;
module_param(single, int, 0644);
MODULE_PARM_DESC(single, "Single mode (default : off)");

static unsigned int ldpc_mode;
module_param(ldpc_mode, int, 0644);
MODULE_PARM_DESC(ldpc_mode, "LDPC mode (0 - broadcast, 1 - asap; default : broadcast)");

static unsigned int no_bcherr;
module_param(no_bcherr, int, 0644);
MODULE_PARM_DESC(no_bcherr, "Disable BCH error check (default : off)");

int stv_verbose=0;
module_param(stv_verbose, int, 0644);
MODULE_PARM_DESC(stv_verbose, "verbose debugging");

LIST_HEAD(stvlist);


#define vprintk(fmt, arg...)																					\
	if(stv_verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)



static inline u32 MulDiv32(u32 a, u32 b, u32 c)
{
	u64 tmp64;

	tmp64 = (u64)a * (u64)b;
	do_div(tmp64, c);

	return (u32) tmp64;
}


struct SInitTable {
	u16 Address;
	u8 Data;
};

struct SLookup {
	s16 Value;
	u32 RegValue;
};



struct SLookup S1_SN_Lookup[] = {
	{ 0, 9242 }, /*C/N= 0dB*/
	{ 05, 9105 }, /*C/N=0.5dB*/
	{ 10, 8950 }, /*C/N=1.0dB*/
	{ 15, 8780 }, /*C/N=1.5dB*/
	{ 20, 8566 }, /*C/N=2.0dB*/
	{ 25, 8366 }, /*C/N=2.5dB*/
	{ 30, 8146 }, /*C/N=3.0dB*/
	{ 35, 7908 }, /*C/N=3.5dB*/
	{ 40, 7666 }, /*C/N=4.0dB*/
	{ 45, 7405 }, /*C/N=4.5dB*/
	{ 50, 7136 }, /*C/N=5.0dB*/
	{ 55, 6861 }, /*C/N=5.5dB*/
	{ 60, 6576 }, /*C/N=6.0dB*/
	{ 65, 6330 }, /*C/N=6.5dB*/
	{ 70, 6048 }, /*C/N=7.0dB*/
	{ 75, 5768 }, /*C/N=7.5dB*/
	{ 80, 5492 }, /*C/N=8.0dB*/
	{ 85, 5224 }, /*C/N=8.5dB*/
	{ 90, 4959 }, /*C/N=9.0dB*/
	{ 95, 4709 }, /*C/N=9.5dB*/
	{ 100, 4467 }, /*C/N=10.0dB*/
	{ 105, 4236 }, /*C/N=10.5dB*/
	{ 110, 4013 }, /*C/N=11.0dB*/
	{ 115, 3800 }, /*C/N=11.5dB*/
	{ 120, 3598 }, /*C/N=12.0dB*/
	{ 125, 3406 }, /*C/N=12.5dB*/
	{ 130, 3225 }, /*C/N=13.0dB*/
	{ 135, 3052 }, /*C/N=13.5dB*/
	{ 140, 2889 }, /*C/N=14.0dB*/
	{ 145, 2733 }, /*C/N=14.5dB*/
	{ 150, 2587 }, /*C/N=15.0dB*/
	{ 160, 2318 }, /*C/N=16.0dB*/
	{ 170, 2077 }, /*C/N=17.0dB*/
	{ 180, 1862 }, /*C/N=18.0dB*/
	{ 190, 1670 }, /*C/N=19.0dB*/
	{ 200, 1499 }, /*C/N=20.0dB*/
	{ 210, 1347 }, /*C/N=21.0dB*/
	{ 220, 1213 }, /*C/N=22.0dB*/
	{ 230, 1095 }, /*C/N=23.0dB*/
	{ 240, 992 }, /*C/N=24.0dB*/
	{ 250, 900 }, /*C/N=25.0dB*/
	{ 260, 826 }, /*C/N=26.0dB*/
	{ 270, 758 }, /*C/N=27.0dB*/
	{ 280, 702 }, /*C/N=28.0dB*/
	{ 290, 653 }, /*C/N=29.0dB*/
	{ 300, 613 }, /*C/N=30.0dB*/
	{ 310, 579 }, /*C/N=31.0dB*/
	{ 320, 550 }, /*C/N=32.0dB*/
	{ 330, 526 }, /*C/N=33.0dB*/
	{ 350, 490 }, /*C/N=33.0dB*/
	{ 400, 445 }, /*C/N=40.0dB*/
	{ 450, 430 }, /*C/N=45.0dB*/
	{ 500, 426 }, /*C/N=50.0dB*/
	{ 510, 425 } /*C/N=51.0dB*/
};

struct SLookup S2_SN_Lookup[] = {
	{ -30, 13950 }, /*C/N=-2.5dB*/
	{ -25, 13580 }, /*C/N=-2.5dB*/
	{ -20, 13150 }, /*C/N=-2.0dB*/
	{ -15, 12760 }, /*C/N=-1.5dB*/
	{ -10, 12345 }, /*C/N=-1.0dB*/
	{ -05, 11900 }, /*C/N=-0.5dB*/
	{ 0, 11520 }, /*C/N= 0dB*/
	{ 05, 11080 }, /*C/N= 0.5dB*/
	{ 10, 10630 }, /*C/N= 1.0dB*/
	{ 15, 10210 }, /*C/N= 1.5dB*/
	{ 20, 9790 }, /*C/N= 2.0dB*/
	{ 25, 9390 }, /*C/N= 2.5dB*/
	{ 30, 8970 }, /*C/N= 3.0dB*/
	{ 35, 8575 }, /*C/N= 3.5dB*/
	{ 40, 8180 }, /*C/N= 4.0dB*/
	{ 45, 7800 }, /*C/N= 4.5dB*/
	{ 50, 7430 }, /*C/N= 5.0dB*/
	{ 55, 7080 }, /*C/N= 5.5dB*/
	{ 60, 6720 }, /*C/N= 6.0dB*/
	{ 65, 6320 }, /*C/N= 6.5dB*/
	{ 70, 6060 }, /*C/N= 7.0dB*/
	{ 75, 5760 }, /*C/N= 7.5dB*/
	{ 80, 5480 }, /*C/N= 8.0dB*/
	{ 85, 5200 }, /*C/N= 8.5dB*/
	{ 90, 4930 }, /*C/N= 9.0dB*/
	{ 95, 4680 }, /*C/N= 9.5dB*/
	{ 100, 4425 }, /*C/N=10.0dB*/
	{ 105, 4210 }, /*C/N=10.5dB*/
	{ 110, 3980 }, /*C/N=11.0dB*/
	{ 115, 3765 }, /*C/N=11.5dB*/
	{ 120, 3570 }, /*C/N=12.0dB*/
	{ 125, 3315 }, /*C/N=12.5dB*/
	{ 130, 3140 }, /*C/N=13.0dB*/
	{ 135, 2980 }, /*C/N=13.5dB*/
	{ 140, 2820 }, /*C/N=14.0dB*/
	{ 145, 2670 }, /*C/N=14.5dB*/
	{ 150, 2535 }, /*C/N=15.0dB*/
	{ 160, 2270 }, /*C/N=16.0dB*/
	{ 170, 2035 }, /*C/N=17.0dB*/
	{ 180, 1825 }, /*C/N=18.0dB*/
	{ 190, 1650 }, /*C/N=19.0dB*/
	{ 200, 1485 }, /*C/N=20.0dB*/
	{ 210, 1340 }, /*C/N=21.0dB*/
	{ 220, 1212 }, /*C/N=22.0dB*/
	{ 230, 1100 }, /*C/N=23.0dB*/
	{ 240, 1000 }, /*C/N=24.0dB*/
	{ 250, 910 }, /*C/N=25.0dB*/
	{ 260, 836 }, /*C/N=26.0dB*/
	{ 270, 772 }, /*C/N=27.0dB*/
	{ 280, 718 }, /*C/N=28.0dB*/
	{ 290, 671 }, /*C/N=29.0dB*/
	{ 300, 635 }, /*C/N=30.0dB*/
	{ 310, 602 }, /*C/N=31.0dB*/
	{ 320, 575 }, /*C/N=32.0dB*/
	{ 330, 550 }, /*C/N=33.0dB*/
	{ 350, 517 }, /*C/N=35.0dB*/
	{ 400, 480 }, /*C/N=40.0dB*/
	{ 450, 466 }, /*C/N=45.0dB*/
	{ 500, 464 }, /*C/N=50.0dB*/
	{ 510, 463 }, /*C/N=51.0dB*/
};

#if 0
FE_STV0910_LOOKUP_t	IQPower_LookUp = 	{
	5,
	{
		{-70,0x94},
		{-75,0x55},
		{-80,0x33},
		{-85,0x20},
		{-90,0x18},
	}
};
#endif

struct SLookup PADC_Lookup[] = {
	{ 0,	118000}, /*PADC=+0dBm*/
	{- 100,	93600}, /*PADC=-1dBm*/
	{- 200,	74500}, /*PADC=-2dBm*/
	{- 300,	59100}, /*PADC=-3dBm*/
	{- 400,	47000}, /*PADC=-4dBm*/
	{- 500,	37300}, /*PADC=-5dBm*/
	{- 600,	29650}, /*PADC=-6dBm*/
	{- 700,	23520}, /*PADC=-7dBm*/
	{- 900,	14850}, /*PADC=-9dBm*/
	{-1100,	9380 }, /*PADC=-11dBm*/
	{-1500, 3730 }, /*PADC=-15dBm*/
	{-1300,	5910 }, /*PADC=-13dBm*/
	{-1700, 2354 }, /*PADC=-17dBm*/
	{-1900, 1485 }, /*PADC=-19dBm*/
	{-2000, 1179 }, /*PADC=-20dBm*/
	{-2100, 1000 }, /*PADC=-21dBm*/
};


/*********************************************************************
Tracking carrier loop carrier QPSK 1/4 to 8PSK 9/10 long Frame
*********************************************************************/
static u8 S2CarLoop[] =	{
	/* Modcod 2MPon 2MPoff 5MPon 5MPoff 10MPon 10MPoff
	 20MPon 20MPoff 30MPon 30MPoff*/
	/* FE_QPSK_14 */
	0x0C, 0x3C, 0x0B, 0x3C, 0x2A, 0x2C, 0x2A, 0x1C, 0x3A, 0x3B,
	/* FE_QPSK_13 */
	0x0C, 0x3C, 0x0B, 0x3C, 0x2A, 0x2C, 0x3A, 0x0C, 0x3A, 0x2B,
	/* FE_QPSK_25 */
	0x1C, 0x3C, 0x1B, 0x3C, 0x3A, 0x1C, 0x3A, 0x3B, 0x3A, 0x2B,
	/* FE_QPSK_12 */
	0x0C, 0x1C, 0x2B, 0x1C, 0x0B, 0x2C, 0x0B, 0x0C, 0x2A, 0x2B,
	/* FE_QPSK_35 */
	0x1C, 0x1C, 0x2B, 0x1C, 0x0B, 0x2C, 0x0B, 0x0C, 0x2A, 0x2B,
	/* FE_QPSK_23 */
	0x2C, 0x2C, 0x2B, 0x1C, 0x0B, 0x2C, 0x0B, 0x0C, 0x2A, 0x2B,
	/* FE_QPSK_34 */
	0x3C, 0x2C, 0x3B, 0x2C, 0x1B, 0x1C, 0x1B, 0x3B, 0x3A, 0x1B,
	/* FE_QPSK_45 */
	0x0D, 0x3C, 0x3B, 0x2C, 0x1B, 0x1C, 0x1B, 0x3B, 0x3A, 0x1B,
	/* FE_QPSK_56 */
	0x1D, 0x3C, 0x0C, 0x2C, 0x2B, 0x1C, 0x1B, 0x3B, 0x0B, 0x1B,
	/* FE_QPSK_89 */
	0x3D, 0x0D, 0x0C, 0x2C, 0x2B, 0x0C, 0x2B, 0x2B, 0x0B, 0x0B,
	/* FE_QPSK_910 */
	0x1E, 0x0D, 0x1C, 0x2C, 0x3B, 0x0C, 0x2B, 0x2B, 0x1B, 0x0B,
	/* FE_8PSK_35 */
	0x28, 0x09, 0x28, 0x09, 0x28, 0x09, 0x28, 0x08, 0x28, 0x27,
	/* FE_8PSK_23 */
	0x19, 0x29, 0x19, 0x29, 0x19, 0x29, 0x38, 0x19, 0x28, 0x09,
	/* FE_8PSK_34 */
	0x1A, 0x0B, 0x1A, 0x3A, 0x0A, 0x2A, 0x39, 0x2A, 0x39, 0x1A,
	/* FE_8PSK_56 */
	0x2B, 0x2B, 0x1B, 0x1B, 0x0B, 0x1B, 0x1A, 0x0B, 0x1A, 0x1A,
	/* FE_8PSK_89 */
	0x0C, 0x0C, 0x3B, 0x3B, 0x1B, 0x1B, 0x2A, 0x0B, 0x2A, 0x2A,
	/* FE_8PSK_910 */
	0x0C, 0x1C, 0x0C, 0x3B, 0x2B, 0x1B, 0x3A, 0x0B, 0x2A, 0x2A,

	/**********************************************************************
	Tracking carrier loop carrier 16APSK 2/3 to 32APSK 9/10 long Frame
	**********************************************************************/
	/*Modcod 2MPon 2MPoff 5MPon 5MPoff 10MPon 10MPoff 20MPon
	 20MPoff 30MPon 30MPoff*/
	/* FE_16APSK_23 */
	0x0A, 0x0A, 0x0A, 0x0A, 0x1A, 0x0A, 0x39, 0x0A, 0x29, 0x0A,
	/* FE_16APSK_34 */
	0x0A, 0x0A, 0x0A, 0x0A, 0x0B, 0x0A, 0x2A, 0x0A, 0x1A, 0x0A,
	/* FE_16APSK_45 */
	0x0A, 0x0A, 0x0A, 0x0A, 0x1B, 0x0A, 0x3A, 0x0A, 0x2A, 0x0A,
	/* FE_16APSK_56 */
	0x0A, 0x0A, 0x0A, 0x0A, 0x1B, 0x0A, 0x3A, 0x0A, 0x2A, 0x0A,
	/* FE_16APSK_89 */
	0x0A, 0x0A, 0x0A, 0x0A, 0x2B, 0x0A, 0x0B, 0x0A, 0x3A, 0x0A,
	/* FE_16APSK_910 */
	0x0A, 0x0A, 0x0A, 0x0A, 0x2B, 0x0A, 0x0B, 0x0A, 0x3A, 0x0A,
	/* FE_32APSK_34 */
	0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
	/* FE_32APSK_45 */
	0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
	/* FE_32APSK_56 */
	0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
	/* FE_32APSK_89 */
	0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
	/* FE_32APSK_910 */
	0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
};

static u8 get_optim_cloop(struct stv *state,
			 enum FE_STV0910_ModCod ModCod, u32 Pilots)
{
	int i = 0;
	if (ModCod >= FE_32APSK_910)
		i = ((int)FE_32APSK_910 - (int)FE_QPSK_14) * 10;
	else if (ModCod >= FE_QPSK_14)
		i = ((int)ModCod - (int)FE_QPSK_14) * 10;

	if (state->symbol_rate <= 3000000)
		i += 0;
	else if (state->symbol_rate <= 7000000)
		i += 2;
	else if (state->symbol_rate <= 15000000)
		i += 4;
	else if (state->symbol_rate <= 25000000)
		i += 6;
	else
		i += 8;

	if (!Pilots)
		i += 1;

	return S2CarLoop[i];
}

static int stv091x_stop_task(struct dvb_frontend *fe);

static int stv091x_symbol_rate(struct stv *state, u32 *p_symbol_rate, bool coarse)
{
	int status = 0;
#if 0
	u8 SymbFreq0;
	u8 SymbFreq1;
	u8 SymbFreq2;
	u8 SymbFreq3;
	u8 TimOffs0;
	u8 TimOffs1;
	u8 TimOffs2;
#endif
	u32 symbol_rate;
	s32 timing_offset;
	u8 regvals[4];
	*p_symbol_rate = 0;
	if (!state->Started)
		return status;
	read_regs(state, RSTV0910_P2_SFR3 + state->regoff, &regvals[0], 4);
	symbol_rate = ((u32) regvals[0] << 24) | ((u32) regvals[1] << 16) |
		((u32) regvals[1] << 8) | (u32) regvals[0];

	symbol_rate = (u32) (((u64) symbol_rate * state->base->mclk) >> 32);
	if(!coarse) {
	read_regs(state, RSTV0910_P2_TMGREG2 + state->regoff, &regvals[0], 3);
	timing_offset = ((u32) regvals[0] << 16) | ((u32) regvals[1] << 8) |
			(u32) regvals[2];
	if ((timing_offset & (1<<23)) != 0)
		timing_offset |= 0xFF000000; /* Sign extend */
	timing_offset = (s32) (((s64) symbol_rate * (s64) timing_offset) >> 29);

	*p_symbol_rate = symbol_rate + timing_offset;
	}
	return 0;
}

static int GetSignalParameters(struct stv *state)
{
	if (!state->Started)
		return -1;

	if (state->ReceiveMode == Mode_DVBS2) {
		u8 tmp;
		u8 rolloff;

		tmp = read_reg(state, RSTV0910_P2_DMDMODCOD + state->regoff);
		state->ModCod = (enum FE_STV0910_ModCod) ((tmp & 0x7c) >> 2);
		state->Pilots = (tmp & 0x01) != 0;
		state->FECType = (enum DVBS2_FECType) ((tmp & 0x02) >> 1);

		rolloff = read_reg(state, RSTV0910_P2_TMGOBS + state->regoff);
		rolloff = rolloff >> 6;
		state->FERollOff = (enum FE_STV0910_RollOff) rolloff;

	} else if (state->ReceiveMode == Mode_DVBS) {
		/* todo */
	}
	return 0;
}

static int TrackingOptimization(struct stv *state)
{
	u32 SymbolRate = 0;
	u8 tmp;

	stv091x_symbol_rate(state, &SymbolRate, false);
	tmp = read_reg(state, RSTV0910_P2_DMDCFGMD + state->regoff);
	tmp &= ~0xF0;

	switch (state->ReceiveMode) {
	case Mode_DVBS:
		tmp |= 0x40; break;
	case Mode_DVBS2:
		tmp |= 0x80; break;
	default:
		tmp |= 0xC0; break;
	}
	write_reg(state, RSTV0910_P2_DMDCFGMD + state->regoff, tmp);
	dprintk("RECEIVEMODE =%d\n", state->ReceiveMode);
	if (state->ReceiveMode == Mode_DVBS2) {
		u8 reg;
		/* force to PRE BCH Rate */
		write_reg(state, RSTV0910_P2_ERRCTRL1 + state->regoff,
			 BER_SRC_S2 | state->BERScale);

		if (state->FECType == DVBS2_64K) {
			u8 aclc = get_optim_cloop(state, state->ModCod,
						 state->Pilots);

			if (state->ModCod <= FE_QPSK_910) {
				write_reg(state, RSTV0910_P2_ACLC2S2Q +
					 state->regoff, aclc);
			} else if (state->ModCod <= FE_8PSK_910) {
				write_reg(state, RSTV0910_P2_ACLC2S2Q +
					 state->regoff, 0x2a);
				write_reg(state, RSTV0910_P2_ACLC2S28 +
					 state->regoff, aclc);
			} else if (state->ModCod <= FE_16APSK_910) {
				write_reg(state, RSTV0910_P2_ACLC2S2Q +
					 state->regoff, 0x2a);
				write_reg(state, RSTV0910_P2_ACLC2S216A +
					 state->regoff, aclc);
			} else if (state->ModCod <= FE_32APSK_910) {
				write_reg(state, RSTV0910_P2_ACLC2S2Q +
					 state->regoff, 0x2a);
				write_reg(state, RSTV0910_P2_ACLC2S232A +
					 state->regoff, aclc);
			}
		}
		/* Check MATYPE flags*/
		tmp = read_reg(state, RSTV0910_P2_MATSTR1 + state->regoff);
		reg = read_reg(state, RSTV0910_P2_TSSTATEM + state->regoff);
		if ( tmp & 0x18 )
			reg &= ~1;
		else
			reg |= 1; /* Disable TS FIFO sync if ACM without ISSYI */
		write_reg(state, RSTV0910_P2_TSSTATEM + state->regoff, reg);
		/*pr_info("stv0910: %d TSSTATEM=0x%02x\n", state->regoff, reg);*/
		reg = read_reg(state, RSTV0910_P2_TSSYNC + state->regoff);
		if ( tmp & 0x18 )
			reg &= ~0x18;
		else
		{
			reg &= ~0x18;
			reg |= 0x10;
		}
		write_reg(state, RSTV0910_P2_TSSYNC + state->regoff, reg);
		/*pr_info("stv0910: %d TSSYNC=0x%02x\n", state->regoff, reg);*/
	}
	if (state->ReceiveMode == Mode_DVBS) {
		u8 tmp;

		tmp = read_reg(state, RSTV0910_P2_VITCURPUN + state->regoff);
		state->PunctureRate = FEC_NONE;
		switch (tmp & 0x1F) {
		case 0x0d:
			state->PunctureRate = FEC_1_2;
			break;
		case 0x12:
			state->PunctureRate = FEC_2_3;
			break;
		case 0x15:
			state->PunctureRate = FEC_3_4;
			break;
		case 0x18:
			state->PunctureRate = FEC_5_6;
			break;
		case 0x1A:
			state->PunctureRate = FEC_7_8;
			break;
		}
	}
	return 0;
}

static s32 TableLookup(struct SLookup *Table,
		 int TableSize, u16 RegValue)
{
	s32 Value;
	int imin = 0;
	int imax = TableSize - 1;
	int i;
	s32 RegDiff;

	// Assumes Table[0].RegValue > Table[imax].RegValue
	if( RegValue >= Table[0].RegValue )
		Value = Table[0].Value;
	else if( RegValue <= Table[imax].RegValue )
		Value = Table[imax].Value;
	else
	{
		while(imax-imin > 1)
		{
			i = (imax + imin) / 2;
			if( (Table[imin].RegValue >= RegValue) && (RegValue >= Table[i].RegValue) )
				imax = i;
			else
				imin = i;
		}

		RegDiff = Table[imax].RegValue - Table[imin].RegValue;
		Value = Table[imin].Value;
		if( RegDiff != 0 )
			Value += ((s32)(RegValue - Table[imin].RegValue) *
				 (s32)(Table[imax].Value - Table[imin].Value))/(RegDiff);
	}

	return Value;
}

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

static int GetSignalToNoise(struct stv *state, s32 *SignalToNoise)
{
	u8 Data0;
	u8 Data1;
	u16 Data;
	int nLookup;
	struct SLookup *Lookup;

	*SignalToNoise = 0;

	if (!state->Started)
		return 0;

	if (state->ReceiveMode == Mode_DVBS2) {
		Data1 = read_reg(state, RSTV0910_P2_NNOSPLHT1 + state->regoff);
		Data0 = read_reg(state, RSTV0910_P2_NNOSPLHT0 + state->regoff);
		nLookup = ARRAY_SIZE(S2_SN_Lookup);
		Lookup = S2_SN_Lookup;
	} else {
		Data1 = read_reg(state, RSTV0910_P2_NNOSDATAT1 + state->regoff);
		Data0 = read_reg(state, RSTV0910_P2_NNOSDATAT0 + state->regoff);
		nLookup = ARRAY_SIZE(S1_SN_Lookup);
		Lookup = S1_SN_Lookup;
	}
	Data = (((u16)Data1) << 8) | (u16) Data0;
 *SignalToNoise = TableLookup(Lookup, nLookup, Data);
	return 0;
}

static int GetBitErrorRateS(struct stv *state, u32 *BERNumerator,
			 u32 *BERDenominator)
{
	u8 Regs[3];

	int status = read_regs(state, RSTV0910_P2_ERRCNT12 + state->regoff,
			 Regs, 3);

	if (status)
		return -1;

	if ((Regs[0] & 0x80) == 0) {
		state->LastBERDenominator = 1 << ((state->BERScale * 2) +
						 10 + 3);
		state->LastBERNumerator = ((u32) (Regs[0] & 0x7F) << 16) |
			((u32) Regs[1] << 8) | Regs[2];
		if (state->LastBERNumerator < 256 && state->BERScale < 6) {
			state->BERScale += 1;
			status = write_reg(state, RSTV0910_P2_ERRCTRL1 +
					 state->regoff,
					 0x20 | state->BERScale);
		} else if (state->LastBERNumerator > 1024 &&
			 state->BERScale > 2) {
			state->BERScale -= 1;
			status = write_reg(state, RSTV0910_P2_ERRCTRL1 +
					 state->regoff, 0x20 |
					 state->BERScale);
		}
	}
	*BERNumerator = state->LastBERNumerator;
	*BERDenominator = state->LastBERDenominator;
	return 0;
}

static u32 DVBS2_nBCH(enum DVBS2_ModCod ModCod, enum DVBS2_FECType FECType)
{
	static u32 nBCH[][2] = {
		{16200, 3240}, /* QPSK_1_4, */
		{21600, 5400}, /* QPSK_1_3, */
		{25920, 6480}, /* QPSK_2_5, */
		{32400, 7200}, /* QPSK_1_2, */
		{38880, 9720}, /* QPSK_3_5, */
		{43200, 10800}, /* QPSK_2_3, */
		{48600, 11880}, /* QPSK_3_4, */
		{51840, 12600}, /* QPSK_4_5, */
		{54000, 13320}, /* QPSK_5_6, */
		{57600, 14400}, /* QPSK_8_9, */
		{58320, 16000}, /* QPSK_9_10, */
		{43200, 9720}, /* 8PSK_3_5, */
		{48600, 10800}, /* 8PSK_2_3, */
		{51840, 11880}, /* 8PSK_3_4, */
		{54000, 13320}, /* 8PSK_5_6, */
		{57600, 14400}, /* 8PSK_8_9, */
		{58320, 16000}, /* 8PSK_9_10, */
		{43200, 10800}, /* 16APSK_2_3, */
		{48600, 11880}, /* 16APSK_3_4, */
		{51840, 12600}, /* 16APSK_4_5, */
		{54000, 13320}, /* 16APSK_5_6, */
		{57600, 14400}, /* 16APSK_8_9, */
		{58320, 16000}, /* 16APSK_9_10 */
		{48600, 11880}, /* 32APSK_3_4, */
		{51840, 12600}, /* 32APSK_4_5, */
		{54000, 13320}, /* 32APSK_5_6, */
		{57600, 14400}, /* 32APSK_8_9, */
		{58320, 16000}, /* 32APSK_9_10 */
	};

	if (ModCod >= DVBS2_QPSK_1_4 &&
	 ModCod <= DVBS2_32APSK_9_10 && FECType <= DVBS2_16K)
		return nBCH[FECType][ModCod];
	return 64800;
}

static int GetBitErrorRateS2(struct stv *state, u32 *BERNumerator,
			 u32 *BERDenominator)
{
	u8 Regs[3];

	int status = read_regs(state, RSTV0910_P2_ERRCNT12 + state->regoff,
			 Regs, 3);

	if (status)
		return -1;

	if ((Regs[0] & 0x80) == 0) {
		state->LastBERDenominator =
			DVBS2_nBCH((enum DVBS2_ModCod) state->ModCod,
				 state->FECType) <<
			(state->BERScale * 2);
		state->LastBERNumerator = (((u32) Regs[0] & 0x7F) << 16) |
			((u32) Regs[1] << 8) | Regs[2];
		if (state->LastBERNumerator < 256 && state->BERScale < 6) {
			state->BERScale += 1;
			write_reg(state, RSTV0910_P2_ERRCTRL1 + state->regoff,
				 0x20 | state->BERScale);
		} else if (state->LastBERNumerator > 1024 &&
			 state->BERScale > 2) {
			state->BERScale -= 1;
			write_reg(state, RSTV0910_P2_ERRCTRL1 + state->regoff,
				 0x20 | state->BERScale);
		}
	}
	*BERNumerator = state->LastBERNumerator;
	*BERDenominator = state->LastBERDenominator;
	return status;
}

static int GetBitErrorRate(struct stv *state, u32 *BERNumerator,
			 u32 *BERDenominator)
{
	*BERNumerator = 0;
	*BERDenominator = 1;

	switch (state->ReceiveMode) {
	case Mode_DVBS:
		return GetBitErrorRateS(state, BERNumerator, BERDenominator);
		break;
	case Mode_DVBS2:
		return GetBitErrorRateS2(state, BERNumerator, BERDenominator);
	default:
		break;
	}
	return 0;
}

static int init(struct dvb_frontend *fe)
{
	return 0;
}

static int set_mclock(struct stv *state, u32 MasterClock)
{
	u32 idf = 1;
	u32 odf = 4;
	u32 quartz = state->base->extclk / 1000000;
	u32 Fphi = MasterClock / 1000000;
	u32 ndiv = (Fphi * odf * idf) / quartz;
	u32 cp = 7;
	u32 fvco;

	if (ndiv >= 7 && ndiv <= 71)
		cp = 7;
	else if (ndiv >= 72 && ndiv <= 79)
		cp = 8;
	else if (ndiv >= 80 && ndiv <= 87)
		cp = 9;
	else if (ndiv >= 88 && ndiv <= 95)
		cp = 10;
	else if (ndiv >= 96 && ndiv <= 103)
		cp = 11;
	else if (ndiv >= 104 && ndiv <= 111)
		cp = 12;
	else if (ndiv >= 112 && ndiv <= 119)
		cp = 13;
	else if (ndiv >= 120 && ndiv <= 127)
		cp = 14;
	else if (ndiv >= 128 && ndiv <= 135)
		cp = 15;
	else if (ndiv >= 136 && ndiv <= 143)
		cp = 16;
	else if (ndiv >= 144 && ndiv <= 151)
		cp = 17;
	else if (ndiv >= 152 && ndiv <= 159)
		cp = 18;
	else if (ndiv >= 160 && ndiv <= 167)
		cp = 19;
	else if (ndiv >= 168 && ndiv <= 175)
		cp = 20;
	else if (ndiv >= 176 && ndiv <= 183)
		cp = 21;
	else if (ndiv >= 184 && ndiv <= 191)
		cp = 22;
	else if (ndiv >= 192 && ndiv <= 199)
		cp = 23;
	else if (ndiv >= 200 && ndiv <= 207)
		cp = 24;
	else if (ndiv >= 208 && ndiv <= 215)
		cp = 25;
	else if (ndiv >= 216 && ndiv <= 223)
		cp = 26;
	else if (ndiv >= 224 && ndiv <= 225)
		cp = 27;

	write_reg(state, RSTV0910_NCOARSE, (cp << 3) | idf);
	write_reg(state, RSTV0910_NCOARSE2, odf);
	write_reg(state, RSTV0910_NCOARSE1, ndiv);

	fvco = (quartz * 2 * ndiv) / idf;
	state->base->mclk = fvco / (2 * odf) * 1000000;

	/*pr_info("ndiv = %d, MasterClock = %d\n", ndiv, state->base->mclk);*/
	return 0;
}

/*
	require_fec_lock: viterbi lock is not enough, we also need data
 */
static int wait_for_dmdlock(struct dvb_frontend *fe, bool require_data)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	u16 timeout = 0;
	u16 timer = 0;
	u8 lock = 0;
	u8 dstatus, dmdstate, pktdelin;
	u8 vstatusvit =0;
	bool tst=false;
	u8 tsstatus = 0;
	state->signal_info.demod_locked = 0;
	state->signal_info.fec_locked = 0;
#if 0
//	fprintk("demod: %d", state->adapterno);
	if(p->algorithm != ALGORITHM_WARM)
		timeout = 1000;
	else if (state->symbol_rate <= 1000000) /* SR <= 1Msps */
		timeout = 1000;
	else if (state->symbol_rate <= 5000000) /* 1Msps < SR <= 2Msps */
		timeout = 1000/(state->symbol_rate/100000);
	else
		timeout = 200;
#else
	if(p->algorithm != ALGORITHM_WARM)
		timeout = 8000;
	else if (state->symbol_rate <= 1000000) {         /*          SR <=  1Msps */
		timeout = 16000;
	} else if (state->symbol_rate <= 2000000) {  /*  1Msps < SR <=  2Msps */
		timeout = 4500;
	} else if (state->symbol_rate <= 5000000) {  /*  2Msps < SR <=  5Msps */
		timeout = 3500;
	} else if (state->symbol_rate <= 10000000) { /*  5Msps < SR <= 10Msps */
		timeout = 2500;
	} else if (state->symbol_rate < 20000000) {  /* 10Msps < SR <= 20Msps */
		timeout = 1500;
	} else {                                 /*          SR >= 20Msps */
		timeout = 1000;
	}

#endif
	dprintk("timeout=%d\n", timeout);

	while (timer < timeout && !lock) {
		enum ReceiveMode receive_mode = Mode_None;
		dmdstate = read_reg(state, RSTV0910_P2_DMDSTATE + state->regoff);
		dstatus = read_reg(state, RSTV0910_P2_DSTATUS + state->regoff);
		if (dmdstate & 0x40) { //DVBS1 or DVBS2 found
			if (dstatus & 0x08)
				receive_mode = (dmdstate & 0x20) ?
					Mode_DVBS : Mode_DVBS2;
		}

		switch (receive_mode) {
		case Mode_DVBS:
			if (dstatus&0x08 /*LOCK_DEFINITIF*/) {
				state->signal_info.demod_locked = 1;
				if(!tst)
					dprintk("LOCK_DEFINITIF achieved timing: %d/%d\n", timer, timeout);
				tst=true;
				if(!require_data) {
					lock =1;
					break;
				}
				vstatusvit = read_reg(state, RSTV0910_P2_VSTATUSVIT + state->regoff);
				//dprintk("vstatusvit=%d\n", vstatusvit);

				if (vstatusvit&0x08 /*LOCKEDVIT viterbi locked*/) {
					state->signal_info.fec_locked = 1;
					lock = 1;
				}
			}
			break;
		case Mode_DVBS2:

			if (dstatus&0x8 /*LOCK_DEFINITIF*/) {
				state->signal_info.demod_locked = 1;
				if(!tst)
					dprintk("LOCK_DEFINITIF achieved timing: %d/%d\n", timer, timeout);
				tst=true;
				if(!require_data) {
					lock=1;
					break;
				}
				pktdelin = read_reg(state, RSTV0910_P2_PDELSTATUS1 + state->regoff);
				//dprintk("pktdelin=%d\n", pktdelin);
				if (pktdelin&0x02 /*packet delineator locked*/) {
#if 0
					if (STV091X_READ_FIELD(state, TSFIFO_LINEOK)) {
						lock = 1;
					}

#else
					state->signal_info.fec_locked = 1;
					dprintk("VITERBI lock achieved: timing=%d/%d\n",  timer, timeout);
					lock=1;
#endif
				}
			}

			break;
		default:
			break;
		}
		if(lock && require_data) {
			tsstatus = read_reg(state, RSTV0910_P2_TSSTATUS + state->regoff);
			lock= ((tsstatus&0x80) /*TSFIFO_LINEOK*/!=0);
			//dprintk("tsstatus=%d\n", tsstatus);
			//lock will be set to 0 if we have errors in the stream
		}
		if (!lock)
			msleep(10);
		timer += 10;
	}

	dprintk("initial lock: %s dstatus=0x%x dmdstate=0x%x vit=0x%x tsstatus=0x%x require_data=%d timing: %d/%d\n",
					lock ? "LOCKED" : "NO LOCK",
					dstatus, dmdstate, vstatusvit, tsstatus, require_data, timer, timeout);
	return lock;
}



static int Stop(struct stv *state)
{
	if (state->Started) {

#if 0 //the following not done in stv091x_Algo
		write_reg(state, RSTV0910_P2_TSCFGH + state->regoff,
							state->tscfgh | 0x01); //reset hardware merger
		write_reg(state, RSTV0910_P2_TSCFGH , state->tscfgh); // restore it to its normal state (see: probe)
#endif
		write_reg_field(state, FSTV0910_P2_ALGOSWRST,0); /*release reset DVBS2 packet delin*/


#if 1 //the following not done in stv091x_Algo
		/* Blind optim*/
		write_reg(state, RSTV0910_P2_AGC2O + state->regoff, 0x5B);
		/* Stop the demod */
#endif
		write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x5c);
		state->Started = 0;
	}
	state->ReceiveMode = Mode_None;
	return 0;
}

//same as stid135
static int set_pls_mode_code(struct stv *state, u8 pls_mode, u32 pls_code)
{
	if (pls_mode == 0 && pls_code == 0)
		pls_code = 1;
	pls_mode &= 0x03;
	pls_code &= 0x3FFFF;
	state->signal_info.pls_mode= pls_mode;
	state->signal_info.pls_code= pls_code;
	//vprintk("SETTING PLS_MODE_CODE = %d %d\n", pls_mode, pls_code);
	write_reg(state, RSTV0910_P2_PLROOT2 + state->regoff, (pls_mode<<2) | (pls_code>>16));
	write_reg(state, RSTV0910_P2_PLROOT1 + state->regoff, pls_code>>8);
	write_reg(state, RSTV0910_P2_PLROOT0 + state->regoff, pls_code);
	return 0;
}

//same as stid135
static int set_stream_index(struct stv *state, int mis)
{
	u8 tmp;

	if (mis == NO_STREAM_ID_FILTER) {
		//pr_warn("%s: disable MIS filtering\n", __func__);
		set_pls_mode_code(state, 0, 1);
		tmp = read_reg(state, RSTV0910_P2_PDELCTRL1 + state->regoff);
		tmp &= ~0x20;
		write_reg(state, RSTV0910_P2_PDELCTRL1 + state->regoff, tmp);
	} else {
		set_pls_mode_code(state, (mis>>26) & 0x3, (mis>>8) & 0x3FFFF);
		tmp = read_reg(state, RSTV0910_P2_PDELCTRL1 + state->regoff);
		tmp |= 0x20;
		write_reg(state, RSTV0910_P2_PDELCTRL1 + state->regoff, tmp);
		//pr_warn("%s: enable MIS filtering - %d\n", __func__, mis & 0xff);
		write_reg(state, RSTV0910_P2_ISIENTRY + state->regoff, mis & 0xff );
		write_reg(state, RSTV0910_P2_ISIBITENA + state->regoff, 0xff );
	}
	return 0;
}

void stv091x_compute_timeouts(s32* demod_timeout, s32* fec_timeout, s32 symbol_rate, enum fe_algorithm algo)
{
	switch (algo) {
	case ALGORITHM_BLIND:
		if(symbol_rate<=1500000) { /*1Msps< SR <=1.5Msps*/
			(*demod_timeout)=2000;
			(*fec_timeout)=500;
		} else if(symbol_rate<=5000000) { /*1.5Msps< SR <=5Msps*/
			(*demod_timeout)=1000;
			(*fec_timeout)=300;
		} else if(symbol_rate<=30000000) { /*5Msps< SR <=30Msps*/
			(*demod_timeout)=700;
			(*fec_timeout)=300;
		} else if(symbol_rate<=45000000) { /*30Msps< SR <=45Msps*/
			(*demod_timeout)=400;
			(*fec_timeout)=200;
		} else { /*SR >45Msps*/
			(*demod_timeout)=300;
			(*fec_timeout)=100;
		}
		break;

	case ALGORITHM_COLD:
	case ALGORITHM_WARM:
	default:
		if(symbol_rate<=1000000) { /*SR <=1Msps*/
			(*demod_timeout)=3000;
			(*fec_timeout)=2000; /*1700 */
		} else if(symbol_rate<=2000000) { /*1Msps < SR <=2Msps*/
			(*demod_timeout)=2500;
			(*fec_timeout)=1300; /*1100 */
		} else if(symbol_rate<=5000000) { /*2Msps< SR <=5Msps*/
			(*demod_timeout)=1000;
			(*fec_timeout)=650; /* 550 */
		} else if(symbol_rate<=10000000) { /*5Msps< SR <=10Msps*/
			(*demod_timeout)=700;
			(*fec_timeout)=350; /*250 */
		} else if(symbol_rate<=20000000) { /*10Msps< SR <=20Msps*/
			(*demod_timeout)=400;
			(*fec_timeout)=200; /* 130 */
		}

		else { /*SR >20Msps*/
			(*demod_timeout)=300;
			(*fec_timeout)=200; /* 150 */
		}
		break;

	}

	if(algo == ALGORITHM_WARM) {
		/*if warm start
			demod timeout = coldtimeout/3
			fec timeout = same as cold*/
		(*demod_timeout)/=2;
	}
}

static void stv091x_set_symbol_rate(struct stv* state, u32 symbol_rate)
{
	u32	symb = MulDiv32(symbol_rate, 65536, state->base->mclk);
	write_reg(state, RSTV0910_P2_SFRINIT1 + state->regoff,
						((symb >> 8) & 0x7F));
	write_reg(state, RSTV0910_P2_SFRINIT0 + state->regoff, (symb & 0xFF));
}

static void stv091x_set_cfrinc(struct stv* state, u32 cfr_increment, bool automatic)
{
	//0x1c71
	u16	value;
	if (automatic) {
		value = 0x038e;
		value = 0x1c71;
	} else {
		value = MulDiv32(cfr_increment, 1<<13, state->base->mclk);
		if(value >= 1<<10) {
			dprintk("cfrinc out of range: %d\n", value);
			value = 1<<10;
		}
		value <<= 3;
		value |= 0x80;
	}
	{
	u16 old = read_reg(state, RSTV0910_P2_CFRINC1 + state->regoff) << 8 |
		read_reg(state, RSTV0910_P2_CFRINC0 + state->regoff);
	write_reg(state, RSTV0910_P2_CFRINC1 + state->regoff,
						((value >> 8)));
	write_reg(state, RSTV0910_P2_CFRINC0 + state->regoff, (value & 0xFF));
	dprintk("CFR INC changed from 0x%x to 0x%x\n", old, value);
	}
}


static u32 stv091x_bandwidth(u32 rolloff, uint32_t symbol_rate)
{
	switch (rolloff) {
	case ROLLOFF_20:
		rolloff = 120;
		break;
	case ROLLOFF_25:
		rolloff = 125;
		break;
	default:
		rolloff = 135;
		break;
	}
	return MulDiv32(symbol_rate, rolloff, 200);
}

int stv091x_set_frequency_symbol_rate_bandwidth(struct stv* state)
{
	struct dtv_frontend_properties *p = &state->fe.dtv_property_cache;
	state-> tuner_frequency = p->frequency;
	if(state->satellite_scan) {
		state->tuner_bw = 2*36000000; 	/* Use maximal bandwidth */
		/* If Blind search set the init symbol rate to 1Msps*/
		dprintk("requesting stv6120 to set: freq=%dkHz bw=%dkHz\n", state->tuner_frequency, state->tuner_bw/1000);
		if(state->fe.ops.tuner_ops.set_frequency_and_bandwidth)
			state->fe.ops.tuner_ops.set_frequency_and_bandwidth(&state->fe, state->tuner_frequency, state->tuner_bw);
		dprintk("set symbol_rate to %dkHz\n", state->symbol_rate/1000);
		stv091x_set_symbol_rate(state, state->symbol_rate);
	} else {
		dprintk("symrate1=%d bw=%d\n", state->symbol_rate, state->tuner_bw);
		state->tuner_bw = stv091x_bandwidth(p->rolloff, state->symbol_rate);
		state->tuner_bw = 2*36000000; 	/* Use maximal bandwidth */
		dprintk("symrate2=%d bw=%d\n", state->symbol_rate, state->tuner_bw);
#if 0
		//bug!
		if(p->algorithm == ALGORITHM_BLIND || p->algorithm == ALGORITHM_BLIND_BEST_GUESS)
			state->tuner_bw += state->search_range;
#endif

		dprintk("symrate=%d bw=%d\n", state->symbol_rate, state->tuner_bw);
		if(state->fe.ops.tuner_ops.set_frequency_and_bandwidth) {
			dprintk("requesting stv6120 to set: freq=%d bw=%d\n", state->tuner_frequency, state->tuner_bw);
			state->fe.ops.tuner_ops.set_frequency_and_bandwidth(&state->fe, state->tuner_frequency, state->tuner_bw);
		}
		stv091x_set_symbol_rate(state, state->symbol_rate);
	}

	return 0;
}


static int stv091x_set_search_range(struct stv *state, struct dtv_frontend_properties *p)
{
	s32 range;
	dprintk("ALGO=%d\n", p->algorithm);
	if(p->algorithm == ALGORITHM_WARM || p->algorithm == ALGORITHM_COLD || p->algorithm == ALGORITHM_COLD_BEST_GUESS) {
		/* CFR min =- (search_range/2 + margin )
			 CFR max = +(search_range/2 + margin)
			 (80KHz margin if SR <=5Msps else margin =600KHz )*/
		if(p->search_range==0)
			state->search_range_hz =  160000000;
		else
			state->search_range_hz = p->search_range;
		range = state->search_range_hz/2000;
		if(state->symbol_rate >=5000000)
			range +=  600;
		else
			range += 80;
	} else if(p->algorithm == ALGORITHM_BLIND ||
#if 0
						ALGORITHM_SEARCH_NEXT||
#endif
						p->algorithm == ALGORITHM_BLIND_BEST_GUESS) {
		if(state->satellite_scan) {
			//set limits on how far we can search for a carrier
			//default search_range=16 000 000; make it less for lower symbol rate
			state->search_range_hz =
				(p->max_symbol_rate==0) ? 60000000 : p->max_symbol_rate;
			dprintk("search range p=%d => s=%d\n", p->max_symbol_rate, state->search_range_hz);

			range = (state->search_range_hz / 2000);
			//range = (range << 16) / (state->base->mclk / 1000);
		} else {
			range = state->search_range_hz / 2000; //for regular blindscan
			dprintk("RANGE=%d\n", range);
		}
	} else {
		BUG_ON(1);
		//set limits on how far we can search for a carrier
		//default search_range=16 000 000; make it less for lower symbol rate
		range = (state->search_range_hz / 2000);
	}
	dprintk("RANGE= set to %dkHz state->search_range=%dkHz\n", range, state->search_range_hz/1000);
	range = (range<<16)/(state->base->mclk/1000); //1Mhz
	write_reg(state, RSTV0910_P2_CFRUP1 + state->regoff,
						(range >> 8) & 0xff);
	write_reg(state, RSTV0910_P2_CFRUP0 + state->regoff, (range & 0xff));
	/*CFR Low Setting*/
	range = -range;
	write_reg(state, RSTV0910_P2_CFRLOW1 + state->regoff,
						(range >> 8) & 0xff);
	write_reg(state, RSTV0910_P2_CFRLOW0 + state->regoff, (range & 0xff));

	return 0;
}

static void stv091x_set_viterbi(struct stv* state, struct dtv_frontend_properties *p)
{

	s32 fecmReg,
		prvitReg;
	fecmReg=RSTV0910_P2_FECM;
	prvitReg=RSTV0910_P2_PRVIT;
	dprintk("system=%d\n", p->delivery_system);

	switch(p->delivery_system) {
	case SYS_AUTO:
			write_reg(state, fecmReg, 0x00);	/* Disable DSS in auto mode search for DVBS1 and DVBS2 only , DSS search is on demande*/
			write_reg(state, prvitReg, 0x2F); /* Enable All PR exept 6/7 */
			break;
	case SYS_DVBS:
		write_reg(state, fecmReg, 0x00);	/* Disable DSS */
		switch(p->fec_inner) {
		case FEC_NONE:
		default:
			write_reg(state, prvitReg, 0x2F); /* Enable All PR exept 6/7 */
			break;

		case FEC_1_2:
			write_reg(state, prvitReg, 0x01); /* Enable 1/2 PR only */
			break;

		case FEC_2_3:
			write_reg(state, prvitReg, 0x02); /* Enable 2/3 PR only */
			break;

		case FEC_3_4:
			write_reg(state, prvitReg, 0x04); /* Enable 3/4 PR only */
			break;

		case FEC_5_6:
			write_reg(state, prvitReg, 0x08); /* Enable 5/6 PR only */
			break;

		case FEC_7_8:
			write_reg(state, prvitReg, 0x20); /* Enable 7/8 PR only */
			break;
		}

		break;

	case SYS_DSS:
		write_reg(state, fecmReg, 0x80);	/* Disable DVBS1 */
		switch(p->fec_inner) {
		case FEC_NONE:
		default:
			write_reg(state, prvitReg, 0x13); /* Enable 1/2, 2/3 and 6/7 PR */
			break;

		case FEC_1_2:
			write_reg(state, prvitReg, 0x01); /* Enable 1/2 PR only */
			break;

		case FEC_2_3:
			write_reg(state, prvitReg, 0x02); /* Enable 2/3 PR only */
			break;

		case FEC_6_7:
			write_reg(state, prvitReg, 0x10); /* Enable All 7/8 PR only */
			break;
		}

		break;

	default:
		break;
	}
}

static int stv091x_set_search_standard(struct stv *state, struct dtv_frontend_properties *p)
{
	u8 regDMDCFGMD;


	if(p->algorithm == ALGORITHM_WARM)
		state->DEMOD |= 0x80; //manual mode for rolloff
	else
		state->DEMOD &= ~0x80; //automatic mode for rolloff
	write_reg(state, RSTV0910_P2_DEMOD + state->regoff, state->DEMOD);

	regDMDCFGMD = read_reg(state, RSTV0910_P2_DMDCFGMD + state->regoff);
	if(p->algorithm== ALGORITHM_BLIND || p->algorithm==ALGORITHM_BLIND_BEST_GUESS)
		write_reg(state, RSTV0910_P2_DMDCFGMD + state->regoff,
						regDMDCFGMD |= 0xF0); 	//enable DVBS2 and DVBS1 and also enable search and scan
	else
		write_reg(state, RSTV0910_P2_DMDCFGMD + state->regoff,
							regDMDCFGMD |= 0xC0); 	//enable DVBS2 and DVBS1 search, but not symbol rate and frequency scanning

	/* Disable DSS */
	write_reg(state, RSTV0910_P2_FECM  + state->regoff, 0x00); //dvb-s1; no dss
	write_reg(state, RSTV0910_P2_PRVIT + state->regoff, 0x7F); //allow all puncture rates, disable auto-adjustment of VTH

	/* 8PSK 3/5, 8PSK 2/3 Poff tracking optimization WA*/
	write_reg(state, RSTV0910_P2_ACLC2S2Q + state->regoff, 0x0B);
	write_reg(state, RSTV0910_P2_ACLC2S28 + state->regoff, 0x0A);
	write_reg(state, RSTV0910_P2_BCLC2S2Q + state->regoff, 0x84);
	write_reg(state, RSTV0910_P2_BCLC2S28 + state->regoff, 0x84);
	write_reg(state, RSTV0910_P2_CARHDR + state->regoff, 0x1C);
	//	write_reg(state, RSTV0910_P2_CARCFG + state->regoff, 0x46);

	/* Reset demod */
	write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x1F);

	return 0;
}

static int stv091x_start_scan(struct stv *state, struct dtv_frontend_properties *p)
{
	dprintk("freq=%d symbol_rate=%d\n", p->frequency, state->symbol_rate);
	//set register to detault: citroen2 phase detection, open carrier1 loop (no derotator()
	write_reg(state, RSTV0910_P2_CARCFG + state->regoff, 0x46);

	stv091x_set_search_range(state, p);

	/* init the demod frequency offset to 0 */
	write_reg(state, RSTV0910_P2_CFRINIT1 + state->regoff, 0);
	write_reg(state, RSTV0910_P2_CFRINIT0 + state->regoff, 0);

#if 0
	//reset demod state machine
	write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x15);
#endif
	/* Trigger acq */
	switch(p->algorithm) {
	case ALGORITHM_BLIND:
		write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x00); //blind zero offset
		break;
	case ALGORITHM_BLIND_BEST_GUESS:
		write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x01); //blind best guess
		break;
	case ALGORITHM_COLD:
		write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x15); //cold zero offset
		break;
	case ALGORITHM_COLD_BEST_GUESS:
		write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x05); //cold best guess
		break;
#if 0
	case ALGORITHM_SEARCH_NEXT:
		write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x14);
		break;
	case ALGORITHM_BANDWIDTH:
		write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x11);
		break;
#endif
	case ALGORITHM_WARM: //todo
		/*better for low symbol rate - only choice which allows tuning to symbolrate 667
			Note that tuner bandwith still takes into account known symbol rate
		 */
		write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x01);
		break;
	default:
		write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x18); //everything known zero foffset
		break;
	}
#if 0
		/* Trigger acq */
	write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x15);
#endif
	return 0;
}


static int init_diseqc(struct stv *state)
{
	u8 Freq = ((state->base->mclk + 11000 * 32) / (22000 * 32));

	/* Disable receiver */
	write_reg(state, RSTV0910_P1_DISRXCFG, 0x00);
	write_reg(state, RSTV0910_P2_DISRXCFG, 0x00);
	write_reg(state, RSTV0910_P1_DISTXCFG, 0x82); /* Reset = 1 */
	write_reg(state, RSTV0910_P1_DISTXCFG, 0x02); /* Reset = 0 */
	write_reg(state, RSTV0910_P2_DISTXCFG, 0x82); /* Reset = 1 */
	write_reg(state, RSTV0910_P2_DISTXCFG, 0x02); /* Reset = 0 */
	write_reg(state, RSTV0910_P1_DISTXF22, Freq);
	write_reg(state, RSTV0910_P2_DISTXF22, Freq);
	return 0;
}

static int stv091x_probe(struct stv *state)
{
	u8 reg;

	if (try_read_reg(state, RSTV0910_MID, &reg) < 0)
		return -1;

	if (reg != 0x51)
		return -EINVAL;
	pr_info("stv0910: found STV0910 id=0x%02x\n", reg);

	 /* Configure the I2C repeater to off */
	write_reg(state, RSTV0910_P1_I2CRPT, 0x24);
	/* Configure the I2C repeater to off */
	write_reg(state, RSTV0910_P2_I2CRPT, 0x24);
	/* Set the I2C to oversampling ratio */
	write_reg(state, RSTV0910_I2CCFG, 0x88);

	write_reg(state, RSTV0910_OUTCFG, 0x00); /* OUTCFG */
	write_reg(state, RSTV0910_PADCFG, 0x05); /* RF AGC Pads Dev = 05 */
	write_reg(state, RSTV0910_SYNTCTRL, 0x02); /* SYNTCTRL */
	write_reg(state, RSTV0910_TSGENERAL, state->tsgeneral); /* TSGENERAL */
	write_reg(state, RSTV0910_CFGEXT, 0x02); /* CFGEXT */

	reg = single ? 0x14 : 0x15; /* Single or dual mode */
	if (ldpc_mode)
		reg &= ~0x10; /* LDPC ASAP mode */
	write_reg(state, RSTV0910_GENCFG, reg); /* GENCFG */

	write_reg(state, RSTV0910_P1_TNRCFG2, 0x02); /* IQSWAP = 0 */
	write_reg(state, RSTV0910_P2_TNRCFG2, 0x02); /* IQSWAP = 0 */

	write_reg(state, RSTV0910_P1_CAR3CFG, 0x02);
	write_reg(state, RSTV0910_P2_CAR3CFG, 0x02);
	write_reg(state, RSTV0910_P1_DMDCFG4, 0x04);
	write_reg(state, RSTV0910_P2_DMDCFG4, 0x04);

	/* BCH error check mode */
	write_reg(state, RSTV0910_P1_PDELCTRL2 , no_bcherr ? 0x01 : 0);
	write_reg(state, RSTV0910_P2_PDELCTRL2 , no_bcherr ? 0x21 : 0x20);
	write_reg(state, RSTV0910_P1_PDELCTRL3 , no_bcherr ? 0x20 : 0);
	write_reg(state, RSTV0910_P2_PDELCTRL3 , no_bcherr ? 0x20 : 0);

	write_reg(state, RSTV0910_TSTRES0, 0x80); /* LDPC Reset */
	write_reg(state, RSTV0910_TSTRES0, 0x00);

	write_reg(state, RSTV0910_P1_TSPIDFLT1, 0x00);
	write_reg(state, RSTV0910_P2_TSPIDFLT1, 0x00);

	write_reg(state, RSTV0910_P1_TMGCFG2, 0x80);
	write_reg(state, RSTV0910_P2_TMGCFG2, 0x80);

	set_mclock(state, 135000000);

	/* TS output options */
	write_reg(state, RSTV0910_P1_TSCFGM , 0); /* TS speed control - auto*/
	write_reg(state, RSTV0910_P2_TSCFGM , 0);
	write_reg(state, RSTV0910_P1_TSCFGL , 0x20);
	write_reg(state, RSTV0910_P2_TSCFGL , 0x20);
	write_reg(state, RSTV0910_P1_TSSPEED , 0x20);
	write_reg(state, RSTV0910_P2_TSSPEED , 0x20);

	/* Reset stream merger */
	write_reg(state, RSTV0910_P1_TSCFGH , state->tscfgh | 0x01);
	write_reg(state, RSTV0910_P1_TSCFGH , state->tscfgh);
	write_reg(state, RSTV0910_P2_TSCFGH , state->tscfgh | 0x01);
	write_reg(state, RSTV0910_P2_TSCFGH , state->tscfgh);

	write_reg(state, RSTV0910_P1_I2CRPT, state->i2crpt);
	write_reg(state, RSTV0910_P2_I2CRPT, state->i2crpt);

	init_diseqc(state);
	return 0;
}


static int gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct stv *state = fe->demodulator_priv;
	u8 i2crpt = state->i2crpt & ~0x86;
	u16 reg;

	if (enable)
		mutex_lock(&state->base->i2c_lock);

	if (enable)
		i2crpt |= 0x80;
	else
		i2crpt |= 0x02;

	switch (state->base->dual_tuner)
	{
	 case 1:
	 reg = RSTV0910_P1_I2CRPT;
	 break;
	 case 2:
	 reg = RSTV0910_P2_I2CRPT;
	 break;
	 default:
	 reg = state->adapterno ? RSTV0910_P2_I2CRPT : RSTV0910_P1_I2CRPT;
	}

	/* pr_info("stv0910: gate_ctrl %d\n", enable); */

	if (write_reg(state, reg , i2crpt) < 0)
		return -EIO;

	state->i2crpt = i2crpt;

	if (!enable)
		mutex_unlock(&state->base->i2c_lock);
	return 0;
}

static void release(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;

	if (state->base->set_lock_led)
		state->base->set_lock_led(fe, 0);

	state->base->count--;
	if (state->base->count == 0) {
		list_del(&state->base->stvlist);
		kfree(state->base);
	}
	kfree(state);
}


static int stv091x_isi_scan(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct stv *state = fe->demodulator_priv;
	u8 CurrentISI;
	u8 matype;
	int i;
	u8 tmp;
	u8 regs[2];

	dprintk("ISI scan\n");
	//disable ISI filtering so that we can observe all stream_id's
	tmp = read_reg(state, RSTV0910_P2_PDELCTRL1 + state->regoff);
	tmp |= 0x20; //0x20: enable mis filter
	write_reg(state, RSTV0910_P2_PDELCTRL1 + state->regoff, tmp);


		//pr_warn("%s: enable MIS filtering - %d\n", __func__, mis & 0xff);
	write_reg(state, RSTV0910_P2_ISIENTRY + state->regoff, 0x00 ); //match value
	write_reg(state, RSTV0910_P2_ISIBITENA + state->regoff, 0x00 );

	//ensure that we will observe the current isi
	write_reg(state, RSTV0910_P2_PDELCTRL0 + state->regoff, 0);
	msleep(40);

	/* Get Current ISI and store in struct */
	memset(&p->isi_bitset[0], 0, sizeof(p->isi_bitset));

	for (i=0; i < 40; i++) {
		uint32_t mask;
		int j;
		regs[0] = read_reg(state, RSTV0910_P2_MATSTR1);
		regs[1] = read_reg(state, RSTV0910_P2_MATSTR0);
		matype = regs[0];
		CurrentISI = regs[1];

		j = CurrentISI/32;
		mask = ((uint32_t)1)<< (CurrentISI%32);
		p->isi_bitset[j] |= mask;
#ifdef TODO
		if(CurrentISI == state->signal_info.isi)
#endif
			p->matype = matype;
		dprintk("XXXX i=%d matype=%d currentISI=%d state->signal_info.isi=%d\n",
						i, matype, CurrentISI, state->signal_info.isi);

		msleep(10);
	}
	//restore proper mis
	if(p->algorithm == ALGORITHM_COLD || p->algorithm == ALGORITHM_COLD_BEST_GUESS) {
		dprintk("SET_STREAM_INDEX: %d\n", p->stream_id);
		set_stream_index(state, p->stream_id);
	}
	return 0;
}

static int stv091x_get_standard(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	u8 dmdstate;
	dmdstate = read_reg(state, RSTV0910_P2_DMDSTATE + state->regoff);
	switch((dmdstate >>5)&0x3) {
	case 0: //searching
		break;
	case 1: //1st DVB-S2 PLHeader detected, searching for residual offset symbol.
	case 2: //DVB-S2 mode
		state->signal_info.standard = SYS_DVBS2;
		break;
	case 3: //DVB-S1/Legacy DTV mode (unsigned)
			state->signal_info.standard = SYS_DVBS;
		break;
	}
	dprintk("STANDARD=%d dmdstate=0x%x\n", state->signal_info.standard, dmdstate);
	return 0;
}

/*
	retrieve information about modulation, frequency, symbol_rate
	but not CNR, BER (different from stid135)

	returns 1 if tuner frequency or bandwidth needs to be updated
 */
static bool stv091x_get_signal_info(struct dvb_frontend *fe)
{
	bool need_retune = false;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct stv *state = fe->demodulator_priv;
	u8 regs[2];
	u32 bandwidth_hz;
	s32 carrier_frequency_offset;
	u8 rolloff_status, rolloff;

	read_regs(state, RSTV0910_P2_CFR2, regs, 2);
	carrier_frequency_offset = (s16)((regs[0]<<8) | regs[1]);
	carrier_frequency_offset *= ((state->base->mclk>>16) / 1000);
	state->signal_info.frequency = state->tuner_frequency + carrier_frequency_offset;
	dprintk("freq: %d/%d/%d offset %d", p->frequency,
					state->tuner_frequency,
					state->signal_info.frequency, carrier_frequency_offset);
	if (carrier_frequency_offset > 1000 || carrier_frequency_offset< -1000)
		need_retune = true;

	stv091x_symbol_rate(state, &state->symbol_rate, false);
	p->symbol_rate = state->symbol_rate;
	p->frequency = state->signal_info.frequency;

	stv091x_get_standard(fe);

	p->delivery_system = state->signal_info.standard;

	regs[0] = read_reg(state, RSTV0910_P2_TMGOBS);
	rolloff_status = (regs[0]>>6);
	switch(rolloff_status) {
		case 0x03:
			rolloff = 115;
			break;
		case 0x02:
			rolloff = 120;
			break;
		case 0x01:
			rolloff = 125;
			break;
		case 0x00:
		default:
			rolloff = 135;
			break;
		}
	dprintk("mis_mode=%d isi=%d pls_mode=%d pls_code=%d\n",
					state->mis_mode, state->signal_info.isi, state->signal_info.pls_mode,
					state->signal_info.pls_code);
	p->stream_id = ((state->mis_mode ? (state->signal_info.isi &0xff) :0xff) |
									(state->signal_info.pls_mode << 26) |
									((state->signal_info.pls_code &0x3FFFF)<<8)
									);

		bandwidth_hz = (state->symbol_rate * rolloff) / 100;

		if (state->tuner_bw > bandwidth_hz || state->tuner_bw < (bandwidth_hz*8)/10)
			need_retune = true; // ask tuner to change its bandwidth, except duting blind scan

		dprintk("SR: %d, BW: %d => %d need_retune=%d\n", state->symbol_rate, state->tuner_bw, bandwidth_hz, need_retune);
		stv091x_isi_scan(fe);
		return need_retune;
}





static int pls_search_list(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i = 0;
	int locked = 0;
	for(i=0; i<p->pls_search_codes_len;++i) {
		u32 pls_code = p->pls_search_codes[i];
		u8 pktdelin;
		u8 timeout = pls_code & 0xff;
		dprintk("Trying scrambling mode=%d code %d timeout=%d\n", (pls_code>>26) & 0x3, (pls_code>>8) & 0x3FFFF, timeout);
		set_pls_mode_code(state, (pls_code>>26) & 0x3, (pls_code>>8) & 0x3FFFF);
		//write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x15);
		//write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x18);
			msleep(timeout? timeout: 15); //0 means: use default
			pktdelin = read_reg(state, RSTV0910_P2_PDELSTATUS1 + state->regoff);
			if (pktdelin&0x02 /*packet delineator locked*/)
				locked=1;
			if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
				dprintk("exiting on should stop\n");
				break;
			}
			//dprintk("RESULT=%d\n", locked);
			if(locked) {
				u8 stream_id;
				write_reg(state, RSTV0910_P2_PDELCTRL0 + state->regoff, 0);
				msleep(40);
				stream_id = read_reg(state, RSTV0910_P2_MATSTR1); //dummy read. Apparantly needed!
				stream_id = read_reg(state, RSTV0910_P2_MATSTR0);
				dprintk("selecting stream_id=%d\n", stream_id);
				p->stream_id = 	(stream_id&0xff) | (pls_code & ~0xff);
				break;
			}
	}
	return locked;
}

static int pls_search_range(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	u32 pls_code = 0;
	int locked = 0;
	u8 timeout = p->pls_search_range_start & 0xff;
	int count=0;
	if(timeout==0)
		timeout = 15;
	atomic_set(&fe->algo_state.cur_index, 0);
	atomic_set(&fe->algo_state.max_index, (p->pls_search_range_end - p->pls_search_range_start)>>8);
	for(pls_code=p->pls_search_range_start; pls_code<p->pls_search_range_end; pls_code += 0xff, count++) {
		u8 pktdelin;
		if((count+= timeout)>=1000) {
			dprintk("Trying scrambling mode=%d code %d timeout=%d ...\n",
							(pls_code>>26) & 0x3, (pls_code>>8) & 0x3FFFF, timeout);
			atomic_add(count, &fe->algo_state.cur_index);
			count=0;
			wake_up_interruptible(&fe->algo_state.wait_queue);
		}
		set_pls_mode_code(state, (pls_code>>26) & 0x3, (pls_code>>8) & 0x3FFFF);
		//write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x15);
		//write_reg(state, RSTV0910_P2_DMDISTATE + state->regoff, 0x18);
		msleep(timeout? timeout: 25); //0 means: use default
		pktdelin = read_reg(state, RSTV0910_P2_PDELSTATUS1 + state->regoff);
		if (pktdelin&0x02 /*packet delineator locked*/)
			locked=1;
		if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
			dprintk("exiting on should stop\n");
			break;
		}
		dprintk("RESULT=%d\n", locked);
		if(locked) {
				u8 stream_id;
				write_reg(state, RSTV0910_P2_PDELCTRL0 + state->regoff, 0);
				msleep(40);
				stream_id = read_reg(state, RSTV0910_P2_MATSTR1); //dummy read. Apparantly needed!
				stream_id = read_reg(state, RSTV0910_P2_MATSTR0);
				dprintk("selecting stream_id=%d\n", stream_id);
				p->stream_id = 	(stream_id&0xff) | (pls_code & ~0xff);
				break;
			}
	}
	return locked;
}



static int get_frontend(struct dvb_frontend *fe, struct dtv_frontend_properties *p)
{
	struct stv *state = fe->demodulator_priv;
	u8 tmp;


	if (state->ReceiveMode == Mode_DVBS2) {
		u32 mc;
		enum fe_modulation modcod2mod[0x20] = {
			QPSK, QPSK, QPSK, QPSK,
			QPSK, QPSK, QPSK, QPSK,
			QPSK, QPSK, QPSK, QPSK,
			PSK_8, PSK_8, PSK_8, PSK_8,
			PSK_8, PSK_8, APSK_16, APSK_16,
			APSK_16, APSK_16, APSK_16, APSK_16,
			APSK_32, APSK_32, APSK_32, APSK_32,
			APSK_32,
		};
		enum fe_code_rate modcod2fec[0x20] = {
			FEC_NONE, FEC_1_4, FEC_1_3, FEC_2_5,
			FEC_1_2, FEC_3_5, FEC_2_3, FEC_3_4,
			FEC_4_5, FEC_5_6, FEC_8_9, FEC_9_10,
			FEC_3_5, FEC_2_3, FEC_3_4, FEC_5_6,
			FEC_8_9, FEC_9_10, FEC_2_3, FEC_3_4,
			FEC_4_5, FEC_5_6, FEC_8_9, FEC_9_10,
			FEC_3_4, FEC_4_5, FEC_5_6, FEC_8_9,
			FEC_9_10
		};
		//dprintk("receive mode = DVBS2\n");
		tmp = read_reg(state, RSTV0910_P2_DMDMODCOD + state->regoff);
		mc = ((tmp & 0x7c) >> 2);
		p->pilot = (tmp & 0x01) ? PILOT_ON : PILOT_OFF;
		p->modulation = modcod2mod[mc];
		p->fec_inner = modcod2fec[mc];
	} else if (state->ReceiveMode == Mode_DVBS) {
		//dprintk("receive mode = DVBS\n");
		tmp = read_reg(state, RSTV0910_P2_VITCURPUN + state->regoff);
		switch( tmp & 0x1F ) {
 case 0x0d:
			p->fec_inner = FEC_1_2;
			break;
 case 0x12:
			p->fec_inner = FEC_2_3;
			break;
 case 0x15:
			p->fec_inner = FEC_3_4;
			break;
 case 0x18:
			p->fec_inner = FEC_5_6;
			break;
 case 0x1a:
			p->fec_inner = FEC_7_8;
			break;
		default:
			p->fec_inner = FEC_NONE;
			break;
		}
		p->rolloff = ROLLOFF_35;
	} else {
		//dprintk("receive mode = %d\n",state->ReceiveMode);
	}
	return 0;
}

/*
	a larger power gain means a lower signal!
 */
static s32 stv091x_agc1_power_gain(struct stv *state)
{
	u8 Reg[2];
	s32 agc_power;
	read_regs(state, RSTV0910_P2_AGCIQIN1 + state->regoff, Reg, 2);

	//convert into into gain according to lookup table of tuner
	agc_power = (((u32) Reg[0]) << 8) | Reg[1];
	return agc_power;
	//todo: Py_AGC, AGC2_INTEGRATOR
}

//units of 0.001dB
static s32 stv091x_agc1_power_gain_dbm(struct stv *state)
{
	u16 agc_gain = stv091x_agc1_power_gain(state); //u16 because of stv6120 interface (todo)
	s32 gain = agc_gain;
	if (state->fe.ops.tuner_ops.agc_to_gain_dbm)
		gain = state->fe.ops.tuner_ops.agc_to_gain_dbm(&state->fe, agc_gain);//in units of 0.01dB
	return 10*gain;
}


static s32 stv091x_iq_power(struct stv *state)
{
	int i;
	u8 Reg[2];
	s32 iq_power;
	iq_power=0;
	//measure I and Q power
	for (i = 0; i < 5; i += 1) {
		read_regs(state, RSTV0910_P2_POWERI + state->regoff, Reg, 2);
		iq_power += (u32) Reg[0] * (u32) Reg[0] + (u32) Reg[1] * (u32) Reg[1];
		msleep(3);
	}
	iq_power /= 5;
	return iq_power;
	//todo: Py_AGC, AGC2_INTEGRATOR
}

//unit 0.001dB
static s32 stv091x_iq_power_dbm(struct stv *state)
{
	s32 iq_power = stv091x_iq_power(state);
	return 10*(s32)(TableLookup(PADC_Lookup, ARRAY_SIZE(PADC_Lookup), iq_power) + 352);
}

/*
	signal power in the stv6120 tuner bandwidth (not representative for narrow band signals)
	unit: 0.001dB
 */
static s32 stv091x_signal_power_dbm(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	//todo: take into account agc2
	return (-520 - stv091x_agc1_power_gain_dbm(state)); //unit is 0.01dB
		/* -0.52 is the output level in dBm*/
	}

/*
	signal power representative for narrow band signals
	unit: 0.001dB
 */
static s32 stv091x_narrow_band_signal_power_dbm(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;
	//todo: take into account agc2
	s32 agc2level = (read_reg_field(state, FSTV0910_P2_AGC2_INTEGRATOR1) <<8) |
		read_reg_field(state, FSTV0910_P2_AGC2_INTEGRATOR0);
	s32	agc2ref = read_reg(state, RSTV0910_P2_AGC2REF);
	s32 x =  20*(s32)STLog10((u32)(agc2ref)); //unit is 0.001dB
	s32 y =  20*(s32)STLog10(agc2level); //unit is 0.001dB
	return (x-y) + (-520 - stv091x_agc1_power_gain_dbm(state)) +7991; //unit is 0.001dB
}

/*
	read rf level, snr, signal quality, lock_status
 */

static int read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	u8 DmdState = 0;
	u8 DStatus = 0;
	enum ReceiveMode CurReceiveMode = Mode_None;
	u32 FECLock = 0;
	s32 snr;
	u32 n, d;
	s32 signal_strength = stv091x_signal_power_dbm(fe); //unit=0.001dB

	/*pr_warn("%s: agc = %d iq_power = %d Padc = %d\n", __func__, agc, iq_power, Padc);*/

	p->cnr.len = p->post_bit_error.len = p->post_bit_count.len = 1;
	p->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	p->strength.len = 2;
	p->strength.stat[0].scale = FE_SCALE_DECIBEL;
	p->strength.stat[0].svalue = signal_strength; //result in units of 0.001dB

	p->strength.stat[1].scale = FE_SCALE_RELATIVE;
	p->strength.stat[1].uvalue = (100 + signal_strength/1000) * 656; //todo: check range

	//todo: if not locked, and signal is too low FE_HAS_SIGNAL should be removed
	*status = FE_HAS_SIGNAL;

	DmdState = read_reg(state, RSTV0910_P2_DMDSTATE + state->regoff);
	//	dprintk("DMDSTATE=0x%x\n", DmdState);
	if (DmdState & 0x40) { //DVBS1 or DVBS2 found
		DStatus = read_reg(state, RSTV0910_P2_DSTATUS + state->regoff);
		dprintk("DStatus=0x%x\n", DStatus);
		if (DStatus & 0x80)
			*status |= FE_HAS_CARRIER; // *carrierlock==True
		if (DStatus & 0x08)
			CurReceiveMode = (DmdState & 0x20) ?
				Mode_DVBS : Mode_DVBS2;
	}
	//dprintk("receive_mode =%d\n", CurReceiveMode);
	if (CurReceiveMode == Mode_None)
	{
 		if (state->base->set_lock_led)
			state->base->set_lock_led(fe, 0);
		return 0;
	}
	//dprintk("state->receive_mode =%d\n", state->ReceiveMode);
	if (state->ReceiveMode == Mode_None) {
		state->ReceiveMode = CurReceiveMode;
		state->DemodLockTime = jiffies;
		state->FirstTimeLock = 1;
		GetSignalParameters(state);
		TrackingOptimization(state);
		write_reg(state, RSTV0910_P2_TSCFGH + state->regoff,
			 state->tscfgh);
		usleep_range(3000, 4000);
		write_reg(state, RSTV0910_P2_TSCFGH + state->regoff,
			 state->tscfgh | 0x01);
		write_reg(state, RSTV0910_P2_TSCFGH + state->regoff,
							state->tscfgh);
	}

	if (DmdState & 0x40) {
		if (state->ReceiveMode == Mode_DVBS2) {
			u8 PDELStatus;
			PDELStatus = read_reg(state, RSTV0910_P2_PDELSTATUS1 + state->regoff);
			FECLock = (PDELStatus & 0x02) != 0; //*datapresent==True
		} else {
			u8 VStatus;
			VStatus = read_reg(state, RSTV0910_P2_VSTATUSVIT + state->regoff);
			FECLock = (VStatus & 0x08) != 0; //*datapresent==True
		}
	}
	dprintk("FECLock=%d\n", FECLock);
	if (!FECLock)
	{
		if (state->base->set_lock_led)
			state->base->set_lock_led(fe, 0);

		return 0;
	}
	dprintk("setting status ok\n");
	*status |= FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;

	if (state->base->set_lock_led)
		state->base->set_lock_led(fe, *status & FE_HAS_LOCK);

	dprintk("FirstTimeLock=%d\n", state->FirstTimeLock);
	if (state->FirstTimeLock) {
		u8 tmp;

		state->FirstTimeLock = 0;

		if (state->ReceiveMode == Mode_DVBS2) {
			/* FSTV0910_P2_MANUALSX_ROLLOFF,
			 FSTV0910_P2_MANUALS2_ROLLOFF = 0 */
			state->DEMOD &= ~0x84;
			write_reg(state, RSTV0910_P2_DEMOD + state->regoff,
				 state->DEMOD);
			tmp = read_reg(state, RSTV0910_P2_PDELCTRL2 + state->regoff);
			/*reset DVBS2 packet delinator error counter */
			tmp |= 0x40;
			write_reg(state, RSTV0910_P2_PDELCTRL2 + state->regoff,
				 tmp);
			/*reset DVBS2 packet delinator error counter */
			tmp &= ~0x40;
			write_reg(state, RSTV0910_P2_PDELCTRL2 + state->regoff,
				 tmp);

			state->BERScale = 2;
			state->LastBERNumerator = 0;
			state->LastBERDenominator = 1;
			/* force to PRE BCH Rate */
			write_reg(state, RSTV0910_P2_ERRCTRL1 + state->regoff,
				 BER_SRC_S2 | state->BERScale);
		} else {
			state->BERScale = 2;
			state->LastBERNumerator = 0;
			state->LastBERDenominator = 1;
			/* force to PRE RS Rate */
			write_reg(state, RSTV0910_P2_ERRCTRL1 + state->regoff,
				 BER_SRC_S | state->BERScale);
		}
		/*Reset the Total packet counter */
		write_reg(state, RSTV0910_P2_FBERCPT4 + state->regoff, 0x00);
		/*Reset the packet Error counter2 (and Set it to
		 infinit error count mode )*/
		write_reg(state, RSTV0910_P2_ERRCTRL2 + state->regoff, 0xc1);
	}

	if (GetSignalToNoise(state, &snr))
		return -EIO;

	p->cnr.len = 2;
	p->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	p->cnr.stat[0].svalue = snr * 100;

	p->cnr.stat[1].scale = FE_SCALE_RELATIVE;
	p->cnr.stat[1].uvalue = snr*328;
	if (p->cnr.stat[1].uvalue > 0xffff)
		p->cnr.stat[1].uvalue = 0xffff;

	if (GetBitErrorRate(state, &n, &d))
		return -EIO;

	p->post_bit_error.len = 1;
	p->post_bit_error.stat[0].scale = FE_SCALE_COUNTER;
	p->post_bit_error.stat[0].uvalue = n;
	p->post_bit_count.len = 1;
	p->post_bit_count.stat[0].scale = FE_SCALE_COUNTER;
	p->post_bit_count.stat[0].uvalue = d;

	return 0;
}

typedef enum
	{
		/*		FE_SAT_NOAGC1=0,
		FE_SAT_AGC1OK,
		FE_SAT_NOTIMING,
		FE_SAT_ANALOGCARRIER,
		*/
		FE_SAT_TUNER_NOSIGNAL,
		FE_SAT_TUNER_JUMP,
		/*
		FE_SAT_TUNER_4_STEP,
		FE_SAT_TUNER_8_STEP,
		FE_SAT_TUNER_16_STEP,
		FE_SAT_TIMINGOK,
		FE_SAT_NOAGC2,
		FE_SAT_AGC2OK,
		*/
		FE_SAT_NOCARRIER,
		/*
		FE_SAT_CARRIEROK,
		FE_SAT_NODATA,
		FE_SAT_DATAOK,
		FE_SAT_OUTOFRANGE,
		FE_SAT_RANGEOK
		*/
	} stv091x_SIGNALTYPE_t;

#define STV0910_BLIND_SEARCH_AGC2BANDWIDTH 40

//check if we are on a leading or falling edge of a potential signal
//returns 0 if nothing found, 1 on rising edge and 2 on falling edge
static int stv091x_carrier_check(struct stv* state, s32 *FinalFreq, s32* frequency_jump)
{
	u32 minagc2level=0xffff,maxagc2level=0x0000,midagc2level,
		agc2level,
		agc2ratio;
	s32	init_freq,freq_step;
	u32	tmp1,tmp2,tmp3,tmp4;
	u32 asperity=0;
	u32 waitforfall=0;
	u32 acculevel=0;
	u32 div=2;
	u32 agc2leveltab[20];

	s32 i,j,k,l,nbSteps;


	write_reg(state, RSTV0910_P2_DMDISTATE,0x1C);
	tmp2= read_reg(state, RSTV0910_P2_CARCFG);
	write_reg(state, RSTV0910_P2_CARCFG,0x06);

	tmp3= read_reg(state, RSTV0910_P2_BCLC);
	write_reg(state, RSTV0910_P2_BCLC,0x00);
	tmp4= read_reg(state, RSTV0910_P2_CARFREQ);
	write_reg(state, RSTV0910_P2_CARFREQ,0x00); //stop coarse carrrier corr

	write_reg(state, RSTV0910_P2_AGC2REF,0x38);

	tmp1= read_reg(state, RSTV0910_P2_DMDCFGMD);
	write_reg_fields(state, RSTV0910_P2_DMDCFGMD,
									 {FSTV0910_P2_DVBS1_ENABLE, 1},
									 {FSTV0910_P2_DVBS2_ENABLE, 1},
									 {FSTV0910_P2_SCAN_ENABLE, 0}, 		/*Enable the SR SCAN*/
									 {FSTV0910_P2_CFR_AUTOSCAN, 0} /*activate the carrier frequency search loop*/
									 );
	stv091x_set_symbol_rate(state, 1000000/div); /*AGC2 bandwidth is 1/div MHz */

	nbSteps=(state->tuner_bw/3000000)*div;
	if(nbSteps<=0)
		nbSteps=1;

	freq_step=((1000000<<8)/(state->base->mclk>>8))/div; /* AGC2 step is 1/div MHz */

	init_freq=0;
	j=0;	/* index after a rising edge is found */

	for(i=0;i<nbSteps;i++)
	{
		/* Scan on the positive part of the tuner Bw */
		//dprintk("freq=%d\n", init_freq);
		write_reg(state, RSTV0910_P2_DMDISTATE, 0x1C);
		write_reg(state, RSTV0910_P2_CFRINIT1, (init_freq >>8) & 0xff);
		write_reg(state, RSTV0910_P2_CFRINIT0, init_freq & 0xff);
		write_reg(state, RSTV0910_P2_DMDISTATE, 0x18); //zero offset start
		msleep(5);

		agc2level=0;
		agc2level = (read_reg_field(state, FSTV0910_P2_AGC2_INTEGRATOR1) <<8) |
			read_reg_field(state, FSTV0910_P2_AGC2_INTEGRATOR0);

		if (i == 0) {
			minagc2level= agc2level;
			maxagc2level= agc2level;
			midagc2level= agc2level;

			for (k=0;k<5*div;k++) {
				agc2leveltab[k]= agc2level;
			}
		} else {

			k= i%(5*div);
			agc2leveltab[k]= agc2level;

			minagc2level=0xffff;
			maxagc2level=0x0000;
			acculevel=0;

			for (l=0;l<5*div;l++) {
				/* Min and max detection */

				if( agc2leveltab[l]<minagc2level ) {
					minagc2level=agc2leveltab[l];
				} else if(agc2leveltab[l] >maxagc2level) {
					maxagc2level=agc2leveltab[l];
				}

				acculevel=acculevel+agc2leveltab[l];
			}

			midagc2level= acculevel/(5*div);

			if (waitforfall==0) {
				agc2ratio = (maxagc2level - minagc2level)*128/midagc2level;
			} else {
				agc2ratio = (agc2level - minagc2level)*128/midagc2level;
			}

			if (agc2ratio > 0xffff)
				agc2ratio = 0xffff;


			if ((agc2ratio > STV0910_BLIND_SEARCH_AGC2BANDWIDTH) && (agc2level==minagc2level)) {	 /* rising edge */
				asperity=1;		 /* The first edge is rising */
				waitforfall=1;
				for (l=0;l<5*div;l++) {
					agc2leveltab[l]= agc2level;
				}
			} else if ((agc2ratio > STV0910_BLIND_SEARCH_AGC2BANDWIDTH)) {
				/* Falling edge */
				if (waitforfall==0) {
					asperity=2; /* the first edge is falling */
				} else {
					asperity=1;
				}

				if (j==1) {
					for (l=0;l<5*div;l++) {
						agc2leveltab[l]= agc2level;
					}

					j=0;				 /* All reset */
					waitforfall=0;
					asperity=0;
				} else {
					break;
				}
			}

			if ((waitforfall==1) && j==(5*div)) {
				break;
			}
			if (waitforfall==1) {
				j+=1;
			}
		} /* end of i!=0 */

		init_freq = init_freq + freq_step;

	} /* End of for (i=0;i<nbSteps) */

	//restore registers
	write_reg(state, RSTV0910_P2_DMDCFGMD,tmp1);
	write_reg(state, RSTV0910_P2_CARCFG,tmp2);
	write_reg(state, RSTV0910_P2_BCLC,tmp3);
	write_reg(state, RSTV0910_P2_CARFREQ,tmp4);
	write_reg(state, RSTV0910_P2_DMDISTATE,0x1C);
	write_reg(state, RSTV0910_P2_CFRINIT1,0);
	write_reg(state, RSTV0910_P2_CFRINIT0,0);

	if (asperity ==1) { /* rising edge followed by a constant level or a falling edge */
		*frequency_jump = (1000/div)*(i-(j+2)/2);
	} else {
		*frequency_jump = (1000/div)*i; /* falling edge */
	}

	return asperity;
}

//returns true if a rising edge in spectrum was found, 2 for falling edge, 0 for nothing found
static int stv091x_carrier_search(struct stv *state,  s32* frequency_jump)
{
	//	bool signal_found = false;
	s32 agc_power_gain;
	s32 iq_power;
	s32 FinalFreq;
	u32 asperity=0;
	//stv091x_SIGNALTYPE_t signalType=FE_SAT_NOCARRIER;

	msleep(10);
	agc_power_gain = stv091x_agc1_power_gain(state);
	if(agc_power_gain==0) {
			/*if AGC1 integrator value is 0 then read POWERI, POWERQ registers*/
			/*Read the IQ power value*/
		iq_power = stv091x_iq_power(state);
	}

	FinalFreq = 0;

	if ( (agc_power_gain == 0) && (iq_power < 30)){
		/*no signal */
		/* Jump of distance of the bandwith tuner */
		//signalType = FE_SAT_TUNER_NOSIGNAL;
		asperity = 0;
	} else {

		/* Perform edge detection */
		asperity = stv091x_carrier_check(state, &FinalFreq, frequency_jump);

		if (asperity == 0) {
			/* No edge detected, jump tuner bandwidth */
			//signalType = FE_SAT_TUNER_NOSIGNAL;

			//TODO: falling edge detected or direct blind to be done:
			//dprintk("unimplemented?\n");
			dprintk("Rien dans la bande, saut de BW/2;  asperity=%d jump=%d\n", asperity, *frequency_jump);
		} else { // asperity==1; rising edge detected, perform blind search at next step */
			//signalType = FE_SAT_TUNER_JUMP;
			dprintk("Saut 4 or more steps; asperity=%d jump=%d\n", asperity, *frequency_jump);

		}
	}
	state->scan_next_frequency += *frequency_jump;
	state->search_range_hz = 24000000 + *frequency_jump *1000;
	if (state->search_range_hz >40000000)
		state->search_range_hz = 40000000;

	write_reg(state, RSTV0910_P2_DMDISTATE,0x5C); /* Demod Stop*/

	return asperity;
}

static int stv091x_tune_once(struct dvb_frontend *fe, bool* need_retune);

/*returns true 1 if a rising edge in spectrum was found, 2 for falling edge, 0 for nothing found
 *locked_ret indicates if a asignal could be locked
 */
static int scan_within_tuner_bw(struct dvb_frontend *fe, bool* locked_ret)
{
	//bool rising_edge_found = false;
	bool locked=false;
	int asperity = 0;
	s32 frequency_jump=0;
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	BUG_ON(!state->satellite_scan);
	state->signal_info.has_timedout=false;
	//u32 IF;
	state->signal_info.fec_locked = 0;
	state->signal_info.demod_locked = 0;
	Stop(state);


	if (state->symbol_rate < 100000 || state->symbol_rate > 70000000)
		return -EINVAL;
	stv091x_compute_timeouts(&state->DemodTimeout, &state->FecTimeout, state->symbol_rate, p->algorithm);
	stv091x_set_frequency_symbol_rate_bandwidth(state); //bw set to max. symbol_rate set to 1MHz


	state->ReceiveMode = Mode_None;
	state->DemodLockTime = 0;

#if 0
	switch(p->algorithm) {
	case ALGORITHM_SEARCH_NEXT:
		break;
	default:
		dprintk("This function should not be called with algorithm=%d\n", p->algorithm);
		break;
	}
#endif

	asperity = stv091x_carrier_search(state, &frequency_jump);
	dprintk("asperity=%d frequency=%dkHz jump=%dkHz\n", asperity, p->frequency, frequency_jump);
	if(asperity==1 ||asperity==2) { //rising or falling edge found
		p->frequency += frequency_jump;

		dprintk("BEFORE state->srate=%d p->srate=%d p->search_range=%d state->search_range=%d\n",
						state->symbol_rate/1000, p->symbol_rate/1000, p->max_symbol_rate/1000, state->search_range_hz/1000);
		state->Started = 1;
		{ bool need_retune;
			int ret;
			int old = p->algorithm;
			p->algorithm = ALGORITHM_BLIND;
			ret = stv091x_tune_once(fe, &need_retune);
			dprintk("tune_once returned stat=%d\n", ret);
			p->algorithm = old;
			locked= !state->signal_info.has_timedout;
		}
		dprintk("AFTER state->srate=%d p->srate=%d p->search_range=%d state->search_range=%d\n",
						state->symbol_rate/1000, p->symbol_rate/1000, p->max_symbol_rate/1000, state->search_range_hz/1000);

		if(locked) {
			//stv091x_get_signal_info(fe); already done in tune_once
		} else {
			state->Started = 0;
		}
	} else {
		p->frequency += ((state->tuner_bw / 3000) - 1000);
		dprintk("frequency=%d\n", p->frequency);
	}
	*locked_ret = locked;
	return asperity;
}


static int stv091x_tune_once(struct dvb_frontend *fe, bool* need_retune)
{
	struct stv* state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct stv_signal_info* signal_info = &state->signal_info;
	bool locked;
	memset(signal_info, 0, sizeof(*signal_info));
	dprintk("[%d] delivery_system=%u modulation=%u frequency=%u symbol_rate=%u inversion=%u stream_id=%d\n",
					state->adapterno,
					p->delivery_system, p->modulation, p->frequency,
					p->symbol_rate, p->inversion, p->stream_id);

	state->signal_info.has_timedout=false;
	*need_retune = 0;

	Stop(state);

	stv091x_compute_timeouts(&state->DemodTimeout, &state->FecTimeout, state->symbol_rate, p->algorithm);

	dprintk("now symbol_rate=%d\n", state->symbol_rate);
	stv091x_set_frequency_symbol_rate_bandwidth(state);
	state->ReceiveMode = Mode_None;
	state->DemodLockTime = 0;

	BUG_ON(state->Started);

	switch(p->algorithm) {
	case ALGORITHM_WARM:
	case ALGORITHM_COLD:
	case ALGORITHM_BLIND:
	case ALGORITHM_BLIND_BEST_GUESS:
	stv091x_set_search_standard(state, p);
	dprintk("now symbol_rate=%d\n", state->symbol_rate);
	if(p->algorithm == ALGORITHM_WARM || p->algorithm == ALGORITHM_COLD ||
		 p->algorithm == ALGORITHM_BLIND_BEST_GUESS)
		dprintk("SET_STREAM_INDEX: %d\n", p->stream_id);
	stv091x_start_scan(state, p);
	break;
	default:
		dprintk("This function should not be called with algorithm=%d\n", p->algorithm);
		break;
	}
	dprintk("Setting stream_id filter: stream_id=%d", state->demod_search_stream_id);
	set_stream_index(state, state->demod_search_stream_id);
	state->Started = 1;

	locked = wait_for_dmdlock(fe, 1 /*require_data*/);

	dprintk("lock=%d timedout=%d\n", locked, state->signal_info.has_timedout);
	if(!locked && p->algorithm != ALGORITHM_WARM && p->algorithm != ALGORITHM_COLD) {
		if(!locked)
			locked = pls_search_list(fe);
		if(!locked)
			locked = pls_search_range(fe);
	} else {
		state->signal_info.has_lock=true;
		set_stream_index(state, p->stream_id);
	}

	state->signal_info.has_timedout = !state->signal_info.has_viterbi;
	if(locked) {
		/*
			retrieve information about modulation, frequency, symbol_rate
			but not CNR, BER (different from stid135)

			returns 1 if tuner frequency or bandwidth needs to be updated
		*/

		*need_retune = stv091x_get_signal_info(fe);
	}
		vprintk("[%d] set_parameters: locked=%d vit=%d sync=%d timeout=%d\n",
					state->adapterno,
					state->signal_info.has_lock,
					state->signal_info.has_viterbi,					state->signal_info.has_sync,
					state->signal_info.has_timedout);

	state->DemodLockTime += TUNING_DELAY;

	dprintk("setting timedout=%d\n", !locked);

#if 0
	if(locked && p->algorithm != ALGORITHM_WARM ) {
		*need_retune = stv091x_get_signal_info(fe);
	}
#endif
	return 0;
}


static int stv091x_constellation_start(struct dvb_frontend *fe,
																			 struct dtv_fe_constellation* user, int max_num_samples);

static int tune(struct dvb_frontend *fe, bool re_tune,
								unsigned int mode_flags, unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int r;
	bool need_retune = false;// could be set during blind search

	bool blind = (p->algorithm == ALGORITHM_BLIND ||p->algorithm == ALGORITHM_BLIND_BEST_GUESS);
	if(blind) {
		if(p->delivery_system == SYS_UNDEFINED)
			p->delivery_system = SYS_AUTO;
	}

	state->symbol_rate =
		(p->symbol_rate==0) ? 60000000: p->symbol_rate; //determines tuner bandwidth set during blindscan


	state->satellite_scan = false;

	if (re_tune) {
		dprintk("tune called with freq=%d srate=%d => %d re_tune=%d\n", p->frequency, p->symbol_rate,
						state->symbol_rate, re_tune);
		stv091x_stop_task(fe);
		stv091x_tune_once(fe, &need_retune);
#if 0
		/*the following does not seem to work for a DVB-S2/QPSK transponder: 28.2E 11385H.
			This TP also has a stream_id and m_type set
			In this case read_status seems to fail; something extra is needed.
		 */
		if(need_retune) {
			dprintk("re setting frequency, symbol rate and bandwidth\n");
			stv091x_set_frequency_symbol_rate_bandwidth(state);
		}
#endif
		state->tune_time = jiffies;
	}

	stv091x_get_signal_info(fe);

	/*
		read rf level, snr, signal quality, lock_status
	*/
	r = read_status(fe, status);

	{
		int max_num_samples = state->symbol_rate /5 ; //we spend max 500 ms on this
		if(max_num_samples > 1024)
			max_num_samples = 1024; //also set an upper limit which should be fast enough
		stv091x_constellation_start(fe, &p->constellation, max_num_samples);
	}
	if (r)
		return r;

	if (*status & FE_HAS_LOCK) {
		return 0;
	} else
		*delay = HZ;
	return 0;
}

/*
	init = 1: start the scan at the search range specified by the user
	init = 0: start the scan just beyond the last found frequency
 */
static int stv091x_sat_scan(struct dvb_frontend *fe, bool init,
														unsigned int *delay,  enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int asperity;
	bool locked = false;
	if(init) {
		stv091x_stop_task(fe);
		//clean up any left over spectrum
	}
	state->Started = 0;
	state->satellite_scan = true;
	//todo: 	MinSymbolRate
	state->search_range_hz =  500000;
	state->symbol_rate =  1000000;							/*	minimum symbol rate for scan = 1Msps	*/
	dprintk("set symbol_rate=%d\n", state->symbol_rate);

	if(init) {
		state->scan_next_frequency = p->scan_start_frequency;
		state->scan_end_frequency = p->scan_end_frequency;
	}

	while(state->scan_next_frequency < state->scan_end_frequency) {
		*status = 0;
		if (kthread_should_stop() || dvb_frontend_task_should_stop(fe)) {
			dprintk("exiting on should stop\n");
			*status =0;
			return 0;
		}
		p->frequency = state->scan_next_frequency;
		dprintk("start=%d end=%d init=%d\n",
						state->scan_next_frequency, state->scan_end_frequency,  init);

		if(p->frequency >= state->scan_end_frequency) {
			dprintk("Invalid ranged");
			return -1;
		}
#if 0
		p->algorithm = ALGORITHM_SEARCH_NEXT;
#endif
		asperity = scan_within_tuner_bw(fe, &locked);
		dprintk("asperity=%d locked=%d freq=%dkHz start_freq=%dkHz end_freq=%dkHz\n", asperity, locked,
						p->frequency/1000, state->scan_next_frequency/1000, state->scan_end_frequency);
		if(locked) {
			//state->scan_end_frequency = p->frequency;
			uint32_t step = 135*((state->symbol_rate/1000) / (2*100));
			uint32_t next = p->frequency + step;
			dprintk("symbol_rate=%d step=%d next-%d\n", state->symbol_rate, step, next);
			if(next < state->scan_next_frequency) {
				next += state->search_range_hz/1000;
				dprintk ("Adjusting next to prevent going backwards: new next=%d\n", next);
			}
			state->tune_time = jiffies;

			state->scan_next_frequency = next;

			*status = FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
			*delay = HZ;

			return 0;
		} else {
			*status = 0;
		}
	}
	if(state->scan_next_frequency >= state->scan_end_frequency) {
		dprintk("Signaling DONE\n");
		*status =  FE_TIMEDOUT|FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	}
	return 0;
}


static enum dvbfe_algo get_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int wait_dis(struct stv *state, u8 flag, u8 val)
{
	int i;
	u8 stat;
	u16 offs = state->adapterno ? 0x40 : 0;

	for (i = 0; i < 10; i++) {
		stat = read_reg(state, RSTV0910_P1_DISTXSTATUS + offs);
		if ((stat & flag) == val)
			return 0;
		msleep(10);
	}
	return -1;
}



static int set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct stv *state = fe->demodulator_priv;
	u16 offs = state->adapterno ? 0x40 : 0;

	switch (tone) {
	case SEC_TONE_ON:
	 write_reg(state, RSTV0910_P1_DISTXCFG + offs, 0x00);
		write_reg(state, RSTV0910_P1_DISTXCFG + offs, 0x80);
		return write_reg(state, RSTV0910_P1_DISTXCFG + offs, 0x00);
	case SEC_TONE_OFF:
		return write_reg(state, RSTV0910_P1_DISTXCFG + offs, 0x80);
	default:
		return -EINVAL;
	}
}

static int send_master_cmd(struct dvb_frontend *fe,
			 struct dvb_diseqc_master_cmd *cmd)
{
	struct stv *state = fe->demodulator_priv;
	u16 offs = state->adapterno ? 0x40 : 0;
	int i;

	write_reg(state, RSTV0910_P1_DISTXCFG + offs, 0x06);
	for (i = 0; i < cmd->msg_len; i++) {
		wait_dis(state, 0x40, 0x00);
		write_reg(state, RSTV0910_P1_DISTXFIFO + offs, cmd->msg[i]);
	}
	write_reg(state, RSTV0910_P1_DISTXCFG + offs, 0x02);
	wait_dis(state, 0x20, 0x20);
	return 0;
}

static int recv_slave_reply(struct dvb_frontend *fe,
			 struct dvb_diseqc_slave_reply *reply)
{
	return 0;
}

static int send_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
	struct stv *state = fe->demodulator_priv;
	u16 offs = state->adapterno ? 0x40 : 0;
	u8 value;

	if (burst == SEC_MINI_A) {
		write_reg(state, RSTV0910_P1_DISTXCFG + offs, 0x07);
		value = 0x00;
	} else {
		write_reg(state, RSTV0910_P1_DISTXCFG + offs, 0x06);
		value = 0xFF;
	}
	wait_dis(state, 0x40, 0x00);
	write_reg(state, RSTV0910_P1_DISTXFIFO + offs, value);
	write_reg(state, RSTV0910_P1_DISTXCFG + offs, 0x02);
	wait_dis(state, 0x20, 0x20);

	return 0;
}

static int sleep(struct dvb_frontend *fe)
{
	struct stv *state = fe->demodulator_priv;

	Stop(state);
	return 0;
}

static int read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*strength = 0;
	for (i=0; i < p->strength.len; i++)
	{
		if (p->strength.stat[i].scale == FE_SCALE_RELATIVE)
			*strength = (u16)p->strength.stat[i].uvalue;
		else if (p->strength.stat[i].scale == FE_SCALE_DECIBEL)
			*strength = ((100000 + (s32)p->strength.stat[i].svalue)/1000) * 656;
	}

	return 0;
}

static int read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i;

	*snr = 0;
	for (i=0; i < p->cnr.len; i++)
		if (p->cnr.stat[i].scale == FE_SCALE_RELATIVE)
		 *snr = (u16)p->cnr.stat[i].uvalue;
	return 0;
}

static int read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	if ( p->post_bit_error.stat[0].scale == FE_SCALE_COUNTER &&
		p->post_bit_count.stat[0].scale == FE_SCALE_COUNTER )
	 *ber = (u32)p->post_bit_count.stat[0].uvalue ? (u32)p->post_bit_error.stat[0].uvalue / (u32)p->post_bit_count.stat[0].uvalue : 0;

	return 0;
}

static int read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

static void spi_read(struct dvb_frontend *fe, struct ecp3_info *ecp3inf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;

	state->base->read_properties(adapter,ecp3inf->reg, &(ecp3inf->data));
	/*pr_info("stv0910: spi_read %x=%x\n",ecp3inf->reg,ecp3inf->data);*/
	return;
}

static void spi_write(struct dvb_frontend *fe,struct ecp3_info *ecp3inf)
{
	struct stv *state = fe->demodulator_priv;
	struct i2c_adapter *adapter = state->base->i2c;
	/*printk("stv0910: spi_write %x == %x\n",ecp3inf->reg, ecp3inf->data);*/
	state->base->write_properties(adapter,ecp3inf->reg, ecp3inf->data);
	return;
}



static int stv091x_spectrum_start(struct dvb_frontend *fe,
																	struct dtv_fe_spectrum* s,
																	unsigned int *delay, enum fe_status *status)
{
	struct stv *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct stv_spectrum_scan_state* ss = &state->scan_state;

	int i;
	u32 start_frequency = p->scan_start_frequency;
	u32 end_frequency = p->scan_end_frequency;
	//u32 bandwidth = end_frequency-start_frequency; //in kHz
	uint32_t frequency;
	int val1, val2, val3;
	uint32_t resolution =  (p->scan_resolution>0) ? p->scan_resolution : 500; //in kHz
	uint32_t bandwidth =  2*resolution; //in kHz
	u32 num_freq = (p->scan_end_frequency-p->scan_start_frequency+ resolution-1)/resolution;
	stv091x_stop_task(fe);
	s->num_freq = num_freq;
	s->num_candidates = 0;
	ss->spectrum_len = num_freq;
	ss->freq = kzalloc(ss->spectrum_len * (sizeof(ss->freq[0])), GFP_KERNEL);
	ss->spectrum = kzalloc(ss->spectrum_len * (sizeof(ss->spectrum[0])), GFP_KERNEL);
	if (!ss->freq || !ss->spectrum) {
		return  -ENOMEM;
	}

	state->tuner_bw = stv091x_bandwidth(ROLLOFF_AUTO, bandwidth);
	s->scale =  FE_SCALE_DECIBEL; //in units of 0.001dB

	dprintk("demod: %d: range=[%d,%d]kHz num_freq=%d resolution=%dkHz bw=%dkHz\n", state->adapterno,
					start_frequency, end_frequency,
					num_freq, resolution, bandwidth/1000);

	write_reg(state, RSTV0910_P2_CFRUP1 + state->regoff, 0);
	write_reg(state, RSTV0910_P2_CFRUP0 + state->regoff, 0);
	/*CFR Low Setting*/

	write_reg(state, RSTV0910_P2_DMDISTATE, 0x18); //warm start

	//write_reg(state, RSTV0910_P2_AGC2O + state->regoff, 0x5B); //reset
#if 0
	reg = read_reg(state, RSTV0910_P2_AGC1CN + state->regoff);
	reg = (reg & ~0x7) | 1; //slows decrease of agciq_beta
	write_reg(state, RSTV0910_P2_AGC1CN, reg);
#endif

	write_reg(state, RSTV0910_P2_DMDISTATE, 0x5C); /* Demod Stop */

#if 0
	reg = read_reg(state, RSTV0910_P2_DMDCFGMD + state->regoff);
	reg &= ~0x18; //disable symbol rate scanning, and disable autoscan
	write_reg(state, RSTV0910_P2_DMDCFGMD + state->regoff, reg);
#endif

	//TODO: reset hardware

	///	write_reg(state, RSTV0910_P2_DMDISTATE, 0x1F); //reset demod

	//write_reg(state, RSTV0910_P2_AGC2REF + state->regoff, 0x38); //reset

	stv091x_set_symbol_rate(state, resolution*1000);

	write_reg(state, RSTV0910_P2_DMDISTATE, 0x5C); //stop demod
#ifdef TEST
#else
		write_reg(state, RSTV0910_P2_DMDISTATE, 0x1C); //stop demod
		write_reg(state, RSTV0910_P2_DMDISTATE, 0x18); //warm
#endif

	for (i = 0; i < num_freq; i++) {
		if ((i% 20==19) &&  (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			dprintk("exiting on should stop\n");
			break;
		}
#ifdef TEST
		write_reg(state, RSTV0910_P2_DMDISTATE, 0x1C); //stop demod
		write_reg(state, RSTV0910_P2_DMDISTATE, 0x18); //warm
#endif

		ss->freq[i]= start_frequency + i*resolution;
		frequency = ss->freq[i];

		if(fe->ops.tuner_ops.set_frequency_and_bandwidth)
			fe->ops.tuner_ops.set_frequency_and_bandwidth(fe, frequency, bandwidth); //todo: check that this sets the proper bandwidth

		//usleep_range(12000, 13000);
		if (i == 0) {
			val1 = stv091x_narrow_band_signal_power_dbm(fe);
			val2 = val3 = val1;
			continue;
		}
		val3 = val2;
		val2 = val1;
		val1 = stv091x_narrow_band_signal_power_dbm(fe);
		if(i>=1)
			ss->spectrum[i-1]  = (val1 + val2 + val3)/3;
	}
	if(num_freq>0)
		ss->spectrum[num_freq-1]  = (2*val1 + val2)/3;

	dprintk("Ending spectrum_scan num_freq=%d\n", num_freq);
	*status =  FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	return 0;
}

static int stv091x_constellation_start(struct dvb_frontend *fe,
																			 struct dtv_fe_constellation* user, int max_num_samples)
{
	struct stv *state = fe->demodulator_priv;
	struct stv_constellation_scan_state* cs = &state->constellation_scan_state;
	int num_samples = user->num_samples;
	s8 buff[2];
	if(num_samples > max_num_samples)
		num_samples = max_num_samples;


	stv091x_stop_task(fe);
	dprintk("demod: %d: constellation num_samples: req=%d max=%d  mode=%d\n", state->adapterno, user->num_samples, max_num_samples, (int)cs->constel_select);

	if(num_samples ==0) {
		return -EINVAL;
	}
	if(cs->samples_len != num_samples) {
		if(cs->samples)
			kfree(cs->samples);
		cs->samples_len = num_samples;
		cs->samples = kzalloc(cs->samples_len * (sizeof(cs->samples[0])), GFP_KERNEL);
		if (!cs->samples) {
			return  -ENOMEM;
		}
	}

	cs->num_samples = 0;
	cs->constel_select =  user->constel_select;

	write_reg_fields(state, RSTV0910_P2_IQCONST,
									 {FSTV0910_P2_CONSTEL_SELECT, 0}, //inverse mode
									 {FSTV0910_P2_IQSYMB_SEL, cs->constel_select}
									 );
	//	write_reg(state, RSTV0910_P2_AGC2REF,0x10);
	for (cs->num_samples = 0; cs->num_samples < cs->samples_len; ++cs->num_samples) {
		if ((cs->num_samples% 20==19) &&  (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			dprintk("exiting on should stop\n");
			break;
		}

		read_regs(state, RSTV0910_P2_ISYMB, buff, 2);
		cs->samples[cs->num_samples].imag = buff[0];
		cs->samples[cs->num_samples].real = buff[1];
	}
	return 0;
}


static int stv091x_stop_task(struct dvb_frontend *fe)
{
	struct stv* state = fe->demodulator_priv;
	struct stv_spectrum_scan_state* ss = &state->scan_state;
	struct stv_constellation_scan_state* cs = &state->constellation_scan_state;
	if(ss->freq)
		kfree(ss->freq);
	if(ss->spectrum)
		kfree(ss->spectrum);
	if(cs->samples)
		kfree(cs->samples);
	memset(ss, 0, sizeof(*ss));
	memset(cs, 0, sizeof(*cs));
	dprintk("Freed memory\n");
	return 0;
}

int stv091x_spectrum_get(struct dvb_frontend *fe, struct dtv_fe_spectrum* user)
{
	struct stv *state = fe->demodulator_priv;
	int error=0;
	dprintk("num_freq= %d %d\n", user->num_freq ,  state->scan_state.spectrum_len);
	if(user->num_freq > state->scan_state.spectrum_len)
		user->num_freq = state->scan_state.spectrum_len;
	if(state->scan_state.freq && state->scan_state.spectrum) {
	if (copy_to_user((void __user*) user->freq, state->scan_state.freq, user->num_freq * sizeof(__u32))) {
			error = -EFAULT;
		}
		if (copy_to_user((void __user*) user->rf_level, state->scan_state.spectrum, user->num_freq * sizeof(__s32))) {
			error = -EFAULT;
		}
	}
	else
		error = -EFAULT;
	user->num_candidates = 0;
	return error;
}


static int stv091x_constellation_get(struct dvb_frontend *fe, struct dtv_fe_constellation* user)
{
	struct stv *state = fe->demodulator_priv;
	struct stv_constellation_scan_state* cs = &state->constellation_scan_state;
	int error = 0;
	dprintk("demod: %d: constellation num_samples=%d/%d\n", state->adapterno, cs->num_samples, user->num_samples);
	if(user->num_samples > cs->num_samples)
		user->num_samples = cs->num_samples;
	if(cs->samples) {
		if (copy_to_user((void __user*) user->samples, cs->samples,
										 user->num_samples * sizeof(cs->samples[0]))) {
			error = -EFAULT;
		}

	}
	else
		error = -EFAULT;
	return error;
}


static struct dvb_frontend_ops stv091x_ops = {
	.delsys = { SYS_DVBS, SYS_DVBS2, SYS_DSS, SYS_AUTO },
	.info = {
		.name			= "STV091x Multistandard",
		.frequency_min_hz	 = 950 * MHz,
		.frequency_max_hz 	= 2150 * MHz,
		.symbol_rate_min	= 100000,
		.symbol_rate_max	= 70000000,
		.caps			= FE_CAN_INVERSION_AUTO |
					 FE_CAN_FEC_AUTO |
					 FE_CAN_QPSK |
					 FE_CAN_2G_MODULATION |
		       FE_CAN_MULTISTREAM,
		.extended_caps = FE_CAN_SPECTRUMSCAN	| FE_CAN_IQ | FE_CAN_BLINDSEARCH
	},
	.init				= init,
	.sleep				= sleep,
	.release = release,
	.i2c_gate_ctrl = gate_ctrl,
	.get_frontend_algo = get_algo,
	.get_frontend = get_frontend,
	.tune = tune,
	.set_tone			= set_tone,

	.diseqc_send_master_cmd		= send_master_cmd,
	.diseqc_send_burst		= send_burst,
	.diseqc_recv_slave_reply	= recv_slave_reply,

	.read_status			= read_status,
	.read_signal_strength		= read_signal_strength,
	.read_snr			= read_snr,
	.read_ber			= read_ber,
	.read_ucblocks			= read_ucblocks,
	.spi_read			= spi_read,
	.spi_write			= spi_write,
	.scan =  stv091x_sat_scan,
	.stop_task = stv091x_stop_task,
	.spectrum_start = stv091x_spectrum_start,
	.spectrum_get = stv091x_spectrum_get,
	.constellation_get	= stv091x_constellation_get,
};

static struct stv_base *match_base(struct i2c_adapter *i2c, u8 adr)
{
	struct stv_base *p;

	list_for_each_entry(p, &stvlist, stvlist)
		if (p->i2c == i2c && p->adr == adr)
			return p;
	return NULL;
}

/*
	adapterno is the index of the adapter on a card, starting at 0
 */
struct dvb_frontend *stv091x_attach(struct i2c_adapter *i2c,
																		struct stv091x_cfg *cfg, int adapterno)
{
	struct stv* state;
	struct stv_base* base;

	state = kzalloc(sizeof(struct stv), GFP_KERNEL);
	if (!state)
		return NULL;

	state->tscfgh = 0x20 | (cfg->parallel ? 0 : 0x40);
	state->tsgeneral = (cfg->parallel == 2) ? 0x02 : 0x00;
	state->i2crpt = 0x0A | ((cfg->rptlvl & 0x07) << 4);
	state->adapterno = adapterno;
	state->regoff = state->adapterno ? 0 : 0x200;
	state->search_range_hz = 16000000;
	state->DEMOD = 0x10; /* Inversion : Auto with reset to 0 */
	state->ReceiveMode = Mode_None;
	state->Started = 0;
	state->FirstTimeLock = 0;

	base = match_base(i2c, cfg->adr);
	if (base) {
		base->count++;
		state->base = base;
	} else {
		base = kzalloc(sizeof(struct stv_base), GFP_KERNEL);
		if (!base)
			goto fail;
		base->i2c = i2c;
		base->adr = cfg->adr;
		base->count = 1;
		base->extclk = cfg->clk ? cfg->clk : 30000000;
		base->dual_tuner = cfg->dual_tuner;
		base->set_lock_led = cfg->set_lock_led;
		base->write_properties = cfg->write_properties;
		base->read_properties = cfg->read_properties;

		mutex_init(&base->i2c_lock);
		mutex_init(&base->reg_lock);
		state->base = base;
		if (stv091x_probe(state) < 0) {
			kfree(base);
			goto fail;
		}
		list_add(&base->stvlist, &stvlist);
	}
	state->fe.ops = stv091x_ops;
	state->fe.demodulator_priv = state;
	state->adapterno = adapterno;

	return &state->fe;

fail:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL_GPL(stv091x_attach);

MODULE_DESCRIPTION("STV091x driver");
MODULE_AUTHOR("Ralph Metzler, Manfred Voelkel, Deep Thought");
MODULE_LICENSE("GPL");

/*
 P2_MANUAL_CFRINC: Method of calculating cfr_inc (the step increment used in carrier
searches):
0: automatic mode, see cfrinc_mode[1:0] below.
1: manual mode. Value set in cfr_inc[13:3]. (unsigned)
<===this is what we need


Px_AUTO_GUP: automatic calculation mode
1: automatic calculation based on SFRUPRATIO,
px_symb_freq_up below is read-only.
0: manual mode, px_symb_freq_up is read/write. (unsigned)

 Coarse acquisition: The coarse (and fine) search range may be specified in absolute
frequency (manual mode) as defined in P1_SFRUPx and P2_SFRUPx (upper bound)
and P1_SFRLOWx and P2_SFRLOWx(lower bound). Alternatively, SFRUP and
SFRLOW may be calculated (automatic mode) from P1_SFRINITx, P2_SFRINITx,
Px_SFRUPRATIO and Px_SFRLOWRATIO. The scan mode (automatic/manual) is set
in bit 7 of Px_SFRUP1 and Px_SFRLOW1. The boundary conditions depend on the
=> incorrect

analogue_gain = tuner gain (stv6120)

analogue power in band
= log10 ( (agc2ref/constant)^2) - log10(agc2^2)) - analogue_gain_dbm + cst

power_channel analogue power * GvDig^2
digital_gain GDig is proportional to  agc2?

	************ Power Channel ************
		 PchAGC2 = agc2ref^2: power after AGC2 amplification?
		 Pch(dBm) = 10xlog( PchAGC2 / GvDig^2) - GainAnalog
		 GvDig: digital gain?
		 GvDig= ((agc2/165.8) * (1550/12)); for demod 1,3
		 log(a/b) = log(a) - log(b)
		 10xlog( PchAGC2 / GvDig^2) =  10xlog( PchAGC2)-10xlog(GvDig^2)

To check: d Px_VAVSRVIT .HYPVIT (faster lock)
 TSFIFO_MANSPEED

*/
