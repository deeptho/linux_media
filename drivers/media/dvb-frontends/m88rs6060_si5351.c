/*
 * Montage Technology M88rs6060 demodulator and tuner drivers
 * some code form m88ds3103
 * Copyright (c) 2021 Davin zhang <Davin@tbsdtv.com> www.Turbosight.com
 *
 *
 * Copyright (C) 2022 Deep Thought <deeptho@gmail.com>: blindscan, spectrum, constellation code
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
#include "m88rs6060_priv.h"
#include "si5351_priv.h"

//for CI - clock generator
static int si5351_write(struct m88rs6060_state *dev,u8 reg,u8 data)
{
	struct i2c_client *client = dev->demod_client;
	u8 buf[] = { reg, data };
	int ret;
	struct i2c_msg msg = {
		.addr = SI5351_BUS_BASE_ADDR/2,.flags = 0,.buf = buf,.len = 2
	};

	ret = i2c_transfer(dev->tuner_client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev,
			"si5351(ret=%i, reg=0x%02x, value=0x%02x)\n",
			 ret, reg, data);
		return -EREMOTEIO;
	}
	return 0;
}

static int si5351_write_bulk(struct m88rs6060_state *dev,u8 reg, u8 len,u8*data)

{
	struct i2c_client *client = dev->demod_client;
	u8 buf[80];
	int ret;
	struct i2c_msg msg = {
		.addr = SI5351_BUS_BASE_ADDR/2,.flags = 0,.buf = buf,.len = len+1
	};

	buf[0] = reg;
	 memcpy(&buf[1],data,len);
	 ret = i2c_transfer(dev->demod_client->adapter, &msg, 1);
	 if (ret != 1) {
		dev_err(&client->dev,
			"si5351(ret=%i, reg=0x%02x, value=0x%02x)\n",
			 ret, reg, *data);
		return -EREMOTEIO;
		}
	 return 0;
}

static u8 si5351_read(struct m88rs6060_state *dev,u8 reg,u8 *data)

{
	struct i2c_client *client = dev->demod_client;
	int ret;
	u8 b0[] = { reg };
	struct i2c_msg msg[] = {
		{
		 .addr = SI5351_BUS_BASE_ADDR/2,
		 .flags = 0,
		 .buf = b0,
		 .len = 1},
		{
		 .addr = SI5351_BUS_BASE_ADDR,
		 .flags = I2C_M_RD,
		 .buf = data,
		 .len = 1}
	};

	ret = i2c_transfer(dev->demod_client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "si5351 (ret=%d, reg=0x%02x)\n",
			 ret, reg);
		return -EREMOTEIO;
	}

	dev_dbg(&client->dev, "si5351 reg 0x%02x, value 0x%02x\n",
					reg, *data);
	return 0;
}

/*
 * Calculate best rational approximation for a given fraction
 * taking into account restricted register size, e.g. to find
 * appropriate values for a pll with 5 bit denominator and
 * 8 bit numerator register fields, trying to set up with a
 * frequency ratio of 3.1415, one would say:
 *
 * rational_best_approximation(31415, 10000,
 *              (1 << 8) - 1, (1 << 5) - 1, &n, &d);
 *
 * you may look at given_numerator as a fixed point number,
 * with the fractional part size described in given_denominator.
 *
 * for theoretical background, see:
 * http://en.wikipedia.org/wiki/Continued_fraction
 */
static void rational_best_approximation(
        unsigned long given_numerator, unsigned long given_denominator,
        unsigned long max_numerator, unsigned long max_denominator,
        unsigned long *best_numerator, unsigned long *best_denominator)
{

	unsigned long n, d, n0, d0, n1, d1;
	n = given_numerator;
	d = given_denominator;
	n0 = d1 = 0;
	n1 = d0 = 1;
	for (;;) {
		unsigned long t, a;
		if ((n1 > max_numerator) || (d1 > max_denominator)) {
			n1 = n0;
			d1 = d0;
			break;
		}
		if (d == 0)
			break;

		t = d;
		a = n / d;
		d = n % d;
		n = t;
		t = n0 + a * n1;
		n0 = n1;
		n1 = t;
		t = d0 + a * d1;
		d0 = d1;
		d1 = t;
	}

	*best_numerator = n1;
	*best_denominator = d1;
}

static u32 pll_calc(u32 freq, struct Si5351RegSet *reg, int correction)

{

	u32 ref_freq = SI5351_XTAL_FREQ;
	u32 rfrac, denom, a,  p1, p2, p3;
	unsigned long b, c;
	u64 lltmp;

	/* Factor calibration value into nominal crystal frequency */
	/* Measured in parts-per-ten million */
	ref_freq += (u32)((correction / 10000000) * ref_freq);

	/* PLL bounds checking */
	if (freq < SI5351_PLL_VCO_MIN)
		freq = SI5351_PLL_VCO_MIN;
	if (freq > SI5351_PLL_VCO_MAX)
		freq = SI5351_PLL_VCO_MAX;

	/* Determine integer part of feedback equation */
	a = freq / ref_freq;
	if (a < SI5351_PLL_A_MIN)
		freq = ref_freq * SI5351_PLL_A_MIN;
	if (a > SI5351_PLL_A_MAX)
		freq = ref_freq * SI5351_PLL_A_MAX;

	/* find best approximation for b/c = fVCO mod fIN */
	denom = 1000L * 1000L;
	lltmp = freq % ref_freq;
	lltmp *= denom;
	do_div(lltmp, ref_freq);
	rfrac = (u32)lltmp;

	b = 0;
	c = 1;
	if (rfrac)
		rational_best_approximation(rfrac, denom, \
				    SI5351_PLL_B_MAX, SI5351_PLL_C_MAX, &b, &c);

	/* calculate parameters */
	p3  = c;
	p2  = (128 * b) % c;
	p1  = 128 * a;
	p1 += (128 * b / c);
	p1 -= 512;

	/* recalculate rate by fIN * (a + b/c) */
	lltmp  = ref_freq;
	lltmp *= b;
	do_div(lltmp, c);

	freq  = (u32)lltmp;
	freq += ref_freq * a;

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;

	return freq;
}
/*
 * si5351_set_pll(uint32_t pll_freq, enum si5351_pll target_pll)
 *
 * Set the specified PLL to a specific oscillation frequency
 *
 * pll_freq - Desired PLL frequency
 * target_pll - Which PLL to set
 *     (use the si5351_pll enum)
 */
void si5351_set_pll(struct m88rs6060_state *dev,u32 pll_freq, enum si5351_pll target_pll)
{

	struct Si5351RegSet pll_reg;

	u8 params[30];
	u8 i = 0;
	u8 temp;

	pll_calc(pll_freq, &pll_reg, 0);
	/* Derive the register values to write */
	/* Prepare an array for parameters to be written to */

	/* Registers 26-27 */
	temp = ((pll_reg.p3 >> 8) & 0xff);
	params[i++] = temp;

	temp = (u8)(pll_reg.p3  & 0xff);
	params[i++] = temp;

	/* Register 28 */
	temp = (u8)((pll_reg.p1 >> 16) & 0x03);
	params[i++] = temp;

	/* Registers 29-30 */
	temp = (u8)((pll_reg.p1 >> 8) & 0xff);
	params[i++] = temp;

	temp = (u8)(pll_reg.p1  & 0xff);
	params[i++] = temp;

	/* Register 31 */
	temp = (u8)((pll_reg.p3 >> 12) & 0xf0);
	temp += (u8)((pll_reg.p2 >> 16) & 0x0f);
	params[i++] = temp;

	/* Registers 32-33 */
	temp = (u8)((pll_reg.p2 >> 8) & 0xff);
	params[i++] = temp;

	temp = (u8)(pll_reg.p2  & 0xff);
	params[i++] = temp;

	/* Write the parameters */
	if(target_pll == SI5351_PLLA)
	{
		si5351_write_bulk(dev,SI5351_PLLA_PARAMETERS, i + 1, params);
	}
	else if(target_pll == SI5351_PLLB)
	{
		si5351_write_bulk(dev,SI5351_PLLB_PARAMETERS, i + 1, params);
	}

}

/*
 * si5351_clock_enable(enum si5351_clock clk, uint8_t enable)
 *
 * Enable or disable a chosen clock
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 */
void si5351_clock_enable(struct m88rs6060_state *dev,enum si5351_clock clk, u8 enable)
{

	u8 reg_val;

	if(si5351_read(dev,SI5351_OUTPUT_ENABLE_CTRL, &reg_val) != 0)
	{
		return;
	}

	if(enable == 1)
	{
		reg_val &= ~(1<<(u8)clk);
	}
	else
	{
		reg_val |= (1<<(u8)clk);
	}

	si5351_write(dev,SI5351_OUTPUT_ENABLE_CTRL, reg_val);

}

/*
 * si5351_drive_strength(enum si5351_clock clk, enum si5351_drive drive)
 *
 * Sets the drive strength of the specified clock output
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * drive - Desired drive level
 *   (use the si5351_drive enum)
 */
void si5351_drive_strength(struct m88rs6060_state *dev, enum si5351_clock clk, enum si5351_drive drive)
{
	u8 reg_val;

	const u8 mask = 0x03;

	if(si5351_read(dev,SI5351_CLK0_CTRL + (u8)clk, &reg_val) != 0)
	{
		return;
	}

	switch(drive)
	{
	case SI5351_DRIVE_2MA:
		reg_val &= ~(mask);
		reg_val |= 0x00;
		break;
	case SI5351_DRIVE_4MA:
		reg_val &= ~(mask);
		reg_val |= 0x01;
		break;
	case SI5351_DRIVE_6MA:
		reg_val &= ~(mask);
		reg_val |= 0x02;
		break;
	case SI5351_DRIVE_8MA:
		reg_val &= ~(mask);
		reg_val |= 0x03;
		break;
	default:
		break;
	}

	si5351_write(dev,SI5351_CLK0_CTRL + (u8)clk, reg_val);
}


static u32 multisynth_calc(u32 freq, struct Si5351RegSet *reg)
{
	u32 pll_freq;
	u64 lltmp;
	u32 a, b, c, p1, p2, p3;
	u8 divby4;

	/* Multisynth bounds checking */
	if (freq > SI5351_MULTISYNTH_MAX_FREQ)
		freq = SI5351_MULTISYNTH_MAX_FREQ;
	if (freq < SI5351_MULTISYNTH_MIN_FREQ)
		freq = SI5351_MULTISYNTH_MIN_FREQ;

	divby4 = 0;
	if (freq > SI5351_MULTISYNTH_DIVBY4_FREQ)
		divby4 = 1;

	/* Find largest integer divider for max */
	/* VCO frequency and given target frequency */
	if (divby4 == 0)
	{
		lltmp = SI5351_PLL_VCO_MAX;
		do_div(lltmp, freq);
		a = (u32)lltmp;
	}
	else
		a = 4;

	b = 0;
	c = 1;
	pll_freq = a * freq;

	/* Recalculate output frequency by fOUT = fIN / (a + b/c) */
	lltmp  = pll_freq;
	lltmp *= c;
	do_div(lltmp, a * c + b);
	freq  = (unsigned long)lltmp;

	/* Calculate parameters */
	if (divby4)
	{
		p3 = 1;
		p2 = 0;
		p1 = 0;
	}
	else
	{
		p3  = c;
		p2  = (128 * b) % c;
		p1  = 128 * a;
		p1 += (128 * b / c);
		p1 -= 512;
	}

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;

	return pll_freq;

}

static u32 multisynth_recalc(u32 freq, u32 pll_freq, struct Si5351RegSet *reg)
{

	u64 lltmp;
	u32 rfrac, denom, a, p1, p2, p3;
	unsigned long b,c;
	u8 divby4;

	/* Multisynth bounds checking */
	if (freq > SI5351_MULTISYNTH_MAX_FREQ)
		freq = SI5351_MULTISYNTH_MAX_FREQ;
	if (freq < SI5351_MULTISYNTH_MIN_FREQ)
		freq = SI5351_MULTISYNTH_MIN_FREQ;

	divby4 = 0;
	if (freq > SI5351_MULTISYNTH_DIVBY4_FREQ)
		divby4 = 1;

	/* Determine integer part of feedback equation */
	a = pll_freq / freq;
	/* TODO: not sure this is correct */
	if (a < SI5351_MULTISYNTH_A_MIN)
		freq = pll_freq / SI5351_MULTISYNTH_A_MIN;
	if (a > SI5351_MULTISYNTH_A_MAX)
		freq = pll_freq / SI5351_MULTISYNTH_A_MAX;

	/* find best approximation for b/c */
	denom = 1000L * 1000L;
	lltmp = pll_freq % freq;
	lltmp *= denom;
	do_div(lltmp, freq);
	rfrac = (u32)lltmp;

	b = 0;
	c = 1;
	if (rfrac)
		rational_best_approximation(rfrac, denom,   \
				    SI5351_MULTISYNTH_B_MAX, SI5351_MULTISYNTH_C_MAX, &b, &c);

	/* Recalculate output frequency by fOUT = fIN / (a + b/c) */
	lltmp  = pll_freq;
	lltmp *= c;
	do_div(lltmp, a * c + b);
	freq  = (unsigned long)lltmp;

	/* Calculate parameters */
	if (divby4)
	{
		p3 = 1;
		p2 = 0;
		p1 = 0;
	}
	else
	{
		p3  = c;
		p2  = (128 * b) % c;
		p1  = 128 * a;
		p1 += (128 * b / c);
		p1 -= 512;
	}

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;

	return freq;
}

static void si5351_set_ms_source(struct m88rs6060_state *dev, enum si5351_clock clk,  enum si5351_pll pll)
{
	u8 reg_val = 0x0c;
	u8 reg_val2;

	if(si5351_read(dev,SI5351_CLK0_CTRL + (u8)clk, &reg_val2) != 0)
	{
		return;
	}

	if(pll == SI5351_PLLA)
	{
		reg_val &= ~(SI5351_CLK_PLL_SELECT);
	}
	else if(pll == SI5351_PLLB)
	{
		reg_val |= SI5351_CLK_PLL_SELECT;
	}
	si5351_write(dev,SI5351_CLK0_CTRL + (u8)clk, reg_val);

}

void si5351_init(struct m88rs6060_state *dev)
{
	si5351_write(dev,SI5351_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_10PF);
	return ;
}

void si5351_set_freq(struct m88rs6060_state *dev,u32 freq, u32 pll_freq, enum si5351_clock clk)
{
	struct Si5351RegSet ms_reg;
	struct Si5351RegSet pll_reg;
	enum si5351_pll target_pll;

	u32 ret;
	u8 params[30];
	u8 i = 0;
	u8 temp;

	/* Calculate the synth parameters */
	/* If pll_freq is 0, let the algorithm pick a PLL frequency */
	if(pll_freq == 0)
	{
		 pll_freq = multisynth_calc(freq, &ms_reg);
	}
	/* TODO: bounds checking */
	else
	{
		multisynth_recalc(freq, pll_freq, &ms_reg);
	}
	/* Determine which PLL to use */
	/* CLK0 gets PLLA, CLK1 gets PLLB */
	/* CLK2 gets PLLB if necessary */
	/* Only good for Si5351A3 variant at the moment */
	if(clk == SI5351_CLK0)
	{
		target_pll = SI5351_PLLA;
		dev->plla_freq = pll_freq;
	}
	else if(clk == SI5351_CLK1)
	{
		target_pll = SI5351_PLLB;
		dev->pllb_freq = pll_freq;
	}
	else
	{
		/* need to account for CLK2 set before CLK1 */
		if(dev->pllb_freq == 0)
		{
			target_pll = SI5351_PLLB;
			dev->pllb_freq = pll_freq;
		}
		else
		{
			target_pll = SI5351_PLLB;
			pll_freq = dev->pllb_freq;
			multisynth_recalc(freq, pll_freq, &ms_reg);
		}
	}

	ret=pll_calc(pll_freq, &pll_reg, 0);
	/* Derive the register values to write */
	/* Prepare an array for parameters to be written to */
	/* PLL parameters first */
	if(ret== 0)
	{
		/* Registers 26-27 */
		temp = ((pll_reg.p3 >> 8) & 0xff);
		params[i++] = temp;
		temp = (u8)(pll_reg.p3  & 0xff);
		params[i++] = temp;

		/* Register 28 */
		temp = (u8)((pll_reg.p1 >> 16) & 0x03);
		params[i++] = temp;

		/* Registers 29-30 */
		temp = (u8)((pll_reg.p1 >> 8) & 0xff);
		params[i++] = temp;
		temp = (u8)(pll_reg.p1  & 0xff);
		params[i++] = temp;

		/* Register 31 */
		temp = (u8)((pll_reg.p3 >> 12) & 0xf0);
		temp += (u8)((pll_reg.p2 >> 16) & 0x0f);
		params[i++] = temp;

		/* Registers 32-33 */
		temp = (u8)((pll_reg.p2 >> 8) & 0xff);
		params[i++] = temp;

		temp = (u8)(pll_reg.p2  & 0xff);
		params[i++] = temp;

		/* Write the parameters */
		if(target_pll == SI5351_PLLA)
		{
			si5351_write_bulk(dev,SI5351_PLLA_PARAMETERS, i + 1, params);
		}
		else if(target_pll == SI5351_PLLB)
		{
			si5351_write_bulk(dev,SI5351_PLLB_PARAMETERS, i + 1, params);
		}
	}
	/* Now the multisynth parameters */
	memset (params, 0, 30);
	i = 0;

	/* Registers 42-43 */
	temp = (u8)((ms_reg.p3 >> 8) & 0xff);
	params[i++] = temp;

	temp = (u8)(ms_reg.p3  & 0xff);
	params[i++] = temp;

	/* Register 44 */
	/* TODO: add code for output divider */
	temp = (u8)((ms_reg.p1 >> 16) & 0x03);
	params[i++] = temp;

	/* Registers 45-46 */
	temp = (u8)((ms_reg.p1 >> 8) & 0xff);
	params[i++] = temp;

	temp = (u8)(ms_reg.p1  & 0xff);
	params[i++] = temp;

	/* Register 47 */
	temp = (u8)((ms_reg.p3 >> 12) & 0xf0);
	temp += (u8)((ms_reg.p2 >> 16) & 0x0f);
	params[i++] = temp;

	/* Registers 48-49 */
	temp = (u8)((ms_reg.p2 >> 8) & 0xff);
	params[i++] = temp;
	temp = (u8)(ms_reg.p2  & 0xff);
	params[i++] = temp;

	/* Write the parameters */
	switch(clk)
	{
	case SI5351_CLK0:
		si5351_write_bulk(dev,SI5351_CLK0_PARAMETERS, i + 1, params);
		si5351_set_ms_source(dev,clk, target_pll);
		break;
	case SI5351_CLK1:
		si5351_write_bulk(dev,SI5351_CLK1_PARAMETERS, i + 1, params);
		si5351_set_ms_source(dev,clk, target_pll);
		break;
	case SI5351_CLK2:
		si5351_write_bulk(dev,SI5351_CLK2_PARAMETERS, i + 1, params);
		si5351_set_ms_source(dev,clk, target_pll);
		break;
	case SI5351_CLK3:
		si5351_write_bulk(dev,SI5351_CLK3_PARAMETERS, i + 1, params);
		si5351_set_ms_source(dev,clk, target_pll);
		break;
	case SI5351_CLK4:
		si5351_write_bulk(dev,SI5351_CLK4_PARAMETERS, i + 1, params);
		si5351_set_ms_source(dev,clk, target_pll);
		break;
	case SI5351_CLK5:
		si5351_write_bulk(dev,SI5351_CLK5_PARAMETERS, i + 1, params);
		si5351_set_ms_source(dev,clk, target_pll);
		break;
	case SI5351_CLK6:
		si5351_write_bulk(dev,SI5351_CLK6_PARAMETERS, i + 1, params);
		si5351_set_ms_source(dev,clk, target_pll);
		break;
	case SI5351_CLK7:
		si5351_write_bulk(dev,SI5351_CLK7_PARAMETERS, i + 1, params);
		si5351_set_ms_source(dev,clk, target_pll);
		break;
	}

}
//end
