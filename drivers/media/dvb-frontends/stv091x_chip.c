/*
 * Driver for the ST STV091x DVB-S/S2 demodulator.
 *
 * Copyright (C) 2014-2015 Ralph Metzler <rjkm@metzlerbros.de>
 *                         Marcus Metzler <mocm@metzlerbros.de>
 *                         developed for Digital Devices GmbH
 * Copyright (C) 2020-2025 Deep Thought <deeptho@gmail.com>
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 */

#include <media/dvb_frontend.h>
#include "stv091x.h"

#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

static s32 field_position(u8 mask)
{
	s32 position=0, i=0;

	while((position == 0)&&(i < 8))
	{
		position = (mask >> i) & 0x01;
		i++;
	}

	return (i-1);
}

static s32 field_bits(s32 mask, s32 position)
{
	s32 bits,bit;
	s32 i =0;

	bits = mask >> position;
	bit = bits ;
	while ((bit > 0)&&(i<8))
	{
		i++;
		bit = bits >> i;

	}
	return i;
}



static inline int i2c_write(struct i2c_adapter *adap, u8 adr,
			    u8 *data, int len)
{
	struct i2c_msg msg = {.addr = adr, .flags = 0,
			      .buf = data, .len = len};

	return (i2c_transfer(adap, &msg, 1) == 1) ? 0 : -1;
}

static int i2c_write_reg16(struct i2c_adapter *adap, u8 adr, u16 reg, u8 val)
{
	u8 msg[3] = {reg >> 8, reg & 0xff, val};

	return i2c_write(adap, adr, msg, 3);
}

int write_reg(struct stv *state, u16 reg, u8 val)
{
	return i2c_write_reg16(state->base->i2c, state->base->adr, reg, val);
}


static inline int i2c_read_reg16(struct i2c_adapter *adapter, u8 adr,
				 u16 reg, u8 *val)
{
	u8 msg[2] = {reg >> 8, reg & 0xff};
	struct i2c_msg msgs[2] = {{.addr = adr, .flags = 0,
				   .buf  = msg, .len   = 2},
				  {.addr = adr, .flags = I2C_M_RD,
				   .buf  = val, .len   = 1 } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

u8 read_reg(struct stv *state, u16 reg)
{
	u8 val;
	i2c_read_reg16(state->base->i2c, state->base->adr, reg, &val);
	return val;
}

int try_read_reg(struct stv *state, u16 reg, u8* val)
{
	return i2c_read_reg16(state->base->i2c, state->base->adr, reg, val);
}


static inline int i2c_read_regs16(struct i2c_adapter *adapter, u8 adr,
				  u16 reg, u8 *val, int len)
{
	u8 msg[2] = {reg >> 8, reg & 0xff};
	struct i2c_msg msgs[2] = {{.addr = adr, .flags = 0,
				   .buf  = msg, .len   = 2},
				  {.addr = adr, .flags = I2C_M_RD,
				   .buf  = val, .len   = len } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

int read_regs(struct stv *state, u16 reg, u8 *val, int len)
{
	return i2c_read_regs16(state->base->i2c, state->base->adr,
			       reg, val, len);
}

int try_read_reg_field(struct stv* state, u32 field_id, s32* val)
{
	int err=0;
	u8 regval;
	s32 mask, is_signed, bits, pos;
	regval = read_reg(state, field_id>>16);
	is_signed = (field_id>>8) & 0x01;
	mask = field_id & 0xff;
	pos = field_position(mask);
	bits = field_bits(mask, pos);

	*val = (regval & mask) >> pos;	/*	Extract field	*/

	if((is_signed )&&(*val & (1<<(bits-1))))
		*val -= (1<<bits);			/*	Compute signed value	*/
	return err;
}

s32 read_reg_field(struct stv* state, u32 field_id)
{
	s32 val;
	try_read_reg_field(state, field_id, &val);
	return val;
}

int  write_reg_field(struct stv* state, u32 field_id, s32 val)
{
	int err=0;
	u8 reg;
	s32
		mask,
		is_signed,
		bits,
		pos;

	err |= try_read_reg(state, (field_id >> 16)&0xFFFF, &reg);
	is_signed = (field_id>>8) & 0x01;
	mask = field_id & 0xff;
	pos = field_position(mask);
	bits = field_bits(mask, pos);

	if(is_signed)
		val = (val > 0 ) ? val : val + (bits);

	val = mask & (val << pos);

	reg =(reg & (~mask)) + val;
	err |= write_reg(state, (field_id >> 16)&0xffff , reg);
	return err;
}

int  write_reg_fields_(struct stv* state, u16 addr, struct reg_field* fields, int num_fields)
{
	int err=0;
	u8 reg;
	int i;
	s32
		mask,
		is_signed,
		bits,
		pos;
	s32 val;
	for(i=0; i < num_fields; ++i) {
		struct reg_field * field = &fields[i];
		if(i==0) {
			err |= try_read_reg(state, (field->field_id >> 16)&0xFFFF, &reg);
			addr = (field->field_id >> 16)&0xffff;
		} else {
			BUG_ON(addr != ((field->field_id >> 16)&0xffff));
		}
		is_signed = (field->field_id>>8) & 0x01;
		mask = field->field_id & 0xff;
		pos = field_position(mask);
		bits = field_bits(mask, pos);
		val = field->val;
		if(is_signed)
			val = (val > 0 ) ? val : val + (bits);

		val = mask & (val << pos);

		reg =(reg & (~mask)) + val;
	}
	err |= write_reg(state, addr , reg);
	return err;
}


/*****************************************************
**FUNCTION	::	XtoPowerY
**ACTION	::	Compute  x^y (where x and y are integers)
**PARAMS IN	::	Number -> x
**				Power -> y
**PARAMS OUT::	NONE
**RETURN	::	2^n
*****************************************************/
u32 XtoPowerY(s32 Number, u32 Power)
{
	u32 i;
	u32 result = 1;
	u32 oldresult = 0;

	if(Power>=32){
		printk("Potentiall unsafe call: Number = %d Power = %d  -- \n",Number,Power);
	}
	for(i=0;i<Power;i++){
		oldresult =  result;
		result *= Number;
		if(result<=oldresult)
			printk("overflow:Number=%d, Power =%d  --\n",Number,Power);
	}
	return result;
}


#define PRECISION 7
#define STlog10_2  301   /* log10_2 = 0.30102999566398119521373889472449 */


static const unsigned short logtable[] =
{
	0, 6, 11,   17,
	22,   28,   33,   39,
	44,   50,   55,   61,
	66,   71,   77,   82,
	87,   93,   98,   103,
	109,  114,  119,  124,
	129,  134,  140,  145,
	150,  155,  160,  165,
	170,  175,  180,  185,
	190,  195,  200,  205,
	209,  214,  219,  224,
	229,  234,  238,  243,
	248,  253,  257,  262,
	267,  271,  276,  281,
	285,  290,  295,  299,
	304,  308,  313,  317,
	322,  326,  331,  335,
	340,  344,  349,  353,
	358,  362,  366,  371,
	375,  379,  384,  388,
	392,  397,  401,  405,
	409,  414,  418,  422,
	426,  430,  435,  439,
	443,  447,  451,  455,
	459,  464,  468,  472,
	476,  480,  484,  488,
	492,  496,  500,  504,
	508,  512,  516,  520,
	524,  527,  531,  535,
	539,  543,  547,  551,
	555,  558,  562,  566,
	570,  574,  577,  581,
	585,  589,  592,  596,
	600,  604,  607,  611,
	615,  618,  622,  626,
	629,  633,  637,  640,
	644,  647,  651,  655,
	658,  662,  665,  669,
	672,  676,  679,  683,
	687,  690,  693,  697,
	700,  704,  707,  711,
	714,  718,  721,  725,
	728,  731,  735,  738,
	741,  745,  748,  752,
	755,  758,  762,  765,
	768,  771,  775,  778,
	781,  785,  788,  791,
	794,  798,  801,  804,
	807,  811,  814,  817,
	820,  823,  827,  830,
	833,  836,  839,  842,
	845,  849,  852,  855,
	858,  861,  864,  867,
	870,  873,  877,  880,
	883,  886,  889,  892,
	895,  898,  901,  904,
	907,  910,  913,  916,
	919,  922,  925,  928,
	931,  934,  937,  940,
	943,  945,  948,  951,
	954,  957,  960,  963,
	966,  969,  972,  974,
	977,  980,  983,  986,
	989,  992,  994,  997
};

static int MostSignificantBit(int x)
{
	int i=0;

	if (!x)
		return 0;

	while(!(x & 0x80000000))
	{
		x<<=1;
		i++;
 }
	return 31-i;
}

unsigned int STLog2(u32 value)
{

	unsigned int msb=0;
	unsigned int logindex=0;
	unsigned int result=0;
	unsigned int interpolation=0;
	int inc = 1;

	msb =  MostSignificantBit(value);

	if ((msb>0)  && ( msb<=PRECISION))
	{
		logindex = value <<(PRECISION - msb);
  }
	else if (msb>0)
	{
		logindex = value >>(msb - PRECISION);
  }

	logindex = (logindex & 0x7F)*2;

	inc = XtoPowerY(2,msb);

	interpolation =  ( (value*256) -(logindex + (256*inc))	) * logtable[logindex] /2;
	interpolation -= ( (logindex+1)+ 256*inc-(256*value)	) * logtable[logindex+1]/2;

	inc = 1000000000;

	result =  (msb*1000 + logtable[logindex]  + (interpolation/inc));

	return   result;

}

//returns 1000*log10(value)
s32 STLog10(u32 value)
{
	return ((STLog2(value) * STlog10_2)/1000);
}
