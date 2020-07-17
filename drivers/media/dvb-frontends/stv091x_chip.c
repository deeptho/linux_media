/*
 * Driver for the ST STV091x DVB-S/S2 demodulator.
 *
 * Copyright (C) 2014-2015 Ralph Metzler <rjkm@metzlerbros.de>
 *                         Marcus Metzler <mocm@metzlerbros.de>
 *                         developed for Digital Devices GmbH
 * Copyright (C) 2020      Deep Thought <deeptho@gmail.com>
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
