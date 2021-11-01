/*
 * AV201x Airoha Technology silicon tuner driver
 *
 * Copyright (C) 2014 Luis Alves <ljalvs@gmail.com>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "av201x.h"
#include "av201x_priv.h"

#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt), __func__, __LINE__, ##arg)


/* write multiple (continuous) registers */
static int av201x_wrm(struct av201x_priv *priv, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg = {
		.addr = priv->cfg->i2c_address,
		.flags = 0, .buf = buf, .len = len };

	dev_dbg(&priv->i2c->dev, "%s() i2c wrm @0x%02x (len=%d) ",
		__func__, buf[0], len);

	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret < 0) {
		dev_warn(&priv->i2c->dev,
			"%s: i2c wrm err(%i) @0x%02x (len=%d)\n",
			KBUILD_MODNAME, ret, buf[0], len);
		return ret;
	}
	return 0;
}

/* write one register */
static int av201x_wr(struct av201x_priv *priv, u8 addr, u8 data)
{
	u8 buf[] = { addr, data };
	return av201x_wrm(priv, buf, 2);
}

/* read multiple (continuous) registers starting at addr */
static int av201x_rdm(struct av201x_priv *priv, u8 addr, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg[] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0,
			.buf = &addr, .len = 1 },
		{ .addr = priv->cfg->i2c_address, .flags = I2C_M_RD,
			.buf = buf, .len = len }
	};

	dev_dbg(&priv->i2c->dev, "%s() i2c rdm @0x%02x (len=%d)\n",
		__func__, addr, len);

	ret = i2c_transfer(priv->i2c, msg, 2);
	if (ret < 0) {
		dev_warn(&priv->i2c->dev,
			"%s: i2c rdm err(%i) @0x%02x (len=%d)\n",
			KBUILD_MODNAME, ret, addr, len);
		return ret;
	}
	return 0;
}

/* read one register */
static int av201x_rd(struct av201x_priv *priv, u8 addr, u8 *data)
{
	return av201x_rdm(priv, addr, data, 1);
}

/* read register, apply masks, write back */
static int av201x_regmask(struct av201x_priv *priv,
	u8 reg, u8 setmask, u8 clrmask)
{
	int ret;
	u8 b = 0;
	if (clrmask != 0xff) {
		ret = av201x_rd(priv, reg, &b);
		if (ret)
			return ret;
		b &= ~clrmask;
	}
	return av201x_wr(priv, reg, b | setmask);
}

static int av201x_wrtable(struct av201x_priv *priv,
	struct av201x_regtable *regtable, int len)
{
	int ret, i;

	for (i = 0; i < len; i++) {
		ret = av201x_regmask(priv, regtable[i].addr,
			regtable[i].setmask, regtable[i].clrmask);
		if (ret)
			return ret;
		if (regtable[i].sleep)
			msleep(regtable[i].sleep);
	}
	return 0;
}

static void av201x_release(struct dvb_frontend *fe)
{
	struct av201x_priv *priv = fe->tuner_priv;
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
}

static int av201x_init(struct dvb_frontend *fe)
{
	struct av201x_priv *priv = fe->tuner_priv;
	int ret;
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	ret = av201x_wrtable(priv, av201x_inittuner0,
		ARRAY_SIZE(av201x_inittuner0));

	switch (priv->cfg->id) {
	case ID_AV2011:
		ret |= av201x_wrtable(priv, av201x_inittuner1a,
			ARRAY_SIZE(av201x_inittuner1a));
		break;
	case ID_AV2012:
	default:
		ret |= av201x_wrtable(priv, av201x_inittuner1b,
			ARRAY_SIZE(av201x_inittuner1b));
		break;
	}

	ret |= av201x_wrtable(priv, av201x_inittuner2,
		ARRAY_SIZE(av201x_inittuner2));

	ret |= av201x_wr(priv, REG_TUNER_CTRL, 0x96);

	msleep(120);

	if (ret)
		dev_dbg(&priv->i2c->dev, "%s() failed\n", __func__);
	return ret;
}

static int av201x_sleep(struct dvb_frontend *fe)
{
	struct av201x_priv *priv = fe->tuner_priv;
	int ret;
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	ret = av201x_regmask(priv, REG_TUNER_CTRL, AV201X_SLEEP, 0);
	if (ret)
		dev_dbg(&priv->i2c->dev, "%s() failed\n", __func__);
	return ret;
}

//frequency in kHz;
static int av201x_set_frequency(struct dvb_frontend *fe, u32 frequency)
{
	struct av201x_priv *priv = fe->tuner_priv;
	u8 buf[5];
	int ret=0;
	//u32 n;
	//u32 n1;
	int Int;
  int Frac;
	int RF;
	dev_dbg(&priv->i2c->dev, "%s() frequency=%d\n", __func__, frequency);

	/*
	   ** PLL setup **
	   RF = (pll_N * ref_freq) / pll_M
	   pll_M = fixed 0x10000
	   PLL output is divided by 2
	   REG_FN = pll_M<24:0>
	*/
	buf[0] = REG_FN;

	Int  =   DIV_ROUND_CLOSEST(frequency, priv->cfg->xtal_freq); //27000
  Frac = (((s32)frequency -  Int*(s32)priv->cfg->xtal_freq  )<<17)/(s32)(priv->cfg->xtal_freq);
  RF = (Int*(s32)priv->cfg->xtal_freq) + ((Frac*(s32)priv->cfg->xtal_freq)>>17);
	buf[1] = (Int > 0xff) ? 0xff : (u8) Int;
	//dprintk("xxx freq=%d Int=%d Frac=%d RF=%d", frequency, Int, Frac, RF);
	buf[2] = (u8) (Frac >> 9);
	buf[3] = (u8) (Frac >> 1);
	buf[4] = (u8) (((Frac << 7) & 0x80) | 0x50);
	ret = av201x_wrm(priv, buf, 5);
	if (ret)
		goto exit;
 exit:
	if (ret)
		dev_dbg(&priv->i2c->dev, "%s() failed\n", __func__);
	return ret;
}


// bw in kHz; returns the actual set bandwidth, which may be different than requested; negative on error
static int av201x_set_bandwith(struct dvb_frontend *fe, u32 bandwidth)
{
	struct av201x_priv *priv = fe->tuner_priv;
	int ret=0;
	u32 bf;
	u32 bw = bandwidth;
	/* check limits (4MHz < bandwidth < 40MHz) */
	if (bw > 40000) {
		dprintk("Bandwidth %dkHz too large! reduced to 40000\n", bw);
		bw = 40000;
	}
	else if (bw < 4000) {
		dprintk("Bandwidth %dkHz too small! increased to 4000\n", bw); //official limit is 4000 (but 500 is possible)
		bw = 4000;
	}
	dprintk("Setting bandwidth %dkHz\n", bw);
	/* bandwidth step = 211kHz */
	bf = DIV_ROUND_CLOSEST(bw * 127, 21100);
	ret = av201x_wr(priv, REG_BWFILTER, (u8) bf);

	/* enable fine tune agc */
	ret |= av201x_wr(priv, REG_FT_CTRL, AV201X_FT_EN | AV201X_FT_BLK);

	ret |= av201x_wr(priv, REG_TUNER_CTRL, 0x96);

	if (ret) {
		dev_dbg(&priv->i2c->dev, "%s() failed\n", __func__);
		return -1;
	}
	return bw;
}

//frequency and bandwith in kHz
static int av201x_set_frequency_and_bandwidth(struct dvb_frontend *fe, u32 frequency, u32 bandwidth)
{
	struct av201x_priv *priv = fe->tuner_priv;
	int ret = 0;
	dev_dbg(&priv->i2c->dev, "%s() frequency=%d \n", __func__, frequency);

	ret= av201x_set_frequency(fe, frequency);
	if(ret)
		return ret;
	msleep(20);
	ret = av201x_set_bandwith(fe, bandwidth);
	if (ret>0)
		ret =0;
	return ret;
}

static int av201x_set_params(struct dvb_frontend *fe)
{
	u32 bw;
	int ret=0;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;

	/* set bandwidth */
	bw = (c->symbol_rate / 1000) * 135/200;
	if (c->symbol_rate < 6500000)
		bw += 6000;
	bw += 2000;
	bw *= 108/100;


	ret = av201x_set_frequency_and_bandwidth(fe, c->frequency, bw);
	msleep(20);
	if(ret)
		dprintk("ERROR: ret=%d", ret);
	return ret;
}

static int av201x_agc_to_gain_dbm(struct dvb_frontend *fe, s32 if_agc)
{
	int dbm = 0;
	if((if_agc>2000)&&(if_agc<=4096))
		dbm=(-((if_agc*1000-2000000)/140)-79000);
	else if((if_agc>1500)&&(if_agc<=2000))
		dbm=(-((if_agc*1000-1500000)/55)-69000);
	else if((if_agc>1000)&&(if_agc<=1500))
		dbm=(-((if_agc*1000-1000000)/25)-48000);
	else if((if_agc>800)&&(if_agc<=1000))
		dbm=(-((if_agc*1000-800000)/20)-37000);
	else if((if_agc>260)&&(if_agc<=800))
		dbm=(-((if_agc*1000-260000)/16)-0);
	else
		dbm=0;
	return dbm + 7500;
}

static int av201x_get_rf_strength(struct dvb_frontend *fe, u16 *st)
{
	//struct av201x_priv *priv = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	s32 gain = av201x_agc_to_gain_dbm(fe, *st);
	*st = 1000 + gain/1000;
	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	c->strength.stat[0].svalue = gain;

	return 0;
}


static const struct dvb_tuner_ops av201x_tuner_ops = {
	.info = {
		.name           = "Airoha Technology AV201x",
		.frequency_min_hz = 850 * MHz,
		.frequency_max_hz = 2300 * MHz,
	},

	.release = av201x_release,

	.init = av201x_init,
	.sleep = av201x_sleep,
	.set_frequency_and_bandwidth = av201x_set_frequency_and_bandwidth,
	.set_bandwidth     = av201x_set_bandwith,
	.set_frequency     = av201x_set_frequency,
	.set_params = av201x_set_params,
	.get_rf_strength = av201x_get_rf_strength,
	.agc_to_gain_dbm   = av201x_agc_to_gain_dbm,
};

struct dvb_frontend *av201x_attach(struct dvb_frontend *fe,
		struct av201x_config *cfg, struct i2c_adapter *i2c)
{
	struct av201x_priv *priv = NULL;

	priv = kzalloc(sizeof(struct av201x_priv), GFP_KERNEL);
	if (priv == NULL) {
		dev_dbg(&i2c->dev, "%s() attach failed\n", __func__);
		return NULL;
	}

	priv->cfg = cfg;
	priv->i2c = i2c;

	dev_info(&priv->i2c->dev,
		"%s: Airoha Technology AV201x successfully attached\n",
		KBUILD_MODNAME);

	memcpy(&fe->ops.tuner_ops, &av201x_tuner_ops,
			sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;
	return fe;
}
EXPORT_SYMBOL(av201x_attach);

MODULE_DESCRIPTION("Airoha Technology AV201x silicon tuner driver");
MODULE_AUTHOR("Luis Alves <ljalvs@gmail.com>");
MODULE_LICENSE("GPL");
