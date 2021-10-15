/*
 * Silicon Labs Si2183(2) DVB-T/T2/C/C2/S/S2 blindscan code
 * (c) Deep Thought <deeptho@gmail.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "si2183.h"
#include <media/dvb_frontend.h>
#include <linux/firmware.h>
#include <linux/i2c-mux.h>
#include <linux/kthread.h>

#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt), __func__, __LINE__, ##arg)


#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
#define SI2183_USE_I2C_MUX
#endif

#define SI2183_B60_FIRMWARE "dvb-demod-si2183-b60-01.fw"

#define SI2183_PROP_MODE	0x100a
#define SI2183_PROP_DVBC_CONST	0x1101
#define SI2183_PROP_DVBC_SR	0x1102
#define SI2183_PROP_DVBT_HIER	0x1201
#define SI2183_PROP_DVBT2_MODE	0x1304
#define SI2183_PROP_DVBS2_SR	0x1401
#define SI2183_PROP_DVBS_SR	0x1501
#define SI2183_PROP_MCNS_CONST 0x1601
#define SI2183_PROP_MCNS_SR 0x1602
#define Si2183_SCAN_FMIN_PROP_CODE 0x000303
#define Si2183_SCAN_FMAX_PROP_CODE 0x000304
#define Si2183_SCAN_SYMB_RATE_MIN_PROP_CODE 0x000305
#define Si2183_SCAN_SYMB_RATE_MAX_PROP_CODE 0x000306
#define Si2183_SCAN_IEN_PROP_CODE 0x000308
#define Si2183_SCAN_INT_SENSE_PROP 0x0307
#define Si2183_SCAN_INT_SENSE_PROP_CODE 0x000307
#define Si2183_DD_MODE_PROP_CODE 0x00100a
#define Si2183_DD_MODE_PROP_AUTO_DETECT_AUTO_DVB_S_S2 2
#define Si2183_SCAN_SAT_CONFIG_PROP_CODE 0x000302
#define Si2183_DD_RESTART_CMD 0x85

#define Si2183_SCAN_CTRL_CMD_ACTION_ABORT 3
#define Si2183_SCAN_CTRL_CMD_ACTION_RESUME 2
#define Si2183_SCAN_CTRL_CMD_ACTION_START 1

#define Si2183_SCAN_STATUS_RESPONSE_BUZ_BUSY 1
#define ADDR3(a,b,c) (a|(b<<8)|(c<<16))

#define Si2183_SCAN_CTRL_CMD_ACTION_ABORT 3
#define Si2183_SCAN_CTRL_CMD 0x31
#define Si2183_SCAN_STATUS_CMD 0x30
#define Si2183_SCAN_STATUS_CMD_INTACK_OK 0
#define Si2183_SCAN_STATUS_CMD_INTACK_CLEAR 1
#define Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_ANALOG_CHANNEL_FOUND 6
#define Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_DEBUG 63
#define Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_DIGITAL_CHANNEL_FOUND 5
#define Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_ENDED 2
#define Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_ERROR 3
#define Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_IDLE 0
#define Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_SEARCHING 1
#define Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_TUNE_REQUEST 4








static int si2183_get_register_unlocked(struct i2c_client *client, u32 address, s32* value_ret)
{
	struct si2183_cmd cmd		= {{0x8f, /*get reg*/
															address &0xff,
															(address>>8)&0xff,
															(address>>16)&0xff}, //address
														 4, 5};
	int ret = si2183_cmd_execute_unlocked(client, &cmd);
	*value_ret = cmd.args[1] | (cmd.args[2]<<8) |(cmd.args[3]<<16) | (cmd.args[4]<<24);

	return ret;
 }

static int si2183_get_register_locked(struct i2c_client *client, u32 address, s32* value_ret)
{
		struct si2183_dev *dev = i2c_get_clientdata(client);
	int ret;


	mutex_lock(&dev->base->i2c_mutex);
	ret = si2183_get_register_unlocked(client, address, value_ret);
	mutex_unlock(&dev->base->i2c_mutex);

	return ret;
}

#if 0
static int si2183_set_register_unlocked(struct i2c_client *client, u32 address, s32 value)
 {
	struct si2183_cmd cmd		= {{0x8e, /*set reg*/
															address &0xff,
															(address>>8)&0xff,
															(address>>16)&0xff, //address
															value &0xff,
															(value>>8)&0xff,
															(value>>16)&0xff,
															(value>>24)&0xff},
															8, 1};
	return si2183_cmd_execute_unlocked(client, &cmd);
 }
#endif

 static int si2183_set_property_unlocked(struct i2c_client *client, u16 property, u32 data)
 {
#define Si2183_SET_PROPERTY_CMD 0x14
	 struct si2183_cmd cmd = {
		 .args={Si2183_SET_PROPERTY_CMD,
						0x00, //reserved
							property & 0xff,
						property >>8,
						data & 0xff,
						data >>8
		 },
		 .wlen=6,
		 .rlen=4
	 };

	 int ret =si2183_cmd_execute_unlocked(client, &cmd);
	 if(ret) {
		 dprintk("set property failed: ret=%d\n", ret);
	 }
	 //one data vyte is in cmd.args[1]
	 return ret;
 }

static int si2183_set_property(struct i2c_client *client, u16 property, u32 data)
{
	struct si2183_dev *dev = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&dev->base->i2c_mutex);
	ret = si2183_set_property_unlocked(client, property, data);
	mutex_unlock(&dev->base->i2c_mutex);

	return ret;
}


static int si2183_get_iq_sample(struct dvb_frontend *fe, s16* ival, s16* qval)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	int ret=-1;
  unsigned int  value;
  switch (p->delivery_system)
  {
    case SYS_DVBC2:
    case SYS_DVBT :
    case SYS_DVBT2: {
#ifdef TODO
      if (Si2183_iq_demap != 0) {
				ret = si2183_get_register_locked(client, 0x0edc1f, &value); //rx_iq
      } else {
#endif
				ret = si2183_get_register_locked(client, 0x072c13, &value); //symb_iq
#ifdef TODO
      }
#endif
      //imask = 0x000ffc00 ; qmask = 0x000003ff ; ishift = 10 ; iq_coef = 4; limit = qmask+1;
			*ival = ((value >>10) << 6); //range: -32768 ... +32767; 10 bit per component
			*qval = (value << 6); //range: -32768 ... +32767
      break;
    }
    case SYS_DVBC_ANNEX_A : {
			ret = si2183_get_register_locked(client, 0x08e81f, &value); //iq_symb
      //imask = 0xffff0000 ; qmask = 0x0000ffff ; ishift = 16 ; iq_coef = 2; limit = qmask+1;
			*ival = (value >> 16); //range: -32768 ... +32767, 16 bit per componentn
			*qval = value; //range: -32768 ... +32767
      break;
    }
    case SYS_DSS  :
    case SYS_DVBS :
    case SYS_DVBS2: {
			ret = si2183_get_register_locked(client, 0x05f80f, &value); //iq_demod_sat
      //imask = 0x0000ff00 ; qmask = 0x000000ff ; ishift =  8 ; iq_coef = 4; limit = qmask+1;
			*ival = ((value >> 8)<<8); //range: -32768 ... +32767, 8 bit per componentn
			*qval = (value<<8); //range: -32768 ... +32767
      break;
    }
    default : {
			dprintk("unknown standard");
      ret = -1;
      break;
		}
  }
	*ival >>= 8;
	*qval >>= 8;
  return ret;
}



s32 si2183_narrow_band_signal_power_dbm(struct dvb_frontend *fe)
{
	//		struct si2183_dev *state = fe->demodulator_priv;
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_cmd cmd;
	u16 agc1, agc2;
	s32 gain1=0;
	int ret=0;
	//todo: make better estimate

	/*[0]=Si2183_DD_EXT_AGC_SAT_CMD
		[1] =flags: agc1_mode, agc2_mode (not used), agc1_inv
		[2,3,4,5] agc1_kloop, agc2_kloop agc1_min, agc2_min
	*/
	memcpy(cmd.args, "\x8a\x00\x00\x00\x00\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 3;
	ret = si2183_cmd_execute(client, &cmd);
	if (ret<0) {
		dprintk("signal_power fe%d cmd_exec failed=%d\n", fe->id, ret);
		dev_err(&client->dev, "signal_power fe%d cmd_exec failed=%d\n", fe->id, ret);
		return 0;
	}

	agc1 = cmd.args[1];
	agc2 = cmd.args[2];
	dprintk("AGC: %d %d\n", agc1, agc2);
	if(fe->ops.tuner_ops.agc_to_gain_dbm)
		gain1 = fe->ops.tuner_ops.agc_to_gain_dbm(fe, ((int)agc1)<<4); //in units of 0.001dB
	else if (fe->ops.tuner_ops.get_rf_strength) {
		gain1 = fe->ops.tuner_ops.get_rf_strength(fe, &agc1); //could be anything (drivers inconsistent)
	}
	dprintk("STRENGTH: %d => %d\n", agc1, gain1);
	return -gain1;
	//missing: agc ref gain
}




int si2183_stop_task(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *state = i2c_get_clientdata(client);
	struct spectrum_scan_state* ss = &state->scan_state;
	struct constellation_scan_state* cs = &state->constellation_scan_state;
	dprintk("called ss=%p\n", ss);
	dprintk("called %p %p %p\n", ss->freq, ss->spectrum, cs->samples);
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


int si2183_constellation_start(struct dvb_frontend *fe, struct dtv_fe_constellation* user, int max_num_samples)
{
	struct si2183_dev *state = fe->demodulator_priv;
	struct constellation_scan_state* cs = &state->constellation_scan_state;

	int num_samples = user->num_samples;
	if(num_samples > max_num_samples)
		num_samples = max_num_samples;
	dprintk("demod: ??: constellation samples=%d/%d constel_select=%d\n", /*state->nr,*/ user->num_samples,
					max_num_samples, (int)user->constel_select);
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

	cs->constel_select =  user->constel_select;
	cs->num_samples = 0;


	for (cs->num_samples = 0; cs->num_samples < cs->samples_len; ++cs->num_samples) {
		if ((cs->num_samples% 20==19) && (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			dprintk("exiting on should stop\n");
			break;
		}
		si2183_get_iq_sample(fe, &cs->samples[cs->num_samples].imag, &cs->samples[cs->num_samples].real);
	}

	dprintk("demod: ??: constellation retrieved samples=%d\n", /*state->nr,*/ cs->num_samples);
	return 0;
}

int si2183_constellation_get(struct dvb_frontend *fe, struct dtv_fe_constellation* user)
{
	struct si2183_dev *state = fe->demodulator_priv;
	struct constellation_scan_state* cs = &state->constellation_scan_state;
	int error = 0;
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




u8 Si2183_convert_to_byte (const u8* buffer, u8 shift, u8 mask) {
	unsigned int rspBuffer = *buffer;
	return ((rspBuffer >> shift) & mask);
}

u32 Si2183_convert_to_ulong(const unsigned char* buffer, unsigned char shift, unsigned long mask) {
	return ((( ( (unsigned long)buffer[0]) | ((unsigned long)buffer[1] << 8) | ((unsigned long)buffer[2]<<16) | ((unsigned long)buffer[3]<<24)) >> shift) & mask );
}
u32 Si2183_convert_to_uint (const unsigned char* buffer, unsigned char shift, unsigned int mask) {
	return (( ( (unsigned int)buffer[0]) | (((unsigned int)buffer[1]) << 8) >> shift) & mask);
}
s16 Si2183_convert_to_int (const unsigned char* buffer, unsigned char shift, unsigned int mask) {
	return (( ( (unsigned int)buffer[0]) | (((unsigned int)buffer[1]) << 8) >> shift) & mask);
}



static int si2183_scan_status_(const char*func, int line,
															 struct i2c_client *client, u8 intack, struct si2183_scan_status_t*s)
{
	 struct si2183_cmd cmd = {
		 .args={Si2183_SCAN_STATUS_CMD,
						(unsigned char) ( ( intack & 0x01) << 0)
		 },
		 .wlen=2,
		 .rlen=11
	 };

	 int ret =si2183_cmd_execute_unlocked(client, &cmd);
	 if(ret) {
		 dprintk("set property failed: ret=%d\n", ret);
		 return ret;
	 }

	 s->buzint = Si2183_convert_to_byte (&cmd.args[ 1], 0, 0x1 );
	 s->reqint = Si2183_convert_to_byte (&cmd.args[ 1], 1, 0x1 );
	 s->buz = Si2183_convert_to_byte (&cmd.args[ 2], 0, 1 );
	 s->req = Si2183_convert_to_byte (&cmd.args[ 2], 1, 0x1 );
	 s->scan_status = Si2183_convert_to_byte (&cmd.args[ 3], 0, 0x3f );
	 s->rf_freq = Si2183_convert_to_ulong (&cmd.args[ 4], 0, 0xffffffff );
	 s->symb_rate = Si2183_convert_to_uint (&cmd.args[ 8], 0, 0xffffffff );
	 s->modulation = Si2183_convert_to_byte (&cmd.args[10], 0, 0x0f );

#if 0
	 printk("%s:%d: buzint=%d reqint=%d buz=%d req=%d scan_status=%d rf_freq=%d symb_rate=%d modulation=%d\n",
					func, line,
					s->buzint, s->reqint, s->buz, s->req, s->scan_status, s->rf_freq, s->symb_rate, s->modulation);
#endif
	 return ret;
}


#define si2183_scan_status(client, intack, s) \
	si2183_scan_status_(__func__, __LINE__, client, intack, s)


/*
	check if device wants interaction from driver
 */
static int si2183_check_interaction(struct i2c_client *client, bool* scanint)
{
	 struct si2183_cmd cmd = {
		 .args={0x0,
		 },
		 .wlen=0,
		 .rlen=1
	 };

	 int ret =si2183_cmd_execute_unlocked(client, &cmd);
	 if(ret) {
		 dprintk("check status failed: ret=%d\n", ret);
		 *scanint = false;
		 return ret;
	 }

	 //ddint = cmd.args[0] & 0x1;
	 *scanint = (cmd.args[0] >>1)&0x1;
	 //err = (cmd.args[0] >>6)&0x1;
		 //cts = (cmd.args[0] >>7)&0x1;


	 return ret;
}




static int si2183_scan_abort(struct i2c_client *client)
{
	//Si2183_L1_SCAN_CTRL (front_end->demod, Si2183_SCAN_CTRL_CMD_ACTION_ABORT , 0);
	s32 tuned_rf_freq =0;
	struct si2183_cmd cmd = {
		.args={Si2183_SCAN_CTRL_CMD, Si2183_SCAN_CTRL_CMD_ACTION_ABORT, 0x00, 0x00, tuned_rf_freq & 0xff ,
					 (tuned_rf_freq>>8)&0xff, (tuned_rf_freq>>16)&0xff, (tuned_rf_freq>>24)&0xff},
		.wlen=8,
		.rlen=1};
	int ret=si2183_cmd_execute(client, &cmd);
	if(ret) {
		dprintk("scan_abort failed\n");
	}
	return ret;
}


/*
	init = 1: start the scan at the search range specified by the user
	init = 0: start the scan just beyond the last found frequency
*/
static int si2183_set_scan_limits(struct i2c_client *client,	struct dtv_frontend_properties *p)
{
	int ret=0;
	u16 scan_fmin= (((uint32_t)p->scan_start_frequency)*1000)>>16;
	s32 scan_fmax=(((uint32_t)p->scan_end_frequency)*1000)>>16;
	s32 sym_rate_min=100000/1000;
	s32 sym_rate_max=45000000/1000;

	ret |= si2183_set_property(client, Si2183_SCAN_FMIN_PROP_CODE, scan_fmin);
	ret |= si2183_set_property(client, Si2183_SCAN_FMAX_PROP_CODE, scan_fmax);
	ret |= si2183_set_property(client, Si2183_SCAN_SYMB_RATE_MIN_PROP_CODE, sym_rate_min);
	ret |= si2183_set_property(client, Si2183_SCAN_SYMB_RATE_MAX_PROP_CODE, sym_rate_max);
	if(ret)
		dprintk("FAILED to set limits\n");
	return ret;
}

static int	si2183_enable_interaction(struct i2c_client *client)
{
	s32 buzien=1;
	s32 reqien =1;
	s32 reqnegen = 0;
	s32 reqposen = 1;
	s32 buznegen = 1;
	s32 buzposen = 0;
	s32 int_sense =(buznegen & 0x01) << 0 |
		(reqnegen & 0x01) << 1 |
		(buzposen & 0x01) << 8 |
		(reqposen & 0x01) << 9 ;
	int ret=0;

	ret |= si2183_set_property(client, Si2183_SCAN_IEN_PROP_CODE, (buzien&0x1) | ((reqien&0x1)<<1));
	ret |= si2183_set_property(client, Si2183_SCAN_INT_SENSE_PROP_CODE, int_sense);
	if(ret)
		dprintk("FAILED to enable interaction\n");
	return ret;
}

static int si2183_set_scan_bandwidth(struct i2c_client *client, s32 bandwidth)
{
	s32 bw = bandwidth/1000; //only important for dvb-t
	s32 modulation = 15; //autodetect
	s32 invert_spectrum = 0; //not inverted
	s32 auto_detect = Si2183_DD_MODE_PROP_AUTO_DETECT_AUTO_DVB_S_S2; //detect any dvb-s

	s32 dd_mode = (bw& 0x0f) << 0 |
			(modulation & 0x0f) << 4 |
		(invert_spectrum & 0x01) << 8 |
		(auto_detect & 0x07 ) << 9;
	int ret= si2183_set_property(client, Si2183_DD_MODE_PROP_CODE, dd_mode);
	if(ret)
		dprintk("Error setting bandwidth\n");
	return ret;
}

static int si2183_set_scan_mode(struct i2c_client *client, s32 debug)
{
	s32 analog_detect = 1; //enabled
	s32 reserved1 = 0;
	s32 reserved2 = 12;
	s32 scan_debug = 0x0; //0x03; use 3 for spectrum scan?
	//Si2183_L1_SetProperty2(front_end->demod, Si2183_SCAN_SAT_CONFIG_PROP_CODE);
	s32 sat_config = (analog_detect & 0x01) << 0 |
		(reserved1 & 0x1f ) << 1 |
		(reserved2 & 0x7f ) << 6 |
		(scan_debug & 0x07 ) << 13 ;
	int ret=si2183_set_property(client, Si2183_SCAN_SAT_CONFIG_PROP_CODE, sat_config);
	if(ret)
		dprintk("Error setting scan mode\n");
	return ret;
}

static int si2183_restart(struct i2c_client *client)
 {
	 struct si2183_cmd cmd = {
		 .args={Si2183_DD_RESTART_CMD},
		 .wlen=1,
		 .rlen=1
	 };

	 int ret =si2183_cmd_execute_unlocked(client, &cmd);
	 if(ret)
		 dprintk("Error setting scan mode\n");
	 return ret;
 }

static int si2183_scan_action(struct i2c_client *client, s32 action, s32 seek_freq)
{
	struct si2183_cmd cmd = {
		.args={Si2183_SCAN_CTRL_CMD, action, 0x00, 0x00, seek_freq & 0xff ,
					 (seek_freq>>8)&0xff, (seek_freq>>16)&0xff, (seek_freq>>24)&0xff},
		.wlen=8,
		.rlen=1};
	int ret = si2183_cmd_execute(client, &cmd);
	dprintk(" Si2183_SCAN_CTRL_CMD_ACTION_START freq=%dkHz\n", seek_freq);
	//action = Si2183_SCAN_CTRL_CMD_ACTION_RESUME;
	if(ret)
		dprintk("scan action failed\n");
	return ret;
}


static int si2183_scan_sat_start(struct dvb_frontend *fe, struct blindscan_state * bs)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct si2183_scan_status_t* scan_status = &bs->scan_status;

	memset(bs, 0, sizeof(struct blindscan_state));
	si2183_stop_task(fe); //clean up
	bs->bandwidth = 40000;
	bs->start_resume = 1;
	bs->skip_resume = 0;

	bs->action = Si2183_SCAN_CTRL_CMD_ACTION_START;
	bs->seek_freq = p->scan_start_frequency;
 //s32 tuned_rf_freq; // = (950000/65536)*1000;//same as start frequency


	if(fe->ops.tuner_ops.set_bandwidth)
		fe->ops.tuner_ops.set_bandwidth(fe, bs->bandwidth);

	si2183_scan_abort(client);
	si2183_set_scan_limits(client, p);
	si2183_enable_interaction(client);
	si2183_set_scan_bandwidth(client, bs->bandwidth);

	si2183_set_scan_mode(client, 0x0 /*use 3 for spectrum*/);
	si2183_restart(client);

	/* Load blindscan status at start */
 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);
 return 0;
}


int si2183_scan_sat(struct dvb_frontend *fe, bool init,
													 unsigned int *delay, enum fe_status *status)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct si2183_dev *state = i2c_get_clientdata(client);
	struct blindscan_state * bs = &state->blindscan_state;
	struct si2183_scan_status_t* scan_status = &bs->scan_status;
	bool found = false;

 //s32 tuned_rf_freq; // = (950000/65536)*1000;//same as start frequency
 //s32 seek_freq=0;
	dprintk("init=%d start=%dkHz end=%dkHz\n", init, p->scan_start_frequency, p->scan_end_frequency);
 if(init) {
	 si2183_scan_sat_start(fe, bs);
 }




 // while(1)
	 {
	 if(bs->start_resume) {
		 int count=0;
		 while (scan_status->buz == Si2183_SCAN_STATUS_RESPONSE_BUZ_BUSY) {
				 if ((count% 20==19) && (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
					 goto _exit;
				 }
				 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);
				 msleep(200);
				 if(++count%20==19) {
					 dprintk("here count=%d\n", count);
				 }
		 }

		 if(si2183_scan_action(client, bs->action, bs->seek_freq))
			 goto _exit;

	 }
	 bs-> action = Si2183_SCAN_CTRL_CMD_ACTION_RESUME;

	 /* The actual search loop... */
	 for (;;) {
		 bool scanint;
		 si2183_check_interaction(client, &scanint);
		 //dprintk("ddint=%d scanint=%d err=%d cts=%d\n", ddint, scanint, err, cts);
		 //si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_CLEAR, &scan_status);//test
		 if((kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			 goto _exit;
		 }


		 if ( (scanint == 1) ) {
			 /* There is an interaction with the FW, refresh the timeoutStartTime */
			 //front_end->timeoutStartTime = system_time();
			 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_CLEAR, scan_status);
			 bs->skip_resume = false;
			 //buzyStartTime = system_time();
			 while (scan_status->buz == 1) {
				 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);

				 if ((kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
					 goto _exit;
				 }
			 }
			 dprintk("SCAN status=%d\n", scan_status->scan_status);
			 switch (scan_status->scan_status) {
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_TUNE_REQUEST: {
				 int old_seek_freq= bs->seek_freq;
				 bs->seek_freq = scan_status->rf_freq;
				 if(fe->ops.tuner_ops.set_bandwidth) {
					 fe->ops.tuner_ops.set_frequency(fe, bs->seek_freq);
				 } else if (fe->ops.tuner_ops.set_frequency_and_bandwidth) {
					 fe->ops.tuner_ops.set_frequency_and_bandwidth(fe, bs->seek_freq, bs->bandwidth);
				 }

				 dprintk("SCAN tune request freq=%d, was %d\n", bs->seek_freq, old_seek_freq);
				 //*freq = front_end->rangeMin = seek_freq;
				 /* as we will not lock in less than min_lock_time_ms, wait a while... */
				 //system_wait(min_lock_time_ms);
					msleep(100);
					break;
			 }
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_DIGITAL_CHANNEL_FOUND: {
				 s32 standard = scan_status->modulation;
				 s32 symbol_rate;
				 s32 frequency = scan_status->rf_freq;
				 symbol_rate = scan_status->symb_rate*1000;

					/* When locked, clear scanint before returning from SeekNext, to avoid seeing it again on the 'RESUME', with fast i2c platforms */
				 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_CLEAR, scan_status);
				 p->delivery_system = si2183_delsys(standard);
				 state->delivery_system = p->delivery_system;
				 dprintk("FOUND freq=%d symrate=%d standard=%d => %d\n", frequency, symbol_rate, standard, p->delivery_system);
					p->frequency = frequency;
					p->symbol_rate = symbol_rate;
					si2183_read_status(fe, status);
					found= true;
					goto _exit;
					break;
				}
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_ERROR:
				 dprintk("ERROR\n");
				 goto _exit;
				 break;

			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_SEARCHING:
				 dprintk("searching count\n");
				 bs->skip_resume = true;
						break;
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_ENDED:
				 dprintk("ended\n");
				 goto _exit;
				 break;

			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_DEBUG:
				 dprintk("DEBUG!!!!!!!!!!!!!!! type=%d: %s\n", scan_status->symb_rate,
								 scan_status->symb_rate == 4 ? "spectrum":
								 scan_status->symb_rate == 9 ? "trylock": "other");

				 break;
			 default: {
				 dprintk("unknown scan_status %d\n", scan_status->scan_status);
				 bs->skip_resume = true;
				 break;
			 }
			 }
			 dprintk("here skip_resume=%d\n", bs->skip_resume);
			 if (bs->skip_resume == false) {
				 if(si2183_scan_action(client, bs->action, bs->seek_freq))
					 goto _exit;

			 }
		 }
			/* Check status every 100 ms */
			msleep(100);
	 }
	 // p->scan_start_frequency = frequency;
 }

 _exit:
 if(!found) {
	 si2183_scan_abort(client);
	 *status = FE_TIMEDOUT;
	 	dprintk("Signaling DONE\n");
 } else {
	 p->scan_start_frequency = p->frequency;
	 *status |= FE_HAS_LOCK;
 }

 return 0;
}

int si2183_spectrum_start(struct dvb_frontend *fe,
																 struct dtv_fe_spectrum* s,
																 unsigned int *delay, enum fe_status *status)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *state = i2c_get_clientdata(client);

	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	bool is_sat =1;
	struct spectrum_scan_state* ss = &state->scan_state;

	int i;
	u32 start_frequency = p->scan_start_frequency;
	u32 end_frequency = p->scan_end_frequency;
	//u32 bandwidth = end_frequency-start_frequency; //in kHz
	//uint32_t frequency;
	uint32_t resolution = (p->scan_resolution>0) ? p->scan_resolution: 1000; //in kHz
	uint32_t bandwidth = resolution; //in kHz
	u32 num_freq = (p->scan_end_frequency-p->scan_start_frequency+ resolution-1)/resolution;
	s32 frequency;

	is_sat = (p->delivery_system == SYS_DVBS ||
								 p->delivery_system == SYS_DVBS2 ||
								 p->delivery_system == SYS_DSS);

	is_sat=true;
	si2183_stop_task(fe);
	dprintk("num_freq=%d\n", num_freq);
	if(num_freq==0) //TODO: temporary
		num_freq=8192;
	s->num_freq = num_freq;
	ss->spectrum_len = num_freq;
	ss->spectrum_max_len = ss->spectrum_len;
	ss->freq = kzalloc(ss->spectrum_len * (sizeof(ss->freq[0])), GFP_KERNEL);
	ss->spectrum = kzalloc(ss->spectrum_len * (sizeof(ss->spectrum[0])), GFP_KERNEL);
	if (!ss->freq || !ss->spectrum) {
		return -ENOMEM;
	}
	if(!fe->ops.tuner_ops.set_frequency_and_bandwidth ||
		 !(fe->ops.tuner_ops.set_frequency && fe->ops.tuner_ops.set_frequency)) {
		dprintk("No tuner support %p %p %p\n", fe->ops.tuner_ops.set_frequency_and_bandwidth,
						fe->ops.tuner_ops.set_frequency,fe->ops.tuner_ops.set_frequency);
	 return -EINVAL; //TODO
	}


	p->frequency = p->scan_start_frequency;

	 p->symbol_rate = resolution*1000;
	 if(is_sat) {
		 p->modulation = QPSK;
		 p->delivery_system = SYS_DVBS; //to avoid setting stream_id
		 p->bandwidth_hz = 4000000; //perhaps not needed
		 p->symbol_rate = 4000000;
	 } else {
		 p->modulation = QAM_16;
		 p->bandwidth_hz = 1000000; //perhaps not needed
		 p->hierarchy = HIERARCHY_AUTO;
		 p->delivery_system=SYS_DVBT; //to avoid setting stream_id
	 }
	 if(fe->ops.tuner_ops.set_bandwidth) {
		 fe->ops.tuner_ops.set_bandwidth(fe, bandwidth);
	 }
	 si2183_set_frontend(fe); //select rf input, delivery system, ...


	 //state->tuner_bw = resolution;
	s->scale = FE_SCALE_DECIBEL; //in units of 0.001dB

	//	p->algorithm = ALGORITHM_NONE;
	//p->symbol_rate = frequency_step; //set bandwidth equal to frequency_step
	dprintk("demod: range=[%d,%d]kHz num_freq=%d resolution=%dkHz bw=%dkHz\n",
					start_frequency, end_frequency,
					num_freq, resolution, bandwidth);

	 /*we assume the following is set:
		 p->delivery_system;
		and set some other params
	 */
	 //p->frequency =0;
	 p->symbol_rate = resolution*1000;
	 if(is_sat) {
		 p->modulation = QPSK;
		 p->delivery_system = SYS_DVBS; //to avoid setting stream_id
	 } else {
		 p->modulation = QAM_16;
		 p->bandwidth_hz = 1000000; //perhaps not needed
		 p->hierarchy = HIERARCHY_AUTO;
		 p->delivery_system=SYS_DVBT; //to avoid setting stream_id
	 }
	 if(fe->ops.tuner_ops.set_bandwidth) {
		 fe->ops.tuner_ops.set_bandwidth(fe, bandwidth);
	 }
	 si2183_set_frontend(fe); //select rf input, delivery system, ...
#ifdef TODO2
	 si2183_read_status(fe, status);
	 #endif
	 dprintk("here num_freq=%d\n", num_freq);
	 for (i = 0; i < num_freq; i++) {
		 dprintk("ITER i=%d\n", i);
		 if ((i% 20==19) && (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			 dprintk("exiting on should stop i=%d\n", i);
			 break;
		 }
		 ss->freq[i]= start_frequency +i*resolution;
		 frequency = ss->freq[i];
		 p->frequency = frequency;
#if 0
		 si2183_set_frontend(fe); //select rf input, delivery system, ...
#else
		 if(fe->ops.tuner_ops.set_bandwidth) {
			 fe->ops.tuner_ops.set_frequency(fe, frequency);
		 } else if (fe->ops.tuner_ops.set_frequency_and_bandwidth) {
			 fe->ops.tuner_ops.set_frequency_and_bandwidth(fe, frequency, bandwidth);
		 }
#endif
		 //todo: check if we need to pause
		 usleep_range(12000, 13000);
		 ss->spectrum[i] = si2183_narrow_band_signal_power_dbm(fe);
	 }
	 *status = FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	return 0;

 }




int si2183_spectrum_get(struct dvb_frontend *fe, struct dtv_fe_spectrum* user)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *state = i2c_get_clientdata(client);

	int error=0;
	dprintk("num_freq: user=%d kernel=%d\n", user->num_freq, state->scan_state.spectrum_len);
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
	return error;
}

enum fe_delivery_system si2183_delsys(int val)
{
	switch(val) {
	case 8: return SYS_DVBS;
	case 9: return SYS_DVBS2;
	case 2: return SYS_DVBT;
	case 3: return SYS_DVBC_ANNEX_A;
	case 4: return SYS_ISDBT;
	case 7: return SYS_DVBT2;
	case 10: return SYS_DSS;
	case 11: return SYS_DVBC2;
	}
	return SYS_UNDEFINED;
}

enum fe_modulation si2183_modulation(int val)
{
	switch(val) {
	case 20: return APSK_16;
	case 24: return APSK_16L;
	case 21: return APSK_32;
	case 26: return APSK_32;
	case 25: return APSK_32L;
	case 23: return APSK_8L;
	case 14: return PSK_8;
	case 3: return QPSK;
	}
	return -1;
}

enum fe_rolloff si2183_rolloff(int val)
{
	switch(val) {
	case 0: return ROLLOFF_35;
	case 6: return ROLLOFF_5;
	case 5: return ROLLOFF_10;
	case 4: return ROLLOFF_15;
	case 2: return ROLLOFF_20;
	case 1: return ROLLOFF_25;
	case 3: return ROLLOFF_AUTO;
	}
	return ROLLOFF_AUTO;
}


enum fe_code_rate si2183_code_rate(int val)
{
	switch(val) {
	case 1: return FEC_1_2;
	case 2: return FEC_2_3;
	case 3: return FEC_3_4;
	case 4: return FEC_4_5;
	case 5: return FEC_5_6;
	case 6: return FEC_6_7;
	case 8: return FEC_8_9;
	case 9: return FEC_9_10;
	case 10: return FEC_1_3;
	case 11: return FEC_1_4;
	case 12: return FEC_2_5;
	case 13: return FEC_3_5;
	}
	return FEC_AUTO;
}
