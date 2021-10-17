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

extern int si2183_verbose;

#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt), __func__, __LINE__, ##arg)

#define vprintk(fmt, arg...)																					\
	if(si2183_verbose) printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

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
		struct si2183_dev* state = i2c_get_clientdata(client);
	int ret;


	mutex_lock(&state->base->i2c_mutex);
	ret = si2183_get_register_unlocked(client, address, value_ret);
	mutex_unlock(&state->base->i2c_mutex);

	return ret;
}


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


static int si2183_set_property_unlocked(struct i2c_client *client, u16 property, u16 data)
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
		 dprintk("set property failed: ret=%d", ret);
	 }
	 //one data byte is in cmd.args[1]
	 return ret;
 }

static int si2183_set_property(struct i2c_client *client, u16 property, u16 data)
{
	struct si2183_dev* state = i2c_get_clientdata(client);
	int ret;
	mutex_lock(&state->base->i2c_mutex);
	ret = si2183_set_property_unlocked(client, property, data);
	mutex_unlock(&state->base->i2c_mutex);

	return ret;
}



static int si2183_get_iq_sample_unlocked(struct dvb_frontend* fe, s16* ival, s16* qval)
{
	struct i2c_client* client = fe->demodulator_priv;
	struct dtv_frontend_properties* p = &fe->dtv_property_cache;

	int ret=-1;
  unsigned int  value;
  switch (p->delivery_system)
  {
    case SYS_DVBC2:
    case SYS_DVBT :
    case SYS_DVBT2: {
#ifdef USE_RF_IQ
			ret = si2183_get_register_unlocked(client, 0x0edc1f, &value); //rx_iq
#else
			ret = si2183_get_register_unlocked(client, 0x072c13, &value); //symb_iq
#endif
			*ival = ((value >>10) << 6); //range: -32768 ... +32767; 10 bit per component
			*qval = (value << 6); //range: -32768 ... +32767
      break;
    }
    case SYS_DVBC_ANNEX_A : {
			ret = si2183_get_register_unlocked(client, 0x08e81f, &value); //iq_symb
			*ival = (value >> 16); //range: -32768 ... +32767, 16 bit per componentn
			*qval = value; //range: -32768 ... +32767
      break;
    }
    case SYS_DSS  :
    case SYS_DVBS :
    case SYS_DVBS2: {
			ret = si2183_get_register_unlocked(client, 0x05f80f, &value); //iq_demod_sat
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



s32 si2183_signal_power_dbm(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_cmd cmd;
	u16 agc1, agc2;
	int ret=0;
	si2183_cmd_str(&cmd, "\x8a\x00\x00\x00\x00\x00", 6, 3); //parameters mean: no change to agc, only reading it
	ret = si2183_cmd_execute(client, &cmd);
	if (ret<0) {
		dprintk("signal_power fe%d cmd_exec failed=%d", fe->id, ret);
		dev_err(&client->dev, "signal_power fe%d cmd_exec failed=%d", fe->id, ret);
		return 0;
	}

	agc1 = cmd.args[1];
	agc2 = cmd.args[2];
	dprintk("AGC: %d %d", agc1, agc2);
	//The following works for tbs6504, but value depends on tuner/demod combination
	return (280-6*(int)agc1 )*100;
}


int si2183_isi_scan(struct dvb_frontend* fe, int num_isi)
{
	struct i2c_client* client = fe->demodulator_priv;
	struct si2183_dev* state = i2c_get_clientdata(client);
	struct dtv_frontend_properties* p = &fe->dtv_property_cache;
	int i, j;
	u32 mask;
	u8 isi_id;
	u8 constell;
	u8 code_rate;
	struct si2183_cmd cmd;
	int ret;
	mutex_lock(&state->base->i2c_mutex);
	memset(&p->isi_bitset[0], 0, sizeof(p->isi_bitset));
	for(i=0; i < num_isi; ++i) {
		cmd.args[0] = 0x72;
		cmd.args[1] = (u8) i;
		cmd.wlen = 2;
		cmd.rlen = 4;
		ret = si2183_cmd_execute_unlocked(client, &cmd);
		if (ret) {
			dprintk("read_isi cmd_exec failed");
			break;
		}
		isi_id = cmd.args[1];
		constell = cmd.args[2] & 0x3f;
		code_rate = cmd.args[3] & 0x1f;
		vprintk("[%d] is_id=%d constell=%d code_rate=%d", i, isi_id, constell, code_rate);
		j = isi_id/32;
		mask = ((uint32_t)1)<< (isi_id%32);
		p->isi_bitset[j] |= mask;
	}
	mutex_unlock(&state->base->i2c_mutex);
	return 0;
}


int si2183_stop_task(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *state = i2c_get_clientdata(client);
	struct spectrum_scan_state* ss = &state->scan_state;
	struct constellation_scan_state* cs = &state->constellation_scan_state;
	//dprintk("called ss=%p", ss);
	//dprintk("called %p %p %p", ss->freq, ss->spectrum, cs->samples);
	if(ss->freq) {
		kfree(ss->freq);
	}
	if(ss->spectrum)
		kfree(ss->spectrum);
	if(cs->samples)
		kfree(cs->samples);
	memset(ss, 0, sizeof(*ss));
	memset(cs, 0, sizeof(*cs));
	vprintk("Freed memory");
	return 0;
}


int si2183_constellation_start(struct dvb_frontend *fe, struct dtv_fe_constellation* user, int max_num_samples)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev* state = i2c_get_clientdata(client);
	struct constellation_scan_state* cs = &state->constellation_scan_state;
	int num_samples = user->num_samples;
	if(num_samples > max_num_samples)
		num_samples = max_num_samples;
	vprintk("demod %d: constellation samples=%d/%d/%d constel_select=%d", state->nr, user->num_samples,
					cs->samples_len, max_num_samples, (int)user->constel_select);
	if(cs->samples_len != num_samples) {
		if(cs->samples) {
			vprintk("samples: Calling kfree %p", cs->samples);
			kfree(cs->samples);
			vprintk("samples: Calling kfree %p done", cs->samples);
			cs->samples = NULL;
		}
		cs->samples_len = num_samples;
		if(cs->samples_len >0) {
			vprintk("samples: Calling kzalloc");
			cs->samples = kzalloc(cs->samples_len * (sizeof(cs->samples[0])), GFP_KERNEL);
			vprintk("samples: Called kzalloc %p", cs->samples);
			if (!cs->samples) {
				cs->samples_len = 0;
				return  -ENOMEM;
			}
		}
	}
	cs->constel_select =  user->constel_select;
	cs->num_samples = 0;

	vprintk("%p len=%d", cs->samples,  cs->samples_len);

	mutex_lock(&state->base->i2c_mutex);
	for (cs->num_samples = 0; cs->num_samples < cs->samples_len; ++cs->num_samples) {
		if ((cs->num_samples% 20==19) && (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			dprintk("exiting on should stop");
			break;
		}
		si2183_get_iq_sample_unlocked(fe, &cs->samples[cs->num_samples].imag, &cs->samples[cs->num_samples].real);
	}
	mutex_unlock(&state->base->i2c_mutex);
	vprintk("demod %d: constellation retrieved samples=%d/%d", state->nr, cs->num_samples, cs->samples_len);
	return 0;
}

int si2183_constellation_get(struct dvb_frontend *fe, struct dtv_fe_constellation* user)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev* state = i2c_get_clientdata(client);
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

static inline
u32 Si2183_convert_to_ulong(const unsigned char* buffer, unsigned char shift, unsigned long mask) {
	return ((( ( (unsigned long)buffer[0]) | ((unsigned long)buffer[1] << 8) | ((unsigned long)buffer[2]<<16)
						 | ((unsigned long)buffer[3]<<24)) >> shift) & mask );
}

static inline
u32 Si2183_convert_to_uint (const unsigned char* buffer, unsigned char shift, unsigned int mask) {
	return (( ( (unsigned int)buffer[0]) | (((unsigned int)buffer[1]) << 8) >> shift) & mask);
}

static inline
s16 Si2183_convert_to_int (const unsigned char* buffer, unsigned char shift, unsigned int mask) {
	return (( ( (unsigned int)buffer[0]) | (((unsigned int)buffer[1]) << 8) >> shift) & mask);
}



int si2183_scan_status(struct i2c_client *client, u8 intack, struct si2183_scan_status_t*s)
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
		 dprintk("set property failed: ret=%d", ret);
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

	 vprintk("buzint=%d reqint=%d buz=%d req=%d scan_status=%d rf_freq=%d symb_rate=%d modulation=%d",
					s->buzint, s->reqint, s->buz, s->req, s->scan_status, s->rf_freq, s->symb_rate, s->modulation);
	 return ret;
}



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
		 dprintk("check status failed: ret=%d", ret);
		 *scanint = false;
		 return ret;
	 }

	 *scanint = (cmd.args[0] >>1)&0x1;
	 return ret;
}




static int si2183_scan_abort(struct i2c_client *client)
{
	s32 tuned_rf_freq =0;
	struct si2183_cmd cmd = {
		.args={Si2183_SCAN_CTRL_CMD, Si2183_SCAN_CTRL_CMD_ACTION_ABORT, 0x00, 0x00, tuned_rf_freq & 0xff ,
					 (tuned_rf_freq>>8)&0xff, (tuned_rf_freq>>16)&0xff, (tuned_rf_freq>>24)&0xff},
		.wlen=8,
		.rlen=1};
	int ret=si2183_cmd_execute(client, &cmd);
	if(ret) {
		dprintk("scan_abort failed");
	}
	return ret;
}

#if 0
static inline int fff(int freq) {
	return freq + 10700000-950000;
}
#else
static inline int fff(int freq) {
	return freq;
}
#endif

/*
	init = 1: start the scan at the search range specified by the user
	init = 0: start the scan just beyond the last found frequency
*/
static int si2183_set_scan_limits(struct i2c_client *client,	struct dtv_frontend_properties *p)
{
	int ret=0;
	u16 scan_fmin= (((uint32_t)p->scan_start_frequency)*1000)>>16; //unsigned in official code
	u16 scan_fmax=(((uint32_t)p->scan_end_frequency)*1000)>>16;
	s32 sym_rate_min=100000/1000;
	s32 sym_rate_max=45000000/1000;

	vprintk("MINMAX: %d %d // %d %d", fff(p->scan_start_frequency),
					fff(p->scan_end_frequency), scan_fmin, scan_fmax);
	ret |= si2183_set_property(client, Si2183_SCAN_FMIN_PROP_CODE, scan_fmin);
	ret |= si2183_set_property(client, Si2183_SCAN_FMAX_PROP_CODE, scan_fmax);
	ret |= si2183_set_property(client, Si2183_SCAN_SYMB_RATE_MIN_PROP_CODE, sym_rate_min);
	ret |= si2183_set_property(client, Si2183_SCAN_SYMB_RATE_MAX_PROP_CODE, sym_rate_max);
	if(ret)
		dprintk("FAILED to set limits");
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
		dprintk("FAILED to enable interaction");
	return ret;
}

static int si2183_set_scan_bandwidth(struct i2c_client *client, s32 bandwidth)
{
	s32 bw = bandwidth ? bandwidth/1000 : 0x8; //only important for dvb-t 0x8 means default
	s32 modulation = 15; //autodetect
	s32 invert_spectrum = 0; //not inverted
	s32 auto_detect = Si2183_DD_MODE_PROP_AUTO_DETECT_AUTO_DVB_S_S2; //detect any dvb-s

	s32 dd_mode = (bw& 0x0f) << 0 |
			(modulation & 0x0f) << 4 |
		(invert_spectrum & 0x01) << 8 |
		(auto_detect & 0x07 ) << 9;
	int ret= si2183_set_property(client, Si2183_DD_MODE_PROP_CODE, dd_mode);
	if(ret)
		dprintk("Error setting bandwidth");
	return ret;
}

static int si2183_set_scan_mode(struct i2c_client *client, s32 debug)
{
	s32 analog_detect = 1; //enabled
	s32 reserved1 = 0;
	s32 reserved2 = 12;
	s32 scan_debug = debug; //0x03; use 3 for spectrum scan?
	//Si2183_L1_SetProperty2(front_end->demod, Si2183_SCAN_SAT_CONFIG_PROP_CODE);
	s32 sat_config = (analog_detect & 0x01) << 0 |
		(reserved1 & 0x1f ) << 1 |
		(reserved2 & 0x7f ) << 6 |
		(scan_debug & 0x07 ) << 13 ;
	int ret=si2183_set_property(client, Si2183_SCAN_SAT_CONFIG_PROP_CODE, sat_config);
	if(ret)
		dprintk("Error setting scan mode");
	else
		vprintk("set_scan_code=0x%x", sat_config);
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
		 dprintk("Error setting scan mode");
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
	dprintk(" Si2183_SCAN_CTRL_CMD_ACTION_START freq=%dkHz", seek_freq);
	//action = Si2183_SCAN_CTRL_CMD_ACTION_RESUME;
	if(ret)
		dprintk("scan action failed");
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

	vprintk("seek_freq=%d bandwidth=%d", 	fff(bs->seek_freq), bs->bandwidth);

	if(fe->ops.tuner_ops.set_bandwidth)
		fe->ops.tuner_ops.set_bandwidth(fe, bs->bandwidth);

	si2183_scan_abort(client);
	si2183_set_scan_limits(client, p);
	si2183_enable_interaction(client);
	si2183_set_scan_bandwidth(client, 0);

	si2183_set_scan_mode(client, 00 /*use 3 for spectrum*/);
	si2183_restart(client);

	/* Load blindscan status at start */
	si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);
 return 0;
}


static int si2183_spectrum_init(struct dvb_frontend *fe, struct blindscan_state * bs)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct si2183_scan_status_t* scan_status = &bs->scan_status;
	struct si2183_dev *state = i2c_get_clientdata(client);
	struct spectrum_scan_state* ss = &state->scan_state;

	//uint32_t resolution = (p->scan_resolution>0) ? p->scan_resolution: 52050; //in kHz
	uint32_t resolution =   106600/2048;
	u32 num_freq = (p->scan_end_frequency-p->scan_start_frequency+ resolution-1)/resolution;

	bool is_sat = (p->delivery_system == SYS_DVBS ||
								 p->delivery_system == SYS_DVBS2 ||
								 p->delivery_system == SYS_DSS);

	is_sat=true;

	memset(bs, 0, sizeof(struct blindscan_state));
	si2183_stop_task(fe); //clean up

	ss->spectrum_len = num_freq;
	ss->spectrum_max_len = ss->spectrum_len;
	ss->freq = kzalloc(ss->spectrum_len * (sizeof(ss->freq[0])), GFP_KERNEL);
	ss->spectrum = kzalloc(ss->spectrum_len * (sizeof(ss->spectrum[0])), GFP_KERNEL);
	if (!ss->freq || !ss->spectrum) {
		return -ENOMEM;
	}
	dprintk("num_freq=%d", num_freq);
	ss->spectrum_len = 0;
	ss->spectrum_max_len = num_freq;


	bs->bandwidth = 40000;
	bs->start_resume = 1;
	bs->skip_resume = 0;

	bs->action = Si2183_SCAN_CTRL_CMD_ACTION_START;
	bs->seek_freq = p->frequency;
 //s32 tuned_rf_freq; // = (950000/65536)*1000;//same as start frequency

	vprintk("seek_freq=%d bandwidth=%d", 	bs->seek_freq, bs->bandwidth);

	if(fe->ops.tuner_ops.set_bandwidth)
		fe->ops.tuner_ops.set_bandwidth(fe, bs->bandwidth);

	si2183_scan_abort(client);
	si2183_set_scan_limits(client, p);
	si2183_enable_interaction(client);
	si2183_set_scan_bandwidth(client, 0);

	si2183_set_scan_mode(client, 0xf /*use 3 for spectrum*/);
	si2183_restart(client);

	/* Load blindscan status at start */
	si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);
 return 0;
}

static int si2183_spectrum_next(struct dvb_frontend *fe, struct blindscan_state * bs)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct si2183_scan_status_t* scan_status = &bs->scan_status;

	bs->bandwidth = 40000;
	bs->start_resume = 1;
	bs->skip_resume = 0;

	bs->action = Si2183_SCAN_CTRL_CMD_ACTION_START;
	bs->seek_freq = p->scan_start_frequency;

	si2183_scan_abort(client);
	si2183_set_scan_limits(client, p);
	si2183_enable_interaction(client);
	si2183_set_scan_bandwidth(client, 0);

	si2183_set_scan_mode(client, 0xf /*use 3 for spectrum*/);
	si2183_restart(client);

	/* Load blindscan status at start */
	si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);
 return 0;
}

static int si2183_get_spectrum_scan_fft_one_band(struct dvb_frontend *fe, u32* freq, s32* rf_level,
																								 int max_fft_size, int tuned_freq);

int si2183_scan_sat_(struct dvb_frontend *fe, bool init,
										unsigned int *delay, enum fe_status *status, bool dont_retune)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct si2183_dev *state = i2c_get_clientdata(client);
	struct blindscan_state * bs = &state->blindscan_state;
	struct si2183_scan_status_t* scan_status = &bs->scan_status;
	bool found = false;
	/*set SAT agc*/
	si2183_set_sat_agc(client);


 //s32 tuned_rf_freq; // = (950000/65536)*1000;//same as start frequency
 //s32 seek_freq=0;
	vprintk("init=%d resume=%d start=%dkHz end=%dkHz", init, bs->start_resume,
					fff(p->scan_start_frequency), fff(p->scan_end_frequency)); //init=1 start=950000kHz end=1950000kHz
 if(init) {
	 si2183_scan_sat_start(fe, bs);
 }



 // while(1)
	 {
		 /* Checking blindscan status before issuing a 'start' or 'resume' */
		 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);
	 if(bs->start_resume) {
		 int count=0;
		 while (scan_status->buz == Si2183_SCAN_STATUS_RESPONSE_BUZ_BUSY) {
				 if ((count% 20==19) && (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
					 goto _exit;
				 }
				 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);
				 msleep(200);
				 if(++count%20==19) {
					 dprintk("count=%d", count);
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
		 if((kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			 goto _exit;
		 }


		 if ( (scanint == 1) ) { // Si2183_STATUS_SCANINT_TRIGGERED
			 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_CLEAR, scan_status);
			 bs->skip_resume = false;

			 while (scan_status->buz == 1) {
				 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);

				 if ((kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
					 goto _exit;
				 }
			 }
			 dprintk("SCAN status=%d", scan_status->scan_status);
			 switch (scan_status->scan_status) {
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_TUNE_REQUEST: {
				 int old_seek_freq= bs->seek_freq;
				 vprintk("SCAN tune request freq=%d, was %d delta=%d", fff(scan_status->rf_freq), fff(old_seek_freq),
								 scan_status->rf_freq - old_seek_freq);
				 if(dont_retune) {
					 scan_status->rf_freq = old_seek_freq;
					 vprintk("rf_freq=%d", fff(scan_status->rf_freq));
				 }
				 bs->seek_freq = scan_status->rf_freq;
				 state->demod_tuned_freq= bs->seek_freq;
				 state->tuner_tuned_freq= bs->seek_freq;
				 if(fe->ops.tuner_ops.set_frequency) {
					 fe->ops.tuner_ops.set_frequency(fe, bs->seek_freq);

				 } else if (fe->ops.tuner_ops.set_frequency_and_bandwidth) {
					 fe->ops.tuner_ops.set_frequency_and_bandwidth(fe, bs->seek_freq, bs->bandwidth);
				 }
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
				 //state->delivery_system = p->delivery_system;
				 vprintk("FOUND freq=%d symrate=%d standard=%d => %d", fff(frequency), symbol_rate, standard, p->delivery_system);
				 if(!dont_retune) {
					 p->frequency = frequency;
					 p->scan_start_frequency = frequency + symbol_rate/2000;
				 }
					p->symbol_rate = symbol_rate;
					si2183_read_status(fe, status);
					vprintk("FOUND1 freq=%d symrate=%d", fff(p->frequency), p->symbol_rate);
					{
						//compensate for the AFC measurement, as this is already included in the received frequency
						int delta = p->frequency - state->demod_tuned_freq;
						p->frequency = frequency - delta;
						state->demod_tuned_freq = frequency -delta;
						vprintk("FOUND2 freq=%d symrate=%d", fff(p->frequency), p->symbol_rate);
					}
					found= true;
					goto _exit;
					break;
				}
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_ERROR:
				 vprintk("ERROR");
				 goto _exit;
				 break;

			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_SEARCHING:
				 vprintk("searching count");
				 bs->skip_resume = true;
						break;
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_ENDED:
				 vprintk("ended");
				 goto _exit;
				 break;

			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_DEBUG:
				 vprintk("DEBUG type=%d: %s", scan_status->symb_rate,
								 scan_status->symb_rate == 4 ? "spectrum":
								 scan_status->symb_rate == 9 ? "trylock": "other");

				 break;
			 default: {
				 vprintk("unknown scan_status %d", scan_status->scan_status);
				 bs->skip_resume = true;
				 break;
			 }
			 }
			 vprintk("skip_resume=%d", bs->skip_resume);
			 if (bs->skip_resume == false) {
				 vprintk("action=%d seek_freq=%d", bs->action, fff(bs->seek_freq));
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
	 vprintk("loop exit");
 if(!found) {
	 si2183_scan_abort(client);
	 *status = FE_TIMEDOUT;
	 	vprintk("Signaling DONE");
 } else {
	 //p->scan_start_frequency = p->frequency;
	 *status |= FE_HAS_LOCK;
 }

 return 0;
}


int si2183_scan_sat(struct dvb_frontend *fe, bool init, unsigned int *delay, enum fe_status *status)
{
	return si2183_scan_sat_(fe, init, delay, status, false);
}

/*
	Retrieve fft data for basenad frequencies starting at 0 and ending at 70.475700Mhz
	In input:
  caller must allocate freq[max_fft_size] and rf_level[max_fft_size]
	max_fft_size >= 677
	tuned_freq: the RF frequency of the center frequency of the band in kHz
	rf_rssi: RF level of the input signal, as estimated from AGC
	Output: rf_level: final RF level, in units of 0.001dB
	       freq: RF frequencies in kHz
  Returns number of samples (normally 677)
 */
static int si2183_get_spectrum_scan_fft_one_band(struct dvb_frontend *fe, u32* freq, s32* rf_level,
																								 int max_fft_size, int tuned_freq)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *state = i2c_get_clientdata(client);

	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	bool is_sat =1;
	int idx = 0;
	s32 nb_word;
	s32  gp_reg32_0;
	s32  gp_reg32_1;
	s32  gp_reg32_2;
	s32  gp_reg16_1;
	s32  gp_reg16_2;
	s32  gp_reg16_3;
	s32 rf_backoff0;
	s32 rf_backoff1;
	s32 rf_backoff2;
	s32 analysis_bw;
	s32 tune_freq;
	s32 spectrum_stop_freq;
	s32	spectrum_start_freq;
	s32 currentRF;
	s32 inter_carrier_space;
	//s32 standard_specific_spectrum_offset;
	//s32 standard_specific_spectrum_scaling;
	s32 standard_specific_freq_unit;
	s32 if_freq_shift;
	s32 dcom_read;
	s32 read_data;
	s32 msb;
	s32 lsb;
	int w;
	int ret=0;

	//u32 bandwidth = end_frequency-start_frequency; //in kHz
	//uint32_t frequency;
	is_sat = (p->delivery_system == SYS_DVBS ||
						p->delivery_system == SYS_DVBS2 ||
						p->delivery_system == SYS_DSS);

	is_sat =true; //TODO: for SYS_AUTO we need to still know system

	mutex_lock(&state->base->i2c_mutex);

	//STOP DSP
	ret = si2183_set_register_unlocked(client, ADDR3(0,8,11), 0x00000000);
	if(ret)
		dprintk("ret=%d", ret);
	ret = si2183_set_register_unlocked(client, ADDR3(31, 16, 11), 0xc0000000);
	if(ret)
		dprintk("ret=%d", ret);
	ret = si2183_set_register_unlocked(client, ADDR3(31, 20, 11), 0x90000000);
	if(ret)
		dprintk("ret=%d", ret);

	ret = si2183_set_register_unlocked(client, ADDR3(31, 24, 11), 0x000004f0);
 	if(ret)
		dprintk("ret=%d", ret);

	ret = si2183_get_register_unlocked(client, ADDR3(15, 80, 11), &nb_word);
	if(ret)
		dprintk("ret=%d", ret);

	ret = si2183_get_register_unlocked(client, ADDR3(31, 68, 11), &gp_reg32_0);
	if(ret)
		dprintk("ret=%d", ret);

	ret = si2183_get_register_unlocked(client, ADDR3(31, 72, 11), &gp_reg32_1);
	if(ret)
		dprintk("ret=%d", ret);

	vprintk("registers: nb_word=%d gp_reg32_0=%d gp_reg32_1=%d ret=%d",
					nb_word, gp_reg32_0, gp_reg32_1, ret);

	ret = si2183_get_register_unlocked(client, ADDR3(31, 76, 11), &gp_reg32_2);
	if(ret)
		dprintk("ret=%d", ret);

	ret = si2183_get_register_unlocked(client, ADDR3(15, 82, 11), &gp_reg16_1);
	if(ret)
		dprintk("ret=%d", ret);
	rf_backoff0 = gp_reg16_1;

	ret = si2183_get_register_unlocked(client, ADDR3(15, 84, 11), &gp_reg16_2);
	if(ret)
		dprintk("ret=%d", ret);
	rf_backoff1 = gp_reg16_2;

	ret = si2183_get_register_unlocked(client, ADDR3(15, 86, 11), &gp_reg16_3);
	if(ret)
		dprintk("ret=%d", ret);
	rf_backoff2 = gp_reg16_3;

	ret = si2183_get_register_unlocked(client, ADDR3(28, 92, 4), &if_freq_shift);
	if(ret)
		dprintk("ret=%d", ret);


	if (!is_sat) {
		inter_carrier_space = (1000000*64/(7*1024))*2;
		//standard_specific_spectrum_offset  =   0;
		//standard_specific_spectrum_scaling = 400;
		standard_specific_freq_unit = 1;
		analysis_bw = nb_word*inter_carrier_space;
		tune_freq           = gp_reg32_2*standard_specific_freq_unit;
		spectrum_stop_freq  = tune_freq + analysis_bw/2;
		spectrum_start_freq = spectrum_stop_freq - inter_carrier_space*nb_word;
	} else {
		inter_carrier_space = (106600000/2048)*2;
		//inter_carrier_space = 40000000/1024;
		//standard_specific_spectrum_offset  = 650;
		//standard_specific_spectrum_scaling =  75;
		standard_specific_freq_unit = 1024;
		analysis_bw = nb_word*inter_carrier_space; //Hz nb_word=677 /home/philips/blindscan/build/src/neumo-blindscan -c blindscan -U9 -C0 -pH -a0  --frontend 1
		//unicable analysis_bw = front_end->demod->prop->scan_sat_unicable_bw.scan_sat_unicable_bw*100000;
		tune_freq           = gp_reg32_2*standard_specific_freq_unit; //Hz
#if 0
		spectrum_stop_freq  = tune_freq + analysis_bw/2;
		spectrum_start_freq = spectrum_stop_freq - inter_carrier_space*nb_word;
#else
		spectrum_start_freq = 0 ;
		spectrum_stop_freq  = inter_carrier_space*nb_word;
#endif
#if 1
		vprintk("tuned_freq=%d if_freq_shift=%d tune_freq=%d start_freq=%d stop_freq=%d analysis_bw=%d",
						tuned_freq,
						if_freq_shift, tune_freq, spectrum_start_freq, spectrum_stop_freq, analysis_bw);
#endif
		tuned_freq -= analysis_bw/2000;
	}
	currentRF = spectrum_start_freq;

/* read spectrum data */

//Si2183_WRITE (front_end->demod, dcom_control_byte, (0x80000000 | (nb_word - 1)) );
	ret = si2183_set_register_unlocked(client, ADDR3(31, 16, 11), (0x80000000 | (nb_word - 1)));
	if(ret)
		dprintk("ret=%d", ret);

	// Si2183_WRITE (front_end->demod, dcom_addr        ,  gp_reg32_0);
	ret = si2183_set_register_unlocked(client, ADDR3(31, 20, 11), gp_reg32_0);
	if(ret)
		dprintk("ret=%d", ret);

	w=0;
	for (w=0; w < 579/*nb_word*/ ; ++w) {
		if ((w% 20==19) &&  (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			dprintk("exiting on should stop");
			break;
		}

		//Si2183_READ (front_end->demod , dcom_read);
		ret = si2183_get_register_unlocked(client, ADDR3(31, 64, 11), &dcom_read);
		if(ret)
			dprintk("ret=%d", ret);

		//Si2183_WRITE(front_end->demod , dcom_read_toggle, 1);
		ret = si2183_set_register_unlocked(client, ADDR3(0, 63, 11), 1);
		if(ret)
			dprintk("ret=%d", ret);

		read_data = dcom_read;
		msb = (read_data >> 16) & 0xffff;
		lsb = (read_data >>  0) & 0xffff;
		if ((w < nb_word-0)) {

			if(idx < max_fft_size) {
				freq[idx]= currentRF/1000 + tuned_freq;
				rf_level[idx] = read_data/0x6400 - 26605;
				idx++;
		}

		}
		currentRF = currentRF + inter_carrier_space;
	}
#if 0
	/* read spectrum additional data tables */
	for (i=0; i< 3; i++) {
		//Si2183_WRITE (front_end->demod, dcom_control_byte, (0x80000000 | (48 - 1)) );
		ret = si2183_set_register_unlocked(client, ADDR3(31, 16, 11), (0x80000000 | (48 - 1)));
	if(ret)
		dprintk("ret=%d", ret);


		//Si2183_WRITE (front_end->demod, dcom_addr        ,  gp_reg32_0 + i*48*4);
	ret = si2183_set_register_unlocked(client, ADDR3(31, 20, 11), gp_reg32_0 +  i*48*4);
	if(ret)
		dprintk("ret=%d", ret);

	nb_channel = (gp_reg32_1 >> (8*i)) & 0xFF;
	vprintk("run %d, %3d channels", i, nb_channel);
	w = 0;
	for(w=0; w < nb_channel ; ++w) {
		//Si2183_READ (front_end->demod , dcom_read);
		ret = si2183_get_register_unlocked(client, ADDR3(31, 64, 11), &dcom_read);
		if(ret)
			dprintk("ret=%d", ret);

		vprintk("channel %d, %2d : 0x%08x tune_freq %8d => %d", i, w, dcom_read, dcom_read, dcom_read*standard_specific_freq_unit/1000 + tuned_freq);

		//Si2183_READ (front_end->demod , dcom_read);
		ret = si2183_get_register_unlocked(client, ADDR3(31, 64, 11), &dcom_read);

		read_data = dcom_read;

		msb = (read_data >> 16) & 0xffff;
		lsb = (read_data >>  0) & 0xffff;
		vprintk(" 0x%08x offset %8d sr %8d  ", dcom_read, msb, lsb);


		//Si2183_READ (front_end->demod , dcom_read);
		ret = si2183_get_register_unlocked(client, ADDR3(31, 64, 11), &dcom_read);
		if(ret)
			dprintk("ret=%d", ret);

		vprintk(" 0x%08x variance %8d ", dcom_read, dcom_read);
	}
	}
#endif

	/* Start Si2183 DSP */
	ret = si2183_set_register_unlocked(client, ADDR3(31, 16, 11), 0xc0000000);
	if(ret)
		dprintk("ret=%d", ret);

	ret = si2183_set_register_unlocked(client, ADDR3(31, 20, 11), 0x90000000);
	if(ret)
		dprintk("ret=%d", ret);

	ret = si2183_set_register_unlocked(client, ADDR3(31, 24, 11),  0x00000000);
	if(ret)
		dprintk("ret=%d", ret);

	mutex_unlock(&state->base->i2c_mutex);
	vprintk("spectrum len now %d", idx);
	return idx;
}

int si2183_spectrum_start(struct dvb_frontend *fe,
																 struct dtv_fe_spectrum* s,
													unsigned int *delay, enum fe_status *status)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct si2183_dev *state = i2c_get_clientdata(client);
	struct blindscan_state * bs = &state->blindscan_state;
	struct spectrum_scan_state* ss = &state->scan_state;
	struct si2183_scan_status_t* scan_status = &bs->scan_status;
	bool found = false;
	int count=0;
	int i;
	u32 start_frequency = p->scan_start_frequency;
	u32 end_frequency = p->scan_end_frequency;
	//uint32_t resolution = (p->scan_resolution>0) ? p->scan_resolution: 52050; //in kHz
	uint32_t resolution =   (106600000/2048)*2/1000;
	uint32_t bandwidth = 40000; //in kHz
	u32 num_freq = (p->scan_end_frequency-p->scan_start_frequency+ resolution-1)/resolution;
	s32 frequency;
	bool is_sat = (p->delivery_system == SYS_DVBS ||
								 p->delivery_system == SYS_DVBS2 ||
								 p->delivery_system == SYS_DSS);
	int max_fft_size = 1024;
	u32* temp_freq;
	s32* temp_rf_level;
	int num_samples;
	vprintk("START=%d END=%d", start_frequency, end_frequency);
	ss->spectrum_max_len = num_freq;
	ss->spectrum_len = 0;

	if(!fe->ops.tuner_ops.set_frequency_and_bandwidth ||
		 !(fe->ops.tuner_ops.set_frequency && fe->ops.tuner_ops.set_frequency)) {
		dprintk("No tuner support %p %p %p", fe->ops.tuner_ops.set_frequency_and_bandwidth,
						fe->ops.tuner_ops.set_frequency,fe->ops.tuner_ops.set_frequency);
	 return -EINVAL; //TODO
	}


	ss->freq = kzalloc(ss->spectrum_len * (sizeof(ss->freq[0])), GFP_KERNEL);
	ss->spectrum = kzalloc(ss->spectrum_len * (sizeof(ss->spectrum[0])), GFP_KERNEL);
	temp_freq = kzalloc(max_fft_size * (sizeof(temp_freq[0])), GFP_KERNEL);
	temp_rf_level = kzalloc(max_fft_size * (sizeof(temp_rf_level[0])), GFP_KERNEL);
	if (!temp_rf_level || !temp_freq || !ss->freq || !ss->spectrum) {
		dprintk("ERROR spectrum_len=%d", ss->spectrum_len);
		return -ENOMEM;
	}

	is_sat=true;


	si2183_spectrum_init(fe, bs);
	while (scan_status->buz == Si2183_SCAN_STATUS_RESPONSE_BUZ_BUSY) {
		if ((count% 20==19) && (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
			goto _exit;
		}
		si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);
		msleep(20);
		if(++count%20==19) {
			vprintk("count=%d", count);
		}
	}
	if(si2183_scan_action(client, bs->action, bs->seek_freq)) {
		dprintk("Scan action failed");
		goto _exit;
	}

	bs-> action = Si2183_SCAN_CTRL_CMD_ACTION_RESUME;

	/* The actual search loop... */
	for (;ss->spectrum_len < ss->spectrum_max_len && start_frequency < end_frequency;) {
		bool scanint;
		si2183_check_interaction(client, &scanint);
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
			 vprintk("SCAN status=%d", scan_status->scan_status);
			 switch (scan_status->scan_status) {
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_TUNE_REQUEST: {
				 int old_seek_freq= bs->seek_freq;
				 bs->seek_freq = scan_status->rf_freq;

				 frequency =  start_frequency + 25000; //we scan a band of 50Mhz around this
				 p->frequency = frequency;
				 vprintk("SCAN tune request freq=%d, was %d freq=%d", bs->seek_freq, old_seek_freq, frequency);
				 state->demod_tuned_freq= frequency;
				 state->tuner_tuned_freq= frequency;
				 if(fe->ops.tuner_ops.set_frequency) {
					 fe->ops.tuner_ops.set_frequency(fe, frequency);
				 } else if (fe->ops.tuner_ops.set_frequency_and_bandwidth) {
					 fe->ops.tuner_ops.set_frequency_and_bandwidth(fe, frequency, bandwidth);
				 }
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
				 //state->delivery_system = p->delivery_system;
				 vprintk("FOUND freq=%d symrate=%d standard=%d => %d", frequency, symbol_rate, standard, p->delivery_system);
					p->frequency = frequency;
					p->symbol_rate = symbol_rate;
					si2183_read_status(fe, status);
					found= true;
					goto _exit;
					break;
				}
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_ERROR:
				 dprintk("ERROR");
				 goto _exit;
				 break;

			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_SEARCHING:
				 vprintk("searching");
				 bs->skip_resume = true;
						break;
			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_ENDED:
				 vprintk("ended");
				 goto _exit;
				 break;

			 case Si2183_SCAN_STATUS_RESPONSE_SCAN_STATUS_DEBUG:
				 vprintk("DEBUG type=%d: %s", scan_status->symb_rate,
								 scan_status->symb_rate == 4 ? "spectrum":
								 scan_status->symb_rate == 9 ? "trylock": "other");
				 num_samples = si2183_get_spectrum_scan_fft_one_band(fe, temp_freq, temp_rf_level,
																								 max_fft_size, p->frequency);
				 for(i=0; i< num_samples; ++i) {
					 if (temp_freq[i] >= p->frequency +25000) {
						 break;
					 }
					 if (temp_freq[i] < p->frequency - 25000)
						 continue;
					 if(ss->spectrum_len < ss->spectrum_max_len) {
						 ss->freq[ss->spectrum_len] = temp_freq[i];
						 ss->spectrum[ss->spectrum_len] = temp_rf_level[i];
						 ss->spectrum_len++;
					 } else {
						 dprintk("Insufficient room: i=%d", i);
					 }

				 }

				 start_frequency = (ss->spectrum_len>=1) ?(ss->freq[ss->spectrum_len-1] + resolution) :
					 (start_frequency+50000);
				 vprintk("debug done: num_samples=%d next_start=%d", num_samples, start_frequency);
				 si2183_spectrum_next(fe, bs);
				 bs->skip_resume = true;
				 while (scan_status->buz == Si2183_SCAN_STATUS_RESPONSE_BUZ_BUSY) {
					 if ((count% 20==19) && (kthread_should_stop() || dvb_frontend_task_should_stop(fe))) {
						 goto _exit;
					 }
					 si2183_scan_status(client, Si2183_SCAN_STATUS_CMD_INTACK_OK, scan_status);
					 msleep(20);
					 if(++count%20==19) {
						 vprintk("count=%d", count);
					 }
				 }
				 dprintk("len=%d/%d", ss->spectrum_len, ss->spectrum_max_len);
				 if(si2183_scan_action(client, bs->action, bs->seek_freq)) {
					 dprintk("Scan action failed");
					 goto _exit;
				 }

				 bs-> action = Si2183_SCAN_CTRL_CMD_ACTION_RESUME;

				 break;
			 default: {
				 dprintk("unknown scan_status %d", scan_status->scan_status);
				 bs->skip_resume = true;
				 break;
			 }
			 }
			 vprintk("skip_resume=%d", bs->skip_resume);
			 if (bs->skip_resume == false) {
				 if(si2183_scan_action(client, bs->action, bs->seek_freq)) {
					 dprintk("scan action failed");
					 goto _exit;
				 }
			 }
		 }
		msleep(50);
	 }

 _exit:
	 *status = FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;
	return 0;

}



int si2183_spectrum_get(struct dvb_frontend *fe, struct dtv_fe_spectrum* user)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev *state = i2c_get_clientdata(client);

	int error=0;
	vprintk("num_freq: user=%d kernel=%d", user->num_freq, state->scan_state.spectrum_len);
	if(user->num_freq > state->scan_state.spectrum_len)
		user->num_freq = state->scan_state.spectrum_len;
#ifdef TODO
	if (user->num_candidates > state->scan_state.num_candidates)
		user->num_candidates = state->scan_state.num_candidates;
#else
	user->num_candidates = 0;
#endif
	if(state->scan_state.freq && state->scan_state.spectrum) {
	if (copy_to_user((void __user*) user->freq, state->scan_state.freq, user->num_freq * sizeof(__u32))) {
			error = -EFAULT;
		}
		if (copy_to_user((void __user*) user->rf_level, state->scan_state.spectrum, user->num_freq * sizeof(__s32))) {
			error = -EFAULT;
		}
#ifdef TODO
		if(user->candidates && user->num_candidates >0 &&
			 copy_to_user((void __user*) user->candidates,
										state->scan_state.candidates,
										user->num_candidates * sizeof(struct spectral_peak_t))) {
			error = -EFAULT;
		}
#endif
	}
	else
		error = -EFAULT;
	return error;
}

enum fe_delivery_system si2183_delsys(int val)
{
	switch(val) {
	case 8:
		return SYS_DVBS;
	case 9:
		return SYS_DVBS2;
	case 2:
		return SYS_DVBT;
	case 3:
		return SYS_DVBC_ANNEX_A;
	case 4:
		return SYS_ISDBT;
	case 7:
		return SYS_DVBT2;
	case 10:
		return SYS_DSS;
	case 11:
		return SYS_DVBC2;
	}
	return SYS_UNDEFINED;
}

enum fe_modulation si2183_modulation(int val)
{
	switch(val) {
	case 0x03:
		return QPSK;
		break;
	case 0x07:
		return QAM_16;
		break;
	case 0x08:
		return QAM_32;
		break;
	case 0x09:
		return QAM_64;
		break;
	case 0x0a:
		return QAM_128;
		break;
	case 0x0b:
		return QAM_256;
		break;
	case 0x0e:
		return PSK_8;
		break;
	case 0x14:
		return APSK_16;
		break;
	case 0x17:
		return APSK_8L;
		break;
	case 0x18:
		return APSK_16L;
		break;
	case 0x15:
		return APSK_32;
		break;
	case 0x19:
		return APSK_32L;
		break;
	case 0x1a:
		return APSK_32;
		break;
	default:
		return QPSK;
		break;
	}
	return -1;
}

enum fe_rolloff si2183_rolloff(int val)
{
	switch(val) {
	case 0: return ROLLOFF_35;
	case 1: return ROLLOFF_25;
	case 2: return ROLLOFF_20;
	case 3: return ROLLOFF_AUTO;
	case 4: return ROLLOFF_15;
	case 5: return ROLLOFF_10;
	case 6: return ROLLOFF_5;
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

int si2183_misc_data(struct i2c_client *client , bool* issyi, bool* npd)
{
	unsigned int value;
	int ret=0;
	ret |= si2183_get_register_locked(client, 0x017a14, &value);
	*issyi = (value == 0 || value == 1);

	ret |= si2183_get_register_locked(client, 0x007814, &value);
	*npd = (value != 0);
	return ret;
}

int si2183_rssi_adc(struct i2c_client *client, bool on_off, int *rssi)
{
	//unsigned int value;
	struct si2183_cmd cmd;
	cmd.args[0]=0x17; /*RSSI_ADC_CMD*/
	cmd.args[1] = on_off;
	cmd.wlen = 2;
	cmd.rlen = 2;
	*rssi = cmd.args[1];

	return -1;
}


int si2183_set_sat_agc(struct i2c_client *client)
{
	struct si2183_cmd cmd;
	int ret;
	struct si2183_dev* state = i2c_get_clientdata(client);
	/*set SAT AGC*/
	si2183_cmd_str(&cmd, "\x8a\x1d\x12\x0\x0\x0", 6, 3);
	cmd.args[1]= state->agc_mode|0x18;
 	ret = si2183_cmd_execute(client, &cmd);
 	if(ret){
 		dprintk("error setting agc");
 	}
	return ret;
}

//returns true if retune is needed
int blind_tune(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct si2183_dev* state = i2c_get_clientdata(client);
	struct dtv_frontend_properties* p = &fe->dtv_property_cache;
	unsigned int delay;
	enum fe_status status;
	bool init = true;
	int delta;
	bool need_retune;
	vprintk("freq=%d", p->frequency);

	p->scan_start_frequency = p->frequency;
	p->scan_end_frequency = p->frequency;

	si2183_scan_sat_(fe, init, &delay, &status, true);
	delta = state->tuner_tuned_freq - state->demod_tuned_freq;
	if(delta<0)
		delta = -delta;
	need_retune = (delta > 2000);
	vprintk("si2183_scan_sat ended need_retune=%d", need_retune);
	if(!need_retune) {
		int bw = (p->symbol_rate/1000)*135/200 + 2000 +delta;
		vprintk("reducing bandwidth to %d", bw);
		if(fe->ops.tuner_ops.set_bandwidth)
			fe->ops.tuner_ops.set_bandwidth(fe, bw);
	}
	return need_retune;
}
