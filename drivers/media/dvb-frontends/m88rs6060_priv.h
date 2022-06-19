#ifndef _M88RS6060_PRIV_H_
#define _M88RS6060_PRIV_H_
#include <media/dvb_frontend.h>
#include <linux/dvb/frontend.h>
#include <linux/timekeeping.h>
#include <neumo-scan.h>

#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/math64.h>

#include "m88rs6060.h"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
#define M88RS6060_FIRMWARE "dvb-demod-m88rs6060.fw"

//Improve the driver capability or not
// 0:4mA, 1:8mA,2 :12mA , 3:16mA
#define MT_FE_ENHANCE_TS_PIN_LEVEL_PARALLEL_CI  0
/*set frequency offset to tuner when symbol rate <5000KSs*/
#define FREQ_OFFSET_AT_SMALL_SYM_RATE_KHz  3000

extern int verbose;



struct m88rs6060_isi_struct_t
{
	u32 isi_bitset[8]; //bitset; 1 bit indicates corresponding ISI is in use
};
typedef  struct m88rs6060_isi_struct_t  m88rs6060_isi_struct;


struct m88rs6060_state {
	int nr;
	struct i2c_client *demod_client;	//demod
	struct i2c_client *tuner_client;
	struct regmap* demod_regmap;	//demod
	enum fe_status fe_status;
	struct dvb_frontend fe;
	struct m88rs6060_cfg config;
	int adapterno; //number of adapter on card, starting at 0

	bool TsClockChecked;  //clock retio
	//bool warm;		// for the init and download fw
	s32 mclk;		/*main mclk */
	s32 detected_pls_code;
	s32 detected_pls_mode;
	enum fe_delivery_system detected_delivery_system;
	bool pls_active;
	bool is_mis;
	s32 active_stream_id;

	u32 dvbv3_ber;		/* for old DVBv3 API read_ber */
	s32 tuned_frequency;    //khz
	s32 center_freq_offset; //khz
	u64 pre_bit_error;
	u64 pre_bit_count;
	u64 last_pre_bit_error;
	u64 last_pre_bit_count;
	ktime_t last_per_time;
	bool satellite_scan;
	s32 scan_next_frequency;
	s32 scan_end_frequency;

	void (*write_properties)(struct i2c_adapter * i2c, u8 reg, u32 buf);
	void (*read_properties)(struct i2c_adapter * i2c, u8 reg, u32 * buf);

	//for si5351
	u32 plla_freq;
	u32 pllb_freq;
	bool newTP;
	bool fec_locked;
	bool demod_locked;
	bool        has_signal;   /*tuning has finished*/
	bool        has_carrier;  /*Some dvbs or dvbs2 signal was found*/
	bool        has_viterbi;
	bool        has_timing_lock;
	bool        has_sync;
	bool        has_timedout;
	bool        has_lock;
	m88rs6060_isi_struct isi_list;
	struct spectrum_scan_state spectrum_scan_state;
	struct constellation_scan_state constellation_scan_state;
};

struct m88rs6060_reg_val {
	u8 reg;
	u8 val;

};
static const struct m88rs6060_reg_val rs6060_reg_tbl_def[] = {
	{0x04, 0x00},

	{0x8a, 0x01},		//modify by Henry 20171211
	{0x16, 0xa7},

	{0x30, 0x88},
	{0x4a, 0x80},		//0x81 for fpga, and 0x80 for asic
	{0x4d, 0x91},
	{0xae, 0x09},
	{0x22, 0x01},
	{0x23, 0x00},
	{0x24, 0x00},
	{0x27, 0x07},
	{0x9c, 0x31},
	{0x9d, 0xc1},
	{0xcb, 0xf4},
	{0xca, 0x00},
	{0x7f, 0x04},
	{0x78, 0x0c},
	{0x85, 0x08},
	//for s2 mode ts out en
	{0x08, 0x47},
	{0xf0, 0x03},

	{0xfa, 0x01},
	{0xf2, 0x00},
	{0xfa, 0x00},
	{0xe6, 0x00},
	{0xe7, 0xf3},

	//for s mode vtb code rate all en
	{0x08, 0x43},
	{0xe0, 0xf8},
	{0x00, 0x00},
	{0xbd, 0x83},
	{0xbe, 0xa1}
};

/*register setting for blind scan*/
static const struct m88rs6060_reg_val rs6060_reg_tbl_bs_def[] =
{
	{0x04, 0x00},

	{0x8a, 0x01},
	{0x16, 0xa7},

	{0x30, 0x88},
	{0x4a, 0x80},
	{0x4d, 0x91},
	{0x63, 0x60},
	{0x64, 0x30},
	{0x65, 0x40},
	{0x68, 0x26},
	{0x69, 0x4c},
	{0xae, 0x09},
	{0x22, 0x01},
	{0x23, 0x00},
	{0x24, 0x00},
	{0x27, 0x07},
	{0x9c, 0x31},
	{0x9d, 0xc1},
	{0xc3, 0x10},
	{0xc4, 0x08},
	{0xc5, 0xf0},
	{0xc6, 0x40},
	{0xcb, 0xf4},
	{0xca, 0x00},
	{0x85, 0x08},
	//for s2 mode ts out en
	{0x08, 0x47},
	{0xf0, 0x03},

	{0xfa, 0x01},
	{0xf2, 0x00},
	{0xfa, 0x00},
	{0xe6, 0x00},
	{0xe7, 0x03},

	//for s mode vtb code rate all en
	{0x08, 0x43},
	{0xe0, 0xf8},
	{0x00, 0x00},
	{0xbd, 0x83},
	{0xbe, 0xa1}
};


struct MT_FE_PLS_INFO {
	u8 iPLSCode;
	bool bValid;
	enum MT_FE_TYPE mDvbType;
	enum MT_FE_MOD_MODE mModMode;
	enum MT_FE_CODE_RATE mCodeRate;
	bool bPilotOn;
	bool bDummyFrame;
	s8 iFrameLength;	/*0: Normal; 1: Short; 2: Medium */
};

struct MT_FE_CHAN_INFO_DVBS2 {
	u8 iPlsCode;
	enum MT_FE_TYPE type;
	enum MT_FE_MOD_MODE mod_mode;
	enum MT_FE_ROLL_OFF roll_off;
	enum MT_FE_CODE_RATE code_rate;
	bool is_pilot_on;
	enum MT_FE_SPECTRUM_MODE is_spectrum_inv;
	bool is_dummy_frame;
	s8 iVcmCycle;
	s8 iFrameLength;	/*0: Normal; 1: Short; 2: Medium */
};

void m88rs6060_tuner_select_mclk(struct m88rs6060_state *dev, u32 freq_mhz, u32 symbol_rate, bool blind);
void m88rs6060_set_mclk_according_to_symbol_rate(struct m88rs6060_state* state, u32 freq_mhz, u32 symbol_rate_kSs, u32 mclk, bool blind);
void m88rs6060_set_default_mclk(struct m88rs6060_state* state);
void m88rs6060_select_mclk(struct m88rs6060_state *dev, u32 freq_mhz, u32 symbol_rate, bool blind);
void m88rs6060_set_ts_mclk(struct m88rs6060_state *dev, u32 mclk);
int m88rs6060_set_tuner(struct m88rs6060_state* state, u32 tuner_freq_mhz, u32 symbol_rate_kss, s32 lpf_offset_khz);
int m88rs6060_set_carrier_offset(struct m88rs6060_state* state, s32 carrier_offset_khz);
int m88rs6060_get_spectrum_scan_fft(struct dvb_frontend *fe,
																		unsigned int *delay, enum fe_status *status);
int m88rs6060_get_gain(struct m88rs6060_state *dev, u32 freq_mhz, s32 * p_gain);
bool m88rs6060_wait_for_analog_agc_lock(struct m88rs6060_state* state);
#endif
