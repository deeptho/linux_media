// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * dvb_demux.c - DVB kernel demux API
 *
 * Copyright (C) 2000-2001 Ralph  Metzler <ralph@convergence.de>
 *		       & Marcus Metzler <marcus@convergence.de>
 *			 for convergence integrated media GmbH
 */

#define pr_fmt(fmt) "dvb_demux: " fmt

#include <linux/sched/signal.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/crc32.h>
#include <linux/uaccess.h>
#include <asm/div64.h>
#include <media/dvb_demux.h>
#include <media/dmxdev-sysfs.h>

static int dvb_demux_tscheck=0;
static int dvb_demux_dtdebug=1;
#define CHECKCRC

module_param(dvb_demux_dtdebug, int, 0644);
MODULE_PARM_DESC(dvb_demux_dtdebug, "enable debug");

module_param(dvb_demux_tscheck, int, 0644);
MODULE_PARM_DESC(dvb_demux_tscheck,
		"enable transport stream continuity and TEI check");

static int dvb_demux_speedcheck;
module_param(dvb_demux_speedcheck, int, 0644);
MODULE_PARM_DESC(dvb_demux_speedcheck,
		"enable transport stream speed check");

static int dvb_demux_feed_err_pkts = 1;
module_param(dvb_demux_feed_err_pkts, int, 0644);
MODULE_PARM_DESC(dvb_demux_feed_err_pkts,
		 "when set to 0, drop packets with the TEI bit set (1 by default)");


#define dprintk(fmt, arg...) do {														\
		if (dvb_demux_dtdebug)																	\
			printk(KERN_DEBUG pr_fmt("%s:%d " fmt),								\
						 __func__, __LINE__, ##arg);										\
	} while (0)

#define dprintk_nice(fmt, arg...) do {																	\
		static int count=0;																									\
		if (count++%100 == 0 && dvb_demux_dtdebug)													\
			printk(KERN_DEBUG pr_fmt("%s:%d count=%d " fmt),									\
						 __func__, __LINE__, count, ##arg);													\
	} while (0)


#define dmx_demux_dprintk(dmx_demux, fmt, arg...) do {									\
		if (dvb_demux_dtdebug)																							\
			printk(KERN_DEBUG pr_fmt("%s:%d dmx_demux[%p] " fmt),							\
						 __func__, __LINE__, dmx_demux, ##arg);											\
	} while (0)

#define feeds_dprintk(feeds, fmt, arg...) do {													\
		if (dvb_demux_dtdebug)																							\
			printk(KERN_DEBUG pr_fmt("%s:%d feeds[%p] embedding_pid=%d feeds.refcount=%d " fmt),	\
				__func__, __LINE__, feeds, feeds? feeds->embedding_pid : -1,		\
						 atomic_read(&feeds->refcount.refcount.refs),								\
						 ##arg);																										\
	} while (0)


#define dmx_demux_dprintk_nice(dmx_demux, fmt, arg...) do {							\
		static int count=0;																									\
		if (count++%100 == 0 && dvb_demux_dtdebug)											\
			printk(KERN_DEBUG pr_fmt("%s:%d dmx_demux[%p] count=%d " fmt),		\
						 __func__, __LINE__, dmx_demux, count, ##arg);							\
	} while (0)

#define embedded_stream_dprintk(emb, fmt, arg...) do {									\
		if (dvb_demux_dtdebug) {																						\
			struct stid_stream* stid = embedded_stream_get_super_class((emb), EMBEDDED_STREAM_TYPE_STID);	\
			if(stid) {																												\
				printk(KERN_DEBUG pr_fmt("%s:%d stid[%p] pid=%d feeds=%p stid.refcount=%d " fmt),	\
							 __func__, __LINE__, stid, (emb)->embedding_pid, (emb)->parent_feeds, \
							 atomic_read(&(emb)->refcount.refcount.refs),							\
							 ##arg);																									\
				break;																													\
			}																																	\
			struct t2mi_stream* t2mi = embedded_stream_get_super_class((emb), EMBEDDED_STREAM_TYPE_T2MI);	\
			if(t2mi) {																												\
				printk(KERN_DEBUG pr_fmt("%s:%d t2mi[%p] pid=%d feeds=%p stid.refcount=%d " fmt),	\
					 __func__, __LINE__, t2mi, (emb)->embedding_pid, (emb)->parent_feeds, \
							 atomic_read(&(emb)->refcount.refcount.refs),							\
							 ##arg);																									\
			}																																	\
		}																																		\
	} while (0)



#define stid_stream_dprintk_nice(stid, fmt, arg...) do {								\
		static int count=0;																									\
		if (count++%100 == 0 && dvb_demux_dtdebug)													\
			printk(KERN_DEBUG pr_fmt("%s:%d stid[%p] pid=%d feeds=%p count=%d " fmt), \
						 __func__, __LINE__, stid, stid->emb.embedding_pid, stid->emb.parent_feeds, count, ##arg); \
	} while (0)


#define t2mi_stream_dprintk(t2mi, fmt, arg...) do {											\
		if (dvb_demux_dtdebug)																							\
			printk(KERN_DEBUG pr_fmt("%s:%d t2mi[%p] pid=%d feeds=%p " fmt),	\
						 __func__, __LINE__, t2mi, t2mi->emb.embedding_pid, t2mi->emb.parent_feeds, ##arg); \
	} while (0)


#define t2mi_stream_dprintk_nice(t2mi, fmt, arg...) do {							\
		static int count=0;																									\
		if (count++%100 == 0 && dvb_demux_dtdebug)													\
			printk(KERN_DEBUG pr_fmt("%s:%d t2mi[%p] pid=%d feeds=%p count=%d " fmt), \
						 __func__, __LINE__, t2mi, t2mi->emb.embedding_pid, t2mi->emb.parent_feeds, count, ##arg);	\
	} while (0)


#define bbf_dprintk(bbf, fmt, arg...) do {															\
		if (dvb_demux_dtdebug)																							\
			printk(KERN_DEBUG pr_fmt("%s:%d bbf[%p] isi=%d stid=%p bbf.refcount=%d " fmt), \
						 __func__, __LINE__, bbf, bbf ? bbf->isi : -1, bbf->parent_embedded_stream, \
						 atomic_read(&bbf->refcount.refcount.refs),									\
						 ##arg);																										\
	} while (0)


#define bbf_dprintk_nice(bbf, fmt, arg...) do {										\
		static int count=0;																									\
		if (count++%100 == 0 && dvb_demux_dtdebug)													\
			printk(KERN_DEBUG pr_fmt("%s:%d bbf[%p] isi=%d count=%d " fmt),	\
						 __func__, __LINE__, bbf, bbf ? bbf->isi : -1, count, ##arg); \
	} while (0)


#define dprintk_tscheck(x...) do {			\
	if (dvb_demux_tscheck && printk_ratelimit())	\
		dprintk(x);				\
} while (0)

#ifdef CONFIG_DVB_DEMUX_SECTION_LOSS_LOG
#  define dprintk_sect_loss(x...) dprintk(x)
#else
#  define dprintk_sect_loss(x...)
#endif

#define set_buf_flags(__feed, __flag)			\
	do {						\
		(__feed)->buffer_flags |= (__flag);	\
	} while (0)

static void stid_stream_add_packet(struct dvb_demux* demux, struct stid_stream* stid, const uint8_t* packet);
static void t2mi_stream_add_packet(struct dvb_demux* demux, struct t2mi_stream* t2mi, const uint8_t* packet);

static void dvb_dmx_swfilter_packet(struct dvb_demux *demux, const uint8_t *buf,
																		struct dvb_demux_feeds* feeds, bool frombbf);


/******************************************************************************
 * static inlined helper functions
 ******************************************************************************/
#define dvb_demux_feed_dprintk(feed, fmt, arg...) do {									\
		if(!feed)																														\
			printk(KERN_DEBUG pr_fmt("%s:%d NO FEED " fmt),										\
						 __func__, __LINE__, ##arg);																\
	else																																	\
		printk(KERN_DEBUG pr_fmt("%s:%d feed[%p] pid=%d ts_feed=%p filter=%p feed.parent_feeds=%p parent_feeds.refcount=%d " fmt), \
					 __func__, __LINE__, (void*)feed, feed->pid, &feed->feed.pid_stream, \
					 (void*)feed->section_filter, feed->parent_feeds,							\
					 atomic_read(&feed->parent_feeds->refcount.refcount.refs), ##arg); \
	} while (0)

#define dvb_demux_feed_dprintk_nice(feed, fmt, arg...) do {							\
		static int count=0;																									\
		if (count++%100 == 0 && dvb_demux_dtdebug) {												\
			if(!feed)																													\
				printk(KERN_DEBUG pr_fmt("%s:%d NO FEED " fmt),									\
							 __func__, __LINE__, ##arg);															\
			else																															\
				printk(KERN_DEBUG pr_fmt("%s:%d feed[%p] pid=%d ts_feed=%p filter=%p " fmt), \
							 __func__, __LINE__, (void*)feed, feed->pid, &feed->feed.pid_stream, \
							 (void*)feed->section_filter, ##arg);											\
		}																																		\
	} while (0)

// taken and adapted from libdtv, (c) Rolf Hakenes
// CRC32 lookup table for polynomial 0x04c11db7
static uint32_t crc32_table[256] = {
	0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9,
	0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
	0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
	0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
	0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9,
	0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
	0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011,
	0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
	0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
	0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
	0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81,
	0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
	0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49,
	0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
	0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
	0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
	0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae,
	0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
	0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16,
	0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
	0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
	0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
	0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066,
	0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
	0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e,
	0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
	0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
	0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
	0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e,
	0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
	0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686,
	0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
	0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
	0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
	0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f,
	0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
	0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47,
	0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
	0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
	0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
	0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7,
	0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
	0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f,
	0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
	0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
	0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
	0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f,
	0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
	0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640,
	0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
	0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
	0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
	0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30,
	0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
	0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088,
	0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
	0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
	0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
	0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18,
	0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
	0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0,
	0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
	0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
	0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4,
};

/* CRC table crc-8, poly=0xD5 */
static uint8_t crc8_table[256] = {
	0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
	0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
	0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
	0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
	0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
	0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
	0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
	0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
	0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
	0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
	0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
	0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
	0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
	0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
	0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
	0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

static uint8_t compute_crc8(const uint8_t *p, uint8_t len)
{
    int    i;
    uint8_t crc = 0, tmp;
		int offset=0;
    for (i = 0; i < len; i++) {
        tmp = p[offset++];
        crc = crc8_table[crc ^ tmp];
    }
    return crc;
}

static void	bbframes_stream_reset(struct bbframes_stream* bbf);

static inline u16 section_length(const u8 *buf)
{
	return 3 + ((buf[1] & 0x0f) << 8) + buf[2];
}

static inline u16 ts_pid(const u8 *buf)
{
	return ((buf[1] & 0x1f) << 8) + buf[2];
}

static inline u8 payload(const u8 *tsp)
{
	if (!(tsp[3] & 0x10))	// no payload?
		return 0;

	if (tsp[3] & 0x20) {	// adaptation field?
		if (tsp[4] > 183)	// corrupted data?
			return 0;
		else
			return 184 - 1 - tsp[4];
	}

	return 184;
}

static u32 dvb_dmx_crc32(struct dvb_demux_feed *f, const u8 *src, size_t len)
{
	return (f->feed.sec.crc_val = crc32_be(f->feed.sec.crc_val, src, len));
}

static void dvb_dmx_memcopy(struct dvb_demux_feed *f, u8 *d, const u8 *s,
			    size_t len)
{
	memcpy(d, s, len);
}

static void dvb_demux_feeds_init(struct dvb_demux_feeds* feeds, struct dvb_demux* dvbdemux,
																 int embedding_pid, int isi)
{
	feeds->demux = dvbdemux;
	feeds->embedding_pid = embedding_pid;
	feeds->isi = isi;
	kref_init(&feeds->refcount);

	INIT_LIST_HEAD(&feeds->output_feed_list);
	xa_init(&feeds->embedded_streams);
	feeds->cnt_storage = vmalloc(MAX_PID + 1);
	dprintk("ALLOC feeds->cnt_storage=%p feeds=%p\n", feeds->cnt_storage, feeds);
	if (!feeds->cnt_storage)
		pr_warn("Couldn't allocate memory for TS/TEI check. Disabling it\n");
	else  {
		int i;
		for(i=0; i<MAX_PID; ++i)
			feeds->cnt_storage[i] = 0xff;
	}
}

static void dvb_demux_feeds_release_(struct kref *kref)
{
	struct dvb_demux_feeds* feeds = container_of(kref, struct dvb_demux_feeds, refcount);
	int output_feeds_list_empty = list_empty(&feeds->output_feed_list);
	int embedded_streams_empty = xa_empty(&feeds->embedded_streams);
	dprintk("feeds=%p refcount has dropped to 0 releasing feeds; "
					"embedded_streams:empty=%d "
					"output_feedlist_:empty=%d\n",
					feeds, embedded_streams_empty, output_feeds_list_empty);
	dprintk("feeds->embedded_streams list:\n");
	{
		unsigned long index;
		void *entry;
		xa_for_each(&feeds->embedded_streams, index, entry) {
			struct stid_stream* stid = embedded_stream_get_super_class(entry, EMBEDDED_STREAM_TYPE_STID);
			if(stid) {
				dprintk("item: stid=%p embedding_pid=%d parent_feeds=%p\n",
								stid, stid->emb.embedding_pid, stid->emb.parent_feeds);
			} else {
				struct t2mi_stream* t2mi = embedded_stream_get_super_class(entry, EMBEDDED_STREAM_TYPE_T2MI);
				if (t2mi)
					dprintk("item: t2mi=%p embedding_pid=%d parent_feeds=%p\n",
									t2mi, t2mi->emb.embedding_pid, t2mi->emb.parent_feeds);
			}
		}
	}
	WARN_ON(!output_feeds_list_empty);
	dprintk("FREE feeds->cnt_storage=%p in feeds=%p\n", feeds->cnt_storage, feeds);
	vfree(feeds->cnt_storage);
	xa_destroy(&feeds->embedded_streams);
	dprintk("FREE feeds=%p\n", feeds);
	kfree(feeds);
}

static void	ts_stream_reset(struct ts_stream* ts) {
	ts->buff_idx = 0;
	ts->synced = false;
}

/*
	reset prior to restarting decoding, e.g., after a crc error
 */
static inline void embedded_stream_reset(struct embedded_stream* emb, bool full_reset)
{
	emb->payload_len = 0;
	emb->packet_count = -1;
	emb->bbf_header_crc8 = -1;
	emb->current_isi = -1;
	emb->cc_counter = -1;
	emb->current_bbf = NULL;
	if(full_reset) {
		memset(emb->isi_plp_bitset, 0, sizeof(emb->isi_plp_bitset));
		memset(emb->high_rolloff_mode, 0, sizeof(emb->high_rolloff_mode));
		emb->num_streams = 0;
		memset(emb->matypes, 0, sizeof(emb->matypes));
	}
	unsigned long index;
	struct bbframes_stream* entry;
	xa_for_each(&emb->bbf_streams, index, entry) {
		bbframes_stream_reset(entry);
	}
}

static inline void embedded_stream_init(struct embedded_stream* emb, struct dvb_demux_feeds*parent_feeds,
																				enum embedded_stream_type embedded_stream_type, int embedding_pid)
{
	dprintk("emb=%p parent_feeds=%p\n", emb, parent_feeds);
	WARN_ON(!parent_feeds);
	emb->embedded_stream_type = embedded_stream_type;
	emb->embedding_pid = embedding_pid;
	xa_init(&emb->bbf_streams);
	kref_init(&emb->refcount);
	emb->parent_feeds = parent_feeds;
	kref_get(&emb->parent_feeds->refcount);
	feeds_dprintk(emb->parent_feeds, "After kref_get: refcount=%d\n",
								atomic_read(&emb->parent_feeds->refcount.refcount.refs));
	embedded_stream_reset(emb, true/*full_reset*/);
}

static inline void embedded_stream_set_matype(struct embedded_stream* emb, uint8_t stream_id, uint8_t matype)
{
	//dmx_demux_dprintk_nice(emb, "QQQ isi=%d matype=%d\n", stream_id, matype);
	int32_t* bitset = &emb->isi_plp_bitset[stream_id >>5];
	if (!! ((bitset[0] >> (stream_id&31))&1)) {
		//noticed before
		int ro = (matype &3);
		int old_ro = (emb->matypes[stream_id] &3);
		if(ro == 3) {
			if(old_ro !=3 ) {
				emb->high_rolloff_mode[stream_id >>5] |= (1 <<(stream_id&31));
			}
		} else {
			if(old_ro ==3 ) {
				emb->high_rolloff_mode[stream_id >>5] |= (1 <<(stream_id&31));
				emb->matypes[stream_id] = matype;
			}
		}
	} else {
		//store matype
		emb->num_streams++;
		emb->matypes[stream_id] = matype;
		bitset[0] |=  (1<<(stream_id&31));
	}
}

static inline void stid_stream_reset(struct stid_stream* stid)
{
	embedded_stream_reset(&stid->emb, false /*full_reset*/);
	stid->section_length = 0;
}

static void stid_stream_init(struct stid_stream* stid, struct dvb_demux_feeds*parent_feeds,
														 int embedding_pid)
{
	embedded_stream_init(&stid->emb, parent_feeds, EMBEDDED_STREAM_TYPE_STID, embedding_pid);
	stid->section_length = 0;
}

/*Reset prior to restarting decoding, e.g., after a crc error
 */
static void t2mi_stream_reset(struct t2mi_stream* t2mi, bool full_reset)
{
	embedded_stream_reset(&t2mi->emb, full_reset);
	t2mi->packet_type = -1;
	t2mi->superframe_idx = 0;
	t2mi->crc_idx = 0;
	t2mi->synced = false;
	t2mi->frame_idx = -1;
	t2mi->plp_id = -1;
	t2mi->packet_header_isi = -1;
	t2mi->intl_frame_start = 0;
	t2mi->t2mi_crc32 = 0;
	t2mi->t2mi_payload_bytes_left = 0;
	t2mi->bbheader_bytes_left = 0;
	t2mi->header_idx = 0;
	t2mi->bbheader_idx = 0;
}

static void t2mi_stream_init(struct t2mi_stream* t2mi, struct dvb_demux_feeds*parent_feeds,
														 int embedding_pid)
{
	embedded_stream_init(&t2mi->emb, parent_feeds, EMBEDDED_STREAM_TYPE_T2MI, embedding_pid);
	t2mi->default_isi = -1;
	t2mi_stream_reset(t2mi, true/*full_reset*/);
	t2mi->num_cc_errors=0;
	t2mi->num_crc8_errors=0;
	t2mi->num_crc32_errors=0;
}

/*
	reset stream when restarting after an errors
 */
static void	bbframes_stream_reset(struct bbframes_stream* bbf) {
	ts_stream_reset(&bbf->ts);
	bbf->matype = -1;
	bbf->upl = -1;
	bbf->dfl = -1;
	bbf->syncd = -1;
	bbf->issy = 0;
	bbf->bbf_crc8 = -1;
	bbf->bbf_payload_bytes_left = 0;
	bbf->syncbyte = 0x47;
	bbf->synced = false;
	bbf->hem_mode = false;
}

static void	bbframes_stream_init(struct bbframes_stream* bbf,
																 struct embedded_stream* parent_embedded_stream, struct dvb_demux* demux,
																 int embedding_pid, int isi) {
	bbf->isi = isi;
	kref_init(&bbf->refcount);
	bbf->parent_embedded_stream = parent_embedded_stream;
	bbf->feeds = kzalloc(sizeof(struct dvb_demux_feeds), GFP_KERNEL);
	embedded_stream_dprintk(parent_embedded_stream, "ALLOC bbf->feeds=%p stream=%p\n", bbf->feeds, bbf);
	dvb_demux_feeds_init(bbf->feeds, demux, embedding_pid, isi);
	bbframes_stream_reset(bbf);
}

static void embedded_stream_release_(struct kref *kref)
{
	struct embedded_stream* emb = container_of(kref, struct embedded_stream, refcount);
	int bbframes_streams_empty = xa_empty(&emb->bbf_streams);
	embedded_stream_dprintk(emb, "refcount has dropped to 0 releasing; "
					"bbframe_streams:empty=%d\n", bbframes_streams_empty);
	WARN_ON(!bbframes_streams_empty);

	embedded_stream_dprintk(emb, "before reducing parent_feeds.refcount\n");
	feeds_dprintk(emb->parent_feeds, "parent_feeds\n");
	kref_put(&emb->parent_feeds->refcount, dvb_demux_feeds_release_);

	embedded_stream_dprintk(emb, "erasing: parent_feeds->bbframe_demuxes[%d]\n", emb->embedding_pid);
	xa_erase(&emb->parent_feeds->embedded_streams, emb->embedding_pid);
	dprintk("FREE emb=%p\n", emb);
	xa_destroy(&emb->bbf_streams);
}


static void embedded_stream_release(struct embedded_stream* emb) {
	embedded_stream_dprintk(emb, "before reducing emb.refcount\n");
	kref_put(&emb->refcount, embedded_stream_release_);
	dprintk("after reducing stid.refcount\n");
}

static void bbframes_stream_release_(struct kref *ref)
{
	struct bbframes_stream* bbf = container_of(ref, struct bbframes_stream, refcount);
	struct dvb_demux_feeds* feeds = bbf->feeds;
	bbf_dprintk(bbf, "erasing bbf=%p for isi=%d in parent=%p\n", bbf, bbf? bbf->isi : -1,
					bbf->parent_embedded_stream);
	int isi = bbf->isi;
	if (isi < 0)
		isi = 256;
	kref_put(&feeds->refcount,  dvb_demux_feeds_release_);
	xa_erase(&bbf->parent_embedded_stream->bbf_streams, isi);
	dprintk("FREE bbf=%p\n", bbf);
	kfree(bbf);
	dprintk("AFTER FREE bbf=%p\n", bbf);
}

static void bbframes_stream_release(struct bbframes_stream* bbf) {
	struct embedded_stream* emb = bbf->parent_embedded_stream; //save because pointer will be erases
	WARN_ON(!bbf);
	int isi = bbf->isi;
	embedded_stream_dprintk(emb, "bbf=%p isi=%d bbf.refcount=%d\n",
												bbf, isi, atomic_read(&bbf->refcount.refcount.refs));
	kref_put(&bbf->refcount, bbframes_stream_release_);
	if(emb) {
		dprintk("calling embedded_stream_release emb=%p\n", emb);
		embedded_stream_release(emb);
	} else {
		dprintk("UNEXPECTED: NOT calling embedded_stream_release emb=%p\n", emb);
	}
	dprintk("done isi=%d \n", isi);
}

/******************************************************************************
 * Software filter functions
 ******************************************************************************/

//dma
static inline int dvb_dmx_swfilter_payload(struct dvb_demux_feed *feed,
					   const u8 *buf)
{
	int count = payload(buf);
	int p;
	int ccok;
	u8 cc;

	if (count == 0)
		return -1;

	p = 188 - count;

	cc = buf[3] & 0x0f;
	ccok = ((feed->cc + 1) & 0x0f) == cc;
	if (!ccok) {
		set_buf_flags(feed, DMX_BUFFER_FLAG_DISCONTINUITY_DETECTED);
		dprintk_sect_loss("missed packet: %d instead of %d!\n",
				  cc, (feed->cc + 1) & 0x0f);
	}
	feed->cc = cc;

	if (buf[1] & 0x40)	// PUSI ?
		feed->peslen = 0xfffa;

	feed->peslen += count;

	return feed->cb.ts(&buf[p], count, NULL, 0, &feed->feed.pid_stream,
			   &feed->buffer_flags);
}

static int dvb_dmx_swfilter_sectionfilter(struct dvb_demux_feed *feed,
					  struct dvb_demux_section_filter *f)
{
	u8 neq = 0;
	int i;

	for (i = 0; i < DVB_DEMUX_MASK_MAX; i++) {
		u8 xor = f->filter.filter_value[i] ^ feed->feed.sec.secbuf[i];

		if (f->maskandmode[i] & xor)
			return 0;

		neq |= f->maskandnotmode[i] & xor;
	}

	if (f->doneq && !neq)
		return 0;

	return feed->cb.sec(feed->feed.sec.secbuf, feed->feed.sec.seclen,
			    NULL, 0, &f->filter, &feed->buffer_flags);
}

static inline int dvb_dmx_swfilter_section_feed(struct dvb_demux_feed *feed)
{
	struct dvb_demux *demux = feed->demux;
	struct dvb_demux_section_filter *f = feed->section_filter;
	struct dmx_section_feed *sec = &feed->feed.sec;
	int section_syntax_indicator;

	if (!sec->is_filtering)
		return 0;

	if (!f)
		return 0;

	if (sec->check_crc) {
		section_syntax_indicator = ((sec->secbuf[1] & 0x80) != 0);
		if (section_syntax_indicator &&
		    demux->check_crc32(feed, sec->secbuf, sec->seclen)) {
			set_buf_flags(feed, DMX_BUFFER_FLAG_HAD_CRC32_DISCARD);
			return -1;
		}
	}

	do {
		if (dvb_dmx_swfilter_sectionfilter(feed, f) < 0)
			return -1;
	} while ((f = f->next) && sec->is_filtering);

	sec->seclen = 0;

	return 0;
}

static void dvb_dmx_swfilter_section_new(struct dvb_demux_feed *feed)
{
	struct dmx_section_feed *sec = &feed->feed.sec;

	if (sec->secbufp < sec->tsfeedp) {
		int n = sec->tsfeedp - sec->secbufp;

		/*
		 * Section padding is done with 0xff bytes entirely.
		 * Due to speed reasons, we won't check all of them
		 * but just first and last.
		 */
		if (sec->secbuf[0] != 0xff || sec->secbuf[n - 1] != 0xff) {
			set_buf_flags(feed,
				      DMX_BUFFER_FLAG_DISCONTINUITY_DETECTED);
			dprintk_sect_loss("section ts padding loss: %d/%d\n",
					  n, sec->tsfeedp);
			dprintk_sect_loss("pad data: %*ph\n", n, sec->secbuf);
		}
	}

	sec->tsfeedp = sec->secbufp = sec->seclen = 0;
	sec->secbuf = sec->secbuf_base;
}

/*
 * Losless Section Demux 1.4.1 by Emard
 * Valsecchi Patrick:
 *  - middle of section A  (no PUSI)
 *  - end of section A and start of section B
 *    (with PUSI pointing to the start of the second section)
 *
 *  In this case, without feed->pusi_seen you'll receive a garbage section
 *  consisting of the end of section A. Basically because tsfeedp
 *  is incemented and the use=0 condition is not raised
 *  when the second packet arrives.
 *
 * Fix:
 * when demux is started, let feed->pusi_seen = false to
 * prevent initial feeding of garbage from the end of
 * previous section. When you for the first time see PUSI=1
 * then set feed->pusi_seen = true
 */
static int dvb_dmx_swfilter_section_copy_dump(struct dvb_demux_feed *feed,
					      const u8 *buf, u8 len)
{
	struct dvb_demux *demux = feed->demux;
	struct dmx_section_feed *sec = &feed->feed.sec;
	u16 limit, seclen;

	if (sec->tsfeedp >= DMX_MAX_SECFEED_SIZE)
		return 0;

	if (sec->tsfeedp + len > DMX_MAX_SECFEED_SIZE) {
		set_buf_flags(feed, DMX_BUFFER_FLAG_DISCONTINUITY_DETECTED);
		dprintk_sect_loss("section buffer full loss: %d/%d\n",
				  sec->tsfeedp + len - DMX_MAX_SECFEED_SIZE,
				  DMX_MAX_SECFEED_SIZE);
		len = DMX_MAX_SECFEED_SIZE - sec->tsfeedp;
	}

	if (len <= 0)
		return 0;

	demux->memcopy(feed, sec->secbuf_base + sec->tsfeedp, buf, len);
	sec->tsfeedp += len;

	/*
	 * Dump all the sections we can find in the data (Emard)
	 */
	limit = sec->tsfeedp;
	if (limit > DMX_MAX_SECFEED_SIZE)
		return -1;	/* internal error should never happen */

	/* to be sure always set secbuf */
	sec->secbuf = sec->secbuf_base + sec->secbufp;

	while (sec->secbufp + 2 < limit) {
		seclen = section_length(sec->secbuf);
		if (seclen <= 0 || seclen > DMX_MAX_SECTION_SIZE
		    || seclen + sec->secbufp > limit)
			return 0;
		sec->seclen = seclen;
		sec->crc_val = ~0;
		/* dump [secbuf .. secbuf+seclen) */
		if (feed->pusi_seen) {
			dvb_dmx_swfilter_section_feed(feed);
		} else {
			set_buf_flags(feed,
				      DMX_BUFFER_FLAG_DISCONTINUITY_DETECTED);
			dprintk_sect_loss("pusi not seen, discarding section data\n");
		}
		sec->secbufp += seclen;	/* secbufp and secbuf moving together is */
		sec->secbuf += seclen;	/* redundant but saves pointer arithmetic */
	}

	return 0;
}

static int dvb_dmx_swfilter_section_packet(struct dvb_demux_feed *feed, const u8 *buf)
{
	u8 p, count;
	int ccok, dc_i = 0;
	u8 cc;

	count = payload(buf);

	if (count == 0)		/* count == 0 if no payload or out of range */
		return -1;

	p = 188 - count;	/* payload start */

	cc = buf[3] & 0x0f;
	ccok = ((feed->cc + 1) & 0x0f) == cc;

	if (buf[3] & 0x20) {
		/* adaption field present, check for discontinuity_indicator */
		if ((buf[4] > 0) && (buf[5] & 0x80))
			dc_i = 1;
	}

	if (!ccok || dc_i) {
		if (dc_i) {
			set_buf_flags(feed,
				      DMX_BUFFER_FLAG_DISCONTINUITY_INDICATOR);
			dprintk_sect_loss("%d frame with disconnect indicator\n",
				cc);
		} else {
			set_buf_flags(feed,
				      DMX_BUFFER_FLAG_DISCONTINUITY_DETECTED);
			dprintk_sect_loss("discontinuity: %d instead of %d. %d bytes lost\n",
				cc, (feed->cc + 1) & 0x0f, count + 4);
		}
		/*
		 * those bytes under some circumstances will again be reported
		 * in the following dvb_dmx_swfilter_section_new
		 */

		/*
		 * Discontinuity detected. Reset pusi_seen to
		 * stop feeding of suspicious data until next PUSI=1 arrives
		 *
		 * FIXME: does it make sense if the MPEG-TS is the one
		 *	reporting discontinuity?
		 */

		feed->pusi_seen = false;
		dvb_dmx_swfilter_section_new(feed);
	}
	feed->cc = cc;

	if (buf[1] & 0x40) {
		/* PUSI=1 (is set), section boundary is here */
		if (count > 1 && buf[p] < count) {
			const u8 *before = &buf[p + 1];
			u8 before_len = buf[p];
			const u8 *after = &before[before_len];
			u8 after_len = count - 1 - before_len;

			dvb_dmx_swfilter_section_copy_dump(feed, before,
							   before_len);
			/* before start of new section, set pusi_seen */
			feed->pusi_seen = true;
			dvb_dmx_swfilter_section_new(feed);
			dvb_dmx_swfilter_section_copy_dump(feed, after,
							   after_len);
		} else if (count > 0) {
			set_buf_flags(feed,
				      DMX_BUFFER_FLAG_DISCONTINUITY_DETECTED);
			dprintk_sect_loss("PUSI=1 but %d bytes lost\n", count);
		}
	} else {
		/* PUSI=0 (is not set), no section boundary */
		dvb_dmx_swfilter_section_copy_dump(feed, &buf[p], count);
	}

	return 0;
}

/* Process packets that are actually delivered to the user of the demux:
	 -strip the packet headers if so requested
	 -filter for specific sections
 */
//dma
static inline void dvb_dmx_swfilter_packet_type(struct dvb_demux_feed *feed, const uint8_t *buf)
{
	switch (feed->type) {
	case DMX_TYPE_TS:
		if (!feed->feed.pid_stream.is_filtering)
			break;
		if (feed->ts_type & TS_PACKET) {
			if (feed->ts_type & TS_PAYLOAD_ONLY)
				dvb_dmx_swfilter_payload(feed, buf);
			else
				feed->cb.ts(buf, 188, NULL, 0, &feed->feed.pid_stream,
					    &feed->buffer_flags);
		}
		/* Used only on full-featured devices */
		if (feed->ts_type & TS_DECODER)
			if (feed->demux->write_to_decoder)
				feed->demux->write_to_decoder(feed, buf, 188);
		break;

	case DMX_TYPE_SEC:
		if (!feed->feed.sec.is_filtering)
			break;
		if (dvb_dmx_swfilter_section_packet(feed, buf) < 0)
			feed->feed.sec.seclen = feed->feed.sec.secbufp = 0;
		break;
	default:
		dvb_demux_feed_dprintk_nice(feed, "IMPLEMENTATION ERROR\n");
		break;
	}
}

#define DVR_FEED(f)																\
	(((f)->type == DMX_TYPE_TS) &&									\
	((f)->feed.pid_stream.is_filtering) &&					\
	(((f)->ts_type & (TS_PACKET | TS_DEMUX)) == TS_PACKET))



static void parse_bbheader(struct bbframes_stream* bbf, bool hem_mode, int isi, const uint8_t* buff)
{
	bbf->hem_mode = hem_mode;
	bbf->matype = buff[0];
	bbf->isi = isi;
	bbf->dfl = (buff[4] <<8) | buff[5];
	if((bbf->dfl & 0x7) !=0) { //expect multiple of 8 bits
		dmx_demux_dprintk_nice(bbf, "Expected multiple of 8 bits");
	}
	bbf->dfl >>=3; //now in bytes

	if (hem_mode) {
		bbf->issy = (buff[2] << 16) | (buff[3]<<8) | buff[6];
		bbf->upl = 188;
		bbf->bbf_payload_bytes_left = bbf->dfl;
		bbf-> syncbyte = 0x47;
	} else {
		bbf->upl = (buff[2] << 8) | buff[3];
		if((bbf->upl & 0x7) !=0) { //expect multiple of 8 bits
			dmx_demux_dprintk_nice(bbf, "Expected multiple of 8 bits");
		}
		bbf->upl >>=3; //now in bytes
		bbf->issy = 0;
		bbf->syncbyte = buff[3];
		bbf->bbf_payload_bytes_left = bbf->dfl;
	}

	bbf->syncd = (buff[7]<<8) | buff[8];
	if((bbf->syncd & 0x7) != 0) { //expect multiple of 8 bits
		dmx_demux_dprintk_nice(bbf, "Expected multiple of 8 bits");
	}
	bbf->syncd >>= 3; //now in bytes
}

static inline uint32_t extend_crc32(struct t2mi_stream* t2mi,  const uint8_t* data, int size) {
	int i;
	for (i = 0; i < size; i++) {
		t2mi->t2mi_crc32 = (t2mi->t2mi_crc32 << 8) ^ crc32_table[((t2mi->t2mi_crc32 >> 24) ^ (uint8_t)data[i])];
	}
	return t2mi->t2mi_crc32;
}

static inline uint8_t extend_crc8(struct bbframes_stream*bbf, const uint8_t *p, uint8_t size)
{
    int    i;
    uint8_t crc = bbf->bbf_crc8, tmp;
		int offset=0;
    for (i = 0; i < size; i++) {
        tmp = p[offset++];
        crc = crc8_table[crc ^ tmp];
    }
		bbf->bbf_crc8 = crc;
    return crc;
}

/*
	buffer the bytes extracted from an stid or t2mi bbframes stream and output complete TS packets.
	Returns NULL on error, else pointer to the next not yet read byte
 */
static const uint8_t* bbf_output_ts_bytes(struct dvb_demux* demux,
																					struct bbframes_stream* bbf,
																					const uint8_t* p, int num_ts_bytes)
{
	struct ts_stream* ts = &bbf->ts;
	bool bad_crc=false;
	int n = 0;
	int num_bytes_to_output = 0;
	bool ts_packet_complete = false;
	if(!ts->synced) {
		//virtually read the bytes of the first incomplete packet, so that they can be skipped
		int n = min(bbf->syncd - ts->buff_idx, num_ts_bytes);
		if(n<0) {
			dmx_demux_dprintk_nice(bbf, "unexpected n=%d syncd=%d, buff_idx=%d num_ts_bytes=%d\n", n,
														 bbf->syncd, ts->buff_idx, num_ts_bytes);
			return p+num_ts_bytes;
		}
		ts->buff_idx += n;
		num_ts_bytes -= n;
		p += n;
		if(ts->buff_idx == bbf->syncd) {
			ts->synced = true;
			ts->buff_idx =0;
		}
		bbf->bbf_payload_bytes_left -= n;
		if(bbf->bbf_payload_bytes_left < 0) {
			dmx_demux_dprintk_nice(bbf, "num_ts_bytes=%d n=%d bbf_payload_bytes_left=%d bbf->syncd=%d bbf->buff_idx=%d\n",
														 num_ts_bytes, n, bbf->bbf_payload_bytes_left, bbf->syncd, ts->buff_idx);
			bbf->bbf_payload_bytes_left = 0;
			return NULL;
		}
	}

	while (num_ts_bytes >0 && ts->buff_idx <= bbf->upl) {
		if(ts->buff_idx == 0 ) {
			if(bbf->hem_mode) {
				ts->buff[0] = 0x47;
				ts->buff_idx ++;
			} else {
				bad_crc = (bbf->bbf_crc8 != p[0] && bbf->bbf_crc8 >=0);
				if (bad_crc) {
					bbf_dprintk_nice(bbf, "Bad crc (bbf->bbf_crc8=%d  p[0]=%d\n", bbf->bbf_crc8,+ p[0]);
					bbf->num_crc_errors++;
					return NULL;
				}
				bbf->bbf_crc8 = 0;
				ts->buff[0] = 0x47;
				ts->buff_idx ++;
				num_ts_bytes--;
				p++;
				bbf->bbf_payload_bytes_left -= 1;
			}
		}
		n = min(bbf->upl - ts->buff_idx, num_ts_bytes);
		if (n==0)
			return p;
		num_bytes_to_output = min(bbf->bbf_payload_bytes_left, n);
		if (num_bytes_to_output > 0) {
			if (ts->buff_idx >=  (int)sizeof(ts->buff)) {
				dmx_demux_dprintk_nice(bbf, "IMPLEMENTATION ERROR: buffer overrun detected\n");
				return NULL;
			}
			int len = min(num_bytes_to_output, (int)sizeof(ts->buff) - ts->buff_idx);
			if(ts->buff_idx+ num_bytes_to_output >= sizeof(ts->buff)) {
				dmx_demux_dprintk_nice(bbf, "buffer oveflow %d/%d\n", ts->buff_idx+ num_bytes_to_output,
															 (int)sizeof(ts->buff));
				return p;
			}
			memcpy(ts->buff + ts->buff_idx, p, len);
			if(!bbf->hem_mode)
				extend_crc8(bbf, p, num_bytes_to_output);
			bbf->bbf_payload_bytes_left -= num_bytes_to_output;
		}
		p += n;
		ts->buff_idx += num_bytes_to_output;
		num_ts_bytes -= n;
		if(ts->buff_idx > bbf->upl) {
			dmx_demux_dprintk_nice(bbf, "implementation error: ts->buff_idx=%d bbf->upl=%d\n",
														 ts->buff_idx, bbf->upl);
			ts->buff_idx = bbf->upl;
			if(ts->buff_idx >= sizeof(ts->buff)) {
				dmx_demux_dprintk_nice(bbf, "buffer oveflow %d/%d\n", ts->buff_idx, (int)sizeof(ts->buff));
				return p;
			}
		}
		ts_packet_complete = ts->buff_idx == bbf->upl;
		if(ts_packet_complete) {
			dvb_dmx_swfilter_packet(demux, &ts->buff[0], bbf->feeds, true);
			ts->buff_idx = 0;
		}
		if(bbf->bbf_payload_bytes_left== 0) {
			p+= num_ts_bytes;
			break;
		}
	}
	return p;
}

/*
	Get the 3-byte t2mi_bbf header and the associated bbframe header
	In t2mi, exactly one bbframe header is included per t2mi packet and it is immediately
	after the packet header
	returns true if header is complete.
 */
static inline bool get_t2mi_bbheader(struct t2mi_stream* t2mi, const uint8_t** p, int num_bytes_available, const uint8_t* pend) {
	int num = min(num_bytes_available, (int)13 - t2mi->bbheader_idx);
	if(t2mi->bbheader_idx + num > sizeof(t2mi->buff)) {
		WARN_ON_ONCE("buffer overrun\n");
		t2mi_stream_dprintk_nice(t2mi, "resetting\n");
		t2mi_stream_reset(t2mi, false /*full_reset*/);
		*p = NULL;
		return true;
	}
	memcpy (&t2mi->buff[t2mi->bbheader_idx], *p, num);
	if(t2mi->bbheader_idx == 0) {
		t2mi->emb.bbf_header_crc8 =  0xFFFFFFFF;
	}
	(*p) += num;
	if (*p > pend) {
		WARN_ON_ONCE("Buffer overflow\n");
		t2mi_stream_dprintk_nice(t2mi, "resetting\n");
		t2mi_stream_reset(t2mi, false /*full_reset*/);
		*p = NULL;
		return true;
	}
	t2mi->bbheader_bytes_left -= num;
	t2mi->t2mi_payload_bytes_left -= num;
	t2mi->bbheader_idx += num;
	bool header_complete = t2mi->bbheader_idx == 13;
	if(!header_complete) {
#if 0
		t2mi_stream_dprintk_nice(t2mi, "INCOMPLETE: idx=%d available=%d num=%d\n",  t2mi->bbheader_idx,
														 num_bytes_available, num);
#endif
		return false;
	}
	uint8_t* buff = &t2mi->buff[0];
	extend_crc32(t2mi, buff, 13);

	//read t2mi specific prefix of bbheader
	t2mi->frame_idx = buff[0];
	t2mi->plp_id = buff[1];
	t2mi->intl_frame_start = !!(buff[12]>>8);
	buff +=3;

	//read 10 bytes bbheader
	t2mi->bbheader_idx = 0; //reset for next time
	if(t2mi->bbheader_bytes_left!=0) {
		t2mi_stream_dprintk_nice(t2mi, "implementation error\n");
		return true;
	}
	int crc = buff[9];
	int tstcrc= compute_crc8(&buff[0], 9);
	int hem_mode = tstcrc ^ crc;
	bool bad_crc  = !!(hem_mode &~1);
	if(bad_crc) {
		t2mi_stream_dprintk_nice(t2mi, "bad t2mi header crc\n");
		t2mi_stream_dprintk_nice(t2mi, "resetting\n");
		t2mi_stream_reset(t2mi, false /*full_reset*/);
		t2mi->num_crc8_errors++;
		*p = NULL;
		return true;
	}
	int matype = buff[0];
	int isi = buff[1];
	//t2mi_stream_dprintk_nice(t2mi, "ISI=%d plp=%d\n", isi, t2mi->plp_id);
	embedded_stream_set_matype(&t2mi->emb, t2mi->plp_id, matype);

	/*
		in theory t2mi streams can have more than one PLP (identified by plp_id,
		which is the same as isi) embedded in them, but almost always they have only one.
		Therefore we allow the called setting isi=T2MI_UNSPECIFIED_PLP as the stream_id,
		In that case we pick a stream_id at random, which will be fine if there is only one present.
	 */
	if(t2mi->default_isi < 0) {
		t2mi_stream_dprintk_nice(t2mi, "Setting default_isi=%d\n", t2mi->plp_id);
		t2mi->default_isi = t2mi->plp_id;
	}

	struct bbframes_stream* bbf = xa_load(&t2mi->emb.bbf_streams, t2mi->plp_id);
	if(!bbf &&  t2mi->plp_id == t2mi->default_isi) {
		bbf = xa_load(&t2mi->emb.bbf_streams, 256);
		t2mi_stream_dprintk_nice(t2mi, "Looked bbf=%p\n", bbf);
	}

	t2mi->emb.current_bbf = bbf;
	if(!bbf) {
#if 0
		t2mi_stream_dprintk_nice(t2mi, "return header_complete=%d: idx=%d available=%d num=%d\n",
														 header_complete,  t2mi->bbheader_idx, num_bytes_available, num);
#endif
		return header_complete;
	}
	if(buff - &t2mi->buff[0] +9 >= sizeof(t2mi->buff)) {
		WARN_ON_ONCE("BUG buffer overrun\n");
		t2mi_stream_dprintk_nice(t2mi, "resetting\n");
		t2mi_stream_reset(t2mi, false /*full_reset*/);
		*p = NULL;
		return true;
	}
	parse_bbheader(bbf, hem_mode, isi, buff);
	if(bbf->bbf_payload_bytes_left > t2mi->t2mi_payload_bytes_left) {
		t2mi_stream_dprintk_nice(t2mi, "Unexpected: bbf->bbf_payload_bytes_lef=%d > "
														 "t2mi->t2mi_payload_bytes_left=%d\n",
														 bbf->bbf_payload_bytes_left, t2mi->t2mi_payload_bytes_left);
		t2mi_stream_dprintk_nice(t2mi, "resetting\n");
		t2mi_stream_reset(t2mi, false /*full_reset*/);
		*p = NULL;
		return true;
	}
#if 0
	t2mi_stream_dprintk_nice(t2mi, "return header_complete=%d: idx=%d available=%d num=%d\n",
													 header_complete,  t2mi->bbheader_idx, num_bytes_available, num);
#endif
	return header_complete;
}


/*
	Get the 9-byte t2mi_bbf header and the associated bbframe header
	In t2mi, excatly one bbframe header is included per t2mi packet and it is immediately
	after the packet header
	returns true if this is the start of a bbframe
 */
inline static bool get_stid_header(struct stid_stream* stid, const uint8_t** p) {
	const uint8_t* buff = *p; //start of bbframes header
	struct bbframes_stream* bbf = NULL;
	if(buff[0] != 0x80 || buff[1] != 0x00) {
		dmx_demux_dprintk_nice(stid, "Unexpected stid header: 0x%x 0x%x\n", buff[0], buff[1]);
	}
	stid->section_length = buff[2]; //section length up to 0xb4
	int packet_count = buff[3];
	*p += 4;
	bool is_start = packet_count == 0xB8;
	if (is_start && stid->section_length != 0xB4) {
		dmx_demux_dprintk_nice(stid, "Unexpected section_length=%d\n", stid->section_length);
	}
	stid->emb.packet_count = is_start? 0 : packet_count;
	if(!is_start)
		return is_start;

	buff = *p; //start of bbframes header

	//read 10 bytes bbheader
	int crc = buff[9];
	int tstcrc= compute_crc8(&buff[0], 9);
	int hem_mode = tstcrc ^ crc;
	bool bad_crc  = !!(hem_mode &~1);
	if(bad_crc) {
		dmx_demux_dprintk_nice(stid,
													 "HEADER CRC error isi=%d crc=0x%02x 0x%02x\n",
													 stid->emb.current_isi, crc, tstcrc);
		return false;
	}
	int matype = buff[0];
	int isi = buff[1];
	bool is_sis = (matype >> 5)& 1;
	embedded_stream_set_matype(&stid->emb, isi, matype);
	if(is_sis)
		isi = 256;
	if(isi != stid->emb.current_isi) {
		bbf = xa_load(&stid->emb.bbf_streams, isi);
		stid->emb.current_bbf = bbf;
		stid->emb.current_isi = isi;
	} else {
		bbf = stid->emb.current_bbf;
		if(bbf && bbf->isi != isi)
			bbf_dprintk_nice(bbf, "loaded bbf for wrong isi\n");
	}
	if(!bbf)
		return false;
	if(!(packet_count == 0xB8 || ! bbf->synced))
		dmx_demux_dprintk_nice(bbf, "Inplementation error\n");
	parse_bbheader(bbf, hem_mode, isi, buff);
	*p += 10;
	return is_start;
}

#define STID_BBF_CHECK


/*
	returns true if header is complete
 */
static inline bool get_t2mi_packet_header(struct t2mi_stream* t2mi, const uint8_t** p, int num_bytes_available, const uint8_t* pend)
{
	if(num_bytes_available<0) {
		t2mi_stream_dprintk_nice(t2mi, "Unexpected\n");
		return false;
	}
	int num = min(num_bytes_available, (int)6 - t2mi->header_idx);
	if(t2mi->header_idx+num > sizeof(t2mi->buff)) {
		t2mi_stream_dprintk_nice(t2mi, "buffer overrun\n");
		return true;
	}
	num = min (num, (int)sizeof(t2mi->buff) - t2mi->header_idx);
	memcpy (&t2mi->buff[t2mi->header_idx], *p, num);
	if(t2mi->header_idx==0) {
		t2mi->crc_idx=0;
		t2mi->emb.payload_len=0;
		t2mi->t2mi_crc32 =  0xFFFFFFFF;
	}

	(*p) += num;
	if( *p > pend) {
		dprintk("buffer overflow\n");
		return true;
	}
	t2mi->header_idx += num;
	bool header_complete= t2mi->header_idx ==  6;
	if(!header_complete)
		return false;
	t2mi->header_idx = 0;
	int j = 0;
	uint8_t* buff = &t2mi->buff[0];

	t2mi->packet_type = buff[j+0];

	uint8_t packet_count = buff[j+1]; //incremented by one for each T2-MI packet sent, irrespective of payload.
	if( (t2mi->emb.packet_count >=0) // already initialized
			&& ((t2mi->emb.packet_count + 1)%256 != packet_count)) { //protocol violation
		t2mi->synced = false;
		return true;
	}
	t2mi->emb.packet_count = packet_count;
	t2mi->superframe_idx = buff[j+2]>>4; //should be incremented for each subsequent super-frame
#ifndef NDEBUG
	int rfu = ((buff[j+2]&0xf) << 8) | (buff[j+3]>>3); //reserved; should be zero
	if(rfu != 0) {
		t2mi_stream_dprintk_nice(t2mi, "unexpected rfu=%d\n", rfu);
	}
#endif
	t2mi->packet_header_isi = buff[j+3]&7; //if non zero then multiple streams are included

	t2mi->emb.payload_len = (buff[j+4]<<8)| buff[j+5];// indicates the payload length in bits.
	t2mi->emb.payload_len = (t2mi->emb.payload_len+7)/8; //now in bytes, including perhaps a few padding bits at the end
	t2mi->t2mi_payload_bytes_left = t2mi->emb.payload_len;
	if(t2mi->packet_type == 0)
		t2mi->bbheader_bytes_left =13;
	else
		t2mi->bbheader_bytes_left = 0;
	t2mi->bbheader_idx = 0;
	extend_crc32(t2mi, &t2mi->buff[0], 6);
#if 0
	t2mi_stream_dprintk_nice(t2mi, "HEADER: packet_type=%d isi=%d payload_len=%d header_idx=%d\n",
													 t2mi->packet_type, t2mi->emb.current_isi,
													 t2mi->emb.payload_len, t2mi->header_idx);
#endif
	return header_complete;
}

static inline bool t2mi_frame_is_complete(struct t2mi_stream* t2mi) {
	return ((t2mi->t2mi_payload_bytes_left==0) && (t2mi->crc_idx==4))|| !t2mi->synced;
}


static inline const uint8_t* skip_t2mi_bytes(struct t2mi_stream* t2mi, const uint8_t*p , int num_ts_bytes)
{
	return p + num_ts_bytes;
}

//dma
static void t2mi_stream_add_packet(struct dvb_demux* demux, struct t2mi_stream* t2mi,  const uint8_t* packet)
{
	if(!t2mi) {
		dprintk("BUG: no t2mi\n");
		return;
	}
#ifndef NDEBUG
	const uint8_t* p_pusi = NULL;
	bool pusi_check = false;
#endif
	//mpgeg ts and pes
	int pid = packet[1];
	pid&=0x01F;
	pid<<=8;
	pid|=(unsigned char)(packet[2]);

	if(pid != t2mi->emb.embedding_pid) {
		dmx_demux_dprintk_nice(demux, "pid=%d t2mi->emb.embedding_pid=%d\n", pid, t2mi->emb.embedding_pid);
		return;
	}
	int new_cc_counter = packet[3] & 0xf;
	int adaptation_field_control = (packet[3]>>4) & 3; //01=payload only
	if( (adaptation_field_control & 1)==0 ) { //payload is present
		t2mi_stream_dprintk_nice(t2mi, "No payload\n");
		return;
	}
	bool has_adaptation_field= !!(adaptation_field_control & 2);
	int tsc = (packet[3]>>6)&3;
	if(tsc) {
		t2mi_stream_dprintk_nice(t2mi, "Scrambled\n");
		return;
	}
	const uint8_t* p = &packet[4];
	bool discontinuity = false;
	if(has_adaptation_field) {
			int adaptation_field_len = *p;
			if(adaptation_field_len > 0) { //the value 0 is a special case to insert a single stuffing byte
				int b = p[1];
				discontinuity = !!(b & 8);
			}
			p += 1 + adaptation_field_len;
		}
		bool pusi = !!(packet[1]& 0x40);
		if (!t2mi->synced && ! pusi)
			return;
		bool cc_error = !discontinuity && new_cc_counter != (t2mi->emb.cc_counter+1)%16 && t2mi->emb.cc_counter>=0;

		if(cc_error) {
			t2mi_stream_dprintk_nice(t2mi, "CC counter error t2mi->emb.cc_counter=%d/%d\n", t2mi->emb.cc_counter, new_cc_counter);
			t2mi->num_cc_errors++;
			t2mi_stream_dprintk(t2mi, "resetting\n");
			t2mi_stream_reset(t2mi, false /*full_reset*/);
			return;
		}

		t2mi->emb.cc_counter = new_cc_counter;

		int uimsbf = 0; /*pointer to the location in this packet where next bbframe starts
											 -1 if no bbframe starts in the current TS packet */
		if(pusi) {
			//process t2mi packet header
			uimsbf = p[0];
			p++;
#ifndef NDEBUG
			p_pusi =  p + uimsbf;
			if(p_pusi -packet>=188) {
				dprintk("OUT OF RANGE\n");
				return;
			}
			pusi_check = false;
#endif
			if(!t2mi->synced) {
				//skip partial packet of incomplete first t2mi frame
				//t2mi->synced = true;
				p+= uimsbf;
				pusi_check = true;
				if(t2mi->header_idx>0) {
					t2mi_stream_dprintk_nice(t2mi, "Unexpected: incomplete header t2mi->header_idx=%d\n", t2mi->header_idx);
					t2mi->synced=false;
					t2mi->header_idx = 0;
					return;
				}
			}
		}
		//t2mi_stream_dprintk_nice(t2mi, "ENDING p-packet=%d\n", p-packet);
		int num_bytes = 188 - (p-packet);
		while (p< packet+188) {
			//t2mi_stream_dprintk_nice(t2mi, "here1\n");
			bool must_read_t2mi_header =  t2mi_frame_is_complete(t2mi) //t2mi frame has ended and a new one starts
				|| (t2mi->header_idx > 0); //we have read some but not all t2mi header bytes
#ifndef NDEBUG
			if(p == p_pusi && !must_read_t2mi_header) {
				t2mi_stream_dprintk_nice(t2mi, "Unexpected: pusi does not agree with must_read_t2mi_header\n");
			}
			pusi_check |= (p == p_pusi);
#endif
			if(must_read_t2mi_header) {
				num_bytes = 188-(p-packet);
				const uint8_t * pold = p;
				bool t2mi_header_complete = get_t2mi_packet_header(t2mi, &p, num_bytes, packet+188); //get next header;
				if( p-pold > num_bytes) {
					dprintk("BUG\n");
				}
				num_bytes -= (p-pold);
				//t2mi_stream_dprintk_nice(t2mi, "here0\n");
				if(p-packet > 188) {
					t2mi_stream_dprintk_nice(t2mi, "BUG\n");
					return;
				}
				if(num_bytes != 188-(p-packet)) {
					t2mi_stream_dprintk_nice(t2mi, "BUG\n");
					return;
				}
				if( !t2mi_header_complete && p !=packet+188) {
					t2mi_stream_dprintk_nice(t2mi, "BUG\n");
					return;
				}
				t2mi->synced=true;
				if(!t2mi_header_complete) {
					//t2mi_stream_dprintk_nice(t2mi, "here0b\n");
					return;
				}
				//t2mi_stream_dprintk_nice(t2mi, "here2\n");
			}
			bool must_read_bbheader =  (t2mi->bbheader_bytes_left > 0);
			if(must_read_bbheader) {
				if(num_bytes != 188-(p-packet)) {
					t2mi_stream_dprintk_nice(t2mi, "BUG\n");
					return;
				}
				const uint8_t * pold = p;
				bool bbheader_complete = get_t2mi_bbheader(t2mi, &p, num_bytes, packet+188); //get next header;
				if(!p)
					return; //something went wrong
				if(p-packet > 188) {
					t2mi_stream_dprintk_nice(t2mi, "BUG: read outside buffer\n");
					t2mi->synced = false;
					t2mi->header_idx = 0;
					return;
				}
				num_bytes -= (p-pold);
				if(num_bytes != 188 - (p-packet)) {
					t2mi_stream_dprintk_nice(t2mi, "BUG\n");
					return;
				}
				if(!bbheader_complete && p!=packet+188) {
					t2mi_stream_dprintk_nice(t2mi, "Unexpected\n");
					return;
				}
				if(!bbheader_complete)
					return;
			}
			if(!t2mi->emb.current_bbf)
				return;
			//t2mi_stream_dprintk_nice(t2mi, "here3\n");
			int bytes_left = t2mi->t2mi_payload_bytes_left;
			if(num_bytes != 188 - (p-packet)) {
				t2mi_stream_dprintk_nice(t2mi, "BUG\n");
				return;
			}
			int n = min(bytes_left, num_bytes);
			//t2mi_stream_dprintk_nice(t2mi, "here4 n=%d num_bytes=%d bytes_left=%d\n", n, num_bytes, bytes_left);
			if(n > 0) {
				extend_crc32(t2mi, p, n);
				if(t2mi->packet_type == 0 && t2mi->emb.current_bbf) {
					const uint8_t * pold = p;
					//t2mi_stream_dprintk_nice(t2mi, "OUT current_bbf=%p p=%p n=%d\n", t2mi->emb.current_bbf, p, n);
					p = bbf_output_ts_bytes(demux, t2mi->emb.current_bbf, p, n);
					//t2mi_stream_dprintk_nice(t2mi, "OUT DONE current_bbf=%p p=%p n=%d\n", t2mi->emb.current_bbf, p, n);
					WARN_ON(!p); //should never occur for a t2mi stream
					if(!p) {
						//a CRC error has occurred
						return;
					}
					n = p - pold;
					if(p-packet > 188) {
						t2mi_stream_dprintk_nice(t2mi, "BUG\n");
						return;
					}
					t2mi->t2mi_payload_bytes_left -= n;
					num_bytes -= n;
					if(num_bytes != 188- (p-packet)) {
						t2mi_stream_dprintk_nice(t2mi, "BUG\n");
						return;
					}
					if(t2mi->t2mi_payload_bytes_left < 0) {
						t2mi_stream_dprintk_nice(t2mi, "BUG\n");
						return;
					}
				} else {
					//t2mi_stream_dprintk_nice(t2mi, "skipping packet_type=%d\n", t2mi->packet_type);
					const uint8_t* pp = skip_t2mi_bytes(t2mi, p, n);
					p += n;
					t2mi->t2mi_payload_bytes_left -= n;
					num_bytes -= n;
					if(t2mi->t2mi_payload_bytes_left < 0) {
						t2mi_stream_dprintk_nice(t2mi, "BUG\n");
						return;
					}
					if(p != pp) {
						t2mi_stream_dprintk_nice(t2mi, "BUG\n");
						return;
					}
				}
			}
#if 0
			t2mi_stream_dprintk_nice(t2mi, "hereX  packet_type=%d isi=%d bbf=%p t2mi->t2mi_payload_bytes_left=%d "
															 "num_bytes=%d/%d crc_idx=%d\n",
															 t2mi->packet_type,
															 t2mi->emb.current_isi, t2mi->emb.current_bbf,
															 t2mi->t2mi_payload_bytes_left, num_bytes, p-packet, t2mi->crc_idx);
#endif
			if(num_bytes != 188- (p-packet)) {
				t2mi_stream_dprintk_nice(t2mi, "BUG\n");
				return;
			}
			if( (t2mi->t2mi_payload_bytes_left==0) && (num_bytes>0)) {
				if(t2mi->crc_idx >=4) {
					t2mi_stream_dprintk_nice(t2mi, "BUG: t2mi->crc_idx=%d\n", t2mi->crc_idx);
					return;
				}
				int n = min(4-t2mi->crc_idx, num_bytes);
				extend_crc32(t2mi, p, n);
				t2mi->crc_idx +=n;
				p+=n;
				if(p-packet > 188) {
					t2mi_stream_dprintk_nice(t2mi, "BUG\n");
					return;
				}
				num_bytes -=n;
#if 0
				if (num_bytes != 188 - (p-packet))
					t2mi_stream_dprintk_nice(t2mi, "hereYZ num_bytes=%lld/%lld\n", num_bytes, 188 - (p-packet));
				else
					t2mi_stream_dprintk_nice(t2mi, "hereY num_bytes=%lld/%lld\n", num_bytes, 188 - (p-packet));
#endif
				if(num_bytes != 188 - (p-packet)) {
					t2mi_stream_dprintk_nice(t2mi, "BUG\n");
					return;
				}
				if(t2mi->crc_idx ==4) {
					if(t2mi->t2mi_crc32 != 0) {
						t2mi_stream_dprintk_nice(t2mi, "crc error crc=%d\n", t2mi->t2mi_crc32);
						t2mi->num_crc32_errors++;
					}
				}
			}
		}
		if(p-packet != 188) {
			t2mi_stream_dprintk_nice(t2mi, "BUG: p-packet=%ld\n", p -packet);
		}
		if(p-packet !=188) {
			t2mi_stream_dprintk_nice(t2mi, "BUG\n");
			return;
		}
#ifndef NDEBUG
		if(p_pusi && !pusi_check) { // at the position indicated by pusi we must have read a header
			t2mi_stream_dprintk_nice(t2mi, "Unexpected\n");
		}
#endif
}

//dma
static void stid_stream_add_packet(struct dvb_demux* demux, struct stid_stream* stid,
																			 const uint8_t* packet)
{
	const uint8_t* p = NULL;
	bool discontinuity = false;
	bool bbframe_starts = false;
	int n = 0;
	int pid = packet[1];
	pid &= 0x01F;
	pid <<= 8;
	pid |= packet[2];
	if(pid != stid->emb.embedding_pid) {
		stid_stream_dprintk_nice(stid, "Unexpected pid: 0x%02x\n", pid);
		return;
	}

	int new_cc_counter = packet[3] & 0xf;
	bool pusi = !!(packet[1]& 0x40); //this type of stream has pusi set in each packet

#ifdef STID_BBF_CHECK
	int adaptation_field_control = (packet[3]>>4) & 3; //01=payload only
	bool has_adaptation_field= !!(adaptation_field_control & 2);
	bool has_payload = !!(adaptation_field_control & 1);
	int tsc = (packet[3]>>6)&3; //transport scrambling control
	if (has_adaptation_field)
		dmx_demux_dprintk_nice(stid, "unexpected adaptation field"); //should never happen with stid135 chip
	if (tsc)
		dmx_demux_dprintk_nice(stid, "scrambled"); //should never happen with stid135 chip
	if (!has_payload) //payload should be present
		dmx_demux_dprintk_nice(stid, "Error no payload"); //should never happen with stid135 chip
	if (!pusi) //payload should be present
		dmx_demux_dprintk_nice(stid, "Unexpected pusi"); //should never happen with stid135 chip
#endif

	p = &packet[4];
	//p++;

#ifdef STID_BBF_CHECK
	if(has_adaptation_field) {
		int adaptation_field_len = *p;
		if(adaptation_field_len > 0) { //the value 0 is a special case to insert a single stuffing byte
			int b = p[1];
			discontinuity = !!(b & 8);
		}
		p += 1 + adaptation_field_len;
	}
	bool cc_error = !discontinuity && new_cc_counter != (stid->emb.cc_counter+1)%16 && stid->emb.cc_counter>=0;
	if(cc_error) {
		dmx_demux_dprintk_nice(stid, "CC error\n");
	}
#endif
	p++;
	bbframe_starts = get_stid_header(stid, &p); //get next header;
	if(!stid->emb.current_bbf || (!stid->emb.current_bbf->synced && !bbframe_starts))
		return;
	stid->emb.current_bbf->synced = true;
	n = 188 - (p-packet);
	if( n<0) {
		dmx_demux_dprintk_nice(stid->emb.current_bbf, "Implementation error n=%d\n", n);
	} else if (n>0) {
		if(stid->emb.current_bbf) {
			const uint8_t * pold = p;
			p = bbf_output_ts_bytes(demux, stid->emb.current_bbf, p, n);
			if(!p) {
				// a crc error has occured
				stid_stream_reset(stid);
				return;
			}
			n = p - pold;
			if(p-packet != 188) {
				dmx_demux_dprintk_nice(stid->emb.current_bbf, "Implementation error: p-packet=%d\n", (int) (p-packet));
			}
		} else {
			p = packet+188; //skip
		}
	}
}

/*
	Process a packet received from sub_demux
 */
//dma
static void dvb_dmx_swfilter_packet(struct dvb_demux *demux, const uint8_t *buf,
																		struct dvb_demux_feeds* feeds, bool frombbf)
{
	struct dvb_demux_feed *feed;
	u16 pid = ts_pid(buf);
	int dvr_done = 0;
	enum dmx_buffer_flags buffer_flags;
	bool flag_error = false;
	if(!demux) {
		dmx_demux_dprintk_nice(demux, "demux=NULL\n");
		return;
	}
	if(!feeds) {
		dmx_demux_dprintk_nice(demux, "BUG feeds=NULL\n");
		return;
	}
	if (dvb_demux_speedcheck) {
		ktime_t cur_time;
		u64 speed_bytes, speed_timedelta;

		feeds->speed_pkts_cnt++;

		/* show speed every SPEED_PKTS_INTERVAL packets */
		if (!(feeds->speed_pkts_cnt % SPEED_PKTS_INTERVAL)) {
			cur_time = ktime_get();

			if (ktime_to_ns(feeds->speed_last_time) != 0) {
				speed_bytes = (u64)feeds->speed_pkts_cnt
					* 188 * 8;
				/* convert to 1024 basis */
				speed_bytes = 1000 * div64_u64(speed_bytes,
						1024);
				speed_timedelta = ktime_ms_delta(cur_time,
							feeds->speed_last_time);
				if (speed_timedelta)
					dprintk("TS speed %llu Kbits/sec \n",
						div64_u64(speed_bytes,
							  speed_timedelta));
			}

			feeds->speed_last_time = cur_time;
			feeds->speed_pkts_cnt = 0;
		}
	}

	if (buf[1] & 0x80) { //process packet with stream error
		//do not pass on erroneous packet to sub_demux, but flag it in all output feeds
		buffer_flags = DMX_BUFFER_FLAG_TEI;
		flag_error = true;
		dprintk_tscheck("TEI detected. PID=0x%x data1=0x%x\n",
				pid, buf[1]);
		/* data in this packet can't be trusted - drop it unless
		 * module option dvb_demux_feed_err_pkts is set */
	} else /* if TEI bit is not set, detect continuity errors and flaf them with TEI=1  */
		if (feeds->cnt_storage && dvb_demux_tscheck) {
			/* check pkt counter */
			if (pid < MAX_PID) {
				//first initialize CC counter if needed
				if (buf[3] & 0x10) { //payload present
					if(feeds->cnt_storage[pid] == 0xff) { //if counter was not initialized
						feeds->cnt_storage[pid] = (buf[3] & 0xf); //initialize counter
					} else {
						feeds->cnt_storage[pid] = (feeds->cnt_storage[pid] + 1) & 0xf; //set expected CC counter value
					}
				} else if(feeds->cnt_storage[pid]==0xff) { //if counter was not initialized
					feeds->cnt_storage[pid] = (buf[3] & 0xf); //initialize counter
				}

				if ((buf[3] & 0xf) != feeds->cnt_storage[pid]) { //CC error detected
					//do not pass on packet to sub demuxes, but flag error in output streams

					buffer_flags = DMX_BUFFER_PKT_COUNTER_MISMATCH;
					flag_error = true;
					dprintk_tscheck("TS packet counter mismatch. PID=%d expected 0x%x got 0x%x\n",
													pid, feeds->cnt_storage[pid],
													buf[3] & 0xf);
					feeds->cnt_storage[pid] = buf[3] & 0xf;
				}
			}
			/* end check */
		}

	/*first pass on embedded frames streams. In this case, there should be no TS errors, so flag_error should be false,
		but we check anyway
	*/
	if(!flag_error)  {
		struct embedded_stream* emb =  (struct embedded_stream*)xa_load(&feeds->embedded_streams, pid);
		struct stid_stream* stid =  embedded_stream_get_super_class(emb, EMBEDDED_STREAM_TYPE_STID);
		//dmx_demux_dprintk_nice(demux, "emb=%p stid=%p\n", emb, stid);
		if(stid) {
			//dmx_demux_dprintk_nice(demux, "calling stid_stream_add_packet stid=%p pid=%d from_bbf=%d\n", stid, pid, frombbf);
			stid_stream_add_packet(demux, stid, buf);
		}
	}

	bool try_default_feeds =  feeds->include_default_feeds && feeds != demux->default_feeds;

	for(int i=0; i < (try_default_feeds ? 2 : 1); ++i) {
		if(i>0) {
			feeds = demux->default_feeds;
			if(!feeds) {
				dmx_demux_dprintk_nice(demux, "No default feeds but include_default_feeds set\n");
				return;
			}
		}
		struct embedded_stream* emb =  (struct embedded_stream*)xa_load(&feeds->embedded_streams, pid);
		struct t2mi_stream* t2mi =  embedded_stream_get_super_class(emb, EMBEDDED_STREAM_TYPE_T2MI);
		//dmx_demux_dprintk_nice(demux, "emb=%p t2mi=%p pid=%d\n", emb, t2mi, pid);
		if(!flag_error && t2mi) {
			t2mi_stream_add_packet(demux, t2mi, buf);
			//do not return, as it is possible that some demux users want the pid itself, rather than the embedded stream
		}
		list_for_each_entry(feed, &feeds->output_feed_list, next) {
			if ((feed->pid != pid) && (feed->pid != 0x2000))
				continue; //skip packets with non-matching PID
			if (flag_error)  {
				set_buf_flags(feed, DMX_BUFFER_PKT_COUNTER_MISMATCH); //flag the CC error
				if (!dvb_demux_feed_err_pkts)
					continue;
			}
			//dvb_demux_feed_dprintk_nice(feed, "DVR_FEED=%d dvr_done=%d pid=%d\n", DVR_FEED(feed), dvr_done, pid);
			/* copy each packet only once to the dvr device, even
			 * if a PID is in multiple filters (e.g. video + PCR) */
			if ((DVR_FEED(feed)) && (dvr_done++))
				continue;
#if 0
			if(frombbf) {
				dmx_demux_dprintk_nice(demux, "HERE i=%d pid=%d feed=%p feed->pid=%d\n", i, pid, feed, feed->pid);
			}
#endif
			if (feed->pid == pid) { //for a matching packet, convert data to ES if needed, or extract section data
#if 0
				dmx_demux_dprintk_nice(demux, "would output packet for pid=%d\n", pid);
#else
				//dmx_demux_dprintk_nice(demux, "calling dvb_dmx_swfilter_packet feed=%p pid=%d from_bbf=%d\n", feed, pid,
				//											 frombbf);

				//Output packet (after possible header stripping or section processing to the demux
				dvb_dmx_swfilter_packet_type(feed, buf);
#endif
			}
			else if (feed->pid == 0x2000) { //except when full TS is requested, then only copy packets as-is
#if 0
				dmx_demux_dprintk_nice(demux, "would output packet for pid=0x2000\n");
#else
				//dmx_demux_dprintk_nice(demux, "calling cb.ts feed=%p pid=%d from_bbf=%d\n", feed, pid,
				//											 frombbf);

				//Output packet to the demux without further processing
				feed->cb.ts(buf, 188, NULL, 0, &feed->feed.pid_stream, &feed->buffer_flags);
#endif
			}
		}
	}
}

//dma
void dvb_dmx_swfilter_packets(struct dvb_demux *demux, const uint8_t *buf, size_t count)
{
	unsigned long flags;
	spin_lock_irqsave(&demux->lock, flags);
	while (count--) {
		if (buf[0] == 0x47) {
			dvb_dmx_swfilter_packet(demux, buf, demux->fe_feeds, false /*frombbf*/);
		} buf += 188;
	}

	spin_unlock_irqrestore(&demux->lock, flags);
}

EXPORT_SYMBOL(dvb_dmx_swfilter_packets);

static inline int find_next_packet(const u8 *buf, int pos, size_t count,
				   const int pktsize)
{
	int start = pos, lost;

	while (pos < count) {
		if (buf[pos] == 0x47 ||
		    (pktsize == 204 && buf[pos] == 0xB8))
			break;
		pos++;
	}

	lost = pos - start;
	if (lost) {
		/* This garbage is part of a valid packet? */
		int backtrack = pos - pktsize;
		if (backtrack >= 0 && (buf[backtrack] == 0x47 ||
		    (pktsize == 204 && buf[backtrack] == 0xB8)))
			return backtrack;
	}

	return pos;
}

/* Filter all pktsize= 188 or 204 sized packets and skip garbage. */
static inline void _dvb_dmx_swfilter(struct dvb_demux *demux, const u8 *buf,
																		 size_t count, const int pktsize)
{
	int p = 0, i, j;
	const u8 *q;
	unsigned long flags;

	spin_lock_irqsave(&demux->lock, flags);

	if (demux->tsbufp) { /* tsbuf[0] is now 0x47. */
		i = demux->tsbufp;
		j = pktsize - i;
		if (count < j) {
			memcpy(&demux->tsbuf[i], buf, count);
			demux->tsbufp += count;
			goto bailout;
		}
		memcpy(&demux->tsbuf[i], buf, j);
		if (demux->tsbuf[0] == 0x47) /* double check */
			dvb_dmx_swfilter_packet(demux, demux->tsbuf, demux->fe_feeds, false);
		demux->tsbufp = 0;
		p += j;
	}

	while (1) {
		p = find_next_packet(buf, p, count, pktsize);
		if (p >= count)
			break;
		if (count - p < pktsize)
			break;

		q = &buf[p];

		if (pktsize == 204 && (*q == 0xB8)) {
			memcpy(demux->tsbuf, q, 188);
			demux->tsbuf[0] = 0x47;
			q = demux->tsbuf;
		}
		dvb_dmx_swfilter_packet(demux, q, demux->fe_feeds, false);
		p += pktsize;
	}

	i = count - p;
	if (i) {
		memcpy(demux->tsbuf, &buf[p], i);
		demux->tsbufp = i;
		if (pktsize == 204 && demux->tsbuf[0] == 0xB8)
			demux->tsbuf[0] = 0x47;
	}

bailout:
	spin_unlock_irqrestore(&demux->lock, flags);
}

void dvb_dmx_swfilter(struct dvb_demux *demux, const u8 *buf, size_t count)
{
	_dvb_dmx_swfilter(demux, buf, count, 188);
}
EXPORT_SYMBOL(dvb_dmx_swfilter);

void dvb_dmx_swfilter_204(struct dvb_demux *demux, const u8 *buf, size_t count)
{
	_dvb_dmx_swfilter(demux, buf, count, 204);
}
EXPORT_SYMBOL(dvb_dmx_swfilter_204);

void dvb_dmx_swfilter_raw(struct dvb_demux *demux, const u8 *buf, size_t count)
{
	unsigned long flags;
	struct dvb_demux_feed* feed = & demux->feedarray[0];
	spin_lock_irqsave(&demux->lock, flags);

	feed->cb.ts(buf, count, NULL, 0, &feed->feed.pid_stream,
			   &feed->buffer_flags);

	spin_unlock_irqrestore(&demux->lock, flags);
}
EXPORT_SYMBOL(dvb_dmx_swfilter_raw);

static struct dvb_demux_section_filter *dvb_dmx_section_filter_alloc(struct dvb_demux *demux)
{
	int i;

	for (i = 0; i < demux->filternum; i++)
		if (demux->section_filter[i].state == DMX_STATE_FREE)
			break;

	if (i == demux->filternum)
		return NULL;

	demux->section_filter[i].state = DMX_STATE_ALLOCATED;
	return &demux->section_filter[i];
}

static struct dvb_demux_feed *dvb_dmx_feed_alloc(struct dvb_demux *demux)
{
	int i;

	for (i = 0; i < demux->feednum; i++)
		if (demux->feedarray[i].state == DMX_STATE_FREE)
			break;

	if (i == demux->feednum)
		return NULL;

	demux->feedarray[i].state = DMX_STATE_ALLOCATED;

	return &demux->feedarray[i];
}

/*find sub feed of  parent_feed or to the feedless default sub_demux if parent_feed==NULL*/
static int dvb_demux_output_feed_find(struct dvb_demux_feed *feed)
{
	struct dvb_demux_feed *entry;
	struct dvb_demux_feeds* feeds =  feed->parent_feeds;
	WARN_ON(!feeds);
	dprintk("feeds->output_feed_list=%p\n",  &feeds->output_feed_list);
	list_for_each_entry(entry, &feeds->output_feed_list, next)
		if (entry == feed)
			return 1;

	return 0;
}

/*add a feed to a parent_feed or to the feedless default sub_demux if parent_feed==NULL*/
static void dvb_demux_output_feed_add(struct dvb_demux_feed *feed)
{
	struct dvb_demux_feeds* feeds = feed->parent_feeds;
	WARN_ON(!feeds);
	dprintk("feeds=%p feeds=%p\n", feeds, feeds);
	spin_lock_irq(&feed->demux->lock);
	if (dvb_demux_output_feed_find(feed)) {
		pr_err("%s: feed already in list (type=%x state=%x pid=%x)\n",
		       __func__, feed->type, feed->state, feed->pid);
		goto out;
	}
	dprintk("before list_add feed=%p list=%p\n", &feed->next, &feeds->output_feed_list);
	list_add(&feed->next, &feeds->output_feed_list);
	kref_get(&feeds->refcount);
	feeds_dprintk(feeds, "After kref_get: feeds.refcount=%d\n",
								atomic_read(&feeds->refcount.refcount.refs));
out:
	spin_unlock_irq(&feed->demux->lock);
}


static void dvb_demux_output_feed_del(struct dvb_demux_feed *feed)
{
	struct dvb_demux_feeds* feeds =  feed->parent_feeds;
	WARN_ON(!feeds);
	spin_lock_irq(&feed->demux->lock);
	if (!(dvb_demux_output_feed_find(feed))) {
		pr_err("%s: feed not in list (type=%x state=%x pid=%x)\n",
		       __func__, feed->type, feed->state, feed->pid);
		goto out;
	}

	feeds_dprintk(feeds, "before list_del feed->next=%p\n", &feed->next);
	list_del(&feed->next);
	feeds_dprintk(feeds, "After list_del feed->next=%p\n", &feed->next);
	feeds_dprintk(feeds, "calling kref_put feeds.refcount=%d\n",
								atomic_read(&feeds->refcount.refcount.refs));
	kref_put(&feeds->refcount, dvb_demux_feeds_release_);
out:
	spin_unlock_irq(&feed->demux->lock);
}

static int demux_ts_feed_start_filtering(struct pid_stream *pid_feed)
{
	struct dvb_demux_feed *feed = container_of(pid_feed, struct dvb_demux_feed, feed.pid_stream);
	struct dvb_demux *demux = feed->demux;
	struct dvb_demux_feeds* feeds = feed->parent_feeds;
	WARN_ON(!feeds);
	int ret;

	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	if (feed->state != DMX_STATE_READY || feed->type != DMX_TYPE_TS) {
		mutex_unlock(&demux->mutex);
		return -EINVAL;
	}

	if (!demux->start_feed) {
		mutex_unlock(&demux->mutex);
		return -ENODEV;
	}
	{
		int i;
		for(i=0; i<MAX_PID; ++i)
			feeds->cnt_storage[i] = 0xff;
	}
	if ((ret = demux->start_feed(feed)) < 0) {
		mutex_unlock(&demux->mutex);
		return ret;
	}

	spin_lock_irq(&demux->lock);
	pid_feed->is_filtering = 1;
	feed->state = DMX_STATE_GO;
	spin_unlock_irq(&demux->lock);
	mutex_unlock(&demux->mutex);

	return 0;
}

static int demux_ts_feed_stop_filtering(struct pid_stream *ts_feed)
{
	struct dvb_demux_feed *feed = (struct dvb_demux_feed *)ts_feed;
	struct dvb_demux *demux = feed->demux;
	int ret;

	mutex_lock(&demux->mutex);

	if (feed->state < DMX_STATE_GO) {
		mutex_unlock(&demux->mutex);
		return -EINVAL;
	}

	if (!demux->stop_feed) {
		mutex_unlock(&demux->mutex);
		return -ENODEV;
	}

	ret = demux->stop_feed(feed);


	spin_lock_irq(&demux->lock);
	ts_feed->is_filtering = 0;
	feed->state = DMX_STATE_ALLOCATED;
	spin_unlock_irq(&demux->lock);
	mutex_unlock(&demux->mutex);

	return ret;
}

static struct bbframes_stream* dvb_dmx_find_or_alloc_bbf_stream
(struct dvb_demux* demux, struct embedded_stream* emb, int embedding_pid, int isi) {
	struct bbframes_stream* bbf = NULL;
	struct bbframes_stream* old_bbf = NULL;
	dprintk("called with demux=%p emb=%p embedding_pid=%d isi=%d bbf_streams=%p\n",
					demux, emb, embedding_pid, isi, emb? &emb->bbf_streams : (struct xarray*) NULL);
	if (isi <0)
		isi = 256;
	bbf = xa_load(&emb->bbf_streams, isi);
	if(!bbf) {
		bbf = kzalloc(sizeof(struct bbframes_stream), GFP_KERNEL);
		dprintk("ALLOC bbf=%p stid.refcount=%d embedding_pid=%d isi=%d\n", bbf,
						atomic_read(&emb->refcount.refcount.refs), embedding_pid, isi);
		bbframes_stream_init(bbf, emb, demux, embedding_pid, isi);
		dprintk("inited bbf\n");
		bbf->isi = isi;
		bbf->bbf_crc8 = -1;
		old_bbf = xa_cmpxchg(&emb->bbf_streams, isi, NULL /*old*/, bbf, GFP_KERNEL);
		dprintk("registered bbf for isi=%d bbf=%p old_bbf=%p bbf.refcount=%d\n",
						isi, bbf, old_bbf,
						atomic_read(&bbf->refcount.refcount.refs));
	} else {
		dprintk("found=%p\n", bbf);
		kref_get(&bbf->refcount);
		dprintk("incremented refcount for isi=%d bbf.refcount=%d\n", isi, atomic_read(&bbf->refcount.refcount.refs));
	}
	if(old_bbf) {
		kref_put(&bbf->refcount, bbframes_stream_release_);
		kref_get(&old_bbf->refcount);
		bbf = old_bbf;
		dprintk("incremented bbf refcount bbf=%p bbf.refcount=%d\n", old_bbf,
						atomic_read(&old_bbf->refcount.refcount.refs));
	} else {
		dprintk("normal no change in refcount bbf=%p bbf.refcount=%d\n", bbf,
						atomic_read(&bbf->refcount.refcount.refs));
	}
	return bbf;
}

static int dvbdmx_allocate_stid_stream_(struct dvb_demux* demux,
																				struct bbframes_stream** bbf_ret,
																				int embedding_pid, int embedded_isi,
																				struct dvb_demux_feeds* parent_feeds)
{
	struct bbframes_stream* bbf =NULL;
	struct embedded_stream* emb = NULL;
	struct stid_stream* stid = NULL;
	struct embedded_stream* old_emb =NULL;
	if(!parent_feeds)
		parent_feeds= demux->fe_feeds;
	WARN_ON(!parent_feeds);
	dprintk("embedded_pid=%d embedded_isi=%d fe_feeds=%p default_feeds=%p parent_feeds=%p parent_feeds->embedded_streams=%p\n",
					embedding_pid, embedded_isi, demux->fe_feeds,
					demux->default_feeds,
					parent_feeds, &parent_feeds->embedded_streams);
	//There is only stid_stream per pid, no matter how many users.
	dprintk("before parent_feeds=%p\n", parent_feeds);
	emb = xa_load(&parent_feeds->embedded_streams, embedding_pid);
	dprintk("here emb=%p\n", emb);
	if(!emb) {
		if (!(stid = kzalloc(sizeof(struct stid_stream), GFP_KERNEL))) {
			dprintk("could not allocate stid_stream for embedded_pid=0x%04x isi=%d\n",
							embedding_pid, embedded_isi);
			return -EBUSY;
		}
		emb = &stid->emb;
		dprintk("calling stid_stream_init\n");
		stid_stream_init(stid, parent_feeds, embedding_pid);
		dprintk("ALLOC stid=%p embedded_isi=%d dvbdemux->feedarray=%p dvbdemux=%p stid.refcount=%d\n",
						stid, embedded_isi, demux->feedarray, demux, atomic_read(&emb->refcount.refcount.refs));

		old_emb = xa_cmpxchg(&parent_feeds->embedded_streams, embedding_pid, NULL /*old*/, stid, GFP_KERNEL);
	} else {
		dprintk("here2 emb=%p\n", emb);
		kref_get(&emb->refcount);
		dprintk("incremented refcount stid.refcount=%d\n", atomic_read(&emb->refcount.refcount.refs));
		stid = embedded_stream_get_super_class(emb, EMBEDDED_STREAM_TYPE_STID);
		dprintk("here3\n");
	}
	if(old_emb) {
		embedded_stream_release(emb);
		dprintk("RELEASED OLD emb=%p\n", emb);
		emb = old_emb;
		stid = embedded_stream_get_super_class(emb, EMBEDDED_STREAM_TYPE_STID);
		kref_get(&emb->refcount);
		dprintk("reused stid=%p stid.refcount=%d\n", stid, atomic_read(&stid->emb.refcount.refcount.refs));
	} else {
		dprintk("normal stid=%p stid.refcount=%d\n", stid, atomic_read(&stid->emb.refcount.refcount.refs));
	}
	dprintk("here5\n");
	if(!stid) {
		/*this could happen when two demux users have conflicting opinions on the type of an
			embedded stream; only one type can be correct. A more graceul way would be to allow
				two conflicting types to co-exist (e.g., use two xarrays, one for stid and one for t2mi,
				and process the same TS stream twice, once as stid and once as t2mi. The solution below is to just
				log the error and then return a stream_ret in the wrong type of data structure.
		*/
		embedded_stream_dprintk(emb, "attempting to allocate an stid stream for other embedded stream\n");
	}

	bbf = dvb_dmx_find_or_alloc_bbf_stream(demux, emb, embedding_pid, embedded_isi);
	dprintk("called  dvb_dmx_find_or_alloc_bbf bbf=%p\n", bbf);
	embedded_stream_dprintk(emb, "before dvb_dmx_feed_alloc feeds=%p bbf=%p\n", emb->parent_feeds, bbf);
	WARN_ON(!bbf->feeds);

	*bbf_ret = bbf;

	return 0;
}

static int dvbdmx_allocate_stid_stream(struct dmx_demux* dmx_demux,
																					 struct dmx_stid_stream* stream_ret,
																					 int embedding_pid, int embedded_isi,
																					 struct dvb_demux_feeds* parent_feeds)
{
	struct dvb_demux* demux =  container_of(dmx_demux, struct dvb_demux, dmx);
	struct bbframes_stream* bbs =NULL;
	int ret;
	if (mutex_lock_interruptible(&demux->mutex)) {
		dmx_demux_dprintk(dmx_demux, "could not lock");
		return -ERESTARTSYS;
	}

	ret=dvbdmx_allocate_stid_stream_(demux, &bbs, embedding_pid, embedded_isi, parent_feeds);
	if(ret>=0) {
		stream_ret->stream = bbs;
		stream_ret->feeds = bbs->feeds;
	}
	mutex_unlock(&demux->mutex);
	return ret;
}


static struct dvb_demux_feeds* dvbdmx_get_fe_feeds(struct dmx_demux* dmx_demux)
{
	struct dvb_demux* demux =  container_of(dmx_demux, struct dvb_demux, dmx);
	return demux->fe_feeds;
}

static int dvbdmx_allocate_t2mi_stream_(struct dvb_demux* demux,
																				struct bbframes_stream** bbf_ret,
																				int embedding_pid, int embedded_isi,
																				struct dvb_demux_feeds* parent_feeds)
{
	struct bbframes_stream* bbf =NULL;
	struct embedded_stream* emb = NULL;
	struct t2mi_stream* t2mi = NULL;
	struct embedded_stream* old_emb =NULL;
	if(!parent_feeds)
		parent_feeds= demux->default_feeds;
	WARN_ON(!parent_feeds);
	dprintk("embedded_pid=%d embedded_isi=%d fe_feeds=%p default_feeds=%p feeds=%p\n",
					embedding_pid, embedded_isi, demux->fe_feeds,
					demux->default_feeds,
					parent_feeds);
	//There is only t2mi_stream per pid, no matter how many users.
	emb = xa_load(&parent_feeds->embedded_streams, embedding_pid);
	if(!emb) {
		if (!(t2mi = kzalloc(sizeof(struct t2mi_stream), GFP_KERNEL))) {
			dprintk("could not allocate t2mi_stream for embedded_pid=0x%04x isi=%d\n",
							embedding_pid, embedded_isi);
			return -EBUSY;
		}
		emb = &t2mi->emb;
		t2mi_stream_init(t2mi, parent_feeds, embedding_pid);
		dprintk("ALLOC t2mi=%p embedded_isi=%d dvbdemux->feedarray=%p dvbdemux=%p t2mi.refcount=%d\n",
						t2mi, embedded_isi, demux->feedarray, demux, atomic_read(&emb->refcount.refcount.refs));

		old_emb = xa_cmpxchg(&parent_feeds->embedded_streams, embedding_pid, NULL /*old*/, t2mi, GFP_KERNEL);
	} else {
		kref_get(&emb->refcount);
		dprintk("incremented refcount t2mi.refcount=%d\n", atomic_read(&emb->refcount.refcount.refs));
		t2mi = embedded_stream_get_super_class(emb, EMBEDDED_STREAM_TYPE_T2MI);
	}
	if(old_emb) {
		embedded_stream_release(emb);
		dprintk("RELEASED OLD emb=%p\n", emb);
		emb = old_emb;
		t2mi = embedded_stream_get_super_class(emb, EMBEDDED_STREAM_TYPE_T2MI);
		kref_get(&emb->refcount);
		dprintk("reused t2mi=%p t2mi.refcount=%d\n", t2mi, atomic_read(&emb->refcount.refcount.refs));
	} else {
		dprintk("normal t2mi=%p t2mi.refcount=%d\n", t2mi, atomic_read(&emb->refcount.refcount.refs));
	}

	if(!t2mi) {
		/*this could happen when two demux users have conflicting opinions on the type of an
			embedded stream; only one type can be correct. A more graceful way would be to allow
			two conflicting types to co-exist (e.g., use two xarrays, one for stid and one for t2mi,
			and process the same TS stream twice, once as stid and once as t2mi. The solution below is to just
			log the error and then return a stream_ret in the wrong type of data structure.
		*/
		embedded_stream_dprintk(emb, "attempting to allocate an stid stream for other embedded stream\n");
	}

	bbf = dvb_dmx_find_or_alloc_bbf_stream(demux, emb, embedding_pid, embedded_isi);
	dprintk("called  dvb_dmx_find_or_alloc_bbf stream=%p\n", bbf);
	embedded_stream_dprintk(emb, "before dvb_dmx_feed_alloc feeds=%p stream=%p\n", emb->parent_feeds, bbf);
	WARN_ON(!bbf->feeds);

	*bbf_ret = bbf;

	return 0;
}

static int dvbdmx_allocate_t2mi_stream(struct dmx_demux* dmx_demux,
																			 struct dmx_t2mi_stream* stream_ret,
																			 int embedding_pid, int embedded_isi,
																			 struct dvb_demux_feeds* parent_feeds)
{
	struct dvb_demux* demux =  container_of(dmx_demux, struct dvb_demux, dmx);
	struct bbframes_stream* bbs =NULL;
	int ret;
	if (mutex_lock_interruptible(&demux->mutex)) {
		dmx_demux_dprintk(dmx_demux, "could not lock");
		return -ERESTARTSYS;
	}

	ret = dvbdmx_allocate_t2mi_stream_(demux, &bbs, embedding_pid, embedded_isi, parent_feeds);
	if(ret>=0) {
		stream_ret->stream = bbs;
		stream_ret->feeds = bbs->feeds;
	}
	mutex_unlock(&demux->mutex);
	return ret;
}

static int dvbdmx_allocate_pid_stream(struct dmx_demux *dmx_demux, struct pid_stream **ts_feed,
																	 dmx_pid_cb callback, u16 pid, int ts_type,
																	 enum dmx_ts_pes pes_type, ktime_t timeout,
																	 struct dvb_demux_feeds* parent_feeds)
{
	struct dvb_demux* demux =  container_of(dmx_demux, struct dvb_demux, dmx);
	struct dvb_demux_feed *feed=0;

	if (pid > DMX_MAX_PID)
		return -EINVAL;

	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	if (!(feed = dvb_dmx_feed_alloc(demux))) {
		mutex_unlock(&demux->mutex);
		return -EBUSY;
	}

	if(!parent_feeds)
		parent_feeds= demux->default_feeds;
	WARN_ON(!parent_feeds);

	feed->type = DMX_TYPE_TS;
	feed->cb.ts = callback;
	feed->demux = demux;
	feed->pid = 0xffff;
	feed->peslen = 0xfffa;
	feed->buffer_flags = 0;
	feed->parent_feeds = parent_feeds;

	(*ts_feed) = &feed->feed.pid_stream;
	(*ts_feed)->priv = NULL;
	(*ts_feed)->is_filtering = 0;
	(*ts_feed)->start_filtering = demux_ts_feed_start_filtering;
	(*ts_feed)->stop_filtering = demux_ts_feed_stop_filtering;

	if (ts_type & TS_DECODER) {
		if (pes_type >= DMX_PES_OTHER) {
			mutex_unlock(&demux->mutex);
			return -EINVAL;
		}

		if (demux->pesfilter[pes_type] &&
		    demux->pesfilter[pes_type] != feed) {
			mutex_unlock(&demux->mutex);
			return -EINVAL;
		}

		demux->pesfilter[pes_type] = feed;
		demux->pids[pes_type] = pid;
	}

	dprintk("before dvb_demux_output_feed_add feed=%p parent_feeds=%p\n", feed, parent_feeds);
	dvb_demux_output_feed_add(feed);

	feed->pid = pid;
	feed->timeout = timeout;
	feed->ts_type = ts_type;
	feed->pes_type = pes_type;

	feed->state = DMX_STATE_READY;
	mutex_unlock(&demux->mutex);
	dprintk("done\n");
	return 0;
}

static int dvbdmx_release_bbframes_stream_(struct dvb_demux* demux, struct bbframes_stream* bbf)
{
	WARN_ON(!bbf->feeds);
	bbf_dprintk(bbf, "release dmx dvb_demux=%p\n", demux);
	embedded_stream_dprintk(bbf->parent_embedded_stream, "release dmx dvb_demux=%p\n", demux);

	dprintk("calling  bbframes_stream_release: stream=%p\n", bbf);
	bbframes_stream_release(bbf);
	dmx_demux_dprintk(demux, "here\n");
	return 0;
}

static int dvbdmx_release_bbframes_stream(struct dmx_demux *dmx, struct bbframes_stream* bbs)
{
	struct dvb_demux *demux = container_of(dmx, struct dvb_demux, dmx);
	int ret;
	mutex_lock(&demux->mutex);
	ret = dvbdmx_release_bbframes_stream_(demux, bbs);
	mutex_unlock(&demux->mutex);
	return ret;
}

static int dvbdmx_release_pid_stream(struct dmx_demux *dmx, struct pid_stream *pid_feed)
{
	struct dvb_demux *demux = container_of(dmx, struct dvb_demux, dmx);
	struct dvb_demux_feed* feed = container_of(pid_feed, struct dvb_demux_feed, feed.pid_stream);
	struct dvb_demux_feeds* feeds = feed->parent_feeds;
	WARN_ON(!feeds);

	dmx_demux_dprintk(dmx, "release dmx dvb_demux=%p feed=%p section_filter=%p\n",
										demux, feed, feed->section_filter);
	mutex_lock(&demux->mutex);
	dprintk("here pid_feed=%p feed=%p parent_feeds=%p", pid_feed, feed, feeds);
	if (feed->state == DMX_STATE_FREE) {
		mutex_unlock(&demux->mutex);
		dprintk("here pid_feed=%p feed=%p parent_feeds=%p", pid_feed, feed, feeds);
		return -EINVAL;
	}
	dprintk("here pid_feed=%p feed=%p parent_feeds=%p\n", pid_feed, feed, feeds);
	feed->state = DMX_STATE_FREE;
	if(feed->section_filter)
		feed->section_filter->state = DMX_STATE_FREE;
	dvb_demux_feed_dprintk(feed, "calling dvb_demux_output_feed_del\n");
	dvb_demux_output_feed_del(feed);
	dvb_demux_feed_dprintk(feed, "called  dvb_demux_feed_del\n");

	feed->pid = 0xffff;

	if (feed->ts_type & TS_DECODER && feed->pes_type < DMX_PES_OTHER)
		demux->pesfilter[feed->pes_type] = NULL;

	mutex_unlock(&demux->mutex);
	return 0;
}


/******************************************************************************
 * dmx_section_feed API calls
 ******************************************************************************/

static int dmx_section_feed_allocate_filter(struct dmx_section_feed *feed,
																						struct dmx_section_filter **filter)
{
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)feed;
	struct dvb_demux *dvbdemux = dvbdmxfeed->demux;
	struct dvb_demux_section_filter *dvbdmxfilter;

	if (mutex_lock_interruptible(&dvbdemux->mutex))
		return -ERESTARTSYS;

	dvbdmxfilter = dvb_dmx_section_filter_alloc(dvbdemux);
	if (!dvbdmxfilter) {
		mutex_unlock(&dvbdemux->mutex);
		return -EBUSY;
	}

	spin_lock_irq(&dvbdemux->lock);
	*filter = &dvbdmxfilter->filter;
	(*filter)->parent_dmx_section_feed = feed;
	(*filter)->priv = NULL;
	dvbdmxfilter->feed = dvbdmxfeed;
	dvbdmxfilter->type = DMX_TYPE_SEC;
	dvbdmxfilter->state = DMX_STATE_READY;
	dvbdmxfilter->next = dvbdmxfeed->section_filter;
	dvbdmxfeed->section_filter = dvbdmxfilter;
	spin_unlock_irq(&dvbdemux->lock);

	mutex_unlock(&dvbdemux->mutex);
	return 0;
}

static void prepare_secfilters(struct dvb_demux_feed *dvbdmxfeed)
{
	int i;
	struct dvb_demux_section_filter *f;
	struct dmx_section_filter *sf;
	u8 mask, mode, doneq;

	if (!(f = dvbdmxfeed->section_filter))
		return;
	do {
		sf = &f->filter;
		doneq = false;
		for (i = 0; i < DVB_DEMUX_MASK_MAX; i++) {
			mode = sf->filter_mode[i];
			mask = sf->filter_mask[i];
			f->maskandmode[i] = mask & mode;
			doneq |= f->maskandnotmode[i] = mask & ~mode;
		}
		f->doneq = doneq ? true : false;
	} while ((f = f->next));
}

static int dmx_section_feed_start_filtering(struct dmx_section_feed* secfeed)
{
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)secfeed;
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	int ret;

	if (mutex_lock_interruptible(&dvbdmx->mutex))
		return -ERESTARTSYS;

	if (secfeed->is_filtering) {
		mutex_unlock(&dvbdmx->mutex);
		return -EBUSY;
	}

	if (!dvbdmxfeed->section_filter) {
		mutex_unlock(&dvbdmx->mutex);
		return -EINVAL;
	}

	dvbdmxfeed->feed.sec.tsfeedp = 0;
	dvbdmxfeed->feed.sec.secbuf = dvbdmxfeed->feed.sec.secbuf_base;
	dvbdmxfeed->feed.sec.secbufp = 0;
	dvbdmxfeed->feed.sec.seclen = 0;
	dvbdmxfeed->pusi_seen = false;

	if (!dvbdmx->start_feed) {
		mutex_unlock(&dvbdmx->mutex);
		return -ENODEV;
	}

	prepare_secfilters(dvbdmxfeed);

	if ((ret = dvbdmx->start_feed(dvbdmxfeed)) < 0) {
		mutex_unlock(&dvbdmx->mutex);
		return ret;
	}

	spin_lock_irq(&dvbdmx->lock);
	secfeed->is_filtering = 1;
	dvbdmxfeed->state = DMX_STATE_GO;
	spin_unlock_irq(&dvbdmx->lock);

	mutex_unlock(&dvbdmx->mutex);
	return 0;
}

static int dmx_section_feed_stop_filtering(struct dmx_section_feed *feed)
{
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)feed;
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	int ret;

	mutex_lock(&dvbdmx->mutex);

	if (!dvbdmx->stop_feed) {
		mutex_unlock(&dvbdmx->mutex);
		return -ENODEV;
	}

	ret = dvbdmx->stop_feed(dvbdmxfeed);
	spin_lock_irq(&dvbdmx->lock);
	dvbdmxfeed->state = DMX_STATE_READY;
	feed->is_filtering = 0;
	spin_unlock_irq(&dvbdmx->lock);

	mutex_unlock(&dvbdmx->mutex);
	return ret;
}

static int dmx_section_feed_release_filter(struct dmx_section_feed* secfeed,
																					 struct dmx_section_filter *filter)
{
	struct dvb_demux_section_filter *dvbdmxfilter = (struct dvb_demux_section_filter *)filter, *f;
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)secfeed;
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;

	mutex_lock(&dvbdmx->mutex);

	if (dvbdmxfilter->feed != dvbdmxfeed) {
		mutex_unlock(&dvbdmx->mutex);
		return -EINVAL;
	}

	if (secfeed->is_filtering) {
		/* release dvbdmx->mutex as far as it is
		   acquired by stop_filtering() itself */
		mutex_unlock(&dvbdmx->mutex);
		secfeed->stop_section_filtering(secfeed);
		mutex_lock(&dvbdmx->mutex);
	}

	spin_lock_irq(&dvbdmx->lock);
	f = dvbdmxfeed->section_filter;

	if (f == dvbdmxfilter) {
		dvbdmxfeed->section_filter = dvbdmxfilter->next;
	} else {
		while (f->next != dvbdmxfilter)
			f = f->next;
		f->next = f->next->next;
	}

	dvbdmxfilter->state = DMX_STATE_FREE;
	spin_unlock_irq(&dvbdmx->lock);
	mutex_unlock(&dvbdmx->mutex);
	return 0;
}

static int dvbdmx_allocate_section_feed(struct dmx_demux *dmx_demux, struct dmx_section_feed ** section_feed,
																				dmx_section_cb callback, u16 pid, bool check_crc,
																				struct dvb_demux_feeds* parent_feeds)
{
	struct dvb_demux* demux =  container_of(dmx_demux, struct dvb_demux, dmx);
	struct dvb_demux_feed *feed;
	if (pid > 0x1fff)
		return -EINVAL;

	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	if (!(feed = dvb_dmx_feed_alloc(demux))) {
		mutex_unlock(&demux->mutex);
		return -EBUSY;
	}
	if(!parent_feeds)
		parent_feeds= demux->default_feeds;
	dprintk("parent_feeds=%p\n", parent_feeds);
	WARN_ON(!parent_feeds);
	feed->type = DMX_TYPE_SEC;
	feed->cb.sec = callback;
	feed->demux = demux;
	feed->pid = 0xffff;
	feed->buffer_flags = 0;
	feed->feed.sec.secbuf = feed->feed.sec.secbuf_base;
	feed->feed.sec.secbufp = feed->feed.sec.seclen = 0;
	feed->feed.sec.tsfeedp = 0;
	feed->section_filter = NULL;
	feed->parent_feeds = parent_feeds;

	(*section_feed) = &feed->feed.sec;
	(*section_feed)->is_filtering = 0;
	(*section_feed)->parent_dmx_demux = dmx_demux;
	(*section_feed)->priv = NULL;

	(*section_feed)->allocate_section_filter = dmx_section_feed_allocate_filter;
	(*section_feed)->start_section_filtering = dmx_section_feed_start_filtering;
	(*section_feed)->stop_section_filtering = dmx_section_feed_stop_filtering;
	(*section_feed)->release_section_filter = dmx_section_feed_release_filter;

	dvb_demux_output_feed_add(feed);

	feed->pid = pid;
	feed->feed.sec.check_crc = check_crc;

	feed->state = DMX_STATE_READY;
	mutex_unlock(&demux->mutex);
	return 0;
}

static int dvbdmx_release_section_feed(struct dmx_demux *demux,
				       struct dmx_section_feed *feed)
{
	struct dvb_demux_feed *dvbdmxfeed = (struct dvb_demux_feed *)feed;
	struct dvb_demux *dvbdmx = (struct dvb_demux *)demux;

	mutex_lock(&dvbdmx->mutex);

	if (dvbdmxfeed->state == DMX_STATE_FREE) {
		mutex_unlock(&dvbdmx->mutex);
		return -EINVAL;
	}
	dvbdmxfeed->state = DMX_STATE_FREE;

	dvb_demux_output_feed_del(dvbdmxfeed);

	dvbdmxfeed->pid = 0xffff;

	mutex_unlock(&dvbdmx->mutex);
	return 0;
}

/******************************************************************************
 * dvb_demux kernel data API calls
 ******************************************************************************/

static int dvbdmx_open(struct dmx_demux *demux)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	if (dvbdemux->users >= MAX_DVB_DEMUX_USERS)
		return -EUSERS;
	dvbdemux->users++;
	return 0;
}

static int dvbdmx_close(struct dmx_demux *demux)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	if (dvbdemux->users == 0)
		return -ENODEV;

	dvbdemux->users--;
	//FIXME: release any unneeded resources if users==0
	return 0;
}

static int dvbdmx_write(struct dmx_demux *demux, const char __user *buf, size_t count)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;
	void *p;
	if ((!demux->frontend) || (demux->frontend->source != DMX_MEMORY_FE))
		return -EINVAL;

	p = memdup_user(buf, count);
	if (IS_ERR(p))
		return PTR_ERR(p);
	if (mutex_lock_interruptible(&dvbdemux->mutex)) {
		kfree(p);
		return -ERESTARTSYS;
	}
	dvb_dmx_swfilter(dvbdemux, p, count);
	kfree(p);
	mutex_unlock(&dvbdemux->mutex);

	if (signal_pending(current))
		return -EINTR;
	return count;
}

static int dvbdmx_add_frontend(struct dmx_demux *demux,
			       struct dmx_frontend *frontend)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;
	struct list_head *head = &dvbdemux->frontend_list;

	list_add(&(frontend->connectivity_list), head);

	return 0;
}

static int dvbdmx_remove_frontend(struct dmx_demux *demux,
				  struct dmx_frontend *frontend)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;
	struct list_head *pos, *n, *head = &dvbdemux->frontend_list;

	list_for_each_safe(pos, n, head) {
		if (DMX_FE_ENTRY(pos) == frontend) {
			list_del(pos);
			return 0;
		}
	}

	return -ENODEV;
}

static struct list_head *dvbdmx_get_frontends(struct dmx_demux *demux)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	if (list_empty(&dvbdemux->frontend_list))
		return NULL;

	return &dvbdemux->frontend_list;
}

static int dvbdmx_connect_frontend(struct dmx_demux *demux,
				   struct dmx_frontend *frontend)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	if (demux->frontend)
		return -EINVAL;

	mutex_lock(&dvbdemux->mutex);

	demux->frontend = frontend;
	mutex_unlock(&dvbdemux->mutex);
	return 0;
}

static int dvbdmx_disconnect_frontend(struct dmx_demux *demux)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	mutex_lock(&dvbdemux->mutex);

	demux->frontend = NULL;
	mutex_unlock(&dvbdemux->mutex);
	return 0;
}

static int dvbdmx_get_pes_pids(struct dmx_demux *demux, u16 * pids)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *)demux;

	memcpy(pids, dvbdemux->pids, 5 * sizeof(u16));
	return 0;
}

int dvb_dmx_init(struct dvb_demux *dvbdemux)
{
	int i;
	struct dmx_demux *dmx = &dvbdemux->dmx;
	dvbdemux->users = 0;
	dvbdemux->section_filter = vmalloc(array_size(sizeof(struct dvb_demux_section_filter),
					      dvbdemux->filternum));

	if (!dvbdemux->section_filter)
		return -ENOMEM;

	dvbdemux->feedarray = vmalloc(array_size(sizeof(struct dvb_demux_feed),
																					 dvbdemux->feednum));
	dprintk("ALLOC dvbdemux->feedarray=%p dvbdemux=%p\n", dvbdemux->feedarray, dvbdemux);
	if (!dvbdemux->feedarray) {
		vfree(dvbdemux->section_filter);
		dvbdemux->section_filter = NULL;
		return -ENOMEM;
	}
	for (i = 0; i < dvbdemux->filternum; i++) {
		dvbdemux->section_filter[i].state = DMX_STATE_FREE;
		dvbdemux->section_filter[i].index = i;
	}
	for (i = 0; i < dvbdemux->feednum; i++) {
		dvbdemux->feedarray[i].state = DMX_STATE_FREE;
		dvbdemux->feedarray[i].index = i;
	}
	INIT_LIST_HEAD(&dvbdemux->frontend_list);

	for (i = 0; i < DMX_PES_OTHER; i++) {
		dvbdemux->pesfilter[i] = NULL;
		dvbdemux->pids[i] = 0xffff;
	}

	dvbdemux->playing = 0;
	dvbdemux->recording = 0;
	dvbdemux->tsbufp = 0;

	if (!dvbdemux->check_crc32)
		dvbdemux->check_crc32 = dvb_dmx_crc32;

	if (!dvbdemux->memcopy)
		dvbdemux->memcopy = dvb_dmx_memcopy;

	dmx->frontend = NULL;
	dmx->priv = dvbdemux;
	dmx->open = dvbdmx_open;
	dmx->close = dvbdmx_close;
	dmx->write = dvbdmx_write;
	dmx->get_fe_feeds = dvbdmx_get_fe_feeds;
	dmx->allocate_pid_stream = dvbdmx_allocate_pid_stream;
	dmx->allocate_stid_stream = dvbdmx_allocate_stid_stream;
	dmx->allocate_t2mi_stream = dvbdmx_allocate_t2mi_stream;
	dmx->release_pid_stream = dvbdmx_release_pid_stream;
	dmx->release_bbf_stream = dvbdmx_release_bbframes_stream;
	dmx->allocate_section_feed = dvbdmx_allocate_section_feed;
	dmx->release_section_feed = dvbdmx_release_section_feed;

	dmx->add_frontend = dvbdmx_add_frontend;
	dmx->remove_frontend = dvbdmx_remove_frontend;
	dmx->get_frontends = dvbdmx_get_frontends;
	dmx->connect_frontend = dvbdmx_connect_frontend;
	dmx->disconnect_frontend = dvbdmx_disconnect_frontend;
	dmx->get_pes_pids = dvbdmx_get_pes_pids;

	dvbdemux->fe_bbframes_stream = NULL;
	dvbdemux->default_stream_id = -1;

	dvbdemux->fe_feeds = kzalloc(sizeof(struct dvb_demux_feeds), GFP_KERNEL);
	dvb_demux_feeds_init(dvbdemux->fe_feeds, dvbdemux, -1, -1);
	dvbdemux->fe_feeds->demux = dvbdemux;
	dvbdemux->fe_feeds->include_default_feeds = true; //for legacy applications

	dvbdemux->default_feeds = kzalloc(sizeof(struct dvb_demux_feeds), GFP_KERNEL);
	dvb_demux_feeds_init(dvbdemux->default_feeds, dvbdemux, -1, -1);
	dvbdemux->default_feeds->demux = dvbdemux;

	mutex_init(&dvbdemux->mutex);
	spin_lock_init(&dvbdemux->lock);
	return 0;
}

EXPORT_SYMBOL(dvb_dmx_init);

void dvb_dmx_release(struct dvb_demux *dvbdemux)
{
	kref_put(&dvbdemux->fe_feeds->refcount, dvb_demux_feeds_release_);
	vfree(dvbdemux->section_filter);
	dprintk("FREE dvbdemux->feedarray=%p dvbdemux=%p\n", dvbdemux->feedarray, dvbdemux);
	vfree(dvbdemux->feedarray);
}

EXPORT_SYMBOL(dvb_dmx_release);

int dvb_demux_set_bbframes_state(struct dvb_demux* demux, bool embedding_is_on, int embedding_pid, int default_stream_id)
{
	if(mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	dprintk("called with embedding_is_on=%d embedding_pid=%d default_stream_id= %d => %d  \n", embedding_is_on,
					embedding_pid, 	demux->default_stream_id, default_stream_id);
	demux->default_stream_id = default_stream_id;
	if(!xa_empty(&demux->default_feeds->embedded_streams))
		dprintk("demux->default_feeds->embedded_streams is not empty\n");

	int ret=0;
	if(demux->fe_bbframes_stream) {
		dprintk("calling dvbdmx_release_bbframes_stream_ to release demux->fe_bbframes_stream=%p\n", demux->fe_bbframes_stream);
		ret = dvbdmx_release_bbframes_stream_(demux, demux->fe_bbframes_stream);
		demux->fe_bbframes_stream = NULL;
		demux->fe_feeds->include_default_feeds = true;
	}
	if (embedding_is_on) {
		dprintk("calling dvbdmx_allocate_stid_stream_ to set demux->fe_bbframes_stream\n");
		ret = dvbdmx_allocate_stid_stream_(demux, &demux->fe_bbframes_stream, embedding_pid, default_stream_id, demux->fe_feeds);
		if(ret<0)
			return ret;

		WARN_ON(!demux->fe_bbframes_stream);
		WARN_ON(!demux->fe_bbframes_stream->feeds);
		demux->fe_bbframes_stream->feeds->include_default_feeds = true;
		demux->fe_feeds->include_default_feeds = false;
	} else {
		WARN_ON(demux->fe_bbframes_stream);
		demux->fe_feeds->include_default_feeds = true;
	}
	mutex_unlock(&demux->mutex);
	return ret;
}

int dvb_demux_get_matypes(struct dvb_demux* demux, int32_t (*isi_bitset)[8], int32_t (*high_rolloff_mode)[8], uint8_t (*matypes)[256])
{
	if(mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;
	int num_streams=0;
	//WARN_ON(!demux->fe_bbframes_stream);
	if(demux->fe_bbframes_stream) {
		struct embedded_stream*  emb = demux->fe_bbframes_stream->parent_embedded_stream;
		if(emb) {
				memcpy(&isi_bitset[0], &emb->isi_plp_bitset[0], sizeof(int32_t)*8);
				memcpy(&high_rolloff_mode[0], &emb->high_rolloff_mode[0], sizeof(int32_t)*8);
				memcpy(&matypes[0], &emb->matypes[0], sizeof(int8_t)*256);
				num_streams = emb->num_streams;
		}
	}
	mutex_unlock(&demux->mutex);
	if(num_streams>0) {
		char buf[4096]="";
		int ret=0;
		int indent=0;
		ret += sprintf(buf+ret, "%*sISI/PLP:matype: ", indent, " ");
		int isi;
		for(isi=0; isi <256;++isi) {
			if(!(((*isi_bitset)[(isi>>5)&0x7] >> (isi&31))&1))
				continue;
			ret += sprintf(buf+ret, " %d:0x%x", isi, (*matypes)[isi]);
			if(((*high_rolloff_mode)[(isi>>5)&0x7] >> (isi&31))&1)
				ret += sprintf(buf+ret, " +");
		}
		ret += sprintf(buf+ret, "\n");
		dprintk("%s", buf);
	}
	return 0;
}

EXPORT_SYMBOL(dvb_demux_set_bbframes_state);
EXPORT_SYMBOL(dvb_demux_get_matypes);
