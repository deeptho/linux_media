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

#define bbframes_demux_dprintk(bbd, fmt, arg...) do {										\
		if (dvb_demux_dtdebug)																							\
			printk(KERN_DEBUG pr_fmt("%s:%d bbd[%p] pid=%d feeds=%p bbd.refcount=%d " fmt),	\
						 __func__, __LINE__, bbd, bbd->embedding_pid, bbd->parent_feeds,	\
						 atomic_read(&bbd->refcount.refcount.refs),									\
						 ##arg);																										\
	} while (0)


#define bbframes_demux_dprintk_nice(bbd, fmt, arg...) do {							\
		static int count=0;																									\
		if (count++%100 == 0 && dvb_demux_dtdebug)													\
			printk(KERN_DEBUG pr_fmt("%s:%d bbd[%p] pid=%d feeds=%p count=%d " fmt), \
						 __func__, __LINE__, bbd, bbd->embedding_pid, bbd->parent_feeds, count, ##arg);	\
	} while (0)


#define stream_dprintk(stream, fmt, arg...) do {												\
		if (dvb_demux_dtdebug)																							\
			printk(KERN_DEBUG pr_fmt("%s:%d stream[%p] isi=%d bbd=%p stream.refcount=%d " fmt),		\
						 __func__, __LINE__, stream, stream ? stream->isi : -1, stream->bbframes_demux, \
						 atomic_read(&stream->refcount.refcount.refs),							\
						 ##arg);																										\
	} while (0)


#define stream_dprintk_nice(stream, fmt, arg...) do {										\
		static int count=0;																									\
		if (count++%100 == 0 && dvb_demux_dtdebug)													\
			printk(KERN_DEBUG pr_fmt("%s:%d stream[%p] isi=%d count=%d " fmt),	\
						 __func__, __LINE__, stream, stream ? stream->isi : -1, count, ##arg); \
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

static void bbframes_stream_add_packet(struct dvb_demux* demux, struct bbframes_demux* bbd, const u8* packet);
static void dvb_dmx_swfilter_packet(struct dvb_demux *demux, const u8 *buf, struct dvb_demux_feeds* feeds);

/******************************************************************************
 * static inlined helper functions
 ******************************************************************************/
#define dvb_demux_feed_dprintk(feed, fmt, arg...) do {									\
		if(!feed)																														\
			printk(KERN_DEBUG pr_fmt("%s:%d NO FEED " fmt),										\
						 __func__, __LINE__, ##arg);																\
	else																																	\
		printk(KERN_DEBUG pr_fmt("%s:%d feed[%p] pid=%d ts_feed=%p filter=%p feed.parent_feeds=%p parent_feeds.refcount=%d " fmt), \
					 __func__, __LINE__, (void*)feed, feed->pid, &feed->feed.ts,	\
					 (void*)feed->section_filter, feed->parent_feeds,							\
					 atomic_read(&feed->parent_feeds->refcount.refcount.refs), ##arg); \
	} while (0)

#define dvb_demux_feed_dprintk_nice(feed, fmt, arg...) do {				\
		static int count=0;																									\
		if (count++%100 == 0 && dvb_demux_dtdebug) {												\
			if(!feed)																													\
				printk(KERN_DEBUG pr_fmt("%s:%d NO FEED " fmt),									\
							 __func__, __LINE__, ##arg);															\
			else																															\
				printk(KERN_DEBUG pr_fmt("%s:%d feed[%p] pid=%d ts_feed=%p filter=%p " fmt), \
							 __func__, __LINE__, (void*)feed, feed->pid, &feed->feed.ts, \
							 (void*)feed->section_filter, ##arg);											\
		}																																		\
	} while (0)

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
	xa_init(&feeds->bbframes_demuxes);

	feeds->cnt_storage = vmalloc(MAX_PID + 1);
	dprintk("ALLOC feeds->cnt_storage=%p feeds=%p\n", feeds->cnt_storage, feeds);
	if (!feeds->cnt_storage)
		pr_warn("Couldn't allocate memory for TS/TEI check. Disabling it\n");
	{
		int i;
		for(i=0; i<MAX_PID; ++i)
			feeds->cnt_storage[i] = 0xff;
	}
}

static void dvb_demux_feeds_release_(struct kref *kref)
{
	struct dvb_demux_feeds* feeds = container_of(kref, struct dvb_demux_feeds, refcount);
	int output_feeds_list_empty = list_empty(&feeds->output_feed_list);
	int bbframes_demuxes_empty = xa_empty(&feeds->bbframes_demuxes);
	dprintk("feeds=%p refcount has dropped to 0 releasing feeds; "
					"bbframe_demuxes:empty=%d "
					"output_feedlist_:empty=%d\n",
					feeds, bbframes_demuxes_empty, output_feeds_list_empty);
	dprintk("feeds->bbframes_demuxes list:\n");
	{
		unsigned long index;
		void *entry;
		xa_for_each(&feeds->bbframes_demuxes, index, entry) {
			struct bbframes_demux* bbd = entry;
			dprintk("item: bbd=%p bbframes_pid=%d parent_feeds=%p\n",
							bbd, bbd->embedding_pid, bbd->parent_feeds);
		}
	}
	BUG_ON(!output_feeds_list_empty);
	dprintk("FREE feeds->cnt_storage=%p in feeds=%p\n", feeds->cnt_storage, feeds);
	vfree(feeds->cnt_storage);
	xa_destroy(&feeds->bbframes_demuxes);
	dprintk("FREE feeds=%p\n", feeds);
	kfree(feeds);
}


static void	bbframes_stream_init(struct bbframes_stream* stream,
																 struct bbframes_demux * bbd, struct dvb_demux* demux,
																 int embedding_pid, int isi) {
	stream->isi = isi;
	stream->last_crc = -1;
	kref_init(&stream->refcount);
	stream->bbframes_demux = bbd;
	stream->feeds = kzalloc(sizeof(struct dvb_demux_feeds), GFP_KERNEL);
	bbframes_demux_dprintk(bbd, "ALLOC stream->feeds=%p stream=%p\n", stream->feeds, stream);
	dvb_demux_feeds_init(stream->feeds, demux, embedding_pid, isi);
}

static void bbframes_demux_release_(struct kref *kref)
{
	struct bbframes_demux* bbd = container_of(kref, struct bbframes_demux, refcount);
	int bbframes_streams_empty = xa_empty(&bbd->bbframes_streams);
	dprintk("bbd=%p refcount has dropped to 0 releasing; "
					"bbframe_streams:empty=%d parent_feeds=%p\n",
					bbd, bbframes_streams_empty, bbd->parent_feeds);
	BUG_ON(!bbframes_streams_empty);

	bbframes_demux_dprintk(bbd, "before reducing parent_feeds.refcount\n");
	feeds_dprintk(bbd->parent_feeds, "parent_feeds\n");
	kref_put(&bbd->parent_feeds->refcount, dvb_demux_feeds_release_);

	dprintk("bbd=%p erasing parent_feeds->bbframe_demuxes[%d]\n", bbd, bbd->embedding_pid);
	xa_erase(&bbd->parent_feeds->bbframes_demuxes, bbd->embedding_pid);
	dprintk("FREE bbd=%p\n", bbd);
	xa_destroy(&bbd->bbframes_streams);
	kfree(bbd);
}

static void	bbframes_demux_release(struct bbframes_demux* bbd) {
	bbframes_demux_dprintk(bbd, "before reducing bbd.refcount\n");
	kref_put(&bbd->refcount, bbframes_demux_release_);
	bbframes_demux_dprintk(bbd, "after reducing bbd.refcount\n");
}

static void bbframes_stream_release_(struct kref *ref)
{
	struct bbframes_stream * stream = container_of(ref, struct bbframes_stream, refcount);
	struct dvb_demux_feeds* feeds = stream->feeds;
	stream_dprintk(stream, "erasing stream=%p for isi=%d in bbframes_demux=%p\n", stream, stream? stream->isi : -1,
					stream->bbframes_demux);
	bbframes_demux_dprintk(stream->bbframes_demux, "erasing stream=%p for isi=%d in bbframes_demux=%p\n", stream, stream? stream->isi : -1,
					stream->bbframes_demux);
	BUG_ON(!stream->bbframes_demux);
	kref_put(&feeds->refcount,  dvb_demux_feeds_release_);
	xa_erase(&stream->bbframes_demux->bbframes_streams, stream->isi);
	dprintk("FREE stream=%p\n", stream);
	kfree(stream);
}

static void bbframes_stream_release(struct bbframes_stream* stream) {
	BUG_ON(!stream);
	int isi = stream->isi ;
	dprintk("bbframes_stream=%p isi=%d stream.refcount=%d\n",
					stream, isi, atomic_read(&stream->refcount.refcount.refs));
	//stream = xa_load(&bbframes_demux->bbframes_stream_list, isi);
	kref_put(&stream->refcount, bbframes_stream_release_);
	dprintk("done isi=%d stream.refcount=%d\n", isi, atomic_read(&stream->refcount.refcount.refs));
	bbframes_demux_release(stream->bbframes_demux);
}

/******************************************************************************
 * Software filter functions
 ******************************************************************************/

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

	return feed->cb.ts(&buf[p], count, NULL, 0, &feed->feed.ts,
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

static inline void dvb_dmx_swfilter_packet_type(struct dvb_demux_feed *feed, const u8 *buf)
{
	switch (feed->type) {
	case DMX_TYPE_TS:
		if (!feed->feed.ts.is_filtering)
			break;
		if (feed->ts_type & TS_PACKET) {
			if (feed->ts_type & TS_PAYLOAD_ONLY)
				dvb_dmx_swfilter_payload(feed, buf);
			else
				feed->cb.ts(buf, 188, NULL, 0, &feed->feed.ts,
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

#define DVR_FEED(f)												\
	(((f)->type == DMX_TYPE_TS) &&					\
	((f)->feed.ts.is_filtering) &&					\
	(((f)->ts_type & (TS_PACKET | TS_DEMUX)) == TS_PACKET))

static void	bbframes_demux_init(struct bbframes_demux* bbd, struct dvb_demux_feeds*parent_feeds,
																int embedding_pid) {
	BUG_ON(!parent_feeds);
	memset(bbd, 0 , sizeof(struct bbframes_demux));
	bbd->embedding_pid = embedding_pid;
	bbd->current_isi = -1;
	bbd->parent_feeds = parent_feeds;
	kref_init(&bbd->refcount);
	xa_init(&bbd->bbframes_streams);
	kref_get(&bbd->parent_feeds->refcount);
	feeds_dprintk(bbd->parent_feeds, "After kref_get: refcount=%d\n",
								atomic_read(&bbd->parent_feeds->refcount.refcount.refs));
}

static void bbframes_stream_output_bytes(struct dvb_demux* demux,
																				 struct bbframes_stream* stream, const uint8_t*start, int len, int isi)
{
	//stream_dprintk_nice(stream, "len=%d isi=%d\n", len, isi);
	if(len<0) {
		/*this can happen when the first sync byte is stored in the next encapsualting packet.
			When then set demux->buff_count to a negative value, with abs(demux->buff_count) equal to the number of bytes
			still to skip.
		*/
		stream->buff_count = len;
		return;
	}
	//if some bytes still need to be skipped, skip them
	if(stream->buff_count < 0) {
		int n = min(-stream->buff_count , len);
		stream->buff_count += n;
		len -= n;
		start += n;
		if(len <=0)
			return;
	}
	while(len>0) {
		int n = min(stream->upl - stream->buff_count , len);
		if( n<= 0) {
			stream_dprintk_nice(stream, "upl=%d buff_count=%d len=%d n=%d\n",
														 stream->upl, stream->buff_count, len, n);
			return;
		}
		//dprintk("here\n");
		if(stream->buff_count > sizeof(stream->buff) ||
			 stream->buff_count+n > sizeof(stream->buff)) {
			stream_dprintk_nice(stream, "out of buffer upl=%d buff_count=%d len=%d n=%d\n",
													stream->upl, stream->buff_count, len, n);
			stream->buff_count = 0;
			return;
		}
		memcpy((&stream->buff[0])+stream->buff_count, start, n);
		stream->buff_count += n;
		start +=n;
		len -= n;
		if(stream->buff_count >= stream->upl) {

			if(stream->buff_count > stream->upl) {
				stream_dprintk_nice(stream, "ERROR: upl=%d buff_count=%d len=%d n=%d\n", stream->upl, stream->buff_count, len, n);
			}
			stream->buff_count = stream->upl;
		}
		if(stream->buff_count == stream->upl) {
			uint8_t stream_crc=stream->buff[0];
			stream->buff[0]= stream->syncbyte;
#ifdef CHECKCRC
			int next_crc = compute_crc8(stream->buff+1, stream->buff_count-1);
			bool bad_crc = (stream->last_crc != stream_crc && stream->last_crc!=-1);
			stream->last_crc = next_crc;
			if(bad_crc) {
#if 1
				stream_dprintk_nice(stream, "PACK isi=%d CRC%d 0x%02x %s: 0x%02x\n",
															 isi, stream->buff_count, stream_crc,
															 "BAD", next_crc);
#endif
				stream->last_crc = next_crc;
				stream->buff_count=0;
				return;
			} else {
			}
#endif
			BUG_ON(!stream->feeds);
#if 1
			dvb_dmx_swfilter_packet(demux, &stream->buff[0], stream->feeds);
#endif
			stream->buff_count=0;
		}
	}
}

/*
	This packet contains embedded data to be processed by embedde_demux
	packet has 188 bytes
 */
static void bbframes_stream_add_packet(struct dvb_demux* demux, struct bbframes_demux* bbd, const u8* packet)
{
	struct bbframes_stream* stream = bbd->current_stream;
	int pid = (unsigned char)(packet[1]);
	pid&=0x01F;
	pid<<=8;
	pid|=(unsigned char)(packet[2]);
	//bbframes_demux_dprintk_nice(bbd, "pid=%d\n", pid);
	if(pid != bbd->embedding_pid) {
		bbframes_demux_dprintk_nice(bbd, "Unexpected pid: 0x%02x\n", pid);
		return;
	}
#if 0
	int cc_counter = packet[3]&0xf;
#endif
	int packet_counter = packet[8]&0xff;
	int section_length= packet[7];

	if(packet_counter == 0xb8) { //header
#if 0
		int matype = packet[9+0];
#endif
		int isi = (packet[9+1]);
		int crc = packet[9+9];

		int tstcrc= compute_crc8(packet+9, 9);
		bool bad_crc= crc !=tstcrc;
		//dprintk("Header\n");
		if(bbd->current_isi != isi) {
			bbd->current_isi = isi;
			bbd->current_stream = xa_load(&bbd->bbframes_streams, bbd->current_isi);
			stream = bbd->current_stream;
			//dprintk("loaded stream: isi=%d stream=%p\n", isi, stream);
		} else {
			//dprintk("continuing stream: isi=%d stream=%p\n", isi, stream);
		}
		if(!bbd->current_stream)
			return;
		stream->upl = ((packet[9+2]<<8) | (packet[9+3]))/8;
		stream->dfl = ((packet[9+4]<<8) | (packet[9+5]))/8;
		stream->syncbyte =  packet[9+6];
		stream->syncd= ((packet[9+7]<<8) | packet[9+8])/8;
		stream->syncbyte =  packet[9+6];
		stream->syncd= ((packet[9+7]<<8) | packet[9+8])/8;
		if(bad_crc)
			dmx_demux_dprintk_nice(bbd,
														 "HEADER CRC error isi=%d upl=%d syncd=%d crc=0x%02x 0x%02x\n",
														 bbd->current_isi, stream->upl, stream->syncd,
														 crc, tstcrc);
		if(!stream->synced) {
			if(stream->syncd == 0xffff)
				return; //no UP starts in the DATA FIELD
			stream->buff_count=0;
			bbframes_stream_output_bytes(demux, stream, &packet[19+stream->syncd],
																		section_length-11-stream->syncd, bbd->current_isi);
			stream->synced =true;
		} else {
			if(stream->syncd != 0xffff //a UP starts in the DATA FIELD
				 && (stream->syncd + stream->buff_count) != stream->upl) {
				stream->buff_count = 0;
				bbframes_stream_output_bytes(demux, stream, &packet[19+stream->syncd],
																		 section_length-11-stream->syncd, bbd->current_isi);
			} else {
				bbframes_stream_output_bytes(demux, stream, &packet[19], section_length-11, bbd->current_isi);
			}
		}
	} else if(stream && stream->synced) { //packet without header
		bbframes_stream_output_bytes(demux, stream, &packet[9], section_length-1, bbd->current_isi);
	}
}

/*
	Process a packet received from sub_demux
 */
static void dvb_dmx_swfilter_packet(struct dvb_demux *demux, const u8 *buf, struct dvb_demux_feeds* feeds)
{
	struct dvb_demux_feed *feed;
	u16 pid = ts_pid(buf);
	int dvr_done = 0;
	enum dmx_buffer_flags buffer_flags;
	bool flag_error=false;
	BUG_ON(!feeds);
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

	if(!flag_error)  {
		struct bbframes_demux* bbd =  (struct bbframes_demux*)xa_load(&feeds->bbframes_demuxes, pid);
		if(bbd) {
			bbframes_stream_add_packet(demux, bbd, buf);
		}
	}

	//dmx_demux_dprintk_nice(demux,"feeds=%p feeds->include_default_feeds=%d\n", pid, feeds, feeds->include_default_feeds);
	for(int i=0; i < (feeds->include_default_feeds ? 2 : 1); ++i) {
		if(i>0) {
			feeds = demux->default_feeds;
		}
		list_for_each_entry(feed, &feeds->output_feed_list, next) {
			//dmx_demux_dprintk_nice(demux,"pid=%d feeds=%p\n", pid, feeds);
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

			if (feed->pid == pid) //for a matching packet, convert data to ES if needed, or extract section data
				dvb_dmx_swfilter_packet_type(feed, buf);
			else if (feed->pid == 0x2000) //except when full TS is requested, then only copy packets as-is
				feed->cb.ts(buf, 188, NULL, 0, &feed->feed.ts, &feed->buffer_flags);
		}
	}
}

void dvb_dmx_swfilter_packets(struct dvb_demux *demux, const u8 *buf, size_t count)
{
	unsigned long flags;

	spin_lock_irqsave(&demux->lock, flags);

	while (count--) {
		if (buf[0] == 0x47) {
			dvb_dmx_swfilter_packet(demux, buf, demux->fe_feeds);
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
			dvb_dmx_swfilter_packet(demux, demux->tsbuf, demux->fe_feeds);
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
		dvb_dmx_swfilter_packet(demux, q, demux->fe_feeds);
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

	feed->cb.ts(buf, count, NULL, 0, &feed->feed.ts,
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
	BUG_ON(!feeds);
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
	BUG_ON(!feeds);
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
	BUG_ON(!feeds);
	spin_lock_irq(&feed->demux->lock);
	if (!(dvb_demux_output_feed_find(feed))) {
		pr_err("%s: feed not in list (type=%x state=%x pid=%x)\n",
		       __func__, feed->type, feed->state, feed->pid);
		goto out;
	}
	list_del(&feed->next);
	feeds_dprintk(feeds, "calling kref_put feeds.refcount=%d\n",
								atomic_read(&feeds->refcount.refcount.refs));
	kref_put(&feeds->refcount, dvb_demux_feeds_release_);
out:
	spin_unlock_irq(&feed->demux->lock);
}

static int dmx_ts_feed_start_filtering(struct dmx_ts_feed *ts_feed)
{
	struct dvb_demux_feed *feed = container_of(ts_feed, struct dvb_demux_feed, feed.ts);
	struct dvb_demux *demux = feed->demux;
	struct dvb_demux_feeds* feeds = feed->parent_feeds;
	BUG_ON(!feeds);
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
#if 1
	{
		int i;
		for(i=0; i<MAX_PID; ++i)
			feeds->cnt_storage[i] = 0xff;
	}
#endif
	if ((ret = demux->start_feed(feed)) < 0) {
		mutex_unlock(&demux->mutex);
		return ret;
	}

	spin_lock_irq(&demux->lock);
	ts_feed->is_filtering = 1;
	feed->state = DMX_STATE_GO;
	spin_unlock_irq(&demux->lock);
	mutex_unlock(&demux->mutex);

	return 0;
}

static int dmx_ts_feed_stop_filtering(struct dmx_ts_feed *ts_feed)
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

static struct bbframes_stream* dvb_dmx_find_or_alloc_bbframes_stream
(struct dvb_demux* demux, struct bbframes_demux* bbd, int embedding_pid, int isi) {
	struct bbframes_stream* stream = NULL;
	struct bbframes_stream* old_stream = NULL;
	stream = xa_load(&bbd->bbframes_streams, isi);
	if(!stream) {
		stream = kzalloc(sizeof(struct bbframes_stream), GFP_KERNEL);
		dprintk("ALLOC stream=%p bbd=%p bbd.refcount=%d embedding_pid=%d isi=%d\n", stream, bbd,
						atomic_read(&bbd->refcount.refcount.refs), embedding_pid, isi);
		bbframes_stream_init(stream, bbd, demux, embedding_pid, isi);
		dprintk("inited stream\n");
		stream->isi = isi;
		stream->last_crc = -1;
		old_stream = xa_cmpxchg(&bbd->bbframes_streams, isi, NULL /*old*/, stream, GFP_KERNEL);
		dprintk("registered stream for isi=%d stream=%p old_stream=%p stream.refcount=%d\n",
						isi, stream, old_stream,
						atomic_read(&stream->refcount.refcount.refs));
	} else {
		kref_get(&stream->refcount);
		dprintk("incremented refcount for isi=%d stream.refcount=%d\n", isi, atomic_read(&stream->refcount.refcount.refs));
	}
	if(old_stream) {
		kref_put(&stream->refcount, bbframes_stream_release_);
		kref_get(&old_stream->refcount);
		stream = old_stream;
		dprintk("incremented stream refcount stream=%p stream.refcount=%d\n", old_stream,
						atomic_read(&old_stream->refcount.refcount.refs));
	} else {
		dprintk("normal no change in refcount stream=%p stream.refcount=%d\n", stream,
						atomic_read(&stream->refcount.refcount.refs));
	}
	return stream;
}

static int dvbdmx_allocate_bbframes_stream_(struct dvb_demux* demux,
																					 struct bbframes_stream** stream_ret,
																					 int embedding_pid, int embedded_isi,
																					 struct dvb_demux_feeds* parent_feeds)
{
	struct bbframes_stream* stream =NULL;
	struct bbframes_demux* bbd = NULL;
	struct bbframes_demux* old_bbd =NULL;
	if(!parent_feeds)
		parent_feeds= demux->fe_feeds;
	BUG_ON(!parent_feeds);
	dprintk("embedded_pid=%d embedded_isi=%d fe_feeds=%p default_feeds=%p feeds=%p\n",
					embedding_pid, embedded_isi, demux->fe_feeds,
					demux->default_feeds,
					parent_feeds);
	//There is only bbframes_demux per pid, no matter how many users.
	bbd = xa_load(&parent_feeds->bbframes_demuxes, embedding_pid);
	if(!bbd) {
		if (!(bbd = kzalloc(sizeof(struct bbframes_stream), GFP_KERNEL))) {
			dprintk("could not bbframes_demux for embedded_pid=0x%04x isi=%d\n",
							embedding_pid, embedded_isi);
			return -EBUSY;
		}
		bbframes_demux_init(bbd, parent_feeds, embedding_pid);
		dprintk("ALLOC bbd=%p dvbdemux->feedarray=%p dvbdemux=%p bbd.refcount=%d\n",
						bbd, demux->feedarray, demux, atomic_read(&bbd->refcount.refcount.refs));

		old_bbd = xa_cmpxchg(&parent_feeds->bbframes_demuxes, embedding_pid, NULL /*old*/, bbd, GFP_KERNEL);
	} else {
		kref_get(&bbd->refcount);
		dprintk("incremented refcount bbd.refcount=%d\n", atomic_read(&bbd->refcount.refcount.refs));
	}
	if(old_bbd) {
		bbframes_demux_release(bbd);
		dprintk("RELEASED bbd=%p\n", bbd);
		bbd = old_bbd;
		kref_get(&bbd->refcount);
		dprintk("reused bbd=%p bbd.refcount=%d\n", bbd, atomic_read(&bbd->refcount.refcount.refs));
	} else {
		dprintk("normal bbd=%p bbd.refcount=%d\n", bbd, atomic_read(&bbd->refcount.refcount.refs));
	}
	stream = dvb_dmx_find_or_alloc_bbframes_stream(demux, bbd, embedding_pid, embedded_isi);
	dprintk("called  dvb_dmx_find_or_alloc_bbframes_stream stream=%p\n", stream);
	dprintk("before dvb_dmx_feed_alloc bbd=%p feeds=%p\n", bbd, bbd->parent_feeds);
	BUG_ON(!stream->feeds);

	*stream_ret = stream;

	return 0;
}

static int dvbdmx_allocate_bbframes_stream(struct dmx_demux* dmx_demux,
																					 struct dmx_bbframes_stream *stream_ret,
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

	ret=dvbdmx_allocate_bbframes_stream_(demux, &bbs, embedding_pid, embedded_isi, parent_feeds);
	if(ret>=0) {
		stream_ret->stream = bbs;
		stream_ret->feeds = bbs->feeds;
	}
	mutex_unlock(&demux->mutex);
	return ret;
}

static int dvbdmx_allocate_ts_feed(struct dmx_demux *dmx_demux, struct dmx_ts_feed **ts_feed,
																	 dmx_ts_cb callback, u16 pid, int ts_type,
																	 enum dmx_ts_pes pes_type, ktime_t timeout,
																	 struct dvb_demux_feeds* parent_feeds)
{
	struct dvb_demux* demux =  container_of(dmx_demux, struct dvb_demux, dmx);
	struct dvb_demux_feed *feed=0;
	if(!parent_feeds)
		parent_feeds= demux->default_feeds;

	BUG_ON(!parent_feeds);
	if (pid > DMX_MAX_PID)
		return -EINVAL;

	if (mutex_lock_interruptible(&demux->mutex))
		return -ERESTARTSYS;

	if (!(feed = dvb_dmx_feed_alloc(demux))) {
		mutex_unlock(&demux->mutex);
		return -EBUSY;
	}

	feed->type = DMX_TYPE_TS;
	feed->cb.ts = callback;
	feed->demux = demux;
	feed->pid = 0xffff;
	feed->peslen = 0xfffa;
	feed->buffer_flags = 0;
	feed->parent_feeds = parent_feeds;

	(*ts_feed) = &feed->feed.ts;
	(*ts_feed)->priv = NULL;
	(*ts_feed)->is_filtering = 0;
	(*ts_feed)->start_filtering = dmx_ts_feed_start_filtering;
	(*ts_feed)->stop_filtering = dmx_ts_feed_stop_filtering;

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

static int dvbdmx_release_bbframes_stream_(struct dvb_demux* demux, struct bbframes_stream* stream)
{
	BUG_ON(!stream->feeds);
	stream_dprintk(stream, "release dmx dvb_demux=%p\n", demux);
	bbframes_demux_dprintk(stream->bbframes_demux, "release dmx dvb_demux=%p\n", demux);

	dprintk("calling  bbframes_stream_release: stream=%p\n", stream);
	bbframes_stream_release(stream);
	bbframes_demux_dprintk(stream->bbframes_demux, "here\n");
	return 0;
}

static int dvbdmx_release_bbframes_stream(struct dmx_demux *dmx, struct dmx_bbframes_stream* dmx_bbframes_stream)
{
	struct dvb_demux *demux = container_of(dmx, struct dvb_demux, dmx);
	struct bbframes_stream* stream = dmx_bbframes_stream->stream;
	int ret;
	mutex_lock(&demux->mutex);
	ret = dvbdmx_release_bbframes_stream_(demux, stream);
	mutex_unlock(&demux->mutex);
	return ret;
}

static int dvbdmx_release_ts_feed(struct dmx_demux *dmx, struct dmx_ts_feed *ts_feed)
{
	struct dvb_demux *demux = container_of(dmx, struct dvb_demux, dmx);
	struct dvb_demux_feed *feed = container_of(ts_feed, struct dvb_demux_feed, feed.ts);
	struct dvb_demux_feeds *feeds = feed->parent_feeds;
	BUG_ON(!feeds);

	dmx_demux_dprintk(dmx, "release dmx dvb_demux=%p section_filter=%p\n",
										demux, feed->section_filter);
	mutex_lock(&demux->mutex);

	if (feed->state == DMX_STATE_FREE) {
		mutex_unlock(&demux->mutex);
		return -EINVAL;
	}

	feed->state = DMX_STATE_FREE;
	if(feed->section_filter)
		feed->section_filter->state = DMX_STATE_FREE;

	dvb_demux_feed_dprintk(feed, "calling  kref_put feeds=%p\n", feeds);
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

static int dvbdmx_allocate_section_feed(struct dmx_demux *demux, struct dmx_section_feed ** section_feed,
																				dmx_section_cb callback, u16 pid, bool check_crc,
																				struct dvb_demux_feeds* parent_feeds)
{
	struct dvb_demux *dvbdmx = (struct dvb_demux *)demux;
	struct dvb_demux_feed *feed;

	if (pid > 0x1fff)
		return -EINVAL;

	if (mutex_lock_interruptible(&dvbdmx->mutex))
		return -ERESTARTSYS;

	if (!(feed = dvb_dmx_feed_alloc(dvbdmx))) {
		mutex_unlock(&dvbdmx->mutex);
		return -EBUSY;
	}

	feed->type = DMX_TYPE_SEC;
	feed->cb.sec = callback;
	feed->demux = dvbdmx;
	feed->pid = 0xffff;
	feed->buffer_flags = 0;
	feed->feed.sec.secbuf = feed->feed.sec.secbuf_base;
	feed->feed.sec.secbufp = feed->feed.sec.seclen = 0;
	feed->feed.sec.tsfeedp = 0;
	feed->section_filter = NULL;
	feed->parent_feeds = parent_feeds;

	(*section_feed) = &feed->feed.sec;
	(*section_feed)->is_filtering = 0;
	(*section_feed)->parent_dmx_demux = demux;
	(*section_feed)->priv = NULL;

	(*section_feed)->allocate_section_filter = dmx_section_feed_allocate_filter;
	(*section_feed)->start_section_filtering = dmx_section_feed_start_filtering;
	(*section_feed)->stop_section_filtering = dmx_section_feed_stop_filtering;
	(*section_feed)->release_section_filter = dmx_section_feed_release_filter;

	dvb_demux_output_feed_add(feed);

	feed->pid = pid;
	feed->feed.sec.check_crc = check_crc;

	feed->state = DMX_STATE_READY;
	mutex_unlock(&dvbdmx->mutex);
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
	dmx->allocate_ts_feed = dvbdmx_allocate_ts_feed;
	dmx->allocate_bbframes_stream = dvbdmx_allocate_bbframes_stream;
	dmx->release_ts_feed = dvbdmx_release_ts_feed;
	dmx->release_bbframes_stream = dvbdmx_release_bbframes_stream;
	dmx->allocate_section_feed = dvbdmx_allocate_section_feed;
	dmx->release_section_feed = dvbdmx_release_section_feed;

	dmx->add_frontend = dvbdmx_add_frontend;
	dmx->remove_frontend = dvbdmx_remove_frontend;
	dmx->get_frontends = dvbdmx_get_frontends;
	dmx->connect_frontend = dvbdmx_connect_frontend;
	dmx->disconnect_frontend = dvbdmx_disconnect_frontend;
	dmx->get_pes_pids = dvbdmx_get_pes_pids;

	dvbdemux->fe_feeds = kzalloc(sizeof(struct dvb_demux_feeds), GFP_KERNEL);
	dvb_demux_feeds_init(dvbdemux->fe_feeds, dvbdemux, -1, -1);
	dvbdemux->fe_feeds->demux = dvbdemux;

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

	demux->default_stream_id = default_stream_id;
	dprintk("called with embedding_is_on=%d embedding_pid=%d default_stream_id=%d \n", default_stream_id, embedding_pid, embedding_is_on);

	int ret;
	//default_feeds is for legacy programs who do not / cannot  call allocate_bbframes_streams
	BUG_ON(!xa_empty(&demux->default_feeds->bbframes_demuxes));
	if(demux->fe_bbframes_stream) {
		dprintk("calling dvbdmx_release_bbframes_stream_ to release demux->fe_bbframes_stream=%p\n", demux->fe_bbframes_stream);
		ret = dvbdmx_release_bbframes_stream_(demux, demux->fe_bbframes_stream);
		demux->fe_bbframes_stream = NULL;
		demux->fe_feeds->include_default_feeds = true;
	}
	if (embedding_is_on) {
		dprintk("calling dvbdmx_allocate_bbframes_stream_ to set demux->fe_bbframes_stream\n");
		ret = dvbdmx_allocate_bbframes_stream_(demux, &demux->fe_bbframes_stream, embedding_pid, default_stream_id, demux->fe_feeds);
		if(ret<0)
			return ret;

		BUG_ON(!demux->fe_bbframes_stream);
		BUG_ON(!demux->fe_bbframes_stream->feeds);
		demux->fe_bbframes_stream->feeds->include_default_feeds = true;
		demux->fe_feeds->include_default_feeds = false;
	} else {
		demux->fe_feeds->include_default_feeds = true;
	}
	mutex_unlock(&demux->mutex);
	return ret;
}

EXPORT_SYMBOL(dvb_demux_set_bbframes_state);
