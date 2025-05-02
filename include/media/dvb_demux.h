/*
 * dvb_demux.h: DVB kernel demux API
 *
 * Copyright (C) 2000-2001 Marcus Metzler & Ralph Metzler
 *                         for convergence integrated media GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _DVB_DEMUX_H_
#define _DVB_DEMUX_H_

#include <linux/time.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#include <media/demux.h>

/**
 * enum dvb_dmx_filter_type - type of demux feed.
 *
 * @DMX_TYPE_TS:	feed is in TS mode.
 * @DMX_TYPE_SEC:	feed is in Section mode.
 */
enum dvb_dmx_filter_type {
	DMX_TYPE_TS,
	DMX_TYPE_SEC
};

/**
 * enum embedded_stream_type - type of embedded stream.
 *
 * @EMBEDDED_STREAM_TYPE_STID: bbbframes embedded in TS stream in stid format
 * @EMBEDDED_STREAM_TYPE_T2MI: bbbframes embedded in TS stream in t2mi format
 */
enum embedded_stream_type {
	EMBEDDED_STREAM_TYPE_UNKNOWN,
	EMBEDDED_STREAM_TYPE_STID,
	EMBEDDED_STREAM_TYPE_T2MI,
};


/**
 * enum dvb_dmx_state - state machine for a demux filter.
 *
 * @DMX_STATE_FREE:		indicates that the filter is freed.
 * @DMX_STATE_ALLOCATED:	indicates that the filter was allocated
 *				to be used.
 * @DMX_STATE_READY:		indicates that the filter is ready
 *				to be used.
 * @DMX_STATE_GO:		indicates that the filter is running.
 */
enum dvb_dmx_state {
	DMX_STATE_FREE,
	DMX_STATE_ALLOCATED,
	DMX_STATE_READY,
	DMX_STATE_GO,
};

#define DVB_DEMUX_MASK_MAX 18

#define MAX_PID 0x1fff

#define SPEED_PKTS_INTERVAL 50000

/**
 * struct dvb_demux_section_filter - Describes a DVB demux section filter.
 *
 * @filter:		Section filter as defined by &struct dmx_section_filter.
 * @maskandmode:	logical ``and`` bit mask.
 * @maskandnotmode:	logical ``and not`` bit mask.
 * @doneq:		flag that indicates when a filter is ready.
 * @next:		pointer to the next section filter.
 * @feed:		&struct dvb_demux_feed pointer.
 * @index:		index of the used demux filter.
 * @state:		state of the filter as described by &enum dvb_dmx_state.
 * @type:		type of the filter as described
 *			by &enum dvb_dmx_filter_type.
 */
struct dvb_demux_section_filter {
	struct dmx_section_filter filter;
	u8 maskandmode[DMX_MAX_FILTER_SIZE];
	u8 maskandnotmode[DMX_MAX_FILTER_SIZE];
	bool doneq;

	struct dvb_demux_section_filter *next;
	struct dvb_demux_feed *feed;
	int index;
	enum dvb_dmx_state state;
	enum dvb_dmx_filter_type type;

	/* private: used only by av7110 */
	u16 hw_handle;
};


/**
 * struct dvb_demux_feeds - Describes a set of feeds used by the demux or by bbframes stream
 *
 * @demux:		The hardware demux using the feeds
 * @embedding_pid:	the PID in which input bbframes are embedded, or -1 if the input is a transport stream
 * @isi:	    The Input Stream Identifier (ISI) of the transport stream (embedded in bbframes) to demux
 * @include_default_feeds: If true, this struct dvb_demux_feeds is used for feeds not specifying a dvb_demux_feeds
 * @refcount:	The number of &struct dvb_demux output feeds and sub demux feeds currently using this dvb_demux_feeds
 * @output_feed_list:	List of &struct dvb_demux_feed output feeds. The feeds can have the same PID if multiple users
               opened the same demux. Some feeds can be outputting sections, and others transport stream data
 * @bbframes_demuxes:	Array of &struct bbframes_demux subdemuxes, one per embedding PID. If multiple users open
               the demux and demux the same pid, then all of them will share the same bbframes_demux, ensuring
							 that de-embedding is done only once.
 * @cnt_storage:	Counters (one per pid) used for TS continuity check
 * @speed_last_time:	&ktime_t used for TS speed check.
 * @speed_pkts_cnt:	packets count used for TS speed check.
 */
struct dvb_demux_feeds {
	struct dvb_demux* demux;
	int embedding_pid;
	int isi;
	bool include_default_feeds;

	struct kref refcount;
	struct list_head output_feed_list; //list of dvb_demux_feed
	struct xarray embedded_streams; //struct stid_stream, or struct t2mi streamindexed by embedding pid
	uint8_t *cnt_storage; /* for TS continuity check */
	ktime_t speed_last_time; /* for TS speed check */
	uint32_t speed_pkts_cnt; /* for TS speed check */
};

/**
 * struct ts_stream - represents the state of a partial TS packet
 * @cc_counter:	The continuity counter of the packet
 * @synced:	we have found the start of a packet (stream is synced)
 * @buff_idx:	number of bytes already in the packet
 * @buff: bytes of the packet
 */
struct ts_stream {
	int buff_idx;
	bool synced;
	uint8_t buff[200];
};


/**
 * struct bbframes_stream - represents a single stream embedded in bbframes
 * @isi:	The Input Stream Identifier (ISI) of the stream.
 * @upl:	User Packet Length.
 * @dfl:	Data Field Length.
 * @syncd:	Syncd field from stream.
 * @issy: issy
 * @syncbyte:	The sync byte (normally 0x47)
 * @synced:		True if the strame has been synced (a bbframe header was received for isi)
 * @last_crc:	 The computed CRC of the last received user packet.
 * @buff_count:	The currently received number of bytes in the current user packet
 * @refcount:	The number dmx_demux_feed feeds plus the number dmx_bbframes_stream feeds currently using the stream
 * @bbframes_demux:	The &struct bbframes_demux that extracts data for this stream
 * @feeds:		The &struct dvb_demux_feeds conttaing a list of struct dvb_demux otuput feeds and
 *            bbframes_demux feeds to which data from this stream will be sent.
 * @buff:	An internal buffer in which to store bbframe data (oversized).
 */
struct bbframes_stream {
	int isi;
	struct kref refcount;
	struct embedded_stream* parent_embedded_stream;
	struct ts_stream ts;
	struct dvb_demux_feeds* feeds;

	int matype;
	int upl;
	int dfl;
	int syncd;
	int issy;

	int bbf_crc8;

	int bbf_payload_bytes_left; //number of bytes still to be read until frame is complete
	uint8_t syncbyte;
	bool synced;
	bool hem_mode;

	int num_crc_errors;
};

/**
 * struct embedded_stream - common part of stid_stream and t2mi_stream:
 * one or more streams embedded in a transport stream
 * @embedded_stream_type: type of embedded stream (e.g., stid or t2mi)
 * @embedding_pid: pid of the TS in which the stream is embedded
 * @bbf_streams: array containing all streams to decode, i.e., all streams for which one or more users are registered
 * @refcount:	The number of &struct bbframes_stream_streams currently using this bbframes_demux.
 *            This is the same as the number of entries in bbframes_streams.
 * @parent_feeds:	The &struct dvb_demux_feeds containing the @bbframes_demuxes xarray from which
 *            which onws this bbframes_demux
 * @payload_len{0}: payload length in bytes
 * @packet_count: packet count
 * @bbf_header_crc8: crc8 of the output TS bytes already received
 * @current_isi: ISI of the stream currently being processed
 * @current_bbf: pointer to the struct bbframes_stream for the stream currently being processed
 * @cc_counter: continuity counter of the input stream
 * @isi_plp_bitset: each one bit indicates an ISI or plp_id value seen in the stream
 * @high_rolloff_mode[256]:  bitset with bitset[idx]=1 if stream with ISI==idx uses high roll-off mode
 * @num_streams[256]: the number of ISIs or PLPs seen in the stream
 * @isi_plps[256]: isi_plps[idx] the matype of the stream with ISI or PLP equal to idx
 */
struct embedded_stream {
	enum embedded_stream_type embedded_stream_type;
	int embedding_pid;
	struct xarray bbf_streams; //bbframes_stream map
	struct kref refcount;
	struct dvb_demux_feeds* parent_feeds;
	int payload_len;
	int packet_count;
	int bbf_header_crc8;
	int current_isi;
	struct bbframes_stream *current_bbf;
	int cc_counter;  //cc_counter of the input packets for this stream
	int32_t isi_plp_bitset[8];
	int32_t high_rolloff_mode[8];
	int num_streams;
	uint8_t matypes[256];
};

//stid bbframes encapsulated stream embedded in ts backets
/**
 * struct stid_stream - state of a bbframes stream embedded in a transport stream
 * @embedding_pid: pid of the TS in which the stream is embedded
 * @payload_len{0}: payload length in bytes
 * @packet_count: packet count
 * @section_length: section length
 * @bbf_header_crc8: crc8 of the output TS bytes already received
 * @currenr_isi: ISI of the stream currently being processed
 * @cc_counter: continuity counter of the input stream
 * @refcount:	The number of &struct bbframes_stream_streams currently using this bbframes_demux.
 *            This is the same as the number of entries in bbframes_streams.
 * @parent_feeds:	The &struct dvb_demux_feeds containing the @bbframes_demuxes xarray from which
 *            which onws this bbframes_demux
 */
struct stid_stream {
	struct embedded_stream emb;
	int section_length;
};

/**
 * struct t2mi_stream - state of a t2mi stream embedded in a transport stream
 * @emb: state common with stid_stream
 * @packet_type: t2mi packet type
 * @superframe_idx: t2mi superframe_idx
 * @crc_idx: number of bytes already read of the 4-byte crc field
 * @synced: true when start of t2mi stream has been located
 * @frame_idx{-1}: t2mi bbframe frame_idx
 * @plp_id: t2mi bbframe  plp_id
 * @intl_frame_start: t2mi intl_frame_start
 * @t2mi_crc32: crc32 of t2mi bytes already read in the current t2mi frame
 * @t2mi_payload_bytes_left{0}: number of bytes still to be read in the t2mi frame
 * @bbheader_bytes_left: number of bytes still to read in bbheader
 * @header_idx; number of header bytes already read in the t2mi header (in case of partial read)
 * @bbheader_idx: number of header bytes alreadt read in the  t2mi-bbheader following the t2mi header
 *  (in case of partial read)
 * @buff:  buffer in which to store t2mi headers, t2mi-bbheaders and bbrame headers
 */
struct t2mi_stream {
	struct embedded_stream emb;
	int default_isi; //arbitrary isi to use if none was specifified by user
	int packet_type;
	int superframe_idx;
	int crc_idx;
	bool synced;
	int frame_idx;
	int plp_id;
	int packet_header_isi;
	uint8_t intl_frame_start;
	uint32_t t2mi_crc32;
	int t2mi_payload_bytes_left;
	int bbheader_bytes_left;
	int header_idx;
	int bbheader_idx;
	uint8_t buff[188];
	int num_crc_errors;
};


/**
 * struct dvb_demux_feed - describes a DVB feed from the point of view of the card
 * Each feed corresponds to one dmxdev_devfeed, which describes the user view of the feed
 *
 * @feed:	a union describing a digital TV feed.
 *		Depending on the feed type, it can be either
 *		@feed.ts or @feed.sec.
 * @feed.ts:	a &struct demux_pid_feed pointer.
 *		For TS feed only.
 * @feed.sec:	a &struct dmx_section_feed pointer.
 *		For section feed only.
 * @cb:		a union describing digital TV callbacks.
 *		Depending on the feed type, it can be either
 *		@cb.ts or @cb.sec.
 * @cb.ts:	a dmx_pid_cb() calback function pointer.
 *		For TS feed only.
 * @cb.sec:	a dmx_section_cb() callback function pointer.
 *		For section feed only.
 * @demux:	pointer to &struct dvb_demux.
 * @priv:	private data that can optionally be used by a DVB driver.
 * @type:	type of the filter, as defined by &enum dvb_dmx_filter_type.
 * @state:	state of the filter as defined by &enum dvb_dmx_state.
 * @pid:	PID to be filtered.
 * @timeout:	feed timeout.
 * @filter:	pointer to &struct dvb_demux_section_filter.
 * @buffer_flags: Buffer flags used to report discontinuity users via DVB
 *		  memory mapped API, as defined by &enum dmx_buffer_flags.
 * @ts_type:	type of TS, as defined by &enum ts_filter_type.
 * @pes_type:	type of PES, as defined by &enum dmx_ts_pes.
 * @cc:		MPEG-TS packet continuity counter
 * @pusi_seen:	if true, indicates that a discontinuity was detected.
 *		it is used to prevent feeding of garbage from previous section.
 * @peslen:	length of the PES (Packet Elementary Stream).
 * @list_head:	head for the list of digital TV demux feeds.
 * @index:	a unique index for each feed. Can be used as hardware
 *		pid filter index.
 * @parent_feeds:	The &struct dvb_demux_feeds data structure which owns this feed
 * @next:	The &struct list_head used to link dvb_demux_feeds which are in use
 */
struct dvb_demux_feed {
	union {
		struct pid_stream pid_stream;
		struct dmx_section_feed sec;
	} feed;

	union {
		dmx_pid_cb ts;
		dmx_section_cb sec;
	} cb;

	struct dvb_demux *demux;
	void *priv;
	enum dvb_dmx_filter_type type;
	enum dvb_dmx_state state;
	u16 pid;

	ktime_t timeout;
	struct dvb_demux_section_filter *section_filter;

	u32 buffer_flags;

	enum ts_filter_type ts_type;
	enum dmx_ts_pes pes_type;

	int cc;
	bool pusi_seen;

	u16 peslen;
	struct dvb_demux_feeds* parent_feeds;
	struct list_head next; //for dvb_demux (card side) list
	unsigned int index;
};

/**
 * struct dvb_demux - represents a digital TV demux
 * @dmx:		embedded &struct dmx_demux with demux capabilities
 *			and callbacks.
 * @priv:		private data that can optionally be used by
 *			a DVB driver.
 * @filternum:		maximum amount of DVB filters.
 * @feednum:		maximum amount of DVB feeds.
 * @start_feed:		callback routine to be called in order to start
 *			a DVB feed.
 * @stop_feed:		callback routine to be called in order to stop
 *			a DVB feed.
 * @write_to_decoder:	callback routine to be called if the feed is TS and
 *			it is routed to an A/V decoder, when a new TS packet
 *			is received.
 *			Used only on av7110-av.c.
 * @check_crc32:	callback routine to check CRC. If not initialized,
 *			dvb_demux will use an internal one.
 * @memcopy:		callback routine to memcopy received data.
 *			If not initialized, dvb_demux will default to memcpy().
 * @users:		counter for the number of demux opened file descriptors.
 *			Currently, it is limited to 10 users.
 * @section_filter:		pointer to &struct dvb_demux_section_filter.
 * @feedarray: &struct dvb_demux_feed array from which to allocate feeds.
 * @frontend_list:	&struct list_head with frontends used by the demux.
 * @pesfilter:		array of &struct dvb_demux_feed with the PES types
 *			that will be filtered.
 * @pids:		list of filtered program IDs.
 * @tsbuf:		temporary buffer used internally to store TS packets.
 * @tsbufp:		temporary buffer index used internally.
 * @mutex:		pointer to &struct mutex used to protect feed set
 *			logic.
 * @lock:		pointer to &spinlock_t, used to protect buffer handling.
 * @default_stream_id: set by frontend to help pick an Input Stream ID for legacy applications not picking one
 * @fe_bbframes_stream: &struct bbframes_stream allocated by frontend when it sends
 *          encapsulated bbfrmaes
 * @fe_feeds: &struct dvb_demux_feed container of feeds which the frontend will address directly
 * @default_feeds: &struct dvb_demux_feed container of feeds used by frontend applications
  * @kobject* sysfs_kobject;
 */
struct dvb_demux {
	struct dmx_demux dmx;
	void *priv;
	int filternum;
	int feednum;
	int (*start_feed)(struct dvb_demux_feed *feed);
	int (*stop_feed)(struct dvb_demux_feed *feed);
	int (*write_to_decoder)(struct dvb_demux_feed *feed,
				 const u8 *buf, size_t len);
	u32 (*check_crc32)(struct dvb_demux_feed *feed,
			    const u8 *buf, size_t len);
	void (*memcopy)(struct dvb_demux_feed *feed, u8 *dst,
			 const u8 *src, size_t len);

	int users; //only used to count max num users
#define MAX_DVB_DEMUX_USERS 10
	struct dvb_demux_section_filter *section_filter;
	struct dvb_demux_feed *feedarray; //array from which to allocate struct dvb_demux_feed

	struct list_head frontend_list;

	struct dvb_demux_feed *pesfilter[DMX_PES_OTHER];
	u16 pids[DMX_PES_OTHER];

#define DMX_MAX_PID 0x2000
	u8 tsbuf[204];
	int tsbufp;

	struct mutex mutex;
	spinlock_t lock;

	/* private: used only on av7110 */
	int playing;
	int recording;

	int default_stream_id;

	struct bbframes_stream* fe_bbframes_stream;

	struct dvb_demux_feeds* fe_feeds;
	struct dvb_demux_feeds* default_feeds;
	struct kobject* sysfs_kobject;
};

/**
 * dvb_dmx_init - initialize a digital TV demux struct.
 *
 * @demux: &struct dvb_demux to be initialized.
 *
 * Before being able to register a digital TV demux struct, drivers
 * should call this routine. On its typical usage, some fields should
 * be initialized at the driver before calling it.
 *
 * A typical usecase is::
 *
 *	dvb->demux.dmx.capabilities =
 *		DMX_TS_FILTERING | DMX_SECTION_FILTERING |
 *		DMX_MEMORY_BASED_FILTERING;
 *	dvb->demux.priv       = dvb;
 *	dvb->demux.filternum  = 256;
 *	dvb->demux.feednum    = 256;
 *	dvb->demux.start_feed = driver_start_feed;
 *	dvb->demux.stop_feed  = driver_stop_feed;
 *	ret = dvb_dmx_init(&dvb->demux);
 *	if (ret < 0)
 *		return ret;
 */
int dvb_dmx_init(struct dvb_demux *demux);

/**
 * dvb_dmx_release - releases a digital TV demux internal buffers.
 *
 * @demux: &struct dvb_demux to be released.
 *
 * The DVB core internally allocates data at @demux. This routine
 * releases those data. Please notice that the struct itelf is not
 * released, as it can be embedded on other structs.
 */
void dvb_dmx_release(struct dvb_demux *demux);

/**
 * dvb_dmx_copy_data - copy data for a buffer with
 *	multiple data packets with a multuple of 188 bytes.
 *
 * @demux: pointer to &struct dvb_demux
 * @buf: buffer with data to be copied
 * @count: number of MPEG-TS packets with size of 188.
 *
 *
 * Use this routine if the DVB demux fills MPEG-TS buffers that are
 * already aligned.
 *
 * NOTE: The @buf size should have size equal to ``count * 188``.
 */
void dvb_dmx_copy_data(struct dvb_demux *demux, const u8 *buf, size_t count);

/**
 * dvb_dmx_swfilter_packets - use dvb software filter for a buffer with
 *	multiple MPEG-TS packets with 188 bytes each.
 *
 * @demux: pointer to &struct dvb_demux
 * @buf: buffer with data to be filtered
 * @count: number of MPEG-TS packets with size of 188.
 *
 * The routine will discard a DVB packet that don't start with 0x47.
 *
 * Use this routine if the DVB demux fills MPEG-TS buffers that are
 * already aligned.
 *
 * NOTE: The @buf size should have size equal to ``count * 188``.
 */
void dvb_dmx_swfilter_packets(struct dvb_demux *demux, const u8 *buf, size_t count);

/**
 * dvb_dmx_swfilter -  use dvb software filter for a buffer with
 *	multiple MPEG-TS packets with 188 bytes each.
 *
 * @demux: pointer to &struct dvb_demux
 * @buf: buffer with data to be filtered
 * @count: number of MPEG-TS packets with size of 188.
 *
 * If a DVB packet doesn't start with 0x47, it will seek for the first
 * byte that starts with 0x47.
 *
 * Use this routine if the DVB demux fill buffers that may not start with
 * a packet start mark (0x47).
 *
 * NOTE: The @buf size should have size equal to ``count * 188``.
 */
void dvb_dmx_swfilter(struct dvb_demux *demux, const u8 *buf, size_t count);

/**
 * dvb_dmx_swfilter_204 -  use dvb software filter for a buffer with
 *	multiple MPEG-TS packets with 204 bytes each.
 *
 * @demux: pointer to &struct dvb_demux
 * @buf: buffer with data to be filtered
 * @count: number of MPEG-TS packets with size of 204.
 *
 * If a DVB packet doesn't start with 0x47, it will seek for the first
 * byte that starts with 0x47.
 *
 * Use this routine if the DVB demux fill buffers that may not start with
 * a packet start mark (0x47).
 *
 * NOTE: The @buf size should have size equal to ``count * 204``.
 */
void dvb_dmx_swfilter_204(struct dvb_demux *demux, const u8 *buf,
			  size_t count);

/**
 * dvb_dmx_swfilter_raw -  make the raw data available to userspace without
 *	filtering
 *
 * @demux: pointer to &struct dvb_demux
 * @buf: buffer with data
 * @count: number of packets to be passed. The actual size of each packet
 *	depends on the &dvb_demux->feed->cb.ts logic.
 *
 * Use it if the driver needs to deliver the raw payload to userspace without
 * passing through the kernel demux. That is meant to support some
 * delivery systems that aren't based on MPEG-TS.
 *
 * This function relies on &dvb_demux->feed->cb.ts to actually handle the
 * buffer.
 */
void dvb_dmx_swfilter_raw(struct dvb_demux *demux, const u8 *buf,
			  size_t count);


int dvb_demux_set_bbframes_state(struct dvb_demux* demux, bool embedding_is_on, int embedding_pid, int default_stream_id);


static inline void* embedded_stream_get_super_class(struct embedded_stream* emb,
																																	enum embedded_stream_type t) {
	if(!emb)
		return NULL;
	bool matches = (t==emb->embedded_stream_type || EMBEDDED_STREAM_TYPE_UNKNOWN);
	switch (emb->embedded_stream_type) {
	case EMBEDDED_STREAM_TYPE_UNKNOWN:
	default:
		BUG_ON(true);
		return NULL;
		break;
	case EMBEDDED_STREAM_TYPE_STID:
		return matches ? container_of(emb, struct stid_stream, emb) : NULL;
	case EMBEDDED_STREAM_TYPE_T2MI:
		return matches ? container_of(emb, struct t2mi_stream, emb) : NULL;
	}
}


#endif /* _DVB_DEMUX_H_ */
