/*
 *
 * Copyright (c) <2025>, Deep Thought <deeptho@gmail.com>, all rights reserved
 * Author(s): deeptho@gmail.com
 *
 */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h> /* min */
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h> /* copy_from_user, copy_to_user */
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <media/dmxdev.h>
#include <media/dmxdev-sysfs.h>
#include <media/dvb_demux.h>

#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

static struct xarray sysfs_kobjects;

static ssize_t store_none(struct kobject* kobj, struct kobj_attribute *attr, const char *buf, size_t count);

static ssize_t dmxdev_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t dvb_demux_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);

static struct kobj_attribute dmxdev_sysfs_attribute =__ATTR(dmxdev, 0444, dmxdev_show, store_none);
static struct kobj_attribute dvb_demux_sysfs_attribute =__ATTR(demux, 0444, dvb_demux_show, store_none);


static ssize_t store_none(struct kobject* kobj, struct kobj_attribute *attr,
                      const char *buf, size_t count)
{
	return 0;
}


static struct dmxdev* find_dmxdev(struct kobject* kobj)
{
	struct dmxdev* dmxdev = (struct dmxdev*)xa_load(&sysfs_kobjects, (unsigned long) kobj);
	return dmxdev;
}


static ssize_t dmxdev_show(struct kobject* kobj, struct kobj_attribute *attr,
													 char *buf)
{
	/*
		show() must not use snprintf() when formatting the value to be
		returned to user space. If you can guarantee that an overflow
		will never happen you can use sprintf() otherwise you must use
		scnprintf().*/
	struct dmxdev* dmxdev = find_dmxdev(kobj);
	struct dvb_adapter* adapter = dmxdev->dvbdev->adapter;
	int adapter_no = adapter->num;
	int i;
	int ret=0;
	struct dmxdev_feed *feed;
	struct dvb_demux* dvb_demux;

	ret += sprintf(buf+ret,
								 "adapter_no=%d\n", adapter_no);
	for (i = 0; i < dmxdev->filternum; i++)
		if (dmxdev->filter[i].state != DMXDEV_STATE_FREE) {
			struct dmxdev_filter* filter =  &dmxdev->filter[i];
			if(!filter) {
				ret += sprintf(buf+ret, "slot[%i]=NULL\n", i);
				continue;
			}
			if(!filter->dev) {
				ret += sprintf(buf+ret, "slot[%i]->dev=NULL\n", i);
				continue;
			}
			dvb_demux = container_of(filter->dev->demux, struct dvb_demux, dmx);
			switch(filter->type) {
			case DMXDEV_TYPE_PES: {
				ret += sprintf(buf+ret, "filter[%d]=%p: current_feeds=%p\n", i, filter, filter->current_feeds);
				list_for_each_entry(feed, &filter->feed.dmxdev_feed_list, next) {

					switch(feed->feed_type) {
					case 	DMXDEV_FEED_TYPE_UNDEFINED:
					default:
						dprintk("Implementation error feed_type=%d\n", feed->feed_type);
						break;
					case DMXDEV_FEED_TYPE_STID: {
						struct dmx_stid_stream* bbs = container_of(feed, struct dmx_stid_stream, f);
						ret += sprintf(buf+ret, "  stid: embedding_pid=%d isi=%d stream=%p feeds=%p\n", bbs->embedding_pid, bbs->isi,
													 bbs->stream, bbs->feeds);
					}
						break;
					case DMXDEV_FEED_TYPE_T2MI: {
						struct dmx_t2mi_stream* t2mi = container_of(feed, struct dmx_t2mi_stream, f);
						ret += sprintf(buf+ret, "  t2mi: embedding_pid=%d isi=%d stream=%p feeds=%p\n", t2mi->embedding_pid,
													 t2mi->isi, t2mi->stream, t2mi->feeds);
					}
						break;
					case DMXDEV_FEED_TYPE_PID: {
						struct dmx_pid_feed* pid_feed = container_of(feed, struct dmx_pid_feed, f);
						ret += sprintf(buf+ret, "  feed: pid=%d stream=%p\n", pid_feed->pid,
													 pid_feed->pid_stream);
					}
					}
				}
				break;
				default:
					ret += sprintf(buf+ret, "filter[%d]=%p: type=%d\n", i, filter, filter->type);
					break;
			}
			}
		}
	return ret;
}

static ssize_t dvb_demux_show_feeds(const char* feeds_name, struct dvb_demux_feeds* feeds,
																		char* buf, int ret, int indent)
{
	unsigned long index;
	unsigned long index1;
	void* entry;
	struct dvb_demux_feed* of;
	ret += sprintf(buf+ret,   "%*s%s=%p:", indent, " ", feeds_name, feeds);
	if(feeds) {
		ret += sprintf(buf+ret, "embedding_pid=%d isi=%d feeds.refcount=%d\n",
									 feeds->embedding_pid, feeds->isi,
									 atomic_read(&feeds->refcount.refcount.refs));
	} else
		ret += sprintf(buf+ret,   "\n");

	if(feeds) {
		indent += 2;
		if(xa_empty(&feeds->embedded_streams))
			ret += sprintf(buf+ret, "%*sNo embedded streams\n", indent, " ");
		else {
			xa_for_each(&feeds->embedded_streams, index, entry) {
				struct embedded_stream* emb = (struct embedded_stream*) entry;
				struct stid_stream* stid = embedded_stream_get_super_class(entry, EMBEDDED_STREAM_TYPE_STID);
				if(stid) {
					ret += sprintf(buf+ret, "%*sstid=%p bbframes_pid=%d parent_feeds=%p stid.refcount=%d\n", indent, " ",
												 stid, stid->emb.embedding_pid, stid->emb.parent_feeds,
												 atomic_read(&stid->emb.refcount.refcount.refs));
					indent += 2;
					ret += sprintf(buf+ret, "%*sStreams:\n", indent, " ");
					indent += 2;
				} else {
					struct t2mi_stream* t2mi = embedded_stream_get_super_class(entry, EMBEDDED_STREAM_TYPE_T2MI);
					if(t2mi) {
						ret += sprintf(buf+ret, "%*st2mi=%p bbframes_pid=%d parent_feeds=%p cc_errs=%d crc8_errs=%d crc32_errs=%d "
													 "stid.refcount=%d\n", indent, " ",
													 t2mi, t2mi->emb.embedding_pid, t2mi->emb.parent_feeds,
													 t2mi->num_cc_errors,
													 t2mi->num_crc8_errors, t2mi->num_crc32_errors,
													 atomic_read(&t2mi->emb.refcount.refcount.refs));
						indent += 2;
						ret += sprintf(buf+ret, "%*sStreams:\n", indent, " ");
						indent += 2;
					} else {
						dprintk("Uninitialized stream type\n");
						WARN_ON(true);
					}
				}
				if(emb && emb->num_streams>0) {
					ret += sprintf(buf+ret, "%*sISI/PLP:matype: ", indent, " ");
					int isi;
					for(isi=0; isi <256;++isi) {
						if(!((emb->isi_plp_bitset[(isi>>5)&0x7] >> (isi&31))&1))
							continue;
						ret += sprintf(buf+ret, " %d:0x%x", isi, emb->matypes[isi]);
						if((emb->high_rolloff_mode[(isi>>5)&0x7] >> (isi&31))&1)
							ret += sprintf(buf+ret, " +");
					}
					ret += sprintf(buf+ret, "\n");
				}
				xa_for_each(&emb->bbf_streams, index1, entry) {
					struct bbframes_stream* bbf = (struct bbframes_stream*) entry;
					char name1[256];
					ret += sprintf(buf+ret, "%*sbbf=%p isi=%d upl=%d dfl=%d issy=%d bbf.refcount=%d emb=%p "
												 "feeds=%p crc8_errs=%d\n", indent, " ",
												 bbf, bbf->isi, bbf->upl, bbf->dfl, bbf->issy,
												 atomic_read(&bbf->refcount.refcount.refs), bbf->parent_embedded_stream, bbf->feeds, bbf->num_crc_errors);
					indent += 2;
					sprintf(name1, "%s[isi=%d]", feeds_name, bbf->isi);
					ret = dvb_demux_show_feeds(name1, bbf->feeds, buf, ret, indent);
					indent -=2;
				}
				indent -= 4;
			}
		}

		if (list_empty(&feeds->output_feed_list))
			ret += sprintf(buf+ret, "%*sNo output feeds\n", indent, " ");
		else {
			list_for_each_entry(of, &feeds->output_feed_list, next) {
				ret += sprintf(buf+ret,   "%*sfeed=%p pid=%d type=%d parent_feeds=%p index=%d\n",
											 indent, " ",
											 of, of->pid, of->type, of->parent_feeds, of->index);
			}
		}
		if(feeds->include_default_feeds)
			ret += sprintf(buf+ret, "%*sIncluding default feeds\n", indent, " ");
	} else
		ret += sprintf(buf+ret,   "\n");
	return ret;
}

static ssize_t dvb_demux_show(struct kobject* kobj, struct kobj_attribute *attr,
													 char *buf)
{
	/*
		show() must not use snprintf() when formatting the value to be
		returned to user space. If you can guarantee that an overflow
		will never happen you can use sprintf() otherwise you must use
		scnprintf().*/
	struct dmxdev* dmxdev = find_dmxdev(kobj);
	struct dvb_adapter* adapter = dmxdev->dvbdev->adapter;
	int adapter_no = adapter->num;
	int ret=0;
	struct dmx_demux* dmx_demux = dmxdev->demux;
	struct dvb_demux* d = container_of(dmx_demux, struct dvb_demux, dmx);
	ret += sprintf(buf+ret,
								 "adapter_no=%d\n", adapter_no);
	ret += sprintf(buf+ret, "dvb_demux=%p: fe_feeds=%p default_feeds=%p default_stream_id=%d\n",
								 d, d->fe_feeds, d->default_feeds, d->default_stream_id);
	ret=dvb_demux_show_feeds("fe_feeds", d->fe_feeds, buf, ret, 2);
	ret=dvb_demux_show_feeds("default_feeds", d->default_feeds, buf, ret, 2);

	return ret;
}


void dvb_dmxdev_remove_sysfs(struct dmxdev* dmxdev) {
	xa_erase(&sysfs_kobjects, (unsigned long) dmxdev->sysfs_kobject);
	kobject_put(dmxdev->sysfs_kobject);
}

int dvb_dmxdev_make_sysfs(struct dmxdev* dmxdev)
{
	int error = 0;
	char name[128];
	struct dvb_device* dvbdev = dmxdev->dvbdev;
	struct dvb_adapter* adapter = dvbdev->adapter;

	struct kobject* mod_kobj = &(((struct module*)(THIS_MODULE))->mkobj).kobj;
	int adapter_no = adapter->num;
	sprintf(name, "demux%d", adapter_no);
	dmxdev->sysfs_kobject = kobject_create_and_add(name, mod_kobj);
	if(!dmxdev->sysfs_kobject)
		return -ENOMEM;
	xa_store(&sysfs_kobjects, (unsigned long) dmxdev->sysfs_kobject, dmxdev, GFP_KERNEL);

	error = sysfs_create_file(dmxdev->sysfs_kobject, &dmxdev_sysfs_attribute.attr);

	if (error) {
		dprintk("failed to create the dbmxdev sysfs file\n");
	}

	error = sysfs_create_file(dmxdev->sysfs_kobject, &dvb_demux_sysfs_attribute.attr);

	if (error) {
		dprintk("failed to create the dvb_demux sysfs file\n");
	}

	return error;
}
