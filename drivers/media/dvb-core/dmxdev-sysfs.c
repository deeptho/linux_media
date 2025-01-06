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
	struct dvb_demux_feeds *feeds;
	struct dvb_demux_feed* demux_feed;
	struct dvb_demux_feed* of;
	struct bbframes_demux* bbd;
	struct dmx_ts_feed* ts_feed ;
	unsigned long index;
	struct dmx_bbframes_stream* stream;
	void* entry;
	struct dvb_demux* dvb_demux;

	ret += sprintf(buf+ret,
								 "adapter_no=%d\n", adapter_no);
	for (i = 0; i < dmxdev->filternum; i++)
		if (dmxdev->filter[i].state != DMXDEV_STATE_FREE) {
			struct dmxdev_filter* f =  &dmxdev->filter[i];
			dvb_demux = container_of(f->dev->demux, struct dvb_demux, dmx);
			switch(f->type) {
			case DMXDEV_TYPE_PES:
				ret += sprintf(buf+ret, "filter[%d]=%p: current_feeds=%p\n", i, f, f->current_feeds);
#if 0
				ret += sprintf(buf+ret, " demux=%p: demux->feedsx=%p\n", dvb_demux, dvb_demux->feedsx);
#else
				ret += sprintf(buf+ret, " demux=%p: demux->fe_feeds=%p demux->default_feeds=%p\n",
											 dvb_demux, dvb_demux->fe_feeds, dvb_demux->default_feeds);
#endif
				ret += sprintf(buf+ret, "\n  <<<ts feeds>>>\n");
				list_for_each_entry(feed, &f->feed.dmxdev_feed_list, next) {
					ts_feed = feed->ts;
					demux_feed = container_of(ts_feed, struct dvb_demux_feed, feed.ts);
					ret += sprintf(buf+ret, "  dmxdev_feed=%p pid=%d tsfeed=%p demux_feed=%p\n",
												 feed, feed->pid, ts_feed, demux_feed);
					feeds = demux_feed->parent_feeds;
					ret += sprintf(buf+ret,   "  pid=%d type=%d\n", demux_feed->pid, demux_feed->type);
					ret += sprintf(buf+ret,   "  parent_feeds=%p index=%d\n", feeds, demux_feed->index);
					if(feeds) {
						ret += sprintf(buf+ret, "  embedding_pid=%d isi=%d refcnt=%d\n", feeds->embedding_pid, feeds->isi,
													 atomic_read(&feeds->refcount.refcount.refs));

						ret += sprintf(buf+ret, "  ###bbframes_demuxes:###\n");
						xa_for_each(&feeds->bbframes_demuxes, index, entry) {
							bbd = (struct bbframes_demux*) entry;
							ret += sprintf(buf+ret, "  bbd=%p bbframes_pid=%d parent_feeds=%p\n",
														 bbd, bbd->embedding_pid, bbd->parent_feeds);
						}
						ret += sprintf(buf+ret, "  ###output_feeds_list:###\n");
						list_for_each_entry(of, &feeds->output_feed_list, next) {
							ret += sprintf(buf+ret, "  demux_feed=%p pid=%d type=%d  parent_feeds=%p index=%d\n",
														 of, of->pid,
														 of->type, of->parent_feeds, of->index);
						}
						ret += sprintf(buf+ret, "  #######################\n");
					}
					ret += sprintf(buf+ret, "\n");
				}

				ret += sprintf(buf+ret, "\n  <<<<<<<>>>>>>>\n");
				ret += sprintf(buf+ret, "\n  <<<bbframes_streams>>>\n");
				list_for_each_entry(stream, &f->dmxdev_bbframes_stream_list, next) {
					ret += sprintf(buf+ret, "  bb_frames_stream=%p embedding_pid=%d isi=%d\n",
												 stream, stream->embedding_pid, stream->isi);
					feeds = stream->feeds;
					ret += sprintf(buf+ret,   "  pid=%d type=%d\n", demux_feed->pid, demux_feed->type);
					ret += sprintf(buf+ret,   "  feeds=%p index=%d\n", feeds, demux_feed->index);
					if(feeds) {
						ret += sprintf(buf+ret, "  embedding_pid=%d isi=%d refcnt=%d\n", feeds->embedding_pid, feeds->isi,
													 atomic_read(&feeds->refcount.refcount.refs));

						ret += sprintf(buf+ret, "  ###bbframes_demuxes:###\n");
						xa_for_each(&feeds->bbframes_demuxes, index, entry) {
							bbd = (struct bbframes_demux*) entry;
							ret += sprintf(buf+ret, "  bbd=%p bbframes_pid=%d parent_feeds=%p\n",
														 bbd, bbd->embedding_pid, bbd->parent_feeds);
						}
						ret += sprintf(buf+ret, "  ###output_feeds_list:###\n");
						list_for_each_entry(of, &feeds->output_feed_list, next) {
							ret += sprintf(buf+ret, "  demux_feed=%p pid=%d type=%d  parent_feeds=%p index=%d\n",
														 of, of->pid,
														 of->type, of->parent_feeds, of->index);
						}
						ret += sprintf(buf+ret, "  #######################\n");
					}
					ret += sprintf(buf+ret, "\n");
				}
				ret += sprintf(buf+ret, "\n  <<<<<<<>>>>>>>\n");
				break;
			default:
				ret += sprintf(buf+ret, "filter[%d]=%p: type=%d\n", i, f, f->type);
				break;
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
	ret += sprintf(buf+ret,   "%*s%s=%p: is_default_feeds=%d", indent, " ", feeds_name, feeds, feeds->is_default_feeds);
	if(feeds) {
		indent += 2;
		ret += sprintf(buf+ret, "%*sembedding_pid=%d isi=%d feeds.refcount=%d\n", indent, " ",
									 feeds->embedding_pid, feeds->isi,
									 atomic_read(&feeds->refcount.refcount.refs));

		ret += sprintf(buf+ret, "%*s###bbframes_demuxes:###\n", indent, " ");
		indent +=2;
		xa_for_each(&feeds->bbframes_demuxes, index, entry) {
			struct bbframes_demux* bbd = (struct bbframes_demux*) entry;
			void*  entry1;
			ret += sprintf(buf+ret, "%*sbbd=%p bbframes_pid=%d parent_feeds=%p bbd.refcount=%d\n", indent, " ",
										 bbd, bbd->embedding_pid, bbd->parent_feeds, atomic_read(&bbd->refcount.refcount.refs));
			ret += sprintf(buf+ret, "%*s###streams:###\n", indent, " ");
			indent+=2;
			xa_for_each(&bbd->bbframes_streams, index1, entry1) {
				struct bbframes_stream* stream = (struct bbframes_stream*) entry1;
				char name1[256];
				ret += sprintf(buf+ret, "%*sstream=%p isi=%d stream.refcount=%d bbd=%p feeds=%p\n",indent, " ",
											 stream, stream->isi, atomic_read(&stream->refcount.refcount.refs), stream->bbframes_demux, stream->feeds);
				sprintf(name1, "%s[isi=%d]", feeds_name, stream->isi);
				ret += dvb_demux_show_feeds(name1, stream->feeds, buf, ret, indent);
			}
			indent -=2;
		}
		indent -=2;
		ret += sprintf(buf+ret, "\n");
		indent +=2;
		list_for_each_entry(of, &feeds->output_feed_list, next) {
			ret += sprintf(buf+ret,   "%*sfeed=%p\n", indent, " ", of);
			ret += sprintf(buf+ret,   "%*spid=%d type=%d parent_feeds=%p index=%d next=%p prev=%p\n",
										 indent, " ", of->pid, of->type,
										 of->parent_feeds, of->index, of->next.next, of->next.prev);
		}
		ret += sprintf(buf+ret, "\n");
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
	ret += sprintf(buf+ret, "dvb_demux=%p: fe_feeds=%p default_feeds=%p\n",
								 d, d->fe_feeds, d->default_feeds);
	ret += sprintf(buf+ret, "default_stream_id=%d\n\n", d->default_stream_id);

	ret=dvb_demux_show_feeds("fe_feeds", d->fe_feeds, buf, ret, 2);
	ret=dvb_demux_show_feeds("default_feeds", d->default_feeds, buf, ret, 2);
	//ret=dvb_demux_show_feeds("feeds", d->feeds, buf, ret, 2);

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
