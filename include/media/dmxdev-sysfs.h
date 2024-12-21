/*
 *
 * Copyright (c) <2025>, Deep Thought <deeptho@gmail.com>, all rights reserved
 * Author(s): deeptho@gmail.com
 *
 */
#ifndef DVB_DEMUX_SYSFS_H
#define DVB_DEMUX_SYSFS_H
struct dmxdev;

void dvb_dmxdev_remove_sysfs(struct dmxdev* dmxdev);
int dvb_dmxdev_make_sysfs(struct dmxdev* dmxdev);

#endif
