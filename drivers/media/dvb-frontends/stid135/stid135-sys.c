/*
 *
* Copyright (c) <2021>-<2022>, Deep Thought, all rights reserved
* Author(s): deeptho@gmail.com
 *
* License terms: BSD 3-clause "New" or "Revised" License.
 *
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "i2c.h"
#include "chip.h"
#include "stid135_drv.h"

#define dprintk(fmt, arg...)																					\
	printk(KERN_DEBUG pr_fmt("%s:%d " fmt),  __func__, __LINE__, ##arg)

static ssize_t stv_chip_registers_show(struct file* filp, struct kobject *kobj, struct bin_attribute* bin_attr,
																			 char *buffer, loff_t pos, size_t size);
static ssize_t stv_chip_registers_store(struct file* filp, struct kobject *kobj, struct bin_attribute* bin_attr,
																				char *buffer, loff_t pos, size_t size);
static ssize_t store_none(struct kobject* kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t stv_card_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t stv_chip_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t stv_demod_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t stv_demod_show_default_rf_input(struct kobject *kobj, struct kobj_attribute *attr,
																							 char *buf);
static ssize_t stv_demod_set_default_rf_input(struct kobject* kobj, struct kobj_attribute *attr,
																							const char *buf, size_t count);

static ssize_t stv_blindscan_always_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t stv_blindscan_always_store(struct kobject* kobj, struct kobj_attribute *attr,
																					const char *buf, size_t count);

static ssize_t stv_temperature_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);

static struct kobj_attribute stv_card_sysfs_attribute =__ATTR(state, 0444, stv_card_show, store_none);
static struct kobj_attribute stv_blindscan_always_sysfs_attribute =__ATTR(blindscan_always, 0774,
																																					stv_blindscan_always_show, stv_blindscan_always_store);
static struct kobj_attribute stv_chip_sysfs_attribute =__ATTR(state, 0444, stv_chip_show, store_none);
static struct kobj_attribute stv_demod_sysfs_attribute =__ATTR(state, 0444, stv_demod_show, store_none);
static struct kobj_attribute stv_demod_sysfs_default_rf_input_attribute =
	__ATTR(default_rf_in, 0774, stv_demod_show_default_rf_input, stv_demod_set_default_rf_input);
static struct kobj_attribute stv_temperature_sysfs_attribute =__ATTR(temperature, 0444,
																																		 stv_temperature_show, store_none);

static struct bin_attribute stv_chip_sysfs_registers_stid135_attribute ={
	.attr = { .name ="stid135", .mode=0774},
	.size=0,
	.read=stv_chip_registers_show,
	.write=stv_chip_registers_store
};

static struct bin_attribute stv_chip_sysfs_registers_soc_attribute ={
	.attr = { .name ="soc", .mode=0774},
	.size=0,
	.read=stv_chip_registers_show,
	.write=stv_chip_registers_store
};

static struct bin_attribute stv_chip_sysfs_registers_vglna_attributes[] = {
		{.attr = { .name ="vglna0", .mode=0774},
	.size=0,
	.read=stv_chip_registers_show,
	.write=stv_chip_registers_store
	},
	{.attr = { .name ="vglna1", .mode=0774},
	.size=0,
	.read=stv_chip_registers_show,
	.write=stv_chip_registers_store
	},
	{.attr = { .name ="vglna2", .mode=0774},
	.size=0,
	.read=stv_chip_registers_show,
	.write=stv_chip_registers_store
	},
	{.attr = { .name ="vglna3", .mode=0774},
	.size=0,
	.read=stv_chip_registers_show,
	.write=stv_chip_registers_store
	}
};

static ssize_t registers_read_(STCHIP_Info_t* hChipHandle, char*buf, loff_t offset, size_t len)
{
    ssize_t ret=0;
		int first = offset/sizeof(STCHIP_Register_t);
		int last;
		int num_regs= len/sizeof(STCHIP_Register_t);
		int num_bytes;
		int idx;
		if(num_regs + first > hChipHandle->NbRegs)
			num_regs = hChipHandle->NbRegs - first;
		last = first + num_regs;
		dprintk("read: handle=%p offset=%lld first=%d num_regs=%d/%d s=%ld\n", hChipHandle, offset, first, num_regs,  hChipHandle->NbRegs,
						sizeof(STCHIP_Register_t));
		if (num_regs<=0)
			return 0;

		for(idx=first; idx < last; ) {
			int firstRegId= hChipHandle->pRegMapImage[first].Addr;
			//int LastRegId= hChipHandle->pRegMapImage[first].Addr;

			//Set n to the number of consecutive registers, with a maximum of 69
			int n=0;
			for(; idx < last && hChipHandle->pRegMapImage[idx].Addr -firstRegId == n && n<70 ; ++idx, ++n)
				{}
			dprintk("Register range: first=%d last=%d first_id=0x%x last_id=0x%x num=%d", first, last, firstRegId, firstRegId+n, n);
			ChipGetRegisters(hChipHandle, firstRegId, n);
			num_bytes = sizeof(STCHIP_Register_t)*n;
			memcpy(buf + ret, hChipHandle->pRegMapImage +first, num_bytes);
			dprintk("copied %d bytes from %p to %p", num_bytes,  hChipHandle->pRegMapImage +first, buf + ret);
			ret += num_bytes;
			first = idx;
		}
		dprintk("READ %ld bytes: first=%d %p\n", ret, first,  hChipHandle->pRegMapImage +first);
    return ret;
}

static ssize_t registers_read(STCHIP_Info_t* hChipHandle, char*buf, loff_t offset, size_t len)
{
    ssize_t ret=0;
		//ssize_t ret1=0;
		int x = offset % sizeof(STCHIP_Register_t);
		int len1 = sizeof(STCHIP_Register_t) -x;
		int ret1=0;
		char temp[sizeof(STCHIP_Register_t)];
		if(x >0) {
			//partial read at start
			ret1 = registers_read_(hChipHandle, temp, offset-x, sizeof(STCHIP_Register_t));
			if(ret1 > len)
				ret1 = len;
			memcpy(buf, temp + x, ret1);
			offset += ret1;
			len -= ret1;
			ret += ret1;
			buf += ret1;
			if(len ==0 || ret1 < len1 )
				return ret;
		}
		x = len % sizeof(STCHIP_Register_t);
		len1 = len  -x;
		if (len1>0) {
			ret1 = registers_read_(hChipHandle, buf, offset, len1);
			offset += ret1;
			len -= ret1;
			ret += ret1;
			buf += ret1;
			if(len ==0 || ret1 < len1)
				return ret;
		}
		//partial read at end
		ret1 = registers_read_(hChipHandle, temp, offset, sizeof(STCHIP_Register_t));
		if (ret1 <  sizeof(STCHIP_Register_t))
			return ret;
		memcpy(buf, temp, x);
		ret += ret1;
		return ret;
}

static ssize_t registers_write(STCHIP_Info_t* hChipHandle, const char __user *buf, size_t len) {
	char kbuf[128];
	STCHIP_Register_t* regdesc = (void*) & kbuf[0];
	//dprintk("Request of size %ld buf=%p\n", len, buf);
	if(len != sizeof(STCHIP_Register_t)) {
		dprintk("Incorrect len: %ld\n", len);
		return len;
	}

	memcpy(kbuf, buf, len);
	//dprintk("Asked to set %d-byte register 0x%04x to value 0x%08x ret=%d\n", regdesc->Size, regdesc->Addr, regdesc->Value, ret);
	ChipSetOneRegister(hChipHandle, regdesc->Addr, regdesc->Value);
	return len;
}


static struct stv_card_t* find_card(struct kobject* kobj)
{
	struct stv_card_t *p;
	list_for_each_entry(p, &stv_card_list, stv_card_list)
		if (p->sysfs_kobject == kobj)
			return p;
	return NULL;
}

static ssize_t stv_card_show(struct kobject *kobj, struct kobj_attribute *attr,
														 char *buf)
{
	/*
		show() must not use snprintf() when formatting the value to be
		returned to user space. If you can guarantee that an overflow
		will never happen you can use sprintf() otherwise you must use
		scnprintf().*/
	struct stv_card_t* card = find_card(kobj);
	int i;
	if (!card)
		return 0;
	int ret = sprintf(buf,
										"card_no = %d dev =%p\n"
										"use_count = %d\n",
										card->card_no, card->dev,
										card->use_count);
	if (card->num_chips>0) {
		ret += sprintf(buf+ret, "chips:");
		for(i=0; i < card->num_chips; ++i) {
			ret += sprintf(buf+ret, " %p", card->chips[i]);
		}
	ret += sprintf(buf+ret, "\n");
	}

	for(i=0; i< sizeof(card->rf_ins)/sizeof(card->rf_ins[0]); ++i) {
		struct stv_rf_in_t* rf_in =  &card->rf_ins[i];
		int controlling_chip_no =  rf_in->controlling_chip ?  rf_in->controlling_chip->chip_no :-1;
		const char*fn = card->lock.func;
		bool locked=mutex_is_locked(&card->lock.mutex);
		if(!fn)
			fn="";
		if (locked)
			ret += sprintf(buf+ret, "rf_in[%d]: use_count=%d owner=%d controller=%d"
										 " voltage=%s tone=%s mutex_locked_by=%d at %s:%d\n",
										 i, rf_in->reservation.use_count, rf_in->reservation.owner, controlling_chip_no,
										 voltage_str(rf_in->voltage), tone_str(rf_in->tone), card->lock.demod, fn, card->lock.line);
		else
			ret += sprintf(buf+ret, "rf_in[%d]: use_count=%d owner=%d controller=%d"
										 " voltage=%s tone=%s\n",
										 i, rf_in->reservation.use_count, rf_in->reservation.owner, controlling_chip_no,
										 voltage_str(rf_in->voltage), tone_str(rf_in->tone));
	}
	return ret;
}

static ssize_t store_none(struct kobject* kobj, struct kobj_attribute *attr,
                      const char *buf, size_t count)
{
	return 0;
}

static ssize_t stv_blindscan_always_show(struct kobject *kobj, struct kobj_attribute *attr,
																							 char *buf)
{
	struct stv_card_t* card = find_card(kobj);
	int blindscan_always = card->blindscan_always;
	int ret=0;
	ret += sprintf(buf+ret, "%d\n", blindscan_always);
	return ret;
}

static ssize_t stv_blindscan_always_store(struct kobject* kobj, struct kobj_attribute *attr,
																							const char *buf, size_t count) {
	int blindscan_always = 0;
	sscanf(buf,"%d",&blindscan_always);
	struct stv_card_t* card = find_card(kobj);
	card->blindscan_always = blindscan_always;
	dprintk("Set blindscan_always=%d\n", blindscan_always);
	return count;
}

int stv_card_make_sysfs(struct stv_card_t* card)
{
	int error = 0;
	int error1 = 0;
	char name[128];
	sprintf(name, "card%d", card->card_no);
	struct kobject* mod_kobj = &(((struct module*)(THIS_MODULE))->mkobj).kobj;
	card->sysfs_kobject = kobject_create_and_add(name, mod_kobj/*kernel_kobj*/);
	if(!card->sysfs_kobject)
		return -ENOMEM;

	error |= (error1 = sysfs_create_file(card->sysfs_kobject, &stv_card_sysfs_attribute.attr));
	if (error1) {
		dprintk("failed to create the stv_card sysfs state file\n");
	}

	error |= (error1 = sysfs_create_file(card->sysfs_kobject, &stv_blindscan_always_sysfs_attribute.attr));
	if (error1) {
		dprintk("failed to create the stv_card sysfs property \n");
	}

	return error;
}


static struct stv_chip_t* find_chip(struct kobject* kobj)
{
	struct stv_chip_t *p;
	list_for_each_entry(p, &stv_chip_list, stv_chip_list)
		if (p->sysfs_kobject == kobj)
			return p;
	return NULL;
}

static ssize_t stv_chip_show(struct kobject *kobj, struct kobj_attribute *attr,
														 char *buf)
{
	/*
		show() must not use snprintf() when formatting the value to be
		returned to user space. If you can guarantee that an overflow
		will never happen you can use sprintf() otherwise you must use
		scnprintf().*/
	struct stv_chip_t* chip = find_chip(kobj);
	int i;
	if (!chip)
		return 0;
	const char*fn = chip->lock.func;
	bool locked=mutex_is_locked(&chip->lock.mutex);
	if(!fn)
		fn="";

	int ret = 0;

	if (locked)
		ret= sprintf(buf, "card_no=%d chip_no=%d addr=0x%x i2c=%p\n"
								 "use_count = %d mutex_locked_by=%d at %s:%d\n",
								 chip->card->card_no, chip->chip_no, chip->adr, chip->i2c, chip->use_count,
								 chip->lock.demod, fn, chip->lock.line);
	else
		ret= sprintf(buf, "card_no=%d chip_no=%d addr=0x%x i2c=%p\n"
								 "use_count = %d\n",
								 chip->card->card_no, chip->chip_no, chip->adr, chip->i2c, chip->use_count);

	for(i=0; i < chip->num_tuners; ++i) {
		struct stv_rf_in_t* rf_in = chip->tuners[i].active_rf_in;
		ret += sprintf(buf+ret, "\ntuner[%d]: use_count=%d power=%d active_rf_in=%d\n",
									 i, chip->tuners[i].reservation.use_count, chip->tuners[i].powered_on,
									 rf_in ? rf_in->rf_in_no : -1);
		if(rf_in) {
			ret += sprintf(buf+ret, "           rf_in_use_count=%d voltage=%s tone=%s\n",
										 rf_in->reservation.use_count, voltage_str(rf_in->voltage), tone_str(rf_in->tone));
		}
		ret += sprintf(buf+ret, "vglna=%d control_22k=%d\n", chip->vglna, chip->control_22k);
	}
	return ret;
}

static ssize_t stv_chip_registers_show(struct file* filp, struct kobject *kobj, struct bin_attribute* bin_attr,
																char *buffer, loff_t pos, size_t size) {
	ssize_t ret=0;
	const char *p="???";
	struct stv_chip_t* chip = find_chip(kobj->parent);
	struct lock_t* lock = &chip->lock;
	mutex_lock(&lock->mutex);
	if (bin_attr == &stv_chip_sysfs_registers_stid135_attribute) {
		dprintk("READ %lu bytes at offset %llu stid135\n", size, pos);
		ret=registers_read(chip->ip.handle_demod, buffer, pos, size);
		dprintk("READ returns %ld\n", ret);
	}
	else if (bin_attr == &stv_chip_sysfs_registers_soc_attribute) {
		p = "soc";
		dprintk("READ %lu bytes at offset %llu soc\n", size, pos);
		ret=registers_read(chip->ip.handle_soc, buffer, pos, size);
		dprintk("READ returns %ld\n", ret);
		//state->chip->ip.handle_demod
	} else {
		int i;
		for(i=0; i< sizeof(stv_chip_sysfs_registers_vglna_attributes)/
					sizeof(stv_chip_sysfs_registers_vglna_attributes[0]); ++i) {
			if(bin_attr == &stv_chip_sysfs_registers_vglna_attributes[i]) {
				dprintk("READ %lu bytes at offset %llu handle=%p\n", size, pos, chip->ip.vglna_handles[i]);
				if(chip->ip.vglna_handles[i]!=NULL)
					ret=registers_read(chip->ip.vglna_handles[i], buffer, pos, size);
				dprintk("READ returns %ld\n", ret);
			}
		}
	}
	mutex_unlock(&lock->mutex);
	return ret;
}

static ssize_t stv_chip_registers_store(struct file* filp, struct kobject *kobj, struct bin_attribute* bin_attr,
																 char *buffer, loff_t pos, size_t size)  {
	dprintk("STORE kobj=%p pos=%lld size=%lud\n", kobj, pos, size);

	ssize_t ret=0;
	const char *p="???";
	struct stv_chip_t* chip = find_chip(kobj->parent);
	struct lock_t* lock = &chip->lock;
	mutex_lock(&lock->mutex);
	if (bin_attr == &stv_chip_sysfs_registers_stid135_attribute) {
		dprintk("WRITE %lu bytes at offset %llu\n", size, pos);
		ret=registers_write(chip->ip.handle_demod, buffer, size);
		dprintk("WRITE returns %ld\n", ret);
	}
	else if (bin_attr == &stv_chip_sysfs_registers_soc_attribute) {
		p = "soc";
		dprintk("WRITE %lu bytes at offset %llu\n", size, pos);
		ret=registers_write(chip->ip.handle_soc, buffer, size);
		dprintk("WRITE returns %ld\n", ret);
	} else {
		int i;
		for(i=0; i< sizeof(stv_chip_sysfs_registers_vglna_attributes)/
					sizeof(stv_chip_sysfs_registers_vglna_attributes[0]); ++i) {
			if(bin_attr == &stv_chip_sysfs_registers_vglna_attributes[i]) {
				dprintk("WRITE %lu bytes at offset %llu handle=%p\n", size, pos, chip->ip.vglna_handles[i]);
				if(chip->ip.vglna_handles[i]!=NULL)
					ret=registers_write(chip->ip.vglna_handles[i], buffer, size);
				dprintk("WRITE returns %ld\n", ret);
			}
		}
	}
	mutex_unlock(&lock->mutex);

	return size;
}


int stv_chip_make_sysfs(struct stv_chip_t* chip)
{
	int error = 0;
	char name[128];
	sprintf(name, "chip%d", chip->chip_no);
	chip->sysfs_kobject = kobject_create_and_add(name, chip->card->sysfs_kobject);
	if(!chip->sysfs_kobject)
		return -ENOMEM;

	error = sysfs_create_file(chip->sysfs_kobject, &stv_chip_sysfs_attribute.attr);
	if (error) {
		dprintk("failed to create the stv_chip sysfs file\n");
	}
	chip->sysfs_registers_kobject = kobject_create_and_add("registers", chip->sysfs_kobject);
	if(!chip->sysfs_registers_kobject)
		return -ENOMEM;
	dprintk("%s: chip=%p\n", name, chip);
	error = sysfs_create_bin_file(chip->sysfs_registers_kobject, &stv_chip_sysfs_registers_stid135_attribute);
	error = sysfs_create_bin_file(chip->sysfs_registers_kobject, &stv_chip_sysfs_registers_soc_attribute);
	if(chip->vglna) {
		int i;
		for(i=0; i < sizeof(chip->ip.vglna_handles)/sizeof(chip->ip.vglna_handles[0]); ++i) {
			error = sysfs_create_bin_file(chip->sysfs_registers_kobject, &stv_chip_sysfs_registers_vglna_attributes[i]);
		}
	}
	error = sysfs_create_file(chip->sysfs_kobject, &stv_temperature_sysfs_attribute.attr);

	dprintk("stv_chip_sysfs_registers_attribute=%p\n", &stv_chip_sysfs_registers_stid135_attribute);
	if (error) {
		dprintk("failed to create the stv_chip sysfs registers file\n");
	}
	return error;
}

int stv_chip_release_sysfs(struct stv_chip_t* chip)
{
	kobject_put(chip->sysfs_kobject);
	kobject_put(chip->sysfs_registers_kobject);
	return 0;
}

static struct stv* find_demod(struct kobject* kobj)
{
	struct stv *p;
	list_for_each_entry(p, &stv_demod_list, stv_demod_list)
		if (p->sysfs_kobject == kobj)
			return p;
	return NULL;
}

static ssize_t stv_temperature_show(struct kobject *kobj, struct kobj_attribute *attr,
															char *buf)
{
	/*
		show() must not use snprintf() when formatting the value to be
		returned to user space. If you can guarantee that an overflow
		will never happen you can use sprintf() otherwise you must use
		scnprintf().*/
	struct stv_chip_t* chip = find_chip(kobj);
	struct lock_t* lock = &chip->lock;
	int err;
	s16 temperature;
	int ret=0;
	mutex_lock(&lock->mutex);
	err = fe_stid135_get_soc_temperature(&chip->ip, &temperature);

	ret += sprintf(buf+ret, "%d\n", temperature);
	mutex_unlock(&lock->mutex);
	return ret;
}

static ssize_t stv_demod_show(struct kobject *kobj, struct kobj_attribute *attr,
															char *buf)
{
	/*
		show() must not use snprintf() when formatting the value to be
		returned to user space. If you can guarantee that an overflow
		will never happen you can use sprintf() otherwise you must use
		scnprintf().*/
	struct stv* state = find_demod(kobj);
	int rf_in = active_rf_in_no(state);
	int ret=0;
	int adapter_no=-1;
	if (!state)
		return 0;
	if(state->fe.dvb) {
		adapter_no= state->fe.dvb->num;
	}
	ret += sprintf(buf+ret,
								 "nr=%d adapter_no=%d default_rf_in=%d selected_rf_in=%d\n",
								 state->nr, adapter_no, state->fe.ops.info.default_rf_input, rf_in);
	ret += sprintf(buf+ret,
										"llr_in_use=%d\nfreq=%d\n",
								 state->llr_in_use, state->tuner_frequency);
	struct fe_sat_signal_info *info = &state->signal_info;
	ret += sprintf(buf+ret
								 ,"lock=%d fec=%d demod=%d error=%d signal=%d carrier=%d viterbi=%d sync=%d timedout=%d timing=%d\n",
								 info->has_lock,
								 info->fec_locked, info->demod_locked, info->has_error, info->has_signal, info->has_carrier, info->has_viterbi,
								 info->has_sync, info->has_timedout, info->has_timing_lock);
	ret += sprintf(buf+ret,"freq=%d sym_rate=%d\n",
								 info->frequency, info->symbol_rate);
	if (info->C_N>=0)
		ret += sprintf(buf+ret,"power=-%d.%ddBm cnr=%d/%ddB ber=%d\n\n",
									 (-info->power)/1000, (-info->power)%1000, info->C_N/10, info->C_N%10, info->ber);
	else
		ret += sprintf(buf+ret,"power=-%d.%ddBm cnr=-%d/%ddB ber=%d\n\n",
									 (-info->power)/1000, (-info->power)%1000, (-info->C_N)/10, (-info->C_N)%10, info->ber);
	return ret;
}

static ssize_t stv_demod_show_default_rf_input(struct kobject *kobj, struct kobj_attribute *attr,
																							 char *buf)
{
	/*
		show() must not use snprintf() when formatting the value to be
		returned to user space. If you can guarantee that an overflow
		will never happen you can use sprintf() otherwise you must use
		scnprintf().*/
	struct stv* state = find_demod(kobj);
	int default_rf_input = state->fe.ops.info.default_rf_input;
	int ret=0;
	ret += sprintf(buf+ret, "%d\n", default_rf_input);
	return ret;
}

static ssize_t stv_demod_set_default_rf_input(struct kobject* kobj, struct kobj_attribute *attr,
																							const char *buf, size_t count) {
	int default_rf_input = 0;
	sscanf(buf,"%d",&default_rf_input);
	struct stv* state = find_demod(kobj);
	state->fe.ops.info.default_rf_input = default_rf_input;
	state_dprintk("Set default_rf_input=%d\n", default_rf_input);
	return count;
}

int stv_demod_make_sysfs(struct stv* state)
{
	int error = 0;
	char name[128];
	sprintf(name, "demod%d", state->nr);
	state->sysfs_kobject = kobject_create_and_add(name, state->chip->sysfs_kobject);
	if(!state->sysfs_kobject)
		return -ENOMEM;

	error = sysfs_create_file(state->sysfs_kobject, &stv_demod_sysfs_attribute.attr);

	if (error) {
		dprintk("failed to create the stv_demod sysfs file\n");
	}

	error = sysfs_create_file(state->sysfs_kobject, &stv_demod_sysfs_default_rf_input_attribute.attr);

	return error;
}
