# What is this?
This repository is a clone of tbs/linux_media.
It contains patches for the following drivers

* stid135: ts6909x and tbs6903x cards (v1 and v2)
* stv091x: tbs5927 tbs6908 tbs6903 tbs6983 tbs6522
* tas2101: tbs5990, tbs6904
* si2183 based cards: tbs6504
* m88rs6060 based cards: tbs6902SE and tbs904SE. On these cards there is not IQ-scan (constellation display)
to support blindscan, to fix bugs and to make improvements.

# Changes in release-1.0.0

##Kernel support
* Update for linux 6 kernel
* compiles 6.1

## Various bugs and improvements
* Improved logging.
* Avoid null pointer dereference when short_name not sent for 2nd frontend
* tbsecp3: set default mode to 1
* Do not send diseqc in mode=0; remove dead code
* Experimental fix for cards without short name; short name for tbs6908
* Bug: incorrect tone and voltage returned when tone and voltage are set via ioctl
* Bug: bbframe mode not reset in some cases
* Set rf_in=0 for non neumodvb cards
* Different way to initialize rf_inputs.
* rename tuner_active to rf_in_selected.
* Improved debug logging: add demod number; use numbering starting at 0
* Enable bbframes for GSE
* ts_nosync (code not yet active
* wideband support (untested)
* matype not set in some cases.
* select_min_isi sometimes fails, resulting in no tune
* Bug: after pls_seacrch fails, restore default pls
* Redefine FE_HAS_TIMING_LOCK to be different from FE_OUT_OF_RESOURCES, which confuses dvblast
* cx231xx correct MAC extraction after power cycle
* BUG: incorrect reporting of FE_OUT_OF_RESOURCES when lock fails
* Temporary fix to avoid crash on module unload
* Bug: when auto-selecting a stream_id, ensure that selected stream_id is returned to driver, rather than -1
* Introduce state_sleep; release mutex during sleep, to avoid delays in parallel operation of multiple demods
* Introduce base_lock, base_unlock, base_trylock
* Increase number of events in event queue
* Bug: slow closing of frontend because of needless locking
* Disallow turning off voltage when tuner is still in use
* Defaults for rf_inputs property; More stable faking of missing mac_address
* Possible module unload bug for tbs6590
* Incorrect processing of DVB_STOP ioctl. Fix includes workaround for bad user space code containing the same bug
* Bug: interrupting fft spectrum sometimes causes crash
* Bug: need to lock mutex during spectrum scan sweep.
* Adapter name retrieval; add short names for adapters.
* Add FE_SET_RF_INPUT ioctl; add rf_inputs fields to extended frontend info;
* Add card_short_name
* Bug: memory access beyond end of array causing kernel crash
* Do not apply center frequency shift for low symbol rates. Improves locking of low symbol rate muxes?
* Bug: voltage remains on after frontend is released
* Bug: crash on module onload (chip_proc_close)
* Always use "local" value for faked mac addresses.
* distinghuish between mac address of card and adapter
* Suppress some compiler warnings
* Expose default rf_in for each dvb adapter

## stv091x based cards
* Fixes for tbs6983 (UNTESTED; may break other stv091x cards)
* Bug: tone not set on high band with stv091x
* Bug: stv091x needs mutex protection when called from dvb_frontend;
* tbs5927 - set correct frequency limits
* tbs5927: add card_address
* tbs5927 add "usb" to bus address
* set_voltage call accidentally removed from tbs5927
* Proper mac address and card name for tbs5927

## stid135 based cards
* tbs6903x: Reduce overclocking to avoid i2c nack errors at startup
* tbs6903x_V2 has only 2 rf inputs.
* tbs6903x: allow both tuners to connect to both rf inputs
* Allow exposing registers of multiple stid135 cards in /proc
* Updated spectral scan algorithm (algo 5)
* debug logging
* Bug: force frame and continous mode not reset
* Add more error detection
* Add dummy plf detection
* simplify fe_stid135_get_signal_info
* stid135 Bug: Add code which ran in get_frontend into read_status because get_frontend is no longer called
* Remove stid135-fe.h; make some functions static
* Improved debug logging: add demod number; use numbering starting at 0
* stid135: improved lock status reporting
* stid135: deadlock in spectrum_scan
* In stid135_fft, assume all functions are called with base_lock held.
* stid135: correctly handle low rolloff factors indicated by alternating matype
* stid135: disable get_frontend
* Handle out of llr condition
* stid135: allow writing registers
* Overclock i2c bus on stid135 based cards.
* Rename spectrum scan related enum; allow rf_input selection on stid135;
* stid135: report that only 4 out of 8 tuners support fft scan.
* Bug: stid135 22kHz still on when device not in use
* Bug: incorrect docs for stid135-fe module parameter
* Bug: accidentally disabled fe_stid135_init_fft
* Re-add tbs6916 (16 tuner optical card)

## em28xx based cards
* em28xx crash on usb disconnect.
* em28xx crash on module unload

## m88rs6060 based cards
* m88rs6060: remove uneeeded reads
* tbs6904se: increase i2c speed (experimental)
* m88rs6060: faster spectrum scan (psyborg55)
* shortname for tbs6522

## tas2101 based cards
* tas2101: pause after activating demod, to return correct SNR
* tas2101: set proper lock flags
* tas2101: set timing lock status
* tas2101: increase constellation size
* tbs5990: set proper card_mac_address and rf_input.
* tbs5990: add bus address
* tbs5990: incorrect mac address read.

## si283 based cards
* si2183 patch
* si2183: report timing lock to avoid problems in user apps.
* Bug: si2183 sets incorrect frequency limits
* Bug: si2183 sets incorrect frequency limits
* tbs6504: ensure all frontends on the same adapter have same faked macaddress
* Workaround for tbs6504 not detecting stream_type correctly



# Changes in release-0.9.0

* Blindscan and spectrum upport for montage m88rs60606 based cards: tbs6902SE and tbs2904SE.
* Support m883s6060 module unloading
* stid135: BER is now reported properly othrough the PRE instead of POST error counters
* Improved correction of discontinuities in spectrum due to small errors in RF level
* stid135: detected multi-stream IDs are now accumalated internally
* Added DTV_BIT_RATE readout via dvbapi
* stid135: improper estimation of required  llr rate, causing rai multistream on 5.0W to not tune sometimes
  (green blocks in picture)
* stid135: changed default spectral resolution: slightly slower scan, but more narrow-band muxes found
* Add support for timing lock flag
* stv091x: return more correct data about currently tuned mux (instead of returning what user requested)


# Known problems
* si2183 based cards: blindscan cannot discover stream_ids in multistreams, unless it is given
 one of them when starting the blindscan. In that case, it will find the other ones


User space tools for using blindscan can be found at
https://github.com/deeptho/blindscan



# Installation

First install the required compilers, git ...
You may also need libproc-processtable-perl (e.g., on ubuntu)

```
mkdir ~/blindscan_kernel
cd  ~/blindscan_kernel
```

Check out the actual drivers. This uses the default branch which is called deepthought

```
git clone --depth=1  https://github.com/deeptho/linux_media.git ./media
```

Then  check out a copy of DeepThought's media_build (try tbs media_build if it does not work)

```
git clone https://github.com/deeptho/media_build
```

Make sure software for kernel compilation is installed.
For instance on fedora, with MYKERNEL the exact kernel version you are running:

```
sudo dnf install -y patchutils
sudo dnf install -y ccache
sudo dnf install -y kernel-devel-MYKERNEL
sudo dnf install -y perl-File-Copy #not needed?
sudo dnf install -y perl
sudo dnf install -y perl-Proc-ProcessTable

cd media_build
git checkout deepthought
git reset --hard
make dir DIR=../media
make distclean
./install.sh
```

Last but not least, install rsyslog so that kernel debug messages are stored in the file system
in /var/log/debug:

```
sudo dnf install -y rsyslog
sudo vi /etc/rsyslog.conf # add "kern.debug /var/log/debug" line
sudo systemctl enable rsyslog
sudo systemctl start rsyslog #to have log messages in /var/log/debug
```


Now install the firmware (if needed):

```
wget http://www.tbsdtv.com/download/document/linux/tbs-tuner-firmwares_v1.0.tar.bz2
sudo tar jxvf tbs-tuner-firmwares_v1.0.tar.bz2 -C /lib/firmware/
```

If you cannot find the 6909 firmware:

```
wget http://www.tbsdtv.com/download/document/linux/dvb-fe-mxl5xx.fw
sudo cp dvb-fe-mxl5xx.fw /lib/firmware/
```


Now load the drivers: either reboot, or try loading the proper module for your card, e.g., tbsecp3
for many cards:

```
sudo modprobe tbsecp3
```

Check /var/log/debug for messages. If there are i2c_xfer error messages, try editing
the file tbsecp3-cards.c. In that file lcate the entry for your card and change i2c_speed
to 9.

If you have this problem then report it. Also report if the solution works,
