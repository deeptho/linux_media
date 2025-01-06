# What is this?
This repository is a clone of tbs/linux_media.
It contains patches for the following drivers

* stid135: ts6909x and tbs6903x cards (v1 and v2)
* stv091x: tbs5927 tbs6908 tbs6903 tbs6983 tbs6522 tbs6983 tbs6903 tbs6909
* tas2101: tbs5990, tbs6904
* si2183 based cards: tbs6504, tbs5580, tbs6916
* m88rs6060 based cards: tbs6902SE and tbs904SE. On these cards there is not IQ-scan (constellation display)
to support blindscan, to fix bugs and to make improvements.

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
git clone https://github.com/deeptho/neumo_media_build
```

Make sure software for kernel compilation is installed.
For instance on fedora:

```
sudo dnf install -y patchutils
sudo dnf install -y ccache
sudo dnf install -y kernel-devel-`uname -r`
sudo dnf install -y perl-File-Copy #not needed?
sudo dnf install -y perl
sudo dnf install -y perl-Proc-ProcessTable

cd neumo_media_build
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

# Usage
These drivers were designed for use with neumoDVB to support advanced features provided by
some cards:
* Spectrum acquisition.
* Constellation plots.
* Blind scanning.
* Supporting cards with RF mixes, which allow multiple demods to connect to the same tuner
* Stable identification of cards even when they are inserted in different orders (USB device)
  or placed in different slots.
* Providing advanced information for the stid135 based cards through sysfs and allowing
  certain parameters to be set through sysfs for those cards.

To make use or optimal use of these features, application programs should adhere to a new
extended ``neumo-DVB'' interface, but the drivers were designed to be also backward compatable

## Using with neumoDVB

neumoDVB detects the presence of these drivers and then will exploit the advanced features;
if the drivers are not loaded if falls back to the DVB-apiV5 interface

## Using with existing programs developed for DVB-apiV5

In principle no changes are needed, and even without changes for stid135 based cards,
through the sysfs interface it is possible to obtain more information about
the cards and what they are doing. Also it is possible to slightly enhance using existing
programs, such as tvheadend.

* Through /sys/module/stid135/... it is possible to figure out which adapter belongs to which
  card. There is no guarantee that the directory layout and content will have this specific
  format in future. So do not rely on it.
* Each card has one *temperature* property per stid135 chip, which contains the current temperature
  in Celcius.
* Each card has a *blindscan_always* property.
  By setting '/sys/module/stid135/card0/blindscan_always' to 1, the stid135 driver will blindscan
  muxes on that card (card0). For instance if TvHeadend thinks that a mux is QPSK and has a symbolrate
  of 20kS/s, but in reality it has PSK8 modulation and a symbolrate of 30kS/s, tuning will fail without this
  setting, but will succeed with it. The downside is that the web interface of TvHeadend will
  still show the erroneous parameters. The upside is that scanning becomes easier.

  Note that the proper way to set a sysfs parameter is (as an example)

  ``` echo 1 | sudo tee /sys/module/stid135/card0/blindscan_always```

* Each adapter (or rather: demodulator) has a *default_rf_in* property. The value of this
  is a number indicating the physical input to which the adapter will be connected by default,
  if applications do not use the neumo-DVB api. For example, on tbs6909X,
  the default value for /sys/module/stid135/card0/chip0/demodX/default_rf_in
  will be X modulo 8, which means that two demodulators are connected to each of the four
  RF inputs.

  In case you only have one cable connected to RF input 0, you can set all of the
  default_rf_in values to 0 and then configure TvHeadend to treat adapters 1...7 as slave adapters
  connected to master adapter 0. This then will allow simulatenous viewing or recording of
  channels spread over 8 different muxes in the same satellite band (e.g., horizontal Ku-low).

  Note that TvHeadend will probably have difficulties coping with that much data, so it is best
  to stick with a smaller number of slave tuners.

## The neumoDVB kernel api

This API has been kept mostly compatible with the existing DVB-V5 api, both internally in
the kernel and externally towards user applications. However, the kernel-side
API is not binary compatile (yet) with DVB-apiV5 and requires recompilation of **all** DVB modules that
the user wants to use, even those not supporting neumoDVB.

User space applications should proceed as follows

* First check for the presence of */sys/module/dvb_core/info/version*. If this entry exists,
  it means that the neumoDVB drivers have been loaded, otherwise the application should fall back to
  DVB-apiV5, i.e., not use any additional features provided by nuemoDVB. The version number in that
  sysfs file can also be used to distinghuish between different versions of the drivers. Note that the
  neumoDVB api is not yet finalized.

* Using the *FE_GET_EXTENDED_INFO* ioctl, get information about the installed cards and adapters,
  specificaly their names (for use in GUI and in log files) and their *MAC address*. The latter
  is a unique id, which can be used to associate configuration information. For instance, the adapter
  number of a specific adapter may differ from one boot to the next when a card has been removed,
  or a new one has been inserted. However, the MAC address will always remain the same. In case
  the card specific drivers do not provide a MAC address, neumoDVB will attemp to construct one
  that is unique based on in which PCIe slot it is inserted. If the card is later in an other slot
  the MAC address will change. For USB devices there are even fewer guarantees, and the card may appear
  at a new MAC address after removing and re-attaching it.

* When tuning to a specific mux, first call the *FE_SET_RF_INPUT* ioctl. This is used to connect
  a demodulator to a specific RF_INPUT, but also to synchronize secondary device (LNBs and switches)
  configuration when multiple demodulators connect to the same tuner and thus to the same LNB and switches.

  Being able to select a specific RF_INPUTs means that card operation becomes much more flexible.
  For instance, it is possible to connect more than 2 demodulators to the same input cable temporarily
  if the card has an internal switch. In case each input is connected to a different LNB for a different satellite,
  it is possible to receive up to 8 different muxes on TBS6909V2 or up to 16 muxes on TBS6916 on the same satellite.
  With the standard DVB-v5 api drivers only 2 would be possible, because of the choice to pair 2 tuners
  to each RF_INPUT in a fixed manner.

  Another problem not handled by the standard drivers is that a slave adapter should wait with tuning
  until it is certain that secondary equipment has been configured: switches need some time to power up,
  some types of LNBs (e.g, unicable) also need some time after power up before they respond to commands,
  and positioners need some time to rotate the dish. The neumo API leaves it to applications to work
  out the details, as this often needs configuration constants, or perhaps even custom software. Instead
  it expectes that exactly one of the connecting threads will performa all needed actions (sending diseqc
  commands, waiting appropriate amounts of time...) and will then inform the drivers when all of this is
  done. Until then, all other frontends trying to connect to the same rf_in through *FE_SET_RF_INPUT* ioctls
  will see these ioctls fail with a return value of *FE_RESERVATION_RETRY*. Those threads should wait a while
  and then retry the ioctl.

  The reason why this is needed is that many programs are multi hreaded. If the thread for the master tuner
  is delayed for some reason, it may happen that the master tuner has not yet finished sending DiSeqC commands
  when the slave tuner starts tuning and then slave tuning will fail. Of course it would be possible to prevent
  this at the application level, but neumoDVB handles it in the drivers.

  Concretely, when calling FE_SET_RF_INPUT, the application can specificy whether the demodulator will
  become master or slave. Typically an application will specify 'master' for the first tune, and 'slave'
  for subsequent tune's using the same RF_INPUT. An application can also specify 'master or slave'. In this
  case the drivers will make the adapter that makes the first ioctl the master and the other ones slaves.
  In all cases, when the drivers receive the ioctl, they will enforce proper execution of master requests
  before related slave requests can start.

  The ioctl FE_SET_RF_INPUT caller should specify

    ** rf_input: the desired tuner and cable to connect to

    ** mode: whether the tuner wants to become master (and agrees to configure LNB and switches), slave
     (agrees to not configure LNB and switches), or both.

    **unicable_mode: whether or not the secondary system attached to the LNB operates in unicable mode.
    In uncable mode, DiSeqC commands need to be sent even by slave tuners, to select specific user bands.
    This creates new opportunities for races between multiple frontends trying to program user bands
    in parallel.

    Also, the output voltage needs to be temporarily raised to 18 volt when unicable commands are sent,
    and must be 12 volt in all other cases. User programs activate new user bands in three steps:
    1) raising voltage to 18 volt; 2) sending unicable DiSeqC unicable command; 3) lowering voltage
    to 12 volt. This three step process is inherited from dvbapiV5, but creates additional opportunities
    for racing. For instance, internal locking prevents step 2 from overlapping with step 2 calls made
    by other frontends, but not prevent other threads from executing step 3 after the current thread
    has executed step 1. This wll then cause user band selection to fail

    Setting unicable_mode = 1 allows slave threads to raise voltages and send unicable DiSeqC commands,
    which it would otherwise prohibit. When unicable_mode active, also setting voltage to 18 VOLT
    will soft-fail with the return value FE_UNICABLE_DISEQC_RETRY when the voltage is already 18 volt,
    which implies that some other frontend is currently sending unicable DiSeqC commands.

    When the FE_SET_VOLTAGE ioctl returns  FE_UNICABLE_DISEQC_RETRY, the thread should retry the ioctl.
    When it succeeds instead, then the thread calling the ioctl will be the only one allowed to
    send unicable commands.

    ** owner: a unique identifier for the calling application. Applications can only use resources (tuners,
     demodulators) if they are not in use by another application. Typically this should be the process id.

    ** config_id: a number which is incremented each time the application will reconfigure an RF_INPUT,
     which means: select a different LNB, or a different satellite band.

     Whenever the driver notices an update in config_id, it will wait for all demodulators to release
     resources reserved with an older config_id value before allowing FE_SET_RF_INPUT ioctl's (including
     the one using the new config_id for the first time) to succeed. This loosens the synchronisation
     requirements on the application. For instance the application can call FE_SET_RF_INPUT for a master
     and slave demodulator in parallel on two threads. If for some reason the master ioctl call is delayed
     until after the slave ioctl call, the driver will do the right thing: fail one or both of the calls
     and inform the calling threads (through the ioctl return value) that this failure is only temporary
     and that they should retry the call after waiting for a brief period.

  The ioctl's return value indicates

  ** a result: master or slave. If the result is 'master', the adapter should proceed with setting
     voltages, tones, sending diseqc commands, and then finally perform a tune. If the result is
     'slave', then the caller can be certain that secondary equipment is now ready and the caller
     can start tuning its adapter.

  ** or permanent failure (e.g., requesting a non-existing RF_INPUT)

  ** or temporary failure: it is not yet possible to select the RF_INPUT, either because a master demodulator
     has not yet finished configuring secondary equipment (which the drivers can tell, see below),
     or because some other demodulators still need to release the tuner but have not done so. The calling
     thread can simply retry after sleeping e.g., 10 milliseconds, or the application can provide a
     locking mechanism to prevent this from happening (warning: this tends
     to be slower)

  If the calling thread receives a 'slave' result, it should proceed directly with tuning. If it
  receives a 'master result', it should make FE_SET_VOLTAGE, FE_SET_TONE and FE_DISEQC_SEND_MASTER_CMD
  commands as needed, and also respect any delays needed by the secondary equipment to
  correctly power up. At this stage the calling thread should tune the adapter, using FE_SET_PROPERTY,
  and include a DTV_SET_SEC_CONFIGURED property. This informs the drivers that the secondary  equipment
  is ready for use. From this stage on FE_SET_RF_INPUT ioctls for the same RF_INPUT will succeed, provided
  they use the current config_id value.

* The remainder of the tuning process is similar as with the standard DVB-v5 api, except that additional
  tuning properties have been added to indicate blind scanning and that additional properties are returned
  to indicate discovered modulation parameters.

# Changes in release-1.5
* Added a new demux interface allowing internal demuxing of bbrames; Extended stid135 driver to make
  use of this demux interface.

  As a result it is now possible to demux several or all streams from a multistream transponder using a
  single demodulutor (but no user programs currently support this). This involves using some new ioctls
  on /dev/dvb/adapterX/demuxY. In this is case there is no need to specify the desired stream_id (ISI) when tuning,
  but instead ask the frontend to output bbframes and pass the desired stream to the new ioctl
  DMX_SET_BBFRAMES_STREAM on /dev/dvb/adapterX/demuxY. By Opening the demux multiple times, additional streams
  can de bemuxed simultaenously.

  For backward compatibility it is still allowed to pass a stream_id to the frontend. In this case the
  frontend will not embedded multistreams in bbframes and demuxing proceeds as before.

* Added a driver option bbframes_auto to support legacy applications like tvheadend. When activated,
  this option will instruct the stid135 driver to encapsulate all multistreams into an embedded bbframes
  stream. The demuxer will automatically pick one of the streams and demux that for the legacy application

This should be set by adding the line
  ``options stid135 bbframes_auto=1''
  in  /etc/modprobe.d/stid135.conf and rebooting, or by runnning the following command as root:
  `` echo 1 > /sys/module/stid135/parameters/bbframes_auto'' (no need to reboot).

* One use of bbframes_auto is to provide a workaround for the non-working multistream 12606V@5.0W Streams a4 and 5 cannot
  be decoded properly due to what is probably a hardware bug. By asking the chip to output bbframes, the buggy bbframe
  decoding is skipped by the chip and instead done in the new demux software driver. As a result, both streams then work
  in user programs such as tvheadend.

# Changes in release-1.4.1
* Fix deadlock introduced by e8c6a729c905f7464bd7

# Changes in release-1.4
* Revert "Experimental workaround for 12606V@5.0W to make stream 4 work" as it caused problems on some multistreams
* Bug: tbs6916: not locking one of the chips when releasing rf_in
* Incorporated latest changes from TBS

# Changes in release-1.3
* Document neumo api
* Updated installation instructions
* Better handling of cards that do not support FE_SET_RF_INPUT
* Add FE_DISEQC_SEND_LONG_MASTER_CMD to support unicable programming
* Integrated changes from tbsdtv
* Changes for kernel 6.10
* Fix compilation warnings
* stid135: Experimental workaround for 12606V@5.0W to make stream 4 work
* stid135: Skip dummy frames when detecting modulation, so that correct modulation is reported
* stid135: Added blindscan_alwasy configuration via sysfs
* stid135: Fix register reading.
* stid135: Implemented unicable mode
* stid135: Erroneous mutex lock detection
* stid135: Bug: stdi35_select_rf_in_ called without locking
* stud135: Detect deadlock condition and work around it.
* m88rs6060: skip dummy frames when detecting modulation, so that correct modulation is reported
* stv091x: implemented recv_slave_reply (untested)
* ms88rs6060: fix driver no longer working.

# Changes in release-1.2
* Added sysfs entries in /sys/modules/stid135/
* Allow setting the default rf input for each demod, for use with tvheadend. This allows more than one slave
  tuner to use the same rf input connector on the card.
* Incorporated changes from the official tbs drivers up to July 11 2024.
* Improved determination of card mac addresses and generating fake mac addresses should the card not have one set.
* Support for the tbs6916 card.
* Bug: stid135 based cards sends out 21kHz tone and diseqc base frequency instead of 22kHz and this causes some
  diseqc switcches to sometimes ignore commands.
* Provide additional ioctls to support "syncrhonized" calls, where one adapter can wait on another adapter
  to finish setting voltages, tones and sending diseqc commands
* Protection against drivers which blindly assume name to be of size 128.
* Bug: fe_read_status should not return -1 on timeout.
* stv091x: fix isi scan.
* Bug: tuning loop runs to early when diseqc is sent but no tuning info is present yet.

# Changes in release-1.1.1

## Kernel support
* Update for linux 6.2 - 6.6 kernels
* compiles also on ubuntu 23.0

## Changes

* Integrated latest changes from tbs linux_media
* Avoid out of bounds array indexing
* Improved voltage setting and related internal state management
* si2157 double free bug
* si2158 integrates some tns code


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
