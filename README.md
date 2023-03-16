# What is this?
This repository is a clone of tbs/linux_media.
It contains patches for the following drivers

* stid135: ts6909x and tbs6903x cards (v1 and v2)
* stv091x: tbs5927 tbs6908 tbs6903
* tas2101: tbs5990, tbs6904
* si2183 based cards: tbs6504
* m88rs6060 based cards: tbs6902SE and tbs904SE. On these cards there is not IQ-scan (constellation display)
to support blindscan, to fix bugs and to make improvements.

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
git checkout master
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

Now load the drivers: either reboot, or try loading the proper module for your card, e.g., tbsecp3
for many cards:

```
sudo modprobe tbsecp3
```

Check /var/log/debug for messages. If there are i2c_xfer error messages, try editing
the file tbsecp3-cards.c. In that file lcate the entry for your card and change i2c_speed
to 9.

If you have this problem then report it. Also report if the solution works,
