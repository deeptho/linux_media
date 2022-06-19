# What is this?
This repository is a clone of tbs/linux_media.
It contains patches for the following drivers
* stid135: ts6909x and tbs6903x cards
* stv091x: tbs5927
* tas2101 (incomplete): tbs5990, tbs6904
* si2183 based cards: tbs6504
* m88rs60606 based cards: tbs6902SE and tbs2904SE. On these cards there is not IQ-scan (constellation display)
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

#check out the actual drivers. This uses the default branch which is called deepthought
git clone --depth=1  https://github.com/deeptho/linux_media.git ./media

#checkout a copy of media_build. Standard media_build should work as well
git clone https://github.com/tbsdtv/media_build.git

cd media_build

#it is important to use a media_build version compatible with media
#The following version works on 14.5.2022
git checkout master

make dir DIR=../media
make distclean
./install.sh
```
