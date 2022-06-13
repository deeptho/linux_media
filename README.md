# What is this?
This repository is a clone of tbs/linux_media.
It contains patches for the following drivers
* stid135: ts6909x and tbs6903x cards
* stv091x: tbs5927
* tas2101 (incomplete): tbs5990, tbs6904
* si2183 based cards: tbs6504
* m88ds3103: pctv 461e v1(B6H9)/Hauppauge wintv-nova-s2 v1.4(B9H9) 

to support blindscan, to fix bugs and to make improvements.


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
./install.h
```
