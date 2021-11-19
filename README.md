# What is this?
This repository is a clone of tbs/linux_media.
It contains patches for the following drivers
* stid135: ts6909x and tbs6903x cards
* stv091x: tbs5927
* tas2101 (incomplete): tbs5990, tbs6904
* si2183 based cards: tbs6504

based cards to support
blindscan, to fix bugs and to make improvements.

User space tools for using blindscan can be found at
https://github.com/deeptho/blindscan



# Installation
```
mkdir ~/blindscan_kernel
cd  ~/blindscan_kernel

#check out the actual drivers. This uses the default branch which is called deepthought
git clone --depth=1  https://github.com/deeptho/linux_media.git ./media

#checkout a copy of media_build. Standard media_build should work as well
git clone https://github.com/tbsdtv/media_build.git

cd media_build

#it is important to use a media_build version compatible with media
#The following version works on 28.9.2021
git checkout ffcdfed96c45c8e80ffd6a154c16d64c81416010

make dir DIR=../media
make distclean
make allyesconfig
make -j8
sudo make install
```
