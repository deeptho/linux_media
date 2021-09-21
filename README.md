# What is this?
This repository is a clone of tbs/linux_media.
It contains patches for stid135 and stv091x based cards to support
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
make dir DIR=../media
make distclean
make allyesconfig
make -j8
sudo make install
```
