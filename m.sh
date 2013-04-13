#!/bin/sh
export ANDROID_PATH=`pwd`
export PATH=$ANDROID_PATH/../prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$PATH
export CROSS_COMPILE=arm-eabi-                                                                                               
export ARCH=arm
export PATH=$ANDROID_PATH/../u-boot/tools:$PATH
#make distclean && \
#make distclean
make panther_android_defconfig && \
make uImage -j4 2>&1 |tee kernel_make.out
#make modules -j4 2>&1 |tee kernel_module.out
cp arch/arm/boot/uImage ../prebuilt_images/Boot_Images
