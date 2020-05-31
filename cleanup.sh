#!/bin/bash
echo "Cleaning work dir"
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=~/Kernel/arm-eabi-4.8/bin/arm-eabi-
make clean && make mrproper
echo "Removing output folder"
rm -rf ./output