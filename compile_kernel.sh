#!/bin/bash
echo "Creating output dir"
mkdir output
echo "Exporting variables"
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=~/Kernel/arm-eabi-4.8/bin/arm-eabi-
export KBUILD_OUTPUT=output

# Starting Timer
SECONDS=0

echo "Starting to compile"
make mt8127_defconfig
make zImage -j12 2>&1 | tee output/build.log

if (( $SECONDS > 3600 )) ; then
    let "hr=SECONDS/3600"
    let "min=(SECONDS%3600)/60"
    let "sec=(SECONDS%3600)%60"
    echo "Make completed in $hr hour(s), $min minute(s) and $sec second(s)" 
elif (( $SECONDS > 60 )) ; then
    let "min=(SECONDS%3600)/60"
    let "sec=(SECONDS%3600)%60"
    echo "Make completed in $min minute(s) and $sec second(s)"
else
    echo "Make completed in $SECONDS seconds"
fi