#!/bin/bash -x

export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabi-
export DATE=$(date +"%b-%d-%Y")

FAIL=0

rm -rf out
mkdir out > /dev/null 2>&1

make mrproper
make ts7680_defconfig
make -j9 u-boot.sb
if [ $? -eq 0 ]; then
	cp u-boot.sb out/ts7680-"$DATE".sb
else
	let FAIL=FAIL+1
fi

if [ $FAIL != 0 ]; then
	echo "$FAIL BUILDS FAILED.  DO NOT RELEASE"
fi
