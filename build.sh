#!/bin/bash -x

export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
export DATE=$(date +"%b-%d-%Y")

FAIL=0

rm -rf out
mkdir out > /dev/null 2>&1

make ts4900-q-2g-1000mhz-c_defconfig
make -j9 u-boot.imx
if [ $? -eq 0 ]; then
	cp u-boot.imx out/ts4900-q-2g-1000mhz-c-"$DATE".imx
else
	let FAIL=FAIL+1
fi

make ts4900-s-1g-1000mhz-c_defconfig
make -j9 u-boot.imx
if [ $? -eq 0 ]; then
	cp u-boot.imx out/ts4900-s-1g-1000mhz-c-"$DATE".imx
else
	let FAIL=FAIL+1
fi

make ts4900-s-1g-1000mhz-c-2_defconfig
make -j9 u-boot.imx
if [ $? -eq 0 ]; then
	cp u-boot.imx out/ts4900-s-1g-1000mhz-c-2-"$DATE".imx
else
	let FAIL=FAIL+1
fi

make ts4900-s-1g-800mhz-i_defconfig
make -j9 u-boot.imx
if [ $? -eq 0 ]; then
	cp u-boot.imx out/ts4900-s-1g-800mhz-i-"$DATE".imx
else
	let FAIL=FAIL+1
fi

make ts7970-q-2g-1000mhz-c_defconfig
make -j9 u-boot.imx
if [ $? -eq 0 ]; then
	cp u-boot.imx out/ts7970-q-2g-1000mhz-c-"$DATE".imx
else
	let FAIL=FAIL+1
fi

make ts7970-s-1g-1000mhz-c_defconfig
make -j9 u-boot.imx
if [ $? -eq 0 ]; then
	cp u-boot.imx out/ts7970-s-1g-1000mhz-c-"$DATE".imx
else
	let FAIL=FAIL+1
fi

make ts7970-s-1g-800mhz-i_defconfig
make -j9 u-boot.imx
if [ $? -eq 0 ]; then
	cp u-boot.imx out/ts7970-s-1g-800mhz-i-"$DATE".imx
else
	let FAIL=FAIL+1
fi

make ts7970-s-512m-1000mhz-c_defconfig
make -j9 u-boot.imx
if [ $? -eq 0 ]; then
	cp u-boot.imx out/ts7970-s-512m-1000mhz-c-"$DATE".imx
else
	let FAIL=FAIL+1
fi

if [ $FAIL != 0 ]; then
	echo "$FAIL BUILDS FAILED.  DO NOT RELEASE"
fi
