#!/bin/bash -x

export ARCH=arm
#export CROSS_COMPILE=arm-linux-gnueabihf-
export CROSS_COMPILE=/opt/poky/2.2.1/sysroots/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-
export DATE=$(date +"%b-%d-%Y")

FAIL=0

rm -rf out
mkdir out > /dev/null 2>&1

case "$1" in

ts4900)
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
	;;

ts7970)
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
	;;

ts7990)
	make ts7990-q-1g-1000mhz-c_defconfig
	make -j9 u-boot.imx
	if [ $? -eq 0 ]; then
		cp u-boot.imx out/ts7990-q-1g-1000mhz-c-"$DATE".imx
	else
		let FAIL=FAIL+1
	fi

	make ts7990-s-1g-800mhz-i_defconfig
	make -j9 u-boot.imx
	if [ $? -eq 0 ]; then
		cp u-boot.imx out/ts7990-s-1g-800mhz-i-"$DATE".imx
	else
		let FAIL=FAIL+1
	fi
	;;
*)
	echo "Usage: ./build-imx6.sh <model># Model being ts4900, ts7970, ts7990, etc"
	exit 1
esac

if [ $FAIL != 0 ]; then
	echo "$FAIL BUILDS FAILED.  DO NOT RELEASE"
	exit 1
fi

exit 0
