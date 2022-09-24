#!/bin/bash

# Functions:
# compile_uboot
# compile_kernel

compile_uboot()
{
	if [ ! -d $UBOOT_BIN ]; then
		mkdir -p $UBOOT_BIN
	fi

	# Perpare souce code
	if [ ! -d $UBOOT ]; then
		whiptail --title "OrangePi Build System" \
			--msgbox "u-boot doesn't exist, pls perpare u-boot source code." \
			10 50 0
		exit 0
	fi

	cd $UBOOT
	echo -e "\e[1;31m Build U-boot \e[0m"

	case "${PLATFORM}" in
		"OrangePiRDA")
			echo -e "\e[1;31m Configure OrangePi 2G-IOT or i96 \e[0m"
		#	make CROSS_COMPILE=${TOOLS} clean
			make CROSS_COMPILE=${TOOLS} rda8810_config
			echo -e "\e[1;31m Compiling Uboot \e[0m"
			make CROSS_COMPILE=${TOOLS} -j${CORES}
			cp -rf ${UBOOT}/u-boot.rda ${UBOOT_BIN}
			;;
		*)
	        	echo -e "\e[1;31m Pls select correct platform \e[0m"
	        	exit 0
			;;
	esac

	echo -e "\e[1;31m Complete U-boot compile.... \e[0m"

	#whiptail --title "OrangePi Build System" \
	#	--msgbox "Build uboot finish. The output path: $BUILD" 10 60 0
}

compile_kernel()
{
	if [ ! -d $BUILD ]; then
		mkdir -p $BUILD
	fi

	if [ ! -d $BUILD/kernel ]; then
		mkdir -p $BUILD/kernel
	fi

	echo -e "\e[1;31m Start compiling the kernel ...\e[0m"

	case "${PLATFORM}" in 
		"OrangePiRDA")
			set -x
			echo -e "\e[1;31m Using ${BOARD}_linux_defconfig\e[0m"
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS ${BOARD}_linux_defconfig

			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS -j${CORES} zImage
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS -j${CORES} modules
			# Install zImage and modules 
			cp -rfa ${LINUX}/arch/arm/boot/zImage ${BUILD}/kernel
			;;
		*)
	        	echo -e "\e[1;31m Pls select correct platform \e[0m"
			exit 0
	esac

	echo -e "\e[1;31m Complete kernel compilation ...\e[0m"
}

compile_module(){
	
	if [ ! -d $BUILD/lib ]; then
	        mkdir -p $BUILD/lib
	else
	        rm -rf $BUILD/lib/*
	fi

	# install module
	echo -e "\e[1;31m Start installing kernel modules ... \e[0m"
	make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} modules
	make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} modules_install INSTALL_MOD_PATH=$BUILD
	echo -e "\e[1;31m Complete kernel module installation ... \e[0m"

	#whiptail --title "OrangePi Build System" --msgbox \
	#	"Build Kernel OK. The path of output file: ${BUILD}" 10 80 0
}
