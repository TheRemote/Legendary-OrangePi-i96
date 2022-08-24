#!/bin/bash

# Functions:
# compile_uboot
# compile_kernel

compile_uboot()
{
	if [ ! -d $UBOOT_BIN ]; then
		mkdir -p $UBOOT_BIN
	fi

	# Perpar souce code
	if [ ! -d $UBOOT ]; then
		whiptail --title "OrangePi Build System" \
			--msgbox "u-boot doesn't exist, pls perpare u-boot source code." \
			10 50 0
		exit 0
	fi

	cd $UBOOT
	echo -e "\e[1;31m Build U-boot \e[0m"

	case "${PLATFORM}" in
		"OrangePiH3" | "OrangePiH6" | "OrangePiH6_Linux4.9")
			if [ ! -f $UBOOT/u-boot-"${CHIP}".bin -o ! -f $UBOOT/boot0_sdcard_"${CHIP}".bin ]; then
			make "${CHIP}"_config 
			fi
			make -j${CORES} CROSS_COMPILE="${UBOOT_COMPILE}"
			make spl -j${CORES} CROSS_COMPILE="${UBOOT_COMPILE}"
			cd -
			pack
			;;
		"OrangePiH3_mainline" | "OrangePiH6_mainline")
			make orangepi_"${BOARD}"_defconfig
			make -j4 ARCH=arm CROSS_COMPILE="${UBOOT_COMPILE}"

			cp "$UBOOT"/u-boot-sunxi-with-spl.bin "$UBOOT_BIN"/u-boot-sunxi-with-spl.bin-"${BOARD}" -f
			;;
		"OrangePiRK3399")
			./make.sh rk3399
			cp -rf uboot.img $UBOOT_BIN
			cp -rf trust.img $UBOOT_BIN
			cp -rf rk3399_loader_v1.22.119.bin $UBOOT_BIN
			cp -rf idbloader.img $UBOOT_BIN
			;;
		"OrangePiRDA")
			echo -e "\e[1;31m Configure OrangePi 2G-IOT or i96 \e[0m"
		#	make CROSS_COMPILE=${TOOLS} clean
			make CROSS_COMPILE=${TOOLS} rda8810_config
			echo -e "\e[1;31m Compiling Uboot \e[0m"
			make CROSS_COMPILE=${TOOLS}
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
		"OrangePiH3")
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} uImage
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} modules
			cp $LINUX/arch/"${ARCH}"/boot/uImage $BUILD/kernel/uImage_$BOARD
			;;
		"OrangePiH6" | "OrangePiH6_Linux4.9")
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} Image
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} dtbs
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES} modules
			mkimage -A arm -n "${PLATFORM}" -O linux -T kernel -C none -a 0x40080000 -e 0x40080000 \
		                -d $LINUX/arch/"${ARCH}"/boot/Image "${BUILD}"/kernel/uImage_"${BOARD}"
			;;
		"OrangePiH3_mainline" | "OrangePiH6_mainline") 
			# make kernel
			make -C $LINUX ARCH="${ARCH}" CROSS_COMPILE=$TOOLS -j${CORES}

			if [ ! -d $BUILD/dtb ]; then
				mkdir -p $BUILD/dtb
			else
				rm -rf $BUILD/dtb/*
			fi
			# copy dtbs
			echo -e "\e[1;31m Start Copy dtbs \e[0m"

			if [ ${PLATFORM} = "OrangePiH3_mainline" ];then
       				cp $LINUX/arch/"${ARCH}"/boot/dts/sun8i-h3-orangepi*.dtb $BUILD/dtb/
       				cp $LINUX/arch/"${ARCH}"/boot/dts/sun8i-h2-plus-orangepi-*.dtb $BUILD/dtb/
			elif [ ${PLATFORM} = "OrangePiH6_mainline" ];then
       				cp $LINUX/arch/"${ARCH}"/boot/dts/allwinner/sun50i-h6-orangepi-${BOARD}.dtb $BUILD/dtb/
			fi


			if [ ${PLATFORM} = "OrangePiH6_mainline" ];then
				cp $LINUX/arch/"${ARCH}"/boot/Image $BUILD/kernel/Image_$BOARD
			elif [ ${PLATFORM} = "OrangePiH3_mainline" ];then
				cp $LINUX/arch/"${ARCH}"/boot/zImage $BUILD/kernel/zImage_$BOARD
			fi

			cp $LINUX/System.map $BUILD/kernel/System.map-$BOARD
			;;
		"OrangePiRK3399")
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS ${BOARD}_linux_defconfig
			echo -e "\e[1;31m Using ${BOARD}_linux_defconfig\e[0m"
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS -j${CORES} rk3399-orangepi-${BOARD}.img
			make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS -j${CORES} modules
			cp $LINUX/boot.img $BUILD/kernel
			;;
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
