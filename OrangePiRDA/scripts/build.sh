#!/bin/bash
# Compression - sudo tar cf Legendary_OrangePi_i96_debian_bullseye_server.img.xz --use-compress-program='xz -T8 -v -9' Legendary_OrangePi_i96_debian_bullseye_server.img
set -e

ROOT=$(pwd)
UBOOT="${ROOT}/uboot"
BUILD="${ROOT}/output"
LINUX="${ROOT}/kernel"
EXTER="${ROOT}/external"
SCRIPTS="${ROOT}/scripts"
DEST="${BUILD}/rootfs"
UBOOT_BIN="$BUILD/uboot"
PACK_OUT="${BUILD}/pack/"

OS=""
BT=""
CHIP=""
ARCH=""
DISTRO=""
ROOTFS=""
BOOT_PATH=""
UBOOT_PATH=""
ROOTFS_PATH=""
BUILD_KERNEL=""
BUILD_MODULE=""

SOURCES="OFCL"
METHOD="download"
KERNEL_NAME="linux"
UNTAR="bsdtar -xpf"
CORES=$(nproc --ignore=1)
PLATFORM="$(basename $(pwd))"

if [[ "${EUID}" == 0 ]]; then
	:
else
	echo " "
	echo -e "\e[1;31m This script requires root privileges, trying to use sudo \e[0m"
	echo " "
	sudo "${ROOT}/build.sh"
	exit $?
fi

source "${SCRIPTS}"/lib/general.sh
source "${SCRIPTS}"/lib/pack.sh
source "${SCRIPTS}"/lib/compilation.sh
source "${SCRIPTS}"/lib/distributions.sh
source "${SCRIPTS}"/lib/build_image.sh
source "${SCRIPTS}"/lib/platform/rk3399.sh

if [ ! -f $BUILD/.prepare_host ]; then
	prepare_host
	touch $BUILD/.prepare_host
fi

MENUSTR="Welcome to Orange Pi Build System. Pls choose Platform."
#################################################################
case "${PLATFORM}" in
"OrangePiH3" | "OrangePiH3_mainline")

	OPTION=$(whiptail --title "Orange Pi Build System" \
		--menu "${MENUSTR}" 20 80 10 --cancel-button Exit --ok-button Select \
		"0" "OrangePi PC Plus" \
		"1" "OrangePi PC" \
		"2" "OrangePi Plus2E" \
		"3" "OrangePi Lite" \
		"4" "OrangePi One" \
		"5" "OrangePi 2" \
		"6" "OrangePi ZeroPlus2 H3" \
		"7" "OrangePi Plus" \
		"8" "OrangePi Zero" \
		"9" "OrangePi R1" \
		3>&1 1>&2 2>&3)

	case "${OPTION}" in
	"0") BOARD="pc-plus" ;;
	"1") BOARD="pc" ;;
	"2") BOARD="plus2e" ;;
	"3") BOARD="lite" ;;
	"4") BOARD="one" ;;
	"5") BOARD="2" ;;
	"6") BOARD="zero_plus2_h3" ;;
	"7") BOARD="plus" ;;
	"8") BOARD="zero" ;;
	"9") BOARD="r1" ;;
	"*")
		echo -e "\e[1;31m Pls select correct board \e[0m"
		exit 0
		;;
	esac

	if [ "${PLATFORM}" = "OrangePiH3" ]; then
		TOOLS=$ROOT/toolchain/gcc-linaro-1.13.1-2012.02-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-
		UBOOT_COMPILE="${TOOLS}"
		KERNEL_NAME="linux3.4.113"
	elif [ "${PLATFORM}" = "OrangePiH3_mainline" ]; then
		TOOLS=$ROOT/toolchain/gcc-linaro-7.2.1-2017.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
		UBOOT_COMPILE="${TOOLS}"
		KERNEL_NAME="linux5.3.5"
	fi

	ARCH="arm"
	CHIP="sun8iw7p1"
	CHIP_BOARD="dolphin-p1"
	;;
"OrangePiH6" | "OrangePiH6_Linux4.9" | "OrangePiH6_mainline")

	OPTION=$(whiptail --title "Orange Pi Build System" \
		--menu "$MENUSTR" 15 60 5 --cancel-button Exit --ok-button Select \
		"0" "OrangePi 3" \
		"1" "OrangePi Lite2" \
		"2" "OrangePi OnePlus" \
		"3" "OrangePi Zero2" \
		3>&1 1>&2 2>&3)

	case "${OPTION}" in
	"0") BOARD="3" ;;
	"1") BOARD="lite2" ;;
	"2") BOARD="oneplus" ;;
	"3") BOARD="zero2" ;;
	"*")
		echo -e "\e[1;31m Pls select correct board \e[0m"
		exit 0
		;;
	esac

	ARCH="arm64"
	CHIP="sun50iw6p1"
	CHIP_BOARD="petrel-p1"
	CHIP_FILE="${EXTER}"/chips/"${CHIP}"
	TOOLS=$ROOT/toolchain/gcc-linaro-4.9-2015.01-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
	UBOOT_COMPILE=$ROOT/toolchain/gcc-linaro-4.9-2015.01-x86_64_aarch64-linux-gnu/gcc-linaro/bin/arm-linux-gnueabi-

	if [ "${PLATFORM}" = "OrangePiH6" ]; then
		KERNEL_NAME="linux3.10"
	elif [ "${PLATFORM}" = "OrangePiH6_Linux4.9" ]; then
		KERNEL_NAME="linux4.9.118"
	elif [ "${PLATFORM}" = "OrangePiH6_mainline" ]; then
		KERNEL_NAME="linux5.3.5"
		TOOLS=$ROOT/toolchain/gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
		UBOOT_COMPILE="${TOOLS}"
	fi
	;;
"OrangePiRK3399")
	OPTION=$(whiptail --title "Orange Pi Build System" \
		--menu "$MENUSTR" 15 60 5 --cancel-button Exit --ok-button Select \
		"0" "OrangePi 4" \
		"1" "OrangePi rk3399" \
		3>&1 1>&2 2>&3)

	case "${OPTION}" in
	"0") BOARD="4" ;;
	"1") BOARD="rk3399" ;;
	*)
		echo -e "\e[1;31m Pls select correct board \e[0m"
		exit 0
		;;
	esac

	ARCH="arm64"
	CHIP="RK3399"
	TOOLS=$ROOT/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
	KERNEL_NAME="linux4.4.179"
	;;
"OrangePiRDA")
	OPTION=$(whiptail --title "Orange Pi Build System" \
		--menu "$MENUSTR" 15 60 5 --cancel-button Exit --ok-button Select \
		"0" "OrangePi i96" \
		"1" "OrangePi 2G-iot" \
		3>&1 1>&2 2>&3)

	case "${OPTION}" in
	"0")
		BOARD="i96"
		;;
	"1")
		BOARD="2G-iot"
		;;
	*)
		echo -e "\e[1;31m Pls select correct board \e[0m"
		exit 0
		;;
	esac

	ARCH="arm"
	CHIP="RDA"
	TOOLS=${ROOT}/toolchain/gcc-linaro-1.13.1-2012.02-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-
	KERNEL_NAME="linux3.10.62"
	;;
*)
	echo -e "\e[1;31m Pls select correct platform \e[0m"
	exit 0
	;;
esac

MENUSTR="Pls select build option"
OPTION=$(whiptail --title "OrangePi Build System" \
	--menu "$MENUSTR" 20 60 10 --cancel-button Finish --ok-button Select \
	"0" "Build Release Image" \
	"1" "Build Rootfs" \
	"2" "Build Uboot" \
	"3" "Build Linux" \
	"4" "Build Module only" \
	"5" "Update Kernel Image" \
	"6" "Update Module" \
	"7" "Update Uboot" \
	3>&1 1>&2 2>&3)

case "${OPTION}" in
"0")
	select_distro
	compile_uboot
	compile_kernel
	build_rootfs
	build_image

	whiptail --title "OrangePi Build System" --msgbox "Succeed to build Image" \
		10 40 0 --ok-button Continue
	;;
"1")
	select_distro
	compile_uboot
	compile_kernel
	build_rootfs
	whiptail --title "OrangePi Build System" --msgbox "Succeed to build rootfs" \
		10 40 0 --ok-button Continue
	;;
"2")
	compile_uboot
	;;
"3")
	compile_kernel
	compile_module
	;;
"4")
	compile_module
	;;
"5")
	[ "${PLATFORM}" = "OrangePiRK3399" ] && uboot_check || boot_check
	kernel_update
	;;
"6")
	rootfs_check
	modules_update
	;;
"7")
	uboot_check
	uboot_update
	;;
*)
	whiptail --title "OrangePi Build System" \
		--msgbox "Pls select correct option" 10 50 0
	;;
esac
