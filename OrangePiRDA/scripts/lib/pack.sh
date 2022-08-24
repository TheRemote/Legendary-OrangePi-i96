#!/bin/bash

do_prepare()
{
	FILE="${CHIP_FILE}"/pack/bin
	TOOLS_DIR="${CHIP_FILE}"/pack/tools
	SYS_CONFIG="${CHIP_FILE}"/sys_config.fex
	export PATH="${TOOLS_DIR}":"${PATH}"

	mkdir -p ${PACK_OUT}

	cp "${UBOOT}"/boot0_sdcard_${CHIP}.bin "${PACK_OUT}"
	cp "${UBOOT}"/u-boot-${CHIP}.bin "${PACK_OUT}"
	cp "${FILE}"/* "${PACK_OUT}"

	if [ ${CHIP} = "sun8iw7p1" ];then
		:
	else
		cp "${SYS_CONFIG}" "${PACK_OUT}"
	fi
}

do_ini_to_dts()
{
	case "${PLATFORM}" in
		"OrangePiH3")
			return
			;;
		"OrangePiH6_Linux4.9")
			local DTC_SRC_PATH="${LINUX}"/arch/"${ARCH}"/boot/dts/sunxi
			;;
		"*")
			local DTC_SRC_PATH="${LINUX}"/arch/"${ARCH}"/boot/dts
			;;
	esac

	local DTC_DEP_FILE="${DTC_SRC_PATH}"/."${CHIP}"-"${CHIP_BOARD}".dtb.d.dtc.tmp
	local DTC_SRC_FILE="${DTC_SRC_PATH}"/."${CHIP}"-"${CHIP_BOARD}".dtb.dts.tmp
	local DTC_INI_FILE="${PACK_OUT}"/sys_config_fix.fex
	cp "${SYS_CONFIG}" "${DTC_INI_FILE}"

        if [ ! -f $DTC_SRC_FILE ]; then
                printf "Script_to_dts: Can not find [%s-%s.dts]. Will use common dts file instead.\n" ${CHIP} ${CHIP_BOARD}
                DTC_DEP_FILE=${DTC_SRC_PATH}/.${CHIP}-soc.dtb.d.dtc.tmp
                DTC_SRC_FILE=${DTC_SRC_PATH}/.${CHIP}-soc.dtb.dts.tmp
        fi


	dtc_alpha -O dtb -o ${PACK_OUT}/sunxi.dtb	\
		-b 0			\
		-i $DTC_SRC_PATH	\
		-F $DTC_INI_FILE	\
		-d $DTC_DEP_FILE $DTC_SRC_FILE

	printf "Conver script to dts ok.\n"
}

do_common()
{
	cd ${PACK_OUT}
	ls
	
	if [ ${CHIP} = "sun8iw7p1" ];then
		./update_boot0 boot0_sdcard_sun8iw7p1.bin sys_config.bin SDMMC_CARD
		./update_uboot u-boot-sun8iw7p1.bin sys_config.bin
	else
		unix2dos sys_config.fex
		script sys_config.fex > /dev/null
		cp sys_config.bin config.fex
		cp sunxi.dtb sunxi.fex
		update_uboot_fdt u-boot-${CHIP}.bin sunxi.fex u-boot-${CHIP}.bin >/dev/null
		update_scp scp.fex sunxi.fex >/dev/null
		update_boot0 boot0_sdcard_${CHIP}.bin	sys_config.bin SDMMC_CARD > /dev/null
		unix2dos boot_package.cfg
		mv u-boot-${CHIP}.bin u-boot.fex
		dragonsecboot -pack boot_package.cfg
		mv u-boot.fex u-boo-${CHIP}.bin
		cp sunxi.fex 	    ${BUILD}/uboot/"${PLATFORM}".dtb
	fi

        cp boot0_sdcard_${CHIP}.bin ${BUILD}/uboot/boot0_sdcard_"${CHIP}".bin
        cp boot_package.fex ${BUILD}/uboot/u-boot-"${CHIP}".bin
	rm -rf ${PACK_OUT}
}

pack(){
	do_prepare
	do_ini_to_dts
	do_common
}
