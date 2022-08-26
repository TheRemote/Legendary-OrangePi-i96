#!/bin/bash

build_rk_image()
{
	VER="v1.2"
	IMAGENAME="Legendary_OrangePi_${BOARD}_${OS}_${DISTRO}_${IMAGETYPE}_${VER}"
	IMAGE="$BUILD/images/$IMAGENAME.img"

	if [ ! -d $BUILD/images ]; then
		mkdir -p $BUILD/images
	fi
        local UBOOT_START=24576
	local UBOOT_END=32767
	local TRUST_START=32768
	local TRUST_END=40959
	local BOOT_START=49152
	local BOOT_END=114687
	local ROOTFS_START=376832
	local LOADER1_START=64
	local IMG_ROOTFS_SIZE=$(expr `du -s $DEST | awk 'END {print $1}'` + 400 \* 1024)
	local GPTIMG_MIN_SIZE=$(expr $IMG_ROOTFS_SIZE \* 1024 + \( $(((${ROOTFS_START}))) \) \* 512)
	local GPT_IMAGE_SIZE=$(expr $GPTIMG_MIN_SIZE \/ 1024 \/ 1024 + 2)

	dd if=/dev/zero of=${IMAGE}2 bs=1M count=$(expr $IMG_ROOTFS_SIZE  \/ 1024 )
	mkfs.ext4 -O ^metadata_csum -F -b 4096 -E stride=2,stripe-width=1024 -L rootfs ${IMAGE}2
	
	if [ ! -d /tmp/tmp ]; then
		mkdir -p /tmp/tmp
	fi

	mount -t ext4 ${IMAGE}2 /tmp/tmp
	# Add rootfs into Image
	cp -rfa $DEST/* /tmp/tmp

	umount /tmp/tmp

	if [ -d $BUILD/orangepi ]; then
		rm -rf $BUILD/orangepi
	fi 

	if [ -d /tmp/tmp ]; then
		rm -rf /tmp/tmp
	fi

	echo "Generate SD boot image : ${SDBOOTIMG} !"
	dd if=/dev/zero of=${IMAGE} bs=1M count=0 seek=$GPT_IMAGE_SIZE
	parted -s $IMAGE mklabel gpt
	parted -s $IMAGE unit s mkpart uboot ${UBOOT_START} ${UBOOT_END}
	parted -s $IMAGE unit s mkpart trust ${TRUST_START} ${TRUST_END}
	parted -s $IMAGE unit s mkpart boot ${BOOT_START} ${BOOT_END}
	parted -s $IMAGE -- unit s mkpart rootfs ${ROOTFS_START} -34s
	set +x
	ROOT_UUID="614e0000-0000-4b53-8000-1d28000054a9"
gdisk $IMAGE <<EOF
x
c
4
${ROOT_UUID}
w
y
EOF
	dd if=$BUILD/uboot/idbloader.img of=$IMAGE seek=$LOADER1_START conv=notrunc
	dd if=$BUILD/uboot/uboot.img of=$IMAGE seek=$UBOOT_START conv=notrunc,fsync
	dd if=$BUILD/uboot/trust.img of=$IMAGE seek=$TRUST_START conv=notrunc,fsync
	dd if=$BUILD/kernel/boot.img of=$IMAGE seek=$BOOT_START conv=notrunc,fsync
	dd if=${IMAGE}2 of=$IMAGE seek=$ROOTFS_START conv=notrunc,fsync
	rm -f ${IMAGE}2
	cd ${BUILD}/images/
	rm -f ${IMAGENAME}.tar.xz
	md5sum ${IMAGENAME}.img > ${IMAGENAME}.img.md5sum
	#tar czvf  ${IMAGENAME}.tar.gz $IMAGENAME.img*
	tar cf  ${IMAGENAME}.tar.xz --use-compress-program='xz -T8 -v -9' $IMAGENAME.img*
	rm -f *.md5sum

	sync

	
}

build_rda_image()
{
	VER="v1.2"
	IMAGENAME="Legendary_OrangePi_${BOARD}_${OS}_${DISTRO}_${IMAGETYPE}_${VER}"
	IMAGE="${BUILD}/images/$IMAGENAME.img"

	if [ ! -d ${BUILD}/images ]; then
		mkdir -p ${BUILD}/images
	fi

	uboot_position=256	#block
	boot_size=50		#MiB
	part_position=2048	#KiB

	uboot=${BUILD}/uboot/u-boot.rda

	# Create beginning of disk
	dd if=/dev/zero bs=1M count=$((part_position/1024)) of="$IMAGE"

	# Create boot file system (VFAT)
	dd if=/dev/zero bs=1M count=${boot_size} of=${IMAGE}1
	mkfs.ext2 -L BOOT ${IMAGE}1

	dd if="${uboot}" conv=notrunc seek=${uboot_position} of="${IMAGE}"

	[ -d /tmp/tmp ] || mkdir -p /tmp/tmp
	mount -t ext2 ${IMAGE}1 /tmp/tmp
	cp -rf ${EXTER}/chips/RDA/bootarg/* /tmp/tmp
	cp -rf ${BUILD}/kernel/zImage /tmp/tmp
	sync
	umount /tmp/tmp

	disk_size=$[(`du -s $DEST | awk 'END {print $1}'`+part_position)/1024+400+boot_size]

	if [ "$disk_size" -lt 60 ]; then
		echo "Disk size must be at least 60 MiB"
		exit 2
	fi

	echo "Creating image $IMAGE of size $disk_size MiB ..."

	dd if=${IMAGE}1 conv=notrunc oflag=append bs=1M seek=$((part_position/1024)) of="$IMAGE"
	rm -f ${IMAGE}1

	# Create additional ext4 file system for rootfs
	dd if=/dev/zero bs=1M count=$((disk_size-boot_size-part_position/1024)) of=${IMAGE}2
	mkfs.ext4 -O ^metadata_csum -F -b 4096 -E stride=2,stripe-width=1024 -L rootfs ${IMAGE}2

	if [ ! -d /media/tmp ]; then
		mkdir -p /media/tmp
	fi

	mount -t ext4 ${IMAGE}2 /media/tmp
	# Add rootfs into Image
	cp -rfa $DEST/* /media/tmp

	umount /media/tmp

	dd if=${IMAGE}2 conv=notrunc oflag=append bs=1M seek=$((part_position/1024+boot_size)) of="$IMAGE"
	rm -f ${IMAGE}2

	if [ -d /media/tmp ]; then
		rm -rf /media/tmp
	fi

	# Add partition table
	cat <<EOF | fdisk "$IMAGE"
o
n
p
1
$((part_position*2))
+${boot_size}M
t
83
n
p
2
$((part_position*2 + boot_size*1024*2))

t
2
83
w
EOF

	cd ${BUILD}/images/ 
	rm -f ${IMAGENAME}.tar.xz
	md5sum ${IMAGENAME}.img > ${IMAGENAME}.img.md5sum
	#tar czvf  ${IMAGENAME}.tar.gz $IMAGENAME.img*
	tar cf  ${IMAGENAME}.tar.xz --use-compress-program='xz -T8 -v -9' $IMAGENAME.img*

	rm -f *.md5sum

	sync
}

build_image()
{	
	if [ ${PLATFORM} = "OrangePiRK3399" ]; then
		build_rk_image
		return
	elif [ ${PLATFORM} = "OrangePiRDA" ]; then
		build_rda_image
		return
	fi
	VER="v1.2"
	IMAGENAME="Legendary_OrangePi_${BOARD}_${OS}_${DISTRO}_${IMAGETYPE}_${VER}"
	IMAGE="${BUILD}/images/$IMAGENAME.img"

	if [ ! -d ${BUILD}/images ]; then
		mkdir -p ${BUILD}/images
	fi

	# Partition Setup
	boot0_position=8      # KiB
	uboot_position=16400  # KiB
	part_position=20480   # KiB
	boot_size=50          # MiB

	# Create beginning of disk
	dd if=/dev/zero bs=1M count=$((part_position/1024)) of="$IMAGE"

	# Create boot file system (VFAT)
	dd if=/dev/zero bs=1M count=${boot_size} of=${IMAGE}1
	mkfs.vfat -n BOOT ${IMAGE}1
	
	case "${PLATFORM}" in
		"OrangePiH3" | "OrangePiH6_Linux4.9")
			cp -rfa "${BUILD}/kernel/uImage_${BOARD}" "${BUILD}/kernel/uImage"

			boot0="${BUILD}/uboot/boot0_sdcard_${CHIP}.bin"
			uboot="${BUILD}/uboot/u-boot-${CHIP}.bin"
			dd if="${boot0}" conv=notrunc bs=1k seek=${boot0_position} of="${IMAGE}"
			dd if="${uboot}" conv=notrunc bs=1k seek=${uboot_position} of="${IMAGE}"


			if [ "${PLATFORM}" = "OrangePiH3" ]; then
				cp -rfa ${EXTER}/script/script.bin_$BOARD $BUILD/script.bin
				mcopy -m -i ${IMAGE}1 ${BUILD}/kernel/uImage ::
				mcopy -sm -i ${IMAGE}1 ${BUILD}/script.bin_${BOARD} :: || true
			elif [ "${PLATFORM}" = "OrangePiH6_Linux4.9" ]; then
				mkdir -p "${BUILD}/orangepi"
				cp -f ${BUILD}/kernel/uImage "${BUILD}/orangepi/"
				cp -f ${BUILD}/uboot/${PLATFORM}.dtb "${BUILD}/orangepi/"

			        mcopy -sm -i ${IMAGE}1 ${BUILD}/orangepi ::
			        mcopy -m -i ${IMAGE}1 ${EXTER}/chips/$CHIP/initrd.img :: || true
			        mcopy -m -i ${IMAGE}1 ${EXTER}/chips/$CHIP/uEnv.txt :: || true

				rm -rf $BUILD/orangepi
			fi
			;;
		"OrangePiH3_mainline" | "OrangePiH6_mainline")
			cp -rfa ${EXTER}/chips/${CHIP}/mainline/boot_files/uInitrd ${BUILD}/uInitrd
			cp -rfa ${EXTER}/chips/${CHIP}/mainline/boot_files/orangepiEnv.txt ${BUILD}/orangepiEnv.txt
			mkimage -C none -A arm -T script -d ${EXTER}/chips/${CHIP}/mainline/boot_files/boot.cmd ${EXTER}/chips/${CHIP}/mainline/boot_files/boot.scr
			cp -rfa ${EXTER}/chips/${CHIP}/mainline/boot_files/boot.* ${BUILD}/
	
			uboot="${BUILD}/uboot/u-boot-sunxi-with-spl.bin-${BOARD}"
			dd if="$uboot" conv=notrunc bs=1k seek=$boot0_position of="$IMAGE"

			if [ ${PLATFORM} = "OrangePiH6_mainline" ];then
				cp -rfa ${BUILD}/kernel/Image_${BOARD} ${BUILD}/kernel/Image
				mcopy -m -i ${IMAGE}1 ${BUILD}/kernel/Image ::
			elif [ ${PLATFORM} = "OrangePiH3_mainline" ];then 
				cp -rfa ${BUILD}/kernel/zImage_${BOARD} ${BUILD}/kernel/zImage
				mcopy -m -i ${IMAGE}1 ${BUILD}/kernel/zImage ::
			fi

			mcopy -m -i ${IMAGE}1 ${BUILD}/uInitrd :: || true
			mcopy -m -i ${IMAGE}1 ${BUILD}/orangepiEnv.txt :: || true
			mcopy -m -i ${IMAGE}1 ${BUILD}/boot.* :: || true
			mcopy -m -i ${IMAGE}1 ${BUILD}/kernel/System.map-${BOARD} :: || true
			mcopy -sm -i ${IMAGE}1 ${BUILD}/dtb :: || true
			;;
		"*")
			echo -e "\e[1;31m Pls select correct platform \e[0m"
			exit 0
			;;
	esac

	disk_size=$[(`du -s $DEST | awk 'END {print $1}'`+part_position)/1024+400+boot_size]

	if [ "$disk_size" -lt 60 ]; then
		echo "Disk size must be at least 60 MiB"
		exit 2
	fi

	echo "Creating image $IMAGE of size $disk_size MiB ..."

	dd if=${IMAGE}1 conv=notrunc oflag=append bs=1M seek=$((part_position/1024)) of="$IMAGE"
	rm -f ${IMAGE}1

	# Create additional ext4 file system for rootfs
	dd if=/dev/zero bs=1M count=$((disk_size-boot_size-part_position/1024)) of=${IMAGE}2
	mkfs.ext4 -O ^metadata_csum -F -b 4096 -E stride=2,stripe-width=1024 -L rootfs ${IMAGE}2

	if [ ! -d /media/tmp ]; then
		mkdir -p /media/tmp
	fi

	mount -t ext4 ${IMAGE}2 /media/tmp
	# Add rootfs into Image
	cp -rfa $DEST/* /media/tmp

	umount /media/tmp

	dd if=${IMAGE}2 conv=notrunc oflag=append bs=1M seek=$((part_position/1024+boot_size)) of="$IMAGE"
	rm -f ${IMAGE}2

	if [ -d ${BUILD}/orangepi ]; then
		rm -rf ${BUILD}/orangepi
	fi 

	if [ -d /media/tmp ]; then
		rm -rf /media/tmp
	fi

	# Add partition table
	cat <<EOF | fdisk "$IMAGE"
o
n
p
1
$((part_position*2))
+${boot_size}M
t
c
n
p
2
$((part_position*2 + boot_size*1024*2))

t
2
83
w
EOF

	cd ${BUILD}/images/ 
	rm -f ${IMAGENAME}.tar.xz
	md5sum ${IMAGE} > ${IMAGE}.md5sum
	#tar czvf  ${IMAGENAME}.tar.gz $IMAGENAME.img*
	tar cf  ${IMAGENAME}.tar.xz --use-compress-program='xz -T8 -v -9' $IMAGENAME.img*
	rm -f *.md5sum

	sync
}
