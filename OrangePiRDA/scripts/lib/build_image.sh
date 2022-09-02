#!/bin/bash

build_rk_image() {
	VER="v1.5"
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
	local IMG_ROOTFS_SIZE=$(expr $(du -s $DEST | awk 'END {print $1}') + 400 \* 1024)
	local GPTIMG_MIN_SIZE=$(expr $IMG_ROOTFS_SIZE \* 1024 + \( $(((${ROOTFS_START}))) \) \* 512)
	local GPT_IMAGE_SIZE=$(expr $GPTIMG_MIN_SIZE \/ 1024 \/ 1024 + 2)

	dd if=/dev/zero of=${IMAGE}2 bs=1M count=$(expr $IMG_ROOTFS_SIZE \/ 1024)
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
	md5sum ${IMAGENAME}.img >${IMAGENAME}.img.md5sum
	#tar czvf  ${IMAGENAME}.tar.gz $IMAGENAME.img*
	tar cf ${IMAGENAME}.tar.xz --use-compress-program='xz -T8 -v -9' $IMAGENAME.img*
	rm -f *.md5sum

	sync

}

build_rda_image() {
	VER="v1.5"
	IMAGENAME="Legendary_OrangePi_${BOARD}_${OS}_${DISTRO}_${IMAGETYPE}_${VER}"
	IMAGE="${BUILD}/images/$IMAGENAME.img"

	if [ ! -d ${BUILD}/images ]; then
		mkdir -p ${BUILD}/images
	fi

	uboot_position=256 #block
	boot_size=50       #MiB
	part_position=2048 #KiB

	uboot=${BUILD}/uboot/u-boot.rda

	# Create beginning of disk
	dd if=/dev/zero bs=1M count=$((part_position / 1024)) of="$IMAGE"

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

	disk_size=$((($(du -s $DEST | awk 'END {print $1}') + part_position) / 1024 + 400 + boot_size))

	if [ "$disk_size" -lt 60 ]; then
		echo "Disk size must be at least 60 MiB"
		exit 2
	fi

	echo "Creating image $IMAGE of size $disk_size MiB ..."

	dd if=${IMAGE}1 conv=notrunc oflag=append bs=1M seek=$((part_position / 1024)) of="$IMAGE"
	rm -f ${IMAGE}1

	# Create additional ext4 file system for rootfs
	dd if=/dev/zero bs=1M count=$((disk_size - boot_size - part_position / 1024)) of=${IMAGE}2
	mkfs.ext4 -O ^metadata_csum -F -b 4096 -E stride=2,stripe-width=1024 -L rootfs ${IMAGE}2

	if [ ! -d /media/tmp ]; then
		mkdir -p /media/tmp
	fi

	mount -t ext4 ${IMAGE}2 /media/tmp
	# Add rootfs into Image
	cp -rfa $DEST/* /media/tmp

	umount /media/tmp

	dd if=${IMAGE}2 conv=notrunc oflag=append bs=1M seek=$((part_position / 1024 + boot_size)) of="$IMAGE"
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
$((part_position * 2))
+${boot_size}M
t
83
n
p
2
$((part_position * 2 + boot_size * 1024 * 2))

t
2
83
w
EOF

	cd ${BUILD}/images/
	rm -f ${IMAGENAME}.tar.xz
	md5sum ${IMAGENAME}.img >${IMAGENAME}.img.md5sum
	#tar czvf  ${IMAGENAME}.tar.gz $IMAGENAME.img*
	tar cf ${IMAGENAME}.tar.xz --use-compress-program='xz -T8 -v -9' $IMAGENAME.img*

	rm -f *.md5sum

	sync
}

build_image() {
	if [ ${PLATFORM} = "OrangePiRDA" ]; then
		build_rda_image
		return
	fi
}
