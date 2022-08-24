#!/bin/bash

setup_front()
{
	cat > "$DEST/type-phase" << EOF
#!/bin/bash -e
apt-get -y install ttf-wqy-zenhei
EOF
	chmod +x "$DEST/type-phase"
 	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
}
setup_resize-helper()
{
	cat > "$DEST/usr/sbin/resize-helper" << "EOF"
#!/bin/sh
# Copyright (c) Fathi Boudra <fathi.boudra@linaro.org>
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.

# we must be root
[ $(whoami) = "root" ] || { echo "E: You must be root" && exit 1; }

# we must have few tools
SGDISK=$(which sgdisk) || { echo "E: You must have sgdisk" && exit 1; }
PARTED=$(which parted) || { echo "E: You must have parted" && exit 1; }
PARTPROBE=$(which partprobe) || { echo "E: You must have partprobe" && exit 1; }
RESIZE2FS=$(which resize2fs) || { echo "E: You must have resize2fs" && exit 1; }

# find root device
# ROOT_DEVICE=$(findmnt --noheadings --output=SOURCE / | cut -d'[' -f1)
ROOT_DEVICE="/dev/"$(sed -n 's/^DEVNAME=//p' /sys/dev/block/$(mountpoint -d /)/uevent 2> /dev/null)
# prune root device (for example UUID)
ROOT_DEVICE=$(realpath ${ROOT_DEVICE})
# get the partition number and type
PART_ENTRY_NUMBER=$(udevadm info --query=property --name=${ROOT_DEVICE} | grep '^ID_PART_ENTRY_NUMBER=' | cut -d'=' -f2)
PART_TABLE_TYPE=$(udevadm info --query=property --name=${ROOT_DEVICE} | grep '^ID_PART_TABLE_TYPE=' | cut -d'=' -f2)
# find the block device
DEVICE=$(udevadm info --query=path --name=${ROOT_DEVICE} | awk -F'/' '{print $(NF-1)}')
DEVICE="/dev/${DEVICE}"

if [ "$PART_TABLE_TYPE" = "gpt" ]; then
	${SGDISK} -e ${DEVICE}
	${PARTPROBE}
fi

${PARTED} ${DEVICE} resizepart ${PART_ENTRY_NUMBER} 100%
${PARTPROBE}
${RESIZE2FS} "${ROOT_DEVICE}"

EOF
	chmod +x $DEST/usr/sbin/resize-helper
	cat > "$DEST/lib/systemd/system/resize-helper.service" <<EOF
[Unit]
Description=Resize root filesystem to fit available disk space
After=systemd-remount-fs.service

[Service]
Type=oneshot
ExecStart=-/usr/sbin/resize-helper
ExecStartPost=/bin/systemctl disable resize-helper.service

[Install]
WantedBy=basic.target
EOF
	cat > "$DEST/type-phase" <<EOF
#!/bin/bash

apt-get -y install gdisk
/bin/systemctl enable resize-helper.service
EOF
	chmod +x "$DEST/type-phase"
 	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
}
install_gpu_lib()
{
	if [ $DISTRO = "bionic" -o $DISTRO = "jammy" -o $DISTRO = "focal" ]; then
	cat > "$DEST/type-phase" <<EOF
#!/bin/bash

mkdir /tmp/libmali -p
dpkg -X /packages/libmali/libmali-rk-midgard-t86x-r14p0_1.6-2_arm64.deb /tmp/libmali
cp /tmp/libmali/usr/lib/aarch64-linux-gnu/lib* /usr/lib/aarch64-linux-gnu/ -rfa

sed 's/^#\(deb-src\)/\1/' -i /etc/apt/sources.list
apt update
apt-get -y build-dep xserver-xorg-core
cp -rfa /packages/xserver/xserver_for_bionic/* / 

apt-get clean
rm -rf /tmp/*
EOF
	elif [ $DISTRO = "xenial" -o $DISTRO = "stretch" -o $DISTRO = "bullseye" ]; then
		cat > "$DEST/type-phase" <<EOF
#!/bin/bash -e

sed 's/^#\(deb-src\)/\1/' -i /etc/apt/sources.list
apt update
apt-get -y build-dep xserver-xorg-core
apt-get remove -y --purge libegl1-mesa-dev:arm64 libgbm-dev:arm64
dpkg -i /packages/libmali/*.deb
rm -rf /usr/lib/aarch64-linux-gnu/mesa-egl

apt-get -y install libxcb-xkb-dev libxfont-dev wayland-protocols

cp -rfa /packages/xserver/xserver_for_$DISTRO/* /

apt-get clean
EOF

fi
	chmod +x "$DEST/type-phase"
 	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
}


compile_gst()
{
	cat > "$DEST/type-phase" << EOF
#!/bin/bash -e

cd /packages/source
mkdir -p /opt/build

if [ $DISTRO = "xenial" -o $DISTRO = "stretch" -o $DISTRO = "bullseye" ]; then
# install orc
tar -xvf orc-0.4.25.tar
cd orc-0.4.25
./autogen.sh --prefix=/usr
make
make install
cd -


# install  xorg-macros 1.12
# wget https://www.x.org/archive/individual/util/util-macros-1.12.0.tar.gz
tar -xvf util-macros-1.12.0.tar.gz
cd util-macros-1.12.0
./configure --prefix=/usr
make
make install
cd -
fi

unzip libdrm-rockchip-rockchip-2.4.74.zip
cd libdrm-rockchip-rockchip-2.4.74
./autogen.sh --prefix=/usr 
make
make install
make install DESTDIR=/opt/build/
cd -

#git clone https://github.com/rockchip-linux/mpp.git
unzip mpp-release.zip
cd mpp-release/build/linux/aarch64
./make-Makefiles.bash
make
make install
make install DESTDIR=/opt/build/
cd -

#git clone https://github.com/rockchip-linux/gstreamer-rockchip.git
unzip gstreamer-rockchip.zip
cd gstreamer-rockchip-master
./autogen.sh --prefix=/usr --enable-gst --disable-rkximage
make 
make install DESTDIR=/opt/build/
cd -


# git clone https://github.com/rockchip-linux/gstreamer-rockchip-extra.git
unzip gstreamer-rockchip-extra.zip
cd gstreamer-rockchip-extra-master
./autogen.sh --prefix=/usr --enable-gst --enable-rkximage
make
make install DESTDIR=/opt/build/
cd -



# git clone https://github.com/rockchip-linux/camera_engine_rkisp.git
tar -xf camera_engine_rkisp.tar.xz
cd camera_engine_rkisp
mkdir -p build
make CROSS_COMPILE=

mkdir -p /opt/build/etc/iqfiles
cp iqfiles/ov13850_CMK-CT0116_Largan-50013A1.xml /opt/build/etc/iqfiles
mkdir -p /opt/build/usr/lib/rkisp/ae
mkdir -p /opt/build/usr/lib/rkisp/af
mkdir -p /opt/build/usr/lib/rkisp/awb

cp ./build/lib/librkisp.so /opt/build/usr/lib  -a
cp ./build/lib/libgstrkisp.so /opt/build/usr/lib/gstreamer-1.0/ -a
cp ./build/ext/rkisp/usr/lib/gstreamer-1.0/libgstvideo4linux2.so /opt/build/usr/lib/  -a
cp ./plugins/3a/rkiq/aec/lib64/librkisp_aec.so /opt/build/usr/lib/rkisp/ae  -a
cp ./plugins/3a/rkiq/af/lib64/librkisp_af.so /opt/build/usr/lib/rkisp/af -a
cp ./plugins/3a/rkiq/awb/lib64/librkisp_awb.so /opt/build/usr/lib/rkisp/awb -a

cd -
mkdir -p /opt/build/usr/local/
cp /packages/test.mp4 /opt/build/usr/local/

EOF
	chmod +x "$DEST/type-phase"
 	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
	
	rm -rf $BUILD/build
	mv $DEST/opt/build $BUILD
	cd $BUILD
	tar czf build_for_${DISTRO}.tar.gz build
        cd -
}
install_gstreamer()
{
	if [ $DISTRO = "bionic" -o $DISTRO = "jammy" -o $DISTRO = "focal" ]; then 
#	cp /etc/resolv.conf "$DEST/etc/resolv.conf"
	cat > "$DEST/type-phase" << EOF
#!/bin/bash -e

apt-get install -y systemd systemctl bison flex libffi-dev libmount-dev libpcre3 libpcre3-dev zlib1g-dev libssl-dev gtk-doc-tools \
        automake autoconf libtool  gettext make autopoint g++ xz-utils net-tools
apt-get install -y libasound2-dev libx11-dev


apt-get install -y unzip cmake make


apt-get -y install gstreamer1.0-plugins-* 
apt-get -y install gstreamer1.0-libav 
apt-get -y install libgstreamer1.0*
apt-get -y install libgstreamer1.0-dev 
apt-get -y install libgstreamer-plugins-base1.0-dev 
apt-get -y install libgstreamer-plugins-bad1.0-dev

apt-get clean
EOF

	
	elif [ $DISTRO = "xenial" -o $DISTRO = "stretch" -o $DISTRO = "bullseye" ]; then
#		cp /etc/resolv.conf "$DEST/etc/resolv.conf"
	        cat > "$DEST/type-phase" << EOF

#!/bin/bash -e

apt-get install -y systemd systemctl bison flex libffi-dev libmount-dev libpcre3 libpcre3-dev zlib1g-dev libssl-dev gtk-doc-tools \
        automake autoconf libtool  gettext make autopoint g++ xz-utils net-tools
apt-get install -y libasound2-dev libx11-dev
	
apt-get -y install unzip
apt-get -y install libxext-dev
apt-get -y install libjpeg62-dev
apt-get -y install gdisk

apt-get -y install libxv-dev libpulse-dev
apt-get -y install libgl1-mesa-dev libgles2-mesa

cp -rfa /packages/others/gstreamer/glib-2.52.3/* /
cp -rfa /packages/others/gstreamer/gstreamer-1.12.2/* /
cp -rfa /packages/others/gstreamer/gst-plugins-base-1.12.2/* /
cp -rfa /packages/others/gstreamer/gst-plugins-good-1.12.2/* /
cp -rfa /packages/others/gstreamer/gst-plugins-bad-1.12.2/* /
cp -rfa /packages/others/gstreamer/gst-plugins-ugly-1.12.2/* /
cp -rfa /packages/others/gstreamer/gst-libav-1.12.2/* /

apt-get clean
EOF

	fi
	chmod +x "$DEST/type-phase"
 	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"

	if [ ! -f $BUILD/build_for_${DISTRO}.tar.gz ]; then
		compile_gst
	fi
	rm -rf $BUILD/build
	tar zxf $BUILD/build_for_${DISTRO}.tar.gz -C $BUILD
	cp -rfa $BUILD/build/* $DEST/ 

	if [ $DISTRO = "bionic" -o $DISTRO = "jammy" -o $DISTRO = "focal" ]; then 
		cp $DEST/usr/lib/gstreamer-1.0/* $DEST/usr/lib/aarch64-linux-gnu/gstreamer-1.0/ -rfa
	fi
	sync
}
