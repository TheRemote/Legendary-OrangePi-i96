#!/bin/bash

install_xfce_desktop() {
	cp /etc/resolv.conf "$DEST/etc/resolv.conf"
	cat >"$DEST/type-phase" <<EOF
#!/bin/bash
export DEBIAN_FRONTEND=noninteractive
apt-get -y install systemctl xorg xfce4 xfce4-goodies vlc network-manager-gnome

apt-get -y autoremove
apt-get clean

EOF
	chmod +x "$DEST/type-phase"
	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
	rm -f "$DEST/etc/resolv.conf"
}

install_lxde_desktop() {
	_user="orangepi"
	_auto="-y -q"
	if [ $DISTRO = "bionic" -o $DISTRO = "xenial" -o $DISTRO = "jammy" -o $DISTRO = "focal" ]; then
		_DST=Ubuntu
	else
		_DST=Debian
	fi
	cp /etc/resolv.conf "$DEST/etc/resolv.conf"
	cat >"$DEST/type-phase" <<EOF
#!/bin/bash

echo ""
date
echo -e "\033[36m======================="
echo -e "Installing LXDE Desktop"
echo -e "=======================\033[37m"
setterm -default
echo ""



echo "Package update..."
apt-get $_auto update 
#echo "Package upgrade..."
#apt-get $_auto upgrade
echo ""

echo "$_DST - $_REL, Installing LXDE DESKTOP..."

# === Install desktop =============================================================================================================================================================
echo "  installing xserver & lxde desktop, please wait..."
apt-get $_auto install xinit xserver-xorg systemctl
apt-get $_auto install lxde lightdm lightdm-gtk-greeter policykit-1 --no-install-recommends
apt-get $_auto install net-tools
apt-get $_auto install lxsession-logout
apt-get clean

if [ "${_DST}" = "Ubuntu" ] ; then
	apt-get $_auto install humanity-icon-theme --no-install-recommends 
fi

apt-get $_auto install pulseaudio pulseaudio-utils alsa-oss alsa-utils alsa-tools libasound2-data pavucontrol
apt-get $_auto install smplayer 
apt-get $_auto install synaptic software-properties-gtk lxtask galculator policykit-1-gnome --no-install-recommends
apt-get clean

# === Install network packages & internet browser =================================================================================================================================
# === you don't have to install internet browser, you can save ~100MB ===

echo "  installing network packages, please wait..."
if [ "${_DST}" = "Ubuntu" ]; then
	apt-get $_auto install chromium-browser gvfs-fuse gvfs-backends --no-install-recommends
else
	apt-get $_auto install chromium gvfs-fuse gvfs-backends --no-install-recommends
fi
apt-get $_auto install network-manager-gnome
apt-get clean


# === Configuration ===============================================================================================================================================================
echo ""
echo "Configuring desktop..."

if [ -f /etc/X11/Xwrapper.config ]; then
    cat /etc/X11/Xwrapper.config | sed s/"allowed_users=console"/"allowed_users=anybody"/g > /tmp/_xwrap
    mv /tmp/_xwrap /etc/X11/Xwrapper.config
fi

if [ -f /etc/lightdm/lightdm-gtk-greeter.conf ]; then
    cat /etc/lightdm/lightdm-gtk-greeter.conf | sed "/background=\/usr/d" > /tmp/_greet
    mv /tmp/_greet /etc/lightdm/lightdm-gtk-greeter.conf
    cat /etc/lightdm/lightdm-gtk-greeter.conf | sed '/\[greeter\]/abackground=\/usr\/share\/lxde\/wallpapers\/orangepi.jpg' > /tmp/_greet
    mv /tmp/_greet /etc/lightdm/lightdm-gtk-greeter.conf
fi

#*********************
# ** CONFIGURE SOUND
#*********************
cat > /etc/asound.conf << _EOF_
pcm.!default {
    type hw
    card 1    #If you want to set HDMI as output ,turn 0 to 1.
    device 0
  }
  ctl.!default {
    type hw
    card 1   #If you want to set HDMI as output ,turn 0 to 1.
  }
_EOF_

if [ -f /etc/pulse/default.pa ]; then
    cat /etc/pulse/default.pa | sed s/"load-module module-udev-detect$"/"load-module module-udev-detect tsched=0"/g > /tmp/default.pa
    mv /tmp/default.pa /etc/pulse/default.pa
fi


usermod -a -G adm,dialout,cdrom,dip,video,plugdev,netdev,fuse $_user

chown -R $_user:$_user /home/$_user

EOF
	chmod +x "$DEST/type-phase"
	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
	rm -f "$DEST/etc/resolv.conf"

}

deboostrap_rootfs() {
	dist="$1"
	tgz="$(readlink -f "$2")"
	TEMP=$(mktemp -d)

	[ "$TEMP" ] || exit 1
	cd $TEMP && pwd

	# this is updated very seldom, so is ok to hardcode
	debian_archive_keyring_deb="${SOURCES}/pool/main/d/debian-archive-keyring/debian-archive-keyring_2021.1.1_all.deb"
	wget -O keyring.deb "$debian_archive_keyring_deb"
	ar -x keyring.deb && rm -f control.tar.gz debian-binary && rm -f keyring.deb
	DATA=$(ls data.tar.*) && compress=${DATA#data.tar.}

	KR=debian-archive-keyring.gpg
	bsdtar --include ./usr/share/keyrings/$KR --strip-components 4 -xvf "$DATA"
	rm -f "$DATA"

	apt-get -y install debootstrap qemu-user-static

	qemu-debootstrap --arch=${ROOTFS_ARCH} --keyring=$TEMP/$KR $dist rootfs ${SOURCES}
	rm -f $KR

	# keeping things clean as this is copied later again
	#	rm -f rootfs/usr/bin/qemu-arm-static
	if [ $ROOTFS_ARCH = "arm64" ]; then
		rm -f rootfs/usr/bin/qemu-aarch64-static
	elif [ $ROOTFS_ARCH = "armhf" ]; then
		rm -f rootfs/usr/bin/qemu-arm-static
	fi

	bsdtar -C $TEMP/rootfs -a -cf $tgz .
	rm -fr $TEMP/rootfs

	cd -
}

do_chroot() {
	# Add qemu emulation.
	#	cp /usr/bin/qemu-arm-static "$DEST/usr/bin"
	if [ $ARCH = "arm64" ]; then
		cp /usr/bin/qemu-aarch64-static "$DEST/usr/bin"
	elif [ $ARCH = "arm" ]; then
		cp /usr/bin/qemu-arm-static "$DEST/usr/bin"
	fi

	cmd="$@"
	chroot "$DEST" mount -t proc proc /proc || true
	chroot "$DEST" mount -t sysfs sys /sys || true
	chroot "$DEST" $cmd
	chroot "$DEST" umount /sys
	chroot "$DEST" umount /proc

	# Clean up
	rm -f "$DEST/usr/bin/qemu-arm-static"
}

do_conffile() {
	mkdir -p $DEST/opt/boot
	if [ "${PLATFORM}" = "OrangePiRDA" ]; then
		cp -rf ${EXTER}/chips/RDA/sbin/* ${DEST}/usr/local/sbin
		cp -rf ${EXTER}/common/rootfs/100-fs-warning ${DEST}/etc/update-motd.d/
		cp -rf ${EXTER}/chips/RDA/CameraTest ${DEST}/home/orangepi
		cp -rf ${EXTER}/chips/RDA/sbin/test_playback.sh ${DEST}/home/orangepi
		cp -rf ${EXTER}/chips/RDA/CameraTest ${DEST}/root
		cp -rf ${EXTER}/chips/RDA/sbin/test_playback.sh ${DEST}/root
	else
		echo -e "\e[1;31m Pls select correct platform \e[0m"
		exit 0
	fi

	mkdir -p "$DEST/etc/ssh"
	cp $EXTER/common/rootfs/sshd_config $DEST/etc/ssh/ -f
	cp $EXTER/common/rootfs/profile_for_root $DEST/root/.profile -f
	#	cp $EXTER/bluetooth/bt.sh $DEST/usr/local/sbin/ -f
	#	cp $EXTER/bluetooth/brcm_patchram_plus/brcm_patchram_plus $DEST/usr/local/sbin/ -f
	chmod +x $DEST/usr/local/sbin/*
}

add_ssh_keygen_service() {
	cat >"$DEST/etc/systemd/system/ssh-keygen.service" <<EOF
[Unit]
Description=Generate SSH keys if not there
Before=ssh.service
ConditionPathExists=|!/etc/ssh/ssh_host_key
ConditionPathExists=|!/etc/ssh/ssh_host_key.pub
ConditionPathExists=|!/etc/ssh/ssh_host_rsa_key
ConditionPathExists=|!/etc/ssh/ssh_host_rsa_key.pub
ConditionPathExists=|!/etc/ssh/ssh_host_dsa_key
ConditionPathExists=|!/etc/ssh/ssh_host_dsa_key.pub
ConditionPathExists=|!/etc/ssh/ssh_host_ecdsa_key
ConditionPathExists=|!/etc/ssh/ssh_host_ecdsa_key.pub
ConditionPathExists=|!/etc/ssh/ssh_host_ed25519_key
ConditionPathExists=|!/etc/ssh/ssh_host_ed25519_key.pub

[Service]
ExecStart=/usr/bin/ssh-keygen -A
Type=oneshot
RemainAfterExit=yes

[Install]
WantedBy=ssh.service
EOF
	do_chroot systemctl enable ssh-keygen
}

pregenerate_ssh_keys() {
	do_chroot /usr/bin/ssh-keygen -A
}

set_locale() {
	cat >"$DEST/type-phase" <<EOF
#!/bin/bash
echo "locales locales/default_environment_locale select en_US.UTF-8" | debconf-set-selections
echo "locales locales/locales_to_be_generated multiselect en_US.UTF-8 UTF-8" | debconf-set-selections
rm -f "/etc/locale.gen"
dpkg-reconfigure --frontend noninteractive locales
EOF
	chmod +x "$DEST/type-phase"
	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
}

add_rclocal_service() {
	echo "Adding rc.local service..."
	cat >"$DEST/lib/systemd/system/rc-local.service" <<EOF
[Unit]
Description=rc.local service

[Service]
ExecStart=/etc/rc.local
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
	cat >"$DEST/etc/rc.local" <<EOF
#!/bin/sh

# Set wireless txpower
iwconfig wlan0 txpower 20

# Trim drives once on startup
fstrim -v /
fstrim -v /media/boot

exit 0
EOF
	chmod +x "$DEST/etc/rc.local"

	cat >"$DEST/type-phase" <<EOF
#!/bin/bash

/bin/systemctl enable rc-local.service
EOF
	chmod +x "$DEST/type-phase"
	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
}

add_fstab() {
	echo "Adding fstab..."
	cat >"$DEST/etc/fstab" <<EOF
# OrangePI fstab
/dev/mmcblk0p2  /  ext4  errors=remount-ro,noatime,nodiratime  0 1
/dev/mmcblk0p1  /media/boot  ext2  defaults  0 0
tmpfs /tmp  tmpfs nodev,nosuid,mode=1777  0 0
EOF
}

add_networking() {
	# Main networking
	mkdir -p "$DEST/etc/network/interfaces.d"
	cat >"$DEST/etc/network/interfaces.d/eth0" <<EOF
auto eth0
iface eth0 inet dhcp
EOF

	cat >"$DEST/etc/hostname" <<EOF
orangepi${BOARD}
EOF
	cat >"$DEST/etc/hosts" <<EOF
127.0.0.1 localhost
127.0.1.1 orangepi${BOARD}

# The following lines are desirable for IPv6 capable hosts
::1     localhost ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
EOF
	cat >"$DEST/etc/resolv.conf" <<EOF
nameserver 8.8.8.8
EOF

	# Add wireless driver
	echo "rdawfmac" >>${DEST}/etc/modules-load.d/modules.conf

	# Remove default eth0 interface
	rm -rf $DEST/etc/network/interfaces.d/eth0

	cat >"$DEST/type-phase" <<EOF
#!/bin/bash

# Enable resolved
/bin/systemctl enable systemd-resolved.service

# Enable haveged
/bin/systemctl enable haveged.service

# Fix AmbientCapabilities of e2scrub_reap.service
sed -i 's/AmbientCapabilities=.*//g' /usr/lib/systemd/system/e2scrub_reap.service
/bin/systemctl daemon-reload
EOF
	chmod +x "$DEST/type-phase"
	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
}

add_bluetooth_service() {
	echo "Adding bluetooth service..."
	cat >"$DEST/usr/sbin/bt_fixup.sh" <<EOF
#!/bin/bash

rfkill unblock bluetooth
bt_init
hciattach -s 921600 /dev/ttyS1 any 921600 flow
EOF
	chmod +x "$DEST/usr/sbin/bt_fixup.sh"
	cat >"$DEST/lib/systemd/system/bluetooth_fixup.service" <<EOF
[Unit]
Description=bluetooth_fixup service

[Service]
ExecStart=/usr/sbin/bt_fixup.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
	cat >"$DEST/type-phase" <<EOF
#!/bin/bash

/bin/systemctl enable bluetooth_fixup.service

# Bluetooth patchram to enable functionality
cd ~
git clone https://github.com/well0nez/RDA5991g_patchram --depth=1
cd RDA5991g_patchram
gcc bt_init.c -o bt_init
cp bt_init /usr/bin/bt_init
chmod +x /usr/bin/bt_init
EOF
	chmod +x "$DEST/type-phase"
	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
}

add_gpio_service() {
	# GPIO fixup
	curl -k -L -o "$DEST/usr/local/sbin/gpio_fixup.sh" https://raw.githubusercontent.com/MehdiZAABAR/OrangePi-I96-Work/master/OrangePiRDA/output/rootfs/usr/local/sbin/gpio_fixup.sh
	chmod +x "$DEST/usr/local/sbin/gpio_fixup.sh"
	curl -k -L -o "$DEST/usr/local/bin/opio" https://wiki.pbeirne.com/patb/opio/raw/master/opio
	chmod +x "$DEST/usr/local/bin/opio"
	curl -k -L -o "$DEST/usr/local/bin/devmem2.py" https://wiki.pbeirne.com/patb/i96/raw/master/devmem2.py
	chmod +x "$DEST/usr/local/bin/devmem2.py"
	cat >"$DEST/lib/systemd/system/gpio_fixup.service" <<EOF
[Unit]
Description=OrangePi GPIO Fixup

[Service]
ExecStart=/usr/local/sbin/gpio_fixup.sh

[Install]
WantedBy=multi-user.target
EOF
	cat >"$DEST/type-phase" <<EOF
#!/bin/bash

/bin/systemctl enable gpio_fixup.service

# Build WiringPi
cd ~
git clone https://github.com/MehdiZAABAR/WiringPi.git --depth=1
cd WiringPi
make -j\$(nproc) && make -j\$(nproc) install

# Orange Pi i96 OPi.GPIO library
cd ..
git clone https://github.com/Farnsworth9qc/OPi.GPIO.git --depth=1
cd OPi.GPIO
python3 setup.py build
python3 setup.py install
groupadd gpio
adduser orangepi gpio
EOF
	chmod +x "$DEST/type-phase"
	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"

	cat >"$DEST/etc/udev/rules.d/99-gpio.rules" <<EOF
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'" SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value ; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"
EOF
}

add_resize_rootfs_service() {
	cat >"$DEST/lib/systemd/system/resize-rootfs.service" <<EOF
[Unit]
Description=Resize root filesystem to fit available disk space
After=systemd-remount-fs.service

[Service]
Type=oneshot
ExecStart=-/usr/local/sbin/resize_rootfs.sh
ExecStartPost=/bin/systemctl disable resize-rootfs.service

[Install]
WantedBy=basic.target
EOF
	cat >"$DEST/type-phase" <<EOF
#!/bin/bash

/bin/systemctl enable resize-rootfs.service
EOF
	chmod +x "$DEST/type-phase"
	do_chroot /type-phase
	sync
	rm -f "$DEST/type-phase"
}

add_debian_apt_sources() {
	local release="$1"
	local aptsrcfile="$DEST/etc/apt/sources.list"
	cat >"$aptsrcfile" <<EOF
deb ${SOURCES} ${release} main contrib non-free
#deb-src ${SOURCES} ${release} main contrib non-free
EOF
	# No separate security or updates repo for unstable/sid
	[ "$release" = "sid" ] || cat >>"$aptsrcfile" <<EOF
deb ${SOURCES} ${release}-updates main contrib non-free
#deb-src ${SOURCES} ${release}-updates main contrib non-free

deb http://security.debian.org/ ${release}-security main contrib non-free
#deb-src http://security.debian.org/ ${release}-security main contrib non-free
EOF
}

add_ubuntu_apt_sources() {
	local release="$1"
	cat >"$DEST/etc/apt/sources.list" <<EOF
deb ${SOURCES} ${release} main restricted universe multiverse
deb-src ${SOURCES} ${release} main restricted universe multiverse

deb ${SOURCES} ${release}-updates main restricted universe multiverse
deb-src ${SOURCES} ${release}-updates main restricted universe multiverse

deb ${SOURCES} ${release}-security main restricted universe multiverse
deb-src $SOURCES ${release}-security main restricted universe multiverse

deb ${SOURCES} ${release}-backports main restricted universe multiverse
deb-src ${SOURCES} ${release}-backports main restricted universe multiverse
EOF
}

prepare_env() {
	if [ ${ARCH} = "arm" ]; then
		QEMU="/usr/bin/qemu-arm-static"
		ROOTFS_ARCH="armhf"
	elif [ ${ARCH} = "arm64" ]; then
		QEMU="/usr/bin/qemu-aarch64-static"
		ROOTFS_ARCH="arm64"
	fi

	if [ ! -d "$DEST" ]; then
		echo "Destination $DEST not found or not a directory."
		echo "Create $DEST"
		mkdir -p $DEST
	fi

	if [ "$(ls -A -Ilost+found $DEST)" ]; then
		echo "Destination $DEST is not empty."
		echo "Clean up space."
		rm -rf $DEST
	fi

	cleanup() {
		if [ -e "$DEST/proc/cmdline" ]; then
			umount "$DEST/proc"
		fi
		if [ -d "$DEST/sys/kernel" ]; then
			umount "$DEST/sys"
		fi
		if [ -d "$TEMP" ]; then
			rm -rf "$TEMP"
		fi
	}
	trap cleanup EXIT

	case $DISTRO in
	xenial | bionic | focal | jammy)
		case $SOURCES in
		"CDN" | "OFCL")
			SOURCES="http://ports.ubuntu.com"
			ROOTFS="http://cdimage.ubuntu.com/ubuntu-base/releases/${DISTRO}/release/ubuntu-base-${DISTRO_NUM}-base-${ROOTFS_ARCH}.tar.gz"
			;;
		"CN")
			#SOURCES="http://mirrors.aliyun.com/ubuntu-ports"
			#SOURCES="http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports"
			SOURCES="http://mirrors.ustc.edu.cn/ubuntu-ports"
			ROOTFS="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-cdimage/ubuntu-base/releases/${DISTRO}/release/ubuntu-base-${DISTRO_NUM}-base-${ROOTFS_ARCH}.tar.gz"
			;;
		*)
			SOURCES="http://ports.ubuntu.com"
			ROOTFS="http://cdimage.ubuntu.com/ubuntu-base/releases/${DISTRO}/release/ubuntu-base-${DISTRO_NUM}-base-${ROOTFS_ARCH}.tar.gz"
			;;
		esac
		;;
	stretch | buster | bullseye)
		ROOTFS="${DISTRO}-base-${ROOTFS_ARCH}.tar.gz"
		METHOD="debootstrap"
		case $SOURCES in
		"CDN")
			SOURCES="http://httpredir.debian.org/debian"
			;;
		"OFCL")
			SOURCES="http://ftp.debian.org/debian"
			;;
		"CN")
			SOURCES="http://ftp2.cn.debian.org/debian"
			;;
		*)
			SOURCES="http://httpredir.debian.org/debian"
			;;
		esac
		;;
	*)
		echo "Unknown distribution: $DISTRO"
		exit 1
		;;
	esac

	TARBALL="$EXTER/$(basename $ROOTFS)"
	if [ ! -e "$TARBALL" ]; then
		if [ "$METHOD" = "download" ]; then
			echo "Downloading $DISTRO rootfs tarball ..."
			wget -O "$TARBALL" "$ROOTFS"
		elif [ "$METHOD" = "debootstrap" ]; then
			deboostrap_rootfs "$DISTRO" "$TARBALL"
		else
			echo "Unknown rootfs creation method"
			exit 1
		fi
	fi

	# Extract with BSD tar
	echo -n "Extracting ... "
	mkdir -p $DEST
	$UNTAR "$TARBALL" -C "$DEST"
	echo "OK"
}

prepare_rootfs_server() {

	rm "$DEST/etc/resolv.conf"
	cp /etc/resolv.conf "$DEST/etc/resolv.conf"
	if [ "$DISTRO" = "xenial" -o "$DISTRO" = "bionic" -o "$DISTRO" = "jammy" -o "$DISTRO" = "focal" ]; then
		DEB=ubuntu
		DEBUSER=orangepi
		EXTRADEBS=""
		ADDPPACMD=
		DISPTOOLCMD=
	elif [ "$DISTRO" = "sid" -o "$DISTRO" = "stretch" -o "$DISTRO" = "stable" -o "$DISTRO" = "bullseye" ]; then
		DEB=debian
		DEBUSER=orangepi
		EXTRADEBS=""
		ADDPPACMD=
		DISPTOOLCMD=
	else
		echo "Unknown DISTRO=$DISTRO"
		exit 2
	fi
	add_${DEB}_apt_sources $DISTRO
	rm -rf "$DEST/etc/apt/sources.list.d/proposed.list"
	cat >"$DEST/second-phase" <<EOF
#!/bin/bash
export DEBIAN_FRONTEND=noninteractive

apt-get -y update
apt-get -y install curl locales sudo imagemagick haveged lshw libnl-3-dev libnl-genl-3-dev libv4l-dev cmake bluez wireless-tools bluez-tools apt-transport-https man-db subversion python3-pip python3-setuptools gpg net-tools g++ libjpeg-dev usbutils dosfstools curl xz-utils iw rfkill ifupdown wpasupplicant openssh-server rsync u-boot-tools vim parted git autoconf gcc libtool ntp libsysfs-dev pkg-config libdrm-dev xutils-dev alsa-utils acl crda cpufrequtils network-manager trace-cmd can-utils

# Install Python spidev library
pip3 install spidev

apt-get install -f

apt-get -y remove --purge modemmanager
$ADDPPACMD
apt-get -y dist-upgrade && apt-get -y autoremove
$DISPTOOLCMD
adduser --gecos $DEBUSER --disabled-login $DEBUSER --uid 1000
adduser --gecos root --disabled-login root --uid 0
echo root:orangepi | chpasswd
chown -R 1000:1000 /home/$DEBUSER
echo "$DEBUSER:$DEBUSER" | chpasswd
usermod -a -G sudo $DEBUSER
usermod -a -G adm $DEBUSER
usermod -a -G video $DEBUSER
usermod -a -G plugdev $DEBUSER
usermod -a -G audio $DEBUSER
usermod -a -G bluetooth $DEBUSER
usermod -a -G netdev $DEBUSER
usermod -a -G dialout $DEBUSER
apt-get clean
ntpd -gq
EOF
	chmod +x "$DEST/second-phase"
	do_chroot /second-phase
	rm -f "$DEST/second-phase"
	rm -f "$DEST/etc/resolv.conf"

	add_networking
	do_conffile
	add_ssh_keygen_service
	add_resize_rootfs_service
	add_fstab
	add_rclocal_service
	add_gpio_service
	add_bluetooth_service

	# Create getty terminal service
	sed -i 's|After=rc.local.service|#\0|;' "$DEST/lib/systemd/system/serial-getty@.service"
	rm -f "$DEST"/etc/ssh/ssh_host_*

	# Create folders for WiFi and Bluetooth MAC addresses
	mkdir -p "$DEST/data/misc/wifi"
	mkdir -p "$DEST/data/misc/bluetooth/"

	cd $BUILD
	tar czf ${DISTRO}_server_rootfs.tar.gz rootfs
	cd -
}

prepare_rootfs_desktop() {
	install_lxde_desktop
	cd $BUILD
	tar czf ${DISTRO}_desktop_rootfs.tar.gz rootfs
	cd -

}

server_setup() {
	# Bring back folders
	mkdir -p "$DEST/lib"
	mkdir -p "$DEST/usr"
	mkdir -p "$DEST/etc/network/interfaces.d"

	# Copy WLANMAC and BTMAC from external if they exist (convenience for building so it always has the same addresses)
	if [ -e "${EXTER}/presets/WLANMAC" ]; then
		cp "${EXTER}/presets/WLANMAC" "$DEST/data/misc/wifi/WLANMAC"
	fi
	if [ -e "${EXTER}/presets/BTMAC" ]; then
		cp "${EXTER}/presets/BTMAC" "$DEST/data/misc/bluetooth/BTMAC"
	fi
	# Copy preconfigured network and crda file if they exist
	if [ -e "${EXTER}/presets/interfaces" ]; then
		cp "${EXTER}/presets/interfaces" "$DEST/etc/network/interfaces"
	fi
	if [ -e "${EXTER}/presets/crda" ]; then
		cp "${EXTER}/presets/crda" "$DEST/etc/default/crda"
	fi
	# Preset locales if locales file is present
	if [ -e "${EXTER}/presets/locales" ]; then
		set_locale
	fi
	# Pregenerate SSH keys if present
	if [ -e "${EXTER}/presets/ssh" ]; then
		pregenerate_ssh_keys
	fi

	if [ ! -d $DEST/lib/modules ]; then
		mkdir "$DEST/lib/modules"
	else
		rm -rf $DEST/lib/modules
		mkdir "$DEST/lib/modules"
	fi

	# Install Kernel modules
	make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS modules_install INSTALL_MOD_PATH="$DEST"

	# Install Kernel headers
	make -C $LINUX ARCH=${ARCH} CROSS_COMPILE=$TOOLS headers_install INSTALL_HDR_PATH="$DEST/usr/local"
	#cp $EXTER/firmware $DEST/lib/ -rf

	#rm -rf $BUILD/${DISTRO}_${IMAGETYPE}_rootfs
	#cp -rfa $DEST $BUILD/${DISTRO}_${IMAGETYPE}_rootfs
}

desktop_setup() {
	echo ""
}

build_rootfs() {
	prepare_env

	if [ $TYPE = "1" ]; then
		if [ -f $BUILD/${DISTRO}_desktop_rootfs.tar.gz ]; then
			rm -rf $DEST
			tar zxf $BUILD/${DISTRO}_desktop_rootfs.tar.gz -C $BUILD
		else
			if [ -f $BUILD/${DISTRO}_server_rootfs.tar.gz ]; then
				rm -rf $DEST
				tar zxf $BUILD/${DISTRO}_server_rootfs.tar.gz -C $BUILD
				prepare_rootfs_desktop
			else
				prepare_rootfs_server
				prepare_rootfs_desktop

			fi
		fi
		server_setup
		desktop_setup
	else
		if [ -f $BUILD/${DISTRO}_server_rootfs.tar.gz ]; then
			rm -rf $DEST
			tar zxf $BUILD/${DISTRO}_server_rootfs.tar.gz -C $BUILD
		else
			prepare_rootfs_server
		fi
		server_setup
	fi
}
