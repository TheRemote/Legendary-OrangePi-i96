# Legendary-OrangePi-i96
An image for the Orange Pi i96 made from fixes around the internet as well as a set of build system modifications (to the official OrangePi_Build tool) to be able to build it yourself if desired.  Debian Bullseye (11) is available.<br>
<br>
<strong>You do not have to build this yourself.</strong>  Images are available <a href="https://github.com/TheRemote/Legendary-OrangePi-i96/releases">here in the releases section</a>.<br>
<br>
My blog post that birthed this image is located <a href="https://jamesachambers.com/orange-pi-i96-getting-started-guide/">here on jamesachambers.com</a>.<br>
<br>
<h2>Fixes</h2>
<ol>
  <li>Adds fixed version of WiringPi tool by MZA as well as an altenative tool by patb called opio and fixes GPIO pin assignments</li>
  <li>Adds spidev interface to access SPI over the GPIO pins</li>
  <li>Adds ntp to assist with fixing time on first startup -- use "sudo ntpd -gq" to force a time sync once you've set your correct timezone (sudo dpkg-reconfigure tzdata)</li>
  <li>Adds ability to use the i96 as a HID device (mouse/keyboard emulation)</li>
  <li>Adds USB serial connection</li>
  <li>Fixed notorious WiFi issues caused by missing crda package and no regulatory domain set (see first startup instructions to set REGDOMAIN)</li>
  <li>Fixed Bluetooth and set it up to work at startup using bluetooth patchram utility</li>
  <li>Fixed wireless MAC address changing each startup</li>
  <li>Fixed USB port to allow "High Speed" USB devices instead of locking them to "Full Speed"</li>
  <li>Fixed buggy UART not resetting properly which often breaks copying/pasting through a serial terminal</li>
  <li>Fixed sound issues that would prevent rebooting the system successfully after first startup</li>
  <li>Fixed GPIO permissions errors on startup</li>
  <li>Fixed orangepi user missing groups audio, bluetooth and netdev</li>
</ol>

<h2>First Startup Instructions</h2>
Set correct timezone:
<pre>sudo dpkg-reconfigure tzdata</pre>
Sync time:
<pre>sudo ntpd -gq</pre>
Set correct locale:
<pre>sudo apt install locales -y && sudo dpkg-reconfigure locales</pre>
Set wireless regulatory country:
<pre>sudo nano /etc/default/crda</pre>
Add your two letter country code (mine is US) to the end of the bottom line (after the equals sign) that says REGDOMAIN=<br>
Press Ctrl+X then Y to save the file.

<h2>Build Instructions</h2>
You should first clone the OrangePi_Build repository:<br>
<pre>git clone https://github.com/orangepi-xunlong/OrangePi_Build.git
cd OrangePi_Build
./Build_OrangePi.sh</pre><br>
Choose the "Orange Pi I96" option which will create the "OrangePiRDA" folder.<br>
<br>
The files in this repository are meant to replace the stock OrangePiRDA ones that are generated (specifically in the scripts folder).  You simply copy them over the top of your generated folder like this:<br>
<pre>cd ..
git clone https://github.com/TheRemote/Legendary-OrangePi-i96.git
cp -R Legendary-OrangePi-i96/OrangePiRDA/* OrangePiRDA/</pre>
You may now build the image the same way I did!<br>
Ubuntu is not building correctly yet.<br>
<br>
<h2>Version History</h2>
<ol>
  <li>September 22nd 2022 - V1.18 - Enable CONFIG_SND_USB to allow for USB soundcard use</li>
  <li>September 20th 2022 - V1.17 - Add further spidev fixes (thanks MZA, <a href="https://github.com/MehdiZAABAR/OrangePi-I96-Work/">OrangePi-I96-Work</a>)</li>
  <li>September 19th 2022 - V1.16 - Add cpufrequtils package (thanks Marco, <a href="https://github.com/TheRemote/Legendary-OrangePi-i96/pull/5">PR #5</a>)</li>
  <li>September 18th 2022 - V1.15 - Fix Bluetooth to have fixed MAC address stored in /data/misc/bluetooth (thanks Marco, <a href="https://github.com/TheRemote/Legendary-OrangePi-i96/pull/4">PR #4</a>)</li>
  <li>September 16th 2022 - V1.14 - Add bluez-tools package</li>
  <li>September 16th 2022 - V1.13 - Add Bluetooth patchram to enable bluetooth (thanks Marco, <a href="https://github.com/well0nez/RDA5991g_patchram">Bluetooth patchram fix</a>)</li>
  <li>September 14th 2022 - V1.12 - Add fixed WiringPi library (thanks MZA, <a href="https://github.com/MehdiZAABAR/WiringPi">WiringPi Fork</a>)</li>
  <li>September 10th 2022 - V1.11 - Fix USB gadget conflict (thanks SteveGotthardt, <a href="https://github.com/TheRemote/Legendary-OrangePi-i96/pull/3">PR #2</a>)</li>
  <li>September 10th 2022 - V1.10 - Adds USB serial connection. Set ACL to remove warnings from log files (thanks SteveGotthardt, <a href="https://github.com/TheRemote/Legendary-OrangePi-i96/pull/2">PR #2</a>)</li>
  <li>September 4th 2022 - V1.9 - Adds ability to use the i96 as a HID device (thanks jakeau, <a href="https://github.com/TheRemote/Legendary-OrangePi-i96/pull/1">PR #1</a>)</li>
  <li>September 3rd 2022 - V1.8 - Add crda package and instructions to configure regulatory domain (REGDOMAIN)</li>
  <li>September 2nd 2022 - V1.7 - Enable systemd-resolved service to help with DNS over WiFi, remove crashing hostapd service, fix e2scrub_all service, add orangepi user to audio, bluetooth, netdev</li>
  <li>September 2nd 2022 - V1.6 - Fix spidev</li>
  <li>September 1st 2022 - V1.5 - Adds spidev interface to access SPI over the GPIO pins</li>
  <li>August 31th 2022 - V1.4 - remove applying default locale due to breaking serial console on some systems (see first startup instructions to apply your correct locale instead of me applying mine which was causing problems), fix startup permissions errors related to GPIO</li>
  <li>August 30th 2022 - V1.3 - Add patb's gpio_fixup.sh script to fix GPIO pins on startup / gpio tool replacement / wireless LAN MAC address fix</li>
  <li>August 26th 2022 - V1.2 - Added ntp package to help with time issues (use "sudo ntpd -gq" on first startup), fixed locales issue</li>
  <li>August 24th 2022 - V1.1 - Added sound fix and UART fix from official OrangePi repository pull requests</li>
  <li>August 23rd 2022 - V1.0 - Initial Release</li>
</ol>
<h2>Credits</h2>
The drama surrounding this board is ridiculous.  Is it a great board?  Not really.  It also <a href="https://amzn.to/3QaYDeY">costs like $7 on Amazon</a>.  Is it a good board for $7?  Yes it is, especially in 2022 when Pis are routinely over $100.<br><br>
This board honestly isn't more broken than any other boards were at release.  It just never got support and it was almost right from the start.  It never received support from Armbian and Orange Pi has never supported it either (I mean most of the fixes in this were submitted to their official repository years ago with no response).  There are much more popular and widely used boards with much more serious hardware defects that are supported by both Armbian and Orange Pi (as they're well aware).<br><br> 
It's hard to say why everyone's jimmies were so rustled but from what I can tell it's mostly groupthink and political nonsense that has nothing to do with how defective the board actually is.  There were only 2 Pull Requests to even integrate from Orange Pi's repository as well as the USB issue.  Those are pretty much the only known issues with the board other than the flaky WiFI.  That is honestly a better track record than definitely most of Orange Pi's other boards and most boards in general.  Again, as someone who has been working with SBCs for nearly a decade and still has their Raspberry Pi 1 Model B's to prove it, what are you guys talking about?  This board is too toxic to touch with a flaky WiFI and some easily resolved kernel driver issues?  What?<br><br>
The lack of support is real though and has real consequences.  What hope is there then?  The community basically.  This image is just a compilation of fixes from people who dreamed of something better for the board and submitted PRs or fixes around the internet as well as patching Orange Pi's imaging tool to produce images that aren't literally 5 years outdated (although the kernel still is).<br><br>
Is it enough?  You'll have to judge for yourself if it's enough but it is enough to make this a decent headless Bullseye board with one working high speed USB 2.0 port.  The original ones really weren't which is why this was born.<br><br>
<br>
<a href="https://forum.armbian.com/topic/3232-orange-pi-2g-iot/page/6/">Credit to Gabor Hidvegi for the patch itself</a> as I found his patch (which wasn't as effective for the 2G version as he would have liked due to the modem initialization dropping the speed) to be working as-is for the regular i96 without the 2G modem present.<br>
<a href="https://github.com/orangepi-xunlong/OrangePiRDA_kernel/pull/2">Credit to GMMan</a> for the pull request on the official repository to fix sound playback kernel parameter issues<br>
<a href="https://github.com/orangepi-xunlong/OrangePiRDA_kernel/pull/3">Credit to GMMan again</a> for the pull request on the official repository to fix UART serial issues fixing copy/pasting<br>
<a href="https://wiki.pbeirne.com/patb/i96/src/master/gpio_fixup.sh">Credit to patb</a> for gpio_fixup.sh / devmem2.py which fixes the GPIO pins and the <a href="https://wiki.pbeirne.com/patb/opio">gpio tool replacement for WiringPi</a> as well as a <a href="https://wiki.pbeirne.com/patb/i96/">wireless driver which prevents the MAC address from changing each boot</a><br>
<a href="https://github.com/MesihK/linux-RDA8810/commit/d7089f4c43bd76082459e6995652b578ce2d10f4?diff=unified">Credit to MesihK</a> for the gpio files permissions fix<br>
<a href="https://4pda.to/forum/index.php?showtopic=813602&st=280">Credit to Yoshie</a> for enough hints to enable the spidev interface<br>
<a href="https://github.com/TheRemote/Legendary-OrangePi-i96/pull/1">Credit to jakeau</a> for adding the ability to use the i96 as a HID device (PR #1)<br>
<a href="https://github.com/TheRemote/Legendary-OrangePi-i96/pull/2">Credit to SteveGotthardt</a> for adding the USB serial connection and fixing ACL entries to eliminate warnings (PR #2)<br>
<a href="https://github.com/MehdiZAABAR/WiringPi">Credit to MZA</a> for fixing the WiringPi library to work with the i96 as well as fixes to the spidev interface<br>
<a href="https://github.com/well0nez/RDA5991g_patchram">Credit to Marco</a> for fixing the Bluetooth patchram utility to work with the i96 as well as fixing the Bluetooth MAC address (PR #4)<br>