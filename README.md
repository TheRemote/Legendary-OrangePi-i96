# Legendary-OrangePi-i96
Image that fixes the USB port on the Orange Pi i96 to use High Speed (480M) instead of Full Speed.<br>
<br>
Also upgrades you to Debian Bullseye!
<br>
You do not have to build this yourself.  Images are available <a href="https://github.com/TheRemote/Legendary-OrangePi-i96/releases">here in the releases section</a>.<br>
<br>
Build Instructions<br>
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
You may now build the images the same way I did!