# Pi fm_transmitter

Use Raspberry Pi as FM transmitter. Works on any RPi board.

This project uses the general clock output to produce frequency modulated radio communication. It is based on idea originaly posted here: [http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter](http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter), but does not use DMA controller in order to distribute samples to output (clock generator),so sound quality is worse as in PiFm project and only mono transmition is available but this makes possible to run it on all kind of boards.

## New features

* works on RPi 1, 2 and 3
* reads mono and stereo files
* reads data from stdin
* based on threads
* Mp3 Support added :) (Thanks to [**CodyJHeiser**](https://github.com/CodyJHeiser "CodyJHeiser"),Source: [https://github.com/CodyJHeiser/PiStation](https://github.com/CodyJHeiser/PiStation))

## How to use it

First compile this project, use commands below:
```
sudo apt-get install make gcc g++
make
``` 
Now, you can use it directly by typing:
```
sudo ./fm_transmitter [-f frequency] [-r] filename
```
You can open WAVE files or read audio data from stdin, i.e.:
```
sox star_wars.wav -r 22050 -c 1 -b 16 -t wav - | sudo ./fm_transmitter -f 100.6 -
arecord -D hw:1,0 -c1 -d 0 -r 22050 -f S16_LE | sudo ./fm_transmitter -f 100.6 -
```
Broadcast from a usb microphone (see arecord manual page for config)

<code>arecord -d0 -c2 -f S16_LE -r 22050 -twav -D copy | sudo ./fm_transmitter -f 103.3 - </code>

## Steps for playing _mp3_ files:

Install **ffmpeg** & other libraries in RaspberryPi by the following steps-->

**Step 1:**
```
sudo apt-get install libmp3lame-dev
cd /usr/src
sudo git clone git://git.videolan.org/x264
cd x264
sudo ./configure --host=arm-unknown-linux-gnueabi --enable-static --disable-opencl
sudo make
sudo make install
```
This installs **libmp3lame** and **x264** libraries

**Step 2:**
```
cd /usr/src
sudo git clone git://source.ffmpeg.org/ffmpeg.git ffmpeg
cd ffmpeg
sudo ./configure --arch=armv7-a --target-os=linux --enable-gpl --enable-libx264 --enable-nonfree --enable-libmp3lame --extra-cflags='-march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard'
sudo make -j4
sudo make install
```
**Note--** use <code>sudo ./configure --arch=armel --target-os=linux --enable-gpl --enable-libx264 --enable-nonfree</code> for devices prior to ***RaspberryPi 2 model B+***.

_This Process will take some time , Have patience :)_

_Also support for other file types can be added , you can add these support during compiling the ffmpeg_
## How to play _mp3_ files :

<code>sudo python ./PiStation.py example.mp3(or wav)</code>

or

<code>sudo python ./PiStation.py -f [desired frequency] [filename]</code>

--------------
About the Program and What it Does
--------------
It uses the **ffmpeg** libraries to convert the mp3 file and stream it to the **fm_transmitter** file for broadcasting.

WAV files must be in the following format: 16 bit (mono or stereo) 22050 hz (generaly these are fine as-is, but you may need to convert or change the bitrate in the **PiStation.py** file it if it doesn't mach the required specs.)  Anything below 16 bits will work as well.

The Raspberry Pi can handle anywhere from 1MHz to 250MHz, however it is limited it in the code to 87.1 and 108.1 beacuse generaly radio bands don't go past these numbers.  There is also majorly restricted frequencies in there like, 121.5 and 243.0 (military aircraft).  You can remove this block by commenting out the code.

The PiStation allows you to use the code (from icrobotics link below) in a more simple way, simply load your music
onto your Pi, and play!

Please keep in mind that transmitting on certain frequencies without special permissions may be illegal in your country.
