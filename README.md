# FM Transmitter
Use the Raspberry Pi as an FM transmitter. Works on every Raspberry Pi board.

Just get an FM receiver, connect a 20 - 40 cm plain wire to the Raspberry Pi's GPIO4 (PIN 7 on GPIO header) to act as an antenna, and you are ready for broadcasting.

This project uses the general clock output to produce frequency modulated radio communication. It is based on an idea originally presented by [Oliver Mattos and Oskar Weigl](http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter) at [PiFM project](http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter).
## Installation and usage
To use this software you will have to build the executable. First, install required dependencies:
```
sudo apt-get update
sudo apt-get install make build-essential
```
Depending on OS (eg. Ubuntu Server 20.10) installing Broadcom libraries may be also required:
```
sudo apt-get install libraspberrypi-dev
```  
After installing dependencies clone this repository and use `make` command in order to build executable:
```
git clone https://github.com/markondej/fm_transmitter
cd fm_transmitter
make
```
After a successful build you can start transmitting by executing the "fm_transmitter" program:
```
sudo ./fm_transmitter -f 100.6 acoustic_guitar_duet.wav
```
Notice:
* -f frequency - Specifies the frequency in MHz, 100.0 by default if not passed
* acoustic_guitar_duet.wav - Sample WAV file, you can use your own

Other options:
* -d dma_channel - Specifies the DMA channel to be used (0 by default), type 255 to disable DMA transfer, CPU will be used instead
* -b bandwidth - Specifies the bandwidth in kHz, 100 by default
* -r - Loops the playback

After transmission has begun, simply tune an FM receiver to chosen frequency, you should hear the playback.
### Raspberry Pi 4
On Raspberry Pi 4 other built-in hardware probably interfers somehow with this software making transmitting not possible on all standard FM broadcasting frequencies. In this case it is recommended to:
1. Compile executable with option to use GPIO21 instead of GPIO4 (PIN 40 on GPIO header):
```
make GPIO21=1
```
2. Changing either ARM core frequency scaling governor settings to "powersave" or changing ARM minimum and maximum core frequencies to one constant value (see: https://www.raspberrypi.org/forums/viewtopic.php?t=152692 ).
```
echo "powersave"| sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```
3. Using lower FM broadcasting frequencies (below 93 MHz) when transmitting.
### Use as general audio output device
[hydranix](https://github.com/markondej/fm_transmitter/issues/144) has came up with simple method of using transmitter as an general audio output device. In order to achieve this you should load "snd-aloop" module and stream output from loopback device to transmitter application:
```
sudo modprobe snd-aloop
arecord -D hw:1,1,0 -c 1 -d 0 -r 22050 -f S16_LE | sudo ./fm_transmitter -f 100.6 - &
```
Please keep in mind loopback device should be set default ALSA device (see [this article](https://www.alsa-project.org/wiki/Setting_the_default_device)). Also parameter "-D hw:X,1,0" should be pointing this device (use card number instead of "X").
### Microphone support
In order to use a microphone live input use the `arecord` command, eg.:
```
arecord -D hw:1,0 -c 1 -d 0 -r 22050 -f S16_LE | sudo ./fm_transmitter -f 100.6 -
```
In cases of a performance drop down use ```plughw:1,0``` instead of ```hw:1,0``` like this:
```
arecord -D plughw:1,0 -c 1 -d 0 -r 22050 -f S16_LE | sudo ./fm_transmitter -f 100.6 -
```
### Supported audio formats
You can transmitt uncompressed WAV (.wav) files directly or read audio data from stdin, eg. using MP3 file:
```
sudo apt-get install sox libsox-fmt-mp3
sox example.mp3 -r 22050 -c 1 -b 16 -t wav - | sudo ./fm_transmitter -f 100.6 -
```
Please note only uncompressed WAV files are supported. If you receive the "corrupted data" error try converting the file, eg. by using SoX:
```
sudo apt-get install sox libsox-fmt-mp3
sox example.mp3 -r 22050 -c 1 -b 16 -t wav converted-example.wav
sudo ./fm_transmitter -f 100.6 converted-example.wav
```
Or you could also use FFMPEG:
```
ffmpeg -i example.webm -f wav -bitexact -acodec pcm_s16le -ar 22050 -ac 1 converted-example.wav
sudo ./fm_transmitter -f 100.6 converted-example.wav
```
## Legal note
Please keep in mind that transmitting on certain frequencies without special permissions may be illegal in your country.
## New features
* DMA peripheral support
* Allows custom frequency and bandwidth settings
* Works on every Raspberry Pi model
* Reads mono and stereo files
* Reads data from stdin

Included sample audio was created by [graham_makes](https://freesound.org/people/graham_makes/sounds/449409/) and published on [freesound.org](https://freesound.org/)
