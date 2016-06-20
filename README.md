# fm_transmitter

Use Raspberry Pi as FM transmitter. Works on any RPi board.

This project uses the general clock output to produce frequency modulated radio communication. It is based on idea originaly posted here: [http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter](http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter), but does not use DMA controller in order to distribute samples to output (clock generator),so sound quality is worse as in PiFm project and only mono transmition is available but this makes possible to run it on all kind of boards.

## How to use it

To compile this project use commands below:
```
sudo apt-get install make gcc g++
make
``` 

Then you can use it by typing:
```
sudo ./fm_transmitter [-f frequency] [-r] filename
```

### WAVE Files
You can open WAVE files or read audio data from stdin, i.e.:
```
sox star_wars.wav -r 22050 -c 1 -b 16 -t wav - | sudo ./fm_transmitter -f 100.6 -
```

### USB sound-card
To use a USB sound-card type this:
```
arecord -D hw:1,0 -c1 -d 0 -r 22050 -f S16_LE | sudo ./fm_transmitter -f 100.6 -
```
Some devices have problems with the one up (there is a warning in the terminal like ``` buffer overflow ``` after a few seconds - the transmitting is slow and will stopp), then you can use the following:
```
arecord -D plughw:1,0 -c1 -d 0 -r 22050 -f S16_LE | sudo ./fm_transmitter -f 100.6 -
```

## Law
Please keep in mind that transmitting on certain frequencies without special permissions may be illegal in your country.

## New features

* works on RPi 1, 2 and 3
* reads mono and stereo files
* reads data from stdin
* based on threads
