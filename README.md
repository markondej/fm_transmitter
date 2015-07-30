# fm_transmitter

Use Raspberry Pi as FM transmitter. Now supports both RPi 1 and RPi 2 boards.

This project uses the general clock output to produce frequency modulated radio communication. It is based on idea originaly posted here: [http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter](http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter), but does not use DMA controller in order to distribute samples to output (clock generator),so sound quality is worse as in PiFm project and only mono transmition is available but this makes possible to run it on all kind of boards.

## How to use it

To compile this project you can use Code::Blocks IDE [(http://codeblocks.org/)](http://codeblocks.org/) or alternatively convert CBP file into makefile.

Then you can use it by typing:
```
sudo ./fm_transmitter [filename] [frequency]
```

You can open WAVE files or read audio data from stdin, i.e.:
```
sox star_wars.wav -r 8000 -c 1 -b 8 -t wav - | sudo ./fm_transmitter - 100.0
```

Please keep in mind that transmitting on certain frequencies without special permissions may be illegal in your country.

## New features

* works both on RPi 1 and RPi 2
* reads mono and stereo files
* reads data from stdin
* based on threads
