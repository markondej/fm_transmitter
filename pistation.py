#!/usr/bin/python

import os, argparse

parser = argparse.ArgumentParser(prog='python PiStation.py', description='Broadcasts WAV/MP3 file over FM using RPI GPIO #4 pin.')
parser.add_argument("song_file")
parser.add_argument("-f", "--frequency", help="Set TX frequency. Acceptable range 87.1-108.2", type=float)
arg = parser.parse_args()

def main():
    os.system('clear')
    print ("Welcome to ImPiFM!  \nVersion 1.0 \nGPLv3 License\n")
    if 87.1 >= arg.frequency >= 108.2:
        print "Frequency argument out of range.";exit()
    try:
        if ".mp3" in arg.song_file.lower():
            os.system("ffmpeg -i %s -f s16le -ar 22.05k -ac 1 - | sudo ./fm_transmitter -f %s - %s") %(str(arg.frequency), arg.song_file)
        elif ".wav" in arg.song_file.lower():
            os.system("sudo ./fm_transmitter -f %s %s 22050 stereo") %(str(arg.frequency), arg.song_file)
        else:
            print "That file extension is not supported."
            print "File name provided: %s" %arg.song_file
            raise IOError
    except Exception:
        print "Something went wrong. Halting."; exit()
    except IOError:
        print "There was an error regarding file selection. Halting."; exit()
    
if __name__ == '__main__':
    main()