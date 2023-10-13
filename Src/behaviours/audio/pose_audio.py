from __future__ import print_function

import os
import sys
import wave
import getopt
import alsaaudio

VERBOSITY = 1
WAV_LOCATION = os.path.join(os.environ['HOME'],
                                    'whistle/wav')

def play(device, f):

	format = None

	# 8bit is unsigned in wav files
	if f.getsampwidth() == 1:
		format = alsaaudio.PCM_FORMAT_U8
	# Otherwise we assume signed data, little endian
	elif f.getsampwidth() == 2:
		format = alsaaudio.PCM_FORMAT_S16_LE
	elif f.getsampwidth() == 3:
		format = alsaaudio.PCM_FORMAT_S24_3LE
	elif f.getsampwidth() == 4:
		format = alsaaudio.PCM_FORMAT_S32_LE
	else:
		raise ValueError('Unsupported format')

	periodsize = f.getframerate() // 8

	if VERBOSITY >= 2:
		print('%d channels, %d sampling rate, format %d, periodsize %d\n' % (f.getnchannels(),
																		 f.getframerate(),
																		 format,
																		 periodsize))

	device = alsaaudio.PCM(type=alsaaudio.PCM_PLAYBACK)
	device.setrate(f.getframerate())
	device.setchannels(f.getnchannels())
	device.setperiodsize(periodsize)
	device.setformat(format)

	if VERBOSITY >= 2:
		print(device.dumpinfo())

	data = f.readframes(periodsize)
	while data:
		# Read data from stdin
		device.write(data)
		data = f.readframes(periodsize)


def play_sound(sound_name):

	device = 'default'
	sound_name = sound_name.replace(' ', '').replace('-','')
	sound_file_path = os.path.join(WAV_LOCATION, sound_name+'.wav')
	f = wave.open(sound_file_path, 'rb')
	play(device, f)
	f.close()

# if __name__ == "__main__":
#     play_sound("/root/redbackbots/Install/NaoHome/whistle/wav/goalred")