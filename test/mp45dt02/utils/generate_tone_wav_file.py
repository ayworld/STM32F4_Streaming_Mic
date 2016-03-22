import numpy as np
import wave
import os
import struct

dir_files = "./output/files/"
sampling_f = 16000

################################################################################

if not os.path.exists(dir_files):
    os.makedirs(dir_files)

################################################################################
t = np.arange(160000) / float(sampling_f)
x = 1.0 * np.cos(t*2*np.pi*2000)

wav_file = dir_files + "tone.wave"
wav = wave.open(wav_file, "w")
wav.setparams((1,                   # nchannels
               2,                   # sampwidth
               sampling_f,          # framerate 
               0,                   # nframes
               "NONE",              # comptype
               "not compressed"     # compname
               ))

packed = []
for sample in x:
    sample = sample * 32767
    packed.append(struct.pack("h", int(sample)))

wav.writeframes("".join(packed))
wav.close()



