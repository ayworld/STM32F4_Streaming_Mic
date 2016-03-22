################################################################################
# Copyright (c) 2016, Alan Barr
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################


################################################################################
# Usage
################################################################################

import numpy as np
import matplotlib.pyplot as plot
from scipy.fftpack import fft
import sys
import re
import os
import wave
import struct

################################################################################
# Configuration 
################################################################################

dir_plots = "./output/comparison/plots/"
dir_files = "./output/comparison/files/"
show_plots = True

file_gdb = sys.argv[1]
sampling_f = 16000
################################################################################
# Helper Functions 
################################################################################
def show_plot(plot):
    if show_plots:
        plot.show()

def get_gdb_struct_variable(struct, var, data):
    struct = struct + "\s+=\s+{(.*?)}"
    var = var + "\s+=\s+(\w+),"
    struct = re.compile(struct, re.DOTALL).search(data).group(1)
    return re.compile(var, re.DOTALL).search(struct).group(1)

def get_gdb_array(arr, data):
    arr = arr + "\s+=\s+{(.*?)}"
    strs = re.compile(arr, re.DOTALL).search(data).group(1).replace(" ","").replace("\n","").split(",")
    return map(float, strs)

def get_arm_signal(gdb_file):
    with open(gdb_file, "r") as f:
        data = f.read()

    signal = get_gdb_array("buffer", data)

    return signal 

def table_print(title, label_keys, label_0, label_1, dict_0, dict_1):
    global file_h

    width = 25
    print >> file_h, "\n\n"
    print >> file_h, "|" + "-" * (width * 3 + 2) + "|"
    print >> file_h, "| " + title.ljust(width * 3 + 1) + "|"
    print >> file_h, "|" + "-" * (width * 3 + 2) + "|"
    print >> file_h, "|%s|%s|%s|" %(label_keys.ljust(width), label_0.ljust(width), label_1.ljust(width))

    for k in dict_0:
        print >> file_h, "|%s|%s|%s|" % (k.ljust(width), 
                            str(dict_0[k]).ljust(width),
                            str(dict_1[k]).ljust(width))
    print >> file_h, "|" + "-" * (width * 3 + 2) + "|"


################################################################################
# Run
################################################################################
gdb_signal = sys.argv[1]

if not os.path.exists(dir_plots):
    os.makedirs(dir_plots)

if not os.path.exists(dir_files):
    os.makedirs(dir_files)

file_h = open(dir_files + "/output.txt", "w")

################################################################################
# Signal
################################################################################
signal =  get_arm_signal(file_gdb)

plot.figure()
plot.plot(signal)
plot.grid()
plot.title("Filtered Signal")
plot.xlabel("Sample Number")
plot.ylabel("Amplitude")
plot.savefig(dir_plots + "1_x_signal.png")

show_plot(plot)
plot.close()


################################################################################
# FFT Of Signal
################################################################################

fft_signal = fft(signal, sampling_f)
fft_signal = fft_signal[0:len(fft_signal)/2]

x = np.arange(0, (sampling_f/2), (sampling_f/2)/len(fft_signal))

start_freq = 100

plot.figure()
plot.plot(x[start_freq:], np.abs(fft_signal[start_freq:]))
plot.grid()
plot.title("FFT of Filtered Signal")
plot.xlabel("Frequency [Hz]")
plot.ylabel("Power")
plot.savefig(dir_plots + "2_x_signal_fft.png")

show_plot(plot)
plot.close()


################################################################################
# Create WAV File 
################################################################################
wav_file = dir_files + "output.wave"
wav = wave.open(wav_file, "w")
wav.setparams((1,                   # nchannels
               2,                   # sampwidth
               16000,               # framerate 
               0,                   # nframes
               "NONE",              # comptype
               "not compressed"     # compname
               ))

packed = []
for sample in signal:
# Should this be possible?
    if sample > 32767:
        sample = 32767

    if sample < -32768:
        sample = -32768
    packed.append(struct.pack("h", int(sample)))

wav.writeframes("".join(packed))
wav.close()

wav = wave.open(wav_file, "r")
print wav.getparams()
wav.close()



