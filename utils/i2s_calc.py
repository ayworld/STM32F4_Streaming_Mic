#! /usr/bin/env python3
################################################################################
# Copyright (c) 2014, Alan Barr
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


### Changeable variables 
PCM_FREQ_KHZ = 16
DECIMATION = 64
I2S_SAMPLE_SIZE_BITS = 16

HSE_FREQ_KHZ = 8000
PLLM = 8

# It should be possible to iterate over every value of these two
PLLI2SN = 258
PLLI2SR = 3

### Don't change
I2S_INPUT_FREQ_KHZ = (HSE_FREQ_KHZ * PLLI2SN/PLLM)/PLLI2SR
I2S_REQUIRED_CLK_FREQ_KHZ = PCM_FREQ_KHZ * DECIMATION
I2S_REQUIRED_AUDIO_FREQ_KHZ = (I2S_REQUIRED_CLK_FREQ_KHZ/(2 * I2S_SAMPLE_SIZE_BITS))
#I2S_REQUIRED_AUDIO_FREQ_KHZ = 192

ERROR_PERCENTAGE = 0 
ERROR_FREQ_KHZ  = 0
#Output bits
I2SODD = 0
I2SDIV = 0

# When the master clock is disabled (MCKOE bit cleared):
# FS = I2SxCLK / [(16*2)*((2*I2SDIV)+ODD))] when the channel frame is 16-bit wide

temp = I2S_INPUT_FREQ_KHZ / I2S_REQUIRED_AUDIO_FREQ_KHZ # = 32 * (2*I2SDIV + ODD)
temp = temp/32 + 0.5

if (int(temp) % 2 == 1):
    I2SODD = 1
else:
    I2SODD = 0

temp = (temp - I2SODD)/2
I2SDIV = int(temp)

I2S_ACTUAL_AUDIO_FREQ_KHZ = I2S_INPUT_FREQ_KHZ/((16 * 2) * ((2 * I2SDIV )+ I2SODD))

ERROR_FREQ_KHZ = I2S_ACTUAL_AUDIO_FREQ_KHZ - I2S_REQUIRED_AUDIO_FREQ_KHZ
if (ERROR_FREQ_KHZ < 0):
    ERROR_FREQ_KHZ = ERROR_FREQ_KHZ * -1
ERROR_PERCENTAGE = ERROR_FREQ_KHZ / I2S_REQUIRED_AUDIO_FREQ_KHZ * 100

print("---Output Information---")
print("Required I2S Audio Frequency (kHz): %.05f" % I2S_REQUIRED_AUDIO_FREQ_KHZ)
print("Obtained I2S Audio Frequency (kHz): %.05f" % I2S_ACTUAL_AUDIO_FREQ_KHZ)

print("---Configuration Information---")
print("I2SDIV: " + str(I2SDIV))
print("I2SODD: " + str(I2SODD))

print("---Error Information---")
print("Difference (kHz): %.05f" % ERROR_FREQ_KHZ)
print("Percentage (%%): %.05f" % ERROR_PERCENTAGE)

