# I2S Setup:

Designed to 1 bit sample the MP45DT02 at 1024 kHz, with factor 64 decimation
to give 16 kHz PCM output. To achieve this the following values are set:

## I2S Settings in src/mp45dt02_pdm.c:

    I2SDIV = 42
    I2SODD = 0
    
## I2S Settings in src/mcuconf.h:

    HSE Clock = 8 MHz
    PLLM      = 8
    PLLI2SN   = 258
    PLLI2SR   = 3

## STM32 pinout:

    MP45DT02 CLK = PB10
    MP45DT02 PDM = PC3

# Operation Notes: 

1. Configured to capture 2 ms of 1024 kHz I2S data between interrupts.
   I2S buffer needs to hold a total of 4 ms worth of data due to interrupts
   occuring at buffer half full.  
   I2S buffer to hold a total of 4096 1-bit samples (spread over 256
   uint16_t's).

2. When an I2S interrupt occurs, each I2S sampled bit is extrapolated into floats
   (where each float represents 1 bit). This requires 2048 floats.

3. This is filtered and decimated by a factor of 64 to produce 32 floats.

4. These are added to the output buffer which stores 1 s of audio (16,000
   floats).

5. A 1 second audio sample will be taken at first run, and with each subsequent
   press of the user button on the STM32F4DISCOVERY.

6. When run with the GDB script (see below) the audio sample will be extracted,
   and can be processed further.

# Utils

## utils/fir_design.py

Computes CMSIS compatible FIR filter coefficients.
The following configuration variables in the file are of note:

* `sampling_f` - Sampling frequency
* `cutoff_f` - Number of taps
* `taps_n` - Cutoff frequency

The output wave file can be found, relative to where the script was run:

* `./output/design/plots/fir.png` - FIR response
* `./output/design/files/*` - Generated C source and header file for CMSIS

## utils/generate_tone_wave_file.py

Generates a simple wave audio file, with a configurable number of
superimposed tones.

Tones can be changed by modifying the list `freqs`.

The output wave file can be found, relative to where the script was run,
`./output/test/audio/tones.wave`.

## utils/gdb_openocd.cfg

GDB commands to be used to run and extract data for testing purposes.

It creates an appropriate breakpoint in the code, which will be hit after a 1
second audio sample has been recorded.
After being hit, the the 1 second sample of audio being copied from the MCU.

It will produce a log file `gdb_output.txt`, relative to where GDB was started.

It can be sourced with `arm-none-eabi-gdb -x utils/gdb_openocd.cfg`.

## utils/signal_inspection_of_gdb_log.py

This is a script which is used to processes the recording previously extracted
from the MCU with GDB.
It will preform an FFT on the signal and also save it to a wave audio file.

It requires one argument, which is the gdb log file to be parsed.

The output from the script will be saved relative to where the script is run:

* `./output/inspection/audio/recorded.wave` - wave PCM audio of waveform
* `./output/inspection/plots/1_signal.png` - the waveform
* `./output/inspection/plots/2_signal_fft.png` - FFT of waveform



