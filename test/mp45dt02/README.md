# I2S Setup:

Designed to 1 bit sample the MP45DT02 at 1024 kHz, with factor 64 decimation
to give 16 kHz PCM output. To achieve this the following values are set:

## I2S Settings in src/mp45dt02_pdm.c:

    I2SDIV = 42
    I2SODD = 0
    
## I2S Settins in src/mcuconf.h:

    HSE Clock = 8 MHz
    PLLM      = 8
    PLLI2SN   = 258
    PLLI2SR   = 3

## STM32 pinout:

    MP45DT02 CLK = PB10
    MP45DT02 PDM = PC3

# Processing Notes: 
1. Configured to capture 2 ms of 1024 kHz I2S data between interrupts.
   I2S buffer needs to hold a total of 4 ms worth of data due to interrupts
   occuring at buffer half full.  
   I2S buffer to hold a total of 4096 1-bit samples (spread over 256
   uint16_t's).

2. When an interrupt occurs, each I2S sampled bit is extrapolated into floats
   (where each float represents 1 bit). This requires 2048 floats.

3. This is filtered and decimated by a factor of 64 to produce 32 floats.

4. These are added to the output buffer which stores 1 s of audio (16,000
   floats).

# Utils

## utils/fir_design.py

## utils/generate_tone_wave_file.py

## utils/signal_inspection_of_gdb_log.py

## utils/gdb_openocd.cfg

This is ideally run through GDB, using the commands in this file. It should be
configured to match OpenOCD's default port.
It can be sourced with `arm-none-eabi-gdb -x utils/gdb_openocd.cfg`



