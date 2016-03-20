/*******************************************************************************
* Copyright (c) 2014, Alan Barr
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <stdint.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "project.h"
#include "autogen_fir_coeffs.h"

/* I2S Setup:
 * Designed to 1 bit sample the MP45DT02 at 1024 kHz, with factor 64 decimation
 * to give 16 kHz PCM output. To achieve this the following values are set:
 *
 * In This File:
 *  I2SDIV = 42
 *  I2SODD = 0
 * Defined Externally:
 *  HSE Clock = 8 MHz
 *  PLLM = 8
 *  PLLI2SN = 258
 *  PLLI2SR = 3
*/

/* MP45DT02 setup:
 * CLK = PB10
 * PDM = PC3
 * MP45DT02 needs to be clocked between 1 and 3.25 MHz
 * L mode: Sample at rising edge.
 * R mode: Sample at falling edge.
 * */

/* TODO
 * Change PLL clock source in mcuconfig
 * Find out what the I2SPLL uses as its source-
 *      PLL is set to operate at 8 MHz / 8= 1 MHz*/

/*
 * 1024 kHz of I2S samples
 * 10 ms of samples:
 *  I2S buffer to hold 10240 bits (640 uint16_t's)
 *
 * But we will be given interrupts every 5ms:
 *  So 5120 bits/ 320 uint16_t's provided in 5ms
 *
 *  Need to extrapolate this data into floats, each float is 1 bit:
 *      5120 floats
 *
 *  Need to decimate this, factor = 64
 *      80 floats output (80 * 200 == 16000)
 *
 */

/*
https://github.com/rowol/stm32_discovery_arm_gcc/blob/7f565a4b02d2adb3d05d05055fad6f0b4c899bf2/STM32F4-Discovery_FW_V1.1.0/Project/Audio_playback_and_record/src/waverecorder.c
*/

#define MP45DT02_I2S_DRIVER                 I2SD2
#define MP45DT02_CLK_PORT                   GPIOB
#define MP45DT02_CLK_PAD                    10
#define MP45DT02_PDM_PORT                   GPIOC
#define MP45DT02_PDM_PAD                    3

#define MP45DT02_I2SDIV                     42
#define MP45DT02_I2SODD                     0
#define I2SODD_SHIFT                        8

#define MP45DT02_RAW_FREQ_KHZ               1024
/* One raw sample provided to processing.*/
#define MP45DT02_RAW_SAMPLE_DURATION_MS     2 

#define MP45DT02_I2S_WORD_SIZE_BITS         16
#define MP45DT02_I2S_SAMPLE_SIZE_BITS       (MP45DT02_RAW_FREQ_KHZ * MP45DT02_RAW_SAMPLE_DURATION_MS * MP45DT02_INTERRUPTS_PER_BUFFER)
#define MP45DT02_I2S_BUFFER_SIZE_2B         (MP45DT02_I2S_SAMPLE_SIZE_BITS / MP45DT02_I2S_WORD_SIZE_BITS)

/* Number of times interrupts are called when filling the buffer.
 * ChibiOS fires twice half full / full */
#define MP45DT02_INTERRUPTS_PER_BUFFER      2

/* Every bit in I2S signal needs to be expanded out into a word. */
/* Note: I2S Interrupts are always fired at half/full buffer point. Thus
 * processed buffers require to be 0.5 * bits in one I2S buffer */
#define MP45DT02_EXTRAPOLATED_BUFFER_SIZE   (MP45DT02_I2S_SAMPLE_SIZE_BITS/MP45DT02_INTERRUPTS_PER_BUFFER) /* AB TODO - 2 since buffer only half full  need to clean up defines..*/


#define MP45DT02_FIR_DECIMATION_FACTOR      64
#define MP45DT02_DECIMATED_BUFFER_SIZE      (MP45DT02_EXTRAPOLATED_BUFFER_SIZE/ MP45DT02_FIR_DECIMATION_FACTOR)

#define MP45DT02_OUTPUT_BUFFER_DURATION_MS  50

#define MP45DT02_OUTPUT_BUFFER_SIZE         (MP45DT02_DECIMATED_BUFFER_SIZE    * \
                                             MP45DT02_OUTPUT_BUFFER_DURATION_MS/ \
                                             MP45DT02_RAW_SAMPLE_DURATION_MS)

#define MEMORY_GUARD                        0xDEADBEEF

static thread_t * pMp45dt02ProcessingThd;
static THD_WORKING_AREA(mp45dt02ProcessingThdWA, 256);
static semaphore_t mp45dt02ProcessingSem;

static struct {
    uint32_t offset;
    uint32_t number;
    uint16_t buffer[MP45DT02_I2S_BUFFER_SIZE_2B];
    uint32_t guard;
} mp45dt02I2sData;

/* AB TODO better name. Holds DSP words with 1 bit samples. */
static float32_t mp45dt02ExtrapolatedBuffer[MP45DT02_EXTRAPOLATED_BUFFER_SIZE];
static uint32_t mp45dt02ExtrapolatedBufferSize = 0;

static float32_t mp45dt02DecimatedBuffer[MP45DT02_DECIMATED_BUFFER_SIZE];
static uint32_t mp45dt02DecimatedBufferSize = 0;

static I2SConfig mp45dt02I2SConfig;

static struct {
    time_measurement_t decimate;
    time_measurement_t extrapolation;
    time_measurement_t callback;
    time_measurement_t totalProcessing;
} debugTimings;

static struct {
    arm_fir_decimate_instance_f32 decimateInstance;
    float32_t state[FIR_COEFFS_LEN + MP45DT02_EXTRAPOLATED_BUFFER_SIZE - 1];
    uint32_t guard;
} cmsisDsp;

static struct {
    uint32_t count;
    float32_t buffer[MP45DT02_OUTPUT_BUFFER_SIZE];
    uint32_t guard;
} output;

static THD_FUNCTION(mp45dt02ProcessingThd, arg)
{
    (void)arg;

    uint32_t i = 0;
    uint32_t extrapolatedIndex = 0;
    uint32_t bitsInWord = 0;

    chRegSetThreadName("mp45dt02ProcessingThd");

    while (1)
    {
        chSemWait(&mp45dt02ProcessingSem);

        chTMStartMeasurementX(&debugTimings.totalProcessing);
        chTMStartMeasurementX(&debugTimings.extrapolation);

        memset(mp45dt02ExtrapolatedBuffer, 0, sizeof(mp45dt02ExtrapolatedBuffer));
        mp45dt02ExtrapolatedBufferSize = mp45dt02I2sData.number * MP45DT02_I2S_WORD_SIZE_BITS;

        if (mp45dt02ExtrapolatedBufferSize > MP45DT02_EXTRAPOLATED_BUFFER_SIZE)
        {
            PRINT_ERROR("Got more samples (%u) than expecting (%u)",
                        mp45dt02ExtrapolatedBufferSize,
                        MP45DT02_EXTRAPOLATED_BUFFER_SIZE);
        }

        if (mp45dt02ExtrapolatedBufferSize != MP45DT02_EXTRAPOLATED_BUFFER_SIZE)
        {
            PRINT_ERROR("Size wasn't as expected (%u) than expecting (%u)",
                        mp45dt02ExtrapolatedBufferSize,
                        MP45DT02_EXTRAPOLATED_BUFFER_SIZE);
        }

        /* Move each bit from each uint16_t word to uint16_t array element. */
        /* AB TODO assumption - data is MSB first - Soooo
         * We move least significant bits first. The are last sampled however,
         * and this we populate the end of the array first. */

        for(i=0, extrapolatedIndex = 0, bitsInWord = MP45DT02_I2S_WORD_SIZE_BITS;
            i < mp45dt02I2sData.number * MP45DT02_I2S_WORD_SIZE_BITS;
            i++)
        {
            uint16_t modifiedCurrentWord = 0;

            if (i % MP45DT02_I2S_WORD_SIZE_BITS)
            {
                modifiedCurrentWord =
                    mp45dt02I2sData.buffer[mp45dt02I2sData.offset +
                                           i/MP45DT02_I2S_WORD_SIZE_BITS];
            }

            /* bitsInWord only ever changed for final pass */
            if ((mp45dt02I2sData.number * MP45DT02_I2S_WORD_SIZE_BITS - i < MP45DT02_I2S_WORD_SIZE_BITS
                 && bitsInWord != MP45DT02_I2S_WORD_SIZE_BITS))
            {
                bitsInWord = mp45dt02I2sData.number * MP45DT02_I2S_WORD_SIZE_BITS - i;
                PRINT("bitsInWord is set to %u", bitsInWord);
            }

            extrapolatedIndex =
                /* Start Boundary */
                ((i / MP45DT02_I2S_WORD_SIZE_BITS) * MP45DT02_I2S_WORD_SIZE_BITS) +
                /* Offset, decrementing towards start boundary */
                                 ((bitsInWord-1) - (i % MP45DT02_I2S_WORD_SIZE_BITS));

            if (extrapolatedIndex > (mp45dt02ExtrapolatedBufferSize-1))
            {
                PRINT_ERROR("Overflow index (%u) greater than max array index (%u)",
                            extrapolatedIndex,
                            mp45dt02ExtrapolatedBufferSize);
            }

            mp45dt02ExtrapolatedBuffer[extrapolatedIndex] =
                (modifiedCurrentWord & 0x0001) * UINT16_MAX;

            modifiedCurrentWord >>= 1;
        }

        chTMStopMeasurementX(&debugTimings.extrapolation);

        if (chThdShouldTerminateX())
        {
            /* exit */
        }

        chTMStartMeasurementX(&debugTimings.decimate);

        /* Do DSP */
        arm_fir_decimate_f32(&cmsisDsp.decimateInstance,
                             mp45dt02ExtrapolatedBuffer,
                             mp45dt02DecimatedBuffer,
                             MP45DT02_EXTRAPOLATED_BUFFER_SIZE);
        chTMStopMeasurementX(&debugTimings.decimate);

#if 1
        memcpy(&output.buffer[output.count * MP45DT02_DECIMATED_BUFFER_SIZE],
               mp45dt02DecimatedBuffer,
               MP45DT02_DECIMATED_BUFFER_SIZE);

        output.count++;

        if (output.count == MP45DT02_OUTPUT_BUFFER_DURATION_MS / 
                            MP45DT02_RAW_SAMPLE_DURATION_MS)
        {
            i2sStopExchange(&MP45DT02_I2S_DRIVER);
            while (1)
            {
                LED_BLUE_TOGGLE();
                chThdSleepMilliseconds(500);
            }
        }
#endif

        /* Transmit */

        chTMStopMeasurementX(&debugTimings.totalProcessing);
    }
}
void HardFault_Handler(void) {
  while (true);
}

/* (*i2scallback_t) */
static void mp45dt02Cb(I2SDriver *i2sp, size_t offset, size_t number)
{
    (void)i2sp;

    chTMStartMeasurementX(&debugTimings.callback);

    chSysLockFromISR();
    mp45dt02I2sData.offset = offset;
    mp45dt02I2sData.number = number;
    chSemSignalI(&mp45dt02ProcessingSem);
    chSysUnlockFromISR();

    chTMStopMeasurementX(&debugTimings.callback);
}

void dspInit(void)
{
    arm_status armStatus;

    memset(&cmsisDsp, 0, sizeof(cmsisDsp));
    cmsisDsp.guard = MEMORY_GUARD;

    if (ARM_MATH_SUCCESS != (armStatus = arm_fir_decimate_init_f32(
                                            &cmsisDsp.decimateInstance,
                                            FIR_COEFFS_LEN,
                                            MP45DT02_FIR_DECIMATION_FACTOR,
                                            firCoeffs,
                                            cmsisDsp.state,
                                            MP45DT02_EXTRAPOLATED_BUFFER_SIZE)))
    {
        PRINT_ERROR("arm_fir_decimate_init_f32 failed with %d", armStatus);
    }
}

void mp45dt02Init(void)
{
    PRINT("Initialising mp45dt02.\n\r"
          "mp45dt02I2sData.buffer size: %u words %u bytes\n\r"
          "mp45dt02ExtrapolatedBuffer size: %u words %u bytes\n\r"
          "MP45DT02_DECIMATED_BUFFER_SIZE: %u",
          MP45DT02_I2S_BUFFER_SIZE_2B, sizeof(mp45dt02I2sData.buffer),
          MP45DT02_EXTRAPOLATED_BUFFER_SIZE, sizeof(mp45dt02ExtrapolatedBuffer),
          MP45DT02_DECIMATED_BUFFER_SIZE);

    chSemObjectInit(&mp45dt02ProcessingSem, 0);

    pMp45dt02ProcessingThd = chThdCreateStatic(mp45dt02ProcessingThdWA,
                                               sizeof(mp45dt02ProcessingThdWA),
                                               NORMALPRIO,
                                               mp45dt02ProcessingThd, NULL);

    dspInit();

    chTMObjectInit(&debugTimings.decimate);
    chTMObjectInit(&debugTimings.extrapolation);
    chTMObjectInit(&debugTimings.callback);
    chTMObjectInit(&debugTimings.totalProcessing);

    memset(&mp45dt02I2SConfig, 0, sizeof(mp45dt02I2SConfig));

    memset(&mp45dt02I2sData, 0, sizeof(mp45dt02I2sData));
    mp45dt02I2sData.guard = MEMORY_GUARD;

    output.guard = MEMORY_GUARD;

    /* ALAN TODO - move pin setup to board.h */
    palSetPadMode(MP45DT02_PDM_PORT, MP45DT02_PDM_PAD,
                  PAL_MODE_ALTERNATE(5)     |
                  PAL_STM32_OSPEED_HIGHEST);

    palSetPadMode(MP45DT02_CLK_PORT, MP45DT02_CLK_PAD,
                  PAL_MODE_ALTERNATE(5)     |
                  PAL_STM32_OTYPE_PUSHPULL  |
                  PAL_STM32_OSPEED_HIGHEST);

    mp45dt02I2SConfig.tx_buffer = NULL;
    mp45dt02I2SConfig.rx_buffer = mp45dt02I2sData.buffer;
    mp45dt02I2SConfig.size      = MP45DT02_I2S_BUFFER_SIZE_2B;
    mp45dt02I2SConfig.end_cb    = mp45dt02Cb;
    mp45dt02I2SConfig.i2scfgr   = SPI_I2SCFGR_I2SSTD_1 | SPI_I2SCFGR_CKPOL;
    mp45dt02I2SConfig.i2spr     = (SPI_I2SPR_I2SDIV & MP45DT02_I2SDIV) |
                                  (SPI_I2SPR_ODD & (MP45DT02_I2SODD << I2SODD_SHIFT));

    i2sStart(&MP45DT02_I2S_DRIVER, &mp45dt02I2SConfig);
    i2sStartExchange(&MP45DT02_I2S_DRIVER);
}

void mp45dt02Shutdown(void)
{
    i2sStopExchange(&MP45DT02_I2S_DRIVER);
    i2sStop(&MP45DT02_I2S_DRIVER);

    chThdTerminate(pMp45dt02ProcessingThd);
    chThdWait(pMp45dt02ProcessingThd);
    chSemReset(&mp45dt02ProcessingSem, 1);
    pMp45dt02ProcessingThd = NULL;
}

/* see https://github.com/hummels/libece486/blob/master/config_mp45dt02.c*/
#if 0

/**
  * @brief Test Micophone MEMS Hardware.
  *   The main objectif of this test is to check the hardware connection of the
  *   Microphone MEMS peripheral.
  * @param None
  * @retval None
  */
void Microphone_MEMS_Test(void)
{
  uint16_t data = 0x00;
  uint8_t index = 0x00;
  I2S_InitTypeDef  I2S_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the SPI clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MOSI_GPIO_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
  GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* Connect SPI pins to AF5 */
  GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);

  /* SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_PIN;
  GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);

  /* I2S configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI2);
  I2S_InitStructure.I2S_AudioFreq = 64000;
  I2S_InitStructure.I2S_Standard = I2S_Standard_MSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  /* Initialize the I2S peripheral with the structure above */
  I2S_Init(SPI2, &I2S_InitStructure);

  /* Enable the I2S peripheral */
  I2S_Cmd(SPI2, ENABLE);

  /* Waiting until MEMS microphone ready : Wake-up Time */
  Delay(10);

  TimingDelay = 500;
  /* Wait until detect the click on the MEMS microphone or TimeOut delay*/
  while((index < 30) && (TimingDelay != 0x00))
  {
    /* Waiting RXNE Flag or TimeOut delay */
    while((SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_RXNE) == RESET)&& (TimingDelay != 0x00))
    {}
    data = SPI_I2S_ReceiveData(SPI2);
    if (data == 0xFFFF)
    {
      index++;
    }
  }

  /* MEMS microphone test status: Timeout occurs */
  if(index != 30)
  {
    Fail_Handler();
  }
}






#############################################
/*!
 * @brief MP45DT02 Microphone SPI2 Initialization
 *
 * SPI2 is used to capture the 1-bit microphone data stream and to trigger
 * the processor to filter and decimate the data stream in 16-bit blocks.
 */
static void init_mp45dt02_spi(uint16_t Freq)
{
  I2S_InitTypeDef I2S_InitStructure;

  /* Enable the SPI clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

  /* SPI configuration */
  SPI_I2S_DeInit(SPI2);

  /*
   * OK...  Here's some bogus clock generation calculations:
   *
   * We're using the I2S clock to drive the clock terminal of the MIC.  The MIC
   * has a 1-bit output, which we'll be filtering and decimating by a factor of
   * 64 to create the output PCM sample stream.  So we need to generate a clock
   * of 64*(desired sample rate).
   *
   * Now, the I2S clock is generated by dividing down the PLLI2SCLK clock, which
   * is set up at 86 MHz in the Discovery board start-up routines.  The I2S_Init()
   * routine will determine and configure the best divisor to match up with
   * a requested I2S_AudioFreq value.  BUT: The I2S clock will be at 32*(I2S_Audio_Freq),
   * (NOT 64) so we need to set I2S_Audio_Freq=2(desired sample rate) in order to get
   * the right output clock rate.
   *
   * It gets a little worse:  The "Freq" parameter that comes into this routine
   * is the integer divisor which is needed to generate the desired sample rate
   * from the Timers... which are driven by an 84MHz clock (NOT the I2S 86 MHz
   * value!).  So:
   *    desired sample rate = 84000000/Freq          (Sample rate the caller is requesting)
   *    I2S_AudioFreq = 2(desired sample rate)       (Sample rate we request from I2S)
   *
   * The actual clock rate used to drive the MIC will be close (but not
   * exactly equal) to 32(I2S_Audio_Freq) = 64(desired_sample_rate).  The actual
   * value must be generated by an integer division of 86 MHz.  So the user WON'T
   * quite get the requested sample rate.  The 86 MHz reference was apparently chosen
   * so that the error is within the I2S specs for common audio sample rates.
   * Usually, the result is within a few Hz.  Here's a table of specific values:
   *
   *    desired sample rate     Actual Sample Rate     Percent Error
   *         50.0000                49.7685                0.463
   *         48.0000                47.9911                0.019
   *         32.0000                31.9940                0.019
   *         25.0000                24.8843                0.463
   *         24.0000                23.9955                0.019
   *         16.0000                15.9970                0.019
   *         12.0000                11.9978                0.019
   *           (All values are in kHz)
   *
   * One other note: Requesting sample rates above 50.78 kHz or below 15.625 kHz
   * will run the MP45DT02 out of spec, since it restricts the input clock range
   * to fall between 1 MHz and 3.25 MHz.
   *
   */

  I2S_InitStructure.I2S_AudioFreq = 2*84000000/Freq;

  I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  /* Initialize the I2S peripheral with the structure above */
  I2S_Init(SPI2, &I2S_InitStructure);

  /* Enable the Rx buffer not empty interrupt */
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

  /* Enable the SPI peripheral */
  I2S_Cmd(SPI2, ENABLE);
}




void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct)
{
  uint16_t tmpreg = 0, i2sdiv = 2, i2sodd = 0, packetlength = 1;
  uint32_t tmp = 0;
  RCC_ClocksTypeDef RCC_Clocks;
  uint32_t sourceclock = 0;

  /* Check the I2S parameters */
  assert_param(IS_SPI_23_PERIPH(SPIx));
  assert_param(IS_I2S_MODE(I2S_InitStruct->I2S_Mode));
  assert_param(IS_I2S_STANDARD(I2S_InitStruct->I2S_Standard));
  assert_param(IS_I2S_DATA_FORMAT(I2S_InitStruct->I2S_DataFormat));
  assert_param(IS_I2S_MCLK_OUTPUT(I2S_InitStruct->I2S_MCLKOutput));
  assert_param(IS_I2S_AUDIO_FREQ(I2S_InitStruct->I2S_AudioFreq));
  assert_param(IS_I2S_CPOL(I2S_InitStruct->I2S_CPOL));

/*----------------------- SPIx I2SCFGR & I2SPR Configuration -----------------*/
  /* Clear I2SMOD, I2SE, I2SCFG, PCMSYNC, I2SSTD, CKPOL, DATLEN and CHLEN bits */
  SPIx->I2SCFGR &= I2SCFGR_CLEAR_Mask;
  SPIx->I2SPR = 0x0002;

  /* Get the I2SCFGR register value */
  tmpreg = SPIx->I2SCFGR;

  /* If the default value has to be written, reinitialize i2sdiv and i2sodd*/
  if(I2S_InitStruct->I2S_AudioFreq == I2S_AudioFreq_Default)
  {
    i2sodd = (uint16_t)0;
    i2sdiv = (uint16_t)2;
  }
  /* If the requested audio frequency is not the default, compute the prescaler */
  else
  {
    /* Check the frame length (For the Prescaler computing) */
    if(I2S_InitStruct->I2S_DataFormat == I2S_DataFormat_16b)
    {
      /* Packet length is 16 bits */
      packetlength = 1;
    }
    else
    {
      /* Packet length is 32 bits */
      packetlength = 2;
    }

    /* I2S Clock source is System clock: Get System Clock frequency */
    RCC_GetClocksFreq(&RCC_Clocks);

    /* Get the source clock value: based on System Clock value */
    sourceclock = RCC_Clocks.SYSCLK_Frequency;

    /* Compute the Real divider depending on the MCLK output state with a flaoting point */
    if(I2S_InitStruct->I2S_MCLKOutput == I2S_MCLKOutput_Enable)
    {
      /* MCLK output is enabled */
      tmp = (uint16_t)(((((sourceclock / 256) * 10) / I2S_InitStruct->I2S_AudioFreq)) + 5);
    }
    else
    {
      /* MCLK output is disabled */
      tmp = (uint16_t)(((((sourceclock / (32 * packetlength)) *10 ) / I2S_InitStruct->I2S_AudioFreq)) + 5);
    }

    /* Remove the flaoting point */
    tmp = tmp / 10;

    /* Check the parity of the divider */
    i2sodd = (uint16_t)(tmp & (uint16_t)0x0001);

    /* Compute the i2sdiv prescaler */
    i2sdiv = (uint16_t)((tmp - i2sodd) / 2);

    /* Get the Mask for the Odd bit (SPI_I2SPR[8]) register */
    i2sodd = (uint16_t) (i2sodd << 8);
  }

  /* Test if the divider is 1 or 0 or greater than 0xFF */
  if ((i2sdiv < 2) || (i2sdiv > 0xFF))
  {
    /* Set the default values */
    i2sdiv = 2;
    i2sodd = 0;
  }

  /* Write to SPIx I2SPR register the computed value */
  SPIx->I2SPR = (uint16_t)(i2sdiv | (uint16_t)(i2sodd | (uint16_t)I2S_InitStruct->I2S_MCLKOutput));

  /* Configure the I2S with the SPI_InitStruct values */
  tmpreg |= (uint16_t)(SPI_I2SCFGR_I2SMOD | (uint16_t)(I2S_InitStruct->I2S_Mode | \
                  (uint16_t)(I2S_InitStruct->I2S_Standard | (uint16_t)(I2S_InitStruct->I2S_DataFormat | \
                  (uint16_t)I2S_InitStruct->I2S_CPOL))));

  /* Write to SPIx I2SCFGR */
  SPIx->I2SCFGR = tmpreg;
}
#endif
