#!/usr/bin/env bash
project_name="stm32f4_mic_eth"
if [ $(basename $PWD) != $project_name ]; then
    echo "Must be run from project root."
    exit 1
fi
project_root=.
chibios_path=${project_root}/lib/ChibiOS
proj_path=${project_root}/src
sys_path=/usr/lib/gcc/arm-none-eabi
files_list="${project_root}/cscope.files"

# arguments: path, list
get_c_files () 
{
    find $1 -name "*.[chxsS]" >> $2
}

search_chibios ()
{
    exclude_strings=( \
    SPC \
    SIMIA32 \
    LPC \
    OLIMEX \
    MSP430 \
    ST_STM32373C_EVAL \
    ST_STM3210C_EVAL \
    ST_NUCLEO \
    ST_STM8 \
    FREESCALE \
    ST_STM32L_DISCOVERY \
    STM32L \
    MAPLEMINI_STM32_F103 \
    ST_STM3210E_EVAL \
    ST_STM32VL_DISCOVERY \
    ST_STM3220G_EVAL \
    STM32F0 \
    STM32F1 \
    STM32F3 \
    STM32F429 \
    ARMCM3-GENERIC-KERNEL \
    Posix \
    ARMCM4-SAM4L \
    AVR \
    ARMCM3-STM32L152-DISCOVERY \
    Win32 \
    AVR \
    PPC \
    KINETIS \
    AT91SAM7 \
    IAR \
    RVCT \
    ARDUINO \
    git \
    testhal \
    template \
    GPIOv1 \
    SPIv2 )
#
    get_c_files $chibios_path $files_list

    for string in ${exclude_strings[@]}
    do 
        eval sed -i "/.*$string.*/d" $files_list 
        eval sed -i '/^$/d' $files_list
    done
}

search_proj () 
{
    get_c_files $proj_path $files_list
}

search_sys ()
{
    get_c_files $sys_path $files_list
}

rm -f $files_list
search_chibios
search_proj
search_sys
cscope -b -k
rm -f $files_list


