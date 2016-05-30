/*******************************************************************************
* Copyright (c) 2016, Alan Barr
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
#include "ch.h"
#include "project.h"
#include "lwipthread.h"
#include <string.h>
#include "lwip/ip_addr.h"
#include "lwip/api.h"
#include "lwip/err.h"

#if 0
/* Eth Pins 
* STM32F4 Pin  STM32F4 ETH Name                        LAN8720 Name      LAN8720 Pin
* PA1          ETH_MII _RX_CLK / ETH_RMII _REF_CLK     NINT/REFCLK0      14
* PA2          ETH _MDIO                               MDIO              12
* PA7          ETH_MII _RX_DV / ETH_RMII _CRS_DV       CRS_DV/MODE2      11
* PB11         ETH _MII_TX_EN / ETH _RMII_TX_EN        TXEN              16
* PB12         ETH _MII_TXD0 / ETH _RMII_TXD0          TXD0              17
* PB13         ETH _MII_TXD1 / ETH _RMII_TXD1          TXD1              18
* PC1          ETH _MDC                                MDC               13
* PC4          ETH_MII_RXD0 / ETH_RMII_RXD0            RXD0/MODE0         8
* PC5          ETH _MII_RXD1/ ETH _RMII_RXD1           RXD1/MODE1         7
* PE2          --                                      NRST              15
*/
#define LAN8720_REFCLK_PORT     GPIOA
#define LAN8720_REFCLK_PAD      1
#define LAN8720_MDIO_PORT       GPIOA
#define LAN8720_MDIO_PAD        2
#define LAN8720_CRS_DV_PORT     GPIOA
#define LAN8720_CRS_DV_PAD      7
#define LAN8720_TXEN_PORT       GPIOB
#define LAN8720_TXEN_PAD        11
#define LAN8720_TXD0_PORT       GPIOB
#define LAN8720_TXD0_PAD        12
#define LAN8720_TXD1_PORT       GPIOB
#define LAN8720_TXD1_PAD        13
#define LAN8720_MDC_PORT        GPIOC
#define LAN8720_MDC_PAD         1
#define LAN8720_RXD0_PORT       GPIOC
#define LAN8720_RXD0_PAD        4
#define LAN8720_RXD1_PORT       GPIOC
#define LAN8720_RXD1_PAD        5
#define LAN8720_NRST_PORT       GPIOE
#define LAN8720_NRST_PAD        2
#endif

#define IP(A,B,C,D)             htonl(A<<24 | B<<16 | C<<8 | D)
#define LAN8720_IPADDR          IP(192, 168, 1, 60)
#define LAN8720_GATEWAY         IP(192, 168, 1, 1)
#define LAN8720_NETMASK         IP(255, 255, 255, 0)

#define LAN8720_ETHADDR_0       0xC2
#define LAN8720_ETHADDR_1       0xAF
#define LAN8720_ETHADDR_2       0x51
#define LAN8720_ETHADDR_3       0x03
#define LAN8720_ETHADDR_4       0xCF
#define LAN8720_ETHADDR_5       0x46

#define LOCAL_PORT              50001

#if 0
void lan8720PreHalInit(void)
{
    uint32_t delay;
    /* Configure Eth Pins */
    palSetPadMode(LAN8720_REFCLK_PORT, LAN8720_REFCLK_PAD,
                  PAL_STM32_OTYPE_PUSHPULL |
                  PAL_MODE_ALTERNATE(11) |
                  PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LAN8720_MDIO_PORT, LAN8720_MDIO_PAD,
                  PAL_STM32_OTYPE_PUSHPULL |
                  PAL_MODE_ALTERNATE(11) |
                  PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LAN8720_CRS_DV_PORT, LAN8720_CRS_DV_PAD,
                  PAL_STM32_OTYPE_PUSHPULL |
                  PAL_MODE_ALTERNATE(11) |
                  PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LAN8720_TXEN_PORT, LAN8720_TXEN_PAD,
                  PAL_STM32_OTYPE_PUSHPULL |
                  PAL_MODE_ALTERNATE(11) |
                  PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LAN8720_TXD0_PORT, LAN8720_TXD0_PAD,
                  PAL_STM32_OTYPE_PUSHPULL |
                  PAL_MODE_ALTERNATE(11) |
                  PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LAN8720_TXD1_PORT, LAN8720_TXD1_PAD,
                  PAL_STM32_OTYPE_PUSHPULL |
                  PAL_MODE_ALTERNATE(11) |
                  PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LAN8720_MDC_PORT, LAN8720_MDC_PAD,
                  PAL_STM32_OTYPE_PUSHrePULL |
                  PAL_MODE_ALTERNATE(11) |
                  PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LAN8720_RXD0_PORT, LAN8720_RXD0_PAD,
                  PAL_STM32_OTYPE_PUSHPULL |
                  PAL_MODE_ALTERNATE(11) |
                  PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LAN8720_RXD1_PORT, LAN8720_RXD1_PAD,
                  PAL_STM32_OTYPE_PUSHPULL |
                  PAL_MODE_ALTERNATE(11) |
                  PAL_STM32_OSPEED_HIGHEST);
    /* Configure Reset Pin */
    palSetPadMode(LAN8720_NRST_PORT, LAN8720_NRST_PAD,
                  PAL_MODE_OUTPUT_PUSHPULL |
                  PAL_STM32_OSPEED_LOWEST);

    /* Reset Phy */
    /* AB TODO:
     * 1. What is the default clock speed?
     * 2. How to do this better so it wont get optimised out? */
    palClearPad(LAN8720_NRST_PORT, LAN8720_NRST_PAD);
    for(delay=0; delay<2000000; delay++);
    palSetPad(LAN8720_NRST_PORT, LAN8720_NRST_PAD);
}
#endif


void lan8720Init(void)
{
    struct lwipthread_opts opts;
    uint8_t optsMAC[6] = {LAN8720_ETHADDR_0,
                          LAN8720_ETHADDR_1,
                          LAN8720_ETHADDR_2,
                          LAN8720_ETHADDR_3,
                          LAN8720_ETHADDR_4,
                          LAN8720_ETHADDR_5};


    memset(&opts, 0, sizeof(opts));
    opts.macaddress = optsMAC;
    opts.address    = LAN8720_IPADDR;
    opts.netmask    = LAN8720_NETMASK;
    opts.gateway    = LAN8720_GATEWAY;

    lwipInit(&opts);
}



void lan8720Shutdown(void)
{


}


void lan8720TestUDP(void)
{
    err_t lwipErr = ERR_OK;
    struct netconn *udpConn = NULL;
    struct netbuf *udpSendBuf = NULL;
    struct netbuf *udpRecvBuf = NULL;
    uint8_t *payload = NULL;
    ip_addr_t remoteIp;
    uint16_t remotePort = 0; 
    uint32_t rxDataLength = 0;

    if (NULL == (udpConn = netconn_new(NETCONN_UDP)))
    {
        while(1);
    }
    
    if (ERR_OK != (lwipErr = netconn_bind(udpConn, IP_ADDR_ANY, LOCAL_PORT)))
    {
        PRINT("LWIP ERROR: %s", lwip_strerr(lwipErr));
        while(1);
    }

    while(1)
    {
        if (ERR_OK != (lwipErr = netconn_recv(udpConn, &udpRecvBuf)))
        {
            if (ERR_IS_FATAL(lwipErr))
            {
                while(1);
            }

            chThdSleep(MS2ST(2));
            continue;
        }

        remoteIp     = *netbuf_fromaddr(udpRecvBuf);
        remotePort   = netbuf_fromport(udpRecvBuf);
        rxDataLength = netbuf_len(udpRecvBuf);

        udpSendBuf = netbuf_new();

        if (NULL == (payload = netbuf_alloc(udpSendBuf, rxDataLength)))
        {
            while(1);
        }

        netbuf_copy(udpRecvBuf, payload, length);
    
        if (ERR_OK != (lwipErr = netconn_sendto(udpConn, udpSendBuf,
                                                &remoteIp, remotePort)))
        {
            PRINT("LWIP ERROR: %s", lwip_strerr(lwipErr));
            while(1);
        }

        netbuf_delete(udpSendBuf);
        netbuf_delete(udpRecvBuf);
    }
}


