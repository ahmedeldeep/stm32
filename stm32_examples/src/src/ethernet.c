/*******************************************************************************
 * @file    ethernet.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    17 Jan 2020
 *          
 * @brief   Ethernet example
 * @note    
 *
@verbatim
Copyright (C) Almohandes.org, 2020

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
@endverbatim
*******************************************************************************/

/* Includes */
#include "ethernet.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup ethernet
 * @brief
 * @{
 */

/**
 * @defgroup ethernet_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ethernet_private_defines
 * @{
 */

/**
 * @brief   CRC stripping for Type frames
 */
#define ETH_MACCR_CSTF                      ((uint32_t)0x02000000)

/**
 * @brief   AF pin masks
 */
#define GPIO_AFRL_AFRL1                     ((uint32_t) 0x000000F0)
#define GPIO_AFRL_AFRL2                     ((uint32_t) 0x00000F00)
#define GPIO_AFRL_AFRL4                     ((uint32_t) 0x000F0000)
#define GPIO_AFRL_AFRL5                     ((uint32_t) 0x00F00000)
#define GPIO_AFRL_AFRL7                     ((uint32_t) 0xF0000000)
#define GPIO_AFRH_AFRH11                    ((uint32_t) 0x0000F000)
#define GPIO_AFRH_AFRH13                    ((uint32_t) 0x00F00000)

#define GPIO_AFRL_AFRL1_AF11                ((uint32_t) 0x000000B0)
#define GPIO_AFRL_AFRL2_AF11                ((uint32_t) 0x00000B00)
#define GPIO_AFRL_AFRL4_AF11                ((uint32_t) 0x000B0000)
#define GPIO_AFRL_AFRL5_AF11                ((uint32_t) 0x00B00000)
#define GPIO_AFRL_AFRL7_AF11                ((uint32_t) 0xB0000000)
#define GPIO_AFRH_AFRH11_AF11               ((uint32_t) 0x0000B000)
#define GPIO_AFRH_AFRH13_AF11               ((uint32_t) 0x00B00000)

/**
 * @brief   Common PHY Registers
 */
#define PHY_BCR                         ((uint8_t)0x00U)    /*!< Transceiver Basic Control Register   */
#define PHY_BSR                         ((uint8_t)0x01U)    /*!< Transceiver Basic Status Register    */

#define PHY_RESET                       ((uint16_t)0x8000U)  /*!< PHY Reset */
#define PHY_LOOPBACK                    ((uint16_t)0x4000U)  /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100U)  /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000U)  /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100U)  /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000U)  /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000U)  /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200U)  /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN                   ((uint16_t)0x0800U)  /*!< Select the power down mode           */
#define PHY_ISOLATE                     ((uint16_t)0x0400U)  /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020U)  /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS               ((uint16_t)0x0004U)  /*!< Valid link established               */
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002U)  /*!< Jabber condition detected            */

/**
 * @brief   Extended PHY Registers
 */

#define PHY_SR                          ((uint16_t)0x1FU)    /*!< PHY status register Offset                      */

#define PHY_SPEED_STATUS                ((uint16_t)0x0004U)  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t)0x0010U)  /*!< PHY Duplex mask                                 */

#define PHY_ISFR                        ((uint16_t)0x001DU)    /*!< PHY Interrupt Source Flag register Offset   */
#define PHY_ISFR_INT4                   ((uint16_t)0x000BU)  /*!< PHY Link down inturrupt       */

/*
  DMA Rx Descriptor
  --------------------------------------------------------------------------------------------------------------------
  RDES0 | OWN(31) |                                             Status [30:0]                                          |
  ---------------------------------------------------------------------------------------------------------------------
  RDES1 | CTRL(31) | Reserved[30:29] | Buffer2 ByteCount[28:16] | CTRL[15:14] | Reserved(13) | Buffer1 ByteCount[12:0] |
  ---------------------------------------------------------------------------------------------------------------------
  RDES2 |                                       Buffer1 Address [31:0]                                                 |
  ---------------------------------------------------------------------------------------------------------------------
  RDES3 |                          Buffer2 Address [31:0] / Next Descriptor Address [31:0]                             |
  ---------------------------------------------------------------------------------------------------------------------
*/

/**
  * @brief  Bit definition of RDES0 register: DMA Rx descriptor status register
  */
#define ETH_DMARXDESC_OWN         0x80000000U  /*!< OWN bit: descriptor is owned by DMA engine  */
#define ETH_DMARXDESC_AFM         0x40000000U  /*!< DA Filter Fail for the rx frame  */
#define ETH_DMARXDESC_FL          0x3FFF0000U  /*!< Receive descriptor frame length  */
#define ETH_DMARXDESC_ES          0x00008000U  /*!< Error summary: OR of the following bits: DE || OE || IPC || LC || RWT || RE || CE */
#define ETH_DMARXDESC_DE          0x00004000U  /*!< Descriptor error: no more descriptors for receive frame  */
#define ETH_DMARXDESC_SAF         0x00002000U  /*!< SA Filter Fail for the received frame */
#define ETH_DMARXDESC_LE          0x00001000U  /*!< Frame size not matching with length field */
#define ETH_DMARXDESC_OE          0x00000800U  /*!< Overflow Error: Frame was damaged due to buffer overflow */
#define ETH_DMARXDESC_VLAN        0x00000400U  /*!< VLAN Tag: received frame is a VLAN frame */
#define ETH_DMARXDESC_FS          0x00000200U  /*!< First descriptor of the frame  */
#define ETH_DMARXDESC_LS          0x00000100U  /*!< Last descriptor of the frame  */
#define ETH_DMARXDESC_IPV4HCE     0x00000080U  /*!< IPC Checksum Error: Rx Ipv4 header checksum error   */
#define ETH_DMARXDESC_LC          0x00000040U  /*!< Late collision occurred during reception   */
#define ETH_DMARXDESC_FT          0x00000020U  /*!< Frame type - Ethernet, otherwise 802.3    */
#define ETH_DMARXDESC_RWT         0x00000010U  /*!< Receive Watchdog Timeout: watchdog timer expired during reception    */
#define ETH_DMARXDESC_RE          0x00000008U  /*!< Receive error: error reported by MII interface  */
#define ETH_DMARXDESC_DBE         0x00000004U  /*!< Dribble bit error: frame contains non int multiple of 8 bits  */
#define ETH_DMARXDESC_CE          0x00000002U  /*!< CRC error */
#define ETH_DMARXDESC_MAMPCE      0x00000001U  /*!< Rx MAC Address/Payload Checksum Error: Rx MAC address matched/ Rx Payload Checksum Error */

/**
  * @brief  Bit definition of RDES1 register
  */
#define ETH_DMARXDESC_DIC   0x80000000U  /*!< Disable Interrupt on Completion */
#define ETH_DMARXDESC_RBS2  0x1FFF0000U  /*!< Receive Buffer2 Size */
#define ETH_DMARXDESC_RER   0x00008000U  /*!< Receive End of Ring */
#define ETH_DMARXDESC_RCH   0x00004000U  /*!< Second Address Chained */
#define ETH_DMARXDESC_RBS1  0x00001FFFU  /*!< Receive Buffer1 Size */

/**
  * @brief  Bit definition of RDES2 register
  */
#define ETH_DMARXDESC_B1AP  0xFFFFFFFFU  /*!< Buffer1 Address Pointer */

/**
  * @brief  Bit definition of RDES3 register
  */
#define ETH_DMARXDESC_B2AP  0xFFFFFFFFU  /*!< Buffer2 Address Pointer */

/*
   DMA Tx Descriptor
  -----------------------------------------------------------------------------------------------
  TDES0 | OWN(31) | CTRL[30:26] | Reserved[25:24] | CTRL[23:20] | Reserved[19:17] | Status[16:0] |
  -----------------------------------------------------------------------------------------------
  TDES1 | Reserved[31:29] | Buffer2 ByteCount[28:16] | Reserved[15:13] | Buffer1 ByteCount[12:0] |
  -----------------------------------------------------------------------------------------------
  TDES2 |                         Buffer1 Address [31:0]                                         |
  -----------------------------------------------------------------------------------------------
  TDES3 |                   Buffer2 Address [31:0] / Next Descriptor Address [31:0]              |
  -----------------------------------------------------------------------------------------------
*/

/**
  * @brief  Bit definition of TDES0 register: DMA Tx descriptor status register
  */
#define ETH_DMATXDESC_OWN                     0x80000000U  /*!< OWN bit: descriptor is owned by DMA engine */
#define ETH_DMATXDESC_IC                      0x40000000U  /*!< Interrupt on Completion */
#define ETH_DMATXDESC_LS                      0x20000000U  /*!< Last Segment */
#define ETH_DMATXDESC_FS                      0x10000000U  /*!< First Segment */
#define ETH_DMATXDESC_DC                      0x08000000U  /*!< Disable CRC */
#define ETH_DMATXDESC_DP                      0x04000000U  /*!< Disable Padding */
#define ETH_DMATXDESC_TTSE                    0x02000000U  /*!< Transmit Time Stamp Enable */
#define ETH_DMATXDESC_CIC                     0x00C00000U  /*!< Checksum Insertion Control: 4 cases */
#define ETH_DMATXDESC_CIC_BYPASS              0x00000000U  /*!< Do Nothing: Checksum Engine is bypassed */
#define ETH_DMATXDESC_CIC_IPV4HEADER          0x00400000U  /*!< IPV4 header Checksum Insertion */
#define ETH_DMATXDESC_CIC_TCPUDPICMP_SEGMENT  0x00800000U  /*!< TCP/UDP/ICMP Checksum Insertion calculated over segment only */
#define ETH_DMATXDESC_CIC_TCPUDPICMP_FULL     0x00C00000U  /*!< TCP/UDP/ICMP Checksum Insertion fully calculated */
#define ETH_DMATXDESC_TER                     0x00200000U  /*!< Transmit End of Ring */
#define ETH_DMATXDESC_TCH                     0x00100000U  /*!< Second Address Chained */
#define ETH_DMATXDESC_TTSS                    0x00020000U  /*!< Tx Time Stamp Status */
#define ETH_DMATXDESC_IHE                     0x00010000U  /*!< IP Header Error */
#define ETH_DMATXDESC_ES                      0x00008000U  /*!< Error summary: OR of the following bits: UE || ED || EC || LCO || NC || LCA || FF || JT */
#define ETH_DMATXDESC_JT                      0x00004000U  /*!< Jabber Timeout */
#define ETH_DMATXDESC_FF                      0x00002000U  /*!< Frame Flushed: DMA/MTL flushed the frame due to SW flush */
#define ETH_DMATXDESC_PCE                     0x00001000U  /*!< Payload Checksum Error */
#define ETH_DMATXDESC_LCA                     0x00000800U  /*!< Loss of Carrier: carrier lost during transmission */
#define ETH_DMATXDESC_NC                      0x00000400U  /*!< No Carrier: no carrier signal from the transceiver */
#define ETH_DMATXDESC_LCO                     0x00000200U  /*!< Late Collision: transmission aborted due to collision */
#define ETH_DMATXDESC_EC                      0x00000100U  /*!< Excessive Collision: transmission aborted after 16 collisions */
#define ETH_DMATXDESC_VF                      0x00000080U  /*!< VLAN Frame */
#define ETH_DMATXDESC_CC                      0x00000078U  /*!< Collision Count */
#define ETH_DMATXDESC_ED                      0x00000004U  /*!< Excessive Deferral */
#define ETH_DMATXDESC_UF                      0x00000002U  /*!< Underflow Error: late data arrival from the memory */
#define ETH_DMATXDESC_DB                      0x00000001U  /*!< Deferred Bit */

/**
  * @brief  Bit definition of TDES1 register
  */
#define ETH_DMATXDESC_TBS2  0x1FFF0000U  /*!< Transmit Buffer2 Size */
#define ETH_DMATXDESC_TBS1  0x00001FFFU  /*!< Transmit Buffer1 Size */

/**
  * @brief  Bit definition of TDES2 register
  */
#define ETH_DMATXDESC_B1AP  0xFFFFFFFFU  /*!< Buffer1 Address Pointer */

/**
  * @brief  Bit definition of TDES3 register
  */
#define ETH_DMATXDESC_B2AP  0xFFFFFFFFU  /*!< Buffer2 Address Pointer */


/**
 * @}
 */

/**
 * @defgroup ethernet_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ethernet_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ethernet_private_variables
 * @{
 */

static uint32_t txDescriptor[4] = {0};
static uint32_t rxDescriptor[4] = {0};

/**
 * @brief     Frame buffer
 * @note
 * @see
 */
typedef struct
{
  uint8_t dstAddr[6];
  uint8_t srcAddr[6];
  uint8_t type[2];
  uint8_t ipVer : 4;
  uint8_t ihl : 4;
  uint8_t dscp : 6;
  uint8_t ecn : 2;
  uint16_t totalLength;
  uint16_t id;
  uint16_t flags : 3;
  uint16_t frgOffset : 13;
  uint8_t ttl;
  uint8_t protocol;
  uint8_t headerChksum[2];
  uint8_t srcIP[4];
  uint8_t dstIP[4];
  uint8_t srcPort[2];
  uint8_t dstPort[2];
  uint8_t length[2];
  uint8_t chksum[2];
  uint8_t data[1538];
} eth_udp_frame_t;

static eth_udp_frame_t rxBuffer1;
static eth_udp_frame_t rxBuffer2;

/**
  * @brief  ARP announcement buffer size in bytes
  */
#define ARB_BUFF_SIZE  (uint32_t) 42

/**
  * @brief  UDP exchange buffer size in bytes
  */
#define UDP_BUFF_SIZE  (uint32_t) 86


static uint8_t txARPBuffer[ARB_BUFF_SIZE] =
{
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x80, 0xe1, 0x00,
  0x00, 0x00, 0x08, 0x06, 0x00, 0x01, 0x08, 0x00, 0x06, 0x04,
  0x00, 0x01, 0x00, 0x80, 0xe1, 0x00, 0x00, 0x00, 0xa9, 0xfe,
  0x34, 0xd2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa9, 0xfe,
  0x34, 0xd2
};

static uint8_t txUDPBuffer[UDP_BUFF_SIZE] =
{
  0x00, 0xe0, 0x4c, 0x68, 0x00, 0x03, 0x00, 0x80, 0xe1, 0x00,
  0x00, 0x00, 0x08, 0x00, 0x45, 0x00, 0x00, 0x47, 0x6d, 0xf1,
  0x00, 0x00, 0xff, 0x11, 0x00, 0x00, 0xa9, 0xfe, 0x34, 0xd2,
  0xa9, 0xf2, 0x34, 0xd0, 0xff, 0x00, 0xff, 0x00, 0x00, 0x33,
  0x00, 0x00, 0x45, 0x6d, 0x62, 0x65, 0x64, 0x64, 0x65, 0x64,
  0x20, 0x73, 0x79, 0x73, 0x74, 0x65, 0x6d, 0x20, 0x75, 0x73,
  0x69, 0x6e, 0x67, 0x20, 0x53, 0x54, 0x4d, 0x33, 0x32, 0x20,
  0x62, 0x79, 0x20, 0x41, 0x68, 0x6d, 0x65, 0x64, 0x20, 0x45,
  0x6c, 0x64, 0x65, 0x65, 0x70, 0x00
};

/**
 * @}
 */

/**
 * @defgroup ethernet_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ethernet_private_functions
 * @{
 */

/**
 * @brief   Read PHY register
 * @note
 * @param   none
 * @retval  none
 */
static uint16_t readPHYReg(uint8_t phy, uint8_t reg)
{
  /* Write PHY address */
  ETH->MACMIIAR |= (((uint32_t)phy << 11U) & ETH_MACMIIAR_PA);

  /* Write PHY register */
  ETH->MACMIIAR |= (((uint32_t)reg << 6U) & ETH_MACMIIAR_MR);

  /* Set the read mode */
  ETH->MACMIIAR &= ~ETH_MACMIIAR_MW;

  /* Set the MII Busy bit */
  ETH->MACMIIAR |= ETH_MACMIIAR_MB;

  /* Wait busy flag */
  while(ETH_MACMIIAR_MB == (ETH_MACMIIAR_MB & ETH->MACMIIAR))
  {

  }

  /* Read data */
  return (uint16_t)(ETH->MACMIIDR);

}

/**
 * @brief   Write PHY register
 * @note
 * @param   none
 * @retval  none
 */
static void writePHYReg(uint8_t phy, uint8_t reg, uint16_t data)
{
  /* Write PHY address */
  ETH->MACMIIAR |= (((uint32_t)phy << 11U) & ETH_MACMIIAR_PA);

  /* Write PHY register */
  ETH->MACMIIAR |= (((uint32_t)reg << 6U) & ETH_MACMIIAR_MR);

  /* Set write data */
  ETH->MACMIIDR = data;

  /* Set the write mode */
  ETH->MACMIIAR |= ETH_MACMIIAR_MW;

  /* Set the MII Busy bit */
  ETH->MACMIIAR |= ETH_MACMIIAR_MB;

  /* Wait busy flag */
  while(ETH_MACMIIAR_MB == (ETH_MACMIIAR_MB & ETH->MACMIIAR))
  {

  }
}

/**
 * @}
 */

/**
 * @defgroup ethernet_exported_functions
 * @{
 */

/**
 * @brief   Configure Ethernet GPIO
 * @note    PC1  --> ETH_MDC
 *          PA1  --> ETH_REF_CLK
 *          PA2  --> ETH_MDIO
 *          PA7  --> ETH_CRS_DV
 *          PC4  --> ETH_RXD0
 *          PC5  --> ETH_RXD1
 *          PB13 --> ETH_TXD1
 *          PG11 --> ETH_TX_EN
 *          PG13 --> ETH_TXD0
 * @param   none
 * @retval  none
 */
void ETH_GPIO_Init(void)
{
  /* Enable port A, B, C, G clock */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
      | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOGEN);

  /* Select alternate function mode */
  GPIOA->MODER &= ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER7);
  GPIOB->MODER &= ~(GPIO_MODER_MODER13);
  GPIOC->MODER &= ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
  GPIOG->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER13);

  GPIOA->MODER |= (GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER7_1);
  GPIOB->MODER |= (GPIO_MODER_MODER13_1);
  GPIOC->MODER |= (GPIO_MODER_MODER1_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
  GPIOG->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1);

  /* Select no pull */
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR7);
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR13);
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);
  GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR13);

  /* Select output speed very high */
  GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR1_0 | GPIO_OSPEEDER_OSPEEDR1_1
      | GPIO_OSPEEDER_OSPEEDR2_0 | GPIO_OSPEEDER_OSPEEDR2_1
      | GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR7_1);
  GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR13_0 | GPIO_OSPEEDER_OSPEEDR13_1);
  GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR1_0 | GPIO_OSPEEDER_OSPEEDR1_1
      | GPIO_OSPEEDER_OSPEEDR4_0 | GPIO_OSPEEDER_OSPEEDR4_1
      | GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR5_1);
  GPIOG->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR11_0 | GPIO_OSPEEDER_OSPEEDR11_1
      | GPIO_OSPEEDER_OSPEEDR13_0 | GPIO_OSPEEDER_OSPEEDR13_1);

  /* Select AF11 */
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL1 | GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL7);
  GPIOA->AFR[0] |= (GPIO_AFRL_AFRL1_AF11 | GPIO_AFRL_AFRL2_AF11 | GPIO_AFRL_AFRL7_AF11);
  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH13);
  GPIOB->AFR[1] |= (GPIO_AFRH_AFRH13_AF11);
  GPIOC->AFR[0] &= ~(GPIO_AFRL_AFRL1 | GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5);
  GPIOC->AFR[0] |= (GPIO_AFRL_AFRL1_AF11 | GPIO_AFRL_AFRL4_AF11 | GPIO_AFRL_AFRL5_AF11);
  GPIOG->AFR[1] &= ~(GPIO_AFRH_AFRH11 | GPIO_AFRH_AFRH13);
  GPIOG->AFR[1] |= (GPIO_AFRH_AFRH11_AF11 | GPIO_AFRH_AFRH13_AF11);

  /* Enable SYSCFG clock */
  RCC->APB2ENR |= RCC_APB2LPENR_SYSCFGLPEN;

  /* RMII PHY interface is selected */
  SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;

  /* Enable compensation cell */
  SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD;

  /* Wait till compensation cell is ready */
  while((SYSCFG->CMPCR & SYSCFG_CMPCR_READY) != SYSCFG_CMPCR_READY)
  {

  }
}

/**
 * @brief   Configure Ethernet MAC
 * @note
 * @param   none
 * @retval  none
 */
void ETH_MAC_Init(void)
{
  /* Enable Ethernet Transmission clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACTXEN;

  /* Enable Ethernet Reception clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACRXEN;

  /* Enable Ethernet MAC clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN;

  /* ******************************************* */

  /* Enable CRC stripping for Type frames */
  ETH->MACCR |= ETH_MACCR_CSTF;

  /* ******************************************* */

  /* Receive all frames*/
  ETH->MACFFR |= ETH_MACFFR_RA;

  /* ******************************************* */

  /* Clock range 100 150-168 MHz HCLK/102 */
  ETH->MACMIIAR &= ~ETH_MACMIIAR_CR;
  ETH->MACMIIAR |= ETH_MACMIIAR_CR_Div102;

  /* Put the PHY in reset mode */
  writePHYReg(0, PHY_BCR, PHY_RESET);

  /* PHY register read data */
  uint16_t phyRegReadData = 0;

  /* Wait until the link is up */
  do
  {
    /* Read PHY status register */
    phyRegReadData = readPHYReg(0, PHY_BSR);
  }
  while(PHY_LINKED_STATUS != ((PHY_LINKED_STATUS & phyRegReadData)));

  /* ******************************************* */

  /* Enable Auto-Negotiation */
  writePHYReg(0, PHY_BCR, PHY_AUTONEGOTIATION);

  /* Wait for auto-negotiation to completed */
  do
  {
    /* Read PHY status register */
    phyRegReadData = readPHYReg(0, PHY_BSR);
  }
  while(PHY_AUTONEGO_COMPLETE != ((PHY_AUTONEGO_COMPLETE & phyRegReadData)));

  /* Read PHY Special Control/Status Register */
  phyRegReadData = readPHYReg(0, PHY_SR);

  /* Check auto-negotiation duplex mode */
  if(PHY_DUPLEX_STATUS == ((PHY_DUPLEX_STATUS & phyRegReadData)))
  {
    /* Full-duplex */
    ETH->MACCR |= ETH_MACCR_DM;
  }
  else
  {
    /* Half-duplex */
    ETH->MACCR &= ~ETH_MACCR_DM;
  }

  /* Check auto-negotiation speed */
  if(PHY_SPEED_STATUS == ((PHY_SPEED_STATUS & phyRegReadData)))
  {
    /* 10M */
    ETH->MACCR &= ~ETH_MACCR_FES;
  }
  else
  {
    /* 100M */
    ETH->MACCR |= ETH_MACCR_FES;
  }

  /* ******************************************* */

  /* Write MAC address,
   * 00 80 e1 00 00 00 */
  ETH->MACA0HR = ((uint32_t) 0x00 << 8) | (uint32_t) 0x80;
  ETH->MACA0LR = ((uint32_t) 0xe1 << 24) | ((uint32_t) 0x00 << 16)
      | ((uint32_t) 0x00 << 8) | 0x00;

  /* ******************************************* */
}

/**
 * @brief   Configure Ethernet DMA
 * @note
 * @param   none
 * @retval  none
 */
void ETH_DMA_Init(void)
{
  /* Enable Receive store and forward */
  ETH->DMAOMR |= ETH_DMAOMR_RSF;

  /* Enable Transmit store and forward */
  ETH->DMAOMR |= ETH_DMAOMR_TSF;

  /* Write TX Descriptor */
  ETH->DMATDLAR = (uint32_t)txDescriptor;

  /* Write RX Descriptor */
  ETH->DMARDLAR = (uint32_t)rxDescriptor;
}

/**
 * @brief   Configure Ethernet DMA TX Descriptor List
 * @note
 * @param   none
 * @retval  none
 */
void ETH_DMA_TX_DescriptorList_Init(void)
{
  /* Set first and last segment */
  txDescriptor[0] |= ETH_DMATXDESC_FS | ETH_DMATXDESC_LS;

  /* Enable IP Header checksum and payload
   * checksum calculation and insertion */
  txDescriptor[0] |= ETH_DMATXDESC_CIC_TCPUDPICMP_FULL;

  /* Transmit end of ring */
  txDescriptor[0] |= ETH_DMATXDESC_TER;

  /* Transmit buffer 2 size */
  txDescriptor[1] &= ~ETH_DMATXDESC_TBS2;
  txDescriptor[1] |= (ARB_BUFF_SIZE << 16);

  /* Transmit buffer 1 size = 0 */
  txDescriptor[1] &= ~ETH_DMATXDESC_TBS1;

  /* TX Buffer address */
  txDescriptor[2] = 0;
  txDescriptor[3] = (uint32_t)txARPBuffer;

  /* Set own bit */
  txDescriptor[0] |= ETH_DMATXDESC_OWN;
}

/**
 * @brief   Configure Ethernet DMA RX Descriptor List
 * @note
 * @param   none
 * @retval  none
 */
void ETH_DMA_RX_DescriptorList_Init(void)
{
  /* Receive buffer 2 size */
  rxDescriptor[1] &= ~ETH_DMARXDESC_RBS2;
  rxDescriptor[1] |= (1024 << 16);

  /* Receive end of ring */
  rxDescriptor[1] |= ETH_DMARXDESC_RER;

  /* Receive buffer 1 size */
  rxDescriptor[1] &= ~ETH_DMARXDESC_RBS1;
  rxDescriptor[1] |= (1024 << 0);

  /* Receive buffer 1 address pointer */
  rxDescriptor[2] = (uint32_t)&rxBuffer1;

  /* Receive buffer 2 address pointer */
  rxDescriptor[3] = (uint32_t)&rxBuffer2;

  /* Set Own bit */
  rxDescriptor[0] |= ETH_DMARXDESC_OWN;
}

/**
 * @brief   Ethernet Transmit
 * @note
 * @param   none
 * @retval  none
 */
void ETH_Transmit(void)
{
  /* Check own bit */
  if(ETH_DMATXDESC_OWN != (ETH_DMATXDESC_OWN & txDescriptor[0]))
  {
    /* Transmit buffer 2 size */
    txDescriptor[1] &= ~ETH_DMATXDESC_TBS2;
    txDescriptor[1] |= (UDP_BUFF_SIZE << 16);

    /* TX Buffer address */
    txDescriptor[2] = 0;
    txDescriptor[3] = (uint32_t)txUDPBuffer;

    /* Increment last byte */
    txUDPBuffer[UDP_BUFF_SIZE-1]++;

    /* Set own bit */
    txDescriptor[0] |= ETH_DMATXDESC_OWN;
  }
  else
  {
    /* Owned by DMA */
  }

  /* When Tx Buffer unavailable flag is set,
   * clear it and resume transmission */
  if ((ETH->DMASR & ETH_DMASR_TBUS) == ETH_DMASR_TBUS)
  {
    /* Clear TBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_TBUS;

    /* Resume DMA transmission*/
    ETH->DMATPDR = 0U;
  }
}

/**
 * @brief   Ethernet Receive
 * @note
 * @param   none
 * @retval  none
 */
void ETH_Receive(void)
{
  /* Check own bit */
  if(ETH_DMARXDESC_OWN != (ETH_DMARXDESC_OWN & rxDescriptor[0]))
  {
    /* Check frame IPv4 & UDP & DA */
    if((rxBuffer1.dstAddr[0] == 0x00) && (rxBuffer1.dstAddr[1] == 0x80)
        && (rxBuffer1.dstAddr[2] == 0xe1) && (rxBuffer1.dstAddr[3] == 0x00)
        && (rxBuffer1.dstAddr[4] == 0x00) && (rxBuffer1.dstAddr[5] == 0x00)
        && (rxBuffer1.type[0] == 0x08) && (rxBuffer1.type[1] == 0x00)
        && (rxBuffer1.protocol == 17))
    {
      /* UDP frame received */
      ETH_Transmit();
    }
    else
    {
      /* Ignore other frames */
    }

    /* Set own bit */
    rxDescriptor[0] |= ETH_DMARXDESC_OWN;
  }
  else
  {
    /* Owned by DMA */
  }
}


/**
 * @brief   Configure Ethernet DMA
 * @note
 * @param   none
 * @retval  none
 */
void ETH_Enable(void)
{
  /* Transmitter enable */
  ETH->MACCR |= ETH_MACCR_TE;

  /* Receiver enable */
  ETH->MACCR |= ETH_MACCR_RE;

  /* Start DMA transmission */
  ETH->DMAOMR |= ETH_DMAOMR_ST;

  /* Start DMA receive */
  ETH->DMAOMR |= ETH_DMAOMR_SR;
}

/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */
