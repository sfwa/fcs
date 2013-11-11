/*
 * Copyright (C) 2002-2012 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** ============================================================================
 *   @file  csl_emac.h
 *
 *   PATH:  \$(CSLPATH)\\inc
 *
 *   @brief  EMAC CSL Implementation on DSP side
 *
 *  NOTE:
 *   When used in an multitasking environment, no EMAC function may be
 *   called while another EMAC function is operating on the same device
 *   handle in another thread. It is the responsibility of the application
 *   to assure adherence to this restriction.
 *
 */

/* =============================================================================
 *  Revision History
 *  ===============
 *  07-Oct-05 XXX File created
 *  14-Mar-06 PSK Added multi core support
 *  22-May-07 PSK Removed multi core support to support new SoC
 *  16-July-2008 Update for Doxygen
 * =============================================================================
 */

/** @defgroup CSL_EMAC_API EMAC
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * The Ethernet Media Access Controller (EMAC) module provides an interface
 * between the C6457 DSP core processor and the networked community. The EMAC
 * supports 10Base-T (10 Mbits/second [Mbps]), and 100BaseTX (100 Mbps), in
 * either half- or full-duplex mode, and 1000BaseT (1000 Mbps) in full-duplex
 * mode, with hardware flow control and quality-of-service (QOS) support. There
 * are two ports of EMAC on C6457 DSP. The EMAC module conforms to the IEEE
 * 802.3-2002 standard, describing the "Carrier Sense Multiple Access with
 * Collision Detection (CSMA/CD) Access Method and Physical Layer"
 * specifications. The IEEE 802.3 standard has also been adopted by ISO/IEC and
 * re-designated as ISO/IEC 8802-3:2000(E). Deviation from this standard, the
 * EMAC module does not use the Transmit Coding Error signal MTXER. Instead of
 * driving the error pin when an underflow condition occurs on a transmitted
 * frame, the EMAC will intentionally generate an incorrect checksum by
 * inverting the frame CRC, so that the transmitted frame will be detected as
 * an error by the network.
 *
 * The EMAC control module is the main interface between the device core
 * processor, the MDIO module, and the EMAC module. The EMAC control module
 * contains the necessary components to allow the EMAC to make efficient use of
 * device memory, plus it controls device interrupts. The EMAC control module
 * incorporates 8K-bytes of internal RAM to hold EMAC buffer descriptors.
 *
 * @subsection References
 *   -# EMAC/MDIO User's Guide SPRUFC4.pdf (May 2008)
 *
 * @subsection Assumptions
 *    The abbreviations EMAC, emac and Emac have been used throughout this
 *    document to refer to Ethernet Media Access Controller and MDIO or mdio
 *    or Mdio refer to Management Data Input/Output.
 */

#ifndef CSL_EMAC_H
#define CSL_EMAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <csl.h>
#include <cslr_emac.h>
#include <cslr_ectl.h>
#include <csl_mdio.h>
#include <soc.h>

/**
@defgroup CSL_EMAC_SYMBOL  EMAC Symbols Defined
@ingroup CSL_EMAC_API
*/
/**
@defgroup CSL_EMAC_DATASTRUCT  EMAC Data Structures
@ingroup CSL_EMAC_API
*/
/**
@defgroup CSL_EMAC_FUNCTION  EMAC Functions
@ingroup CSL_EMAC_API
*/

/**************************************************************************\
* Peripheral Base Address
\**************************************************************************/
/** EMAC Module registers */
#define EMAC_REGS                 ((CSL_EmacRegs *)CSL_EMAC_0_REGS)

/** EMAC Control Module registers */
#define ECTL_REGS                 ((CSL_EctlRegs *)CSL_ECTL_0_REGS)


/**
@addtogroup CSL_EMAC_SYMBOL
@{
*/

/**
 * \brief EMACPktFlags Packet Buffer Flags set in Flags
 *
 *
 */
#define EMAC_PKT_FLAGS_SOP              0x80000000u
/**< Start of packet                                                        */
#define EMAC_PKT_FLAGS_EOP              0x40000000u
/**< End of packet                                                          */

/**
 *  \brief   RecvPktFlags    Receive Packet flags
 *
 * The Following Packet flags are set in Flags on RX packets only
 *
 */
#define EMAC_PKT_FLAGS_HASCRC           0x04000000u
/**< RxCrc: PKT has 4byte CRC                                               */
#define EMAC_PKT_FLAGS_JABBER           0x02000000u
/**< RxErr: Jabber                                                          */
#define EMAC_PKT_FLAGS_OVERSIZE         0x01000000u
/**< RxErr: Oversize                                                        */
#define EMAC_PKT_FLAGS_FRAGMENT         0x00800000u
/**< RxErr: Fragment                                                        */
#define EMAC_PKT_FLAGS_UNDERSIZED       0x00400000u
/**< RxErr: Undersized                                                      */
#define EMAC_PKT_FLAGS_CONTROL          0x00200000u
/**< RxCtl: Control Frame                                                   */
#define EMAC_PKT_FLAGS_OVERRUN          0x00100000u
/**< RxErr: Overrun                                                         */
#define EMAC_PKT_FLAGS_CODEERROR        0x00080000u
/**< RxErr: Code Error                                                      */
#define EMAC_PKT_FLAGS_ALIGNERROR       0x00040000u
/**< RxErr: Alignment Error                                                 */
#define EMAC_PKT_FLAGS_CRCERROR         0x00020000u
/**< RxErr: Bad CRC                                                         */
#define EMAC_PKT_FLAGS_NOMATCH          0x00010000u
/**< RxPrm: No Match                                                        */

/**
 *  \brief  CfgModeFlags Configuration Mode Flags
 *
 */
#define EMAC_CONFIG_MODEFLG_CHPRIORITY          0x00001
/**< Use Tx channel priority                                                */
#define EMAC_CONFIG_MODEFLG_MACLOOPBACK         0x00002
/**< MAC internal loopback                                                  */
#define EMAC_CONFIG_MODEFLG_RXCRC               0x00004
/**< Include CRC in RX frames                                               */
#define EMAC_CONFIG_MODEFLG_TXCRC               0x00008
/**< Tx frames include CRC                                                  */
#define EMAC_CONFIG_MODEFLG_PASSERROR           0x00010
/**< Pass error frames                                                      */
#define EMAC_CONFIG_MODEFLG_PASSCONTROL         0x00020
/**< Pass control frames                                                    */
#define EMAC_CONFIG_MODEFLG_PASSALL             0x00040
/**< pass all frames                                                        */
#define EMAC_CONFIG_MODEFLG_RXQOS               0x00080
/**< Enable QOS at receive side                                             */
#define EMAC_CONFIG_MODEFLG_RXNOCHAIN           0x00100
/**< Select no buffer chaining                                              */
#define EMAC_CONFIG_MODEFLG_RXOFFLENBLOCK       0x00200
/**< Enable offset/length blocking                                          */
#define EMAC_CONFIG_MODEFLG_RXOWNERSHIP         0x00400
/**< Use ownership bit as 1                                                 */
#define EMAC_CONFIG_MODEFLG_RXFIFOFLOWCNTL      0x00800
/**< Enable rx fifo flow control                                            */
#define EMAC_CONFIG_MODEFLG_CMDIDLE             0x01000
/**< Enable IDLE command                                                    */
#define EMAC_CONFIG_MODEFLG_TXSHORTGAPEN        0x02000
/**< Enable tx short gap                                                    */
#define EMAC_CONFIG_MODEFLG_TXPACE              0x04000
/**< Enable tx pacing                                                       */
#define EMAC_CONFIG_MODEFLG_TXFLOWCNTL          0x08000
/**< Enable tx flow control                                                 */
#define EMAC_CONFIG_MODEFLG_RXBUFFERFLOWCNTL    0x10000
/**< Enable rx buffer flow control                                          */
#define EMAC_CONFIG_MODEFLG_FULLDUPLEX          0x20000
/**< Set full duplex mode                                                   */
#define EMAC_CONFIG_MODEFLG_GIGABIT             0x40000
/**< Set gigabit                                                            */
#define EMAC_CONFIG_MODEFLG_EXTEN             0x80000
/**< Set external enable bit                                                */
#define EMAC_CONFIG_MODEFLAG_GIGFORCE           0x100000
/**< Set gigabit force mode */
#define EMAC_CONFIG_MODEFLG_GMIIEN              0x800000
/**< Set GMII mode*/


/**
 *  \brief  Descriptor memory selection Flags
 *
 */
#define EMAC_DESC_BASE_L2          0x00001
/**< Use L2 as Descriptor memory                                            */
#define EMAC_DESC_BASE_CPPI        0x00002
/**< Use CPPI RAM as desriptor memory                                       */
#define EMAC_DESC_BASE_DDR         0x00004
/**< Use DDR as descriptor memory                                           */




#define EMAC_TEARDOWN_CHANNEL(x)    (1 << x)
/**< Macro to tear down selective Rx/Tx channels */

/**
 * \brief  PktFiltering Packet Filtering
 *
 *  Packet Filtering Settings (cumulative)
 *
 */
#define EMAC_RXFILTER_NOTHING           0
/**< Receive filter set to Nothing                                          */
#define EMAC_RXFILTER_DIRECT            1
/**< Receive filter set to Direct                                           */
#define EMAC_RXFILTER_BROADCAST         2
/**< Receive filter set to Broadcast                                        */
#define EMAC_RXFILTER_MULTICAST         3
/**< Receive filter set to Multicast                                        */
#define EMAC_RXFILTER_ALLMULTICAST      4
/**< Receive filter set to All Mcast                                        */
#define EMAC_RXFILTER_ALL               5
/**< Receive filter set to All                                              */

/**
 *  \brief   ErrCodes    STANDARD ERROR CODES
 */
#define EMAC_ERROR_ALREADY              1
/**< Operation has already been started                                     */
#define EMAC_ERROR_NOTREADY             2
/**< Device is not open or not ready                                        */
#define EMAC_ERROR_DEVICE               3
/**< Device hardware error                                                  */
#define EMAC_ERROR_INVALID              4
/**< Function or calling parameter is invalid                               */
#define EMAC_ERROR_BADPACKET            5
/**< Supplied packet was invalid                                            */
#define EMAC_ERROR_MACFATAL             6
/**< Fatal Error - EMAC_close() required                                    */


#define EMAC_DEVMAGIC   0x0aceface
/**< Device Magic number                                                    */
#define EMAC_NUMSTATS   36
/**< Number of statistics regs                                              */
#define CSL_EMAC_NUMCHANNELS            8                   
/**< Number of Tx/Rx channels                                               */
#define CSL_EMAC_NUMCORES               1                   
/**< Number of cores                                                        */
#define CSL_EMAC_NUMMACADDRS            32                   
/**< Number of MAC addresses supported                                      */
/**
@}
*/


/** @addtogroup CSL_EMAC_DATASTRUCT
 @{ */

/**
 *  \brief  EMACObjects   EMAC Objects
 */

/**
 *  \brief  EMAC_Pkt
 *
 *  The packet structure defines the basic unit of memory used to hold data
 *  packets for the EMAC device.
 *
 *  A packet is comprised of one or more packet buffers. Each packet buffer
 *  contains a packet buffer header, and a pointer to the buffer data.
 *  The EMAC_Pkt structure defines the packet buffer header.
 *
 *  The pDataBuffer field points to the packet data. This is set when the
 *  buffer is allocated, and is not altered.
 *
 *  BufferLen holds the the total length of the data buffer that is used to
 *  store the packet (or packet fragment). This size is set by the entity
 *  that originally allocates the buffer, and is not altered.
 *
 *  The Flags field contains additional information about the packet
 *
 *  ValidLen holds the length of the valid data currently contained in the
 *  data buffer.
 *
 *  DataOffset is the byte offset from the start of the data buffer to the
 *  first byte of valid data. Thus (ValidLen+DataOffet)<=BufferLen.
 *
 *  Note that for receive buffer packets, the DataOffset field may be
 *  assigned before there is any valid data in the packet buffer. This allows
 *  the application to reserve space at the top of data buffer for private
 *  use. In all instances, the DataOffset field must be valid for all packets
 *  handled by EMAC.
 *
 *  The data portion of the packet buffer represents a packet or a fragment
 *  of a larger packet. This is determined by the Flags parameter. At the
 *  start of every packet, the SOP bit is set in Flags. If the EOP bit is
 *  also set, then the packet is not fragmented. Otherwise; the next packet
 *  structure pointed to by the pNext field will contain the next fragment in
 *  the packet. On either type of buffer, when the SOP bit is set in Flags,
 *  then the PktChannel, PktLength, and PktFrags fields must also be valid.
 *  These fields contain additional information about the packet.
 *
 *  The PktChannel field detetmines what channel the packet has arrived on,
 *  or what channel it should be transmitted on. The EMAC library supports
 *  only a single receive channel, but allows for up to eight transmit
 *  channels. Transmit channels can be treated as round-robin or priority
 *  queues.
 *
 *  The PktLength field holds the size of the entire packet. On single frag
 *  packets (both SOP and EOP set in BufFlags), PktLength and ValidLen will
 *  be equal.
 *
 *  The PktFrags field holds the number of fragments (EMAC_Pkt records) used
 *  to describe the packet. If more than 1 frag is present, the first record
 *  must have EMAC_PKT_FLAGS_SOP flag set, with corresponding fields validated.
 *  Each frag/record must be linked list using the pNext field, and the final
 *  frag/record must have EMAC_PKT_FLAGS_EOP flag set and pNext=0.
 *
 *  In systems where the packet resides in cacheable memory, the data buffer
 *  must start on a cache line boundary and be an even multiple of cache
 *  lines in size. The EMAC_Pkt header must not appear in the same cache line
 *  as the data portion of the packet. On multi-fragment packets, some packet
 *  fragments may reside in cacheable memory where others do not.
 *
 *  @verbatim
    <b> NOTE: It is up to the caller to assure that all packet buffers
    residing in cacheable memory are not currently stored in L1 or L2
    cache when passed to any EMAC function. </b>
    @endverbatim
 *
 *  Some of the packet Flags can only be set if the device is in the
 *  proper configuration to receive the corresponding frames. In order to
 *  enable these flags, the following modes must be set:
 *        RxCrc Flag  : RXCRC Mode in EMAC_Config
 *        RxErr Flags : PASSERROR Mode in EMAC_Config
 *        RxCtl Flags : PASSCONTROL Mode in EMAC_Config
 *        RxPrm Flag  : EMAC_RXFILTER_ALL in EMAC_setReceiveFilter()
 *
 */
typedef struct _EMAC_Pkt {
    Uint32           AppPrivate;
    /**< For use by the application                                         */
    struct _EMAC_Pkt *pPrev;
    /**< Previous record                                                    */
    struct _EMAC_Pkt *pNext;
    /**< Next record                                                        */
    Uint8            *pDataBuffer;
    /**< Pointer to Data Buffer (read only)                                 */
    Uint32           BufferLen;
    /**< Physical Length of buffer (read only)                              */
    Uint32           Flags;
    /**< Packet Flags                                                       */
    Uint32           ValidLen;
    /**< Length of valid data in buffer                                     */
    Uint32           DataOffset;
    /**< Byte offset to valid data                                          */
    Uint32           PktChannel;
    /**< Tx/Rx Channel/Priority 0-7 (SOP only)                              */
    Uint32           PktLength;
    /**< Length of Packet (SOP only) (same as ValidLen on single frag Pkt)  */
    Uint32           PktFrags;
    /**< No of frags in packet (SOP only) frag is EMAC_Pkt record-normally 1*/
} EMAC_Pkt;

/**
 *  \brief Packet Queue
 *
 * We keep a local packet queue for transmit and receive packets.
 * The queue structure is OS independent.
 */
typedef struct _pktq {
  Uint32             Count;
  /**< Number of packets in queue                                           */
  EMAC_Pkt          *pHead;
  /**< Pointer to first packet                                              */
  EMAC_Pkt          *pTail;
  /**< Pointer to last packet                                               */
} PKTQ;

/**
 *  \brief Transmit/Receive Descriptor Channel Structure
 *
 *  (One receive and up to 8 transmit in this example)
 */
typedef struct _EMAC_DescCh {
    struct _EMAC_Device *pd;
    /**< Pointer to parent structure                                        */
    PKTQ            DescQueue;
    /**< Packets queued as desc                                             */
    PKTQ            WaitQueue;
    /**< Packets waiting for TX desc                                        */
    Uint32          ChannelIndex;
    /**< Channel index 0-7                                                  */
    Uint32          DescMax;
    /**< Max number of desc (buffs)                                         */
    Uint32          DescCount;
    /**< Current number of desc                                             */
    EMAC_Desc       *pDescFirst;
    /**< First desc location                                                */
    EMAC_Desc       *pDescLast;
    /**< Last desc location                                                 */
    EMAC_Desc       *pDescRead;
    /**< Location to read next desc                                         */
    EMAC_Desc       *pDescWrite;
    /**< Location to write next desc                                        */
} EMAC_DescCh;


/**
 *  \brief Transmit/Receive Channel info Structure
 *
 *  (one receive and up to 8 transmit per core in this example)
 */
typedef struct _EMAC_ChannelInfo {
    Uint32    TxChanEnable;
    /**< Which specific Tx Channels(0-7) to use
      * if TxChannels = 0 does not allocate the Tx channels for the core
      * if TxChannels = 1, 2, 4, 8, ... allocates which specific TxChannels to use
	  * 0x01: channel 0, 0x02: channel 1, 0x04: channel 2, 0x80: channel 7
      * User has to take care of allocating a portion from total allocation
      * for the cores */
    Uint32    RxChanEnable;
    /**< Which specific Rx Channels(0-7) to use
      *  if RxChannel = 0 does not allocate the Rx channel for the core
      *  if RxChannel = 1, 2,4,8,... which specific channel to use for the core
	  * 0x01: channel 0, 0x02: channel 1, 0x04: channel 2, 0x80: channel 7 */
} EMAC_ChannelInfo;

/**
 *  \brief MAC addresses configuration Structure
 *
 */
typedef struct _EMAC_AddrConfig {
	Uint8   ChannelNum;
	/* Receive Channel number to which the MAC address to be assigned */
	Uint8 Addr[6];
	/* MAC address specific to channel */
} EMAC_AddrConfig;

/**
 *  \brief  EMAC_Core_Config
 *
 *  The EMAC_Core_Config structure defines configurations specific to 
 *  individual cores when the EMAC device is operating. It is passed to the 
 *  device when the device is initialized for individual cores (EMAC_coreInit()),
 *  and remains in effect until the device is de-initialized for the corresponding
 *  cores (EMAC_coreDeInit()).
 *
 *  The following is a short description of the configuration fields:
 *
 *  A list of callback functions is used to register callback functions with
 *  a particular core the EMAC peripheral instance. Callback functions are
 *  used by EMAC to communicate with the application. These functions are
 *  REQUIRED for operation. The same callback table can be used for multiple
 *  cores and/or multiple driver instances.
 *
 *  <b>pfcbGetPacket </b> -  Called by EMAC to get a free packet buffer from
 *                   the application layer for receive data. This function
 *                   should return NULL is no free packets are available.
 *                   The size of the packet buffer must be large enough
 *                   to accommodate a full sized packet (1514 or 1518
 *                   depending on the EMAC_CONFIG_MODEFLG_RXCRC flag), plus
 *                   any application buffer padding (DataOffset).
 *
 *  <b>pfcbFreePacket </b> - Called by EMAC to give a free packet buffer back
 *                   to the application layer. This function is used to
 *                   return transmit packets. Note that at the time of the
 *                   call, structure fields other than pDataBuffer and
 *                   BufferLen are in an undefined state.
 *
 *  <b>pfcbRxPacket </b>   - Called to give a received data packet to the
 *                   application layer. The applicaiton must accept the packet.
 *                   When the application is finished with the packet, it
 *                   can return it to its own free queue. This function also
 *                   returns a pointer to a free packet to replace the received
 *                   packet on the EMAC free list. It returns NULL when no free
 *                   packets are available. The return packet is the same as
 *                   would be returned by pfcbGetPacket. Thus if a newly
 *                   received packet is not desired, it can simply be returned
 *                   to EMAC via the return value.
 *
 *  <b>pfcbStatus </b>     - Called to indicate to the application that it
 *                   should call EMAC_getStatus() to read the current
 *                   device status. This call is made when device status
 *                   changes.
 *
 *  <b>pfcbStatistics </b> - Called to indicate to the application that it
 *                   should call EMAC_getStatistics() to read the
 *                   current Ethernet statistics. Called when the
 *                   statistic counters are to the point of overflow.
 *
 *  The hApplication calling argument is the application's handle as supplied to 
 *  the EMAC device for individual cores in the EMAC_coreInit() function.
 *
 *  DescBase      - Descriptor memory selction to place the EMAC descriptors
 *
 *  ChannelInfo   - Tx and Rx Channel information to be used by the core
 *
 *  NumOfMacAddrs - Number of MAC addresses to be assigned to the core
 *
 *  MacAddr       - Device MAC addresses for the core
 *
 *  RxMaxPktPool  - Max Rx packet buffers to get from pool
 *                  (Must be in the range of 8 to 192)
 */

typedef struct _EMAC_Core_Config {
    EMAC_Pkt *  (*pfcbGetPacket)(Handle hApplication);
    /**< Get packet call back                                               */
    void        (*pfcbFreePacket)(Handle hApplication, EMAC_Pkt *pPacket);
    /**< Free packet call back                                              */
    EMAC_Pkt *  (*pfcbRxPacket)(Handle hApplication, EMAC_Pkt *pPacket);
    /**< Receive packet call back                                           */
    void        (*pfcbStatus)(Handle hApplication);
    /**< Get status call back                                               */
    void        (*pfcbStatistics)(Handle hApplication);
    /**< Get statistics call back                                           */
    Uint8       DescBase;
    /**< This member is for descriptor memory selction to place the EMAC
	     descriptors in CPPI RAM or  L2 RAM or DDR memory */
    EMAC_ChannelInfo ChannelInfo;
    /**< Tx and Rx Channel information for individual cores to use          */
	Uint8       NumOfMacAddrs;
	/**< Number of MAC addresses to be assigned for individual cores        */
    EMAC_AddrConfig **MacAddr;
    /**< Mac Addresses structure                                            */
    Uint32      RxMaxPktPool;
    /**< Max Rx packet buffers to get from pool                             */
} EMAC_Core_Config;

/**
 *  \brief  EMAC_Common_Config
 *
 *  The EMAC_Common_Config structure defines configurations common to all the cors
 *  when the EMAC device is operating. It is passed to the device when the device is 
 *  initialized one time by the master core (EMAC_commonInit()), and remains in effect 
 *  until the device is de-initialized by the master core (EMAC_commonDeInit()).
 *
 *  The following is a short description of the configuration fields:
 *
 *  UseMdio      - Uses MDIO configuration if required. In case of SGMII
 *                 MAC to MAC communication MDIO is not required. If this 
 *                 field is one (1) configures MDIO
 *                          zero (0) does not configure MDIO
 *
 *  ModeFlags    - Specify the Fixed Operating Mode of the Device:
 *      - EMAC_CONFIG_MODEFLG_CHPRIORITY  - Treat TX channels as Priority Levels
 *                                   (Channel 7 is highest, 0 is lowest)
 *      - EMAC_CONFIG_MODEFLG_MACLOOPBACK - Set MAC in Internal Loopback for
 *                                          Testing
 *      - EMAC_CONFIG_MODEFLG_RXCRC       - Include the 4 byte EtherCRC in RX
 *                                          frames
 *      - EMAC_CONFIG_MODEFLG_TXCRC       - Assume TX Frames Include 4 byte
 *                                          EtherCRC
 *      - EMAC_CONFIG_MODEFLG_PASSERROR   - Receive Error Frames for Testing
 *      - EMAC_CONFIG_MODEFLG_PASSCONTROL - Receive Control Frames for
 *                                          Testing
 *
 *  MdioModeFlags - Specify the MDIO/PHY Operation (See csl_MDIO.H)
 *
 *  CoreNum       - Specify the master core which does common initialization
 *                  and de-initialization
 *
 *  PktMTU        - Specify the maximal physical packet size
 */
typedef struct _EMAC_Common_Config {
    Uint8       UseMdio;
    /**< MDIO Configuation select. User has to pass one (1) if MDIO 
         Configuration  is needed, if not should pass zero (0)              */
    Uint32      ModeFlags;
    /**< Configuation Mode Flags                                            */
    Uint32      MdioModeFlags;
    /**< CSL_MDIO Mode Flags (see CSL_MDIO.H)                               */
    Uint32      MdioPhyAddr;
    /**< PHY address (0-31) to be monitored by MDIO, specified by user
         when MDIO_MODEFLG_SPECPHYADDR is set in MdioModeFlags              */
    Uint8       CoreNum;
    /**< This member is for core selction to does the EMAC configuration
          i.e user can select the specific core to configure EMAC one time  */
    Uint32      PktMTU;
    /**< Max physical packet size                                           */
} EMAC_Common_Config;


/**
 *  \brief  EMAC_Config
 *
 *  The EMAC_Config structure defines how the EMAC device should operate. It is
 *  passed to the device when the device is opened (EMAC_open()), and remains 
 *  in effect until the device is closed (EMAC_close).
 *
 *  The following is a short description of the configuration fields:
 *
 *  EMACCommonConfig - Specify configurations common to all the cores
 *
 *  EMACCoreConfig   - Specify configurations specific to individual cores
*/
typedef struct _EMAC_Config {
    EMAC_Common_Config EMACCommonConfig;
    /**< Configurations common to all the cores                             */
    EMAC_Core_Config EMACCoreConfig;
    /**< Configurations specific to individual cores                        */
} EMAC_Config;

/**
 *  \brief  EMAC_Status
 *
 *  The status structure contains information about the MAC's run-time
 *  status.
 *
 *  The following is a short description of the configuration fields:
 *
 *  MdioLinkStatus - Current link stat (non-zero on link; see CSL_MDIO.H)
 *
 *  PhyDev         - Current PHY device in use (0-31)
 *
 *  RxPktHeld      - Current number of Rx packets held by the EMAC device
 *
 *  TxPktHeld      - Current number of Tx packets held by the EMAC device
 *
 *  FatalError     - Fatal Error Code (TBD)
 */
typedef struct _EMAC_Status {
    Uint32      MdioLinkStatus;
    /**< CSL_MDIO Link status (see csl_mdio.h)                              */
    Uint32      PhyDev;
    /**< Current PHY device in use (0-31)                                   */
    Uint32      RxPktHeld;
    /**< Number of packets held for Rx                                      */
    Uint32      TxPktHeld;
    /**< Number of packets held for Tx                                      */
    Uint32      FatalError;
    /**< Fatal Error when non-zero                                          */
} EMAC_Status;


/**
 *  \brief  EMAC_Statistics
 *
 *  The statistics structure is the used to retrieve the current count
 *  of various packet events in the system. These values represent the
 *  delta values from the last time the statistics were read.
 */
typedef struct _EMAC_Statistics {
    Uint32 RxGoodFrames;     /**< Good Frames Received                      */
    Uint32 RxBCastFrames;    /**< Good Broadcast Frames Received            */
    Uint32 RxMCastFrames;    /**< Good Multicast Frames Received            */
    Uint32 RxPauseFrames;    /**< PauseRx Frames Received                   */
    Uint32 RxCRCErrors;      /**< Frames Received with CRC Errors           */
    Uint32 RxAlignCodeErrors;/**< Frames Received with Alignment/Code Errors*/
    Uint32 RxOversized;      /**< Oversized Frames Received                 */
    Uint32 RxJabber;         /**< Jabber Frames Received                    */
    Uint32 RxUndersized;     /**< Undersized Frames Received                */
    Uint32 RxFragments;      /**< Rx Frame Fragments Received               */
    Uint32 RxFiltered;       /**< Rx Frames Filtered Based on Address       */
    Uint32 RxQOSFiltered;    /**< Rx Frames Filtered Based on QoS Filtering */
    Uint32 RxOctets;         /**< Total Received Bytes in Good Frames       */
    Uint32 TxGoodFrames;     /**< Good Frames Sent                          */
    Uint32 TxBCastFrames;    /**< Good Broadcast Frames Sent                */
    Uint32 TxMCastFrames;    /**< Good Multicast Frames Sent                */
    Uint32 TxPauseFrames;    /**< PauseTx Frames Sent                       */
    Uint32 TxDeferred;       /**< Frames Where Transmission was Deferred    */
    Uint32 TxCollision;      /**< Total Frames Sent With Collision          */
    Uint32 TxSingleColl;     /**< Frames Sent with Exactly One Collision    */
    Uint32 TxMultiColl;      /**< Frames Sent with Multiple Colisions       */
    Uint32 TxExcessiveColl;  /**< Tx Frames Lost Due to Excessive Collisions*/
    Uint32 TxLateColl;       /**< Tx Frames Lost Due to a Late Collision    */
    Uint32 TxUnderrun;       /**< Tx Frames Lost with Tx Underrun Error     */
    Uint32 TxCarrierSLoss;   /**< Tx Frames Lost Due to Carrier Sense Loss  */
    Uint32 TxOctets;         /**< Total Transmitted Bytes in Good Frames    */
    Uint32 Frame64;          /**< Total Tx&Rx with Octet Size of 64         */
    Uint32 Frame65t127;      /**< Total Tx&Rx with Octet Size of 65 to 127  */
    Uint32 Frame128t255;     /**< Total Tx&Rx with Octet Size of 128 to 255 */
    Uint32 Frame256t511;     /**< Total Tx&Rx with Octet Size of 256 to 511 */
    Uint32 Frame512t1023;    /**< Total Tx&Rx with Octet Size of 512 to 1023*/
    Uint32 Frame1024tUp;     /**< Total Tx&Rx with Octet Size of >=1024     */
    Uint32 NetOctets;        /**< Sum of all Octets Tx or Rx on the Network */
    Uint32 RxSOFOverruns;    /**< Total Rx Start of Frame Overruns          */
    Uint32 RxMOFOverruns;    /**< Total Rx Middle of Frame Overruns         */
    Uint32 RxDMAOverruns;    /**< Total Rx DMA Overruns                     */
} EMAC_Statistics;

/**
 *  \brief  EMAC Core Instance Structure
 *
 */
typedef struct _EMAC_Core {
    Handle          hApplication;
    /**< Calling Application's Handle                  */
    EMAC_DescCh     RxCh[CSL_EMAC_NUMCHANNELS];
    /**< Receive channel status                                             */
    EMAC_DescCh     TxCh[CSL_EMAC_NUMCHANNELS];
    /**< Transmit channel status                                            */
} EMAC_Core;

/**
 *  \brief  EMAC Main Device Instance Structure
 *
 */
typedef struct _EMAC_Device {
    CSL_EmacRegs*   emacRegs;
    /**< Pointer to the register overlay structure of the EMAC              */
    CSL_EctlRegs*   ectlRegs;
    /**< Pointer to the register overlay structure of the ECTL              */
    CSL_InstNum     perNum;
    /**< Instance of EMAC being referred by this object                     */
    Uint32          DevMagic;
    /**< Magic ID for this instance                                         */
    MDIO_Device     MdioDev;
    /**< MDIO device structure                                              */
    Uint32          RxFilter;
    /**< Current RX filter value                                            */
    Uint32          PktMTU;
    /**< Max physical packet size                                           */
    Uint32          MacHash1;
    /**< Hash value cache                                                   */
    Uint32          MacHash2;
    /**< Hash value cache                                                   */
    Uint32          FatalError;
    /**< Fatal Error Code                                                   */
    EMAC_Common_Config     Config;
    /**< Original User Configuration common to all cores                    */
    EMAC_Core_Config       CoreConfig[CSL_EMAC_NUMCORES];
    /**< Original User Configuration specific to individaul cores           */
    EMAC_Statistics Stats;
    /**< Current running statistics                                         */
    Uint8           InitOnce;
    /**< EMAC common configuration has been initialized once                */
    Uint8           MacIndexUsed[CSL_EMAC_NUMMACADDRS];
    /**< Store core ID which is using each mac index                        */
	EMAC_Core*      pEMACCore[CSL_EMAC_NUMCORES];
    /**< Array of pointers to EMAC Core Instance for individual cores       */
} EMAC_Device;

/* @} */

/** @addtogroup CSL_EMAC_FUNCTION
 @{ */

/**
 *  \brief  EMACInterfaces   EMAC Interfaces
 *
 *  Interfaces for EMAC module.
 *
 *  \par
 *  <b> Note: </b>The application is charged with verifying that only
 *  one of the following API calls may be executing at a given time across all
 *  threads and all interrupt functions.
 *  \par
 *  Hence, when used in an multitasking environment, no EMAC function may be
 *  called while another EMAC function is operating on the same device
 *  handle in another thread. It is the responsibility of the application
 *  to assure adherence to this restriction.
 *
 */

/** ============================================================================
 *  @n@b  EMAC_enumerate()
 *
 *  @b Description
 *  @n Enumerates the EMAC peripherals installed in the system and returns an
 *     integer count. The EMAC devices are enumerated in a consistent
 *     fashion so that each device can be later referenced by its physical
 *     index value ranging from "1" to "n" where "n" is the count returned
 *     by this function.
 *
 *  <b> Return Value </b>  None
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  None
 *
 *  @b Example
 *  @verbatim
        EMAC_enumerate( );
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_enumerate( void );

/** ============================================================================
 *  @n@b  EMAC_open()
 *
 *  @b Description
 *  @n Opens the EMAC peripheral at the given physical index and initializes
 *     it to an embryonic state.
 *
 *     The calling application must supply operating configurations that
 *     includes the ones common to all the cores if on the master core and 
 *     also those specific to individual cores. The EMAC device must be closed
 *     and then re-opened when new configuration is required.
 *
 *     The application layer may pass in an hApplication callback handle, that 
 *     will be supplied by the EMAC device for the core when making calls to the
 *     core's application callback functions.
 *
 *     A valid EMAC device handle should be passed to this API to which the
 *     configuration and operating state of the EMAC device will be written.
 *
 *     A valid EMAC core instance handle should be passed to this API to which the
 *     operating state of the EMAC device specific to individual cores will be written.
 *
 *     The default receive filter prevents normal packets from being received
 *     until the receive filter is specified by calling EMAC_receiveFilter().
 *
 *     By calling EMAC_close() followed by EMAC_open() on the master core, 
 *     EMAC device reset is achieved. When the same sequence is called on a
 *     non-master core, a core reset for EMAC operation is achieved
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error codes include:
 *       EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        instNum         EMAC Peripheral ID to identify the EMAC controller to be'
                        initialized.
        hApplication    Application handle
        pEMACConfig     EMAC's configuration structure
        hEMAC           Handle to the EMAC device which needs to be initialized.
        hCore           Handle to the EMAC core instance which needs to be initialized.
    @endverbatim
 *
 *
 *  <b> Return Value </b>  Success (0)
 *  @n     EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  Opens the EMAC peripheral at the given physical index and initializes it.
 *
 *  @b Example
 *  @verbatim
 
        #define MASTER_CORE 0           // Which core is responsible for EMAC common initialization

        EMAC_Device            EMACObj; // In multi-core scenario, this must be placed in shared memory,
                                           and initialized by application.
        EMAC_Core              EMACCore;// In multi-core scenario, it is desirable to place EMACCore in local memory
                                           so that run-time cache operation on EMACCore can be avoided
        Uint32                 coreNum;
        Uint32                 i = 0, j = 0;
        EMAC_Config            ecfg;
        EMAC_AddrConfig*       addrCfg;

        // Initialize our EMAC Dev structure.
	    if(coreNum == MASTER_CORE) 
	    {
			memset(&EMACObj, 0, sizeof(EMAC_Device));

			// Not using the MDIO configuration
			ecfg.EMACCommonConfig.UseMdio = 0;
			// core 0 is master core performing common initialization of the EMAC
			ecfg.EMACCommonConfig.CoreNum = 0; 
			//packet size
			ecfg.EMACCommonConfig.PktMTU = 1600;

			// Setup the EMAC local loopback
			ecfg.EMACCommonConfig.ModeFlags      = EMAC_CONFIG_MODEFLG_MACLOOPBACK | EMAC_CONFIG_MODEFLG_GMIIEN
												 | EMAC_CONFIG_MODEFLG_FULLDUPLEX;
			ecfg.EMACCommonConfig.MdioModeFlags  = MDIO_MODEFLG_FD1000;
		}

        // Initialize EMAC core instance structure.
        memset(&EMACCore, 0, sizeof(EMAC_Core));

        //Total 3 MAC addresses allocated for the receive channel 
        ecfg.EMACCoreConfig.NumOfMacAddrs = 3;
        // selects CPPI RAM for Descriptor memory 
        ecfg.EMACCoreConfig.DescBase = EMAC_DESC_BASE_CPPI;

        ecfg.EMACCoreConfig.RxMaxPktPool              = 8;
        ecfg.EMACCoreConfig.pfcbGetPacket             = &GetPacket;
        ecfg.EMACCoreConfig.pfcbFreePacket            = &FreePacket;
        ecfg.EMACCoreConfig.pfcbRxPacket              = &RxPacket;
        ecfg.EMACCoreConfig.pfcbStatus                = &StatusUpdate;
        ecfg.EMACCoreConfig.pfcbStatistics            = &StatisticsUpdate;

        switch(coreNum) {
            default:
            case 0:                   // core 0 use channel 0 
                ecfg.EMACCoreConfig.ChannelInfo.TxChanEnable = 1;
                ecfg.EMACCoreConfig.ChannelInfo.RxChanEnable = 1;
                break;
            case 1:                   // core 1 use channel 1 
                ecfg.EMACCoreConfig.ChannelInfo.TxChanEnable = 2;
                ecfg.EMACCoreConfig.ChannelInfo.RxChanEnable = 2;
                break;
            case 2:                   // core 2 use channel 2 
                ecfg.EMACCoreConfig.ChannelInfo.TxChanEnable = 4;
                ecfg.EMACCoreConfig.ChannelInfo.RxChanEnable = 4;
                break;
        }

        //Configure the number of MAC addresses per channel
        //Hardware gives support for 32 MAC addresses for 8 receive channels
        //Here total 9 MAC addresses are assigned to 3 receive channels
        //3 MAC addresses per channel
        //MAC addresses and channels allocated are like mentioned below
        //  core no	   channel assigned       MAC address
        //   core0         channel 0        00.01.02.03.04.05
        //						            10.11.12.13.14.15
        //								    20.21.22.23.24.25(address used for loopback test)
        //
        //   core1         channel 1		30.31.32.33.34.35
        //									40.41.42.43.44.45
        //									50.51.52.53.54.55(address used for loopback test)
        //
        //	 core2         channel 2		60.61.62.63.64.65
        //									70.71.72.73.74.75
        //									80.81.82.83.84.85(address used for loopback test)

        ecfg.EMACCoreConfig.MacAddr = (EMAC_AddrConfig **)
                                      malloc(ecfg.EMACCoreConfig.NumOfMacAddrs * sizeof(EMAC_AddrConfig *));

        for (j=0; j<ecfg.EMACCoreConfig.NumOfMacAddrs; j++){
            ecfg.EMACCoreConfig.MacAddr[j] = (EMAC_AddrConfig *)malloc(sizeof(EMAC_AddrConfig));
        }

        for(j=0; (Uint8)j<(ecfg.EMACCoreConfig.NumOfMacAddrs); j++){
            addrCfg = ecfg.EMACCoreConfig.MacAddr[j];
            addrCfg->ChannelNum = coreNum;
            for (i=0; i<6; i++)
            {
                addrCfg->Addr[i] = j * 0x10 + i + coreNum * 0x30;
            }
        }

        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_open( int instNum, Handle hApplication, EMAC_Config *pEMACConfig, Handle hEMAC, Handle hCore );

/** ============================================================================
 *  @n@b  EMAC_commonInit()
 *
 *  @b Description
 *  @n Open the EMAC peripheral at the given physical index and perform
 *     initialization common to all the cores, including resetting the EMAC control
 *     module and stats, initializing DMA, configuring common registers such as
 *     the MACCONTROL register, initializing MDIO, and etc. This function has to 
 *     be called only once by the master core.
 *
 *     The calling application must supply an operating configuration. Data from 
 *     this config structure is copied into the device's internal instance structure
 *     so the structure may be discarded after EMAC_commonInit() returns. In order
 *     to change an item in the configuration, the EMAC device must be de-initialized
 *     by EMAC_commonDeInit() and then re-initialized by calling EMAC_commonInit()
 *     with the new configuration.
 *
 *     A valid EMAC device handle should be passed to this API to which the
 *     configuration and operating state of the EMAC device common to all the cores
 *     will be written. The EMAC device structure for the handle needs to be 
 *     initialized before calling EMAC_commonInit().
 *
 *     The default receive filter prevents normal packets from being received
 *     until the receive filter is specified by calling EMAC_receiveFilter().
 *
 *     A device reset is achieved by calling EMAC_commonDeInit() followed by 
 *     EMAC_commonInit().
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error code include:
 *         EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        instNum           EMAC Peripheral ID to identify the EMAC controller to be
                          initialized.
        pEMACCommonConfig EMAC's configuration common to all cores.
        hEMAC             Handle to the EMAC device which needs to be initialized.
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  None
 *
 *  <b> Post Condition </b>
 *  @n  Open the EMAC peripheral at the given physical index and perform 
 *      initialization common to all the cores.
 *
 *  @b Example
 *  @verbatim
        EMAC_Device            EMACObj; // In multi-core scenario, this must be placed in shared memory,
                                           and initialized by application. Refer to "emac_core_restart"
                                           example project for more details.

        EMAC_Common_Config     commonCfg;

        // Initialize our EMAC Dev structure.
        memset(&EMACObj, 0, sizeof(EMAC_Device));

        // Not using the MDIO configuration
        commonCfg.UseMdio = 0;
        // core 0 is master core performing common initialization of the EMAC
        commonCfg.CoreNum = 0; 
        //packet size
        commonCfg.PktMTU = 1600;

        // Setup the EMAC local loopback
        commonCfg.ModeFlags      = EMAC_CONFIG_MODEFLG_MACLOOPBACK | EMAC_CONFIG_MODEFLG_GMIIEN
                                 | EMAC_CONFIG_MODEFLG_FULLDUPLEX;
        commonCfg.MdioModeFlags  = MDIO_MODEFLG_FD1000;

        //Common initialization of the EMAC peripheral
        EMAC_commonInit(0, &commonCfg, &EMACObj);
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_commonInit( int instNum, EMAC_Common_Config *pEMACCommonConfig, Handle hEMAC );

/** ============================================================================
 *  @n@b  EMAC_coreInit()
 *
 *  @b Description
 *  @n Core specific initialization to use the EMAC peripheral at the given 
 *     physical index when every core starts/restarts. The per core configuration 
 *     includes setting up Tx/Rx channels, allocating MAC addresses, enabling 
 *     interrupts in the EMAC control module for the channels that this core uses,
 *     and etc.
 *
 *     The calling application must supply an operating configuration. Data from 
 *     this config structure is copied into the device's internal instance structure
 *     so the structure may be discarded after EMAC_coreInit() returns. In order
 *     to change an item in the configuration, the core must be de-initialized from
 *     the EMAC device by calling EMAC_coreDeInit() and then re-initialized by 
 *     calling EMAC_coreInit() with the new configuration.
 *
 *     A valid EMAC core instance handle should be passed to this API to which the
 *     operating state of the EMAC device specific to individual cores will be written.
 *     The EMAC core instance structure for the handle needs to be initialized
 *     before calling EMAC_coreInit().
 *
 *     The application layer may pass in an hApplication callback handle, that 
 *     will be supplied by the EMAC device for the core when making calls to the
 *     core's application callback functions.
 *
 *     A valid EMAC device handle should be passed to this API to which the per 
 *     core configuration and operating state of the EMAC device will be written.
 *
 *     A core reset for EMAC operation is achieved by calling EMAC_coreDeInit() 
 *     followed by EMAC_coreInit().
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error code include:
 *         EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        hEMAC           Handle to the EMAC device which needs to be initialized.
        hApplication    Application handle
        pEMACCoreConfig EMAC's configuration specific to the core
        hCore           Handle to the EMAC core instance which needs to be initialized.
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  Before calling this API, EMAC_commonInit function must be called on the master
 *      core and then EMAC_coreDeInit must be called for this particular core.
 *
 *  <b> Post Condition </b>
 *  @n  Core specific initialization to use the EMAC peripheral at the given 
 *      physical index.
 *
 *  @b Example
 *  @verbatim
        #define MASTER_CORE 0           // Which core is responsible for EMAC common initialization

        Uint32                 coreNum;
        Uint32                 i = 0, j = 0;
        EMAC_Common_Config     commonCfg;
        EMAC_Core_Config       coreCfg;
        EMAC_Device            EMACObj;
        EMAC_Core              EMACCore;
        EMAC_AddrConfig*       addrCfg;

        ...

        if (coreNum == MASTER_CORE)
		{
            // Initialize our EMAC Dev structure.
            memset(&EMACObj, 0, sizeof(EMAC_Device));
            // Common initialization of the EMAC peripheral
            EMAC_commonInit(0, &commonCfg, &EMACObj);
		}
        // Initialize EMAC core instance structure.
        memset(&EMACCore, 0, sizeof(EMAC_Core));

        //Total 3 MAC addresses allocated for the receive channel 
        coreCfg.NumOfMacAddrs = 3;
        // selects CPPI RAM for Descriptor memory 
        coreCfg.DescBase = EMAC_DESC_BASE_CPPI;

        coreCfg.RxMaxPktPool              = 8;
        coreCfg.pfcbGetPacket             = &GetPacket;
        coreCfg.pfcbFreePacket            = &FreePacket;
        coreCfg.pfcbRxPacket              = &RxPacket;
        coreCfg.pfcbStatus                = &StatusUpdate;
        coreCfg.pfcbStatistics            = &StatisticsUpdate;

        switch(coreNum) {
            default:
            case 0:                   // core 0 use channel 0 
                coreCfg.ChannelInfo.TxChanEnable = 1;
                coreCfg.ChannelInfo.RxChanEnable = 1;
                break;
            case 1:                   // core 1 use channel 1 
                coreCfg.ChannelInfo.TxChanEnable = 2;
                coreCfg.ChannelInfo.RxChanEnable = 2;
                break;
            case 2:                   // core 2 use channel 2 
                coreCfg.ChannelInfo.TxChanEnable = 4;
                coreCfg.ChannelInfo.RxChanEnable = 4;
                break;
        }

        //Configure the number of MAC addresses per channel
        //Hardware gives support for 32 MAC addresses for 8 receive channels
        //Here total 9 MAC addresses are assigned to 3 receive channels
        //3 MAC addresses per channel
        //MAC addresses and channels allocated are like mentioned below
        //  core no	   channel assigned       MAC address
        //   core0         channel 0        00.01.02.03.04.05
        //						            10.11.12.13.14.15
        //								    20.21.22.23.24.25(address used for loopback test)
        //
        //   core1         channel 1		30.31.32.33.34.35
        //									40.41.42.43.44.45
        //									50.51.52.53.54.55(address used for loopback test)
        //
        //	 core2         channel 2		60.61.62.63.64.65
        //									70.71.72.73.74.75
        //									80.81.82.83.84.85(address used for loopback test)

        coreCfg.MacAddr = (EMAC_AddrConfig **)
                          malloc(coreCfg.NumOfMacAddrs * sizeof(EMAC_AddrConfig *));

        for (j=0; j<coreCfg.NumOfMacAddrs; j++){
            coreCfg.MacAddr[j] = (EMAC_AddrConfig *)malloc(sizeof(EMAC_AddrConfig));
        }

        for(j=0; (Uint8)j<(coreCfg.NumOfMacAddrs); j++){
            addrCfg = coreCfg.MacAddr[j];
            addrCfg->ChannelNum = coreNum;
            for (i=0; i<6; i++)
            {
                addrCfg->Addr[i] = j * 0x10 + i + coreNum * 0x30;
            }
        }

        EMAC_coreDeInit(&EMACObj);
        EMAC_coreInit(&EMACObj, (Handle)0x12345678, &coreCfg, &EMACCore);
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_coreInit( Handle hEMAC, Handle hApplication, EMAC_Core_Config *pEMACCoreConfig, Handle hCore);

/** ============================================================================
 *  @n@b  EMAC_commonDeInit()
 *
 *  @b Description
 *  @n Shut down EMAC peripheral indicated by the supplied EMAC device handle: 
 *     tear down both send and receive operations, resets common configuration,
 *     and bring down the MDIO link. Called by the master core only. 
 *   . 
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error code include:
 *         EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        hEMAC       handle to opened the EMAC device
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC_open (on the master core) or EMAC_commonInit function must be called
 *      before calling this API.
 *
 *  <b> Post Condition </b>
 *  @n  The EMAC device will shutdown both send and receive operations, resets common
 *      configuration, and bring down the MDIO link.
 *
 *  @b Example
 *  @verbatim
        EMAC_Config     ecfg;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;
        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...

        //Shut down EMAC operation for a core
        EMAC_coreDeInit( &EMACObj );

        ...

        //Shut down the EMAC peripheral
        EMAC_commonDeInit( &EMACObj );
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_commonDeInit(Handle hEMAC);

/** ============================================================================
 *  @n@b  EMAC_coreDeInit()
 *
 *  @b Description
 *  @n Shut down EMAC operation for a core. When called, free all pending transmit
 *     and receive packets for the core, and release Tx/Rx channels used and 
 *     MAC addresses assigned
 *   . 
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error code include:
 *         EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        hEMAC       handle to opened the EMAC device
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC_open (on the master core) or EMAC_commonInit function must be called
 *      before calling this API.
 *
 *  <b> Post Condition </b>
 *  @n  The EMAC device will free all pending transmit and receive packets for the 
 *      core, and also release Tx/Rx channels used and MAC addresses assigned.
 *
 *  @b Example
 *  @verbatim
        EMAC_Config     ecfg;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...

        //Shut down EMAC operation for a core
        EMAC_coreDeInit( &EMACObj );
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_coreDeInit(Handle hEMAC);

/** ============================================================================
 *  @n@b  EMAC_close()
 *
 *  @b Description
 *  @n When called by a core, free all pending transmit and receive packets, and 
 *     release Tx/Rx channels used and MAC addresses assigned via.calling 
 *     EMAC_coreDeInit(). When called by the master core, furhter close the EMAC
 *     peripheral indicated by the supplied EMAC device handle, shutdown both send
 *     and receive operations, and bring down the MDIO link via calling 
 *     EMAC_commonDeInit().
 *   . 
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error code include:
 *         EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        hEMAC   handle to opened the EMAC device
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC_open (on the master core) or EMAC_commonInit function must be called
 *      before calling this API.
 *
 *  <b> Post Condition </b>
 *  @n  For any core, the EMAC device will free all pending transmit and receive 
 *      packets, release Tx/Rx channels used and MAC addresses assigned. For master
 *      core, the EMAC device will further shutdown both send and receive operations
 *      and bring down the MDIO link.
 *
 *  @b Example
 *  @verbatim
        EMAC_Config  ecfg;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;
        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...

        //Close the EMAC peripheral
        EMAC_close( &EMACObj );
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_close( Handle hEMAC );

/** ============================================================================
 *  @n@b  EMAC_getStatus()
 *
 *  @b Description
 *  @n Called to get the current status of the device. The device status
 *     is copied into the supplied data structure.
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error code include:
 *      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        hEMAC   handle to the opened EMAC device
        pStatus Status of the EMAC
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC peripheral instance must be opened before calling this API.
 *
 *  <b> Post Condition </b>
 *  @n  The current status of the device is copied into the supplied data
 *      structure.
 *
 *  @b Example
 *  @verbatim
        EMAC_Status     pStatus;
        EMAC_Config  ecfg;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...

        EMAC_getStatus( &EMACObj, &pStatus);
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_getStatus( Handle hEMAC, EMAC_Status *pStatus);

/** ============================================================================
 *  @n@b  EMAC_setReceiveFilter()
 *
 *  @b Description
 *  @n Called to set the packet filter for received packets. The filtering
 *     level is inclusive, so BROADCAST would include both BROADCAST and
 *     DIRECTED (UNICAST) packets.
 *
 *     Available filtering modes include the following:
 *         - EMAC_RXFILTER_NOTHING      - Receive nothing
 *         - EMAC_RXFILTER_DIRECT       - Receive only Unicast to local MAC addr
 *         - EMAC_RXFILTER_BROADCAST    - Receive direct and Broadcast
 *         - EMAC_RXFILTER_MULTICAST    - Receive above plus multicast in mcast list
 *         - EMAC_RXFILTER_ALLMULTICAST - Receive above plus all multicast
 *         - EMAC_RXFILTER_ALL          - Receive all packets
 *
 *     Note that if error frames and control frames are desired, reception of
 *     these must be specified in the device configuration.
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error code include:
 *         EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
         hEMAC           handle to the opened EMAC device
         ReceiveFilter   Filtering modes
         masterChannel   master core channel which receives broadcast frames
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC peripheral instance must be opened before calling this API
 *
 *  <b> Post Condition </b>
 *  @n  Sets the packet filter for received packets
 *
 *  @b Example
 *  @verbatim
        #define EMAC_RXFILTER_DIRECT       1
        EMAC_Config  ecfg;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...

        EMAC_setReceiveFilter(&EMACObj, EMAC_RXFILTER_DIRECT, 0);

    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_setReceiveFilter( Handle hEMAC, Uint32 ReceiveFilter, Uint8 masterChannel );

/** ============================================================================
 *  @n@b  EMAC_getReceiveFilter()
 *
 *  @b Description
 *  @n Called to get the current packet filter setting for received packets.
 *     The filter values are the same as those used in EMAC_setReceiveFilter().
 *
 *     The current filter value is written to the pointer supplied in
 *     pReceiveFilter.
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error code include:
 *       EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        hEMAC           handle to the opened EMAC device
        pReceiveFilter  Current receive packet filter
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC peripheral instance must be opened before calling this API and
 *      must be set the packet filter value.
 *
 *  <b> Post Condition </b>
 *  @n  The current filter value is written to the pointer supplied
 *
 *  @b Example
 *  @verbatim
        #define EMAC_RXFILTER_DIRECT       1
        EMAC_Config     ecfg;
        Uint32          pReceiveFilter;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...

        EMAC_setReceiveFilter(&EMACObj, EMAC_RXFILTER_DIRECT, 0);

        EMAC_getReceiveFilter(&EMACObj, &pReceiveFilter );
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_getReceiveFilter( Handle hEMAC, Uint32 *pReceiveFilter );

/** ============================================================================
 *  @n@b  EMAC_getStatistics()
 *
 *  @b Description
 *  @n Called to get the current device statistics. The statistics structure
 *     contains a collection of event counts for various packet sent and
 *     receive properties. Reading the statistics also clears the current
 *     statistic counters, so the values read represent a delta from the last
 *     call.
 *
 *     The statistics information is copied into the structure pointed to
 *     by the pStatistics argument.
 *
 *     The function returns zero on success, or an error code on failure.
 *
 *     Possible error code include:
 *      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        hEMAC       handle to the opened EMAC device
        pStatistics Get the device statistics
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC peripheral instance must be opened before calling this API
 *
 *  <b> Post Condition </b>
 *      -# Statistics are read for various packects sent and received.
        -# Reading the statistics also clears the current
           statistic counters, so the values read represent a delta from the
           last call.
 *
 *  @b Example
 *  @verbatim
        EMAC_Config      ecfg;
        EMAC_Statistics  pStatistics;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...

        EMAC_getStatistics(&EMACObj, &pStatistics );
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_getStatistics( Handle hEMAC, EMAC_Statistics *pStatistics );

/** ============================================================================
 *  @n@b  EMAC_setMulticast()
 *
 *  @b Description
 *  @n This function is called to install a list of multicast addresses for
 *     use in multicast address filtering. Each time this function is called,
 *     any current multicast configuration is discarded in favor of the new
 *     list. Thus a set with a list size of zero will remove all multicast
 *     addresses from the device.
 *
 *     Note that the multicast list configuration is stateless in that the
 *     list of multicast addresses used to build the configuration is not
 *     retained. Thus it is impossible to examine a list of currently installed
 *     addresses.
 *
 *     The addresses to install are pointed to by pMCastList. The length of
 *     this list in bytes is 6 times the value of AddrCnt. When AddrCnt is
 *     zero, the pMCastList parameter can be NULL.
 *
 *     The function returns zero on success, or an error code on failure.
 *     The multicast list settings are not altered in the event of a failure
 *     code.
 *
 *     Possible error code include:
 *       EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  @b Arguments
 *  @verbatim
        hEMAC       handle to the opened EMAC device
        AddrCount   number of addresses to multicast
        pMCastList  pointer to the multi cast list
    @endverbatim
 *
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC peripheral instance must be opened and set multicast filter.
 *
 *  <b> Post Condition </b>
        -# Install a list of multicast addresses for use in multicast
           address filtering.
        -# A set with a list size of zero will remove all multicast addresses
           from the device.
 *
 *  @b Example
 *  @verbatim
        #define EMAC_RXFILTER_ALLMULTICAST 4

        Uint32          AddrCnt;
        Uint8           pMCastList;
        EMAC_Config     ecfg;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...

        EMAC_setReceiveFilter( &EMACObj, EMAC_RXFILTER_ALLMULTICAST, 0);

        EMAC_setMulticast( &EMACObj, AddrCnt, &pMCastList );
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_setMulticast( Handle hEMAC, Uint32 AddrCnt, Uint8 *pMCastList );

/** ============================================================================
 *  @n@b  EMAC_sendPacket()
 *
 *  @b Description
 *  @n Sends a Ethernet data packet out the EMAC device. On a non-error return,
 *     the EMAC device takes ownership of the packet. The packet is returned
 *     to the application's free pool once it has been transmitted.
 *
 *     The function returns zero on success, or an error code on failure.
 *     When an error code is returned, the EMAC device has not taken ownership
 *     of the packet.
 *
 *     Possible error codes include:
 *       EMAC_ERROR_INVALID   - A calling parameter is invalid
 *       EMAC_ERROR_BADPACKET - The packet structure is invalid
 *
 *  @b Arguments
 *  @verbatim
        hEMAC       handle to the opened EMAC device
        pPkt        EMAC packet structure
    @endverbatim
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *  @n      EMAC_ERROR_BADPACKET - The packet structure is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC peripheral instance must be opened and get a packet
 *      buffer from private queue
 *
 *  <b> Post Condition </b>
 *  @n  Sends a ethernet data packet out the EMAC device and is returned to the
 *      application's free pool once it has been transmitted.
 *
 *  @b Example
 *  @verbatim
        #define EMAC_RXFILTER_DIRECT       1
        #define EMAC_PKT_FLAGS_SOP         0x80000000u
        #define EMAC_PKT_FLAGS_EOP         0x40000000u

        EMAC_Config     ecfg;
        EMAC_Pkt        *pPkt;
        Uint32          size, TxCount = 0;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...

        //set the receive filter
        EMAC_setReceiveFilter( &EMACObj, EMAC_RXFILTER_DIRECT, 0);

        //Fill the packet options fields
        size = TxCount + 60;
        pPkt->Flags      = EMAC_PKT_FLAGS_SOP | EMAC_PKT_FLAGS_EOP;
        pPkt->ValidLen   = size;
        pPkt->DataOffset = 0;
        pPkt->PktChannel = 0;
        pPkt->PktLength  = size;
        pPkt->PktFrags   = 1;

        EMAC_sendPacket( &EMACObj, pPkt );

    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_sendPacket( Handle hEMAC, EMAC_Pkt *pPkt);

/** ============================================================================
 *  @n@b  EMAC_RxServiceCheck()
 *
 *  @b Description
 *  @n This function should be called every time there is an EMAC device Rx
 *     interrupt. It maintains the status the EMAC.
 *
 *     Note that the application has the responsibility for mapping the
 *     physical device index to the correct EMAC_serviceCheck() function. If
 *     more than one EMAC device is on the same interrupt, the function must be
 *     called for each device.
 *
 *     Possible error codes include:
 *       EMAC_ERROR_INVALID   - A calling parameter is invalid
 *       EMAC_ERROR_MACFATAL  - Fatal error in the MAC - Call EMAC_close()
 *
 *  @b Arguments
 *  @verbatim
        hEMAC       handle to the opened EMAC device
    @endverbatim
 *  <b> Return Value </b>  Success (0)
 *  @n     EMAC_ERROR_INVALID   - A calling parameter is invalid
 *  @n     EMAC_ERROR_MACFATAL  - Fatal error in the MAC - Call EMAC_close()
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC_open function must be called before calling this API.
 *
 *  <b> Post Condition </b>
 *  @n  None
 *
 *  @b Example
 *  @verbatim
        static CSL_IntcContext context;
        static CSL_IntcEventHandlerRecord Record[13];
        static CSL_IntcObj intcEMACRx;
        static CSL_IntcHandle hIntcEMACRx;

        //CSL_IntcParam vectId1;
        CSL_IntcParam vectId2;

        CSL_IntcGlobalEnableState state;

        // Setup the global Interrupt
        context.numEvtEntries = 13;
        context.eventhandlerRecord = Record;

        // VectorID for the Event
        vectId2 = CSL_INTC_VECTID_6;

        CSL_intcInit(&context);
        // Enable NMIs
        CSL_intcGlobalNmiEnable();
        // Enable Global Interrupts
        CSL_intcGlobalEnable(&state);

        // Opening a handle for EMAC Rx interrupt
        hIntcEMACRx=CSL_intcOpen(&intcEMACRx,CSL_INTC_EVENTID_6,&vectId2,NULL);

        //Hook the ISRs
        CSL_intcHookIsr(vectId2,&HwRxInt);

        CSL_intcHwControl(hIntcEMACRx, CSL_INTC_CMD_EVTENABLE, NULL);

        // This function is called when Rx interrupt occurs
        Void HwRxInt (void)
        {
            EMAC_Device     EMACObj;
            EMAC_Core       EMACCore;

            ...

            //Open the EMAC peripheral
            EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

            ...
	        
            EMAC_RxServiceCheck(&EMACObj);
        }

    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_RxServiceCheck( Handle hEMAC);


/** ============================================================================
 *  @n@b  EMAC_TxServiceCheck()
 *
 *  @b Description
 *  @n This function should be called every time there is an EMAC device Tx
 *     interrupt. It maintains the status the EMAC.
 *
 *     Note that the application has the responsibility for mapping the
 *     physical device index to the correct EMAC_serviceCheck() function. If
 *     more than one EMAC device is on the same interrupt, the function must be
 *     called for each device.
 *
 *     Possible error codes include:
 *       EMAC_ERROR_INVALID   - A calling parameter is invalid
 *       EMAC_ERROR_MACFATAL  - Fatal error in the MAC - Call EMAC_close()
 *
 *  @b Arguments
 *  @verbatim
        hEMAC       handle to the opened EMAC device
    @endverbatim
 *  <b> Return Value </b>  Success (0)
 *  @n     EMAC_ERROR_INVALID   - A calling parameter is invalid
 *  @n     EMAC_ERROR_MACFATAL  - Fatal error in the MAC - Call EMAC_close()
 *
 *  <b> Pre Condition </b>
 *  @n EMAC_open function must be called before calling this API.
 *
 *  <b> Post Condition </b>
 *  @n  None
 *
 *  @b Example
 *  @verbatim
        static CSL_IntcContext context;
        static CSL_IntcEventHandlerRecord Record[13];
        static CSL_IntcObj intcEMACTx;
        static CSL_IntcHandle hIntcEMACTx;

        //CSL_IntcParam vectId1;
        CSL_IntcParam vectId2;

        CSL_IntcGlobalEnableState state;

        // Setup the global Interrupt
        context.numEvtEntries = 13;
        context.eventhandlerRecord = Record;

        // VectorID for the Event
        vectId2 = CSL_INTC_VECTID_6;

        CSL_intcInit(&context);
        // Enable NMIs
        CSL_intcGlobalNmiEnable();
        // Enable Global Interrupts
        CSL_intcGlobalEnable(&state);

        // Opening a handle for EMAC Tx interrupt
        hIntcEMACTx=CSL_intcOpen(&intcEMACTx,CSL_INTC_EVENTID_6,&vectId2,NULL);

        //Hook the ISRs
        CSL_intcHookIsr(vectId2,&HwTxInt);

        CSL_intcHwControl(hIntcEMACTx, CSL_INTC_CMD_EVTENABLE, NULL);

        // This function is called when Rx interrupt occurs
        Void HwTxInt (void)
        {
            EMAC_Device     EMACObj;
            EMAC_Core       EMACCore;

            ...

            //Open the EMAC peripheral
            EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

            ...

            EMAC_TxServiceCheck(&EMACObj);
        }

    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_TxServiceCheck( Handle hEMAC);


/** ============================================================================
 *  @n@b  EMAC_timerTick()
 *
 *  @b Description
 *  @n This function should be called for each device in the system on a
 *     periodic basis of 100mS (10 times a second). It is used to check the
 *     status of the EMAC and MDIO device, and to potentially recover from
 *     low Rx buffer conditions.
 *
 *     Strict timing is not required, but the application should make a
 *     reasonable attempt to adhere to the 100mS mark. A missed call should
 *     not be "made up" by making multiple sequential calls.
 *
 *     A "polling" driver (one that calls EMAC_serviceCheck() in a tight loop),
 *     must also adhere to the 100mS timing on this function.
 *
 *     Possible error codes include:
 *       EMAC_ERROR_INVALID   - A calling parameter is invalid

 *  @b Arguments
 *  @verbatim
        hEMAC       handle to the opened EMAC device
    @endverbatim
 *
 *  <b> Return Value </b>  Success (0)
 *  @n      EMAC_ERROR_INVALID   - A calling parameter is invalid
 *
 *  <b> Pre Condition </b>
 *  @n  EMAC peripheral instance must be opened
 *
 *  <b> Post Condition </b>
 *  @n  Re-fill Rx buffer queue if needed and modifies  EMAC CONTROL register.
 *
 *  @b Example
 *  @verbatim
        EMAC_Config ecfg;
        EMAC_Device     EMACObj;
        EMAC_Core       EMACCore;

        ...

        //Open the EMAC peripheral
        EMAC_open(0, (Handle)0x12345678, &ecfg, &EMACObj, &EMACCore);

        ...


        EMAC_timerTick( &EMACObj );
    @endverbatim
 * ============================================================================
 */
Uint32 EMAC_timerTick( Handle hEMAC);

/* @} */
#ifdef __cplusplus
}
#endif
#endif /* CSL_EMAC_H */

