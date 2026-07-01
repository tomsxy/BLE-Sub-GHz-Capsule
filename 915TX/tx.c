/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <string.h>

/* TI-RTOS Header files */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>
#include <ti/display/Display.h>
#include <ti/sysbios/knl/Clock.h>

/* Board Header files */
#include "Board.h"

/* Application specific Header files */
#include "menu.h"
#include "RFQueue.h"

#ifdef Board_SYSCONFIG_PREVIEW
#include <smartrf_settings/smartrf_settings.h>
#else
#include <smartrf_settings/smartrf_settings.h>
#include <smartrf_settings/smartrf_settings_predefined.h>
#if (defined Board_CC2650DK_7ID)        || (defined Board_CC2650_LAUNCHXL)    || \
    (defined Board_CC2640R2_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL)    || \
    (defined Board_CC1350_LAUNCHXL_433) || (defined Board_CC1350STK)          || \
    (defined Board_CC1352R1_LAUNCHXL)   || (defined Board_CC1352P1_LAUNCHXL)  || \
    (defined Board_CC1352P_2_LAUNCHXL)  || (defined Board_CC1352P_4_LAUNCHXL) || \
    (defined Board_CC26X2R1_LAUNCHXL)
#include "smartrf_settings/smartrf_settings_ble.h"
#endif //(defined Board_CC2650DK_7ID)
#endif // Board_SYSCONFIG_PREVIEW

/***** Defines *****/
#define MAX_PAYLOAD_LENGTH      254 // Maximum length of the packet to send (Even due to HS requirement)
#define MAX_BLE_PAYLOAD_LENGTH  30  // Maximum length of the BLE4/5 packet to send
#define DATA_ENTRY_HEADER_SIZE  8   // Constant header size of a Generic Data Entry
#define MAX_LENGTH              254 // Set the length of the data entry
#define NUM_DATA_ENTRIES        1
#define NUM_APPENDED_BYTES      0

#define EXTENDED_HEADER_LENGTH  9
#define BLE_BASE_FREQUENCY      2300 // When programming the channel in the BLE TX command it is the
                                     // offset from 2300 MHz

#define MAX_BLE_PWR_LEVEL_DBM   5
#define MAX_SUB1_PWR_LEVEL_DBM  13

#define ABORT_GRACEFUL          1   // Option for the RF cancel command
#define ABORT_ABRUPT            0   // Option for the RF cancel command
#define TX_CMD_TERM_EVENTS      (RF_EventLastCmdDone | RF_EventCmdCancelled | \
                                 RF_EventCmdStopped  | RF_EventCmdAborted)

/* Inter-packet intervals for each phy mode in ms*/
#define PKT_INTERVAL_MS_2GFSK   60
#define PKT_INTERVAL_MS_CUSTOM  60
#define PKT_INTERVAL_MS_SLR     80
#define PKT_INTERVAL_MS_LRM     500
#define PKT_INTERVAL_MS_OOK     100
#define PKT_INTERVAL_MS_HSM     50
#define PKT_INTERVAL_MS_BLE     100

#define RF_TX20_ENABLED         0xFFFF // Tx power setting when high PA is in use
#define CENTER_FREQ_EU          0x0364 // Center Frequency 868 MHz
#define CENTER_FREQ_US          0x0393 // Center Frequency 915 MHz

/* IEEE 802.15.4g Header Configuration
 * _S indicates the shift for a given bit field
 * _M indicates the mask required to isolate a given bit field
 */
#define IEEE_HDR_LEN_S          0U
#define IEEE_HDR_LEN_M          0x00FFU
#define IEEE_HDR_CRC_S          12U
#define IEEE_HDR_CRC_M          0x1000U
#define IEEE_HDR_WHTNG_S        11U
#define IEEE_HDR_WHTNG_M        0x0800U
#define IEEE_HDR_CRC_2BYTE      1U
#define IEEE_HDR_CRC_4BYTE      0U
#define IEEE_HDR_WHTNG_EN       1U
#define IEEE_HDR_WHTNG_DIS      0U

#define IEEE_HDR_CREATE(crc, whitening, length) {            \
    (crc << IEEE_HDR_CRC_S | whitening << IEEE_HDR_WHTNG_S | \
    ((length << IEEE_HDR_LEN_S) & IEEE_HDR_LEN_M))           \
}



/***** Prototypes *****/
static void tx_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static void spi_callback(SPI_Handle handle, SPI_Transaction *trans);
static void spi_start(void);
static void spi_stop(void);
static void tx_prepare_payload(uint16_t payloadLength);
static bool spi_dequeue_frame(uint8_t *frameIndex, uint16_t *frameLen);
static void spi_release_frame(uint8_t frameIndex);
static void spi_resume_if_needed(void);
static uint8_t spi_find_free_buffer(uint8_t startIndex);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

static uint8_t packet[MAX_PAYLOAD_LENGTH];
static uint8_t dataOffset = 0;
static volatile uint16_t seqNumber = 0;
static volatile uint32_t packetCount = 0;
static ApplicationConfig localConfig;
static volatile uint32_t time = 0;
static volatile bool bPacketTxDone = false;
static volatile RF_CmdHandle cmdHandle;

static uint8_t triggerType = TRIG_NOW;

static tx_metrics txMetrics = {
    .transmitPowerDbm = RF_TxPowerTable_INVALID_DBM,
    .dataRateBps      = 0,
    .packetIntervalMs = 0
};

/* TX power override (dBm); RF_TxPowerTable_INVALID_DBM = disabled */
static volatile int8_t g_txp_override_dbm = RF_TxPowerTable_INVALID_DBM;
static volatile int8_t g_last_txp_dbm = RF_TxPowerTable_INVALID_DBM;

void tx_set_txp_dbm(int8_t dbm)
{
    g_txp_override_dbm = dbm;
}

int8_t tx_get_txp_dbm(void)
{
    return g_txp_override_dbm;
}

int8_t tx_get_last_txp_dbm(void)
{
    return g_last_txp_dbm;
}

static Display_Handle display;

#define SPI_FRAME_SIZE  (250)
#define SPI_BUF_SIZE    (SPI_FRAME_SIZE)
#define SPI_STAGING_BUF_COUNT (8U)

static SPI_Handle spiHandle;
static SPI_Transaction spiTransaction;
static uint8_t spiRxBuffer[SPI_STAGING_BUF_COUNT][SPI_BUF_SIZE];
static uint8_t spiTxBuffer[SPI_STAGING_BUF_COUNT][SPI_BUF_SIZE];
static volatile uint8_t spiActiveIndex = 0;
static volatile uint8_t spiFrameIndex = 0;
static volatile uint16_t spiFrameLen[SPI_STAGING_BUF_COUNT] = {0};
static volatile uint32_t spiFrameSeq[SPI_STAGING_BUF_COUNT] = {0};
static volatile bool spiFrameReady[SPI_STAGING_BUF_COUNT] = {false};
static volatile bool spiFrameInUse[SPI_STAGING_BUF_COUNT] = {false};
static volatile uint32_t spiNextSeq = 0;
static volatile bool spiInitialized = false;
static volatile bool spiRxPaused = false;
static volatile uint32_t spiBytes = 0;
static volatile uint32_t spiTransfers = 0;
static volatile uint32_t spiDrops = 0;
static volatile uint32_t spiDropsTotal = 0;
static volatile bool txInFlight = false;
static volatile uint16_t txPendingFrameLen = 0;
static volatile uint32_t txBytes = 0;
static volatile uint32_t txPackets = 0;
static volatile uint32_t txCmdErrors = 0;
static volatile uint32_t txCmdPostFails = 0;
static volatile RF_EventMask txLastCmdEvent = 0;

/* Keep the slave transaction fixed at 250 bytes.  RETURN_PARTIAL makes the
 * driver complete a transfer on CS deassert, which showed up as zero/short
 * frames when the master sends many small transactions back-to-back.
 */
#define SPI_RETURN_PARTIAL_CMD  SPICC26XXDMA_CMD_RETURN_PARTIAL_DISABLE

/*
This interval is dependent on data rate and packet length, and might need to be changed
if any of these parameter changes
*/
uint32_t packetInterval;

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN (txDataEntryBuffer, 4);
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
#elif defined(__GNUC__)
__attribute__ ((aligned (4)))
#else
#error This compiler is not supported.
#endif
static uint8_t txDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];

/* TX queue or RF Core to read data from */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t *pPacket;
#if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
rfc_ble5ExtAdvEntry_t ble5ExtAdvPacket;
#endif

/* Runs the transmitting part of the test application and returns a result. */
TestResult tx_runTxTest(const ApplicationConfig* config)
{
    uint32_t lastpacketCount = 0;
    uint16_t cmdTxPower      = 0;

#if (defined Board_CC1310_LAUNCHXL)    || (defined Board_CC1350_LAUNCHXL)     || \
    (defined Board_CC1350STK)          || (defined Board_CC1350_LAUNCHXL_433) || \
    (defined Board_CC1312R1_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)   || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL)  || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC2640R2_LAUNCHXL)
    RF_TxPowerTable_Entry *rfPowerTable = NULL;
    uint8_t rfPowerTableSize = 0;
#endif
    dataOffset       = 0;

    if(config == NULL)
    {
        while(1);
    }
    memcpy((void *)&localConfig, config, sizeof(ApplicationConfig));
    localConfig.rfSetup = RfSetup_Hsm;
    localConfig.frequencyTable = config_frequencyTable_Lut[RfSetup_Hsm];
    localConfig.frequency = 1;
    localConfig.payloadLength = SPI_FRAME_SIZE;
    localConfig.intervalMode = IntervalMode_No;
    if (display == NULL)
    {
        display = Display_open(Display_Type_UART, NULL);
    }
    spi_start();

    RF_Params rfParams;
    RF_Params_init(&rfParams);
    if(localConfig.intervalMode == IntervalMode_Yes)
    {
        triggerType = TRIG_ABSTIME;
    }
    else
    {
        triggerType = TRIG_NOW;
    }

    RF_cmdPropTx.pktLen = localConfig.payloadLength;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = triggerType;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

#if (defined Board_CC1312R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL)
    RF_cmdPropTxAdv_preDef.pktLen = localConfig.payloadLength;
    RF_cmdPropTxAdv_preDef.pPkt = packet;
    RF_cmdPropTxAdv_preDef.startTrigger.triggerType = triggerType;
    RF_cmdPropTxAdv_preDef.startTrigger.pastTrig = 1;
    RF_cmdPropTxAdv_preDef.startTime = 0;
#endif

    if( RFQueue_defineQueue(&dataQueue,
                            txDataEntryBuffer,
                            sizeof(txDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(true);
    }

#if !(defined Board_CC2650DK_7ID)       && !(defined Board_CC2650_LAUNCHXL)     && \
    !(defined Board_CC2640R2_LAUNCHXL)  && !(defined Board_CC1350_LAUNCHXL_433) && \
    !(defined Board_CC1352R1_LAUNCHXL)  && !(defined Board_CC1352P1_LAUNCHXL)   && \
    !(defined Board_CC1352P_2_LAUNCHXL) && !(defined Board_CC1352P_4_LAUNCHXL)  && \
    !(defined Board_CC26X2R1_LAUNCHXL)  && !(defined Board_CC1312R1_LAUNCHXL)
    RF_cmdTxHS.pQueue = &dataQueue;
    RF_cmdTxHS.startTrigger.triggerType  = triggerType;
    RF_cmdTxHS.startTrigger.pastTrig = 1;
    RF_cmdTxHS.startTime = 0;
#endif

#if (defined Board_CC2650DK_7ID)        || (defined Board_CC2650_LAUNCHXL)    || \
    (defined Board_CC2640R2_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL)    || \
    (defined Board_CC1350_LAUNCHXL_433) || (defined Board_CC1350STK)          || \
    (defined Board_CC26X2R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL)
#if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
    RF_ble_cmdBle5AdvAux.pParams->pAdvPkt = (uint8_t *)&ble5ExtAdvPacket;
    ble5ExtAdvPacket.extHdrInfo.length = EXTENDED_HEADER_LENGTH;
    ble5ExtAdvPacket.advDataLen = MAX_BLE_PAYLOAD_LENGTH - EXTENDED_HEADER_LENGTH - 1;
    ble5ExtAdvPacket.pAdvData = packet;
    RF_ble_cmdBle5AdvAux.startTrigger.triggerType  = triggerType;
    RF_ble_cmdBle5AdvAux.startTrigger.pastTrig = 1;
    RF_ble_cmdBle5AdvAux.channel = 0xFF;
    RF_ble_cmdBle5AdvAux.whitening.bOverride = 1;
    RF_ble_cmdBle5AdvAux.whitening.init = localConfig.frequencyTable[localConfig.frequency].whitening;
    RF_ble_cmdBle5AdvAux.startTime = 0;
#endif
    RF_ble_cmdBleAdvNc.pParams->pAdvData = packet;
    RF_ble_cmdBleAdvNc.startTrigger.triggerType  = triggerType;
    RF_ble_cmdBleAdvNc.startTrigger.pastTrig = 1;
    RF_ble_cmdBleAdvNc.channel = 0xFF;
    RF_ble_cmdBleAdvNc.whitening.bOverride = 1;
    RF_ble_cmdBleAdvNc.whitening.init = localConfig.frequencyTable[localConfig.frequency].whitening;
    RF_ble_cmdBleAdvNc.startTime = 0;
#endif

    currentDataEntry = (rfc_dataEntryGeneral_t*)&txDataEntryBuffer;
    currentDataEntry->length = localConfig.payloadLength;
    pPacket = &currentDataEntry->data;

#if (defined Board_CC1310_LAUNCHXL)

    /* Modify Setup command and TX Power depending on using Range Extender or not
     * Using CC1310 + CC1190 can only be done for the following PHYs:
     * fsk (50 kbps, 2-GFSK)
     * lrm (Legacy Long Range)
     * sl_lr (SimpleLink Long Range) */
    if (localConfig.rangeExtender == RangeExtender_Dis)
    {
        /* Settings used for the CC1310 LAUNCHXL */
        uint16_t txPower = RF_TxPowerTable_findValue((RF_TxPowerTable_Entry *)RF_PROP_txPowerTable, 14).rawValue;
        RF_cmdPropRadioDivSetup_fsk.txPower   = txPower;
        RF_cmdPropRadioDivSetup_lrm.txPower   = txPower;
        RF_cmdPropRadioDivSetup_sl_lr.txPower = txPower;
        RF_cmdPropRadioDivSetup_ook.txPower   = txPower;
        {
            uint8_t i = 0;
            do
            {
                if ((pOverrides_fsk[i] & 0x0000FFFF) ==  0x000088A3)
                {
                    pOverrides_fsk[i] = (uint32_t)0x00FB88A3;
                }
            } while ((pOverrides_fsk[i++] != 0xFFFFFFFF));

            i = 0;
            do
            {
                if ((pOverrides_lrm[i] & 0x0000FFFF) ==  0x000088A3)
                {
                    pOverrides_lrm[i] = (uint32_t)0x00FB88A3;
                }
            } while ((pOverrides_lrm[i++] != 0xFFFFFFFF));

            i = 0;
            do
            {
                if ((pOverrides_sl_lr[i] & 0x0000FFFF) ==  0x000088A3)
                {
                    pOverrides_sl_lr[i] = (uint32_t)0x00FB88A3;
                }
            } while ((pOverrides_sl_lr[i++] != 0xFFFFFFFF));

            i = 0;
            do
            {
                if ((pOverrides_ook[i] & 0x0000FFFF) ==  0x000088A3)
                {
                    pOverrides_ook[i] = (uint32_t)0x00FB88A3;
                }
            } while ((pOverrides_ook[i++] != 0xFFFFFFFF));
        }
    }
    else
    {
        /* Settings used for the CC1310 CC1190 LAUNCHXL */
        if(localConfig.frequencyTable[localConfig.frequency].frequency == CENTER_FREQ_EU) // 868 MHz
        {
            uint16_t txPower = RF_TxPowerTable_findValue((RF_TxPowerTable_Entry *)RF_PROP_txPowerTableREEU, 25).rawValue;
            RF_cmdPropRadioDivSetup_fsk.txPower   = txPower;
            RF_cmdPropRadioDivSetup_lrm.txPower   = txPower;
            RF_cmdPropRadioDivSetup_sl_lr.txPower = txPower;
            RF_cmdPropRadioDivSetup_ook.txPower   = txPower;
            {
                uint8_t i = 0;
                do
                {
                    if ((pOverrides_fsk[i] & 0x0000FFFF) ==  0x000088A3)
                    {
                        pOverrides_fsk[i] = (uint32_t)0x000188A3;
                    }
                } while ((pOverrides_fsk[i++] != 0xFFFFFFFF));

                i = 0;
                do
                {
                    if ((pOverrides_lrm[i] & 0x0000FFFF) ==  0x000088A3)
                    {
                        pOverrides_lrm[i] = (uint32_t)0x000188A3;
                    }
                } while ((pOverrides_lrm[i++] != 0xFFFFFFFF));

                i = 0;
                do
                {
                    if ((pOverrides_sl_lr[i] & 0x0000FFFF) ==  0x000088A3)
                    {
                        pOverrides_sl_lr[i] = (uint32_t)0x000188A3;
                    }
                } while ((pOverrides_sl_lr[i++] != 0xFFFFFFFF));

                i = 0;
                do
                {
                    if ((pOverrides_ook[i] & 0x0000FFFF) ==  0x000088A3)
                    {
                        pOverrides_ook[i] = (uint32_t)0x000188A3;
                    }
                } while ((pOverrides_ook[i++] != 0xFFFFFFFF));
            }
        }
        else if(localConfig.frequencyTable[localConfig.frequency].frequency == CENTER_FREQ_US) // 915 MHz
        {
            uint16_t txPower = RF_TxPowerTable_findValue((RF_TxPowerTable_Entry *)RF_PROP_txPowerTableREUS, 25).rawValue;
            RF_cmdPropRadioDivSetup_fsk.txPower   = txPower;
            RF_cmdPropRadioDivSetup_lrm.txPower   = txPower;
            RF_cmdPropRadioDivSetup_sl_lr.txPower = txPower;
            RF_cmdPropRadioDivSetup_ook.txPower   = txPower;
            {
                uint8_t i = 0;
                do
                {
                    if ((pOverrides_fsk[i] & 0x0000FFFF) ==  0x000088A3)
                    {
                        pOverrides_fsk[i] = (uint32_t)0x000388A3;
                    }
                } while ((pOverrides_fsk[i++] != 0xFFFFFFFF));

                i = 0;
                do
                {
                    if ((pOverrides_lrm[i] & 0x0000FFFF) ==  0x000088A3)
                    {
                        pOverrides_lrm[i] = (uint32_t)0x000388A3;
                    }
                } while ((pOverrides_lrm[i++] != 0xFFFFFFFF));

                i = 0;
                do
                {
                    if ((pOverrides_sl_lr[i] & 0x0000FFFF) ==  0x000088A3)
                    {
                        pOverrides_sl_lr[i] = (uint32_t)0x000388A3;
                    }
                } while ((pOverrides_sl_lr[i++] != 0xFFFFFFFF));

                i = 0;
                do
                {
                    if ((pOverrides_ook[i] & 0x0000FFFF) ==  0x000088A3)
                    {
                        pOverrides_ook[i] = (uint32_t)0x000388A3;
                    }
                } while ((pOverrides_ook[i++] != 0xFFFFFFFF));
            }
        }
    }
#endif

    /* Request access to the radio (HSM only) */
    switch (localConfig.rfSetup)
    {
#if !(defined Board_CC2650DK_7ID)       && !(defined Board_CC2650_LAUNCHXL)     && \
    !(defined Board_CC2640R2_LAUNCHXL)  && !(defined Board_CC1350_LAUNCHXL_433) && \
    !(defined Board_CC1352R1_LAUNCHXL)  && !(defined Board_CC1352P1_LAUNCHXL)   && \
    !(defined Board_CC1352P_2_LAUNCHXL) && !(defined Board_CC1352P_4_LAUNCHXL)  && \
    !(defined Board_CC26X2R1_LAUNCHXL)  && !(defined Board_CC1312R1_LAUNCHXL)
        case RfSetup_Hsm:
            rfHandle = RF_open(&rfObject, &RF_prop_hsm, (RF_RadioSetup*)&RF_cmdRadioSetup_hsm, &rfParams);
            packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_HSM));
            cmdTxPower     = RF_cmdRadioSetup_hsm.txPower;
            break;
#endif
        default:
            while (1);
    }

    /* Optional TX power override (dBm) */
    if (g_txp_override_dbm != RF_TxPowerTable_INVALID_DBM)
    {
        RF_TxPowerTable_Value v = RF_TxPowerTable_findValue(
            (RF_TxPowerTable_Entry *)RF_PROP_txPowerTable, g_txp_override_dbm);
        (void)RF_setTxPower(rfHandle, v);
        cmdTxPower = v.rawValue;
    }

    /* Set the packet interval for display purposes */
    if(localConfig.intervalMode == IntervalMode_Yes)
    {
        txMetrics.packetIntervalMs = RF_convertRatTicksToMs(packetInterval);
    }
    else
    {
        // packets sent back-to-back
        txMetrics.packetIntervalMs = 0;
    }

    /* Determine the transmission power in dBm */
#if (defined Board_CC1310_LAUNCHXL)    || (defined Board_CC1350_LAUNCHXL)     || \
    (defined Board_CC1350STK)          || (defined Board_CC1350_LAUNCHXL_433) || \
    (defined Board_CC1312R1_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)   || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL)  || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC2640R2_LAUNCHXL)
    rfPowerTable = (RF_TxPowerTable_Entry *)RF_PROP_txPowerTable;
    rfPowerTableSize = RF_PROP_TX_POWER_TABLE_SIZE;

#if (defined Board_CC1310_LAUNCHXL)
    /* Overwrite table pointer if range extender is enabled and the current
     * selected mode is not HSM */
     if((localConfig.rangeExtender == RangeExtender_En) &&
         (localConfig.frequencyTable[localConfig.frequency].frequency == CENTER_FREQ_EU) &&
         (localConfig.rfSetup != RfSetup_Hsm))
    {
        rfPowerTable = (RF_TxPowerTable_Entry *)RF_PROP_txPowerTableREEU;
        rfPowerTableSize = RF_PROP_TX_POWER_TABLE_SIZE_REEU;
    }
        else if((localConfig.rangeExtender == RangeExtender_En) &&
            (localConfig.frequencyTable[localConfig.frequency].frequency == CENTER_FREQ_US) &&
            (localConfig.rfSetup != RfSetup_Hsm))
    {
        rfPowerTable = (RF_TxPowerTable_Entry *)RF_PROP_txPowerTableREUS;
        rfPowerTableSize = RF_PROP_TX_POWER_TABLE_SIZE_REUS;
    }
#endif

#if (defined Board_CC1350_LAUNCHXL)    || (defined Board_CC1350_LAUNCHXL_433) || \
    (defined Board_CC1350STK)          || (defined Board_CC1352R1_LAUNCHXL)   || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL)  || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC2640R2_LAUNCHXL)
    /* Overwrite table pointer in BLE mode */
    if((localConfig.rfSetup == RfSetup_Ble)
#if (defined Board_CC1352R1_LAUNCHXL)  || (defined Board_CC1352P1_LAUNCHXL)  || \
    (defined Board_CC1352P_2_LAUNCHXL) || (defined Board_CC1352P_4_LAUNCHXL) || \
    (defined Board_CC2640R2_LAUNCHXL)
    || (localConfig.rfSetup == RfSetup_Ble5)
#endif
    )
    {
        rfPowerTable = (RF_TxPowerTable_Entry *)RF_BLE_txPowerTable;
        rfPowerTableSize = RF_BLE_TX_POWER_TABLE_SIZE;
    }
#endif

#if ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
     (defined Board_CC1352P_4_LAUNCHXL))
    /* Set the power level based on the choice of high or default PA */
    int8_t powerLevelDbm;
    if(localConfig.highPa == HighPa_En)
    {
        powerLevelDbm = RF_TxPowerTable_MAX_DBM;
    }
    else
    {
        if((localConfig.rfSetup == RfSetup_Ble) || (localConfig.rfSetup == RfSetup_Ble5))
        {
            powerLevelDbm = MAX_BLE_PWR_LEVEL_DBM;
        }
        else
        {
            powerLevelDbm = MAX_SUB1_PWR_LEVEL_DBM;
        }
    }
    RF_TxPowerTable_Value powerTableValue =  RF_TxPowerTable_findValue(rfPowerTable, powerLevelDbm);
    RF_Stat status = RF_setTxPower(rfHandle, powerTableValue);
    if(status != RF_StatSuccess)
    {
        txMetrics.transmitPowerDbm = RF_TxPowerTable_INVALID_DBM;
    }
#endif

    /*
     * Exceptions for
     *   1. BLE: Tx power is set to the second highest entry when boost mode
     *      is turned off (CCFG_FORCE_VDDR_HH = 0)
     *   2. High Speed Mode: The Tx power is set to the highest entry in the
     *      power table when boost mode is turned on (CCFG_FORCE_VDDR_HH = 1)
     *      or the second highest entry when boost is turned off
     *      (CCFG_FORCE_VDDR_HH = 0)
     */
    if((rfPowerTable == NULL) || (cmdTxPower == 0))
    {
        txMetrics.transmitPowerDbm = RF_TxPowerTable_INVALID_DBM;
    }
    else
    {
        RF_TxPowerTable_Value currValue = RF_getTxPower(rfHandle);
        txMetrics.transmitPowerDbm = RF_TxPowerTable_findPowerLevel(rfPowerTable, currValue);
        g_last_txp_dbm = txMetrics.transmitPowerDbm;

    //if CCFG_FORCE_VDDR_HH is not set max power cannot be achieved in Sub-1GHz
    //mode; this does not apply to 2.4GHz proprietary modes either
#if ((CCFG_FORCE_VDDR_HH != 0x1))
        if((currValue.paType == RF_TxPowerTable_DefaultPA) &&
           (txMetrics.transmitPowerDbm == rfPowerTable[rfPowerTableSize-2].power))
        {
#if (defined Board_CC1350_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL_433) || \
    (defined Board_CC1350STK)         || (defined Board_CC1352R1_LAUNCHXL)   || \
    (defined Board_CC1352P1_LAUNCHXL) || (defined Board_CC1352P_2_LAUNCHXL)  || \
    (defined Board_CC1352P_4_LAUNCHXL)
            if ((localConfig.rfSetup != RfSetup_Ble)
#if (defined Board_CC1352R1_LAUNCHXL)  || (defined Board_CC1352P1_LAUNCHXL) || \
    (defined Board_CC1352P_2_LAUNCHXL) || (defined Board_CC1352P_4_LAUNCHXL)
             && (localConfig.rfSetup != RfSetup_Ble5)
#endif
            )
#endif
            {
#if !(defined Board_CC2640R2_LAUNCHXL)
                txMetrics.transmitPowerDbm = rfPowerTable[rfPowerTableSize-3].power;
#endif
            }
#if (defined Board_CC1350_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL_433) || \
    (defined Board_CC1350STK)
            else
            {
                txMetrics.transmitPowerDbm = rfPowerTable[rfPowerTableSize-6].power;
            }
#endif
        }
#endif

#if !(defined Board_CC2650DK_7ID)       && !(defined Board_CC2650_LAUNCHXL)     && \
    !(defined Board_CC2640R2_LAUNCHXL)  && !(defined Board_CC1350_LAUNCHXL_433) && \
    !(defined Board_CC1352R1_LAUNCHXL)  && !(defined Board_CC1352P1_LAUNCHXL)   && \
    !(defined Board_CC1352P_2_LAUNCHXL) && !(defined Board_CC1352P_4_LAUNCHXL)  && \
    !(defined Board_CC26X2R1_LAUNCHXL)  && !(defined Board_CC1312R1_LAUNCHXL)
        if(localConfig.rfSetup == RfSetup_Hsm)
        {
#if (CCFG_FORCE_VDDR_HH == 0x1)
            txMetrics.transmitPowerDbm = rfPowerTable[rfPowerTableSize-2].power;
#else
            txMetrics.transmitPowerDbm = rfPowerTable[rfPowerTableSize-3].power;
#endif
        }
#endif
    }
#else
    uint8_t powerTableIndex;
    for(powerTableIndex = 0; powerTableIndex < rfPowerTableSize; powerTableIndex++)
    {
        if(rfPowerTable[powerTableIndex].txPower == cmdTxPower)
        {
            txMetrics.transmitPowerDbm = rfPowerTable[powerTableIndex].dbm;
            break;
        }
        else
        {
            // TX power configuration not found in the power table
            txMetrics.transmitPowerDbm = RF_TxPowerTable_INVALID_DBM;
        }
    }
#if (defined Board_CC2650DK_7ID)      || (defined Board_CC2650_LAUNCHXL)   || \
    (defined Board_CC26X2R1_LAUNCHXL)
    if((localConfig.rfSetup == RfSetup_Ble)
#if (defined Board_CC26X2R1_LAUNCHXL)
    || (localConfig.rfSetup == RfSetup_Ble5)
#endif
    )
    {
        // If BLE mode is enabled, power is set to 5 dBm
        txMetrics.transmitPowerDbm = MAX_BLE_PWR_LEVEL_DBM;
    }
#endif
#endif

    /* Determine the data rate in bits per seconds */
    txMetrics.dataRateBps = config_dataRateTable_Lut[localConfig.rfSetup];

    menu_updateTxMetricScreen(&txMetrics);

    /* Set the frequency (HSM 915 MHz) */
    RF_cmdFs_preDef.frequency = localConfig.frequencyTable[localConfig.frequency].frequency;
    RF_cmdFs_preDef.fractFreq = localConfig.frequencyTable[localConfig.frequency].fractFreq;
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs_preDef, RF_PriorityNormal, NULL, 0);

    txInFlight = false;
    cmdHandle = RF_ALLOC_ERROR;
    txCmdErrors = 0;
    txCmdPostFails = 0;
    txLastCmdEvent = 0;

    uint32_t lastPrintTicks = Clock_getTicks();
    while (!bPacketTxDone)
    {
        /* Check, whether a button has been pressed */
        if (menu_isButtonPressed())
        {
            /* If there is an ongoing Tx command, cancel it */
            if ((txInFlight) && (cmdHandle >= 0))
            {
                (void)RF_cancelCmd(rfHandle, cmdHandle, ABORT_GRACEFUL);
                RF_pendCmd(rfHandle, cmdHandle, (RF_EventCmdCancelled | RF_EventCmdStopped | RF_EventCmdAborted));
            }
            RF_close(rfHandle);
            spi_stop();

            /* Do a final update to indicate #packets sent*/
            menu_updateTxScreen(packetCount);

            bPacketTxDone = false;
            packetCount = 0;
            seqNumber = 0;
            return TestResult_Aborted;
        }
        else if(packetCount != lastpacketCount)
        {
            /* Update the display */
            menu_updateTxScreen(packetCount);
            lastpacketCount = packetCount;
        }

        if (!txInFlight)
        {
            uint8_t frameIndex = 0;
            uint16_t frameLen = 0;
            if (spi_dequeue_frame(&frameIndex, &frameLen))
            {
                if (frameLen == SPI_FRAME_SIZE)
                {
                    spiFrameIndex = frameIndex;
                    tx_prepare_payload(localConfig.payloadLength);
                    spi_release_frame(frameIndex);
                    spi_resume_if_needed();

                    time = RF_getCurrentTime();
                    RF_cmdTxHS.startTime = time;
                    cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdTxHS,
                                           RF_PriorityNormal, &tx_callback, TX_CMD_TERM_EVENTS);
                    if (cmdHandle >= 0)
                    {
                        txInFlight = true;
                    }
                    else
                    {
                        txInFlight = false;
                        txCmdPostFails++;
                    }
                }
                else if (frameLen > 0)
                {
                    spiDrops++;
                    spiDropsTotal++;
                    spi_release_frame(frameIndex);
                    spi_resume_if_needed();
                }
                else
                {
                    spi_release_frame(frameIndex);
                    spi_resume_if_needed();
                }
            }
        }

        uint32_t nowTicks = Clock_getTicks();
        uint32_t elapsedTicks = nowTicks - lastPrintTicks;
        uint32_t elapsedUs = (Clock_tickPeriod != 0) ?
            (elapsedTicks * (uint32_t)Clock_tickPeriod) :
            (elapsedTicks * 1000U);
        if (elapsedUs >= 1000000U)
        {
            uint32_t kbps = (uint32_t)((uint64_t)spiBytes * 8ULL * 1000000ULL /
                                       (uint64_t)elapsedUs / 1000ULL);
            uint32_t txKbps = (uint32_t)((uint64_t)txBytes * 8ULL * 1000000ULL /
                                         (uint64_t)elapsedUs / 1000ULL);
            if (display != NULL)
            {
                Display_printf(display, 0, 0,
                               "SPI RX: %lu kbps, 915TX: %lu kbps\n",
                               (unsigned long)kbps,
                               (unsigned long)txKbps);
            }
            spiBytes = 0;
            spiTransfers = 0;
            spiDrops = 0;
            txBytes = 0;
            txPackets = 0;
            lastPrintTicks = nowTicks;
        }
    }

    if(packetCount == localConfig.packetCount)
    {
        /* Do a final update to indicate all packets were sent*/
        menu_updateTxScreen(packetCount);
    }

    bPacketTxDone = false;
    packetCount = 0;
    seqNumber = 0;
    RF_close(rfHandle);
    spi_stop();
    return TestResult_Finished;
}

static void spi_callback(SPI_Handle handle, SPI_Transaction *trans)
{
    (void)handle;
    uint8_t doneIndex = spiActiveIndex;
    uint8_t nextIndex;
    uint16_t rxCount = (uint16_t)trans->count;

    if (rxCount > SPI_FRAME_SIZE)
    {
        rxCount = SPI_FRAME_SIZE;
    }

    if (spiFrameReady[doneIndex] || spiFrameInUse[doneIndex])
    {
        spiDrops++;
        spiDropsTotal++;
    }

    spiFrameLen[doneIndex] = rxCount;
    spiFrameReady[doneIndex] = true;
    spiBytes += rxCount;
    spiTransfers++;

    spiFrameSeq[doneIndex] = spiNextSeq++;

    nextIndex = spi_find_free_buffer((uint8_t)((doneIndex + 1U) % SPI_STAGING_BUF_COUNT));
    if (nextIndex == 0xFFU)
    {
        /* No free staging buffer: pause re-arming to avoid overwriting unread data. */
        spiRxPaused = true;
        return;
    }

    spiActiveIndex = nextIndex;
    spiRxPaused = false;
    memset(spiRxBuffer[spiActiveIndex], 0, SPI_BUF_SIZE);
    memset(spiTxBuffer[spiActiveIndex], 0xFF, SPI_BUF_SIZE);
    SPI_control(spiHandle, SPI_RETURN_PARTIAL_CMD, NULL);
    spiTransaction.count = SPI_BUF_SIZE;
    spiTransaction.txBuf = spiTxBuffer[spiActiveIndex];
    spiTransaction.rxBuf = spiRxBuffer[spiActiveIndex];
    (void)SPI_transfer(spiHandle, &spiTransaction);
}

static void spi_start(void)
{
    uint8_t i;

    if (spiInitialized)
    {
        return;
    }

    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.mode = SPI_SLAVE;
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn = spi_callback;
    spiParams.dataSize = 8;

    spiHandle = SPI_open(Board_SPI_SLAVE, &spiParams);
    if (spiHandle == NULL)
    {
        spiInitialized = false;
        if (display != NULL)
        {
            Display_printf(display, 0, 0, "SPI open failed\n");
        }
        return;
    }

    spiActiveIndex = 0;
    spiFrameIndex = 0;
    spiNextSeq = 0;
    txPendingFrameLen = 0U;
    for (i = 0; i < SPI_STAGING_BUF_COUNT; i++)
    {
        spiFrameLen[i] = 0;
        spiFrameSeq[i] = 0;
        spiFrameReady[i] = false;
        spiFrameInUse[i] = false;
    }
    spiRxPaused = false;

    memset(spiRxBuffer[spiActiveIndex], 0, SPI_BUF_SIZE);
    memset(spiTxBuffer[spiActiveIndex], 0xFF, SPI_BUF_SIZE);
    SPI_control(spiHandle, SPI_RETURN_PARTIAL_CMD, NULL);
    spiTransaction.count = SPI_BUF_SIZE;
    spiTransaction.txBuf = spiTxBuffer[spiActiveIndex];
    spiTransaction.rxBuf = spiRxBuffer[spiActiveIndex];
    if (!SPI_transfer(spiHandle, &spiTransaction))
    {
        SPI_close(spiHandle);
        spiHandle = NULL;
        spiInitialized = false;
        if (display != NULL)
        {
            Display_printf(display, 0, 0, "SPI transfer start failed\n");
        }
        return;
    }

    spiInitialized = true;
}

static void spi_stop(void)
{
    if (!spiInitialized)
    {
        return;
    }

    SPI_close(spiHandle);
    spiHandle = NULL;
    spiInitialized = false;
}

static void tx_prepare_payload(uint16_t payloadLength)
{
    if (payloadLength <= dataOffset)
    {
        return;
    }

    uint16_t payloadDataLen = (uint16_t)(payloadLength - dataOffset);

    if (spiInitialized)
    {
        uint16_t copyLen = spiFrameLen[spiFrameIndex];
        if (copyLen > payloadDataLen)
        {
            copyLen = payloadDataLen;
        }

        memset(&packet[dataOffset], 0, payloadDataLen);
        memset(&pPacket[dataOffset], 0, payloadDataLen);
        if (copyLen > 0)
        {
            uint8_t index = spiFrameIndex;
            memcpy(&packet[dataOffset], spiRxBuffer[index], copyLen);
            memcpy(&pPacket[dataOffset], spiRxBuffer[index], copyLen);
        }
        txPendingFrameLen = copyLen;
    }
    else
    {
        pPacket[dataOffset + 0] = packet[dataOffset + 0] = (uint8_t)(seqNumber >> 8);
        pPacket[dataOffset + 1] = packet[dataOffset + 1] = (uint8_t)(seqNumber++);
        uint16_t i;
        for (i = dataOffset + 2; i < payloadLength; i++)
        {
            pPacket[i] = packet[i] = rand();
        }
    }
}

void tx_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    (void)h;
    (void)ch;
    txLastCmdEvent = e;

    if(e & RF_EventLastCmdDone)
    {
        /* Increment the packet counter */
        packetCount++;
        txPackets++;
        txBytes += txPendingFrameLen;
        txPendingFrameLen = 0U;
    }

    if (e & (RF_EventCmdCancelled | RF_EventCmdStopped | RF_EventCmdAborted))
    {
        txCmdErrors++;
    }

    if (e & TX_CMD_TERM_EVENTS)
    {
        txInFlight = false;
    }
}

static bool spi_dequeue_frame(uint8_t *frameIndex, uint16_t *frameLen)
{
    uint8_t i;
    uint8_t bestIndex = 0xFFU;
    uint32_t bestSeq = 0xFFFFFFFFUL;

    for (i = 0; i < SPI_STAGING_BUF_COUNT; i++)
    {
        if (spiFrameReady[i] && !spiFrameInUse[i] &&
            (bestIndex == 0xFFU || spiFrameSeq[i] < bestSeq))
        {
            bestIndex = i;
            bestSeq = spiFrameSeq[i];
        }
    }

    if (bestIndex == 0xFFU)
    {
        return false;
    }

    *frameIndex = bestIndex;
    *frameLen = spiFrameLen[*frameIndex];
    spiFrameInUse[*frameIndex] = true;
    return true;
}

static void spi_release_frame(uint8_t frameIndex)
{
    if (frameIndex >= SPI_STAGING_BUF_COUNT)
    {
        return;
    }

    spiFrameReady[frameIndex] = false;
    spiFrameInUse[frameIndex] = false;
    spiFrameLen[frameIndex] = 0U;
}

static uint8_t spi_find_free_buffer(uint8_t startIndex)
{
    uint8_t i;
    uint8_t idx;

    if (startIndex >= SPI_STAGING_BUF_COUNT)
    {
        startIndex = 0U;
    }

    for (i = 0; i < SPI_STAGING_BUF_COUNT; i++)
    {
        idx = (uint8_t)((startIndex + i) % SPI_STAGING_BUF_COUNT);
        if (!spiFrameReady[idx] && !spiFrameInUse[idx])
        {
            return idx;
        }
    }

    return 0xFFU;
}

static void spi_resume_if_needed(void)
{
    uint8_t freeIndex;

    if (!spiInitialized || !spiRxPaused)
    {
        return;
    }

    freeIndex = spi_find_free_buffer(spiActiveIndex);
    if (freeIndex == 0xFFU)
    {
        return;
    }

    spiActiveIndex = freeIndex;
    spiRxPaused = false;
    memset(spiRxBuffer[spiActiveIndex], 0, SPI_BUF_SIZE);
    memset(spiTxBuffer[spiActiveIndex], 0xFF, SPI_BUF_SIZE);
    SPI_control(spiHandle, SPI_RETURN_PARTIAL_CMD, NULL);
    spiTransaction.count = SPI_BUF_SIZE;
    spiTransaction.txBuf = spiTxBuffer[spiActiveIndex];
    spiTransaction.rxBuf = spiRxBuffer[spiActiveIndex];
    if (!SPI_transfer(spiHandle, &spiTransaction))
    {
        spiRxPaused = true;
        spiDrops++;
        spiDropsTotal++;
    }
}
