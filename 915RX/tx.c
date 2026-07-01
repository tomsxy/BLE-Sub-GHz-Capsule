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

/* TI-RTOS Header files */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

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
#define CENTER_FREQ_433         0x01B1 // 433 MHz

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

/* tx.c 顶部加查询当前功率 */
static volatile int8_t g_last_txp_dbm = RF_TxPowerTable_INVALID_DBM;

int8_t tx_get_last_txp_dbm(void)
{
    return g_last_txp_dbm;
}

static volatile uint32_t g_interval_override_0p1ms = 0; /* 0.1ms units */

void tx_set_interval_override_0p1ms(uint32_t t01ms)
{
    g_interval_override_0p1ms = t01ms; /* 0 = disable override */
}

uint32_t tx_get_interval_override_0p1ms(void)
{
    return g_interval_override_0p1ms;
}


/*
 * CC13x0 Sub-1 GHz: CMD_PROP_RADIO_DIV_SETUP needs a correct loDivider for the
 * selected band (433 vs 868/915). If not updated, TX/RX may hang when
 * switching band at runtime.
 */
static inline uint8_t tx_getLoDivider(uint16_t centerFreqMhz)
{
    return (centerFreqMhz < 600U) ? 0x0AU : 0x05U;
}

/***** Prototypes *****/
static void tx_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

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

    RF_cmdPropTx.pktLen = config->payloadLength;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = triggerType;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

#if (defined Board_CC1312R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL)
    RF_cmdPropTxAdv_preDef.pktLen = config->payloadLength;
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
    RF_ble_cmdBle5AdvAux.whitening.init = config->frequencyTable[config->frequency].whitening;
    RF_ble_cmdBle5AdvAux.startTime = 0;
#endif
    RF_ble_cmdBleAdvNc.pParams->pAdvData = packet;
    RF_ble_cmdBleAdvNc.startTrigger.triggerType  = triggerType;
    RF_ble_cmdBleAdvNc.startTrigger.pastTrig = 1;
    RF_ble_cmdBleAdvNc.channel = 0xFF;
    RF_ble_cmdBleAdvNc.whitening.bOverride = 1;
    RF_ble_cmdBleAdvNc.whitening.init = config->frequencyTable[config->frequency].whitening;
    RF_ble_cmdBleAdvNc.startTime = 0;
#endif

    currentDataEntry = (rfc_dataEntryGeneral_t*)&txDataEntryBuffer;
    currentDataEntry->length = config->payloadLength;
    pPacket = &currentDataEntry->data;

#if (defined Board_CC1310_LAUNCHXL)

    /* Modify Setup command and TX Power depending on using Range Extender or not
     * Using CC1310 + CC1190 can only be done for the following PHYs:
     * fsk (50 kbps, 2-GFSK)
     * lrm (Legacy Long Range)
     * sl_lr (SimpleLink Long Range) */
    if (config->rangeExtender == RangeExtender_Dis)
    {
        /* Settings used for the CC1310 LAUNCHXL */
        uint16_t txPower = RF_TxPowerTable_findValue((RF_TxPowerTable_Entry *)RF_PROP_txPowerTable, 15).rawValue;
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
        if(config->frequencyTable[config->frequency].frequency == CENTER_FREQ_EU) // 868 MHz
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
        else if(config->frequencyTable[config->frequency].frequency == CENTER_FREQ_US) // 915 MHz
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

    /* Request access to the radio based on test case*/
    switch (config->rfSetup)
    {
        case RfSetup_Custom:
#if (defined DeviceFamily_CC26X0R2)
            rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
            cmdTxPower = RF_cmdPropRadioSetup.txPower;
#else
            rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
            cmdTxPower = RF_cmdPropRadioDivSetup.txPower;
#endif
            packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_CUSTOM)); // Set packet interval to 60 ms
            break;

        case RfSetup_Fsk:
#if !(defined Board_CC2650DK_7ID)      && !(defined Board_CC2650_LAUNCHXL)   && \
    !(defined Board_CC2640R2_LAUNCHXL) && !(defined Board_CC26X2R1_LAUNCHXL)
            RF_cmdPropRadioDivSetup_fsk.centerFreq = config->frequencyTable[config->frequency].frequency;
            RF_cmdPropRadioDivSetup_fsk.loDivider  = tx_getLoDivider(config->frequencyTable[config->frequency].frequency);
            rfHandle = RF_open(&rfObject, &RF_prop_fsk, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_fsk, &rfParams);
            cmdTxPower     = RF_cmdPropRadioDivSetup_fsk.txPower;
#else
            rfHandle = RF_open(&rfObject, &RF_prop_2_4G_fsk, (RF_RadioSetup*)&RF_cmdPropRadioSetup_2_4G_fsk, &rfParams);
            cmdTxPower = RF_cmdPropRadioSetup_2_4G_fsk.txPower;
#endif
            packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_2GFSK)); // Set packet interval to 60 ms
            break;

#if (defined Board_CC2640R2_LAUNCHXL)
        case RfSetup_Fsk_100:
            rfHandle = RF_open(&rfObject, &RF_prop_2_4G_fsk, (RF_RadioSetup*)&RF_cmdPropRadioSetup_2_4G_fsk_100, &rfParams);
            cmdTxPower = RF_cmdPropRadioSetup_2_4G_fsk_100.txPower;
            packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_2GFSK)); // Set packet interval to 60 ms
            break;
#endif

#if !(defined Board_CC2650DK_7ID)      && !(defined Board_CC2650_LAUNCHXL)   && \
    !(defined Board_CC2640R2_LAUNCHXL) && !(defined Board_CC26X2R1_LAUNCHXL)
        case RfSetup_Sl_lr:
            RF_cmdPropRadioDivSetup_sl_lr.centerFreq = config->frequencyTable[config->frequency].frequency;
            RF_cmdPropRadioDivSetup_sl_lr.loDivider  = tx_getLoDivider(config->frequencyTable[config->frequency].frequency);
            rfHandle = RF_open(&rfObject, &RF_prop_sl_lr, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_sl_lr, &rfParams);
            packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_SLR)); // Set packet interval to 80 ms
            cmdTxPower     = RF_cmdPropRadioDivSetup_sl_lr.txPower;
            break;

#if  !(defined Board_CC1352R1_LAUNCHXL)  && !(defined Board_CC1352P1_LAUNCHXL)  && \
     !(defined Board_CC1352P_2_LAUNCHXL) && !(defined Board_CC1352P_4_LAUNCHXL) && \
     !(defined Board_CC1312R1_LAUNCHXL)  && !(defined Board_CC26X2R1_LAUNCHXL)
        case RfSetup_Lrm:
            RF_cmdPropRadioDivSetup_lrm.centerFreq = config->frequencyTable[config->frequency].frequency;
            RF_cmdPropRadioDivSetup_lrm.loDivider  = tx_getLoDivider(config->frequencyTable[config->frequency].frequency);
            rfHandle = RF_open(&rfObject, &RF_prop_lrm, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_lrm, &rfParams);
            packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_LRM)); // Set packet interval to 500 ms
            cmdTxPower     = RF_cmdPropRadioDivSetup_lrm.txPower;
            break;
#if !(defined Board_CC1350_LAUNCHXL_433)
        case RfSetup_Ook:
            RF_cmdPropRadioDivSetup_ook.centerFreq = config->frequencyTable[config->frequency].frequency;
            RF_cmdPropRadioDivSetup_ook.loDivider  = tx_getLoDivider(config->frequencyTable[config->frequency].frequency);
            rfHandle = RF_open(&rfObject, &RF_prop_ook, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_ook, &rfParams);
            packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_OOK)); // Set packet interval to 100 ms
            cmdTxPower     = RF_cmdPropRadioDivSetup_ook.txPower;
            break;

        case RfSetup_Hsm:
{
    if (config->frequencyTable[config->frequency].frequency == CENTER_FREQ_US) // 915 MHz
    {
        RF_TxPowerTable_Entry *tbl = (RF_TxPowerTable_Entry *)RF_PROP_txPowerTable;
        RF_TxPowerTable_Value v = RF_TxPowerTable_findValue(tbl, 8);  // 8 dBm（若无精确档位会取最接近）
        RF_cmdRadioSetup_hsm.txPower = v.rawValue;                    // 关键：RF_open 前写入
    }

    rfHandle = RF_open(&rfObject, &RF_prop_hsm, (RF_RadioSetup*)&RF_cmdRadioSetup_hsm, &rfParams);

    /* 可选：双保险（有些情况下你想确保 driver 当前值也被更新） */
    if (config->frequencyTable[config->frequency].frequency == CENTER_FREQ_US)
    {
        RF_TxPowerTable_Entry *tbl = (RF_TxPowerTable_Entry *)RF_PROP_txPowerTable;
        RF_TxPowerTable_Value v = RF_TxPowerTable_findValue(tbl, 8);
        (void)RF_setTxPower(rfHandle, v);
    }

    packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_HSM));
    cmdTxPower     = RF_cmdRadioSetup_hsm.txPower;
    break;
}

#endif
#endif
#endif

#if (defined Board_CC2650DK_7ID)        || (defined Board_CC2650_LAUNCHXL)    || \
    (defined Board_CC2640R2_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL)    || \
    (defined Board_CC1350_LAUNCHXL_433) || (defined Board_CC1350STK)          || \
    (defined Board_CC26X2R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL)
        case RfSetup_Ble:
#if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
        case RfSetup_Ble5:
#endif
            rfHandle = RF_open(&rfObject, &RF_modeBle, (RF_RadioSetup*)&RF_ble_cmdRadioSetup, &rfParams);
            packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_BLE)); // Set packet interval to 100 ms
            cmdTxPower     = RF_ble_cmdRadioSetup.txPower;
            break;
#endif

#if (defined Board_CC1312R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL)
        case RfSetup_Fsk_200kbps:
            rfHandle = RF_open(&rfObject, &RF_prop_fsk_200kbps, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup_fsk_200kbps, &rfParams);
            cmdTxPower = RF_cmdPropRadioDivSetup_fsk_200kbps.txPower;
            dataOffset = 2;
            packetInterval = (uint32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_2GFSK)); // Set packet interval to 60 ms
            break;
#endif

        default:
            break;
    }
/* ===================== Force TXP = 8 dBm for 915 / 433 (ALL PHYs) ===================== */
uint16_t f_mhz = config->frequencyTable[config->frequency].frequency;

if ((f_mhz == CENTER_FREQ_US) || (f_mhz == CENTER_FREQ_433))
{
    RF_TxPowerTable_Entry *tbl = (RF_TxPowerTable_Entry *)RF_PROP_txPowerTable;

#if (defined Board_CC1310_LAUNCHXL)
    /* Range Extender 只对 868/915 那套表有意义；433 一般仍用默认表 */
    if ((f_mhz == CENTER_FREQ_US) && (config->rangeExtender == RangeExtender_En))
    {
        tbl = (RF_TxPowerTable_Entry *)RF_PROP_txPowerTableREUS;
    }
#endif

    RF_TxPowerTable_Value v = RF_TxPowerTable_findValue(tbl, 8); /* 8 dBm（无精确档位会取最接近） */
    (void)RF_setTxPower(rfHandle, v);

    /* 可选：让后续显示/匹配逻辑用最新 rawValue */
    cmdTxPower = v.rawValue;
}
/* ===================== End force ===================== */



     if (config->intervalMode == IntervalMode_Yes) {
    uint32_t t01ms = g_interval_override_0p1ms;
    if (t01ms > 0) {
        /* ticksPerMs is exact in the RF driver; do fixed-point scaling */
        uint32_t ticksPerMs = RF_convertMsToRatTicks(1);
        uint64_t ticks = ((uint64_t)t01ms * (uint64_t)ticksPerMs + 5u) / 10u; /* +5 for rounding */
        if (ticks == 0) ticks = 1; /* safety */
        packetInterval = (uint32_t)ticks;
    }
}

    
    /* Set the packet interval for display purposes */
    if(config->intervalMode == IntervalMode_Yes)
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
    if((config->rangeExtender == RangeExtender_En) &&
       (config->frequencyTable[config->frequency].frequency == CENTER_FREQ_EU) &&
       (config->rfSetup != RfSetup_Hsm))
    {
        rfPowerTable = (RF_TxPowerTable_Entry *)RF_PROP_txPowerTableREEU;
        rfPowerTableSize = RF_PROP_TX_POWER_TABLE_SIZE_REEU;
    }
    else if((config->rangeExtender == RangeExtender_En) &&
            (config->frequencyTable[config->frequency].frequency == CENTER_FREQ_US) &&
            (config->rfSetup != RfSetup_Hsm))
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
    if((config->rfSetup == RfSetup_Ble)
#if (defined Board_CC1352R1_LAUNCHXL)  || (defined Board_CC1352P1_LAUNCHXL)  || \
    (defined Board_CC1352P_2_LAUNCHXL) || (defined Board_CC1352P_4_LAUNCHXL) || \
    (defined Board_CC2640R2_LAUNCHXL)
    || (config->rfSetup == RfSetup_Ble5)
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
    if(config->highPa == HighPa_En)
    {
        powerLevelDbm = RF_TxPowerTable_MAX_DBM;
    }
    else
    {
        if((config->rfSetup == RfSetup_Ble) || (config->rfSetup == RfSetup_Ble5))
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
            if ((config->rfSetup != RfSetup_Ble)
#if (defined Board_CC1352R1_LAUNCHXL)  || (defined Board_CC1352P1_LAUNCHXL) || \
    (defined Board_CC1352P_2_LAUNCHXL) || (defined Board_CC1352P_4_LAUNCHXL)
             && (config->rfSetup != RfSetup_Ble5)
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
//         if(config->rfSetup == RfSetup_Hsm)
//         {
// #if (CCFG_FORCE_VDDR_HH == 0x1)
//             txMetrics.transmitPowerDbm = rfPowerTable[rfPowerTableSize-2].power;
// #else
//             txMetrics.transmitPowerDbm = rfPowerTable[rfPowerTableSize-3].power;
// #endif
//         }
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
    if((config->rfSetup == RfSetup_Ble)
#if (defined Board_CC26X2R1_LAUNCHXL)
    || (config->rfSetup == RfSetup_Ble5)
#endif
    )
    {
        // If BLE mode is enabled, power is set to 5 dBm
        txMetrics.transmitPowerDbm = MAX_BLE_PWR_LEVEL_DBM;
    }
#endif
#endif

    /* Determine the data rate in bits per seconds */
    txMetrics.dataRateBps = config_dataRateTable_Lut[config->rfSetup];

    menu_updateTxMetricScreen(&txMetrics);

    /* Set the frequency */
    if(config->rfSetup == RfSetup_Custom)
    {
        /* Custom settings exported from SmartRf studio shall use the exported frequency */
        RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    }
#if (defined Board_CC2650DK_7ID)        || (defined Board_CC2650_LAUNCHXL)    || \
    (defined Board_CC2640R2_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL)    || \
    (defined Board_CC1350_LAUNCHXL_433) || (defined Board_CC1350STK)          || \
    (defined Board_CC26X2R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL)
    else if((config->rfSetup == RfSetup_Ble)
#if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
            || (config->rfSetup == RfSetup_Ble5)
#endif
            )
    {
        RF_ble_cmdFs.frequency = config->frequencyTable[config->frequency].frequency;
        RF_ble_cmdFs.fractFreq = config->frequencyTable[config->frequency].fractFreq;
        RF_runCmd(rfHandle, (RF_Op*)&RF_ble_cmdFs, RF_PriorityNormal, NULL, 0);
        if(config->intervalMode == IntervalMode_No)
        {
            /* If BLE packets are sent back-to-back the synthesizer is turned
             * off after the first transmission if the advertisement channel is
             * set to 255. The channel must be directly written in the
             * advertisement command itself; it is an offset from 2300 MHz.
             */
            RF_ble_cmdBleAdvNc.channel = config->frequencyTable[config->frequency].frequency - BLE_BASE_FREQUENCY;
#if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
            RF_ble_cmdBle5AdvAux.channel = config->frequencyTable[config->frequency].frequency - BLE_BASE_FREQUENCY;
#endif
        }
    }
#endif
    else
    {
        RF_cmdFs_preDef.frequency = config->frequencyTable[config->frequency].frequency;
        RF_cmdFs_preDef.fractFreq = config->frequencyTable[config->frequency].fractFreq;
        RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs_preDef, RF_PriorityNormal, NULL, 0);
    }

    /* Get current time */
    time = RF_getCurrentTime();

    /* Create packet with incrementing sequence number and random payload */
#if (defined Board_CC1312R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL)
    if(config->rfSetup == RfSetup_Fsk_200kbps)
    {
        /* Compose the IEEE 802.15.4g header
         * Use a 2-byte CRC with whitening enabled
         */
        uint16_t ieeeHdr = IEEE_HDR_CREATE(IEEE_HDR_CRC_2BYTE, IEEE_HDR_WHTNG_EN, config->payloadLength);
        pPacket[0] = packet[0] = (uint8_t)(ieeeHdr & 0x00FF);
        pPacket[1] = packet[1] = (uint8_t)((ieeeHdr & 0xFF00) >> 8);
    }
#endif
    pPacket[dataOffset + 0] = packet[dataOffset + 0] = (uint8_t)(seqNumber >> 8);
    pPacket[dataOffset + 1] = packet[dataOffset + 1] = (uint8_t)(seqNumber++);
    uint16_t i;
    for (i = dataOffset + 2; i < config->payloadLength; i++)
    {
        pPacket[i] = packet[i] = rand();
    }

    /* Set absolute TX time to utilize automatic power management */
    time += packetInterval;

    /* Send packet */
    switch (config->rfSetup)
    {
#if !(defined Board_CC2650DK_7ID)       && !(defined Board_CC2650_LAUNCHXL)     && \
    !(defined Board_CC2640R2_LAUNCHXL)  && !(defined Board_CC1350_LAUNCHXL_433) && \
    !(defined Board_CC1352R1_LAUNCHXL)  && !(defined Board_CC1352P1_LAUNCHXL)   && \
    !(defined Board_CC1352P_2_LAUNCHXL) && !(defined Board_CC1352P_4_LAUNCHXL)  && \
    !(defined Board_CC26X2R1_LAUNCHXL)  && !(defined Board_CC1312R1_LAUNCHXL)
        case RfSetup_Hsm:
        {
            RF_cmdTxHS.startTime = time;
            cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdTxHS, RF_PriorityNormal, &tx_callback, 0);
            break;
        }
#endif

#if (defined Board_CC2650DK_7ID)        || (defined Board_CC2650_LAUNCHXL)    || \
    (defined Board_CC2640R2_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL)    || \
    (defined Board_CC1350_LAUNCHXL_433) || (defined Board_CC1350STK)          || \
    (defined Board_CC26X2R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL)
        case RfSetup_Ble:
        {
            RF_ble_cmdBleAdvNc.startTime = time;
            cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_ble_cmdBleAdvNc, RF_PriorityNormal, &tx_callback, 0);
            break;
        }
#if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
        case RfSetup_Ble5:
        {
            RF_ble_cmdBle5AdvAux.startTime = time;
            cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_ble_cmdBle5AdvAux, RF_PriorityNormal, &tx_callback, 0);
            break;
        }
#endif
#endif
#if (defined Board_CC1312R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL)
        case RfSetup_Fsk_200kbps:
        {
            RF_cmdPropTxAdv_preDef.startTime = time;
            cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv_preDef, RF_PriorityNormal, &tx_callback, 0);
            break;
        }
#endif
        default:
        {
            RF_cmdPropTx.startTime = time;
            cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, &tx_callback, 0);
            break;
        }
    }

    while (!bPacketTxDone)
    {
        /* Check, whether a button has been pressed */
        if (menu_isButtonPressed())
        {
            /* If there is an ongoing Tx command, cancel it */
            (void)RF_cancelCmd(rfHandle, cmdHandle, ABORT_GRACEFUL);
            RF_pendCmd(rfHandle, cmdHandle, (RF_EventCmdCancelled | RF_EventCmdStopped | RF_EventCmdAborted));
            RF_close(rfHandle);

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
    }

    if(packetCount == config->packetCount)
    {
        /* Do a final update to indicate all packets were sent*/
        menu_updateTxScreen(packetCount);
    }

    bPacketTxDone = false;
    packetCount = 0;
    seqNumber = 0;
    RF_close(rfHandle);
    return TestResult_Finished;
}

void tx_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if(e & RF_EventLastCmdDone)
    {
        /* Increment the packet counter */
        packetCount++;

        if(packetCount < localConfig.packetCount)
        {
            /* Increment the sequence number for the next packet but keep
             * the same data */
            pPacket[dataOffset + 0] = packet[dataOffset + 0] = (uint8_t)(seqNumber >> 8);
            pPacket[dataOffset + 1] = packet[dataOffset + 1] = (uint8_t)(seqNumber++);

            /* Set absolute TX time to utilize automatic power management */
            time += packetInterval;

            /* Send packet */
            switch (localConfig.rfSetup)
            {
        #if !(defined Board_CC2650DK_7ID)       && !(defined Board_CC2650_LAUNCHXL)     && \
            !(defined Board_CC2640R2_LAUNCHXL)  && !(defined Board_CC1350_LAUNCHXL_433) && \
            !(defined Board_CC1352R1_LAUNCHXL)  && !(defined Board_CC1352P1_LAUNCHXL)   && \
            !(defined Board_CC1352P_2_LAUNCHXL) && !(defined Board_CC1352P_4_LAUNCHXL)  && \
            !(defined Board_CC26X2R1_LAUNCHXL)  && !(defined Board_CC1312R1_LAUNCHXL)
                case RfSetup_Hsm:
                {
                    RF_cmdTxHS.startTime = time;
                    cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdTxHS, RF_PriorityNormal, &tx_callback, 0);
                    break;
                }
        #endif

        #if (defined Board_CC2650DK_7ID)        || (defined Board_CC2650_LAUNCHXL)    || \
            (defined Board_CC2640R2_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL)    || \
            (defined Board_CC1350_LAUNCHXL_433) || (defined Board_CC1350STK)          || \
            (defined Board_CC26X2R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
            (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
            (defined Board_CC1352P_4_LAUNCHXL)
                case RfSetup_Ble:
                {
                    RF_ble_cmdBleAdvNc.startTime = time;
                    cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_ble_cmdBleAdvNc, RF_PriorityNormal, &tx_callback, 0);
                    break;
                }
        #if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
            (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
            (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
                case RfSetup_Ble5:
                {
                    RF_ble_cmdBle5AdvAux.startTime = time;
                    cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_ble_cmdBle5AdvAux, RF_PriorityNormal, &tx_callback, 0);
                    break;
                }
        #endif
        #endif
        #if (defined Board_CC1312R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
            (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL)
                case RfSetup_Fsk_200kbps:
                {
                    RF_cmdPropTxAdv_preDef.startTime = time;
                    cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv_preDef, RF_PriorityNormal, &tx_callback, 0);
                    break;
                }
        #endif
                default:
                {
                    RF_cmdPropTx.startTime = time;
                    cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, &tx_callback, 0);
                    break;
                }
            }
        }
        else
        {
            bPacketTxDone = true;
        }
    }
}
