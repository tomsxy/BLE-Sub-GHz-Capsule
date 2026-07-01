/*
 * Copyright (c) 2016-2018, Texas Instruments Incorporated
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
#ifndef MENU_H
#define MENU_H

#include "Board.h"
#include "config.h"

/* Contains the outcome of a test case execution */
typedef enum
{
    TestResult_Aborted = 0,
    TestResult_Finished
} TestResult;

/* Button identifiers */
typedef enum
{
    Button_Navigate = Board_PIN_BUTTON0,
    Button_Select = Board_PIN_BUTTON1
} Button;

/* Rx Operation Metrics */
typedef struct _rx_metrics_{
    uint32_t  packetsReceived; // Number of packets recevied
    uint32_t  packetsMissed;   // Number of packets missed (missing sequence numbers)
    uint32_t  packetsExpected; // Last received packet number during Rx OR
                               // configured packet count after Rx Timeout
    uint32_t  nRxTimeouts;     // Number of timeout events
    uint16_t  nPktsPerTimeout; // Each timeout is a certain number of packet intervals long
    int8_t   rssi;             // Last reported RSSI (last packet sync)
    uint16_t crcOK;            // CRC status of the received packet
    uint32_t crcNOK;           // Number of packets received with CRC error
    uint32_t throughput;       // Integer throughput calculated
    uint32_t bytesReceived;
}rx_metrics;

/* Tx Operation Metrics */
typedef struct _tx_metrics_{
    int8_t    transmitPowerDbm; // Transmission power in dBm
    uint32_t  dataRateBps;      // Data rate in bit-per-second
    uint32_t  packetIntervalMs; // Packet Interval in milliseconds
}tx_metrics;

/* Sets up tasks and kernel resources */
void menu_init();

/* Returns true if a button has been pressed (non-blocking) */
bool menu_isButtonPressed();

/* Callback for button interrupts */
void menu_notifyButtonPressed(Button button);

/* Callback for the communication task */
void menu_updateRxScreen(rx_metrics *metrics);

/* Callback for the communication task */
void menu_updateTxScreen(uint32_t packetsSent);

/* Callback for the communication task to update the transmission metrics */
void menu_updateTxMetricScreen(tx_metrics *metrics);

uint16_t menu_get_band_mhz(void);

/* Optional: RX per-packet logging (useful for latency tests) */
void menu_set_rxlog_enabled(bool en);
bool menu_get_rxlog_enabled(void);
void menu_log_rx_packet(uint16_t seq, uint16_t len, int8_t rssi, uint16_t crc_ok);
void menu_log_spi_tx(uint32_t kbps,
                     uint32_t rf_frames,
                     uint32_t spi_frames,
                     uint32_t queue_drops,
                     uint32_t len_drops,
                     uint32_t rf_gaps,
                     uint32_t rf_wraps);
void menu_log_rx_validate_ref(uint32_t size, uint32_t packets, uint32_t last_bytes,
                              uint8_t soi, uint8_t eoi, uint32_t hash);
void menu_log_rx_validate(uint32_t rf_kbps, uint32_t app_kbps, uint32_t crc_fail,
                          uint32_t frames, uint32_t ok, uint32_t rf_bad_len,
                          uint32_t no_sync, uint32_t bad_header,
                          uint32_t app_bad_len, uint32_t bad_range,
                          uint32_t gap, uint32_t wrap, uint32_t done,
                          uint32_t mismatches);


/*
Receive and transmit functions
The definitions are provided in separate source files.
*/
TestResult rx_runRxTest(const ApplicationConfig* config);
TestResult tx_runTxTest(const ApplicationConfig* config);

#endif
