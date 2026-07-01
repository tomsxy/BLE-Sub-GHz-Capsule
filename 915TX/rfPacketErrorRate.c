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

/***** Includes *****/

/* XDCtools Header files */
#include <xdc/std.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/display/Display.h>

/* Board Header files */
#include "Board.h"

/* Application specific Header files */
#include "menu.h"
#include "config.h"

#include <string.h>

/***** Variable declarations *****/

static Task_Struct txTask;
#define TX_TASK_STACKSIZE 1024
static uint8_t txTaskStack[TX_TASK_STACKSIZE];

static void txTaskFxn(UArg arg0, UArg arg1)
{
    (void)arg0;
    (void)arg1;

    tx_set_txp_dbm(8);

    ApplicationConfig config;
    memset(&config, 0, sizeof(config));
    config.rfSetup = RfSetup_Hsm;
    config.intervalMode = IntervalMode_No;
    config.testMode = TestMode_Tx;
    config.packetCount = 0xFFFFFFFFu;
    config.payloadLength = 250;
    config.frequencyTable = config_frequencyTable_Lut[RfSetup_Hsm];
    config.frequency = 2; /* 915 MHz */
#if (defined Board_CC1310_LAUNCHXL)
    config.rangeExtender = RangeExtender_Dis;
#elif ((defined Board_CC1352P1_LAUNCHXL) || (defined Board_CC1352P_2_LAUNCHXL) || \
       (defined Board_CC1352P_4_LAUNCHXL))
    config.highPa = HighPa_Dis;
#endif

    (void)tx_runTxTest(&config);

    while (1)
    {
        Task_sleep(1000);
    }
}

int main(void)
{
    /* Initialize pins and peripherals */
    Board_initGeneral();
    Display_init();
    UART_init();
    SPI_init();

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = TX_TASK_STACKSIZE;
    taskParams.stack = txTaskStack;
    Task_construct(&txTask, (Task_FuncPtr)txTaskFxn, &taskParams, NULL);

    /* Start task execution */
    BIOS_start();

    return (0);
}
