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
#include <xdc/runtime/Assert.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* TI-RTOS Header files */
#include <ti/display/Display.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/knl/Task.h>

/* Board Header files */
#include "Board.h"

/* Application specific Header files */
#include "menu.h"
#include "config.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)

/***** Variable declarations *****/

#define RX_TASK_STACKSIZE 1024
#define RX_TASK_PRIORITY  3

static Task_Struct rxTask;
static uint8_t rxTaskStack[RX_TASK_STACKSIZE];

/* RF switch / PA control pins (no buttons) */
static PIN_Handle rfPinHandle;
static PIN_State rfPinState;

static PIN_Config rfPinTable[] =
{
#if (defined Board_CC1350_LAUNCHXL) || (defined Board_CC1350_LAUNCHXL_433)
    Board_DIO1_RFSW | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
#if defined Board_CC1310_LAUNCHXL
    Board_HGM | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LNA_EN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PA_EN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
    PIN_TERMINATE,
};

static void rxAutoTask(UArg arg0, UArg arg1)
{
    (void)arg0;
    (void)arg1;

    /* Initialize UART logging in task context */
    menu_init();

    ApplicationConfig cfg = {
        .rfSetup      = RfSetup_Hsm,
        .intervalMode = IntervalMode_No,
        .testMode     = TestMode_Rx,
        .packetCount  = 0xFFFFFFFFu,
        .payloadLength = 250,
        .frequencyTable = NULL,
        .frequency = 2
#if (defined Board_CC1310_LAUNCHXL)
        , .rangeExtender = RangeExtender_Dis
#elif ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
       (defined Board_CC1352P_4_LAUNCHXL))
        , .highPa = HighPa_Dis
#endif
    };

    cfg.frequencyTable = config_frequencyTable_Lut[cfg.rfSetup];

    (void)rx_runRxTest(&cfg);

    for (;;) {
        Task_sleep(1000);
    }
}

int main(void)
{
    /* Initialize pins and peripherals */
    Board_initGeneral();

    /* Initialize UART and SPI drivers */
    UART_init();
    SPI_init();

    /* Configure RF front-end control pins (if present) */
    rfPinHandle = PIN_open(&rfPinState, rfPinTable);
    Assert_isTrue(rfPinHandle != NULL, NULL);

    /* Create auto RX task */
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = rxTaskStack;
    taskParams.stackSize = RX_TASK_STACKSIZE;
    taskParams.priority = RX_TASK_PRIORITY;
    Task_construct(&rxTask, (Task_FuncPtr)rxAutoTask, &taskParams, NULL);

    /* Start task execution */
    BIOS_start();

    return (0);
}
