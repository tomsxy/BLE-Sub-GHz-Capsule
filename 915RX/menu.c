/*
 * Minimal menu stubs for auto RX build.
 * Keeps UART logging to RX/SPI throughput and RF CRC failures.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include <ti/drivers/UART.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "menu.h"
#include "Board.h"

static UART_Handle s_uart = NULL;
static volatile bool g_rxlog_enabled = false;

static void uart_writeln(const char *fmt, ...)
{
    if (!s_uart) {
        return;
    }
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = System_vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (n > (int)sizeof(buf) - 3) {
        n = (int)sizeof(buf) - 3;
    }
    buf[n++] = '\r';
    buf[n++] = '\n';
    UART_write(s_uart, buf, (size_t)n);
}

void menu_init(void)
{
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    s_uart = UART_open(Board_UART0, &uartParams);
}

bool menu_isButtonPressed(void)
{
    return false;
}

void menu_notifyButtonPressed(Button button)
{
    (void)button;
}

void menu_updateRxScreen(rx_metrics *metrics)
{
    static uint32_t lastTicks = 0;
    static uint32_t lastBytes = 0;
    static bool firstPrint = true;

    if (!metrics) {
        return;
    }

    uint32_t now = Clock_getTicks();
    if (lastTicks == 0) {
        lastTicks = now;
        lastBytes = metrics->bytesReceived;
        if (firstPrint) {
            firstPrint = false;
            uart_writeln("915RX: 0 kbps, CRC_FAIL=%u", (unsigned)metrics->crcNOK);
        }
        return;
    }

    uint32_t elapsedTicks = now - lastTicks;
    uint32_t elapsedUs;

    if (Clock_tickPeriod != 0) {
        elapsedUs = elapsedTicks * (uint32_t)Clock_tickPeriod;
    } else {
        elapsedUs = elapsedTicks * 1000U;
    }

    if (elapsedUs >= 1000000U) {
        uint32_t bytesNow = metrics->bytesReceived;
        uint32_t deltaBytes = bytesNow - lastBytes;
        uint32_t tp_bps = (elapsedUs == 0) ? 0U :
            (uint32_t)((uint64_t)deltaBytes * 8ULL * 1000000ULL / (uint64_t)elapsedUs);

        lastTicks = now;
        lastBytes = bytesNow;

        uart_writeln("915RX: %u kbps, CRC_FAIL=%u",
                     (unsigned)(tp_bps / 1000U),
                     (unsigned)metrics->crcNOK);
    }
}

void menu_updateTxScreen(uint32_t packetsSent)
{
    (void)packetsSent;
}

void menu_updateTxMetricScreen(tx_metrics *metrics)
{
    (void)metrics;
}

uint16_t menu_get_band_mhz(void)
{
    return 915;
}

void menu_set_rxlog_enabled(bool en)
{
    g_rxlog_enabled = en;
}

bool menu_get_rxlog_enabled(void)
{
    return g_rxlog_enabled;
}

void menu_log_rx_packet(uint16_t seq, uint16_t len, int8_t rssi, uint16_t crc_ok)
{
    (void)seq;
    (void)len;
    (void)rssi;
    (void)crc_ok;
}

void menu_log_spi_tx(uint32_t kbps,
                     uint32_t rf_frames,
                     uint32_t spi_frames,
                     uint32_t queue_drops,
                     uint32_t len_drops,
                     uint32_t rf_gaps,
                     uint32_t rf_wraps)
{
    uart_writeln("SPI TX: %u kbps, rf=%lu spi=%lu qdrop=%lu ldrop=%lu rgap=%lu rwrap=%lu",
                 (unsigned)kbps,
                 (unsigned long)rf_frames,
                 (unsigned long)spi_frames,
                 (unsigned long)queue_drops,
                 (unsigned long)len_drops,
                 (unsigned long)rf_gaps,
                 (unsigned long)rf_wraps);
}

void menu_log_rx_validate_ref(uint32_t size, uint32_t packets, uint32_t last_bytes,
                              uint8_t soi, uint8_t eoi, uint32_t hash)
{
    uart_writeln("RF APP validate only: SPI TX bypassed");
    uart_writeln("FIXED REF: size=%lu pkts=%lu last=%lu SOI=%u EOI=%u hash=%08lx",
                 (unsigned long)size,
                 (unsigned long)packets,
                 (unsigned long)last_bytes,
                 (unsigned)soi,
                 (unsigned)eoi,
                 (unsigned long)hash);
}

void menu_log_rx_validate(uint32_t rf_kbps, uint32_t app_kbps, uint32_t crc_fail,
                          uint32_t frames, uint32_t ok, uint32_t rf_bad_len,
                          uint32_t no_sync, uint32_t bad_header,
                          uint32_t app_bad_len, uint32_t bad_range,
                          uint32_t gap, uint32_t wrap, uint32_t done,
                          uint32_t mismatches)
{
    uart_writeln("915RX=%lu APP=%lu CRC_FAIL=%lu frm=%lu ok=%lu rbl=%lu ns=%lu bh=%lu bl=%lu br=%lu gap=%lu wrap=%lu done=%lu mis=%lu",
                 (unsigned long)rf_kbps,
                 (unsigned long)app_kbps,
                 (unsigned long)crc_fail,
                 (unsigned long)frames,
                 (unsigned long)ok,
                 (unsigned long)rf_bad_len,
                 (unsigned long)no_sync,
                 (unsigned long)bad_header,
                 (unsigned long)app_bad_len,
                 (unsigned long)bad_range,
                 (unsigned long)gap,
                 (unsigned long)wrap,
                 (unsigned long)done,
                 (unsigned long)mismatches);
}

#if 0
/*
 * Copyright (c) 2016-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Modified: CLI control via UART input (no buttons needed).
 */

/***** Includes *****/
/* TI-RTOS Header files */
#include <ti/drivers/rf/RF.h>
#include <ti/display/Display.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include <xdc/runtime/Assert.h>
#include <xdc/runtime/System.h>

/* C lib */
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

/* Application specific Header files */
#include "menu.h"
#include "config.h"

#include <Board.h>

#ifdef Board_SYSCONFIG_PREVIEW
#include "smartrf_settings/smartrf_settings.h"
#else
#include "smartrf_settings/smartrf_settings_predefined.h"
#endif //Board_SYSCONFIG_PREVIEW

/***** Defines *****/
#define MENU_TASK_STACKSIZE     2048
#define MENU_TASK_PRIORITY      2
#define MENU_TASK_NAME          "menuTask"
#define MENU_EVENT_NAME         "menuEvent"

#define CLI_LINE_MAX            128

/* Backward-compatible wrappers (ms) so old menu.c calls still link */
void tx_set_interval_override_ms(uint32_t ms)
{
    tx_set_interval_override_0p1ms(ms * 10U);
}

uint32_t tx_get_interval_override_ms(void)
{
    return tx_get_interval_override_0p1ms() / 10U; /* floor */
}

extern void     tx_set_interval_override_0p1ms(uint32_t t01ms);  /* 0=clear */
extern uint32_t tx_get_interval_override_0p1ms(void);

extern int8_t tx_get_last_txp_dbm(void);

static volatile bool g_txp_print_pending = false;


/* RX per-packet logging switch (off by default to avoid flooding UART) */
static volatile bool g_rxlog_enabled = false;

void menu_set_rxlog_enabled(bool en)
{
    g_rxlog_enabled = en;
}

bool menu_get_rxlog_enabled(void)
{
    return g_rxlog_enabled;
}


static bool cli_set_phy(const char *phy);
static bool cli_set_band(uint32_t mhz);
static int  cli_find_freq_index(uint32_t mhz);

static uint16_t g_band_mhz = 915;
uint16_t menu_get_band_mhz(void) { return g_band_mhz; }

/***** Variable declarations *****/
static Task_Struct menu_task;
static uint8_t taskStack[MENU_TASK_STACKSIZE];

static Event_Struct menu_event;

/* CLI UART handle (for input+output) */
static UART_Handle cliUart = NULL;

/* Abort flag: reuse menu_isButtonPressed() so tx/rx can stop */
static volatile bool g_abort = false;

/* Holds the configuration for the current test case (official type!) */
static ApplicationConfig config =
{
    RfSetup_Fsk,
    IntervalMode_Yes,
    TestMode_Rx,
    10,
    30,
    NULL,
    0,
#if (defined Board_CC1310_LAUNCHXL)
    RangeExtender_Dis
#elif ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
       (defined Board_CC1352P_4_LAUNCHXL))
    HighPa_Dis
#endif
};

/***** Prototypes *****/
/* Menu task main function */
void menu_runTask(UArg arg0, UArg arg1);
static bool cli_parse_ms_1dp_to_0p1ms(const char *s, uint32_t *out01ms)
{
    /* parse "X" or "X.Y" (Y is one digit) -> tenths of ms */
    if (!s || !out01ms) return false;

    uint32_t intpart = 0;
    uint32_t frac = 0;
    bool seen_digit = false;

    /* int part */
    while (*s >= '0' && *s <= '9') {
        seen_digit = true;
        intpart = intpart * 10u + (uint32_t)(*s - '0');
        s++;
    }

    if (!seen_digit) return false;

    if (*s == '\0') {
        *out01ms = intpart * 10u;
        return true;
    }

    if (*s != '.') return false;
    s++;

    /* exactly 1 fractional digit */
    if (*s < '0' || *s > '9') return false;
    frac = (uint32_t)(*s - '0');
    s++;

    /* no extra chars */
    if (*s != '\0') return false;

    *out01ms = intpart * 10u + frac;
    return true;
}

/***** CLI helpers *****/
static void cli_writeln(const char *fmt, ...)
{
    if (!cliUart) return;
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = System_vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 3) n = (int)sizeof(buf) - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';
    UART_write(cliUart, buf, (size_t)n);
}

void menu_log_rx_packet(uint16_t seq, uint16_t len, int8_t rssi, uint16_t crc_ok)
{
    if (!g_rxlog_enabled) return;
    /* Single-line marker for Python timing script */
    cli_writeln("[RX_PKT] seq=%u, len=%u, RSSI=%d dBm, crc_ok=%u",
               (unsigned)seq, (unsigned)len, (int)rssi, (unsigned)crc_ok);
}



static void cli_write(const char *s)
{
    if (!cliUart || !s) return;
    UART_write(cliUart, s, strlen(s));
}

static void cli_prompt(void)
{
    cli_write("\r\n> ");
}

static int cli_readline(char *out, int outMax)
{
    if (!cliUart || !out || outMax < 2) return 0;

    int len = 0;
    for (;;) {
        char c = 0;
        UART_read(cliUart, &c, 1);

        if (c == '\r' || c == '\n') {
            cli_write("\r\n");
            out[len] = '\0';
            return len;
        }

        if (c == '\b' || c == 0x7F) {
            if (len > 0) {
                len--;
                cli_write("\b \b");
            }
            continue;
        }

        if ((unsigned char)c < 0x20 || (unsigned char)c > 0x7E) {
            continue;
        }

        if (len < outMax - 1) {
            out[len++] = c;
            UART_write(cliUart, &c, 1); /* echo */
        }
    }
}

static int cli_tokenize(char *line, char *argv[], int argvMax)
{
    int argc = 0;
    char *p = line;

    while (*p && argc < argvMax) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }
    return argc;
}

static void cli_help(void)
{
    cli_writeln("Commands:");
    cli_writeln("  help                 Show help");
    cli_writeln("  status               Print current config");
    cli_writeln("  txp                  Show current TX power (raw + dBm if matched)");
    cli_writeln("  max                  Max throughput defaults (payload=254, intervalMode=No, pktCount=INF)");
    cli_writeln("  interval off         Back-to-back (max throughput)");
    cli_writeln("  interval <ms>        Enable interval mode and override interval in ms (affects TX rate)");
    cli_writeln("  start rx|tx          Start RX/TX (runs until 'stop')");
    cli_writeln("  stop                 Stop current test (abort)");
    cli_writeln("  band 433|868|915      Select band");
    cli_writeln("  phy  hsm|fsk|lrm      Select PHY");
    
}


/* 简化：433/868/915 直接映射到 sub1 table index，避免依赖字段名 */
static int cli_find_freq_index(uint32_t mhz)
{
    if (mhz == 433) return 0; /* 433.92 */
    if (mhz == 868) return 1; /* 868.0  */
    if (mhz == 915) return 2; /* 915.0  */
    return -1;
}

static bool cli_set_phy(const char *phy)
{
    if (!strcmp(phy, "hsm")) {
        config.rfSetup = RfSetup_Hsm;
    } else if (!strcmp(phy, "fsk")) {
        config.rfSetup = RfSetup_Fsk;
    } else if (!strcmp(phy, "lrm")) {
        config.rfSetup = RfSetup_Lrm;
    } else {
        return false;
    }

    /* 切 PHY 后同步频率表 */
    config.frequencyTable = config_frequencyTable_Lut[config.rfSetup];
    return true;
}

static bool cli_set_band(uint32_t mhz)
{
    int idx = cli_find_freq_index(mhz);
    if (idx < 0) {
        return false;
    }

    /* 只改频点索引，不改 rfSetup */
    config.frequency = (uint32_t)idx;

    /* 保持：frequencyTable 仍然跟随当前 rfSetup */
    config.frequencyTable = config_frequencyTable_Lut[config.rfSetup];
    return true;
}


/* ---- TXP helper: map current cmd txPower -> dBm by scanning table ---- */
static uint16_t cli_get_cmd_txpower_raw(const ApplicationConfig *cfg)
{
    switch (cfg->rfSetup) {
        case RfSetup_Fsk: return RF_cmdPropRadioDivSetup_fsk.txPower;
        case RfSetup_Lrm: return RF_cmdPropRadioDivSetup_lrm.txPower;
        case RfSetup_Sl_lr: return RF_cmdPropRadioDivSetup_sl_lr.txPower;
        case RfSetup_Ook: return RF_cmdPropRadioDivSetup_ook.txPower;
        case RfSetup_Hsm: return RF_cmdRadioSetup_hsm.txPower;
        default: break;
    }
    return 0;
}


static void cli_show_txp(void)
{
    uint16_t raw = cli_get_cmd_txpower_raw(&config);

    const RF_TxPowerTable_Entry *tbl = RF_PROP_txPowerTable;

    int found = 0;
    int8_t dbm = 0;
    uint32_t i = 0;

    if (tbl != NULL) {
        for (i = 0; ; i++) {
            if (tbl[i].power == RF_TxPowerTable_INVALID_DBM) {
                break;
            }

            /* 关键：value 是 struct/union，要取 rawValue */
            if (tbl[i].value.rawValue == raw) {
                dbm = tbl[i].power;
                found = 1;
                break;
            }
        }
    }

    int8_t p = tx_get_last_txp_dbm();
if (p != RF_TxPowerTable_INVALID_DBM) {
    cli_writeln("TXP (driver): %d dBm", (int)p);
} else {
    cli_writeln("TXP (driver): N/A (run 'start tx' once to let RF driver set/read tx power)");
}

}



static void cli_status(void)
{
    uint32_t t01 = tx_get_interval_override_0p1ms();
cli_writeln("Config: rfSetup=%d, intervalMode=%d, testMode=%d, freqIndex=%u, pktCount=%u, payload=%u, interval=%.1f ms",
           (int)config.rfSetup,
           (int)config.intervalMode,
           (int)config.testMode,
           (unsigned)config.frequency,
           (unsigned)config.packetCount,
           (unsigned)config.payloadLength,
           (double)t01 / 10.0);

}



    static void cli_apply(char *line)
{
    char *argv[8] = {0};
    int argc = cli_tokenize(line, argv, 8);
    if (argc == 0) return;

    if (!strcmp(argv[0], "help") || !strcmp(argv[0], "?")) {
        cli_help();
        return;
    }

    if (!strcmp(argv[0], "status")) {
        cli_status();
        return;
    }

    if (!strcmp(argv[0], "txp")) {
        cli_show_txp();
        return;
    }


    if (!strcmp(argv[0], "payload") && argc >= 2) {
    uint32_t n = (uint32_t)atoi(argv[1]);
    uint32_t maxLen = 254;

    /* BLE 模式下 payload 最大 30（保持原工程限制） */
//     if ((config.rfSetup == RfSetup_Ble)
// #if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
//     (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
//     (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
//         || (config.rfSetup == RfSetup_Ble5)
// #endif
//         ) {
//         maxLen = 30;
//     }

    if (n < 2 || n > maxLen) {
        cli_writeln("Bad payload. Use 2..%u for current PHY", (unsigned)maxLen);
        return;
    }

    /* HSM 要求偶数长度 */
    if ((config.rfSetup == RfSetup_Hsm) && (n & 0x1u)) {
        cli_writeln("Bad payload for HSM. Must be even (e.g., 244)");
        return;
    }

    config.payloadLength = n;
    cli_writeln("Payload set: %u bytes", (unsigned)n);
    return;
}

if (!strcmp(argv[0], "rxlog") && argc >= 2) {
    if (!strcmp(argv[1], "on") || !strcmp(argv[1], "1")) {
        menu_set_rxlog_enabled(true);
        cli_writeln("RX per-packet log: ON");
        return;
    }
    if (!strcmp(argv[1], "off") || !strcmp(argv[1], "0")) {
        menu_set_rxlog_enabled(false);
        cli_writeln("RX per-packet log: OFF");
        return;
    }
    cli_writeln("Bad rxlog. Use: rxlog on|off");
    return;
}


    if (!strcmp(argv[0], "interval") && argc >= 2) {
    if (!strcmp(argv[1], "off") || !strcmp(argv[1], "0")) {
        config.intervalMode = IntervalMode_No;
        tx_set_interval_override_0p1ms(0);
        cli_writeln("Interval: OFF (back-to-back).");
        return;
    } else {
        uint32_t t01 = 0;
        if (!cli_parse_ms_1dp_to_0p1ms(argv[1], &t01) || t01 == 0) {
            cli_writeln("Bad interval. Use: interval off | interval <ms> | interval <ms.1>");
            return;
        }
        config.intervalMode = IntervalMode_Yes;
        tx_set_interval_override_0p1ms(t01);
        cli_writeln("Interval: ON, override=%u (%.1f ms)", (unsigned)t01, (double)t01/10.0);
        return;
    }
}

    if (!strcmp(argv[0], "stop")) {
        g_abort = true;
        cli_writeln("Stopping...");
        return;
    }

    if (!strcmp(argv[0], "max")) {
        config.payloadLength = 254;
        config.packetCount = 0xFFFFFFFFu;
        config.intervalMode = IntervalMode_No; /* TRIG_NOW/back-to-back */
        tx_set_interval_override_ms(0);        /* max 模式强制清 override */
        cli_writeln("MaxTP applied.");
        return;
    }

    if (!strcmp(argv[0], "phy") && argc >= 2) {
    if (!cli_set_phy(argv[1])) {
        cli_writeln("Bad phy. Use: hsm|fsk|lrm");
        return;
    }
    cli_writeln("PHY set: rfSetup=%d", (int)config.rfSetup);
    return;
}

if (!strcmp(argv[0], "band") && argc >= 2) {
    uint32_t mhz = (uint32_t)atoi(argv[1]);
    if (!cli_set_band(mhz)) {
        cli_writeln("Bad band. Use: 433|868|915");
        return;
    }
    cli_writeln("Band set: %u MHz (freqIndex=%u, rfSetup=%d)",
                (unsigned)mhz, (unsigned)config.frequency, (int)config.rfSetup);
    return;
}


    if (!strcmp(argv[0], "start") && argc >= 2) {
        g_abort = false;

        if (!strcmp(argv[1], "rx")) {
            config.testMode = TestMode_Rx;
            cli_writeln("Starting RX...");
            (void)rx_runRxTest(&config);
            cli_writeln("RX exited.");
            return;
        }

        if (!strcmp(argv[1], "tx")) {
    config.testMode = TestMode_Tx;
    cli_writeln("Starting TX...");
    g_txp_print_pending = true;
    (void)tx_runTxTest(&config);
    cli_writeln("TX exited.");
    return;
}
        if (!strcmp(argv[1], "test")) {
    /* Same as TX but only send ONE packet, then return */
    config.testMode = TestMode_Tx;

    uint32_t oldCount = config.packetCount;
    config.packetCount = 1;

    cli_writeln("Starting TEST (one packet)...");
    g_txp_print_pending = true;
    (void)tx_runTxTest(&config);
    cli_writeln("TEST exited.");

    config.packetCount = oldCount;
    return;
}


        cli_writeln("Bad start. Use: start rx|tx");
        return;
    }

    cli_writeln("Unknown cmd. Type 'help'.");
}

/***** Public API *****/
void menu_init()
{
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.instance->name = (xdc_String)MENU_TASK_NAME;
    taskParams.stackSize = MENU_TASK_STACKSIZE;
    taskParams.stack = taskStack;
    taskParams.priority = MENU_TASK_PRIORITY;
    Task_construct(&menu_task, (Task_FuncPtr)&menu_runTask, &taskParams, NULL);

    Event_Params eventParams;
    Event_Params_init(&eventParams);
    eventParams.instance->name = (xdc_String)(MENU_EVENT_NAME);
    Event_construct(&menu_event, &eventParams);
}

bool menu_isButtonPressed()
{
    return g_abort;
}

void menu_notifyButtonPressed(Button button)
{
    (void)button;
    g_abort = true;
}

/* RX: print once per second (throttle) */
void menu_updateRxScreen(rx_metrics *metrics)
{
    static uint32_t lastTicks = 0;
static uint32_t lastBytes = 0;

uint32_t now = Clock_getTicks();

if (lastTicks == 0) {
    lastTicks = now;
    lastBytes = metrics->bytesReceived;
    return;
}

uint32_t elapsedTicks = now - lastTicks;
uint32_t elapsedUs;

if (Clock_tickPeriod != 0) {
    elapsedUs = elapsedTicks * (uint32_t)Clock_tickPeriod; // tickPeriod is us/tick
} else {
    elapsedUs = elapsedTicks * 1000U; // fallback
}

if (elapsedUs >= 1000000U) {
    uint32_t bytesNow   = metrics->bytesReceived;
    uint32_t deltaBytes = bytesNow - lastBytes;

    uint32_t tp_bps = (elapsedUs == 0) ? 0 :
        (uint32_t)((uint64_t)deltaBytes * 8ULL * 1000000ULL / (uint64_t)elapsedUs);

    lastTicks = now;
    lastBytes = bytesNow;

    cli_writeln("[TP/1s] pkts_ok=%u, RSSI=%d dBm, TP=%u bps (%u kbps)",
                (unsigned)metrics->packetsReceived,
                (int)metrics->rssi,
                (unsigned)tp_bps,
                (unsigned)(tp_bps / 1000U));
}

}

void menu_updateTxScreen(uint32_t packetsSent)
{
    (void)packetsSent;
}

void menu_updateTxMetricScreen(tx_metrics *metrics)
{
    if (!metrics) return;

    if (g_txp_print_pending) {
        g_txp_print_pending = false;
        cli_writeln("[TX] TXP=%d dBm", (int)metrics->transmitPowerDbm);
    }
}



/***** Task entry *****/
void menu_runTask(UArg arg0, UArg arg1)
{
    (void)arg0;
    (void)arg1;

    /* Open UART for CLI input/output */
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    cliUart = UART_open(Board_UART0, &uartParams);
    Assert_isTrue(cliUart != NULL, NULL);

    cli_writeln("");
    cli_writeln("rfPacketErrorRate (CLI mode)");
    cli_writeln("Type 'help' for commands.");

    /* Default: MaxTP baseline */
    config.payloadLength = 254;
    config.packetCount = 0xFFFFFFFFu;
    config.intervalMode = IntervalMode_No;
    config.frequencyTable = config_frequencyTable_Lut[config.rfSetup];
    /* Default: 915 + HSM */
    cli_set_band(915);


    for (;;) {
        char line[CLI_LINE_MAX];
        cli_prompt();
        int n = cli_readline(line, sizeof(line));
        if (n > 0) {
            cli_apply(line);
        }
    }
}

#endif
