// src/main.c
/*
 * Central for ITS service: subscribe notifications, compute 1s throughput, print RSSI each second.
 *
 * 依赖：
 *  - Zephyr 蓝牙中心角色、bt_gatt_dm（GATT Discovery Manager）、bt_scan（扫描助手）、LOG、SETTINGS
 *  - 你的 include/its.h（已提供，定义了 BT_UUID_ITS / *_TX / *_RX / *_IMG_INFO 等）
 */

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include "its.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
/* TXP adaptive control disabled. */
#if 0
#include "PID.h"
#endif

LOG_MODULE_REGISTER(central_its, LOG_LEVEL_INF);

#if 0
/* 目标与平滑 */
#define RATE_TARGET_KBPS   800.0f   /* 目标吞吐（kbps），按需修改 */
#define EWMA_ALPHA         0.30f    /* 吞吐 EWMA 平滑系数 */

/* TX 功率上下限（如已有可不要重复定义） */
#ifndef TX_POWER_MAX
#define TX_POWER_MAX       20
#endif
#ifndef TX_POWER_MIN
#define TX_POWER_MIN      (-36)
#endif

/* PID：和你给定的参数保持一致 */
static PID  TXP_LOOP;
static bool pid_initialized = false;

/* 状态 */
static float  thr_ewma_kbps  = 0.0f;
static int8_t TXpower        = 20;     /* 初始功率 */
/* static int8_t last_sent_txp  = 127; */

/* -------- 可按需修改 -------- */
#define TARGET_NAME          "ZDynamic test beacon"   /* 目标外设名（若用 UUID 过滤可注释） */
#define PRINT_PERIOD_MS      1000                     /* 吞吐量与 RSSI 打印周期（ms） */
#endif
#define TARGET_NAME          "ZDynamic test beacon"
#define PRINT_PERIOD_MS      1000
#define REQ_SECURITY_LEVEL   BT_SECURITY_L2
/* --------------------------- */
static struct bt_conn *default_conn;
static bool start_cmd_sent = false;   // 全局

/* ITS 句柄 */
static uint16_t its_tx_handle = 0;
static uint16_t its_tx_ccc_handle = 0;
static uint16_t its_rx_handle = 0;
static uint16_t its_img_info_handle = 0;
static uint16_t its_img_info_ccc_handle = 0;

/* 订阅参数 */
static struct bt_gatt_subscribe_params sub_tx;
static struct bt_gatt_subscribe_params sub_img_info;

/* 1 秒吞吐统计（两路 + 总计） */
static atomic_t bytes_rx_its_tx   = ATOMIC_INIT(0);
static atomic_t bytes_rx_img_info = ATOMIC_INIT(0);
static atomic_t bytes_spi_tx      = ATOMIC_INIT(0);

/* 周期任务：每秒打印吞吐并读取 RSSI */
static struct k_work_delayable tick_work;
static struct k_work_delayable spi_stat_work;

/* ---- SPI application-frame queue ---- */
#define SPI_SLOT_COUNT     8U
#define PING_PONG_BUF_SIZE 250U
#define APP_HDR_SIZE       10U
#define APP_PAYLOAD_SIZE   (PING_PONG_BUF_SIZE - APP_HDR_SIZE)
#define SPI_PACKET_GAP_US  300U
#define APP_SYNC           0xAA
#define APP_TAIL           0xBB

struct spi_ping_pong_slot {
    uint8_t data[PING_PONG_BUF_SIZE];
    size_t  len;      /* valid application payload bytes in this slot */
    uint32_t image_size;
    uint32_t offset;  /* image offset of data[APP_HDR_SIZE] */
    uint32_t seq;
    bool    ready;
    bool    in_use;
};

static struct spi_ping_pong_slot spi_buffers[SPI_SLOT_COUNT];
static uint8_t                   spi_fill_idx;
static uint32_t                  spi_seq_next;
static uint32_t                  app_image_size;
static uint32_t                  app_image_offset;
static bool                      app_image_active;
static struct k_work             spi_work;
static atomic_t                  spi_ready = ATOMIC_INIT(0);
static struct k_spinlock         spi_lock;
static const struct device      *cc1310_spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi21));
static const struct gpio_dt_spec cc1310_spi_cs_gpio =
    GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi21), cs_gpios, 0);
static struct spi_config         cc1310_spi_cfg = {
    .frequency = 4000000U,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_MODE_CPHA,//匹配CC1310极性
    .cs = {
        .gpio = cc1310_spi_cs_gpio,
        .delay = 0U,
    },
};

/* ---- readiness flags（最小改动新增） ---- */
static atomic_t tick_started = ATOMIC_INIT(0);   /* 确保 tick 只启动一次 */
static bool     conn_param_ok = false;           /* 收到 LE 连接参数更新并命中期望区间 */
static bool     subs_ok = false;                 /* 两路 notify 订阅就绪 */

/* 你期望的连接间隔区间（单位 1.25ms），日志显示 ~11ms ≈ 9 units */
#define CI_OK_MIN_UNITS   6   /* 10.0 ms */
#define CI_OK_MAX_UNITS   100  /* 12.5 ms */

/* ------- 前置声明（最小改动新增） ------- */
static void try_start_stream_pipeline(void);
static void gatt_discover(struct bt_conn *conn);
static uint8_t its_tx_notify_cb(struct bt_conn *conn,
                                struct bt_gatt_subscribe_params *params,
                                const void *data, uint16_t length);
static uint8_t its_img_info_notify_cb(struct bt_conn *conn,
                                      struct bt_gatt_subscribe_params *params,
                                      const void *data, uint16_t length);
static void pipeline_enqueue(const uint8_t *data, uint16_t length);
static void pipeline_reset(void);
static void pipeline_finalize_partial(void);
static void spi_work_handler(struct k_work *work);
static void spi_stat_work_handler(struct k_work *work);
/* ---------------------------------------- */

static const char *hci_err_to_str(uint8_t err)
{
    switch (err) {
    case 0x00: return "Success";
    case 0x01: return "Unknown HCI Command";
    case 0x02: return "Unknown Connection Identifier";
    default:   return "Other";
    }
}

static void pipeline_reset(void)
{
    k_spinlock_key_t key = k_spin_lock(&spi_lock);
    for (size_t i = 0; i < ARRAY_SIZE(spi_buffers); ++i) {
        spi_buffers[i].ready = false;
        if (!spi_buffers[i].in_use) {
            spi_buffers[i].len = 0U;
            spi_buffers[i].image_size = 0U;
            spi_buffers[i].offset = 0U;
            spi_buffers[i].seq = 0U;
        }
    }
    spi_fill_idx = 0U;
    for (size_t i = 0; i < ARRAY_SIZE(spi_buffers); ++i) {
        if (!spi_buffers[i].in_use) {
            spi_fill_idx = (uint8_t)i;
            break;
        }
    }
    k_spin_unlock(&spi_lock, key);
}

static void app_prepare_slot(struct spi_ping_pong_slot *slot)
{
    memset(slot->data, 0, sizeof(slot->data));
    slot->image_size = app_image_size;
    slot->offset = app_image_offset;
    slot->data[0] = APP_SYNC;
    sys_put_le32(slot->image_size, &slot->data[1]);
    sys_put_le32(slot->offset, &slot->data[5]);
    slot->data[9] = APP_TAIL;
}

static void app_finalize_slot(struct spi_ping_pong_slot *slot)
{
    slot->data[0] = APP_SYNC;
    sys_put_le32(slot->image_size, &slot->data[1]);
    sys_put_le32(slot->offset, &slot->data[5]);
    slot->data[9] = APP_TAIL;
    slot->seq = spi_seq_next++;
    slot->ready = true;
}

static void pipeline_finalize_partial(void)
{
    bool submit = false;
    k_spinlock_key_t key = k_spin_lock(&spi_lock);

    for (size_t i = 0; i < ARRAY_SIZE(spi_buffers); ++i) {
        if (spi_buffers[i].len > 0U &&
            !spi_buffers[i].ready &&
            !spi_buffers[i].in_use) {
            app_finalize_slot(&spi_buffers[i]);
            if (spi_fill_idx == i) {
                spi_fill_idx = (uint8_t)((spi_fill_idx + 1U) % ARRAY_SIZE(spi_buffers));
            }
            submit = true;
        }
    }

    k_spin_unlock(&spi_lock, key);

    if (submit) {
        k_work_submit(&spi_work);
    }
}

static struct spi_ping_pong_slot *pipeline_get_fill_slot_locked(void)
{
    struct spi_ping_pong_slot *slot = &spi_buffers[spi_fill_idx];

    if (!slot->ready && !slot->in_use) {
        return slot;
    }

    for (size_t i = 0; i < ARRAY_SIZE(spi_buffers); ++i) {
        if (!spi_buffers[i].ready && !spi_buffers[i].in_use) {
            spi_fill_idx = (uint8_t)i;
            return &spi_buffers[i];
        }
    }

    return NULL;
}

static void app_advance_offset(size_t bytes)
{
    if (bytes == 0U) {
        return;
    }

    if (app_image_active && app_image_size > 0U) {
        uint32_t left = (app_image_offset < app_image_size) ?
            (app_image_size - app_image_offset) : 0U;

        if (bytes > left) {
            bytes = left;
        }
    }

    app_image_offset += (uint32_t)bytes;
    if (app_image_active && app_image_offset >= app_image_size) {
        app_image_active = false;
    }
}

static void pipeline_enqueue(const uint8_t *data, uint16_t length)
{
    if (!data || length == 0U) {
        return;
    }
    if (!atomic_get(&spi_ready)) {
        return;
    }

    size_t remaining = length;
    const uint8_t *cursor = data;

    while (remaining > 0U) {
        bool submit = false;
        bool image_done = false;
        bool was_image_active = false;
        size_t chunk;

        if (app_image_active && app_image_offset >= app_image_size) {
            LOG_WRN("Dropping %zu bytes beyond announced image size", remaining);
            return;
        }

        k_spinlock_key_t key = k_spin_lock(&spi_lock);
        struct spi_ping_pong_slot *slot = pipeline_get_fill_slot_locked();

        if (!slot) {
            k_spin_unlock(&spi_lock, key);
            app_advance_offset(remaining);
            LOG_WRN("SPI buffers busy, dropping %zu app bytes", remaining);
            return;
        }

        if (slot->len == 0U) {
            app_prepare_slot(slot);
        }

        chunk = MIN(remaining, (size_t)(APP_PAYLOAD_SIZE - slot->len));
        if (app_image_active) {
            uint32_t image_left = app_image_size - app_image_offset;
            chunk = MIN(chunk, (size_t)image_left);
        }

        if (chunk == 0U) {
            k_spin_unlock(&spi_lock, key);
            return;
        }

        memcpy(&slot->data[APP_HDR_SIZE + slot->len], cursor, chunk);
        slot->len += chunk;
        remaining -= chunk;
        cursor += chunk;
        was_image_active = app_image_active;
        app_advance_offset(chunk);
        image_done = was_image_active && (app_image_size > 0U) &&
            (app_image_offset >= app_image_size);

        if (slot->len == APP_PAYLOAD_SIZE || image_done) {
            app_finalize_slot(slot);
            spi_fill_idx = (uint8_t)((spi_fill_idx + 1U) % ARRAY_SIZE(spi_buffers));
            struct spi_ping_pong_slot *next = &spi_buffers[spi_fill_idx];
            if (!next->ready && !next->in_use) {
                next->len = 0U;
                next->image_size = 0U;
                next->offset = 0U;
                next->seq = 0U;
            }
            submit = true;
        }

        k_spin_unlock(&spi_lock, key);

        if (submit) {
            k_work_submit(&spi_work);
        }

        if (image_done && remaining > 0U) {
            LOG_WRN("Dropping %zu extra bytes after image end", remaining);
            return;
        }
    }
}

static void spi_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!atomic_get(&spi_ready)) {
        return;
    }

    while (true) {
        uint8_t idx = UINT8_MAX;
        size_t len = 0U;
        uint32_t best_seq = UINT32_MAX;

        k_spinlock_key_t key = k_spin_lock(&spi_lock);
        for (size_t i = 0; i < ARRAY_SIZE(spi_buffers); ++i) {
            if (spi_buffers[i].ready && !spi_buffers[i].in_use &&
                (idx == UINT8_MAX || spi_buffers[i].seq < best_seq)) {
                idx = (uint8_t)i;
                len = spi_buffers[i].len;
                best_seq = spi_buffers[i].seq;
            }
        }
        if (idx != UINT8_MAX) {
            spi_buffers[idx].in_use = true;
        }
        k_spin_unlock(&spi_lock, key);

        if (idx == UINT8_MAX) {
            break;
        }

        int err = 0;

        if (len > 0U && len <= APP_PAYLOAD_SIZE) {
            struct spi_buf tx_buf = {
                .buf = spi_buffers[idx].data,
                .len = PING_PONG_BUF_SIZE,
            };
            struct spi_buf_set tx = {
                .buffers = &tx_buf,
                .count = 1,
            };

            err = spi_write(cc1310_spi_dev, &cc1310_spi_cfg, &tx);
            if (err) {
                LOG_WRN("SPI write failed (%d), buf=%u", err, idx);
            } else {
                atomic_add(&bytes_spi_tx, PING_PONG_BUF_SIZE);
            }
            if (SPI_PACKET_GAP_US > 0U) {
                k_sleep(K_USEC(SPI_PACKET_GAP_US));
            }
        } else if (len > 0U) {
            LOG_WRN("SPI app buffer invalid (%zu payload bytes)", len);
        }

        key = k_spin_lock(&spi_lock);
        spi_buffers[idx].ready = false;
        spi_buffers[idx].in_use = false;
        spi_buffers[idx].len = 0U;
        spi_buffers[idx].image_size = 0U;
        spi_buffers[idx].offset = 0U;
        spi_buffers[idx].seq = 0U;
        k_spin_unlock(&spi_lock, key);
    }
}

/* ============ Notify 回调 ============ */

static uint8_t its_tx_notify_cb(struct bt_conn *conn,
                                struct bt_gatt_subscribe_params *params,
                                const void *data, uint16_t length)
{
    if (!data) {
        LOG_INF("ITS_TX notification disabled");
        return BT_GATT_ITER_CONTINUE;
    }
    atomic_add(&bytes_rx_its_tx, length);
    pipeline_enqueue(data, length);
    return BT_GATT_ITER_CONTINUE;
}

static uint8_t its_img_info_notify_cb(struct bt_conn *conn,
                                      struct bt_gatt_subscribe_params *params,
                                      const void *data, uint16_t length)
{
    if (!data) {
        LOG_INF("ITS_IMG_INFO notification disabled");
        return BT_GATT_ITER_CONTINUE;
    }
    atomic_add(&bytes_rx_img_info, length);
    const uint8_t *buf = data;
    if (length >= (1U + sizeof(struct its_img_info_t)) &&
        buf[0] == ITS_IMG_INFO_DATA_TYPE_IMG_INFO) {
        uint32_t file_size = sys_get_le32(&buf[1]);

        pipeline_finalize_partial();
        app_image_size = file_size;
        app_image_offset = 0U;
        app_image_active = (file_size > 0U);
        LOG_DBG("IMG_INFO: image_size=%u", (unsigned int)file_size);
    }
    return BT_GATT_ITER_CONTINUE;
}

/* ============ GATT 发现 ============ */

static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
    LOG_INF("GATT discovery completed (ITS)");

    /* 先清零句柄 */
    its_tx_handle = its_tx_ccc_handle = 0;
    its_rx_handle = 0;
    its_img_info_handle = its_img_info_ccc_handle = 0;

    /* ITS_TX + CCC */
    const struct bt_gatt_dm_attr *ch_tx =
        bt_gatt_dm_char_by_uuid(dm, BT_UUID_ITS_TX);
    if (ch_tx) {
        const struct bt_gatt_dm_attr *val_tx =
            bt_gatt_dm_attr_next(dm, ch_tx);
        if (val_tx) {
            its_tx_handle = val_tx->handle;
            const struct bt_gatt_dm_attr *ccc_tx =
                bt_gatt_dm_desc_by_uuid(dm, val_tx, BT_UUID_GATT_CCC);
            if (ccc_tx) {
                its_tx_ccc_handle = ccc_tx->handle;
            }
        }
    }

    /* ITS_RX */
    const struct bt_gatt_dm_attr *ch_rx =
        bt_gatt_dm_char_by_uuid(dm, BT_UUID_ITS_RX);
    if (ch_rx) {
        const struct bt_gatt_dm_attr *val_rx =
            bt_gatt_dm_attr_next(dm, ch_rx);
        if (val_rx) {
            its_rx_handle = val_rx->handle;
        }
    }

    /* ITS_IMG_INFO + CCC */
    const struct bt_gatt_dm_attr *ch_img =
        bt_gatt_dm_char_by_uuid(dm, BT_UUID_ITS_IMG_INFO);
    if (ch_img) {
        const struct bt_gatt_dm_attr *val_img =
            bt_gatt_dm_attr_next(dm, ch_img);
        if (val_img) {
            its_img_info_handle = val_img->handle;
            const struct bt_gatt_dm_attr *ccc_img =
                bt_gatt_dm_desc_by_uuid(dm, val_img, BT_UUID_GATT_CCC);
            if (ccc_img) {
                its_img_info_ccc_handle = ccc_img->handle;
            }
        }
    }

    /* 发现结束，释放 dm */
    bt_gatt_dm_data_release(dm);

    /* 订阅 ITS_TX 通知 */
    if (its_tx_handle && its_tx_ccc_handle) {
        memset(&sub_tx, 0, sizeof(sub_tx));
        sub_tx.ccc_handle   = its_tx_ccc_handle;
        sub_tx.value_handle = its_tx_handle;
        sub_tx.value        = BT_GATT_CCC_NOTIFY;
        sub_tx.notify       = its_tx_notify_cb;

        int err = bt_gatt_subscribe(default_conn, &sub_tx);
        if (err && err != -EALREADY) {
            LOG_ERR("Subscribe ITS_TX failed (%d)", err);
        } else {
            LOG_INF("Subscribed ITS_TX");
        }
    } else {
        LOG_WRN("ITS_TX or its CCC not found");
    }

    /* 订阅 ITS_IMG_INFO 通知 */
    if (its_img_info_handle && its_img_info_ccc_handle) {
        memset(&sub_img_info, 0, sizeof(sub_img_info));
        sub_img_info.ccc_handle   = its_img_info_ccc_handle;
        sub_img_info.value_handle = its_img_info_handle;
        sub_img_info.value        = BT_GATT_CCC_NOTIFY;
        sub_img_info.notify       = its_img_info_notify_cb;

        int err = bt_gatt_subscribe(default_conn, &sub_img_info);
        if (err && err != -EALREADY) {
            LOG_ERR("Subscribe ITS_IMG_INFO failed (%d)", err);
        } else {
            LOG_INF("Subscribed ITS_IMG_INFO");
        }
    } else {
        LOG_WRN("ITS_IMG_INFO or its CCC not found");
    }

    /* ---- 订阅就绪标志（最小改动新增） ---- */
    subs_ok = (its_tx_handle && its_tx_ccc_handle &&
               its_img_info_handle && its_img_info_ccc_handle);
    try_start_stream_pipeline();
}

static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
    LOG_WRN("ITS service not found on peer");
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
    LOG_ERR("Discovery error: %d", err);
}

static struct bt_gatt_dm_cb discovery_cb = {
    .completed         = discovery_complete,
    .service_not_found = discovery_service_not_found,
    .error_found       = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
    if (conn != default_conn) return;

    int err = bt_gatt_dm_start(conn, BT_UUID_ITS, &discovery_cb, NULL);
    if (err) {
        LOG_ERR("ITS discovery start failed: %d", err);
    }
}

/* ============ 就绪即启动（最小改动新增） ============ */
static void try_start_stream_pipeline(void)
{
    if (!default_conn) return;

    if (!(conn_param_ok && subs_ok)) {
        LOG_INF("Pipeline not ready: conn=%d subs=%d",
                conn_param_ok, subs_ok);
        return;
    }

    /* ✅ 0x02 发送与 tick_started 解耦 */
    if (!start_cmd_sent && its_rx_handle) {
        uint8_t cmd = 0x02;
        int werr = bt_gatt_write_without_response(default_conn,
                                                  its_rx_handle,
                                                  &cmd, 1, false);
        if (!werr) {
            start_cmd_sent = true;
            LOG_INF("ITS_RX wrote 0x02");
        } else {
            LOG_WRN("ITS_RX write 0x02 failed (%d)", werr);
            return;  // 失败就别启动 tick
        }
    }

    /* ✅ tick 只管 tick */
    if (atomic_cas(&tick_started, 0, 1)) {
        k_work_schedule(&tick_work, K_NO_WAIT);
        LOG_INF("Tick started");
    }
}


/* ============ 每秒吞吐 + RSSI ============ */

static void tick_work_handler(struct k_work *work)
{
    /* ===== 1) 吞吐统计（1s窗口） ===== */
    uint32_t b_tx   = atomic_set(&bytes_rx_its_tx, 0);
    uint32_t b_info = atomic_set(&bytes_rx_img_info, 0);
    double kbps_tx   = (b_tx   * 8.0) / 1024.0;
    double kbps_info = (b_info * 8.0) / 1024.0;
    double kbps_sum  = kbps_tx + kbps_info;
    LOG_INF("[BLE 1s] TOTAL=%.1f kbps ", kbps_sum);

#if 0
    /* ===== 2) EWMA 平滑 ===== */
    if (!pid_initialized || thr_ewma_kbps == 0.0f) {
        thr_ewma_kbps = (float)kbps_sum;
    } else {
        thr_ewma_kbps = EWMA_ALPHA * (float)kbps_sum
                      + (1.0f - EWMA_ALPHA) * thr_ewma_kbps;
    }

    /* ===== 3) PID 初始化 ===== */
    if (!pid_initialized) {
        PID_Init(&TXP_LOOP,
                 0.01f,    /* Kp */
                 0.00f,   /* Ki */
                 0.01f,   /* Kd */
                 0.5f,    /* 积分限幅 */
                 3.0f);   /* 输出限幅（每次ΔTXP最大±3） */
        pid_initialized = true;
    }

    /* ===== 4) PID 计算（参考=目标吞吐，反馈=EWMA 吞吐） ===== */
    PID_Calc(&TXP_LOOP, RATE_TARGET_KBPS, thr_ewma_kbps);

    /* 把 PID 输出当作“增量 ΔTXP” */
    int8_t txp_adaptive = (int8_t)TXP_LOOP.output;

    /* ===== 5) 累加 & 钳位 ===== */
    int16_t new_txp = (int16_t)TXpower + (int16_t)txp_adaptive;
    if (new_txp > TX_POWER_MAX) new_txp = TX_POWER_MAX;
    if (new_txp < TX_POWER_MIN) new_txp = TX_POWER_MIN;
    TXpower = (int8_t)new_txp;

    LOG_INF("[CTRL] THR_EWMA=%.1f kbps  PID_out(Δ)=%d  -> TXP=%d dBm",
            (double)thr_ewma_kbps, (int)txp_adaptive, (int)TXpower);

    /* ===== 6) RSSI打印 ===== */
#endif
    if (default_conn) {
        uint16_t conn_handle;
        int err = bt_hci_get_conn_handle(default_conn, &conn_handle);
        if (!err) {
            struct net_buf *buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI,
                                                    sizeof(struct bt_hci_cp_read_rssi));
            if (buf) {
                struct bt_hci_cp_read_rssi *cp = net_buf_add(buf, sizeof(*cp));
                cp->handle = sys_cpu_to_le16(conn_handle);

                struct net_buf *rsp = NULL;
                err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
                if (!err && rsp) {
                    struct bt_hci_rp_read_rssi *rp = (void *)rsp->data;
                    if (!rp->status) {
                        LOG_INF("RSSI: %d dBm", rp->rssi);
                    } else {
                        LOG_WRN("Read RSSI status=0x%02x", rp->status);
                    }
                    net_buf_unref(rsp);
                } else {
                    LOG_WRN("Read RSSI error: %d", err);
                }
            } else {
                LOG_WRN("No HCI buf for Read RSSI");
            }
        } else {
            LOG_WRN("get_conn_handle err=%d", err);
        }
    }

//     /* ===== 7) 首次启动命令 —— 已迁移到 try_start_stream_pipeline()，这里保留兜底可选 ===== */
// #if 0
//     if (!start_cmd_sent && default_conn && its_rx_handle) {
//         uint8_t cmd = 0x02;
//         int werr = bt_gatt_write_without_response(default_conn, its_rx_handle, &cmd, sizeof(cmd), false);
//         if (!werr) {
//             start_cmd_sent = true;
//             LOG_INF("ITS_RX wrote 0x02 (once)");
//         } else {
//             LOG_WRN("ITS_RX write 0x02 failed (%d)", werr);
//         }
//     }
// #endif

    /* ===== 8) 下发 TXP：格式 “0x07, TXP”（仅变化时发送） ===== */
    // if (default_conn && its_rx_handle && TXpower != last_sent_txp) {
    //     uint8_t payload[2] = { 0x07, (uint8_t)TXpower };
    //     int werr = bt_gatt_write_without_response(default_conn, its_rx_handle, payload, sizeof(payload), false);
    //     if (werr) {
    //         LOG_WRN("ITS_RX write TXP failed (%d)", werr);
    //     } else {
    //         last_sent_txp = TXpower;
    //         LOG_INF("ITS_RX wrote TXP: %d dBm", (int)TXpower);
    //     }
    // }

    /* ===== 9) 计划下一次 ===== */
    k_work_reschedule(&tick_work, K_MSEC(PRINT_PERIOD_MS));
}

static void spi_stat_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint32_t b_spi = atomic_set(&bytes_spi_tx, 0);
    double kbps_spi = (b_spi * 8.0) / 1024.0;
    LOG_INF("[SPI 1s] TX=%.1f kbps", kbps_spi);

    k_work_reschedule(&spi_stat_work, K_MSEC(PRINT_PERIOD_MS));
}


/* ============ 连接/扫描回调 ============ */

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
    if (!err) {
        LOG_INF("MTU exchange done");
    } else {
        LOG_WRN("MTU exchange failed (err %u)", err);
    }
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        LOG_INF("Failed to connect to %s, 0x%02x %s", addr, conn_err, hci_err_to_str(conn_err));
        if (default_conn == conn) {
            bt_conn_unref(default_conn);
            default_conn = NULL;
            int err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
            if (err) {
                LOG_ERR("Scanning failed to start (err %d)", err);
            }
        }
        return;
    }

    LOG_INF("Connected: %s", addr);

    static struct bt_gatt_exchange_params exchange_params;
    exchange_params.func = exchange_func;

    int err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_WRN("MTU exchange failed (err %d)", err);
    }

    err = bt_conn_set_security(conn, REQ_SECURITY_LEVEL);
    if (err) {
        LOG_WRN("Set security failed: %d", err);
        /* 即便失败也继续发现 */
        gatt_discover(conn);
    }

    err = bt_scan_stop();
    if ((!err) && (err != -EALREADY)) {
        LOG_ERR("Stop LE scan failed (err %d)", err);
    }

    /* ⚠️ 最小改动：不在这里启动 tick，由就绪状态机统一触发 */
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s, reason 0x%02x", addr, reason);
    if (default_conn != conn) return;

    k_work_cancel_delayable(&tick_work);
    atomic_set(&tick_started, 0);   /* 允许下次连上再启动 */
    start_cmd_sent = false;

    pipeline_reset();
    app_image_size = 0U;
    app_image_offset = 0U;
    app_image_active = false;

    /* 复位就绪标志（最小改动新增） */
    conn_param_ok = false;
    subs_ok = false;

    bt_conn_unref(default_conn);
    default_conn = NULL;

    int err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
    }
}

/* 安全层变更仍旧在这里触发发现 */
static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_INF("Security changed: %s level %u", addr, level);
    } else {
        LOG_WRN("Security failed: %s level %u err %d", addr, level, err);
    }
    gatt_discover(conn);
}

/* 连接参数更新回调（最小改动新增）：等到 ~11ms 区间再启动业务 */
static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                             uint16_t latency, uint16_t timeout)
{
    if (conn != default_conn) return;

    double ci_ms = interval * 1.25;
    LOG_INF("LE param updated: CI=%.2f ms, Lat=%u, TO=%u ms",
            ci_ms, latency, timeout * 10);

      conn_param_ok = true;

        try_start_stream_pipeline();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected,
    .disconnected     = disconnected,
    .security_changed = security_changed,
    .le_param_updated = le_param_updated,   /* 最小改动新增 */
};

/* 扫描回调与初始化 */

static void scan_filter_match(struct bt_scan_device_info *device_info,
                              struct bt_scan_filter_match *filter_match,
                              bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
    LOG_INF("Filters matched: %s connectable=%d", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    LOG_WRN("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
    default_conn = bt_conn_ref(conn);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static int scan_init(void)
{
    int err;
    struct bt_scan_init_param scan_init = {
        .connect_if_match = 1,
    };

    bt_scan_init(&scan_init);
    bt_scan_cb_register(&scan_cb);

    /* 按 UUID 过滤（推荐） */
    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_ITS);
    if (err) return err;
    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) return err;

    LOG_INF("Scan module initialized");
    return 0;
}

/* ============ 主函数 ============ */

int main(void)
{
    int err;

    k_work_init(&spi_work, spi_work_handler);
    k_work_init_delayable(&tick_work, tick_work_handler);
    k_work_init_delayable(&spi_stat_work, spi_stat_work_handler);
    pipeline_reset();

    if (!device_is_ready(cc1310_spi_dev) ||
        !gpio_is_ready_dt(&cc1310_spi_cs_gpio)) {
        LOG_ERR("CC1310 SPI interface not ready");
    } else {
        atomic_set(&spi_ready, 1);
        LOG_INF("SPI link to CC1310 ready");
        k_work_schedule(&spi_stat_work, K_NO_WAIT);
    }

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }
    LOG_INF("Bluetooth initialized");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = scan_init();
    if (err) {
        LOG_ERR("scan_init failed (err %d)", err);
        return 0;
    }

    LOG_INF("Start scanning ...");
    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return 0;
    }

    /* 主线程空转，所有工作在回调/定时任务中 */
    for (;;) {
        k_sleep(K_SECONDS(60));
    }
    return 0;
}
