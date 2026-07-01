/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief WiFi Camera Demo
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(WiFiCam, CONFIG_LOG_DEFAULT_LEVEL);

#if defined(CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT) || NRF_CLOCK_HAS_HFCLK192M
#include <nrfx_clock.h>
#endif

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>   /* 调试计数用 */
// #include <zephyr/drivers/video.h>
// #include <zephyr/drivers/video/arducam_mega.h>

#include "app_bluetooth.h"
#include "socket_util.h"
#include "fixed_image_40.h"

/**********External Resources START**************/
extern struct sockaddr_in pc_addr;
/**********External Resources END**************/

#define RESET_CAMERA               0xFF
#define SET_PICTURE_RESOLUTION     0x01
#define SET_VIDEO_RESOLUTION       0x02
#define SET_BRIGHTNESS             0x03
#define SET_CONTRAST               0x04
#define SET_SATURATION             0x05
#define SET_EV                     0x06
#define SET_WHITEBALANCE           0x07
#define SET_SPECIAL_EFFECTS        0x08
#define SET_FOCUS_ENABLE           0x09
#define SET_EXPOSURE_GAIN_ENABLE   0x0A
#define SET_WHITE_BALANCE_ENABLE   0x0C
#define SET_MANUAL_GAIN            0x0D
#define SET_MANUAL_EXPOSURE        0x0E
#define GET_CAMERA_INFO            0x0F
#define TAKE_PICTURE               0x10
#define SET_SHARPNESS              0x11
#define DEBUG_WRITE_REGISTER       0x12
#define STOP_STREAM                0x21
#define GET_FRM_VER_INFO           0x30
#define GET_SDK_VER_INFO           0x40
#define SET_IMAGE_QUALITY          0x50
#define SET_LOWPOWER_MODE          0x60

extern const uint8_t fixed_image_data[FIXED_IMAGE_SIZE];

/* ========== 客户端与命令 ========= */
enum client_type {
    client_type_NONE,
    client_type_BLE,
    client_type_SOCKET
};
enum app_command_type {
    APPCMD_CAM_COMMAND, APPCMD_TAKE_PICTURE, APPCMD_START_STOP_STREAM, APPCMD_SOCKET_RX
};
struct app_command_t {
    enum app_command_type type;
    enum client_type mode;
    uint8_t cam_cmd;
    /* 放大，避免 socket 回调中的数据被截断 */
    uint8_t data[128];
};
K_MSGQ_DEFINE(msgq_app_commands, sizeof(struct app_command_t), 8, 4);

struct client_state {
    uint8_t req_stream      : 1;
    uint8_t req_stream_stop : 1;
    uint8_t stream_active   : 1;
};
 struct client_state client_state_ble    = {0};
 struct client_state client_state_socket = {0};

/* ========== 速率统计 & 调试计数 ========= */
static void on_timer_count_bytes_func(struct k_timer *timer);
K_TIMER_DEFINE(m_timer_count_bytes, on_timer_count_bytes_func, NULL);
static int counted_bytes_sent = 0;
/* 近 1 秒发送/填充的块数（仅 Socket），用于速率打印 */
static atomic_t dbg_chunks_sent   = ATOMIC_INIT(0);
static atomic_t dbg_chunks_filled = ATOMIC_INIT(0);

/* ========== Socket 帧头/尾模板 ========= */
static uint8_t socket_head_and_tail[] = {0xff, 0xaa, 0x00, 0xff, 0xbb};

/* ========== 前置声明（修复隐式声明报错） ========= */
static void client_request_single_capture(struct client_state *state);
static void client_request_stream_start_stop(struct client_state *state, bool start);
static int  client_check_start_request(struct client_state *state);
static int  client_check_stop_request(struct client_state *state);

/* ========== 仅 Wi-Fi 使用：Ping-Pong 双缓冲 =========
   - 两块固定缓存严格交替
   - 生产者：pp2_try_fill 负责把源数据尽可能写入当前写块，写满就翻转并标记 ready
   - 消费者：pp2_try_send_socket 负责把 ready 的读块发出去，发完清空并翻转
*/
#define PP_CHUNK_SZ  4096  /* 每块缓存大小，可根据协议改为 1024/1460/4096 等 */
typedef struct {
    uint8_t  buf[2][PP_CHUNK_SZ];
    uint16_t len[2];     /* 每块已写入的有效长度 */
    uint8_t  ready[2];   /* 0=空闲, 1=待发送 */
    uint8_t  rd;         /* 当前读块索引 */
    uint8_t  wr;         /* 当前写块索引 */
} ppbuf2_t;

/* 仅为 Socket 保留一套 ping-pong 双缓冲与帧状态 */
static ppbuf2_t s_pp_sock;
static ppbuf2_t s_pp_ble;
static size_t   s_img_pos_sock = 0;
static size_t   s_img_pos_ble = 0;

static inline void pp2_init(ppbuf2_t *q) {
    q->len[0] = q->len[1] = 0;
    q->ready[0] = q->ready[1] = 0;
    q->rd = 0;
    q->wr = 0;
}

/* === 调试：随时打印 ping-pong 状态（放在全局变量定义之后，避免未声明引用） === */
static inline void pp2_dump_state(const char *tag, const ppbuf2_t *q)
{
    LOG_DBG("[%s] rd=%u wr=%u | len[0]=%u rdy[0]=%u | len[1]=%u rdy[1]=%u | s_img_pos=%u/%u",
            tag, q->rd, q->wr,
            (unsigned)q->len[0], (unsigned)q->ready[0],
            (unsigned)q->len[1], (unsigned)q->ready[1],
            (unsigned)s_img_pos_sock, (unsigned)FIXED_IMAGE_SIZE);
}

/* 生产者：尽可能把 src 拷入当前写块；如果两块都待发（写不进），返回 0 */
/* 生产者：尽可能把 src 拷入当前写块；如果两块都待发（写不进），返回 0（不再 warn） */
static size_t pp2_try_fill(ppbuf2_t *q, const uint8_t *src, size_t len)
{
    size_t total = 0;

    while (len > 0) {
        /* 两块都 ready：背压，静默返回，让消费者去清 */
        if (q->ready[0] && q->ready[1]) {
            return total;  /* 不再打印 WR blocked 警告 */
        }
        /* 当前写块如果已标 ready，切到另一块 */
        if (q->ready[q->wr]) {
            q->wr ^= 1;
            continue;
        }

        size_t cur  = q->len[q->wr];
        size_t room = PP_CHUNK_SZ - cur;
        if (room == 0) {
            q->ready[q->wr] = 1;   /* 填满 => 标 ready 并换块 */
            q->wr ^= 1;
            continue;
        }

        /* 限制一次只写“当前写块剩余空间”，避免一口气把两块都顶满 */
        size_t chunk = (len < room) ? len : room;
        memcpy(&q->buf[q->wr][cur], src, chunk);
        q->len[q->wr] += chunk;

        src  += chunk;
        len  -= chunk;
        total += chunk;

        if (q->len[q->wr] >= PP_CHUNK_SZ) {
            q->ready[q->wr] = 1;
            q->wr ^= 1;
        }
    }
    return total;
}


/* 消费者：如果有 ready 块就发送；成功后清空并翻转读指针。返回本次发送字节数。 */
static size_t pp2_try_send_socket(ppbuf2_t *q)
{
    if (!q->ready[q->rd]) return 0;

    const uint8_t *p = q->buf[q->rd];
    int n = q->len[q->rd];

    int left = n;
    while (left >= 1024) {
        cam_send(p, 1024);
        p    += 1024;
        left -= 1024;
    }
    if (left > 0) {
        cam_send(p, left);
    }

    // /* 关键：给网络栈一点时间处理已提交的数据 */
    // k_yield(); /* 或者 k_sleep(K_MSEC(0)); */
    // k_sleep(K_MSEC(1));

    q->ready[q->rd] = 0;
    q->len[q->rd]   = 0;
    q->rd ^= 1;

    counted_bytes_sent += n;
    return (size_t)n;
}

/* BLE 消费者：把 ready 的块整块交给 BLE 层发送（底层会自动按 MTU 切片） */
static size_t pp2_try_send_ble(ppbuf2_t *q)
{
    if (!q->ready[q->rd]) {
        return 0;
    }

    const uint8_t *p = q->buf[q->rd];
    int n = q->len[q->rd];

    app_bt_send_picture_data(p, n);   /* 内部已自动切片 */

    q->ready[q->rd] = 0;
    q->len[q->rd]   = 0;
    q->rd ^= 1;

    counted_bytes_sent += n;
    return (size_t)n;
}


static inline bool pp2_all_empty(const ppbuf2_t *q) {
    return (q->ready[0] == 0 && q->len[0] == 0) &&
           (q->ready[1] == 0 && q->len[1] == 0);
}

static inline void start_new_frame_sock(void) {
    pp2_init(&s_pp_sock);
    s_img_pos_sock = 0;
    pp2_dump_state("FRAME-RESET", &s_pp_sock);
}

/* ========== 发送函数 ========= */
static inline void cam_send_picture_data_socket(const uint8_t *data, int length)
{
    /* 目前仅在 pp2_try_send_socket 内累加统计，这里保留给 BLE 共用场景 */
    counted_bytes_sent += length;
    while (length >= 1024) {
        cam_send(data, 1024);
        data += 1024;
        length -= 1024;
    }
    if (length > 0) cam_send(data, length);
}
static inline void cam_send_picture_data_ble(const uint8_t *data, int length)
{
    counted_bytes_sent += length;
    app_bt_send_picture_data(data, length);
}

/* 修正：不要 &socket_head_and_tail 这种“数组指针” */
static void cam_to_pc_command_send(uint8_t type, uint8_t *buffer, uint32_t length)
{
    socket_head_and_tail[2] = type;
    cam_send(socket_head_and_tail, 3);
    if (length != 0) {
        cam_send((uint8_t *)&length, 4); /* 注意端序：此处是小端；PC 需匹配 */
        cam_send(buffer, length);
    }
    cam_send(&socket_head_and_tail[3], 2);
    LOG_DBG("cam_to_pc_command_send: type=0x%02x len=%u", type, (unsigned)length);
}

/* ========== 信息上报 ========= */
static int report_mega_info(void)
{
    static const char info[] =
        "ReportCameraInfo\r\n"
        "Camera Type:Virtual\r\n"
        "Camera Support Resolution:96x96,320x240,640x480,1280x720,2048x1536\r\n"
        "Camera Support special effects:0\r\n"
        "Camera Support Focus:0\r\n"
        "Camera Exposure Value Max:0\r\n"
        "Camera Exposure Value Min:0\r\n"
        "Camera Gain Value Max:0\r\n"
        "Camera Gain Value Min:0\r\n"
        "Camera Support Sharpness:0\r\n";
    uint32_t len = strlen(info);
    cam_to_pc_command_send(0x02, (uint8_t *)info, len);
    return 0;
}

/* ========== 命令接收 ========= */
static uint8_t recv_process(uint8_t *buff)
{
    LOG_INF("recv_process: cmd %x, data %x", buff[0], buff[1]);
    switch (buff[0]) {
    case SET_PICTURE_RESOLUTION:
        LOG_INF("camcmd: SET_PICTURE_RESOLUTION");
        break;
    case SET_VIDEO_RESOLUTION:
        LOG_INF("camcmd: SET_VIDEO_RESOLUTION");
        if (!client_state_socket.stream_active) {
            client_request_stream_start_stop(&client_state_socket, true);
        }
        break;
    case GET_CAMERA_INFO:
        report_mega_info();
        break;
    case TAKE_PICTURE:
        LOG_INF("Take picture");
        client_request_single_capture(&client_state_socket);
        break;
    case STOP_STREAM:
        if (client_state_socket.stream_active) {
            LOG_INF("Stop video stream");
            client_request_stream_start_stop(&client_state_socket, false);
        }
        break;
    default:
        break;
    }
    return buff[0];
}

/* ========== 客户端请求/状态 ========= */
static void client_request_single_capture(struct client_state *state)
{
    if (!state->stream_active) {
        state->req_stream = 1;
        state->req_stream_stop = 1; /* 单帧完成即停 */
    }
}
static void client_request_stream_start_stop(struct client_state *state, bool start)
{
    if (start) state->req_stream = 1;
    else       state->req_stream_stop = 1;
}
static int client_check_start_request(struct client_state *state)
{
    if (state->req_stream && !state->stream_active) {
        state->req_stream = 0;
        state->stream_active = 1;
        return 1;
    }
    return 0;
}
static int client_check_stop_request(struct client_state *state)
{
    if (state->req_stream_stop && state->stream_active) {
        state->req_stream_stop = 0;
        state->stream_active = 0;
        return 1;
    }
    return 0;
}

/* ========== BLE 回调 ========= */
static void register_app_command(const struct app_command_t *command)
{
    if (k_msgq_put(&msgq_app_commands, command, K_NO_WAIT) != 0) {
        LOG_ERR("Command buffer full!");
    }
}
static void app_bt_connected_callback(void)    { LOG_INF("Bluetooth connection established"); }
static void app_bt_ready_callback(void)        { LOG_INF("Bluetooth client ready"); }
static void app_bt_disconnected_callback(void)
{
    LOG_INF("Bluetooth disconnected");
    client_state_ble.req_stream = 0;
    client_state_ble.req_stream_stop = 0;
    client_state_ble.stream_active = 0;
}
static void app_bt_take_picture_callback(void)
{
    static struct app_command_t cmd = {.type = APPCMD_TAKE_PICTURE, .mode = client_type_BLE};
    register_app_command(&cmd);
}
static void app_bt_enable_stream_callback(bool enable)
{
    static struct app_command_t cmd = {.type = APPCMD_START_STOP_STREAM, .mode = client_type_BLE};
    cmd.data[0] = enable ? 1 : 0;
    register_app_command(&cmd);
}
static void app_bt_change_resolution_callback(uint8_t resolution)
{
    static struct app_command_t cmd = {.type = APPCMD_CAM_COMMAND, .cam_cmd = SET_PICTURE_RESOLUTION};
    cmd.data[0] = 0x10 | (resolution & 0xF);
    register_app_command(&cmd);
}
const struct app_bt_cb app_bt_callbacks = {
    .connected        = app_bt_connected_callback,
    .ready            = app_bt_ready_callback,
    .disconnected     = app_bt_disconnected_callback,
    .take_picture     = app_bt_take_picture_callback,
    .enable_stream    = app_bt_enable_stream_callback,
    .change_resolution= app_bt_change_resolution_callback,
};

/* ========== 速率统计 ========= */
static void on_timer_count_bytes_func(struct k_timer *timer)
{
    // if (counted_bytes_sent > 0) {
    //     int kbps   = (counted_bytes_sent * 8) / 1024;
    //     int chunks = atomic_set(&dbg_chunks_sent, 0);   /* 读出并清零 */
    //     int filled = atomic_set(&dbg_chunks_filled, 0); /* 读出并清零 */
    //     LOG_INF("Data transferred: %i kbps | chunks(sent=%d filled=%d) | pos=%u/%u "
    //             "| rd=%u wr=%u rdy[0]=%u rdy[1]=%u len[0]=%u len[1]=%u",
    //             kbps, chunks, filled,
    //             (unsigned)s_img_pos_sock, (unsigned)FIXED_IMAGE_SIZE,
    //             (unsigned)s_pp_sock.rd, (unsigned)s_pp_sock.wr,
    //             (unsigned)s_pp_sock.ready[0], (unsigned)s_pp_sock.ready[1],
    //             (unsigned)s_pp_sock.len[0], (unsigned)s_pp_sock.len[1]);
    //     counted_bytes_sent = 0;
    // }
}

/* ========== Socket RX ========= */
void socket_rx_callback(uint8_t *data, uint16_t len)
{
    static struct app_command_t app_cmd_socket = {.type = APPCMD_SOCKET_RX};
    LOG_DBG("SOCKET RX callback");
    uint16_t cpy = len > sizeof(app_cmd_socket.data) ? sizeof(app_cmd_socket.data) : len;
    memcpy(app_cmd_socket.data, data, cpy);
    register_app_command(&app_cmd_socket);
}

// void video_preview(void)
// {
//     static bool inited = false;
//     static bool s_sock_frame_open = false; 
//     static bool s_ble_frame_open = false; /* 本帧帧头是否已发 */

//     if (!inited) {
//         pp2_init(&s_pp_sock);
//         pp2_init(&s_pp_ble);
//         s_img_pos_sock   = 0;
//         s_img_pos_ble   = 0;
//         s_sock_frame_open = false;
//         s_ble_frame_open = false;
//         inited = true;
//     }


//             /* ===== Socket：是否需要开新帧 ===== */
//     if (client_check_start_request(&client_state_ble)) {
//         /* 准备开始一帧：先清 ping-pong & 位置，再等下面发帧头 */;
//         s_ble_frame_open = false;
//     }

    
//     if (client_check_start_request(&client_state_socket)) {

//         s_sock_frame_open = false;
//     }

// /* 若流处于 active 但本帧还没开，则发帧头+长度 */
// if (client_state_ble.stream_active && !s_ble_frame_open) {
//     pp2_init(&s_pp_ble);
//     s_img_pos_ble = 0;

//    app_bt_send_picture_header(FIXED_IMAGE_SIZE);
//     s_ble_frame_open = true;
//     // k_yield();
// }

// /* 若流处于 active 但本帧还没开，则发帧头+长度 */
// if (client_state_socket.stream_active && !s_sock_frame_open) {
//     /* <<< 新增：每次开帧都清状态，保证能从 0 开始写 >>> */
//     pp2_init(&s_pp_sock);
//     s_img_pos_sock = 0;

//     socket_head_and_tail[2] = 0x01;
//     cam_send(&socket_head_and_tail[0], 3);
//     uint32_t len_le = FIXED_IMAGE_SIZE;   /* PC 若按网络序，请改 htonl */
//     cam_send((uint8_t *)&len_le, 4);
//     s_sock_frame_open = true;
//     // k_yield();
// }


  

//     /* ===== 先生产：只有“本帧已开启”时才去填数据 ===== */
//     if (client_state_socket.stream_active && s_sock_frame_open &&
//         (s_img_pos_sock < FIXED_IMAGE_SIZE)) {

//         size_t remain = FIXED_IMAGE_SIZE - s_img_pos_sock;
//         size_t wrote  = pp2_try_fill(&s_pp_sock, &fixed_image_data[s_img_pos_sock], remain);
//         s_img_pos_sock += wrote;

//         /* 帧末：最后一个未满块也要 flush 成 ready */
//         if (s_img_pos_sock >= FIXED_IMAGE_SIZE) {
//             uint8_t cur = s_pp_sock.wr;
//             if (s_pp_sock.len[cur] > 0 && !s_pp_sock.ready[cur]) {
//                 s_pp_sock.ready[cur] = 1;
//             }
//         }
//         /* 如果 wrote 为 0，说明两块都 ready，等待下一轮消费者清空即可 */
//     }


//         /* ===== 先生产：只有“本帧已开启”时才去填数据 ===== */
//     if (client_state_ble.stream_active && s_ble_frame_open &&
//         (s_img_pos_ble < FIXED_IMAGE_SIZE)) {

//         size_t remain = FIXED_IMAGE_SIZE - s_img_pos_ble;
//         size_t wrote  = pp2_try_fill(&s_pp_ble, &fixed_image_data[s_img_pos_ble], remain);
//         s_img_pos_ble += wrote;

//         /* 帧末：最后一个未满块也要 flush 成 ready */
//         if (s_img_pos_ble >= FIXED_IMAGE_SIZE) {
//             uint8_t cur = s_pp_ble.wr;
//             if (s_pp_ble.len[cur] > 0 && !s_pp_ble.ready[cur]) {
//                 s_pp_ble.ready[cur] = 1;
//             }
//         }
//         /* 如果 wrote 为 0，说明两块都 ready，等待下一轮消费者清空即可 */
//     }
//       /* ===== 后消费：把 ready 块全部发掉 ===== */
//     if (client_state_socket.stream_active && s_sock_frame_open) {
//         pp2_try_send_socket(&s_pp_sock); 
        
//     }

//    /* 发送 ready 块 —— BLE */
// if (client_state_ble.stream_active && s_ble_frame_open) {
//     pp2_try_send_ble(&s_pp_ble);   // 不用 while 循环
// }

    
    
//     if (client_state_ble.stream_active &&
//     s_ble_frame_open &&
//     (s_img_pos_ble >= FIXED_IMAGE_SIZE) &&
//     pp2_all_empty(&s_pp_ble)) {


//     s_ble_frame_open = false;

//     // if (client_state_ble.req_stream_stop) {
//     //     client_check_stop_request(&client_state_ble);  // 单帧模式
//     // } else {
//     //     pp2_init(&s_pp_ble);                           // 连续推流：复位生产侧
//     //     s_img_pos_ble = 0;
//     //     // 下一轮会在 "!s_ble_frame_open" 分支里重新发帧头
//     // }
// }



//     /* ===== 帧尾：当本帧数据已写完 且 两块都空，发送帧尾 ===== */
//     if (client_state_socket.stream_active &&
//         s_sock_frame_open &&
//         (s_img_pos_sock >= FIXED_IMAGE_SIZE) &&
//         pp2_all_empty(&s_pp_sock)) {

//         cam_send(&socket_head_and_tail[3], 2); /* 0xFF,0xBB 帧尾 */
//         s_sock_frame_open = false;            /* 结束本帧 */

//         // if (client_state_socket.req_stream_stop) {
//         //     /* 单帧/STOP：真正停流 */
//         //     client_check_stop_request(&client_state_socket);
//         // } else {
//         //     /* 连续推流：下一帧由上面的逻辑自动开启（下一轮会发帧头） */
//         //     /* 此处不立即发帧头，给 PC/网络栈一点喘息，更稳 */
//         // }
//     }

//     // /* 兜底：处理外部停流 */
//     client_check_stop_request(&client_state_ble);
//     client_check_stop_request(&client_state_socket);
// }





void video_preview(void)
{
    /* 只用状态，不用 ping-pong */
    static bool inited = false;

    /* 是否已对某个 client 发出“开帧头”（仅用于连续 stream 时避免重复发头） */
    static bool sock_frame_open = false;
    static bool ble_frame_open  = false;

    if (!inited) {
        sock_frame_open = false;
        ble_frame_open  = false;
        inited = true;
    }

    /* ========= 处理“开始请求” ========= */
    if (client_check_start_request(&client_state_socket)) {
        sock_frame_open = false; /* 下一次循环会重新发帧头 */
    }
    if (client_check_start_request(&client_state_ble)) {
        ble_frame_open = false;
    }

    /* ========= Socket (WiFi/TCP) 发送一帧 =========
     * 发送格式：
     *   [FF AA][0x01][len(4, little endian)][payload...][FF BB]
     *
     * 说明：你之前 PC 端已经改成按 length 截取 payload，
     * 所以帧尾 FF BB 可要可不要。
     * 这里我保留帧尾，方便你兼容旧逻辑。
     */
    if (client_state_socket.stream_active) {

        if (!sock_frame_open) {
            /* 帧头 + type */
            socket_head_and_tail[2] = 0x01;                /* Capture/Video frame */
            cam_send(&socket_head_and_tail[0], 3);         /* FF AA 01 */

            /* len（小端） */
            uint32_t len_le = (uint32_t)FIXED_IMAGE_SIZE;
            cam_send((uint8_t *)&len_le, 4);

            sock_frame_open = true;
        }

        /* 关键：整帧一次性发（不要 1024 切块） */
        cam_send((uint8_t *)fixed_image_data, FIXED_IMAGE_SIZE);

        /* 帧尾（可选） */
        cam_send(&socket_head_and_tail[3], 2);             /* FF BB */

        sock_frame_open = false; /* 一帧结束 */

        // /* 单帧模式：如果 req_stream_stop 被置位，则真正停流 */
        // if (client_state_socket.req_stream_stop) {
        //     client_check_stop_request(&client_state_socket);
        // }

        /* 连续推流：如果你希望不断发同一张图当 video stream，
         * 那么不需要 stop_request，下一轮 while(1) 会再次进入这里发下一帧。
         * 如果你想降低占用，可以在 main loop 加一个 k_sleep(K_MSEC(x)).
         */
    }

    /* ========= BLE 发送一帧 =========
     * BLE 通常需要先发 header（长度），然后分片发送 payload。
     * 你这边 app_bt_send_picture_data() 内部会按 MTU 切片，所以我们直接整帧交给它。
     */
    if (client_state_ble.stream_active) {

        if (!ble_frame_open) {
            app_bt_send_picture_header(FIXED_IMAGE_SIZE);
            ble_frame_open = true;
        }

        /* 整帧交给 BLE 层，内部自己切片 */
        app_bt_send_picture_data(fixed_image_data, FIXED_IMAGE_SIZE);

        ble_frame_open = false; /* 一帧结束 */

        if (client_state_ble.req_stream_stop) {
            client_check_stop_request(&client_state_ble);
        }
    }

    /* ========= 外部 STOP 兜底 ========= */
    client_check_stop_request(&client_state_ble);
    client_check_stop_request(&client_state_socket);
}


// typedef enum {
//     HO_IDLE = 0,
//     HO_WIFI_TO_BLE,
//     HO_BLE_TO_WIFI,
// } handover_state_t;

// static handover_state_t g_handover = HO_IDLE;
// void video_preview(void)
// {
//     static bool inited = false;

//     if (!inited) {
//         g_handover = HO_IDLE;
//         inited = true;
//     }

//     /* =====================================================
//      * 1. 处理 start 意图（只决定 handover，不立刻切）
//      * ===================================================== */

//     /* BLE 请求 start */
//     if (client_state_ble.req_stream) {
//         client_state_ble.req_stream = 0;

//         if (client_state_socket.stream_active) {
//             /* Wi-Fi 正在推流，等 Wi-Fi 当前帧结束 */
//             g_handover = HO_WIFI_TO_BLE;
//             LOG_INF("Request BLE start → pending Wi-Fi → BLE");
//         } else {
//             /* Wi-Fi 没在推，直接启 BLE */
//             client_state_ble.stream_active = 1;
//             LOG_INF("BLE stream started");
//         }
//     }

//     /* Wi-Fi 请求 start */
//     if (client_state_socket.req_stream) {
//         client_state_socket.req_stream = 0;

//         if (client_state_ble.stream_active) {
//             /* BLE 正在推流，等 BLE 当前帧结束 */
//             g_handover = HO_BLE_TO_WIFI;
//             LOG_INF("Request Wi-Fi start → pending BLE → Wi-Fi");
//         } else {
//             /* BLE 没在推，直接启 Wi-Fi */
//             client_state_socket.stream_active = 1;
//             LOG_INF("Wi-Fi stream started");
//         }
//     }

//     /* =====================================================
//      * 2. Wi-Fi 连续推流（一轮 = 一帧）
//      * ===================================================== */
//     if (client_state_socket.stream_active) {

//         /* 帧头 */
//         socket_head_and_tail[2] = 0x01;
//         cam_send(socket_head_and_tail, 3);

//         uint32_t len_le = (uint32_t)FIXED_IMAGE_SIZE;
//         cam_send((uint8_t *)&len_le, 4);

//         /* payload */
//         cam_send((uint8_t *)fixed_image_data, FIXED_IMAGE_SIZE);

//         /* 帧尾 */
//         cam_send(&socket_head_and_tail[3], 2);

//         /* ===== 一帧 Wi-Fi 发送完成 ===== */

//         if (g_handover == HO_WIFI_TO_BLE) {
//             /* 只在帧结束点切换 */
//             client_state_socket.stream_active = 0;
//             client_state_ble.stream_active = 1;
//             g_handover = HO_IDLE;
//             LOG_INF("Handover done: Wi-Fi → BLE");
//         }
//         /* 否则：连续 Wi-Fi 推流，下一轮继续发 */
//     }

//     /* =====================================================
//      * 3. BLE 连续推流（一轮 = 一帧）
//      * ===================================================== */
//     if (client_state_ble.stream_active) {

//         /* header */
//         app_bt_send_picture_header(FIXED_IMAGE_SIZE);

//         /* payload（内部自动按 MTU 切片） */
//         app_bt_send_picture_data(fixed_image_data, FIXED_IMAGE_SIZE);

//         /* ===== 一帧 BLE 发送完成 ===== */

//         if (g_handover == HO_BLE_TO_WIFI) {
//             /* 只在帧结束点切换 */
//             client_state_ble.stream_active = 0;
//             client_state_socket.stream_active = 1;
//             g_handover = HO_IDLE;
//             LOG_INF("Handover done: BLE → Wi-Fi");
//         }
//         /* 否则：连续 BLE 推流 */
//     }

//     /* =====================================================
//      * 4. stop 请求（帧结束后自然生效）
//      * ===================================================== */

//     if (client_state_socket.req_stream_stop) {
//         client_state_socket.req_stream_stop = 0;
//         client_state_socket.stream_active = 0;
//         g_handover = HO_IDLE;
//         LOG_INF("Wi-Fi stream stopped");
//     }

//     if (client_state_ble.req_stream_stop) {
//         client_state_ble.req_stream_stop = 0;
//         client_state_ble.stream_active = 0;
//         g_handover = HO_IDLE;
//         LOG_INF("BLE stream stopped");
//     }
// }




/* ========== main ========= */
int main(void)
{
    int ret;

#if defined(CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT) || NRF_CLOCK_HAS_HFCLK192M
    /* hardcode to 128MHz */
    nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
#endif
    k_sleep(K_SECONDS(1));
    printk("Starting %s with CPU frequency: %d MHz\n", CONFIG_BOARD, SystemCoreClock / MHZ(1));

    ret = app_bt_init(&app_bt_callbacks);
    if (ret < 0) {
        LOG_ERR("Error initializing Bluetooth");
        return -1;
    }

    net_util_set_callback(socket_rx_callback);
    k_timer_start(&m_timer_count_bytes, K_MSEC(1000), K_MSEC(1000));

    while (1) {
        struct app_command_t new_command;
        if (k_msgq_get(&msgq_app_commands, &new_command, K_USEC(50)) == 0) {
            switch (new_command.type) {
            case APPCMD_TAKE_PICTURE:
                LOG_INF("TAKE PICTURE");
                /* BLE 单帧 */
                client_request_single_capture(&client_state_ble);
                break;
            case APPCMD_CAM_COMMAND:
                if (new_command.cam_cmd == SET_PICTURE_RESOLUTION) {
                    LOG_INF("Change resolution to 0x%x", new_command.data[0]);
                }
                break;
            case APPCMD_START_STOP_STREAM: {
                bool enable = new_command.data[0] > 0;
                if (enable) {
                    LOG_INF("Starting BLE stream!");
                    client_request_stream_start_stop(&client_state_ble, true);
                } else {
                    LOG_INF("Stopping BLE stream");
                    client_request_stream_start_stop(&client_state_ble, false);
                }
                break;
            }
            case APPCMD_SOCKET_RX: {
                uint8_t socket_cmd_buf[CAM_COMMAND_MAX_SIZE - 2];
                ret = process_socket_rx_buffer(new_command.data, socket_cmd_buf);
                if (ret > 0) {
                    LOG_INF("Valid command received. Length:%d", ret);
                    LOG_HEXDUMP_INF(socket_cmd_buf, ret, "Data: ");
                    recv_process(socket_cmd_buf);
                } else {
                    LOG_INF("Invalid SOCKET command received:%d", ret);
                }
                break;
            }
            default:
                break;
            }
        }

        video_preview();
        // k_sleep(K_USEC(100)); // 需要时可打开降低占用
    }
    return 0;
}
