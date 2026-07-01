#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_ctrl, CONFIG_LOG_DEFAULT_LEVEL);

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/net/dhcpv4_server.h>

/* Shell 相关 */
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

/* ==== Zephyr 兼容：STA 枚举其实叫 WIFI_MODE_INFRA ==== */
#ifndef WIFI_MODE_STA
#define WIFI_MODE_STA WIFI_MODE_INFRA
#endif

/* ==== 官方入口：无条件声明，避免切模式时隐式声明告警 ==== */
extern int wifi_softap_mode_ready(void);
extern int wifi_station_mode_ready(void);

/* ---------------- 模式选择：AP / STA ----------------
 * 默认改为 STA（你可切回 AP：把 WIFI_CLI_FIXED_MODE_STA 设为 0）
 */
#ifndef WIFI_CLI_FIXED_MODE_STA
#define WIFI_CLI_FIXED_MODE_STA  0    /* 0=AP, 1=STA */
#endif

#define WIFI_CLI_MODE_AP   0
#define WIFI_CLI_MODE_STA  1

#if WIFI_CLI_FIXED_MODE_STA
  #define WIFI_CLI_FIXED_MODE  WIFI_CLI_MODE_STA
#else
  #define WIFI_CLI_FIXED_MODE  WIFI_CLI_MODE_AP
#endif

/* 便捷：拿到第一个 Wi-Fi 接口 */
static inline struct net_if *wifi_iface(void)
{
    return net_if_get_first_wifi();
}

/* ---------------- 控制线程与命令 ---------------- */
enum wifi_cmd_type {
    WIFI_CMD_UP = 1,
    WIFI_CMD_SHUTDOWN = 2,
};
struct wifi_cmd {
    enum wifi_cmd_type type;
};
K_MSGQ_DEFINE(wifi_cmd_q, sizeof(struct wifi_cmd), 8, 4);   /* 深度 8，4 字节对齐 */

/* 冷却窗口：刚关机后延迟再起，避免残留事件与资源撞车 */
static int64_t last_shutdown_ms;
#define WIFI_RESTART_COOLDOWN_MS 300

/* 独立控制线程（避免阻塞系统 workqueue） */
#define WIFI_CTRL_STACK_SIZE  (8 * 1024)   /* STA/SoftAP 里 wpa_supp 回调较深，6KB 更稳 */
#define WIFI_CTRL_PRIO        10
static K_THREAD_STACK_DEFINE(wifi_ctrl_stack, WIFI_CTRL_STACK_SIZE);
static struct k_thread wifi_ctrl_thread;

/* 运行状态机（通用） */
enum wifi_run_state {
    WIFI_STATE_OFF = 0,
    WIFI_STATE_STARTING,
    WIFI_STATE_RUNNING,
};
static atomic_t g_state = ATOMIC_INIT(WIFI_STATE_OFF);

static inline enum wifi_run_state wifi_get_state(void)
{
    return (enum wifi_run_state)atomic_get(&g_state);
}
static inline void wifi_set_state(enum wifi_run_state s)
{
    atomic_set(&g_state, s);
}

/* ---------------- Shell 工具：执行一条命令 ---------------- */
static int run_shell_cmd(const char *fmt, ...)
{
    const struct shell *sh = shell_backend_uart_get_ptr();
    if (!sh) {
        /* 没有后端也返回成功，避免阻断主流程 */
        LOG_WRN("No shell backend UART.");
        return 0;
    }

    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    /* 注意：不要用 LOG_INF("%s", buf)（需要 log_strdup），直接 printk 即可 */
    printk("SHELL$ %s\n", buf);
    return shell_execute_cmd(sh, buf);
}

/* ---------------- STA “上电并自动连接” 子线程 ----------------
 * 目的：先进入 wifi_station_mode_ready()（注册回调并等待拿到 IPv4），
 * 再在外层线程里异步发送 wifi_cred add/auto_connect，避免事件丢失。
 */
#define STA_FLOW_STACK_SIZE  (3 * 1024)
#define STA_FLOW_PRIO        (WIFI_CTRL_PRIO + 1)
static K_THREAD_STACK_DEFINE(sta_flow_stack, STA_FLOW_STACK_SIZE);
static struct k_thread sta_flow_thread;

/* 提前声明，供子线程内出错时调用 */
static void wifi_do_shutdown(void);

/* 子线程入口：阻塞到拿到 IPv4（在 wifi_station_mode_ready 里完成） */
static void sta_flow_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    int r = wifi_station_mode_ready();
    if (r) {
        LOG_ERR("Station flow failed (%d)", r);
        wifi_do_shutdown();
        return;
    }

    /* 能走到这里说明已成功拿到 IPv4（DHCP 或静态） */
    wifi_set_state(WIFI_STATE_RUNNING);
    LOG_INF("STA is RUNNING.");
}

/* ---- 关机路径（幂等 + 深清理） ---- */
static void wifi_do_shutdown(void)
{
    struct net_if *iface = wifi_iface();
    if (!iface) {
        LOG_ERR("No Wi-Fi interface found.");
        return;
    }

    /* 1) AP 禁用（若当前是 AP；不是则返回 -ENOTSUP / -EINVAL，忽略即可） */
    (void)net_mgmt(NET_REQUEST_WIFI_AP_DISABLE, iface, NULL, 0);

    /* 2) 停 DHCPv4 Server（官方样例 up 时先启 DHCP，关机务必停） */
    net_dhcpv4_server_stop(iface);   /* 幂等，已停则无事 */

    /* 3) STA 断开（仅当 iface_mode 为 STA/INFRA 时才断开） */
    struct wifi_iface_status st = {0};
    if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &st, sizeof(st)) == 0) {
        if (st.iface_mode == WIFI_MODE_STA) {
            (void)net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
        }
    }

    /* 4) 给事件线程一个收尾窗口 */
    k_sleep(K_MSEC(150));

    /* 5) 最后再 down 接口（幂等） */
    if (net_if_is_admin_up(iface)) {
        int ret = net_if_down(iface);
        if (ret) {
            LOG_ERR("net_if_down() failed (%d)", ret);
        } else {
            LOG_INF("Wi-Fi down.");
        }
    } else {
        LOG_INF("Wi-Fi already down.");
    }

    wifi_set_state(WIFI_STATE_OFF);
    last_shutdown_ms = k_uptime_get();
}

/* ---- AP 上电路径（阻塞在专用控制线程中） ---- */
static int __unused wifi_do_up_ap(void)
{
    struct net_if *iface = wifi_iface();
    if (!iface) {
        LOG_ERR("No Wi-Fi interface found.");
        return -ENODEV;
    }

    enum wifi_run_state st = wifi_get_state();
    if (st == WIFI_STATE_STARTING || st == WIFI_STATE_RUNNING) {
        LOG_WRN("AP is already %s",
                st == WIFI_STATE_RUNNING ? "RUNNING" : "STARTING");
        return 0;
    }

    if (!net_if_is_admin_up(iface)) {
        int ret = net_if_up(iface);
        if (ret) {
            LOG_ERR("net_if_up() failed (%d)", ret);
            return ret;
        }
        LOG_INF("Interface up.");
    }

    wifi_set_state(WIFI_STATE_STARTING);
    LOG_INF("Fixed mode = AP, run SoftAP flow...");

    int r = wifi_softap_mode_ready();
    if (r) {
        LOG_ERR("SoftAP flow failed (%d)", r);
        wifi_do_shutdown();
        return r;
    }

    wifi_set_state(WIFI_STATE_RUNNING);
    return 0;
}

/* ---- STA 上电路径：启动 wait 线程 + 由本文件注入 wifi_cred 命令 ---- */
static int __unused wifi_do_up_sta(void)
{
    struct net_if *iface = wifi_iface();
    if (!iface) {
        LOG_ERR("No Wi-Fi interface found.");
        return -ENODEV;
    }

    enum wifi_run_state st = wifi_get_state();
    if (st == WIFI_STATE_STARTING || st == WIFI_STATE_RUNNING) {
        LOG_WRN("STA is already %s",
                st == WIFI_STATE_RUNNING ? "RUNNING" : "STARTING");
        return 0;
    }

    if (!net_if_is_admin_up(iface)) {
        int ret = net_if_up(iface);
        if (ret) {
            LOG_ERR("net_if_up() failed (%d)", ret);
            return ret;
        }
        LOG_INF("Interface up.");
    }

    /* 清理可能的残留连接（幂等） */
    (void)net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);

    wifi_set_state(WIFI_STATE_STARTING);
    LOG_INF("Fixed mode = STA, run Station flow...");

    /* 1) 先启动“等待拿到 IPv4”的子线程（里面调用 wifi_station_mode_ready()） */
    k_thread_create(&sta_flow_thread, sta_flow_stack, STA_FLOW_STACK_SIZE,
                    sta_flow_thread_fn, NULL, NULL, NULL,
                    STA_FLOW_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&sta_flow_thread, "sta_flow");

    /* 2) 稍等 100ms，确保回调已注册，再注入 shell 命令去连网 */
    k_sleep(K_MSEC(100));

#if defined(CONFIG_WIFI_CREDENTIALS_STATIC)
    /* 可选：先把旧条目清掉，避免重复/冲突 */
    run_shell_cmd("wifi_cred delete all");

    /* 加入一条静态凭据并启动自动连接 */
    run_shell_cmd("wifi_cred add \"%s\" WPA2-PSK %s",
                  CONFIG_WIFI_CREDENTIALS_STATIC_SSID,
                  CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD);
#else
    /* 若不用静态 Kconfig，可改成你自己的默认 SSID/PASS */
    /* run_shell_cmd("wifi_cred add \"<SSID>\" WPA2-PSK <PASS>"); */
#endif
    run_shell_cmd("wifi_cred auto_connect");

    /* 主线程不阻塞，RUNNING 的置位交由子线程在拿到 IPv4 后完成 */
    return 0;
}

/* ---- 统一的 up 调度 ---- */
static void wifi_do_up(void)
{
#if (WIFI_CLI_FIXED_MODE == WIFI_CLI_MODE_AP)
    (void)wifi_do_up_ap();
#else
    (void)wifi_do_up_sta();
#endif
}

/* ---- 控制线程主体（合并命令 + 冷却窗口 + AP 预清理） ---- */
static void wifi_ctrl_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    struct wifi_cmd cmd, last;

    while (1) {
        k_msgq_get(&wifi_cmd_q, &cmd, K_FOREVER);

        /* 吞并后只执行最后一条 */
        last = cmd;
        while (k_msgq_get(&wifi_cmd_q, &cmd, K_NO_WAIT) == 0) {
            last = cmd;
        }

        switch (last.type) {
        case WIFI_CMD_UP: {
            int64_t now = k_uptime_get();
            if ((now - last_shutdown_ms) < WIFI_RESTART_COOLDOWN_MS) {
                k_sleep(K_MSEC(WIFI_RESTART_COOLDOWN_MS - (now - last_shutdown_ms)));
            }

#if (WIFI_CLI_FIXED_MODE == WIFI_CLI_MODE_AP)
            struct net_if *iface = wifi_iface();
            if (iface) {
                (void)net_mgmt(NET_REQUEST_WIFI_AP_DISABLE, iface, NULL, 0);
                net_dhcpv4_server_stop(iface);
                (void)net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
                k_sleep(K_MSEC(50));
            }
#endif
            wifi_do_up();
            break;
        }
        case WIFI_CMD_SHUTDOWN:
            wifi_do_shutdown();
            break;
        default:
            break;
        }
    }
}

/* ---------------- 对外 API（给 BLE 调用） ---------------- */
int wifi_cli_init(void)
{
    k_thread_create(&wifi_ctrl_thread, wifi_ctrl_stack, WIFI_CTRL_STACK_SIZE,
                    wifi_ctrl_thread_fn, NULL, NULL, NULL,
                    WIFI_CTRL_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&wifi_ctrl_thread, "wifi_ctrl");
    return 0;
}

int wifi_cli_do_up(void)
{
    const struct wifi_cmd cmd = { .type = WIFI_CMD_UP };
    int ret = k_msgq_put(&wifi_cmd_q, &cmd, K_NO_WAIT);
    if (ret) {
        struct wifi_cmd dump;
        (void)k_msgq_get(&wifi_cmd_q, &dump, K_NO_WAIT);
        ret = k_msgq_put(&wifi_cmd_q, &cmd, K_NO_WAIT);
    }
    return ret;
}

int wifi_cli_do_shutdown(void)
{
    const struct wifi_cmd cmd = { .type = WIFI_CMD_SHUTDOWN };
    int ret = k_msgq_put(&wifi_cmd_q, &cmd, K_NO_WAIT);
    if (ret) {
        struct wifi_cmd dump;
        (void)k_msgq_get(&wifi_cmd_q, &dump, K_NO_WAIT);
        ret = k_msgq_put(&wifi_cmd_q, &cmd, K_NO_WAIT);
    }
    return ret;
}
