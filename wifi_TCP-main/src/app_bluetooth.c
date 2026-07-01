#include "app_bluetooth.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include "its.h"
#include <zephyr/bluetooth/conn.h>
 #include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>     // sys_cpu_to_le16()
#include <zephyr/bluetooth/conn.h>    // bt_conn_get_handle()
#include "wifi_cli.h"
#define LOG_MODULE_NAME app_bluetooth
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

static const struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(6, 6, 0, 400);


struct client_state {
    uint8_t req_stream      : 1;
    uint8_t req_stream_stop : 1;
    uint8_t stream_active   : 1;
};


extern struct client_state client_state_socket;

static struct bt_conn *current_conn;
 static uint16_t default_conn_handle;

static app_bt_connected_cb app_callback_connected;
static app_bt_ready_cb app_callback_ready;
static app_bt_disconnected_cb app_callback_disconnected;
static app_bt_take_picture_cb app_callback_take_picture;
static app_bt_enable_stream_cb app_callback_enable_stream;
static app_bt_change_resolution_cb app_callback_change_resolution;
static struct its_ble_params_info_t ble_params_info = {.con_interval = 0, .mtu = 23, .tx_phy = 1, .rx_phy = 1};

// In order to maximize data throughput, scale the notifications after the TX data length
static int le_tx_data_length = 20;

#include <string.h>  // memcpy()

static float read_le_f32(const uint8_t *p)
{
    uint32_t u = sys_get_le32(p);  // <zephyr/sys/byteorder.h> 已经包含
    float f;
    memcpy(&f, &u, sizeof(f));     // 避免严格别名引发的问题
    return f;
}


 static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
 {
	 struct bt_hci_cp_vs_write_tx_power_level *cp;
	 struct bt_hci_rp_vs_write_tx_power_level *rp;
	 struct net_buf *buf, *rsp = NULL;
	 int err;
 
	 buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				 sizeof(*cp));
	 if (!buf) {
		 printk("Unable to allocate command buffer\n");
		 return;
	 }
 
	 cp = net_buf_add(buf, sizeof(*cp));
	 cp->handle = sys_cpu_to_le16(handle);
	 cp->handle_type = handle_type;
	 cp->tx_power_level = tx_pwr_lvl;
 
	 err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
					buf, &rsp);
	 if (err) {
		 printk("Set Tx power err: %d\n", err);
		 return;
	 }
 
	 rp = (void *)rsp->data;
	 //printk("Actual Tx Power: %d\n", rp->selected_tx_power);
 
	 net_buf_unref(rsp);
 }
 
 static void get_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl)
 {
	 struct bt_hci_cp_vs_read_tx_power_level *cp;
	 struct bt_hci_rp_vs_read_tx_power_level *rp;
	 struct net_buf *buf, *rsp = NULL;
	 int err;
 
	 *tx_pwr_lvl = 0xFF;
	 buf = bt_hci_cmd_create(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
				 sizeof(*cp));
	 if (!buf) {
		 printk("Unable to allocate command buffer\n");
		 return;
	 }
 
	 cp = net_buf_add(buf, sizeof(*cp));
	 cp->handle = sys_cpu_to_le16(handle);
	 cp->handle_type = handle_type;
 
	 err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
					buf, &rsp);
	 if (err) {
		 printk("Read Tx power err: %d\n", err);
		 return;
	 }
 
	 rp = (void *)rsp->data;
	 *tx_pwr_lvl = rp->tx_power_level;
 
	 net_buf_unref(rsp);
 }

enum app_bt_internal_commands {APP_BT_INT_ITS_RX_EVT, APP_BT_INT_SCHEDULE_CONNECTED_CB, APP_BT_INT_SCHEDULE_READY_CB, APP_BT_INT_SCHEDULE_DISCONNECTED_CB, APP_BT_INT_SCHEDULE_BLE_PARAMS_INFO_UPDATE};
static struct its_rx_cb_evt_t internal_command_evt;

void schedule_ble_params_info_update(void);

struct app_bt_command {
	enum app_bt_internal_commands command;
	struct its_rx_cb_evt_t its_rx_event;
};
K_MSGQ_DEFINE(msgq_its_rx_commands, sizeof(struct app_bt_command), 8, 4);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ITS_VAL),
};

static int schedule_internal_command(enum app_bt_internal_commands command)
{
	internal_command_evt.command = command;
	if (k_msgq_put(&msgq_its_rx_commands, &internal_command_evt, K_NO_WAIT) != 0){
		LOG_ERR("RX cmd message queue full!");
		return -ENOMEM;
	}
	return 0;
}

void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_INF("MTU Updated: tx %d, rx %d", tx, rx);
}

static struct bt_gatt_cb gatt_cb = {
	.att_mtu_updated = att_mtu_updated,
};

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		int att_mtu = bt_gatt_get_mtu(conn);
		LOG_INF("MTU exchange successful. %i bytes", att_mtu - 3); 
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
 	int ret = bt_hci_get_conn_handle(current_conn,
						  &default_conn_handle);
	static struct bt_gatt_exchange_params exchange_params;
	exchange_params.func = mtu_exchange_cb;

	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		printk("MTU exchange failed (err %d)\n", err);
	} else {
		printk("MTU exchange pending\n");
	}

	err = bt_conn_le_data_len_update(conn, BT_LE_DATA_LEN_PARAM_MAX);
	if (err) {
		LOG_ERR("LE data length update request failed: %d",  err);
	}

	err = bt_conn_le_param_update(conn, conn_param);
	if (err) {
		LOG_ERR("Cannot update connection parameter (err: %d)", err);
	}

	err = bt_conn_le_phy_update(conn, BT_CONN_LE_PHY_PARAM_2M);
    if (err) {
        LOG_ERR("PHY(2M) request failed: %d", err);
    }
	set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
                                       default_conn_handle, 20);
              

                int8_t txp_get = 0;
                get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
                                   default_conn_handle, &txp_get);
                
                LOG_INF("Inital TXP = %d\n", txp_get);
	schedule_internal_command(APP_BT_INT_SCHEDULE_CONNECTED_CB);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	schedule_internal_command(APP_BT_INT_SCHEDULE_DISCONNECTED_CB);
}

static void connection_param_update(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	LOG_INF("Con params updated. Connection interval %d ms, latency %i, timeout %i ms", (int) ((float)interval*1.25f), latency, timeout * 10);
	ble_params_info.con_interval = interval;
	schedule_ble_params_info_update();
}

static void le_data_length_updated(struct bt_conn *conn,
				   struct bt_conn_le_data_len_info *info)
{
	LOG_INF("LE data length updated: TX (len: %d time: %d) RX (len: %d time: %d)", 
			info->tx_max_len, info->tx_max_time, info->rx_max_len, info->rx_max_time);

	ble_params_info.mtu = info->tx_max_len;
	schedule_ble_params_info_update();

	// Set the TX data length, which will determine the size of image data transfers. 
	// Subtract 3 bytes for ATT header and 4 bytes for L2CAP header. 
	le_tx_data_length = info->tx_max_len - 7;
	LOG_INF("Notification data length set to %i bytes", le_tx_data_length);
}

static void le_phy_updated(struct bt_conn *conn,
			   struct bt_conn_le_phy_info *param)
{
	LOG_INF("LE PHY updated: TX PHY %i, RX PHY %i\n", param->tx_phy, param->rx_phy);
	ble_params_info.tx_phy = param->tx_phy;
	ble_params_info.rx_phy = param->rx_phy;
	schedule_ble_params_info_update();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	.le_param_updated = connection_param_update,
	.le_data_len_updated = le_data_length_updated,
	.le_phy_updated = le_phy_updated,
};

void schedule_ble_params_info_update(void)
{
	internal_command_evt.command = APP_BT_INT_SCHEDULE_BLE_PARAMS_INFO_UPDATE;
	if (k_msgq_put(&msgq_its_rx_commands, &internal_command_evt, K_NO_WAIT) != 0){
		LOG_ERR("RX cmd message queue full!");
	}
}

void its_ready_callback(void)
{
	static struct app_bt_command bt_cmd = {.command = APP_BT_INT_SCHEDULE_READY_CB};
	if (k_msgq_put(&msgq_its_rx_commands, &bt_cmd, K_NO_WAIT) != 0) {
		LOG_ERR("APP BT RX CMD queue full");
	}
}

// void its_rx_callback(struct its_rx_cb_evt_t *evt)
// {
// 	static struct app_bt_command bt_cmd = {.command = APP_BT_INT_ITS_RX_EVT};
// 	bt_cmd.its_rx_event = *evt;
// 	if (k_msgq_put(&msgq_its_rx_commands, &bt_cmd, K_NO_WAIT) != 0) {
// 		LOG_ERR("APP BT RX CMD queue full");
// 	}
// }
void its_rx_callback(struct its_rx_cb_evt_t *evt)
{
    /* 先把命令字打出来 */
    printk("ITS RX: cmd=0x%02X, len=%u, data:", (unsigned)evt->command, (unsigned)evt->len);
    for (size_t i = 0; i < evt->len; i++) {
        printk(" %02X", evt->data[i]);
    }
    printk("\n");

    /* 原有入队逻辑 */
    static struct app_bt_command bt_cmd = {.command = APP_BT_INT_ITS_RX_EVT};
    bt_cmd.its_rx_event = *evt;
    if (k_msgq_put(&msgq_its_rx_commands, &bt_cmd, K_NO_WAIT) != 0) {
        LOG_ERR("APP BT RX CMD queue full");
    }
}







static struct bt_its_cb its_cb = {
	.ready_cb = its_ready_callback,
	.rx_cb = its_rx_callback,
};

static void app_bt_thread_func(void)
{
	int err;
	struct app_bt_command app_cmd;
	while (1) {
		k_msgq_get(&msgq_its_rx_commands, &app_cmd, K_FOREVER);

		if(app_cmd.command == APP_BT_INT_ITS_RX_EVT) {
			// Commands originating from the Image Transfer Service
			switch (app_cmd.its_rx_event.command) {
				case ITS_RX_CMD_SINGLE_CAPTURE:
					LOG_DBG("ITS RX CMD: SingleCapture");
					if (app_callback_take_picture) {
						app_callback_take_picture();
					}
					break;
				case ITS_RX_CMD_START_STREAM:
				LOG_DBG("ITS RX CMD: Start Stream");

				/* 只表达 BLE 的意图，不直接干预 Wi-Fi */
				if (app_callback_enable_stream) {
					app_callback_enable_stream(true);
				}
				break;

				case ITS_RX_CMD_STOP_STREAM:
					LOG_DBG("ITS RX CMD: Stop stream");

					/* 只表达“BLE 想停”，不直接停 */
					if (app_callback_enable_stream) {
						app_callback_enable_stream(false);
					}

					/* TX power 是否调整，取决于你的策略 */
					set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
								default_conn_handle, 20);

					int8_t txp_get = 0;
					get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
								default_conn_handle, &txp_get);

					LOG_INF("Fixed TXP = %d\n", txp_get);
					break;

				case ITS_RX_CMD_CHANGE_RESOLUTION:
					LOG_DBG("ITS RX CMD: Change res");
					if (app_callback_change_resolution) {
						app_callback_change_resolution(app_cmd.its_rx_event.data[0]);
					}
					break;
				case ITS_RX_CMD_CHANGE_PHY:
					LOG_DBG("ITS RX CMD: Change phy (%i)", app_cmd.its_rx_event.data[0]);
					err = bt_conn_le_phy_update(current_conn, (app_cmd.its_rx_event.data[0] == 1) ? BT_CONN_LE_PHY_PARAM_2M : BT_CONN_LE_PHY_PARAM_1M);
					if (err) {
						LOG_ERR("Phy update request failed: %d",  err);
					}
					break;
				case ITS_RX_CMD_SEND_BLE_PARAMS:
					LOG_DBG("ITS RX CMD: Send ble params");
					break;

				case ITS_RX_CMD_TXP: {
                /* 期望 payload: int8 dBm（两补码） */
                if (app_cmd.its_rx_event.len < 1) {
                    LOG_WRN("ITS_RX_CMD_TXP: missing param");
                    break;
                }
                // if (!current_conn || default_conn_handle == 0xFFFF) {
                //     LOG_WRN("ITS_RX_CMD_TXP: no active connection handle");
                //     break;
                // }

                int8_t req_dbm = (int8_t)app_cmd.its_rx_event.data[0];


                /* 你指定的 printk 文案 */
                printk("Adaptive Tx power selected = %d\n", req_dbm);

                set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
                                       default_conn_handle, req_dbm);
              

                int8_t txp_get = 0;
                get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
                                   default_conn_handle, &txp_get);
                
                printk("Real TXP = %d\n", txp_get);
            
                break;
            }

            case ITS_RX_CMD_WIFIONOFF: {
					if (app_cmd.its_rx_event.len < 1) {
						LOG_WRN("ITS_RX_CMD_WIFIONOFF: missing param");
						break;
					}
					uint8_t v = app_cmd.its_rx_event.data[0];

					if (v == 0x01) {
						LOG_INF("Wi-Fi: power on via wifi_cli_do_up()");
						int r = wifi_cli_do_up();
						if (r) { LOG_ERR("wifi_cli_do_up() failed: %d", r); }
					} else if (v == 0x00) {
						LOG_INF("Wi-Fi: shutdown requested");
						client_state_socket.req_stream_stop = 1;//inactive wifi
						LOG_INF("Wi-Fi: shutdown via wifi_cli_do_shutdown()");
						
						int r = wifi_cli_do_shutdown();          // ★再关机
						if (r) { LOG_ERR("wifi_cli_do_shutdown() failed: %d", r); }
					} else {
						LOG_WRN("Wi-Fi: unknown param 0x%02X", v);
					}
					break;
				}

		case ITS_RX_CMD_WIFIACTIVE: {
			uint8_t x = app_cmd.its_rx_event.data[0];

			if (x == 0x01) {
				/* 表达：Wi-Fi 想开始 */
				client_state_socket.req_stream = 1;
				printk("WIFI ACTIVE (intent)\n");
			} else if (x == 0x00) {
				/* 表达：Wi-Fi 想停止 */
				client_state_socket.req_stream_stop = 1;
				printk("WIFI INACTIVE (intent)\n");
			}
			break;
		}


	
		case ITS_RX_CMD_THROUGHPUT_NOTIFY: {
			if (app_cmd.its_rx_event.len < 4) {
				LOG_WRN("ITS_RX_CMD_THROUGHPUT_NOTIFY: payload too short (%u)", app_cmd.its_rx_event.len);
				break;
			}
			float tp = read_le_f32(&app_cmd.its_rx_event.data[0]);
			/* 按两位小数输出，例如 1200.00 */
			// printk("THROUGHPUT = %.2f\n", tp);
			LOG_INF("Throughput notify: %.2f", (double)tp);
			break;
		}

		case ITS_RX_CMD_BORDER_NOTIFY: {
			if (app_cmd.its_rx_event.len < 8) {
				LOG_WRN("ITS_RX_CMD_BORDER_NOTIFY: payload too short (%u)", app_cmd.its_rx_event.len);
				break;
			}
			float x = read_le_f32(&app_cmd.its_rx_event.data[0]);
			float y = read_le_f32(&app_cmd.its_rx_event.data[4]);
			/* 按两位小数输出，例如 (300.00, 400.00) */
			// printk("BORDER = (%.2f, %.2f)\n", x, y);
			LOG_INF("Border notify: (%.2f, %.2f)", (double)x, (double)y);
			break;
		}

			
				default:
					LOG_ERR("CMD:Invalid command!");
					break;
			}
		} else {
			switch (app_cmd.command) {
				case APP_BT_INT_SCHEDULE_CONNECTED_CB:
					if (app_callback_connected) {
						app_callback_connected();
					}
					break;
				case APP_BT_INT_SCHEDULE_READY_CB:
					if (app_callback_ready) {
						app_callback_ready();
					}
					break;
				case APP_BT_INT_SCHEDULE_DISCONNECTED_CB:
					if (app_callback_disconnected) {
						app_callback_disconnected();
					}
					break;
				case APP_BT_INT_SCHEDULE_BLE_PARAMS_INFO_UPDATE:
					err = bt_its_send_ble_params_info(&ble_params_info);	
					if (err) {
						LOG_ERR("Error sending ble params");
					}
					break;
				default:
					break;
			}
		}
	}
}

int app_bt_init(const struct app_bt_cb *callbacks)
{
	int err = 0;

	if (callbacks) {
		app_callback_connected = callbacks->connected;
		app_callback_ready = callbacks->ready;
		app_callback_disconnected = callbacks->disconnected;
		app_callback_take_picture = callbacks->take_picture;
		app_callback_enable_stream = callbacks->enable_stream;
		app_callback_change_resolution = callbacks->change_resolution;
	}

	err = bt_enable(NULL);
	if (err) {
		return err;
	}

	LOG_DBG("Bluetooth initialized");

	bt_gatt_cb_register(&gatt_cb);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

    err = bt_its_init(&its_cb);
	if (err) {
		LOG_ERR("Failed to initialize Image Transfer Service (err: %d)", err);
		return 0;
	}
				/* 扫描功率：handle 固定用 0 */
		set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_SCAN, 0, 20);

		int8_t scan_txp = 0;
		get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_SCAN, 0, &scan_txp);
		LOG_INF("SCAN TXP = %d dBm", scan_txp);
			// 广播句柄（legacy/默认广告集为 0）
		static const uint16_t adv_handle = 0;

		set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, adv_handle, 20);

		int8_t adv_txp;
		get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, adv_handle, &adv_txp);
		LOG_INF("ADV TXP = %d dBm\n", adv_txp);


	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}
	err = wifi_cli_init();
    if (err) {
        LOG_ERR("wifi_cli_init() failed: %d", err);
        return err;
    }
    LOG_INF("wifi_cli_init OK");

    return 0;
}

int app_bt_send_picture_header(uint32_t pic_size)
{
	struct its_img_info_t img_info = {.file_size_bytes = pic_size};
	return bt_its_send_img_info(&img_info);
}

int app_bt_send_picture_data(const uint8_t *buf, uint16_t len)
{
	return bt_its_send_img_data(current_conn, buf, len, le_tx_data_length);
}

int app_bt_send_client_status(uint8_t cam_model, uint8_t resolution)
{
	struct its_client_status_t client_status = {.camera_type = cam_model, .selected_resolution_index = resolution};
	return bt_its_send_client_status(&client_status);
}

K_THREAD_DEFINE(app_bt_thread, 2048, app_bt_thread_func, NULL, NULL, NULL, 
				K_PRIO_PREEMPT(K_LOWEST_APPLICATION_THREAD_PRIO), 0, 0);