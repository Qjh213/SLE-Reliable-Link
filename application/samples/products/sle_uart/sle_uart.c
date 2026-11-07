/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: SLE UART Sample Source. \n
 *
 * History: \n
 * 2023-07-17, Create file. \n
 */
#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "pinctrl.h"
#include "uart.h"
#include "tcxo.h"
#include "sle_rl_protocol.h"
// #include "pm_clock.h"
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
#include "securec.h"
#include "sle_uart_server.h"
#include "sle_uart_server_adv.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
#define SLE_UART_TASK_STACK_SIZE            0x600
#include "sle_connection_manager.h"
#include "sle_ssap_client.h"
#include "sle_uart_client.h"
#endif  /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */

#define SLE_UART_TASK_PRIO                  28
#define SLE_UART_TASK_DURATION_MS           2000
#define SLE_UART_BAUDRATE                   115200
#define SLE_UART_TRANSFER_SIZE              512

static uint8_t g_app_uart_rx_buff[SLE_UART_TRANSFER_SIZE] = { 0 };

static uart_buffer_config_t g_app_uart_buffer_config = {
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = SLE_UART_TRANSFER_SIZE
};

static void uart_init_pin(void)
{
    if (CONFIG_SLE_UART_BUS == 0) {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);       
    }else if (CONFIG_SLE_UART_BUS == 1) {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);       
    }
}

static void uart_init_config(void)
{
    uart_attr_t attr = {
        .baud_rate = SLE_UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE
    };

    uart_pin_config_t pin_config = {
        .tx_pin = CONFIG_UART_TXD_PIN,
        .rx_pin = CONFIG_UART_RXD_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };
    uapi_uart_deinit(CONFIG_SLE_UART_BUS);
    uapi_uart_init(CONFIG_SLE_UART_BUS, &pin_config, &attr, NULL, &g_app_uart_buffer_config);

}

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
#define SLE_UART_SERVER_DELAY_COUNT         5

#define SLE_UART_TASK_STACK_SIZE            0x1200
#define SLE_ADV_HANDLE_DEFAULT              1
#define SLE_UART_SERVER_MSG_QUEUE_LEN       5
#define SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE  32
#define SLE_UART_SERVER_QUEUE_DELAY         0xFFFFFFFF
#define SLE_UART_SERVER_BUFF_MAX_SIZE       800
#define SLE_RL_SERVER_TASK_STACK_SIZE       0x1000
#define SLE_RL_SERVER_TASK_PRIO             29
#define SLE_RL_PERIOD_DEFAULT_MS            1000U
#define SLE_RL_PERIOD_MIN_MS                100U
#define SLE_RL_PERIOD_MAX_MS                10000U
#define SLE_RL_TELEMETRY_PAYLOAD_LEN        20U

unsigned long g_sle_uart_server_msgqueue_id;
#define SLE_UART_SERVER_LOG                 "[sle uart server]"
#define SLE_LATENCY_MAGIC_0                 0x4C
#define SLE_LATENCY_MAGIC_1                 0x54
#define SLE_LATENCY_PACKET_LEN              16U
#define SLE_LATENCY_WARMUP_COUNT            0U
#define SLE_LATENCY_SAMPLE_COUNT            100U
#define SLE_LATENCY_TOTAL_COUNT             (SLE_LATENCY_WARMUP_COUNT + SLE_LATENCY_SAMPLE_COUNT)

static uint32_t g_sle_latency_rx_packets = 0;
static uint32_t g_sle_latency_expected_seq = 0;
static uint32_t g_sle_latency_sequence_errors = 0;
static uint32_t g_sle_latency_content_errors = 0;
static uint32_t g_sle_latency_echo_errors = 0;
static volatile bool g_sle_rl_telemetry_enabled = false;
static volatile uint32_t g_sle_rl_period_ms = SLE_RL_PERIOD_DEFAULT_MS;
static volatile uint32_t g_sle_rl_telemetry_seq = 0;
static volatile uint32_t g_sle_rl_tx_ok = 0;
static volatile uint32_t g_sle_rl_tx_error = 0;
static volatile uint32_t g_sle_rl_command_rx = 0;
static volatile uint32_t g_sle_rl_parse_error = 0;
static volatile uint32_t g_sle_rl_ack_seq = 0;
static osal_semaphore g_sle_rl_tx_sem;
static volatile bool g_sle_rl_tx_sem_ready = false;

static errcode_t sle_rl_server_send_frame(uint8_t type, uint32_t seq,
    const uint8_t *payload, uint16_t payload_len)
{
    uint8_t frame[SLE_RL_MAX_FRAME_LEN] = {0};
    uint16_t frame_len = sle_rl_encode(frame, sizeof(frame), type, 0, seq, payload, payload_len);
    if (frame_len == 0) {
        return ERRCODE_FAIL;
    }
    if (!g_sle_rl_tx_sem_ready || (osal_sem_down(&g_sle_rl_tx_sem) != OSAL_SUCCESS)) {
        return ERRCODE_FAIL;
    }
    errcode_t ret = sle_uart_server_send_report_by_handle(frame, frame_len);
    osal_sem_up(&g_sle_rl_tx_sem);
    return ret;
}

static void sle_rl_server_send_ack(uint32_t command_seq, uint8_t command,
    uint8_t ack_status, const uint8_t *extra, uint16_t extra_len)
{
    uint8_t payload[SLE_RL_MAX_PAYLOAD_LEN] = {0};
    uint16_t payload_len = 6;
    payload[0] = command;
    payload[1] = ack_status;
    sle_rl_put_le32(&payload[2], command_seq);
    if ((extra != NULL) && (extra_len <= (SLE_RL_MAX_PAYLOAD_LEN - payload_len))) {
        (void)memcpy_s(&payload[payload_len], sizeof(payload) - payload_len, extra, extra_len);
        payload_len = (uint16_t)(payload_len + extra_len);
    }

    errcode_t ret = sle_rl_server_send_frame(SLE_RL_TYPE_COMMAND_ACK,
        g_sle_rl_ack_seq++, payload, payload_len);
    osal_printk("[RL][ACK_TX] command_seq=%u command=%u status=%u ret=0x%x\r\n",
        command_seq, command, ack_status, ret);
}

static void sle_rl_server_process_command(const sle_rl_frame_view_t *view)
{
    if ((view == NULL) || (view->payload_len < 1)) {
        return;
    }

    uint8_t command = view->payload[0];
    uint8_t ack_status = SLE_RL_ACK_OK;
    uint8_t extra[SLE_RL_MAX_PAYLOAD_LEN - 6] = {0};
    uint16_t extra_len = 0;
    g_sle_rl_command_rx++;

    switch (command) {
        case SLE_RL_CMD_START:
            if (view->payload_len != 1) {
                ack_status = SLE_RL_ACK_BAD_LENGTH;
            } else {
                g_sle_rl_telemetry_enabled = true;
            }
            break;
        case SLE_RL_CMD_STOP:
            if (view->payload_len != 1) {
                ack_status = SLE_RL_ACK_BAD_LENGTH;
            } else {
                g_sle_rl_telemetry_enabled = false;
            }
            break;
        case SLE_RL_CMD_SET_PERIOD:
            if (view->payload_len != 5) {
                ack_status = SLE_RL_ACK_BAD_LENGTH;
            } else {
                uint32_t period_ms = sle_rl_get_le32(&view->payload[1]);
                if ((period_ms < SLE_RL_PERIOD_MIN_MS) || (period_ms > SLE_RL_PERIOD_MAX_MS)) {
                    ack_status = SLE_RL_ACK_BAD_PARAMETER;
                } else {
                    g_sle_rl_period_ms = period_ms;
                    sle_rl_put_le32(extra, period_ms);
                    extra_len = 4;
                }
            }
            break;
        case SLE_RL_CMD_GET_STATUS:
            if (view->payload_len != 1) {
                ack_status = SLE_RL_ACK_BAD_LENGTH;
            } else {
                extra[0] = g_sle_rl_telemetry_enabled ? 1 : 0;
                sle_rl_put_le32(&extra[1], g_sle_rl_period_ms);
                sle_rl_put_le32(&extra[5], g_sle_rl_telemetry_seq);
                sle_rl_put_le32(&extra[9], g_sle_rl_tx_ok);
                sle_rl_put_le32(&extra[13], g_sle_rl_tx_error);
                extra_len = 17;
            }
            break;
        case SLE_RL_CMD_RESET_STATS:
            if (view->payload_len != 1) {
                ack_status = SLE_RL_ACK_BAD_LENGTH;
            } else {
                g_sle_rl_tx_ok = 0;
                g_sle_rl_tx_error = 0;
                g_sle_rl_command_rx = 0;
                g_sle_rl_parse_error = 0;
            }
            break;
        case SLE_RL_CMD_PING:
            if ((view->payload_len < 2) ||
                (view->payload_len > (SLE_RL_MAX_PAYLOAD_LEN - 5U))) {
                ack_status = SLE_RL_ACK_BAD_LENGTH;
            } else {
                extra_len = (uint16_t)(view->payload_len - 1);
                (void)memcpy_s(extra, sizeof(extra), &view->payload[1], extra_len);
            }
            break;
        default:
            ack_status = SLE_RL_ACK_UNKNOWN_COMMAND;
            break;
    }

    osal_printk("[RL][COMMAND] seq=%u command=%u len=%u status=%u\r\n",
        view->seq, command, view->payload_len, ack_status);
    sle_rl_server_send_ack(view->seq, command, ack_status, extra, extra_len);
}

static bool sle_rl_server_handle_frame(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len < 2) ||
        (data[0] != SLE_RL_MAGIC_0) || (data[1] != SLE_RL_MAGIC_1)) {
        return false;
    }

    sle_rl_frame_view_t view = {0};
    sle_rl_parse_result_t parse_ret = sle_rl_decode(data, len, &view);
    if (parse_ret != SLE_RL_PARSE_OK) {
        g_sle_rl_parse_error++;
        osal_printk("[RL][ERROR] direction=CLIENT_TO_SERVER reason=%s len=%u total=%u\r\n",
            sle_rl_parse_result_name(parse_ret), len, g_sle_rl_parse_error);
        return true;
    }
    if (view.type != SLE_RL_TYPE_COMMAND) {
        g_sle_rl_parse_error++;
        osal_printk("[RL][ERROR] direction=CLIENT_TO_SERVER reason=UNEXPECTED_TYPE type=%u\r\n",
            view.type);
        return true;
    }
    sle_rl_server_process_command(&view);
    return true;
}

static void *sle_rl_server_telemetry_task(const char *arg)
{
    unused(arg);
    osal_printk("[RL][PROTOCOL_SELF_TEST] role=server result=%s\r\n",
        sle_rl_protocol_self_test() ? "PASS" : "FAIL");
    osal_printk("[RL][SERVER_INIT] period_ms=%u payload_len=%u\r\n",
        g_sle_rl_period_ms, SLE_RL_TELEMETRY_PAYLOAD_LEN);
    while (1) {
        if (!g_sle_rl_telemetry_enabled || !sle_uart_client_is_connected()) {
            osal_msleep(100);
            continue;
        }

        uint8_t payload[SLE_RL_TELEMETRY_PAYLOAD_LEN] = {0};
        uint32_t seq = g_sle_rl_telemetry_seq++;
        uint32_t uptime_ms = (uint32_t)(uapi_tcxo_get_us() / 1000U);
        sle_rl_put_le32(&payload[0], uptime_ms);
        sle_rl_put_le32(&payload[4], seq);
        sle_rl_put_le32(&payload[8], g_sle_rl_period_ms);
        sle_rl_put_le32(&payload[12], g_sle_rl_tx_ok);
        sle_rl_put_le32(&payload[16], g_sle_rl_tx_error);

        errcode_t ret = sle_rl_server_send_frame(SLE_RL_TYPE_TELEMETRY, seq,
            payload, sizeof(payload));
        if (ret == ERRCODE_SUCC) {
            g_sle_rl_tx_ok++;
        } else {
            g_sle_rl_tx_error++;
        }
        if (((seq + 1U) % 100U) == 0) {
            osal_printk("[RL][SERVER_SUMMARY] telemetry_seq=%u tx_ok=%u tx_error=%u "
                "command_rx=%u parse_error=%u\r\n", seq, g_sle_rl_tx_ok,
                g_sle_rl_tx_error, g_sle_rl_command_rx, g_sle_rl_parse_error);
        }
        osal_msleep(g_sle_rl_period_ms);
    }
    return NULL;
}

static uint32_t sle_latency_get_u32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

static void sle_latency_reset_rx(void)
{
    g_sle_latency_rx_packets = 0;
    g_sle_latency_expected_seq = 0;
    g_sle_latency_sequence_errors = 0;
    g_sle_latency_content_errors = 0;
    g_sle_latency_echo_errors = 0;
}

static void ssaps_server_read_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb_para,
    errcode_t status)
{
    osal_printk("%s ssaps read request cbk callback server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, read_cb_para->handle, status);
}
static void ssaps_server_write_request_cbk(uint8_t server_id, uint16_t conn_id,
    ssaps_req_write_cb_t *write_cb_para, errcode_t status)
{
    unused(server_id);
    unused(conn_id);

    if ((status != ERRCODE_SUCC) || (write_cb_para == NULL) ||
        (write_cb_para->value == NULL) || (write_cb_para->length == 0)) {
        osal_printk("%s invalid write status=0x%x\r\n", SLE_UART_SERVER_LOG, status);
        return;
    }

    const uint8_t *data = write_cb_para->value;
    uint16_t len = write_cb_para->length;

    if (sle_rl_server_handle_frame(data, len)) {
        return;
    }

    if ((len == SLE_LATENCY_PACKET_LEN) && (data[0] == SLE_LATENCY_MAGIC_0) &&
        (data[1] == SLE_LATENCY_MAGIC_1)) {
        uint32_t seq = sle_latency_get_u32_le(&data[2]);
        if (seq == 0) {
            sle_latency_reset_rx();
        }

        g_sle_latency_rx_packets++;
        if (seq != g_sle_latency_expected_seq) {
            g_sle_latency_sequence_errors++;
        }
        if ((data[14] != 0x07) || (data[15] != 0x01)) {
            g_sle_latency_content_errors++;
        }

        g_sle_latency_expected_seq = seq + 1;
        errcode_t echo_ret = sle_uart_server_send_report_by_handle(data, len);
        if (echo_ret != ERRCODE_SUCC) {
            g_sle_latency_echo_errors++;
        }

        if (seq == (SLE_LATENCY_TOTAL_COUNT - 1)) {
            osal_printk("[RECOVERY][SERVER_DATA_SUMMARY] transport=ssap_write_notification "
                "warmup=%u samples=%u total=%u sequence_error=%u "
                "content_error=%u echo_error=%u result=%s\r\n",
                SLE_LATENCY_WARMUP_COUNT, SLE_LATENCY_SAMPLE_COUNT,
                g_sle_latency_rx_packets, g_sle_latency_sequence_errors,
                g_sle_latency_content_errors, g_sle_latency_echo_errors,
                (g_sle_latency_rx_packets == SLE_LATENCY_TOTAL_COUNT &&
                g_sle_latency_sequence_errors == 0 && g_sle_latency_content_errors == 0 &&
                g_sle_latency_echo_errors == 0) ? "PASS" : "FAIL");
        }
        return;
    }

    /* Keep regular UART forwarding length-safe. */
    osal_printk("\r\n sle uart received data, len:%u, data:", len);
    uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)data, len, 0);
    osal_printk("\r\n");
}

static void sle_uart_server_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    if (sle_uart_client_is_connected()) {
    sle_uart_server_send_report_by_handle(buffer, length);
    } else {
        osal_printk("%s sle client is not connected! \r\n", SLE_UART_SERVER_LOG);
    }
}


static void sle_uart_server_create_msgqueue(void)
{
    if (osal_msg_queue_create("sle_uart_server_msgqueue", SLE_UART_SERVER_MSG_QUEUE_LEN, \
        (unsigned long *)&g_sle_uart_server_msgqueue_id, 0, SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE) != OSAL_SUCCESS) {
        osal_printk("^%s sle_uart_server_create_msgqueue message queue create failed!\n", SLE_UART_SERVER_LOG);
    }
}

static void sle_uart_server_delete_msgqueue(void)
{
    osal_msg_queue_delete(g_sle_uart_server_msgqueue_id);
}

static void sle_uart_server_write_msgqueue(uint8_t *buffer_addr, uint16_t buffer_size)
{
    osal_msg_queue_write_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr, \
                              (uint32_t)buffer_size, 0);
}

static int32_t sle_uart_server_receive_msgqueue(uint8_t *buffer_addr, uint32_t *buffer_size)
{
    return osal_msg_queue_read_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr, \
                                    buffer_size, SLE_UART_SERVER_QUEUE_DELAY);
}
static void sle_uart_server_rx_buf_init(uint8_t *buffer_addr, uint32_t *buffer_size)
{
    *buffer_size = SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE;
    (void)memset_s(buffer_addr, *buffer_size, 0, *buffer_size);
}

static void *sle_uart_server_task(const char *arg)
{
    unused(arg);
    uint8_t rx_buf[SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE] = {0};
    uint32_t rx_length = SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE;
    uint8_t sle_connect_state[] = "sle_dis_connect";

    sle_uart_server_create_msgqueue();
    sle_uart_server_register_msg(sle_uart_server_write_msgqueue);
    sle_uart_server_init(ssaps_server_read_request_cbk, ssaps_server_write_request_cbk);


    /* UART pinmux. */
    uart_init_pin();

    /* UART init config. */
    uart_init_config();

    uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
                                                   UART_RX_CONDITION_FULL_OR_IDLE,
                                                   1, sle_uart_server_read_int_handler);
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s Register uart callback fail.[%x]\r\n", SLE_UART_SERVER_LOG, ret);
        return NULL;
    }
    while (1) {
        sle_uart_server_rx_buf_init(rx_buf, &rx_length);
        sle_uart_server_receive_msgqueue(rx_buf, &rx_length);
        if (strncmp((const char *)rx_buf, (const char *)sle_connect_state, sizeof(sle_connect_state)) == 0) {
            ret = sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
            if (ret != ERRCODE_SLE_SUCCESS) {
                osal_printk("%s sle_connect_state_changed_cbk,sle_start_announce fail :%02x\r\n",
                    SLE_UART_SERVER_LOG, ret);
            }
        }
        osal_msleep(SLE_UART_TASK_DURATION_MS);
    }
    sle_uart_server_delete_msgqueue();
    return NULL;
}
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)

void sle_uart_notification_cb(uint8_t client_id, uint16_t conn_id,
    ssapc_handle_value_t *data, errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);

    if ((data != NULL) && (data->data != NULL) &&
        sle_rl_client_handle_notification(data->data, data->data_len)) {
        return;
    }

    if ((data != NULL) && (data->data != NULL) && (data->data_len > 0)) {
        osal_printk("\n sle uart received data, len:%u, data:", data->data_len);
        uapi_uart_write(CONFIG_SLE_UART_BUS,
            (uint8_t *)data->data, data->data_len, 0);
        osal_printk("\r\n");
    }
}

void sle_uart_indication_cb(uint8_t client_id, uint16_t conn_id,
    ssapc_handle_value_t *data, errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);

    if ((data != NULL) && (data->data != NULL) && (data->data_len > 0)) {
        osal_printk("\n sle uart received data, len:%u, data:", data->data_len);
        uapi_uart_write(CONFIG_SLE_UART_BUS,
            (uint8_t *)data->data, data->data_len, 0);
        osal_printk("\r\n");
    }
}

static void sle_uart_client_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    ssapc_write_param_t *sle_uart_send_param = get_g_sle_uart_send_param();
    uint16_t g_sle_uart_conn_id = get_g_sle_uart_conn_id();
    sle_uart_send_param->data_len = length;
    sle_uart_send_param->data = (uint8_t *)buffer;
    ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param);
}

static void *sle_uart_client_task(const char *arg)
{
    unused(arg);
    /* UART pinmux. */
    uart_init_pin();

    /* UART init config. */
    uart_init_config();

    uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
                                                   UART_RX_CONDITION_FULL_OR_IDLE,
                                                   1, sle_uart_client_read_int_handler);
    sle_uart_client_init(sle_uart_notification_cb, sle_uart_indication_cb);
    
    if (ret != ERRCODE_SUCC) {
        osal_printk("Register uart callback fail.");
        return NULL;
    }

    return NULL;
}
#endif  /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */

static void sle_uart_entry(void)
{
    osal_task *task_handle = NULL;
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
    osal_task *rl_task_handle = NULL;
    if (osal_sem_init(&g_sle_rl_tx_sem, 1) == OSAL_SUCCESS) {
        g_sle_rl_tx_sem_ready = true;
    } else {
        osal_printk("[RL][ERROR] role=server reason=TX_SEMAPHORE_INIT\r\n");
    }
#endif
    osal_kthread_lock();
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_server_task, 0, "SLEUartServerTask",
                                      SLE_UART_TASK_STACK_SIZE);
    rl_task_handle = osal_kthread_create((osal_kthread_handler)sle_rl_server_telemetry_task, 0,
                                         "SLEReliableServer", SLE_RL_SERVER_TASK_STACK_SIZE);
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_client_task, 0, "SLEUartDongleTask",
                                      SLE_UART_TASK_STACK_SIZE);
#endif /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, SLE_UART_TASK_PRIO);
    }
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
    if (rl_task_handle != NULL) {
        osal_kthread_set_priority(rl_task_handle, SLE_RL_SERVER_TASK_PRIO);
    }
#endif
    osal_kthread_unlock();
}

/* Run the sle_uart_entry. */
app_run(sle_uart_entry);
