/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: SLE uart sample of client. \n
 *
 * History: \n
 * 2023-04-03, Create file. \n
 */
#include "common_def.h"
#include "soc_osal.h"
#include "securec.h"
#include "product.h"
#include "bts_le_gap.h"
#include "tcxo.h"

#include "sle_device_discovery.h"
#include "sle_connection_manager.h"
#include "sle_uart_client.h"
#include "sle_rl_protocol.h"
#define SLE_MTU_SIZE_DEFAULT            256
#define SLE_SEEK_INTERVAL_DEFAULT       100
#define SLE_SEEK_WINDOW_DEFAULT         100
#define UUID_16BIT_LEN                  2
#define UUID_128BIT_LEN                 16
#define SLE_UART_TASK_DELAY_MS          1000
#define SLE_UART_WAIT_SLE_CORE_READY_MS 5000
#define SLE_UART_RECV_CNT               1000
#define SLE_UART_LOW_LATENCY_2K         2000
#ifndef SLE_UART_SERVER_NAME
#define SLE_UART_SERVER_NAME            "sle_uart_server"
#endif
#define SLE_UART_CLIENT_LOG             "[sle uart client]"
#define SLE_LATENCY_TASK_STACK_SIZE    0x1000
#define SLE_LATENCY_TASK_PRIO          28
#define SLE_CONN_PARAM_INTERVAL        30U
#define SLE_CONN_PARAM_MAX_LATENCY     0U
#define SLE_CONN_PARAM_SUPERVISION     200U
#define SLE_CONN_PARAM_TIMEOUT_MS      3000U
#define SLE_RECOVERY_CYCLE_COUNT       5U
/* 0: remove the paired record before reconnecting; 1: retain it for the control group. */
#define SLE_RECOVERY_RETAIN_PAIR       1
/* In retain mode, remove a stale local record only after a pairing failure. */
#define SLE_RECOVERY_STALE_PAIR_FALLBACK 1
#define SLE_RL_COMMAND_TIMEOUT_MS       5000U
#define SLE_RL_TELEMETRY_TIMEOUT_MS     15000U
#define SLE_RL_FUNCTIONAL_COUNT         10U
#define SLE_RL_RELIABILITY_COUNT        1000U
#define SLE_RL_TEST_PERIOD_MS           100U
#define SLE_RL_TEST_MODE_FUNCTIONAL     1
#define SLE_RL_TEST_MODE_RECOVERY       2
#define SLE_RL_TEST_MODE                SLE_RL_TEST_MODE_RECOVERY
#define SLE_RL_RECOVERY_FRAME_COUNT     100U

static ssapc_find_service_result_t g_sle_uart_find_service_result = { 0 };
static sle_announce_seek_callbacks_t g_sle_uart_seek_cbk = { 0 };
static sle_connection_callbacks_t g_sle_uart_connect_cbk = { 0 };
static ssapc_callbacks_t g_sle_uart_ssapc_cbk = { 0 };
static sle_addr_t g_sle_uart_remote_addr = { 0 };
ssapc_write_param_t g_sle_uart_send_param = { 0 };
uint16_t g_sle_uart_conn_id = 0;
static osal_semaphore g_sle_latency_ready_sem;
static osal_semaphore g_sle_latency_cfm_sem;
static osal_semaphore g_sle_latency_echo_sem;
static osal_semaphore g_sle_conn_param_sem;
static osal_semaphore g_sle_rl_telemetry_sem;
static volatile bool g_sle_latency_initialized = false;
static volatile bool g_sle_latency_ready_signaled = false;
static volatile bool g_sle_latency_active = false;
static volatile errcode_t g_sle_latency_last_cfm_status = ERRCODE_SUCC;
static volatile bool g_sle_conn_param_callback_received = false;
static volatile errcode_t g_sle_conn_param_status = ERRCODE_FAIL;
static volatile uint16_t g_sle_conn_param_actual_interval = 0;
static volatile uint16_t g_sle_conn_param_actual_latency = 0;
static volatile uint16_t g_sle_conn_param_actual_supervision = 0;
static volatile uint64_t g_sle_recovery_disconnected_us = 0;
static volatile uint64_t g_sle_recovery_connected_us = 0;
static volatile uint64_t g_sle_recovery_pair_complete_us = 0;
static volatile uint64_t g_sle_recovery_ready_us = 0;
static volatile uint32_t g_sle_recovery_cycle = 0;
static volatile uint32_t g_sle_recovery_last_completed_cycle = 0;
static volatile uint32_t g_sle_rl_command_seq = 0;
static volatile uint8_t g_sle_rl_expected_command = 0;
static volatile uint32_t g_sle_rl_expected_command_seq = 0;
static volatile uint8_t g_sle_rl_last_ack_status = SLE_RL_ACK_INTERNAL_ERROR;
static volatile uint64_t g_sle_rl_command_send_us = 0;
static volatile uint64_t g_sle_rl_command_rtt_us = 0;
static volatile uint32_t g_sle_rl_rx_total = 0;
static volatile uint32_t g_sle_rl_rx_lost = 0;
static volatile uint32_t g_sle_rl_rx_duplicate = 0;
static volatile uint32_t g_sle_rl_rx_out_of_order = 0;
static volatile uint32_t g_sle_rl_rx_crc_error = 0;
static volatile uint32_t g_sle_rl_rx_content_error = 0;
static volatile uint32_t g_sle_rl_expected_telemetry_seq = 0;
static volatile bool g_sle_rl_have_telemetry_seq = false;


static void sle_uart_client_connect_param_update_req_cbk(uint16_t conn_id, errcode_t status,
    const sle_connection_param_update_req_t *param)
{
    if (param == NULL) {
        osal_printk("[CONN_PARAM][CLIENT_REQ] conn_id=%u status=0x%x param=NULL\r\n", conn_id, status);
        return;
    }
    osal_printk("[CONN_PARAM][CLIENT_REQ] conn_id=%u status=0x%x interval_min=%u interval_max=%u "
        "max_latency=%u supervision=%u\r\n", conn_id, status, param->interval_min,
        param->interval_max, param->max_latency, param->supervision_timeout);
}

static void sle_uart_client_connect_param_update_cbk(uint16_t conn_id, errcode_t status,
    const sle_connection_param_update_evt_t *param)
{
    g_sle_conn_param_status = status;
    g_sle_conn_param_callback_received = true;
    if (param != NULL) {
        g_sle_conn_param_actual_interval = param->interval;
        g_sle_conn_param_actual_latency = param->latency;
        g_sle_conn_param_actual_supervision = param->supervision;
        osal_printk("[CONN_PARAM][CLIENT_UPDATED] conn_id=%u status=0x%x interval=%u latency=%u "
            "supervision=%u\r\n", conn_id, status, param->interval, param->latency,
            param->supervision);
    } else {
        osal_printk("[CONN_PARAM][CLIENT_UPDATED] conn_id=%u status=0x%x param=NULL\r\n", conn_id, status);
    }
    osal_sem_up(&g_sle_conn_param_sem);
}

uint16_t get_g_sle_uart_conn_id(void)
{
    return g_sle_uart_conn_id;
}

ssapc_write_param_t *get_g_sle_uart_send_param(void)
{
    return &g_sle_uart_send_param;
}

bool sle_rl_client_handle_notification(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len < 2) ||
        (data[0] != SLE_RL_MAGIC_0) || (data[1] != SLE_RL_MAGIC_1)) {
        return false;
    }

    sle_rl_frame_view_t view = {0};
    sle_rl_parse_result_t parse_ret = sle_rl_decode(data, len, &view);
    if (parse_ret != SLE_RL_PARSE_OK) {
        if (parse_ret == SLE_RL_PARSE_CRC) {
            g_sle_rl_rx_crc_error++;
        }
        osal_printk("[RL][ERROR] direction=SERVER_TO_CLIENT reason=%s len=%u crc_error=%u\r\n",
            sle_rl_parse_result_name(parse_ret), len, g_sle_rl_rx_crc_error);
        return true;
    }

    if (view.type == SLE_RL_TYPE_TELEMETRY) {
        if (view.payload_len != 20U) {
            g_sle_rl_rx_content_error++;
            osal_printk("[RL][ERROR] reason=TELEMETRY_LENGTH len=%u\r\n", view.payload_len);
            return true;
        }
        uint32_t payload_seq = sle_rl_get_le32(&view.payload[4]);
        if (payload_seq != view.seq) {
            g_sle_rl_rx_content_error++;
        }
        if (!g_sle_rl_have_telemetry_seq) {
            g_sle_rl_have_telemetry_seq = true;
            g_sle_rl_expected_telemetry_seq = view.seq + 1U;
        } else if (view.seq == g_sle_rl_expected_telemetry_seq) {
            g_sle_rl_expected_telemetry_seq++;
        } else if (view.seq > g_sle_rl_expected_telemetry_seq) {
            g_sle_rl_rx_lost += view.seq - g_sle_rl_expected_telemetry_seq;
            g_sle_rl_expected_telemetry_seq = view.seq + 1U;
        } else if ((view.seq + 1U) == g_sle_rl_expected_telemetry_seq) {
            g_sle_rl_rx_duplicate++;
        } else {
            g_sle_rl_rx_out_of_order++;
        }
        g_sle_rl_rx_total++;
        osal_sem_up(&g_sle_rl_telemetry_sem);
        if ((g_sle_rl_rx_total % 100U) == 0) {
            osal_printk("[RL][CLIENT_SUMMARY] rx=%u lost=%u duplicate=%u out_of_order=%u "
                "crc_error=%u content_error=%u last_seq=%u\r\n", g_sle_rl_rx_total,
                g_sle_rl_rx_lost, g_sle_rl_rx_duplicate, g_sle_rl_rx_out_of_order,
                g_sle_rl_rx_crc_error, g_sle_rl_rx_content_error, view.seq);
        }
        return true;
    }

    if (view.type == SLE_RL_TYPE_COMMAND_ACK) {
        if (view.payload_len < 6U) {
            g_sle_rl_rx_content_error++;
            return true;
        }
        uint8_t command = view.payload[0];
        uint8_t ack_status = view.payload[1];
        uint32_t command_seq = sle_rl_get_le32(&view.payload[2]);
        osal_printk("[RL][ACK_RX] command_seq=%u command=%u status=%u frame_seq=%u len=%u\r\n",
            command_seq, command, ack_status, view.seq, view.payload_len);
        if (g_sle_latency_active && (command == g_sle_rl_expected_command) &&
            (command_seq == g_sle_rl_expected_command_seq)) {
            g_sle_rl_last_ack_status = ack_status;
            g_sle_rl_command_rtt_us = uapi_tcxo_get_us() - g_sle_rl_command_send_us;
            osal_sem_up(&g_sle_latency_echo_sem);
        }
        return true;
    }

    osal_printk("[RL][ERROR] reason=UNEXPECTED_TYPE type=%u\r\n", view.type);
    return true;
}

static bool sle_rl_client_send_command_expect(uint8_t command, const uint8_t *param,
    uint16_t param_len, uint8_t expected_ack_status)
{
    if ((param_len > (SLE_RL_MAX_PAYLOAD_LEN - 1U)) ||
        ((param_len > 0) && (param == NULL))) {
        return false;
    }

    uint8_t payload[SLE_RL_MAX_PAYLOAD_LEN] = {0};
    uint8_t frame[SLE_RL_MAX_FRAME_LEN] = {0};
    uint32_t command_seq = g_sle_rl_command_seq++;
    payload[0] = command;
    if (param_len > 0) {
        (void)memcpy_s(&payload[1], sizeof(payload) - 1U, param, param_len);
    }
    uint16_t frame_len = sle_rl_encode(frame, sizeof(frame), SLE_RL_TYPE_COMMAND, 0,
        command_seq, payload, (uint16_t)(param_len + 1U));
    if (frame_len == 0) {
        return false;
    }

    g_sle_rl_expected_command = command;
    g_sle_rl_expected_command_seq = command_seq;
    g_sle_rl_last_ack_status = SLE_RL_ACK_INTERNAL_ERROR;
    g_sle_rl_command_rtt_us = 0;
    g_sle_latency_last_cfm_status = ERRCODE_SUCC;
    g_sle_latency_active = true;
    g_sle_rl_command_send_us = uapi_tcxo_get_us();
    g_sle_uart_send_param.data_len = frame_len;
    g_sle_uart_send_param.data = frame;

    errcode_t ret = ssapc_write_req(0, g_sle_uart_conn_id, &g_sle_uart_send_param);
    if (ret != ERRCODE_SUCC) {
        g_sle_latency_active = false;
        osal_printk("[RL][COMMAND_RESULT] seq=%u command=%u result=SUBMIT_FAIL ret=0x%x\r\n",
            command_seq, command, ret);
        return false;
    }
    if (osal_sem_down_timeout(&g_sle_latency_cfm_sem, SLE_RL_COMMAND_TIMEOUT_MS) != OSAL_SUCCESS) {
        g_sle_latency_active = false;
        osal_printk("[RL][COMMAND_RESULT] seq=%u command=%u result=CFM_TIMEOUT\r\n",
            command_seq, command);
        return false;
    }
    if (g_sle_latency_last_cfm_status != ERRCODE_SUCC) {
        g_sle_latency_active = false;
        osal_printk("[RL][COMMAND_RESULT] seq=%u command=%u result=CFM_FAIL status=0x%x\r\n",
            command_seq, command, g_sle_latency_last_cfm_status);
        return false;
    }
    if (osal_sem_down_timeout(&g_sle_latency_echo_sem, SLE_RL_COMMAND_TIMEOUT_MS) != OSAL_SUCCESS) {
        g_sle_latency_active = false;
        osal_printk("[RL][COMMAND_RESULT] seq=%u command=%u result=ACK_TIMEOUT\r\n",
            command_seq, command);
        return false;
    }
    g_sle_latency_active = false;
    bool pass = g_sle_rl_last_ack_status == expected_ack_status;
    osal_printk("[RL][COMMAND_RESULT] seq=%u command=%u ack_status=%u expected_status=%u "
        "rtt_us=%llu result=%s\r\n", command_seq, command, g_sle_rl_last_ack_status,
        expected_ack_status, g_sle_rl_command_rtt_us,
        pass ? "PASS" : "FAIL");
    return pass;
}

static bool sle_rl_client_send_command(uint8_t command, const uint8_t *param,
    uint16_t param_len)
{
    return sle_rl_client_send_command_expect(command, param, param_len, SLE_RL_ACK_OK);
}

#if SLE_RL_TEST_MODE == SLE_RL_TEST_MODE_FUNCTIONAL
static bool sle_rl_client_submit_raw(const uint8_t *frame, uint16_t frame_len,
    const char *case_name)
{
    g_sle_latency_last_cfm_status = ERRCODE_SUCC;
    g_sle_latency_active = true;
    g_sle_uart_send_param.data_len = frame_len;
    g_sle_uart_send_param.data = (uint8_t *)frame;
    errcode_t ret = ssapc_write_req(0, g_sle_uart_conn_id, &g_sle_uart_send_param);
    if (ret != ERRCODE_SUCC) {
        g_sle_latency_active = false;
        osal_printk("[RL][INVALID_SUBMIT] case=%s result=SUBMIT_FAIL ret=0x%x\r\n",
            case_name, ret);
        return false;
    }
    if (osal_sem_down_timeout(&g_sle_latency_cfm_sem, SLE_RL_COMMAND_TIMEOUT_MS) != OSAL_SUCCESS) {
        g_sle_latency_active = false;
        osal_printk("[RL][INVALID_SUBMIT] case=%s result=CFM_TIMEOUT\r\n", case_name);
        return false;
    }
    g_sle_latency_active = false;
    bool pass = g_sle_latency_last_cfm_status == ERRCODE_SUCC;
    osal_printk("[RL][INVALID_SUBMIT] case=%s cfm_status=0x%x result=%s\r\n",
        case_name, g_sle_latency_last_cfm_status, pass ? "SUBMITTED" : "CFM_FAIL");
    osal_msleep(100);
    return pass;
}

static bool sle_rl_client_run_invalid_tests(void)
{
    bool pass = true;
    uint8_t payload[] = {SLE_RL_CMD_GET_STATUS};
    uint8_t frame[SLE_RL_MAX_FRAME_LEN] = {0};
    uint16_t frame_len = sle_rl_encode(frame, sizeof(frame), SLE_RL_TYPE_COMMAND, 0,
        g_sle_rl_command_seq++, payload, sizeof(payload));
    if (frame_len == 0) {
        return false;
    }

    frame[frame_len - 1U] ^= 0x01U;
    pass = sle_rl_client_submit_raw(frame, frame_len, "CRC") && pass;
    frame[frame_len - 1U] ^= 0x01U;

    frame[2] = (uint8_t)(SLE_RL_VERSION + 1U);
    pass = sle_rl_client_submit_raw(frame, frame_len, "VERSION") && pass;
    frame[2] = SLE_RL_VERSION;

    frame[9] = (uint8_t)(frame[9] + 1U);
    pass = sle_rl_client_submit_raw(frame, frame_len, "LENGTH") && pass;

    pass = sle_rl_client_send_command_expect(0x7FU, NULL, 0,
        SLE_RL_ACK_UNKNOWN_COMMAND) && pass;
    uint8_t period_param[4] = {0};
    sle_rl_put_le32(period_param, 50U);
    pass = sle_rl_client_send_command_expect(SLE_RL_CMD_SET_PERIOD,
        period_param, sizeof(period_param), SLE_RL_ACK_BAD_PARAMETER) && pass;

    osal_printk("[RL][INVALID_SUMMARY] cases=5 result=%s\r\n", pass ? "PASS" : "FAIL");
    return pass;
}
#endif

static bool sle_rl_client_wait_telemetry(uint32_t count, uint32_t timeout_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        if (osal_sem_down_timeout(&g_sle_rl_telemetry_sem, timeout_ms) != OSAL_SUCCESS) {
            osal_printk("[RL][ERROR] reason=TELEMETRY_TIMEOUT received_in_step=%u expected=%u\r\n",
                i, count);
            return false;
        }
    }
    return true;
}

static void sle_rl_client_reset_rx_stats(void)
{
    g_sle_rl_rx_total = 0;
    g_sle_rl_rx_lost = 0;
    g_sle_rl_rx_duplicate = 0;
    g_sle_rl_rx_out_of_order = 0;
    g_sle_rl_rx_crc_error = 0;
    g_sle_rl_rx_content_error = 0;
    g_sle_rl_have_telemetry_seq = false;
}

static void *sle_rl_client_recovery_task(const char *arg)
{
    unused(arg);
    osal_printk("[RL][RECOVERY_CONFIG] cycles=%u frames_per_cycle=%u period_ms=%u "
        "pair_mode=%s stale_pair_fallback=%u\r\n", SLE_RECOVERY_CYCLE_COUNT,
        SLE_RL_RECOVERY_FRAME_COUNT, SLE_RL_TEST_PERIOD_MS,
        SLE_RECOVERY_RETAIN_PAIR ? "retain_pair" : "remove_pair",
        SLE_RECOVERY_STALE_PAIR_FALLBACK);

    for (uint32_t cycle = 1; cycle <= SLE_RECOVERY_CYCLE_COUNT; cycle++) {
        g_sle_recovery_cycle = cycle;
        g_sle_recovery_disconnected_us = 0;
        osal_printk("[RL][RECOVERY_WAIT] cycle=%u waiting_for_service_ready\r\n", cycle);
        if (osal_sem_down(&g_sle_latency_ready_sem) != OSAL_SUCCESS) {
            osal_printk("[RL][RECOVERY_ERROR] cycle=%u reason=WAIT_READY_FAILED\r\n", cycle);
            break;
        }
        osal_msleep(200);

        sle_rl_client_reset_rx_stats();
        uint8_t period_param[4] = {0};
        sle_rl_put_le32(period_param, SLE_RL_TEST_PERIOD_MS);
        bool pass = sle_rl_client_send_command(SLE_RL_CMD_SET_PERIOD,
            period_param, sizeof(period_param));
        pass = sle_rl_client_send_command(SLE_RL_CMD_START, NULL, 0) && pass;
        pass = sle_rl_client_wait_telemetry(SLE_RL_RECOVERY_FRAME_COUNT,
            SLE_RL_TELEMETRY_TIMEOUT_MS) && pass;
        pass = sle_rl_client_send_command(SLE_RL_CMD_STOP, NULL, 0) && pass;

        bool data_pass = pass && (g_sle_rl_rx_total == SLE_RL_RECOVERY_FRAME_COUNT) &&
            (g_sle_rl_rx_lost == 0) && (g_sle_rl_rx_duplicate == 0) &&
            (g_sle_rl_rx_out_of_order == 0) && (g_sle_rl_rx_crc_error == 0) &&
            (g_sle_rl_rx_content_error == 0);
        osal_printk("[RL][RECOVERY_PHASE] cycle=%u disconnect_to_connect_us=%llu "
            "connect_to_pair_us=%llu pair_to_ready_us=%llu disconnect_to_ready_us=%llu\r\n",
            cycle,
            (g_sle_recovery_disconnected_us > 0) ?
                (g_sle_recovery_connected_us - g_sle_recovery_disconnected_us) : 0,
            g_sle_recovery_pair_complete_us - g_sle_recovery_connected_us,
            g_sle_recovery_ready_us - g_sle_recovery_pair_complete_us,
            (g_sle_recovery_disconnected_us > 0) ?
                (g_sle_recovery_ready_us - g_sle_recovery_disconnected_us) : 0);
        osal_printk("[RL][RECOVERY_DATA] cycle=%u expected=%u rx=%u lost=%u duplicate=%u "
            "out_of_order=%u crc_error=%u content_error=%u result=%s\r\n",
            cycle, SLE_RL_RECOVERY_FRAME_COUNT, g_sle_rl_rx_total,
            g_sle_rl_rx_lost, g_sle_rl_rx_duplicate, g_sle_rl_rx_out_of_order,
            g_sle_rl_rx_crc_error, g_sle_rl_rx_content_error,
            data_pass ? "PASS" : "FAIL");
        g_sle_recovery_last_completed_cycle = cycle;
        if (cycle < SLE_RECOVERY_CYCLE_COUNT) {
            osal_printk("[RL][RECOVERY_ACTION] cycle=%u complete; reset Server now\r\n", cycle);
        }
    }

    osal_printk("[RL][RECOVERY_COMPLETE] cycles=%u\r\n", SLE_RECOVERY_CYCLE_COUNT);
    return NULL;
}

static void *sle_latency_task(const char *arg)
{
#if SLE_RL_TEST_MODE == SLE_RL_TEST_MODE_RECOVERY
    return sle_rl_client_recovery_task(arg);
#else
    unused(arg);
    osal_printk("[RL][PROTOCOL_SELF_TEST] role=client result=%s\r\n",
        sle_rl_protocol_self_test() ? "PASS" : "FAIL");
    osal_printk("[RL][CLIENT_INIT] waiting_for_service_ready\r\n");
    if (osal_sem_down(&g_sle_latency_ready_sem) != OSAL_SUCCESS) {
        osal_printk("[RL][ERROR] reason=WAIT_READY_FAILED\r\n");
        return NULL;
    }
    osal_msleep(200);
    osal_printk("[RL][LINK_READY] conn_id=%u handle=%u\r\n",
        g_sle_uart_conn_id, g_sle_uart_send_param.handle);

    uint8_t period_param[4] = {0};
    bool pass = true;
    pass = sle_rl_client_send_command(SLE_RL_CMD_GET_STATUS, NULL, 0) && pass;
    pass = sle_rl_client_send_command(SLE_RL_CMD_START, NULL, 0) && pass;
    pass = sle_rl_client_wait_telemetry(SLE_RL_FUNCTIONAL_COUNT,
        SLE_RL_TELEMETRY_TIMEOUT_MS) && pass;

    sle_rl_put_le32(period_param, 200U);
    pass = sle_rl_client_send_command(SLE_RL_CMD_SET_PERIOD,
        period_param, sizeof(period_param)) && pass;
    pass = sle_rl_client_wait_telemetry(SLE_RL_FUNCTIONAL_COUNT,
        SLE_RL_TELEMETRY_TIMEOUT_MS) && pass;

    uint8_t ping_param[8] = {0};
    uint64_t ping_value = uapi_tcxo_get_us();
    for (uint32_t i = 0; i < sizeof(ping_param); i++) {
        ping_param[i] = (uint8_t)((ping_value >> (i * 8U)) & 0xFFU);
    }
    pass = sle_rl_client_send_command(SLE_RL_CMD_PING,
        ping_param, sizeof(ping_param)) && pass;
    pass = sle_rl_client_send_command(SLE_RL_CMD_STOP, NULL, 0) && pass;
    osal_printk("[RL][FUNCTIONAL_SUMMARY] rx=%u lost=%u duplicate=%u crc_error=%u "
        "content_error=%u result=%s\r\n", g_sle_rl_rx_total, g_sle_rl_rx_lost,
        g_sle_rl_rx_duplicate, g_sle_rl_rx_crc_error, g_sle_rl_rx_content_error,
        pass ? "PASS" : "FAIL");

    pass = sle_rl_client_send_command(SLE_RL_CMD_RESET_STATS, NULL, 0) && pass;
    pass = sle_rl_client_run_invalid_tests() && pass;

    sle_rl_client_reset_rx_stats();
    sle_rl_put_le32(period_param, SLE_RL_TEST_PERIOD_MS);
    pass = sle_rl_client_send_command(SLE_RL_CMD_SET_PERIOD,
        period_param, sizeof(period_param));
    pass = sle_rl_client_send_command(SLE_RL_CMD_START, NULL, 0) && pass;
    pass = sle_rl_client_wait_telemetry(SLE_RL_RELIABILITY_COUNT,
        SLE_RL_TELEMETRY_TIMEOUT_MS) && pass;
    pass = sle_rl_client_send_command(SLE_RL_CMD_STOP, NULL, 0) && pass;

    bool reliability_pass = pass && (g_sle_rl_rx_total == SLE_RL_RELIABILITY_COUNT) &&
        (g_sle_rl_rx_lost == 0) && (g_sle_rl_rx_duplicate == 0) &&
        (g_sle_rl_rx_out_of_order == 0) && (g_sle_rl_rx_crc_error == 0) &&
        (g_sle_rl_rx_content_error == 0);
    osal_printk("[RL][RELIABILITY_SUMMARY] expected=%u rx=%u lost=%u duplicate=%u "
        "out_of_order=%u crc_error=%u content_error=%u result=%s\r\n",
        SLE_RL_RELIABILITY_COUNT, g_sle_rl_rx_total, g_sle_rl_rx_lost,
        g_sle_rl_rx_duplicate, g_sle_rl_rx_out_of_order, g_sle_rl_rx_crc_error,
        g_sle_rl_rx_content_error, reliability_pass ? "PASS" : "FAIL");

    return NULL;
#endif
}

static void sle_latency_init_once(void)
{
    osal_task *task_handle = NULL;

    if (g_sle_latency_initialized) {
        return;
    }

    if ((osal_sem_init(&g_sle_latency_ready_sem, 0) != OSAL_SUCCESS) ||
        (osal_sem_init(&g_sle_latency_cfm_sem, 0) != OSAL_SUCCESS) ||
        (osal_sem_init(&g_sle_latency_echo_sem, 0) != OSAL_SUCCESS) ||
        (osal_sem_init(&g_sle_conn_param_sem, 0) != OSAL_SUCCESS) ||
        (osal_sem_init(&g_sle_rl_telemetry_sem, 0) != OSAL_SUCCESS)) {
        osal_printk("[RL] semaphore init failed\r\n");
        return;
    }

    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)sle_latency_task,
        0, "SLEReliableClient", SLE_LATENCY_TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, SLE_LATENCY_TASK_PRIO);
        g_sle_latency_initialized = true;
        osal_printk("[RL] client task initialized\r\n");
    }
    osal_kthread_unlock();

    if (task_handle == NULL) {
        osal_printk("[RL] client task create failed\r\n");
    }
}

void sle_uart_start_scan(void)
{
    sle_seek_param_t param = { 0 };
    param.own_addr_type = 0;
    param.filter_duplicates = 0;
    param.seek_filter_policy = 0;
    param.seek_phys = 1;
    param.seek_type[0] = 1;
    param.seek_interval[0] = SLE_SEEK_INTERVAL_DEFAULT;
    param.seek_window[0] = SLE_SEEK_WINDOW_DEFAULT;
    sle_set_seek_param(&param);
    sle_start_seek();
}

static void sle_uart_client_sample_sle_enable_cbk(errcode_t status)
{
    osal_printk("sle enable: %d.\r\n", status);
    sle_uart_client_init(sle_uart_notification_cb, sle_uart_indication_cb);
    sle_uart_start_scan();
}

static void sle_uart_client_sample_seek_enable_cbk(errcode_t status)
{
    if (status != 0) {
        osal_printk("%s sle_uart_client_sample_seek_enable_cbk,status error\r\n", SLE_UART_CLIENT_LOG);
    }
}

static void sle_uart_client_sample_seek_result_info_cbk(sle_seek_result_info_t *seek_result_data)
{
    osal_printk("%s sle uart scan data :%s\r\n", SLE_UART_CLIENT_LOG, seek_result_data->data);
    if (seek_result_data == NULL) {
        osal_printk("status error\r\n");
    } else if (strstr((const char *)seek_result_data->data, SLE_UART_SERVER_NAME) != NULL) {
        memcpy_s(&g_sle_uart_remote_addr, sizeof(sle_addr_t), &seek_result_data->addr, sizeof(sle_addr_t));
        sle_stop_seek();
    }
}

static void sle_uart_client_sample_seek_disable_cbk(errcode_t status)
{
    if (status != 0) {
        osal_printk("%s sle_uart_client_sample_seek_disable_cbk,status error = %x\r\n", SLE_UART_CLIENT_LOG, status);
    } else {
#if !SLE_RECOVERY_RETAIN_PAIR
        sle_remove_paired_remote_device(&g_sle_uart_remote_addr);
#endif
        sle_connect_remote_device(&g_sle_uart_remote_addr);
    }
}

static void sle_uart_client_sample_seek_cbk_register(void)
{
    g_sle_uart_seek_cbk.sle_enable_cb = sle_uart_client_sample_sle_enable_cbk;
    g_sle_uart_seek_cbk.seek_enable_cb = sle_uart_client_sample_seek_enable_cbk;
    g_sle_uart_seek_cbk.seek_result_cb = sle_uart_client_sample_seek_result_info_cbk;
    g_sle_uart_seek_cbk.seek_disable_cb = sle_uart_client_sample_seek_disable_cbk;
    sle_announce_seek_register_callbacks(&g_sle_uart_seek_cbk);
}

static void sle_uart_client_sample_connect_state_changed_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                                             sle_acb_state_t conn_state, sle_pair_state_t pair_state,
                                                             sle_disc_reason_t disc_reason)
{
    unused(addr);
    unused(pair_state);
    osal_printk("%s conn state changed disc_reason:0x%x\r\n", SLE_UART_CLIENT_LOG, disc_reason);
    g_sle_uart_conn_id = conn_id;
    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        g_sle_recovery_connected_us = uapi_tcxo_get_us();
        osal_printk("%s SLE_ACB_STATE_CONNECTED\r\n", SLE_UART_CLIENT_LOG);
        osal_printk("[SECURITY] cycle=%u pair_state_on_connect=%u mode=%s\r\n",
            (g_sle_recovery_cycle == 0) ? 1 : g_sle_recovery_cycle, pair_state,
            SLE_RECOVERY_RETAIN_PAIR ? "retain_pair" : "remove_pair");
        if (pair_state == SLE_PAIR_NONE) {
            sle_pair_remote_device(&g_sle_uart_remote_addr);
        } else {
            ssap_exchange_info_t info = {0};
            g_sle_recovery_pair_complete_us = g_sle_recovery_connected_us;
            info.mtu_size = SLE_MTU_SIZE_DEFAULT;
            info.version = 1;
            ssapc_exchange_info_req(0, g_sle_uart_conn_id, &info);
        }
    } else if (conn_state == SLE_ACB_STATE_NONE) {
        osal_printk("%s SLE_ACB_STATE_NONE\r\n", SLE_UART_CLIENT_LOG);
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        if (g_sle_recovery_disconnected_us == 0) {
            g_sle_recovery_disconnected_us = uapi_tcxo_get_us();
        }
        g_sle_latency_active = false;
        g_sle_latency_ready_signaled = false;
        g_sle_uart_send_param.handle = 0;
        osal_printk("%s SLE_ACB_STATE_DISCONNECTED\r\n", SLE_UART_CLIENT_LOG);
#if !SLE_RECOVERY_RETAIN_PAIR
        sle_remove_paired_remote_device(&g_sle_uart_remote_addr);
#endif
        osal_printk("[RECOVERY][DISCONNECTED] completed_cycle=%u reason=0x%x mode=%s\r\n",
            g_sle_recovery_last_completed_cycle, disc_reason,
            SLE_RECOVERY_RETAIN_PAIR ? "retain_pair" : "remove_pair");
        sle_uart_start_scan();
    } else {
        osal_printk("%s status error\r\n", SLE_UART_CLIENT_LOG);
    }
}

void  sle_uart_client_sample_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    g_sle_recovery_pair_complete_us = uapi_tcxo_get_us();
    osal_printk("%s pair complete conn_id:%d, addr:%02x***%02x%02x\n", SLE_UART_CLIENT_LOG, conn_id,
                addr->addr[0], addr->addr[4], addr->addr[5]);
    osal_printk("[SECURITY] cycle=%u pair_complete status=0x%x\r\n",
        (g_sle_recovery_cycle == 0) ? 1 : g_sle_recovery_cycle, status);
    if (status == 0) {
        ssap_exchange_info_t info = {0};
        info.mtu_size = SLE_MTU_SIZE_DEFAULT;
        info.version = 1;
        ssapc_exchange_info_req(0, g_sle_uart_conn_id, &info);
#if SLE_RECOVERY_RETAIN_PAIR && SLE_RECOVERY_STALE_PAIR_FALLBACK
    } else {
        errcode_t remove_ret = sle_remove_paired_remote_device(&g_sle_uart_remote_addr);
        osal_printk("[SECURITY][STALE_PAIR_FALLBACK] cycle=%u pair_status=0x%x "
            "remove_local_record_ret=0x%x action=retry_after_disconnect\r\n",
            (g_sle_recovery_cycle == 0) ? 1 : g_sle_recovery_cycle,
            status, remove_ret);
#endif
    }
}

static void sle_uart_client_sample_connect_cbk_register(void)
{
    g_sle_uart_connect_cbk.connect_state_changed_cb = sle_uart_client_sample_connect_state_changed_cbk;
    g_sle_uart_connect_cbk.connect_param_update_req_cb = sle_uart_client_connect_param_update_req_cbk;
    g_sle_uart_connect_cbk.connect_param_update_cb = sle_uart_client_connect_param_update_cbk;
    g_sle_uart_connect_cbk.pair_complete_cb =  sle_uart_client_sample_pair_complete_cbk;
    sle_connection_register_callbacks(&g_sle_uart_connect_cbk);
}

static void sle_uart_client_sample_exchange_info_cbk(uint8_t client_id, uint16_t conn_id, ssap_exchange_info_t *param,
                                                     errcode_t status)
{
    osal_printk("%s exchange_info_cbk,pair complete client id:%d status:%d\r\n",
                SLE_UART_CLIENT_LOG, client_id, status);
    osal_printk("%s exchange mtu, mtu size: %d, version: %d.\r\n", SLE_UART_CLIENT_LOG,
                param->mtu_size, param->version);
    ssapc_find_structure_param_t find_param = { 0 };
    find_param.type = SSAP_FIND_TYPE_PROPERTY;
    find_param.start_hdl = 1;
    find_param.end_hdl = 0xFFFF;
    ssapc_find_structure(0, conn_id, &find_param);
}

static void sle_uart_client_sample_find_structure_cbk(uint8_t client_id, uint16_t conn_id,
                                                      ssapc_find_service_result_t *service,
                                                      errcode_t status)
{
    osal_printk("%s find structure cbk client: %d conn_id:%d status: %d \r\n", SLE_UART_CLIENT_LOG,
                client_id, conn_id, status);
    osal_printk("%s find structure start_hdl:[0x%02x], end_hdl:[0x%02x], uuid len:%d\r\n", SLE_UART_CLIENT_LOG,
                service->start_hdl, service->end_hdl, service->uuid.len);
    g_sle_uart_find_service_result.start_hdl = service->start_hdl;
    g_sle_uart_find_service_result.end_hdl = service->end_hdl;
    memcpy_s(&g_sle_uart_find_service_result.uuid, sizeof(sle_uuid_t), &service->uuid, sizeof(sle_uuid_t));
}

static void sle_uart_client_sample_find_property_cbk(uint8_t client_id, uint16_t conn_id,
                                                     ssapc_find_property_result_t *property, errcode_t status)
{
    osal_printk("%s sle_uart_client_sample_find_property_cbk, client id: %d, conn id: %d, operate ind: %d, "
                "descriptors count: %d status:%d property->handle %d\r\n", SLE_UART_CLIENT_LOG,
                client_id, conn_id, property->operate_indication,
                property->descriptors_count, status, property->handle);
    g_sle_uart_send_param.handle = property->handle;
    g_sle_uart_send_param.type = SSAP_PROPERTY_TYPE_VALUE;
}

static void sle_uart_client_sample_find_structure_cmp_cbk(uint8_t client_id, uint16_t conn_id,
                                                          ssapc_find_structure_result_t *structure_result,
                                                          errcode_t status)
{
    unused(conn_id);
    osal_printk("%s sle_uart_client_sample_find_structure_cmp_cbk,client id:%d status:%d type:%d uuid len:%d \r\n",
                SLE_UART_CLIENT_LOG, client_id, status, structure_result->type, structure_result->uuid.len);
    if (g_sle_latency_initialized && !g_sle_latency_ready_signaled && (status == ERRCODE_SUCC) &&
        (g_sle_uart_send_param.handle != 0)) {
        g_sle_latency_ready_signaled = true;
        g_sle_recovery_ready_us = uapi_tcxo_get_us();
        osal_printk("[RECOVERY][READY] cycle=%u handle=%u\r\n",
            (g_sle_recovery_cycle == 0) ? 1 : g_sle_recovery_cycle,
            g_sle_uart_send_param.handle);
        osal_sem_up(&g_sle_latency_ready_sem);
    }
}

static void sle_uart_client_sample_write_cfm_cb(uint8_t client_id, uint16_t conn_id,
                                                ssapc_write_result_t *write_result, errcode_t status)
{
    if (g_sle_latency_active) {
        g_sle_latency_last_cfm_status = status;
        osal_sem_up(&g_sle_latency_cfm_sem);
        return;
    }
    osal_printk("%s sle_uart_client_sample_write_cfm_cb, conn_id:%d client id:%d status:%d handle:%02x type:%02x\r\n",
                SLE_UART_CLIENT_LOG, conn_id, client_id, status, write_result->handle, write_result->type);
}

static void sle_uart_client_sample_ssapc_cbk_register(ssapc_notification_callback notification_cb,
                                                      ssapc_notification_callback indication_cb)
{
    g_sle_uart_ssapc_cbk.exchange_info_cb = sle_uart_client_sample_exchange_info_cbk;
    g_sle_uart_ssapc_cbk.find_structure_cb = sle_uart_client_sample_find_structure_cbk;
    g_sle_uart_ssapc_cbk.ssapc_find_property_cbk = sle_uart_client_sample_find_property_cbk;
    g_sle_uart_ssapc_cbk.find_structure_cmp_cb = sle_uart_client_sample_find_structure_cmp_cbk;
    g_sle_uart_ssapc_cbk.write_cfm_cb = sle_uart_client_sample_write_cfm_cb;
    g_sle_uart_ssapc_cbk.notification_cb = notification_cb;
    g_sle_uart_ssapc_cbk.indication_cb = indication_cb;
    ssapc_register_callbacks(&g_sle_uart_ssapc_cbk);
}


void sle_uart_client_init(ssapc_notification_callback notification_cb, ssapc_indication_callback indication_cb)
{
    sle_latency_init_once();
    (void)osal_msleep(5000); /* 延时5s，等待SLE初始化完毕 */
    osal_printk("[SLE Client] try enable.\r\n");
    sle_uart_client_sample_seek_cbk_register();
    sle_uart_client_sample_connect_cbk_register();
    sle_uart_client_sample_ssapc_cbk_register(notification_cb, indication_cb);
    if (enable_sle() != ERRCODE_SUCC) {
        osal_printk("[SLE Client] sle enbale fail !\r\n");
    }
}
