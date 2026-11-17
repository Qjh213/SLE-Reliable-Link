/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: SLE UART sample of client. \n
 *
 * History: \n
 * 2023-04-03, Create file. \n
 */
#ifndef SLE_UART_CLIENT_H
#define SLE_UART_CLIENT_H

#include <stdbool.h>
#include "sle_ssap_client.h"

void sle_uart_client_init(ssapc_notification_callback notification_cb, ssapc_indication_callback indication_cb);

void sle_uart_start_scan(void);

uint16_t get_g_sle_uart_conn_id(void);

ssapc_write_param_t *get_g_sle_uart_send_param(void);
bool sle_rl_client_handle_notification(const uint8_t *data, uint16_t len);

/* 手动控制模式：串口字符缓存 + 是否启用（供 sle_uart.c 的 UART 回调判断） */
bool sle_rl_client_manual_enabled(void);
void sle_rl_client_manual_feed(const uint8_t *data, uint16_t len);
void sle_uart_notification_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data, errcode_t status);
void sle_uart_indication_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data, errcode_t status);

#endif
