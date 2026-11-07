/**
 * Reliable-link application protocol for the SLE UART sample.
 */
#ifndef SLE_RL_PROTOCOL_H
#define SLE_RL_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SLE_RL_MAGIC_0                 0x53U /* 'S' */
#define SLE_RL_MAGIC_1                 0x4CU /* 'L' */
#define SLE_RL_VERSION                 0x01U
#define SLE_RL_HEADER_LEN              11U
#define SLE_RL_CRC_LEN                 2U
#define SLE_RL_MAX_PAYLOAD_LEN         48U
#define SLE_RL_MAX_FRAME_LEN           (SLE_RL_HEADER_LEN + SLE_RL_MAX_PAYLOAD_LEN + SLE_RL_CRC_LEN)

typedef enum {
    SLE_RL_TYPE_TELEMETRY = 0x01,
    SLE_RL_TYPE_COMMAND = 0x02,
    SLE_RL_TYPE_COMMAND_ACK = 0x03,
    SLE_RL_TYPE_HEARTBEAT = 0x04,
    SLE_RL_TYPE_STATS = 0x05,
} sle_rl_msg_type_t;

typedef enum {
    SLE_RL_CMD_START = 0x01,
    SLE_RL_CMD_STOP = 0x02,
    SLE_RL_CMD_SET_PERIOD = 0x03,
    SLE_RL_CMD_GET_STATUS = 0x04,
    SLE_RL_CMD_RESET_STATS = 0x05,
    SLE_RL_CMD_PING = 0x06,
} sle_rl_command_t;

typedef enum {
    SLE_RL_ACK_OK = 0x00,
    SLE_RL_ACK_UNKNOWN_COMMAND = 0x01,
    SLE_RL_ACK_BAD_LENGTH = 0x02,
    SLE_RL_ACK_BAD_PARAMETER = 0x03,
    SLE_RL_ACK_INTERNAL_ERROR = 0x04,
} sle_rl_ack_status_t;

typedef enum {
    SLE_RL_PARSE_OK = 0,
    SLE_RL_PARSE_NULL,
    SLE_RL_PARSE_TOO_SHORT,
    SLE_RL_PARSE_MAGIC,
    SLE_RL_PARSE_VERSION,
    SLE_RL_PARSE_LENGTH,
    SLE_RL_PARSE_CRC,
    SLE_RL_PARSE_TYPE,
} sle_rl_parse_result_t;

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint32_t seq;
    uint16_t payload_len;
    const uint8_t *payload;
    uint16_t received_crc;
    uint16_t calculated_crc;
} sle_rl_frame_view_t;

uint16_t sle_rl_crc16_ccitt(const uint8_t *data, uint16_t len);
void sle_rl_put_le16(uint8_t *dst, uint16_t value);
void sle_rl_put_le32(uint8_t *dst, uint32_t value);
uint16_t sle_rl_get_le16(const uint8_t *src);
uint32_t sle_rl_get_le32(const uint8_t *src);
bool sle_rl_type_is_valid(uint8_t type);

/**
 * Encodes one complete frame. Returns the frame length, or 0 on invalid input.
 */
uint16_t sle_rl_encode(uint8_t *out, uint16_t out_size, uint8_t type, uint8_t flags,
    uint32_t seq, const uint8_t *payload, uint16_t payload_len);

/**
 * Parses a frame without copying its payload. The input buffer must remain valid
 * while the returned view is used.
 */
sle_rl_parse_result_t sle_rl_decode(const uint8_t *frame, uint16_t frame_len,
    sle_rl_frame_view_t *view);

const char *sle_rl_parse_result_name(sle_rl_parse_result_t result);
bool sle_rl_protocol_self_test(void);

#ifdef __cplusplus
}
#endif

#endif
