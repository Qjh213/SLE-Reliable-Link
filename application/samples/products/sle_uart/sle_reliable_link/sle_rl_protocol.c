/**
 * Reliable-link application protocol for the SLE UART sample.
 */
#include <stddef.h>
#include <string.h>
#include "sle_rl_protocol.h"

#define SLE_RL_CRC_INITIAL             0xFFFFU
#define SLE_RL_CRC_POLYNOMIAL          0x1021U
#define SLE_RL_CRC_INPUT_OFFSET        2U
#define SLE_RL_CRC_FIXED_INPUT_LEN     (SLE_RL_HEADER_LEN - SLE_RL_CRC_INPUT_OFFSET)

uint16_t sle_rl_crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = SLE_RL_CRC_INITIAL;
    if ((data == NULL) && (len != 0)) {
        return 0;
    }

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000U) ?
                (uint16_t)((crc << 1) ^ SLE_RL_CRC_POLYNOMIAL) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void sle_rl_put_le16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void sle_rl_put_le32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

uint16_t sle_rl_get_le16(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

uint32_t sle_rl_get_le32(const uint8_t *src)
{
    return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
        ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

bool sle_rl_type_is_valid(uint8_t type)
{
    return (type >= SLE_RL_TYPE_TELEMETRY) && (type <= SLE_RL_TYPE_STATS);
}

uint16_t sle_rl_encode(uint8_t *out, uint16_t out_size, uint8_t type, uint8_t flags,
    uint32_t seq, const uint8_t *payload, uint16_t payload_len)
{
    if ((out == NULL) || !sle_rl_type_is_valid(type) ||
        (payload_len > SLE_RL_MAX_PAYLOAD_LEN) ||
        ((payload_len > 0) && (payload == NULL))) {
        return 0;
    }

    uint16_t frame_len = (uint16_t)(SLE_RL_HEADER_LEN + payload_len + SLE_RL_CRC_LEN);
    if (out_size < frame_len) {
        return 0;
    }

    out[0] = SLE_RL_MAGIC_0;
    out[1] = SLE_RL_MAGIC_1;
    out[2] = SLE_RL_VERSION;
    out[3] = type;
    out[4] = flags;
    sle_rl_put_le32(&out[5], seq);
    sle_rl_put_le16(&out[9], payload_len);
    if (payload_len > 0) {
        (void)memcpy(&out[SLE_RL_HEADER_LEN], payload, payload_len);
    }

    uint16_t crc_input_len = (uint16_t)(SLE_RL_CRC_FIXED_INPUT_LEN + payload_len);
    uint16_t crc = sle_rl_crc16_ccitt(&out[SLE_RL_CRC_INPUT_OFFSET], crc_input_len);
    sle_rl_put_le16(&out[SLE_RL_HEADER_LEN + payload_len], crc);
    return frame_len;
}

sle_rl_parse_result_t sle_rl_decode(const uint8_t *frame, uint16_t frame_len,
    sle_rl_frame_view_t *view)
{
    if ((frame == NULL) || (view == NULL)) {
        return SLE_RL_PARSE_NULL;
    }
    if (frame_len < (SLE_RL_HEADER_LEN + SLE_RL_CRC_LEN)) {
        return SLE_RL_PARSE_TOO_SHORT;
    }
    if ((frame[0] != SLE_RL_MAGIC_0) || (frame[1] != SLE_RL_MAGIC_1)) {
        return SLE_RL_PARSE_MAGIC;
    }
    if (frame[2] != SLE_RL_VERSION) {
        return SLE_RL_PARSE_VERSION;
    }
    if (!sle_rl_type_is_valid(frame[3])) {
        return SLE_RL_PARSE_TYPE;
    }

    uint16_t payload_len = sle_rl_get_le16(&frame[9]);
    if (payload_len > SLE_RL_MAX_PAYLOAD_LEN) {
        return SLE_RL_PARSE_LENGTH;
    }
    uint16_t expected_len = (uint16_t)(SLE_RL_HEADER_LEN + payload_len + SLE_RL_CRC_LEN);
    if (frame_len != expected_len) {
        return SLE_RL_PARSE_LENGTH;
    }

    uint16_t received_crc = sle_rl_get_le16(&frame[SLE_RL_HEADER_LEN + payload_len]);
    uint16_t crc_input_len = (uint16_t)(SLE_RL_CRC_FIXED_INPUT_LEN + payload_len);
    uint16_t calculated_crc = sle_rl_crc16_ccitt(&frame[SLE_RL_CRC_INPUT_OFFSET], crc_input_len);
    if (received_crc != calculated_crc) {
        return SLE_RL_PARSE_CRC;
    }

    view->type = frame[3];
    view->flags = frame[4];
    view->seq = sle_rl_get_le32(&frame[5]);
    view->payload_len = payload_len;
    view->payload = &frame[SLE_RL_HEADER_LEN];
    view->received_crc = received_crc;
    view->calculated_crc = calculated_crc;
    return SLE_RL_PARSE_OK;
}

const char *sle_rl_parse_result_name(sle_rl_parse_result_t result)
{
    switch (result) {
        case SLE_RL_PARSE_OK:
            return "OK";
        case SLE_RL_PARSE_NULL:
            return "NULL";
        case SLE_RL_PARSE_TOO_SHORT:
            return "TOO_SHORT";
        case SLE_RL_PARSE_MAGIC:
            return "MAGIC";
        case SLE_RL_PARSE_VERSION:
            return "VERSION";
        case SLE_RL_PARSE_LENGTH:
            return "LENGTH";
        case SLE_RL_PARSE_CRC:
            return "CRC";
        case SLE_RL_PARSE_TYPE:
            return "TYPE";
        default:
            return "UNKNOWN";
    }
}

bool sle_rl_protocol_self_test(void)
{
    static const uint8_t crc_vector[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    if (sle_rl_crc16_ccitt(crc_vector, sizeof(crc_vector)) != 0x29B1U) {
        return false;
    }

    uint8_t payload[] = {SLE_RL_CMD_PING, 0x12, 0x34, 0x56, 0x78};
    uint8_t frame[SLE_RL_MAX_FRAME_LEN] = {0};
    uint16_t frame_len = sle_rl_encode(frame, sizeof(frame), SLE_RL_TYPE_COMMAND,
        0xA5U, 0x12345678U, payload, sizeof(payload));
    if (frame_len == 0) {
        return false;
    }

    sle_rl_frame_view_t view = {0};
    if ((sle_rl_decode(frame, frame_len, &view) != SLE_RL_PARSE_OK) ||
        (view.type != SLE_RL_TYPE_COMMAND) || (view.flags != 0xA5U) ||
        (view.seq != 0x12345678U) || (view.payload_len != sizeof(payload)) ||
        (memcmp(view.payload, payload, sizeof(payload)) != 0)) {
        return false;
    }

    frame[frame_len - 1U] ^= 0x01U;
    if (sle_rl_decode(frame, frame_len, &view) != SLE_RL_PARSE_CRC) {
        return false;
    }
    frame[frame_len - 1U] ^= 0x01U;
    frame[2] = (uint8_t)(SLE_RL_VERSION + 1U);
    if (sle_rl_decode(frame, frame_len, &view) != SLE_RL_PARSE_VERSION) {
        return false;
    }
    return true;
}
