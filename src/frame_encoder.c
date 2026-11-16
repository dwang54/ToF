/**
 ******************************************************************************
 * @file    frame_encoder.c
 * @brief   Binary wire protocol encoder. See frame_encoder.h for the layout.
 ******************************************************************************
 */

#include "frame_encoder.h"
#include "uart_api.h"

#include <string.h>

/* ------------------------------------------------------------------------- */
/* Little-endian writers                                                     */
/* ------------------------------------------------------------------------- */

uint16_t put_u8(uint8_t *buf, uint16_t off, uint8_t v)
{
    buf[off] = v;
    return off + 1;
}

uint16_t put_u16(uint8_t *buf, uint16_t off, uint16_t v)
{
    buf[off]     = (uint8_t)(v & 0xFFU);
    buf[off + 1] = (uint8_t)((v >> 8) & 0xFFU);
    return off + 2;
}

uint16_t put_u32(uint8_t *buf, uint16_t off, uint32_t v)
{
    buf[off]     = (uint8_t)(v & 0xFFU);
    buf[off + 1] = (uint8_t)((v >> 8)  & 0xFFU);
    buf[off + 2] = (uint8_t)((v >> 16) & 0xFFU);
    buf[off + 3] = (uint8_t)((v >> 24) & 0xFFU);
    return off + 4;
}

/* ------------------------------------------------------------------------- */
/* CRC-16/CCITT-FALSE                                                        */
/*   poly 0x1021, init 0xFFFF, no reflection, no final XOR                   */
/*   check: crc16_ccitt("123456789", 9) == 0x29B1                            */
/* ------------------------------------------------------------------------- */

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ------------------------------------------------------------------------- */
/* COBS encode                                                               */
/*                                                                           */
/* Walks the input emitting groups. Each group is a code byte followed by the */
/* non-zero bytes it covers. A code of N means "N-1 data bytes follow, then   */
/* an implicit zero". A code of 0xFF is the exception: 254 data bytes and NO  */
/* implicit zero -- that is how runs longer than 254 are split.              */
/* ------------------------------------------------------------------------- */

uint16_t cobs_encode(const uint8_t *src, uint16_t src_len, uint8_t *dst)
{
    uint16_t out      = 0;      /* write cursor in dst                    */
    uint16_t code_idx = 0;      /* where the current group's code byte is */
    uint8_t  code     = 1;      /* bytes in the current group, plus one   */

    dst[out++] = 0;             /* placeholder for the first code byte */

    for (uint16_t i = 0; i < src_len; i++) {
        if (src[i] != 0U) {
            dst[out++] = src[i];
            code++;

            if (code == 0xFFU) {
                /* Group full. Close it. Only open a new one if input
                 * remains -- otherwise we would emit a stray trailing
                 * code byte that the decoder must special-case. */
                dst[code_idx] = code;
                if (i + 1 < src_len) {
                    code_idx = out;
                    dst[out++] = 0;
                    code = 1;
                } else {
                    return out;
                }
            }
        } else {
            /* Zero byte: close the group. The code byte encodes where the
             * zero was, and the zero itself is not transmitted. */
            dst[code_idx] = code;
            code_idx = out;
            dst[out++] = 0;
            code = 1;
        }
    }

    dst[code_idx] = code;
    return out;
}

/* ------------------------------------------------------------------------- */
/* Frame assembly                                                            */
/* ------------------------------------------------------------------------- */

uint16_t frame_build_ranging(uint8_t *buf, uint16_t buf_len,
                             const VL53L7CX_ResultsData *res,
                             uint8_t grid_width,
                             uint32_t seq, uint32_t timestamp_ms)
{
    if ((buf == NULL) || (res == NULL)) {
        return 0;
    }
    if ((grid_width != 4U) && (grid_width != 8U)) {
        return 0;
    }

    const uint16_t zones = (uint16_t)grid_width * (uint16_t)grid_width;
    const uint16_t body  = FRAME_HEADER_LEN
                         + (uint16_t)(zones * ZONE_RECORD_LEN)
                         + FRAME_CRC_LEN;

    if (buf_len < body) {
        return 0;
    }

    uint16_t off = 0;
    off = put_u8 (buf, off, PROTO_VERSION);
    off = put_u8 (buf, off, FRAME_TYPE_RANGING);
    off = put_u8 (buf, off, grid_width);
    off = put_u32(buf, off, seq);
    off = put_u32(buf, off, timestamp_ms);

    for (uint16_t i = 0; i < zones; i++) {
        /* Index into the ULD arrays accounts for multiple targets per zone.
         * With VL53L7CX_NB_TARGET_PER_ZONE == 1 this is just i, but writing
         * it out means raising that macro does not silently corrupt the
         * mapping. */
        const uint16_t k = (uint16_t)(i * VL53L7CX_NB_TARGET_PER_ZONE);

        /* Cast through the unsigned type of the same width: the bit pattern
         * is what matters, and Python's '<h' reinterprets it as signed. */
        off = put_u16(buf, off, (uint16_t)(int16_t)res->distance_mm[k]);
        off = put_u16(buf, off, (uint16_t)res->range_sigma_mm[k]);
        off = put_u8 (buf, off, res->target_status[k]);
    }

    /* CRC covers everything written so far, i.e. bytes 0 .. off-1. */
    off = put_u16(buf, off, crc16_ccitt(buf, off));

    return off;
}

/* ------------------------------------------------------------------------- */
/* Send                                                                      */
/* ------------------------------------------------------------------------- */

void frame_send_ranging(const VL53L7CX_ResultsData *res,
                        uint8_t grid_width, uint32_t seq)
{
    /* Static, not stack: 333 + 336 bytes is far too much for an embedded
     * stack frame, and both buffers are reused every frame anyway. */
    static uint8_t body[FRAME_BODY_MAX];
    static uint8_t wire[FRAME_WIRE_MAX];

    const uint16_t body_len = frame_build_ranging(body, sizeof(body), res,
                                                  grid_width, seq,
                                                  HAL_GetTick());
    if (body_len == 0U) {
        return;
    }

    uint16_t wire_len = cobs_encode(body, body_len, wire);
    wire[wire_len++] = 0x00;                 /* frame delimiter */

    /* One frame, one write. Splitting it would force the receiver to
     * reassemble and the CRC would no longer cover a whole unit. */
    UART_API_Write(wire, wire_len);
}

/* ------------------------------------------------------------------------- */
/* Extended frame: base record + signal_per_spad + ambient_per_spad         */
/* ------------------------------------------------------------------------- */

static uint16_t clamp_u16(uint32_t v)
{
    return (v > 0xFFFFU) ? 0xFFFFU : (uint16_t)v;
}

uint16_t frame_build_ranging_ext(uint8_t *buf, uint16_t buf_len,
                                 const VL53L7CX_ResultsData *res,
                                 uint8_t grid_width,
                                 uint32_t seq, uint32_t timestamp_ms)
{
    if ((buf == NULL) || (res == NULL)) {
        return 0;
    }
    if ((grid_width != 4U) && (grid_width != 8U)) {
        return 0;
    }

    const uint16_t zones = (uint16_t)grid_width * (uint16_t)grid_width;
    const uint16_t body  = FRAME_HEADER_LEN
                         + (uint16_t)(zones * ZONE_RECORD_LEN_EXT)
                         + FRAME_CRC_LEN;

    if (buf_len < body) {
        return 0;
    }

    uint16_t off = 0;
    off = put_u8 (buf, off, PROTO_VERSION);
    off = put_u8 (buf, off, FRAME_TYPE_RANGING_EXT);
    off = put_u8 (buf, off, grid_width);
    off = put_u32(buf, off, seq);
    off = put_u32(buf, off, timestamp_ms);

    for (uint16_t i = 0; i < zones; i++) {
        const uint16_t k = (uint16_t)(i * VL53L7CX_NB_TARGET_PER_ZONE);

        off = put_u16(buf, off, (uint16_t)(int16_t)res->distance_mm[k]);
        off = put_u16(buf, off, (uint16_t)res->range_sigma_mm[k]);
        off = put_u8 (buf, off, res->target_status[k]);

        /* signal_per_spad is per-target (index k); ambient_per_spad is
         * per-zone (index i, shared across targets). Both are kcps/spad
         * as uint32 in the driver -- clamp to uint16 for the wire, which
         * is fine for trend logging (values realistically stay well
         * under 65535 kcps/spad on this sensor). */
        off = put_u16(buf, off, clamp_u16(res->signal_per_spad[k]));
        off = put_u16(buf, off, clamp_u16(res->ambient_per_spad[i]));
    }

    off = put_u16(buf, off, crc16_ccitt(buf, off));

    return off;
}

void frame_send_ranging_ext(const VL53L7CX_ResultsData *res,
                            uint8_t grid_width, uint32_t seq)
{
    static uint8_t body[FRAME_BODY_MAX];
    static uint8_t wire[FRAME_WIRE_MAX];

    const uint16_t body_len = frame_build_ranging_ext(body, sizeof(body), res,
                                                      grid_width, seq,
                                                      HAL_GetTick());
    if (body_len == 0U) {
        return;
    }

    uint16_t wire_len = cobs_encode(body, body_len, wire);
    wire[wire_len++] = 0x00;

    UART_API_Write(wire, wire_len);
}

void frame_send_text(const char *msg)
{
    static uint8_t body[FRAME_BODY_MAX];
    static uint8_t wire[FRAME_WIRE_MAX];

    if (msg == NULL) {
        return;
    }

    size_t n = strlen(msg);
    const size_t room = FRAME_BODY_MAX - FRAME_HEADER_LEN - FRAME_CRC_LEN;
    if (n > room) {
        n = room;
    }

    uint16_t off = 0;
    off = put_u8 (body, off, PROTO_VERSION);
    off = put_u8 (body, off, FRAME_TYPE_TEXT);
    off = put_u8 (body, off, 0);                  /* grid width unused */
    off = put_u32(body, off, 0);                  /* sequence unused   */
    off = put_u32(body, off, HAL_GetTick());

    memcpy(&body[off], msg, n);
    off = (uint16_t)(off + n);

    off = put_u16(body, off, crc16_ccitt(body, off));

    uint16_t wire_len = cobs_encode(body, off, wire);
    wire[wire_len++] = 0x00;

    UART_API_Write(wire, wire_len);
}

/* ------------------------------------------------------------------------- */
/* Self-test                                                                 */
/*                                                                           */
/* Run this BEFORE switching the link to binary, while printf still works.   */
/* A wrong CRC or a broken COBS edge case produces "every frame fails" on the */
/* host with no indication of which side is at fault.                        */
/* ------------------------------------------------------------------------- */

bool frame_encoder_selftest(void)
{
    /* 1. CRC against the published check vector. */
    if (crc16_ccitt((const uint8_t *)"123456789", 9) != 0x29B1U) {
        return false;
    }

    /* 2. COBS must never emit a zero byte, and must handle the 254
     *    boundary. A 300-byte run of non-zeros exercises the 0xFF path. */
    static uint8_t src[300];
    static uint8_t dst[400];

    for (uint16_t i = 0; i < sizeof(src); i++) {
        src[i] = (uint8_t)(i % 255U + 1U);        /* never zero */
    }

    uint16_t n = cobs_encode(src, sizeof(src), dst);
    if (n == 0U) {
        return false;
    }
    for (uint16_t i = 0; i < n; i++) {
        if (dst[i] == 0U) {
            return false;                          /* framing invariant broken */
        }
    }

    /* 3. Zeros in the payload must survive too. */
    src[0] = 0; src[7] = 0; src[299] = 0;
    n = cobs_encode(src, sizeof(src), dst);
    for (uint16_t i = 0; i < n; i++) {
        if (dst[i] == 0U) {
            return false;
        }
    }

    return true;
}