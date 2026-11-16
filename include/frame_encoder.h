/**
 ******************************************************************************
 * @file    frame_encoder.h
 * @brief   Binary wire protocol encoder for VL53L7CX ranging frames.
 *
 * Frame layout (before COBS, little-endian throughout):
 *   0    1   version       0x01
 *   1    1   frame type    0x01 = ranging, 0x02 = text
 *   2    1   grid width    4 or 8
 *   3    4   sequence      uint32
 *   7    4   timestamp_ms  uint32
 *   11   N   zone records  N = grid_width^2 * 5
 *   11+N 2   CRC-16/CCITT  over bytes 0 .. 11+N-1
 *
 * Zone record, 5 bytes: int16 distance_mm, uint16 range_sigma_mm,
 * uint8 target_status.
 *
 * ORDER MATTERS: build frame -> append CRC -> COBS encode -> append 0x00.
 * The CRC covers the RAW frame, not the encoded bytes. frame_codec.py on
 * the host does the same; if the two disagree, every frame fails.
 ******************************************************************************
 */

#ifndef FRAME_ENCODER_H
#define FRAME_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "vl53l7cx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_VERSION       0x01
#define FRAME_TYPE_RANGING  0x01
#define FRAME_TYPE_TEXT     0x02
#define FRAME_TYPE_RANGING_EXT 0x03  /* adds signal_per_spad + ambient_per_spad, for range/SNR characterization */

#define FRAME_HEADER_LEN    11
#define ZONE_RECORD_LEN     5
#define ZONE_RECORD_LEN_EXT 9   /* base 5 + uint16 signal_kcps + uint16 ambient_kcps */
#define FRAME_CRC_LEN       2

/* Worst case is 8x8: 11 + 320 + 2 = 333 body bytes. COBS adds at most
 * len/254 + 1, then one delimiter byte. Size for 8x8 even at 4x4.
 * The EXT variant is 11 + 64*9 + 2 = 589 bytes -- FRAME_BODY_MAX below
 * already covers it since it's sized off the larger of the two record
 * lengths. */
#define FRAME_BODY_MAX      (FRAME_HEADER_LEN + 64 * ZONE_RECORD_LEN_EXT + FRAME_CRC_LEN)
#define FRAME_WIRE_MAX      (FRAME_BODY_MAX + FRAME_BODY_MAX / 254 + 2)

/* --- little-endian writers ---------------------------------------------- */
/* Each returns the new offset so calls chain without a manual cursor.
 * Used instead of memcpy'ing a struct: the compiler pads for alignment and
 * a 5-byte record would silently become 6. */
uint16_t put_u8 (uint8_t *buf, uint16_t off, uint8_t  v);
uint16_t put_u16(uint8_t *buf, uint16_t off, uint16_t v);
uint16_t put_u32(uint8_t *buf, uint16_t off, uint32_t v);

/* --- CRC-16/CCITT-FALSE -------------------------------------------------- */
/* Poly 0x1021, init 0xFFFF, no reflection, no final XOR.
 * MUST satisfy: crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1 */
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

/* --- COBS ---------------------------------------------------------------- */
/* dst must hold at least src_len + src_len/254 + 1 bytes.
 * Returns encoded length, excluding any delimiter the caller appends. */
uint16_t cobs_encode(const uint8_t *src, uint16_t src_len, uint8_t *dst);

/* --- frame assembly ------------------------------------------------------ */
/* Writes header + zone records + CRC. Returns body length, or 0 on bad
 * arguments. Does NOT COBS-encode. */
uint16_t frame_build_ranging(uint8_t *buf, uint16_t buf_len,
                             const VL53L7CX_ResultsData *res,
                             uint8_t grid_width,
                             uint32_t seq, uint32_t timestamp_ms);

/* Extended variant: adds signal_per_spad_kcps + ambient_per_spad_kcps per
 * zone (each truncated/clamped to uint16 -- plenty of precision for trend
 * logging, not meant for calibration-grade analysis). Use this for range/
 * SNR characterization tests; use the plain variant for normal streaming
 * since it's ~1.7x the bytes on the wire per frame. */
uint16_t frame_build_ranging_ext(uint8_t *buf, uint16_t buf_len,
                                 const VL53L7CX_ResultsData *res,
                                 uint8_t grid_width,
                                 uint32_t seq, uint32_t timestamp_ms);

/* --- send ---------------------------------------------------------------- */
/* Builds, CRCs, COBS-encodes and transmits one frame in a single UART write.
 * One frame = one message; never split it across calls. */
void frame_send_ranging(const VL53L7CX_ResultsData *res,
                        uint8_t grid_width, uint32_t seq);

/* Extended-record counterpart of frame_send_ranging(). */
void frame_send_ranging_ext(const VL53L7CX_ResultsData *res,
                            uint8_t grid_width, uint32_t seq);

/* Lets log messages share the UART with binary frames. A bare printf would
 * merge into the next frame and corrupt it -- ASCII contains no 0x00. */
void frame_send_text(const char *msg);

/* Self-test: verifies the CRC check vector and a COBS round-trip.
 * Call once at startup, before switching the link to binary. */
bool frame_encoder_selftest(void);

#ifdef __cplusplus
}
#endif

#endif /* FRAME_ENCODER_H */