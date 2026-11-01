#include "ZED-F9P-02B.h"
#include <string.h>  // for memcpy
#include "Inc/stm32h5xx_hal.h"  // for HAL_I2C_Mem_Read, HAL_I2C_Master_Transmit
/****
 *
 * I2C Interface via Fast-Mode
 * 
 * Slave Mode maximum bit rate = 400 kbps
 *  > Hold SCL LOW to pause transaction [max 20 ms]
 *  > 300 ns transition time required from SCL falling edge 
 * 
 * UBX-MON-COMMS PortId - I2C = 0x0000 
 * 
 */


/* 

UBX Protocol Constants 

UBX Format: [Sync Byte 1] [Sync Byte 2] [Class] [ID] [payload length]
                  [actual payload] [Checksum A] [Checksum B]
                  "Fletcher -8 Algorithm" // rnalamalapu3

little endian paylaod length

*/
#define UBX_SYNC1            0xB5u
#define UBX_SYNC2            0x62u
#define UBX_CLASS_NAV        0x01u
#define UBX_ID_NAV_PVT       0x07u    // Parse this one for info x/ v/ t
#define UBX_CLASS_CFG        0x06u
#define UBX_ID_CFG_RATE      0x08u
#define UBX_ID_CFG_MSG       0x01u
#define NAV_PVT_PAYLOAD_LEN  92u

// *note 3    rnalamalapu3

/* I2C Register Map Source: UBX-18010802 Section 3.8.2.1 */
#define GPS_REG_LEN_MSB       0xFDu
#define GPS_REG_STREAM        0xFFu
// *note 2    rnalamalapu3

/*GPS Timing constants and limitations*/
#define GPS_I2C_TIMEOUT_MS    3u    // Data timeout
#define GPS_I2C_READ_CHUCK    48u   // Pulled over I2C while draining
#define GPS_MEAS_PERIOD_MS    50u
// *note 2    rnalamalapu3


enum {
    PS_SYNC1 = 0, PS_SYNC2, PS_CLASS, PS_ID, PS_LEN1, PS_LEN2,
    PS_PAYLOAD, PS_CKA, PS_CKB,
};
 
static struct {
    uint8_t  state;         // Which field
    uint8_t  cls;           //class byte
    uint8_t  id;            // id of the byte
    uint16_t len;           // length of payload
    uint16_t idx;           // index into payload buffer
    uint8_t  ck_a;          // checksum byte A
    uint8_t  ck_b;          // checksum byte b
    bool     want_payload;  // true if NAV-PVT is desired
    uint8_t  payload[NAV_PVT_PAYLOAD_LEN];      // Buffer that fills only if we want payload
} s_parser;
 

// Read from GPS at (0xFD/0xFE) to hear if N bytes of UBX is waiting
//Tellsyou how many bytes are waiting to be read from the GPS module.
// The variables are the actual reg values 0xFD/0xFE
static GPS_Status i2c_available(I2C_HandleTypeDef *hi2c, uint16_t *available)
{
    uint8_t buf[2];
    // Check if successful read
    if (HAL_I2C_Mem_Read(hi2c, GPS_I2C_ADDR, GPS_REG_LEN_MSB, I2C_MEMADD_SIZE_8BIT,
                          buf, sizeof(buf), GPS_I2C_TIMEOUT_MS) != HAL_OK) {
        return GPS_I2C_ERROR;
    }
    *available = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1]; /* MSB at 0xFD */
    return GPS_OK;
}

static GPS_Status i2c_read_stream(I2C_HandleTypeDef *hi2c, uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Read(s_hi2c, GPS_I2C_ADDR, GPS_REG_STREAM, I2C_MEMADD_SIZE_8BIT,
                              buf, len, GPS_I2C_TIMEOUT_MS) == HAL_OK) ? GPS_OK : GPS_I2C_ERROR;
}
 
static GPS_Status i2c_write_raw(const uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Master_Transmit(s_hi2c, GPS_I2C_ADDR, (uint8_t *)buf, len,
                                     GPS_I2C_TIMEOUT_MS) == HAL_OK) ? GPS_OK : GPS_I2C_ERROR;
}

/////// Review

/* ---- UBX message building -------------------------------------------------- */

// The UBX checksum (8-bit Fletcher) over `data[0..len)`. Computed over
// class, id, length, and payload -- NOT over the sync bytes.
// ck_a accumulates a running sum; ck_b accumulates a running sum-of-sums.
// This exact two-accumulator trick is what the receiver does too, so it
// can tell if any byte got corrupted in transit.
static void ubx_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (uint16_t i = 0; i < len; i++) {
        a = (uint8_t)(a + data[i]);  // uint8_t wraps automatically -- that's the "mod 256" the algorithm wants
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

// Builds a complete UBX frame (sync + class + id + len + payload + checksum)
// and sends it. Used for every outgoing message -- both our CFG commands.
static GPS_Status ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t payload_len)
{
    uint8_t pkt[6 + 16 + 2]; /* header(4)+len(2)+payload(<=16)+checksum(2); plenty for CFG-RATE/CFG-MSG */
    if (payload_len > 16u) {
        return GPS_I2C_ERROR;
    }
    pkt[0] = UBX_SYNC1;
    pkt[1] = UBX_SYNC2;
    pkt[2] = cls;
    pkt[3] = id;
    pkt[4] = (uint8_t)(payload_len & 0xFFu);        // length, low byte
    pkt[5] = (uint8_t)((payload_len >> 8) & 0xFFu);  // length, high byte
    if (payload_len > 0u && payload != NULL) {
        memcpy(&pkt[6], payload, payload_len);
    }

    uint8_t ck_a, ck_b;
    ubx_checksum(&pkt[2], (uint16_t)(4u + payload_len), &ck_a, &ck_b); // checksum covers class..payload, not sync bytes
    pkt[6 + payload_len]     = ck_a;
    pkt[6 + payload_len + 1] = ck_b;

    return i2c_write_raw(pkt, (uint16_t)(6u + payload_len + 2u));
}

/* ---- NAV-PVT decode ---------------------------------------------------------- */

// Little-endian 4-byte -> signed 32-bit reconstruction. UBX fields are
// always little-endian, so byte[0] is the least-significant byte.
static int32_t rd_i32(const uint8_t *p)
{
    uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (int32_t)v;
}

// Given a complete, checksum-validated NAV-PVT payload, pull out the
// fields we actually care about and write them into the caller's struct.
static GPS_Status decode_nav_pvt(const uint8_t *p, gps_data_t *out)
{
    /* Offsets per the standard UBX-NAV-PVT (92-byte) layout: 20 fixType,
     * 21 flags (bit0 = gnssFixOK), 24 lon (1e-7 deg), 28 lat (1e-7 deg),
     * 32 height above ellipsoid (mm), 36 height above MSL (mm). */
    out->fix_type = p[20];
    out->fix_ok   = (p[21] & 0x01u) != 0u;   // bit 0 of the flags byte = "is this fix good"

    if (!out->fix_ok) {
        out->GPS_DRDY = false;
        return GPS_NO_FIX; /* lat/lon/altitude intentionally left untouched -- don't trust a bad fix */
    }

    int32_t lon_raw = rd_i32(&p[24]);
    int32_t lat_raw = rd_i32(&p[28]);
    int32_t h_ell    = rd_i32(&p[32]);   /* using ellipsoid height; switch to
                                           * offset 36 for height-above-MSL if
                                           * that's what the rest of your
                                           * nav stack expects */

    // Raw values are integers scaled by the module (so no floating point
    // drifts into the wire format). lon/lat come in units of 1e-7 degree,
    // height comes in millimeters -- we convert to familiar units here.
    out->longitude = (double)lon_raw * 1e-7;
    out->latitude  = (double)lat_raw * 1e-7;
    out->altitude  = (double)h_ell * 1e-3;
    out->GPS_DRDY  = true;
    return GPS_OK;
}

/* ---- Streaming UBX parser ----------------------------------------------------
 * Byte-at-a-time state machine so messages can safely straddle separate I2C
 * reads. Returns true the moment a complete, checksum-valid message has been
 * handed off (via decoded_status), regardless of fix validity. */

static void parser_reset(void) { s_parser.state = PS_SYNC1; }

// Feed ONE byte into the parser. Think of this as a little state machine
// walking through the message shape described at the top of my reply:
// sync1 -> sync2 -> class -> id -> len(lo) -> len(hi) -> payload... -> ck_a -> ck_b
// Every state checks/accumulates the checksum as it goes, so by the time
// we hit ck_a/ck_b we can compare against what we computed ourselves.
static bool parser_feed_byte(uint8_t b, gps_data_t *out, GPS_Status *decoded_status)
{
    switch (s_parser.state) {
    case PS_SYNC1:
        // Sitting here until we see 0xB5. Anything else, keep waiting.
        s_parser.state = (b == UBX_SYNC1) ? PS_SYNC2 : PS_SYNC1;
        break;

    case PS_SYNC2:
        // Need 0x62 right after 0xB5, or it wasn't really a sync -- start over.
        s_parser.state = (b == UBX_SYNC2) ? PS_CLASS : PS_SYNC1;
        break;

    case PS_CLASS:
        // First byte of what the checksum covers, so checksum starts here.
        s_parser.cls  = b;
        s_parser.ck_a = b;
        s_parser.ck_b = b;
        s_parser.state = PS_ID;
        break;

    case PS_ID:
        s_parser.id = b;
        s_parser.ck_a = (uint8_t)(s_parser.ck_a + b);
        s_parser.ck_b = (uint8_t)(s_parser.ck_b + s_parser.ck_a);
        s_parser.state = PS_LEN1;
        break;

    case PS_LEN1:
        s_parser.len  = b;                 // length, low byte
        s_parser.ck_a = (uint8_t)(s_parser.ck_a + b);
        s_parser.ck_b = (uint8_t)(s_parser.ck_b + s_parser.ck_a);
        s_parser.state = PS_LEN2;
        break;

    case PS_LEN2:
        s_parser.len |= ((uint16_t)b << 8); // length, high byte -- len is now complete
        s_parser.ck_a = (uint8_t)(s_parser.ck_a + b);
        s_parser.ck_b = (uint8_t)(s_parser.ck_b + s_parser.ck_a);
        s_parser.idx = 0;
        // Only bother buffering the payload if this is actually a NAV-PVT
        // of the expected size -- anything else we just count past.
        s_parser.want_payload = (s_parser.cls == UBX_CLASS_NAV &&
                                  s_parser.id == UBX_ID_NAV_PVT &&
                                  s_parser.len == NAV_PVT_PAYLOAD_LEN);
        s_parser.state = (s_parser.len == 0u) ? PS_CKA : PS_PAYLOAD; // zero-length payload? skip straight to checksum
        break;

    case PS_PAYLOAD:
        s_parser.ck_a = (uint8_t)(s_parser.ck_a + b);
        s_parser.ck_b = (uint8_t)(s_parser.ck_b + s_parser.ck_a);
        if (s_parser.want_payload && s_parser.idx < sizeof(s_parser.payload)) {
            s_parser.payload[s_parser.idx] = b;  // only NAV-PVT bytes actually get stored
        }
        s_parser.idx++;
        if (s_parser.idx >= s_parser.len) {
            s_parser.state = PS_CKA;  // consumed the whole payload, move to checksum
        }
        break;

    case PS_CKA:
        if (b != s_parser.ck_a) {
            parser_reset(); // our computed checksum doesn't match -- this wasn't a real message, resync
            break;
        }
        s_parser.state = PS_CKB;
        break;

    case PS_CKB: {
        bool checksum_ok = (b == s_parser.ck_b);
        bool want_payload = s_parser.want_payload;
        parser_reset(); // either way, we're done with this message -- go look for the next sync bytes
        if (checksum_ok && want_payload) {
            *decoded_status = decode_nav_pvt(s_parser.payload, out);
            return true; // tell the caller: a full NAV-PVT was just decoded
        }
        break;
    }

    default:
        parser_reset();
        break;
    }
    return false; // still mid-message, or this wasn't a message we care about
}

/* ---- Public API --------------------------------------------------------------- */

// Call once at startup. Points the driver at your I2C handle, then tells
// the module how often to compute a fix (CFG-RATE) and to automatically
// push NAV-PVT out the I2C port every time it does (CFG-MSG) -- so after
// this, the module pushes data on its own; we don't have to keep asking.
// 
// Need to fix.... rnalamalapu3  USING DEPRECATED MESSAGE TYPE
//                               SENDS BOTH WITHOUT ACKNOWLEDGEMENT
GPS_Status GPS_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    // s_i2c_error_count = 0;
    parser_reset();

    /* UBX-CFG-RATE: measRate(u2) navRate(u2)=1 timeRef(u2)=0 (UTC) */
    uint8_t rate_payload[6] = {
        (uint8_t)(GPS_MEAS_PERIOD_MS & 0xFFu), (uint8_t)((GPS_MEAS_PERIOD_MS >> 8) & 0xFFu),
        0x01u, 0x00u,   // navRate = 1 (compute a nav solution every measurement)
        0x00u, 0x00u,   // timeRef = 0 (UTC)
    };
    GPS_Status st_rate = ubx_send(UBX_CLASS_CFG, UBX_ID_CFG_RATE, rate_payload, sizeof(rate_payload));

    /* UBX-CFG-MSG (all ports form): enable NAV-PVT on the I2C (DDC) port,
     * once per navigation solution. rate[] order: DDC, UART1, UART2, USB, SPI, reserved */
    uint8_t msg_payload[8] = { UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1u, 0u, 0u, 0u, 0u, 0u };
    GPS_Status st_msg = ubx_send(UBX_CLASS_CFG, UBX_ID_CFG_MSG, msg_payload, sizeof(msg_payload));

    return (st_rate == GPS_OK && st_msg == GPS_OK) ? GPS_OK : GPS_I2C_ERROR;
}

// The function your 200 Hz loop calls every tick. Checks how many bytes
// are waiting, drains them all through the parser, and reports what (if
// anything) was found. See the "available == 0" fast path below -- that's
// what keeps this cheap on the ~9 out of 10 calls where nothing's new.
GPS_Status extract_gps_data(gps_data_t *gps_output)
{
    gps_output->GPS_DRDY = false;

    uint16_t available = 0;
    if (i2c_available(&available) != GPS_OK) {
        //s_i2c_error_count++;
        return GPS_I2C_ERROR;
    }
    if (available == 0u) {
        return GPS_NO_NEW_DATA; /* the common case at 200 Hz -- cheap and fast */
    }

    GPS_Status result = GPS_NO_NEW_DATA;
    uint8_t chunk[GPS_I2C_READ_CHUNK];

    // Drain everything that's waiting, GPS_I2C_READ_CHUNK bytes at a time,
    // feeding every byte through the parser one at a time.
    while (available > 0u) {
        uint16_t n = (available > GPS_I2C_READ_CHUNK) ? GPS_I2C_READ_CHUNK : available;
        if (i2c_read_stream(chunk, n) != GPS_OK) {
            // s_i2c_error_count++;
            /* Keep whatever we already decoded this call rather than
             * discarding a good fix because a later chunk read failed. */
            return (result == GPS_NO_NEW_DATA) ? GPS_I2C_ERROR : result;
        }
        for (uint16_t i = 0; i < n; i++) {
            GPS_Status decoded;
            if (parser_feed_byte(chunk[i], gps_output, &decoded)) {
                result = decoded; // remember the most recent full message found this call
            }
        }
        available = (uint16_t)(available - n);
    }
    return result;
}

