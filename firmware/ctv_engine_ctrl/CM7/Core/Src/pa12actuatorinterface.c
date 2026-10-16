#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// ==========================================
// 1. CONFIGURATION & CONSTANTS
// ==========================================
#define PA12_HEADER        0xFF
#define PA12_MAX_PACKET    16

#define STROKE_MIN_MM  0.0f
#define STROKE_MAX_MM  50.0f
#define COUNTS_PER_MM  81.92f

// ==========================================
// 2. DATA STRUCTURES
// ==========================================
typedef struct {
    uint8_t id;
    uint8_t instruction;
    uint8_t params[8];
    uint8_t param_len;
} PA12Command;

typedef struct {
    uint8_t id;
    uint8_t status;
    uint8_t data[8];
    uint8_t data_len;
    bool valid;
} PA12Response;

typedef struct {
    // Hardware abstraction function pointers
    void (*set_dir_pin)(bool high); // High = TX, Low = RX
    void (*uart_send)(const uint8_t *data, uint16_t len);
    bool (*is_tx_complete)(void);   // Checks UART TC flag
    uint32_t (*get_tick_ms)(void);
    
    // State tracking
    bool is_transmitting;
    uint32_t deadline_ms;
} PA12DriverChannel;

typedef struct {
    PA12DriverChannel channel;
    bool safing_latched;
    float current_position_mm;
    uint8_t fault_flags;
} PA12Actuator;

// ==========================================
// 3. CODEC FUNCTIONS (Packet Handling)
// ==========================================
static uint8_t pa12_calculate_checksum(const uint8_t *buf, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return (uint8_t)(~sum);
}

int pa12_encode_packet(const PA12Command *cmd, uint8_t *out_buf, uint8_t max_len) {
    uint8_t total_len = 5 + cmd->param_len;
    if (total_len > max_len) return -1;

    out_buf[0] = PA12_HEADER;
    out_buf[1] = PA12_HEADER;
    out_buf[2] = cmd->id;
    out_buf[3] = cmd->param_len + 2;
    out_buf[4] = cmd->instruction;
    
    for (uint8_t i = 0; i < cmd->param_len; i++) {
        out_buf[5 + i] = cmd->params[i];
    }
    
    out_buf[total_len - 1] = pa12_calculate_checksum(out_buf, total_len - 1);
    return total_len;
}

bool pa12_parse_response(const uint8_t *buf, uint8_t len, PA12Response *resp) {
    if (len < 6) return false;
    if (buf[0] != PA12_HEADER || buf[1] != PA12_HEADER) return false;

    uint8_t calc_chk = pa12_calculate_checksum(buf, len - 1);
    if (calc_chk != buf[len - 1]) return false;

    resp->id = buf[2];
    resp->status = buf[4];
    resp->data_len = buf[3] - 2;
    
    if (resp->data_len > 8 || (5 + resp->data_len) > len) return false;

    memcpy(resp->data, &buf[5], resp->data_len);
    resp->valid = true;
    return true;
}

// ==========================================
// 4. LOW-LEVEL BUS DRIVER FUNCTIONS
// ==========================================
void pa12_init_channel(PA12DriverChannel *channel, void (*dir_fn)(bool), void (*send_fn)(const uint8_t*, uint16_t), bool (*tc_fn)(void), uint32_t (*tick_fn)(void)) {
    channel->set_dir_pin = dir_fn;
    channel->uart_send = send_fn;
    channel->is_tx_complete = tc_fn;
    channel->get_tick_ms = tick_fn;
    channel->is_transmitting = false;
    channel->set_dir_pin(false); // Start in RX mode (DIR Low)
}

bool pa12_begin_transmission(PA12DriverChannel *channel, const uint8_t *data, uint16_t len, uint32_t timeout_ms) {
    if (channel->is_transmitting) return false;

    // Switch direction to TX (DIR High)
    channel->set_dir_pin(true);
    
    channel->is_transmitting = true;
    channel->uart_send(data, len);
    
    // Wait for hardware Transmission Complete (TC)
    while (!channel->is_tx_complete()) {
        // Add safeguard timeout here if needed
    }
    
    // Return to RX mode (DIR Low)
    channel->set_dir_pin(false);
    channel->deadline_ms = channel->get_tick_ms() + timeout_ms;
    
    return true;
}

// ==========================================
// 5. HIGH-LEVEL CONTROLLER & API
// ==========================================
void pa12_apply_safing(PA12Actuator *act) {
    act->safing_latched = true;
    // Flush queued motion goals here
}

void pa12_clear_safing_latch(PA12Actuator *act) {
    act->safing_latched = false;
}

bool pa12_set_position_mm(PA12Actuator *act, float target_mm) {
    if (isnan(target_mm) || isinf(target_mm)) return false;
    if (act->safing_latched) return false;

    if (target_mm < STROKE_MIN_MM) target_mm = STROKE_MIN_MM;
    if (target_mm > STROKE_MAX_MM) target_mm = STROKE_MAX_MM;

    uint16_t protocol_counts = (uint16_t)(target_mm * COUNTS_PER_MM);
    
    // Build and send packet via channel...
    (void)protocol_counts;
    return true;
}