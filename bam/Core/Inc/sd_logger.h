#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>
#include "comms.h"

#define SD_LOG_FORMAT_VERSION  1U
#define SD_LOG_RATE_HZ         1000U

typedef struct {
  uint32_t timestamp_ms;

  state_vector_t state;
  float32_t acceleration[3];
  float32_t angular_rate[3];
  float32_t magnetometer[3];
  float32_t temperature;
  float32_t pressure;
} sd_log_record_t;

_Static_assert(sizeof(sd_log_record_t) == 136U,
               "Unexpected SD log record size");

/** @brief Open a new FAT32 log once after SPI6 and GPIO initialization. */
void SD_Logger_Init(void);

/**
 * @brief Drain and close the log when a stop has been requested.
 * @note Call from the foreground loop. Set log_stop_requested in the debugger.
 */
void SD_Logger_PollStop(void);

/**
 * @brief Capture a completed sample and write a batch when the queue is ready.
 * @param completed Published RECO sensor and EKF data.
 * @note Call from the foreground loop. SD writes and syncs can block.
 */
void SD_Logger_Record(const reco_message_t *completed);

#endif
