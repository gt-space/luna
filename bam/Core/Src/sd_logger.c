/** @file sd_logger.c
 * @brief Buffered FatFs logging for BAM's completed sensor and EKF records.
 */
#include "sd_logger.h"
#include "fatfs.h"

#include <string.h>

#define SD_LOG_QUEUE_CAPACITY 64U

static sd_log_record_t log_queue[SD_LOG_QUEUE_CAPACITY];
static uint32_t log_head;
static uint32_t log_tail;
static uint32_t log_count;
static volatile uint32_t log_dropped;

/**
 * @brief Copy a record into the RAM queue or count it as dropped.
 * @param record Record to copy.
 */
static void enqueue_sd_log_record(const sd_log_record_t *record)
{
  if (log_count == SD_LOG_QUEUE_CAPACITY) {
    ++log_dropped;
    return;
  }

  log_queue[log_head] = *record;
  log_head = (log_head + 1U) % SD_LOG_QUEUE_CAPACITY;
  ++log_count;
}

/**
 * @brief Capture a timestamp and copy the logged fields from the RECO buffer.
 * @param record Destination record.
 * @param completed Completed sensor and EKF data.
 */
static void fill_sd_log_record(sd_log_record_t *record, const reco_message_t *completed)
{
  record->timestamp_ms = HAL_GetTick();

  memcpy(&record->state, completed, sizeof record->state);
  memcpy(record->acceleration, completed->linAccel, sizeof record->acceleration);
  memcpy(record->angular_rate, completed->angularRate, sizeof record->angular_rate);
  memcpy(record->magnetometer, completed->magData, sizeof record->magnetometer);

  record->temperature = completed->temperature;
  record->pressure = completed->pressure;
}

#define SD_LOG_BATCH_RECORDS 16U
_Static_assert(SD_LOG_QUEUE_CAPACITY % SD_LOG_BATCH_RECORDS == 0U,
               "Log batches must fit evenly in the queue");

static volatile FRESULT log_result = FR_NOT_READY;
static bool log_open;
static volatile bool log_stop_requested;
static uint32_t batches_since_sync;

/**
 * @brief Register the disk driver, mount FAT32, and create the log file.
 * @return FatFs status. An existing log is preserved.
 */
static FRESULT open_sd_log(void)
{
  MX_FATFS_Init();

  if (retUSER != 0U) {
    return FR_NOT_ENABLED;
  }

  FRESULT result = f_mount(&USERFatFS, USERPath, 1);
  if (result != FR_OK) {
    return result;
  }

  if (USERFatFS.fs_type != FS_FAT32) {
    return FR_NO_FILESYSTEM;
  }

  return f_open(&USERFile, "0:/BAMLOG.BIN", FA_WRITE | FA_CREATE_NEW);
}

/**
 * @brief Write a queued batch and advance the queue after a complete write.
 * @param flush Allow a partial batch while stopping.
 * @return FatFs status, or FR_DENIED for a short write.
 */
static FRESULT write_sd_log_batch(bool flush)
{
  if (log_count == 0U || (!flush && log_count < SD_LOG_BATCH_RECORDS))
  {
    return FR_OK;
  }

  /* The tail advances in full batches until the final write. */
  const uint32_t records = log_count < SD_LOG_BATCH_RECORDS
      ? log_count : SD_LOG_BATCH_RECORDS;
  const UINT bytes = records * sizeof log_queue[0];
  UINT written = 0U;

  FRESULT result = f_write(&USERFile, &log_queue[log_tail], bytes, &written);

  if (result != FR_OK)
  {
    return result;
  }

  if (written != bytes)
  {
    return FR_DENIED;
  }

  log_tail = (log_tail + records) % SD_LOG_QUEUE_CAPACITY;
  log_count -= records;

  if (!flush && ++batches_since_sync == 8U)
  {
    batches_since_sync = 0U;
    return f_sync(&USERFile);
  }

  return FR_OK;
}

/** @brief Drain pending records, close the file, and preserve the first error. */
static void stop_sd_log(void)
{
  if (!log_open)
  {
    return;
  }

  while (log_count > 0U && log_result == FR_OK)
  {
    log_result = write_sd_log_batch(true);
  }

  FRESULT result = f_close(&USERFile);
  if (log_result == FR_OK)
  {
    log_result = result;
  }
  log_open = false;
}

void SD_Logger_Init(void)
{
  log_result = open_sd_log();
  log_open = (log_result == FR_OK);
}

void SD_Logger_PollStop(void)
{
  if (log_stop_requested)
  {
    log_stop_requested = false;
    stop_sd_log();
  }
}

void SD_Logger_Record(const reco_message_t *completed)
{
  if (log_open && log_result == FR_OK)
  {
    sd_log_record_t record;
    fill_sd_log_record(&record, completed);
    enqueue_sd_log_record(&record);
    log_result = write_sd_log_batch(false);
  }
}
