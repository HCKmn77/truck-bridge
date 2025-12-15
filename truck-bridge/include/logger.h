#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// Log levels
typedef enum {
  LOG_OFF = -1,
  LOG_LEVEL_DEBUG = 0,
  LOG_LEVEL_INFO = 1,
  LOG_LEVEL_WARN = 2,
  LOG_LEVEL_ERROR = 3
} LogLevel;

// Initialize logger queue and set minimum log level
void logger_init(LogLevel min_level);

// Queue a log message (task-safe, non-blocking with timeout)
// Returns pdTRUE if queued successfully, pdFALSE if queue full
BaseType_t logger_queue_message(
  LogLevel level,
  const char *task_name,
  const char *format,
  ...
);

// Logger task entry point (call via xTaskCreatePinnedToCore)
void logger_task(void *pvParameters);

// Shorthand macros for different log levels
#define LOG_DEBUG(task, fmt, ...) \
  logger_queue_message(LOG_LEVEL_DEBUG, task, fmt, ##__VA_ARGS__)

#define LOG_INFO(task, fmt, ...) \
  logger_queue_message(LOG_LEVEL_INFO, task, fmt, ##__VA_ARGS__)

#define LOG_WARN(task, fmt, ...) \
  logger_queue_message(LOG_LEVEL_WARN, task, fmt, ##__VA_ARGS__)

#define LOG_ERROR(task, fmt, ...) \
  logger_queue_message(LOG_LEVEL_ERROR, task, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
