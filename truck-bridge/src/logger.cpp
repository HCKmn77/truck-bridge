#include "logger.h"
#include "config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// Log message structure
typedef struct {
  LogLevel level;
  unsigned long timestamp;
  char task_name[16];
  char message[128];
} LogMessage;

// Queue handle
static QueueHandle_t log_queue = NULL;
static SemaphoreHandle_t serial_mutex = NULL;
static LogLevel min_log_level = LOG_LEVEL_DEBUG;

// Log level strings
static const char *level_strings[] = {
  "DEBUG",
  "INFO",
  "WARN",
  "ERROR"
};

void logger_init(LogLevel min_level) {
  min_log_level = min_level;
  
  // Create queue: up to 32 log messages
  log_queue = xQueueCreate(32, sizeof(LogMessage));
  if (log_queue == NULL) {
    DEBUG_SERIAL.println("[LOGGER] ERROR: Failed to create log queue!");
    return;
  }
  
  // Create mutex for serial port protection
  //TODO: Doublecheck if it is still needed because of wrapper-log-msg
  serial_mutex = xSemaphoreCreateMutex();
  if (serial_mutex == NULL) {
    DEBUG_SERIAL.println("[LOGGER] ERROR: Failed to create serial mutex!");
    return;
  }
}

BaseType_t logger_queue_message(
  LogLevel level,
  const char *task_name,
  const char *format,
  ...
) {
  if (log_queue == NULL) {
    return pdFALSE;
  }

  // Skip if below minimum log level
  if (level < min_log_level) {
    return pdTRUE;
  }

  LogMessage msg;
  msg.level = level;
  msg.timestamp = millis();
  
  // Copy task name (truncate if too long)
  strncpy(msg.task_name, task_name, sizeof(msg.task_name) - 1);
  msg.task_name[sizeof(msg.task_name) - 1] = '\0';
  
  // Format message
  va_list args;
  va_start(args, format);
  vsnprintf(msg.message, sizeof(msg.message), format, args);
  va_end(args);

  // Queue message with 10ms timeout (non-blocking)
  return xQueueSend(log_queue, &msg, pdMS_TO_TICKS(10));
}

void logger_task(void *pvParameters) {
  // Wait a moment for serial to stabilize
  vTaskDelay(pdMS_TO_TICKS(100));
  
  DEBUG_SERIAL.println("[LOGGER] Starting LOGGER-TASK...");
  
  LogMessage msg;
  
  while (1) {
    // Wait for message in queue (100ms timeout)
    if (xQueueReceive(log_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Acquire mutex before writing to serial
      if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Format and print: [timestamp] [LEVEL] [TaskName] message
        
        DEBUG_SERIAL.print("[");
        DEBUG_SERIAL.print(msg.timestamp);
        DEBUG_SERIAL.print("] [");
        DEBUG_SERIAL.print(level_strings[msg.level]);
        DEBUG_SERIAL.print("] [");
        DEBUG_SERIAL.print(msg.task_name);
        DEBUG_SERIAL.print("] ");
        DEBUG_SERIAL.println(msg.message);
        
        // Release mutex
        xSemaphoreGive(serial_mutex);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
