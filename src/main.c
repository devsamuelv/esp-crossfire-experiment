#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "csrf.hpp"
#include "inttypes.h"
#include "math.h"
#include "esp_task.h"
#include "esp_task_wdt.h"
#include "hal/wdt_hal.h"

// Dimensions the buffer that the task being created will use as its stack.
// NOTE:  This is the number of bytes the stack will hold, not the number of
// words as found in vanilla FreeRTOS.
#define STACK_SIZE 4096
#define MAX_FRAME_SIZE 64
#define BAUDRATE 420000
#define RX_STACK_SIZE 64
#define TAG "crossfire-listener"

typedef enum _
{
  SYNC = 0,
  TYPE = 1,
  LENGTH = 2,
  PAYLOAD = 3,
  CRC = 4
} FrameSection_t;

FrameSection_t currentSection = SYNC;

// Structure that will hold the TCB of the task being created.
StaticTask_t xTaskBuffer;

// Buffer that the task being created will use as its stack.  Note this is
// an array of StackType_t variables.  The size of StackType_t is dependent on
// the RTOS port.
StackType_t xStack[STACK_SIZE];

int process(uint8_t *uart_buffer, uint8_t *crsf_buffer)
{
  int frame_length = -1;
  int max_length = round(MAX_FRAME_SIZE / sizeof(uint8_t)) - 1;

  for (int i = 0; i < max_length; i++)
  {
    uint8_t byte = uart_buffer[i];

    // If a sync byte
    if (byte == 0xC8)
    {
      int frame_length = uart_buffer[i + 1] / sizeof(uint8_t);
      bool buffer_contains_frame = (i + frame_length) <= max_length;

      if (buffer_contains_frame)
      {
        int end_frame_index = (i + frame_length) - 1;
        for (int i2 = 0; i2 < end_frame_index; i2++)
        {
          crsf_buffer[i2] = uart_buffer[i + i2];
        }

        frame_length = max_length;
      }
    }
  }

  return frame_length;
};

union channels_s
{
  crsf_channels_s channels_t;
  int channels_int;
};

void crossfire_listener(void *pvParameters)
{
  // The parameter value is expected to be 1 as 1 is passed in the
  // pvParameters value in the call to xTaskCreateStatic().
  configASSERT((uint32_t)pvParameters == 1UL);

  uint8_t *uart_buffer = (uint8_t *)malloc(RX_STACK_SIZE);

  // The plus one was added as a spot for eol char
  uint8_t *crsf_buffer = (uint8_t *)malloc(MAX_FRAME_SIZE + 1);

  const uart_port_t uart_num = UART_NUM_2;
  const uint32_t rx_timeout_ms = 150;
  uart_config_t uart_config = {
      .baud_rate = BAUDRATE,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1};

  // Setup UART buffered IO with event queue
  const int uart_buffer_size = (1024 * 4);
  QueueHandle_t uart_queue;
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,
                                      uart_buffer_size, 1, &uart_queue, 0));

  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

  // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
  ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  wdt_hal_context_t rtc_wdt_ctx = {.inst = WDT_RWDT, .rwdt_dev = &RTCCNTL};

  for (;;)
  {
    const int rx_bytes = uart_read_bytes(uart_num, uart_buffer, RX_STACK_SIZE, rx_timeout_ms / portTICK_PERIOD_MS);

    if (rx_bytes >= 0)
    {
      union channels_s ps;
      rcPacket_t packet;
      int length = process(uart_buffer, crsf_buffer);
      wdt_hal_feed(&rtc_wdt_ctx);

      ESP_LOGI(TAG, "hi");
      ESP_LOGI(TAG, "Read %" PRIu32 " bytes", (uint32_t)rx_bytes);
      ESP_LOG_BUFFER_HEXDUMP(TAG, crsf_buffer, (uint32_t)rx_bytes, ESP_LOG_INFO);

      if (length > 0)
      {
        int max_length = length - 2;
        for (int i = 0; i < max_length; i++)
        {
          ESP_LOGI(TAG, "section[%" PRId8 "]: %" PRId8, i, crsf_buffer[i]);
        }
        ESP_LOGI(TAG, "max: %" PRId8 " length: %" PRId8, max_length, length);
        ESP_LOGI(TAG, "\n");
      }
    }
    else
    {
      ESP_LOGI(TAG, "UART Error");
    }
  }
};

void app_main()
{
  TaskHandle_t handle = NULL;

  xTaskCreate(crossfire_listener, "LISTENER", STACK_SIZE, (void *)1, 1, &handle);
  esp_task_wdt_add(handle);
}