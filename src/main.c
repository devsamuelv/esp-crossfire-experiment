#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "csrf.hpp"

// Dimensions the buffer that the task being created will use as its stack.
// NOTE:  This is the number of bytes the stack will hold, not the number of
// words as found in vanilla FreeRTOS.
#define STACK_SIZE 4096
#define BAUDRATE 400000
#define RX_STACK_SIZE 256
#define TAG "crossfire-listener"

// Structure that will hold the TCB of the task being created.
StaticTask_t xTaskBuffer;

// Buffer that the task being created will use as its stack.  Note this is
// an array of StackType_t variables.  The size of StackType_t is dependent on
// the RTOS port.
StackType_t xStack[STACK_SIZE];

void crossfire_listener(void *pvParameters)
{
  // The parameter value is expected to be 1 as 1 is passed in the
  // pvParameters value in the call to xTaskCreateStatic().
  configASSERT((uint32_t)pvParameters == 1UL);

  uint8_t *buffer = (uint8_t *)malloc(RX_STACK_SIZE);

  const uart_port_t uart_num = UART_NUM_2;
  const uint32_t rx_timeout_ms = 15000;
  uart_config_t uart_config = {
      .baud_rate = BAUDRATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
      .rx_flow_ctrl_thresh = 122,
  };
  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

  // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, 18, 19));

  // Setup UART buffered IO with event queue
  const int uart_buffer_size = (1024 * 2);
  QueueHandle_t uart_queue;
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size,
                                      uart_buffer_size, 10, &uart_queue, 0));

  for (;;)
  {
    const uint32_t rx_bytes = uart_read_bytes(uart_num, buffer, RX_STACK_SIZE, rx_timeout_ms / portTICK_PERIOD_MS);
    const int64_t message_received_us = esp_timer_get_time();

    ESP_LOGD(TAG, "Read %d bytes at %lldus - bytes:", rx_bytes, message_received_us);
    ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, rx_bytes, ESP_LOG_INFO);
  }
};

void app_main()
{
  TaskHandle_t handle = NULL;

  xTaskCreate(crossfire_listener, "LISTENER", STACK_SIZE, (void *)1, 1, &handle);
}