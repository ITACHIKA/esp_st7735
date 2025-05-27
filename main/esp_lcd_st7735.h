#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"

typedef uint16_t st7735_color_t;

typedef struct
{
    spi_host_device_t spi_host_device;
    spi_device_handle_t spi_device_handle;
    gpio_num_t pin_dc;
    gpio_num_t pin_rst;
    gpio_num_t pin_bl;
    uint16_t width;
    uint16_t height;
    size_t max_io_bytes;                   // max transfer bytes in one transaction
    QueueHandle_t msgQueue;                // Queue for async spi
} st7735_handle_t;

typedef enum
{
    MSG_FLUSH_SEND,
    MSG_FLUSH_COMPLETE
} async_spi_msg_type_t;

typedef struct
{
    async_spi_msg_type_t msg;
    spi_transaction_t* transaction;
} async_spi_queue_msg_t;

typedef struct
{
    st7735_handle_t *st7735_dev;
    QueueHandle_t spi_data_queue;
    lv_display_t *lvgl_disp;
} async_recv_params;


esp_err_t st7735_init(spi_host_device_t hostid, spi_bus_config_t *buscfg, spi_device_interface_config_t *devcfg, st7735_handle_t *dev);

esp_err_t st7735_send_cmd(st7735_handle_t *dev, uint8_t cmd);

esp_err_t st7735_send_data(st7735_handle_t *dev, const uint8_t *data, size_t data_length);

esp_err_t st7735_send_data_async(st7735_handle_t *dev, const uint8_t *data, size_t data_length);

esp_err_t st7735_send_byte(st7735_handle_t *dev, uint8_t data, size_t data_length);

BaseType_t start_async_data_recv(st7735_handle_t *dev, QueueHandle_t spi_data_queue, lv_display_t *disp);

esp_err_t st7735_reset(st7735_handle_t *dev);

esp_err_t st7735_set_window(st7735_handle_t *dev, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

esp_err_t st7735_write_pixels(st7735_handle_t *dev, const st7735_color_t *data, size_t length);

esp_err_t st7735_fill_screen(st7735_handle_t *dev, st7735_color_t color, uint8_t *line_buf);

esp_err_t st7735_set_backlight(st7735_handle_t *dev, bool on);

void st7735_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p);