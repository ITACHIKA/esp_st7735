#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_st7735.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "lvgl.h"
#include "esp_lcd_7735_config.h"


esp_err_t st7735_init(spi_host_device_t hostid, spi_bus_config_t *spibuscfg, spi_device_interface_config_t *spidevcfg, st7735_handle_t *lcddev)
{
    ESP_ERROR_CHECK(spi_bus_initialize(hostid, spibuscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(hostid, spidevcfg, &lcddev->spi_device_handle));
    gpio_set_direction(lcddev->pin_dc, GPIO_MODE_OUTPUT);
    gpio_set_direction(lcddev->pin_rst, GPIO_MODE_OUTPUT);
    gpio_set_direction(lcddev->pin_bl, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(spi_bus_get_max_transaction_len(hostid, &lcddev->max_io_bytes));
    return ESP_OK;
}

esp_err_t st7735_reset(st7735_handle_t *dev)
{
    gpio_set_level(dev->pin_rst, 0); // hard reset
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(dev->pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    st7735_send_cmd(dev, 0x01); // Software Reset
    vTaskDelay(pdMS_TO_TICKS(150));

    st7735_send_cmd(dev, 0x11); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(150));

    st7735_send_cmd(dev, 0xB1);     // Frame Rate Control (normal mode)
    st7735_send_byte(dev, 0x01, 8); // RTNA: Division ratio (0x01 = fosc/1)
    st7735_send_byte(dev, 0x2C, 8); // Front porch
    st7735_send_byte(dev, 0x2D, 8); // Back porch

    st7735_send_cmd(dev, 0xB2);     // Frame Rate Control (idle mode)
    st7735_send_byte(dev, 0x01, 8); // RTNA
    st7735_send_byte(dev, 0x2C, 8); // Front porch
    st7735_send_byte(dev, 0x2D, 8); // Back porch

    st7735_send_cmd(dev, 0xB3);     // Frame Rate Control (partial mode/full colors)
    st7735_send_byte(dev, 0x01, 8); // RTNA
    st7735_send_byte(dev, 0x2C, 8);
    st7735_send_byte(dev, 0x2D, 8);
    st7735_send_byte(dev, 0x01, 8); // RTNB (for idle mode)
    st7735_send_byte(dev, 0x2C, 8);
    st7735_send_byte(dev, 0x2D, 8);

    st7735_send_cmd(dev, 0xB4);     // Display Inversion Control
    st7735_send_byte(dev, 0x07, 8); // Normal display, line inversion

    st7735_send_cmd(dev, 0xC0);     // Power Control 1
    st7735_send_byte(dev, 0xA2, 8); // AVDD: 4.6V, GVDD: 4.6V
    st7735_send_byte(dev, 0x02, 8); // VGH = AVDD * 2
    st7735_send_byte(dev, 0x84, 8); // VGL = -AVDD

    st7735_send_cmd(dev, 0xC1);     // Power Control 2
    st7735_send_byte(dev, 0xC5, 8); // VGH/VGL voltage ratio

    st7735_send_cmd(dev, 0xC2);     // Power Control 3
    st7735_send_byte(dev, 0x0A, 8); // Opamp current small
    st7735_send_byte(dev, 0x00, 8); // Boost frequency

    st7735_send_cmd(dev, 0xC3);     // Power Control 4 (for normal mode)
    st7735_send_byte(dev, 0x8A, 8); // VCOMH voltage
    st7735_send_byte(dev, 0x2A, 8); // VCOML voltage

    st7735_send_cmd(dev, 0xC4); // Power Control 5 (for idle mode)
    st7735_send_byte(dev, 0x8A, 8);
    st7735_send_byte(dev, 0xEE, 8);

    st7735_send_cmd(dev, 0xC5);     // VCOM Control
    st7735_send_byte(dev, 0x0E, 8); // VCOM = VCOMH - VCOML

    st7735_send_cmd(dev, 0x36);     // Memory Access Control
    st7735_send_byte(dev, 0xC8, 8); // MX, MY, RGB: row/col exchange + BGR

    st7735_send_cmd(dev, 0x3A);     // Interface Pixel Format
    st7735_send_byte(dev, 0x05, 8); // 16 bits/pixel (RGB565)

    st7735_send_cmd(dev, 0xE0); // Positive Gamma Correction
    st7735_send_byte(dev, 0x0F, 8);
    st7735_send_byte(dev, 0x1A, 8);
    st7735_send_byte(dev, 0x0F, 8);
    st7735_send_byte(dev, 0x18, 8);
    st7735_send_byte(dev, 0x2F, 8);
    st7735_send_byte(dev, 0x28, 8);
    st7735_send_byte(dev, 0x20, 8);
    st7735_send_byte(dev, 0x22, 8);
    st7735_send_byte(dev, 0x1F, 8);
    st7735_send_byte(dev, 0x1B, 8);
    st7735_send_byte(dev, 0x23, 8);
    st7735_send_byte(dev, 0x37, 8);
    st7735_send_byte(dev, 0x00, 8);
    st7735_send_byte(dev, 0x07, 8);
    st7735_send_byte(dev, 0x02, 8);
    st7735_send_byte(dev, 0x10, 8);

    st7735_send_cmd(dev, 0xE1); // Negative Gamma Correction
    st7735_send_byte(dev, 0x0F, 8);
    st7735_send_byte(dev, 0x1B, 8);
    st7735_send_byte(dev, 0x0F, 8);
    st7735_send_byte(dev, 0x17, 8);
    st7735_send_byte(dev, 0x33, 8);
    st7735_send_byte(dev, 0x2C, 8);
    st7735_send_byte(dev, 0x29, 8);
    st7735_send_byte(dev, 0x2E, 8);
    st7735_send_byte(dev, 0x30, 8);
    st7735_send_byte(dev, 0x30, 8);
    st7735_send_byte(dev, 0x39, 8);
    st7735_send_byte(dev, 0x3F, 8);
    st7735_send_byte(dev, 0x00, 8);
    st7735_send_byte(dev, 0x07, 8);
    st7735_send_byte(dev, 0x03, 8);
    st7735_send_byte(dev, 0x10, 8);

    st7735_send_cmd(dev, 0x13); // Normal Display Mode On
    vTaskDelay(pdMS_TO_TICKS(10));

    st7735_send_cmd(dev, 0x29); // Display ON
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

esp_err_t st7735_send_cmd(st7735_handle_t *dev, uint8_t cmd)
{
    gpio_set_level(dev->pin_dc, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    ESP_ERROR_CHECK(spi_device_transmit(dev->spi_device_handle, &t));
    return ESP_OK;
}

/*
send st7735 data, data_length in bytes - sync mode
*/
esp_err_t st7735_send_data(st7735_handle_t *dev, const uint8_t *data, size_t data_length)
{
    gpio_set_level(dev->pin_dc, 1);
    size_t max_io_bytes = dev->max_io_bytes;
    size_t sent = 0;
    while (sent < data_length)
    {
        spi_transaction_t t = {0};
        if (data_length - sent <= max_io_bytes)
        {
            t.length = (data_length - sent) * 8;
            t.tx_buffer = data + sent;
            sent = data_length;
            ESP_ERROR_CHECK(spi_device_transmit(dev->spi_device_handle, &t));
        }
        else
        {
            t.length = max_io_bytes * 8;
            t.tx_buffer = data + sent;
            sent += max_io_bytes;
            ESP_ERROR_CHECK(spi_device_transmit(dev->spi_device_handle, &t));
        }
    }
    return ESP_OK;
}

/*
send st7735 data, data_length in bytes - async mode - must work with async receiver
*/
esp_err_t st7735_send_data_async(st7735_handle_t *dev, const uint8_t *data, size_t data_length)
{
    gpio_set_level(dev->pin_dc, 1);
    size_t max_io_bytes = dev->max_io_bytes;
    size_t sent = 0;
    while (sent < data_length)
    {
        spi_transaction_t* t = heap_caps_calloc(1, sizeof(spi_transaction_t), MALLOC_CAP_DMA); //dma aligned heap mem
        assert(t);
        if (data_length - sent <= max_io_bytes)
        {
            t->length = (data_length - sent) * 8;
            t->tx_buffer = data + sent;
            sent = data_length;
            ESP_ERROR_CHECK(spi_device_queue_trans(dev->spi_device_handle, t, portMAX_DELAY));
            async_spi_queue_msg_t msg = {
                .transaction = t,
                .msg = MSG_FLUSH_COMPLETE,
            };
            xQueueSend(dev->msgQueue, &msg, portMAX_DELAY);
        }
        else
        {
            t->length = max_io_bytes * 8;
            t->tx_buffer = data + sent;
            sent += max_io_bytes;
            ESP_ERROR_CHECK(spi_device_queue_trans(dev->spi_device_handle, t, portMAX_DELAY));
            async_spi_queue_msg_t msg = {
                .transaction = t,
                .msg = MSG_FLUSH_SEND,
            };
            xQueueSend(dev->msgQueue, &msg, portMAX_DELAY);
        }
    }
    return ESP_OK;
}

/*
Transmission result task for async spi
*/
void st7735_recv_data_async(void* pvParameters) // this is going to be a rtos task
{
    async_recv_params *params=(async_recv_params*)pvParameters;
    while (1)
    {
        async_spi_queue_msg_t msg;
        xQueueReceive(params->spi_data_queue, &msg, portMAX_DELAY);
        spi_transaction_t *ret_trans;
        if (msg.msg == MSG_FLUSH_SEND)
        {
            spi_device_get_trans_result(params->st7735_dev->spi_device_handle, &ret_trans, portMAX_DELAY);
        }
        if (msg.msg == MSG_FLUSH_COMPLETE)
        {
            spi_device_get_trans_result(params->st7735_dev->spi_device_handle, &ret_trans, portMAX_DELAY);
            lv_display_flush_ready(params->lvgl_disp);
        }
        assert(ret_trans == msg.transaction);
        free(msg.transaction);
    }
}

//call once only
BaseType_t start_async_data_recv(st7735_handle_t *dev, QueueHandle_t spi_data_queue, lv_display_t *disp)
{
    async_recv_params *params = malloc(sizeof(async_recv_params));
    dev->msgQueue = spi_data_queue;
    params->st7735_dev = dev;
    params->lvgl_disp = disp;
    params->spi_data_queue = spi_data_queue;
    return xTaskCreate(st7735_recv_data_async,"async_spi_dat_recv",4096,params,5,NULL);
}

/*
send st7735 data, only sends 1 byte regardless of data_length
*/
esp_err_t st7735_send_byte(st7735_handle_t *dev, uint8_t data, size_t data_length)
{
    (void)data_length;
    gpio_set_level(dev->pin_dc, 1);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(dev->spi_device_handle, &t));
    return ESP_OK;
}

esp_err_t st7735_set_window(st7735_handle_t *dev, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    // x addr
    data[0] = x0 >> 8;
    data[1] = x0 & 0xFF;
    data[2] = x1 >> 8;
    data[3] = x1 & 0xFF;
    st7735_send_cmd(dev, 0x2A); // Column Address Set
    st7735_send_data(dev, data, 4);

    // y addr
    data[0] = y0 >> 8;
    data[1] = y0 & 0xFF;
    data[2] = y1 >> 8;
    data[3] = y1 & 0xFF;
    st7735_send_cmd(dev, 0x2B); // Row Address Set
    st7735_send_data(dev, data, 4);

    st7735_send_cmd(dev, 0x2C); // Memory Write
    return ESP_OK;
}

esp_err_t st7735_fill_screen(st7735_handle_t *dev, st7735_color_t color, uint8_t *line_buf)
{
    uint16_t w = dev->width - 1;
    uint16_t h = dev->height - 1;
    st7735_set_window(dev, 0, 0, w, h);
    size_t total_pixels = dev->width * dev->height;
    size_t buffer_pixels = dev->width;      //send 1 line each time
    size_t buffer_size = buffer_pixels * 2;

    assert(line_buf);

    for (size_t i = 0; i < buffer_pixels; ++i)
    {
        line_buf[i * 2] = color >> 8;
        line_buf[i * 2 + 1] = color & 0xFF;
    }
    lv_draw_sw_rgb565_swap(line_buf, buffer_size);

    for (size_t row = 0; row < dev->height; ++row)
    {
        ESP_ERROR_CHECK(st7735_send_data(dev, line_buf, buffer_size));
    }
    return ESP_OK;
}

//unused since the display have no backlight control
esp_err_t st7735_set_backlight(st7735_handle_t *dev, bool on);

void st7735_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    st7735_handle_t *lcddev = (st7735_handle_t *)lv_display_get_user_data(disp);

    uint32_t x1 = area->x1;
    uint32_t y1 = area->y1;
    uint32_t x2 = area->x2;
    uint32_t y2 = area->y2;

    uint32_t w = x2 - x1 + 1;
    uint32_t h = y2 - y1 + 1;

    lv_draw_sw_rgb565_swap(color_p, w * h); //swap takes pixels, not bytes
    st7735_set_window(lcddev, x1, y1, x2, y2);
    #ifdef SPI_TRANSFER_ASYNC
        st7735_send_data_async(lcddev, color_p, w * h * 2);
    #else
        ESP_ERROR_CHECK(st7735_send_data(lcddev, color_p, w * h * 2));
        lv_display_flush_ready(disp);
    #endif
}