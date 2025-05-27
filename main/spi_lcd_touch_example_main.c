/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "misc/lv_log.h"
#include "esp_lcd_st7735.h"
#include "esp_heap_trace.h"

static const char *TAG = "SPI LCD";

// Using SPI2 in the example
#define LCD_HOST SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL 1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_SCLK 18
#define PIN_NUM_MOSI 19
#define PIN_NUM_MISO 21
#define PIN_NUM_LCD_DC 5
#define PIN_NUM_LCD_RST 15
#define PIN_NUM_LCD_CS 4
#define PIN_NUM_BK_LIGHT 2

#define LCD_H 128
#define LCD_W 160

// Bit number used to represent command and parameter
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8
#define LVGL_DRAW_BUF_LINES 20 // number of display lines in each draw buffer
#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE (4 * 1024)
#define LVGL_TASK_PRIORITY 2

SemaphoreHandle_t lvgl_mutex;

esp_timer_handle_t lvgl_esp_timer_handle;

extern void example_lvgl_demo_ui(lv_disp_t *disp);

static void lvgl_rtos_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    uint32_t time_threshold_ms = 1000 / CONFIG_FREERTOS_HZ;
    while (1)
    {
        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
        time_till_next_ms = lv_timer_handler();
        xSemaphoreGive(lvgl_mutex);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
        vTaskDelay(pdMS_TO_TICKS(time_till_next_ms));
    }
}

void lv_log_print(lv_log_level_t level, const char *buf)
{
    esp_rom_printf("%s", buf);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    ESP_LOGI(__func__, "Stack overflow detected in task: %s\n", pcTaskName);
    while (1)
        ;
}

uint32_t ticks = 0;

void lvgl_tick_handler()
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
    ticks++;
}

void app_main(void)
{

    lvgl_mutex = xSemaphoreCreateMutex();

    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H * 80 * sizeof(uint16_t),
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LCD_PIXEL_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = PIN_NUM_LCD_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    st7735_handle_t *st7735_dev = malloc(sizeof(st7735_handle_t));
    st7735_dev->width = LCD_W,
    st7735_dev->height = LCD_H,
    st7735_dev->max_io_bytes = 0,
    st7735_dev->pin_bl = PIN_NUM_BK_LIGHT,
    st7735_dev->pin_dc = PIN_NUM_LCD_DC,
    st7735_dev->pin_rst = PIN_NUM_LCD_RST,
    st7735_dev->spi_host_device = LCD_HOST,
    st7735_dev->msgQueue = NULL,

    st7735_init(LCD_HOST, &buscfg, &devcfg, st7735_dev);
    st7735_reset(st7735_dev);

    lv_init();
    lv_display_t *st7735_display = lv_display_create(LCD_H, LCD_W);
    lv_display_set_color_format(st7735_display, LV_COLOR_FORMAT_RGB565);

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    size_t draw_buffer_sz = LCD_H * 20 * lv_color_format_get_size(lv_display_get_color_format(st7735_display));
    ESP_LOGI(TAG,"buffer size:%u\r\n", draw_buffer_sz);
    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);

    //  initialize LVGL draw buffers
    lv_display_set_buffers(st7735_display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // associate the mipi panel handle to the display
    lv_display_set_user_data(st7735_display, st7735_dev);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(st7735_display, st7735_flush_cb);

    QueueHandle_t async_msg_queue = xQueueCreate(7, sizeof(async_spi_queue_msg_t));
    assert(async_msg_queue);

    start_async_data_recv(st7735_dev, async_msg_queue, st7735_display);

    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = lvgl_tick_handler,
        .name = "lvgl_tick"};
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_esp_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_esp_timer_handle, LVGL_TICK_PERIOD_MS * 1000));

    TaskHandle_t lvgl_handler_task;
    assert(xTaskCreate(lvgl_rtos_task,"lvgl task",LVGL_TASK_STACK_SIZE,NULL,4,&lvgl_handler_task)==pdPASS);

    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    example_lvgl_demo_ui(st7735_display);
    xSemaphoreGive(lvgl_mutex);

    // while (1)
    // {
    //     st7735_fill_screen(&st7735_dev, 0xF800, line_buf);
    //     vTaskDelay(pdMS_TO_TICKS(500));
    //     st7735_fill_screen(&st7735_dev, 0x07E0, line_buf);
    //     vTaskDelay(pdMS_TO_TICKS(500));
    //     st7735_fill_screen(&st7735_dev, 0x001F, line_buf);
    //     vTaskDelay(pdMS_TO_TICKS(500));
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}