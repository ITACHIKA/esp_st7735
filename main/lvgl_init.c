#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_st7735.h"
#include "lvgl_init.h"

#define LVGL_DRAW_BUF_LINES 20 // number of display lines in each draw buffer
#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE (4 * 1024)
#define LVGL_TASK_PRIORITY 2

#define LCD_HOST SPI2_HOST

#define LCD_H 128
#define LCD_W 160

SemaphoreHandle_t sys_lvgl_mutex;

esp_timer_handle_t lvgl_esp_timer_handle;

static const char *TAG = "LVGL";

static void lvgl_rtos_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    uint32_t time_threshold_ms = 1000 / CONFIG_FREERTOS_HZ;
    while (1)
    {
        xSemaphoreTake(sys_lvgl_mutex, portMAX_DELAY);
        time_till_next_ms = lv_timer_handler();
        xSemaphoreGive(sys_lvgl_mutex);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
        vTaskDelay(pdMS_TO_TICKS(time_till_next_ms));
    }
}

void lvgl_tick_callback()
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

lv_display_t* st7735_lvgl_init(SemaphoreHandle_t user_lvgl_mutex)
{
    sys_lvgl_mutex = user_lvgl_mutex;

    st7735_handle_t *st7735_dev = st7735_display_init();

    lv_init();
    lv_display_t *st7735_display = lv_display_create(LCD_H, LCD_W);
    lv_display_set_color_format(st7735_display, LV_COLOR_FORMAT_RGB565);

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    size_t draw_buffer_sz = LCD_H * LVGL_DRAW_BUF_LINES * lv_color_format_get_size(lv_display_get_color_format(st7735_display));
    ESP_LOGI(TAG, "buffer size:%u\r\n", draw_buffer_sz);
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
        .callback = lvgl_tick_callback,
        .name = "lvgl_tick"};
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_esp_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_esp_timer_handle, LVGL_TICK_PERIOD_MS * 1000));

    TaskHandle_t lvgl_handler_task;
    assert(xTaskCreate(lvgl_rtos_task, "lvgl task", LVGL_TASK_STACK_SIZE, NULL, 4, &lvgl_handler_task) == pdPASS);

    // xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    // example_lvgl_demo_ui(st7735_display);
    // xSemaphoreGive(lvgl_mutex);
    return st7735_display;
}