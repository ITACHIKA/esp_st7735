/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "misc/lv_log.h"
#include "lvgl_init.h"
#include "lvgl.h"

static const char *TAG = "SPI LCD";


static lv_obj_t * btn;
static lv_display_rotation_t rotation = LV_DISP_ROTATION_0;

static void btn_cb(lv_event_t * e)
{
    lv_display_t *disp = lv_event_get_user_data(e);
    rotation++;
    if (rotation > LV_DISP_ROTATION_270) {
        rotation = LV_DISP_ROTATION_0;
    }
    lv_disp_set_rotation(disp, rotation);
}
static void set_angle(void * obj, int32_t v)
{
    lv_arc_set_value(obj, v);
}

void example_lvgl_demo_ui(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);

    btn = lv_button_create(scr);
    lv_obj_t * lbl = lv_label_create(btn);
    lv_label_set_text_static(lbl, LV_SYMBOL_REFRESH" ROTATE");
    lv_obj_align(btn, LV_ALIGN_BOTTOM_LEFT, 30, -30);
    /*Button event*/
    lv_obj_add_event_cb(btn, btn_cb, LV_EVENT_CLICKED, disp);

    /*Create an Arc*/
    lv_obj_t * arc = lv_arc_create(scr);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(arc);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, arc);
    lv_anim_set_exec_cb(&a, set_angle);
    lv_anim_set_duration(&a, 500);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);    /*Just for the demo*/
    //lv_anim_set_repeat_delay(&a, 500);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_start(&a);    
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    ESP_LOGI(__func__, "Stack overflow detected in task: %s\n", pcTaskName);
    while (1);
}

SemaphoreHandle_t user_lvgl_mutex;

void app_main(void)
{
    user_lvgl_mutex = xSemaphoreCreateMutex();
    lv_display_t* lv_disp= st7735_lvgl_init(user_lvgl_mutex);
    xSemaphoreTake(user_lvgl_mutex,portMAX_DELAY);
    example_lvgl_demo_ui(lv_disp);
    xSemaphoreGive(user_lvgl_mutex);
}