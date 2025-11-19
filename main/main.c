#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "driver/ledc.h"
#include "LCD_Driver/ST7701S.h"
#include "demos/lv_demos.h"
#include "ui.h"
#include "ui/images.h" 

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          6      // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (0)    // Initial duty = 0%
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz

static const char *TAG = "example";

/********************* LCD *********************/
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (18 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT       -1
#define EXAMPLE_PIN_NUM_HSYNC          38
#define EXAMPLE_PIN_NUM_VSYNC          39
#define EXAMPLE_PIN_NUM_DE             40
#define EXAMPLE_PIN_NUM_PCLK           41
#define EXAMPLE_PIN_NUM_DATA0          5  // B0
#define EXAMPLE_PIN_NUM_DATA1          45 // B1
#define EXAMPLE_PIN_NUM_DATA2          48 // B2
#define EXAMPLE_PIN_NUM_DATA3          47 // B3
#define EXAMPLE_PIN_NUM_DATA4          21 // B4
#define EXAMPLE_PIN_NUM_DATA5          14 // G0
#define EXAMPLE_PIN_NUM_DATA6          13 // G1
#define EXAMPLE_PIN_NUM_DATA7          12 // G2
#define EXAMPLE_PIN_NUM_DATA8          11 // G3
#define EXAMPLE_PIN_NUM_DATA9          10 // G4
#define EXAMPLE_PIN_NUM_DATA10         9  // G5
#define EXAMPLE_PIN_NUM_DATA11         46 // R0
#define EXAMPLE_PIN_NUM_DATA12         3  // R1
#define EXAMPLE_PIN_NUM_DATA13         8  // R2
#define EXAMPLE_PIN_NUM_DATA14         18 // R3
#define EXAMPLE_PIN_NUM_DATA15         17 // R4
#define EXAMPLE_PIN_NUM_DISP_EN        -1

#define EXAMPLE_LCD_H_RES              400
#define EXAMPLE_LCD_V_RES              960

#if CONFIG_EXAMPLE_DOUBLE_FB
#define EXAMPLE_LCD_NUM_FB             2
#else
#define EXAMPLE_LCD_NUM_FB             1
#endif 

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2

#define SPI_SDA 1
#define SPI_SCL 2
#define SPI_CS  42

#define ST7701S_ORIENTATION_PORTRAIT 0
#define ST7701S_ORIENTATION_LANDSCAPE 1
#define ST7701S_ORIENTATION_PORTRAIT_ROTATED_180 2
#define ST7701S_ORIENTATION_LANDSCAPE_ROTATED_360 3


/********************* BackLight *********************/
static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}



static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    return high_task_awoken == pdTRUE;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
     int x_offset = 120;  // mueve 20 píxeles a la derecha
    int y_offset = 40;  // mueve 40 píxeles hacia abajo

    int offsetx1 = area->x1 + x_offset;
    int offsetx2 = area->x2 + x_offset;
    int offsety1 = area->y1 + y_offset;
    int offsety2 = area->y2 + y_offset;


    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void app_main(void)
{   
    // Set the LEDC peripheral configuration
    example_ledc_init();
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    ST7701S_handle st7701s = ST7701S_newObject(SPI_SDA, SPI_SCL, SPI_CS, SPI3_HOST, SPI_METHOD);
    ST7701S_screen_init(st7701s, 1);

  //  ST7701S_SetOrientation(st7701s, ST7701S_ORIENTATION_PORTRAIT_ROTATED_180);

    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;



/********************* RGB LCD panel driver *********************/
    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16,
        .psram_trans_align = 64,
        .num_fbs = EXAMPLE_LCD_NUM_FB,
#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
        .clk_src = LCD_CLK_SRC_PLL240M,
        .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
        .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .de_gpio_num = EXAMPLE_PIN_NUM_DE,
        
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES, 
        .hsync_pulse_width = 10,
        .hsync_back_porch = 30,
        .hsync_front_porch = 50,
        .vsync_pulse_width = 2,
        .vsync_back_porch = 15,
        .vsync_front_porch = 17,
            .flags.pclk_active_neg = false,
        },
        .flags.fb_in_psram = true,
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    void *buf1 = NULL;
    void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#endif

    ESP_LOGI(TAG, "Register display driver to LVGL");
        lv_disp_drv_init(&disp_drv);

    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true;
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    //lv_disp_set_rotation(disp, LV_DISP_ROT_90);
    //lv_disp_set_rotation(NULL, LV_DISP_ROT_90);
    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

/********************* Demo *********************/
      ui_init();

      
    while (1) {
        
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();



    }
}
