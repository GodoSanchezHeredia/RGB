#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;
uint32_t active_theme_index = 0;
// Callback para el fade (opacidad)
static void anim_set_opacity(void *obj, int32_t v) {
    lv_obj_set_style_opa(obj, v, 0);
}

// Callback que se ejecuta cuando termina el zoom
static void zoom_finished_cb(lv_anim_t *a) {
    ui_foro();
}

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 400, 960);

    lv_obj_t *parent_obj = obj;

    // --- Crear el logo ---
    lv_obj_t *logo = lv_img_create(parent_obj);
    lv_obj_set_pos(logo, -108, 334);
    lv_obj_set_size(logo, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_img_set_src(logo, &img_logo_prueba_1);
    lv_img_set_zoom(logo, 1);               // Empieza pequeño
    lv_obj_set_style_opa(logo, LV_OPA_COVER, 0); // Visible
    lv_img_set_angle(logo, 900);

    // --- Animación de zoom ---
    lv_anim_t a_zoom;
    lv_anim_init(&a_zoom);
    lv_anim_set_var(&a_zoom, logo);
    lv_anim_set_exec_cb(&a_zoom, (lv_anim_exec_xcb_t)lv_img_set_zoom);
    lv_anim_set_time(&a_zoom, 5000);        // 2 segundos
    lv_anim_set_values(&a_zoom, 1, 256);    // Crece
    lv_anim_set_path_cb(&a_zoom, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a_zoom, zoom_finished_cb); // Cuando termina, difuminar
    lv_anim_start(&a_zoom);

    tick_screen_main();
}


void tick_screen_main() {
}
static lv_obj_t *truck1;
static lv_obj_t *truck2;

static void inclinometro_update_cb(lv_timer_t *t) {
        // 900 = 90° → variación ±20° (200 unidades)
    int base_angle = 900;      // Centro de inclinación = 90°
    int variation = 200;       // 20° en unidades LVGL (20 * 10)

    int ang1 = base_angle - variation + (rand() % (variation * 2));
    int ang2 = base_angle - variation + (rand() % (variation * 2));

    lv_img_set_angle(truck1, ang1);
    lv_img_set_angle(truck2, ang2);
}
void create_screen_foro() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.foro = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 400, 960);

    truck1 = lv_img_create(obj);
    lv_obj_set_pos(truck1, 63, 78);
    lv_img_set_src(truck1, &img_image1_2_4);
    lv_img_set_angle(truck1, 900);

    truck2 = lv_img_create(obj);
    lv_obj_set_pos(truck2, 4, 594);
    lv_img_set_src(truck2, &img_image1_8);
    lv_img_set_angle(truck2, 900);

    // Logo de tu UI deja igual si lo deseas
    lv_obj_t *logo = lv_img_create(obj);
    lv_obj_set_pos(logo, 0, 303);
    lv_img_set_src(logo, &img_logo_prueba_1);
    lv_img_set_zoom(logo, 60);
    lv_img_set_angle(logo, 900);
    
    tick_screen_foro();
}

void tick_screen_foro() {
    static bool started = false;
    if (!started) {
        started = true;
        lv_timer_create(inclinometro_update_cb, 800, NULL); // 120 ms → suave
    }
}



typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
    tick_screen_foro,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

void create_screens() {
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_main();
    create_screen_foro();
}
