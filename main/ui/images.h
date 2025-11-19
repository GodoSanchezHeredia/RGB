#ifndef EEZ_LVGL_UI_IMAGES_H
#define EEZ_LVGL_UI_IMAGES_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const lv_img_dsc_t img_logo_prueba_1;
extern const lv_img_dsc_t img_image1_2_4;
extern const lv_img_dsc_t img_image1_8;
extern const lv_img_dsc_t img_path7_1_5;
extern const lv_img_dsc_t img_path7_2;

#ifndef EXT_IMG_DESC_T
#define EXT_IMG_DESC_T
typedef struct _ext_img_desc_t {
    const char *name;
    const lv_img_dsc_t *img_dsc;
} ext_img_desc_t;
#endif

extern const ext_img_desc_t images[5];


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_IMAGES_H*/