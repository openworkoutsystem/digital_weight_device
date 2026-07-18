// Placeholder image to satisfy LV_IMG_DECLARE(ui_img_886805914)
#include "lvgl.h"

#if defined __has_include
  #if __has_include("../ui.h")
    #include "../ui.h"
  #endif
#endif

static const uint8_t ui_img_886805914_map[] = { 0x00, 0x00 }; // 1x1 black pixel for LV_COLOR_DEPTH=16

const lv_img_dsc_t ui_img_886805914 = {
    .header.always_zero = 0,
    .header.w = 1,
    .header.h = 1,
    .data_size = sizeof(ui_img_886805914_map),
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = ui_img_886805914_map,
};
