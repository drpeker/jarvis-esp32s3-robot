// Ready-made bitmap fonts read straight from flash: nothing is drawn at run time and the
// glyph bitmaps never go to RAM. Only the few font headers are copied (to PSRAM).
#pragma once
#include <lvgl.h>

namespace font_bitmap {
// Caption font (Noto Sans 14 px, 1 bpp, with Turkish letters). nullptr if unusable.
const lv_font_t* Caption();
}  // namespace font_bitmap
