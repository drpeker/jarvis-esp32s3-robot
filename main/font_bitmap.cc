#include "font_bitmap.h"

#include <cstdint>
#include <cstring>

#include <esp_heap_caps.h>
#include <esp_log.h>

#define TAG "font"

// main/fonts/caption_font.S.in
extern "C" const uint8_t caption_font_start[];
extern "C" const uint8_t caption_font_end[];

namespace {

// The CBIN file holds an lv_font_t, an lv_font_fmt_txt_dsc_t, the cmaps (as packed 20-byte
// records) and the kerning tables, with every pointer stored as an offset. The small
// structures are copied to PSRAM and their pointers rebased; the large tables (glyph
// bitmaps, glyph descriptions, cmap lists, kerning values) stay in flash.

template <typename T>
T* CopyToPsram(const uint8_t* src) {
    auto* dst = static_cast<T*>(heap_caps_malloc(sizeof(T), MALLOC_CAP_SPIRAM));
    if (dst) std::memcpy(dst, src, sizeof(T));
    return dst;
}

template <typename P>
void Rebase(P& ptr, const uint8_t* base) {
    if (ptr != nullptr) ptr = reinterpret_cast<P>(base + reinterpret_cast<uintptr_t>(ptr));
}

uint32_t Read32(const uint8_t* p) {
    uint32_t v;
    std::memcpy(&v, p, 4);
    return v;
}
uint16_t Read16(const uint8_t* p) {
    uint16_t v;
    std::memcpy(&v, p, 2);
    return v;
}

const lv_font_t* Load(const uint8_t* bin, size_t size) {
    if (size < sizeof(lv_font_t) || (reinterpret_cast<uintptr_t>(bin) & 3) != 0) return nullptr;
    lv_font_t* font = CopyToPsram<lv_font_t>(bin);
    if (!font) return nullptr;
    font->get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt;
    font->get_glyph_bitmap = lv_font_get_bitmap_fmt_txt;
    font->release_glyph = nullptr;
    font->fallback = nullptr;
    font->user_data = nullptr;

    const uint8_t* dsc_base = bin + reinterpret_cast<uintptr_t>(font->dsc);
    auto* dsc = CopyToPsram<lv_font_fmt_txt_dsc_t>(dsc_base);
    if (!dsc) return nullptr;
    font->dsc = dsc;
    Rebase(dsc->glyph_bitmap, dsc_base);
    Rebase(dsc->glyph_dsc, dsc_base);

    if (dsc->cmap_num != 0) {
        const uint8_t* cm = dsc_base + reinterpret_cast<uintptr_t>(dsc->cmaps);
        auto* cmaps = static_cast<lv_font_fmt_txt_cmap_t*>(
            heap_caps_calloc(dsc->cmap_num, sizeof(lv_font_fmt_txt_cmap_t), MALLOC_CAP_SPIRAM));
        if (!cmaps) return nullptr;
        const uint8_t* rec = cm;
        for (unsigned i = 0; i < dsc->cmap_num; ++i, rec += 20) {
            cmaps[i].range_start = Read32(rec);
            cmaps[i].range_length = Read16(rec + 4);
            cmaps[i].glyph_id_start = Read16(rec + 6);
            const uint32_t list = Read32(rec + 8), ofs = Read32(rec + 12);
            cmaps[i].unicode_list = list ? reinterpret_cast<const uint16_t*>(cm + list) : nullptr;
            cmaps[i].glyph_id_ofs_list = ofs ? static_cast<const void*>(cm + ofs) : nullptr;
            cmaps[i].list_length = Read16(rec + 16);
            cmaps[i].type = static_cast<lv_font_fmt_txt_cmap_type_t>(rec[18]);
        }
        dsc->cmaps = cmaps;
    }

    if (dsc->kern_dsc != nullptr) {
        const uint8_t* kern = dsc_base + reinterpret_cast<uintptr_t>(dsc->kern_dsc);
        if (dsc->kern_classes == 1) {
            auto* k = CopyToPsram<lv_font_fmt_txt_kern_classes_t>(kern);
            if (!k) return nullptr;
            Rebase(k->class_pair_values, kern);
            Rebase(k->left_class_mapping, kern);
            Rebase(k->right_class_mapping, kern);
            dsc->kern_dsc = k;
        } else {
            auto* k = CopyToPsram<lv_font_fmt_txt_kern_pair_t>(kern);
            if (!k) return nullptr;
            Rebase(k->glyph_ids, kern);
            Rebase(k->values, kern);
            dsc->kern_dsc = k;
        }
    }
    ESP_LOGI(TAG, "caption font: %u bytes in flash, line height %d, %u cmaps, %u bpp", (unsigned)size,
             font->line_height, (unsigned)dsc->cmap_num, (unsigned)dsc->bpp);
    return font;
}

}  // namespace

namespace font_bitmap {

const lv_font_t* Caption() {
    static const lv_font_t* font = Load(caption_font_start, caption_font_end - caption_font_start);
    return font;
}

}  // namespace font_bitmap
