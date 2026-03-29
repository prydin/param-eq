#include "ui_lvgl.h"

#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

#include "../../common/constants.h"
#include "../../common/filter.h"

namespace
{

#ifndef UI_LIGHT_MODE
#define UI_LIGHT_MODE 0
#endif

    constexpr uint16_t kScreenWidth = 320;
    constexpr uint16_t kScreenHeight = 240;
    constexpr uint16_t kDrawBufferLines = 24;
    constexpr int16_t kChartMinDb = -15;
    constexpr int16_t kChartMaxDb = 15;
    constexpr int16_t kChartDbScale = 10;
    constexpr float kMinGraphFrequency = 2.0f;
    constexpr float kMaxGraphFrequency = 20000.0f;
    constexpr size_t kChartPoints = 240;
    constexpr lv_coord_t kChartX = 28;
    constexpr lv_coord_t kChartY = 10;
    constexpr lv_coord_t kChartWidth = 230;
    constexpr lv_coord_t kChartHeight = 170;
    constexpr lv_coord_t kPanelX = 266;
    constexpr lv_coord_t kPanelY = 10;
    constexpr lv_coord_t kPanelWidth = 46;
    constexpr lv_coord_t kPanelHeight = 170;
    constexpr lv_coord_t kMetricX = 272;
    constexpr lv_coord_t kBottomBadgeY = 208;
    constexpr lv_coord_t kSimpleVuWidth = 40;
    constexpr lv_coord_t kSimpleVuFramePad = 2;
    constexpr lv_coord_t kSimpleVuFrameWidth = kSimpleVuWidth + (2 * kSimpleVuFramePad);
    constexpr lv_coord_t kSimpleFrameGap = 6;
    constexpr lv_coord_t kSimpleCenterFrameX = kSimpleVuFrameWidth + kSimpleFrameGap;
    constexpr lv_coord_t kSimpleCenterFrameWidth = kScreenWidth - (2 * (kSimpleVuFrameWidth + kSimpleFrameGap));
    constexpr lv_coord_t kSimpleTextInset = 24;
    constexpr lv_coord_t kSimpleVuRedHeight = (kScreenHeight + 9) / 10;
    constexpr uint8_t kSimpleVuTickCount = 5;
    constexpr lv_coord_t kSimplePeakMarkerHeight = 2;
    constexpr uint8_t kFftBinCount = 16;
    constexpr lv_coord_t kFftFrameX = 8;
    constexpr lv_coord_t kFftFrameY = 8;
    constexpr lv_coord_t kFftFrameWidth = 304;
    constexpr lv_coord_t kFftFrameHeight = 224;
    constexpr lv_coord_t kFftTrackX = 12;
    constexpr lv_coord_t kFftTrackY = 24;
    constexpr lv_coord_t kFftTrackHeight = 182;
    constexpr lv_coord_t kFftBarGroupGap = 2;
    constexpr uint32_t kVuAttackMs = 2;
    constexpr uint32_t kVuReleaseMs = 210;
    constexpr uint32_t kVuPeakHoldMs = 220;
    constexpr uint32_t kVuPeakFallPerSecond = 24000;
    constexpr uint32_t kUiModeDetailed = 0;
    constexpr uint32_t kUiModeSimple = 1;
    constexpr uint32_t kUiModeFft = 2;

    TFT_eSPI *g_tft = nullptr;
    lv_disp_draw_buf_t g_drawBuf;
    lv_color_t g_buf1[kScreenWidth * kDrawBufferLines];

    lv_obj_t *g_chart = nullptr;
    lv_chart_series_t *g_seriesSelected = nullptr;
    lv_chart_series_t *g_seriesCombined = nullptr;

    lv_obj_t *g_modeDetailedRoot = nullptr;
    lv_obj_t *g_modeSimpleRoot = nullptr;
    lv_obj_t *g_modeFftRoot = nullptr;
    lv_obj_t *g_simpleLeftVu = nullptr;
    lv_obj_t *g_simpleLeftVuBgGreen = nullptr;
    lv_obj_t *g_simpleLeftVuBgRed = nullptr;
    lv_obj_t *g_simpleLeftVuGreen = nullptr;
    lv_obj_t *g_simpleLeftVuRed = nullptr;
    lv_obj_t *g_simpleLeftVuGlow = nullptr;
    lv_obj_t *g_simpleLeftPeak = nullptr;
    lv_obj_t *g_simpleRightVu = nullptr;
    lv_obj_t *g_simpleRightVuBgGreen = nullptr;
    lv_obj_t *g_simpleRightVuBgRed = nullptr;
    lv_obj_t *g_simpleRightVuGreen = nullptr;
    lv_obj_t *g_simpleRightVuRed = nullptr;
    lv_obj_t *g_simpleRightVuGlow = nullptr;
    lv_obj_t *g_simpleRightPeak = nullptr;
    lv_obj_t *g_simpleSampleRate = nullptr;
    lv_obj_t *g_simpleOutGainLabel = nullptr;
    lv_obj_t *g_simpleOutGainValue = nullptr;
    lv_obj_t *g_simpleInGainLabel = nullptr;
    lv_obj_t *g_simpleInGainValue = nullptr;
    lv_obj_t *g_fftTrack[kFftBinCount] = {};
    lv_obj_t *g_fftBgGreen[kFftBinCount] = {};
    lv_obj_t *g_fftBgRed[kFftBinCount] = {};
    lv_obj_t *g_fftGreen[kFftBinCount] = {};
    lv_obj_t *g_fftRed[kFftBinCount] = {};
    lv_obj_t *g_fftGlow[kFftBinCount] = {};
    lv_obj_t *g_fftPeak[kFftBinCount] = {};
    uint32_t g_activeUiMode = kUiModeDetailed;

    lv_obj_t *g_valueFs = nullptr;
    lv_obj_t *g_valueM = nullptr;
    lv_obj_t *g_valueF = nullptr;
    lv_obj_t *g_valueQ = nullptr;
    lv_obj_t *g_valueVol = nullptr;
    lv_obj_t *g_valueFreq = nullptr;
    lv_obj_t *g_freqCaret = nullptr;

    lv_obj_t *g_filterLabels[3] = {nullptr, nullptr, nullptr};
    lv_obj_t *g_indexLabels[FILTER_BANDS] = {};

    uint16_t g_vuDisplayedLeft = 0;
    uint16_t g_vuDisplayedRight = 0;
    uint16_t g_vuPeakLeft = 0;
    uint16_t g_vuPeakRight = 0;
    uint32_t g_vuPeakHoldUntilLeft = 0;
    uint32_t g_vuPeakHoldUntilRight = 0;
    uint32_t g_lastVuAnimMs = 0;
    uint16_t g_fftDisplayed[kFftBinCount] = {};
    uint16_t g_fftPeakLevel[kFftBinCount] = {};
    uint32_t g_fftPeakHoldUntil[kFftBinCount] = {};
    uint32_t g_lastFftAnimMs = 0;

    uint32_t g_lastTickMs = 0;

    lv_color_t colorScreenBg() { return UI_LIGHT_MODE ? lv_color_hex(0xF3F4F6) : lv_color_hex(0x000000); }
    lv_color_t colorChartBg() { return UI_LIGHT_MODE ? lv_color_hex(0xd8d8d8) : lv_color_hex(0x0B1220); }
    lv_color_t colorChartBorder() { return UI_LIGHT_MODE ? lv_color_hex(0x64748B) : lv_color_hex(0x374151); }
    lv_color_t colorChartSelected() { return UI_LIGHT_MODE ? lv_color_hex(0x0369A1) : lv_color_hex(0x00D1FF); }
    lv_color_t colorChartCombined() { return UI_LIGHT_MODE ? lv_color_hex(0x16A34A) : lv_color_hex(0x22C55E); }
    lv_color_t colorOn() { return UI_LIGHT_MODE ? lv_color_hex(0x15803D) : lv_color_hex(0x16A34A); }
    lv_color_t colorOnText() { return UI_LIGHT_MODE ? lv_color_hex(0xFFFFFF) : lv_color_hex(0x000000); }
    lv_color_t colorOffBg() { return UI_LIGHT_MODE ? lv_color_hex(0xE5E7EB) : lv_color_hex(0x1F2937); }
    lv_color_t colorOffText() { return UI_LIGHT_MODE ? lv_color_hex(0x475569) : lv_color_hex(0x9CA3AF); }
    lv_color_t colorMetricLabel() { return UI_LIGHT_MODE ? lv_color_hex(0x1D4ED8) : lv_color_hex(0x93C5FD); }
    lv_color_t colorMetricValue() { return UI_LIGHT_MODE ? lv_color_hex(0x0F172A) : lv_color_hex(0xF9FAFB); }
    lv_color_t colorMetricPanel() { return UI_LIGHT_MODE ? lv_color_hex(0xE2E8F0) : lv_color_hex(0x0A0F18); }
    lv_color_t colorMetricBorder() { return UI_LIGHT_MODE ? lv_color_hex(0x64748B) : lv_color_hex(0x94A3B8); }
    lv_color_t colorMetricLabelBg() { return UI_LIGHT_MODE ? lv_color_hex(0xDBEAFE) : lv_color_hex(0x2A3A52); }
    lv_color_t colorMetricValueBg() { return UI_LIGHT_MODE ? lv_color_hex(0xFFFFFF) : lv_color_hex(0x020617); }
    lv_color_t colorMetricValueBorder() { return UI_LIGHT_MODE ? lv_color_hex(0x94A3B8) : lv_color_hex(0x334155); }
    lv_color_t colorAxisText() { return UI_LIGHT_MODE ? lv_color_hex(0x475569) : lv_color_hex(0x94A3B8); }
    lv_color_t colorFooterFrame() { return UI_LIGHT_MODE ? lv_color_hex(0x94A3B8) : lv_color_hex(0x475569); }
    lv_color_t colorInstrumentFrame() { return UI_LIGHT_MODE ? lv_color_hex(0x64748B) : lv_color_hex(0x6B7280); }
    lv_color_t colorInstrumentFrameInner() { return UI_LIGHT_MODE ? lv_color_hex(0xCBD5E1) : lv_color_hex(0x1F2937); }
    lv_color_t colorEngraving() { return UI_LIGHT_MODE ? lv_color_hex(0x334155) : lv_color_hex(0x9CA3AF); }
    lv_color_t colorVuTrack() { return UI_LIGHT_MODE ? lv_color_hex(0x0D2A14) : lv_color_hex(0x081C0D); }
        lv_color_t colorVuTrackTop()
        {
    #ifdef ST7796_DRIVER
        // Compensate for panel inversion so the warning zone appears dark red on screen.
        return lv_color_hex(0x113A3A);
    #else
        return UI_LIGHT_MODE ? lv_color_hex(0x3A1111) : lv_color_hex(0x220909);
    #endif
        }
    lv_color_t colorVuGreen() { return UI_LIGHT_MODE ? lv_color_hex(0x16A34A) : lv_color_hex(0x22C55E); }
    lv_color_t colorVuGlow() { return UI_LIGHT_MODE ? lv_color_hex(0x4ADE80) : lv_color_hex(0x86EFAC); }
    lv_color_t colorVuPeak() { return UI_LIGHT_MODE ? lv_color_hex(0xFDE047) : lv_color_hex(0xFACC15); }
        lv_color_t colorVuRed()
        {
    #ifdef ST7796_DRIVER
        // The ST7796 path enables panel inversion; using cyan here renders as red on this panel.
        return lv_color_hex(0x00FFFF);
    #else
        return UI_LIGHT_MODE ? lv_color_hex(0xDC2626) : lv_color_hex(0xEF4444);
    #endif
        }

    void tftFlush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
    {
        if (g_tft == nullptr)
        {
            lv_disp_flush_ready(disp);
            return;
        }

        uint32_t width = static_cast<uint32_t>(area->x2 - area->x1 + 1);
        uint32_t height = static_cast<uint32_t>(area->y2 - area->y1 + 1);

        g_tft->startWrite();
        g_tft->setAddrWindow(area->x1, area->y1, width, height);
        g_tft->pushColors(reinterpret_cast<uint16_t *>(&color_p->full), width * height, true);
        g_tft->endWrite();
        lv_disp_flush_ready(disp);
    }

    void makeBadge(lv_obj_t *obj, const char *text, int x, int y)
    {
        lv_label_set_text(obj, text);
        lv_obj_set_pos(obj, x, y);
        lv_obj_set_style_pad_left(obj, 5, 0);
        lv_obj_set_style_pad_right(obj, 5, 0);
        lv_obj_set_style_pad_top(obj, 2, 0);
        lv_obj_set_style_pad_bottom(obj, 2, 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(obj, colorOffBg(), 0);
        lv_obj_set_style_text_color(obj, colorOffText(), 0);
        lv_obj_set_style_radius(obj, 3, 0);
    }

    void setBadgeState(lv_obj_t *obj, bool active)
    {
        if (active)
        {
            lv_obj_set_style_bg_color(obj, colorOn(), 0);
            lv_obj_set_style_text_color(obj, colorOnText(), 0);
        }
        else
        {
            lv_obj_set_style_bg_color(obj, colorOffBg(), 0);
            lv_obj_set_style_text_color(obj, colorOffText(), 0);
        }
    }

    void setLabelValue(lv_obj_t *obj, const char *format, float value)
    {
        char buf[24];
        snprintf(buf, sizeof(buf), format, value);
        lv_label_set_text(obj, buf);
    }

    void styleMetricLabel(lv_obj_t *obj)
    {
        lv_obj_set_style_text_color(obj, colorMetricLabel(), 0);
        lv_obj_set_style_text_font(obj, &lv_font_montserrat_8, 0);
        lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_style_bg_color(obj, colorMetricLabelBg(), 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(obj, 1, 0);
        lv_obj_set_style_pad_left(obj, 2, 0);
        lv_obj_set_style_pad_right(obj, 2, 0);
        lv_obj_set_style_pad_top(obj, 0, 0);
        lv_obj_set_style_pad_bottom(obj, 0, 0);
    }

    void styleMetricValue(lv_obj_t *obj)
    {
        lv_obj_set_style_text_color(obj, colorMetricValue(), 0);
        lv_obj_set_style_text_font(obj, lv_theme_get_font_small(obj), 0);
        lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_style_bg_color(obj, colorMetricValueBg(), 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(obj, colorMetricValueBorder(), 0);
        lv_obj_set_style_border_width(obj, 1, 0);
        lv_obj_set_style_radius(obj, 1, 0);
        lv_obj_set_style_pad_left(obj, 2, 0);
        lv_obj_set_style_pad_right(obj, 2, 0);
        lv_obj_set_style_pad_top(obj, 0, 0);
        lv_obj_set_style_pad_bottom(obj, 0, 0);
    }

    void formatSampleRateKhz(char *buf, size_t len, uint32_t sampleRate)
    {
        if (sampleRate == 0)
        {
            snprintf(buf, len, "----");
            return;
        }

        if (sampleRate >= 100000)
        {
            snprintf(buf, len, "%4.0f", sampleRate / 1000.0f);
        }
        else
        {
            snprintf(buf, len, "%4.1f", sampleRate / 1000.0f);
        }
    }

    void setUiMode(uint32_t uiMode)
    {
        const uint32_t previousMode = g_activeUiMode;
        if (uiMode == kUiModeSimple)
        {
            g_activeUiMode = kUiModeSimple;
        }
        else if (uiMode == kUiModeFft)
        {
            g_activeUiMode = kUiModeFft;
        }
        else
        {
            g_activeUiMode = kUiModeDetailed;
        }

        if (g_modeDetailedRoot != nullptr)
        {
            if (g_activeUiMode == kUiModeDetailed)
            {
                lv_obj_clear_flag(g_modeDetailedRoot, LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                lv_obj_add_flag(g_modeDetailedRoot, LV_OBJ_FLAG_HIDDEN);
            }
        }

        if (g_modeSimpleRoot != nullptr)
        {
            if (g_activeUiMode == kUiModeSimple)
            {
                lv_obj_clear_flag(g_modeSimpleRoot, LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                lv_obj_add_flag(g_modeSimpleRoot, LV_OBJ_FLAG_HIDDEN);
            }
        }

        if (g_modeFftRoot != nullptr)
        {
            if (g_activeUiMode == kUiModeFft)
            {
                lv_obj_clear_flag(g_modeFftRoot, LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                lv_obj_add_flag(g_modeFftRoot, LV_OBJ_FLAG_HIDDEN);
            }
        }

        if (g_activeUiMode == kUiModeFft && previousMode != kUiModeFft)
        {
            g_lastFftAnimMs = 0;
        }
    }

    void styleSimpleVuTrack(lv_obj_t *obj)
    {
        lv_obj_remove_style_all(obj);
        lv_obj_set_style_bg_color(obj, colorVuTrack(), 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(obj, colorMetricValueBorder(), 0);
        lv_obj_set_style_border_width(obj, 1, 0);
        lv_obj_set_style_radius(obj, 0, 0);
        lv_obj_set_style_pad_all(obj, 0, 0);
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    }

    void styleSimpleFrame(lv_obj_t *obj)
    {
        lv_obj_remove_style_all(obj);
        lv_obj_set_style_bg_color(obj, colorInstrumentFrameInner(), 0);
        lv_obj_set_style_bg_opa(obj, UI_LIGHT_MODE ? LV_OPA_20 : LV_OPA_30, 0);
        lv_obj_set_style_border_color(obj, colorInstrumentFrame(), 0);
        lv_obj_set_style_border_width(obj, 2, 0);
        lv_obj_set_style_outline_color(obj, colorMetricValueBorder(), 0);
        lv_obj_set_style_outline_width(obj, 1, 0);
        lv_obj_set_style_outline_pad(obj, 0, 0);
        lv_obj_set_style_radius(obj, 2, 0);
        lv_obj_set_style_pad_all(obj, 0, 0);
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    }

    void styleSimpleVuFill(lv_obj_t *obj, lv_color_t color)
    {
        lv_obj_remove_style_all(obj);
        lv_obj_set_style_bg_color(obj, color, 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(obj, 0, 0);
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    }

    void styleSimpleVuGlow(lv_obj_t *obj)
    {
        lv_obj_remove_style_all(obj);
        lv_obj_set_style_bg_color(obj, colorVuGlow(), 0);
        lv_obj_set_style_bg_opa(obj, UI_LIGHT_MODE ? LV_OPA_40 : LV_OPA_30, 0);
        lv_obj_set_style_radius(obj, 0, 0);
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    }

    void styleSimplePeakMarker(lv_obj_t *obj)
    {
        lv_obj_remove_style_all(obj);
        lv_obj_set_style_bg_color(obj, colorVuPeak(), 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(obj, 0, 0);
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    }

    void addSimpleVuEngraving(lv_obj_t *frame, const char *channelText, bool ticksOnRight)
    {
        if (frame == nullptr)
        {
            return;
        }

        lv_obj_t *channel = lv_label_create(frame);
        lv_obj_set_style_text_color(channel, colorEngraving(), 0);
        lv_obj_set_style_text_font(channel, &lv_font_montserrat_8, 0);
        lv_obj_set_style_text_opa(channel, UI_LIGHT_MODE ? LV_OPA_80 : LV_OPA_60, 0);
        lv_label_set_text(channel, channelText);
        lv_obj_align(channel, LV_ALIGN_TOP_MID, 0, 3);

        for (uint8_t i = 0; i < kSimpleVuTickCount; i++)
        {
            lv_obj_t *tick = lv_obj_create(frame);
            lv_obj_remove_style_all(tick);
            lv_obj_set_style_bg_color(tick, colorEngraving(), 0);
            lv_obj_set_style_bg_opa(tick, UI_LIGHT_MODE ? LV_OPA_40 : LV_OPA_20, 0);
            lv_obj_set_size(tick, 6, 1);

            const lv_coord_t y = 12 + (i * (kScreenHeight - 24)) / (kSimpleVuTickCount - 1);
            const lv_coord_t x = ticksOnRight ? (kSimpleVuFrameWidth - 8) : 2;
            lv_obj_set_pos(tick, x, y);
        }
    }

    void setSimpleVuBackground(lv_obj_t *track, lv_obj_t *greenBg, lv_obj_t *redBg)
    {
        if (track == nullptr || greenBg == nullptr || redBg == nullptr)
        {
            return;
        }

        const lv_coord_t totalHeight = lv_obj_get_height(track);
        const lv_coord_t innerWidth = lv_obj_get_width(track) - 2;
        const lv_coord_t usableHeight = totalHeight - 2;
        const lv_coord_t redHeight = kSimpleVuRedHeight > usableHeight ? usableHeight : kSimpleVuRedHeight;
        const lv_coord_t greenHeight = usableHeight - redHeight;

        lv_obj_set_size(greenBg, innerWidth, greenHeight);
        lv_obj_set_pos(greenBg, 1, totalHeight - 1 - greenHeight);

        lv_obj_set_size(redBg, innerWidth, redHeight);
        lv_obj_set_pos(redBg, 1, 1);
    }

    void styleSimpleGainLabel(lv_obj_t *obj)
    {
        lv_obj_remove_style_all(obj);
        lv_obj_set_style_text_color(obj, colorAxisText(), 0);
        lv_obj_set_style_text_font(obj, lv_theme_get_font_normal(obj), 0);
        lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_LEFT, 0);
    }

    void styleSimpleGainValue(lv_obj_t *obj)
    {
        lv_obj_remove_style_all(obj);
        lv_obj_set_style_text_color(obj, colorMetricValue(), 0);
        lv_obj_set_style_text_font(obj, lv_theme_get_font_normal(obj), 0);
        lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_style_bg_color(obj, colorMetricValueBg(), 0);
        lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(obj, colorMetricValueBorder(), 0);
        lv_obj_set_style_border_width(obj, 1, 0);
        lv_obj_set_style_radius(obj, 2, 0);
        lv_obj_set_style_pad_left(obj, 4, 0);
        lv_obj_set_style_pad_right(obj, 4, 0);
        lv_obj_set_style_pad_top(obj, 1, 0);
        lv_obj_set_style_pad_bottom(obj, 1, 0);
    }

    void positionSimpleGainWidgets()
    {
        if (g_simpleSampleRate == nullptr ||
            g_simpleOutGainLabel == nullptr || g_simpleOutGainValue == nullptr ||
            g_simpleInGainLabel == nullptr || g_simpleInGainValue == nullptr)
        {
            return;
        }

        lv_obj_update_layout(g_simpleSampleRate);

        const lv_coord_t rowLeft = kSimpleCenterFrameX + kSimpleTextInset;
        const lv_coord_t rowWidth = kSimpleCenterFrameWidth - (2 * kSimpleTextInset);
        const lv_coord_t labelWidth = 68;
        const lv_coord_t valueWidth = rowWidth - labelWidth - 6;
        const lv_coord_t rowHeight = 22;
        const lv_coord_t firstRowY = lv_obj_get_y(g_simpleSampleRate) + lv_obj_get_height(g_simpleSampleRate) + 12;

        lv_obj_set_pos(g_simpleOutGainLabel, rowLeft, firstRowY);
        lv_obj_set_size(g_simpleOutGainLabel, labelWidth, rowHeight);
        lv_obj_set_pos(g_simpleOutGainValue, rowLeft + labelWidth + 6, firstRowY);
        lv_obj_set_size(g_simpleOutGainValue, valueWidth, rowHeight);

        const lv_coord_t secondRowY = firstRowY + rowHeight + 8;
        lv_obj_set_pos(g_simpleInGainLabel, rowLeft, secondRowY);
        lv_obj_set_size(g_simpleInGainLabel, labelWidth, rowHeight);
        lv_obj_set_pos(g_simpleInGainValue, rowLeft + labelWidth + 6, secondRowY);
        lv_obj_set_size(g_simpleInGainValue, valueWidth, rowHeight);
    }

    uint16_t applyVuBallistic(uint16_t currentLevel, uint16_t targetLevel, uint32_t dtMs)
    {
        if (targetLevel == currentLevel)
        {
            return currentLevel;
        }

        const uint32_t tau = (targetLevel > currentLevel) ? kVuAttackMs : kVuReleaseMs;
        const float alpha = float(dtMs) / float(dtMs + tau);
        const float next = float(currentLevel) + (float(targetLevel) - float(currentLevel)) * alpha;
        if (next < 0.0f)
        {
            return 0;
        }
        if (next > 65535.0f)
        {
            return 65535;
        }
        return static_cast<uint16_t>(next + 0.5f);
    }

    void updateVuPeak(uint16_t displayedLevel, uint16_t &peakLevel, uint32_t &holdUntilMs, uint32_t nowMs, uint32_t dtMs)
    {
        if (displayedLevel >= peakLevel)
        {
            peakLevel = displayedLevel;
            holdUntilMs = nowMs + kVuPeakHoldMs;
            return;
        }

        if (nowMs < holdUntilMs)
        {
            return;
        }

        const uint32_t drop = (kVuPeakFallPerSecond * dtMs) / 1000U;
        if (drop >= peakLevel)
        {
            peakLevel = displayedLevel;
        }
        else
        {
            peakLevel = static_cast<uint16_t>(peakLevel - drop);
            if (peakLevel < displayedLevel)
            {
                peakLevel = displayedLevel;
            }
        }
    }

    void setSimpleVuChannelLevel(lv_obj_t *track, lv_obj_t *greenFill, lv_obj_t *redFill, lv_obj_t *glowFill, lv_obj_t *peakMarker, uint16_t level, uint16_t peakLevel)
    {
        if (track == nullptr || greenFill == nullptr || redFill == nullptr || glowFill == nullptr || peakMarker == nullptr)
        {
            return;
        }

        const lv_coord_t totalHeight = lv_obj_get_height(track);
        const lv_coord_t innerWidth = lv_obj_get_width(track) - 2;
        const lv_coord_t usableHeight = totalHeight - 2;
        const lv_coord_t redZoneStart = usableHeight - kSimpleVuRedHeight;
        const lv_coord_t fillHeight = static_cast<lv_coord_t>((static_cast<uint32_t>(level) * usableHeight + 32767U) / 65535U);
        const lv_coord_t redHeight = fillHeight > redZoneStart ? fillHeight - redZoneStart : 0;
        const lv_coord_t greenHeight = fillHeight - redHeight;

        lv_obj_set_size(greenFill, innerWidth, greenHeight);
        lv_obj_set_pos(greenFill, 1, totalHeight - 1 - greenHeight);

        lv_obj_set_size(redFill, innerWidth, redHeight);
        lv_obj_set_pos(redFill, 1, totalHeight - 1 - fillHeight);

        const lv_coord_t glowHeight = fillHeight > 0 ? (fillHeight < 6 ? fillHeight : 6) : 0;
        lv_obj_set_size(glowFill, innerWidth, glowHeight);
        lv_obj_set_pos(glowFill, 1, totalHeight - 1 - fillHeight);

        const lv_coord_t peakFillHeight = static_cast<lv_coord_t>((static_cast<uint32_t>(peakLevel) * usableHeight + 32767U) / 65535U);
        lv_coord_t peakY = totalHeight - 1 - peakFillHeight;
        if (peakY < 1)
        {
            peakY = 1;
        }
        if (peakY > totalHeight - 1 - kSimplePeakMarkerHeight)
        {
            peakY = totalHeight - 1 - kSimplePeakMarkerHeight;
        }
        lv_obj_set_size(peakMarker, innerWidth, kSimplePeakMarkerHeight);
        lv_obj_set_pos(peakMarker, 1, peakY);

        if (peakFillHeight >= redZoneStart)
        {
            lv_obj_set_style_bg_color(peakMarker, colorVuRed(), 0);
        }
        else
        {
            lv_obj_set_style_bg_color(peakMarker, colorVuPeak(), 0);
        }
    }

    void updateSimpleVuMeters(uint16_t leftLevel, uint16_t rightLevel)
    {
        const uint32_t now = millis();
        if (g_lastVuAnimMs == 0)
        {
            g_lastVuAnimMs = now;
            g_vuDisplayedLeft = leftLevel;
            g_vuDisplayedRight = rightLevel;
            g_vuPeakLeft = leftLevel;
            g_vuPeakRight = rightLevel;
            g_vuPeakHoldUntilLeft = now + kVuPeakHoldMs;
            g_vuPeakHoldUntilRight = now + kVuPeakHoldMs;
        }

        uint32_t dt = now - g_lastVuAnimMs;
        if (dt > 100)
        {
            dt = 100;
        }
        g_lastVuAnimMs = now;

        g_vuDisplayedLeft = applyVuBallistic(g_vuDisplayedLeft, leftLevel, dt);
        g_vuDisplayedRight = applyVuBallistic(g_vuDisplayedRight, rightLevel, dt);

        updateVuPeak(g_vuDisplayedLeft, g_vuPeakLeft, g_vuPeakHoldUntilLeft, now, dt);
        updateVuPeak(g_vuDisplayedRight, g_vuPeakRight, g_vuPeakHoldUntilRight, now, dt);

        setSimpleVuChannelLevel(g_simpleLeftVu, g_simpleLeftVuGreen, g_simpleLeftVuRed, g_simpleLeftVuGlow, g_simpleLeftPeak, g_vuDisplayedLeft, g_vuPeakLeft);
        setSimpleVuChannelLevel(g_simpleRightVu, g_simpleRightVuGreen, g_simpleRightVuRed, g_simpleRightVuGlow, g_simpleRightPeak, g_vuDisplayedRight, g_vuPeakRight);
    }

    uint16_t fftBinToLevel(uint8_t bin)
    {
        return static_cast<uint16_t>((static_cast<uint16_t>(bin) << 8) | bin);
    }

    void updateFftMeters(const uint8_t *leftBins, const uint8_t *rightBins)
    {
        if (leftBins == nullptr || rightBins == nullptr)
        {
            return;
        }

        const uint32_t now = millis();
        if (g_lastFftAnimMs == 0)
        {
            g_lastFftAnimMs = now;
            for (uint8_t i = 0; i < kFftBinCount; i++)
            {
                const uint16_t averagedLevel = fftBinToLevel(static_cast<uint8_t>((static_cast<uint16_t>(leftBins[i]) + static_cast<uint16_t>(rightBins[i])) / 2U));
                g_fftDisplayed[i] = averagedLevel;
                g_fftPeakLevel[i] = averagedLevel;
                g_fftPeakHoldUntil[i] = now + kVuPeakHoldMs;
            }
        }

        uint32_t dt = now - g_lastFftAnimMs;
        if (dt > 100)
        {
            dt = 100;
        }
        g_lastFftAnimMs = now;

        for (uint8_t i = 0; i < kFftBinCount; i++)
        {
            const uint8_t averagedBin = static_cast<uint8_t>((static_cast<uint16_t>(leftBins[i]) + static_cast<uint16_t>(rightBins[i])) / 2U);
            const uint16_t targetLevel = fftBinToLevel(averagedBin);

            g_fftDisplayed[i] = applyVuBallistic(g_fftDisplayed[i], targetLevel, dt);
            updateVuPeak(g_fftDisplayed[i], g_fftPeakLevel[i], g_fftPeakHoldUntil[i], now, dt);

            setSimpleVuChannelLevel(g_fftTrack[i], g_fftGreen[i], g_fftRed[i], g_fftGlow[i], g_fftPeak[i], g_fftDisplayed[i], g_fftPeakLevel[i]);
        }
    }

    void clampAndWriteSeries(lv_chart_series_t *series, const float *values, size_t count)
    {
        for (size_t i = 0; i < kChartPoints; i++)
        {
            if (i >= count || values == nullptr)
            {
                series->y_points[i] = LV_CHART_POINT_NONE;
                continue;
            }

            int16_t point = static_cast<int16_t>(values[i]);
            point = static_cast<int16_t>(values[i] * kChartDbScale);
            if (point < kChartMinDb * kChartDbScale)
            {
                point = kChartMinDb * kChartDbScale;
            }
            if (point > kChartMaxDb * kChartDbScale)
            {
                point = kChartMaxDb * kChartDbScale;
            }
            series->y_points[i] = point;
        }
    }

    lv_coord_t chartXFromFrequency(float frequency)
    {
        const float clamped = frequency < kMinGraphFrequency ? kMinGraphFrequency : (frequency > kMaxGraphFrequency ? kMaxGraphFrequency : frequency);
        const float minLog = log10f(kMinGraphFrequency);
        const float maxLog = log10f(kMaxGraphFrequency);
        const float ratio = (log10f(clamped) - minLog) / (maxLog - minLog);
        return kChartX + static_cast<lv_coord_t>(ratio * float(kChartWidth - 1));
    }

    void positionFrequencyLabel(float frequency)
    {
        if (g_valueFreq == nullptr)
        {
            return;
        }

        const lv_coord_t axisY = kChartY + kChartHeight + 14;
        const lv_coord_t caretY = axisY - 8;
        const lv_coord_t centerX = chartXFromFrequency(frequency);
        const lv_coord_t labelWidth = lv_obj_get_width(g_valueFreq);
        lv_coord_t x = centerX - (labelWidth / 2);

        if (x < kChartX)
        {
            x = kChartX;
        }
        const lv_coord_t maxX = kChartX + kChartWidth - labelWidth;
        if (x > maxX)
        {
            x = maxX;
        }

        lv_obj_set_pos(g_valueFreq, x, axisY);

        if (g_freqCaret != nullptr)
        {
            const lv_coord_t caretWidth = lv_obj_get_width(g_freqCaret);
            lv_coord_t caretX = centerX - (caretWidth / 2);
            const lv_coord_t minCaretX = kChartX;
            const lv_coord_t maxCaretX = kChartX + kChartWidth - caretWidth;
            if (caretX < minCaretX)
            {
                caretX = minCaretX;
            }
            if (caretX > maxCaretX)
            {
                caretX = maxCaretX;
            }
            lv_obj_set_pos(g_freqCaret, caretX, caretY);
        }
    }

} // namespace

void ui_lvgl_init(TFT_eSPI &tft)
{
    g_tft = &tft;

    lv_init();
    lv_disp_draw_buf_init(&g_drawBuf, g_buf1, nullptr, kScreenWidth * kDrawBufferLines);

    static lv_disp_drv_t dispDrv;
    lv_disp_drv_init(&dispDrv);
    dispDrv.hor_res = kScreenWidth;
    dispDrv.ver_res = kScreenHeight;
    dispDrv.flush_cb = tftFlush;
    dispDrv.draw_buf = &g_drawBuf;
    lv_disp_drv_register(&dispDrv);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, colorScreenBg(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    g_modeDetailedRoot = lv_obj_create(scr);
    lv_obj_remove_style_all(g_modeDetailedRoot);
    lv_obj_set_size(g_modeDetailedRoot, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(g_modeDetailedRoot, 0, 0);

    g_modeSimpleRoot = lv_obj_create(scr);
    lv_obj_remove_style_all(g_modeSimpleRoot);
    lv_obj_set_size(g_modeSimpleRoot, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(g_modeSimpleRoot, 0, 0);
    lv_obj_add_flag(g_modeSimpleRoot, LV_OBJ_FLAG_HIDDEN);

    g_modeFftRoot = lv_obj_create(scr);
    lv_obj_remove_style_all(g_modeFftRoot);
    lv_obj_set_size(g_modeFftRoot, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(g_modeFftRoot, 0, 0);
    lv_obj_add_flag(g_modeFftRoot, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *fftFrame = lv_obj_create(g_modeFftRoot);
    styleSimpleFrame(fftFrame);
    lv_obj_set_pos(fftFrame, kFftFrameX, kFftFrameY);
    lv_obj_set_size(fftFrame, kFftFrameWidth, kFftFrameHeight);

    lv_obj_t *fftTitle = lv_label_create(g_modeFftRoot);
    lv_obj_set_style_text_color(fftTitle, colorAxisText(), 0);
    lv_obj_set_style_text_font(fftTitle, &lv_font_montserrat_8, 0);
    lv_label_set_text(fftTitle, "FFT");
    lv_obj_set_pos(fftTitle, kFftFrameX + 6, kFftFrameY + 2);

    const lv_coord_t fftBarsAreaWidth = kFftFrameWidth - (2 * kFftTrackX);
    const lv_coord_t barWidth = (fftBarsAreaWidth - ((kFftBinCount - 1) * kFftBarGroupGap)) / kFftBinCount;

    for (uint8_t i = 0; i < kFftBinCount; i++)
    {
        const lv_coord_t x = kFftFrameX + kFftTrackX + i * (barWidth + kFftBarGroupGap);
        const lv_coord_t y = kFftFrameY + kFftTrackY;

        g_fftTrack[i] = lv_obj_create(g_modeFftRoot);
        styleSimpleVuTrack(g_fftTrack[i]);
        lv_obj_set_pos(g_fftTrack[i], x, y);
        lv_obj_set_size(g_fftTrack[i], barWidth, kFftTrackHeight);

        g_fftBgGreen[i] = lv_obj_create(g_fftTrack[i]);
        styleSimpleVuFill(g_fftBgGreen[i], colorVuTrack());
        g_fftBgRed[i] = lv_obj_create(g_fftTrack[i]);
        styleSimpleVuFill(g_fftBgRed[i], colorVuTrackTop());
        g_fftGreen[i] = lv_obj_create(g_fftTrack[i]);
        styleSimpleVuFill(g_fftGreen[i], colorVuGreen());
        g_fftRed[i] = lv_obj_create(g_fftTrack[i]);
        styleSimpleVuFill(g_fftRed[i], colorVuRed());
        g_fftGlow[i] = lv_obj_create(g_fftTrack[i]);
        styleSimpleVuGlow(g_fftGlow[i]);
        g_fftPeak[i] = lv_obj_create(g_fftTrack[i]);
        styleSimplePeakMarker(g_fftPeak[i]);

        setSimpleVuBackground(g_fftTrack[i], g_fftBgGreen[i], g_fftBgRed[i]);
        setSimpleVuChannelLevel(g_fftTrack[i], g_fftGreen[i], g_fftRed[i], g_fftGlow[i], g_fftPeak[i], 0, 0);
    }

    lv_obj_t *simpleLeftVuFrame = lv_obj_create(g_modeSimpleRoot);
    styleSimpleFrame(simpleLeftVuFrame);
    lv_obj_set_pos(simpleLeftVuFrame, 0, 0);
    lv_obj_set_size(simpleLeftVuFrame, kSimpleVuFrameWidth, kScreenHeight);
    addSimpleVuEngraving(simpleLeftVuFrame, "L", true);

    lv_obj_t *simpleRightVuFrame = lv_obj_create(g_modeSimpleRoot);
    styleSimpleFrame(simpleRightVuFrame);
    lv_obj_set_pos(simpleRightVuFrame, kScreenWidth - kSimpleVuFrameWidth, 0);
    lv_obj_set_size(simpleRightVuFrame, kSimpleVuFrameWidth, kScreenHeight);
    addSimpleVuEngraving(simpleRightVuFrame, "R", false);

    lv_obj_t *simpleCenterFrame = lv_obj_create(g_modeSimpleRoot);
    styleSimpleFrame(simpleCenterFrame);
    lv_obj_set_pos(simpleCenterFrame, kSimpleCenterFrameX, 0);
    lv_obj_set_size(simpleCenterFrame, kSimpleCenterFrameWidth, kScreenHeight);

    for (lv_coord_t x = 6; x < (kSimpleCenterFrameWidth - 4); x += 16)
    {
        lv_obj_t *grain = lv_obj_create(simpleCenterFrame);
        lv_obj_remove_style_all(grain);
        lv_obj_set_style_bg_color(grain, colorMetricValueBorder(), 0);
        lv_obj_set_style_bg_opa(grain, UI_LIGHT_MODE ? LV_OPA_20 : LV_OPA_10, 0);
        lv_obj_set_size(grain, 1, kScreenHeight - 6);
        lv_obj_set_pos(grain, x, 3);
    }

    g_simpleLeftVu = lv_obj_create(g_modeSimpleRoot);
    styleSimpleVuTrack(g_simpleLeftVu);
    lv_obj_set_pos(g_simpleLeftVu, kSimpleVuFramePad, 0);
    lv_obj_set_size(g_simpleLeftVu, kSimpleVuWidth, kScreenHeight);

    g_simpleLeftVuBgGreen = lv_obj_create(g_simpleLeftVu);
    styleSimpleVuFill(g_simpleLeftVuBgGreen, colorVuTrack());

    g_simpleLeftVuBgRed = lv_obj_create(g_simpleLeftVu);
    styleSimpleVuFill(g_simpleLeftVuBgRed, colorVuTrackTop());

    g_simpleLeftVuGreen = lv_obj_create(g_simpleLeftVu);
    styleSimpleVuFill(g_simpleLeftVuGreen, colorVuGreen());
    lv_obj_set_size(g_simpleLeftVuGreen, kSimpleVuWidth - 2, 0);
    lv_obj_set_pos(g_simpleLeftVuGreen, 1, kScreenHeight - 1);

    g_simpleLeftVuRed = lv_obj_create(g_simpleLeftVu);
    styleSimpleVuFill(g_simpleLeftVuRed, colorVuRed());
    lv_obj_set_size(g_simpleLeftVuRed, kSimpleVuWidth - 2, 0);
    lv_obj_set_pos(g_simpleLeftVuRed, 1, kScreenHeight - 1);

    g_simpleLeftVuGlow = lv_obj_create(g_simpleLeftVu);
    styleSimpleVuGlow(g_simpleLeftVuGlow);
    lv_obj_set_size(g_simpleLeftVuGlow, kSimpleVuWidth - 2, 0);
    lv_obj_set_pos(g_simpleLeftVuGlow, 1, kScreenHeight - 1);

    g_simpleLeftPeak = lv_obj_create(g_simpleLeftVu);
    styleSimplePeakMarker(g_simpleLeftPeak);
    lv_obj_set_size(g_simpleLeftPeak, kSimpleVuWidth - 2, kSimplePeakMarkerHeight);
    lv_obj_set_pos(g_simpleLeftPeak, 1, kScreenHeight - 2);

    g_simpleRightVu = lv_obj_create(g_modeSimpleRoot);
    styleSimpleVuTrack(g_simpleRightVu);
    lv_obj_set_pos(g_simpleRightVu, kScreenWidth - kSimpleVuFrameWidth + kSimpleVuFramePad, 0);
    lv_obj_set_size(g_simpleRightVu, kSimpleVuWidth, kScreenHeight);

    g_simpleRightVuBgGreen = lv_obj_create(g_simpleRightVu);
    styleSimpleVuFill(g_simpleRightVuBgGreen, colorVuTrack());

    g_simpleRightVuBgRed = lv_obj_create(g_simpleRightVu);
    styleSimpleVuFill(g_simpleRightVuBgRed, colorVuTrackTop());

    g_simpleRightVuGreen = lv_obj_create(g_simpleRightVu);
    styleSimpleVuFill(g_simpleRightVuGreen, colorVuGreen());
    lv_obj_set_size(g_simpleRightVuGreen, kSimpleVuWidth - 2, 0);
    lv_obj_set_pos(g_simpleRightVuGreen, 1, kScreenHeight - 1);

    g_simpleRightVuRed = lv_obj_create(g_simpleRightVu);
    styleSimpleVuFill(g_simpleRightVuRed, colorVuRed());
    lv_obj_set_size(g_simpleRightVuRed, kSimpleVuWidth - 2, 0);
    lv_obj_set_pos(g_simpleRightVuRed, 1, kScreenHeight - 1);

    g_simpleRightVuGlow = lv_obj_create(g_simpleRightVu);
    styleSimpleVuGlow(g_simpleRightVuGlow);
    lv_obj_set_size(g_simpleRightVuGlow, kSimpleVuWidth - 2, 0);
    lv_obj_set_pos(g_simpleRightVuGlow, 1, kScreenHeight - 1);

    g_simpleRightPeak = lv_obj_create(g_simpleRightVu);
    styleSimplePeakMarker(g_simpleRightPeak);
    lv_obj_set_size(g_simpleRightPeak, kSimpleVuWidth - 2, kSimplePeakMarkerHeight);
    lv_obj_set_pos(g_simpleRightPeak, 1, kScreenHeight - 2);

    // Y-axis labels aligned to chart max, mid and min values.
    lv_obj_t *yMaxLabel = lv_label_create(g_modeDetailedRoot);
    lv_label_set_text(yMaxLabel, "15");
    lv_obj_set_style_text_color(yMaxLabel, colorAxisText(), 0);
    lv_obj_set_style_text_font(yMaxLabel, &lv_font_montserrat_8, 0);
    lv_obj_set_style_text_align(yMaxLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(yMaxLabel, 4, kChartY - 2);
    lv_obj_set_size(yMaxLabel, kChartX - 8, LV_SIZE_CONTENT);

    lv_obj_t *yMidLabel = lv_label_create(g_modeDetailedRoot);
    lv_label_set_text(yMidLabel, "0");
    lv_obj_set_style_text_color(yMidLabel, colorAxisText(), 0);
    lv_obj_set_style_text_font(yMidLabel, &lv_font_montserrat_8, 0);
    lv_obj_set_style_text_align(yMidLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(yMidLabel, 4, kChartY + (kChartHeight / 2) - 4);
    lv_obj_set_size(yMidLabel, kChartX - 8, LV_SIZE_CONTENT);

    lv_obj_t *yMinLabel = lv_label_create(g_modeDetailedRoot);
    lv_label_set_text(yMinLabel, "-15");
    lv_obj_set_style_text_color(yMinLabel, colorAxisText(), 0);
    lv_obj_set_style_text_font(yMinLabel, &lv_font_montserrat_8, 0);
    lv_obj_set_style_text_align(yMinLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(yMinLabel, 4, kChartY + kChartHeight - 8);
    lv_obj_set_size(yMinLabel, kChartX - 8, LV_SIZE_CONTENT);

    g_chart = lv_chart_create(g_modeDetailedRoot);
    lv_obj_set_pos(g_chart, kChartX, kChartY);
    lv_obj_set_size(g_chart, kChartWidth, kChartHeight);
    lv_chart_set_type(g_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(g_chart, kChartPoints);
    lv_chart_set_range(g_chart, LV_CHART_AXIS_PRIMARY_Y, kChartMinDb * kChartDbScale, kChartMaxDb * kChartDbScale);
    lv_obj_set_style_bg_color(g_chart, colorChartBg(), 0);
    lv_obj_set_style_bg_opa(g_chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_chart, colorChartBorder(), 0);
    lv_obj_set_style_border_width(g_chart, 1, 0);
    lv_obj_set_style_radius(g_chart, 0, 0);
    lv_obj_set_style_pad_all(g_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(g_chart, true, LV_PART_ITEMS);
    lv_obj_set_style_line_width(g_chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(g_chart, 0, LV_PART_INDICATOR);
    lv_chart_set_div_line_count(g_chart, 7, 5);
    lv_obj_set_style_line_color(g_chart, lv_color_hex(0x808080), LV_PART_MAIN);




    // X-axis labels aligned with decade grid lines: 2, 20, 200, 2000, 20000.
    const char *xLabels[5] = {"2", "20", "200", "2000", "20000"};
    for (int i = 0; i < 5; i++)
    {
        lv_obj_t *xLabel = lv_label_create(g_modeDetailedRoot);
        lv_label_set_text(xLabel, xLabels[i]);
        lv_obj_set_style_text_color(xLabel, colorAxisText(), 0);
        lv_obj_set_style_text_font(xLabel, &lv_font_montserrat_8, 0);

        // Place labels at 0%, 25%, 50%, 75%, 100% of chart width.
        lv_coord_t x = kChartX + static_cast<lv_coord_t>((i * (kChartWidth - 1)) / 4);
        lv_obj_update_layout(xLabel);
        lv_coord_t w = lv_obj_get_width(xLabel);

        if (i == 0)
        {
            lv_obj_set_pos(xLabel, x, kChartY + kChartHeight + 2);
        }
        else if (i == 4)
        {
            lv_obj_set_pos(xLabel, x - w, kChartY + kChartHeight + 2);
        }
        else
        {
            lv_obj_set_pos(xLabel, x - (w / 2), kChartY + kChartHeight + 2);
        }
    }

    g_seriesSelected = lv_chart_add_series(g_chart, colorChartSelected(), LV_CHART_AXIS_PRIMARY_Y);
    g_seriesCombined = lv_chart_add_series(g_chart, colorChartCombined(), LV_CHART_AXIS_PRIMARY_Y);

    lv_obj_t *panel = lv_obj_create(g_modeDetailedRoot);
    lv_obj_set_pos(panel, kPanelX, kPanelY);
    lv_obj_set_size(panel, kPanelWidth, kPanelHeight);
    lv_obj_set_style_bg_color(panel, colorMetricPanel(), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(panel, colorMetricBorder(), 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_radius(panel, 2, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);

    lv_obj_t *labelFs = lv_label_create(g_modeDetailedRoot);
    lv_label_set_text(labelFs, "FSampl");
    lv_obj_set_pos(labelFs, kPanelX + 4, 21);
    lv_obj_set_size(labelFs, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelFs);

    lv_obj_t *labelM = lv_label_create(g_modeDetailedRoot);
    lv_label_set_text(labelM, "Pre Gain");
    lv_obj_set_pos(labelM, kPanelX + 4, 53);
    lv_obj_set_size(labelM, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelM);

    lv_obj_t *labelF = lv_label_create(g_modeDetailedRoot);
    lv_label_set_text(labelF, "Filt Gain");
    lv_obj_set_pos(labelF, kPanelX + 4, 85);
    lv_obj_set_size(labelF, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelF);

    lv_obj_t *labelQ = lv_label_create(g_modeDetailedRoot);
    lv_label_set_text(labelQ, "Q");
    lv_obj_set_pos(labelQ, kPanelX + 4, 117);
    lv_obj_set_size(labelQ, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelQ);

    lv_obj_t *labelVol = lv_label_create(g_modeDetailedRoot);
    lv_label_set_text(labelVol, "Volume");
    lv_obj_set_pos(labelVol, kPanelX + 4, 147);
    lv_obj_set_size(labelVol, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelVol);

    g_valueFs = lv_label_create(g_modeDetailedRoot);
    lv_obj_set_pos(g_valueFs, kPanelX + 4, 32);
    lv_obj_set_size(g_valueFs, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueFs, "----");
    styleMetricValue(g_valueFs);

    g_valueM = lv_label_create(g_modeDetailedRoot);
    lv_obj_set_pos(g_valueM, kPanelX + 4, 64);
    lv_obj_set_size(g_valueM, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueM, "0.0");
    styleMetricValue(g_valueM);

    g_valueF = lv_label_create(g_modeDetailedRoot);
    lv_obj_set_pos(g_valueF, kPanelX + 4, 96);
    lv_obj_set_size(g_valueF, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueF, "0.0");
    styleMetricValue(g_valueF);

    g_valueQ = lv_label_create(g_modeDetailedRoot);
    lv_obj_set_pos(g_valueQ, kPanelX + 4, 128);
    lv_obj_set_size(g_valueQ, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueQ, "0.0");
    styleMetricValue(g_valueQ);

    g_valueVol = lv_label_create(g_modeDetailedRoot);
    lv_obj_set_pos(g_valueVol, kPanelX + 4, 158);
    lv_obj_set_size(g_valueVol, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueVol, "0.0");
    styleMetricValue(g_valueVol);

    g_valueFreq = lv_label_create(g_modeDetailedRoot);
    lv_obj_set_style_text_color(g_valueFreq, colorAxisText(), 0);
    lv_obj_set_style_text_font(g_valueFreq, &lv_font_montserrat_8, 0);
    lv_label_set_text(g_valueFreq, "--");
    lv_obj_update_layout(g_valueFreq);

    g_freqCaret = lv_label_create(g_modeDetailedRoot);
    lv_obj_set_style_text_color(g_freqCaret, colorAxisText(), 0);
    lv_obj_set_style_text_font(g_freqCaret, &lv_font_montserrat_8, 0);
    lv_label_set_text(g_freqCaret, "^");
    lv_obj_update_layout(g_freqCaret);

    positionFrequencyLabel(kMinGraphFrequency);

    // Frames around footer groups: filter type badges and filter band badges.
    lv_obj_t *typeFrame = lv_obj_create(g_modeDetailedRoot);
    lv_obj_set_pos(typeFrame, 8, kBottomBadgeY - 2);
    lv_obj_set_size(typeFrame, 165, 23);
    lv_obj_set_style_bg_opa(typeFrame, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(typeFrame, colorFooterFrame(), 0);
    lv_obj_set_style_border_width(typeFrame, 1, 0);
    lv_obj_set_style_radius(typeFrame, 2, 0);
    lv_obj_set_style_pad_all(typeFrame, 0, 0);

    lv_obj_t *bandFrame = lv_obj_create(g_modeDetailedRoot);
    lv_obj_set_pos(bandFrame, 186, kBottomBadgeY - 2);
    lv_obj_set_size(bandFrame, 126, 23);
    lv_obj_set_style_bg_opa(bandFrame, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(bandFrame, colorFooterFrame(), 0);
    lv_obj_set_style_border_width(bandFrame, 1, 0);
    lv_obj_set_style_radius(bandFrame, 2, 0);
    lv_obj_set_style_pad_all(bandFrame, 0, 0);

    g_filterLabels[0] = lv_label_create(g_modeDetailedRoot);
    g_filterLabels[1] = lv_label_create(g_modeDetailedRoot);
    g_filterLabels[2] = lv_label_create(g_modeDetailedRoot);

    makeBadge(g_filterLabels[0], "LoSh", 12, kBottomBadgeY);
    makeBadge(g_filterLabels[1], "HiSh", 68, kBottomBadgeY);
    makeBadge(g_filterLabels[2], "Peak", 124, kBottomBadgeY);

    for (int i = 0; i < FILTER_BANDS; i++)
    {
        g_indexLabels[i] = lv_label_create(g_modeDetailedRoot);
        char indexBuf[4];
        snprintf(indexBuf, sizeof(indexBuf), "%d", i + 1);
        makeBadge(g_indexLabels[i], indexBuf, 190 + i * 25, kBottomBadgeY);
    }

    g_simpleSampleRate = lv_label_create(g_modeSimpleRoot);
    lv_obj_set_style_text_color(g_simpleSampleRate, colorMetricValue(), 0);
    lv_obj_set_style_text_font(g_simpleSampleRate, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(g_simpleSampleRate, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(g_simpleSampleRate, "----");
    lv_obj_set_width(g_simpleSampleRate, kSimpleCenterFrameWidth - (2 * kSimpleTextInset));
    lv_obj_align(g_simpleSampleRate, LV_ALIGN_CENTER, 0, -20);

    g_simpleOutGainLabel = lv_label_create(g_modeSimpleRoot);
    styleSimpleGainLabel(g_simpleOutGainLabel);
    lv_label_set_text(g_simpleOutGainLabel, "Out gain");

    g_simpleOutGainValue = lv_label_create(g_modeSimpleRoot);
    styleSimpleGainValue(g_simpleOutGainValue);
    lv_label_set_text(g_simpleOutGainValue, "0.0 dB");

    g_simpleInGainLabel = lv_label_create(g_modeSimpleRoot);
    styleSimpleGainLabel(g_simpleInGainLabel);
    lv_label_set_text(g_simpleInGainLabel, "In gain");

    g_simpleInGainValue = lv_label_create(g_modeSimpleRoot);
    styleSimpleGainValue(g_simpleInGainValue);
    lv_label_set_text(g_simpleInGainValue, "0.0 dB");

    positionSimpleGainWidgets();

    setSimpleVuBackground(g_simpleLeftVu, g_simpleLeftVuBgGreen, g_simpleLeftVuBgRed);
    setSimpleVuBackground(g_simpleRightVu, g_simpleRightVuBgGreen, g_simpleRightVuBgRed);
    updateSimpleVuMeters(0, 0);

    setUiMode(kUiModeSimple);

    g_lastTickMs = millis();
}

void ui_lvgl_task()
{
    uint32_t now = millis();
    uint32_t elapsed = now - g_lastTickMs;
    g_lastTickMs = now;

    lv_tick_inc(elapsed);
    lv_timer_handler();
}

void ui_lvgl_update(const UiData &data, const float *selectedResponse, const float *combinedResponse, size_t pointCount)
{
    setUiMode(data.uiMode);

    if (g_activeUiMode == kUiModeFft)
    {
        updateFftMeters(data.fftLeft, data.fftRight);
        return;
    }

    if (g_activeUiMode == kUiModeSimple)
    {
        if (g_simpleSampleRate != nullptr)
        {
            char srBuf[24];
            formatSampleRateKhz(srBuf, sizeof(srBuf), data.sampleRate);

            char srText[32];
            if (data.sampleRate == 0)
            {
                snprintf(srText, sizeof(srText), "----");
            }
            else
            {
                snprintf(srText, sizeof(srText), "%s", srBuf);
            }
            lv_label_set_text(g_simpleSampleRate, srText);
            lv_obj_align(g_simpleSampleRate, LV_ALIGN_CENTER, 0, -16);
        }

        positionSimpleGainWidgets();

        if (g_simpleOutGainValue != nullptr)
        {
            char volText[32];
            if (data.outputGain <= -100.0f)
            {
                snprintf(volText, sizeof(volText), "%4.0f dB", data.outputGain);
            }
            else
            {
                snprintf(volText, sizeof(volText), "%4.1f dB", data.outputGain);
            }
            lv_label_set_text(g_simpleOutGainValue, volText);
        }

        if (g_simpleInGainValue != nullptr)
        {
            char inText[32];
            if (data.inputGain <= -100.0f)
            {
                snprintf(inText, sizeof(inText), "%4.0f dB", data.inputGain);
            }
            else
            {
                snprintf(inText, sizeof(inText), "%4.1f dB", data.inputGain);
            }
            lv_label_set_text(g_simpleInGainValue, inText);
        }

        updateSimpleVuMeters(data.vuLeft, data.vuRight);

        return;
    }

    if (g_chart == nullptr || g_seriesSelected == nullptr || g_seriesCombined == nullptr)
    {
        return;
    }

    char srBuf[24];
    formatSampleRateKhz(srBuf, sizeof(srBuf), data.sampleRate);
    lv_label_set_text(g_valueFs, srBuf);

    setLabelValue(g_valueM, "%4.1f", data.inputGain);
    setLabelValue(g_valueF, "%4.1f", data.filterGain);
    setLabelValue(g_valueQ, "%4.1f", data.q);

    if (data.outputGain <= -100.0f)
    {
        setLabelValue(g_valueVol, "%4.0f", data.outputGain);
    }
    else
    {
        setLabelValue(g_valueVol, "%4.1f", data.outputGain);
    }

    setLabelValue(g_valueFreq, "%.0f", data.frequency);
    lv_obj_update_layout(g_valueFreq);
    positionFrequencyLabel(data.frequency);

    setBadgeState(g_filterLabels[0], data.filterType == 0);
    setBadgeState(g_filterLabels[1], data.filterType == 1);
    setBadgeState(g_filterLabels[2], data.filterType == 2);

    for (int i = 0; i < FILTER_BANDS; i++)
    {
        setBadgeState(g_indexLabels[i], data.filterIndex == i);
    }

    bool combinedMode = (data.displayMode == DISPLAY_MODE_COMBINED);

    clampAndWriteSeries(g_seriesSelected, selectedResponse, pointCount);
    if (combinedMode)
    {
        clampAndWriteSeries(g_seriesCombined, combinedResponse, pointCount);
    }
    else
    {
        clampAndWriteSeries(g_seriesCombined, nullptr, 0);
    }

    lv_chart_refresh(g_chart);
}

void ui_lvgl_update_vu(uint16_t leftLevel, uint16_t rightLevel)
{
    if (g_activeUiMode != kUiModeSimple)
    {
        return;
    }

    updateSimpleVuMeters(leftLevel, rightLevel);
}
