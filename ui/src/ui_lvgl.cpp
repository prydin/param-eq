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
    constexpr float kMinGraphFrequency = 2.0f;
    constexpr float kMaxGraphFrequency = 20000.0f;
    constexpr size_t kChartPoints = 100;
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

    TFT_eSPI *g_tft = nullptr;
    lv_disp_draw_buf_t g_drawBuf;
    lv_color_t g_buf1[kScreenWidth * kDrawBufferLines];

    lv_obj_t *g_chart = nullptr;
    lv_chart_series_t *g_seriesSelected = nullptr;
    lv_chart_series_t *g_seriesCombined = nullptr;

    lv_obj_t *g_valueFs = nullptr;
    lv_obj_t *g_valueM = nullptr;
    lv_obj_t *g_valueF = nullptr;
    lv_obj_t *g_valueQ = nullptr;
    lv_obj_t *g_valueVol = nullptr;
    lv_obj_t *g_valueFreq = nullptr;
    lv_obj_t *g_freqCaret = nullptr;

    lv_obj_t *g_filterLabels[3] = {nullptr, nullptr, nullptr};
    lv_obj_t *g_indexLabels[FILTER_BANDS] = {};

    uint32_t g_lastTickMs = 0;

    lv_color_t colorScreenBg() { return UI_LIGHT_MODE ? lv_color_hex(0xF3F4F6) : lv_color_hex(0x05070A); }
    lv_color_t colorChartBg() { return UI_LIGHT_MODE ? lv_color_hex(0xd8d8d8) : lv_color_hex(0x0B1220); }
    lv_color_t colorChartBorder() { return UI_LIGHT_MODE ? lv_color_hex(0x64748B) : lv_color_hex(0x374151); }
    lv_color_t colorChartSelected() { return UI_LIGHT_MODE ? lv_color_hex(0x0369A1) : lv_color_hex(0x00D1FF); }
    lv_color_t colorChartCombined() { return UI_LIGHT_MODE ? lv_color_hex(0x6B7280) : lv_color_hex(0xD1D5DB); }
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
            if (point < kChartMinDb)
            {
                point = kChartMinDb;
            }
            if (point > kChartMaxDb)
            {
                point = kChartMaxDb;
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

    // Y-axis labels aligned to chart max, mid and min values.
    lv_obj_t *yMaxLabel = lv_label_create(scr);
    lv_label_set_text(yMaxLabel, "15");
    lv_obj_set_style_text_color(yMaxLabel, colorAxisText(), 0);
    lv_obj_set_style_text_font(yMaxLabel, &lv_font_montserrat_8, 0);
    lv_obj_set_style_text_align(yMaxLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(yMaxLabel, 4, kChartY - 2);
    lv_obj_set_size(yMaxLabel, kChartX - 8, LV_SIZE_CONTENT);

    lv_obj_t *yMidLabel = lv_label_create(scr);
    lv_label_set_text(yMidLabel, "0");
    lv_obj_set_style_text_color(yMidLabel, colorAxisText(), 0);
    lv_obj_set_style_text_font(yMidLabel, &lv_font_montserrat_8, 0);
    lv_obj_set_style_text_align(yMidLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(yMidLabel, 4, kChartY + (kChartHeight / 2) - 4);
    lv_obj_set_size(yMidLabel, kChartX - 8, LV_SIZE_CONTENT);

    lv_obj_t *yMinLabel = lv_label_create(scr);
    lv_label_set_text(yMinLabel, "-15");
    lv_obj_set_style_text_color(yMinLabel, colorAxisText(), 0);
    lv_obj_set_style_text_font(yMinLabel, &lv_font_montserrat_8, 0);
    lv_obj_set_style_text_align(yMinLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(yMinLabel, 4, kChartY + kChartHeight - 8);
    lv_obj_set_size(yMinLabel, kChartX - 8, LV_SIZE_CONTENT);

    g_chart = lv_chart_create(scr);
    lv_obj_set_pos(g_chart, kChartX, kChartY);
    lv_obj_set_size(g_chart, kChartWidth, kChartHeight);
    lv_chart_set_type(g_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(g_chart, kChartPoints);
    lv_chart_set_range(g_chart, LV_CHART_AXIS_PRIMARY_Y, kChartMinDb, kChartMaxDb);
    lv_obj_set_style_bg_color(g_chart, colorChartBg(), 0);
    lv_obj_set_style_bg_opa(g_chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_chart, colorChartBorder(), 0);
    lv_obj_set_style_border_width(g_chart, 1, 0);
    lv_obj_set_style_radius(g_chart, 0, 0);
    lv_obj_set_style_pad_all(g_chart, 0, LV_PART_MAIN);
    lv_obj_set_style_line_width(g_chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_line_rounded(g_chart, false, LV_PART_ITEMS);
    lv_obj_set_style_size(g_chart, 0, LV_PART_INDICATOR);
    lv_chart_set_div_line_count(g_chart, 7, 5);
    lv_obj_set_style_line_color(g_chart, lv_color_hex(0x808080), LV_PART_MAIN);




    // X-axis labels aligned with decade grid lines: 2, 20, 200, 2000, 20000.
    const char *xLabels[5] = {"2", "20", "200", "2000", "20000"};
    for (int i = 0; i < 5; i++)
    {
        lv_obj_t *xLabel = lv_label_create(scr);
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

    lv_obj_t *panel = lv_obj_create(scr);
    lv_obj_set_pos(panel, kPanelX, kPanelY);
    lv_obj_set_size(panel, kPanelWidth, kPanelHeight);
    lv_obj_set_style_bg_color(panel, colorMetricPanel(), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(panel, colorMetricBorder(), 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_radius(panel, 2, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);

    lv_obj_t *labelFs = lv_label_create(scr);
    lv_label_set_text(labelFs, "Fs");
    lv_obj_set_pos(labelFs, kPanelX + 4, 16);
    lv_obj_set_size(labelFs, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelFs);

    lv_obj_t *labelM = lv_label_create(scr);
    lv_label_set_text(labelM, "MG");
    lv_obj_set_pos(labelM, kPanelX + 4, 48);
    lv_obj_set_size(labelM, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelM);

    lv_obj_t *labelF = lv_label_create(scr);
    lv_label_set_text(labelF, "FG");
    lv_obj_set_pos(labelF, kPanelX + 4, 80);
    lv_obj_set_size(labelF, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelF);

    lv_obj_t *labelQ = lv_label_create(scr);
    lv_label_set_text(labelQ, "Q");
    lv_obj_set_pos(labelQ, kPanelX + 4, 112);
    lv_obj_set_size(labelQ, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelQ);

    lv_obj_t *labelVol = lv_label_create(scr);
    lv_label_set_text(labelVol, "Vol");
    lv_obj_set_pos(labelVol, kPanelX + 4, 142);
    lv_obj_set_size(labelVol, kPanelWidth - 8, LV_SIZE_CONTENT);
    styleMetricLabel(labelVol);

    g_valueFs = lv_label_create(scr);
    lv_obj_set_pos(g_valueFs, kPanelX + 4, 32);
    lv_obj_set_size(g_valueFs, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueFs, "----");
    styleMetricValue(g_valueFs);

    g_valueM = lv_label_create(scr);
    lv_obj_set_pos(g_valueM, kPanelX + 4, 64);
    lv_obj_set_size(g_valueM, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueM, "0.0");
    styleMetricValue(g_valueM);

    g_valueF = lv_label_create(scr);
    lv_obj_set_pos(g_valueF, kPanelX + 4, 96);
    lv_obj_set_size(g_valueF, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueF, "0.0");
    styleMetricValue(g_valueF);

    g_valueQ = lv_label_create(scr);
    lv_obj_set_pos(g_valueQ, kPanelX + 4, 128);
    lv_obj_set_size(g_valueQ, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueQ, "0.0");
    styleMetricValue(g_valueQ);

    g_valueVol = lv_label_create(scr);
    lv_obj_set_pos(g_valueVol, kPanelX + 4, 158);
    lv_obj_set_size(g_valueVol, kPanelWidth - 8, LV_SIZE_CONTENT);
    lv_label_set_text(g_valueVol, "0.0");
    styleMetricValue(g_valueVol);

    g_valueFreq = lv_label_create(scr);
    lv_obj_set_style_text_color(g_valueFreq, colorAxisText(), 0);
    lv_obj_set_style_text_font(g_valueFreq, &lv_font_montserrat_8, 0);
    lv_label_set_text(g_valueFreq, "--");
    lv_obj_update_layout(g_valueFreq);

    g_freqCaret = lv_label_create(scr);
    lv_obj_set_style_text_color(g_freqCaret, colorAxisText(), 0);
    lv_obj_set_style_text_font(g_freqCaret, &lv_font_montserrat_8, 0);
    lv_label_set_text(g_freqCaret, "^");
    lv_obj_update_layout(g_freqCaret);

    positionFrequencyLabel(kMinGraphFrequency);

    // Frames around footer groups: filter type badges and filter band badges.
    lv_obj_t *typeFrame = lv_obj_create(scr);
    lv_obj_set_pos(typeFrame, 8, kBottomBadgeY - 2);
    lv_obj_set_size(typeFrame, 165, 23);
    lv_obj_set_style_bg_opa(typeFrame, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(typeFrame, colorFooterFrame(), 0);
    lv_obj_set_style_border_width(typeFrame, 1, 0);
    lv_obj_set_style_radius(typeFrame, 2, 0);
    lv_obj_set_style_pad_all(typeFrame, 0, 0);

    lv_obj_t *bandFrame = lv_obj_create(scr);
    lv_obj_set_pos(bandFrame, 186, kBottomBadgeY - 2);
    lv_obj_set_size(bandFrame, 126, 23);
    lv_obj_set_style_bg_opa(bandFrame, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(bandFrame, colorFooterFrame(), 0);
    lv_obj_set_style_border_width(bandFrame, 1, 0);
    lv_obj_set_style_radius(bandFrame, 2, 0);
    lv_obj_set_style_pad_all(bandFrame, 0, 0);

    g_filterLabels[0] = lv_label_create(scr);
    g_filterLabels[1] = lv_label_create(scr);
    g_filterLabels[2] = lv_label_create(scr);

    makeBadge(g_filterLabels[0], "LoSh", 12, kBottomBadgeY);
    makeBadge(g_filterLabels[1], "HiSh", 68, kBottomBadgeY);
    makeBadge(g_filterLabels[2], "Peak", 124, kBottomBadgeY);

    for (int i = 0; i < FILTER_BANDS; i++)
    {
        g_indexLabels[i] = lv_label_create(scr);
        char indexBuf[4];
        snprintf(indexBuf, sizeof(indexBuf), "%d", i + 1);
        makeBadge(g_indexLabels[i], indexBuf, 190 + i * 25, kBottomBadgeY);
    }

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
    if (g_chart == nullptr || g_seriesSelected == nullptr || g_seriesCombined == nullptr)
    {
        return;
    }

    if (data.sampleRate == 0)
    {
        lv_label_set_text(g_valueFs, "----");
    }
    else
    {
        char srBuf[24];
        if (data.sampleRate >= 100000)
        {
            snprintf(srBuf, sizeof(srBuf), "%4.0f", data.sampleRate / 1000.0f);
        }
        else
        {
            snprintf(srBuf, sizeof(srBuf), "%4.1f", data.sampleRate / 1000.0f);
        }
        lv_label_set_text(g_valueFs, srBuf);
    }

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
