#include "lvgl_display.h"
#include "lvgl.h"
#include <stdint.h>
#include "main.h"
#include "core_cm7.h"

#define MY_DISP_HOR_RES 800
#define MY_DISP_VER_RES 480

static uint8_t buf1[240 * 800];
static uint8_t buf2[240 * 800];
static lv_display_t *disp;

static void disp_flush(lv_display_t *display,
                       const lv_area_t *area,
                       uint8_t *px_map);
static void
disp_flush_complete(DMA2D_HandleTypeDef *hdma2d);

extern DMA2D_HandleTypeDef hlcd_dma2d;

void lvgl_display_init(void)
{
    /* display initialization */

    disp = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);

    lv_display_set_buffers(disp, (void *)buf1, (void *)buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* interrupt callback for DMA2D transfer */
    hlcd_dma2d.XferCpltCallback = disp_flush_complete;

    lv_display_set_flush_cb(disp, disp_flush);
}

static void disp_flush(lv_display_t *display,
                       const lv_area_t *area,
                       uint8_t *px_map)
{

    lv_coord_t width = lv_area_get_width(area);
    lv_coord_t height = lv_area_get_height(area);

    SCB_CleanInvalidateDCache();

    /* DMA2D transfer */

    DMA2D->CR = 0x0U << DMA2D_CR_MODE_Pos;
    DMA2D->CR |= DMA2D_M2M_PFC;
    DMA2D->FGPFCCR = DMA2D_INPUT_RGB888;
    DMA2D->FGMAR = (uint32_t)px_map;
    DMA2D->FGOR = 0;
    DMA2D->OPFCCR = DMA2D_OUTPUT_ARGB8888;
    DMA2D->OMAR = 0xD0000000 + 4 *
                                   (area->y1 * MY_DISP_HOR_RES + area->x1);
    DMA2D->OOR = MY_DISP_HOR_RES - width;
    DMA2D->NLR = (width << DMA2D_NLR_PL_Pos) | (height << DMA2D_NLR_NL_Pos);
    DMA2D->IFCR = 0x3FU;
    DMA2D->CR |= DMA2D_CR_TCIE;
    DMA2D->CR |= DMA2D_CR_START;
}

static void
disp_flush_complete(DMA2D_HandleTypeDef *hdma2d)
{
    //	lv_thread_sync_signal_isr(&sync);

    lv_display_flush_ready(disp);
}
