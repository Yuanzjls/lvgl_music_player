#include "lvgl_touch.h"
#include "lvgl.h"
#include "stm32h747i_discovery_ts.h"
static volatile uint8_t touch_irq = 0;

static void
lvgl_touchscreen_read(lv_indev_t *indev,
                      lv_indev_data_t *data);

void lvgl_touchscreen_init(void)
{
    /* basic LVGL driver initialization */
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_touchscreen_read);
}

static void
lvgl_touchscreen_read(lv_indev_t *indev,
                      lv_indev_data_t *data)
{
    /*Use the saved coordinates if there were an interrupt*/
    if (touch_irq)
    {
        /* reset interrupt flag */
        TS_State_t ts_state;
        touch_irq = 0;
        BSP_TS_GetState(0, &ts_state);
        data->point.x = ts_state.TouchX;
        data->point.y = ts_state.TouchY;
        data->state = ts_state.TouchDetected ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    }
    /*If there is no interrupt the touch is released*/
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void BSP_TS_Callback(uint32_t Instance)
{
    UNUSED(Instance);

    touch_irq = 1;
}
