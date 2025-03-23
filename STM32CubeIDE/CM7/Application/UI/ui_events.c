#include "ui_events.h"
#include "ui.h"
#include "cmsis_os.h"

extern osThreadId_t music_file_Handle;
void play_pause_button_cb(lv_event_t * event)
{
	// state: 0 is playing the music; 1 is pausing the music
    static int state = 0;
    lv_obj_t * button = lv_event_get_target(event);

    if (state == 0) {
        // pause the music
        state = 1;
        lv_obj_set_style_bg_image_src(button, &ui_img_btn_play_png, LV_PART_MAIN | LV_STATE_DEFAULT);
        osThreadFlagsSet(music_file_Handle, 0x04);
    } else {
        // resume the music
        state = 0;
        lv_obj_set_style_bg_image_src(button, &ui_img_btn_pause_png, LV_PART_MAIN | LV_STATE_DEFAULT);
        osThreadFlagsSet(music_file_Handle, 0x08);
    }
}

void next_button_cb(lv_event_t * event)
{
    osThreadFlagsSet(music_file_Handle, 0x10);
}
