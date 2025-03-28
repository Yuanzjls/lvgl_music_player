// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 9.2.2
// Project name: music_player

#ifndef _MUSIC_PLAYER_UI_H
#define _MUSIC_PLAYER_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"


// SCREEN: ui_main
void ui_main_screen_init(void);
extern lv_obj_t * ui_main;
extern lv_obj_t * ui_next;
extern lv_obj_t * ui_playPause;
extern lv_obj_t * ui_Image2;
extern lv_obj_t * ui_Image3;
extern lv_obj_t * ui_Image4;
extern lv_obj_t * ui_Image5;
// CUSTOM VARIABLES

// EVENTS

extern lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS
LV_IMG_DECLARE(ui_img_btn_next_png);    // assets/btn_next.png
LV_IMG_DECLARE(ui_img_btn_play_png);    // assets/btn_play.png
LV_IMG_DECLARE(ui_img_btn_pause_png);    // assets/wave_bottom.png

// UI INIT
void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
