#ifndef _audo_player_h
#define _audio_player_h

#include "main.h"

void Audio_Player_Init(uint8_t* data, uint32_t length);
void Audio_Player_Play(uint32_t sample_rate, uint32_t resolution);
void Audio_Player_Pause();
void Audio_Player_Resume();
void Audio_Player_Stop();

#endif
