#include "audio_player.h"
#include "stm32h747i_discovery_audio.h"
#include "cmsis_os.h"
/* Private Variable*/
static uint32_t AudioFreq[9] ={8000 , 11025, 16000, 22050, 32000, 44100, 48000, 96000, 192000};
BSP_AUDIO_Init_t AudioPlayInit;
static uint8_t* pData;
static uint32_t aLength;
/* Private Variable*/

/* External Variables*/
extern osThreadId_t music_file_Handle;
/* External Variables*/

void Audio_Player_Init(uint8_t* data, uint32_t length) {
  pData = data;
  aLength = length;
  AudioPlayInit.Device = AUDIO_OUT_DEVICE_HEADPHONE;
  AudioPlayInit.ChannelsNbr =  data[0x16];//2;
  AudioPlayInit.SampleRate = *(uint32_t*)(data + 0x18); // AUDIO_FREQUENCY_44K
  AudioPlayInit.BitsPerSample = data[0x22]; // AUDIO_RESOLUTION_16B
  AudioPlayInit.Volume = 30;
  BSP_AUDIO_OUT_Init(0, &AudioPlayInit);
  BSP_AUDIO_OUT_SetVolume(0, AudioPlayInit.Volume);
}

void Audio_Player_Play(uint32_t sample_rate, uint32_t resolution) {
  BSP_AUDIO_OUT_SetSampleRate(0, sample_rate);
  BSP_AUDIO_OUT_SetBitsPerSample(0, resolution);

  BSP_AUDIO_OUT_Play(0, pData, aLength);
}


void Audio_Player_Pause() {
  BSP_AUDIO_OUT_Pause(0);
}

void Audio_Player_Resume() {
  BSP_AUDIO_OUT_Resume(0);
}

void Audio_Player_Stop() {
  BSP_AUDIO_OUT_Stop(0);
}

/**
  * @brief  Manages the DMA full Transfer complete event
  * @param  Instance : AUDIO OUT Instance. It can only be 0 (SAI)
  * @retval None
  */
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);
  /* Manage the remaining file size and new address offset: This function
     should be coded by user (its prototype is already declared in stm32f769i_discovery_audio.h) */
  osThreadFlagsSet(music_file_Handle, 0x01);
}
 
 /**
   * @brief  Manages the DMA Half Transfer complete event
   * @param  Instance : AUDIO OUT Instance. It can only be 0 (SAI)
   * @retval None
   */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Instance);

    /* Manage the remaining file size and new address offset: This function
     should be coded by user (its prototype is already declared in stm32f769i_discovery_audio.h) */
     osThreadFlagsSet(music_file_Handle, 0x02);
}
