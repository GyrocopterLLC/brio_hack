/*
 * audio.h
 *
 *  Created on: Dec 28, 2023
 *      Author: david
 */

#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

#include <stdbool.h>

void init_audio(DMA_HandleTypeDef* hd, TIM_HandleTypeDef* ht);
bool start_playing_audio_file(uint32_t audio_file_address, uint32_t audio_file_length);
void abort_audio();
bool currently_playing();

void mp3_dma_callback(); // call this from the DMA complete callback

#endif /* INC_AUDIO_H_ */
