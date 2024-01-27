#include "main.h"
#include "coder.h"
#include "mp3dec.h"
#include <stdbool.h>
#include <stdint.h>

static void signed_to_unsigned_offset(int16_t* buf, uint32_t buflen);

int16_t audio_buf_0[MAX_NSAMP];
int16_t audio_buf_1[MAX_NSAMP];

typedef struct mp3_player_info
{
	HMP3Decoder decoder;
	int bytesleft;
	unsigned char* fileptr;
	int next_numsamps;
	bool buffer_0_in_use;
	volatile bool finished; // TODO: check if volatile actually needed here. Anywhere else too?
	bool last_chunk;
	DMA_HandleTypeDef* hdma;
	TIM_HandleTypeDef* htim;
} mp3_player_t;

mp3_player_t player_info;

void init_audio(DMA_HandleTypeDef* hd, TIM_HandleTypeDef* ht) {
	player_info.finished = true;
	player_info.hdma = hd;
	player_info.htim = ht;
	player_info.decoder = MP3InitDecoder();
}

bool start_playing_audio_file(uint32_t audio_file_address, uint32_t audio_file_length)
{
	MP3FrameInfo frameinfo;

	player_info.bytesleft = audio_file_length;
	player_info.fileptr = (unsigned char*) audio_file_address;
	player_info.buffer_0_in_use = true;
	// Find the starting address of the first MP3 header
	// Usually the file starts with some ID3 header, so we have to skip past that

	int skip_bytes = MP3FindSyncWord(player_info.fileptr, player_info.bytesleft);
	if(skip_bytes < 0)
	{
		// sync word not found!
		return false;
	}

	player_info.bytesleft -= skip_bytes;
	player_info.fileptr += skip_bytes;

	// Preload the two ping-pong buffers
	if (MP3Decode(player_info.decoder, &player_info.fileptr, &player_info.bytesleft, audio_buf_0, 0))
	{
		return false;
	}

	MP3GetLastFrameInfo(player_info.decoder, &frameinfo);
	int num_samps_buf0 = frameinfo.outputSamps;
	signed_to_unsigned_offset(audio_buf_0, num_samps_buf0);

	if (MP3Decode(player_info.decoder, &player_info.fileptr, &player_info.bytesleft, audio_buf_1, 0))
	{
		return false;
	}
	MP3GetLastFrameInfo(player_info.decoder, &frameinfo);
	signed_to_unsigned_offset(audio_buf_1, frameinfo.outputSamps);
	player_info.next_numsamps = frameinfo.outputSamps;

	// Enable the DAC output of the mcu and the external speaker amp
	DAC1->CR |= DAC_CR_EN1;
	// HAL_GPIO_WritePin(Amp_SD_GPIO_Port, Amp_SD_Pin, GPIO_PIN_SET);
	Amp_SD_GPIO_Port->BSRR = Amp_SD_Pin;

	// Start DMA transfer of first buffer
	HAL_DMA_Start_IT(player_info.hdma, (uint32_t)audio_buf_0, (uint32_t)&(DAC1->DHR12L1), num_samps_buf0);

	// Figure out the timer period. We need to match the audio rate of the input file.
	HAL_RCC_GetSysClockFreq(); // discard the return, we just need it to update internal values
	uint32_t pclk = HAL_RCC_GetPCLK1Freq();
	if((RCC->CFGR & RCC_CFGR_PPRE) != 0) {
		pclk = pclk * 2; // timers run at 2x PCLK frequency if any PCLK divider is anything other than 1x
	}
	uint32_t arr_period = (pclk / ((uint32_t)frameinfo.samprate)) - 1;
	player_info.htim->Instance->ARR = arr_period;

	HAL_TIM_Base_Start(player_info.htim);
	player_info.htim->Instance->DIER = TIM_DIER_UDE; // enable the update dma flag from the timer

	player_info.finished = false;
	player_info.last_chunk = false;
	return true;
}

void abort_audio()
{
	HAL_DMA_Abort_IT(player_info.hdma);
	HAL_TIM_Base_Stop(player_info.htim);
	DAC1->CR &= ~(DAC_CR_EN1);
	Amp_SD_GPIO_Port->BRR = Amp_SD_Pin;
	player_info.finished = true;
	audio_finished(); // main's finished callback
}

bool currently_playing()
{
	return (false == player_info.finished);
}

void mp3_dma_callback()
{
	// Called when the DMA is finished transferring a block of samples
	// Need to setup the next block, or just quit if nothing is left
	if(player_info.last_chunk)
	{
		// we just hit the end of the final chunk
		// turn everything off and get outta here
		abort_audio();
		player_info.finished = true;
		return;
	}

	// assign the ping-pong buffers now to temporary pointers
	// makes it easier than a bunch of "if" switches every time we use them later
	int16_t* playback_buffer;
	int16_t* filling_buffer;
	if(player_info.buffer_0_in_use) {
		// Buffer 0 just finished, now play from buffer 1
		playback_buffer = audio_buf_1;
		filling_buffer = audio_buf_0;
	}
	else
	{
		// Buffer 1 just finished, play from buffer 0
		playback_buffer = audio_buf_0;
		filling_buffer = audio_buf_1;
	}

	// Immediately start the other buffer's playback.
	HAL_DMA_Start_IT(player_info.hdma, (uint32_t)playback_buffer, (uint32_t)&(DAC1->DHR12L1), player_info.next_numsamps);

	if(player_info.bytesleft <= 0)
	{
		// the chunk in the buffer that was just set up to start playing will be the last one
		player_info.last_chunk = true; // one more DMA interrupt will be fired when the last chunk is played
		return;
	}

	// Fill the buffer that just finished
	if(MP3Decode(player_info.decoder, &player_info.fileptr, &player_info.bytesleft, filling_buffer, 0))
	{
		// on error, simply quit DMA (and stop the timer)
		abort_audio();
		return;
	}
	MP3FrameInfo frameinfo;
	MP3GetLastFrameInfo(player_info.decoder, &frameinfo);
	signed_to_unsigned_offset(filling_buffer, frameinfo.outputSamps);
	player_info.next_numsamps = frameinfo.outputSamps;


	// Swap buffers for next time
	player_info.buffer_0_in_use = !player_info.buffer_0_in_use;

}

static void signed_to_unsigned_offset(int16_t* buf, uint32_t buflen)
{
	/* Adds 0x8000 to each half-word in the array to cause it to wrap around
	this is used because we need to send signed integers to the DAC, which
	only takes in unsigned data.
	So anything with value above 0x8000 (which in normal int16 land is a
	negative value), will actually just be a high value. Sending a bunch
	of values that are right around 0 (like a little bit positive, then
	a little bit negative) will result in huge full-scale swings in the output,
	because the little bit negative values will appear to be very large values!
	*/

	uint32_t* temp_buf = (uint32_t*)buf;
	bool one_extra = ((buflen % 2) == 1); // if buflen is odd, we need to do an extra half-word operation
	uint32_t buflen32 = buflen / 2;

	for(uint32_t i = 0; i < buflen32; i++)
	{
		// First we clear the least significant bit of the upper word. This prevents
		// any carryover from the lower word's addition from affecting the upper word
		// This doesn't matter in the end since the DAC only takes the most significant
		// 12 bits of the 16 anyway.
		temp_buf[i] &= ~(0x00010000u);

		// Then add the value of 32768 to both the upper and lower half words
		temp_buf[i] += 0x80008000u;
	}
	if(one_extra)
	{
		uint16_t* ubuf = (uint16_t*)buf;
		ubuf[buflen-1] += (uint16_t)(0x8000u);
	}
}
