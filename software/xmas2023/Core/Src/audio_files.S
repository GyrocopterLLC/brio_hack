/*
 * audio_files.s
 *
 *  Created on: Jan 2, 2024
 *      Author: david
 */


.syntax unified
.cpu cortex-m0plus
.fpu softvfp
.thumb

.section .rodata,"a",%progbits

// ----- Ding ding ding ding -----
.global audio1_data
.type audio1_data, %object
.align 4

audio1_data:
.incbin "output_16kHz.mp3"

.global audio1_size
.type audio1_size, %object
.align 4
audio1_size:
.int audio1_size - audio1_data

// ----- i love you -----
/*
.global love_data
.type love_data, %object
.align 4

love_data:
.incbin "love_trains.mp3"

.global love_size
.type love_size, %object
.align 4
love_size:
.int love_size - love_data
*/
// ----- whoooooo whooooo -----
.global whistle_data
.type whistle_data, %object
.align 4

whistle_data:
.incbin "whistle1.mp3"

.global whistle_size
.type whistle_size, %object
.align 4
whistle_size:
.int whistle_size - whistle_data

// ----- chug chug chug chug -----
.global chug_data
.type chug_data, %object
.align 4

chug_data:
.incbin "ChugChug.mp3"

.global chug_size
.type chug_size, %object
.align 4
chug_size:
.int chug_size - chug_data
// ----- get crabbin -----
/*
.global crab_data
.type crab_data, %object
.align 4

crab_data:
.incbin "CrabRave_16kHz.mp3"

.global crab_size
.type crab_size, %object
.align 4
crab_size:
.int crab_size - crab_data
*/
