/* ***** BEGIN LICENSE BLOCK ***** 
 * Version: RCSL 1.0/RPSL 1.0 
 *  
 * Portions Copyright (c) 1995-2002 RealNetworks, Inc. All Rights Reserved. 
 *      
 * The contents of this file, and the files included with this file, are 
 * subject to the current version of the RealNetworks Public Source License 
 * Version 1.0 (the "RPSL") available at 
 * http://www.helixcommunity.org/content/rpsl unless you have licensed 
 * the file under the RealNetworks Community Source License Version 1.0 
 * (the "RCSL") available at http://www.helixcommunity.org/content/rcsl, 
 * in which case the RCSL will apply. You may also obtain the license terms 
 * directly from RealNetworks.  You may not use this file except in 
 * compliance with the RPSL or, if you have a valid RCSL with RealNetworks 
 * applicable to this file, the RCSL.  Please see the applicable RPSL or 
 * RCSL for the rights, obligations and limitations governing use of the 
 * contents of the file.  
 *  
 * This file is part of the Helix DNA Technology. RealNetworks is the 
 * developer of the Original Code and owns the copyrights in the portions 
 * it created. 
 *  
 * This file, and the files included with this file, is distributed and made 
 * available on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
 * EXPRESS OR IMPLIED, AND REALNETWORKS HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
 * FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. 
 * 
 * Technology Compatibility Kit Test Suite(s) Location: 
 *    http://www.helixcommunity.org/content/tck 
 * 
 * Contributor(s): 
 *  
 * ***** END LICENSE BLOCK ***** */ 

/**************************************************************************************
 * Fixed-point MP3 decoder
 * Jon Recker (jrecker@real.com), Ken Cooke (kenc@real.com)
 * June 2003
 *
 * assembly.h - assembly language functions and prototypes for supported platforms
 *
 * - inline rountines with access to 64-bit multiply results 
 * - x86 (_WIN32) and ARM (ARM_ADS, _WIN32_WCE) versions included
 * - some inline functions are mix of asm and C for speed
 * - some functions are in native asm files, so only the prototype is given here
 *
 * MULSHIFT32(x, y)    signed multiply of two 32-bit integers (x and y), returns top 32 bits of 64-bit result
 * FASTABS(x)          branchless absolute value of signed integer x
 * CLZ(x)              count leading zeros in x
 * MADD64(sum, x, y)   (Windows only) sum [64-bit] += x [32-bit] * y [32-bit]
 * SHL64(sum, x, y)    (Windows only) 64-bit left shift using __int64
 * SAR64(sum, x, y)    (Windows only) 64-bit right shift using __int64
 */

#ifndef _ASSEMBLY_H
#define _ASSEMBLY_H

#if (defined _WIN32 && !defined _WIN32_WCE) || (defined __WINS__ && defined _SYMBIAN) || defined(_OPENWAVE_SIMULATOR) || defined(WINCE_EMULATOR)    /* Symbian emulator for Ix86 */

#pragma warning( disable : 4035 )	/* complains about inline asm not returning a value */

static __inline int MULSHIFT32(int x, int y)	
{
    __asm {
		mov		eax, x
	    imul	y
	    mov		eax, edx
	}
}

static __inline int FASTABS(int x) 
{
	int sign;

	sign = x >> (sizeof(int) * 8 - 1);
	x ^= sign;
	x -= sign;

	return x;
}

static __inline int CLZ(int x)
{
	int numZeros;

	if (!x)
		return (sizeof(int) * 8);

	numZeros = 0;
	while (!(x & 0x80000000)) {
		numZeros++;
		x <<= 1;
	} 

	return numZeros;
}

/* MADD64, SHL64, SAR64:
 * write in assembly to avoid dependency on run-time lib for 64-bit shifts, muls
 *  (sometimes compiler thunks to function calls instead of code generating)
 * required for Symbian emulator
 */
#ifdef __CW32__
typedef long long Word64;
#else
typedef __int64 Word64;
#endif

static __inline Word64 MADD64(Word64 sum, int x, int y)
{
	unsigned int sumLo = ((unsigned int *)&sum)[0];
	int sumHi = ((int *)&sum)[1];

	__asm {
		mov		eax, x
		imul	y
		add		eax, sumLo
		adc		edx, sumHi
	}

	/* equivalent to return (sum + ((__int64)x * y)); */
}

static __inline Word64 SHL64(Word64 x, int n)
{
	unsigned int xLo = ((unsigned int *)&x)[0];
	int xHi = ((int *)&x)[1];
	unsigned char nb = (unsigned char)n;

	if (n < 32) {
		__asm {
			mov		edx, xHi
			mov		eax, xLo
			mov		cl, nb
			shld    edx, eax, cl
			shl     eax, cl
		}
	} else if (n < 64) {
		/* shl masks cl to 0x1f */
		__asm {
			mov		edx, xLo
			mov		cl, nb
			xor     eax, eax
			shl     edx, cl
		}
	} else {
		__asm {
			xor		edx, edx
			xor		eax, eax
		}
	}
}

static __inline Word64 SAR64(Word64 x, int n)
{
	unsigned int xLo = ((unsigned int *)&x)[0];
	int xHi = ((int *)&x)[1];
	unsigned char nb = (unsigned char)n;

	if (n < 32) {
		__asm {
			mov		edx, xHi
			mov		eax, xLo
			mov		cl, nb
			shrd	eax, edx, cl
			sar		edx, cl
		}
	} else if (n < 64) {
		/* sar masks cl to 0x1f */
		__asm {
			mov		edx, xHi
			mov		eax, xHi
			mov		cl, nb
			sar		edx, 31
			sar		eax, cl
		}
	} else {
		__asm {
			sar		xHi, 31
			mov		eax, xHi
			mov		edx, xHi
		}
	}
}

#elif (defined _WIN32) && (defined _WIN32_WCE)

/* use asm function for now (EVC++ 3.0 does horrible job compiling __int64 version) */
#define MULSHIFT32	xmp3_MULSHIFT32
int MULSHIFT32(int x, int y);

static __inline int FASTABS(int x) 
{
	int sign;

	sign = x >> (sizeof(int) * 8 - 1);
	x ^= sign;
	x -= sign;

	return x;
}

static __inline int CLZ(int x)
{
	int numZeros;

	if (!x)
		return (sizeof(int) * 8);

	numZeros = 0;
	while (!(x & 0x80000000)) {
		numZeros++;
		x <<= 1;
	} 

	return numZeros;
}

#elif defined ARM_ADS

typedef long long Word64;

#define MULSHIFT32	xmp3_MULSHIFT32
extern int MULSHIFT32(int x, int y);


#define FASTABS	xmp3_FASTABS
int FASTABS(int x);


static __inline int CLZ(int x)
{
	int numZeros;

	if (!x)
		return (sizeof(int) * 8);

	numZeros = 0;
	while (!(x & 0x80000000)) {
		numZeros++;
		x <<= 1;
	} 

	return numZeros;
}

#elif defined(__GNUC__) && defined(ARM)

typedef long long int Word64;

static inline int MULSHIFT32(int u, int v) {
	unsigned u0, v0, w0;
	int u1, v1, w1, w2, t;
	u0 = u & 0xFFFF; u1 = u >> 16;
	v0 = v & 0xFFFF; v1 = v >> 16;
	w0 = u0*v0;
	t = u1*v0 + (w0 >> 16);
	w1 = t & 0xFFFF;
	w2 = t >> 16;
	w1 = u0*v1 + w1;
	return u1*v1 + w2 + (w1 >> 16);
}

static inline Word64 MADD64(Word64 sum, int u, int v){
	unsigned u0, v0, w0;
	int u1, v1, w1, w2, t;
	Word64 temp;
    unsigned int* ptemp = (unsigned int*)&temp;
	u0 = u & 0xFFFF; u1 = u >> 16;
	v0 = v & 0xFFFF; v1 = v >> 16;
	w0 = u0*v0;
	t = u1*v0 + (w0 >> 16);
	w1 = t & 0xFFFF;
	w2 = t >> 16;
	w1 = u0*v1 + w1;
    ptemp[0] = (w0&0xFFFF) + (w1 << 16);
    ptemp[1] = u1*v1 + w2 + (w1 >> 16);
	return sum + temp;
}

static inline int FASTABS(int x){
	int y, scr;		// instructions according to godbolt.org:
	scr = x >> 31;	// asrs r3, r0
	y = x ^ scr;	// eors r0, r3
	y = y - scr;	// subs r0, r0, r3
	/*
	asm volatile(
			"asr %[scratch], %[input]\n"
			"eor %[input], %[scratch]\n"
			"sub %[output], %[input], %[scratch]\n"
			:[output] "=r" (y), [scratch] "=r" (scr)
			  :[input] "r" (x)
			);
	*/
	return y;
}
static inline int CLZ(unsigned int x){
    extern unsigned char const log2Lkup[256];

    if (x >= 0x00010000U) {
        if (x >= 0x01000000U) {
            return 8U - log2Lkup[x >> 24];
        }
        else {
            return 16U - log2Lkup[x >> 16];
        }
    }
    else {
        if (x >= 0x00000100U) {
            return 24U - log2Lkup[x >> 8];
        }
        else {
            return 32U - log2Lkup[x];
        }
    }
}

static inline Word64 SAR64(Word64 x, int n) {
	return x >> n;
}

/*
static inline Word64 MADD64(Word64 sum, int u, int v){
    // split inputs into 16-bit half words
    unsigned u0, v0;
    int u1, v1;
    u0 = u & 0xFFFF;
    u1 = u >> 16;
    v0 = v & 0xFFFF;
    v1 = v >> 16;
    // intermediate results
    int ll = u0 * v0;
    int m0 = u1 * v0;
    int m1 = u0 * v1;
    int hh = u1 * v1;
    // split intermediates into half words
    int lll = ll & 0xFFFF;
    int hll = ll >> 16;
    int lm0 = m0 & 0xFFFF;
    int hm0 = m0 >> 16;
    int lm1 = m1 & 0xFFFF;
    int hm1 = m1 >> 16;
    int lhh = hh & 0xFFFF;
    int hhh = hh >> 16;

    // create sums for final half-words
    long long result =
        (long long)lll
        + ((long long)(lm0+lm1+hll) << 16)
        + ((long long)(hm0+hm1+lhh) << 32)
        + ((long long)(hhh) << 48);

    return sum + result;
}
*/

#else

#error Unsupported platform in assembly.h

#endif	/* platforms */

#endif /* _ASSEMBLY_H */
