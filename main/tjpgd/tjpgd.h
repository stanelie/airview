/*----------------------------------------------------------------------------/
/ TJpgDec - Tiny JPEG Decompressor R0.03 include file         (C)ChaN, 2021
/----------------------------------------------------------------------------*/
#ifndef DEF_TJPGDEC
#define DEF_TJPGDEC

#ifdef __cplusplus
extern "C" {
#endif

#include "tjpgdcnf.h"
#include <string.h>
#include <stdint.h>

#if JD_FASTDECODE >= 1
typedef int16_t jd_yuv_t;
#else
typedef uint8_t jd_yuv_t;
#endif

/* Error code */
typedef enum {
	JDR_OK = 0,
	JDR_INTR,
	JDR_INP,
	JDR_MEM1,
	JDR_MEM2,
	JDR_PAR,
	JDR_FMT1,
	JDR_FMT2,
	JDR_FMT3
} JRESULT;

/* Rectangular region in the output image */
typedef struct {
	uint16_t left;
	uint16_t right;
	uint16_t top;
	uint16_t bottom;
} JRECT;

/* Decompressor object structure */
typedef struct JDEC JDEC;
struct JDEC {
	size_t dctr;
	uint8_t* dptr;
	uint8_t* inbuf;
	uint8_t dbit;
	uint8_t scale;
	uint8_t msx, msy;
	uint8_t qtid[3];
	uint8_t ncomp;
	int16_t dcv[3];
	uint16_t nrst;
	uint16_t width, height;
	uint8_t* huffbits[2][2];
	uint16_t* huffcode[2][2];
	uint8_t* huffdata[2][2];
	int32_t* qttbl[4];
#if JD_FASTDECODE >= 1
	uint32_t wreg;
	uint8_t marker;
#if JD_FASTDECODE == 2
	uint8_t longofs[2][2];
	uint16_t* hufflut_ac[2];
	uint8_t* hufflut_dc[2];
#endif
#endif
	void* workbuf;
	jd_yuv_t* mcubuf;
	void* pool;
	size_t sz_pool;
	size_t (*infunc)(JDEC*, uint8_t*, size_t);
	void* device;
};

JRESULT jd_prepare(JDEC* jd, size_t (*infunc)(JDEC*,uint8_t*,size_t), void* pool, size_t sz_pool, void* dev);
JRESULT jd_decomp(JDEC* jd, int (*outfunc)(JDEC*,void*,JRECT*), uint8_t scale);

#ifdef __cplusplus
}
#endif

#endif /* DEF_TJPGDEC */
