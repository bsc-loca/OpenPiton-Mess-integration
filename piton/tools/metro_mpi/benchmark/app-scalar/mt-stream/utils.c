/*
 * Copyright (c) 2024, Barcelona Supercomputing Center
 * Contact: pouya.esmaili    [at] bsc [dot] es
 *          alireza.monemi   [at] bsc [dot] es
 *          petar.radojkovic [at] bsc [dot] es
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *     * Neither the name of the copyright holder nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <unistd.h>
#include "utils.h"

#define MERGE_(a, b) a##b
#define LABEL_(a) MERGE_(nop_, a)
#define UNIQUE_NAME LABEL_(__LINE__)

#define STR(x) #x
#define XSTR(s) STR(s)


// Define an assembly macro
#define MY_ASM_MACRO(label) \
  "addi  x25, x22,  1;\n" \
   XSTR(UNIQUE_NAME)": " \
  "nop; " \
  "addi x25, x25, -1; " \
  "bne x25, x0," XSTR(UNIQUE_NAME)";\n"


#define NOP  MY_ASM_MACRO(1) 




#if RD_RATIO == 0
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_0:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_0;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 2
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_2:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_2;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 4
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_4:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_4;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 6
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_6:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_6;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 8
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_8:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_8;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 10
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_10:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_10;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}


#endif
#if RD_RATIO == 12
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_12:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "sd x24, 704(x23);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_12;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 14
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_14:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_14;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 16
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_16:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_16;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 18
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_18:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_18;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 20
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_20:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_20;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 22
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_22:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_22;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 24
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_24:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "sd x24, 256(x23);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_24;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 26
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_26:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_26;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 28
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_28:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "sd x24, 1024(x23);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_28;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 30
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_30:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_30;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 32
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_32:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_32;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}


#endif
#if RD_RATIO == 34
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_34:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "sd x24, 1536(x23);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_34;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 36
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_36:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_36;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 38 
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_38:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "sd x24, 448(x23);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_38;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}


#endif
#if RD_RATIO == 40
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_40:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "sd x24, 64(x23);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_40;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 42
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_42:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "sd x24, 896(x23);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_42;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 44
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_44:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_44;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 46
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_46:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_46;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 48
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_48:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "sd x24, 128(x23);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_48;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 50
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_50:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_50;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 52
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_52:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_52;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 54
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_54:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "sd x24, 1408(x23);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_54;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 56
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_56:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_56;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 58
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_58:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "sd x24, 576(x23);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_58;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 60
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_60:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_60;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 62
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_62:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "sd x24, 320(x23);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_62;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 64
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_64:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "sd x24, 1344(x23);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_64;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 66
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_66:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "sd x24, 1280(x23);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_66;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 68
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_68:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_68;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 70
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_70:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "sd x24, 640(x23);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_70;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 72
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_72:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "sd x24, 0(x23);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_72;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 74
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_74:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "sd x24, 1216(x23);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_74;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 76
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_76:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "sd x24, 960(x23);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_76;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 78
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_78:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "sd x24, 384(x23);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_78;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 80
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_80:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_80;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 82
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_82:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "sd x24, 448(x23);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_82;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 84
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_84:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "sd x24, 832(x23);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_84;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 86
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_86:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "sd x24, 768(x23);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_86;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 88
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_88:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "sd x24, 512(x23);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_88;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 90
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_90:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "sd x24, 1472(x23);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_90;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 92
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_92:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_92;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 94
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_94:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "ld x19, 192(x20);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "sd x24, 1152(x23);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_94;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 96
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_96:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "sd x24, 1088(x23);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "ld x19, 192(x20);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_96;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}

#endif
#if RD_RATIO == 98
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x22, %3;\n"
      "mv x24, x0;\n"
      "..L_98:\n"

      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "sd x24, 192(x23);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"
      "add x23, %4, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "ld x19, 192(x20);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP

      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_98;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}


#endif
#if RD_RATIO == 100
void STREAM_copy (double *a_array, double *b_array, ssize_t *array_size, const int* const pause)
{

    register ssize_t i;
    i = 0;
    uint64_t tmp = 8*(*array_size);

    asm __volatile__ (
      "mv x18, x0;\n"
      "mv x20, %0;\n"
      "mv x22, %3;\n"
      "..L_100:\n"
      "add x20, %0, x18;\n"
      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "ld x19, 192(x20);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP
      "addi x18, x18, 1600;\n"
      "add x20, %0, x18;\n"

      "ld x19, 0(x20);\n"
      "ld x19, 64(x20);\n"
      "ld x19, 128(x20);\n"
      "ld x19, 192(x20);\n"
      "ld x19, 256(x20);\n"
 NOP
      "ld x19, 320(x20);\n"
      "ld x19, 384(x20);\n"
      "ld x19, 448(x20);\n"
      "ld x19, 512(x20);\n"
      "ld x19, 576(x20);\n"
 NOP
      "ld x19, 640(x20);\n"
      "ld x19, 704(x20);\n"
      "ld x19, 768(x20);\n"
      "ld x19, 832(x20);\n"
      "ld x19, 896(x20);\n"
 NOP
      "ld x19, 960(x20);\n"
      "ld x19, 1024(x20);\n"
      "ld x19, 1088(x20);\n"
      "ld x19, 1152(x20);\n"
      "ld x19, 1216(x20);\n"
 NOP
      "ld x19, 1280(x20);\n"
      "ld x19, 1344(x20);\n"
      "ld x19, 1408(x20);\n"
      "ld x19, 1472(x20);\n"
      "ld x19, 1536(x20);\n"
 NOP
      "addi x18, x18, 1600;\n"

      "blt x18, %2, ..L_100;\n"
      :
      : "r" (a_array), "r" (i), "r" (tmp), "r" (*pause), "r" (b_array)
      : "x18", "x19", "x20", "x21", "x22", "x23", "x24" , "x25"
    );
}
#endif
