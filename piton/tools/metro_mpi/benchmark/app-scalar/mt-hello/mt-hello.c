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

//**************************************************************************
// Multi-threaded axpy benchmark
//--------------------------------------------------------------------------
// 
//  
//
//
// This benchmark This benchmark runs several AXPY operations in parallle. AXPY is a Level 1 operation in the
// Basic Linear Algebra Subprograms (BLAS) package, and is a common operation in
//computations with vector processors. AXPY is a combination of scalar
//multiplication and vector addition. The input data (and reference data) should be generated
// using the axpy_gendata.pl perl script 

//--------------------------------------------------------------------------
// Includes 

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include "custom_def.h"


//--------------------------------------------------------------------------
// Input/Reference Data


 

//--------------------------------------------------------------------------
// Basic Utilities and Multi-thread Support

#include "util.h"

   



//--------------------------------------------------------------------------
// Main
//
// all threads start executing thread_entry(). Use their "coreid" to
// differentiate between threads (each thread is running on a separate core).
  
//int thread_entry(int cid, int nc){
//int main(int argc, char** argv) {

//   uint32_t cid, nc;
//   cid = argv[0][0];
 //  nc = argv[0][1];


int MAIN(){
    INIT_CID();

  // synchronization variable
  volatile static uint32_t amo_cnt = 0;

  // synchronize with other cores and wait until it is this core's turn
  while(argv[0][0] != amo_cnt);

  // assemble number and print
  printf("Hello world, this is hart %d of %d harts!\n", argv[0][0], argv[0][1]);

  // increment atomic counter
  ATOMIC_OP(amo_cnt, 1, add, w);

   if (argv[0][0] == argv[0][1]-1) {
  
    exit(0);
  }
  

  return 0;
}
