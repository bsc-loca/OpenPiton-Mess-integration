// See LICENSE for license details.

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
