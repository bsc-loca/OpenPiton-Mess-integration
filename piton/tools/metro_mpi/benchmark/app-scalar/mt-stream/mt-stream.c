// See LICENSE for license details.

//**************************************************************************
// Multi-threaded stream benchmark
//--------------------------------------------------------------------------
// 
//  
//
//


//--------------------------------------------------------------------------
// Includes 

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include "custom_def.h"


//--------------------------------------------------------------------------
// Input/Reference Data

#include "dataset.h"
 

//--------------------------------------------------------------------------
// Basic Utilities and Multi-thread Support

#include "util.h"


   
//--------------------------------------------------------------------------
// axpy function
 
extern void __attribute__((noinline)) stream (const size_t coreid, const size_t ncores,const size_t lda , data_t A[], data_t B[], const int p, const int l );



int MAIN(){
   INIT_CID();
   if(cid==0) {printf("We are %d cores. Rd ratio=%d  Pause=%d Loop=%d \n",nc, RD_RATIO,PAUSE, LOOP);}  
   BARRIER();
   STATS(stream(cid,nc,ARRAY_SIZE,input1_data,input2_data, PAUSE, LOOP); BARRIER(),ARRAY_SIZE);
   BARRIER();
   exit(0);  
}
