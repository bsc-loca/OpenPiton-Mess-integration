
#include "dataset.h"
#include "util.h"
#include <stddef.h>


#include "utils.c"


void stream(const ssize_t coreid, const ssize_t ncores,const ssize_t lda , data_t A[], data_t B[], const int pause, const int loop ){

    ssize_t i, block, start, end;
       
    block = lda / ncores;
    if ((block*ncores) != lda) block++;
    start = block * coreid;
    end   = start + block;
    if (end > lda) return; //should assert error instead
       
    
   for (uint64_t i = 0; i < loop; ++i)
   {   
    	STREAM_copy (&A[start], &B[start], &block, &pause);  
   }        
}





