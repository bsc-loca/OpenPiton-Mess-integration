#ifndef __CUSTOM_DEF_H
#define __CUSTOM_DEF_H

    #ifdef REPORT_CUSTOM_HPM 
        #include "custom_hpm.h"
        #define STATS     pmu_stats
    #endif
    
    
    #ifdef REPORT_HPM_METRICS 
        #include "hpm.h"
        #define STATS     pmu_stats
    #endif

    #ifdef REPORT_OP_METRICS
        #include "all_stats.h"
        #define STATS     all_stats
    #endif

    #ifndef STATS
        #define STATS     stats
    #endif


    #if defined(COYOTE_TILE)   	
     	 
          #define BARRIER()   simfence()
          #define MAIN()      thread_entry(int cid, int nc) 
          #define INIT_CID()
          
   #elif  defined(LOX_TILE)   
  
         #define BARRIER()   barrier(nc)
         #define MAIN()      main(int argc, int** argv)  
         #define INIT_CID()  uint32_t cid, nc; \
         cid = argv[0][0]; \
         nc = argv[0][1];          
  
   #else  //Ariane   lagarto ...

         #define BARRIER()   barrier(nc)
         #define MAIN()      main(int argc, int** argv)  
         #define INIT_CID()  uint32_t cid, nc; \
         cid = argv[0][0]; \
         nc = argv[0][1];          
               
    #endif //else
     
#endif

