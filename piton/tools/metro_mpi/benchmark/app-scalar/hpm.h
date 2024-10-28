/* -----------------------------------------------
 * Project Name   : OpenPiton + Lagarto
 * File           : all_stats.h
 * Organization   : Barcelona Supercomputing Center
 * Author(s)      : Noelia Oliete Escuin
 * Email(s)       : noelia.oliete@bsc.es
 * -----------------------------------------------*/
#ifndef __HPM_H
#define __HPM_H
#include "util.h"


#ifndef EXTERNAL_HPM_EVENTS
    #define EXTERNAL_HPM_EVENTS  10
#endif

static uint64_t cycles;
static uint64_t instructions;

static uint64_t sargantana_counters[31];

static void init_hpm() {
    write_csr(mhpmevent3,   1);
    write_csr(mhpmevent4,   2);
    write_csr(mhpmevent5,   3);
    write_csr(mhpmevent6,   4);
    write_csr(mhpmevent7,   5);
    write_csr(mhpmevent8,   6);
    write_csr(mhpmevent9,   7);
    write_csr(mhpmevent10,  8);
    write_csr(mhpmevent11,  9);
    write_csr(mhpmevent12, 10);
    write_csr(mhpmevent13, 11);
    write_csr(mhpmevent14, 12);
    write_csr(mhpmevent15, 13);
    write_csr(mhpmevent16, 14);
    write_csr(mhpmevent17, 15);
    write_csr(mhpmevent18, 16);
    write_csr(mhpmevent19, 17);
    write_csr(mhpmevent20, 18);
#if EXTERNAL_HPM_EVENTS > 0    
    write_csr(mhpmevent21, 41);
    write_csr(mhpmevent22, 42);
    write_csr(mhpmevent23, 43);
    write_csr(mhpmevent24, 44);
#endif
    
#if EXTERNAL_HPM_EVENTS == 10    
    write_csr(mhpmevent25, 45);
    write_csr(mhpmevent26, 46);
    write_csr(mhpmevent27, 47);
    write_csr(mhpmevent28, 48);
    write_csr(mhpmevent29, 49);
    write_csr(mhpmevent30, 50);
#endif

#if EXTERNAL_HPM_EVENTS == 6   
    write_csr(mhpmevent25, 45);
    write_csr(mhpmevent26, 46); 
#endif
    
    
}


uint32_t roi_start (void){

    init_hpm();
    cycles = read_csr(mcycle);
    instructions = read_csr(minstret);
    sargantana_counters[3] = read_csr(mhpmcounter3);
    sargantana_counters[4] = read_csr(mhpmcounter4);
    sargantana_counters[5] = read_csr(mhpmcounter5);
    sargantana_counters[6] = read_csr(mhpmcounter6);
    sargantana_counters[7] = read_csr(mhpmcounter7);
    sargantana_counters[8] = read_csr(mhpmcounter8);
    sargantana_counters[9] = read_csr(mhpmcounter9);
    sargantana_counters[10] = read_csr(mhpmcounter10);
    sargantana_counters[11] = read_csr(mhpmcounter11);
    sargantana_counters[12] = read_csr(mhpmcounter12);
    sargantana_counters[13] = read_csr(mhpmcounter13);
    sargantana_counters[14] = read_csr(mhpmcounter14);
    sargantana_counters[15] = read_csr(mhpmcounter15);
    sargantana_counters[16] = read_csr(mhpmcounter16);
    sargantana_counters[17] = read_csr(mhpmcounter17);
    sargantana_counters[18] = read_csr(mhpmcounter18);
    sargantana_counters[19] = read_csr(mhpmcounter19);
    sargantana_counters[10] = read_csr(mhpmcounter20);
    sargantana_counters[21] = read_csr(mhpmcounter21);
    sargantana_counters[22] = read_csr(mhpmcounter22);
    sargantana_counters[23] = read_csr(mhpmcounter23);
    sargantana_counters[24] = read_csr(mhpmcounter24);
    sargantana_counters[25] = read_csr(mhpmcounter25);
    sargantana_counters[26] = read_csr(mhpmcounter26);
    sargantana_counters[27] = read_csr(mhpmcounter27);
    sargantana_counters[28] = read_csr(mhpmcounter28);
    sargantana_counters[29] = read_csr(mhpmcounter29);
    sargantana_counters[30] = read_csr(mhpmcounter30);


    return 0; 
}

uint32_t roi_end (void){
    cycles = read_csr(mcycle) - cycles;
    instructions = read_csr(minstret) - instructions;
    sargantana_counters[3] = read_csr(mhpmcounter3) - sargantana_counters[3];
    sargantana_counters[4] = read_csr(mhpmcounter4) - sargantana_counters[4];
    sargantana_counters[5] = read_csr(mhpmcounter5) - sargantana_counters[5];
    sargantana_counters[6] = read_csr(mhpmcounter6) - sargantana_counters[6];
    sargantana_counters[7] = read_csr(mhpmcounter7) - sargantana_counters[7];
    sargantana_counters[8] = read_csr(mhpmcounter8) - sargantana_counters[8];
    sargantana_counters[9] = read_csr(mhpmcounter9) - sargantana_counters[9];
    sargantana_counters[10] = read_csr(mhpmcounter10) - sargantana_counters[10];
    sargantana_counters[11] = read_csr(mhpmcounter11) - sargantana_counters[11];
    sargantana_counters[12] = read_csr(mhpmcounter12) - sargantana_counters[12];
    sargantana_counters[13] = read_csr(mhpmcounter13) - sargantana_counters[13];
    sargantana_counters[14] = read_csr(mhpmcounter14) - sargantana_counters[14];
    sargantana_counters[15] = read_csr(mhpmcounter15) - sargantana_counters[15];
    sargantana_counters[16] = read_csr(mhpmcounter16) - sargantana_counters[16];
    sargantana_counters[17] = read_csr(mhpmcounter17) - sargantana_counters[17];
    sargantana_counters[18] = read_csr(mhpmcounter18) - sargantana_counters[18];
    sargantana_counters[19] = read_csr(mhpmcounter19) - sargantana_counters[19];
    sargantana_counters[10] = read_csr(mhpmcounter20) - sargantana_counters[10];
    sargantana_counters[21] = read_csr(mhpmcounter21) - sargantana_counters[21];
    sargantana_counters[22] = read_csr(mhpmcounter22) - sargantana_counters[22];
    sargantana_counters[23] = read_csr(mhpmcounter23) - sargantana_counters[23];
    sargantana_counters[24] = read_csr(mhpmcounter24) - sargantana_counters[24];
    sargantana_counters[25] = read_csr(mhpmcounter25) - sargantana_counters[25];
    sargantana_counters[26] = read_csr(mhpmcounter26) - sargantana_counters[26];
    sargantana_counters[27] = read_csr(mhpmcounter27) - sargantana_counters[27];
    sargantana_counters[28] = read_csr(mhpmcounter28) - sargantana_counters[28];
    sargantana_counters[29] = read_csr(mhpmcounter29) - sargantana_counters[29];
    sargantana_counters[30] = read_csr(mhpmcounter30) - sargantana_counters[30];
    return 0; 
}

uint32_t print_metrics (char *test_name ){
    printf("\n");
    printf("--  %s  -- \n", test_name);
    printf("Cycles:  %d \n", cycles);
    printf("Instructions:  %d \n\n", instructions);

    printf("\n*** BRANCHES ***\n");
    printf("Branch Misses:  %d \n", sargantana_counters[3]);
    printf("Branches Executed:  %d \n", sargantana_counters[4]);
    printf("Branches Taken:  %d \n", sargantana_counters[5]);
    
    printf("\n*** LOAD/STORES ***\n");
    printf("Stores Executed:  %d \n", sargantana_counters[6]);
    printf("Loads Executed:  %d \n", sargantana_counters[7]);

    printf("\n*** L1 iCache ***\n");
    printf("iCache Requests:  %d \n", sargantana_counters[8]);
    printf("iCache Kills:  %d \n", sargantana_counters[9]);
    printf("iCache Miss Kills:  %d \n", sargantana_counters[16]);
    printf("iCache Busy:  %d \n", sargantana_counters[17]);
    printf("iCache Miss Time:  %d \n", sargantana_counters[18]);

    printf("\n*** PIPELINE ***\n");
    printf("Fetch Stalls:  %d \n", sargantana_counters[10]);
    printf("Decode Stalls:  %d \n", sargantana_counters[11]);
    printf("Read Register Stalls:  %d \n", sargantana_counters[12]);
    printf("Execute Stalls:  %d \n", sargantana_counters[13]);
    printf("Writeback Stalls:  %d \n", sargantana_counters[14]);
    printf("Stalls by Data Dependencies:  %d \n", sargantana_counters[20]);
    printf("Cycles of Load blocked by Store:  %d \n", sargantana_counters[19]);


#if EXTERNAL_HPM_EVENTS > 0  
    printf ("\n*** L2 ***\n");    
    printf("miss:  %d \n", sargantana_counters[21]);
    printf("access:    %d \n", sargantana_counters[22]);
    printf ("\n*** L15 ***\n");       
    printf("miss:  %d \n", sargantana_counters[23]);
    printf("access:    %d \n", sargantana_counters[24]);
#endif

#if EXTERNAL_HPM_EVENTS == 10    
    printf ("\n*** NoCs flit cnt ***\n");      
    printf("NoC1 :  %d \n", sargantana_counters[25]);
    printf("NoC2 :  %d \n", sargantana_counters[26]);
    printf("NoC3 :  %d \n", sargantana_counters[27]);
    printf ("\n*** NoCs stall ***\n");  
    printf("NoC1 :  %d \n", sargantana_counters[28]);
    printf("NoC2 :  %d \n", sargantana_counters[29]);
    printf("NoC3 :  %d \n", sargantana_counters[30]);
#endif

#if EXTERNAL_HPM_EVENTS == 4
    printf ("\n*** NoCs ***\n");      
    printf("NoCs flit cnt:  %d \n", sargantana_counters[25]);
    printf("NoCs stall cnt:  %d \n", sargantana_counters[26]);
#endif    
        

    return 0; 
}



#define pmu_stats(code, iter) do { \
    roi_start(); \
    code; \
    roi_end(); \
    if(argv[0][0] == nc-1) { \
        print_metrics(stringify(code));\
    }\
    BARRIER();\
} while(0)


#endif


