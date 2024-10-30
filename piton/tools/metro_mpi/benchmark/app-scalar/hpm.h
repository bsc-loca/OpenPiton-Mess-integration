/*
 * Copyright (c) 2024, Barcelona Supercomputing Center
 * Contact: alireza.monemi   [at] bsc [dot] es
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
#ifndef __HPM_H
#define __HPM_H
#include "util.h"


#ifndef EXTERNAL_HPM_EVENTS
    #define EXTERNAL_HPM_EVENTS  10
#endif

static uint64_t cycles;
static uint64_t instructions;

static uint64_t hpm_counters[31];

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
    hpm_counters[3] = read_csr(mhpmcounter3);
    hpm_counters[4] = read_csr(mhpmcounter4);
    hpm_counters[5] = read_csr(mhpmcounter5);
    hpm_counters[6] = read_csr(mhpmcounter6);
    hpm_counters[7] = read_csr(mhpmcounter7);
    hpm_counters[8] = read_csr(mhpmcounter8);
    hpm_counters[9] = read_csr(mhpmcounter9);
    hpm_counters[10] = read_csr(mhpmcounter10);
    hpm_counters[11] = read_csr(mhpmcounter11);
    hpm_counters[12] = read_csr(mhpmcounter12);
    hpm_counters[13] = read_csr(mhpmcounter13);
    hpm_counters[14] = read_csr(mhpmcounter14);
    hpm_counters[15] = read_csr(mhpmcounter15);
    hpm_counters[16] = read_csr(mhpmcounter16);
    hpm_counters[17] = read_csr(mhpmcounter17);
    hpm_counters[18] = read_csr(mhpmcounter18);
    hpm_counters[19] = read_csr(mhpmcounter19);
    hpm_counters[10] = read_csr(mhpmcounter20);
    hpm_counters[21] = read_csr(mhpmcounter21);
    hpm_counters[22] = read_csr(mhpmcounter22);
    hpm_counters[23] = read_csr(mhpmcounter23);
    hpm_counters[24] = read_csr(mhpmcounter24);
    hpm_counters[25] = read_csr(mhpmcounter25);
    hpm_counters[26] = read_csr(mhpmcounter26);
    hpm_counters[27] = read_csr(mhpmcounter27);
    hpm_counters[28] = read_csr(mhpmcounter28);
    hpm_counters[29] = read_csr(mhpmcounter29);
    hpm_counters[30] = read_csr(mhpmcounter30);


    return 0; 
}

uint32_t roi_end (void){
    cycles = read_csr(mcycle) - cycles;
    instructions = read_csr(minstret) - instructions;
    hpm_counters[3] = read_csr(mhpmcounter3) - hpm_counters[3];
    hpm_counters[4] = read_csr(mhpmcounter4) - hpm_counters[4];
    hpm_counters[5] = read_csr(mhpmcounter5) - hpm_counters[5];
    hpm_counters[6] = read_csr(mhpmcounter6) - hpm_counters[6];
    hpm_counters[7] = read_csr(mhpmcounter7) - hpm_counters[7];
    hpm_counters[8] = read_csr(mhpmcounter8) - hpm_counters[8];
    hpm_counters[9] = read_csr(mhpmcounter9) - hpm_counters[9];
    hpm_counters[10] = read_csr(mhpmcounter10) - hpm_counters[10];
    hpm_counters[11] = read_csr(mhpmcounter11) - hpm_counters[11];
    hpm_counters[12] = read_csr(mhpmcounter12) - hpm_counters[12];
    hpm_counters[13] = read_csr(mhpmcounter13) - hpm_counters[13];
    hpm_counters[14] = read_csr(mhpmcounter14) - hpm_counters[14];
    hpm_counters[15] = read_csr(mhpmcounter15) - hpm_counters[15];
    hpm_counters[16] = read_csr(mhpmcounter16) - hpm_counters[16];
    hpm_counters[17] = read_csr(mhpmcounter17) - hpm_counters[17];
    hpm_counters[18] = read_csr(mhpmcounter18) - hpm_counters[18];
    hpm_counters[19] = read_csr(mhpmcounter19) - hpm_counters[19];
    hpm_counters[10] = read_csr(mhpmcounter20) - hpm_counters[10];
    hpm_counters[21] = read_csr(mhpmcounter21) - hpm_counters[21];
    hpm_counters[22] = read_csr(mhpmcounter22) - hpm_counters[22];
    hpm_counters[23] = read_csr(mhpmcounter23) - hpm_counters[23];
    hpm_counters[24] = read_csr(mhpmcounter24) - hpm_counters[24];
    hpm_counters[25] = read_csr(mhpmcounter25) - hpm_counters[25];
    hpm_counters[26] = read_csr(mhpmcounter26) - hpm_counters[26];
    hpm_counters[27] = read_csr(mhpmcounter27) - hpm_counters[27];
    hpm_counters[28] = read_csr(mhpmcounter28) - hpm_counters[28];
    hpm_counters[29] = read_csr(mhpmcounter29) - hpm_counters[29];
    hpm_counters[30] = read_csr(mhpmcounter30) - hpm_counters[30];
    return 0; 
}

uint32_t print_metrics (char *test_name ){
    printf("\n");
    printf("--  %s  -- \n", test_name);
    printf("Cycles:  %d \n", cycles);
    printf("Instructions:  %d \n\n", instructions);

    printf("\n*** BRANCHES ***\n");
    printf("Branch Misses:  %d \n", hpm_counters[3]);
    printf("Branches Executed:  %d \n", hpm_counters[4]);
    printf("Branches Taken:  %d \n", hpm_counters[5]);
    
    printf("\n*** LOAD/STORES ***\n");
    printf("Stores Executed:  %d \n", hpm_counters[6]);
    printf("Loads Executed:  %d \n", hpm_counters[7]);

    printf("\n*** L1 iCache ***\n");
    printf("iCache Requests:  %d \n", hpm_counters[8]);
    printf("iCache Kills:  %d \n", hpm_counters[9]);
    printf("iCache Miss Kills:  %d \n", hpm_counters[16]);
    printf("iCache Busy:  %d \n", hpm_counters[17]);
    printf("iCache Miss Time:  %d \n", hpm_counters[18]);

    printf("\n*** PIPELINE ***\n");
    printf("Fetch Stalls:  %d \n", hpm_counters[10]);
    printf("Decode Stalls:  %d \n", hpm_counters[11]);
    printf("Read Register Stalls:  %d \n", hpm_counters[12]);
    printf("Execute Stalls:  %d \n", hpm_counters[13]);
    printf("Writeback Stalls:  %d \n", hpm_counters[14]);
    printf("Stalls by Data Dependencies:  %d \n", hpm_counters[20]);
    printf("Cycles of Load blocked by Store:  %d \n", hpm_counters[19]);


#if EXTERNAL_HPM_EVENTS > 0  
    printf ("\n*** L2 ***\n");    
    printf("miss:  %d \n", hpm_counters[21]);
    printf("access:    %d \n", hpm_counters[22]);
    printf ("\n*** L15 ***\n");       
    printf("miss:  %d \n", hpm_counters[23]);
    printf("access:    %d \n", hpm_counters[24]);
#endif

#if EXTERNAL_HPM_EVENTS == 10    
    printf ("\n*** NoCs flit cnt ***\n");      
    printf("NoC1 :  %d \n", hpm_counters[25]);
    printf("NoC2 :  %d \n", hpm_counters[26]);
    printf("NoC3 :  %d \n", hpm_counters[27]);
    printf ("\n*** NoCs stall ***\n");  
    printf("NoC1 :  %d \n", hpm_counters[28]);
    printf("NoC2 :  %d \n", hpm_counters[29]);
    printf("NoC3 :  %d \n", hpm_counters[30]);
#endif

#if EXTERNAL_HPM_EVENTS == 4
    printf ("\n*** NoCs ***\n");      
    printf("NoCs flit cnt:  %d \n", hpm_counters[25]);
    printf("NoCs stall cnt:  %d \n", hpm_counters[26]);
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


