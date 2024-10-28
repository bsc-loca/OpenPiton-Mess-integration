/* -----------------------------------------------
 * Project Name   : OpenPiton + Lagarto
 * File           : all_stats.h
 * Organization   : Barcelona Supercomputing Center
 * Author(s)      : Noelia Oliete Escuin
 * Email(s)       : noelia.oliete@bsc.es
 * -----------------------------------------------*/
#ifndef __ALL_STATS_H
#define __ALL_STATS_H
#include "cache_metrics.h"
#include "util.h"

typedef struct MY_STATIC {
  unsigned long cycle;
  unsigned long max_val;
  unsigned long min_val;
  unsigned long avg_val;
  int      max_core;
  int      min_core;
} MY_STATIC_t;

static MY_STATIC_t instruction_st_; 

#define all_stats(code, iter) do { \
    volatile static uint32_t _amo_cnt = 0; \
    reset_L2_metrics(cid); \
    unsigned long _c = -read_csr(mcycle), _i = -read_csr(minstret); init_L2_metrics(cid); \
    code; \
    stop_L2_metrics(cid); \
    _c += read_csr(mcycle), _i += read_csr(minstret); \
    unsigned long _access = read_L2_access(cid), _miss = read_L2_misses(cid); \
    if(argv[0][0] == 0){ \
    	instruction_st_.max_val=_i; instruction_st_.min_val=_i; instruction_st_.avg_val=0; \
    	instruction_st_.max_core=0; instruction_st_.min_core=0; instruction_st_.cycle=_c;\
    	printf("\n%s: \n", stringify(code)); \
    	/*printf("--Stats---\nCID, cycles, cycles/iter, CPI, L2_access, L2_mis\n");*/\
    }\
    while(argv[0][0] != _amo_cnt); \
   /* printf("%d, %ld,  %ld.%ld, %ld.%ld, %ld, %ld\n",cid, _c, _c/iter, 10*_c/iter%10, _c/_i, 10*_c/_i%10, _access,_miss );*/\
    if(instruction_st_.max_val < _i ){ instruction_st_.max_val=_i;instruction_st_.max_core=argv[0][0];} \
    if(instruction_st_.min_val > _i ){ instruction_st_.min_val=_i;instruction_st_.min_core=argv[0][0];} \
    instruction_st_.avg_val+=_i; \
    ATOMIC_OP(_amo_cnt, 1, add, w); \
   /* if(argv[0][0] == nc-1) printf("--Stats---\n");*/ \
    if(argv[0][0] == nc-1) { \
    printf("--Stats---\ncycles, inst_max, inst_min, inst_avg, inst_max_core, inst_min_core, avg_CPI\n"); \
    printf("%lu, %lu, %lu, %lu, %d, %d,%ld.%ld\n--Stats---\n", \
           instruction_st_.cycle, \
           instruction_st_.max_val,\
           instruction_st_.min_val,\
           instruction_st_.avg_val/=nc,\
           instruction_st_.max_core,\
           instruction_st_.min_core,\
           instruction_st_.cycle/instruction_st_.avg_val, 10*instruction_st_.cycle/instruction_st_.avg_val%10);} \
  } while(0)

#endif   //__ALL_STATS_H


