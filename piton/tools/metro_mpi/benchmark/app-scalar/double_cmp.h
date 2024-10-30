/*
 * Copyright (c) 2024, Barcelona Supercomputing Center
 * Contact: alireza.monemi   [at] bsc [dot] es *          
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
#ifndef __DOUBLE_CMP_H
#define __DOUBLE_CMP_H

#ifndef  DATA_TYPE   
  typedef double data_t;
#endif


int compare_double(data_t f1, data_t f2)
 {
  data_t precision =(data_t) 0.000001;
  if (((f1 - precision) > f2) ||  ((f1 + precision) < f2)) return 0;
  return 1;  
 }


int verify_Double(int n,   data_t* test,   data_t* verify)
{
  int i;
  for (i = 0; i < n; i++)
  {
    if( !compare_double(test[i],verify[i])){
    //  printf("Error: n=%u,%f!=%f\n",i,test[i],verify[i]);
      return i+1;
    }
  }
    return 0;
}


int mt_verify(const size_t coreid, const size_t ncores,const size_t lda , data_t* test,   data_t* verify ){

    size_t i, k, block, start, end;
      block = lda / ncores;
    if ((block*ncores) != lda) block++;
    start = block * coreid;
    end   = start + block;
     if (end > lda) end = lda;
    if (start> lda) return 0; 
    return verify_Double(end-start, test + start,  verify+ start);
}

#endif
