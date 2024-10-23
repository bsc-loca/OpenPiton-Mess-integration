/*
Copyright (c) 2019 Princeton University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Princeton University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY PRINCETON UNIVERSITY "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL PRINCETON UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <iostream>
#include <iomanip>

const int YUMMY_NOC_1  = 0;
const int DATA_NOC_1   = 1;
const int YUMMY_NOC_2  = 2;
const int DATA_NOC_2   = 3;
const int YUMMY_NOC_3  = 4;
const int DATA_NOC_3   = 5;
const int TEST_FINISH  = 6;
const int DATA_ALL_NOC = 7;

uint64_t main_time = 0; // Current simulation time
uint64_t clk = 0;
int rank, dest, size;
short test_end=0;

void initialize();

// MPI Yummy functions
unsigned short mpi_receive_yummy(int origin, int flag);

void mpi_send_yummy(unsigned short message, int dest, int rank, int flag);
// MPI data&Valid functions
void mpi_send_data(unsigned long long data, unsigned char valid, int dest, int rank, int flag);

unsigned long long mpi_receive_data(int origin, unsigned short* valid, int flag);

int getRank();

int getSize();

void finalize();

unsigned short mpi_receive_finish();

void mpi_send_finish(unsigned short message, int rank);

typedef struct {
    unsigned long long data_0;
    unsigned long long data_1;
    unsigned long long data_2;
    unsigned short valid_0;
    unsigned short valid_1;
    unsigned short valid_2;
} mpi_noc_t;

void mpi_send_all_noc(unsigned long long data_0, unsigned char valid_0,
                      unsigned long long data_1, unsigned char valid_1,
                      unsigned long long data_2, unsigned char valid_2,
                      int dest, int rank, int flag);

mpi_noc_t mpi_receive_all_noc(int origin, int flag);

int main(int argc, char **argv, char **env) {
    std::cout << "Started" << std::endl << std::flush;

    // MPI work 
    initialize();
    rank = getRank();
    size = getSize();
    std::cout << "size: " << size << ", rank: " << rank <<  std::endl;
    if (rank==0) {
        dest = 1;
        std::cout << "Before sending " << std::endl;
        mpi_send_all_noc(0,0,0,0,0,0,dest,rank,DATA_ALL_NOC);
        std::cout << "After sending " << std::endl;
    } else {
        dest = 0;
        std::cout << "Before receving " << std::endl;
        mpi_noc_t aux = mpi_receive_all_noc(dest,DATA_ALL_NOC);
        std::cout << aux.valid_0 << std::endl;
        std::cout << "After Receiving " << std::endl;
    }
    finalize();
    exit(0);
}
