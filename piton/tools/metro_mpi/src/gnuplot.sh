#!/bin/bash

# Copyright (c) 2024, Barcelona Supercomputing Center
# Contact: alireza.monemi [at] bsc [dot] es
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the copyright holder nor the names
#       of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

allThreads=(1 2 4 8 16 32 64 128) 

gnuplot -persist <<-EOFMarker

set terminal png size 800,800 enhanced font "Helvetica,20"
set output 'output.png'

red = "#FF0000"; green = "#00FF00"; blue = "#0000FF"; skyblue = "#880088";
set yrange [0:]
set style data histogram
set style histogram cluster gap 1
set style fill solid
#set boxwidth 0.9
set xtics format ""
set grid ytics
set xtics rotate by 60 right
set xtics font ", 11"
set ytics font ", 11"
set xlabel font ", 16"
set ylabel font ", 16"
set key  bmargin horizontal reverse samplen 1 width -4 maxrows 1 maxcols 12
set key font ", 14"
#set size square 0.95,0.95
set bmargin at screen 0.35
set lmargin 10
#set rmargin 0
#set tmargin 0
set ylabel "Count"

set title "A Sample Bar Chart"
plot "bar.dat" using 2:xtic(1) title "M0-ariane-L2(64k)-L15(8K)" ,   \
     "bar.dat" using 3 title "M0-ariane-L2(64k)-L15(8K)" ,   \
     "bar.dat" using 4 title "JM0-ariane-L2(64k)-L15(8K)" ,    \
     "bar.dat" using 5 title "M0-ariane-L2(64k)-L15(8K)" linecolor rgb skyblue
     
EOFMarker
