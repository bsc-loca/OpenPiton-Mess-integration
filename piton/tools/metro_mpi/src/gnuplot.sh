#!/bin/bash


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
