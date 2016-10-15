#!/bin/sh
echo set title '"'$1'"' > plotfile.gnu
#echo set yrange [40:90] > plotfile.gnu
echo plot '"'$1'"' using 1 title "'Ti temp'" with lines linetype 1, "''" using 2 title "'heater'" with lines linetype 3, "''" using 3 title "'relay state'" with lines linetype 4, "''" using 4 title "'error'" with lines linetype 5  >>plotfile.gnu
echo pause -1 >>plotfile.gnu
echo save \"diagramm.gnu\"
#Step 2: call gnuplot
gnuplot plotfile.gnu

