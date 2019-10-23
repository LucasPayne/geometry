#!/bin/bash

for f in Python/images/*; do
    convert $f -trim $f
done

printf "Messing around with computational geometry.\n\n" > README.md

for f in Python/images/*; do
    dimensions=$(identify $f | awk '{print $3; exit}')
    width=$(echo $dimensions | awk -F "x" '{print $1}')
    height=$(echo $dimensions | awk -F "x" '{print $2}')

    # Normalize to a specific height
    mul=$(echo "($height * 10000) / 500" | bc)
    height=500
    width=$(echo "($width * $mul) / 10000" | bc)

    printf "<img src=\"$f\" alt=\"$f\" width=\"$width\" height=\"$height\">\n" >> README.md
done
