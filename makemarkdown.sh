#!/bin/bash

for f in Python/images/*; do
    convert $f -trim $f
done

printf "Messing around with computational geometry.\n\n" > README.md

for f in Python/images/*; do
    echo $(identify $f)
    width=350
    height=350
    # width=($((($(identify $f))[2] | tr 'x' ' ')))[0]
    # height=($((($(identify $f))[2] | tr 'x' ' ')))[1]
    printf "<img src=\"$f\" alt=\"$f\" width=\"$width\" height=\"$height\">\n" >> README.md
done
