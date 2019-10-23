#!/bin/bash

for f in Python/images/*; do
    convert $f -trim $f
done

echo "Messing around with computational geometry." > README.md

for f in Python/images/*; do
    echo "<img src=\"$f\" alt=\"$f\" width="350" height="350">" >> README.md
done
