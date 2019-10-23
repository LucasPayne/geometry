#!/bin/bash

rm Python/images/created/*

find Python/images/ -type f -name "*.png" -o -name "*.gif" | while IFS= read -r f; do
    created_filename="Python/images/created/$(basename "$f")"
    convert "$f" -trim -colorspace Gray "$created_filename"
done

printf "Messing around with computational geometry.\n\n" > README.md

find Python/images/created/ -type f -name "*.png" -o -name "*.gif" | while IFS= read -r f; do
    dimensions=$(identify "$f" | awk '{print $3; exit}')
    width=$(echo $dimensions | awk -F "x" '{print $1}')
    height=$(echo $dimensions | awk -F "x" '{print $2}')
    normalizing_height=300

    # Normalize to a specific height
    mul=$(echo "($normalizing_height * 10000) / $height" | bc)

    height=$normalizing_height
    width=$(echo "($width * $mul) / 10000" | bc)

    printf "<img src=\"$f\" alt=\"$f\" width=\"$width\" height=\"$height\"> " >> README.md
done
