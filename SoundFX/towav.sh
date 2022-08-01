#!/bin/bash
for fname in "$@"
do
    echo "convert ${fname} to wav ..."
    ffmpeg -i ${fname} -ar 22050 -ac 1 ${fname%.*}.wav
done
