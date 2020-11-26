#!/bin/bash

for i in {1..9}
do
if [ $(($i%4)) = 0 ]; then
        echo skydip start
        ipython ../../necst-1p85m2019/scripts/skydip.py
        echo skydip end
        sleep 30
fi
echo otf start
ipython otf_scan.py
echo otf end
sleep 180
done

