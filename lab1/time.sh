#!/bin/sh
i="0"
while [ $i -le 300 ]
do
    echo -n "a" > /dev/ttyACM0
    sleep 1
    i=$[$i+1]
done
