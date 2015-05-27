#!/bin/bash
CAMERA_SERIAL=$(ls /dev/ | grep xtion | grep color | head -1 - | sed 's/xtion_//g' | sed 's/_color//g')

while [[ $# > 0 ]]
do
    key="$1"

    case $key in
        -c|-color|--color)
            CAMERA_SERIAL=$(ls /dev/xtion_* | grep color | head -1 -)
            shift
            ;;
        -d|-depth|--depth)
            CAMERA_SERIAL=$(ls /dev/xtion_* | grep depth | head -1 - | awk '{print $1}')
            shift
            ;;
    esac
    shift
done
echo -ne $CAMERA_SERIAL
