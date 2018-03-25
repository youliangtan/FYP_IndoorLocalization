#!/bin/bash
# Make default camera /dev/video0 point to the "best" camera present.

if [ -h /dev/media1 ]; then 
   sudo rm /dev/media1   # not first run: remove our old symlink
elif [ -e /dev/media1 ]; then
   sudo mv /dev/media1 /dev/media1.original   # first run: rename original video0
fi 
if [ -e /dev/media0 ]; then
   sudo ln -s /dev/media0 /dev/media1   # symlink to video1 since it exists
   echo "Set default camera /dev/media1 --> external camera /dev/media0"
elif [ -e /dev/media1.original ]; then  # symlink to video0.original otherwise
   sudo ln -s /dev/media1.original /dev/media1
   echo "Set default camera /dev/media0 --> integrated camera /dev/media1.original"
else
   echo "Sorry, does this machine have no camera devices?"
   ls -l /dev/video*
fi
