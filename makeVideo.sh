ffmpeg -r 8 -i "VideoFrames/${1}/Frame_%07d.png" -s 1834x1001 -an "${1}.mov"
