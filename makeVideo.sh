ffmpeg -r 8 -i "VideoFrames/${1}/Frame_%07d.png" -qscale:v 15 "${1}.avi"
