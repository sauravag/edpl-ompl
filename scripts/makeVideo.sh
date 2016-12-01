ffmpeg -r 8 -i "${1}/Frame_%07d.png" -s 1834x1001 -an "${2}.mov"
