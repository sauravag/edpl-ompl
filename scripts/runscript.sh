#!/bin/bash
# Basic while loop
counter=0
while [ $counter -lt 10 ]
do
echo $counter
./bin/Debug/./FIRM-OMPL
let counter=counter+1
done
echo All done
