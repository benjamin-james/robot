#!/bin/bash
# Usage: $0
echo "Opening /dev/ttyACM0"
stty -F /dev/ttyACM0 cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts
printf ">"
while read line
do
    if [ "$line" == "exit" ] ; then
	exit
    fi
    echo $line > /dev/ttyACM0
    tail -f /dev/ttyACM0
    printf ">"
done
exit
