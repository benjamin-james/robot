#!/bin/bash
# Usage: $0 /dev/ttywhatever
echo "Opening $1"
stty -F $1 cs8 9600 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts
printf ">"
while read line
do
    if [ "$line" == "exit" ] ; then
	exit
    fi
    echo $line > $1
    tail -f $1
    printf ">"
done
exit
