CC=gcc
CFLAGS=-march=native -g -Wall -O2
all: demo

demo: src/demo.c robot.o
	$(CC) $(CFLAGS) src/demo.c robot.o -o demo -lm
robot.o: src/robot.c 
	$(CC) $(CFLAGS) -c src/robot.c -o robot.o
