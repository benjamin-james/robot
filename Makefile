CC=gcc

all: demo

demo: src/demo.c robot.o
	$(CC) -g -Wall src/demo.c robot.o -O3 -o demo -lm
robot.o: src/robot.c 
	$(CC) -g -Wall -c src/robot.c -O3 -o robot.o
