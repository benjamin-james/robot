CC=gcc
CFLAGS=-g -Wall -ansi -pedantic -O2
LDFLAGS=-lm
all: pc_demo.o robot.o
	$(CC) $(CFLAGS) $(LDFLAGS) pc_demo.o robot.o -o pc_demo
pc_demo.o: src/pc_demo.c
	$(CC) $(CFLAGS) -c src/pc_demo.c -o pc_demo.o
robot.o: src/robot.c
	$(CC) $(CFLAGS) -c src/robot.c -o robot.o
clean:
	rm *.o
	rm pc_demo 
