CC=gcc
CFLAGS=-march=native -g -Wall -O2
LDFLAGS=-lm
all: demo.o robot.o
        $(CC) $(CFLAGS) $(LDFLAGS) demo.o robot.o -o demo
demo.o: src/demo.c
        $(CC) $(CFLAGS) -c src/demo.c -o demo.o
robot.o: src/robot.c
        $(CC) $(CFLAGS) -c src/robot.c -o robot.o
clean:
        rm *.o
        rm demo 
