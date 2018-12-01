CFLAGS=-Wall -g
COMPILER= g++

main: main.cpp
		${COMPILER} -std=c++11 main.cpp -o out `pkg-config --cflags --libs opencv` -fopenmp

read: read.cpp
		${COMPILER} -std=c++11 read.cpp -o out `pkg-config --cflags --libs opencv`
