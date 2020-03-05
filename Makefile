CC=gcc
CFLAGS=-I/Users/jerry/git/librealsense/include
LDFLAGS=-lSDL2 -L/Users/jerry/git/librealsense/build -lrealsense2 -lm
SOURCES=src/main.cpp
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=main

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) -std=c++11 -lstdc++ $(CFLAGS) -o $(EXECUTABLE) $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	$(CC) -std=c++11 -lstdc++ $(CFLAGS) $(LDFLAGS) -c -o $@ $<

clean:
	rm main || rm src/*.o

run:
	LD_LIBRARY_PATH=/Users/jerry/git/librealsense/build ./main
