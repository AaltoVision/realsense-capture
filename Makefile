CC=gcc

CFLAGS=-std=c++14 -Wall -Wextra -O2 -lstdc++

CFLAGS+=-I./librealsense/include
CFLAGS+=-I./odometry/recorder
CFLAGS+=-I./odometry/lib/json/include

LDFLAGS=-lSDL2 -L./librealsense/build -lrealsense2 -lm
LDFLAGS+=-L./odometry/target/recorder -lrecorder
LDFLAGS+=-L./odometry/target -lloguru

SOURCES=src/main.cpp
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=main

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(EXECUTABLE) $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) -c -o $@ $<

clean:
	rm main || rm src/*.o

run:
	LD_LIBRARY_PATH=/Users/jerry/git/librealsense/build ./main
