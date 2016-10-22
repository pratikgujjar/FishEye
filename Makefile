#   Makefile - universe project
#   version 3
#   Richard Vaughan  

# this should work on Linux with MESA
#GLUTLIBS = -L/usr/X11R6/lib -lGLU -lGL -lglut -lX11 -lXext -lXmu -lXi
#GLUTFLAGS = -I/usr/local/include/GL

# this works on Mac OS X
#GLUTFLAGS = -framework OpenGL -framework GLUT

LIBS = -L/usr/lib/x86_64-linux-gnu/ -lGLEW -lglut -lGLU -lGL
# -L /usr/lib/x86_64-linux-gnu/libGL.so -L /usr/lib/x86_64-linux-gnu/libglut.so.3 -L /usr/lib/x86_64-linux-gnu/libglut.so.3.9.0 -L /usr/lib/x86_64-linux-gnu/libGLU.so -L /usr/lib/x86_64-linux-gnu/libGLU.a -L /usr/lib/x86_64-linux-gnu/libGLU.so.1 -L /usr/lib/x86_64-linux-gnu/libGLU.so.1.3.1 -L /usr/lib/x86_64-linux-gnu/libglut.a -L /usr/lib/x86_64-linux-gnu/libglut.so.3 -lm 

CC = g++ -std=c++11
CXXFLAGS = -g -Wall -O3
# LIBS =  -g $(GLUTLIBS)

SRC = universe.h universe.cc controller.cc

all: universe

universe: $(SRC)
	$(CC) $(CXXFLAGS) -o $@ $(SRC) $(LIBS) 

clean:
	rm *.o universe

