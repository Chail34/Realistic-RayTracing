# Makefile pour un unique exécutable
#
# Utilisation :
#   make && ./main
#   make clean && make && ./main
#
# Les dépendances sur les headers sont générées automatiquement (-MMD -MP) :
# modifier un .h (Scene.h, Sphere.h, ...) recompile les fichiers concernés.

CIBLE = main
SRCS  = main.cpp src/Camera.cpp src/Trackball.cpp src/imageLoader.cpp src/Mesh.cpp src/BVH.cpp
LIBS  = -lglut -lGLU -lGL -lm -lpthread

CXX      = g++
CXXFLAGS = -Wall -O3
CPPFLAGS = -I. -MMD -MP
LDFLAGS  =
LDLIBS   = $(LIBS)

OBJS = $(SRCS:.cpp=.o)
DEPS = $(OBJS:.o=.d)

# cible par défaut
$(CIBLE): $(OBJS)
	$(CXX) $(LDFLAGS) $^ $(LDLIBS) -o $@

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *~ $(CIBLE) $(OBJS) $(DEPS)

.PHONY: clean

-include $(DEPS)
