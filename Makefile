CXX = g++
CXXFLAGS = -Wcpp -Wall -MMD -I src/include
LDFLAGS = -Lsrc/lib
LDLIBS = -lSDL3

TARGET = test
SRCS = main.cpp Rendering.cpp GameObject.cpp SpaceShip.cpp Player.cpp Scene.cpp
OBJS = $(SRCS:.cpp=.o)
DEPS = $(OBJS:.o=.d)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include $(DEPS)

clean:
	del /Q $(OBJS) $(DEPS) $(TARGET).exe || exit 0
