# MacBook Lid / Hinge Sensor Reader
# Compile the C++ program with macOS IOKit and CoreFoundation frameworks

CXX      = clang++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
FRAMEWORKS = -framework IOKit -framework CoreFoundation
TARGET   = lid_sensor
SRC      = lid_sensor.cpp

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(FRAMEWORKS) -o $@ $<

clean:
	rm -f $(TARGET)
