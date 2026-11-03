CXX = g++
CXXFLAGS = -std=c++17 -Iinclude -Wall -Wextra -O2

SRC = $(wildcard src/*.cpp)
HDR = $(wildcard include/*.hpp)
LIB_SRC = $(filter-out src/main.cpp,$(SRC))
TARGET = output_test
STA_CHECK = tests/sta_bruteforce

.PHONY: all test clean

all: $(TARGET)

$(TARGET): $(SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

# Independent brute-force check of the STA engine, used by the test suite
$(STA_CHECK): $(STA_CHECK).cpp $(LIB_SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(STA_CHECK).cpp $(LIB_SRC) -o $(STA_CHECK)

test: $(TARGET) $(STA_CHECK)
	bash tests/run_tests.sh ./$(TARGET)

clean:
	rm -f $(TARGET) $(STA_CHECK)
