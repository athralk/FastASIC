CXX ?= g++
OPTFLAGS ?= -O3
CXXFLAGS = -std=c++17 -Iinclude -Wall -Wextra -pthread $(OPTFLAGS)

SRC = $(wildcard src/*.cpp)
HDR = $(wildcard include/*.hpp)
MAINS = src/main.cpp src/lsta_main.cpp src/fastpnr_main.cpp
LIB_SRC = $(filter-out $(MAINS),$(SRC))
TARGET = FastASIC
LSTA = lsta
PNR = fastpnr
STA_CHECK = tests/sta_bruteforce
SAT_CHECK = tests/sat_fuzz

.PHONY: all test clean

all: $(TARGET) $(LSTA) $(PNR)

$(TARGET): src/main.cpp $(LIB_SRC) $(HDR)
	$(CXX) $(CXXFLAGS) src/main.cpp $(LIB_SRC) -o $(TARGET)

# Standalone static timing analyser for gate-level netlists
$(LSTA): src/lsta_main.cpp $(LIB_SRC) $(HDR)
	$(CXX) $(CXXFLAGS) src/lsta_main.cpp $(LIB_SRC) -o $(LSTA)

# Physical design: floorplan (DEF output); placement and routing come next
$(PNR): src/fastpnr_main.cpp $(LIB_SRC) $(HDR)
	$(CXX) $(CXXFLAGS) src/fastpnr_main.cpp $(LIB_SRC) -o $(PNR)

# Independent brute-force check of the STA engine, used by the test suite
$(STA_CHECK): $(STA_CHECK).cpp $(LIB_SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(STA_CHECK).cpp $(LIB_SRC) -o $(STA_CHECK)

# Independent fuzz test of the CDCL solver against brute force, used by the test suite
$(SAT_CHECK): $(SAT_CHECK).cpp src/cdcl_solver.cpp include/sat_solver.hpp
	$(CXX) $(CXXFLAGS) $(SAT_CHECK).cpp src/cdcl_solver.cpp -o $(SAT_CHECK)

test: $(TARGET) $(LSTA) $(PNR) $(STA_CHECK) $(SAT_CHECK)
	bash tests/run_tests.sh ./$(TARGET)

clean:
	rm -f $(TARGET) $(LSTA) $(PNR) $(STA_CHECK) $(SAT_CHECK)
