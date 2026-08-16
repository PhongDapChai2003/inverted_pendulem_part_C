CXX ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -Iinclude
BUILD := build

.PHONY: all test simulate clean

all: $(BUILD)/controller_tests $(BUILD)/pendulum_sim

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/controller.o: src/controller.cpp include/pendulum/controller.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/estimator.o: src/estimator.cpp include/pendulum/estimator.hpp include/pendulum/controller.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/controller_tests: tests/controller_tests.cpp $(BUILD)/controller.o $(BUILD)/estimator.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/pendulum_sim: src/sim_main.cpp $(BUILD)/controller.o
	$(CXX) $(CXXFLAGS) $^ -o $@

test: $(BUILD)/controller_tests
	./$(BUILD)/controller_tests

simulate: $(BUILD)/pendulum_sim
	mkdir -p output/simulation
	./$(BUILD)/pendulum_sim output/simulation/nominal_run.csv

clean:
	rm -f $(BUILD)/controller.o $(BUILD)/estimator.o $(BUILD)/controller_tests $(BUILD)/pendulum_sim
